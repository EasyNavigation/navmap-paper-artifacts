// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Planning benchmark, protocol of the Nav2 survey: random reachable start/goal pairs, the
// planners of EasyNav as the stacks run them (CostmapPlanner on the inflated costmap, A* on the
// NavMap with its inflated layer), one thread. Per pair: wall time of the plugin's update(),
// path length, mean inflation cost along the path (0-252) and ascent.
//   flat:  bench_plan map.yaml pairs inflation_radius inscribed [cost_scaling]
//          (both planners on the same occupancy grid; parameters of the Kobuki stacks of SQ5)
//   3d:    bench_plan map.navmap pairs inflation_radius inscribed [cost_scaling] [max_slope_deg]
//          (A* on NavMap only; slope_weight 100 as the Summit stacks)
// JSON lines on stdout, one per planner and pair; summarize_plan.py makes the table.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "easynav_common/types/NavState.hpp"
#include "easynav_costmap_common/cost_values.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_maps_manager/filters/InflationFilter.hpp"
#include "easynav_costmap_planner/CostmapPlanner.hpp"
#include "easynav_navmap_maps_manager/filters/InflationFilter.hpp"
#include "easynav_navmap_planner/AStarPlanner.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navmap_core/Descriptors.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"
#include "navmap_ros/navmap_io.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using Clock = std::chrono::steady_clock;
using navmap::NavCelId;

namespace
{

constexpr NavCelId kNoCel = std::numeric_limits<NavCelId>::max();

// map_server's yaml + pgm (trinary), as bench_flat
nav_msgs::msg::OccupancyGrid load_map(const std::string & yaml)
{
  std::ifstream y(yaml);
  std::string line, image;
  double res = 0.05, ox = 0, oy = 0, occ = 0.65, freet = 0.196;
  int negate = 0;
  while (std::getline(y, line)) {
    std::istringstream s(line);
    std::string k;
    s >> k;
    if (k == "image:") {s >> image;}
    if (k == "resolution:") {s >> res;}
    if (k == "negate:") {s >> negate;}
    if (k == "occupied_thresh:") {s >> occ;}
    if (k == "free_thresh:") {s >> freet;}
    if (k == "origin:") {char c; s >> c >> ox >> c >> oy;}
  }
  const std::string dir = yaml.substr(0, yaml.find_last_of('/') + 1);
  std::ifstream f(image[0] == '/' ? image : dir + image, std::ios::binary);
  std::string magic;
  int w, h, maxv;
  f >> magic;
  auto skip = [&] {
      while (f >> std::ws && f.peek() == '#') {std::getline(f, line);}
    };
  skip(); f >> w; skip(); f >> h; skip(); f >> maxv; f.get();
  std::vector<unsigned char> px(static_cast<std::size_t>(w) * h);
  f.read(reinterpret_cast<char *>(px.data()), px.size());
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = "map";
  g.info.resolution = res;
  g.info.width = w;
  g.info.height = h;
  g.info.origin.position.x = ox;
  g.info.origin.position.y = oy;
  g.info.origin.orientation.w = 1.0;
  g.data.resize(px.size());
  for (int j = 0; j < h; ++j) {
    for (int i = 0; i < w; ++i) {
      double p = px[j * w + i] / 255.0;
      if (!negate) {p = 1.0 - p;}
      g.data[(h - 1 - j) * w + i] = p > occ ? 100 : (p < freet ? 0 : -1);
    }
  }
  return g;
}

// Exposes the inflation parameters, set by a node in EasyNav
struct Inflation : easynav::InflationFilter
{
  Inflation(easynav::Costmap2D & cm, double radius, double inscribed, double scaling)
  {
    inflation_radius_ = radius;
    inscribed_radius_ = inscribed;
    cost_scaling_factor_ = scaling;
    matchSize(cm);
  }
};

struct Pt
{
  double x, y, z;
};

struct Result
{
  bool ok{false};
  double ms{0}, length{0}, cost{0}, ascent{0};
};

// Union-find over the valid cells / NavCels
struct DSU
{
  std::vector<int> p;
  explicit DSU(std::size_t n)
  : p(n) {std::iota(p.begin(), p.end(), 0);}
  int find(int a) {while (p[a] != a) {a = p[a] = p[p[a]];} return a;}
  void unite(int a, int b) {a = find(a); b = find(b); if (a != b) {p[a] = b;}}
};

nav_msgs::msg::Odometry odom_at(const Pt & p)
{
  nav_msgs::msg::Odometry o;
  o.header.frame_id = "map";
  o.pose.pose.position.x = p.x;
  o.pose.pose.position.y = p.y;
  o.pose.pose.position.z = p.z;
  o.pose.pose.orientation.w = 1.0;
  return o;
}

nav_msgs::msg::Goals goals_at(const Pt & p, const rclcpp::Time & t)
{
  nav_msgs::msg::Goals g;
  g.header.frame_id = "map";
  g.header.stamp = t;
  geometry_msgs::msg::PoseStamped ps;
  ps.header = g.header;
  ps.pose.position.x = p.x;
  ps.pose.position.y = p.y;
  ps.pose.position.z = p.z;
  ps.pose.orientation.w = 1.0;
  g.goals.push_back(ps);
  return g;
}

// update() is protected in the plugins: the stacks call it through the method base
template<typename P>
struct Exposed : P
{
  using P::update;
};

// Runs one plan and measures it; cost_at samples the inflation layer of the stack along the path
template<typename Planner, typename CostAt>
Result plan(
  Planner & planner, easynav::NavState & st, const Pt & a, const Pt & b,
  const rclcpp::Time & now, CostAt && cost_at)
{
  st.set("robot_pose", odom_at(a));
  st.set("goals", goals_at(b, now));
  const auto t0 = Clock::now();
  planner.update(st);
  Result r;
  r.ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
  const auto & path = st.get<nav_msgs::msg::Path>("path");
  if (path.poses.size() < 2) {return r;}
  r.ok = true;
  double cost = 0;
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose.position;
    cost += cost_at(p.x, p.y, p.z);
    if (i > 0) {
      const auto & q = path.poses[i - 1].pose.position;
      r.length += std::hypot(p.x - q.x, p.y - q.y, p.z - q.z);
      r.ascent += std::max(0.0, p.z - q.z);
    }
  }
  r.cost = cost / path.poses.size();
  return r;
}

void print(
  const std::string & map, const std::string & planner, int pair, const Pt & a, const Pt & b,
  const Result & r)
{
  std::printf(
    "{\"map\": \"%s\", \"planner\": \"%s\", \"pair\": %d, \"start\": [%.2f, %.2f, %.2f], "
    "\"goal\": [%.2f, %.2f, %.2f], \"ok\": %s, \"time_ms\": %.3f, \"length_m\": %.3f, "
    "\"cost\": %.2f, \"ascent_m\": %.3f}\n", map.c_str(), planner.c_str(), pair, a.x, a.y, a.z,
    b.x, b.y, b.z, r.ok ? "true" : "false", r.ms, r.length, r.cost, r.ascent);
  std::fflush(stdout);
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 5) {
    std::fprintf(stderr, "usage: bench_plan map.yaml|map.navmap pairs radius inscribed "
      "[scaling=3] [max_slope_deg=90]\n");
    return 1;
  }
  const std::string file = argv[1];
  const int pairs = std::atoi(argv[2]);
  const double radius = std::atof(argv[3]), inscribed = std::atof(argv[4]);
  const double scaling = argc > 5 ? std::atof(argv[5]) : 3.0;
  const double max_slope = argc > 6 ? std::atof(argv[6]) : 90.0;
  const bool flat = file.size() > 5 && file.substr(file.size() - 5) == ".yaml";
  std::string name = file.substr(file.find_last_of('/') + 1);
  name = name.substr(0, name.find('.'));

  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions opts;
  // The planners' parameters of the SQ5 stacks: Kobuki (flat) and Summit (3D)
  opts.parameter_overrides({
      rclcpp::Parameter("costmap.cost_factor", 10.0),
      rclcpp::Parameter("costmap.continuous_replan", true),
      rclcpp::Parameter("navmap.continuous_replan", true),
      rclcpp::Parameter("navmap.cost_weight", flat ? 5.0 : 3.0),
      rclcpp::Parameter("navmap.terrain.slope_weight", flat ? 0.0 : 100.0),
      rclcpp::Parameter("navmap.terrain.max_slope_deg", max_slope)});
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("bench_plan", opts);
  Exposed<easynav::navmap::AStarPlanner> astar;
  astar.initialize(node, "navmap");
  easynav::NavState st_nm;

  std::mt19937 rng(7);
  std::vector<Pt> starts, goals;
  const double min_dist = flat ? 2.0 : 5.0;

  if (flat) {
    const auto grid = load_map(file);
    const double res = grid.info.resolution;
    const double ox = grid.info.origin.position.x, oy = grid.info.origin.position.y;
    // Costmap stack: the grid inflated by its InflationFilter
    easynav::Costmap2D cm(grid);
    Inflation infl(cm, radius, inscribed, scaling);
    infl.updateCosts(cm, 0, 0, cm.getSizeInCellsX(), cm.getSizeInCellsY());
    // NavMap stack: the same grid, "obstacles" from the occupancy, inflated by its filter
    auto nm = navmap_ros::from_occupancy_grid(grid);
    for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
      nm.layer_set<uint8_t>("obstacles", c, nm.layer_get<uint8_t>("occupancy", c, 0));
    }
    easynav::navmap::InflationFilter nm_infl;
    nm_infl.inflate_layer_u8(nm, "obstacles", "inflated_obstacles", radius, scaling, inscribed);
    nm.rebuild_geometry_accels();
    st_nm.set("map.navmap", nm);
    const auto & nmc = st_nm.get<navmap::NavMap>("map.navmap");

    Exposed<easynav::CostmapPlanner> cplanner;
    cplanner.initialize(node, "costmap");
    easynav::NavState st_cm;
    st_cm.set("map", cm);
    st_cm.set("map_time", node->now());

    // Valid cells (below the inscribed cost) and their components, 8-connected
    const int W = cm.getSizeInCellsX(), H = cm.getSizeInCellsY();
    auto valid = [&](int i, int j) {
        return i >= 0 && j >= 0 && i < W && j < H &&
               cm.getCost(i, j) < easynav::INSCRIBED_INFLATED_OBSTACLE;
      };
    DSU dsu(static_cast<std::size_t>(W) * H);
    std::vector<int> cells;
    for (int j = 0; j < H; ++j) {
      for (int i = 0; i < W; ++i) {
        if (!valid(i, j)) {continue;}
        cells.push_back(j * W + i);
        for (int dj = -1; dj <= 1; ++dj) {
          for (int di = -1; di <= 1; ++di) {
            if (valid(i + di, j + dj)) {dsu.unite(j * W + i, (j + dj) * W + i + di);}
          }
        }
      }
    }
    std::uniform_int_distribution<std::size_t> U(0, cells.size() - 1);
    while (static_cast<int>(starts.size()) < pairs) {
      const int a = cells[U(rng)], b = cells[U(rng)];
      if (dsu.find(a) != dsu.find(b)) {continue;}
      const Pt pa{ox + (a % W + 0.5) * res, oy + (a / W + 0.5) * res, 0.0};
      const Pt pb{ox + (b % W + 0.5) * res, oy + (b / W + 0.5) * res, 0.0};
      if (std::hypot(pa.x - pb.x, pa.y - pb.y) < min_dist) {continue;}
      starts.push_back(pa);
      goals.push_back(pb);
    }
    std::fprintf(stderr, "%s: %d pairs among %zu valid cells\n", name.c_str(), pairs, cells.size());

    auto cm_cost = [&](double x, double y, double) {
        unsigned i, j;
        return cm.worldToMap(x, y, i, j) ? static_cast<double>(cm.getCost(i, j)) : 0.0;
      };
    NavCelId hint = kNoCel;
    auto nm_cost = [&](double x, double y, double z) {
        std::size_t s;
        NavCelId c = kNoCel;
        Eigen::Vector3f bary, hit;
        navmap::NavMap::LocateOpts o;
        if (hint != kNoCel) {o.hint_cid = hint;}
        nmc.locate_navcel(Eigen::Vector3f(x, y, z + 0.1f), s, c, bary, &hit, o);
        if (c == kNoCel) {return 0.0;}
        hint = c;
        return static_cast<double>(nmc.layer_get<uint8_t>("inflated_obstacles", c, 0));
      };
    for (int k = 0; k < pairs; ++k) {
      print(name, "costmap", k, starts[k], goals[k],
        plan(cplanner, st_cm, starts[k], goals[k], node->now(), cm_cost));
      hint = kNoCel;
      print(name, "navmap", k, starts[k], goals[k],
        plan(astar, st_nm, starts[k], goals[k], node->now(), nm_cost));
    }
  } else {
    navmap::NavMap nm;
    if (!navmap_ros::io::load_from_file(file, nm)) {
      std::fprintf(stderr, "cannot load %s\n", file.c_str());
      return 1;
    }
    nm.rebuild_geometry_accels();
    // The stack's static map has no obstacles: the inflation comes from the map's borders
    // (the filter seeds them), as it does before the robot sees anything
    if (!nm.layers.get("obstacles")) {
      for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
        nm.layer_set<uint8_t>("obstacles", c, nm.layer_get<uint8_t>("occupancy", c, 0));
      }
    }
    easynav::navmap::InflationFilter nm_infl;
    nm_infl.inflate_layer_u8(nm, "obstacles", "inflated_obstacles", radius, scaling, inscribed);
    navmap::DescriptorParams dp;
    dp.roughness = dp.step = dp.clearance = false;
    navmap::compute_descriptors(nm, dp);
    st_nm.set("map.navmap", nm);
    const auto & nmc = st_nm.get<navmap::NavMap>("map.navmap");

    // Valid NavCels: below the inscribed cost and gentler than the slope limit (30 deg at most
    // for the samples: the robot does not start or stop on a ramp)
    const double sample_slope = std::min(max_slope, 30.0);
    std::vector<char> valid(nmc.navcels.size(), 0);
    std::vector<NavCelId> cells;
    for (NavCelId c = 0; c < nmc.navcels.size(); ++c) {
      valid[c] = nmc.layer_get<uint8_t>("inflated_obstacles", c, 0) <
        easynav::INSCRIBED_INFLATED_OBSTACLE && nmc.layer_get<float>("slope", c, 0.0f) < sample_slope;
      if (valid[c]) {cells.push_back(c);}
    }
    DSU dsu(nmc.navcels.size());
    for (const NavCelId c : cells) {
      for (const NavCelId n : nmc.navcel_neighbors(c)) {
        if (valid[n]) {dsu.unite(c, n);}
      }
    }
    std::uniform_int_distribution<std::size_t> U(0, cells.size() - 1);
    auto at = [&](NavCelId c) {
        const auto p = nmc.navcel_centroid(c);
        return Pt{p.x(), p.y(), p.z()};
      };
    while (static_cast<int>(starts.size()) < pairs) {
      const NavCelId a = cells[U(rng)], b = cells[U(rng)];
      if (dsu.find(a) != dsu.find(b)) {continue;}
      const Pt pa = at(a), pb = at(b);
      if (std::hypot(pa.x - pb.x, pa.y - pb.y, pa.z - pb.z) < min_dist) {continue;}
      starts.push_back(pa);
      goals.push_back(pb);
    }
    std::fprintf(stderr, "%s: %d pairs among %zu valid NavCels (%zu)\n", name.c_str(), pairs,
      cells.size(), nmc.navcels.size());

    NavCelId hint = kNoCel;
    auto nm_cost = [&](double x, double y, double z) {
        std::size_t s;
        NavCelId c = kNoCel;
        Eigen::Vector3f bary, hit;
        navmap::NavMap::LocateOpts o;
        if (hint != kNoCel) {o.hint_cid = hint;}
        nmc.locate_navcel(Eigen::Vector3f(x, y, z + 0.1f), s, c, bary, &hit, o);
        if (c == kNoCel) {return 0.0;}
        hint = c;
        return static_cast<double>(nmc.layer_get<uint8_t>("inflated_obstacles", c, 0));
      };
    for (int k = 0; k < pairs; ++k) {
      hint = kNoCel;
      print(name, "navmap", k, starts[k], goals[k],
        plan(astar, st_nm, starts[k], goals[k], node->now(), nm_cost));
    }
  }
  rclcpp::shutdown();
  return 0;
}
