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

// SQ4: memory against fidelity. A reference NavMap (an occupancy grid at its resolution, or a
// cloud with sensor positions at 0.2 m) is simplified keeping its layers, with growing max_error,
// with and without a dense layer (inflation: a cost that changes with the distance to obstacles
// and to the borders). Each version, and NavMaps built at coarser uniform resolutions, are
// measured against the reference at 20 000 of its NavCel centroids: vertical error, ground lost,
// and layer values that changed. One JSON object per line.
// Usage: sq4 grid map.yaml | sq4 cloud cloud_vp.pcd

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <unistd.h>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navmap_core/DistanceField.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_core/Simplify.hpp"
#include "navmap_ros/conversions.hpp"
#include "navmap_ros/navmap_io.hpp"
#include "pcl/io/pcd_io.h"

using navmap::NavCelId;
using navmap::NavMap;

namespace
{

constexpr float kInflation = 0.7f, kInscribed = 0.3f, kScaling = 3.0f;

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

// Coarser grid: a cell is occupied if any of its children is, unknown if all are
nav_msgs::msg::OccupancyGrid coarsen(const nav_msgs::msg::OccupancyGrid & g, int k)
{
  nav_msgs::msg::OccupancyGrid c = g;
  c.info.resolution = g.info.resolution * k;
  c.info.width = g.info.width / k;
  c.info.height = g.info.height / k;
  c.data.assign(static_cast<std::size_t>(c.info.width) * c.info.height, -1);
  for (unsigned j = 0; j < c.info.height; ++j) {
    for (unsigned i = 0; i < c.info.width; ++i) {
      int8_t v = -1;
      for (int b = 0; b < k; ++b) {
        for (int a = 0; a < k; ++a) {
          const int8_t x = g.data[(j * k + b) * g.info.width + i * k + a];
          if (x == 100 || (x == 0 && v != 100)) {v = x;}
        }
      }
      c.data[j * c.info.width + i] = v;
    }
  }
  return c;
}

uint8_t cost_of(float d)
{
  if (d == 0.0f) {return 254;}
  if (d <= kInscribed) {return 253;}
  if (!(d <= kInflation)) {return 0;}
  return static_cast<uint8_t>(252 * std::exp(-kScaling * (d - kInscribed)));
}

// The dense layer: inflation of the obstacle NavCels (occupancy 254) and of the borders
void add_inflation(NavMap & nm)
{
  std::vector<NavCelId> seeds;
  if (auto occ = std::dynamic_pointer_cast<navmap::LayerView<uint8_t>>(nm.layers.get("occupancy"))) {
    for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
      if ((*occ)[c] == 254) {seeds.push_back(c);}
    }
  }
  navmap::DistanceField df(nm, true, kInflation);
  df.compute(seeds);
  auto & inf = nm.layers.add_or_get<uint8_t>("inflation", nm.navcels.size(),
      navmap::LayerType::U8)->data();
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {inf[c] = cost_of(df[c]);}
}

std::size_t ram_bytes(const NavMap & nm)
{
  std::size_t b = nm.positions.size() * 3 * sizeof(float) +
    nm.navcels.size() * sizeof(navmap::NavCel);
  for (const auto & s : nm.surfaces) {
    b += s.bvh.size() * sizeof(navmap::BVHNode) + (s.prim_indices.size() + s.navcels.size()) * 4;
  }
  for (const auto & name : nm.layers.list()) {
    const auto l = nm.layers.get(name);
    const std::size_t item = l->type() == navmap::LayerType::U8 ? 1 :
      l->type() == navmap::LayerType::I16 ? 2 : l->type() == navmap::LayerType::F32 ? 4 : 8;
    b += l->size() * item;
  }
  return b;
}

std::size_t file_bytes(const NavMap & nm)
{
  const std::string p = "/tmp/sq4_" + std::to_string(::getpid()) + ".navmap";
  navmap_ros::io::save_to_file(nm, p);
  const auto n = std::filesystem::file_size(p);
  std::filesystem::remove(p);
  return n;
}

struct Sample
{
  Eigen::Vector3f p;
  uint8_t occ, inf;
};

void measure(const std::string & scene, const std::string & variant, const std::string & layers,
  double max_error, const NavMap & nm, const std::vector<Sample> & samples, bool has_inf)
{
  auto occ = std::dynamic_pointer_cast<navmap::LayerView<uint8_t>>(nm.layers.get("occupancy"));
  auto inf = std::dynamic_pointer_cast<navmap::LayerView<uint8_t>>(nm.layers.get("inflation"));
  std::vector<float> dz;
  std::size_t lost = 0, occ_diff = 0, inf_diff = 0;
  for (const auto & s : samples) {
    std::size_t si;
    NavCelId c;
    Eigen::Vector3f bary, hit;
    navmap::NavMap::LocateOpts o;
    o.height_eps = 0.5f;
    if (!nm.locate_navcel(s.p, si, c, bary, &hit, o) || std::fabs(hit.z() - s.p.z()) > 0.5f) {
      ++lost;
      continue;
    }
    dz.push_back(std::fabs(hit.z() - s.p.z()));
    if (occ && (*occ)[c] != s.occ) {++occ_diff;}
    if (has_inf && inf && (*inf)[c] != s.inf) {++inf_diff;}
  }
  std::sort(dz.begin(), dz.end());
  auto pct = [&](double q) {return dz.empty() ? 0.0 : dz[std::min(dz.size() - 1,
             static_cast<std::size_t>(q * dz.size()))];};
  const double n = samples.size();
  std::printf(
    "{\"scene\": \"%s\", \"variant\": \"%s\", \"layers\": \"%s\", \"max_error\": %.3f, "
    "\"navcels\": %zu, \"ram_MB\": %.3f, \"file_KB\": %.1f, \"dz_p50\": %.4f, \"dz_p95\": %.4f, "
    "\"dz_max\": %.4f, \"lost\": %.4f, \"occupancy_changed\": %.4f, \"inflation_changed\": %.4f}\n",
    scene.c_str(), variant.c_str(), layers.c_str(), max_error, nm.navcels.size(),
    1e-6 * ram_bytes(nm), 1e-3 * file_bytes(nm), pct(0.5), pct(0.95),
    dz.empty() ? 0.0 : dz.back(), lost / n, occ_diff / n, inf_diff / n);
  std::fflush(stdout);
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string kind = argv[1], path = argv[2];
  std::string scene = path.substr(path.find_last_of('/') + 1);
  scene = scene.substr(0, scene.find('.'));

  // Reference and coarser uniform versions
  NavMap ref;
  std::vector<std::pair<std::string, NavMap>> uniform;
  nav_msgs::msg::OccupancyGrid grid;
  pcl::PointCloud<pcl::PointWithViewpoint> cloud;
  navmap_ros::BuildParams bp;
  bp.max_dz = 0.25f;
  bp.max_slope_deg = 30.0f;
  if (kind == "grid") {
    grid = load_map(path);
    ref = navmap_ros::from_occupancy_grid(grid);
    for (const int k : {2, 4}) {
      uniform.push_back({"uniform_x" + std::to_string(k),
          navmap_ros::from_occupancy_grid(coarsen(grid, k))});
    }
  } else {
    pcl::io::loadPCDFile(path, cloud);
    navmap_ros_interfaces::msg::NavMap msg;
    bp.resolution = 0.2f;
    ref = navmap_ros::from_points(cloud, msg, bp);
    for (const float r : {0.4f, 0.8f}) {
      bp.resolution = r;
      uniform.push_back({"uniform_" + std::to_string(r).substr(0, 3),
          navmap_ros::from_points(cloud, msg, bp)});
    }
  }
  ref.rebuild_geometry_accels();
  for (auto & u : uniform) {u.second.rebuild_geometry_accels();}

  for (const bool with_inf : {false, true}) {
    const std::string layers = with_inf ? "occupancy+inflation" : "occupancy";
    NavMap base = ref;
    if (with_inf) {add_inflation(base);}
    // Samples: centroids of the reference NavCels, 5 cm up, with their layer values
    std::mt19937 rng(1);
    std::uniform_int_distribution<NavCelId> C(0, static_cast<NavCelId>(base.navcels.size() - 1));
    auto occ = std::dynamic_pointer_cast<navmap::LayerView<uint8_t>>(base.layers.get("occupancy"));
    auto inf = std::dynamic_pointer_cast<navmap::LayerView<uint8_t>>(base.layers.get("inflation"));
    std::vector<Sample> samples(20000);
    for (auto & s : samples) {
      const NavCelId c = C(rng);
      s.p = base.navcel_centroid(c) + Eigen::Vector3f(0, 0, 0.05f);
      s.occ = occ ? (*occ)[c] : 0;
      s.inf = inf ? (*inf)[c] : 0;
      s.p.z() -= 0.05f;  // on the surface: the vertical error is measured from it
    }

    for (const double e : {0.0, 0.01, 0.02, 0.05, 0.1, 0.2}) {
      NavMap m = base;
      if (e > 0.0) {
        navmap::SimplifyParams sp;
        sp.max_error = static_cast<float>(e);
        sp.ignore_layers = {"elevation", "support"};
        navmap::simplify(m, sp);
        m.rebuild_geometry_accels();
      }
      measure(scene, "simplified", layers, e, m, samples, with_inf);
    }
    for (auto & u : uniform) {
      NavMap m = u.second;
      if (with_inf) {add_inflation(m);}
      measure(scene, u.first, layers, 0.0, m, samples, with_inf);
    }
  }
  return 0;
}
