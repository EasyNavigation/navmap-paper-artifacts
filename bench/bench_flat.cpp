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

// Flat maps: the same occupancy grid as a costmap (EasyNav's Costmap2D and its InflationFilter,
// ports of Nav2's) and as a NavMap, doing the same work. One thread, median and IQR over R runs.
// Usage: bench_flat map.yaml [runs=9]   -> one JSON object per line

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "easynav_costmap_common/cost_values.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_maps_manager/filters/InflationFilter.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navmap_core/DistanceField.hpp"
#include "navmap_core/Iterators.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"

using Clock = std::chrono::steady_clock;
using navmap::NavCelId;

namespace
{

constexpr NavCelId kNoCel = std::numeric_limits<NavCelId>::max();
constexpr float kInflation = 0.7f, kInscribed = 0.3f, kScaling = 3.0f;
constexpr int kRays = 720;
constexpr float kRange = 10.0f;

// map_server's yaml + pgm (trinary)
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

// Ray tracing is protected in the costmap, used by its layers (as in Nav2)
struct TracingCostmap : easynav::Costmap2D
{
  explicit TracingCostmap(const easynav::Costmap2D & c)
  : easynav::Costmap2D(c) {}
  TracingCostmap & operator=(const easynav::Costmap2D & c)
  {
    easynav::Costmap2D::operator=(c);
    return *this;
  }
  using easynav::Costmap2D::raytraceLine;
};

// Exposes the inflation parameters, set by a node in EasyNav
struct Inflation : easynav::InflationFilter
{
  explicit Inflation(easynav::Costmap2D & cm)
  {
    inflation_radius_ = kInflation;
    inscribed_radius_ = kInscribed;
    cost_scaling_factor_ = kScaling;
    matchSize(cm);
  }
};

uint8_t cost_of(float d)
{
  // Nav2's inflation cost as a function of the distance in metres
  if (d == 0.0f) {return easynav::LETHAL_OBSTACLE;}
  if (d <= kInscribed) {return easynav::INSCRIBED_INFLATED_OBSTACLE;}
  if (!(d <= kInflation)) {return easynav::FREE_SPACE;}
  return static_cast<uint8_t>(
    (easynav::INSCRIBED_INFLATED_OBSTACLE - 1) * std::exp(-kScaling * (d - kInscribed)));
}

struct Stat
{
  double median, q1, q3;
};

// us per call over runs of n calls; setup() before each run, not timed
template<typename S, typename F>
Stat measure(int runs, int n, S && setup, F && f)
{
  std::vector<double> t;
  for (int r = 0; r < runs; ++r) {
    setup();
    const auto t0 = Clock::now();
    for (int k = 0; k < n; ++k) {
      f(k);
    }
    t.push_back(std::chrono::duration<double, std::micro>(Clock::now() - t0).count() / n);
  }
  std::sort(t.begin(), t.end());
  return {t[t.size() / 2], t[t.size() / 4], t[(3 * t.size()) / 4]};
}

void print(const std::string & map, const std::string & rep, const std::string & op, const Stat & s,
  const std::string & unit = "us")
{
  std::printf(
    "{\"map\": \"%s\", \"rep\": \"%s\", \"op\": \"%s\", \"unit\": \"%s\", \"median\": %.4f, "
    "\"q1\": %.4f, \"q3\": %.4f}\n", map.c_str(), rep.c_str(), op.c_str(), unit.c_str(), s.median,
    s.q1, s.q3);
  std::fflush(stdout);
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string yaml = argv[1];
  const int R = argc > 2 ? std::atoi(argv[2]) : 9;
  std::string name = yaml.substr(yaml.find_last_of('/') + 1);
  name = name.substr(0, name.find('.'));
  const auto grid = load_map(yaml);
  const double res = grid.info.resolution;
  const double ox = grid.info.origin.position.x, oy = grid.info.origin.position.y;

  easynav::Costmap2D base(grid);
  navmap::NavMap nm = navmap_ros::from_occupancy_grid(grid);
  nm.rebuild_geometry_accels();
  auto occ = std::dynamic_pointer_cast<navmap::LayerView<uint8_t>>(nm.layers.get("occupancy"));
  const auto base_occ = occ->data();
  auto & cost = nm.layers.add_or_get<uint8_t>("inflated", nm.navcels.size(),
      navmap::LayerType::U8)->data();

  // Memory: the cells, and the geometry the NavMap adds
  std::size_t geo = nm.positions.size() * 3 * sizeof(float) +
    nm.navcels.size() * sizeof(navmap::NavCel);
  for (const auto & s : nm.surfaces) {
    geo += s.bvh.size() * sizeof(navmap::BVHNode) + (s.prim_indices.size() + s.navcels.size()) * 4;
  }
  const double cm_mb = 1e-6 * base.getSizeInCellsX() * base.getSizeInCellsY();
  print(name, "costmap", "memory_one_layer", {cm_mb, cm_mb, cm_mb}, "MB");
  const double nm_mb = 1e-6 * (geo + nm.navcels.size());
  print(name, "navmap", "memory_one_layer", {nm_mb, nm_mb, nm_mb}, "MB");
  print(name, "navmap", "navcels", {1.0 * nm.navcels.size(), 0, 0}, "count");
  print(name, "costmap", "cells", {1.0 * base.getSizeInCellsX() * base.getSizeInCellsY(), 0, 0},
    "count");

  // Free cells to stand on, and their NavCels
  std::mt19937 rng(1);
  std::vector<Eigen::Vector3f> free_pts;
  {
    std::uniform_int_distribution<unsigned> X(0, base.getSizeInCellsX() - 1),
    Y(0, base.getSizeInCellsY() - 1);
    while (free_pts.size() < 2000) {
      const unsigned i = X(rng), j = Y(rng);
      if (base.getCost(i, j) == easynav::FREE_SPACE) {
        free_pts.push_back({static_cast<float>(ox + (i + 0.5) * res),
            static_cast<float>(oy + (j + 0.5) * res), 0.0f});
      }
    }
  }
  std::size_t s;
  Eigen::Vector3f bary, hit;
  auto locate = [&](const Eigen::Vector3f & p, NavCelId hint = kNoCel) {
      NavCelId c = kNoCel;
      navmap::NavMap::LocateOpts o;
      if (hint != kNoCel) {o.hint_cid = hint;}
      nm.locate_navcel(p + Eigen::Vector3f(0, 0, 0.1f), s, c, bary, &hit, o);
      return c;
    };
  auto nothing = [] {};

  // 1) Point queries: random, and along a path (2 cm steps, previous cell as hint)
  const int N = 20000;
  volatile unsigned sink = 0;
  print(name, "costmap", "query_random", measure(R, N, nothing, [&](int k) {
      unsigned i, j;
      const auto & p = free_pts[k % free_pts.size()];
      if (base.worldToMap(p.x(), p.y(), i, j)) {sink = sink + base.getCost(i, j);}
    }));
  print(name, "navmap", "query_random", measure(R, N, nothing, [&](int k) {
      const NavCelId c = locate(free_pts[k % free_pts.size()]);
      if (c != kNoCel) {sink = sink + (*occ)[c];}
    }));
  std::vector<Eigen::Vector3f> path;
  for (int k = 0; k < N; ++k) {
    const auto & a = free_pts[(k / 200) % free_pts.size()];
    const auto & b = free_pts[(k / 200 + 1) % free_pts.size()];
    const float t = (k % 200) / 200.0f;
    path.push_back(a + t * (b - a));
  }
  print(name, "costmap", "query_path", measure(R, N, nothing, [&](int k) {
      unsigned i, j;
      if (base.worldToMap(path[k].x(), path[k].y(), i, j)) {sink = sink + base.getCost(i, j);}
    }));
  NavCelId prev = locate(path[0]);
  print(name, "navmap", "query_path", measure(R, N, nothing, [&](int k) {
      prev = locate(path[k], prev);
      if (prev != kNoCel) {sink = sink + (*occ)[prev];}
    }));

  // 2) A 720-ray scan from a free cell (endpoints at the first occupied cell, up to 10 m):
  // clear along each ray, mark its end
  struct Scan
  {
    Eigen::Vector3f o;
    std::vector<Eigen::Vector3f> ends;
  };
  std::vector<Scan> scans(50);
  for (std::size_t k = 0; k < scans.size(); ++k) {
    scans[k].o = free_pts[k];
    for (int a = 0; a < kRays; ++a) {
      const float th = 2.0f * static_cast<float>(M_PI) * a / kRays;
      const Eigen::Vector3f d(std::cos(th), std::sin(th), 0.0f);
      Eigen::Vector3f e = scans[k].o;
      for (float r = 0.0f; r < kRange; r += 0.5f * res) {
        e = scans[k].o + r * d;
        unsigned i, j;
        if (!base.worldToMap(e.x(), e.y(), i, j) || base.getCost(i, j) != easynav::FREE_SPACE) {
          e = scans[k].o + std::max(0.0f, r - static_cast<float>(res)) * d;
          break;
        }
      }
      scans[k].ends.push_back(e);
    }
  }
  TracingCostmap cm(base);
  struct Clear
  {
    unsigned char * data;
    void operator()(unsigned int off) {data[off] = easynav::FREE_SPACE;}
  };
  print(name, "costmap", "scan_update_720", measure(R, scans.size(), [&] {cm = base;}, [&](int k) {
      unsigned x0, y0, x1, y1;
      cm.worldToMap(scans[k].o.x(), scans[k].o.y(), x0, y0);
      for (const auto & e : scans[k].ends) {
        if (!cm.worldToMap(e.x(), e.y(), x1, y1)) {continue;}
        cm.raytraceLine(Clear{cm.getCharMap()}, x0, y0, x1, y1);
        cm.setCost(x1, y1, easynav::LETHAL_OBSTACLE);
      }
    }));
  std::vector<NavCelId> origin_cel(scans.size());
  for (std::size_t k = 0; k < scans.size(); ++k) {
    origin_cel[k] = locate(scans[k].o);
  }
  print(name, "navmap", "scan_update_720", measure(R, scans.size(), [&] {occ->data() = base_occ;},
    [&](int k) {
      for (const auto & e : scans[k].ends) {
        NavCelId last = kNoCel;
        navmap::for_each_on_segment(nm, origin_cel[k], scans[k].o, e,
        [&](NavCelId c, float, float) {(*occ)[c] = 0; last = c;});
        if (last != kNoCel) {(*occ)[last] = 254;}
      }
    }));

  // 3) Inflation (0.7 m, inscribed 0.3 m): full, and after moving 1 / 10 obstacle cells
  std::vector<NavCelId> seeds;
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    if (base_occ[c] >= 253 && base_occ[c] != 255) {seeds.push_back(c);}
  }
  Inflation infl(cm);
  const int sx = base.getSizeInCellsX(), sy = base.getSizeInCellsY();
  print(name, "costmap", "inflation_full", measure(R, 1, [&] {cm = base;}, [&](int) {
      infl.updateCosts(cm, 0, 0, sx, sy);
    }), "us");
  navmap::DistanceField df(nm, false, kInflation);
  print(name, "navmap", "inflation_full", measure(R, 1, nothing, [&](int) {
      df.compute(seeds);
      const auto & d = df.distances();
      for (NavCelId c = 0; c < nm.navcels.size(); ++c) {cost[c] = cost_of(d[c]);}
    }), "us");

  // Obstacle cells to move: lethal cells of the costmap and their NavCels (two per cell)
  std::vector<std::pair<unsigned, unsigned>> lethal;
  for (int j = 0; j < sy; ++j) {
    for (int i = 0; i < sx; ++i) {
      if (base.getCost(i, j) == easynav::LETHAL_OBSTACLE) {lethal.push_back({i, j});}
    }
  }
  const int win = static_cast<int>(std::ceil(kInflation / res)) + 1;
  for (const int moved : {1, 10}) {
    // The same cells move to random free cells
    std::vector<std::pair<unsigned, unsigned>> from(moved), to(moved);
    std::uniform_int_distribution<std::size_t> L(0, lethal.size() - 1);
    for (int k = 0; k < moved; ++k) {
      from[k] = lethal[L(rng)];
      unsigned i, j;
      base.worldToMap(free_pts[100 + k].x(), free_pts[100 + k].y(), i, j);
      to[k] = {i, j};
    }
    // Costmap: re-inflate the windows around the changed cells, as Nav2's bounds do
    easynav::Costmap2D moved_base(base);
    for (int k = 0; k < moved; ++k) {
      moved_base.setCost(from[k].first, from[k].second, easynav::FREE_SPACE);
      moved_base.setCost(to[k].first, to[k].second, easynav::LETHAL_OBSTACLE);
    }
    easynav::Costmap2D inflated(base);
    infl.updateCosts(inflated, 0, 0, sx, sy);
    print(name, "costmap", "inflation_move_" + std::to_string(moved),
      measure(R, 1, [&] {cm = inflated;}, [&](int) {
        int x0 = sx, y0 = sy, x1 = 0, y1 = 0;
        for (const auto & c : from) {
          x0 = std::min<int>(x0, c.first); y0 = std::min<int>(y0, c.second);
          x1 = std::max<int>(x1, c.first); y1 = std::max<int>(y1, c.second);
        }
        for (const auto & c : to) {
          x0 = std::min<int>(x0, c.first); y0 = std::min<int>(y0, c.second);
          x1 = std::max<int>(x1, c.first); y1 = std::max<int>(y1, c.second);
        }
        // Each changed cell's window separately when they are far apart
        auto redo = [&](int a0, int b0, int a1, int b1) {
            a0 = std::max(0, a0 - win); b0 = std::max(0, b0 - win);
            a1 = std::min(sx, a1 + win + 1); b1 = std::min(sy, b1 + win + 1);
            for (int j = b0; j < b1; ++j) {
              for (int i = a0; i < a1; ++i) {cm.setCost(i, j, moved_base.getCost(i, j));}
            }
            infl.updateCosts(cm, a0, b0, a1, b1);
          };
        if ((x1 - x0) * (y1 - y0) < 4 * win * win * moved) {
          redo(x0, y0, x1, y1);
        } else {
          for (const auto & c : from) {redo(c.first, c.second, c.first, c.second);}
          for (const auto & c : to) {redo(c.first, c.second, c.first, c.second);}
        }
      }), "us");
    // NavMap: incremental distance field, costs rewritten where the distance changed
    auto cels = [&](const std::vector<std::pair<unsigned, unsigned>> & v) {
        std::vector<NavCelId> out;
        for (const auto & c : v) {
          const NavCelId n = locate({static_cast<float>(ox + (c.first + 0.5) * res),
                static_cast<float>(oy + (c.second + 0.5) * res), 0.0f});
          if (n == kNoCel) {continue;}
          out.push_back(n);
          // The cell's other triangle: the neighbour whose centroid lies in the same cell
          const Eigen::Vector3f cn = nm.navcel_centroid(n);
          for (const NavCelId m : nm.navcels[n].neighbor) {
            if (m == kNoCel) {continue;}
            const Eigen::Vector3f cm2 = nm.navcel_centroid(m);
            if (std::floor((cm2.x() - ox) / res) == std::floor((cn.x() - ox) / res) &&
              std::floor((cm2.y() - oy) / res) == std::floor((cn.y() - oy) / res))
            {
              out.push_back(m);
            }
          }
        }
        return out;
      };
    const auto rem = cels(from), add = cels(to);
    df.compute(seeds);
    print(name, "navmap", "inflation_move_" + std::to_string(moved), measure(R, 1, [&] {
        df.compute(seeds);
      }, [&](int) {
        df.update(add, rem);
        for (const NavCelId c : df.changed()) {cost[c] = cost_of(df[c]);}
      }), "us");
  }
  (void)sink;
  return 0;
}
