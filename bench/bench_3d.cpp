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

// 3D: the same scans (a cloud with each point's sensor position) as a NavMap, an OctoMap and a
// Bonxai map at the same resolution. Build time, memory, point queries, raycasts and the update
// from one scan. One thread (OMP_NUM_THREADS=1 for NavMap's carving), median and IQR over R runs.
// Usage: bench_3d cloud_vp.pcd res [runs=9] [max_range=30]   -> one JSON object per line

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <limits>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "bonxai/bonxai.hpp"
#include "bonxai/probabilistic_map.hpp"
#include "navmap_core/Iterators.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"
#include "octomap/octomap.h"
#include "pcl/io/pcd_io.h"

using Clock = std::chrono::steady_clock;
using navmap::NavCelId;

namespace
{

constexpr NavCelId kNoCel = std::numeric_limits<NavCelId>::max();

struct Stat
{
  double median, q1, q3;
};

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

double once_ms(auto && f)
{
  const auto t0 = Clock::now();
  f();
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

std::string g_map;
void print(const std::string & rep, const std::string & op, const Stat & s, const std::string & unit)
{
  std::printf(
    "{\"map\": \"%s\", \"rep\": \"%s\", \"op\": \"%s\", \"unit\": \"%s\", \"median\": %.4f, "
    "\"q1\": %.4f, \"q3\": %.4f}\n", g_map.c_str(), rep.c_str(), op.c_str(), unit.c_str(),
    s.median, s.q1, s.q3);
  std::fflush(stdout);
}
void print1(const std::string & rep, const std::string & op, double v, const std::string & unit)
{
  print(rep, op, {v, v, v}, unit);
}

// What a planner needs from a volumetric map: the voxels a robot can stand on (occupied, with
// free height above) and their connected components (26-neighbours within one voxel of step),
// as SQ1's voxel floor. Returns the number of standable voxels.
std::size_t extract_floor(const std::vector<std::array<int, 3>> & occupied, float res,
  float clearance)
{
  auto key = [](int x, int y, int z) {
      return (static_cast<uint64_t>(x + (1 << 20)) << 42) |
             (static_cast<uint64_t>(y + (1 << 20)) << 21) | static_cast<uint64_t>(z + (1 << 20));
    };
  std::unordered_set<uint64_t> occ;
  occ.reserve(occupied.size() * 2);
  for (const auto & c : occupied) {occ.insert(key(c[0], c[1], c[2]));}
  const int free_cells = static_cast<int>(std::ceil(clearance / res));
  std::unordered_map<uint64_t, uint32_t> stand;
  std::vector<std::array<int, 3>> cells;
  for (const auto & c : occupied) {
    bool ok = true;
    for (int dz = 1; dz <= free_cells && ok; ++dz) {
      ok = !occ.count(key(c[0], c[1], c[2] + dz));
    }
    if (ok) {
      stand.emplace(key(c[0], c[1], c[2]), static_cast<uint32_t>(cells.size()));
      cells.push_back(c);
    }
  }
  std::vector<uint32_t> parent(cells.size());
  for (uint32_t i = 0; i < parent.size(); ++i) {parent[i] = i;}
  auto find = [&](uint32_t i) {
      while (parent[i] != i) {i = parent[i] = parent[parent[i]];}
      return i;
    };
  for (uint32_t i = 0; i < cells.size(); ++i) {
    const auto & c = cells[i];
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const auto it = stand.find(key(c[0] + dx, c[1] + dy, c[2] + dz));
          if (it != stand.end()) {parent[find(i)] = find(it->second);}
        }
      }
    }
  }
  return cells.size();
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string path = argv[1];
  const float res = std::stof(argv[2]);
  const int R = argc > 3 ? std::atoi(argv[3]) : 9;
  const float max_range = argc > 4 ? std::stof(argv[4]) : 30.0f;
  g_map = path.substr(path.find_last_of('/') + 1);
  g_map = g_map.substr(0, g_map.find('.')) + "_" + argv[2];

  pcl::PointCloud<pcl::PointWithViewpoint> cloud;
  pcl::io::loadPCDFile(path, cloud);
  // Scans: runs of points with the same sensor position
  struct Scan
  {
    Eigen::Vector3f o;
    std::vector<Eigen::Vector3f> pts;
  };
  std::vector<Scan> scans;
  for (const auto & p : cloud) {
    const Eigen::Vector3f o(p.vp_x, p.vp_y, p.vp_z);
    if (scans.empty() || !scans.back().o.isApprox(o)) {scans.push_back({o, {}});}
    scans.back().pts.push_back({p.x, p.y, p.z});
  }
  print1("input", "points", cloud.size(), "count");
  print1("input", "scans", scans.size(), "count");

  // --- Build from all the scans ---
  navmap_ros::BuildParams bp;
  bp.resolution = res;
  bp.max_dz = 0.25f;
  bp.max_slope_deg = 30.0f;
  navmap_ros_interfaces::msg::NavMap msg;
  navmap::NavMap nm;
  print1("navmap", "build_ms", once_ms([&] {nm = navmap_ros::from_points(cloud, msg, bp);}), "ms");
  {
    pcl::PointCloud<pcl::PointXYZ> xyz;
    for (const auto & p : cloud) {xyz.push_back({p.x, p.y, p.z});}
    navmap_ros_interfaces::msg::NavMap m2;
    print1("navmap_bare", "build_ms", once_ms([&] {(void)navmap_ros::from_points(xyz, m2, bp);}),
      "ms");
  }
  nm.rebuild_geometry_accels();
  auto occ = nm.layers.add_or_get<uint8_t>("occupancy", nm.navcels.size(), navmap::LayerType::U8);

  octomap::OcTree ot(res);
  print1("octomap", "build_ms", once_ms([&] {
      for (const auto & s : scans) {
        octomap::Pointcloud pc;
        for (const auto & q : s.pts) {pc.push_back(q.x(), q.y(), q.z());}
        ot.insertPointCloud(pc, octomap::point3d(s.o.x(), s.o.y(), s.o.z()), max_range);
      }
    }), "ms");
  Bonxai::ProbabilisticMap bx(res);
  print1("bonxai", "build_ms", once_ms([&] {
      for (const auto & s : scans) {bx.insertPointCloud(s.pts, s.o, max_range);}
    }), "ms");

  // --- Memory ---
  std::size_t geo = nm.positions.size() * 3 * sizeof(float) +
    nm.navcels.size() * sizeof(navmap::NavCel);
  for (const auto & s : nm.surfaces) {
    geo += s.bvh.size() * sizeof(navmap::BVHNode) + (s.prim_indices.size() + s.navcels.size()) * 4;
  }
  print1("navmap", "memory_one_layer_MB", 1e-6 * (geo + nm.navcels.size()), "MB");
  print1("navmap", "navcels", nm.navcels.size(), "count");
  print1("octomap", "memory_MB", 1e-6 * ot.memoryUsage(), "MB");
  print1("octomap", "nodes", ot.size(), "count");
  print1("bonxai", "memory_MB", 1e-6 * bx.grid().memUsage(), "MB");
  {
    std::vector<Bonxai::CoordT> occv;
    bx.getOccupiedVoxels(occv);
    print1("bonxai", "occupied_voxels", occv.size(), "count");
  }

  // --- Standable surface and its connectivity, which NavMap already holds ---
  {
    std::vector<std::array<int, 3>> occv;
    print("octomap", "floor_extraction", measure(R, 1, [&] {occv.clear();}, [&](int) {
        for (auto it = ot.begin_leafs(); it != ot.end_leafs(); ++it) {
          if (!ot.isNodeOccupied(*it)) {continue;}
          // Leaves can be larger than one voxel: one cell per voxel of the leaf
          const int n = static_cast<int>(std::lround(it.getSize() / res));
          const auto k = ot.coordToKey(it.getCoordinate());
          for (int a = 0; a < n; ++a) {
            for (int b = 0; b < n; ++b) {
              for (int d = 0; d < n; ++d) {
                occv.push_back({k[0] - n / 2 + a, k[1] - n / 2 + b, k[2] - n / 2 + d});
              }
            }
          }
        }
        (void)extract_floor(occv, res, 1.0f);
      }), "us");
    std::vector<Bonxai::CoordT> bc;
    print("bonxai", "floor_extraction", measure(R, 1, [&] {bc.clear(); occv.clear();}, [&](int) {
        bx.getOccupiedVoxels(bc);
        for (const auto & c : bc) {occv.push_back({c.x, c.y, c.z});}
        print1("bonxai", "standable_voxels", extract_floor(occv, res, 1.0f), "count");
      }), "us");
  }

  // --- Queries at points on the ground (NavCel centroids, 5 cm up) ---
  std::mt19937 rng(1);
  std::uniform_int_distribution<NavCelId> C(0, static_cast<NavCelId>(nm.navcels.size() - 1));
  const int N = 20000;
  std::vector<Eigen::Vector3f> q(N);
  for (auto & p : q) {p = nm.navcel_centroid(C(rng)) + Eigen::Vector3f(0, 0, 0.05f);}
  auto nothing = [] {};
  volatile double sink = 0;
  std::size_t s;
  NavCelId c;
  Eigen::Vector3f bary, hit;
  print("navmap", "query_point", measure(R, N, nothing, [&](int k) {
      if (nm.locate_navcel(q[k], s, c, bary, &hit)) {sink = sink + (*occ)[c];}
    }), "us");
  print("octomap", "query_point", measure(R, N, nothing, [&](int k) {
      if (auto * n = ot.search(q[k].x(), q[k].y(), q[k].z() - 0.1)) {
        sink = sink + n->getOccupancy();
      }
    }), "us");
  print("bonxai", "query_point", measure(R, N, nothing, [&](int k) {
      sink = sink + bx.queryProbability(Eigen::Vector3f(q[k].x(), q[k].y(), q[k].z() - 0.1f));
    }), "us");

  // --- Raycasts from the sensor positions, random directions (first hit within max_range) ---
  std::normal_distribution<float> G(0.0f, 1.0f);
  std::uniform_int_distribution<std::size_t> S(0, scans.size() - 1);
  std::vector<Eigen::Vector3f> ro(N), rd(N);
  for (int k = 0; k < N; ++k) {
    ro[k] = scans[S(rng)].o;
    rd[k] = Eigen::Vector3f(G(rng), G(rng), G(rng)).normalized();
  }
  float t;
  print("navmap", "raycast", measure(R, N, nothing, [&](int k) {
      if (nm.raycast(ro[k], rd[k], c, t, hit)) {sink = sink + t;}
    }), "us");
  octomap::point3d end;
  print("octomap", "raycast", measure(R, N, nothing, [&](int k) {
      if (ot.castRay(octomap::point3d(ro[k].x(), ro[k].y(), ro[k].z()),
      octomap::point3d(rd[k].x(), rd[k].y(), rd[k].z()), end, true, max_range))
      {
        sink = sink + end.x();
      }
    }), "us");

  // --- Update from one scan (median over 50 scans) ---
  std::vector<std::size_t> pick(50);
  for (auto & p : pick) {p = S(rng);}
  std::size_t pts_per_scan = 0;
  for (const auto p : pick) {pts_per_scan += scans[p].pts.size();}
  print1("input", "points_per_update_scan", 1.0 * pts_per_scan / pick.size(), "count");
  print("octomap", "scan_update", measure(R, pick.size(), nothing, [&](int k) {
      const auto & sc = scans[pick[k]];
      octomap::Pointcloud pc;
      for (const auto & p : sc.pts) {pc.push_back(p.x(), p.y(), p.z());}
      ot.insertPointCloud(pc, octomap::point3d(sc.o.x(), sc.o.y(), sc.o.z()), max_range);
    }), "us");
  print("bonxai", "scan_update", measure(R, pick.size(), nothing, [&](int k) {
      const auto & sc = scans[pick[k]];
      bx.insertPointCloud(sc.pts, sc.o, max_range);
    }), "us");
  // NavMap, as its obstacle filter: mark the NavCel under each point and clear along the surface
  // from the robot's NavCel (under the sensor) to it
  print("navmap", "scan_update", measure(R, pick.size(), nothing, [&](int k) {
      const auto & sc = scans[pick[k]];
      NavCelId robot = kNoCel;
      if (!nm.locate_navcel(sc.o, s, robot, bary, &hit)) {return;}
      const Eigen::Vector3f foot = hit;
      NavCelId last = robot;
      for (const auto & p : sc.pts) {
        if ((p - sc.o).norm() > max_range) {continue;}
        navmap::NavMap::LocateOpts o;
        o.hint_cid = last;
        NavCelId cp;
        if (!nm.locate_navcel(p, s, cp, bary, &hit, o)) {continue;}
        last = cp;
        navmap::for_each_on_segment(nm, robot, foot, hit,
        [&](NavCelId x, float, float) {(*occ)[x] = 0;});
        (*occ)[cp] = 254;
      }
    }), "us");
  (void)sink;
  return 0;
}
