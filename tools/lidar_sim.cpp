// Simulated VLP-16 scans over a binary STL terrain, accumulated into an irregular PCD.
// Usage: lidar_sim mesh.stl out.pcd z_offset step_m [noise_m] [max_range] [outliers] [max_pose_z]
// VIEWPOINT=1 also writes each point's sensor position (vp_x, vp_y, vp_z).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "navmap_core/NavMap.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

int main(int argc, char ** argv)
{
  const std::string stl = argv[1], out = argv[2];
  const float z_off = std::stof(argv[3]), step = std::stof(argv[4]);
  const float noise = argc > 5 ? std::stof(argv[5]) : 0.02f;
  const float max_range = argc > 6 ? std::stof(argv[6]) : 50.0f;
  const float outliers = argc > 7 ? std::stof(argv[7]) : 0.0f;  // fraction of spikes
  std::uniform_real_distribution<float> U(0.0f, 1.0f);

  std::ifstream f(stl, std::ios::binary);
  std::vector<char> hdr(80);
  f.read(hdr.data(), 80);
  uint32_t n;
  f.read(reinterpret_cast<char *>(&n), 4);
  navmap::NavMap nm;
  const auto s = nm.create_surface("map");
  for (uint32_t t = 0; t < n; ++t) {
    float rec[12];
    uint16_t attr;
    f.read(reinterpret_cast<char *>(rec), 48);
    f.read(reinterpret_cast<char *>(&attr), 2);
    uint32_t v[3];
    for (int k = 0; k < 3; ++k) {
      v[k] = nm.add_vertex({rec[3 + 3 * k], rec[4 + 3 * k], rec[5 + 3 * k] + z_off});
    }
    nm.add_navcel_to_surface(s, nm.add_navcel(v[0], v[1], v[2]));
  }
  nm.rebuild_geometry_accels();

  // Robot poses: grid over the map, on every floor (each upward hit going down a vertical ray)
  // with gentle slope and free space above, below max_pose_z; sensor 0.8 m above the ground.
  std::mt19937 rng(7);
  std::normal_distribution<float> N(0.0f, noise);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointWithViewpoint> vcloud;
  const bool with_vp = getenv("VIEWPOINT") != nullptr;
  int poses = 0;
  const float max_pose_z = argc > 8 ? std::stof(argv[8]) : 1e9f;
  auto scan_from = [&](const Eigen::Vector3f & o) {
      navmap::NavCelId cid;
      float t;
      Eigen::Vector3f hit;
      for (int ring = 0; ring < 16; ++ring) {
        const float el = (-15.0f + 2.0f * ring) * static_cast<float>(M_PI) / 180.0f;
        for (int az = 0; az < 900; ++az) {  // 0.4 deg
          const float a = az * 0.4f * static_cast<float>(M_PI) / 180.0f;
          const Eigen::Vector3f d(std::cos(el) * std::cos(a), std::cos(el) * std::sin(a), std::sin(el));
          if (nm.raycast(o, d, cid, t, hit) && t < max_range) {
            Eigen::Vector3f p = o + (t + N(rng)) * d;
            if (U(rng) < outliers) {p.z() += 0.3f * U(rng);}  // spurious returns above
            if (with_vp) {
              vcloud.push_back(pcl::PointWithViewpoint(p.x(), p.y(), p.z(), o.x(), o.y(), o.z()));
            } else {
              cloud.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
            }
          }
        }
      }
    };
  Eigen::Vector3f lo(1e9f, 1e9f, 1e9f), hi(-1e9f, -1e9f, -1e9f);
  for (std::size_t v = 0; v < nm.positions.size(); ++v) {
    lo = lo.cwiseMin(nm.positions.at(static_cast<uint32_t>(v)));
    hi = hi.cwiseMax(nm.positions.at(static_cast<uint32_t>(v)));
  }
  for (float x = lo.x() + 0.5f * step; x <= hi.x(); x += step) {
    for (float y = lo.y() + 0.5f * step; y <= hi.y(); y += step) {
      Eigen::Vector3f o(x, y, hi.z() + 1.0f);
      navmap::NavCelId cid;
      float t;
      Eigen::Vector3f hit;
      while (nm.raycast(o, {0.0f, 0.0f, -1.0f}, cid, t, hit)) {
        o = hit - Eigen::Vector3f(0.0f, 0.0f, 0.02f);  // continue below this surface
        if (nm.navcels[cid].normal.z() < std::cos(20.0f * M_PI / 180.0f) || hit.z() > max_pose_z) {
          continue;
        }
        navmap::NavCelId c2;
        float t2;
        Eigen::Vector3f h2;
        const Eigen::Vector3f base = hit + Eigen::Vector3f(0.0f, 0.0f, 0.05f);
        if (nm.raycast(base, {0.0f, 0.0f, 1.0f}, c2, t2, h2) && t2 < 1.5f) {continue;}  // no room
        ++poses;
        scan_from(hit + Eigen::Vector3f(0.0f, 0.0f, 0.8f));
      }
    }
  }
  if (with_vp) {
    pcl::io::savePCDFileBinary(out, vcloud);
  } else {
    pcl::io::savePCDFileBinary(out, cloud);
  }
  std::printf("%d poses, %zu points\n", poses, with_vp ? vcloud.size() : cloud.size());
  return 0;
}
