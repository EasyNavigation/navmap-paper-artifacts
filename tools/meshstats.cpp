#include <chrono>
#include <cstdio>
#include <map>
#include <string>
#include "pcl/io/pcd_io.h"
#include "pcl/conversions.h"
#include "navmap_ros/conversions.hpp"
#include "navmap_ros/navmap_io.hpp"
// With vp_x/vp_y/vp_z fields the viewpoint overload is used (NOVP=1 ignores them; NOCARVE=1,
// NOBELOW=1 disable each filter; CARVE_RATIO, CARVE_VOXEL set the carving; DUMP=file.pcd saves the filtered cloud).
int main(int argc, char ** argv) {
  pcl::PCLPointCloud2 raw;
  if (pcl::io::loadPCDFile(argv[1], raw) < 0) {return 1;}
  bool has_vp = false;
  for (const auto & f : raw.fields) {has_vp = has_vp || f.name == "vp_x";}
  has_vp = has_vp && !getenv("NOVP");
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointWithViewpoint> vcloud;
  pcl::fromPCLPointCloud2(raw, cloud);
  if (has_vp) {pcl::fromPCLPointCloud2(raw, vcloud);}
  navmap_ros::BuildParams p; p.resolution = std::stof(argv[2]); 
#ifndef OLD
  if (argc > 3) {p.min_clearance = std::stof(argv[3]);}
#endif
  if (argc > 4) {p.max_dz = std::stof(argv[4]);}
  if (argc > 5) {p.min_surface_area = std::stof(argv[5]);}
  if (getenv("HOLE")) {p.max_hole_area = std::stof(getenv("HOLE"));}
  if (getenv("SLOPE")) {p.max_slope_deg = std::stof(getenv("SLOPE"));}
  navmap_ros_interfaces::msg::NavMap msg;
  auto t0 = std::chrono::steady_clock::now();
  if (getenv("NOCARVE")) {p.carve_free_space = false;}
  if (getenv("NOBELOW")) {p.drop_seen_from_below = false;}
  if (getenv("CARVE_RATIO")) {p.carve_ratio = std::stof(getenv("CARVE_RATIO"));}
  if (getenv("CARVE_VOXEL")) {p.carve_voxel = std::stof(getenv("CARVE_VOXEL"));}
  navmap_ros::ViewpointFilterStats st;
  if (has_vp) {cloud = navmap_ros::filter_with_viewpoints(vcloud, p, &st);}
  if (has_vp && getenv("DUMP")) {pcl::io::savePCDFileBinary(getenv("DUMP"), cloud);}
  navmap::NavMap nm = navmap_ros::from_points(cloud, msg, p);
  if (has_vp) {
    std::printf("viewpoints: %zu points, carved %zu, seen from below %zu\n", st.input, st.carved,
      st.seen_from_below);
  }
  double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  std::map<std::pair<uint32_t, uint32_t>, int> uses; double area = 0;
  for (auto & c : nm.navcels) { area += c.area; for (int e = 0; e < 3; ++e) { auto a = c.v[e], b = c.v[(e+1)%3]; ++uses[{std::min(a,b), std::max(a,b)}]; } }
  int maxuse = 0; for (auto & [k, n] : uses) maxuse = std::max(maxuse, n);
  size_t big = nm.surfaces.empty() ? 0 : nm.surfaces[0].navcels.size();
  std::printf("%s res=%.2f clr=%.2f dz=%.2f pts=%zu  %.0f ms  verts=%zu tris=%zu area=%.0f m2  surfaces=%zu (largest %zu tris)  max_edge_use=%d nonmanifold=%zu\n",
    argv[1], p.resolution,
#ifndef OLD
 p.min_clearance,
#else
 0.0f,
#endif
 p.max_dz, cloud.size(), ms, nm.positions.size(), nm.navcels.size(), area, nm.surfaces.size(), big, maxuse, nm.nonmanifold_edges());
  if (argc > 6) {
    msg.header.frame_id = "map";
    std::error_code ec;
    bool ok = navmap_ros::io::save_msg_to_file(msg, argv[6], {}, &ec);
    std::printf("saved %s: %s\n", argv[6], ok ? "ok" : ec.message().c_str());
  }
}
