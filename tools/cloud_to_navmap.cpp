// A NavMap from a cloud, as the playgrounds' maps are built: from_points at a resolution,
// dropping surfaces smaller than a minimum area.
// Usage: cloud_to_navmap cloud.pcd out.navmap [resolution 1.0] [min_surface_area 4.0]
#include <cstdio>
#include <cstdlib>
#include "pcl/io/pcd_io.h"
#include "navmap_ros/conversions.hpp"
#include "navmap_ros/navmap_io.hpp"

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr, "usage: cloud_to_navmap cloud.pcd out.navmap [res] [min_area]\n");
    return 1;
  }
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(argv[1], cloud) < 0) {return 1;}
  navmap_ros::BuildParams p;
  p.resolution = argc > 3 ? std::atof(argv[3]) : 1.0f;
  p.min_surface_area = argc > 4 ? std::atof(argv[4]) : 4.0f;
  navmap_ros_interfaces::msg::NavMap msg;
  const navmap::NavMap nm = navmap_ros::from_points(cloud, msg, p);
  std::error_code ec;
  if (!navmap_ros::io::save_msg_to_file(msg, argv[2], {}, &ec)) {
    std::fprintf(stderr, "cannot save %s: %s\n", argv[2], ec.message().c_str());
    return 1;
  }
  std::printf("%zu navcels, %zu surfaces\n", nm.navcels.size(), nm.surfaces.size());
  return 0;
}
