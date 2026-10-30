// Per-NavCel dump for SQ3: centroid, normal z, surface and distance to the border along the surface
// (NavMap::distance_field from open edges). Usage: dist_dump map.navmap out.bin
// out.bin: N records of 6 float32 (cx, cy, cz, nz, surface, dist).
#include <cstdio>
#include <fstream>
#include <vector>

#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"
#include "navmap_ros/navmap_io.hpp"

int main(int argc, char ** argv)
{
  if (argc < 3) {return 1;}
  navmap_ros_interfaces::msg::NavMap msg;
  std::error_code ec;
  if (!navmap_ros::io::load_msg_from_file(argv[1], msg, &ec)) {return 1;}
  const navmap::NavMap nm = navmap_ros::from_msg(msg);
  const auto dist = nm.distance_field({}, true);
  std::vector<float> out;
  out.reserve(nm.navcels.size() * 6);
  for (navmap::NavCelId c = 0; c < nm.navcels.size(); ++c) {
    const auto p = nm.navcel_centroid(c);
    out.insert(
      out.end(), {p.x(), p.y(), p.z(), nm.navcels[c].normal.z(),
        static_cast<float>(nm.surface_of(c)), dist[c]});
  }
  std::ofstream(argv[2], std::ios::binary).write(
    reinterpret_cast<const char *>(out.data()), out.size() * sizeof(float));
  std::printf("%zu navcels\n", nm.navcels.size());
  return 0;
}
