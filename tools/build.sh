#!/bin/bash
# Builds the C++ tools (meshstats, lidar_sim, dist_dump, cloud_to_navmap) against the sourced NavMap.
set -e
cd "$(dirname "$0")"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
