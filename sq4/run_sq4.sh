#!/bin/bash
# SQ4: memory against fidelity (simplification keeping the layers, with and without a dense
# inflation layer, against coarser uniform NavMaps): the house grid at 5 cm, the garage and the
# dense excavation at 0.2 m. Results in $NAVMAP_DATA/sq4/sq4.jsonl. Minutes.
source "$(dirname "$0")/../env.sh"
set -e
cd "$(dirname "$0")"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release > /dev/null && cmake --build build -j"$(nproc)" > /dev/null
mkdir -p "$NAVMAP_DATA/sq4"
PG=$EASYNAV_WS/src/easynav_playgrounds
{
  build/sq4 grid "$PG/playground_kobuki/easynav_playground_kobuki_worlds/maps/home2.yaml"
  build/sq4 cloud "$NAVMAP_DATA/clouds/parking_garage_lidar_full_vp.pcd"
  build/sq4 cloud "$NAVMAP_DATA/clouds/lidar_dense_vp.pcd"
} > "$NAVMAP_DATA/sq4/sq4.jsonl"
