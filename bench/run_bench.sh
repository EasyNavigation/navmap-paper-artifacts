#!/bin/bash
# Computational cost: NavMap against a costmap on flat maps (house, warehouse at 5 cm), and against
# OctoMap and Bonxai from the same scans (garage, dense excavation at 0.2 m). One thread on one
# core. Results in $NAVMAP_DATA/bench/*.jsonl. Needs bench/build.sh first. Tens of minutes.
source "$(dirname "$0")/../env.sh"
set -e
OUT=$NAVMAP_DATA/bench
mkdir -p "$OUT"
PG=$EASYNAV_WS/src/easynav_playgrounds
B=$ARTIFACTS/bench/build
for m in $PG/playground_kobuki/easynav_playground_kobuki_worlds/maps/home2.yaml \
  $PG/playground_summit/easynav_playground_summit_worlds/maps/warehouse.yaml; do
  taskset -c 2 "$B/bench_flat" "$m" 9
done > "$OUT/flat.jsonl"
export OMP_NUM_THREADS=1
taskset -c 3 "$B/bench_3d" "$NAVMAP_DATA/clouds/parking_garage_lidar_full_vp.pcd" 0.2 9 30 > "$OUT/garage_3d.jsonl"
taskset -c 3 "$B/bench_3d" "$NAVMAP_DATA/clouds/lidar_dense_vp.pcd" 0.2 9 50 > "$OUT/excavation_3d.jsonl"
grep -m1 "model name" /proc/cpuinfo > "$OUT/cpu.txt"
