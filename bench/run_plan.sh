#!/bin/bash
# Planning benchmark (protocol of the Nav2 survey: random reachable pairs). Flat maps: the
# costmap and the NavMap stacks of the Kobuki (inflation 1.0 m, inscribed 0.178 m, scaling 5) on
# home2 and warehouse; 3D: the Summit's NavMap stack (inscribed 0.7 m) on the garage (inflation
# 2.0 m) and the excavation (1.5 m, slopes up to 25 deg). One thread on one core. Results in
# $NAVMAP_DATA/bench/plan.jsonl; table by summarize_plan.py. Minutes.
source "$(dirname "$0")/../env.sh"
set -e
OUT=$NAVMAP_DATA/bench
mkdir -p "$OUT"
PG=$EASYNAV_WS/src/easynav_playgrounds
B=$ARTIFACTS/bench/build
CORE=${CORE:-2}  # the core the planners run on
N=${PAIRS:-1000}
{
  taskset -c $CORE "$B/bench_plan" "$PG/playground_kobuki/easynav_playground_kobuki_worlds/maps/home2.yaml" $N 1.0 0.178 5.0
  taskset -c $CORE "$B/bench_plan" "$PG/playground_summit/easynav_playground_summit_worlds/maps/warehouse.yaml" $N 1.0 0.178 5.0
  taskset -c $CORE "$B/bench_plan" "$PG/playground_summit/easynav_playground_summit_worlds/maps/parking_garage.navmap" $N 2.0 0.7 3.0 90
  taskset -c $CORE "$B/bench_plan" "$PG/playground_summit/easynav_playground_summit_worlds/maps/excavation_urjc.navmap" $N 1.5 0.7 3.0 25
} > "$OUT/plan.jsonl"
python3 "$ARTIFACTS/bench/summarize_plan.py" "$OUT/plan.jsonl" | tee "$OUT/plan_summary.txt"
