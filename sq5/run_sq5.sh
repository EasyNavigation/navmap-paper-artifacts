#!/bin/bash
# SQ5 in the Kobuki house: the costmap and the NavMap stacks (sq5/make_params.py) drive the same
# goals (sq5/goals_house.json), RUNS times each, headless. Results in $NAVMAP_DATA/sq5/.
# Usage: run_sq5.sh [configs="costmap navmap"] [runs=3] [goals=sq5/goals_house.json]
# SQ5_OUT: results elsewhere; SQ5_TRACE=1: a trace per run (run_goals.py --trace); SQ5_BAG=1: a
# rosbag per run with the localizer's inputs and the true pose (offline replay of the localizer)
source "$(dirname "$0")/../env.sh"
CONFIGS=${1:-"costmap navmap"}
RUNS=${2:-3}
GOALS=${3:-$ARTIFACTS/sq5/goals_house.json}
OUT=${SQ5_OUT:-$NAVMAP_DATA/sq5}
python3 "$ARTIFACTS/sq5/make_params.py" "$OUT/params" > /dev/null

leftovers() {  # simulators or EasyNav from anyone (PlanSys2's processes are not ours to judge)
  pgrep -af "gz sim|ruby.*gz|system_main|rviz2|parameter_bridge" | grep -v pgrep | grep -v plansys2
}
if leftovers; then echo "Simulation processes already running: not starting" >&2; exit 1; fi

for cfg in $CONFIGS; do
  for run in $(seq 1 "$RUNS"); do
    log=$OUT/${cfg}_run$run.log
    setsid ros2 launch "$ARTIFACTS/sq5/kobuki_house.launch.yaml" \
      params_file:="$OUT/params/kobuki_$cfg.yaml" > "$log" 2>&1 &
    pgid=$!
    if [ -n "$SQ5_BAG" ]; then
      rm -rf "$OUT/${cfg}_run$run.bag"
      setsid ros2 bag record -o "$OUT/${cfg}_run$run.bag" --use-sim-time --topics \
        /scan_raw /tf /tf_static /clock /gt_pose > "$OUT/${cfg}_run$run.bag.log" 2>&1 &
      bagpid=$!
    fi
    python3 "$ARTIFACTS/sq5/run_goals.py" "$GOALS" "$OUT/${cfg}_run$run.json" --timeout 120 \
      ${SQ5_TRACE:+--trace "$OUT/${cfg}_run$run.csv"} 2>> "$log"
    if [ -n "$SQ5_BAG" ]; then  # the recorder closes the file on SIGINT; it may ignore it
      kill -INT -- -$bagpid 2>/dev/null
      for _ in $(seq 1 30); do kill -0 $bagpid 2>/dev/null || break; sleep 1; done
      kill -TERM -- -$bagpid 2>/dev/null
    fi
    kill -INT -- -$pgid 2>/dev/null
    for _ in $(seq 1 30); do leftovers > /dev/null || break; sleep 1; done
    kill -KILL -- -$pgid 2>/dev/null
    sleep 2
    if leftovers; then echo "processes left after $cfg run $run" >&2; exit 1; fi
  done
done
