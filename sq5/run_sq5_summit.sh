#!/bin/bash
# SQ5 in the Summit worlds, NavMap only: the playground's own parameters drive the goals of
# sq5/goals_<scene>.json (sq5/goals3d.py), RUNS times each, headless. Results in $NAVMAP_DATA/sq5/.
# Usage: run_sq5_summit.sh [scenes="garage excavation"] [runs=3]
# SQ5_OUT: results elsewhere; SQ5_GOALS: folder of the goals files; SQ5_TRACE=1: a trace per run;
# SQ5_PARAMS: folder with other params files, <scene>.yaml; SQ5_BAG=1: a rosbag per run with the
# localizer's inputs and the true pose (for an offline replay of the localizer)
source "$(dirname "$0")/../env.sh"
SCENES=${1:-"garage excavation"}
RUNS=${2:-3}
OUT=${SQ5_OUT:-$NAVMAP_DATA/sq5}
mkdir -p "$OUT"
SUMMIT=$EASYNAV_WS/install/easynav_playground_summit/share/easynav_playground_summit
WORLDS=$EASYNAV_WS/install/easynav_playground_summit_worlds/share/easynav_playground_summit_worlds

leftovers() {  # simulators or EasyNav from anyone (PlanSys2's processes are not ours to judge)
  pgrep -af "gz sim|ruby.*gz|system_main|rviz2|parameter_bridge" | grep -v pgrep | grep -v plansys2
}
if leftovers; then echo "Simulation processes already running: not starting" >&2; exit 1; fi

for scene in $SCENES; do
  case $scene in
    garage) def_params=$SUMMIT/params/parking_garage.amcl.params.yaml
      world=$WORLDS/worlds/parking_garage.world; world_name=parking_garage_world; timeout=240 ;;
    excavation) def_params=$SUMMIT/params/bonxai.amcl.params.yaml
      world=$WORLDS/worlds/urjc_excavation.world; world_name=urjc_excavation_world; timeout=360 ;;
    *) echo "unknown scene $scene" >&2; exit 1 ;;
  esac
  params=${SQ5_PARAMS:+$SQ5_PARAMS/$scene.yaml}
  params=${params:-$def_params}
  for run in $(seq 1 "$RUNS"); do
    log=$OUT/summit_${scene}_run$run.log
    setsid ros2 launch "$ARTIFACTS/sq5/summit_scene.launch.yaml" params_file:="$params" \
      world:="$world" world_name:="$world_name" > "$log" 2>&1 &
    pgid=$!
    if [ -n "$SQ5_BAG" ]; then
      rm -rf "$OUT/summit_${scene}_run$run.bag"
      setsid ros2 bag record -o "$OUT/summit_${scene}_run$run.bag" --use-sim-time --topics \
        /front_laser/points /front_camera/depth/color/points /imu/data /tf /tf_static /clock \
        /gt_pose > "$OUT/summit_${scene}_run$run.bag.log" 2>&1 &
      bagpid=$!
    fi
    python3 "$ARTIFACTS/sq5/run_goals.py" "${SQ5_GOALS:-$ARTIFACTS/sq5}/goals_$scene.json" \
      "$OUT/summit_${scene}_run$run.json" --timeout $timeout --model summit_xl \
      --world $world_name --scan '' --settle 15 \
      ${SQ5_TRACE:+--trace "$OUT/summit_${scene}_run$run.csv"} 2>> "$log"
    if [ -n "$SQ5_BAG" ]; then  # the recorder closes the file on SIGINT; it may ignore it
      kill -INT -- -$bagpid 2>/dev/null
      for _ in $(seq 1 30); do kill -0 $bagpid 2>/dev/null || break; sleep 1; done
      kill -TERM -- -$bagpid 2>/dev/null
    fi
    kill -INT -- -$pgid 2>/dev/null
    for _ in $(seq 1 30); do leftovers > /dev/null || break; sleep 1; done
    kill -KILL -- -$pgid 2>/dev/null
    sleep 2
    if leftovers; then echo "processes left after $scene run $run" >&2; exit 1; fi
  done
done
