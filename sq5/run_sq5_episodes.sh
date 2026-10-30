#!/bin/bash
# SQ5 (review 2, E11): independent episodes. Each leg of the goals file is driven in a freshly
# launched simulation: the robot is placed at the previous goal (the spawn for leg 0), the
# localizer is reset there and the goal is sent (run_goals.py --episode). House: both stacks,
# their order alternating with the leg and the repetition; Summit scenes: the NavMap stack.
# Usage: run_sq5_episodes.sh scene(house|garage|excavation) [reps=3] [legs="0 1 ... n-1"]
# A leg is a goal index K (from goal K-1) or J:K (from goal J). SQ5_PARAMS: folder with other
# params files, <scene>.yaml (e.g. without the recovery manager).
# Results in $SQ5_OUT (default $NAVMAP_DATA/sq5/episodes)/<scene>/<stack>_g<leg>_r<rep>.json
source "$(dirname "$0")/../env.sh"
SCENE=${1:?scene}
REPS=${2:-3}
OUT=${SQ5_OUT:-$NAVMAP_DATA/sq5/episodes}/$SCENE
mkdir -p "$OUT"
SUMMIT=$EASYNAV_WS/install/easynav_playground_summit/share/easynav_playground_summit
WORLDS=$EASYNAV_WS/install/easynav_playground_summit_worlds/share/easynav_playground_summit_worlds

leftovers() {  # simulators or EasyNav from anyone (PlanSys2's processes are not ours to judge)
  pgrep -af "gz sim|ruby.*gz|system_main|rviz2|parameter_bridge" | grep -v pgrep | grep -v plansys2
}
if leftovers; then echo "Simulation processes already running: not starting" >&2; exit 1; fi

case $SCENE in
  house)
    python3 "$ARTIFACTS/sq5/make_params.py" "$OUT/params" > /dev/null
    GOALS=$ARTIFACTS/sq5/goals_house.json; STACKS="costmap navmap"; TIMEOUT=120 ;;
  garage|excavation)
    GOALS=$ARTIFACTS/sq5/goals_$SCENE.json; STACKS=navmap
    if [ $SCENE = garage ]; then
      params=$SUMMIT/params/parking_garage.amcl.params.yaml
      world=$WORLDS/worlds/parking_garage.world; world_name=parking_garage_world; TIMEOUT=240
    else
      params=$SUMMIT/params/bonxai.amcl.params.yaml
      world=$WORLDS/worlds/urjc_excavation.world; world_name=urjc_excavation_world; TIMEOUT=360
    fi
    [ -n "$SQ5_PARAMS" ] && params=$SQ5_PARAMS/$SCENE.yaml ;;
  *) echo "unknown scene $SCENE" >&2; exit 1 ;;
esac
n=$(python3 -c "import json; print(len(json.load(open('$GOALS'))['goals']))")
LEGS=${3:-$(seq 0 $((n - 1)))}

episode() {  # stack leg rep
  local cfg=$1 leg=${2##*:} rep=$3 tag=${1}_g${2/:/-}_r$3 from=
  [[ $2 == *:* ]] && from="--episode-from ${2%%:*}"
  [ -f "$OUT/$tag.json" ] && { echo "$tag done"; return; }
  if [ $SCENE = house ]; then
    setsid ros2 launch "$ARTIFACTS/sq5/kobuki_house.launch.yaml" \
      params_file:="$OUT/params/kobuki_$cfg.yaml" > "$OUT/$tag.log" 2>&1 &
    local pgid=$!
    python3 "$ARTIFACTS/sq5/run_goals.py" "$GOALS" "$OUT/$tag.json" --timeout $TIMEOUT \
      --settle ${SQ5_SETTLE:-25} --warmup --episode $leg $from ${SQ5_TRACE:+--trace "$OUT/$tag.csv"} 2>> "$OUT/$tag.log"
  else
    setsid ros2 launch "$ARTIFACTS/sq5/summit_scene.launch.yaml" params_file:="$params" \
      world:="$world" world_name:="$world_name" > "$OUT/$tag.log" 2>&1 &
    local pgid=$!
    python3 "$ARTIFACTS/sq5/run_goals.py" "$GOALS" "$OUT/$tag.json" --timeout $TIMEOUT \
      --model summit_xl --world $world_name --scan '' --settle ${SQ5_SETTLE:-30} --warmup --episode $leg $from \
      ${SQ5_TRACE:+--trace "$OUT/$tag.csv"} 2>> "$OUT/$tag.log"
  fi
  kill -INT -- -$pgid 2>/dev/null
  for _ in $(seq 1 30); do leftovers > /dev/null || break; sleep 1; done
  kill -KILL -- -$pgid 2>/dev/null
  sleep 2
  if leftovers; then echo "processes left after $tag" >&2; exit 1; fi
}

for rep in $(seq 1 "$REPS"); do
  for leg in $LEGS; do
    order=$STACKS
    [ $(( (${leg##*:} + rep) % 2 )) = 1 ] && order=$(echo $STACKS | tr ' ' '\n' | tac | tr '\n' ' ')
    for cfg in $order; do episode $cfg $leg $rep; done
  done
done
