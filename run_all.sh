#!/bin/bash
# Reruns every result of the paper on the current workspace, stage by stage, and times each one.
# Usage: run_all.sh [stage ...]   (default: every stage, in this order)
#   prev        moves the previous results to data/prev_<date>/ (inputs and caches stay)
#   build       colcon build of the workspace (clean environment, Rolling)
#   tools       tools/, bench/ and sq4/ binaries against the built workspace
#   versions    data/VERSIONS.txt with the commit of every repository
#   inputs      data/generate_inputs.sh, then the SHA-256 of every input against data/SHA256SUMS
#   sq1 mls sq2 sq3 sq4 bench real   the offline experiments (their run_*.sh)
#   sq5_house sq5_summit             navigation in Gazebo (RUNS runs each; refuses if any
#                                    simulator or EasyNav is already running)
#   figures     figures/make_figures.py into the paper's images/
# Env: RUNS (3), MEMMAX (24G: cgroup cap of every stage), REAL_FROM_BAG=1 (redo scans and poses
#      from the rosbag), ROS_DOMAIN_ID/GZ_PARTITION for sq5 (42 / sq5 by default).
# Expected (16 cores): build 15 min, inputs 10, sq1 60, mls 40, sq2-sq4 10, bench 30, real 15
# (2 h from the bag), sq5_house 45, sq5_summit 3 h. Log: data/run_all_<date>.log
# (no set -u: the ROS setup files read unset variables)
cd "$(dirname "$0")"
source env.sh
STAGES=${*:-"prev build tools versions inputs sq1 mls sq2 sq3 sq4 bench real sq5_house sq5_summit figures"}
RUNS=${RUNS:-3}
MEMMAX=${MEMMAX:-24G}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} GZ_PARTITION=${GZ_PARTITION:-sq5}
DATE=$(date +%F)
LOG=$NAVMAP_DATA/run_all_$DATE.log
PAPER=$EASYNAV_WS/navmap-paper

capped() {  # whole stage inside a cgroup with a hard memory cap
  systemd-run --user --scope -q -p MemoryMax=$MEMMAX -p MemorySwapMax=0 "$@"
}
clean_ros() {  # the shell may have another workspace sourced: start from nothing
  env -i HOME="$HOME" PATH=/usr/local/bin:/usr/bin:/bin TERM="${TERM:-xterm}" \
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}" \
    bash --norc --noprofile -c "source /opt/ros/rolling/setup.bash && $*"
}
leftovers() {
  pgrep -af "gz sim|ruby.*gz|system_main|rviz2|parameter_bridge" | grep -v pgrep | grep -v plansys2
}

stage_prev() {
  local P=$NAVMAP_DATA/prev_$DATE
  for d in sq1/main sq1/mls sq2 sq4 real/eval bench; do
    [ -d "$NAVMAP_DATA/$d" ] || continue
    mkdir -p "$P/$d" && mv "$NAVMAP_DATA/$d"/* "$P/$d/" 2>/dev/null
  done
  mkdir -p "$P/sq3" "$P/sq5" "$P/sq1"
  mv "$NAVMAP_DATA"/sq3/*.json "$NAVMAP_DATA"/sq3/*.npz "$P/sq3/" 2>/dev/null
  mv "$NAVMAP_DATA"/sq5/*.json "$NAVMAP_DATA"/sq5/*.log "$NAVMAP_DATA"/sq5/*.csv "$P/sq5/" 2>/dev/null
  [ -d "$NAVMAP_DATA/sq5/summit" ] && mkdir -p "$P/sq5/summit" && \
    mv "$NAVMAP_DATA"/sq5/summit/*.json "$NAVMAP_DATA"/sq5/summit/*.log "$NAVMAP_DATA"/sq5/summit/*.csv "$P/sq5/summit/" 2>/dev/null
  # Caches from the meshes and the NavMaps built by the evaluators: regenerated (the excavation
  # mesh changed in easynav_playgrounds #16)
  mv "$NAVMAP_DATA"/sq1/*_gt.npz "$NAVMAP_DATA"/sq1/*.navmap "$P/sq1/" 2>/dev/null
  echo "previous results in $P"
}
stage_build() {
  clean_ros "set -o pipefail; cd $EASYNAV_WS && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_cohesion- 2>&1 | tail -6"
}
stage_tools() { tools/build.sh && bench/build.sh && (cd sq4 && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release >/dev/null && cmake --build build -j"$(nproc)" >/dev/null); }
stage_versions() {
  { for r in NavMap easynav_plugins easynav_playgrounds EasyNavigation; do
      printf '%-22s %s %s\n' "$r" "$(git -C "$EASYNAV_WS/src/$r" rev-parse --abbrev-ref HEAD)" "$(git -C "$EASYNAV_WS/src/$r" rev-parse --short HEAD)"
    done; echo "date $DATE"; } | tee "$NAVMAP_DATA/VERSIONS.txt"
}
stage_inputs() {
  data/generate_inputs.sh || return 1
  (cd "$NAVMAP_DATA" && sha256sum -c SHA256SUMS 2>/dev/null | grep -v ': OK$' | grep -v 'real/\|rosbags/' ;
   grep -v 'real/\|rosbags/' SHA256SUMS | awk '{print $2}' | xargs sha256sum > SHA256SUMS.new 2>/dev/null)
  echo "inputs that changed are listed above (expected: the excavation files); new sums in SHA256SUMS.new"
}
stage_sq1() { sq1/run_sq1.sh && python3 sq1/summarize_sq1.py | tee "$NAVMAP_DATA/sq1/summary.txt"; }
stage_mls() { sq1/run_sq1_mls.sh && python3 sq1/summarize_sq1.py > "$NAVMAP_DATA/sq1/summary.txt"; }
stage_sq2() { sq2/run_sq2.sh; }
stage_sq3() { sq3/run_sq3.sh; }
stage_sq4() { sq4/run_sq4.sh; }
stage_bench() { bench/run_bench.sh; }
stage_real() { real/run_real.sh; }
stage_sq5_house() { sq5/run_sq5.sh "costmap navmap" "$RUNS"; }
stage_sq5_summit() { sq5/run_sq5_summit.sh "garage excavation" "$RUNS"; }
stage_figures() { python3 figures/make_figures.py "$PAPER/images"; }

if leftovers; then echo "simulation or EasyNav processes running: not starting" >&2; exit 1; fi
free -g | head -2
for s in $STAGES; do
  echo "=== $s  $(date '+%F %T')" | tee -a "$LOG"
  t0=$(date +%s)
  if ! capped bash -c "source env.sh && $(declare -f stage_$s clean_ros leftovers) && RUNS=$RUNS DATE=$DATE PAPER=$PAPER stage_$s" >> "$LOG" 2>&1; then
    echo "=== $s FAILED after $(( $(date +%s) - t0 )) s, see $LOG" | tee -a "$LOG"; exit 1
  fi
  echo "=== $s done in $(( ($(date +%s) - t0) / 60 )) min" | tee -a "$LOG"
done
