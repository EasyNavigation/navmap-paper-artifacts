#!/bin/bash
# SQ1 table: NavMap and the baselines built from the same simulated lidar cloud, against the
# collision mesh. Scenes: parking garage, excavation (scans every 3 m and every 10 m); cells of
# 0.2 and 0.4 m (0.6 and 0.8 m too for the sparse cloud); seeds of the query samples in SEEDS (default 1 to 5). The baselines and NavMap
# use the bare points (NOVP); "navmap+vp" is NavMap with the sensor positions (free-space
# carving, faces seen from below). The observed truth is what was seen from above.
# Results in $NAVMAP_DATA/sq1/main/, table by summarize_sq1.py. Heavy: about an hour.
source "$(dirname "$0")/../env.sh"
OUT=$NAVMAP_DATA/sq1
mkdir -p "$OUT/main"
GARAGE=("$PLAYGROUND_MODELS/parking_garage/meshes/parking_garage.dae" --ground lowest
  --truth-cache "$OUT/garage_gt.npz")
EXCAV=("$PLAYGROUND_MODELS/urjc_excavation/meshes/URJC_excavation.stl" --ground highest
  --truth-res 0.2 --truth-cache "$OUT/excavation_gt.npz")
for seed in ${SEEDS:-1 2 3 4 5}; do
  for res in 0.2 0.4 0.6 0.8; do
    for scene in garage excavation_dense excavation_10m; do
      # 0.6 and 0.8 m only for the sparse cloud: where NavMap needs a coarser cell
      if [ "$res" != 0.2 ] && [ "$res" != 0.4 ] && [ "$scene" != excavation_10m ]; then continue; fi
      case $scene in
        garage) args=("${GARAGE[@]}"); cloud=parking_garage_lidar_full_vp ;;
        excavation_dense) args=("${EXCAV[@]}"); cloud=lidar_dense_vp ;;
        excavation_10m) args=("${EXCAV[@]}"); cloud=lidar_10m_vp ;;
      esac
      common=("${args[0]}" "$NAVMAP_DATA/clouds/$cloud.pcd" $res "${args[@]:1}" --seed $seed
        --work "$OUT/main")
      NOVP=1 python3 "$ARTIFACTS/sq1/eval_sq1.py" "${common[@]}" \
        --json "$OUT/main/${scene}_${res}_plain_s$seed.json"
      python3 "$ARTIFACTS/sq1/eval_sq1.py" "${common[@]}" --reps navmap \
        --json "$OUT/main/${scene}_${res}_vp_s$seed.json"
    done
  done
done
