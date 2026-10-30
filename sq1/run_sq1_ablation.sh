#!/bin/bash
# Ablation of the sensor positions (review M1): on the garage (0.2 m) and the dense excavation
# (0.4 m), every representation from the same cloud after free-space carving only (the carving is
# representation-independent: the carved cloud is dumped and read as bare points), and NavMap
# with carving but without dropping the faces seen from below. With the main runs this gives
# bare -> carving -> carving + faces seen from below for NavMap, and bare -> carving for the rest.
# Results in $NAVMAP_DATA/sq1/ablation/. About an hour.
source "$(dirname "$0")/../env.sh"
OUT=$NAVMAP_DATA/sq1
mkdir -p "$OUT/ablation"
GARAGE=("$PLAYGROUND_MODELS/parking_garage/meshes/parking_garage.dae" --ground lowest --truth-cache "$OUT/garage_gt.npz")
EXCAV=("$PLAYGROUND_MODELS/urjc_excavation/meshes/URJC_excavation.stl" --ground highest --truth-res 0.2 --truth-cache "$OUT/excavation_gt.npz")
for scene in ${SCENES:-garage excavation_dense}; do
  case $scene in
    garage) args=("${GARAGE[@]}"); cloud=parking_garage_lidar_full_vp; res=0.2 ;;
    excavation_dense) args=("${EXCAV[@]}"); cloud=lidar_dense_vp; res=0.4 ;;
  esac
  carved=$OUT/ablation/${scene}_${res}_carved.pcd
  # The carved cloud (dumped without viewpoints: the observed truth is taken from the original
  # cloud with --observe-cloud, so that every condition is scored on the same truth samples)
  [ -f "$carved" ] || NOBELOW=1 DUMP="$carved" "$MESHSTATS" "$NAVMAP_DATA/clouds/$cloud.pcd" $res 1.0 0.25 4 > /dev/null
  for seed in ${SEEDS:-1 2 3 4 5}; do
    common=("${args[0]}" "${args[@]:1}" --seed $seed --work "$OUT/ablation")
    # Every representation from the carved points
    NOVP=1 python3 "$ARTIFACTS/sq1/eval_sq1.py" "${common[0]}" "$carved" $res "${common[@]:1}" \
      --observe-cloud "$NAVMAP_DATA/clouds/$cloud.pcd" --json "$OUT/ablation/${scene}_${res}_carved_s$seed.json"
    # NavMap with carving only (faces seen from below kept)
    NOBELOW=1 python3 "$ARTIFACTS/sq1/eval_sq1.py" "${common[0]}" "$NAVMAP_DATA/clouds/$cloud.pcd" $res "${common[@]:1}" \
      --reps navmap --json "$OUT/ablation/${scene}_${res}_carveonly_s$seed.json"
  done
done
