#!/bin/bash
# Sensitivity of NavMap+vp to the carving parameters (review M7): the ratio of passes to hits
# that carves a voxel (default 1) and the carving voxel (default max(0.1, resolution / 2)), on
# the garage at 0.2 m (SCENE=garage, default) or the dense excavation at 0.4 m
# (SCENE=excavation), seed 1. Results in $NAVMAP_DATA/sq1/carving/. Minutes.
source "$(dirname "$0")/../env.sh"
OUT=$NAVMAP_DATA/sq1
mkdir -p "$OUT/carving"
case ${SCENE:-garage} in
  garage) N=garage_0.2; G=("$PLAYGROUND_MODELS/parking_garage/meshes/parking_garage.dae" "$NAVMAP_DATA/clouds/parking_garage_lidar_full_vp.pcd" 0.2
    --ground lowest --truth-cache "$OUT/garage_gt.npz" --seed 1 --reps navmap) ;;
  excavation) N=excavation_dense_0.4; G=("$PLAYGROUND_MODELS/urjc_excavation/meshes/URJC_excavation.stl" "$NAVMAP_DATA/clouds/lidar_dense_vp.pcd" 0.4
    --ground highest --truth-res 0.2 --truth-cache "$OUT/excavation_gt.npz" --seed 1 --reps navmap) ;;
esac
for ratio in 0.5 2.0 4.0; do
  CARVE_RATIO=$ratio python3 "$ARTIFACTS/sq1/eval_sq1.py" "${G[@]}" --work "$OUT/carving/r$ratio" --json "$OUT/carving/${N}_ratio${ratio}_s1.json"
done
for voxel in 0.05 0.2 0.4; do
  CARVE_VOXEL=$voxel python3 "$ARTIFACTS/sq1/eval_sq1.py" "${G[@]}" --work "$OUT/carving/v$voxel" --json "$OUT/carving/${N}_voxel${voxel}_s1.json"
done
