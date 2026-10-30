#!/bin/bash
# Sensitivity of the MLS baseline to its two parameters: the vertical gap that splits patches and
# the noise margin of the patch thickness, on the clouds of run_sq1.sh (observed truth: seen
# from above). Results in $NAVMAP_DATA/sq1/mls/.
source "$(dirname "$0")/../env.sh"
OUT=$NAVMAP_DATA/sq1
mkdir -p "$OUT/mls"
for res in 0.2 0.4; do
  for gap in 0.15 0.25 0.5 1.0; do
    for margin in 0.0 0.1 0.2 0.4 0.8; do
      tag="g${gap}_m${margin}"
      python3 "$ARTIFACTS/sq1/eval_sq1.py" "$PLAYGROUND_MODELS/parking_garage/meshes/parking_garage.dae" \
        "$NAVMAP_DATA/clouds/parking_garage_lidar_full_vp.pcd" $res --ground lowest --reps mls \
        --mls-gap $gap --mls-margin $margin \
        --truth-cache "$OUT/garage_gt.npz" --work "$OUT" --json "$OUT/mls/garage_${res}_$tag.json"
      python3 "$ARTIFACTS/sq1/eval_sq1.py" "$PLAYGROUND_MODELS/urjc_excavation/meshes/URJC_excavation.stl" \
        "$NAVMAP_DATA/clouds/lidar_dense_vp.pcd" $res --ground highest --truth-res 0.2 --reps mls \
        --mls-gap $gap --mls-margin $margin \
        --truth-cache "$OUT/excavation_gt.npz" --work "$OUT" \
        --json "$OUT/mls/excavation_lidar_dense_${res}_$tag.json"
    done
  done
done
