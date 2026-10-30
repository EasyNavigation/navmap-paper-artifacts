#!/bin/bash
# eval_sq1.py with the environment of env.sh:  run_eval.sh mesh cloud res [args...]
source "$(dirname "$0")/../env.sh"
exec python3 "$ARTIFACTS/sq1/eval_sq1.py" "$@"
