#!/bin/bash
# Builds the comparative benchmarks against the sourced workspace (Release).
set -e
cd "$(dirname "$0")"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
