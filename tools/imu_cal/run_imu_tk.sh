#!/usr/bin/env bash
#
# Wrapper for the imu_tk (Tedaldi) Docker image — runs test_imu_calib
# inside ghcr.io/owhinata/ubuntu-imu_tk on the cwd's accel.txt + gyro.txt
# and leaves test_imu_acc.calib / test_imu_gyro.calib alongside them.
#
# Usage:
#   ./run_imu_tk.sh                       # uses ./accel.txt, ./gyro.txt
#   ./run_imu_tk.sh <session_dir>         # uses <session_dir>/accel.txt etc.

set -euo pipefail

IMAGE="${IMU_TK_IMAGE:-ghcr.io/owhinata/ubuntu-imu_tk:latest}"

session_dir="${1:-.}"
session_dir="$(cd "$session_dir" && pwd)"

for f in accel.txt gyro.txt; do
  if [[ ! -f "$session_dir/$f" ]]; then
    echo "error: $session_dir/$f not found (run bin_to_ascii.py first)" >&2
    exit 1
  fi
done

if ! command -v docker >/dev/null; then
  echo "error: docker is required but not on PATH" >&2
  exit 1
fi

echo "running test_imu_calib on $session_dir/{accel,gyro}.txt"

# `--user` keeps output files owned by the host user instead of root.
# `-w /work` makes the .calib outputs land alongside the inputs.
docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "$session_dir:/work" \
  -w /work \
  "$IMAGE" \
  test_imu_calib /work/accel.txt /work/gyro.txt

for f in test_imu_acc.calib test_imu_gyro.calib; do
  if [[ ! -f "$session_dir/$f" ]]; then
    echo "error: $session_dir/$f was not produced — check imu_tk output above" >&2
    exit 1
  fi
done

echo "done:"
echo "  $session_dir/test_imu_acc.calib"
echo "  $session_dir/test_imu_gyro.calib"
