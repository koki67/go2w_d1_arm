#!/bin/bash
set -euo pipefail

# shellcheck disable=SC1091
source /setup_condition.sh

print_runtime_summary
validate_runtime_artifacts

if package_prefix="$(ros2 pkg prefix go2w_d1_arm 2>/dev/null)"; then
  echo "[go2w_d1_arm] package prefix=${package_prefix}"
else
  runtime_error "ros2 cannot resolve package 'go2w_d1_arm' after sourcing /setup_condition.sh"
fi

if executables="$(ros2 pkg executables go2w_d1_arm 2>/dev/null)"; then
  echo "[go2w_d1_arm] package executables:"
  printf '%s\n' "$executables"
else
  runtime_error "ros2 cannot list executables for package 'go2w_d1_arm'"
fi

if command -v ip >/dev/null 2>&1; then
  echo "[go2w_d1_arm] ip -br addr:"
  ip -br addr
fi

echo "[go2w_d1_arm] doctor checks passed"
