#!/bin/bash
set -euo pipefail

export DOCKER_API_VERSION=1.43

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_NAME="go2w_d1_arm"

if [ "${1:-}" = "build" ]; then
  if [ ! -f "$REPO_ROOT/third_party/cyclonedds/CMakeLists.txt" ]; then
    echo "ERROR: third_party/cyclonedds submodule is not populated." >&2
    echo "Run from the repository root:" >&2
    echo "  git submodule update --init --recursive" >&2
    exit 1
  fi

  if [ ! -f "$REPO_ROOT/third_party/cyclonedds-cxx/CMakeLists.txt" ]; then
    echo "ERROR: third_party/cyclonedds-cxx submodule is not populated." >&2
    echo "Run from the repository root:" >&2
    echo "  git submodule update --init --recursive" >&2
    exit 1
  fi
fi

exec docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.yml" "$@"
