#!/usr/bin/env bash
# Compose helper: sets REPO_NAME from the checkout folder and runs docker compose from repo root.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export REPO_NAME="$(basename "$REPO_ROOT")"

cd "$REPO_ROOT"
exec docker compose -f docker/compose.jetson.yaml "$@"
