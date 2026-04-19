#!/usr/bin/env bash
# tbai installer — clone the repo, then build tbai_sdk, tbai_mujoco,
# and tbai itself. Assumes the build environment (just, cmake, compilers,
# CONDA_PREFIX, ...) is already active.
#
# Usage (one-shot):
#   curl -LsSf https://raw.githubusercontent.com/tbai-lab/tbai/main/install.sh | bash
#
# Pick what to install via TBAI_TARGET (default: python):
#   TBAI_TARGET=cpp     install the C++ libraries only
#   TBAI_TARGET=python  install the Python bindings
#   curl -LsSf https://raw.githubusercontent.com/tbai-lab/tbai/main/install.sh | TBAI_TARGET=cpp bash

set -euo pipefail
rm -rf /tmp/tbai_* 2>/dev/null || true

REPO_URL="https://github.com/tbai-lab/tbai.git"
BRANCH="${BRANCH:-main}"
TARGET="${TBAI_TARGET:-python}"
TMP_DIR=$(mktemp -d -t tbai_XXXXXXXXXX)
JOBS=$(nproc 2>/dev/null || echo 4)
OLD_DIR=$(pwd)

case "$TARGET" in
    cpp|python) ;;
    *) printf '\033[1;31m[tbai]\033[0m invalid TBAI_TARGET=%q (expected: cpp, python)\n' "$TARGET" >&2; exit 1 ;;
esac

say() { printf '\033[1;36m[tbai]\033[0m %s\n' "$*"; }
err() { printf '\033[1;31m[tbai]\033[0m %s\n' "$*" >&2; }

need() {
    command -v "$1" >/dev/null 2>&1 || { err "missing required tool: '$1'. Make sure it is in your PATH."; exit 1; }
}

say "checking for required tools (target: $TARGET)"
need git
need just
need cmake
need rustc
need gcc
need python3
need pip

if [[ "$TARGET" == "python" ]]; then
    say "checking for required python packages"
    python3 -c "import torch" 2>/dev/null || { err "missing required Python package: torch. Install it with 'pip install torch'."; exit 1; }
fi

if [[ -d "$TMP_DIR/.git" ]]; then
    say "repo already present at $TMP_DIR — pulling latest"
    git -C "$TMP_DIR" pull --ff-only
else
    say "cloning $REPO_URL into $TMP_DIR"
    git clone --branch "$BRANCH" "$REPO_URL" "$TMP_DIR"
fi

cd "$TMP_DIR"

if [[ "$TARGET" == "python" ]]; then
    say "installing all python components"
    just install-all-python
fi

if [[ "$TARGET" == "cpp" ]]; then
    if [[ -z "${CONDA_PREFIX:-}" ]]; then
        err "CONDA_PREFIX is not set. Please activate a conda environment before running this script."
        exit 1
    fi
    say "installing all cpp components (parallel jobs: $JOBS)"
    just install-all-cpp
fi

cd $OLD_DIR
say "done. enjoy 🤗"