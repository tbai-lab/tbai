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
#
# Pick which modules to build (comma-separated, case-insensitive). Unset = all defaults.
#   TBAI_INSTALL_ROBOTS       go2, go2_unitree, go2w, g1, g1_unitree, franka, spot,
#                             anymal_b, anymal_c, anymal_d   (or "all" / "none")
#   TBAI_INSTALL_CONTROLLERS  bob, wtw, np3o, mpc, dtc, joe   (or "all" / "none")
# Example:
#   TBAI_INSTALL_ROBOTS=go2 TBAI_INSTALL_CONTROLLERS=bob,np3o ./install.sh

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

ALL_ROBOTS=(GO2 GO2_UNITREE GO2W G1 G1_UNITREE FRANKA SPOT ANYMAL_B ANYMAL_C ANYMAL_D)
ALL_CONTROLLERS=(BOB WTW NP3O MPC DTC JOE)
# Controllers that pull in the OCS2 framework; if any are enabled, OCS2 must be too.
OCS2_DEPENDENTS=(MPC DTC JOE)

declare -a MODULE_FLAGS=()

# pick_modules <label> <selection-csv> <cmake_prefix> <valid-name>...
# Empty selection -> CMakeLists defaults (no flags emitted).
# Otherwise emit -D<prefix><NAME>=ON for chosen names and =OFF for the rest.
pick_modules() {
    local label="$1" selection="$2" prefix="$3"
    shift 3
    local -a all=("$@")
    [[ -z "$selection" ]] && return 0

    selection="${selection,,}"
    selection="${selection// /,}"

    local -a chosen=()
    if [[ "$selection" == "all" ]]; then
        chosen=("${all[@]}")
    elif [[ "$selection" == "none" ]]; then
        chosen=()
    else
        local -a raw_arr=()
        IFS=',' read -ra raw_arr <<< "$selection"
        local raw up ok v
        for raw in "${raw_arr[@]}"; do
            [[ -z "$raw" ]] && continue
            up="${raw^^}"
            ok=0
            for v in "${all[@]}"; do [[ "$v" == "$up" ]] && ok=1; done
            if [[ $ok -eq 0 ]]; then
                local lower="${all[*]}"; lower="${lower,,}"
                err "unknown $label: '$raw'. Valid: $lower"
                exit 1
            fi
            chosen+=("$up")
        done
    fi

    local pretty=""
    for v in "${chosen[@]}"; do pretty+="${v,,} "; done
    say "$label: ${pretty:-<none>}"

    for v in "${all[@]}"; do
        local on=OFF c
        for c in "${chosen[@]}"; do [[ "$v" == "$c" ]] && on=ON; done
        MODULE_FLAGS+=("-D${prefix}${v}=${on}")
    done
}

pick_modules robots      "${TBAI_INSTALL_ROBOTS:-}"      "TBAI_BUILD_DEPLOY_" "${ALL_ROBOTS[@]}"
pick_modules controllers "${TBAI_INSTALL_CONTROLLERS:-}" "TBAI_BUILD_"        "${ALL_CONTROLLERS[@]}"

# OCS2 isn't a controller — it's a framework that mpc/dtc/joe link against. If
# the user picked any of them, force OCS2 on; otherwise leave it off so the
# user's "only build what I need" intent stands. When no controller selection
# was made (defaults), don't touch OCS2 — CMakeLists' default (ON) applies.
if [[ -n "${TBAI_INSTALL_CONTROLLERS:-}" ]]; then
    ocs2_state=OFF
    for f in "${MODULE_FLAGS[@]}"; do
        for dep in "${OCS2_DEPENDENTS[@]}"; do
            [[ "$f" == "-DTBAI_BUILD_${dep}=ON" ]] && ocs2_state=ON
        done
    done
    MODULE_FLAGS+=("-DTBAI_BUILD_OCS2=${ocs2_state}")
    say "ocs2 framework: ${ocs2_state,,} (auto)"
fi

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

say "building tbai_sdk"
just build-tbai-sdk

say "building tbai_mujoco"
just build-tbai-mujoco

if [[ "$TARGET" == "python" ]]; then
    say "building tbai python bindings (parallel jobs: $JOBS)"
    CMAKE_ARGS="${CMAKE_ARGS:-} ${MODULE_FLAGS[*]}" \
        pip install ./tbai_python --verbose --no-build-isolation
fi

if [[ "$TARGET" == "cpp" ]]; then
    if [[ -z "${CONDA_PREFIX:-}" ]]; then
        err "CONDA_PREFIX is not set. Please activate a conda environment before running this script."
        exit 1
    fi
    say "building tbai and installing to $CONDA_PREFIX (parallel jobs: $JOBS)"
    cmake -Bbuild -DCMAKE_BUILD_TYPE=Release "${MODULE_FLAGS[@]}"
    cmake --build build --parallel "$JOBS" --target install
fi

say "installing python dependencies"
pip install -r ./tbai_examples/requirements.txt


cd $OLD_DIR
say "done. enjoy 🤗"