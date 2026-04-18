# TBAI justfile

zenoh_c_build_dir := "/tmp/tbai_zenoh_c_build"
zenoh_cpp_build_dir := "/tmp/tbai_zenoh_cpp_build"
tbai_sdk_build_dir := "/tmp/tbai_sdk_build"
tbai_mujoco_build_dir := "/tmp/tbai_mujoco_build"

# Show available commands
[group('utils')]
help:
    @just --list

# Format C++ files in all directories (excluding dependencies and build)
[group('utils')]
format:
    #!/usr/bin/env bash
    OCS2_THIRD_PARTY_DIR="tbai_ocs2/ocs2_thirdparty"
    folders=$(ls -d */ | grep -v dependencies | grep -v build | grep -v "$OCS2_THIRD_PARTY_DIR")
    for folder in $folders; do
        for file in $(find $folder -type f \( -name "*.hpp" -o -name "*.cpp" \) ! -path "$OCS2_THIRD_PARTY_DIR/*"); do
            echo "[TBAI] Formatting $file"
            clang-format -i -style=file $file
        done
    done

# Generate conda environments for all pixi environments
[group('utils')]
pixi-generate-conda-envs:
    #!/usr/bin/env bash
    set -euo pipefail
    all_envs=$(pixi workspace environment list | grep -E '^- ' | cut -d':' -f1 | sed 's/^- //')
    for env in $all_envs; do
        pixi workspace export conda-environment -e $env > .conda/$env.yaml
    done

# Install the project (moved from pixi.toml)
[group('utils')]
install:
    cmake -B/tmp/cpmbuild -S. -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX && cmake --build /tmp/cpmbuild --parallel 8 && cmake --build /tmp/cpmbuild --target install

# Build tests (moved from pixi.toml)
[group('utils')]
build-tests:
    cmake -Bbuild -S. \
        -DTBAI_BUILD_TESTS=ON \
        -DTBAI_BUILD_NP3O=ON \
        -DTBAI_BUILD_WTW=ON \
        -DTBAI_BUILD_BOB=OFF \
        -DTBAI_BUILD_MUSE=OFF \
        -DTBAI_BUILD_MPC=OFF \
        -DTBAI_BUILD_DTC=OFF \
        -DTBAI_BUILD_JOE=OFF \
        -DTBAI_BUILD_OCS2=OFF \
        -DTBAI_BUILD_PYTHON=OFF \
        -DTBAI_BUILD_DOCS=OFF \
        -DTBAI_FETCH_TORCH=OFF \
        -DTBAI_BUILD_DEPLOY_GO2=OFF \
        -DTBAI_BUILD_DEPLOY_GO2_UNITREE=OFF \
        -DTBAI_BUILD_DEPLOY_GO2W=OFF \
        -DTBAI_BUILD_DEPLOY_G1=OFF \
        -DTBAI_BUILD_DEPLOY_G1_UNITREE=OFF \
        -DTBAI_BUILD_DEPLOY_FRANKA=OFF \
        -DTBAI_BUILD_DEPLOY_SPOT=OFF \
        -DTBAI_BUILD_DEPLOY_ANYMAL_B=OFF \
        -DTBAI_BUILD_DEPLOY_ANYMAL_C=OFF \
        -DTBAI_BUILD_DEPLOY_ANYMAL_D=OFF \
    && cmake --build build --parallel 8

# Run tests (requires build-tests to have been run)
[group('utils')]
run-tests:
    cd build && ctest --output-on-failure

# Build and run tests
[group('utils')]
test: build-tests run-tests

# Build and install zenoh-c (requires Rust)
[group('zenoh')]
build-zenoh-c:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ -f "$CONDA_PREFIX/lib/libzenohc.so" ]; then
        echo "[TBAI] zenoh-c already installed, skipping."
        exit 0
    fi
    echo "[TBAI] Building zenoh-c (requires Rust)..."
    [ -d "thirdparty/zenoh-c" ] || \
        git clone --depth 1 https://github.com/eclipse-zenoh/zenoh-c.git thirdparty/zenoh-c
    cmake -S thirdparty/zenoh-c -B {{zenoh_c_build_dir}} \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX" \
        -DZENOHC_BUILD_WITH_UNSTABLE_API=ON \
        -DZENOHC_BUILD_WITH_SHARED_MEMORY=ON
    cmake --build {{zenoh_c_build_dir}} --config Release -j"$(nproc)" --target install
    echo "[TBAI] zenoh-c installed."

# Build and install zenoh-cpp headers
[group('zenoh')]
build-zenoh-cpp: build-zenoh-c
    #!/usr/bin/env bash
    set -euo pipefail
    if [ -f "$CONDA_PREFIX/lib/cmake/zenohcxx/zenohcxxConfig.cmake" ]; then
        echo "[TBAI] zenoh-cpp already installed, skipping."
        exit 0
    fi
    echo "[TBAI] Installing zenoh-cpp headers..."
    [ -d "thirdparty/zenoh-cpp" ] || \
        git clone --depth 1 https://github.com/eclipse-zenoh/zenoh-cpp.git thirdparty/zenoh-cpp
    cmake -S thirdparty/zenoh-cpp -B {{zenoh_cpp_build_dir}} \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX" \
        -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
        -DZENOHCXX_ZENOHC=ON -DZENOHCXX_ZENOHPICO=OFF
    cmake --build {{zenoh_cpp_build_dir}} --config Release -j"$(nproc)" --target install
    echo "[TBAI] zenoh-cpp installed."

# Clone tbai_sdk (skips if already exists)
[group('tbai_sdk')]
clone-tbai-sdk:
    #!/usr/bin/env bash
    set -euo pipefail
    TBAI_SDK_DIR="thirdparty/tbai_sdk"
    if [[ ! -d "$TBAI_SDK_DIR" ]]; then
        echo "[TBAI] Cloning tbai_sdk..."
        git clone --depth 1 git@github.com:tbai-lab/tbai_sdk.git "$TBAI_SDK_DIR"
    else
        echo "[TBAI] tbai_sdk already exists at $TBAI_SDK_DIR"
        if [[ -d "$TBAI_SDK_DIR/.git" ]]; then
            echo "[TBAI] Pulling latest changes..."
            git -C "$TBAI_SDK_DIR" pull
        fi
    fi

# Build and install tbai_sdk
[group('tbai_sdk')]
build-tbai-sdk: build-zenoh-cpp clone-tbai-sdk
    #!/usr/bin/env bash
    set -euo pipefail
    TBAI_SDK_DIR="thirdparty/tbai_sdk"
    if [ -f "$CONDA_PREFIX/lib/cmake/tbai_sdk/tbai_sdkConfig.cmake" ]; then
        echo "[TBAI] tbai_sdk already installed, skipping."
        exit 0
    fi
    echo "[TBAI] Building and installing tbai_sdk..."
    cmake -S "$TBAI_SDK_DIR" -B {{tbai_sdk_build_dir}} \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX"
    cmake --build {{tbai_sdk_build_dir}} --parallel "$(nproc)" --target install
    echo "[TBAI] tbai_sdk installed."

# Clone tbai_mujoco (skips if already exists)
[group('tbai_mujoco')]
clone-tbai-mujoco:
    #!/usr/bin/env bash
    set -euo pipefail
    if [[ ! -d "thirdparty/tbai_mujoco" ]]; then
        echo "[TBAI] Cloning tbai_mujoco..."
        git clone --depth 1 git@github.com:tbai-lab/tbai_mujoco.git thirdparty/tbai_mujoco
    else
        echo "[TBAI] tbai_mujoco already exists"
        if [[ -d "thirdparty/tbai_mujoco/.git" ]]; then
            echo "[TBAI] Pulling latest changes..."
            git -C thirdparty/tbai_mujoco pull
        fi
    fi

# Build tbai_mujoco
[group('tbai_mujoco')]
build-tbai-mujoco: build-tbai-sdk clone-tbai-mujoco
    #!/usr/bin/env bash
    set -euo pipefail
    echo "[TBAI] Building tbai_mujoco..."
    cmake -S thirdparty/tbai_mujoco -B {{tbai_mujoco_build_dir}} \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX"
    cmake --build {{tbai_mujoco_build_dir}} --parallel "$(nproc)" --target install
    echo "[TBAI] tbai_mujoco installed."
