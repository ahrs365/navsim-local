#!/usr/bin/env bash
# Helper script to configure, build, and run the pure C++ MPC simulation.

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-${PROJECT_ROOT}/build/pure_cpp}"
CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"

# You may export ACADOS_SOURCE_DIR before calling this script.
# If it is not set we try to fall back to a local third_party checkout.
DEFAULT_ACADOS_DIR="${PROJECT_ROOT}/third_party/acados"
ACADOS_DIR="${ACADOS_SOURCE_DIR:-${DEFAULT_ACADOS_DIR}}"

if [[ ! -d "${ACADOS_DIR}" ]]; then
    echo "[ERROR] ACADOS directory not found."
    echo "        Export ACADOS_SOURCE_DIR=/absolute/path/to/acados, or"
    echo "        clone acados into ${DEFAULT_ACADOS_DIR}."
    exit 1
fi

export ACADOS_SOURCE_DIR="${ACADOS_DIR}"
export LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH:-}"

if [[ -z "${LD_PRELOAD:-}" ]]; then
    for candidate in /usr/lib/x86_64-linux-gnu/libopenblas.so.0 /usr/lib/x86_64-linux-gnu/libblas.so.3; do
        if [[ -f "${candidate}" ]]; then
            export LD_PRELOAD="${candidate}"
            echo "[INFO] Using LD_PRELOAD=${LD_PRELOAD} to avoid BLAS symbol clashes"
            break
        fi
    done
fi

echo "[INFO] Using ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}"
echo "[INFO] Updated LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"
echo "[INFO] Current LD_PRELOAD=${LD_PRELOAD:-<unset>}"
echo "[INFO] Build directory: ${BUILD_DIR}"
echo "[INFO] Build type: ${CMAKE_BUILD_TYPE}"

cmake -S "${PROJECT_ROOT}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"

cmake --build "${BUILD_DIR}" --target mpc_planner_main -j"$(nproc)"

# 统一使用 mpc_planner_jackalsimulator/config 作为配置目录
CONFIG_INPUT="${1:-${PROJECT_ROOT}/mpc_planner_jackalsimulator/config}"
if [[ -d "${CONFIG_INPUT}" ]]; then
    CONFIG_PATH="${CONFIG_INPUT}"
elif [[ -f "${CONFIG_INPUT}" ]]; then
    CONFIG_PATH="${CONFIG_INPUT}"
else
    echo "[ERROR] Configuration path '${CONFIG_INPUT}' does not exist."
    exit 1
fi

echo "[INFO] Launching simulation with config: ${CONFIG_PATH}"
echo "[HINT] Ensure your Python environment provides matplotlib (required by matplotlib-cpp)."

"${BUILD_DIR}/mpc_planner_main" "${CONFIG_PATH}"
