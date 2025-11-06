#!/bin/bash
# T-MPC Planner Test Script

# 获取脚本所在目录（navsim-local 根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Set library path for ACADOS solver
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$SCRIPT_DIR/plugins/planning/t_mpc/algorithm/mpc_planner_solver/acados/Solver

# 从 build 目录运行（确保工作目录正确）
cd "$SCRIPT_DIR/build" && ./navsim_local_debug --scenario ../scenarios/map17.json --planner TMPCPlanner --verbose

