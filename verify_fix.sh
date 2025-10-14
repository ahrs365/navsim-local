#!/bin/bash

# 快速验证修复是否生效

echo "=========================================="
echo "验证仿真开始信号修复"
echo "=========================================="
echo ""

echo "🔍 检查关键修改..."
echo ""

# 检查 navsim-online 修改
echo "1. 检查 navsim-online/server/main.py..."
if grep -q "include_static_next_tick = True" ../navsim-online/server/main.py; then
    if grep -A 2 "command == \"resume\" or command == \"start\"" ../navsim-online/server/main.py | grep -q "include_static_next_tick = True"; then
        echo "   ✅ navsim-online: 开始时发送静态地图 - OK"
    else
        echo "   ❌ navsim-online: 开始时发送静态地图 - MISSING"
    fi
else
    echo "   ❌ navsim-online: 修改未找到"
fi

# 检查 navsim-local Bridge 修改
echo "2. 检查 navsim-local Bridge..."
if grep -q "SimulationStateCallback" include/core/bridge.hpp; then
    echo "   ✅ Bridge: SimulationStateCallback - OK"
else
    echo "   ❌ Bridge: SimulationStateCallback - MISSING"
fi

if grep -q "/sim_ctrl" src/core/bridge.cpp; then
    echo "   ✅ Bridge: /sim_ctrl 消息处理 - OK"
else
    echo "   ❌ Bridge: /sim_ctrl 消息处理 - MISSING"
fi

# 检查 AlgorithmManager 修改
echo "3. 检查 AlgorithmManager..."
if grep -q "simulation_started_" include/core/algorithm_manager.hpp; then
    echo "   ✅ AlgorithmManager: simulation_started_ - OK"
else
    echo "   ❌ AlgorithmManager: simulation_started_ - MISSING"
fi

if grep -q "isSimulationStarted" include/core/algorithm_manager.hpp; then
    echo "   ✅ AlgorithmManager: isSimulationStarted() - OK"
else
    echo "   ❌ AlgorithmManager: isSimulationStarted() - MISSING"
fi

# 检查 main.cpp 修改
echo "4. 检查 main.cpp..."
if grep -q "set_simulation_state_callback" src/core/main.cpp; then
    echo "   ✅ main.cpp: 设置仿真状态回调 - OK"
else
    echo "   ❌ main.cpp: 设置仿真状态回调 - MISSING"
fi

if grep -q "isSimulationStarted" src/core/main.cpp; then
    echo "   ✅ main.cpp: 检查仿真状态 - OK"
else
    echo "   ❌ main.cpp: 检查仿真状态 - MISSING"
fi

echo ""
echo "=========================================="
echo "📝 预期行为："
echo "=========================================="
echo ""
echo "启动 navsim-local 后，应该看到："
echo "  [Main] ⏸️  Waiting for simulation to start..."
echo "  [Main] Please click the 'Start' button in the Web interface"
echo ""
echo "并且 **不应该** 看到："
echo "  [DEBUG] Sending plan:"
echo "  [Bridge] Sent plan with X points"
echo ""
echo "点击 Web 界面的'开始'按钮后，应该看到："
echo "  [Bridge] ✅ Simulation STARTED"
echo "  [Main] ✅ Simulation STARTED"
echo "  [BEVExtractor] Has static_map: 1"
echo "  [DEBUG] Sending plan:"
echo ""
echo "=========================================="
echo "🧪 运行测试："
echo "=========================================="
echo ""
echo "请按以下步骤测试："
echo ""
echo "1. 确保 navsim-online 正在运行"
echo "2. 运行: ./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/with_visualization.json"
echo "3. 观察日志，确认没有 '[DEBUG] Sending plan:' 消息"
echo "4. 在 Web 界面放置障碍物"
echo "5. 点击'开始'按钮"
echo "6. 观察日志，确认收到 '[Bridge] ✅ Simulation STARTED' 消息"
echo "7. 观察可视化窗口，确认看到橙色圆形（静态障碍物）"
echo ""

