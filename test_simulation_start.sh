#!/bin/bash

# 测试仿真开始信号修复
# 此脚本用于验证静态障碍物显示问题的修复

echo "=========================================="
echo "仿真开始信号修复 - 测试脚本"
echo "=========================================="
echo ""

echo "📋 测试目标："
echo "  1. navsim-local 连接后不立即执行算法"
echo "  2. 用户可以先在 Web 界面放置障碍物"
echo "  3. 点击'开始'按钮后，算法开始执行"
echo "  4. 静态障碍物正确显示在可视化窗口中"
echo ""

echo "🔧 前置条件检查："
echo ""

# 检查 navsim-local 是否已编译
if [ ! -f "./build/navsim_algo" ]; then
    echo "❌ navsim_algo 未找到，请先编译："
    echo "   cmake --build build -j\$(nproc)"
    exit 1
fi
echo "✅ navsim_algo 已编译"

# 检查配置文件
if [ ! -f "./config/with_visualization.json" ]; then
    echo "❌ 配置文件未找到：config/with_visualization.json"
    exit 1
fi
echo "✅ 配置文件存在"

echo ""
echo "=========================================="
echo "📝 测试步骤："
echo "=========================================="
echo ""

echo "步骤 1: 启动 navsim-online"
echo "----------------------------------------"
echo "请在另一个终端运行："
echo "  cd navsim-online"
echo "  python3 -m server.main"
echo ""
echo "等待看到以下输出："
echo "  [Room demo] Room created"
echo "  WebSocket server started on ws://0.0.0.0:8080/ws"
echo ""
read -p "navsim-online 是否已启动？(y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "❌ 请先启动 navsim-online"
    exit 1
fi
echo "✅ navsim-online 已启动"
echo ""

echo "步骤 2: 启动 navsim-local"
echo "----------------------------------------"
echo "即将启动 navsim-local，请观察以下内容："
echo ""
echo "预期输出："
echo "  [Bridge] Connecting to ws://127.0.0.1:8080/ws?room=demo"
echo "  [Bridge] WebSocket connection opened"
echo "  [Main] ⏸️  Waiting for simulation to start..."
echo "  [Main] Please click the 'Start' button in the Web interface"
echo ""
echo "可视化窗口："
echo "  - 状态栏显示：⏸️ Waiting for START button"
echo "  - 窗口保持响应，但不执行算法"
echo ""
read -p "按 Enter 键启动 navsim-local..." 
echo ""

echo "🚀 启动 navsim-local..."
echo ""
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/with_visualization.json

echo ""
echo "=========================================="
echo "测试完成"
echo "=========================================="

