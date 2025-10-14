#!/bin/bash

# 测试可视化功能

echo "=========================================="
echo "NavSim Local - Visualization Test"
echo "=========================================="
echo ""

# 检查可执行文件
if [ ! -f "build/navsim_algo" ]; then
    echo "❌ navsim_algo not found!"
    echo "Please run: ./build_with_visualization.sh"
    exit 1
fi

echo "✅ navsim_algo found"
echo ""

# 检查配置文件
if [ ! -f "config/with_visualization.json" ]; then
    echo "❌ config/with_visualization.json not found!"
    exit 1
fi

echo "✅ Configuration file found"
echo ""

# 检查是否启用了可视化
if ! grep -q "ENABLE_VISUALIZATION" build/CMakeCache.txt 2>/dev/null; then
    echo "⚠️  Visualization might not be enabled in build"
fi

# 显示配置
echo "📋 Configuration:"
echo "  - Primary Planner: $(grep -A1 'primary_planner' config/with_visualization.json | tail -1 | cut -d'"' -f4)"
echo "  - Visualization: $(grep -A1 'enable_visualization' config/with_visualization.json | tail -1 | awk '{print $2}' | tr -d ',')"
echo ""

echo "=========================================="
echo "Ready to test!"
echo "=========================================="
echo ""
echo "To run with visualization:"
echo "  1. Start navsim-online server in another terminal:"
echo "     cd ../navsim-online && bash run_navsim.sh"
echo ""
echo "  2. Run this command:"
echo "     ./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/with_visualization.json"
echo ""
echo "Expected behavior:"
echo "  - A window should open showing the visualization"
echo "  - Press F to toggle follow ego mode"
echo "  - Press +/- to zoom"
echo "  - Press ESC to close"
echo ""

