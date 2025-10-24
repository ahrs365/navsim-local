#!/bin/bash

# 测试可视化线程模型修复
# 用法: ./test_visualization_fix.sh

set -e

echo "========================================="
echo "  可视化线程模型修复测试"
echo "========================================="
echo ""

# 检查是否在正确的目录
if [ ! -f "CMakeLists.txt" ]; then
    echo "❌ 错误：请在 navsim-local 目录下运行此脚本"
    exit 1
fi

# 检查是否已编译
if [ ! -f "build/navsim_algo" ]; then
    echo "⚠️  未找到 navsim_algo 可执行文件"
    echo "正在编译..."
    mkdir -p build
    cd build
    cmake .. -DENABLE_VISUALIZATION=ON -DBUILD_PLUGINS=ON
    make -j$(nproc)
    cd ..
    echo "✅ 编译完成"
    echo ""
fi

# 检查场景文件
if [ ! -f "scenarios/map1.json" ]; then
    echo "❌ 错误：未找到场景文件 scenarios/map1.json"
    exit 1
fi

echo "========================================="
echo "  测试 1: 可视化模式（主线程）"
echo "========================================="
echo ""
echo "预期行为："
echo "  ✅ 窗口立即显示内容（无黑屏）"
echo "  ✅ 规划日志和可视化同步更新"
echo "  ✅ 按 Ctrl+C 或关闭窗口退出"
echo ""
echo "启动命令："
echo "  ./build/navsim_algo --local-sim --scenario=scenarios/map1.json --visualize"
echo ""
read -p "按 Enter 键启动可视化模式测试..."

./build/navsim_algo --local-sim --scenario=scenarios/map1.json --visualize

echo ""
echo "========================================="
echo "  测试 2: 无可视化模式（仿真线程）"
echo "========================================="
echo ""
echo "预期行为："
echo "  ✅ 仿真在单独线程运行"
echo "  ✅ 主线程等待中断信号"
echo "  ✅ 按 Ctrl+C 退出"
echo ""
echo "启动命令："
echo "  ./build/navsim_algo --local-sim --scenario=scenarios/map1.json"
echo ""
read -p "按 Enter 键启动无可视化模式测试（按 Ctrl+C 停止）..."

timeout 5 ./build/navsim_algo --local-sim --scenario=scenarios/map1.json || true

echo ""
echo "========================================="
echo "  测试完成"
echo "========================================="
echo ""
echo "如果两个测试都正常运行，说明修复成功！"
echo ""

