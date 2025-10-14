#!/bin/bash

# 静态障碍物显示测试脚本

echo "=========================================="
echo "静态障碍物显示测试"
echo "=========================================="
echo ""

echo "📋 测试步骤："
echo ""
echo "1. 确保 navsim-online 已经启动"
echo "   cd ../navsim-online && ./run_navsim.sh"
echo ""
echo "2. 在 Web 界面添加静态障碍物："
echo "   - 打开 http://localhost:8080"
echo "   - 勾选'静态圆形'"
echo "   - 点击'放置'按钮"
echo "   - 在场景中点击几个位置"
echo "   - 点击'提交地图'按钮"
echo ""
echo "3. 运行 navsim-local（按 Ctrl+C 停止）"
echo ""
echo "=========================================="
echo ""

# 检查 navsim-online 是否运行
if ! curl -s http://localhost:8080 > /dev/null 2>&1; then
    echo "❌ 错误：navsim-online 未运行！"
    echo ""
    echo "请先启动 navsim-online："
    echo "  cd ../navsim-online && ./run_navsim.sh"
    echo ""
    exit 1
fi

echo "✅ navsim-online 正在运行"
echo ""

# 检查可执行文件是否存在
if [ ! -f "./build/navsim_algo" ]; then
    echo "❌ 错误：navsim_algo 未编译！"
    echo ""
    echo "请先编译："
    echo "  cmake --build build -j\$(nproc)"
    echo ""
    exit 1
fi

echo "✅ navsim_algo 已编译"
echo ""

# 检查配置文件是否存在
if [ ! -f "./config/with_visualization.json" ]; then
    echo "❌ 错误：配置文件不存在！"
    echo ""
    echo "请检查 config/with_visualization.json"
    echo ""
    exit 1
fi

echo "✅ 配置文件存在"
echo ""

echo "=========================================="
echo "🚀 启动 navsim-local（带可视化）"
echo "=========================================="
echo ""
echo "📝 查看日志，应该看到："
echo ""
echo "  [Room demo] New client connected, will send static map in next tick"
echo "  [BEVExtractor] Has static_map: 1"
echo "  [BEVExtractor] StaticMap circles: N  (N > 0)"
echo "  [BEVExtractor] Extracted circles: M  (M > 0)"
echo ""
echo "🎨 查看可视化窗口，应该看到："
echo ""
echo "  🟠 橙色圆形 - 静态障碍物"
echo "  🟢 绿色圆形 + 箭头 - 自车"
echo "  🔴 红色圆形 - 目标点"
echo "  🔵 青色线条 - 规划轨迹"
echo ""
echo "=========================================="
echo ""

# 运行 navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/with_visualization.json

