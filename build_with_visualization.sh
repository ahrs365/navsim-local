#!/bin/bash

# NavSim Local - 带可视化编译脚本

set -e  # 遇到错误立即退出

echo "=========================================="
echo "NavSim Local - Build with Visualization"
echo "=========================================="
echo ""

# 检查 ImGui 是否存在
if [ ! -d "third_party/imgui" ]; then
    echo "❌ ImGui not found!"
    echo "📥 Downloading ImGui..."
    cd third_party
    git clone https://github.com/ocornut/imgui.git --depth 1
    cd ..
    echo "✅ ImGui downloaded successfully"
    echo ""
fi

# 检查 SDL2 是否安装
echo "🔍 Checking SDL2..."
if ! pkg-config --exists sdl2; then
    echo "❌ SDL2 not found!"
    echo ""
    echo "Please install SDL2:"
    echo "  Ubuntu/Debian: sudo apt-get install libsdl2-dev"
    echo "  macOS:         brew install sdl2"
    echo ""
    exit 1
fi
echo "✅ SDL2 found: $(pkg-config --modversion sdl2)"
echo ""

# 清理旧的构建
if [ -d "build" ]; then
    echo "🧹 Cleaning old build..."
    rm -rf build
fi

# 配置 CMake
echo "⚙️  Configuring CMake with visualization enabled..."
cmake -B build -S . \
    -DENABLE_VISUALIZATION=ON \
    -DBUILD_PLUGINS=ON \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo ""

# 编译
echo "🔨 Building..."
cmake --build build -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

echo ""
echo "=========================================="
echo "✅ Build completed successfully!"
echo "=========================================="
echo ""
echo "📝 Usage:"
echo "  ./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/with_visualization.json"
echo ""
echo "⌨️  Controls:"
echo "  F       - Toggle follow ego"
echo "  +/-     - Zoom in/out"
echo "  ESC     - Close window"
echo ""

