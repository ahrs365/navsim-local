#!/bin/bash

# NavSim Local - 编译和运行脚本（新架构）

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_header() {
    echo ""
    echo "=========================================="
    echo "$1"
    echo "=========================================="
    echo ""
}

# 显示帮助信息
show_help() {
    cat << EOF
NavSim Local - 编译和运行脚本

用法: $0 [选项] [模式] [地图]

选项:
  -h, --help              显示此帮助信息
  -c, --clean             清理旧的构建
  -d, --debug             使用 Debug 构建类型
  -r, --release           使用 Release 构建类型
  --no-build              跳过编译，直接运行
  --build-only            只编译，不运行
  -m, --map <map_name>    指定地图文件（默认：map1）

模式:
  local                   本地仿真模式（从 JSON 文件加载场景）
  websocket               WebSocket 在线模式（从前端接收场景）

地图:
  map1                    默认地图（scenarios/map1.json）
  map2                    第二个地图（scenarios/map2.json）
  <custom>                自定义地图名称（scenarios/<custom>.json）

示例:
  # 编译并运行本地仿真模式（默认 map1）
  $0 local

  # 使用 map2 运行本地仿真模式
  $0 -m map2 local
  或
  $0 local map2

  # 编译并运行 WebSocket 在线模式
  $0 websocket

  # 清理并重新编译
  $0 -c local

  # 只编译，不运行
  $0 --build-only

  # 跳过编译，直接运行
  $0 --no-build local

EOF
}

# 默认参数
CLEAN_BUILD=false
BUILD_TYPE="RelWithDebInfo"
NO_BUILD=false
BUILD_ONLY=false
RUN_MODE=""
MAP_NAME="map1"  # 默认地图

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -c|--clean)
            CLEAN_BUILD=true
            shift
            ;;
        -d|--debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        -r|--release)
            BUILD_TYPE="Release"
            shift
            ;;
        --no-build)
            NO_BUILD=true
            shift
            ;;
        --build-only)
            BUILD_ONLY=true
            shift
            ;;
        -m|--map)
            MAP_NAME=$2
            shift 2
            ;;
        local|websocket)
            RUN_MODE=$1
            shift
            # 检查下一个参数是否是地图名称（不以 - 开头）
            if [[ $# -gt 0 && ! $1 =~ ^- ]]; then
                MAP_NAME=$1
                shift
            fi
            ;;
        *)
            print_error "未知参数: $1"
            echo ""
            show_help
            exit 1
            ;;
    esac
done

# 如果没有指定运行模式且不是只编译，显示帮助
if [ -z "$RUN_MODE" ] && [ "$BUILD_ONLY" = false ] && [ "$NO_BUILD" = false ]; then
    show_help
    exit 0
fi

# 切换到脚本所在目录
cd "$(dirname "$0")"

# ==================== 编译阶段 ====================

if [ "$NO_BUILD" = false ]; then
    print_header "NavSim Local - 编译"

    # 检查依赖
    print_info "检查依赖..."

    # 检查 ImGui（始终启用可视化）
    if [ ! -d "third_party/imgui" ]; then
        print_warning "ImGui not found!"
        print_info "下载 ImGui..."
        mkdir -p third_party
        cd third_party
        git clone https://github.com/ocornut/imgui.git --depth 1
        cd ..
        print_success "ImGui 下载成功"
    else
        print_success "ImGui 已存在"
    fi

    # 检查 SDL2
    if ! pkg-config --exists sdl2; then
        print_error "SDL2 未安装!"
        echo ""
        echo "请安装 SDL2:"
        echo "  Ubuntu/Debian: sudo apt-get install libsdl2-dev"
        echo "  macOS:         brew install sdl2"
        echo ""
        exit 1
    fi
    print_success "SDL2 已安装: $(pkg-config --modversion sdl2)"

    # 清理旧的构建
    if [ "$CLEAN_BUILD" = true ] && [ -d "build" ]; then
        print_info "清理旧的构建..."
        rm -rf build
        print_success "清理完成"
    fi

    # 配置 CMake
    print_info "配置 CMake..."
    echo "  - 构建类型: $BUILD_TYPE"
    echo "  - 可视化: 启用"
    echo "  - 插件: 启用"
    echo ""

    cmake -B build -S . \
        -DENABLE_VISUALIZATION=ON \
        -DBUILD_PLUGINS=ON \
        -DCMAKE_BUILD_TYPE=$BUILD_TYPE

    echo ""

    # 编译
    print_info "编译中..."
    NPROC=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
    cmake --build build -j$NPROC

    echo ""
    print_success "编译完成!"
    echo ""
fi

# 如果只编译，退出
if [ "$BUILD_ONLY" = true ]; then
    print_header "编译完成"
    print_info "可执行文件: ./build/navsim_algo"
    exit 0
fi

# ==================== 运行阶段 ====================

if [ -n "$RUN_MODE" ]; then
    print_header "NavSim Local - 运行"

    # 检查可执行文件是否存在
    if [ ! -f "build/navsim_algo" ]; then
        print_error "可执行文件不存在: build/navsim_algo"
        print_info "请先运行编译: $0 --build-only"
        exit 1
    fi

    case $RUN_MODE in
        local)
            # 检查地图文件是否存在
            SCENARIO_FILE="scenarios/${MAP_NAME}.json"
            if [ ! -f "$SCENARIO_FILE" ]; then
                print_error "地图文件不存在: $SCENARIO_FILE"
                echo ""
                print_info "可用的地图文件:"
                ls -1 scenarios/*.json 2>/dev/null | sed 's/scenarios\//  - /' | sed 's/\.json$//'
                exit 1
            fi

            print_info "运行模式: 本地仿真"
            print_info "场景文件: $SCENARIO_FILE"
            print_info "配置文件: config/default.json"
            echo ""

            print_info "控制键:"
            echo "  F       - 跟随自车"
            echo "  +/-     - 缩放"
            echo "  ESC     - 关闭窗口"
            echo "  Ctrl+C  - 停止仿真"
            echo ""
            print_header "开始运行"

            # 构建命令参数（路径相对于 build 目录）
            CMD_ARGS="--local-sim --scenario=../$SCENARIO_FILE --config=../config/default.json"

            # 从 build 目录运行，确保工作目录正确
            cd build && ./navsim_algo $CMD_ARGS
            ;;
            
        websocket)
            print_info "运行模式: WebSocket 在线"
            print_info "WebSocket URL: ws://127.0.0.1:8080/ws"
            print_info "Room ID: demo"
            print_info "配置文件: config/default.json"
            echo ""
            
            print_warning "请确保 navsim-online 服务器已启动!"
            print_info "启动服务器: cd ../navsim-online/server && python3 main.py"
            echo ""
            
            print_info "控制键:"
            echo "  Ctrl+C  - 停止连接"
            echo ""
            
            print_info "使用步骤:"
            echo "  1. 打开浏览器: http://localhost:8080"
            echo "  2. 在网页上绘制场景（起点、终点、障碍物）"
            echo "  3. 点击'开始'按钮触发仿真"
            echo "  4. 观察本地日志和前端可视化"
            echo ""
            print_header "开始运行"

            # 从 build 目录运行，确保工作目录正确
            cd build && ./navsim_algo ws://127.0.0.1:8080/ws demo \
                --config=../config/default.json
            ;;
    esac
fi
