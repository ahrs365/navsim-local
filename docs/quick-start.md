# 快速上手指南

希望立刻体验 NavSim Local？照着下面几个步骤走就能跑起来：


## 1. 手动准备环境

- Linux / macOS（建议）
- C++17 编译器（GCC / Clang）
- CMake ≥ 3.16
- Protobuf（编译器及开发库）
- SDL2 开发包（Linux: `sudo apt-get install libsdl2-dev`，macOS: `brew install sdl2`）

首次运行脚本会自动拉取 ImGui / ImPlot；如果失败，可手动执行：

```bash
cd navsim-local
git submodule update --init --recursive   # 若启用了子模块
mkdir -p third_party && cd third_party
git clone https://github.com/ocornut/imgui.git --depth 1
git clone https://github.com/epezent/implot.git --depth 1
```


## 2. 构建 + 运行

```bash
cd navsim-local
./build.sh local
```

脚本会：

1. 检查/下载 ImGui & ImPlot；
2. 使用 `RelWithDebInfo` 配置 CMake（启用可视化与内置插件）；
3. 在 `build/` 下编译二进制；
4. 自动运行本地仿真，默认加载 `scenarios/map1.json`。

若只想编译、不运行：

```bash
./build.sh --build-only
```


## 3. 最小命令行运行

```bash
./build/navsim_algo \
  --local-sim \
  --scenario=scenarios/map3.json \
  --config=config/default.json
```

常用操作：

- 鼠标滑轮放大
- 右键拖动
- 点击start开始仿真
- 仿真结束后，点击reset,再点击start,重新仿真
- 直接在输入栏中填 map1.json，然后点击load,即可加载新的地图

---

## 进阶：切换场景与配置

- 场景文件位于 `scenarios/`，JSON 含起终点、障碍物、底盘约束等。
- 可复制 `config/default.json`，修改插件/规划器参数后通过 `--config=<your_config>` 加载。

可通过在线场景编辑器进行绘制地图：[网址](https://www.gl-robotics.com/navsim-online/index.html)
。下载地图为json文件，把文件放入navsim-local/scenarios/目录中即可

---

就这么简单，祝玩得愉快 🚀！
