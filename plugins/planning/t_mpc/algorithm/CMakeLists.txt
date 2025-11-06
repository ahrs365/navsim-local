cmake_minimum_required(VERSION 3.8)
project(mpc_planner VERSION 1.0.0 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# 设置位置无关代码（用于静态库）
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 打印配置信息
message(STATUS "========================================")
message(STATUS "MPC Planner - Pure C++ Project")
message(STATUS "========================================")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "========================================")

# 查找依赖包
find_package(Eigen3 REQUIRED)
message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")

# 选项：选择要构建的模块
option(BUILD_ROS_TOOLS_NO_ROS "Build ros_tools_no_ros library" ON)
option(BUILD_MPC_PLANNER_TYPES "Build mpc_planner_types library" ON)
option(BUILD_MPC_PLANNER_UTIL "Build mpc_planner_util library" ON)
option(BUILD_MPC_PLANNER_SOLVER "Build mpc_planner_solver library" ON)
option(BUILD_GUIDANCE_PLANNER "Build guidance_planner library" ON)

# 添加第三方库
message(STATUS "----------------------------------------")
message(STATUS "Adding third-party libraries...")
message(STATUS "----------------------------------------")

# yaml-cpp
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp/CMakeLists.txt")
  message(STATUS "Adding yaml-cpp...")
  add_subdirectory(third_party/yaml-cpp)
else()
  message(WARNING "yaml-cpp not found. Please run: git submodule update --init --recursive")
endif()

# 添加子项目
message(STATUS "----------------------------------------")
message(STATUS "Adding subprojects...")
message(STATUS "----------------------------------------")

# ros_tools_no_ros - 基础工具库（无 ROS 依赖）
if(BUILD_ROS_TOOLS_NO_ROS)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ros_tools_no_ros/CMakeLists.txt")
    message(STATUS "Adding ros_tools_no_ros...")
    add_subdirectory(ros_tools_no_ros)
  else()
    message(WARNING "ros_tools_no_ros not found")
  endif()
endif()

# mpc_planner_types - 数据类型定义
if(BUILD_MPC_PLANNER_TYPES)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/mpc_planner_types/CMakeLists.txt")
    message(STATUS "Adding mpc_planner_types...")
    add_subdirectory(mpc_planner_types)
  else()
    message(WARNING "mpc_planner_types not found")
  endif()
endif()

# mpc_planner_util - 工具库
if(BUILD_MPC_PLANNER_UTIL)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/mpc_planner_util/CMakeLists.txt")
    message(STATUS "Adding mpc_planner_util...")
    add_subdirectory(mpc_planner_util)
  else()
    message(WARNING "mpc_planner_util not found")
  endif()
endif()

# mpc_planner_solver - 求解器
if(BUILD_MPC_PLANNER_SOLVER)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/mpc_planner_solver/CMakeLists.txt")
    message(STATUS "Adding mpc_planner_solver...")
    add_subdirectory(mpc_planner_solver)
  else()
    message(WARNING "mpc_planner_solver not found")
  endif()
endif()

# guidance_planner - 引导规划器
if(BUILD_GUIDANCE_PLANNER)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/guidance_planner/CMakeLists.txt")
    message(STATUS "Adding guidance_planner...")
    add_subdirectory(guidance_planner)
  else()
    message(WARNING "guidance_planner not found")
  endif()
endif()

# DecompUtil - 分解工具（无 ROS 依赖）
option(BUILD_DECOMP_UTIL "Build DecompUtil library" ON)
if(BUILD_DECOMP_UTIL)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/DecompUtil/CMakeLists.txt")
    message(STATUS "Adding DecompUtil...")
    add_subdirectory(DecompUtil)
  else()
    message(WARNING "DecompUtil not found")
  endif()
endif()

# mpc_planner_modules - MPC 模块
option(BUILD_MPC_PLANNER_MODULES "Build mpc_planner_modules library" ON)
if(BUILD_MPC_PLANNER_MODULES)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/mpc_planner_modules/CMakeLists.txt")
    message(STATUS "Adding mpc_planner_modules...")
    add_subdirectory(mpc_planner_modules)
  else()
    message(WARNING "mpc_planner_modules not found")
  endif()
endif()

# mpc_planner - 核心规划器
option(BUILD_MPC_PLANNER "Build mpc_planner library" ON)
if(BUILD_MPC_PLANNER)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/mpc_planner/CMakeLists.txt")
    message(STATUS "Adding mpc_planner...")
    add_subdirectory(mpc_planner)
  else()
    message(WARNING "mpc_planner not found")
  endif()
endif()

# 打印构建总结
message(STATUS "========================================")
message(STATUS "Build Summary:")
message(STATUS "----------------------------------------")
message(STATUS "ros_tools_no_ros:       ${BUILD_ROS_TOOLS_NO_ROS}")
message(STATUS "mpc_planner_types:      ${BUILD_MPC_PLANNER_TYPES}")
message(STATUS "mpc_planner_util:       ${BUILD_MPC_PLANNER_UTIL}")
message(STATUS "mpc_planner_solver:     ${BUILD_MPC_PLANNER_SOLVER}")
message(STATUS "guidance_planner:       ${BUILD_GUIDANCE_PLANNER}")
message(STATUS "DecompUtil:             ${BUILD_DECOMP_UTIL}")
message(STATUS "mpc_planner_modules:    ${BUILD_MPC_PLANNER_MODULES}")
message(STATUS "mpc_planner:            ${BUILD_MPC_PLANNER}")
message(STATUS "========================================")

# ========================================
# 主程序可执行文件
# ========================================
option(BUILD_MAIN_EXECUTABLE "Build main executable" ON)
if(BUILD_MAIN_EXECUTABLE AND BUILD_MPC_PLANNER)
  message(STATUS "----------------------------------------")
  message(STATUS "Building main executable...")
  message(STATUS "----------------------------------------")

  # 设置 ACADOS 路径
  if(NOT DEFINED ENV{ACADOS_SOURCE_DIR})
    message(WARNING "ACADOS_SOURCE_DIR environment variable not set!")
    message(WARNING "Main executable will not be built.")
    set(BUILD_MAIN_EXECUTABLE OFF)
  else()
    set(ACADOS_SOURCE_DIR $ENV{ACADOS_SOURCE_DIR})
    message(STATUS "ACADOS_SOURCE_DIR: ${ACADOS_SOURCE_DIR}")

    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

    set(MATPLOTLIB_CPP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/matplotlib-cpp)

    # 创建主程序可执行文件
    add_executable(mpc_planner_main main.cpp)

    # 设置包含目录
    target_include_directories(mpc_planner_main
      PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/mpc_planner/include
        ${CMAKE_CURRENT_SOURCE_DIR}/mpc_planner_modules/include
        ${CMAKE_CURRENT_SOURCE_DIR}/mpc_planner_solver/include
        ${CMAKE_CURRENT_SOURCE_DIR}/mpc_planner_util/include
        ${CMAKE_CURRENT_SOURCE_DIR}/mpc_planner_types/include
        ${CMAKE_CURRENT_SOURCE_DIR}/ros_tools_no_ros/include
        ${CMAKE_CURRENT_SOURCE_DIR}/guidance_planner/include
        ${MATPLOTLIB_CPP_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp/include
        ${EIGEN3_INCLUDE_DIR}
        ${ACADOS_SOURCE_DIR}/include
        ${Python3_INCLUDE_DIRS}
    )

    # 链接库
    target_link_libraries(mpc_planner_main
      PRIVATE
        mpc_planner
        mpc_planner_modules
        mpc_planner_solver
        mpc_planner_util
        mpc_planner_types
        guidance_planner
        ros_tools_no_ros
        yaml-cpp
        ${Python3_LIBRARIES}
    )

    # 链接 ACADOS 库
    target_link_directories(mpc_planner_main
      PRIVATE
        ${ACADOS_SOURCE_DIR}/lib
    )

    target_link_libraries(mpc_planner_main
      PRIVATE
        acados
        blasfeo
        hpipm
        pthread
        dl
        m
    )

    # 安装主程序
    install(TARGETS mpc_planner_main
      RUNTIME DESTINATION bin
    )

    message(STATUS "Main executable configured: mpc_planner_main")
  endif()
endif()

# 安装规则
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/README.md
  DESTINATION share/doc/${PROJECT_NAME}
)

# 导出配置（可选）
# include(CMakePackageConfigHelpers)
# write_basic_package_version_file(
#   "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
#   VERSION ${PROJECT_VERSION}
#   COMPATIBILITY AnyNewerVersion
# )
