cmake_minimum_required(VERSION 3.8)
project(mpc_planner_types VERSION 1.0.0 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# 查找依赖包
find_package(Eigen3 REQUIRED)

# 设置依赖路径
set(ROS_TOOLS_NO_ROS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ros_tools_no_ros")

# 检查依赖是否存在
if(NOT EXISTS "${ROS_TOOLS_NO_ROS_DIR}/include")
  message(WARNING "ros_tools_no_ros not found at ${ROS_TOOLS_NO_ROS_DIR}")
endif()

# 源文件
set(LIBRARY_SOURCES
  src/data_types.cpp
  src/module_data.cpp
)

# 创建共享库
add_library(${PROJECT_NAME} SHARED ${LIBRARY_SOURCES})

# 设置库的包含目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${ROS_TOOLS_NO_ROS_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接依赖
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
)

# 链接 ros_tools_no_ros 库（如果作为子项目构建）
if(TARGET ros_tools_no_ros)
  target_link_libraries(${PROJECT_NAME} PUBLIC ros_tools_no_ros)
elseif(EXISTS "${ROS_TOOLS_NO_ROS_DIR}/build/libros_tools_no_ros.a")
  target_link_libraries(${PROJECT_NAME} PUBLIC
    ${ROS_TOOLS_NO_ROS_DIR}/build/libros_tools_no_ros.a
  )
endif()

# 安装规则
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# 打印配置信息
message(STATUS "========================================")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Eigen3: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "ros_tools_no_ros: ${ROS_TOOLS_NO_ROS_DIR}")
message(STATUS "========================================")