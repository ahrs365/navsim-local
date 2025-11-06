cmake_minimum_required(VERSION 3.8)
project(mpc_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find OpenMP
find_package(OpenMP)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(WARNING "Compiling without OPENMP")
endif()

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Build library
add_library(${PROJECT_NAME} SHARED
  src/planner.cpp
  src/data_preparation.cpp
  src/experiment_util.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    mpc_planner_modules
    mpc_planner_solver
    mpc_planner_util
    mpc_planner_types
    ros_tools_no_ros
)

# Note: MPC_PLANNER_ROS is NOT defined for pure C++ build
# If you need ROS features (visualization, data saving), define it manually
