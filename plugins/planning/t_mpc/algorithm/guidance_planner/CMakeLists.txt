cmake_minimum_required(VERSION 3.8)
project(guidance_planner)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# OpenMP
find_package(OpenMP)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(WARNING "Compiling without OPENMP")
endif()

# Find dependencies
find_package(Eigen3 REQUIRED)

# ADD GSL FOR HOMOTOPY
find_package(PkgConfig REQUIRED)
pkg_check_modules(gsl REQUIRED gsl)

# Find ros_tools_no_ros
if(NOT TARGET ros_tools_no_ros)
  find_package(ros_tools_no_ros REQUIRED)
endif()

# Find yaml-cpp
if(NOT TARGET yaml-cpp)
  find_package(yaml-cpp REQUIRED)
endif()

# Include directories
include_directories(
  include
  debug
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../ros_tools_no_ros/include
)

# Homotopy library
add_library(${PROJECT_NAME}_homotopy SHARED
  src/homotopy_comparison/uvd.cpp
  src/homotopy_comparison/winding_angle.cpp
  src/homotopy_comparison/homology.cpp
)
target_link_libraries(${PROJECT_NAME}_homotopy
  ${gsl_LIBRARIES}
)

# Main library
add_library(${PROJECT_NAME} SHARED
  src/config.cpp
  src/global_guidance.cpp
  src/prm.cpp
  src/sampler.cpp
  src/graph.cpp
  src/graph_search.cpp
  src/types/paths.cpp
  src/types/types.cpp
  src/types/node.cpp
  src/types/connection.cpp
  src/environment.cpp
  src/cubic_spline.cpp
  src/third_party/dubins.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_homotopy
  ros_tools_no_ros
  yaml-cpp
  Eigen3::Eigen
)

# Export include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME}_homotopy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install
install(TARGETS
  ${PROJECT_NAME}_homotopy
  ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE
  PATTERN "learning_types.h" EXCLUDE
  PATTERN "learning_guidance.h" EXCLUDE
)