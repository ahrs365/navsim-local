cmake_minimum_required(VERSION 3.8)
project(decomp_util)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")

find_package(Eigen3 REQUIRED)

# Create interface library (header-only)
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

# Tests (optional)
option(BUILD_DECOMP_TESTS "Build DecompUtil tests" OFF)
if(BUILD_DECOMP_TESTS)
  include(CTest)

  add_executable(test_seed_decomp test/test_seed_decomp.cpp)
  target_link_libraries(test_seed_decomp ${PROJECT_NAME})
  add_test(test_seed_decomp test_seed_decomp)

  add_executable(test_line_segment test/test_line_segment.cpp)
  target_link_libraries(test_line_segment ${PROJECT_NAME})
  add_test(test_line_segment test_line_segment ${CMAKE_SOURCE_DIR}/data/obstacles.txt)

  add_executable(test_ellipsoid_decomp test/test_ellipsoid_decomp.cpp)
  target_link_libraries(test_ellipsoid_decomp ${PROJECT_NAME})
  add_test(test_ellipsoid_decomp test_ellipsoid_decomp ${CMAKE_SOURCE_DIR}/data/obstacles.txt)

  add_executable(test_iterative_decomp test/test_iterative_decomp.cpp)
  target_link_libraries(test_iterative_decomp ${PROJECT_NAME})
  add_test(test_iterative_decomp test_iterative_decomp ${CMAKE_SOURCE_DIR}/data/obstacles.txt)
endif()

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  INCLUDES DESTINATION include
)

install(DIRECTORY include/decomp_basis include/decomp_geometry include/decomp_util
  DESTINATION include
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)
