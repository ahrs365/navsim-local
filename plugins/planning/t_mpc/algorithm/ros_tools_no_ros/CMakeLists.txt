cmake_minimum_required(VERSION 3.10)
project(ros_tools_no_ros VERSION 1.0.0 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 查找依赖包
find_package(Eigen3 REQUIRED)

# 包含目录
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# 收集所有源文件
set(LIBRARY_SOURCES
    src/data_saver.cpp
    src/math.cpp
    src/profiling.cpp
    src/random_generator.cpp
    src/spline.cpp
    src/third_party/clothoid.cpp
    src/third_party/tkspline.cpp
)

# 创建库
add_library(${PROJECT_NAME} STATIC ${LIBRARY_SOURCES})

# 设置库的包含目录
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 链接 Eigen3
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

# 设置库的编译特性
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# 创建示例可执行文件
option(BUILD_EXAMPLES "Build example programs" ON)

if(BUILD_EXAMPLES)
    # 简单示例程序（无 ROS 依赖）
    add_executable(simple_example examples/simple_example.cpp)
    target_link_libraries(simple_example ${PROJECT_NAME})

    message(STATUS "Examples enabled")
endif()

# 测试部分
option(BUILD_TESTS "Build tests" ON)

if(BUILD_TESTS)
    # 查找 Google Test
    find_package(GTest QUIET)
    
    if(GTest_FOUND)
        enable_testing()
        
        # 创建测试可执行文件
        add_executable(test_spline test/test_spline.cpp)
        target_link_libraries(test_spline 
            ${PROJECT_NAME}
            GTest::GTest
            GTest::Main
        )
        
        # 添加测试
        add_test(NAME SplineTest COMMAND test_spline)
        
        message(STATUS "Tests enabled - GTest found")
    else()
        message(WARNING "GTest not found - tests will not be built. Install GTest to enable tests.")
    endif()
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
message(STATUS "Eigen3 Include: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Build Tests: ${BUILD_TESTS}")
message(STATUS "Build Examples: ${BUILD_EXAMPLES}")
message(STATUS "========================================")

