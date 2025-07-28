cmake_minimum_required(VERSION 3.15)
project(Forma CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages first
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(Boost REQUIRED)

set(SOURCES
    src/tests.cpp
    src/forma/forma.cpp
)

# Create executable
add_executable(tests ${SOURCES})
target_include_directories(tests PRIVATE 
${OpenCV_INCLUDE_DIRS}
src
)

# Link libraries
target_link_libraries(tests PRIVATE 
    # 链接 Boost 的头文件接口
    Boost::headers
    # 链接所有找到的 OpenCV 库
    ${OpenCV_LIBS}

)