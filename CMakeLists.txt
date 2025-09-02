cmake_minimum_required(VERSION 3.15)
project(Forma CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages first
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(Boost REQUIRED)

# ignore BOOST deprecated headers
add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")

add_library(forma SHARED
    src/forma/forma.cpp
    src/object.hpp
)
target_include_directories(forma PUBLIC 
${OpenCV_INCLUDE_DIRS}
src
)

# Link libraries
target_link_libraries(forma PRIVATE 
    # 链接 Boost 的头文件接口
    Boost::headers
    # 链接所有找到的 OpenCV 库
    ${OpenCV_LIBS}

)

# CMAKE_CURRENT_BINARY_DIR 包含了生成的头文件，需要添加到包含路径
target_include_directories(forma PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Create executable
add_executable(tests src/tests.cpp)
target_link_libraries(tests PRIVATE forma ${OpenCV_LIBS})