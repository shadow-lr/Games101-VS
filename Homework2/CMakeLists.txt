cmake_minimum_required(VERSION 3.10)
project(Homework2)

set(OpenCV_DIR D:\\mingw64\\WinGW64_build\\install)

find_package(OpenCV REQUIRED)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)
include_directories(/user/include/eigen3)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Homework2 main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)

target_link_libraries(Homework2 ${OpenCV_LIBRARIES})
target_link_libraries (Homework2 Eigen3::Eigen)
#target_link_libraries(Homework2 ${OpenCV_LIBS})