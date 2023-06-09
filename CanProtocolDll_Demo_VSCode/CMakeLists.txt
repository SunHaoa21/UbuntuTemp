cmake_minimum_required(VERSION 3.0.0)

project(CanProtocolDll_Demo_VSCode)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

link_directories(${PROJECT_SOURCE_DIR}/bin)
add_executable(CanProtocolDll_Demo_VSCode main.cpp)
target_link_libraries(CanProtocolDll_Demo_VSCode CanProtocolDll)
