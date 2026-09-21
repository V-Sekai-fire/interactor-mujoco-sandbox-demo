cmake_minimum_required(VERSION 3.16)
project(minguest CXX C)

get_filename_component(DEMO_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
add_subdirectory("${DEMO_ROOT}/third_party/sandbox-api/cmake" sandbox_cmake)

add_sandbox_program(minstep main.cpp)
