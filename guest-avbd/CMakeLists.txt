cmake_minimum_required(VERSION 3.16)
project(avbdguest CXX)

get_filename_component(DEMO_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

add_subdirectory("${DEMO_ROOT}/third_party/sandbox-api/cmake" sandbox_cmake)

# avbd_cpu.cpp #includes the thirteen Slang kernel emits from gen/, so it is
# the only translation unit that needs the generated code; cloth_grid.cpp and
# main.cpp are ordinary. The Slang cpp prelude is vendored under slang-rt/ so
# the build depends on no host path.
add_sandbox_program(avbd main.cpp avbd_cpu.cpp cloth_grid.cpp)
target_include_directories(avbd PRIVATE
	"${CMAKE_CURRENT_LIST_DIR}/gen"
	"${CMAKE_CURRENT_LIST_DIR}/slang-rt")
target_link_libraries(avbd PRIVATE -lm -lstdc++)
