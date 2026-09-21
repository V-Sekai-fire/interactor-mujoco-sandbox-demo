cmake_minimum_required(VERSION 3.16)
project(mjtests CXX C)

get_filename_component(DEMO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(MJ "${DEMO_ROOT}/third_party/riscv64-mujoco")

add_subdirectory("${DEMO_ROOT}/third_party/sandbox-api/cmake" sandbox_cmake)

# The suite runs inside the emulator, because that is where the defects it
# looks for live. Nothing here runs on the host.
add_sandbox_program(mjtests
	main.cpp
	test_strings.cpp
	test_alloc.cpp
	test_xml.cpp
	test_wrapped_libc.cpp)

target_include_directories(mjtests PRIVATE
	"${MJ}/include"
	"${DEMO_ROOT}/third_party/witness-cpp/include"
	"${DEMO_ROOT}/third_party/doctest")

target_link_libraries(mjtests PRIVATE
	"${MJ}/lib/libmujoco.a" "${MJ}/lib/libccd.a" "${MJ}/lib/libtinyxml2.a"
	"${MJ}/lib/libqhullstatic_r.a" "${MJ}/lib/libtinyobjloader.a" "${MJ}/lib/libminiz.a" "${MJ}/lib/liblodepng.a"
	-lm -lstdc++)
