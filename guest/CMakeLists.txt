cmake_minimum_required(VERSION 3.16)
project(mjguest CXX C)

get_filename_component(DEMO_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(MJ "${DEMO_ROOT}/third_party/riscv64-mujoco")

add_subdirectory("${DEMO_ROOT}/third_party/sandbox-api/cmake" sandbox_cmake)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

# The .xml is the source of truth; the header is regenerated from it on every
# build so the model in the binary cannot drift from the model in the repo.
add_custom_command(
	OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/model_data.h"
	COMMAND ${Python3_EXECUTABLE} "${DEMO_ROOT}/scripts/embed_model.py"
			"${DEMO_ROOT}/project/plans/newtons_cradle.xml"
			"${CMAKE_CURRENT_BINARY_DIR}/model_data.h"
	DEPENDS "${DEMO_ROOT}/project/plans/newtons_cradle.xml"
			"${DEMO_ROOT}/scripts/embed_model.py"
	COMMENT "Embedding newtons_cradle.xml into the guest")

add_sandbox_program(mjstep main.cpp "${CMAKE_CURRENT_BINARY_DIR}/model_data.h")
target_include_directories(mjstep PRIVATE "${MJ}/include" "${CMAKE_CURRENT_BINARY_DIR}")
target_link_libraries(mjstep PRIVATE
	"${MJ}/lib/libmujoco.a" "${MJ}/lib/libccd.a" "${MJ}/lib/libtinyxml2.a"
	"${MJ}/lib/libqhullstatic_r.a" "${MJ}/lib/libtinyobjloader.a" "${MJ}/lib/libminiz.a" "${MJ}/lib/liblodepng.a"
	-lm -lstdc++)
