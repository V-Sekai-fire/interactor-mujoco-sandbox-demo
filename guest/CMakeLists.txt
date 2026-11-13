cmake_minimum_required(VERSION 3.16)
project(mjguest CXX C)

get_filename_component(DEMO_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(MJ "${DEMO_ROOT}/third_party/riscv64-mujoco")

# The guest API comes from the manifest's contract-guest-runtime when it is checked out beside this
# repository, so the guest carries its .sandbox_variant size; the vendored copy is the fallback.
set(_runtime_api "${DEMO_ROOT}/../../2-contract/guest-runtime/vendor/sandbox-api")
if(EXISTS "${_runtime_api}/docker/api/api.cpp")
	set(SANDBOX_API "${_runtime_api}" CACHE PATH "godot-sandbox guest API checkout")
else()
	set(SANDBOX_API "${DEMO_ROOT}/third_party/sandbox-api" CACHE PATH "godot-sandbox guest API checkout")
endif()
if(DOUBLE_PRECISION)
	file(STRINGS "${SANDBOX_API}/cmake/CMakeLists.txt" _api_double REGEX "sandbox_api PUBLIC DOUBLE_PRECISION_REAL_T")
	if(NOT _api_double)
		message(FATAL_ERROR "DOUBLE_PRECISION needs the guest API of contract-guest-runtime; ${SANDBOX_API} builds single precision only")
	endif()
endif()
set(DOWNLOAD_RUNTIME_API OFF CACHE BOOL "" FORCE)
set(ELF_NAME "mjstep" CACHE STRING "The guest program's name")
add_subdirectory("${SANDBOX_API}/cmake" sandbox_cmake)

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

include("${CMAKE_CURRENT_LIST_DIR}/embed_tree.cmake")
embed_tree("${DEMO_ROOT}/third_party/ms_human_700" "${CMAKE_CURRENT_BINARY_DIR}/human_files.c" "${CMAKE_CURRENT_BINARY_DIR}/human_files.h")

add_sandbox_program(${ELF_NAME} main.cpp station.cpp human.cpp "${CMAKE_CURRENT_BINARY_DIR}/human_files.c"
	"${CMAKE_CURRENT_BINARY_DIR}/model_data.h")
target_include_directories(${ELF_NAME} PRIVATE "${MJ}/include" "${CMAKE_CURRENT_BINARY_DIR}")
target_link_libraries(${ELF_NAME} PRIVATE
	"${MJ}/lib/libmujoco.a" "${MJ}/lib/libccd.a" "${MJ}/lib/libtinyxml2.a"
	"${MJ}/lib/libqhullstatic_r.a" "${MJ}/lib/libtinyobjloader.a" "${MJ}/lib/libminiz.a" "${MJ}/lib/liblodepng.a"
	-lm -lstdc++)
