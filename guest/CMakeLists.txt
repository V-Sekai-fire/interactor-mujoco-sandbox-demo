cmake_minimum_required(VERSION 3.16)
project(mjguest CXX C)
add_subdirectory("C:/Users/ernest.lee/AppData/Local/Temp/gs/program/cpp/cmake" sandbox_cmake)

set(MJ "C:/contract-manifest/5-repository/riscv64-mujoco")
add_sandbox_program(mjstep main.cpp)
target_include_directories(mjstep PRIVATE "${MJ}/include")
target_link_libraries(mjstep PRIVATE
	"${MJ}/lib/libmujoco.a" "${MJ}/lib/libccd.a" "${MJ}/lib/libtinyxml2.a"
	"${MJ}/lib/libqhullstatic_r.a" "${MJ}/lib/libtinyobjloader.a" "${MJ}/lib/libminiz.a"
	-lm -lstdc++)
