# Cross-compile to riscv64-linux-gnu with clang and this sysroot.
#
#   cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=<this>/toolchain.cmake
#
# clang is multi-target, so no cross-toolchain is needed. What it does need is
# telling where the C++ headers are: clang finds libstdc++ by detecting a GCC
# installation, and a bare sysroot has none, so without the two -isystem paths
# below every C++ file fails on <cstddef> while C compiles perfectly.

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR riscv64)

get_filename_component(RV_ROOT "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
set(RV_SYSROOT "${RV_ROOT}/sysroot")
set(RV_GCCLIB "${RV_SYSROOT}/lib/gcc-cross/14")
set(RV_TRIPLE riscv64-unknown-linux-gnu)

set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_C_COMPILER_TARGET ${RV_TRIPLE})
set(CMAKE_CXX_COMPILER_TARGET ${RV_TRIPLE})
set(CMAKE_SYSROOT ${RV_SYSROOT})

set(CMAKE_CXX_FLAGS_INIT
    "-isystem ${RV_SYSROOT}/include/c++/14 -isystem ${RV_SYSROOT}/include/c++/14/riscv64-linux-gnu")

# lld, because the GNU ld on a typical host is not built for riscv64.
set(RV_LINK "-fuse-ld=lld -B${RV_GCCLIB} -L${RV_GCCLIB}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${RV_LINK}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${RV_LINK}")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "${RV_LINK}")

# Probing with a static library avoids needing a working shared link for every
# dependency before the project has configured.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH ${RV_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
