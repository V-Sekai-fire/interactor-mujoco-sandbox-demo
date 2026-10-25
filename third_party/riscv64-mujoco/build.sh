#!/usr/bin/env bash
# Rebuild lib/*.a: MuJoCo 3.14.0 and the dependencies its CMake fetches,
# cross-compiled for riscv64-linux-gnu with clang, as ThinLTO bitcode archives.
#
#   bash build.sh [<riscv64-sysroot checkout>] [<work dir>]
#
# Defaults: the sysroot is ../riscv64-sysroot (the copy in this repository) and
# the work dir is ./mj-build. clang, clang++, lld and llvm-ar 23.1.1 must come
# first in PATH: the archives hold LLVM bitcode, and a guest links them only
# with an LLVM at least as new as the one that wrote them. A pixi environment
# with clang, clangxx, lld and llvm-tools pinned to ==23.1.1 (conda-forge)
# gives exactly that.
#
# No build-machine path may be left in the archives. Every compile runs through
# a launcher that passes the source file relative to the build directory, so the
# bitcode's source_filename is relative, and -ffile-prefix-map rewrites the
# absolute paths that __FILE__ would otherwise expand to inside headers.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYSROOT_DIR="$(cd "${1:-$HERE/../riscv64-sysroot}" && pwd)"
W="${2:-$PWD/mj-build}"
OUT="${OUT:-$HERE/lib}"

MUJOCO_TAG=3.14.0
MUJOCO_COMMIT=9ecbb9d7b5ee623f54745638d36799ff90e6f7cd
CLANG_VERSION=23.1.1

clang --version | head -1 | grep -q "clang version $CLANG_VERSION" ||
  { echo "need clang $CLANG_VERSION first in PATH, found: $(clang --version | head -1)" >&2; exit 1; }
clang --print-targets | grep -q riscv64 || { echo "this clang has no riscv64 target" >&2; exit 1; }
for t in clang++ ld.lld llvm-ar llvm-ranlib; do command -v "$t" >/dev/null || { echo "missing $t" >&2; exit 1; }; done
[ -f "$SYSROOT_DIR/toolchain.cmake" ] || { echo "no toolchain.cmake in $SYSROOT_DIR" >&2; exit 1; }

mkdir -p "$W"
W="$(cd "$W" && pwd)"
SRC="$W/mujoco"
BUILD="$W/build"

if [ ! -d "$SRC/.git" ]; then
  git clone --quiet --depth 1 --branch "$MUJOCO_TAG" https://github.com/google-deepmind/mujoco "$SRC"
fi
[ "$(git -C "$SRC" rev-parse HEAD)" = "$MUJOCO_COMMIT" ] ||
  { echo "$SRC is not at $MUJOCO_TAG ($MUJOCO_COMMIT)" >&2; exit 1; }

git -C "$SRC" checkout --quiet -- .   # undo any local edits to the source

# The launcher: make the source argument relative to the build directory, and
# put -Wno-error last so it outlasts the -Werror that MuJoCo adds per target
# (its OpenGL loader, unreachable from a headless guest, trips
# -Wunused-but-set-global). Neither changes the code generated.
cat > "$W/relsrc.py" <<'PY'
import os, subprocess, sys
args = sys.argv[1:]
out = [args[0]]
for a in args[1:]:
    if os.path.isabs(a) and a.endswith((".c", ".cc", ".cpp", ".cxx")) and os.path.isfile(a):
        a = os.path.relpath(a, os.getcwd())
    out.append(a)
out.append("-Wno-error")
sys.exit(subprocess.call(out))
PY

MAP=""
for pair in "$SRC=mujoco" "$BUILD=mujoco-build" "$SYSROOT_DIR=riscv64-sysroot"; do
  MAP="$MAP -ffile-prefix-map=$pair -fmacro-prefix-map=$pair -fdebug-prefix-map=$pair"
done

# No -march: the target's default (rv64imafdc, lp64d) is what the archives were
# first built with, and the guest's -march=rv64g link accepts it.
export CFLAGS="$MAP"
export CXXFLAGS="$MAP"

cmake -S "$SRC" -B "$BUILD" -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$SYSROOT_DIR/toolchain.cmake" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=OFF \
  -DCMAKE_AR="$(command -v llvm-ar)" \
  -DCMAKE_RANLIB="$(command -v llvm-ranlib)" \
  -DCMAKE_C_COMPILER_LAUNCHER="python3;$W/relsrc.py" \
  -DCMAKE_CXX_COMPILER_LAUNCHER="python3;$W/relsrc.py" \
  -DMUJOCO_BUILD_EXAMPLES=OFF \
  -DMUJOCO_BUILD_SIMULATE=OFF \
  -DMUJOCO_BUILD_TESTS=OFF \
  -DMUJOCO_TEST_PYTHON_UTIL=OFF \
  -DMUJOCO_ENABLE_AVX=OFF \
  -DMUJOCO_ENABLE_AVX_INTRINSICS=OFF
cmake --build "$BUILD" --target mujoco ccd lodepng miniz qhullstatic_r tinyobjloader tinyxml2

# MuJoCo's CMake makes libmujoco SHARED on every platform but the web, and a
# STATIC switch breaks its install(EXPORT). A guest links statically, so archive
# the objects of that same target: the compile commands are the ones the shared
# library is made from (MuJoCo builds everything -fPIC). Members go in sorted
# path order, the order the first archive had.
mkdir -p "$OUT"
objs="$(ninja -C "$BUILD" -t commands mujoco | grep -e ' -shared ' | grep 'libmujoco\.so' | tr ' ' '\n' | grep '\.o$' | grep -v '/lodepng\.cpp\.o$' | LC_ALL=C sort)"
[ -n "$objs" ] || { echo "found no objects for libmujoco" >&2; exit 1; }
rm -f "$OUT/libmujoco.a"
(cd "$BUILD" && llvm-ar qcsD "$OUT/libmujoco.a" $objs)
# lodepng is an OBJECT library that MuJoCo folds into its shared library; the
# guest links it as an archive of its own.
lodepng_o="$(find "$BUILD" -name lodepng.cpp.o -print -quit)"
rm -f "$OUT/liblodepng.a"
llvm-ar qcsD "$OUT/liblodepng.a" "$lodepng_o"

for a in libccd.a libtinyxml2.a libqhullstatic_r.a libtinyobjloader.a libminiz.a; do
  f="$(find "$BUILD" -name "$a" -print -quit)"
  [ -n "$f" ] || { echo "the build made no $a" >&2; exit 1; }
  cp "$f" "$OUT/$a"
done

# The gate this script exists for: no user name or home directory in any archive.
bad=0
for a in "$OUT"/*.a; do
  n="$(strings -a "$a" | grep -cE 'Users/|ernest|/home/' || true)"
  echo "$(basename "$a"): $n build-path strings"
  [ "$n" = 0 ] || bad=1
done
exit $bad
