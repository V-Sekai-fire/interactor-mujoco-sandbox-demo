#!/usr/bin/env bash
# One-step AVBD guest build: regenerate the Slang kernels from Lean, compile
# them to portable C++, cross-compile the guest ELF for the RISC-V sandbox, and
# drop it into the cloth demo. Run the native test suites first so a broken
# solver never reaches the guest.
#
#   ./build.sh              # regenerate kernels, run native tests, build the ELF
#   ./build.sh --no-emit    # skip Lean/slangc, use the committed gen/ emits
#
# Tools are taken from PATH: clang++ (with a riscv64 target), ninja, cmake, and
# for a full run lake + slangc. Point AVBD_DEMO_REPO at the cloth demo checkout
# to install the ELF into it; otherwise the install step is skipped.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO_ROOT="$(cd "$HERE/.." && pwd)"
CLOTH="$(cd "$HERE/../../cloth-dynamics" && pwd)"
SLANG_RT="$HERE/slang-rt"
DEMO_REPO="${AVBD_DEMO_REPO:-}"

need() {
	if ! command -v "$1" >/dev/null 2>&1; then
		echo "error: '$1' is not on PATH" >&2
		exit 1
	fi
}
need clang++
need cmake
need ninja
if ! clang++ --print-targets 2>/dev/null | grep -qi riscv64; then
	echo "error: the clang++ on PATH has no riscv64 target (a mingw-only clang will not do)" >&2
	exit 1
fi

# The 13 kernels the CPU driver dispatches (10 core + 3 AL dual updates).
KERNELS="vbd_init spring_force vbd_gather_spring attachment_force_al \
  vbd_gather_attachment triangle_membrane_force_al vbd_gather_triangle \
  triangle_bending_force_al vbd_gather_bending vbd_solve_apply \
  attachment_dual_update triangle_membrane_dual_update triangle_bending_dual_update"

if [ "${1:-}" != "--no-emit" ]; then
	echo "== emitting Slang kernels from Lean =="
	need lake
	need slangc
	( cd "$CLOTH/lean" && lake exe emit_shaders "${TMPDIR:-/tmp}/emitted_shaders" >/dev/null )
	mkdir -p "$HERE/gen"
	for k in $KERNELS; do
		slangc -target cpp -stage compute -entry main \
			-o "$HERE/gen/${k}_emit.cpp" "${TMPDIR:-/tmp}/emitted_shaders/${k}.slang"
	done
fi

echo "== native tests =="
mkdir -p "$HERE/build"
for t in oracle drape parallel_islands split_panel; do
	clang++ -std=c++17 -O2 -pthread -Wno-unused-parameter \
		-I "$HERE/gen" -I "$SLANG_RT" \
		"$HERE/avbd_cpu.cpp" "$HERE/cloth_grid.cpp" "$HERE/tests/$t.cpp" \
		-o "$HERE/build/$t.exe"
	"$HERE/build/$t.exe"
done

echo "== cross-compiling the guest ELF =="
# The toolchain file selects a bare `clang++`, so the riscv64-capable one must
# be first on PATH; the guard above checks that.
cmake -S "$HERE" -B "$HERE/build/guest" -G Ninja \
	-DCMAKE_MAKE_PROGRAM="$(command -v ninja)" \
	-DCMAKE_TOOLCHAIN_FILE="$DEMO_ROOT/third_party/riscv64-sysroot/toolchain.cmake"
cmake --build "$HERE/build/guest"

ELF="$HERE/build/guest/avbd"
if [ -f "$ELF" ]; then
	file "$ELF"
	if [ -n "$DEMO_REPO" ] && [ -d "$DEMO_REPO/project/plans" ]; then
		cp "$ELF" "$DEMO_REPO/project/plans/avbd.elf"
		echo "== installed $DEMO_REPO/project/plans/avbd.elf =="
	else
		echo "== set AVBD_DEMO_REPO to install avbd.elf into the demo =="
	fi
fi
