#!/usr/bin/env bash
# One-step AVBD guest build: regenerate the Slang kernels from Lean, compile
# them to portable C++, cross-compile the guest ELF for the RISC-V sandbox, and
# drop it into the cloth demo. Run the native test suites first so a broken
# solver never reaches the guest.
#
#   ./build.sh              # regenerate kernels, run native tests, build the ELF
#   ./build.sh --no-emit    # skip Lean/slangc, use the committed gen/ emits
#   ./build.sh --no-tests   # skip the native test suites (build the ELF only)
#
# Tools are taken from PATH: a riscv64-capable clang++ (auto-located if the bare
# clang++ is mingw-only), cmake, and either ninja or make. A full run also needs
# lake + slangc. Point AVBD_DEMO_REPO at the cloth demo checkout to install the
# ELF into it; otherwise the install step is skipped.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO_ROOT="$(cd "$HERE/.." && pwd)"
CLOTH="$(cd "$HERE/../../cloth-dynamics" && pwd)"
SLANG_RT="$HERE/slang-rt"
DEMO_REPO="${AVBD_DEMO_REPO:-}"

NO_EMIT=0
NO_TESTS=0
for a in "$@"; do
	case "$a" in
		--no-emit) NO_EMIT=1 ;;
		--no-tests) NO_TESTS=1 ;;
		*) echo "unknown option: $a" >&2; exit 2 ;;
	esac
done

need() {
	if ! command -v "$1" >/dev/null 2>&1; then
		echo "error: '$1' is not on PATH" >&2
		exit 1
	fi
}
need cmake

# The toolchain file and the native tests both invoke a bare `clang++`, so a
# riscv64-capable one must resolve first on PATH. If the bare clang++ is a
# mingw build with no riscv64 target, put a capable one ahead of it.
has_riscv() { "$1" --print-targets 2>/dev/null | grep -qi riscv64; }
if ! { command -v clang++ >/dev/null 2>&1 && has_riscv clang++; }; then
	FOUND=""
	for c in "$HOME/scoop/apps/llvm/current/bin/clang++" "/c/Program Files/LLVM/bin/clang++"; do
		if [ -x "$c" ] && has_riscv "$c"; then FOUND="$c"; break; fi
	done
	if [ -z "$FOUND" ]; then
		echo "error: no clang++ with a riscv64 target found (a mingw-only clang will not do)" >&2
		exit 1
	fi
	export PATH="$(dirname "$FOUND"):$PATH"
fi

# Prefer ninja; fall back to make so the build does not hard-require ninja.
if command -v ninja >/dev/null 2>&1; then
	GEN=(-G Ninja -DCMAKE_MAKE_PROGRAM="$(command -v ninja)")
elif command -v make >/dev/null 2>&1; then
	GEN=(-G "Unix Makefiles" -DCMAKE_MAKE_PROGRAM="$(command -v make)")
elif command -v mingw32-make >/dev/null 2>&1; then
	GEN=(-G "Unix Makefiles" -DCMAKE_MAKE_PROGRAM="$(command -v mingw32-make)")
else
	echo "error: need ninja or make on PATH" >&2
	exit 1
fi

# The 13 kernels the CPU driver dispatches (10 core + 3 AL dual updates).
KERNELS="vbd_init spring_force vbd_gather_spring attachment_force_al \
  vbd_gather_attachment triangle_membrane_force_al vbd_gather_triangle \
  triangle_bending_force_al vbd_gather_bending vbd_solve_apply \
  attachment_dual_update triangle_membrane_dual_update triangle_bending_dual_update"

if [ "$NO_EMIT" = 0 ]; then
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

if [ "$NO_TESTS" = 0 ]; then
	echo "== native tests =="
	mkdir -p "$HERE/build"
	for t in oracle drape parallel_islands split_panel; do
		clang++ -std=c++17 -O2 -pthread -Wno-unused-parameter \
			-I "$HERE/gen" -I "$SLANG_RT" \
			"$HERE/avbd_cpu.cpp" "$HERE/cloth_grid.cpp" "$HERE/tests/$t.cpp" \
			-o "$HERE/build/$t.exe"
		"$HERE/build/$t.exe"
	done
fi

echo "== cross-compiling the guest ELF =="
cmake -S "$HERE" -B "$HERE/build/guest" "${GEN[@]}" \
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
