MuJoCo prebuilt for riscv64-linux-gnu, so a sandbox guest can be linked against it with clang and no cross-toolchain or container.

`lib/libmujoco.a` is MuJoCo 3.14.0 cross-compiled for `riscv64`, beside the
vendored dependencies it needs (ccd, tinyxml2, qhull, tinyobjloader, miniz,
lodepng).
`include/mujoco/` is the public API.

Used with the sysroot placed next to it at `5-repository/riscv64-sysroot`:

    clang --target=riscv64-unknown-linux-gnu \
          --sysroot=../riscv64-sysroot/sysroot \
          -march=rv64g -mabi=lp64d -I include \
          -fuse-ld=lld -B ../riscv64-sysroot/sysroot/lib/gcc-cross/14 \
          -static guest.c -o guest.elf \
          lib/libmujoco.a lib/libccd.a lib/libtinyxml2.a \
          lib/libqhullstatic_r.a lib/libtinyobjloader.a lib/libminiz.a -lm -lstdc++

`-march=rv64g -mabi=lp64d -static` matches what godot-sandbox builds its guests
with, so the result loads in the same host.

## Checking it

    python verify.py              # links a program against MuJoCo
    python verify.py --self-test  # 3 controls

The check links rather than listing files, because an archive that is present
but cannot satisfy a link is the failure worth catching.

## What this cannot do

The guest has no filesystem, so `mj_loadXML` is out. Compile the model to MJB
on the host and load it from memory with `mj_loadModel`.

## How the archives are built

    bash build.sh [<riscv64-sysroot checkout>] [<work dir>]

`build.sh` clones MuJoCo 3.14.0 (`9ecbb9d`), configures it with the sysroot's
`toolchain.cmake`, builds it with MuJoCo's own CMake, and writes `lib/*.a`. It
needs clang, clang++, lld and llvm-ar 23.1.1 first in `PATH`; a pixi
environment with `clang`, `clangxx`, `lld` and `llvm-tools` pinned to
`==23.1.1` from conda-forge gives exactly those.

- Configuration: `CMAKE_BUILD_TYPE=Release` (`-O3 -DNDEBUG`), MuJoCo's default
  ThinLTO (the archives hold LLVM 23 bitcode, so a guest must link them with
  LLVM 23 or newer) and `-fPIC`; examples, simulate, tests and AVX off.
- Target: `riscv64-unknown-linux-gnu` with no `-march`, so the code is the
  target's default `rv64imafdc` with the `lp64d` ABI. That is what the first
  archives were built with; a guest linked with `-march=rv64g` accepts it.
- No build path is kept. Each compile goes through a launcher that passes the
  source file relative to the build directory, so the bitcode's
  `source_filename` and `__FILE__` are relative, and
  `-ffile-prefix-map`, `-fmacro-prefix-map` and `-fdebug-prefix-map` rewrite the
  MuJoCo source, build and sysroot directories to `mujoco`, `mujoco-build` and
  `riscv64-sysroot`. The script fails if `strings` finds `Users/`, `ernest` or
  `/home/` in any archive.
- MuJoCo makes `libmujoco` a shared library, so the script archives the
  objects of that same target (sorted by path) into `libmujoco.a`, and
  lodepng's object library into `liblodepng.a`.
- `-Werror` is relaxed (`-Wno-error`, last on the command line): MuJoCo's
  OpenGL loader trips `-Wunused-but-set-global`, and that file is not
  reachable from a headless guest.

The current archives were built this way on Linux. Compared with the first
archives (built on Windows from MuJoCo `53197e6`, whose engine, user, xml and
dependency sources are identical to 3.14.0's), 106 of the 107 members are the
same LLVM IR once source paths, and the hashes derived from them, are set aside. The exception is
`ui_main.c.o`, whose compile-time-folded trigonometric constants differ in the
last bit because LLVM folds them with the build host's libm; the UI is not
linked into the guest.

## Provenance and licence

MuJoCo 3.14.0 (tag `3.14.0`, commit `9ecbb9d`), Apache-2.0, from
`https://github.com/google-deepmind/mujoco`, built with clang 23.1.1 against
the Debian-derived sysroot in `5-repository/riscv64-sysroot` by `build.sh`.
