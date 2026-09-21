MuJoCo prebuilt for riscv64-linux-gnu, so a sandbox guest can be linked against it with clang and no cross-toolchain or container.

`lib/libmujoco.a` is MuJoCo 3.14.0 cross-compiled for `riscv64`, beside the five
vendored dependencies it needs (ccd, tinyxml2, qhull, tinyobjloader, miniz).
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

## Provenance and licence

MuJoCo 3.14.0 at `53197e6`, Apache-2.0, from
`https://github.com/google-deepmind/mujoco`. Built with clang against the
Debian-derived sysroot in `5-repository/riscv64-sysroot`. `-Werror` was relaxed
for the build: MuJoCo's OpenGL loader trips `-Wunused-but-set-global`, and that
file is not reachable from a headless guest.
