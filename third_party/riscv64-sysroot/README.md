A riscv64-linux-gnu sysroot, so clang on any host can build and link RISC-V Linux binaries without a cross-toolchain or a container.

`sysroot/` is headers and libraries only, assembled from seven sha256-pinned
Debian packages named in `pins.txt`. It is committed rather than fetched at
build time so a checkout is self-contained.

## Using it

    clang --target=riscv64-unknown-linux-gnu \
          --sysroot=<this>/sysroot -fuse-ld=lld \
          -B<this>/sysroot/lib/gcc-cross/14 \
          -L<this>/sysroot/lib/gcc-cross/14 \
          hello.c -o hello.elf -lm

That produces a real `ELF 64-bit LSB pie executable, UCB RISC-V, RVC,
double-float ABI`, dynamically linked against glibc. It is what godot-sandbox
guests are built as, and it needs no Docker and no WSL.

## The one thing that is not a plain copy

Debian ships `lib/libc.so` as a GNU ld script naming absolute paths under
`/usr/riscv64-linux-gnu/lib/`, which exist only on a Debian host. They are
rewritten to `/lib/`, which `lld` resolves inside the sysroot. Without that
every link fails on the first `-lc`, so `assemble.py --verify` treats an
unrelocated script as a failure rather than a note.

## Checking it

    python assemble.py --verify      # the committed tree is complete
    python assemble.py --assemble    # rebuild it from pins.txt
    python assemble.py --self-test   # 6 controls

## Licence

The contents are Debian binaries, redistributed unmodified apart from the ld
script above. glibc is LGPL-2.1-or-later; libgcc and libstdc++ are GPL-3.0
with the GCC Runtime Library Exception, which is what makes linking them into
a program with its own terms fine. The corresponding sources are the Debian
source packages named with their versions in `pins.txt`, available from
`https://deb.debian.org/debian/` and its archive.

This repository carries no code of its own beyond `assemble.py`.
