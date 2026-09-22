# Patches to vendored dependencies

These are local changes to submodules of the `godot-sandbox` fork, which live
in a checkout under `%TEMP%` and would otherwise be lost. They are kept here so
the extension can be rebuilt from what is committed.

`libriscv-paging-and-includes.patch`

- `serialize.cpp` no longer refuses a flat read-write arena. Arena pages already
  appear in `m_pages` as non-owning entries pointing into the buffer, and
  `deserialize_from` already rebuilds them that way, so only the throw stood
  between a flat arena and a snapshot. `model/SandboxMem.lean` states why this
  is sound.
- `tr_compiler.cpp` and `tr_emit.cpp` gain `<cstdlib>`. They use `getenv` and
  `std::abs` without it, which libc++ does not provide transitively.

`godot-cpp-stdlib-include.patch`

- `src/godot.cpp` gains `<stdlib.h>`. It calls `realloc` and `free` without it.

Both include additions are portability gaps upstream rather than anything this
project needs specially, and belong in pull requests to those projects.
