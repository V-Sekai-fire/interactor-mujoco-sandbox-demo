# interactor-mujoco-sandbox-demo

A physics library built as a RISC-V guest program that an engine project loads into a sandbox and steps from script.

## What it is for

The host owns the tick: every step happens because the script asked for one, so a run repeats bit for bit on any machine. The guest answers stroke-crossing queries for the curve network and holds a scene's colliders for the ray and contact queries a walking player needs. RFD 2274 owns the crossings design.

## Build and run

    godot --path project --import
    godot --path project

The import registers the sandbox extension on a fresh clone.

To rebuild the guest, configure `guest/` with CMake, using `third_party/riscv64-sysroot/toolchain.cmake` as the toolchain file, and copy the built `mjstep` program to `project/plans/mujoco.elf`, the path the host loads.

## Vendored dependencies

Every dependency is a squashed git subtree, so a clone needs no submodule step and a refresh is one command:

    git subtree pull --prefix=<path> https://github.com/<repository> <ref> --squash

| Path | Repository | Ref | Licence |
|---|---|---|---|
| `third_party/doctest` | doctest/doctest | `master` (vendored at `ae7a135`) | MIT |
| `third_party/riscv64-mujoco` | V-Sekai-archive/repository-riscv64-mujoco | `main/main` | Apache-2.0 |
| `third_party/riscv64-sysroot` | V-Sekai-fire/repository-riscv64-sysroot | `main` | LGPL-2.1-or-later, GPL-3.0-with-GCC-exception |
| `third_party/sandbox-api` | V-Sekai-fire/godot-sandbox | `subtree/program-cpp` | BSD-3-Clause |
| `third_party/witness-cpp` | V-Sekai-fire/repository-witness-cpp | `main` | MIT |
| `project/addons/canvas_plane` | V-Sekai/canvas_plane | `main` | MIT |
| `project/addons/distance_field_outlines` | pink-arcana/godot-distance-field-outlines | `main` | MIT |
| `project/addons/godot_sandbox` | V-Sekai-fire/godot-sandbox | `subtree/bin-addons-godot-sandbox` | BSD-3-Clause |
| `project/addons/interaction_system` | V-Sekai-archive/entities-multiplayer-fabric-interaction-system | `main` | MIT |
| `project/addons/vsekai_godot_mcp` | V-Sekai-fire/transport-godot-mcp | `subtree/addons-vsekai-godot-mcp` | MIT |

The last `Squashed '<path>/'` commit on a path names the content that was taken.

Three refs are split branches of their repository rather than the repository itself, because the content sits in a subdirectory: `program/cpp`, `bin/addons/godot_sandbox` and `addons/vsekai_godot_mcp`. `git subtree split` cannot make those splits here, since it skips every commit after a subtree merge with a different prefix and those repositories carry such merges; the branch is made with `git filter-branch --subdirectory-filter <dir>` and pushed.

`third_party/sandbox-api` follows the fork's `main`, and the committed guest programs are built from it, so a pull that changes the API rebuilds them. `project/addons/godot_sandbox` carries what the release ships and the source directory does not hold: the built libraries, the compiler ELF, the licence files, the `.uid` sidecars, and the `linux.arm64` keys in `bin/godot-riscv.gdextension`, which the determinism workflow's arm64 leg needs to load the extension at all.

## Licence

MIT. See [LICENSE](LICENSE). Vendored projects under `third_party/` carry their own licences.
