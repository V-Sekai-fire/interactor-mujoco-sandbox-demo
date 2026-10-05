# interactor-mujoco-sandbox-demo

A physics library built as a RISC-V guest program that an engine project loads into a sandbox and steps from script.

## What it is for

The host owns the tick: every step happens because the script asked for one, so a run repeats bit for bit on any machine. The guest answers stroke-crossing queries for the curve network and holds a scene's colliders for the ray and contact queries a walking player needs. RFD 2274 owns the crossings design.

## Build and run

    godot --path project

To rebuild the guest, configure `guest/` with CMake, using `third_party/riscv64-sysroot/toolchain.cmake` as the toolchain file.

## Licence

The licence is not stated. Vendored projects under `third_party/` carry their own licences.
