MuJoCo running as a RISC-V sandbox guest inside Godot, stepped from GDScript, with no native extension of our own and no container.

    godot --headless --path project --import      # registers the GDExtension
    godot --headless --path project --quit-after 60

    guest functions: ["mjc_version", "mjc_load", "mjc_step", "mjc_nq", "mjc_qpos"]
    MuJoCo version: 3014000
    step with no model -> -1.0  (negative means refused)

`3014000` is MuJoCo 3.14.0 reporting its own version from inside the emulated
guest.

## How it fits together

`project/plans/mujoco.elf` is a RISC-V program with MuJoCo statically linked in,
built from `guest/main.cpp` against godot-sandbox's guest API. `project/host.gd`
loads it into a `Sandbox` and calls it.

**The host owns the tick.** Every step happens because GDScript asked for one,
which is what makes a run repeatable instead of racing a frame clock.

**Two programs, not one.** GDScript cannot link a C library, so the physics
lives in its own guest. A second `Sandbox` can load a `.sgd` guest holding the
logic; they meet in the host rather than being linked together.

## Rebuilding the guest

Needs the two placed packages and clang — no cross-toolchain, no Docker:

    cmake -S guest -B guest/build -G Ninja \
      -DCMAKE_TOOLCHAIN_FILE=../../5-repository/riscv64-sysroot/toolchain.cmake
    cmake --build guest/build

`5-repository/riscv64-sysroot` supplies headers, startup files and glibc;
`5-repository/riscv64-mujoco` supplies the physics archives.

## Where it stops, as of this commit

`mjc_load_xml` **faults**. The XML crosses from GDScript and reaches the guest —
`model xml bytes: 317` prints — then MuJoCo's model compiler takes a protection
fault inside the sandbox:

    Exception: Protection fault (data: 0)
    -> _ZL12mjc_load_xml7Variant

Everything either side of it works: the guest loads, reports `MuJoCo version:
3014000` from inside the emulator, and exposes its six bindings. Stepping
without a model correctly returns `-1.0`.

Two candidates, neither confirmed. The model compiler allocates far more than a
stepping loop and may be hitting the guest's allocation ceiling rather than a
genuine fault. Or `mjVFS` is a large stack object and the guest stack is 2 MB.
The next step is to raise the guest's limits and see which moves.

An MJB compiled on the host and passed to `mjc_load` avoids the compiler
entirely and is the other route; `mj_loadModel` takes a path and is unreachable
from a guest, `mj_loadModelBuffer` is the one that works.

The vendored addon prints `Object 'Sandbox' already has member ''` on load. It
is upstream's prebuilt binary and the message is harmless here, but it points at
a binding mismatch worth tracing before anyone trusts it further.

## The MCP addon

`project/addons/vsekai_godot_mcp` is enabled alongside the sandbox: a pure
GDScript MCP server, so the editor itself is the endpoint and no external
process is needed. With it, an agent can inspect the scene, the guest's
bindings and the step results from outside.
