# Hosts and tiers

## Four hosts

`fp-determinism` runs the committed guest on four hosts and compares the physics digest: Linux
x86_64, Linux arm64, Windows x86_64 and macOS arm64. The comparison needs exactly four digests
and one value. With four hosts, one that reports a wrong value is outvoted by three and is named
in the job's output. With three, a second fault could not be told from the first, and a missing
host would pass by absence.

The Linux and macOS legs build the sandbox extension from the fork. The Windows leg uses the
committed DLL. The macOS framework in the repository holds an `Info.plist` and no binary, so the
macOS leg places its build in the framework under the name the `Info.plist` gives.

## One tier

A guest can run on three tiers: interpreted, the in-process JIT, and binary translation. Every
gate switches the JIT off before it creates its sandbox, where the class has the switch
(`Sandbox.set_jit_enabled(false)`), so the four hosts compare the interpreted tier.

The switch is a static of the class. It must be set before the guest loads, and a second sandbox
in the same process keeps the first one's translation, so a tier is chosen once for a process.

## Known defect: the JIT on macOS arm64 gives a wrong inverse

Found on 2026-11-13, when the macOS host first joined the check and disagreed with the other
three.

| Host and tier | `mjc_inverse` of 85 zeros, element 34 |
| --- | --- |
| macOS arm64, stock Godot 4.7.2, the extension built with CMake, JIT on (`is_jit()` true) | `0.0` |
| the same, with the JIT switched off before the guest loads | `-43780.0558484725` |
| macOS arm64, an engine build with the sandbox module and no JIT feature | `-43780.0558484725` |
| native MuJoCo 3.16.0 on the same model | `-43780.055848` |
| Linux x86_64, Linux arm64 and Windows x86_64 in CI | the gate passes, as on the interpreter |

Element 34 is `knee_angle_l_beta_translation1` of MS-Human-700, a joint an equality constraint
drives. The forward pass is not affected: `mjc_qacc`, `mjc_qfrc_actuator` and `mjc_digest` are
equal on both tiers. Only the result of `mj_inverse` differs, by the whole constraint force on
that joint, as if the constraint were not active. Building the extension with
`-ffp-contract=off` does not change it.

To reproduce: build `godot-riscv` from the fork's `main` on macOS arm64 and place it in the
addon's framework. From `project/`, with stock Godot 4.7.2, load `plans/mujoco.elf` in a sandbox
with unboxed arguments, call `mjc_load_human(-1)`, `mjc_forward()` and `mjc_inverse` with 85
zeros, and read element 34. Run it once as it is and once after
`ClassDB.class_call_static("Sandbox", "set_jit_enabled", false)`, each in its own process.

Not known: which instruction the JIT gets wrong, whether other guests are affected, and whether
the JIT is on by default on the Linux and Windows hosts. No check yet holds the tiers to one
answer. Until the defect is fixed such a check would fail on macOS arm64, and no result from the
JIT tier on that host should be trusted.
