# Taking and putting back the state

Two calls move the whole integration state of the selected data: time, positions, velocities,
actuator activations, the solver's warm start and the controls. It is the state `mjc_digest`
hashes.

| Call | What it does |
| --- | --- |
| `mjc_state() -> PackedFloat64Array` | the full integration state, 57 numbers for the built-in cradle |
| `mjc_set_state(state) -> bool` | replaces the state. False when the array is not the model's state size. It does not run a forward pass, so the digest straight after equals the digest the state was taken at |

They exist so that one replica of a shared object can take the state of another and check what it
took. `interactor-presence` uses them in its replica gate: a replica whose digest differs from
the other three takes the state from one of them and keeps it only if `mjc_digest` then equals
that replica's digest.

Positions of bodies and geoms are not part of the state. After `mjc_set_state` they are those of
the state before, until the next `mjc_step` or `mjc_forward`.

## `tools/determinism/state_check.gd -- <mujoco.elf> [--plant-state]`

The gate steps the cradle 200 times, takes the state and the digest, steps 200 more and takes
the digest again. It puts the state back and requires the digest it was taken at. It steps 200
more and requires the digest the first run ended at, so a run from a state put back is the same
run. A state one value short must be refused.

The control `--plant-state` adds 1e-9 to one value of the state before it is put back, and the
gate must fail.
