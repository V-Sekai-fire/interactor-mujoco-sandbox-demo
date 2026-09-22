# Contributing

A Godot 4 addon that provides an input delegation and raycasting
interaction system for 3D and VR environments.  `ActionHost` owns
the action registry; `InteractionManager` drives per-frame raycast
queries; `InputDelegator` routes hardware events to the correct
handler.  `canvas_plane` integration enables UI panel interaction
in world space.  Place the addon under `addons/interaction_system/`.

## Guiding principles

- **One responsibility per node.** `ActionHost`, `InteractionManager`,
  and `InputDelegator` each own a single concern.  Cross-cutting state
  is passed via signals, not direct node references.
- **VR-first, desktop-compatible.** All interaction paths must work
  with both hand-controller ray and mouse ray inputs.  Controller
  presence is detected at runtime; do not `#ifdef` for it.
- **No autoloads.** The addon must not register global autoloads.
  Consumers instantiate what they need and wire signals themselves.
  This keeps the addon composable in projects that have their own
  input management.
- **Editor-first testing.** Run `test/` scenes headless to confirm
  core logic:
  ```
  godot --headless --path . --quit-after 5 test/run_tests.tscn
  ```
- **Signal-driven, not polling.** Interaction state changes emit
  signals (`focused`, `unfocused`, `activated`).  Do not poll
  interaction state in `_process`; subscribe to signals instead.

## Workflow

1. Open the project in Godot 4.x.
2. Modify the relevant `.gd` file under `addons/interaction_system/`.
3. Run the `example/` scene to confirm the change works end-to-end.
4. Run the headless test suite (see above).
5. Commit with a sentence-case message describing what changed,
   e.g. `Add canvas plane hover event forwarding`.

## Design notes

### Raycast priority and masking

`InteractionManager` casts against a configurable collision mask.
Consumers set the mask to their interaction layer; the addon does not
hard-code layer numbers.  Priority between overlapping interactables
is resolved by distance to the ray origin — closest wins.

### canvas_plane integration

`canvas_plane` wraps a `SubViewport` in a world-space mesh so that
Godot UI controls receive projected pointer events.  The interaction
system forwards raycast hits on canvas planes as synthetic
`InputEventMouseButton` / `InputEventMouseMotion` events injected into
the viewport.  Do not send raw hardware events to the viewport; always
project through the ray-to-plane intersection math in
`mouse_interaction_helper.gd`.

### lassodb

`lassodb.gd` provides a lightweight in-process key-value store used to
persist interaction state across scene transitions.  It is intentionally
simple — no serialization, no persistence to disk.  If persistence is
needed, consumers serialize the relevant keys to a `ConfigFile`.
