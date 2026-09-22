# AGENTS.md — multiplayer-fabric-interaction-system

Guidance for AI coding agents working in this submodule.

## What this is

Godot 4 addon that delegates input events (mouse clicks, VR controller
raycast) into a 3D scene via the Lasso database. Supports Canvas UI through
`canvas_plane` so that 3D raycasts reach flat UI elements. Install as
`addons/interaction_system` in the target project.

Designed to work with the `canvas_plane` addon on its `interaction_system`
branch at `addons/canvas_plane`.

## Key files

| Path | Purpose |
|------|---------|
| `plugin.cfg` | Godot addon manifest |
| `plugin.gd` | Addon registration |
| `interaction_manager.gd` | Core raycast dispatch and Lasso query |
| `input_delegator.gd` | Routes OS input events to the interaction manager |
| `action_host.gd` | Action binding host for desktop input |
| `xr_action_host.gd` | Action binding host for XR controllers |
| `xr_controller_interaction_helper.gd` | VR controller raycast helper |
| `lassodb.gd` | Lasso database wrapper for spatial UI queries |
| `test/` | Addon tests |

## Conventions

- This is an addon — do not add project-level scenes or autoloads.
- The addon is work in progress; file issues for broken behaviour rather
  than silently patching around it.
- GDScript files need SPDX headers:
  ```gdscript
  # SPDX-License-Identifier: MIT
  # Copyright (c) 2026 K. S. Ernest (iFire) Lee
  ```
- Commit message style: sentence case, no `type(scope):` prefix.
  Example: `Pass canvas_plane UV hit point through Lasso query`
