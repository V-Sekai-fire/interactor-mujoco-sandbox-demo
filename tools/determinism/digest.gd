extends SceneTree

## Prints a digest of the cradle's physics state after a fixed number of steps.
## The guest is RISC-V, emulated identically by libriscv on every host, so the
## digest must be bit-identical across OSes and architectures. The determinism
## CI runs this on each platform and compares. Run:
##   godot --headless --path project -s ../tools/determinism/digest.gd
##
## Deterministic by construction: the model opens from a fixed keyframe (drawn
## back, at rest) and is stepped a fixed count with no wall-clock or RNG input.

const STEPS := 4000

func _init() -> void:
	var sb = ClassDB.instantiate("Sandbox")
	if sb == null:
		printerr("DETERMINISM: Sandbox class missing (extension not loaded?)")
		quit(1); return
	sb.set("program", load("res://plans/mujoco.elf"))
	sb.set_memory_max(1024)
	sb.set_allocations_max(1 << 21)
	sb.set_unboxed_arguments(true)
	if not sb.vmcall("mjc_load_builtin"):
		printerr("DETERMINISM: model failed to load")
		quit(1); return
	for i in range(STEPS):
		sb.vmcall("mjc_step")
	# mjc_digest is FNV-1a over mj_getState(INTEGRATION): the full dynamic state.
	print("DIGEST=%d STEPS=%d NQ=%d" % [sb.vmcall("mjc_digest"), STEPS, sb.vmcall("mjc_nq")])
	quit(0)
