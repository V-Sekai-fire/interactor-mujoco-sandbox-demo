extends SceneTree

const STEPS := 200


func _end(line: String, code: int) -> void:
	print(line)
	quit(code)


func _init() -> void:
	var args := OS.get_cmdline_user_args()
	if args.size() < 1:
		_end("RESULT: FAIL usage: -- <mujoco.elf> [--plant-state]", 1)
		return
	var plant := args.has("--plant-state")
	if ClassDB.class_has_method("Sandbox", "set_jit_enabled"):
		ClassDB.class_call_static("Sandbox", "set_jit_enabled", false)
	var sb = ClassDB.instantiate("Sandbox")
	if sb == null:
		_end("RESULT: FAIL Sandbox class missing", 1)
		return
	sb.set_memory_max(1024)
	sb.set_allocations_max(1 << 21)
	sb.set_unboxed_arguments(true)
	sb.load_buffer(FileAccess.get_file_as_bytes(args[0]))
	if not sb.has_function("mjc_set_state") or sb.vmcall("mjc_load_builtin") != true:
		_end("RESULT: FAIL the guest has no model or no mjc_set_state", 1)
		return
	for i in STEPS:
		sb.vmcall("mjc_step")
	var kept: PackedFloat64Array = sb.vmcall("mjc_state")
	var at_keep: int = sb.vmcall("mjc_digest")
	for i in STEPS:
		sb.vmcall("mjc_step")
	var at_end: int = sb.vmcall("mjc_digest")
	var short: PackedFloat64Array = kept.slice(0, kept.size() - 1)
	if sb.vmcall("mjc_set_state", short) != false:
		_end("RESULT: FAIL a state one value short was accepted", 1)
		return
	var given := kept.duplicate()
	if plant:
		given[given.size() / 2] += 1e-9
	if sb.vmcall("mjc_set_state", given) != true:
		_end("RESULT: FAIL the state was refused", 1)
		return
	var back: int = sb.vmcall("mjc_digest")
	for i in STEPS:
		sb.vmcall("mjc_step")
	var again: int = sb.vmcall("mjc_digest")
	print("STATE values=%d kept=%d back=%d end=%d again=%d" % [kept.size(), at_keep, back, at_end, again])
	if back != at_keep:
		_end("RESULT: FAIL the digest after the state was put back is not the digest it was taken at", 1)
		return
	if again != at_end:
		_end("RESULT: FAIL the run from the state put back does not end where the first run ended", 1)
		return
	_end("RESULT: PASS", 0)
