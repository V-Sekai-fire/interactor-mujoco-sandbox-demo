extends SceneTree

## Gate for the MS-Human-700 model the guest builds from its embedded tree. Run from project/:
##   godot --headless --path . -s ../tools/human/human_check.gd -- <mujoco.elf> [--withhold-rib] [--plant-held] [--qpos <file>]
## Checks: the sizes of the free-root model, every joint name of the retargeted clips, the total
## mass, the meshes the build kept against the embedded ones, and that the forward pass's own
## acceleration comes back from the inverse as the muscles' own passive force and nothing more,
## on two people who share the model.
## Controls: --withhold-rib leaves one rib mesh out of the build, which must then fail, and
## --plant-held feeds the held torque to the inverse check. --qpos reads 86 numbers, one per
## line, as the pose; without it the pose is the model's own.

const MASS_KG := 60.22  # MS-Human-700 as published, to 0.01 kg
const RATIO_MAX := 1e-6  # residual over the holding force
const OBJ_JOINT := 3
const OBJ_ACTUATOR := 19
const ROUNDS := 20


func _fail(msg: String) -> void:
	printerr("HUMAN-FAIL " + msg)
	print("RESULT: FAIL " + msg)
	quit(1)


func _peak(v: PackedFloat64Array) -> float:
	var m := 0.0
	for x in v:
		m = maxf(m, absf(x))
	return m


func _init() -> void:
	var args := OS.get_cmdline_user_args()
	if args.size() < 1:
		_fail("usage: -- <mujoco.elf> [--withhold-rib] [--plant-held] [--qpos <file>]")
		return
	var withhold_rib := args.has("--withhold-rib")
	var plant := args.has("--plant-held")
	if ClassDB.class_has_method("Sandbox", "set_jit_enabled"):
		ClassDB.class_call_static("Sandbox", "set_jit_enabled", false)
	var sb = ClassDB.instantiate("Sandbox")
	if sb == null:
		_fail("Sandbox class missing")
		return
	sb.set_memory_max(1024)
	sb.set_allocations_max(1 << 22)
	sb.set_unboxed_arguments(true)
	# The model build runs past the default budget of one call.
	sb.set("execution_timeout", 4000000)
	sb.load_buffer(FileAccess.get_file_as_bytes(args[0]))
	if not sb.has_function("mjc_load_human"):
		_fail("%s has no mjc_load_human" % args[0])
		return

	var files: PackedStringArray = sb.vmcall("mjc_human_files")
	var withhold := -1
	if withhold_rib:
		for i in files.size():
			if files[i].get_file().begins_with("Rib"):
				withhold = i
				break
		if withhold < 0:
			_fail("control: no rib mesh among the embedded files")
			return
	var t0 := Time.get_ticks_usec()
	# Untyped: a call that throws inside the guest answers null, and the gate must still end.
	var loaded = sb.vmcall("mjc_load_human", withhold)
	var load_ms := (Time.get_ticks_usec() - t0) / 1000.0
	if loaded != true:
		_fail("the model did not build: %s" % str(sb.vmcall("mjc_human_error")))
		return

	var nq: int = sb.vmcall("mjc_nq")
	var nv: int = sb.vmcall("mjc_nv")
	var muscles: PackedStringArray = sb.vmcall("mjc_names", OBJ_ACTUATOR)
	var joints: PackedStringArray = sb.vmcall("mjc_names", OBJ_JOINT)
	var mass: float = sb.vmcall("mjc_mass")
	print("HUMAN nq=%d nv=%d muscles=%d joints=%d mass=%.4f kg load=%.1f ms" % [nq, nv, muscles.size(), joints.size(), mass, load_ms])
	if nq != 86 or nv != 85 or muscles.size() != 700:
		_fail("sizes nq %d, nv %d, muscles %d against 86, 85, 700" % [nq, nv, muscles.size()])
		return
	if absf(mass - MASS_KG) > 0.005:
		_fail("mass %.4f kg against %.2f kg" % [mass, MASS_KG])
		return

	var want := FileAccess.get_file_as_string(get_script().resource_path.get_base_dir().path_join("clip_joints.txt")).replace("\r", "").split("\n", false)
	var missing := PackedStringArray()
	for name in want:
		if not joints.has(name):
			missing.append(name)
	if want.size() != 80 or missing.size() != 0:
		_fail("%d clip joint names read, missing %s" % [want.size(), str(missing)])
		return

	var kept: PackedStringArray = sb.vmcall("mjc_human_kept")
	var meshes := PackedStringArray()
	for f in files:
		if not f.ends_with(".xml"):
			meshes.append(f)
	kept.sort()
	meshes.sort()
	if kept != meshes:
		_fail("the build kept %d meshes and %d are embedded" % [kept.size(), meshes.size()])
		return

	var pose := "the model's own"
	var at := args.find("--qpos")
	var qpos := PackedFloat64Array()
	if at >= 0 and at + 1 < args.size():
		for line in FileAccess.get_file_as_string(args[at + 1]).replace("\r", "").split("\n", false):
			qpos.append(line.to_float())
		if qpos.size() != nq:
			_fail("%s holds %d numbers against nq %d" % [args[at + 1], qpos.size(), nq])
			return
		pose = args[at + 1].get_file()

	# Two people on one model: the second holds the pose, the first stays as loaded.
	var zero := PackedFloat64Array()
	zero.resize(nv)
	var residual := 0.0
	var holding := 0.0
	var forward_us := 0
	var inverse_us := 0
	for person in [1, 0]:
		if not sb.vmcall("mjc_person", person):
			_fail("person %d was refused" % person)
			return
		if person == 1 and qpos.size() == nq and not sb.vmcall("mjc_set_qpos", qpos):
			_fail("the pose was refused")
			return
		t0 = Time.get_ticks_usec()
		for i in ROUNDS:
			sb.vmcall("mjc_forward")
		forward_us += Time.get_ticks_usec() - t0
		var qacc: PackedFloat64Array = sb.vmcall("mjc_qacc")
		t0 = Time.get_ticks_usec()
		var free := PackedFloat64Array()
		for i in ROUNDS:
			free = sb.vmcall("mjc_inverse", qacc)
		inverse_us += Time.get_ticks_usec() - t0
		var held: PackedFloat64Array = sb.vmcall("mjc_inverse", zero)
		var passive: PackedFloat64Array = sb.vmcall("mjc_qfrc_actuator")
		if passive.size() != nv:
			_fail("actuator forces are %d long against nv %d" % [passive.size(), nv])
			return
		for k in nv:
			free[k] -= passive[k]
			held[k] -= passive[k]
		if free.size() != nv or held.size() != nv:
			_fail("sizes free %d, held %d against nv %d" % [free.size(), held.size(), nv])
			return
		residual = maxf(residual, _peak(held if plant else free))
		holding = maxf(holding, _peak(held))
	if qpos.size() == nq:
		sb.vmcall("mjc_person", 1)
		var back: PackedFloat64Array = sb.vmcall("mjc_qpos")
		sb.vmcall("mjc_person", 0)
		var first: PackedFloat64Array = sb.vmcall("mjc_qpos")
		if back != qpos or first == qpos:
			_fail("two people do not hold two poses")
			return
	var tier := "unknown"
	if sb.has_method("is_binary_translated"):
		tier = "translated" if sb.is_binary_translated() else "not translated"
	print(
		(
			"HUMAN pose=%s residual=%s holding=%s bound=%s forward=%.2f ms inverse=%.2f ms tier=%s"
			% [
				pose,
				String.num_scientific(residual),
				String.num_scientific(holding),
				String.num_scientific(holding * RATIO_MAX),
				forward_us / 1000.0 / (ROUNDS * 2),
				inverse_us / 1000.0 / (ROUNDS * 2),
				tier,
			]
		)
	)
	if holding <= 0.0:
		_fail("control: the held body shows no force")
		return
	if residual > holding * RATIO_MAX:
		_fail("the acceleration comes back as %s" % String.num_scientific(residual))
		return
	print("RESULT: PASS")
	quit(0)
