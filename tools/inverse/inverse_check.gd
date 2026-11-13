extends SceneTree

## Gate for mjc_inverse on the built-in cradle, drawn back and at rest. Run from project/:
##   godot --headless --path . -s ../tools/inverse/inverse_check.gd -- <mujoco.elf> [--plant-held]
## The forward pass's own acceleration must come back as no force. Two controls: zero
## acceleration holds the ball, so a torque appears, and --plant-held feeds that to the gate,
## which must fail; an array one short of nv must come back empty.

const RATIO_MAX := 1e-6  # residual over the holding torque, both in N m


func _fail(msg: String) -> void:
	printerr("INVERSE-FAIL " + msg)
	quit(1)


func _peak(v: PackedFloat64Array) -> float:
	var m := 0.0
	for x in v:
		m = maxf(m, absf(x))
	return m


func _init() -> void:
	var args := OS.get_cmdline_user_args()
	var plant := args.size() == 2 and args[1] == "--plant-held"
	if args.size() < 1 or args.size() > 2 or (args.size() == 2 and not plant):
		_fail("usage: -- <mujoco.elf> [--plant-held]")
		return
	var sb = ClassDB.instantiate("Sandbox")
	if sb == null:
		_fail("Sandbox class missing")
		return
	sb.set_memory_max(1024)
	sb.set_allocations_max(1 << 21)
	sb.set_unboxed_arguments(true)
	sb.load_buffer(FileAccess.get_file_as_bytes(args[0]))
	if not sb.has_function("mjc_inverse"):
		_fail("%s has no mjc_inverse" % args[0])
		return
	if not sb.vmcall("mjc_load_builtin"):
		_fail("model failed to load")
		return
	var nv: int = sb.vmcall("mjc_nv")
	if nv < 1:
		_fail("nv is %d" % nv)
		return
	sb.vmcall("mjc_forward")
	var qacc: PackedFloat64Array = sb.vmcall("mjc_qacc")
	var free: PackedFloat64Array = sb.vmcall("mjc_inverse", qacc)
	var zero := PackedFloat64Array()
	zero.resize(nv)
	var held: PackedFloat64Array = sb.vmcall("mjc_inverse", zero)
	var short: PackedFloat64Array = sb.vmcall("mjc_inverse", zero.slice(0, nv - 1))
	if qacc.size() != nv or free.size() != nv or held.size() != nv:
		_fail("sizes qacc %d, free %d, held %d against nv %d" % [qacc.size(), free.size(), held.size(), nv])
		return
	var residual := _peak(held if plant else free)
	var holding := _peak(held)
	print(
		(
			"INVERSE nv=%d residual=%s N m holding=%s N m bound=%s N m"
			% [nv, String.num_scientific(residual), String.num_scientific(holding), String.num_scientific(holding * RATIO_MAX)]
		)
	)
	if holding <= 0.0:
		_fail("control: the held ball shows no torque")
		return
	if residual > holding * RATIO_MAX:
		_fail("the acceleration comes back as %s N m" % String.num_scientific(residual))
		return
	if short.size() != 0:
		_fail("control: an array of %d was accepted for nv %d" % [nv - 1, nv])
		return
	print("INVERSE-OK controls=2")
	quit(0)
