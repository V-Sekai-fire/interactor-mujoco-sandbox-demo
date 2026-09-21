extends Node

## Drives a cat's cradle MuJoCo guest from GDScript.
##
## The guest is a RISC-V ELF with MuJoCo linked into it, running in a Sandbox.
## The host owns the tick: every step happens because this script asked for one,
## which is what makes a run repeatable rather than racing a frame clock.
##
## The model is compiled into the guest. A machine that can be serialised has
## the flat arena off, and host-to-guest transfer allocates through that arena,
## so a guest carrying its own model is the one that can also be migrated.

const PHYSICS_ELF := "res://plans/mujoco.elf"
const STEPS := 50


func _ready() -> void:
	var sb := _load_guest(PHYSICS_ELF)
	if sb == null:
		return

	print("MuJoCo version: ", sb.vmcall("mjc_version"))
	print("minimal model builds: ", sb.vmcall("mjc_xml_selftest"))

	if not sb.vmcall("mjc_load_builtin"):
		push_error("the guest could not build its model")
		return

	print("nq=%d neq=%d" % [sb.vmcall("mjc_nq"), sb.vmcall("mjc_neq")])
	print("lowest at rest: ", _mm(sb.vmcall("mjc_lowest_mm")))
	for i in range(STEPS):
		sb.vmcall("mjc_step")
	print("after %d steps: ncon=%d lowest=%s" % [STEPS, sb.vmcall("mjc_ncon"), _mm(sb.vmcall("mjc_lowest_mm"))])


func _load_guest(path: String) -> Object:
	if not FileAccess.file_exists(path):
		push_error("missing guest: " + path)
		return null
	var sb = ClassDB.instantiate("Sandbox")
	if sb == null:
		push_error("Sandbox class not registered; is the addon enabled?")
		return null
	sb.set("program", load(path))
	# The addon defaults to a 32 MB heap and 4000 allocations, sized for a script
	# rather than a model compiler. Raised after load, because loading resets them.
	sb.set_memory_max(512)
	sb.set_allocations_max(1 << 20)
	return sb


## Millimetres paired with something a reader can picture, because "4.3 mm" does
## not say whether an error matters.
func _mm(v: float) -> String:
	var anchors := {"a credit card": 0.76, "a penny": 1.52, "a pencil": 7.0,
		"a AA battery": 14.5, "a nickel": 21.2, "a golf ball": 42.7,
		"an adult wrist": 57.0, "a soda can": 66.0}
	var best := ""
	var best_err := INF
	for name in anchors:
		var n: float = abs(v) / float(anchors[name])
		var err: float = abs(n - round(n))
		if n >= 0.5 and err < best_err:
			best_err = err
			best = "%.1f x %s" % [n, name]
	return "%.1f mm (%s)" % [v, best] if best != "" else "%.1f mm" % v
