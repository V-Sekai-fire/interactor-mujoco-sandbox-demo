extends Node

## Drives a MuJoCo physics guest from GDScript.
##
## The guest is a RISC-V ELF with MuJoCo linked into it, running in a Sandbox.
## The host owns the tick: every step happens because this script asked for one,
## which is what makes a run repeatable rather than racing a frame clock.
##
## The model crosses as text, not a path. The guest has no filesystem, so the
## XML goes into MuJoCo's virtual filesystem inside the guest and is parsed
## there; nothing is precompiled on the host.

const PHYSICS_ELF := "res://plans/mujoco.elf"
const MODEL_XML := "res://plans/pendulum.xml"

var _physics: Object = null


func _ready() -> void:
	_physics = _load_guest(PHYSICS_ELF)
	if _physics == null:
		push_error("MuJoCo guest did not load")
		return

	print("guest functions: ", _physics.get_functions())
	print("MuJoCo version: ", _physics.vmcall("mjc_version"))

	var xml := FileAccess.get_file_as_string(MODEL_XML)
	if xml.is_empty():
		push_error("could not read " + MODEL_XML)
		return
	print("model xml bytes: ", xml.length())

	if not _physics.vmcall("mjc_load_xml", xml):
		push_error("guest refused the model")
		return
	print("model loaded, nq = ", _physics.vmcall("mjc_nq"))

	# A pendulum released off-centre must fall. Printing the angle each step is
	# what shows the simulation actually advanced, rather than that a call
	# returned without error.
	print("qpos at rest: ", _physics.vmcall("mjc_qpos"))
	for i in range(5):
		var t = _physics.vmcall("mjc_step")
		print("  t=%.3f  qpos=%s" % [t, str(_physics.vmcall("mjc_qpos"))])


func _load_guest(path: String) -> Object:
	if not FileAccess.file_exists(path):
		push_error("missing guest: " + path)
		return null
	var sb = ClassDB.instantiate("Sandbox")
	if sb == null:
		push_error("Sandbox class not registered; is the addon enabled?")
		return null
	sb.set("program", load(path))
	return sb


## Steps the simulation n times, returning the simulated time after each, so a
## caller can see it advance rather than trusting that it did.
func step_many(n: int) -> Array:
	var times: Array = []
	for i in range(n):
		times.append(_physics.vmcall("mjc_step"))
	return times
