extends Node

## Drives a MuJoCo physics guest from GDScript.
##
## The guest is a RISC-V ELF with MuJoCo linked into it, running in a Sandbox.
## The host owns the tick: every step happens because this script asked for one,
## which is what makes a run repeatable rather than racing a frame clock.
##
## Two programs is the arrangement, not one: this demo loads the physics guest,
## and a second Sandbox can load a .sgd guest holding the logic. They meet here
## rather than being linked together, because GDScript cannot link a C library.

const PHYSICS_ELF := "res://plans/mujoco.elf"

var _physics: Object = null


func _ready() -> void:
	_physics = _load_guest(PHYSICS_ELF)
	if _physics == null:
		push_error("MuJoCo guest did not load")
		return

	print("guest functions: ", _physics.get_functions())
	print("MuJoCo version: ", _physics.vmcall("mjc_version"))
	print("nq before a model is loaded: ", _physics.vmcall("mjc_nq"))

	# Without a model, stepping must report that it did nothing rather than
	# pretend it advanced. A demo that looks alive with no model is a demo that
	# would look alive with a broken one.
	var t = _physics.vmcall("mjc_step")
	print("step with no model -> ", t, "  (negative means refused)")


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


## Steps the simulation n times and returns the simulated time after each,
## so a caller can see it advance rather than trusting that it did.
func step_many(n: int) -> Array:
	var times: Array = []
	for i in range(n):
		times.append(_physics.vmcall("mjc_step"))
	return times
