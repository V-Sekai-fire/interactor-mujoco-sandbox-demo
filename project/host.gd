extends Node3D

## A cat's cradle simulated inside a RISC-V sandbox, and moved between two of
## them while it runs.
##
## Two sandboxes stand side by side. The left one steps the figure; the right
## one is never given a model. A snapshot is the whole machine, so restoring it
## into the right sandbox is what makes the figure appear there, and the two
## then step in lockstep. Matching digests are the evidence: a migrated run and
## a merely running one look identical without them.
##
## The model is compiled into the guest. Serialising a machine needs the flat
## arena off, and host-to-guest transfer allocates through that arena, so a
## guest that carries its own model is the one that can also be moved.

const PHYSICS_ELF := "res://plans/mujoco.elf"
const SNAPSHOT_PATH := "user://cradle.snapshot"
const SEG_RADIUS := 0.003
const SEG_LENGTH := 0.0348

var _a: Object = null
var _b: Object = null
var _a_meshes: Array[MeshInstance3D] = []
var _b_meshes: Array[MeshInstance3D] = []
var _running := true
var _snapshot := PackedByteArray()

var _status: RichTextLabel
var _log: RichTextLabel


func _ready() -> void:
	_build_ui()
	_a = _make_sandbox()
	_b = _make_sandbox()
	if _a == null or _b == null:
		return

	if not _a.vmcall("mjc_load_builtin"):
		_say("[color=red]the guest could not build its model[/color]")
		return

	_a_meshes = _build_meshes(Vector3(-0.22, 0, 0), Color(0.95, 0.85, 0.55))
	_b_meshes = _build_meshes(Vector3(0.22, 0, 0), Color(0.55, 0.8, 0.95))
	_say("left sandbox loaded the figure: nq=%d, %d equality constraints" % [
		_a.vmcall("mjc_nq"), _a.vmcall("mjc_neq")])
	_say("right sandbox has no model at all")


## The controls are built here rather than in the scene, so the scene stays a
## camera and a light and there is one place to read what each button does.
func _build_ui() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)
	var panel := PanelContainer.new()
	panel.set_anchors_preset(Control.PRESET_BOTTOM_WIDE)
	panel.offset_left = 12
	panel.offset_right = -12
	panel.offset_top = -250
	panel.offset_bottom = -12
	layer.add_child(panel)

	var box := VBoxContainer.new()
	panel.add_child(box)

	var row := HBoxContainer.new()
	box.add_child(row)
	var buttons := [
		["Snapshot left", _on_snapshot_pressed],
		["Restore into right", _on_restore_pressed],
		["Save to file", _on_save_file_pressed],
		["Load from file", _on_load_file_pressed],
	]
	for spec in buttons:
		var b := Button.new()
		b.text = spec[0]
		b.pressed.connect(spec[1])
		row.add_child(b)
	var pause := CheckButton.new()
	pause.text = "Pause"
	pause.toggled.connect(_on_pause_toggled)
	row.add_child(pause)

	_status = RichTextLabel.new()
	_status.bbcode_enabled = true
	_status.custom_minimum_size = Vector2(0, 64)
	box.add_child(_status)

	_log = RichTextLabel.new()
	_log.bbcode_enabled = true
	_log.scroll_following = true
	_log.custom_minimum_size = Vector2(0, 120)
	box.add_child(_log)


func _make_sandbox() -> Object:
	var sb = ClassDB.instantiate("Sandbox")
	if sb == null:
		_say("[color=red]Sandbox class not registered; is the addon enabled?[/color]")
		return null
	sb.set("program", load(PHYSICS_ELF))
	# The model compiler allocates far more than a stepping loop, and the addon's
	# defaults are sized for a script. Raised after load, which resets them.
	sb.set_memory_max(1024)
	sb.set_allocations_max(1 << 21)
	return sb


## One capsule per rope segment. The world body at index 0 is skipped: it has no
## geometry and sits at the origin.
func _build_meshes(offset: Vector3, tint: Color) -> Array[MeshInstance3D]:
	var mesh := CapsuleMesh.new()
	mesh.radius = SEG_RADIUS
	mesh.height = SEG_LENGTH + SEG_RADIUS * 2.0
	var mat := StandardMaterial3D.new()
	mat.albedo_color = tint
	mat.roughness = 0.6

	# MuJoCo is Z-up, Godot is Y-up. Rotating the holder a quarter turn about X
	# converts both the positions and the orientations underneath it, so the
	# guest keeps handing over MuJoCo coordinates untouched.
	var holder := Node3D.new()
	holder.position = offset
	holder.rotation = Vector3(-PI / 2.0, 0, 0)
	add_child(holder)

	var out: Array[MeshInstance3D] = []
	for i in range(24):
		var mi := MeshInstance3D.new()
		mi.mesh = mesh
		mi.material_override = mat
		mi.visible = false
		holder.add_child(mi)
		out.append(mi)
	return out


func _process(_delta: float) -> void:
	if _a == null:
		return
	if _running:
		for i in range(4):
			_a.vmcall("mjc_step")
			if _b.vmcall("mjc_nq") > 0:
				_b.vmcall("mjc_step")
	_draw(_a, _a_meshes)
	_draw(_b, _b_meshes)
	_refresh_status()


## MuJoCo hands back a capsule's body frame; Godot's CapsuleMesh stands along
## +Y and the segments were authored along the body's local Z, so the mesh is
## rotated a quarter turn to match.
func _draw(sb: Object, meshes: Array[MeshInstance3D]) -> void:
	if sb == null or sb.vmcall("mjc_nq") == 0:
		for m in meshes:
			m.visible = false
		return
	var b: PackedFloat64Array = sb.vmcall("mjc_bodies")
	var n := mini(meshes.size(), int(b.size() / 7) - 1)
	for i in range(n):
		var o := (i + 1) * 7
		var t := Transform3D()
		t.basis = Basis(Quaternion(b[o + 4], b[o + 5], b[o + 6], b[o + 3])) * Basis(Vector3(1, 0, 0), PI / 2.0)
		t.origin = Vector3(b[o + 0], b[o + 1], b[o + 2])
		meshes[i].transform = t
		meshes[i].visible = true
	for i in range(n, meshes.size()):
		meshes[i].visible = false


func _refresh_status() -> void:
	var da: int = _a.vmcall("mjc_digest")
	var db: int = _b.vmcall("mjc_digest")
	var lines := []
	lines.append("[b]left[/b]   t=%.2fs  contacts=%d  lowest=%s" % [
		_a.vmcall("mjc_time"), _a.vmcall("mjc_ncon"), _mm(_a.vmcall("mjc_lowest_mm"))])
	if _b.vmcall("mjc_nq") > 0:
		lines.append("[b]right[/b]  t=%.2fs  contacts=%d  lowest=%s" % [
			_b.vmcall("mjc_time"), _b.vmcall("mjc_ncon"), _mm(_b.vmcall("mjc_lowest_mm"))])
		if da == db:
			lines.append("[color=#7fdc7f]digests match: %d[/color]" % da)
		else:
			lines.append("[color=#ff7f7f]digests differ: %d vs %d[/color]" % [da, db])
	else:
		lines.append("[b]right[/b]  empty")
	_status.text = "\n".join(lines)


func _say(msg: String) -> void:
	_log.text += msg + "\n"


## Millimetres paired with something a reader can picture, because "4.3 mm"
## does not say whether an error matters.
func _mm(v: float) -> String:
	var anchors := {"a pencil": 7.0, "a AA battery": 14.5, "a nickel": 21.2,
		"a golf ball": 42.7, "an adult wrist": 57.0, "a soda can": 66.0}
	var best := ""
	var best_err := INF
	for name in anchors:
		var k: float = absf(v) / float(anchors[name])
		var err: float = absf(k - roundf(k))
		if k >= 0.8 and err < best_err:
			best_err = err
			best = "%.1f x %s" % [k, name]
	return "%.0f mm (%s)" % [v, best] if best != "" else "%.0f mm" % v


func _on_snapshot_pressed() -> void:
	if not _a.call("can_save_state"):
		_say("[color=red]refused: the machine cannot be serialised right now[/color]")
		return
	_snapshot = _a.call("save_state")
	_say("snapshot taken at t=%.2fs: %s" % [_a.vmcall("mjc_time"), _bytes(_snapshot.size())])


func _on_restore_pressed() -> void:
	if _snapshot.is_empty():
		_say("take a snapshot first")
		return
	var before: int = _b.vmcall("mjc_nq")
	if _b.call("restore_state", _snapshot):
		_say("restored into the right sandbox: nq %d -> %d, resumed at t=%.2fs" % [
			before, _b.vmcall("mjc_nq"), _b.vmcall("mjc_time")])
	else:
		_say("[color=red]restore refused[/color]")


func _on_save_file_pressed() -> void:
	if _snapshot.is_empty():
		_say("take a snapshot first")
		return
	var f := FileAccess.open(SNAPSHOT_PATH, FileAccess.WRITE)
	if f == null:
		_say("[color=red]could not open %s[/color]" % SNAPSHOT_PATH)
		return
	f.store_buffer(_snapshot)
	f.close()
	_say("wrote the snapshot to disk (%s)" % _bytes(_snapshot.size()))


func _on_load_file_pressed() -> void:
	if not FileAccess.file_exists(SNAPSHOT_PATH):
		_say("no snapshot on disk yet")
		return
	var f := FileAccess.open(SNAPSHOT_PATH, FileAccess.READ)
	_snapshot = f.get_buffer(f.get_length())
	f.close()
	_say("read the snapshot back from disk (%s)" % _bytes(_snapshot.size()))


func _on_pause_toggled(pressed: bool) -> void:
	_running = not pressed


func _bytes(n: int) -> String:
	return "%.1f MB" % (float(n) / 1048576.0) if n >= 1048576 else "%d bytes" % n
