extends Node3D

## Newton's cradle simulated inside a RISC-V sandbox, and moved between two of
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

var _a: Object = null
var _b: Object = null
var _a_holder: Node3D = null
var _b_holder: Node3D = null
var _running := true
var _snapshot := PackedByteArray()

var _status: RichTextLabel
var _log: RichTextLabel

# Snapping comes from the interaction system's LassoDB rather than a raycast:
# it scores every candidate against the ray, so an imprecise aim still picks
# the ball the pointer is nearest to.
const LassoScript := preload("res://addons/interaction_system/lassodb.gd")
var _lasso = null
var _ball_points := []          # LassoPoint per ball, index is the joint index
var _ball_nodes: Array[Node3D] = []
var _hovered := -1
var _grabbed := -1
var _pivots: PackedFloat64Array = PackedFloat64Array()


func _ready() -> void:
	_build_ui()
	_a = _make_sandbox()
	_b = _make_sandbox()
	if _a == null or _b == null:
		return

	if not _a.vmcall("mjc_load_builtin"):
		_say("[color=red]the guest could not build its model[/color]")
		return

	_a_holder = _build_meshes(Vector3(-0.09, 0, 0), Color(0.93, 0.80, 0.42))
	_b_holder = _build_meshes(Vector3(0.09, 0, 0), Color(0.55, 0.78, 0.96))
	_pivots = _a.vmcall("mjc_pivots")
	_say("left sandbox loaded the figure: nq=%d, %d equality constraints" % [
		_a.vmcall("mjc_nq"), _a.vmcall("mjc_neq")])
	_say("right sandbox has no model at all")


## The controls are built here rather than in the scene, so the scene stays a
## camera and a light and there is one place to read what each button does.
func _build_ui() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)

	# Top-left and only as wide as it needs to be, so the figure in the middle
	# stays clear. Everything but the buttons ignores the mouse: a panel that
	# spans the view swallows the drags meant for the balls behind it.
	var panel := PanelContainer.new()
	panel.set_anchors_preset(Control.PRESET_TOP_LEFT)
	panel.position = Vector2(12, 12)
	panel.custom_minimum_size = Vector2(520, 0)
	panel.mouse_filter = Control.MOUSE_FILTER_IGNORE
	layer.add_child(panel)

	var box := VBoxContainer.new()
	box.mouse_filter = Control.MOUSE_FILTER_IGNORE
	panel.add_child(box)

	var row := HBoxContainer.new()
	row.mouse_filter = Control.MOUSE_FILTER_IGNORE
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
	_status.fit_content = true
	_status.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_status.custom_minimum_size = Vector2(0, 60)
	box.add_child(_status)

	_log = RichTextLabel.new()
	_log.bbcode_enabled = true
	_log.scroll_following = true
	_log.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_log.custom_minimum_size = Vector2(0, 96)
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
	# Arguments reach the guest in registers rather than as Variant pointers,
	# which is what the guest's typed parameters expect. With this off a typed
	# parameter reads the pointer as a number and answers with garbage.
	sb.set_unboxed_arguments(true)
	return sb


## The host owns the tick. Four steps a frame at a 0.2 ms timestep keeps the
## collision stiff enough to read as elastic without the frame rate deciding
## how fast the figure swings.
func _process(_delta: float) -> void:
	if _a == null:
		return
	if _running:
		for i in range(4):
			_a.vmcall("mjc_step")
			if _b.vmcall("mjc_nq") > 0:
				_b.vmcall("mjc_step")
	_register_balls()
	_draw(_a, _a_holder)
	_draw(_b, _b_holder)
	_refresh_status()


## Meshes are built from the model's own geoms on first draw, so the host is
## not told what the figure is made of. MuJoCo geom types: 2 sphere, 3 capsule.
func _build_meshes(offset: Vector3, tint: Color) -> Node3D:
	# MuJoCo is Z-up, Godot is Y-up. A quarter turn about X on the holder
	# converts positions and orientations together, so the guest keeps handing
	# over MuJoCo coordinates untouched.
	var holder := Node3D.new()
	holder.position = offset
	holder.rotation = Vector3(-PI / 2.0, 0, 0)
	add_child(holder)
	holder.set_meta(&"tint", tint)
	# Kept explicitly: the holder also carries the pointing anchors, so child
	# order stopped being the geom index the moment those were added.
	holder.set_meta(&"meshes", [])
	return holder


func _mesh_for(g_type: int, sx: float, sy: float) -> Mesh:
	if g_type == 2:
		var s := SphereMesh.new()
		s.radius = sx
		s.height = sx * 2.0
		return s
	var c := CapsuleMesh.new()
	c.radius = sx
	# MuJoCo gives a capsule's half-length; Godot wants the whole height.
	c.height = sy * 2.0 + sx * 2.0
	return c


func _draw(sb: Object, holder: Node3D) -> void:
	if sb == null or holder == null:
		return
	if sb.vmcall("mjc_nq") == 0:
		holder.visible = false
		return
	holder.visible = true
	var g: PackedFloat64Array = sb.vmcall("mjc_geoms")
	var n := int(g.size() / 11.0)
	var meshes: Array = holder.get_meta(&"meshes")
	while meshes.size() < n:
		var mi := MeshInstance3D.new()
		var mat := StandardMaterial3D.new()
		mat.albedo_color = holder.get_meta(&"tint")
		mat.roughness = 0.35
		mat.metallic = 0.6
		mi.material_override = mat
		holder.add_child(mi)
		meshes.append(mi)
	for i in range(n):
		var o := i * 11
		var mi: MeshInstance3D = meshes[i]
		if mi.mesh == null:
			mi.mesh = _mesh_for(int(g[o]), g[o + 1], g[o + 2])
		var xform := Transform3D()
		# A capsule stands along +Z in MuJoCo and +Y in Godot.
		xform.basis = Basis(Quaternion(g[o + 8], g[o + 9], g[o + 10], g[o + 7])) * Basis(Vector3(1, 0, 0), PI / 2.0)
		xform.origin = Vector3(g[o + 4], g[o + 5], g[o + 6])
		mi.transform = xform
		if holder == _a_holder:
			var ball := _sphere_rank(g, i)
			if ball >= 0 and ball < _ball_nodes.size():
				_ball_nodes[ball].position = xform.origin
				var m: StandardMaterial3D = mi.material_override
				var lit := ball == _hovered or ball == _grabbed
				m.emission_enabled = lit
				m.emission = Color(1.0, 0.95, 0.7)
				m.emission_energy_multiplier = 1.6 if ball == _grabbed else 0.7


## One lasso point per ball, anchored to a node the draw loop keeps on the
## ball. Registered on the first frame that has geometry, because the ball
## count comes from the model rather than from here.
func _register_balls() -> void:
	if _lasso != null or _a_holder == null:
		return
	var g: PackedFloat64Array = _a.vmcall("mjc_geoms")
	if g.is_empty():
		return
	_lasso = LassoScript.new()
	for i in range(int(g.size() / 11.0)):
		if int(g[i * 11]) != 2:      # spheres are the balls
			continue
		var n := Node3D.new()
		_a_holder.add_child(n)
		_ball_nodes.append(n)
		var pt = LassoScript.LassoPoint.new()
		# A ball's own radius: the pointer has to fall within the ball to count
		# as aimed straight at it.
		pt.size = g[i * 11 + 1]
		pt.register_point(_lasso, n)
		_ball_points.append(pt)
	_say("%d balls registered for pointing" % _ball_points.size())


## Where the pointer is, in the model's own coordinates: the camera ray met
## with the plane the figure swings in.
func _pointer_in_model(screen_pos: Vector2) -> Vector3:
	var cam := get_viewport().get_camera_3d()
	var inv := _a_holder.global_transform.affine_inverse()
	var o: Vector3 = inv * cam.project_ray_origin(screen_pos)
	var d: Vector3 = inv.basis * cam.project_ray_normal(screen_pos)
	if absf(d.y) < 1e-6:
		return Vector3.ZERO
	return o + d * (-o.y / d.y)


func _input(event: InputEvent) -> void:
	if _lasso == null:
		return
	var mm := event as InputEventMouseMotion
	if mm != null:
		var cam := get_viewport().get_camera_3d()
		var q = LassoScript.LassoQuery.new()
		q.set_source(cam.project_ray_origin(mm.position), cam.project_ray_normal(mm.position))
		_lasso.query(q)
		_hovered = _ball_points.find(q.out_best_poi)
		if _grabbed >= 0:
			_hold(_grabbed, mm.position)
		return
	var mb := event as InputEventMouseButton
	if mb != null and mb.button_index == MOUSE_BUTTON_LEFT:
		if mb.pressed:
			_grabbed = _hovered
			if _grabbed >= 0:
				_say("holding ball %d" % (_grabbed + 1))
		else:
			if _grabbed >= 0:
				_say("released ball %d" % (_grabbed + 1))
			_grabbed = -1


## A held ball follows the pointer around its own pivot. The angle is the one
## the pivot and the pointer imply, so the string stays the length it is.
func _hold(index: int, screen_pos: Vector2) -> void:
	if index * 3 + 2 >= _pivots.size():
		return
	var px := _pivots[index * 3 + 0]
	var pz := _pivots[index * 3 + 1]
	var p := _pointer_in_model(screen_pos)
	_a.vmcall("mjc_select", index)
	# A +y hinge puts the ball at ball_x - px = -L*sin(theta), so the angle that
	# lands it under the pointer is atan2(px - p.x, ...). Writing p.x - px flips
	# it, and the ball chases the pointer the wrong way.
	_a.vmcall("mjc_hold", atan2(px - p.x, pz - p.z))


## Which ball a geom is, counting only spheres.
func _sphere_rank(g: PackedFloat64Array, geom_index: int) -> int:
	if int(g[geom_index * 11]) != 2:
		return -1
	var rank := 0
	for k in range(geom_index):
		if int(g[k * 11]) == 2:
			rank += 1
	return rank


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
	# Also to stdout, so a headless run reports what the on-screen log would.
	print(msg)
	if _log != null:
		_log.text += msg + "\n"


## Millimetres paired with something a reader can picture, because "4.3 mm"
## does not say whether an error matters.
func _mm(v: float) -> String:
	var anchors := {"a pencil": 7.0, "a AA battery": 14.5, "a nickel": 21.2,
		"a golf ball": 42.7, "an adult wrist": 57.0, "a soda can": 66.0}
	var best := ""
	var best_err := INF
	for anchor_name in anchors:
		var k: float = absf(v) / float(anchors[anchor_name])
		var err: float = absf(k - roundf(k))
		if k >= 0.8 and err < best_err:
			best_err = err
			best = "%.1f x %s" % [k, anchor_name]
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
