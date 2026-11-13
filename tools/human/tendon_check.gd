extends SceneTree


const TENDONS := 700
const WRAPPING := 160  # tendons whose path names a wrapping sphere or cylinder
const COLLIDING := 19  # geoms with contype or conaffinity not zero
const MESHES := 28  # geoms of type mesh
const STRIDE := 13  # body, type, contype, conaffinity, size xyz, pos xyz, z axis xyz
const TYPE_PLANE := 0
const TYPE_MESH := 7
const OBJ_GEOM := 5
const TOLERANCE := 1e-9
const ROUNDS := 20


func _fail(msg: String) -> void:
	printerr("TENDON-FAIL " + msg)
	print("RESULT: FAIL " + msg)
	quit(1)


func _polyline(points: PackedFloat64Array) -> float:
	var total := 0.0
	for k in range(3, points.size(), 3):
		if is_nan(points[k - 3]) or is_nan(points[k]):
			continue
		var dx: float = points[k] - points[k - 3]
		var dy: float = points[k + 1] - points[k - 2]
		var dz: float = points[k + 2] - points[k - 1]
		total += sqrt(dx * dx + dy * dy + dz * dz)
	return total


func _init() -> void:
	var args := OS.get_cmdline_user_args()
	if args.size() < 1:
		_fail("usage: -- <mujoco.elf> [--plant-swap] [--plant-geom]")
		return
	var plant_swap := args.has("--plant-swap")
	var plant_geom := args.has("--plant-geom")
	if ClassDB.class_has_method("Sandbox", "set_jit_enabled"):
		ClassDB.class_call_static("Sandbox", "set_jit_enabled", false)
	var sb = ClassDB.instantiate("Sandbox")
	if sb == null:
		_fail("Sandbox class missing")
		return
	sb.set_memory_max(1024)
	sb.set_allocations_max(1 << 22)
	sb.set_unboxed_arguments(true)
	sb.set("execution_timeout", 4000000)
	sb.load_buffer(FileAccess.get_file_as_bytes(args[0]))
	for call in ["mjc_load_human", "mjc_tendon_paths", "mjc_tendon_wraps", "mjc_geom_table"]:
		if not sb.has_function(call):
			_fail("%s has no %s" % [args[0], call])
			return

	var loaded = sb.vmcall("mjc_load_human", -1)
	if loaded != true:
		_fail("the model did not build: %s" % str(sb.vmcall("mjc_human_error")))
		return
	if sb.vmcall("mjc_forward") != true:
		_fail("the forward pass was refused")
		return

	var wraps = sb.vmcall("mjc_tendon_wraps")
	var flat = sb.vmcall("mjc_tendon_paths")
	var t0 := Time.get_ticks_usec()
	for i in ROUNDS:
		sb.vmcall("mjc_tendon_paths")
	var paths_ms := (Time.get_ticks_usec() - t0) / 1000.0 / ROUNDS
	if not (wraps is PackedInt32Array) or not (flat is PackedFloat64Array):
		_fail("the tendon calls answered %s and %s" % [type_string(typeof(wraps)), type_string(typeof(flat))])
		return

	var lengths := PackedFloat64Array()
	var points: Array[PackedFloat64Array] = []
	var at := 0
	while at + 2 <= flat.size():
		var count := int(flat[at])
		if count < 0 or at + 2 + count * 3 > flat.size():
			_fail("tendon %d claims %d points past the end of the reply" % [lengths.size(), count])
			return
		lengths.append(flat[at + 1])
		points.append(flat.slice(at + 2, at + 2 + count * 3))
		at += 2 + count * 3
	if at != flat.size() or lengths.size() != TENDONS or wraps.size() != TENDONS:
		_fail("%d tendons read to %d of %d numbers, %d wrap flags, against %d" % [lengths.size(), at, flat.size(), wraps.size(), TENDONS])
		return

	if plant_swap:
		var first: int = wraps.find(0)
		var other := -1
		for i in range(first + 1, TENDONS):
			if first >= 0 and wraps[i] == 0 and absf(lengths[i] - lengths[first]) > 1e-3:
				other = i
				break
		if other < 0:
			_fail("control: no two tendons of different length to swap")
			return
		print("TENDON control: points of tendons %d and %d swapped" % [first, other])
		var held := points[first]
		points[first] = points[other]
		points[other] = held

	var wrapping := 0
	var arcs := 0
	var total_points := 0
	var worst := 0.0
	for i in TENDONS:
		var count := points[i].size() / 3
		total_points += count
		if count < 2:
			_fail("tendon %d has %d points" % [i, count])
			return
		var poly := _polyline(points[i])
		if not is_finite(poly) or not is_finite(lengths[i]) or lengths[i] <= 0.0:
			_fail("tendon %d has length %s and polyline %s" % [i, str(lengths[i]), str(poly)])
			return
		if wraps[i] == 0:
			worst = maxf(worst, absf(poly - lengths[i]) / lengths[i])
			if absf(poly - lengths[i]) > TOLERANCE * lengths[i]:
				_fail("tendon %d wraps nothing, polyline %.12f against length %.12f" % [i, poly, lengths[i]])
				return
		if poly > lengths[i] + TOLERANCE:
			_fail("tendon %d polyline %.12f is longer than its length %.12f" % [i, poly, lengths[i]])
			return
		if wraps[i] != 0:
			wrapping += 1
			if lengths[i] - poly > TOLERANCE * lengths[i]:
				arcs += 1
	print(
		(
			"TENDON tendons=%d points=%d wrapping=%d on_a_surface=%d worst=%s paths=%.3f ms"
			% [TENDONS, total_points, wrapping, arcs, String.num_scientific(worst), paths_ms]
		)
	)

	var table = sb.vmcall("mjc_geom_table")
	var ngeom: int = sb.vmcall("mjc_ngeom")
	var nbody: int = sb.vmcall("mjc_nbody")
	var names: PackedStringArray = sb.vmcall("mjc_names", OBJ_GEOM)
	if not (table is PackedFloat64Array) or table.size() != ngeom * STRIDE or ngeom <= 0:
		_fail("the geom table is %s against %d geoms of %d" % [str(table.size()) if table is PackedFloat64Array else "missing", ngeom, STRIDE])
		return
	if plant_geom:
		table[(ngeom / 2) * STRIDE] = nbody
	var colliding := 0
	var meshes := 0
	for i in ngeom:
		var row: PackedFloat64Array = table.slice(i * STRIDE, (i + 1) * STRIDE)
		if row[0] < 0.0 or row[0] >= nbody or row[0] != floorf(row[0]):
			_fail("geom %d is on body %s of %d" % [i, str(row[0]), nbody])
			return
		var axis := sqrt(row[10] * row[10] + row[11] * row[11] + row[12] * row[12])
		if absf(axis - 1.0) > TOLERANCE:
			_fail("geom %d has an axis of length %.12f" % [i, axis])
			return
		if row[2] != 0.0 or row[3] != 0.0:
			colliding += 1
		if int(row[1]) == TYPE_MESH:
			meshes += 1
	var floor_at := names.find("floor")
	print("GEOM geoms=%d bodies=%d colliding=%d meshes=%d floor=%d" % [ngeom, nbody, colliding, meshes, floor_at])
	if floor_at < 0:
		_fail("no geom is named floor")
		return
	var floor_row: PackedFloat64Array = table.slice(floor_at * STRIDE, (floor_at + 1) * STRIDE)
	if int(floor_row[0]) != 0 or int(floor_row[1]) != TYPE_PLANE or floor_row[2] == 0.0 or absf(floor_row[12] - 1.0) > TOLERANCE:
		_fail("the floor is body %s, type %s, contype %s, up %s" % [str(floor_row[0]), str(floor_row[1]), str(floor_row[2]), str(floor_row[12])])
		return
	if wrapping != WRAPPING or colliding != COLLIDING or meshes != MESHES:
		_fail("wrapping %d, colliding %d, meshes %d against %d, %d, %d" % [wrapping, colliding, meshes, WRAPPING, COLLIDING, MESHES])
		return
	print("RESULT: PASS")
	quit(0)
