# MS-Human-700 in the guest

The guest builds the MS-Human-700 muscle skeleton (MuJoCo Menagerie, Apache-2.0) from files
embedded in the ELF. This page holds what the source does not say in comments: the source keeps
its comments under 3% of its lines, which `tools/density/comment_density.exs` checks in CI.

## The model build, `guest/human.cpp`

The stock model is edited in the guest and nowhere else, so no second copy of its XML exists.

1. The six pelvis joints become one free joint named `root`, as the retargeted clips expect. The
   stock keyframe is 85 values for the six-joint root, so it no longer fits and is removed.
2. A floor plane is added.
3. A mesh geom goes when it neither collides nor gives its body mass, and then every mesh,
   texture and material nothing refers to goes with it. A body without an `<inertial>` takes its
   mass from its geoms, so the meshes of such a body stay and no inertia is typed by hand. Of 189
   meshes 28 stay: the 4 that collide and the 24 ribs.
4. A mesh that stays is read from OpenUSD text and given to the model as points and triangles.
   The stock XML names an STL or OBJ file. The embedded mesh has that name with `.usda` in its
   place, and MuJoCo's own mesh readers are not linked. Only the XML files go to MuJoCo's reader.
   The parser has already made a mesh's path relative to the model directory, and a path such as
   `a/b/../c/d.stl` is read as `a/c/d.stl`.

The `.usda` reader takes the one `Mesh` that `tools/human/vendor.exs` writes. It reads the
numbers of three array attributes, each as the text between `name = [` and the closing bracket:
`faceVertexCounts`, where every count must be 3, `faceVertexIndices` and `points`.

`human::build(withhold, error, kept)` compiles the model. `withhold` is the index of one embedded
file to leave out, or -1; a control uses it to show that a missing file stops the build. On
failure the answer is null and `error` says why. `kept` receives the mesh files the built model
reads. `human::files()` gives the paths of the embedded files, relative to the model directory.

## Embedding, `guest/embed_tree.cmake`

`embed_tree` embeds every file named in `third_party/ms_human_700/FILES.txt`: a C file whose
assembly includes the bytes, and a header with one table of name, begin and end. The first row is
the main model file.

A double-precision guest needs a guest API whose library carries the double definition too,
because the Variant size marker is compiled there. The vendored copy defines it on the program
only, and its sources assert the single-precision sizes, so it would link a guest that a double
engine refuses to load. `guest/CMakeLists.txt` stops at configure time when a double build is
pointed at such an API.

## Vendoring, `tools/human/vendor.exs`

```sh
elixir tools/human/vendor.exs <menagerie>/ms_human_700 <kept-meshes.txt>
```

It copies the part of the model the guest embeds into `third_party/ms_human_700` and writes
`FILES.txt`. The XML files are the ones `MS-Human-700.xml` reaches through `<include>`, where an
include is relative to the file that names it, and they are copied unchanged. The meshes are the
ones named in `<kept-meshes.txt>`, one path a line relative to the model directory: the list the
guest reports from `mjc_human_kept` after its own rule ran, which the model gate compares with
the embedded ones.

Each mesh is written as OpenUSD text. A binary STL is an 80-byte header, a count, then 50 bytes
a triangle; its points stay 32-bit floats and its equal corners are joined, as MuJoCo's own
reader does. Only the first object of an OBJ file is the mesh, as MuJoCo reads it: `Rib1L.obj`
carries twelve. Its corners are renumbered to the points it uses, a quad is cut along its shorter
diagonal, and a larger polygon becomes a fan. A number is written as the shortest text that reads
back as the same number.

## Calls

| Call | What it does |
| --- | --- |
| `mjc_load_human(withhold) -> bool` | builds the model with a free root and a floor |
| `mjc_human_error()`, `mjc_human_files()`, `mjc_human_kept()` | why the last build failed; the embedded paths; the mesh files the built model reads |
| `mjc_person(index) -> bool` | selects which person's data every other call reads and writes. One model serves several people, each with their own data. Data for a new index is made at the model's default pose. At most 8 people |
| `mjc_names(objtype) -> PackedStringArray` | names of every object of one MuJoCo object type, in id order: 1 body, 3 joint, 5 geom, 7 mesh, 18 tendon, 19 actuator. An unnamed object gives an empty string |
| `mjc_mass() -> float` | total mass in kilograms |
| `mjc_qfrc_actuator()` | generalised forces of the actuators in the last forward pass. A muscle pulls passively at zero control, so on a muscle model the inverse of the forward acceleration returns these, not zero |
| `mjc_inverse(qacc)` | inverse dynamics at the set positions and velocities. An empty answer means `qacc` was not `nv` long |
| `mjc_tendon_paths()` | every tendon's path for the selected person, as the last forward pass left it. Flat, for each tendon in id order: the number of path points, the tendon's length, then that many world-space x, y, z |
| `mjc_tendon_wraps()` | for each tendon, 1 when the model's path for it names a wrapping sphere or cylinder, else 0 |
| `mjc_geom_table()` | what `mjc_geoms` leaves out for a host that sorts geoms by body and by collision. 13 numbers a geom, in id order: body id, type, contype, conaffinity, three sizes, world position, and the world z axis of the geom's frame, which is the long axis of a capsule or cylinder |

Consecutive points of a tendon path join as the engine's own renderer joins them: a straight
segment, which over a wrapping geom is the chord between the two points on its surface, so the
polyline is never longer than the tendon. A tendon that names a wrapping geom may run over its
surface, and then its length is more than its polyline. A pulley ends one branch and starts the
next; its point is three NaN and no segment touches it.

## Gates

Each runs from `project/` and prints a `RESULT:` or `INVERSE-` line. The model build runs past
the default budget of one guest call, so the gates raise `execution_timeout`. A call that throws
inside the guest answers null, so the gates keep such answers untyped and still end.

`../tools/human/human_check.gd -- <mujoco.elf> [--withhold-rib] [--plant-held] [--qpos <file>]`
checks the sizes of the free-root model (nq 86, nv 85, 700 muscles), every joint name of the
retargeted clips, the total mass, the meshes the build kept against the embedded ones, and that
the forward pass's own acceleration comes back from the inverse as the muscles' passive force
and nothing more, on two people who share the model: the second holds the pose and the first
stays as loaded. `--withhold-rib` leaves one rib mesh out of the build, which must then fail.
`--plant-held` feeds the held torque to the inverse check. `--qpos` reads 86 numbers, one a
line, as the pose; without it the pose is the model's own.

`../tools/human/tendon_check.gd -- <mujoco.elf> [--plant-swap] [--plant-geom]` checks 700
tendons, each with two points or more; a tendon whose path names no wrapping geom exactly as long
as its polyline; a polyline never longer than its tendon; one geom row for each geom, each on a
body the model has; the floor plane; and the measured counts of wrapping tendons, colliding
geoms and mesh geoms. The equality is checked first where it holds, because it is the stricter
check and the one the swap is for. `--plant-swap` exchanges the points of two tendons of
different length that wrap nothing. `--plant-geom` puts one geom on a body the model does not
have. Each must fail.

`../tools/inverse/inverse_check.gd -- <mujoco.elf> [--plant-held]` runs on the built-in cradle,
drawn back and at rest. The forward pass's own acceleration must come back as no force. Zero
acceleration holds the ball, so a torque appears, and `--plant-held` feeds that to the gate,
which must fail. An array one short of `nv` must come back empty.

`elixir tools/elf/check_variant.exs [--plant] <elf>=<24|40> ...` reads the Variant size a
committed guest was built for from its `SBXV` marker, which is those four letters and a 32-bit
size. A single-precision engine loads 24 and a double-precision engine loads 40, and either
refuses the other, so a wrong marker is a guest that never runs. `--plant` reads each marker as
the other size, and the check must then fail.
