# Changes from MuJoCo Menagerie

This directory holds part of `ms_human_700` from `google-deepmind/mujoco_menagerie` (Apache-2.0,
see `LICENSE`), as `tools/human/vendor.exs` writes it.

- The XML files are the 23 that `MS-Human-700.xml` reaches through `<include>`. They are unchanged.
- Of the 189 meshes, 28 are here: the 4 that collide and the 24 ribs, whose bodies take their mass
  from them. Each was converted from STL or OBJ to OpenUSD text (`.usda`). An STL has its equal
  corners joined. An OBJ gives its first object only, a quad is cut along its shorter diagonal and
  a larger polygon becomes a fan.
- Everything else of the model (the other meshes, the texture, the scenes, the two other variants)
  is left out.

With these meshes the model's mass is 60.2221 kg, against 60.2193 kg with the stock files.
`FILES.txt` lists what the guest embeds.
