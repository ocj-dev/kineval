# Mesh-format example assets

Real robot meshes, each one an actual `<mesh filename="...">` target in a real
URDF, used by the deck's three geometry-format slides so each format is shown
as it's genuinely used in the wild rather than as a synthetic sample shape.

| File | Format | From | Referenced in that URDF as |
|---|---|---|---|
| `fetch_l_wheel_link.STL` | STL | Fetch (`fetch_description`, via [autorob/kineval-stencil](https://github.com/autorob/kineval-stencil) `robots/fetch/`) | `package://fetch_description/meshes/l_wheel_link.STL` |
| `fetch_head_pan_link.dae` + `head_pan_uv.png` | Collada | Fetch (same source) | `package://fetch_description/meshes/head_pan_link.dae` |
| `spot_front_left_lower_leg.obj` | OBJ | Spot (`spot_description`, from this course's `autorob_agentic` robots) | `package://spot_description/meshes/base/visual/front_left_lower_leg.obj` |

The Collada file references `head_pan_uv.png` internally (`<init_from>`), so that
texture has to sit beside it — three.js's `ColladaLoader` resolves it relative to
the `.dae`. The OBJ file names `spot.mtl` in an `mtllib` line, but three.js's
`OBJLoader` never fetches that on its own (materials only load via `MTLLoader`),
and `MeshFormatExamplePanel.vue` assigns its own material anyway, so the `.mtl`
is deliberately not vendored.

**Why OBJ comes from Spot and not Fetch or PR2:** neither of those two ships any
OBJ meshes — Fetch is STL + Collada, PR2 is STL + Collada. That's itself one of
the points the OBJ slide makes.
