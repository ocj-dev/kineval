# Full working KinEval reference viewer (forward kinematics, so far)

A working port of the upstream kineval-stencil (github.com/autorob/kineval-stencil) viewer —
`home.html` plus the `kineval/*.js` modules it loads — scoped for now to what the
`forward_kinematics/` module's completed STENCIL work can drive: load a robot, navigate and
rotate its joints interactively, move its base around the ground plane, and watch forward
kinematics place every link in the world every frame. Same `robot.control` → `applyControls` →
forward-kinematics → render loop sequence as upstream's own `my_animate()`, and the same
key bindings (`w/s/a/d/q/e` base motion, `j/k/l/h` active-joint navigation, `u/i` joint control,
`z/x` camera zoom) ported from upstream's `kineval_userinput.js`, unchanged.

Live at `https://ocj-dev.github.io/kineval/kineval/`.

Everything this page needs beyond `forward_kinematics/` — inverse kinematics, RRT-Connect motion
planning, collision detection, PID/dynamics setpoint control — isn't built yet (see `../README.md`
for the overall build order); those controls (`p`, `m`/`n`/`b`, `o`/`c`, `0`-`9`, `r`/`f` in
upstream's own scheme) will be added back in as each of those modules lands.

## Test robots

Three of the upstream stencil's own `robots/` examples are ported in `robots/` here — select
between them with `?robot=`:

| `?robot=` | Upstream source | Notes |
|---|---|---|
| `urdf_example` (default `forward_kinematics/`'s own test case) | `robots/robot_urdf_example.js` | simple branching 4-link arm |
| `mr2` | `robots/robot_mr2.js` | upstream's own partial (right-arm-only) humanoid stencil exercise |
| `crawler` **(default here)** | `robots/robot_crawler.js` | 8-legged crawler — the widest-branching test case |

**Not included: `baxter`, `fetch`, `sawyer`** — upstream's three real-world robots, each shipping
20–35MB of STL/Collada meshes. Impractical to vendor into this teaching repo; all three use the
exact same `<visual>`/mesh-loading machinery the `forward_kinematics/` deck's geometry-format
slides already cover with small hand-authored samples, so nothing conceptual is lost by leaving
them out here.

## Path planning reference, already live

The 2D path-planning reference implementation from `../pathfinding/reference/` is deployed here
as a standalone page, independent of the full 3D viewer above:
`https://ocj-dev.github.io/kineval/kineval/pathplanning/`. It's the same
`search_canvas.html`/`graph_search.js`/etc. shown in the `pathfinding/` deck's code-walkthrough
slides, just served at this URL too so it can be linked to directly (see that deck's "Try this
yourself" slide).
