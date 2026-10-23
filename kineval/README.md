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

Jacobian-based inverse kinematics and P-servo/clock setpoint control are now built
(`inverse_kinematics.js`, `servo_control.js`), matching upstream's own key scheme:

- `p` — one-shot: iterate inverse kinematics one step toward `params.ik_target` (the manipulator
  Jacobian transpose or pseudoinverse times the position error, applied through `.control` like
  every other input in this viewer)
- `r`/`f` — held: move the IK target up/down along world Y
- `o` — held: run the P-servo controller toward `params.setpoint_target`
- `c` — held: derive `params.setpoint_target` from the wall clock, then P-servo toward it

The IK Jacobian is built over the end-effector's **arm chain**, derived generically by walking
`.parent` links/joints from `robot.endeffector.frame` back to `robot.base` (not hardcoded per
robot) — see `inverse_kinematics.js#deriveArmChain`. All six ported robots carry a
`robot.endeffector = {frame, position}` field, so all six have a usable arm chain; for mr2 this
correctly derives `['clavicle_right_yaw','shoulder_right_yaw','upperarm_right_pitch',
'forearm_right_yaw']`, excluding `clavicle_left_roll` (a disconnected sibling branch off the base
link that the backward walk never visits).

Still not built yet (see `../README.md` for the overall build order): RRT-Connect motion planning
and collision detection — those controls (`m`/`n`/`b`, `0`-`9` pose-setpoint slots in upstream's
own scheme) will be added back in as those modules land.

## Test robots

Six of the upstream stencil's own `robots/` examples are ported in `robots/` here — select
between them with `?robot=`:

| `?robot=` | Upstream source | Notes |
|---|---|---|
| `urdf_example` | `robots/robot_urdf_example.js` | simple branching 4-link arm |
| `mr2` | `robots/robot_mr2.js` | upstream's own partial (right-arm-only) humanoid stencil exercise |
| `crawler` **(default here)** | `robots/robot_crawler.js` | 8-legged crawler — the widest-branching test case |
| `fetch` | `robots/fetch/fetch.urdf.js` | 21 links; the only one with **prismatic** joints |
| `baxter` | `robots/baxter/baxter.urdf.js` | 20 links, two 7-DOF arms |
| `sawyer` | `robots/sawyer/sawyer.urdf.js` | 10 links, a single 7-DOF arm |

`fetch`, `baxter` and `sawyer` are ported with their **kinematics intact but their meshes left
out** — 20–35MB apiece, impractical to vendor into this teaching repo. `scene.js` synthesizes a
skeleton for them from each link's own joint offsets, so the proportions are the robot's real link
lengths. Forward kinematics reads origins, axes and angles and never meshes, so it is unaffected.

One upstream data defect is corrected in the port: `baxter.urdf.js` declares two fixed joints,
`headnod` and `display_joint`, **both** joining `head` → `screen`, which makes the description a
DAG rather than a tree and leaves a depth-first traversal visiting `screen` twice. The port keeps
`headnod` (declared first) and drops `display_joint`.

## Path planning reference, already live

The 2D path-planning reference implementation from `../pathfinding/reference/` is deployed here
as a standalone page, independent of the full 3D viewer above:
`https://ocj-dev.github.io/kineval/kineval/pathplanning/`. It's the same
`search_canvas.html`/`graph_search.js`/etc. shown in the `pathfinding/` deck's code-walkthrough
slides, just served at this URL too so it can be linked to directly (see that deck's "Try this
yourself" slide).
