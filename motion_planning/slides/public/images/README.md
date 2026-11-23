# Image provenance

Every image in this directory was generated from this repository's own
code. There are no third-party assets here, so there is nothing to
attribute and no licence other than the project's own Michigan Honor
License.

Each is a headless-Chromium screenshot of `kineval/index.html` — the live
viewer this deck teaches — driven through `window.kinevalTest`, framed
with `frameCameraToWorld`, captured at `deviceScaleFactor: 2`. Every one
is reproducible in the browser from the URL and seed in its row, because
the module draws every random choice from one seeded generator
(`kineval/random.js`) and the HUD echoes the seed.

| File | URL | Start / goal | What it shows |
|---|---|---|---|
| `viewer_local_minima.png` | `kineval/?robot=mr2&world=local_minima&seed=2024` | base (-2.6, 2.4) → the default goal | The three-sphere pocket, the search tree flat on the ground, the translucent goal ghost at the origin, and the robot part-way along the plan at index 3/5. |
| `viewer_goal_ghost.png` | `kineval/?robot=mr2&world=basic&seed=77` | base (-3.4, -3.2) → the default goal | The goal ghost on its own: a translucent copy of the robot's own geometry standing at the zero configuration at the world origin. |
| `viewer_y.png` | `kineval/?robot=mr2&world=y&seed=2024` | opposite sides of the gate | The genuinely 3D world: two low spheres and one overhead at `y = 5.0`. Disproves the "x-z plane only" comment every upstream world file carries. |
| `viewer_narrow_passage.png` | `kineval/?robot=mr2&world=narrow_passage&seed=2024` | opposite sides of the wall | The gap sized to the robot. 8052 collision checks against 724 nearest-neighbour comparisons — the cost model, in one picture. |
| `viewer_bug_trap.png` | `kineval/?robot=mr2&world=bug_trap&seed=909` | inside the pocket → outside | Why a second tree matters: the tree rooted inside the trap cannot easily get out; the one rooted outside has no such problem. |
| `viewer_sealed_enclosure.png` | `kineval/?robot=mr2&world=sealed_enclosure&seed=31337` | inside the ring → outside | The termination case. No path exists; the planner reports `failed` at the iteration cap rather than hanging. |
| `viewer_s_higher.png` | `kineval/?robot=fetch&world=s_higher&seed=555` | sampled free | A high-DOF robot in a walled world, hitting the iteration cap — the curse of dimensionality, measured. |
| `viewer_collision_highlight.png` | `kineval/?robot=crawler&world=basic` | page default | The default page load, in collision: `leg5_lower` against obstacle 1, highlighted red by `scene.js` from `poseIsCollision`'s return value. |

The capture script is `/ai/development/misc/` work-in-progress tooling and
is not part of the repository; the URLs above are the reproducible record.
