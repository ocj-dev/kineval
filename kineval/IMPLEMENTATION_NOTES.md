# kineval/ — motion planning implementation notes

What was built, what was fixed, and what surprised us, while adding
collision detection and RRT-Connect motion planning to the viewer.
Written 2026-11-22.

Files added: `collision.js`, `motion_plan.js`, `random.js`, `worlds/` (nine
ports plus five parametric generators).
Files modified: `index.html`, `scene.js`, `userinput.js`, `README.md`,
`robots/robot_*.js` (all six).
Files deliberately **not** modified: `kinematics.js` — see "Pure versus
side-effecting FK" below.

---

## What a configuration is here

```
q = [ base_x, base_y, base_z, base_roll, base_pitch, base_yaw,
      ...one entry per NON-FIXED joint, in robot.joints enumeration order ]
```

`collision.js#buildConfigurationIndex` derives this from whatever
`ROBOT_FACTORIES` hands over. No robot is named anywhere in `collision.js`
or `motion_plan.js`, and the Phase 4 tests loop over the cross product of
registered robots and worlds rather than naming one — so adding a seventh
robot extends coverage with no code or test change.

**Divergence from upstream: fixed joints are skipped.**
`kineval_rrt_connect.js:111` includes every joint, giving each fixed joint
a q dimension whose value can never matter. Fetch has five of twenty. That
wastes sampler effort in proportion to how many there are and, worse, tells
a student that a welded-shut joint is part of the robot's configuration. It
is not.

---

## Pure versus side-effecting forward kinematics

The single best design observation this module produced, and the one worth
a slide.

`kinematics.js#traverseFKLink` assigns the world transform into the link as
it walks:

```js
// kinematics.js:117-119
function traverseFKLink(robot, linkName, xform) {
    var link = robot.links[linkName];
    link.xform = xform;
```

That is exactly right for rendering — the viewer wants one current pose,
and `scene.js#updateRobotMeshes` reads `link.xform` straight back out. It
is fatal for planning, where every question is about a configuration the
robot is *not* in. Calling `robotForwardKinematics` to test a candidate
would destroy the pose on screen, and a planner evaluating thousands of
candidates per second would make the robot flicker through its own search.

So `collision.js#collisionForwardKinematics(robot, qmap, q)` does the same
traversal and **returns** a `{linkName: xform}` map, writing nothing. The
Phase 4 tests assert that property directly: snapshot every `link.xform`,
run a collision query on a different configuration, compare.

`kinematics.js` is an already-taught deliverable of the forward-kinematics
module and is deliberately left alone. The duplicated traversal is the
price of that, and it is small — but the two **must** agree, so
`collisionJointTransform` mirrors `traverseFKJoint`'s type branch exactly.
If they ever diverge, the planner is checking collisions against a robot
that is not the one on screen.

---

## The collision test, and why it is deliberately wrong at the corners

Upstream's form, kept exactly (`kineval_collision.js:161-186`):

1. transform the obstacle sphere's centre into the link's own frame, via
   `matrix_invert_affine(link.xform)`;
2. compare that centre against the link's bounding box **inflated by the
   sphere radius on every axis** — a pure min/max comparison, no distance
   computation, no square roots.

This is conservative. Inflating a box by `r` on all three axes replaces the
sphere with a box near the box's corners, so it reports collisions that are
not geometrically real.

**That is accepted on purpose, and the exact test is not implemented here
in any form, not even behind a flag.** The collision lecture
(`autorob_17_collision_detection`, slides 131–136) walks a sphere to a
corner, asks "Is this obstacle in collision?", annotates **"True separating
axis not tested"**, shows all four comparisons firing anyway, and closes
with "umm… take comp geom". The false positive is *taught*.

Phase 4 contains a test, `corner conservatism (deliberate false positive)`,
that places a sphere centre `(0.8r, 0.8r, 0.8r)` beyond a box corner — a
true distance of `√3 × 0.8r = 1.39r`, so the real sphere is clear of the
corner — and asserts that collision **is** reported. **If someone adds the
exact clamp-and-compare test to `collision.js`, that test fails.** That is
the point. Please do not helpfully finish it; the exact test is deck
material, built in the presentation panel's own JavaScript so the two
verdicts can be visualized disagreeing.

### Why the link frame

Because the test runs in the link's own frame, a rotated link needs no
re-axis-alignment of its box — that is the whole point of inverting
`link.xform`, and it is why the **sphere** is what gets transformed. A
sphere is rotation invariant; a box is not. Working in the world frame
would force re-axis-aligning over the transformed corners and would be
looser still.

Worth saying because the collision deck sets up a confusion it never
resolves: slide 114 defines an AABB as world-frame and an OBB as
object-frame, then slide 119 says "AABB link tested against spherical
obstacles in link frame". By slide 114's own definition, `link.bbox` seen
from the world *is* an OBB. KinEval never needs the Separating Axis Theorem
machinery that normally implies, because it moves the sphere instead of the
box. That is the one-line insight, and it is what the Phase 4 rotation-
invariance test checks.

### Divergences from upstream in the collision code

- **The boundary test uses every link's bounding-box corners, not just the
  base origin.** Upstream (`kineval_collision.js:53`, and the assignment
  slide repeats it) tests `q[0]` and `q[2]` only — the base *origin*, no
  extent — so a robot can drive until its arm is well past the wall as long
  as its origin is inside. This is an **enhancement, not a port**.
- **`matrix_invert_affine` replaces `numeric.inv`.** Upstream calls a
  general 4×4 matrix inversion on a rigid transform, in the innermost loop
  of the function that dominates planning time; the affine version is
  sitting right there in upstream's own source, commented out.
- **The visual-origin offset is applied.** Upstream's
  `traverse_collision_forward_kinematics_link` computes `local_link_xform`
  from `link.visual.origin.xyz` and then inverts `mstack` instead, never
  using it — so any link whose visual origin is offset from its joint frame
  is tested in the wrong frame.
- **`poseIsCollision` returns *what* collided** (`{link, obstacle}` or
  `{link, boundary}`) rather than a bare link name. It costs nothing, lets
  `scene.js` colour the offending link red, and turns "the planner never
  finds a path" into a sentence.
- **Plain top-level functions with explicit arguments**, not upstream's
  `kineval = {}` namespace object — this project's house style, and the
  same choice `servo_control.js` already documents in its own header.

---

## The planner

`motion_plan.js` is a **second** implementation of RRT-Connect in this
repository. The first, `motion_planning/reference/rrt.js`, plans over a 2D
canvas where configuration space *is* the plane. The duplication is
deliberate — reading them side by side is the clearest statement of what
"configuration space" buys you — and structurally required, since each
module's `reference/` must be self-contained for the Slidev deck to quote
it.

### What this does that upstream does not

Each of these traces to a defect recorded in the Phase 0 review findings
(§K-6.1).

| Upstream | Here |
|---|---|
| `new_config` tests only the endpoint it arrives at | `edgeIsValid` validates along the whole edge |
| `compute_configuration_distance` sums metres and radians raw (its weighting branch computes the same thing on both sides) | per-DOF weights, each DOF divided by its own range |
| the `KE 2` note "distance computation needs to consider modulo" is unaddressed | circular difference for genuinely circular DOFs |
| `eps = 0.5` hardcoded in raw mixed units | step size is a fixed fraction of the C-space diameter |
| `Math.random()` | one seeded generator (`random.js`) |
| loops on an `rrt_iterate` flag | hard iteration cap, reports `failed` |
| dead "normalize all angles" block (35 lines; loops over `vertices[i].length` on an object) | not carried over |
| `find_path` recursive DFS whose visited-reset never executes | iterative parent-pointer walk |
| no counters | collision-check and nearest-neighbour counters, in the HUD |

### Angles are on a continuous real line, with no modulo wrap

**Instructed, and it reverses both upstream and the implementation plan.**
Upstream's own standing note is "Distance computation needs to consider
modulo for joint angles" (`kineval_rrt_connect.js:29`, never acted on) and
the plan asked for a modulo-aware circular difference. Neither is what this
does. `configurationDifference(a, b)` is `a - b`, for every DOF including
every angular one. A joint at 0.1 rad and one at 6.3 rad are **6.2 rad
apart, not 0.08** — reaching the second means winding the joint almost all
the way round, and the planner plans, validates and executes that motion
rather than pretending it is a twitch.

Three reasons it is the right call, two of which were learned here rather
than reasoned out in advance:

1. **A revolute joint with limits cannot wrap at all.** Its short way round
   from −2.2 to +2.2 passes through a region the joint physically cannot
   enter, so a modulo metric reports a distance the robot cannot travel and
   an interpolation that drives the joint through its own hard stop.
2. **Even for a genuinely continuous joint, wrapping makes the metric
   disagree with the motion.** Under a modulo difference the edge from yaw
   0 to yaw 2π has length **zero**, so edge validation performs a single
   check and the planner is free to splice in a full revolution it never
   examined. Found during verification, not in theory.
3. **One rule covers every DOF.** Metres and radians are treated alike; the
   weighting is the only thing making them comparable, and there is no
   second notion of distance to keep in sync with interpolation, sampling
   and limit enforcement.

Two consequences, both accepted:

- Two configurations that look identical on screen can be far apart in
  configuration space, and the planner may wind a continuous joint the
  long way round. That is honest — the robot really does have to turn
  that far.
- It costs real time. Edges that the wrapping version collapsed to nothing
  are now full revolutions that must be checked. `crawler`/`world_basic`
  went from 46 iterations and 0.8 s to 450 iterations and 9 s. That is the
  price of the path being the path that gets executed.

`jointLimitRange` therefore reports `bounded: true/false` rather than
`circular`. For a continuous joint `[0, 2π)` is the **sampling interval,
not a constraint**: a value outside it is a joint wound further round, not
a limit violation, and the verification's limit check skips unbounded
joints for exactly that reason.

### One definition of "along an edge"

`interpolateConfiguration(qA, qB, t, weights)` is the single definition,
used by `edgeIsValid`, by `newConfig`, and exposed on the test hook so the
verification cannot roll its own. It did once: an early audit used
`a + t*(b - a)` while the planner was still wrapping, and reported 173
colliding configurations on an edge whose endpoints differed by exactly 2π
of yaw — a zero-length edge, swept the long way round, through a wall the
robot never passed through. The planner was right and the test was wrong,
which is the worst way for a test to fail.

### Goal convention — and a default that made the planner look broken

The goal is **`params.goal_config`**, one q vector, set in exactly three
ways: the **Set Goal** button (the current configuration), loading a
pose-setpoint slot with `[1-9]`, or the default. One field, so there is
one answer to "what is the goal", and the HUD names its source.

**The default is the zero configuration at the world origin**, matching
upstream (`kineval_rrt_connect.js:128-131`) and the assignment the
collision lecture states: *"generate a collision free motion plan to the
world origin and zero joint angle configuration"*.

An earlier version defaulted to the active setpoint slot instead,
reasoning that the zero configuration is sometimes itself in collision.
That reasoning was right about the risk and **badly wrong about the
consequence**. With no slot stored, `setpoint_target` is `{}`, so the
goal silently became the **start** configuration — and pressing `m`
"solved" a zero-length problem in one iteration, reporting `found` with a
two-configuration path. The planner appeared never to run at all. It was
reported as exactly that.

Two lessons worth keeping:

- **A default that is occasionally unsolvable, and says so, beats one
  that is always trivially solved.** The failure mode of the first is a
  readable message; the failure mode of the second is a user concluding
  the feature is broken.
- **A goal equal to the start is now refused explicitly**, with a message
  telling the user to drive away or press Set Goal. It is not an error in
  the algorithm, but it is always a mistake in the query, and silence
  about it was the whole bug.

The setpoint slots store the **base pose as well as** the joint angles,
which upstream's joint-only setpoints do not. Without that, "plan to slot
1" would mean "reach that arm configuration wherever you happen to be
standing", and thirteen of the fourteen worlds would be solvable without
the base moving at all.

Both ends of a query are checked for collision up front and refused with a
readable reason, rather than planned from and silently never solved.

### `Shift+[1-9]` never worked, because of `e.key`

Storing a slot was dead code from the day it was written. The handler
read `e.key >= '1' && e.key <= '9'` and branched on `e.shiftKey` — and
**`Shift+3` does not produce `'3'`**. On a US layout it produces `'#'`;
on other layouts, something else again. So the shifted branch was
unreachable and storing a slot did nothing, silently.

The fix is to key on **`e.code`** (`Digit1`…`Digit9`), which is the
physical key and is layout-independent. The in-code comment that
asserted the opposite is gone.

Found by a headless test pressing the key and reading the event back,
not by reading the code — the code looked obviously right.

### What the HUD shows, and why

The line under `active joint` reports, every frame:

```
planner = found   extend = advanced   connect = reached
iterations = 1066   tree configs = 282 (start 274, goal 8)
```

`state` is what the whole run is doing (`idle` / `planning` / `found` /
`failed`); `extend` and `connect` are the **algorithm's own three-valued
step results**, which is what makes a search in progress legible rather
than a spinning counter. Tree sizes are reported as **start** and
**goal**, never as A and B: `T_a` and `T_b` swap every iteration, so
A/B labels would make both numbers jump between meanings twice a second.
The two names match the two tree colours on the ground plane.

Slot stores, slot loads and goal changes post a timestamped
`status_message` that the HUD shows for six seconds. A keystroke that
changes the planning goal and gives no feedback is indistinguishable from
one that did nothing — which is how the `Shift+[1-9]` bug survived.

### The goal ghost

`scene.js#buildGoalGhost` makes a translucent copy of the robot at the
goal configuration, posed each frame from
`collisionForwardKinematics(robot, qmap, goal_config)` — the **pure**
traversal, reused to ask "what would this pose look like" instead of
"would this pose collide". Using the rendering FK here would overwrite
the live robot's own transforms, which is the entire reason the pure
version exists.

It **shares each link's geometry** with the live robot, so the ghost is
exactly the robot rather than an approximation, and therefore must never
dispose that geometry — `removeRobotMeshes` owns it, and the ghost is
torn down first on a robot switch. `depthWrite: false` keeps the
translucent links from occluding each other in draw order, which
otherwise reads as a solid object with holes in it.

### Ground-plane indicators

Tree vertices are drawn **flat on the ground**, not at the
configuration's own base height. Upstream puts its cube at the base
location (`add_config_origin_indicator_geom`), which is wrong for two of
this viewer's robots: the crawler's base origin sits at y = 1 and
fetch's at y = 0.1, so their search trees floated in mid-air at a height
that had nothing to do with the search. On the ground, the picture is
what it actually is — a plan view of where the base went — and two
robots of different heights are directly comparable.

Marker size scales with the world (`span * 0.008`): the previous fixed
0.06 was invisible in `world_random`'s 52-metre boundary.

---

## Joint limits: a data gap, now filled

**No robot file contained the string `limit` anywhere.** Without limits,
"sample uniformly in q" is undefined for fetch's `torso_lift_joint` and its
gripper fingers.

Every revolute and prismatic joint now carries `limit: {lower, upper}`,
taken **verbatim from the real upstream URDFs** vendored in the
kineval-stencil (`robots/fetch/fetch.urdf`, `robots/baxter/baxter.urdf.js`,
`robots/sawyer/sawyer.urdf.js`). No value is estimated, and each file's
header cites its source. The planner reads limits generically and **fails
loudly**, naming the joint, if a revolute or prismatic joint has none — it
never substitutes a default at plan time.

Two things found while doing it:

- **The plan quoted fetch's `torso_lift_joint` upper limit as 0.3861 m. The
  URDF says 0.4.** That is the Fetch's advertised travel versus what the
  description actually declares; the description wins, since it is what the
  planner reads.
- **`bellows_joint` is an independent prismatic DOF by the description.**
  The Fetch's bellows is physically a passive cover that follows the torso
  lift, but the URDF declares it a plain prismatic joint with limits
  `[0, 0.4]` and **no `<mimic>` element**. So the robot has two independent
  prismatic DOFs, not one, and the lecture's "DOFs: 13" counts it as one.
  The Rust/pubsub pass found the identical thing in `fetch.urdf` (review
  findings X-1). Same slide, two languages.

**Three of the six robots declare no joint types at all.**
`robot_crawler.js`, `robot_mr2.js` and `robot_urdf_example.js` have zero
`type:` fields, so every joint falls to upstream's "untyped means
continuous" rule and is sampled over `[0, 2π)`. For the crawler's 24 leg
joints that is physically generous — a real hip does not rotate through its
own body — but inventing joint types and limits the upstream description
does not contain would be fabricating robot data, which is worse than
planning in a space larger than necessary. Left as-is, noted in each file.

---

## The worlds

Nine ports plus five parametric generators, all data-returning factories
registered in `WORLD_FACTORIES` and selected with `?world=`. `scene.js`
remains the only file in the viewer that builds three.js objects.

### Two defects the plan expected are not there

Verified line by line; both claims are retracted in the review findings
(§K-5.1, §K-5.2).

- **`world_s.js` does not drop an obstacle.** Line 22 really does overwrite
  `robot_obstacles[10]`, but **line 34 re-adds the displaced sphere at
  index 21**. The net set is complete; there is no gap in the S-wall. What
  remains is a readability defect, and the port assigns each sphere once.
- **`world_barrier.js` has no such defect at all.** Its block comment opens
  at line **22**, *before* the line the plan identified as a live
  overwrite, so the entire second wall is commented out. Carried unchanged.

So the dropped-obstacle defect is in **zero** files, not two. The
presentation plan's appendix should not claim it, and rendering a
before/after of the S-wall is pointless.

### Defects that are real, and fixed

- **`world_s_higher_tighter.js` assigns `robot_boundary` twice** (line 5,
  then line 9 in terms of `wall2_x`). The first is dead; the port keeps the
  second.
- **`world_random.js`** — four fixes and one behaviour change, all in that
  file's header. Upstream multiplies by the boundary *minimum* instead of
  interpolating, so it never samples the region out to +2; puts sphere
  centres at `y ∈ [-0.6, 2.4)`, sinking some below the floor; draws radii
  from `Math.random()*3.0`, admitting invisible and enormous ones; and uses
  `Math.random()`, so no two page loads agree. The behaviour change, stated
  as such: every sphere is now placed at `y = radius`, resting on the
  ground plane.
- **The comment "collision only checked in x-z plane" is wrong in all
  nine.** It is true of the boundary test and false of obstacle collision,
  which is fully 3D against each link's bounding box. `world_y.js` exists
  precisely to exercise the 3D case, so the comment contradicts the file it
  sits in. Corrected in every port.

### What the parametric generators needed that was not obvious

They take `{extent, reach, height}`, and all three are necessary — which
was not the first guess.

- `extent` — the widest single **link**. The right scale for *how wide must
  a gap be for this robot to fit through*.
- `reach` — the whole **assembly's** ground-plane span from the base. The
  right scale for *how big must an enclosure be for this robot to stand
  inside*. The first version sized `world_sealed_enclosure`'s ring at
  `2.5 × extent`, and for several robots the robot was already wedged into
  its own enclosure before planning started, so the termination test could
  not run at all. The crawler is the clearest case: a 2.3-long base box
  with eight legs reaching well past it.
- `height` — how tall the robot is. **A single course of spheres is not a
  wall.** Obstacle collision is fully 3D, so a robot whose links all sit
  above a knee-high ring simply walks over it. The first version of
  `world_sealed_enclosure` did exactly that and the Phase 4 termination
  test duly reported a path out of a sealed enclosure. All four
  wall-building generators now stack courses to above the robot's own
  height. Upstream hit the same thing and hand-fixed it for one robot —
  that is what `world_s_higher.js`'s two courses at `y = 0.5` and `y = 1.5`
  and its comment "obstacles near boundaries to force arm upright" are.
  Deriving it means it stays fixed for all of them.

---

## The hardest bug: edge validation measured in the wrong units

Worth reading before touching `edgeIsValid`, because the broken version
looks right.

Edges are validated by sampling configurations along them. The first
version set the spacing as a fixed fraction of the step size **under the
weighted metric** — which is the natural thing to write, since that is
the metric everything else in the planner uses. It tunnels.

The weighted metric deliberately divides base translation by the **whole
world's span**, so that metres and radians are commensurable (see
`configurationWeights`). In a 21-metre world, a weighted edge length of
0.13 — one step — is a **base movement of a full metre**. Four samples
along it, which is what the weighted criterion asked for, step the robot
0.25 m at a time past 0.6 m-radius spheres while most of the "distance"
is being spent on joint angles. The planner declares the edge free, and
the robot walks through a wall.

**It was caught by verification, not by reading.** A plan escaped
`world_sealed_enclosure` — a world with no exit by construction — and
re-checking the offending edge at 100× the planner's resolution found
**324 colliding configurations** along a step the planner had called
free. That is exactly what that world is for.

The fix is to set the resolution in **metres**, derived from the feature
that can actually be tunnelled through: no sample may let any point on
the robot move further than a fraction of the **smallest obstacle radius
in the loaded world**. `workspaceMotionBound` supplies the motion in
metres — base translation exactly, plus every rotational DOF's angle
change times the robot's total reach as a safe lever arm. The bound is
loose in the safe direction.

Two consequences worth keeping:

- There is **no resolution that is provably safe.** This is discrete
  collision checking. The honest alternative is continuous collision
  detection, which is a different lecture. `PLANNER_MAX_EDGE_SAMPLES`
  clamps a pathological edge and **warns loudly** when it does, rather
  than quietly reintroducing the bug.
- The general lesson generalizes past this module: **a metric chosen to
  make a planner's search behave well is not automatically the right
  metric for validating what the planner found.** One is about
  exploration, the other about geometry.

### The residual, measured — and why it is not chased further

Auditing finished paths at 300 samples per edge still finds occasional
**shallow grazes**: a link corner clipping a sphere's surface for about
2–4% of one edge. That is not a leftover bug, and the numbers say so.

Probing the worst such edge: the planner's own samples were spaced
**0.084 m** of link-corner travel apart, comfortably inside its 0.2 m
resolution, so the motion bound held exactly as designed. The colliding
window was about **2% of the edge** — narrower than the sample spacing.
Any sampler with spacing *d* misses a window narrower than *d*, and there
is always a shallower graze.

And a resolution sweep confirms it is not a resolution problem. Making
the planner's spacing **five times finer** costs **ten times the collision
checks and five times the wall-clock**, and leaves the number of grazing
edges essentially unchanged:

| `PLANNER_EDGE_RESOLUTION` | collision checks | time | grazing edges |
|---|---|---|---|
| 0.40 | 22 k | 7.9 s | 6 |
| 0.25 | 57 k | 16.2 s | 5 |
| 0.15 | 94 k | 26.4 s | 3 |
| 0.08 | 214 k | 41.3 s | 3 |

So 0.4 stays, and the verification asserts the distinction that actually
matters rather than an unmeetable absolute. The real bug put **57% of an
edge** inside an obstacle and walked a robot out of a sealed enclosure; a
graze is under 4%. The check fails above 15%, and two exact checks sit
alongside it: **every path vertex is collision-free**, and **every edge
re-validates at the planner's own resolution**. Both hold everywhere.

`motion_planning/reference/rrt.js` does not have this problem — there,
configuration space *is* the workspace, so the two units coincide. That
is itself the cleanest statement of what goes wrong here.

---

## The cost model: the plan's expected inequality does not hold by count

The planner counts collision checks and nearest-neighbour comparisons per
plan and shows both in the HUD, because neither lecture deck states that
collision checking dominates sampling-based planning runtime (review
findings G-1) and the honest response is to measure it rather than repeat
it.

The plan predicted that *the collision-check counter would be strictly
larger than the nearest-neighbour counter* on cluttered worlds. **Measured,
that is not reliably true, and the prediction is about the wrong quantity.**
`nearestNeighbor` is a linear scan over a tree that grows every iteration,
so comparisons accumulate quadratically while edges — and therefore
collision checks — accumulate linearly. On a long run the comparison count
overtakes the check count.

What *is* true, and what the claim actually means, is about **time**. Phase
4 measures per-call cost directly, and the gap is enormous: a collision
check is **~965×** more expensive than a nearest-neighbour comparison — a
whole forward-kinematics traversal plus a link-by-link,
obstacle-by-obstacle sweep, versus one weighted sum over the DOFs.

Weighted by that, over the full 6 robots × 14 worlds:

| | collision checking | nearest neighbour |
|---|---|---|
| by **count** | 1.13 M | 62.8 M |
| by **time** | 31.8 s | 1.8 s |

**17:1 in favour of collision checking by time, 1:56 against it by count.**
It holds in all 45 solved cluttered runs and in all 15 that hit the
iteration cap. The verification asserts the time-weighted form and prints
both.

That inversion is a better slide than the one originally planned. A
student who instruments the obvious counter measures the opposite of the
right answer, and the reason is not subtle once stated: the two operations
differ by three orders of magnitude per call, so counting them as though
they were comparable is the mistake. It is also the cleanest motivation
for a k-d tree that this module can give — the *only* reason the
nearest-neighbour count gets that large is that `nearestNeighbor` is an
O(n) linear scan over a tree that grows every iteration.

(An earlier measurement, before angles were put on a continuous line, did
show the time-weighted ratio inverting on long capped runs. Removing the
modulo wrap made edges genuinely longer, which raised the collision-check
count enough that collision now dominates everywhere measured. Recorded
because it is a real sensitivity: the answer depends on the metric.)

---

## Changes to existing viewer files

- **`index.html`** — `WORLD_FACTORIES`, `?world=`, `?seed=`, the world
  `<select>`, the new script tags, planner params in `app.params`, HUD
  lines for the planner state and the two counters, and a `window.
  kinevalTest` hook so the Phase 4 harness drives this real page rather
  than a reimplementation of it. Everything on that hook is a thin wrapper
  over something the page already calls; there is no test-only code path.
- **`scene.js`** — obstacle spheres and the boundary rectangle
  (`buildWorldMeshes`), the search visualization (`updatePlannerMeshes`),
  and a red highlight on the colliding link. Also a bug fix found on the
  way: `buildRobotMeshes` cloned a material per link and per joint, but
  `updateRobotMeshes` replaces whole materials every frame, so every clone
  was allocated, immediately discarded and leaked — 49 orphaned materials
  per robot switch on the crawler. Materials are now shared and disposed
  once.
- **`userinput.js`** — `m` / `n` / `b`, `Shift+[1-9]`, `[1-9]`, `g`, and
  the extended `HELP_TEXT`. `n`/`b` clamp at both ends rather than wrapping;
  upstream's persist-traversal mode wraps with `% motion_plan.length`,
  which teleports the robot from goal back to start in one frame and looks
  like the plan is broken.
- **`README.md`** — the "Still not built yet" paragraph replaced. It
  described the controls as "`0`-`9` pose-setpoint slots"; the **verified**
  upstream semantics (`kineval_userinput.js:65-86`) are `Shift+[1-9]` to
  assign and `[1-9]` to load, and the rewrite adopts the verified ones.

### The default page loads in collision, on purpose

`?robot=crawler&world=basic` — the defaults — starts with a crawler leg
touching an obstacle, and the HUD says so while the link turns red. That is
not a bug: the crawler is 2.3 m long with legs reaching further, and
`world_basic`'s spheres are at ±2. It is left that way because it
demonstrates collision detection working the instant the page opens. Press
`w`/`s` to drive clear before planning.

---

## Visualization honesty

`updatePlannerMeshes` draws one marker per tree vertex **at that
configuration's base location**. A tree vertex is a point in an
n-dimensional configuration space and there is no honest way to draw one in
three dimensions, so what is on screen is a *shadow*: two vertices with
identical base poses and completely different arm configurations land on
top of each other. Upstream does the same thing
(`add_config_origin_indicator_geom`) without saying so.

---

## Snapshots and drift

`motion_planning/reference/` carries dated snapshots of `collision.js` and
`motion_plan.js`, and a byte-identical copy of `random.js`. The deck quotes
those copies, not these files, because Slidev's `<<<` importer cannot read
outside the deck's workspace root and `motion_planning/pnpm-workspace.yaml`
pins that root at `motion_planning/`.

**Edits belong here and must be re-snapshotted.** The Phase 4 harness
diffs them and fails if they drift — which it did, correctly, the first
time `robotReach` was added here after snapshotting.

Known and accepted cost of the per-module `reference/` convention: the
FK/IK modules' copies of viewer code have **already** drifted from
`kineval/`. The alternative is a build step, and this repository
deliberately has none. A `diff` is the cheap check; run it.

---

## Deployment handoff

The viewer ships via `deploy.yml`'s static `cp -r kineval site/`, so
`collision.js`, `motion_plan.js`, `random.js` and `worlds/` reach the live
site with **no workflow change**. Nothing in this plan verifies that. The
presentation plan's Phase 5 adds a live-URL smoke check of
`/kineval/kineval/?world=...` once it is touching the workflow anyway.

`.github/workflows/deploy.yml`, `motion_planning/slides/`, and flipping the
`motion_planning` row to *Built* are all out of scope here and belong to
`autorob_motion_planning_implementation_presentation_kineval`.

---

## Re-running verification

The harness lives **outside** the repository, at
`/ai/development/misc/autorob_kineval_motion_planning_verify.mjs`, because
`deploy.yml` statically copies `motion_planning` and `kineval` into the
published site and anything in either tree gets served.

```sh
source /ai/development/usr_docker/playwright.bash
node /ai/development/misc/autorob_kineval_motion_planning_verify.mjs          # full
node /ai/development/misc/autorob_kineval_motion_planning_verify.mjs --quick  # subset
node /ai/development/misc/autorob_kineval_motion_planning_verify.mjs --shots  # + screenshots
```

It drives the real pages under headless Chromium. No test body names a
robot; every property test runs over the cross product of registered robots
and worlds.

---

## Measuring anything on this machine: beware your own background jobs

Recorded because it nearly produced a false bug report during the
goal-convention work.

`world_sealed_enclosure` with mr2 runs a fixed 5000 iterations and a
fixed 71 804 collision checks every time — the work is deterministic.
Its **wall-clock time over one afternoon, on identical code**, was:

| | |
|---|---|
| 8.0 s | nothing else running |
| 11.1 s, 11.7 s | nothing else running |
| 14.9 s, 28.3 s | one background suite running |
| 62.5 s, 81.8 s, 99.8 s | two or three background jobs running |

A **12x spread**, with iteration and collision-check counts identical
throughout. Chasing the 81.8 s figure as a regression cost a profiling
pass and a checkout of the previous commit to disprove; the new code
then measured *faster* than the old once the machine was quiet.

Two consequences:

- **Wall-clock assertions must be hang detectors, not performance
  tests.** Phase 4's termination check now allows 300 s and separately
  asserts the thing that actually matters — that the planner stops *at
  the iteration cap* rather than by running out of time. That assertion
  is load-independent.
- **Profile before attributing.** The profiling pass was still worth it:
  it showed 94% of the time is inside `poseIsCollision` at ~1.3 ms per
  call, against ~40 µs in `world_barrier`. The difference is obstacle
  count — 99 stacked spheres versus 11 — and it is a genuine property of
  the cost model, not of the change under suspicion. A free configuration
  is also more expensive to verify than a colliding one, because "no
  collision" has to exhaust every link-obstacle pair while a collision
  returns early.

Related: the same failure mode is recorded for `cargo test` in the
pubsub module.
