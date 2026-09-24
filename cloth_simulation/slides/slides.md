---
theme: default
title: 2D Cloth Simulation — KinEval Lab
base: /kineval/cloth_simulation/
colorSchema: light
info: |
  ## 2D Cloth Simulation
  A KinEval lab-session walkthrough of maximal-coordinate, constraint-based physical simulation, via a 2D cloth, built for the AutoRob course (autorob.org).
class: text-center
highlighter: shiki
lineNumbers: false
drawings:
  enabled: false
transition: slide-left
mdc: true
fonts:
  sans: 'Roboto'
  mono: 'Roboto Mono'
---

# 2D <span class="accent">Cloth Simulation</span>

### Particles, rigid bodies, and constraints satisfied by relaxation

<div class="pt-8">
<div class="swatch-bar"><span class="swatch maize" /><span class="swatch blue" />Maize &amp; Blue, node by node</div>
</div>

<div class="title-footer">
<div class="nav-hint">Press &rarr; to move forward through the deck</div>
<div class="credit">AutoRob (autorob.org) &#183; Chad Jenkins (ocj@umich.edu) &#183; ocj-dev.github.io/kineval/cloth_simulation</div>
</div>

<!--
Title slide. This deck complements the AutoRob lab sections: a fully working
KinEval reference implementation of a 2D cloth simulator, walked through step
by step. Built from Thomas Jakobsen's "Advanced Character Physics"
(graphics.cs.cmu.edu/nsp/course/15-869/2006/papers/jakobsen.htm) and Adam
Brooks' Tearable Cloth (codepen.io/dissimulate/pen/nYQrNP,
github.com/dissimulate/Tearable-Cloth) -- both credited again on the next slide
and in the reference implementation's own header comment.
-->

---
layout: default
---

# A brief history of constraint-based simulation

<div class="panel text-sm">

In 1967, physicist **Loup Verlet** published an integration scheme for molecular dynamics that
stores a particle's *previous* position instead of an explicit velocity -- stable, simple, and
naturally compatible with clamping positions directly to satisfy constraints. In 2001, game
programmer **Thomas Jakobsen** (IO Interactive, makers of *Hitman: Codename 47*) published
["Advanced Character Physics"](https://graphics.cs.cmu.edu/nsp/course/15-869/2006/papers/jakobsen.htm),
showing that a cloth, rope, or ragdoll could be built entirely from **particles in maximal
coordinates** -- one state per node, no joint-angle bookkeeping -- linked by simple distance
constraints, and that those constraints could be **satisfied by iterated relaxation** rather than
solved as a stiff system of equations. The result ran fast enough for real-time games and is still
numerically stable even when badly overstretched or torn.

Around the same time, Russell Smith's [Open Dynamics Engine](https://www.ode.org/) (ODE, first
released 2001) took the opposite route to a similar goal: general rigid-body simulation via
Lagrange-multiplier constraint forces, solved as a linear complementarity problem each step,
rather than positional relaxation. ODE became a standard physics backend for early game engines
and robotics simulators alike -- the same problem, two different constraint-solving philosophies,
both still very much in use today.

This module's reference implementation extends that same approach with an alternative rigid-square
node type integrated via the **Newton-Euler equations of motion**, following Adam Brooks'
["Tearable Cloth"](https://github.com/dissimulate/Tearable-Cloth) for its concrete
particle/constraint/tear parameterization.

</div>

---
layout: default
---

# Why it still matters

<div class="panel text-sm">

Jakobsen's relaxation-based approach is the direct ancestor of **position-based dynamics (PBD)**
and **projective dynamics**, the solver family behind most real-time cloth, hair, and soft-body
simulation shipping today: Unity's cloth component, Unreal Engine's Chaos Cloth, NVIDIA
FleX/PhysX, and Blender's cloth simulator all satisfy geometric constraints by iterated correction
rather than assembling and inverting a stiffness matrix every frame. The same idea, extended to
rigid bodies (as this module's `node_type=rigid` does) and full 3D, underpins modern robotics
physics engines as well, such as [Genesis](https://github.com/Genesis-Embodied-AI/genesis-world),
a current research/robotics simulator built on exactly this family of solver.

The appeal is the same reason it worked for a 2001 real-time game: relaxation is simple to
implement, trivially parallel (every constraint's correction can be computed independently before
being applied), numerically stable under extreme deformation, and degrades gracefully -- fewer
iterations just mean a softer, springier result instead of a diverging solve.

</div>

<div class="history-note mt-2">
<b>Note on collision, ahead</b>
This deck's cloth also collides with the canvas walls and any ground plane -- handled the same way
as every other constraint, by relaxation, alongside the distance/corner constraints. General
collision detection (cloth self-collision, cloth-object contact, arbitrary obstacle geometry) is
deliberately out of scope; simulators like Genesis build on this same maximal-coordinate,
constraint-based lineage at that much larger scale.
</div>

---
layout: default
---

# Maximal vs. generalized coordinates

<div class="panel text-sm">

A robot arm is usually simulated in **generalized** coordinates -- one joint angle per degree of
freedom, guaranteed to satisfy every joint constraint automatically by construction. Cloth (and
ragdolls, and most game physics) instead uses **maximal** coordinates -- every node has its own
free position (and, here, orientation), and constraints between nodes are enforced explicitly and
only approximately, every frame, by relaxation rather than by the coordinate choice itself.

</div>

<div class="history-note mt-2">
<b>The tradeoff</b>
Maximal coordinates give up exactness -- a constraint is only ever <i>approximately</i> satisfied,
to within whatever `accuracy` relaxation passes converge to. In exchange: any constraint can be
added, removed, or broken on the fly (this is what makes tearing possible at all), and every
node's per-frame update stays simple, uniform, and independent of how many other nodes or
constraints exist -- which is exactly what makes this family of solver so common in real-time
graphics, and so easy to parallelize.
</div>

---
layout: default
---

# Reference implementation <a class="accent" href="/kineval/cloth_simulation/reference/cloth_canvas.html" target="_blank">(link)</a>

<div class="panel text-xs mt-2">

The reference implementation runs standalone, no build step, straight from `cloth_canvas.html`.
Every run is configured entirely through URL parameters, extending Tearable Cloth's own:

| Parameter | Values | Meaning |
|---|---|---|
| `accuracy` | integer (default `4`) | constraint-relaxation passes per frame |
| `gravity` | number (default `0.4`) | downward acceleration on every free node |
| `cloth_x`, `cloth_y` | integers (default `20`,`14`) | cloth grid size, in nodes |
| `spacing` | number (default `22`) | pixel distance between adjacent node centers |
| `tear` | `true`\|`false` (default `false`) | whether overstretched constraints break |
| `tear_dist` | number (default `spacing*3`) | break distance, when `tear=true` |
| `friction` | number (default `0.98`) | Verlet velocity damping |
| `bounce` | number (default `0.5`) | restitution on wall/ground collision |
| `node_type` | `particle`\|`rigid` (default `particle`) | point particles, or rigid squares linked at corners |
| `mass` | number (default `1.0`) | per-node mass -- for rigid squares this scales moment of inertia too |
| `pin_mode` | `row`\|`top_center` (default `row`) | pin the whole top row, or just its center node(s) |
| `stiffness` | 0&ndash;1 (default `0.25`) | scale on each relaxation pass's correction |
| `michigan_colors` | `true`\|`false` (default `true`) | maize/blue block-M node coloring |
| `wind` | `[x,y]` (default `[0,0]`) | base wind force -- gusts in magnitude/direction over time when non-zero |

```text
cloth_canvas.html
  ?node_type=rigid?tear=true?wind=[3,0]?stiffness=0.6
```

Drag with the mouse to pull the cloth; right-click near a node to cut it directly.

</div>

---
layout: default
---

# The simulation loop

<div class="grid grid-cols-2 gap-6 mt-2">
<div class="panel text-xs">

**Force accumulation &amp; integration**

Every node collects the forces (and, for rigid squares, torques) acting on it this frame, then
Verlet-integrates: the *difference* between the current and previous position stands in for
velocity, so no explicit velocity is ever stored.

**Constraint relaxation**

Every constraint -- inter-node distance/corner constraints, and wall/ground collisions alike -- is
satisfied by a small positional correction, `accuracy` times per frame. No linear system is ever
assembled; this is Gauss-Seidel-style iterated relaxation, exactly as Jakobsen describes.

</div>
<div class="panel">

**Simulation pseudocode**

<PseudocodePanel :active-line="-1" :lines="[
  'build the cloth: a grid of nodes (particles or rigid squares), and',
  'the constraints between adjacent nodes',
  'every frame:',
  '    for each node',
  '        accumulate forces (gravity, wind, mouse) -- and torque, for rigid nodes',
  '        Verlet-integrate the node\'s position -- and orientation, for rigid nodes',
  '    repeat `accuracy` times:',
  '        for each constraint, satisfy it by relaxation',
  '        for each node, satisfy collisions (walls, and any ground plane)',
  '    draw every node and the constraints between them',
]" />

</div>
</div>

---
layout: default
---

# Component breakdown

<div class="panel text-sm mt-4">

The next several slides walk the reference implementation
(`kineval/cloth_simulation/reference/{physics,cloth}.js`) one interactive example at a time, from
a single particle up to the full grid:

1. **Single particle** -- Newton's Second Law, force accumulation, Verlet integration
2. **Single rigid square** -- the same, plus Euler's rotation equation for orientation
3. **Single constraint** (particle) -- accumulate/integrate/relax, one node anchored
4. **Single constraint** (rigid) -- the same, with 2 corner constraints, position+orientation anchored
5. **A single constraint instead of two** -- one corner-to-corner "rope" constraint, free to rotate
6. **Single constraint with collision** -- a 4th phase, against a wall/ground boundary
7. **Blob simulation** -- 5 unanchored particles, full connectivity, ground plane, mouse dragging
8. **3&times;3 cloth grid** -- particle and rigid variants, side by side, mouse dragging

Every code snippet is imported directly from the shipped reference files -- not retyped -- with
line numbers matching those files exactly. An appendix at the end of this deck covers the
supporting code (rendering, mouse interaction, URL parsing) none of these slides touch directly.

</div>

---
layout: default
---

# Single particle: Newton's Second Law

<div class="content-body">
<SingleParticlePanel />
</div>

<!--
F = ma: toggle gravity and wind independently, and drag the joystick to set
wind's magnitude and direction. The red arrow is the resultant of whichever
forces are active -- what verletIntegrateParticle() actually applies.
-->

---
layout: default
---

# Accumulating force, then integrating

<div class="panel text-xs mt-2">
Every node collects its forces into <code>force_x</code>/<code>force_y</code> each frame (Newton's
Second Law: <b>F = ma</b>, and since this module uses unit mass throughout, force and acceleration
are the same number), then Verlet-integrates: the previous-position-difference stands in for
velocity, damped by <code>friction</code>, plus the new acceleration.
</div>

<<< ../reference/physics.js#accumulate-forces {*}{lines:true,startLine:72,maxHeight:'220px'}
<<< ../reference/physics.js#verlet-integrate-particle {*}{lines:true,startLine:93,maxHeight:'220px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: lines 4-5 (accumulate forces, then Verlet-integrate).</div>

---
layout: default
---

# Single rigid square: adding rotation

<div class="content-body">
<SingleRigidBodyPanel />
</div>

<!--
Gravity acts at the center of mass (no torque); wind is modeled as striking
the top-right corner, so toggling it on visibly spins the square as well as
translating it -- exactly what accumulateTorque()'s r x F cross product
predicts.
-->

---
layout: default
---

# Euler's rotation equation, integrated the Verlet way

<div class="panel text-xs mt-2">
In 3D, Euler's rotation equation in vector form is <b>I(d&omega;/dt) + &omega; &times; (I&omega;) = &tau;</b>.
In 2D the gyroscopic term <b>&omega; &times; (I&omega;)</b> vanishes (both <b>&omega;</b> and the
inertia axis are perpendicular to the plane of motion), leaving the scalar special case
<b>I(d&omega;/dt) = &tau;</b> used below -- integrated in orientation exactly the same Verlet way
position is, with <code>theta</code>/<code>ptheta</code> standing in for <code>x</code>/<code>px</code>.
</div>

<<< ../reference/physics.js#accumulate-torque {*}{lines:true,startLine:201,maxHeight:'200px'}
<<< ../reference/physics.js#verlet-integrate-rigid {*}{lines:true,startLine:215,maxHeight:'220px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: lines 4-5 (accumulate forces/torque, then integrate) -- see the next slide for the matrix form behind both.</div>

---
layout: default
---

# Newton-Euler in matrix form

<div class="panel text-sm">

Stacking the translational and rotational equations from the previous slide into one generalized
coordinate vector **q = [x, y, &theta;]** gives exactly the general rigid-body dynamics equation
used throughout robotics, **M(q) q&#776; = &tau;**:

$$
\underbrace{\begin{bmatrix} m & 0 & 0 \\ 0 & m & 0 \\ 0 & 0 & I \end{bmatrix}}_{M(q)}
\begin{bmatrix} \ddot{x} \\ \ddot{y} \\ \ddot{\theta} \end{bmatrix}
\;=\;
\begin{bmatrix} F_x \\ F_y \\ \tau \end{bmatrix}
$$

</div>

<div class="history-note mt-2">
<b>Why M is constant here: the center of mass</b>
For a general multi-body system, <b>M(q)</b> depends on configuration and couples every body's
coordinates together through the constraint Jacobian. For a single free rigid body, though, <b>M</b>
collapses to this simple, <i>constant</i>, block-diagonal form -- no coupling between translation
and rotation at all -- specifically because <b>x, y</b> track the body's <b>center of mass</b>.
Referencing any other point on the body would introduce off-diagonal mass-moment terms coupling
force to angular acceleration and torque to linear acceleration. This is exactly why
<code>RigidSquare</code>'s <code>x, y</code> is its center, not a corner, and why <code>I</code> is
computed about that same center on the previous slide. Constraint relaxation is what
re-introduces coupling <i>between</i> bodies -- not a globally assembled <b>M(q)</b>, but repeated
local corrections applied to each body's own simple, center-of-mass-referenced equation of motion.
</div>

---
layout: default
---

# Single constraint: satisfying it by relaxation

<div class="content-body">
<ConstraintPanel :collision="false" />
</div>

<!--
p1 (the blue square) is pinned; p2 (the maize circle) starts stretched past
the rest length and hangs under gravity. Step through: Accumulate and
Integrate both show the force vector that's driving the motion; Relax
switches to showing the *correction* vector -- the whole point of this slide,
per the deck's convention that relaxation gets its own vector, not a force.
-->

---
layout: default
---

# Line by line: one distance constraint

<div class="panel text-xs mt-2">
One relaxation pass: measure how far the two particles are from <code>rest_length</code>, then
move each halfway to close that gap, scaled by <code>stiffness</code> (a soft-constraint knob the
original Tearable Cloth doesn't have). Iterating this over every constraint, <code>accuracy</code>
times per frame, <i>is</i> "solving multiple concurrent constraints by relaxation" -- no linear
system is ever assembled.
</div>

<<< ../reference/physics.js#satisfy-constraint-particle {*}{lines:true,startLine:128,maxHeight:'420px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 7 (satisfy each constraint by relaxation).</div>

---
layout: default
---

# Single constraint, rigid squares

<div class="content-body">
<ConstraintRigidPanel />
</div>

<!--
bodyA (blue, top-left) is pinned in BOTH position and orientation. bodyB
starts displaced and rotated away from the shared-edge rest configuration;
watch Relax pull it back into alignment via its two corner constraints (one
per corner along the shared edge) -- both correction vectors are shown at once.
-->

---
layout: default
---

# Two corners, one shared edge

<div class="panel text-xs mt-2">
The rigid-body analog of the particle constraint: two corners (one on each of two adjacent
squares) relax toward <i>coincidence</i> instead of a fixed spacing. The correction at a corner is
split into a translation and a rotation using the standard generalized-mass point-constraint solve
from position-based rigid body dynamics -- a documented extension with no direct precedent in
either source material, since Jakobsen's own paper represents rigid bodies with extra "shadow"
particles instead.
</div>

<<< ../reference/physics.js#apply-corner-correction {*}{lines:true,startLine:255,maxHeight:'420px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 7, the helper every rigid-body relaxation call below uses.</div>

---
layout: default
---

# ...and the 2 constraints it takes to use it

<div class="panel text-xs mt-2">
Two of the corner-correction calls above -- one per corner pair along the shared edge -- is what
keeps neighboring squares from hinging freely about a single point, the same way <code>build-cloth</code>
wires up every adjacent pair in the full grid.
</div>

<<< ../reference/physics.js#satisfy-constraint-rigid {*}{lines:true,startLine:295,maxHeight:'380px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 7 (satisfy each constraint by relaxation).</div>

---
layout: default
---

# One constraint instead of two: a rope, not a weld

<div class="content-body">
<ConstraintRigidRopePanel />
</div>

<!--
Same satisfyConstraintRigid() shown on the previous slide, but called once
(not twice) between the two squares' bottom-right corners, with a nonzero
rest length (the square's side length) instead of 0. With only one
constraint, bodyB is free to swing and rotate about that single point --
contrast with the 2-corner "weld" a couple slides back, which fully locks
relative orientation between adjacent squares. Gravity acts on bodyB, which
can be picked up and dragged from anywhere within its rotated bounds, not
just a point near its center.
-->

---
layout: default
---

# Single constraint, with collision

<div class="content-body">
<ConstraintPanel :collision="true" />
</div>

<!--
Same scene as the particle constraint slide, but positioned so the hanging
node's rest swing carries it into the floor. A 4th phase appears after Relax:
Collision detect & respond, shown with its own (purple) response vector,
clamping the position and reflecting the Verlet "previous position" scaled by
`bounce` -- generalizing Tearable Cloth's own wall-bounce formula to any
axis-aligned boundary.
-->

---
layout: default
---

# Collision, treated as just another constraint

<div class="panel text-xs mt-2">
Following Jakobsen's framing, collision against a canvas wall or ground plane is satisfied every
relaxation pass, exactly alongside the distance/corner constraints -- not as a separate physics
system, and not only once per frame -- so a fast-moving node can't tunnel through a boundary
between one <code>accuracy</code> iteration and the next.
</div>

<<< ../reference/physics.js#satisfy-collisions-particle {*}{lines:true,startLine:331,maxHeight:'420px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 8 (satisfy collisions against walls/ground).</div>

---
layout: default
---

# Blob simulation: five particles, full connectivity

<div class="content-body">
<BlobPanel />
</div>

<!--
Five unanchored particles at the corners of a pentagon, every pair connected
(5 choose 2 = 10 constraints) -- nothing is pinned, so the whole blob falls,
jostles into a stable pentagon under mutual constraint tension, and settles
on the ground plane / canvas walls. This is the one place in this deck where
"collision" means a genuinely 2D response (not just a 1D swing hitting a
floor) at real multi-constraint scale.
-->

---
layout: default
---

# 3&times;3 cloth grid: particle and rigid

<div class="content-body">
<ClothGridPanel />
</div>

<!--
The smallest recognizable "cloth": a 3x3 grid with its top row pinned.
Switch the Node type picker between Particle and Rigid square to compare --
same topology, same top-row pinning, different per-node physics.
-->

---
layout: default
---

# Building the grid

<div class="panel text-xs mt-2">
Both node types share one construction: walk the grid, create a node per cell (pinning the top
row -- for rigid squares this fixes position <i>and</i> orientation, since every integrate/relax
function above skips pinned bodies entirely), and wire a horizontal and vertical constraint to
each earlier neighbor -- one distance constraint for particles, or two corner constraints (one per
shared-edge corner pair) for rigid squares.
</div>

<<< ../reference/cloth.js#build-cloth {*}{lines:true,startLine:106,maxHeight:'420px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: lines 0-1 (build the grid and its constraints, once, before the per-frame loop begins).</div>

---
layout: default
---

# Putting it together: `simulateStep()`

<div class="panel text-xs mt-2">
Every function shown so far is called from one place, once per rendered frame, in exactly
Jakobsen's order: accumulate forces, Verlet-integrate every node once, then relax every
constraint -- distance/corner constraints and collisions alike -- <code>accuracy</code> times, so the
whole concurrent system of constraints converges toward mutual satisfaction.
</div>

<<< ../reference/physics.js#simulate-step {*}{lines:true,startLine:419,maxHeight:'420px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: lines 2-9, the entire per-frame loop.</div>

---
layout: default
---

# Test cases

<div class="panel text-xs mt-2">

| Case | What it tests | Run it |
|---|---|---|
| Low stiffness (default) | Maize/blue block-M coloring, particle nodes, floppy/slow-converging constraints | <a href="/kineval/cloth_simulation/reference/cloth_canvas.html" target="_blank">&#9654;</a> |
| Higher stiffness | `stiffness=1.0` -- a taut, less stretchy drape | <a href="/kineval/cloth_simulation/reference/cloth_canvas.html?stiffness=1.0" target="_blank">&#9654;</a> |
| Tearable | `tear=true?michigan_colors=false` -- yank the cloth apart by hand or past `tear_dist`, plain coloring | <a href="/kineval/cloth_simulation/reference/cloth_canvas.html?tear=true?michigan_colors=false" target="_blank">&#9654;</a> |
| Rigid-body grid | `node_type=rigid?mass=0.5` -- squares linked at their corners, half mass | <a href="/kineval/cloth_simulation/reference/cloth_canvas.html?node_type=rigid?mass=0.5" target="_blank">&#9654;</a> |
| Windy flag | `wind=[2,0]?cloth_x=10?cloth_y=16` -- gusting speed/direction, with a wind-vector indicator | <a href="/kineval/cloth_simulation/reference/cloth_canvas.html?wind=[2,0]?cloth_x=10?cloth_y=16" target="_blank">&#9654;</a> |

</div>

---
layout: default
class: text-center
---

<div class="image-slide">
<img src="/images/go-blue-robot.jpg" class="hero-image" alt="An MBot omni-wheel robot with a flexible chainmail-like panel woven in a maize and blue block-M pattern" />
<div class="image-caption">Go Blue!</div>
</div>

<div class="credit" style="position:absolute; left:0; right:0; bottom:0.4em;">AutoRob (autorob.org) &#183; Chad Jenkins (ocj@umich.edu) &#183; ocj-dev.github.io/kineval/cloth_simulation</div>

<!--
Concluding slide, before the appendix.
-->

---
layout: default
---

# Appendix: the rest of the reference implementation

<div class="panel text-sm mt-4">

The slides above cover every essential piece of the simulator, but the reference implementation
(`physics.js`, `cloth.js`, `infrastructure.js`, `draw.js`, `cloth_canvas.html`) includes a handful
of supporting pieces none of them touch directly. For completeness, the next few slides cover:

1. **Rigid-body collision** (`physics.js`) -- the corner-by-corner companion to the particle
   collision shown earlier
2. **Michigan-mask node coloring** (`cloth.js`) -- the analytic block-M lookup
3. **Mouse drag &amp; cut** (`cloth.js`, `infrastructure.js`) -- dragging and cutting the cloth by hand
4. **Rendering** (`draw.js`) -- how each node type is actually drawn to the canvas
5. **Gusting wind** (`physics.js`, `draw.js`) -- the time-varying wind vector and its on-canvas indicator
6. **The animation loop** (`draw.js`) -- what calls `simulateStep()` once per frame
7. **A responsive canvas** (`infrastructure.js`) -- sizing the canvas to fill its container
8. **Reading the URL parameters** (`cloth_canvas.html`) -- how every parameter on the earlier table
   gets parsed into the globals every function above reads

</div>

---
layout: default
---

# Appendix: rigid-body collision

<div class="panel text-xs mt-2">
Each of a rigid square's 4 corners is checked individually against the walls/ground; a
penetrating corner's needed correction folds back onto the body's center position and orientation
with the same <code>applyCornerCorrection()</code> split used for corner-to-corner constraints,
treating the boundary itself as if it were "the other body."
</div>

<<< ../reference/physics.js#satisfy-collisions-rigid {*}{lines:true,startLine:370,maxHeight:'420px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 8 (satisfy collisions against walls/ground).</div>

---
layout: default
---

# Appendix: Michigan-mask node coloring

<div class="panel text-xs mt-2">
No external logo asset -- a stylized block "M" is generated analytically over the unit square
(two solid legs, plus two diagonal strokes converging toward the horizontal center), then sampled
per node by its normalized grid position, keeping the module fully self-contained.
</div>

<<< ../reference/cloth.js#michigan-mask {*}{lines:true,startLine:36,maxHeight:'420px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: not part of the per-frame loop -- runs once, at grid-build time (line 0).</div>

---
layout: default
---

# Appendix: mouse drag &amp; cut

<div class="panel text-xs mt-2">
Carried over from Tearable Cloth: while the primary button is held, nodes within
<code>mouse.influence</code> are dragged along with the cursor (by displacing the node's
<i>previous</i> position, so Verlet integration reads the drag back as velocity); nodes within the
smaller <code>mouse.cut</code> radius instead have their constraints deactivated -- reusing the
same <code>active</code> flag tearing uses, so a hand-cut and an overstretch-torn constraint are
indistinguishable to the renderer.
</div>

<<< ../reference/cloth.js#mouse-interaction {*}{lines:true,startLine:165,maxHeight:'340px'}
<<< ../reference/infrastructure.js#mouse-handlers {*}{lines:true,startLine:82,maxHeight:'260px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: the drag itself feeds into line 4 (a node's position/force each frame); the DOM event wiring here is setup, outside the loop.</div>

---
layout: default
---

# Appendix: rendering

<div class="panel text-xs mt-2">
Particle mode draws the active constraint lines, then a colored dot per node; rigid mode fills
each square's 4 world-space corners as a polygon. Both read each node's <code>color</code>,
assigned once at grid-build time from the Michigan-mask lookup.
</div>

<<< ../reference/draw.js#draw-particle-cloth {*}{lines:true,startLine:79,maxHeight:'200px'}
<<< ../reference/draw.js#draw-rigid-cloth {*}{lines:true,startLine:103,maxHeight:'200px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 9 (draw every node and the constraints between them).</div>

---
layout: default
---

# Appendix: gusting wind

<div class="panel text-xs mt-2">
A "windy flag" should feel like weather, not a fan: <code>currentWind()</code> takes the configured
<code>wind</code> parameter as a base magnitude/direction and gusts it over time (two independent
sine waves, one for magnitude, one for direction). <code>drawArrow()</code> is the same vector-line
primitive used throughout this deck's interactive panels, drawn here with plain canvas path calls
(no images); <code>drawWindIndicator()</code> uses it to show the current gust every frame.
</div>

<<< ../reference/physics.js#wind-time-varying {*}{lines:true,startLine:52,maxHeight:'220px'}
<<< ../reference/draw.js#draw-arrow {*}{lines:true,startLine:35,maxHeight:'220px'}
<<< ../reference/draw.js#draw-wind-indicator {*}{lines:true,startLine:67,maxHeight:'140px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: <code>currentWind()</code> feeds line 4 (wind is a force); drawing the indicator is part of line 9.</div>

---
layout: default
---

# Appendix: the animation loop

<div class="panel text-xs mt-2">
Called once per rendered frame via <code>requestAnimationFrame</code>: advance the simulation by
one fixed unit timestep, then redraw. This is the dispatch that calls <code>simulateStep()</code>.
</div>

<<< ../reference/draw.js#appendix-animate {*}{lines:true,startLine:144,maxHeight:'220px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 2 (<code>every frame:</code>) -- this is what actually triggers it.</div>

---
layout: default
---

# Appendix: a responsive canvas

<div class="panel text-xs mt-2">
The canvas element is laid out by CSS to fill whatever space it's given -- the page, or an
embedding iframe -- and its drawing-buffer resolution is read from, and kept in sync with, its
actual rendered size, rather than a fixed <code>width</code>/<code>height</code> attribute.
</div>

<<< ../reference/infrastructure.js#resize-canvas {*}{lines:true,startLine:57,maxHeight:'200px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: not part of the per-frame loop -- setup code, run at load and on resize.</div>

---
layout: default
---

# Appendix: reading the URL parameters

<div class="panel text-xs mt-2">
<code>cloth_canvas.html</code>'s inline script sets defaults, then overwrites them from the page's
own URL -- this is what makes every example in this deck (and the <code>(link)</code> on the
"Reference implementation" slide) just a plain hyperlink, no server or build step involved.
</div>

<<< ../reference/cloth_canvas.html#appendix-url-params {*}{lines:true,startLine:89,maxHeight:'420px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: not part of the per-frame loop -- setup code, run once before line 0.</div>
