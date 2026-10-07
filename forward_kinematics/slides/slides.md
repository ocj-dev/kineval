---
theme: default
title: Forward Kinematics and Quaternions — KinEval Lab
base: /kineval/forward_kinematics/
colorSchema: light
info: |
  ## Forward Kinematics and Quaternions
  A KinEval lab-session walkthrough of URDF, mesh geometry formats, matrix-stack forward kinematics, and quaternion rotation, built for the AutoRob course (autorob.org).
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

# Forward Kinematics <span class="accent">and Quaternions</span>

### URDF, mesh geometry, the matrix stack, and quaternion rotation

<div class="pt-8">
<div class="swatch-bar"><span class="swatch amber" /><span class="swatch indigo" />Fixed origin offset &amp; variable joint motion</div>
</div>

<div class="title-footer">
<div class="nav-hint">Press &rarr; to move forward through the deck</div>
<div class="credit">AutoRob (autorob.org) &#183; Chad Jenkins (ocj@umich.edu) &#183; ocj-dev.github.io/kineval/forward_kinematics</div>
</div>

<!--
Title slide. This deck complements the AutoRob lab sections: a fully working
KinEval reference implementation of forward kinematics -- composing a robot's
kinematic tree through a matrix stack, with quaternions driving each joint's
own rotation -- walked through step by step. Built from the AutoRob "Matrix
Stack" and "Quaternions" lectures, and completing the upstream kineval-stencil
(github.com/autorob/kineval-stencil) forward-kinematics, quaternion, and
robot-init modules.
-->

---
layout: default
---

# A brief history of robot forward kinematics

<div class="panel text-sm">

Unlike this project's dynamics and path-planning decks, the AutoRob matrix-stack lecture doesn't
open with a named historical figure for *this* technique -- the matrix stack itself is standard
computer-graphics machinery (the same push/pop composition OpenGL's old fixed-function pipeline
used for scene graphs), applied here to a robot's kinematic tree instead of a 3D scene. What the
lecture does credit by name is the **Denavit-Hartenberg (D-H) convention** -- **Jacques Denavit**
and **Richard Hartenberg**, 1955 -- which the lecture calls the *traditional* way to define a
robot's kinematics, with **URDF** the one the field has moved to "in recent years", because
URDF's link/joint tree is, in the lecture's own words, "amenable to matrix stack recursion."
The frame-labeling notation used
throughout both lectures is explicitly attributed to the **Spong textbook** (*Robot Modeling and
Control*), kept "consistent" with it on purpose so the two notations never clash.

</div>

<div class="history-note">
<b>Why this matters here</b>
Forward kinematics has no single inventor the way Newton/Euler or Hamilton get credited elsewhere
in this course -- it's a convention (how do we *label and compose* the transforms?), not a
discovery. This module uses one popular convention (matrix stack + quaternions); the comparison
slide near the end weighs it against D-H, dual quaternions, and products of exponentials.
</div>

---
layout: default
---

# A brief history of axis-angle rotation: Rodrigues

<div class="panel text-sm">

Before quaternions, there was a cleaner way to rotate a vector by an angle about an arbitrary axis
than three stacked Euler-angle rotations: the **Rodrigues rotation formula**, named for
**Benjamin Olinde Rodrigues (1795-1851)**. Split any vector **b** into components parallel and
perpendicular to the rotation axis **a**, rotate only the perpendicular part, and recombine:

<div class="panel" style="margin-top:0.6em">

**b&prime; = (1 &minus; cos&theta;)(a&middot;b)a + b cos&theta; + (a&times;b) sin&theta;**

</div>

This is the same "axis + angle" idea `quaternion_from_axisangle` in this module's reference
implementation is built from -- Rodrigues' formula can be written directly as a rotation matrix (via
a skew-symmetric matrix of the axis and an outer-product term), and a quaternion built from an
axis and an angle is, underneath, encoding exactly this formula in four numbers instead of nine.

</div>

---
layout: default
---

# A brief history of quaternions: Hamilton

<div class="split-panel">
<div>

<div class="panel text-sm">

**Sir William Rowan Hamilton (1805-1865)** discovered quaternion multiplication in a flash of
insight on 16 October 1843, while walking along the Royal Canal in Dublin -- famously scratching
the fundamental formula into the stone of Broom (Brougham) Bridge on the spot, rather than lose it
before he got home. That plaque is still there today.

A quaternion extends the complex numbers (**a + bi**, one imaginary unit) to **four** terms --
**q = a + bi + cj + dk** -- with **i&sup2; = j&sup2; = k&sup2; = ijk = &minus;1**. The real part
**a** and the three imaginary parts **(b,c,d)** together encode a 3D rotation (angle and axis) in
one algebraic object that composes by multiplication, the same way a unit complex number composes
2D rotations -- this deck's own 2D-rotation panel is deliberately the one-dimension-down version of
the same idea.

</div>
</div>
<div>

<div class="history-note" style="height:100%">
<b>The plaque</b>

<div style="font-style:italic; margin:0.4em 0;">
"Here as he walked by on the 16th of October 1843 Sir William Rowan Hamilton in a flash of genius
discovered the fundamental formula for quaternion multiplication<br>
i&sup2; = j&sup2; = k&sup2; = ijk = &minus;1<br>
&amp; cut it on a stone of this bridge."
</div>

&mdash; Broom (Brougham) Bridge, Dublin

</div>
</div>
</div>

---
layout: default
---

# Why not just Euler angles? Gimbal lock

<div class="panel text-sm">

Three stacked axis rotations -- **R = R<sub>z</sub>(&theta;<sub>z</sub>) R<sub>y</sub>(&theta;<sub>y</sub>)
R<sub>x</sub>(&theta;<sub>x</sub>)**, AutoRob's own XYZ order -- are the most direct way to build a
3D rotation, and the one this module's `matrix_from_rpy` uses for every *fixed* `<origin rpy="...">`
offset. But driving a robot joint's own *variable* rotation this way has a real failure mode:
**gimbal lock**, where two of the three axes rotate into alignment and a full 3 degrees of freedom
collapses to 2. The lecture demonstrates it two ways: symbolically, setting the middle angle
&theta;<sub>y</sub> = 90&deg; collapses the stacked matrix to a single effective axis; and
physically, with a "shoulder" example -- applying **R<sub>z</sub>(90&deg;) R<sub>y</sub>(90&deg;)
R<sub>x</sub>(90&deg;)** to an arm leaves its z-axis realigned with its *original* x-axis, so one
whole axis of further rotation has silently vanished.

This is exactly why `traverseFKJoint` never stacks Euler-angle rotations for a joint's own motion --
every joint gets a single axis-angle rotation, converted to a quaternion and then a matrix, which
has no equivalent singularity. (Fixed `<origin rpy="...">` offsets are safe to leave as Euler
angles precisely because they're author-time constants, never driven through a singularity at
runtime.)

</div>

---
layout: default
---

# The kinematic tree, revisited

<div class="panel text-sm">

This module picks up where [simulation](/kineval/simulation/)'s "maximal vs. generalized
coordinates" slide left off: a robot is a **tree** of rigid links connected by joints, rooted at a
**base** link. Forward kinematics answers one question for every link in that tree -- *given the
robot's current joint angles, where is this link in the world?* -- by walking the tree once, root
to leaf, composing one transform per edge along the way.

Two things compose at every joint: a **fixed** offset (where the joint sits relative to its parent
link -- authored once, in the robot description, and never changes) and a **variable** motion (how
far the joint has actually rotated or translated *right now*). Keeping those two cleanly separate
-- one matrix from `matrix_from_origin`, one quaternion-built matrix from the joint's own axis and
angle -- is the organizing idea behind every piece of code in this module.

</div>

---
layout: default
---

# URDF: `<link>`, `<joint>`, and what hangs off each link

<div class="panel text-sm" style="font-size:0.78em">

A real URDF (Unified Robot Description Format) file is XML. Every `<link>` can carry up to three
independent descriptions of its physical extent: **`<visual>`** (what's rendered) and
**`<collision>`** (what a planner checks against) each hold their own `<geometry>`, specifiable
separately because a robot's visual mesh is usually far more detailed than collision-checking
wants; **`<inertial>`** holds no geometry at all -- just `<mass>` and the `<inertia>` tensor,
which a modeller typically *derives* from a geometry rather than storing one. Every `<joint>`
names its parent/child link, a **fixed** `<origin>` offset, and (for a movable joint) a
rotation/translation **`<axis>`**; `revolute` joints additionally require a **`<limit>`**.

</div>

```xml {lines:true}
<link name="torso">
  <visual>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <geometry><mesh filename="package://baxter_description/meshes/torso/base_link.DAE"/></geometry>
    <material name="darkgray"><color rgba=".2 .2 .2 1"/></material>
  </visual>
  <collision>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <geometry><mesh filename="package://baxter_description/meshes/torso/base_link_collision.DAE"/></geometry>
  </collision>
  <inertial>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <mass value="35.336455"/>
    <inertia ixx="1.849155" ixy="-0.000354" ixz="-0.154188" iyy="1.662671" iyz="0.003292" izz="0.802239"/>
  </inertial>
</link>
```

<div class="credit">excerpted from the upstream kineval-stencil's robots/baxter/baxter.urdf (github.com/autorob/kineval-stencil)</div>

---
layout: default
---

# URDF `<visual>` &harr; KinEval JSON

<div class="panel text-sm">

This module's robot descriptions (`mobile_arm.urdf.js`, `urdf_example.js`) are KinEval's own plain-JS
convenience representation of a URDF **`<visual>`** tree -- the same `<link>`/`<joint>` structure,
the same fixed `<origin xyz rpy>` offsets and `<axis>`, just as object literals instead of XML so
the reference implementation needs no XML parser. `<inertial>` and `<collision>` hang off the
*same* link/joint nodes in a real URDF -- "the same tree" -- but this module only ever builds the
visual tree; KinEval represents the other two elsewhere: `<inertial>` (mass/length) already drives
the [simulation](/kineval/simulation/) module's Pendularm dynamics, and `<collision>` is this
module's own `robot.collision` boolean placeholder, with real collision *geometry* left to the
future `motion_planning/` (RRT) module.

</div>

<div class="split-panel" style="margin-top:0.5em">
<div>

```xml
<joint name="joint1" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link1"/>
  <origin xyz="0 0.25 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <!-- required for revolute -->
  <limit lower="-3.14" upper="3.14"
         effort="100" velocity="10"/>
</joint>
```

</div>
<div>

```js
joint1: {
  parent: 'base_link', child: 'arm_link1',
  type: 'revolute',
  origin: { xyz: [0, 0.25, 0], rpy: [0, 0, 0] },
  axis: [0, 1, 0], angle: 0
}
```

</div>
</div>

---
layout: default
---

# Geometry: vertices, edges, faces

<div class="panel text-sm">

Whatever file format a link's `<visual>`/`<collision>` mesh is stored in, it's describing the same
three things: a list of **vertices** (3D points), **edges** connecting pairs of them, and **faces**
-- almost always triangles -- bounding the surface those edges enclose. A format's job is just
*how* it writes that list down: as a flat text file with little structure (STL), as a general-purpose
scene-description XML document (Collada), or as a lightweight line-oriented text format (OBJ) -- the
next three slides show each, rendered live.

</div>

---
layout: default
---

# Geometry format: STL

<div class="content-body">
<div class="split-panel">
<div>

<div class="panel text-sm">

**STL** ("stereolithography") stores a mesh as nothing but a flat list of triangles -- each with its
own facet normal and three vertices, **no shared-vertex indexing, no color, no hierarchy**. That
makes it trivial to parse and universally supported (every 3D printer and CAD tool reads it), but
wasteful (every triangle repeats its vertices in full) and visually bare -- no material or texture
information at all. Common use: **3D-printing and CAD export**, rarely a final rendered asset.

</div>
</div>
<div>
<div class="side-image" style="background:#f5f5f5">
<MeshFormatExamplePanel format="stl" mesh-url="/meshes/sample_tetrahedron.stl" :color="0xd98236" />
</div>
<div class="side-caption">live three.js render, via STLLoader -- public/meshes/sample_tetrahedron.stl</div>
</div>
</div>
</div>

---
layout: default
---

# Geometry format: Collada (.dae)

<div class="content-body">
<div class="split-panel">
<div>

<div class="panel text-sm">

**Collada** is an XML scene-interchange format -- a mesh's vertices/faces live inside a
`<library_geometries>` block, but a .dae file can also carry materials, a full **node hierarchy**,
skeletal rigging, and animation, which is why several robots in the upstream kineval-stencil
(baxter, fetch, sawyer) ship their visual meshes as .dae. The tradeoff for that richness is a much
more verbose, harder-to-hand-author file than STL or OBJ. Common use: **richly-textured, rigged, or
multi-part assets** carried between different authoring/rendering tools.

</div>
</div>
<div>
<div class="side-image" style="background:#f5f5f5">
<MeshFormatExamplePanel format="collada" mesh-url="/meshes/sample_tetrahedron.dae" :color="0x27966b" />
</div>
<div class="side-caption">live three.js render, via ColladaLoader -- public/meshes/sample_tetrahedron.dae</div>
</div>
</div>
</div>

---
layout: default
---

# Geometry format: OBJ

<div class="content-body">
<div class="split-panel">
<div>

<div class="panel text-sm">

**OBJ** sits between the two: a plain-text, line-oriented format (`v x y z` for each vertex, `f ...`
for each face, referencing vertices *by index* rather than repeating their coordinates) -- more
compact than STL for a mesh with shared vertices, far more human-readable than Collada's XML, and
almost universally supported. It has no hierarchy or animation, and materials live in a separate
companion `.mtl` file rather than inline. Common use: a **lowest-common-denominator exchange
format** between modeling tools and renderers.

</div>
</div>
<div>
<div class="side-image" style="background:#f5f5f5">
<MeshFormatExamplePanel format="obj" mesh-url="/meshes/sample_cube.obj" :color="0x3b6ea5" />
</div>
<div class="side-caption">live three.js render, via OBJLoader -- public/meshes/sample_cube.obj</div>
</div>
</div>
</div>

---
layout: default
---

# Reference implementation <a class="accent" href="/kineval/forward_kinematics/reference/forward_kinematics.html" target="_blank">(link)</a>

<div class="panel text-sm">

`buildFKTransforms(robot)` walks the robot's kinematic tree depth-first, root to leaf, composing a
**matrix stack** as it goes -- one matrix pushed per joint, popped on the way back up:

</div>

<<< ../reference/kinematics.js#forward-kinematics {js}{lines:true,startLine:117,maxHeight:'420px'}

---
layout: default
---

# Component breakdown

<div class="panel">

1. **Matrix stack & transforms** -- `matrix_multiply`, `generate_translation_matrix`,
   `generate_rotation_matrix_{X,Y,Z}`, `matrix_from_rpy`, `matrix_from_origin`
2. **DFS traversal of the kinematic tree** -- `traverseFKBase`, `traverseFKLink`, `traverseFKJoint`
3. **Quaternions for a joint's own rotation** -- `quaternion_from_axisangle`,
   `quaternion_normalize`, `quaternion_multiply`, `quaternion_to_rotation_matrix`
4. **Interactive**: 2D rotation by a complex number, 3D rotation by a quaternion
5. **A new robot**: `mobile_arm.urdf.js`
6. **Comparison**: D-H convention, dual quaternions, products of exponentials

</div>

---
layout: default
---

# 1. Matrix stack: fixed transforms

<div class="panel text-sm">

Every fixed offset -- a joint's `<origin xyz rpy>`, authored once and never touched again at
runtime -- becomes a translation matrix and three axis-rotation matrices, composed in AutoRob's
own XYZ order: **R = R<sub>z</sub>(yaw) R<sub>y</sub>(pitch) R<sub>x</sub>(roll)**.

</div>

<<< ../reference/matrix.js#generate-translation-matrix {js}{lines:true,startLine:241,maxHeight:'140px'}
<<< ../reference/kinematics.js#matrix-from-rpy {js}{lines:true,startLine:118,maxHeight:'140px'}
<<< ../reference/kinematics.js#matrix-from-origin {js}{lines:true,startLine:132,maxHeight:'100px'}

---
layout: default
---

# Matrix multiplication, composing the stack

<<< ../reference/matrix.js#matrix-multiply {js}{lines:true,startLine:51,maxHeight:'300px'}

<div class="panel text-sm" style="margin-top:0.5em">

Composition is always **parent-left, child-right**: `matrix_multiply(parentMatrix, localOffset)`,
exactly the convention both AutoRob lectures use (`T<sub>ij</sub>`, composed outward from the
base). With column vectors the **rightmost matrix acts on the point first**, so the child's own
local offset is applied before the parent's transform carries it out into the world. Order
matters: matrix multiplication does not commute, and neither does 3D rotation.

</div>

---
layout: default
---

# 2. The DFS traversal: `mobile_arm` test robot

<div class="panel text-sm" style="margin-bottom:0.4em">

Step through the traversal below: a **link** is entered, its matrix set from the parent's matrix,
then each of its **child joints** is visited in turn -- each composing its own fixed origin and
variable motion before recursing into *its* child link. Watch the pseudocode panel's active line
and the schematic tree assemble itself link by link.

</div>

<div class="content-body">
<MatrixStackPanel robot-name="mobile_arm" />
</div>

---
layout: default
---

# `traverseFKJoint`: fixed offset, then variable motion

<<< ../reference/kinematics.js#traverse-fk-joint {js}{lines:true,startLine:139,maxHeight:'420px'}

---
layout: default
---

# `traverseFKLink` and `traverseFKBase`

<<< ../reference/kinematics.js#traverse-fk-link {js}{lines:true,startLine:169,maxHeight:'220px'}
<<< ../reference/kinematics.js#traverse-fk-base {js}{lines:true,startLine:181,maxHeight:'220px'}

---
layout: default
---

# 3. Quaternions: a joint's own rotation

<div class="panel text-sm">

Every joint's variable rotation is built the same two-step way the quaternions lecture itself
lays out: **(1)** form a unit quaternion from the joint's axis and its current angle; **(2)**
convert that quaternion straight to a 4&times;4 rotation matrix, ready to fold into the matrix
stack like any other transform.

</div>

<<< ../reference/quaternion.js#quaternion-from-axisangle {js}{lines:true,startLine:33,maxHeight:'200px'}
<<< ../reference/quaternion.js#quaternion-to-rotation-matrix {js}{lines:true,startLine:76,maxHeight:'200px'}

---
layout: default
---

# Quaternion multiplication: composing rotations

<<< ../reference/quaternion.js#quaternion-multiply {js}{lines:true,startLine:62,maxHeight:'260px'}

<div class="panel text-sm" style="margin-top:0.5em">

The Hamilton product is **not commutative** -- `q1*q2 != q2*q1` in general -- for exactly the same
reason matrix multiplication isn't: composing two 3D rotations is itself order-dependent. A unit
quaternion and its negation (`q` and `-q`) represent the *same* rotation, which is why
`quaternion_normalize` only ever needs to fix the overall scale, never a sign ambiguity.

</div>

---
layout: default
---

# Interactive: 2D rotation by a complex number

<div class="panel text-sm" style="margin-bottom:0.4em">

A unit complex number **a + bi = (cos&theta;, sin&theta;)** rotates a 2D point by multiplication --
drag the angle and watch the point (gray) rotate to its image (amber).

</div>

<div class="content-body">
<ComplexRotation2DPanel />
</div>

---
layout: default
---

# Interactive: 3D rotation by a quaternion

<div class="panel text-sm" style="margin-bottom:0.4em">

The same idea, one dimension up: a unit quaternion built from an axis and an angle rotates a 3D
point by conjugation, **v&prime; = q v q<sup>-1</sup>**. Drag the axis and angle and watch the
point (gray) rotate to its image (amber) around the dashed axis line.

</div>

<div class="content-body">
<QuaternionRotation3DPanel />
</div>

---
layout: default
---

# 4. A new robot: `mobile_arm`

<div class="panel text-sm">

A single mobile-base link with a 2-joint **planar arm** mounted on top -- 3 links total, matching
this lab's own brief. Both arm joints rotate about the same (world "up") axis, so the whole arm
sweeps one horizontal plane as it moves; the base link's own world pose is free to translate/rotate
in that same plane, independent of the arm.

</div>

<<< ../reference/robots/mobile_arm.urdf.js#create-mobile-arm-robot {js}{lines:true,startLine:45,maxHeight:'380px'}

---
layout: default
---

# `mobile_arm`, stepped (and `urdf_example`, for branching)

<div class="panel text-sm" style="margin-bottom:0.4em">

`urdf_example` (ported from the upstream kineval-stencil's own
`robots/robot_urdf_example.js`) is this module's second test robot specifically *because* its tree
**branches** -- `link1` has two child joints -- exercising `traverseFKLink`'s "for each child
joint" loop more than once, which the single-chain `mobile_arm` never does.

</div>

<div class="content-body">
<MatrixStackPanel robot-name="urdf_example" />
</div>

---
layout: default
---

# 5. Comparison: matrix stack vs. Denavit-Hartenberg

<div class="panel text-sm">

Straight from the quaternions lecture's own "D-H versus Matrix stack" slide:

</div>

<div class="split-panel" style="margin-top:0.5em">
<div>

<div class="panel">

**Denavit-Hartenberg**
- 4 parameters to transform between links
- 4 matrices to compose per joint
- Uniformity in frame selection
- Only link frames are necessary for computations
- Less intuitive

</div>
</div>
<div>

<div class="panel">

**Matrix stack (this module)**
- 10 parameters to transform between links
- 3 matrices to compose per joint
- Flexibility in frame selection
- Child link shares frame with parent joint
- More bookkeeping

</div>
</div>
</div>

---
layout: default
---

# Comparison, continued: dual quaternions, PoE, and USD

<div class="panel text-sm">

Two more alternatives the quaternions lecture names (citing **[Kenwright 2012; Daniilidis 1999]**
and **[Lynch &amp; Park 2017]** respectively), without a worked example in either source:

- **Dual quaternions** fold rotation *and* translation into one algebraic object -- a real
  quaternion (rotation) plus a dual part built from the translation -- so a single conjugation
  does what this module needs a separate matrix-stack translation *and* quaternion rotation for.
  Pitched for representing general **screw motion** cleanly.
- **Products of exponentials (PoE)** describe a joint's motion as the exponential of its screw
  axis, composed along the chain -- named only, with no formulas given in either lecture (see
  Lynch &amp; Park, *Modern Robotics*, for the full treatment).

</div>

<div class="history-note">
<b>Beyond URDF: USD</b>
Universal Scene Description (USD) is <em>not</em> a mesh-geometry format comparable to STL/Collada/
OBJ -- the standard ROS URDF mesh loaders don't consume <code>.usd</code> at all. It's a whole-
<em>scene</em> description format (materials, variants, animation) that competes with URDF+SDF at
the robot-description level, increasingly relevant in robotics simulation (NVIDIA Isaac Sim/
Omniverse) but outside this module's scope.
</div>

---
layout: default
---

# Test cases

<div class="panel text-sm">

Every test case is a plain hyperlink into <code>forward_kinematics.html</code>, same convention as
every other module in this project -- click any URL below to open that test case in a new tab:

</div>

<div class="panel test-case-table" style="font-size:0.74em">

| Test case | URL |
|---|---|
| `mobile_arm`, default pose | <a href="/kineval/forward_kinematics/reference/forward_kinematics.html?robot=mobile_arm" target="_blank">?robot=mobile_arm</a> |
| `mobile_arm`, posed via sliders | <a href="/kineval/forward_kinematics/reference/forward_kinematics.html?robot=mobile_arm&angles=0.785,-0.524" target="_blank">?robot=mobile_arm&angles=0.785,-0.524</a> |
| `urdf_example`, branching tree | <a href="/kineval/forward_kinematics/reference/forward_kinematics.html?robot=urdf_example" target="_blank">?robot=urdf_example</a> |
| Mobile base translated + yawed | <a href="/kineval/forward_kinematics/reference/forward_kinematics.html?robot=mobile_arm&base_x=1&base_z=-0.5&base_yaw=0.6" target="_blank">?robot=mobile_arm&base_x=1&base_z=-0.5&base_yaw=0.6</a> |
| **New 1**: arm fully extended | <a href="/kineval/forward_kinematics/reference/forward_kinematics.html?robot=mobile_arm&angles=0,0" target="_blank">?robot=mobile_arm&angles=0,0</a> |
| **New 2**: arm folded back on itself | <a href="/kineval/forward_kinematics/reference/forward_kinematics.html?robot=mobile_arm&angles=1.5708,3.1416" target="_blank">?robot=mobile_arm&angles=1.5708,3.1416</a> |
| **New 3**: `urdf_example`'s two branches at extremes | <a href="/kineval/forward_kinematics/reference/forward_kinematics.html?robot=urdf_example&angles=1.5708,-1.5708,1.5708" target="_blank">?robot=urdf_example&angles=1.5708,-1.5708,1.5708</a> |
| **New 4**: slow step-through for projection | <a href="/kineval/forward_kinematics/reference/forward_kinematics.html?robot=urdf_example&step_ms=1500" target="_blank">?robot=urdf_example&step_ms=1500</a> |
| **New 5**: base far from world origin, arm swept | <a href="/kineval/forward_kinematics/reference/forward_kinematics.html?robot=mobile_arm&base_x=-2&base_z=2&angles=-1.2,0.8" target="_blank">?robot=mobile_arm&base_x=-2&base_z=2&angles=-1.2,0.8</a> |

</div>

---
layout: default
---

# Appendix: linear algebra used in this module

<div class="panel text-sm">

Every transform above is a **4&times;4 homogeneous matrix** acting on a **homogeneous 4&times;1
column vector**: `[x,y,z,1]^T` for a **point**, `[x,y,z,0]^T` for a **direction** -- the extra
coordinate is what lets a single matrix multiply express translation *and* rotation together, and
the `w = 0` case is what makes a direction immune to the translation part. `matrix_multiply`
(shown earlier) is the one operation every composition in this module reduces to.

Getting that `w` wrong is a real trap: `robot.heading`/`robot.lateral` look like directions (the
upstream stencil's own comment calls them that), but their consumer subtracts the base position
back off them, so they have to be built as **points** -- `w = 1`. Built with `w = 0` instead,
"drive forward" silently stops pointing forward the moment the robot leaves the world origin.

</div>

<<< ../reference/matrix.js#matrix-invert-affine {js}{lines:true,startLine:167,maxHeight:'260px'}

<div class="panel text-sm" style="margin-top:0.4em">
A rigid transform's inverse is cheap because its rotation block is <b>orthonormal</b>
(R<sup>-1</sup> = R<sup>T</sup>) -- no general matrix inversion needed, just a transpose and a
sign-flipped translation.
</div>

---
layout: default
---

# Appendix: cross product, normalization, pseudoinverse

<<< ../reference/matrix.js#vector-cross {js}{lines:true,startLine:218,maxHeight:'140px'}
<<< ../reference/matrix.js#vector-normalize {js}{lines:true,startLine:201,maxHeight:'140px'}

<div class="panel text-sm" style="margin-top:0.4em">

`matrix_pseudoinverse` (below) is part of the completed `kineval_matrix.js` STENCIL but **unused by
forward kinematics itself** -- it's what `inverse_kinematics/` (this project's next planned module)
will need to turn a Jacobian into a joint-velocity update.

</div>

<<< ../reference/matrix.js#matrix-pseudoinverse {js}{lines:true,startLine:143,maxHeight:'220px'}

---
layout: default
---

# Appendix: matrix utilities

<div class="panel text-sm">

`matrix_copy` and `matrix_transpose` are the two small, general-purpose array operations every
other matrix.js function is built from; `matrix_square_invert` is the Gauss-Jordan elimination
helper `matrix_pseudoinverse` (shown earlier) needs internally -- not itself one of the stencil's
named functions, but the smallest piece that made completing `matrix_pseudoinverse` without an
external linear-algebra dependency possible.

</div>

<<< ../reference/matrix.js#matrix-copy {js}{lines:true,startLine:36,maxHeight:'140px'}
<<< ../reference/matrix.js#matrix-transpose {js}{lines:true,startLine:70,maxHeight:'140px'}
<<< ../reference/matrix.js#matrix-square-invert {js}{lines:true,startLine:85,maxHeight:'280px'}

---
layout: default
---

# Appendix: the rest of the matrix.js STENCIL

<<< ../reference/matrix.js#generate-identity {js}{lines:true,startLine:229,maxHeight:'120px'}
<<< ../reference/matrix.js#generate-rotation-matrix-x {js}{lines:true,startLine:253,maxHeight:'140px'}
<<< ../reference/matrix.js#generate-rotation-matrix-y {js}{lines:true,startLine:266,maxHeight:'140px'}
<<< ../reference/matrix.js#generate-rotation-matrix-z {js}{lines:true,startLine:279,maxHeight:'140px'}

---
layout: default
---

# Appendix: robot init and the joint hierarchy

<div class="panel text-sm">

A robot description only states each *joint's* parent/child link; `initRobotJoints` builds the
reverse lookup every `traverseFKLink` call needs -- which joints hang off a given link, and which
joint connects a link back to its own parent.

</div>

<<< ../reference/kinematics.js#init-robot-links {js}{lines:true,startLine:75,maxHeight:'140px'}
<<< ../reference/kinematics.js#init-robot-joints {js}{lines:true,startLine:87,maxHeight:'220px'}
<<< ../reference/kinematics.js#init-robot {js}{lines:true,startLine:111,maxHeight:'80px'}

---
layout: default
---

# Appendix: `quaternion_normalize` and `buildFKTransforms`

<<< ../reference/quaternion.js#quaternion-normalize {js}{lines:true,startLine:51,maxHeight:'160px'}
<<< ../reference/kinematics.js#build-fk-transforms {js}{lines:true,startLine:196,maxHeight:'100px'}

---
layout: default
---

# Appendix: three.js scene construction

<div class="panel text-sm">

Deliberately does **not** use three.js's own parent-child nesting to pose the robot -- every link
mesh sits flat in the scene, and `updateRobotMeshes` copies the already-fully-composed matrix
`kinematics.js` computed straight onto it, so the matrix-stack math in this deck is what's actually
driving the picture, not three.js's own scene graph.

</div>

<<< ../reference/scene.js#make-link-geometry {js}{lines:true,startLine:47,maxHeight:'220px'}
<<< ../reference/scene.js#create-viewer {js}{lines:true,startLine:72,maxHeight:'380px'}

---
layout: default
---

# Appendix: posing and revealing link meshes

<<< ../reference/scene.js#update-robot-meshes {js}{lines:true,startLine:153,maxHeight:'380px'}
<<< ../reference/scene.js#resize-renderer {js}{lines:true,startLine:193,maxHeight:'100px'}

---
layout: default
---

# Appendix: URL parameters and robot selection

<<< ../reference/infrastructure.js#appendix-url-params {js}{lines:true,startLine:36,maxHeight:'420px'}

---
layout: default
---

# Appendix: the DFS step-through HUD

<div class="panel text-sm">

The standalone reference page's own Start/Pause/Reset/Step controls step through
`buildFKTransforms`'s DFS **traversal order** one node at a time -- there's no elapsed "time" in a
forward-kinematics evaluation, only tree order, so this HUD's animation loop advances a step index
instead of a clock.

</div>

<<< ../reference/infrastructure.js#fk-traversal-order {js}{lines:true,startLine:81,maxHeight:'220px'}
<<< ../reference/infrastructure.js#appendix-animate {js}{lines:true,startLine:214,maxHeight:'180px'}

---
layout: default
---

# Appendix: the HUD panel and joint sliders

<<< ../reference/infrastructure.js#hud-panel {js}{lines:true,startLine:111,maxHeight:'420px'}

---
layout: default
---

# Appendix: `urdf_example`, the branching test robot

<div class="panel text-sm">

Ported from the upstream kineval-stencil's own `robots/robot_urdf_example.js` onto this module's
geometry-as-plain-data convention -- kept specifically because its tree branches (`link1` has two
child joints), unlike the single-chain `mobile_arm`.

</div>

<<< ../reference/robots/urdf_example.js#create-urdf-example-robot {js}{lines:true,startLine:36,maxHeight:'380px'}

---
layout: center
class: text-center
---

# That's forward kinematics and quaternions

<div class="panel" style="display:inline-block; text-align:left; margin-top:1em">

- A matrix stack composes every **fixed** offset in a robot's kinematic tree
- A quaternion, built from axis + angle, composes every joint's **variable** motion
- URDF's `<visual>`/`<collision>`/`<inertial>` all hang off the same link -- this module builds
  only the first
- D-H, dual quaternions, and products of exponentials all solve the same problem differently

</div>

<div class="title-footer">
<div class="nav-hint">Next: inverse kinematics and optimization</div>
<div class="credit">AutoRob (autorob.org) &#183; Chad Jenkins (ocj@umich.edu) &#183; ocj-dev.github.io/kineval/forward_kinematics</div>
</div>
