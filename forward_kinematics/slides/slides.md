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

<div class="content-body">
<div class="split-panel">
<div>

<div class="panel text-sm">

Before quaternions, there was a cleaner way to rotate a vector about an arbitrary axis than three
stacked Euler-angle rotations: the **Rodrigues rotation formula**, named for **Benjamin Olinde
Rodrigues (1795-1851)**. Split **b** into parts parallel and perpendicular to the axis **a**,
rotate only the perpendicular part, recombine:

<div class="panel" style="margin-top:0.5em; font-size:1.05em">

**b&prime; = (1 &minus; cos&theta;)(a&middot;b)a + b cos&theta; + (a&times;b) sin&theta;**

</div>

Each term is drawn on the right. Green **(a&middot;b)a** is the part of **b** lying *along* the
axis, which the rotation never touches -- that is why **b&prime;** rides a circle, not a sphere.
Purple **(a&times;b) sin&theta;** gives the second perpendicular direction needed to sweep the
rest of **b** round. The next two slides unpack the **skew-symmetric matrix** and **outer product**
this becomes as a matrix.

</div>
</div>
<div>
<RodriguesPanel />
</div>
</div>
</div>

---
layout: default
---

# The cross product as a matrix: skew-symmetric form

<div class="panel text-sm">

The **a&times;b** term above is a *vector* operation, but Rodrigues' formula is on its way to
becoming a *matrix*. The bridge is that crossing with a fixed vector is a **linear** operation, so
it can be written as a matrix acting on **b**. That matrix is the **skew-symmetric matrix** of
**a**, written **[a]<sub>&times;</sub>**:

</div>

<div class="split-panel" style="margin-top:0.4em">
<div>

<div class="panel">

**[a]<sub>&times;</sub> =**

| | | |
|---|---|---|
| 0 | &minus;a<sub>z</sub> | a<sub>y</sub> |
| a<sub>z</sub> | 0 | &minus;a<sub>x</sub> |
| &minus;a<sub>y</sub> | a<sub>x</sub> | 0 |

so that **[a]<sub>&times;</sub> b = a &times; b** for every **b**.

</div>
</div>
<div>

<div class="panel text-sm">

"Skew-symmetric" means **[a]<sub>&times;</sub><sup>T</sup> = &minus;[a]<sub>&times;</sub>** --
transposing it flips every sign. Two consequences worth keeping:

- the diagonal must be zero (only 0 equals its own negation), so it carries just **3** independent
  numbers, exactly the 3 in **a**
- **a &times; a = 0** falls straight out, which is the algebraic version of "a rotation leaves its
  own axis alone"

This module's `vector_cross` computes the same thing directly, without ever forming the matrix --
cheaper when you only need one product.

</div>
</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em">

With it, Rodrigues collapses to the **matrix** form
**R = I + sin&theta; [a]<sub>&times;</sub> + (1 &minus; cos&theta;) [a]<sub>&times;</sub><sup>2</sup>**
-- a rotation matrix built from an axis and an angle, no Euler stack anywhere.

</div>

---
layout: default
---

# The outer product: projecting onto the axis

<div class="panel text-sm">

The other term, **(a&middot;b)a**, is also linear in **b**, and it gets the same treatment. The
**outer product** **a a<sup>T</sup>** multiplies a 3&times;1 by a 1&times;3 to give a **3&times;3
matrix** -- the mirror image of the inner (dot) product **a<sup>T</sup>a**, which multiplies the
same two the other way round to give a single **scalar**.

</div>

<div class="split-panel" style="margin-top:0.4em">
<div>

<div class="panel">

**a a<sup>T</sup> =**

| | | |
|---|---|---|
| a<sub>x</sub>a<sub>x</sub> | a<sub>x</sub>a<sub>y</sub> | a<sub>x</sub>a<sub>z</sub> |
| a<sub>y</sub>a<sub>x</sub> | a<sub>y</sub>a<sub>y</sub> | a<sub>y</sub>a<sub>z</sub> |
| a<sub>z</sub>a<sub>x</sub> | a<sub>z</sub>a<sub>y</sub> | a<sub>z</sub>a<sub>z</sub> |

**(a a<sup>T</sup>) b = a (a&middot;b) = (a&middot;b) a**

</div>
</div>
<div>

<div class="panel text-sm">

For a **unit** **a**, **a a<sup>T</sup>** is a **projection matrix**: it throws away everything
perpendicular to **a** and keeps only the part along it -- precisely the green vector on the
Rodrigues panel. Its properties follow from that reading:

- **idempotent**: (a a<sup>T</sup>)(a a<sup>T</sup>) = a a<sup>T</sup>. Projecting twice is the
  same as projecting once
- **rank 1**: every output is a multiple of **a**, so the matrix flattens all of 3D onto one line
- and **I &minus; a a<sup>T</sup>** is the complementary projection, keeping only the perpendicular
  part -- the part the rotation actually moves

</div>
</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em">

Writing Rodrigues with both pieces gives its third equivalent form:
**R = cos&theta; I + sin&theta; [a]<sub>&times;</sub> + (1 &minus; cos&theta;) a a<sup>T</sup>** --
"keep a bit of the original, swing some of it with the cross product, and leave the axis-parallel
part untouched", stated in matrices.

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

<div class="side-image">
<img src="/images/broom_bridge_quaternion_plaque.jpg" alt="Stone plaque on Broom Bridge, Dublin, reading: Here as he walked by on the 16th of October 1843 Sir William Rowan Hamilton in a flash of genius discovered the fundamental formula for quaternion multiplication i squared equals j squared equals k squared equals ijk equals minus one and cut it on a stone of this bridge" />
</div>
<div class="side-caption">
The plaque on Broom (Brougham) Bridge, Dublin &mdash; <i>"&hellip;discovered the fundamental formula
for quaternion multiplication i&sup2; = j&sup2; = k&sup2; = ijk = &minus;1 &amp; cut it on a stone
of this bridge."</i><br>
Photo: Cone83, <a href="https://creativecommons.org/licenses/by-sa/4.0/">CC BY-SA 4.0</a>, via Wikimedia Commons
</div>
</div>
</div>

---
layout: default
---

# Why not just Euler angles? Gimbal lock

<div class="content-body">
<div class="split-panel">
<div>

<div class="panel text-sm">

Three stacked axis rotations -- **R = R<sub>z</sub>R<sub>y</sub>R<sub>x</sub>**, AutoRob's XYZ
order -- are the most direct way to build a 3D rotation, and what `matrix_from_rpy` uses for every
*fixed* `<origin rpy="...">`. But driving a joint's *variable* rotation this way has a failure
mode: **gimbal lock**, where two axes rotate into alignment and 3 DOF collapse to 2.

Drive the **pitch** ring to &plusmn;90&deg;. The roll axis swings onto the yaw axis, the matrix
turns red, and the two stop being independent. The lecture's "shoulder" version is this same
event: **R<sub>z</sub>(90&deg;)R<sub>y</sub>(90&deg;)R<sub>x</sub>(90&deg;)** leaves an arm's
z-axis on its *original* x-axis.

This is why `traverseFKJoint` never stacks Euler angles for a joint's own motion -- each joint gets
one axis-angle rotation via a quaternion, which has no such singularity.

</div>
</div>
<div>
<GimbalLockPanel />
</div>
</div>
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

A real URDF file is XML. A `<link>` carries up to three independent descriptions of its physical
extent: **`<visual>`** (rendered) and **`<collision>`** (what a planner checks) each hold their own
`<geometry>` -- specifiable separately because a visual mesh is usually far finer than collision
needs -- while **`<inertial>`** holds *no* geometry, only `<mass>` and the `<inertia>` tensor. A
`<joint>` names its parent/child link, a fixed `<origin>`, and an `<axis>`; `revolute` also
requires a `<limit>`.

</div>

```xml {*}{lines:true,maxHeight:'300px'}
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
  <parent link="mast_link"/>
  <child link="arm_link1"/>
  <origin xyz="0 0.3 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <!-- required for revolute -->
  <limit lower="-3.14" upper="3.14"
         effort="100" velocity="10"/>
</joint>
```

</div>
<div>

```js
joint1: {
  parent: 'mast_link', child: 'arm_link1',
  type: 'revolute',
  origin: { xyz: [0, 0.3, 0], rpy: [0, 0, 0] },
  axis: [0, 0, 1], angle: 0
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
information at all. Common use: **3D-printing and CAD export**, and -- because a format with no
material data costs nothing to strip down -- **collision geometry**.

The wheel on the right is Fetch's real `l_wheel_link.STL`. Fetch leans on exactly that split:
`fetch.urdf` points `<visual>` at this file and `<collision>` at a *separate*, coarser
`l_wheel_link_collision.STL` -- the same link, two meshes, two jobs.

</div>
</div>
<div>
<div class="side-image" style="background:#f5f5f5">
<MeshFormatExamplePanel format="stl" mesh-url="meshes/fetch_l_wheel_link.STL" :color="0xd98236" />
</div>
<div class="side-caption">live three.js render via STLLoader &mdash; Fetch's <code>l_wheel_link.STL</code>, straight out of <code>fetch.urdf</code></div>
</div>
</div>
</div>

---
layout: default
---

# Anatomy of an STL file

<div class="panel text-sm">

Fetch's `l_wheel_link.STL` is a **binary** STL, as most real robot STLs are -- there is no text to
read. The whole file is a header, a triangle count, then one fixed 50-byte record per triangle.

</div>

<div class="split-panel" style="margin-top:0.4em">
<div>

<div class="panel" style="font-size:0.8em">

| bytes | field |
|---|---|
| `0 .. 79` | header, ignored (all zero here) |
| `80 .. 83` | `uint32` triangle count = **2086** |
| `84 ..` | 2086 &times; the 50-byte record below |
| | |
| `+0 .. 11` | 3 &times; `float32` facet normal |
| `+12 .. 23` | 3 &times; `float32` vertex 1 |
| `+24 .. 35` | 3 &times; `float32` vertex 2 |
| `+36 .. 47` | 3 &times; `float32` vertex 3 |
| `+48 .. 49` | `uint16` attribute, unused |

</div>
</div>
<div>

<div class="panel" style="font-size:0.78em">

**The real first triangle, decoded**

```text
normal  = ( +0.00000, +1.00000, +0.00000 )
vertex1 = ( +0.00727, +0.04250, -0.00420 )
vertex2 = ( +0.01535, +0.04250, -0.01433 )
vertex3 = ( +0.00494, +0.04250, -0.00680 )
```

84 + 50&times;2086 = **104,384 bytes**, exactly the file size: nothing is optional, so triangle
count alone fixes the size. All three vertices share `y = +0.0425` -- a flat face on the wheel.

</div>
</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em">

STL's **ASCII** form carries identical data as `solid` / `facet normal` / `vertex` / `endfacet`
keywords in roughly 6&times; the bytes. Loaders sniff the header to tell them apart.

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
skeletal rigging, and animation, which is why Baxter, Fetch and Sawyer all ship their *visual*
meshes as .dae. The tradeoff for that richness is a much more verbose, harder-to-hand-author file
than STL or OBJ. Common use: **richly-textured, rigged, or multi-part assets** carried between
different authoring/rendering tools.

The head on the right is Fetch's real `head_pan_link.dae`, and note what the STL slide's wheel
could not do: it arrives **already textured**, because the file names its own image
(`head_pan_uv.png`) internally. Nothing in this deck assigns it a material.

</div>
</div>
<div>
<div class="side-image" style="background:#f5f5f5">
<MeshFormatExamplePanel format="collada" mesh-url="meshes/fetch_head_pan_link.dae" :color="0x27966b" />
</div>
<div class="side-caption">live three.js render via ColladaLoader &mdash; Fetch's <code>head_pan_link.dae</code>, <b>with its own texture</b>, which STL could not carry</div>
</div>
</div>
</div>

---
layout: default
---

# Anatomy of a Collada file

<div class="panel text-sm">

Collada is XML, so it *is* readable -- the cost is that what STL spends 50 bytes on, Collada
spells out in tags. These are the real sections of Fetch's `head_pan_link.dae`.

</div>

<div class="split-panel" style="margin-top:0.4em">
<div>

```xml {*}{maxHeight:'330px'}
<COLLADA version="1.4.1">
  <asset><up_axis>Z_UP</up_axis></asset>
  <library_images/>        <!-- head_pan_uv.png -->
  <library_effects/>       <!-- shading params -->
  <library_materials/>
  <library_geometries><mesh>
    <source id="...-positions"/>
    <source id="...-normals"/>
    <source id="...-map-0"/> <!-- UVs -->
    <vertices/>
    <triangles material="...">
      <input semantic="VERTEX"   offset="0"/>
      <input semantic="NORMAL"   offset="1"/>
      <input semantic="TEXCOORD" offset="2"/>
      <p>... index stream ...</p>
    </triangles>
  </mesh></library_geometries>
  <library_visual_scenes/> <!-- node tree -->
</COLLADA>
```

</div>
<div>

<div class="panel" style="font-size:0.78em">

```xml
<float_array
  id="head_pan_link-mesh-positions-array"
  count="5532">
0.1974999 -0.1282082 0.05799853
0.1974999 -0.1083217 0.05799853 ...
```

</div>

<div class="panel text-sm" style="margin-top:0.4em">

`count="5532"` counts **floats**, not points: 5532/3 = **1844 positions**, each stored once and
then referenced by index from `<p>`. That indexing is what STL cannot express.

The three `<input>` lines are the payoff -- one triangle corner carries a position **and** a normal
**and** a UV, each from its own array. `<library_images>` is how this file knows about its texture
and arrives already shaded.

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

Worth noticing which robot this one came from: **neither Fetch nor PR2 ships a single `.obj`** --
both are STL + Collada throughout. The leg on the right is Spot's
`front_left_lower_leg.obj`. In a URDF pipeline OBJ is the format you meet when a mesh arrives
from outside the ROS ecosystem, rather than the one roboticists reach for first.

</div>
</div>
<div>
<div class="side-image" style="background:#f5f5f5">
<MeshFormatExamplePanel format="obj" mesh-url="meshes/spot_front_left_lower_leg.obj" :color="0x3b6ea5" />
</div>
<div class="side-caption">live three.js render via OBJLoader &mdash; Spot's <code>front_left_lower_leg.obj</code> (not Fetch or PR2: neither ships OBJ)</div>
</div>
</div>
</div>

---
layout: default
---

# Anatomy of an OBJ file

<div class="panel text-sm">

OBJ is line-oriented plain text: one record per line, first token naming the type. Every line
below is real, from Spot's `front_left_lower_leg.obj`.

</div>

<div class="split-panel" style="margin-top:0.4em">
<div>

<div class="panel" style="font-size:0.8em">

| keyword | count | meaning |
|---|---|---|
| `mtllib` | 1 | companion material file |
| `o` / `g` | 1 / 1 | object and group name |
| `usemtl` | 1 | material for following faces |
| `s` | 1 | smoothing group (`0` = off) |
| `v` | 21858 | vertex position |
| `vn` | 7521 | vertex normal |
| `vt` | 3 | texture coordinate |
| `f` | 7548 | face |

</div>
</div>
<div>

<div class="panel" style="font-size:0.78em">

```text
mtllib ../../spot.mtl
usemtl BlackAbs
s      0
v      -0.034704 0.008309 -0.334707
vn     -0.9859 0.1514 -0.0707
vt     0.535156 0.148438
f      1/1/1 2/2/1 3/2/1
```

</div>

<div class="panel text-sm" style="margin-top:0.4em">

Each `f` corner is **`position/texcoord/normal`** -- three *1-based* indices into the lists above,
the indirection that lets one `v` serve many faces. This file barely uses it: 21858 positions for
7548 faces is near-zero sharing, because it came from CAD. The format permits sharing; the
exporter decides.

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

<<< ../reference/kinematics.js#forward-kinematics {*}{lines:true,startLine:117,maxHeight:'420px'}

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

<<< ../reference/matrix.js#generate-translation-matrix {*}{lines:true,startLine:241,maxHeight:'140px'}
<<< ../reference/kinematics.js#matrix-from-rpy {*}{lines:true,startLine:118,maxHeight:'140px'}
<<< ../reference/kinematics.js#matrix-from-origin {*}{lines:true,startLine:132,maxHeight:'100px'}

---
layout: default
---

# Matrix multiplication, composing the stack

<<< ../reference/matrix.js#matrix-multiply {*}{lines:true,startLine:51,maxHeight:'300px'}

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

A **link** is entered, its matrix set from the parent's; then each **child joint** composes its
fixed origin and variable motion before recursing into *its* child link. The **call stack** on the
left *is* the matrix stack -- the transforms live in the recursion, pushed and popped with it.

</div>

<div class="content-body">
<MatrixStackPanel robot-name="mobile_arm" />
</div>

---
layout: default
---

# `traverseFKJoint`: fixed offset, then variable motion

<<< ../reference/kinematics.js#traverse-fk-joint {*}{lines:true,startLine:139,maxHeight:'420px'}

---
layout: default
---

# `traverseFKLink` and `traverseFKBase`

<<< ../reference/kinematics.js#traverse-fk-link {*}{lines:true,startLine:169,maxHeight:'220px'}
<<< ../reference/kinematics.js#traverse-fk-base {*}{lines:true,startLine:181,maxHeight:'220px'}

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

<<< ../reference/quaternion.js#quaternion-from-axisangle {*}{lines:true,startLine:33,maxHeight:'200px'}
<<< ../reference/quaternion.js#quaternion-to-rotation-matrix {*}{lines:true,startLine:76,maxHeight:'200px'}

---
layout: default
---

# Quaternion multiplication: composing rotations

<<< ../reference/quaternion.js#quaternion-multiply {*}{lines:true,startLine:62,maxHeight:'260px'}

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

One dimension up: a unit quaternion built from an axis and an angle rotates a 3D point by
conjugation, **v&prime; = q v q<sup>-1</sup>**. Drag the axis, the angle, or **v** itself. The amber
ring is where **v** lands for *every* &theta; -- the circle it traces about the axis. Tilt the axis
onto **v** and the ring shrinks to nothing: a vector along the rotation axis is what the rotation
leaves fixed. **Orbit view** turns the camera, since any fixed 2D view of a 3D circle can catch it
edge-on.

</div>

<div class="content-body">
<QuaternionRotation3DPanel />
</div>

---
layout: default
---

# 4. A new robot: `mobile_arm`

<div class="panel text-sm">

A mobile base carrying a pitching mast with a 2-joint **planar arm** on top. All three joints turn
about the **lateral +z axis**, which is what makes the arm planar in a **vertical** plane -- it
reaches up and out, rather than sweeping a horizontal turntable. `joint_pitch` tilts the mast, and
with it that entire plane, nose-up/nose-down; `joint1`/`joint2` articulate the arm inside it. The
base link's own world pose stays free to translate/rotate on the ground plane, independent of all
three.

</div>

<<< ../reference/robots/mobile_arm.urdf.js#create-mobile-arm-robot {*}{lines:true,startLine:45,maxHeight:'380px'}

---
layout: default
---

# `mobile_arm`, stepped (and `urdf_example`, for branching)

<div class="panel text-sm" style="margin-bottom:0.4em">

`urdf_example` (from the upstream stencil's own `robots/robot_urdf_example.js`) branches --
`link1` has two child joints -- so `traverseFKLink`'s "for each child joint" loop runs more than
once, and the call stack visibly pops back down to `link1` before descending the second branch.

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

<div class="panel text-sm" style="font-size:0.78em">

Every case is a plain hyperlink, same convention as every other module here -- click any URL to
open it in a new tab. The first group runs in this module's own
<code>forward_kinematics.html</code>; the second runs the **full KinEval viewer**, which drives the
same completed FK code over the upstream stencil's own robots.

</div>

<div class="split-panel" style="margin-top:0.4em">
<div>

<div class="panel test-case-table" style="font-size:0.62em">

**this module's reference page**

| Test case | URL |
|---|---|
| `mobile_arm`, default pose | <a href="/kineval/forward_kinematics/reference/forward_kinematics.html?robot=mobile_arm" target="_blank">?robot=mobile_arm</a> |
| `mobile_arm`, pitched mast + arm swept | <a href="/kineval/forward_kinematics/reference/forward_kinematics.html?robot=mobile_arm&angles=0.5,0.8,-1.1" target="_blank">?robot=mobile_arm&angles=0.5,0.8,-1.1</a> |
| `mobile_arm`, base driven off the origin | <a href="/kineval/forward_kinematics/reference/forward_kinematics.html?robot=mobile_arm&base_x=-2&base_z=2&base_yaw=0.6" target="_blank">?robot=mobile_arm&base_x=-2&base_z=2&base_yaw=0.6</a> |
| `urdf_example`, branching tree | <a href="/kineval/forward_kinematics/reference/forward_kinematics.html?robot=urdf_example" target="_blank">?robot=urdf_example</a> |
| `urdf_example`, both branches at extremes | <a href="/kineval/forward_kinematics/reference/forward_kinematics.html?robot=urdf_example&angles=1.5708,-1.5708,1.5708" target="_blank">?robot=urdf_example&angles=1.5708,-1.5708,1.5708</a> |

</div>
</div>
<div>

<div class="panel test-case-table" style="font-size:0.62em">

**full KinEval viewer** &mdash; `w/s a/d q/e` drive the base, `j/k/l/h` pick a joint, `u/i` turn it

| Test case | URL |
|---|---|
| **crawler** &mdash; 25 links, eight 3-joint legs off one base | <a href="/kineval/kineval/?robot=crawler" target="_blank">?robot=crawler</a> |
| **mr2** &mdash; upstream's partial humanoid stencil | <a href="/kineval/kineval/?robot=mr2" target="_blank">?robot=mr2</a> |
| **fetch** &mdash; 21 links; the only one with *prismatic* joints | <a href="/kineval/kineval/?robot=fetch" target="_blank">?robot=fetch</a> |
| **baxter** &mdash; 20 links, two 7-DOF arms | <a href="/kineval/kineval/?robot=baxter" target="_blank">?robot=baxter</a> |
| **sawyer** &mdash; 10 links, a single 7-DOF arm | <a href="/kineval/kineval/?robot=sawyer" target="_blank">?robot=sawyer</a> |

</div>
</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em; font-size:0.72em">

`fetch`, `baxter` and `sawyer` are ported from the upstream stencil's own `robots/` descriptions
with their **kinematics intact but their meshes left out** -- 20-35MB apiece -- so the viewer
draws a skeleton synthesized from each link's own joint offsets. Forward kinematics reads origins,
axes and angles, never meshes, so it is unaffected.

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

<<< ../reference/matrix.js#matrix-invert-affine {*}{lines:true,startLine:167,maxHeight:'260px'}

<div class="panel text-sm" style="margin-top:0.4em">
A rigid transform's inverse is cheap because its rotation block is <b>orthonormal</b>
(R<sup>-1</sup> = R<sup>T</sup>) -- no general matrix inversion needed, just a transpose and a
sign-flipped translation.
</div>

---
layout: default
---

# Appendix: cross product, normalization, pseudoinverse

<<< ../reference/matrix.js#vector-cross {*}{lines:true,startLine:218,maxHeight:'140px'}
<<< ../reference/matrix.js#vector-normalize {*}{lines:true,startLine:201,maxHeight:'140px'}

<div class="panel text-sm" style="margin-top:0.4em">

`matrix_pseudoinverse` (below) is part of the completed `kineval_matrix.js` STENCIL but **unused by
forward kinematics itself** -- it's what `inverse_kinematics/` (this project's next planned module)
will need to turn a Jacobian into a joint-velocity update.

</div>

<<< ../reference/matrix.js#matrix-pseudoinverse {*}{lines:true,startLine:143,maxHeight:'220px'}

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

<<< ../reference/matrix.js#matrix-copy {*}{lines:true,startLine:36,maxHeight:'140px'}
<<< ../reference/matrix.js#matrix-transpose {*}{lines:true,startLine:70,maxHeight:'140px'}
<<< ../reference/matrix.js#matrix-square-invert {*}{lines:true,startLine:85,maxHeight:'280px'}

---
layout: default
---

# Appendix: the rest of the matrix.js STENCIL

<<< ../reference/matrix.js#generate-identity {*}{lines:true,startLine:229,maxHeight:'120px'}
<<< ../reference/matrix.js#generate-rotation-matrix-x {*}{lines:true,startLine:253,maxHeight:'140px'}
<<< ../reference/matrix.js#generate-rotation-matrix-y {*}{lines:true,startLine:266,maxHeight:'140px'}
<<< ../reference/matrix.js#generate-rotation-matrix-z {*}{lines:true,startLine:279,maxHeight:'140px'}

---
layout: default
---

# Appendix: robot init and the joint hierarchy

<div class="panel text-sm">

A robot description only states each *joint's* parent/child link; `initRobotJoints` builds the
reverse lookup every `traverseFKLink` call needs -- which joints hang off a given link, and which
joint connects a link back to its own parent.

</div>

<<< ../reference/kinematics.js#init-robot-links {*}{lines:true,startLine:75,maxHeight:'140px'}
<<< ../reference/kinematics.js#init-robot-joints {*}{lines:true,startLine:87,maxHeight:'220px'}
<<< ../reference/kinematics.js#init-robot {*}{lines:true,startLine:111,maxHeight:'80px'}

---
layout: default
---

# Appendix: `quaternion_normalize` and `buildFKTransforms`

<<< ../reference/quaternion.js#quaternion-normalize {*}{lines:true,startLine:51,maxHeight:'160px'}
<<< ../reference/kinematics.js#build-fk-transforms {*}{lines:true,startLine:196,maxHeight:'100px'}

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

<<< ../reference/scene.js#make-link-geometry {*}{lines:true,startLine:47,maxHeight:'220px'}
<<< ../reference/scene.js#create-viewer {*}{lines:true,startLine:72,maxHeight:'380px'}

---
layout: default
---

# Appendix: posing and revealing link meshes

<<< ../reference/scene.js#update-robot-meshes {*}{lines:true,startLine:153,maxHeight:'380px'}
<<< ../reference/scene.js#resize-renderer {*}{lines:true,startLine:193,maxHeight:'100px'}

---
layout: default
---

# Appendix: URL parameters and robot selection

<<< ../reference/infrastructure.js#appendix-url-params {*}{lines:true,startLine:36,maxHeight:'420px'}

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

<<< ../reference/infrastructure.js#fk-traversal-order {*}{lines:true,startLine:81,maxHeight:'220px'}
<<< ../reference/infrastructure.js#appendix-animate {*}{lines:true,startLine:214,maxHeight:'180px'}

---
layout: default
---

# Appendix: the HUD panel and joint sliders

<<< ../reference/infrastructure.js#hud-panel {*}{lines:true,startLine:111,maxHeight:'420px'}

---
layout: default
---

# Appendix: `urdf_example`, the branching test robot

<div class="panel text-sm">

Ported from the upstream kineval-stencil's own `robots/robot_urdf_example.js` onto this module's
geometry-as-plain-data convention -- kept specifically because its tree branches (`link1` has two
child joints), unlike the single-chain `mobile_arm`.

</div>

<<< ../reference/robots/urdf_example.js#create-urdf-example-robot {*}{lines:true,startLine:36,maxHeight:'380px'}

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
