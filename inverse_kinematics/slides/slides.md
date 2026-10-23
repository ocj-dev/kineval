---
theme: default
title: Inverse Kinematics and Optimization — KinEval Lab
base: /kineval/inverse_kinematics/
colorSchema: light
info: |
  ## Inverse Kinematics and Optimization
  A KinEval lab-session walkthrough of Jacobian-based inverse kinematics --
  the manipulator Jacobian, error minimization by transpose/pseudoinverse/
  damped least squares, singularities, and P-servo iteration -- built for
  the AutoRob course (autorob.org).
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

# Inverse Kinematics <span class="accent">and Optimization</span>

### The manipulator Jacobian, error minimization, and iterative convergence

<div class="pt-8">
<div class="swatch-bar"><span class="swatch amber" /><span class="swatch indigo" />Endeffector error &amp; joint-velocity correction</div>
</div>

<div class="title-footer">
<div class="nav-hint">Press &rarr; to move forward through the deck</div>
<div class="credit">AutoRob (autorob.org) &#183; Chad Jenkins (ocj@umich.edu) &#183; ocj-dev.github.io/kineval/inverse_kinematics</div>
</div>

<!--
Title slide. This deck complements the AutoRob lab sections: a fully working
KinEval reference implementation of Jacobian-based inverse kinematics --
building a manipulator Jacobian column by column, minimizing endeffector
error by transpose/pseudoinverse/damped least squares, and iterating to
convergence -- walked through step by step. Built from the AutoRob
"Inverse Kinematics: Optimization" lecture, and completing the upstream
kineval-stencil (github.com/autorob/kineval-stencil) inverse-kinematics and
servo-control modules. Picks up directly where forward_kinematics/ left off:
every joint's/link's .matrix this module reads was put there by that
module's buildFKTransforms.
-->

---
layout: default
---

# Forward kinematics, briefly

<div class="panel text-sm">

A quick recap before inverting anything: forward kinematics takes joint angles in and produces a
single, unique endeffector pose out, via one depth-first pass over the kinematic tree, composing
each joint's local transform onto its parent's as it goes (`buildFKTransforms`, this project's
forward-kinematics module). No iteration, no ambiguity -- one set of joint angles, one pose, every
time.

</div>

<div class="panel text-sm" style="margin-top:0.4em">

This deck assumes that machinery already exists and is correct -- every `joint.matrix`/`link.matrix`
read anywhere in this module (`computeManipulatorJacobian`'s `z_i`/`p_i`, `iterateIK`'s `p_ee`) was
put there by that same DFS pass. For the full derivation -- the matrix stack, quaternion composition,
the traversal itself -- see the forward-kinematics deck:
<a href="https://ocj-dev.github.io/kineval/forward_kinematics/" target="_blank">ocj-dev.github.io/kineval/forward_kinematics/</a>.

</div>

---
layout: default
---

# The inverse kinematics problem

<div class="panel text-sm" style="padding:0.55em 0.9em">

Forward kinematics answers "given joint angles, where is the endeffector?" Inverse kinematics asks
the reverse: given a *desired* endeffector pose, find joint angles that produce it -- and loses every
one of forward kinematics' closed-form guarantees:

- **rarely closed-form** -- only special chain geometries (certain 6-DOF arms) admit an algebraic inverse
- **multiple solutions** -- a 2-link planar arm reaching a point can bend its elbow up *or* down
- **zero solutions** -- a target outside the chain's reachable workspace has no solution at all
- **redundancy** -- more joints than task dimensions (mr2's 4-joint arm vs. a 3D target) means
  *infinitely many* solutions

</div>

<div class="panel text-sm" style="margin-top:0.35em; padding:0.55em 0.9em">

**Four realistic families of solutions**
- **Optimization** -- minimize endeffector error by Jacobian-based iteration; what the rest of this
  deck builds
- **Closed-form / algebraic** -- hand-derived per-robot geometric solutions; fast and exact, but not general
- **CCD (cyclic coordinate descent)** -- no Jacobian, adjusts one joint at a time from the endeffector
  back toward the base; this deck's own dedicated slide (just ahead) walks through it
- **FABRIK** (Forward And Backward Reaching Inverse Kinematics) -- a point-based iterative heuristic,
  also no Jacobian (**Aristidou &amp; Lasenby, 2011**, <i>Graphical Models</i>, vol. 73, no. 5, pp.
  243&ndash;260 -- verified against the published record, same discipline this deck's Whitney-1969
  citation uses)

</div>

---
layout: default
---

# A brief history: from resolved-rate control to Jacobian IK

<div class="panel text-sm">

The Jacobian-based view of inverse kinematics -- treat the endeffector's velocity as a linear
function of joint velocities, then invert *that* linear relationship instead of the nonlinear
position map -- traces to **Daniel E. Whitney**'s 1969 paper <i>"Resolved Motion Rate Control of
Manipulators and Human Prostheses"</i> (<i>IEEE Transactions on Man-Machine Systems</i>, vol. 10, no.
2, pp. 47&ndash;53), which introduced **resolved-rate control**: command the endeffector's Cartesian
velocity directly, and let the Jacobian's inverse (or pseudoinverse) resolve it into joint rates.
Every method this deck builds -- transpose, pseudoinverse, damped least squares -- is a different way
of approximately inverting that same linear map, iterated instead of solved once, so it also works
where no clean inverse exists.

</div>

<div class="history-note">
<b>Verified, not assumed</b>
This date and attribution were checked against the published IEEE record before going on this
slide, same discipline the forward-kinematics deck used for its own historical claims (D-H,
Hamilton, Rodrigues) -- the AutoRob lecture itself does not name Whitney, so this slide adds a
verified citation rather than repeating an unverified one from the source material.
</div>

---
layout: default
---

# A non-Jacobian alternative: Cyclic Coordinate Descent

<div class="panel text-sm" style="padding:0.55em 0.9em">

The lecture poses CCD as "maybe there is a simpler approach to IK?", citing **&#91;Wang, Chen 1991&#93;**.
Instead of a Jacobian, CCD adjusts **one joint at a time**, working from the endeffector back toward
the base: rotate the current joint so the endeffector lies as close as possible to the target, then
move to the next joint inward, and repeat the whole sweep until convergence.

</div>

<div class="split-panel" style="margin-top:0.3em; height:auto; flex:0 0 auto">
<div>

<div class="panel" style="padding:0.55em 0.9em">

**Cyclic Coordinate Descent**
- &#43; fast to compute, simple to implement
- &minus; smoothness over time not considered

</div>
</div>
<div>

<div class="panel" style="padding:0.55em 0.9em">

**Jacobian-based methods (this deck)**
- &#43; general transform of velocities/wrenches between frames
- &minus; slower, subject to numerical issues and local minima

</div>
</div>
</div>

<div class="panel text-sm" style="margin-top:0.3em; padding:0.55em 0.9em">

CCD never builds anything like `robot.jacobian` -- it's a genuinely different algorithm family, kept
here as the contrast that makes the rest of this deck's choice (a Jacobian) legible as a choice.

</div>

---
layout: default
---

# The overall algorithmic process

<div class="panel text-sm" style="margin-bottom:0.4em">

Every piece this deck builds slots into one loop: gate on whether an update was requested, build the
Jacobian, compute the error `dx`, solve for a joint-velocity correction `dq` (method-dependent),
scale it by the step length, and accumulate it onto each chain joint's `.control` -- never `.angle`
directly (see this slide's own code quote for why).

</div>

<div class="content-body">
<PseudocodePanel :lines="['robotInverseKinematics(robot, target, eeJoint, eeLocalPos, ikParams, updateRequested):','    if (!updateRequested) return','    iterateIK(robot, target, eeJoint, eeLocalPos, ikParams)','iterateIK(robot, target, eeJoint, eeLocalPos, ikParams):','    p_ee = eeJoint.matrix * eeLocalPos','    robot.dx = target.position - p_ee        -- (+ 3 orientation rows if included)','    robot.jacobian = computeManipulatorJacobian(robot, chainJointNames, eeJoint, eeLocalPos, ...)','    JT = matrix_transpose(robot.jacobian)','    robot.dq = solve(ikParams.method, JT, robot.jacobian, robot.dx)   -- transpose|pseudoinverse|damped','    for each joint i in chainJointNames:','        joint[i].control += ikParams.steplength * robot.dq[i]']" :active-line="-1" />
</div>

---
layout: default
---

# Interactive: drag a target, watch it converge

<div class="panel text-sm" style="margin-bottom:0.4em">

Before building each piece -- the Jacobian, the error term, the solve methods -- watch the whole loop
run end to end. Drag the magenta sphere anywhere in 3D; the arm converges toward it **live**, solved by
`runIkIteration` -- the same TS port of `iterateIK` this deck's later step-through panel calls, not a
separately-tuned approximation for the picture. Switch methods and watch the qualitative difference
this deck's upcoming derivations will explain: `transpose` is cheap but can wander or oscillate near
convergence; `pseudoinverse` is the least-squares-optimal direction but can swing wildly approaching a
near-singular pose; `damped` trades a little accuracy there for staying bounded.

</div>

<div class="content-body">
<IkConvergePanel robot-name="mr2" />
</div>

---
layout: default
---

# `robotInverseKinematics`: the per-frame gate

<<< ../reference/inverse_kinematics.js#robot-inverse-kinematics {*}{lines:true,startLine:63,maxHeight:'260px'}

<div class="panel text-sm" style="margin-top:0.4em">

`updateRequested` stands in for upstream's `kineval.params.update_ik || kineval.params.persist_ik`
globals -- this module's no-global-namespace house style (every function takes `robot` and whatever
else it needs as explicit arguments) pushes that per-frame decision out to the caller
(`infrastructure.js`'s animation loop), gated here with one `if`.

</div>

---
layout: default
---

# Component breakdown

<div class="panel">

0. **Gradient descent** -- the general optimization idea every "solve method" below specializes:
   minimize a cost by repeatedly stepping opposite its gradient
1. **Manipulator Jacobian construction** -- `computeManipulatorJacobian`, one column per chain joint
2. **The error term** -- `robot.dx`, position-only vs. 6D pose with quaternion orientation error
3. **Solve methods** -- transpose, pseudoinverse (left/right), damped least squares
4. **Singularities** -- where the Jacobian loses rank, and why damping survives it
5. **Iteration** -- step length, convergence, and the P servo (`robotArmControllerSetpoint`)
6. **Interactive**: step through a Jacobian build, drag a live-converging target, step through
   `iterateIK` itself
7. **Test cases**: `urdf_example`, `mr2`, and more via `infrastructure.js`'s URL-param harness

</div>

---
layout: default
---

# 0. Gradient descent

<div class="panel text-sm">

Gradient descent is the general-purpose idea underneath every "solve method" this deck builds: to
minimize a cost, repeatedly step **opposite** its gradient (the direction of steepest increase),
scaled by a step size. In one dimension, minimizing a scalar function `f(x)`:

</div>

<div class="panel" style="margin-top:0.4em; font-family:var(--font-mono); font-size:0.85em; text-align:center">

x<sub>n+1</sub> = x<sub>n</sub> &minus; &gamma; f&prime;(x<sub>n</sub>)

</div>

<div class="panel text-sm" style="margin-top:0.4em">

generalizes directly to higher dimensions, replacing the scalar derivative with a gradient vector:

</div>

<div class="panel" style="margin-top:0.4em; font-family:var(--font-mono); font-size:0.85em; text-align:center">

x<sub>n+1</sub> = x<sub>n</sub> &minus; &gamma; &nabla;f(x<sub>n</sub>)

</div>

<div class="panel text-sm" style="margin-top:0.4em">

This is exactly the shape of `iterateIK`'s own update -- `dq = ...; joint.control += steplength * dq`.
This deck's **transpose** method *is* gradient descent on squared endeffector error: this deck's own
appendix derives that directly (cost in `dq`, its derivative, evaluated at `dq = 0`, yielding the
boxed `&gamma;J<sup>T</sup>dx` result by moving opposite the gradient -- see "Appendix: Jacobian
transpose, the full derivation").

</div>

---
layout: default
---

# Interactive: gradient descent on a 1D function

<div class="panel text-sm" style="margin-bottom:0.4em">

Minimizing **f(x) = cos(3&pi;x) / x** on **0 &lt; x &le; 2** (x = 0 is undefined, so the domain stays
strictly positive) by the exact update from the previous slide: **x<sub>n+1</sub> = x<sub>n</sub>
&minus; &gamma;f&prime;(x<sub>n</sub>)**, with **f&prime;(x) = (&minus;3&pi;x&middot;sin(3&pi;x)
&minus; cos(3&pi;x)) / x&sup2;** by the quotient rule (verified against a central finite-difference
check before shipping this panel). The oscillating `cos(3&pi;x)` term gives this function multiple
local minima and maxima on `(0, 2]` -- drag the starting `x` across the domain and watch gradient
descent land in *different* valleys, or fail to settle at all near the steep region close to zero.
That's a real limitation, and the Jacobian-based IK the rest of this deck builds inherits it too:
multiple solutions, local minima, and an outcome that depends on where you start.

</div>

<div class="content-body">
<GradientDescent1DPanel />
</div>

---
layout: default
---

# 1. What a Jacobian column means

<div class="panel text-sm">

Go back to a single rotating point: for a frame spinning at angular velocity **&omega;** about an
axis through its origin, a point **r** fixed in that frame has linear velocity **v = &omega; &times;
r** (pp. 21&ndash;30 of the lecture derive this from the skew-symmetric form, the same **&#91;a&#93;<sub>&times;</sub>**
this project's forward-kinematics deck introduces for Rodrigues). A robot joint is exactly this
picture, localized: rotating joint *i* at unit angular velocity about its own world axis **z<sub>i</sub>**
moves every point downstream of it -- including the endeffector -- at **z<sub>i</sub> &times; r**,
where **r** is the vector from the joint's origin to that point.

</div>

<div class="panel text-sm" style="margin-top:0.4em">

One **column** of the Jacobian is exactly that: "if joint *i* alone spun at unit angular velocity,
how would the endeffector move?" Stack one column per joint, base to endeffector, and `J dq` gives
the endeffector's velocity for *any* combination of joint velocities `dq` -- linear in `dq`, even
though the underlying forward-kinematics map is not.

</div>

---
layout: default
---

# Interactive: Jacobian vector expressions, `urdf_example`

<div class="panel text-sm" style="margin-bottom:0.4em">

Every chain joint's per-column math, all at once, instantiated with `urdf_example`'s actual numbers at
its current pose -- the same **z<sub>i</sub> = R<sub>i</sub> &middot; axis<sub>i</sub>**, **p<sub>i</sub>
= translation(R<sub>i</sub>)**, lever arm **= p<sub>ee</sub> &minus; p<sub>i</sub>**, and column
**J<sub>vi</sub> = z<sub>i</sub> &times; (p<sub>ee</sub> &minus; p<sub>i</sub>)** the next few slides
derive, and the following `computeManipulatorJacobian` build-through panel steps through one joint at
a time. Pick a joint from the dropdown to highlight its axis arrow and lever-arm vector at full
saturation against the rest of the chain, rendered at 60% opacity.

</div>

<div class="content-body">
<JacobianExpressionPanel robot-name="urdf_example" />
</div>

---
layout: default
---

# The geometric Jacobian: structure

<div class="panel text-sm">

A 6&times;N matrix (3 linear rows over 3 angular rows, N joint columns) stacks two 3&times;N blocks:

</div>

<div class="split-panel" style="margin-top:0.4em; height:auto; flex:0 0 auto">
<div>

<div class="panel">

**Column *i*, revolute joint**
- linear: **z<sub>i</sub> &times; (o<sub>n</sub> &minus; o<sub>i</sub>)**
- angular: **z<sub>i</sub>**

</div>
</div>
<div>

<div class="panel">

**Column *i*, prismatic joint**
- linear: **z<sub>i</sub>**
- angular: **0**

</div>
</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em">

**o<sub>n</sub>** is the endeffector's world position, **o<sub>i</sub>** joint *i*'s world origin, **z<sub>i</sub>**
its world rotation axis -- every quantity expressed in the *world* frame (the lecture's own
"Important" box, p. 65). This module's reference implementation only ever builds revolute chains, so
`computeManipulatorJacobian` implements the left column only -- the next slide is that function.

</div>

<div class="history-note">
<b>A one-line clarifying note on z<sub>i</sub></b>
The lecture computes <b>z<sub>i</sub></b> as <b>{k<sub>i</sub>}<sup>w</sup> &minus; {o<sub>i</sub>}<sup>w</sup></b> --
a point one unit along the joint's local axis, transformed by the joint's full homogeneous world
matrix, minus the joint's own world origin. That subtraction is what makes the result a pure
<em>direction</em>: the translation part of the homogeneous transform affects both terms identically
and cancels, leaving just the rotated axis. Correct, but easy to misread as doing something fancier
than "rotate the axis by the joint's world rotation block" -- which is exactly what
`computeManipulatorJacobian`'s own `z_i` computation does directly, without building the extra point.
</div>

---
layout: default
---

# Code: `computeManipulatorJacobian`

<<< ../reference/inverse_kinematics.js#manipulator-jacobian {*}{lines:true,startLine:80,maxHeight:'420px'}

---
layout: default
---

# Interactive: building the Jacobian, `urdf_example`

<div class="content-body">
<JacobianBuildPanel robot-name="urdf_example" />
</div>

---
layout: default
---

# Interactive: building the Jacobian, `mr2`

<div class="content-body">
<JacobianBuildPanel robot-name="mr2" />
</div>

---
layout: default
---

# 2. The error term: `robot.dx`

<div class="panel text-sm">

`iterateIK` computes the endeffector's current world position the same way
`computeManipulatorJacobian` does (`eeJoint.matrix * eeLocalPos`), then subtracts it from the target:
`dx = target.position - p_ee`, a plain 3&times;1 column for position-only IK. With
`ikParams.orientationIncluded` (and a target that actually carries an `.orientation` quaternion),
three more rows are appended, making `dx` 6&times;1 to match a 6&times;N Jacobian.

</div>

<div class="panel text-sm" style="margin-top:0.4em">

Orientation error isn't a simple subtraction -- quaternions don't subtract meaningfully. `iterateIK`
instead takes the "difference" quaternion `q_err = q_target * conj(q_current)` and doubles its
imaginary part: **d_theta &asymp; 2 &middot; vec(q_err)**. This is the standard small-angle axis-angle
approximation -- exact only as the rotation shrinks to zero, which is precisely the regime a
*converging* IK iteration runs in, so the approximation gets better the closer the solve gets.
`rotationMatrixToQuaternion` (extracting `q_current` from `endeffector_joint.matrix`'s rotation
block) and the conjugate helper live in this file rather than `quaternion.js`, since that file
completes only its own four upstream STENCIL functions -- see this deck's appendix.

</div>

---
layout: default
---

# 3. Solve methods: three ways to invert a non-invertible map

<div class="panel text-sm">

`robot.jacobian` is rarely square, so "invert it" isn't literally available -- `iterateIK` instead
picks `dq` by one of three methods, each trading accuracy, cost, and robustness differently. The next
three slides derive each in turn; all three branch inside the same function, shown in full after.

</div>

<div class="split-panel" style="margin-top:0.4em; height:auto; flex:0 0 auto">
<div>

<div class="panel">

**transpose**<br>**dq = J<sup>T</sup> dx**<br>cheap, no matrix inversion, can be slow/oscillatory

</div>
</div>
<div>

<div class="panel">

**pseudoinverse**<br>**dq = J<sup>+</sup> dx**<br>least-squares optimal, costs an inversion, unstable near singularities

</div>
</div>
</div>

<div class="panel" style="margin-top:0.4em">

**damped least squares**<br>**dq = (J<sup>T</sup>J + &lambda;<sup>2</sup>I)<sup>-1</sup> J<sup>T</sup> dx**<br>regularized pseudoinverse, the method built to survive the singularities the other two cannot

</div>

---
layout: default
---

# Interactive: gradient descent in 3D -- a two-variable example

<div class="panel text-sm" style="margin-bottom:0.4em">

The bridge from Step 0's scalar-function gradient descent to the vector-valued manipulator Jacobian
the rest of this deck uses: **g(x,y) = (x&sup2; &minus; y, x + y&sup2;)**, a map from R&sup2; to
R&sup2; with Jacobian **J(x,y) = [[2x, &minus;1], [1, 2y]]** (verified against a central
finite-difference check before shipping this panel). Minimizing **C(x,y) = &#8741;g<sub>target</sub>
&minus; g(x,y)&#8741;&sup2;** by the *same* update shape `iterateIK` uses -- form the real Jacobian at
the current point, compute the error, update via **dq = J<sup>T</sup>e** (transpose method, for
consistency with this deck's gradient-descent framing) -- descends the cost surface below exactly like
the 1D panel descended a curve, just one dimension higher.

</div>

<div class="content-body">
<GradientDescent2DPanel />
</div>

---
layout: default
---

# Code: `iterateIK`, all three methods

<<< ../reference/inverse_kinematics.js#iterate-inverse-kinematics {*}{lines:true,startLine:201,maxHeight:'420px'}

---
layout: default
---

# Code: `matrix_pseudoinverse`

<<< ../reference/matrix.js#matrix-pseudoinverse {*}{lines:true,startLine:151,maxHeight:'260px'}

<div class="panel text-sm" style="margin-top:0.4em">

`matrix_square_invert` (Gauss-Jordan elimination with partial pivoting, shown in this deck's
appendix) is the one dependency-free linear-algebra building block both branches above share.

</div>

---
layout: default
---

# 4. Singularities: where the Jacobian loses rank

<div class="panel text-sm">

A Jacobian column is **z_i &times; (lever arm)** -- a cross product, which is zero exactly when the two
vectors are **parallel**. A chain hits a singularity when enough columns become parallel (or zero)
that `J` loses rank: geometrically, this is the classic *straight arm* configuration, where extending
fully makes further columns collapse onto the same direction and the arm can no longer produce
endeffector velocity along some direction at all, however hard the joints spin.

</div>

<div class="panel text-sm" style="margin-top:0.4em">

Both pseudoinverse forms **divide by something that goes to zero** there (**det(JJ<sup>T</sup>)** or
**det(J<sup>T</sup>J)** &rarr; 0), which is exactly why plain `matrix_pseudoinverse` has no
regularization against it -- a converging arm can genuinely drive itself through this configuration,
and `matrix_square_invert` divides by an exact-zero pivot the same way `1/0` would. Damped least
squares' **+ &lambda;<sup>2</sup>I** is precisely the fix: it keeps the matrix being inverted bounded
away from singular, at the cost of no longer solving the *exact* least-squares problem near there.

</div>

---
layout: default
---

# Interactive: a 2-link arm approaching singularity

<div class="panel text-sm" style="margin-bottom:0.4em">

A planar 2-link arm's position Jacobian has **det(J) = L<sub>1</sub>L<sub>2</sub> sin(&theta;<sub>2</sub>)**
-- exactly zero when the elbow is fully extended (&theta;<sub>2</sub> = 0) or folded back on itself
(&theta;<sub>2</sub> = &plusmn;&pi;). Drive the elbow slider toward either and watch the arm, and the
matrix readout, turn the same lock-red this deck's forward-kinematics sibling used for **gimbal
lock** -- you've already seen this failure mode once, just for rotation composition instead of a
manipulator Jacobian. Both are "two things that should be independent degrees of freedom have
rotated into alignment and a dimension of motion is gone."

</div>

<div class="content-body">
<SingularityPanel />
</div>

---
layout: default
---

# 5. Iteration: step length and convergence

<div class="panel text-sm">

Every method above produces `robot.dq` **without** the step length folded in -- `iterateIK`'s own
`[NOTICE]` comment keeps `robot.dx`/`robot.jacobian`/`robot.dq` as the raw, gradable values, and only
scales by `ikParams.steplength` when actually writing `.control`. Too large a step overshoots; too
small converges slowly. One more defensive guard: if any `dq[i]` comes back non-finite (a genuinely
singular, unregularized configuration under `pseudoinverse`), the whole iteration's `.control` write
is skipped rather than corrupting `.angle` with a `NaN` that would propagate through every later
`buildFKTransforms` call.

</div>

<div class="panel text-sm" style="margin-top:0.4em">

As with every other control source in this project, `iterateIK` writes `.control`, not `.angle`
directly -- `applyControls` (this deck's appendix) is the one place every source (user input, the P
servo below, and IK here alike) gets folded into `.angle`, exactly once per frame, then zeroed.

</div>

---
layout: default
---

# The P servo: `robotArmControllerSetpoint`

<div class="panel text-sm">

A second, simpler controller this module also completes: drive each chain joint's `.control` toward
closing the gap between a **setpoint** and its current `.angle`, a plain proportional (P) servo --
useful on its own, and as the thing a setpoint *source* feeds (a clock sweep, or a canned dance
sequence of poses).

</div>

<<< ../reference/servo_control.js#robot-arm-controller-setpoint {*}{lines:true,startLine:88,maxHeight:'260px'}

---
layout: default
---

# Interactive: step through `iterateIK` itself

<div class="panel text-sm" style="margin-bottom:0.4em">

Distinct from the per-joint Jacobian-build panels earlier: this one picks up **after** a Jacobian
already exists and steps through `iterateIK`'s own remaining body -- the gate check, the error term,
the transpose, the method-dependent solve, and the scaled write to `.control` -- showing
`robot.dx`/`robot.jacobian`/`robot.dq` exactly as `iterateIK` itself computes them. Switch the method
selector mid-walkthrough to see how only the "solve" stage changes.

</div>

<div class="content-body">
<IkStepPanel robot-name="urdf_example" />
</div>

---
layout: default
---

# Test cases

<div class="panel text-sm" style="font-size:0.78em">

Every case is a plain hyperlink, same convention as this project's other modules -- click any URL to
open it in a new tab, running this module's own `inverse_kinematics.html` (see
`infrastructure.js#appendix-url-params` for the full parameter list).

</div>

<div class="split-panel" style="margin-top:0.4em; height:auto; flex:0 0 auto">
<div>

<div class="panel test-case-table" style="font-size:0.6em">

**convergence, different methods**

| Test case | URL |
|---|---|
| `urdf_example`, transpose | <a href="/kineval/inverse_kinematics/reference/inverse_kinematics.html?robot=urdf_example&method=transpose" target="_blank">?robot=urdf_example&method=transpose</a> |
| `urdf_example`, pseudoinverse | <a href="/kineval/inverse_kinematics/reference/inverse_kinematics.html?robot=urdf_example&method=pseudoinverse" target="_blank">?robot=urdf_example&method=pseudoinverse</a> |
| `mr2`, damped, tight target | <a href="/kineval/inverse_kinematics/reference/inverse_kinematics.html?robot=mr2&method=damped&target_x=0.4&target_y=1.15&target_z=0.45" target="_blank">?robot=mr2&method=damped&target_...</a> |
| `mr2`, orientation included | <a href="/kineval/inverse_kinematics/reference/inverse_kinematics.html?robot=mr2&orientation=1" target="_blank">?robot=mr2&orientation=1</a> |
| `fetch`, pseudoinverse (prismatic + revolute) | <a href="/kineval/inverse_kinematics/reference/inverse_kinematics.html?robot=fetch&method=pseudoinverse" target="_blank">?robot=fetch&method=pseudoinverse</a> |

</div>
</div>
<div>

<div class="panel test-case-table" style="font-size:0.6em">

**underactuated: `urdf_example` can't fully converge**

| Test case | URL |
|---|---|
| `urdf_example`, 2 joints vs. a 3D target | <a href="/kineval/inverse_kinematics/reference/inverse_kinematics.html?robot=urdf_example" target="_blank">?robot=urdf_example</a> |
| `mobile_arm`, planar reach | <a href="/kineval/inverse_kinematics/reference/inverse_kinematics.html?robot=mobile_arm" target="_blank">?robot=mobile_arm</a> |

</div>
</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em; font-size:0.72em">

`urdf_example`'s chain (`joint2`, `joint3`) is only **2 revolute joints** against a full 3D position
target -- 2 DOF chasing 3 task dimensions. Smoke-testing the reference implementation confirmed what
the math predicts: `|dx|` settles to a small but **persistent nonzero residual** rather than reaching
zero, because no joint configuration reaches every point in R<sup>3</sup> from a 2-DOF chain. This is
the opposite failure mode from a singularity (too few independent directions available at all,
everywhere, not just at one configuration) -- both are visible, honest limits of the method, not bugs.

`fetch` is this deck's one chain with a **prismatic** joint (`torso_lift_joint`) ahead of eight
revolute ones -- its Jacobian column is just that joint's world axis directly, no cross product
(a prismatic joint's motion already *is* a translation), exercising the branch
`computeManipulatorJacobian` added alongside the all-revolute chains above.

</div>

---
layout: center
class: text-center
---

# That's inverse kinematics and optimization

<div class="panel" style="display:inline-block; text-align:left; margin-top:1em">

- A manipulator Jacobian's columns say "if this joint alone moved, how would the endeffector move?"
- No single inverse exists -- transpose, pseudoinverse (left *and* right), and damped least squares
  are three different approximate answers, each with its own failure mode
- Singularities are where the Jacobian loses rank; damping is how the solver survives them
- The same `.control`-then-`applyControls` pipeline carries IK's correction, exactly like the P servo
  and every other controller in this project

</div>

<div class="title-footer">
<div class="nav-hint">Next: appendix -- infrastructure, scene construction, and robot descriptions</div>
<div class="credit">AutoRob (autorob.org) &#183; Chad Jenkins (ocj@umich.edu) &#183; ocj-dev.github.io/kineval/inverse_kinematics</div>
</div>

---
layout: default
---

# Appendix: orientation-error helpers

<div class="panel text-sm">

`quaternion.js` completes only its own four upstream STENCIL functions (verbatim from the
forward-kinematics module); the conjugate and rotation-matrix-to-quaternion extraction `iterateIK`'s
orientation-error term needs stay local to this file instead.

</div>

<<< ../reference/inverse_kinematics.js#orientation-error-helpers {*}{lines:true,startLine:149,maxHeight:'380px'}

---
layout: default
---

# Appendix: Jacobian transpose, the full derivation

<div class="panel text-sm" style="padding:0.6em 0.9em">

Define squared endeffector error as a cost in `dq`, take its derivative, evaluate at `dq = 0` (the
start of each iteration):

</div>

<div class="panel" style="margin-top:0.3em; padding:0.5em 0.9em; font-family:var(--font-mono); font-size:0.74em; line-height:1.6">

C = &Delta;q<sup>T</sup>J<sup>T</sup>J&Delta;q &minus; 2&Delta;q<sup>T</sup>J<sup>T</sup>&Delta;x + &Delta;x<sup>T</sup>&Delta;x &nbsp;&rarr;&nbsp;
dC/d&Delta;q = 2J<sup>T</sup>J&Delta;q &minus; 2J<sup>T</sup>&Delta;x &nbsp;&rarr;&nbsp;
dC/d&Delta;q |<sub>&Delta;q=0</sub> = &minus;2J<sup>T</sup>&Delta;x

</div>

<div class="panel text-sm" style="margin-top:0.3em; padding:0.5em 0.9em">

To minimize `C`, move opposite the gradient: update by &minus;&gamma; times the derivative above. The
two minus signs cancel, with the sign flip absorbed into the step length &gamma;:

</div>

<div class="panel" style="margin-top:0.3em; padding:0.4em 0.9em; font-family:var(--font-mono); font-size:0.78em; text-align:center">

&minus;&gamma;(&minus;2J<sup>T</sup>&Delta;x) = <b>&gamma;J<sup>T</sup>&Delta;x</b>

</div>

---
layout: default
---

# Jacobian pseudoinverse: left form, right form

<div class="panel text-sm">

The lecture's own IK-specific derivation (pp. 88&ndash;91) sets **dC/d&Delta;q = 0** and solves the
**normal equations** directly: **&Delta;q = (J<sup>T</sup>J)<sup>-1</sup>J<sup>T</sup>&Delta;x** -- the
**left** (tall) pseudoinverse form. A later, separate "Which Pseudoinverse" slide (pp. 96&ndash;97)
does state both forms generically for an N&times;M matrix -- but never reconnects that back to say
*a redundant real robot's Jacobian is usually the wide case*, leaving a gap this slide fills:

</div>

<div class="split-panel" style="margin-top:0.4em; height:auto; flex:0 0 auto">
<div>

<div class="panel">

**Left (tall): rows &ge; cols**<br>J<sup>+</sup> = (J<sup>T</sup>J)<sup>-1</sup>J<sup>T</sup><br>fewer joints than task dimensions (overdetermined)

</div>
</div>
<div>

<div class="panel">

**Right (wide): rows &lt; cols**<br>J<sup>+</sup> = J<sup>T</sup>(JJ<sup>T</sup>)<sup>-1</sup><br>more joints than task dimensions (redundant, underdetermined)

</div>
</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em">

For a wide `J`, **J<sup>T</sup>J** is **singular** (**(J<sup>T</sup>J)<sup>-1</sup>** doesn't exist), so
the left form breaks down exactly where it matters most: `mr2`'s position-only Jacobian is 3 rows
(x,y,z) by 4 columns (4 chain joints) -- wide -- so only the right form actually works, matching this
project's own worksheet finding that **det(J<sup>T</sup>J) &asymp; 0** there. `matrix_pseudoinverse`
(this deck's own "Code" slide, main flow) branches on exactly `rows >= cols` for exactly this reason.

</div>

---
layout: default
---

# Damped least squares: regularizing near singularities

<div class="panel text-sm">

Both forms above divide by something that can be exactly (or nearly) singular. **Damped least
squares** (Levenberg-Marquardt) adds a regularization term to the normal equations before inverting:

</div>

<div class="panel" style="margin-top:0.4em; font-family:var(--font-mono); font-size:0.85em; text-align:center">

&Delta;q = (J<sup>T</sup>J + &lambda;<sup>2</sup>I)<sup>-1</sup> J<sup>T</sup>&Delta;x

</div>

<div class="panel text-sm" style="margin-top:0.4em">

**&lambda;** (`ikParams.damping`) penalizes large joint displacements directly in the cost being
minimized, so **J<sup>T</sup>J + &lambda;<sup>2</sup>I** stays invertible even when **J<sup>T</sup>J**
alone is singular or near it -- the diagonal boost keeps every eigenvalue strictly positive. The
reference implementation adds **&lambda;<sup>2</sup>** directly onto **J<sup>T</sup>J**'s diagonal rather
than building a separate **&lambda;<sup>2</sup>I** matrix and adding it (`matrix.js` has no
scalar-multiply/matrix-add helpers, and `generate_identity` is a fixed 4&times;4, sized for
homogeneous transforms, not this chain's joint count) -- same numeric result, fewer helpers.

</div>

---
layout: default
---

# Setpoint sources: clock movement and dance sequence

<<< ../reference/servo_control.js#setpoint-clock-movement {*}{lines:true,startLine:38,maxHeight:'180px'}
<<< ../reference/servo_control.js#setpoint-dance-sequence {*}{lines:true,startLine:55,maxHeight:'260px'}

---
layout: default
---

# Beyond this lab: optimization considerations

<div class="panel text-sm">

The lecture closes its Jacobian section with a survey of refinements this module doesn't implement,
worth knowing exist:

</div>

<div class="panel">

- **joint limits, multiple endeffectors, preferred poses** -- additional constraints beyond "reach
  the target"
- **null-space optimization** -- apply constraints hierarchically, using any slack a redundant chain
  leaves after satisfying the primary one
- **resolved rate** -- hold endeffector velocity at constant magnitude (Whitney's original framing)
- **stochastic gradient descent** -- update from one randomly chosen Jacobian column at a time
- **downhill simplex** -- derivative-free optimization, no Jacobian needed at all
- **manipulability** -- analyze the Jacobian itself to relate configuration velocity to the scaling
  ellipsoid of resulting endeffector velocities (how "equally capable" the arm is in every direction)

</div>

---
layout: default
---

# Appendix: dependencies on the forward-kinematics module

<div class="panel text-sm" style="padding:0.5em 0.9em">

This module reuses several forward-kinematics-module functions **unmodified**. None of them are
re-derived here -- see the forward-kinematics deck's own slides for their full derivation and code
walkthrough:

</div>

<div class="panel text-sm" style="margin-top:0.3em; padding:0.5em 0.9em">

- **`kinematics.js`**'s `buildFKTransforms` DFS traversal -- every joint's/link's `.matrix` this
  module reads (`computeManipulatorJacobian`'s `z_i`/`p_i`, `iterateIK`'s `p_ee`) is put there by it
- **`quaternion.js`**'s axis-angle&rarr;quaternion&rarr;matrix rotation math -- every joint's own
  variable rotation still goes through this pipeline
- **`matrix.js`**'s generic linear-algebra helpers: `matrix_copy`, `matrix_transpose`,
  `matrix_square_invert`, `generate_identity`, `generate_translation_matrix`, `matrix_invert_affine`,
  `generate_rotation_matrix_{X,Y,Z}`, `vector_cross`, `vector_normalize`

</div>

<div class="panel text-sm" style="margin-top:0.3em; padding:0.5em 0.9em">

Full derivation and code walkthrough for all of the above:
<a href="https://ocj-dev.github.io/kineval/forward_kinematics/" target="_blank">ocj-dev.github.io/kineval/forward_kinematics/</a>.
`matrix_pseudoinverse` (this deck's own "Code" slide, main flow) is **not** on this list -- it's
substantively IK content, not forward-kinematics material, even though it lives in the same
`matrix.js` file.

</div>

---
layout: default
---

# Appendix: `matrix.js` -- what's new here

<<< ../reference/matrix.js#vector-subtract {*}{lines:true,startLine:228,maxHeight:'160px'}

<div class="panel text-sm" style="margin-top:0.4em">

`vector_subtract` is the one addition beyond the forward-kinematics module's verbatim `matrix.js` --
plain elementwise subtraction, added for `computeManipulatorJacobian`'s `p_ee - p_i` and `iterateIK`'s
`target - current` error term, in the same style as every other `vector_*` helper here.

</div>

---
layout: default
---

# Appendix: three.js scene construction

<div class="panel text-sm">

Based directly on the forward-kinematics module's own `scene.js`: every link mesh sits flat in the
scene with `matrixAutoUpdate` disabled, posed every frame straight from `robot.links[name].matrix` --
three.js's own parent-child nesting is deliberately unused, so nothing silently redoes, in the engine,
the matrix-stack composition `kinematics.js` is responsible for. New in this module: meshes default to
**visible immediately** (IK has no DFS traversal to progressively reveal -- the whole arm is always on
screen, converging), and `instantiateThreeRobot` lets `robots/robot_mr2.js`'s upstream-style
`createRobotMr2(THREE)` factory run without any file but this one ever importing three.js.

</div>

<<< ../reference/scene.js#create-viewer {*}{lines:true,startLine:102,maxHeight:'340px'}

---
layout: default
---

# Appendix: posing meshes and the target/endeffector markers

<<< ../reference/scene.js#update-robot-meshes {*}{lines:true,startLine:174,maxHeight:'220px'}
<<< ../reference/scene.js#target-marker {*}{lines:true,startLine:194,maxHeight:'260px'}

---
layout: default
---

# Appendix: URL parameters and robot/chain selection

<div class="panel text-sm">

The chain tables (`CHAIN_JOINT_NAMES`) are the authoritative source this deck's own `robots.ts` port
mirrors by hand -- the same ancestor-only exclusion logic this deck's Jacobian slides walk through for
`urdf_example`'s `joint1` and `mr2`'s `clavicle_left_roll`. `fetch`'s own entry is the longest chain
here -- nine joints, `torso_lift_joint` through `gripper_axis` -- and the only one mixing a prismatic
joint in among revolute ones.

</div>

<<< ../reference/infrastructure.js#appendix-url-params {*}{lines:true,startLine:43,maxHeight:'420px'}

---
layout: default
---

# Appendix: `applyControls` and the HUD/animation loop

<div class="panel text-sm">

`applyControls` is the one place every control source in this project -- user input, the P servo,
`iterateIK` here -- gets folded from `.control` into `.angle`, once per frame, then zeroed. The
animation loop below gates on **elapsed wall time** (`step_ms`) rather than forward kinematics' own
"one DFS node per frame": an IK iteration is a real, perceptible step, so `step_ms` paces convergence
in real time instead of tying it to render speed.

</div>

<<< ../reference/infrastructure.js#apply-controls {*}{lines:true,startLine:184,maxHeight:'180px'}
<<< ../reference/infrastructure.js#appendix-animate {*}{lines:true,startLine:386,maxHeight:'220px'}

---
layout: default
---

# Appendix: robot descriptions -- `urdf_example`, `mobile_arm`, `mr2`, `fetch`

<div class="panel text-sm">

`urdf_example` and `mobile_arm` are ported straight from the forward-kinematics module's own robot
files (same geometry-as-plain-data convention); `robot_mr2.js` is upstream's own right-arm-only
humanoid stencil exercise, ported with its left arm deliberately left incomplete, same as upstream.
`robots/robot_fetch.js` (not shown in full here, same plain-data convention) ports upstream's Fetch
arm verbatim down to joint types/origins/axes, with placeholder box geometry per link since this
buildless reference has no runtime skeleton synthesis from meshes the way the full `kineval/` viewer
does.

</div>

<<< ../reference/robots/urdf_example.js#create-urdf-example-robot {*}{lines:true,startLine:37,maxHeight:'260px'}
<<< ../reference/robots/mobile_arm.urdf.js#create-mobile-arm-robot {*}{lines:true,startLine:48,maxHeight:'260px'}

---
layout: default
---

# Appendix: `robots/robot_mr2.js` in full

<div class="panel text-sm">

No `#region` markers in this file -- it's ported close to verbatim from upstream's own
`robots/robot_mr2.js`, factory-wrapped and with `CubeGeometry`/`applyMatrix` modernized to
`BoxGeometry`/`translate`, every link/joint/origin/axis value otherwise unchanged.

</div>

<<< ../reference/robots/robot_mr2.js {*}{lines:true,maxHeight:'420px'}
