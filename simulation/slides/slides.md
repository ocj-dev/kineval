---
theme: default
title: Pendularm Dynamical Simulation — KinEval Lab
base: /kineval/simulation/
colorSchema: light
info: |
  ## Pendularm Dynamical Simulation
  A KinEval lab-session walkthrough of numerical integration and PID motion control, via a single- and double-link pendulum, built for the AutoRob course (autorob.org).
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

# Pendularm <span class="accent">Dynamical Simulation</span>

### Numerical integration and PID control, one swing at a time

<div class="pt-8">
<div class="swatch-bar"><span class="swatch amber" /><span class="swatch indigo" />Motor torque &amp; gravity torque</div>
</div>

<div class="title-footer">
<div class="nav-hint">Press &rarr; to move forward through the deck</div>
<div class="credit">AutoRob (autorob.org) &#183; Chad Jenkins (ocj@umich.edu) &#183; ocj-dev.github.io/kineval/simulation</div>
</div>

<!--
Title slide. This deck complements the AutoRob lab sections: a fully
working KinEval reference implementation of Pendularm -- the classic
"Assignment 2" single- and double-link pendulum, numerically integrated and
PID-controlled -- walked through step by step. Built from the AutoRob
"Dynamics and Numerical Integration" and "Motion Control and PID" lectures,
and completing the upstream kineval-stencil (github.com/autorob/kineval-stencil)
project_pendularm module.
-->

---
layout: default
---

# A brief history of simulating motion

<div class="panel text-sm">

The equations underneath this whole module go back to **Isaac Newton** (1643-1727) and
**Leonhard Euler** (1707-1783): Newton's second law for translation, and Euler's own extension of
it to rotation, are what let any rigid body's motion be *simulated* rather than solved in closed
form. The notation for what we do with those equations -- **d&theta;/dt**, from
**Gottfried Leibniz** (1646-1716) -- and the energy-based reformulation that makes deriving a
pendulum's equation of motion mechanical rather than ad hoc -- the Euler-Lagrange equation, from
**Joseph-Louis Lagrange** (1736-1813) -- are exactly the tools this module's equations of motion
are built from: **L = KE - PE**, then **d/dt(&part;L/&part;&theta;&#775;) - &part;L/&part;&theta; = &tau;**.

**Explicit Euler integration** -- advance position using the current velocity, then velocity using
the current acceleration -- is the oldest and simplest way to step that equation forward in time,
and (as this deck's own integrator-comparison panel shows) also the least stable. A large family of
alternatives exist for exactly that reason: **Carl Runge** and **Martin Kutta** published their
family of higher-order methods around 1900 -- the 4th-order member used here, "RK4", turns out to
implement Simpson's rule under the hood -- and **Loup Verlet**'s 1967 molecular-dynamics scheme
(already introduced in this project's [cloth_simulation](/kineval/cloth_simulation/) deck) trades
raw accuracy for a *symplectic* structure that keeps a conservative system's energy bounded over
very long runs, something even RK4 cannot always promise.

</div>

---
layout: default
---

# A brief history of motion control

<div class="panel text-sm">

Reaching a target angle -- not just simulating an uncontrolled swing -- is a **control** problem,
and the proportional-integral-derivative (PID) controller used throughout this module is the most
common controller in engineering practice, precisely because each of its three terms has a simple,
separately-motivated origin. The proportional term's intuition is a full three centuries older than
the rest: **Robert Hooke** (1635-1703) and Hooke's Law, **F = -kx**, a restoring force proportional
to displacement -- exactly what **k<sub>p</sub> &times; error** is, here applied to an angle instead
of a spring's stretch. The integral term accumulates *past* error to close the gap a P-only
controller leaves against a constant disturbance (here, gravity); the derivative term estimates the
error's *future* trend to damp overshoot. Manual tuning heuristics for the three gains together go
back to **Ziegler and Nichols**' 1942 method -- still cited today, including in this course's own
lecture, as a starting point worth exploring beyond the "raise k<sub>p</sub>, then
k<sub>d</sub>, then k<sub>i</sub>" order this deck's own PID panel follows.

</div>

---
layout: default
---

# Maximal vs. generalized coordinates, revisited

<div class="panel text-sm">

Pendularm uses **generalized coordinates**: one angle &theta; per degree of freedom, with every
joint constraint satisfied automatically by construction -- there is no way for this pendulum's rod
to stretch or its pivot to separate, because &theta; alone fully determines its shape. This is the
opposite choice from <a href="/kineval/cloth_simulation/">cloth_simulation</a>'s **maximal**
coordinates, where every node carries its own free position and constraints are enforced
approximately, every frame, by relaxation. The AutoRob "Motion Control and PID" lecture makes this
contrast explicit by showing the *same* pendulum re-expressed both ways -- and extends it to a
maximal-coordinate double pendulum and even a maximal-coordinate 5-link Pendularm, each held
together by distance constraints instead of joint angles.

</div>

<div class="history-note mt-2">
<b>Why generalized coordinates, here</b>
For a chain of rigid links with no branching and no need to add/remove connections at runtime
(tearing, cutting), generalized coordinates are the natural choice: fewer state variables, no
constraint-satisfaction pass needed every frame, and the equations of motion -- while more work to
derive up front -- fall directly out of the Euler-Lagrange equation, as this deck's own EOM slides
show. This is also the standard representation used by robot-dynamics libraries in practice, and the
one the general <b>M(q)q&#776; + C(q,q&#775;) = &tau;</b> form (used by this module's double-pendulum
equations of motion) is built for.
</div>

---
layout: default
---

# Reference implementation <a class="accent" href="/kineval/simulation/reference/pendularm.html" target="_blank">(link)</a>

<div class="panel text-xs mt-2">

The reference implementation runs standalone, no build step, straight from `pendularm.html`. Every
run is configured entirely through URL parameters:

| Parameter | Values | Meaning |
|---|---|---|
| `links` | `1`\|`2` (default `1`) | single pendulum, or double pendulum |
| `integrator` | `none`\|`euler`\|`verlet`\|`velocity-verlet`\|`runge-kutta` (default `runge-kutta`) | numerical integration method |
| `dt` | number (default `0.01`) | simulation timestep, in seconds |
| `mass`, `length` | number, or comma pair (default `2.0`) | per-link mass / rod length |
| `gravity` | number (default `9.81`) | gravitational acceleration |
| `angle0` | radians, or comma pair (default `1.5708` i.e. horizontal) | initial angle(s), from the downward vertical |
| `desired` | radians, or comma pair (default `-1.0`) | PID setpoint angle(s) |
| `servo` | `1`\|`0` (default `0`) | whether the PID servo is active on load |
| `kp`, `kd`, `ki` | number, or comma pair | PID gains (default: the known-good `150`/`60`/`4`) |
| `color` | `michigan`\|`red`\|`gray` (default `michigan`) | link/bob color scheme -- Michigan blue links with maize bobs, uniform red (the upstream stencil's own coloring), or uniform dark gray |

```text
pendularm.html?links=2&integrator=euler&dt=0.3
```

Keys `[0-4]` select the integrator, `a`/`d` apply an impulse, `q`/`e` (and `w`/`r` for joint 2)
adjust the desired angle, `c`/`x` toggle the servo, `s` disables it -- carried over from the upstream
stencil. Start/Pause/Reset/Step and every parameter above are also available as on-screen controls.

</div>

---
layout: default
---

# The simulation loop

<div class="grid grid-cols-2 gap-6 mt-2">
<div class="panel text-xs">

**PID control, then dynamics, then integration**

Every frame, if the servo is active, a PID controller turns the angle error into a control torque
(the AutoRob PID lecture's own block diagram: controller &rarr; dynamics &rarr; state feedback).
That torque (zero, if the servo is off) enters the equations of motion alongside gravity to produce
an angular acceleration, which is then integrated forward by exactly one of four interchangeable
methods.

**Integrators are interchangeable**

Every integrator in this module is written once, generically, against an arbitrary-length state
array -- the same four functions drive both the single- and double-pendulum equations of motion.
Swapping `runge-kutta` for `euler` changes nothing about the physics being simulated, only how
accurately (and stably) it's stepped forward in time.

</div>
<div class="panel">

**Simulation pseudocode**

<PseudocodePanel :active-line="-1" :lines="[
  'initialize pendulum state (angle, angle_dot), gravity, integrator, PID gains',
  'every frame:',
  '    if servo is active:',
  '        error = desired - angle',
  '        control = PID(error, accumulated_error, dt)   -- kp*error + ki*sum(error*dt) + kd*d(error)/dt',
  '    else:',
  '        control = 0',
  '    angle_dot_dot = equations_of_motion(angle, angle_dot, control, gravity)',
  '    (angle, angle_dot) = integrate(angle, angle_dot, angle_dot_dot, dt)   -- Euler | Verlet | Velocity Verlet | RK4',
  '    t = t + dt',
  '    render pendulum at new angle',
]" />

</div>
</div>

---
layout: default
---

# Component breakdown

<div class="panel text-sm mt-4">

The next several slides walk the reference implementation
(`kineval/simulation/reference/{dynamics,scene,infrastructure}.js`) one piece at a time, in
pseudocode dependency order:

1. **The equation of motion** -- Newton/Euler-Lagrange, &theta;&#776; = &tau;/(ml&sup2;) - (g/l)sin&theta;
2. **Euler integrator** -- the simplest, and least stable, way to step it forward
3. **Verlet integrator** -- position-only, needs a one-time init step
4. **Velocity Verlet integrator** -- predict with old acceleration, correct with the average
5. **Runge-Kutta 4 integrator** -- four stages, Simpson's-rule weights
6. **Integrator comparison** -- energy conservation, side by side, at a shared timestep
7. **PID controller** -- error, accumulated error, error rate, three gains
8. **Coupled double-pendulum equations of motion** -- the same Euler-Lagrange derivation, twice over
9. **Putting it together** -- `simulateStep()`, the full per-frame loop

Every code snippet is imported directly from the shipped reference files -- not retyped -- with line
numbers matching those files exactly. An appendix at the end of this deck covers the supporting code
(three.js scene construction, URL parsing, keyboard input, the HUD) none of these slides touch
directly.

</div>

---
layout: default
---

# 1. The equation of motion

<div class="content-body">
<EomPanel />
</div>

<!--
Drag the release-angle slider, then step through Accelerate -- the EOM
recomputes theta_dot_dot fresh every step from whatever angle the pendulum
is currently at, driven here by a fixed, stable Velocity Verlet integrator
so the focus stays on the equation itself, not the integration method.
-->

---
layout: default
---

# Newton's second law, for rotation

<div class="panel text-xs mt-2">
A rigid massless rod of length <code>l</code> with a point mass <code>m</code> at its end, pivoting
frictionlessly, plus an idealized motor applying torque &tau; at the pivot. Rotational inertia about
the pivot is <code>I = m*l&sup2;</code>; Newton's second law for rotation,
<code>I*theta_dot_dot = tau_net</code>, with gravity's own restoring torque
<code>-m*g*l*sin(theta)</code>, gives the closed form below -- exactly what the Euler-Lagrange
equation <code>d/dt(&part;L/&part;&theta;&#775;) - &part;L/&part;&theta; = &tau;</code> derives from
<code>L = KE - PE = &frac12;ml&sup2;&theta;&#775;&sup2; - (-mgl cos&theta;)</code>.
</div>

<<< ../reference/dynamics.js#pendulum-acceleration-single {*}{lines:true,startLine:53,maxHeight:'380px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 7 (compute acceleration from the equations of motion).</div>

---
layout: default
---

# 2. Euler integrator

<div class="content-body">
<EulerPanel />
</div>

<!--
Drag dt up and watch Euler's arm (orange) peel away from the RK4 "ghost"
reference (gray) within a few swings -- the exact "naive integrator is
unstable at large dt" lesson the AutoRob dynamics lecture opens with.
-->

---
layout: default
---

# Explicit Euler: the naive integrator

<div class="panel text-xs mt-2">
Advance position using the <i>current</i> velocity, and velocity using the <i>current</i>
acceleration -- both updates read only old-timestep values, so nothing in this step ever corrects
for the error the step itself introduces. Cheap, and visibly unstable once <code>dt</code> is large
relative to the pendulum's own oscillation period.
</div>

<<< ../reference/dynamics.js#integrate-euler {*}{lines:true,startLine:127,maxHeight:'260px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 8 (integrate forward by dt).</div>

---
layout: default
---

# 3. Verlet integrator

<div class="content-body">
<VerletPanel />
</div>

<!--
The faint arm is angle_previous; the solid arm is the current angle. Step
through Accelerate -> Integrate and watch the position-only update rule --
no angle_dot ever enters it, which is exactly what makes this method
symplectic.
-->

---
layout: default
---

# Position-only, symplectic by construction

<div class="panel text-xs mt-2">
The next position is extrapolated from the current and previous positions plus the current
acceleration, via the central-difference identity
<code>theta_dot_dot &asymp; (theta_next - 2*theta + theta_previous) / dt&sup2;</code>, solved for
<code>theta_next</code>. A velocity is still reported (for the HUD/energy displays this deck's own
integrator-comparison panel needs), but it plays no role in the position update itself.
</div>

<<< ../reference/dynamics.js#init-verlet-integrator {*}{lines:true,startLine:145,maxHeight:'160px'}
<<< ../reference/dynamics.js#integrate-verlet {*}{lines:true,startLine:161,maxHeight:'260px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 8 (integrate forward by dt) -- the init step above runs once, before line 0.</div>

---
layout: default
---

# 4. Velocity Verlet integrator

<div class="content-body">
<VelocityVerletPanel />
</div>

<!--
Predict (gray) advances position using the OLD acceleration only; Correct
(orange) re-evaluates acceleration at the new position and averages
old+new to advance velocity. Unlike basic Verlet, angle_dot is explicit
throughout -- just not updated until Correct.
-->

---
layout: default
---

# Predict, then correct

<div class="panel text-xs mt-2">
Splits the step in two: advance position using the <i>old</i> acceleration, then average the old and
newly-recomputed acceleration to advance velocity. This assumes acceleration depends only on
position between the two evaluations -- a good approximation here -- and (like basic Verlet) is
symplectic, where plain Euler is not.
</div>

<<< ../reference/dynamics.js#integrate-velocity-verlet {*}{lines:true,startLine:188,maxHeight:'320px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 8 (integrate forward by dt).</div>

---
layout: default
---

# 5. Runge-Kutta 4 integrator

<div class="content-body">
<Rk4Panel />
</div>

<!--
Each of k1-k4 shows its probe angle as a faint ghost arm before Combine
applies the final weighted average -- k_x{1..4} advance angle, k_v{1..4}
advance angle_dot, matching the lecture's own notation.
-->

---
layout: default
---

# Four stages, Simpson's-rule weights

<div class="panel text-xs mt-2">
Evaluated at the start, twice at the midpoint, and once at the endpoint of the interval, then
combined with weights <code>[1,2,2,1]/6</code> -- the same weights Simpson's rule integrates a
parabola with. RK1 (weight <code>[1]</code>, one stage) is plain Euler; RK4 is dramatically more
accurate per step, though (see the next slide) not automatically more <i>stable</i> over very many
steps.
</div>

<<< ../reference/dynamics.js#integrate-rk4 {*}{lines:true,startLine:210,maxHeight:'420px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 8 (integrate forward by dt).</div>

---
layout: default
---

# 6. Integrator comparison: energy over time

<div class="content-body">
<IntegratorComparisonPanel />
</div>

<!--
All four integrators run in parallel on the same undriven pendulum. Euler's
energy runs away almost immediately. Verlet and Velocity Verlet
(symplectic) hold energy in a small bounded band indefinitely. RK4 -- far
more accurate per step than either -- can still show a slow secular drift
over MANY periods at a coarse dt, since it isn't symplectic. Raise dt to
see all of this happen faster.
-->

---
layout: default
---

# Accurate per-step is not the same as stable forever

<div class="panel text-sm">

This is the single most important lesson a numerical-integration lab can teach, and it only shows up
by watching energy over a <i>long</i> run: RK4's local error is <code>O(dt&#8309;)</code>, far
smaller than Verlet's <code>O(dt&sup2;)</code> per step -- and yet, at a shared coarse
<code>dt</code>, RK4's total energy can drift steadily over hundreds of periods while Verlet's stays
bounded in a small oscillating band the entire time. RK4 is not a <b>symplectic</b> method (it does
not exactly preserve the phase-space volume a Hamiltonian system's true flow preserves); Verlet and
Velocity Verlet are. For a short simulation, RK4's extra accuracy usually wins outright. For a long
one -- exactly the setting real physics engines and orbital mechanics both live in -- the symplectic
methods' bounded-forever error is often worth more than a smaller per-step error that very slowly
accumulates in one direction.

</div>

---
layout: default
---

# 7. PID controller

<div class="content-body">
<PidPanel />
</div>

<!--
Defaults are the known-good kp=150/kd=60/ki=4 (from the TA's own lab
overview slide). Drag kd to 0 to watch the overshoot stop damping; drag ki
to 0 to watch a small steady-state gap remain that gravity never lets a
P+D-only controller close -- reproducing the lecture's own P -> D -> I
tuning order in reverse.
-->

---
layout: default
---

# Current error, past error, future error

<div class="panel text-xs mt-2">
<b>P</b> reacts to the <i>current</i> error like a spring pulling toward the setpoint (Hooke's Law
intuition, <code>F=-kx</code>); <b>I</b> accumulates <i>past</i> error (<code>sum(error*dt)</code>)
to cancel the steady-state gap a P-only controller leaves against a constant disturbance like
gravity; <b>D</b> estimates the error's <i>future</i> trend via its own rate of change, damping
overshoot. Gains are tuned in exactly that order: raise <code>kp</code> until roughly reaching the
setpoint, then <code>kd</code> until the oscillation stops, then <code>ki</code> until the last bit
of steady-state error closes.
</div>

<<< ../reference/dynamics.js#pid-controller {*}{lines:true,startLine:255,maxHeight:'420px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: lines 3-4 (compute error, then PID control torque).</div>

---
layout: default
---

# 8. Coupled double-pendulum equations of motion

<div class="content-body">
<DoublePendulumPanel />
</div>

<!--
Two copies, both undriven, RK4, starting only 0.001 rad apart on joint 2.
Watch them track closely at first, then visibly diverge within a few
swings -- sensitive dependence on initial conditions, the double
pendulum's defining feature.
-->

---
layout: default
---

# The same derivation, twice over -- now coupled

<div class="panel text-xs mt-2">
Deriving the Lagrangian from each mass's <code>(x,y)</code> position and applying Euler-Lagrange to
both <code>theta1</code> and <code>theta2</code> yields the general robot-dynamics form
<code>M(theta)*theta_dot_dot + C(theta,theta_dot) = G(theta) + tau</code> -- a 2&times;2 mass matrix
coupling both joints' accelerations together, plus Coriolis-like and gravity terms. Solved as
<code>theta_dot_dot = M<sup>-1</sup>(tau + G - C)</code>, a direct 2&times;2 matrix inverse, no
iterative solve needed.
</div>

<<< ../reference/dynamics.js#pendulum-acceleration-double {*}{lines:true,startLine:72,maxHeight:'420px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 7 (compute acceleration from the equations of motion) -- the two-joint case.</div>

---
layout: default
---

# 9. Putting it together: `simulateStep()`

<div class="content-body">
<PuttingTogetherPanel />
</div>

<!--
Toggle the servo on/off and step through every phase in order: check the
servo, compute error and PID torque (or zero it out), accelerate,
integrate, advance time -- exactly Jenkins' own dynamics.js#simulate-step,
called once per rendered frame.
-->

---
layout: default
---

# Every function above, called from one place

<div class="panel text-xs mt-2">
Called once per rendered frame, in exactly the order the AutoRob PID lecture's own block diagram
shows: PID control (if active) &rarr; equations of motion &rarr; the selected integrator. Swapping
<code>pendulum.integrator</code> or toggling <code>pendulum.servo_active</code> changes nothing else
about this function -- every branch below is one of the pieces walked through above.
</div>

<<< ../reference/dynamics.js#simulate-step {*}{lines:true,startLine:299,maxHeight:'420px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: lines 2-10, the entire per-frame loop.</div>

---
layout: default
---

# Test cases

<div class="panel text-xs mt-2">

| Case | What it tests | Run it |
|---|---|---|
| Default (RK4, undriven) | Single pendulum released from horizontal, no servo | <a href="/kineval/simulation/reference/pendularm.html" target="_blank">&#9654;</a> |
| Double pendulum | `links=2` -- coupled dynamics, chaotic if released energetically | <a href="/kineval/simulation/reference/pendularm.html?links=2" target="_blank">&#9654;</a> |
| Servo to setpoint | `servo=1&desired=-1.0` -- default kp/kd/ki reaching a setpoint | <a href="/kineval/simulation/reference/pendularm.html?servo=1&desired=-1.0" target="_blank">&#9654;</a> |
| **New:** Large-timestep Euler instability | `integrator=euler&dt=0.3` -- visible energy blow-up within seconds | <a href="/kineval/simulation/reference/pendularm.html?integrator=euler&dt=0.3" target="_blank">&#9654;</a> |
| **New:** Energy-conserving Velocity Verlet | `integrator=velocity-verlet&dt=0.05` -- same scene, bounded energy indefinitely | <a href="/kineval/simulation/reference/pendularm.html?integrator=velocity-verlet&dt=0.05" target="_blank">&#9654;</a> |
| **New:** Known-good PID | `servo=1&kp=150&kd=60&ki=4&desired=-1.0` -- the TA slide's own tuned gains | <a href="/kineval/simulation/reference/pendularm.html?servo=1&kp=150&kd=60&ki=4&desired=-1.0" target="_blank">&#9654;</a> |
| **New:** Undamped P-only servo | `servo=1&kp=150&kd=0&ki=0&desired=-1.0` -- growing oscillation, never settles | <a href="/kineval/simulation/reference/pendularm.html?servo=1&kp=150&kd=0&ki=0&desired=-1.0" target="_blank">&#9654;</a> |
| **New:** Chaotic double pendulum | `links=2&angle0=1.55,1.55&integrator=runge-kutta` -- released near-horizontal | <a href="/kineval/simulation/reference/pendularm.html?links=2&angle0=1.55,1.55&integrator=runge-kutta" target="_blank">&#9654;</a> |
| **New:** Controlled double pendulum | `links=2&servo=1&desired=-1.0,1.0` -- PID on both joints, coupled dynamics and all | <a href="/kineval/simulation/reference/pendularm.html?links=2&servo=1&desired=-1.0,1.0" target="_blank">&#9654;</a> |
| **New:** Balanced at the top | `angle0=3.141592653589793` -- released exactly inverted, undriven: a genuine unstable equilibrium, stays balanced | <a href="/kineval/simulation/reference/pendularm.html?angle0=3.141592653589793" target="_blank">&#9654;</a> |
| **New:** 0.005 rad off balanced | `angle0=3.136592653589793` -- looks balanced at first, then swings through and re-balances on the far side | <a href="/kineval/simulation/reference/pendularm.html?angle0=3.136592653589793" target="_blank">&#9654;</a> |

</div>

<div class="history-note mt-2">
<b>Two cases worth watching longer than they look like they need</b>
Undamped P-only (<code>kd=0</code>): a digitally-sampled P-only controller can genuinely
<i>inject</i> energy over many cycles on a frictionless plant -- a real zero-order-hold sampling
effect, not a numerical artifact, and the clearest argument for why the derivative term exists.
Balanced at the top: <code>theta=pi</code> is just as valid a solution to
<code>theta_dot_dot=-(g/l)*sin(theta)=0</code> as <code>theta=0</code> -- it's simply unstable, so
0.005 rad off is enough for it to visibly fall, swing through the bottom, and re-balance on the far
side after a few seconds.
</div>

---
layout: default
class: text-center
---

<div class="image-slide">
<img src="/images/pendularm-stencil-hero.png" class="hero-image" alt="Screenshot of the original KinEval Pendularm stencil: a red pendulum arm hanging from a gray test stand, mid-swing under Velocity Verlet integration" />
<div class="image-caption">Making progress, and happy hacking</div>
</div>

<div class="credit" style="position:absolute; left:0; right:0; bottom:0.4em;">AutoRob (autorob.org) &#183; Chad Jenkins (ocj@umich.edu) &#183; ocj-dev.github.io/kineval/simulation</div>

<!--
Concluding slide, before the appendix. The image is an authentic
screenshot of the original KinEval project_pendularm stencil this module
completes.
-->

---
layout: default
---

# Appendix: the rest of the reference implementation

<div class="panel text-sm mt-4">

The slides above cover every essential piece of the simulator's physics, but the reference
implementation (`dynamics.js`, `scene.js`, `scene_altdraw.js`, `infrastructure.js`,
`pendularm.html`, `pendularm_altdraw.html`, plus a locally vendored three.js) includes a handful of
supporting pieces none of them touch directly. For completeness, the next few slides cover:

1. **Three.js scene construction** (`scene.js`) -- camera, lighting, ground plane, OrbitControls
2. **The stand and pendulum rig** (`scene.js`) -- a faithful port of the upstream stencil's own geometry
3. **Rendering the pendulum each frame** (`scene.js`) -- the per-frame rotation.y/rotation.z drive
4. **The altdraw variant's rotation axis** (`scene_altdraw.js`) -- why it swings about a different axis
5. **Reading the URL parameters** (`infrastructure.js`) -- how every parameter on the earlier table gets parsed
6. **Building the pendulum state** (`infrastructure.js`) -- turning parsed parameters into simulation state
7. **Keyboard input** (`infrastructure.js`) -- the upstream stencil's own bindings, preserved
8. **The HUD panel** (`infrastructure.js`) -- Start/Pause/Reset/Step and live status
9. **The animation loop** (`infrastructure.js`) -- what calls `simulateStep()` once per frame
10. **A responsive scene** (`infrastructure.js`, `scene.js`) -- sizing the canvas to fill its container

</div>

---
layout: default
---

# Appendix: three.js scene construction

<div class="panel text-xs mt-2">
Camera, lighting, ground plane, and OrbitControls, then the stand and pendulum rig -- a faithful
port of the upstream stencil's own <code>createScene()</code> (see the next slide for the stand and
pendulum geometry itself). A current three.js release is vendored locally as ES modules, in place
of the stencil's pinned non-module r92 build.
</div>

<<< ../reference/scene.js#create-scene {*}{lines:true,startLine:47,maxHeight:'340px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: not part of the per-frame loop -- setup code, run once before line 0.</div>

---
layout: default
---

# Appendix: the stand and pendulum rig

<div class="panel text-xs mt-2">
The 4-leg table stand is built exactly as the stencil's own <code>createScene()</code> does -- same
legs/sidebars/crossbar hierarchy, same one-time <code>rotateOnAxis()</code> calls. The pendulum rig
(<code>pendulum.geom</code>/<code>pendulum_link</code>/<code>pendulum_mass</code>, plus a second
link for the double-pendulum case) is added directly to the scene rather than nested under the
stand -- again matching the original, whose own <code>crossbar.add(pendulum.geom)</code> line is
commented out.
</div>

<<< ../reference/scene.js#build-stand {*}{lines:true,startLine:98,maxHeight:'220px'}
<<< ../reference/scene.js#build-pendulum-rig {*}{lines:true,startLine:136,maxHeight:'220px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: not part of the per-frame loop -- setup code, run once before line 0.</div>

---
layout: default
---

# Appendix: rendering the pendulum each frame

<div class="panel text-xs mt-2">
<code>pendulum.geom.rotation.y = angle[0]</code> drives the whole assembly -- world-space, since
<code>pendulum.geom</code> sits directly on the scene, not nested under the stand -- and for the
double pendulum, <code>pendulum_mass.rotation.z = angle[1]</code> drives the second link on top of
that, exactly the stencil's own two per-frame assignments (its own header comment: "second arm of
pendulum must be in world coordinates, not parent link coordinates"). This also fixes a bug the
reference implementation's own vector arrows had: they were being positioned and pointed every
frame, but never actually added to the scene, so they never rendered at all.
</div>

<<< ../reference/scene.js#update-pendulum-meshes {*}{lines:true,startLine:193,maxHeight:'260px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 10 (render pendulum at new angle).</div>

---
layout: default
---

# Appendix: the altdraw variant's rotation axis

<div class="panel text-xs mt-2">
<code>scene_altdraw.js</code>'s stand places its crossbar and pivot at the <i>same</i> position, both
lying in the X-Y plane at <code>z=0</code>. Swinging by rotating about <b>Z</b> keeps the rod in that
same plane for every angle -- sweeping it straight through the crossbar and post, since the pivot
starts right at the crossbar's own location. Rotating about <b>X</b> instead sweeps the rod through
the Y-Z plane: X stays fixed at the pivot's own position while Z varies, moving the rod away from the
stand's geometry as soon as the angle leaves zero.
</div>

<<< ../reference/scene_altdraw.js#update-pendulum-meshes {*}{lines:true,startLine:168,maxHeight:'320px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 10 (render pendulum at new angle) -- the altdraw variant only.</div>

---
layout: default
---

# Appendix: reading the URL parameters

<div class="panel text-xs mt-2">
Every parameter is optional; defaults reproduce a single pendulum released from horizontal with a
good general-purpose integrator, servo off. This is what makes every "Test cases" link above (and
the reference implementation's own on-screen controls) just a plain hyperlink -- no server or build
step involved.
</div>

<<< ../reference/infrastructure.js#appendix-url-params {*}{lines:true,startLine:31,maxHeight:'380px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: not part of the per-frame loop -- setup code, run once before line 0.</div>

---
layout: default
---

# Appendix: building the pendulum state

<div class="panel text-xs mt-2">
Turns the parsed parameters into the actual simulation state object every function in
<code>dynamics.js</code> reads and writes -- including running the Verlet init step immediately, if
that's the selected integrator, since it needs to happen before the very first frame.
</div>

<<< ../reference/infrastructure.js#create-pendulum {*}{lines:true,startLine:87,maxHeight:'380px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 0 (initialize pendulum state).</div>

---
layout: default
---

# Appendix: keyboard input

<div class="panel text-xs mt-2">
Preserves the upstream stencil's own bindings exactly: <code>[0-4]</code> selects the integrator,
<code>a</code>/<code>d</code> apply a user-force impulse, <code>q</code>/<code>e</code> (and
<code>w</code>/<code>r</code> for joint 2) adjust the desired angle, <code>c</code>/<code>x</code>
toggle the servo, <code>s</code> disables it. A minimal inline replacement for the stencil's vendored
<code>THREEx.KeyboardState</code> helper -- one dependency fewer, same continuous-hold behavior.
</div>

<<< ../reference/infrastructure.js#keyboard-state {*}{lines:true,startLine:131,maxHeight:'380px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: feeds into every line of the loop via user interaction, outside the loop itself.</div>

---
layout: default
---

# Appendix: the HUD panel

<div class="panel text-xs mt-2">
Start/Pause/Reset/Step, an integrator picker, and a servo toggle, plus a live status readout --
upgrading the stencil's plain text-only <code>textbar</code> into real on-screen controls, so every
test case works by URL alone with no keyboard interaction required.
</div>

<<< ../reference/infrastructure.js#hud-panel {*}{lines:true,startLine:172,maxHeight:'420px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: setup code (buttons/controls), wired to lines 0-10 via their event handlers.</div>

---
layout: default
---

# Appendix: the animation loop

<div class="panel text-xs mt-2">
Advances the simulation by exactly one fixed-size timestep per call -- decoupled from wall-clock
frame time, so the physics is deterministic and reproducible regardless of the browser's actual
framerate, exactly as every URL-parameter test case expects.
</div>

<<< ../reference/infrastructure.js#appendix-animate {*}{lines:true,startLine:247,maxHeight:'220px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: line 1 (<code>every frame:</code>) -- this is what actually triggers it.</div>

---
layout: default
---

# Appendix: a responsive scene

<div class="panel text-xs mt-2">
The three.js renderer and camera are kept in sync with the container element's actual rendered size
via a <code>ResizeObserver</code>, rather than a fixed <code>width</code>/<code>height</code>.
</div>

<<< ../reference/infrastructure.js#resize-canvas {*}{lines:true,startLine:265,maxHeight:'140px'}
<<< ../reference/scene.js#resize-renderer {*}{lines:true,startLine:225,maxHeight:'140px'}

<div class="mt-1 text-xs opacity-60">Pseudocode: not part of the per-frame loop -- setup code, run at load and on resize.</div>
