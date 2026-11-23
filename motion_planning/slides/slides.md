---
theme: default
title: RRT-Connect Motion Planning — KinEval Lab
base: /kineval/motion_planning/
colorSchema: light
info: |
  ## RRT-Connect Motion Planning
  A KinEval lab-session walkthrough of sampling-based motion planning and collision detection, built for the AutoRob course (autorob.org).
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

# RRT-Connect <span class="accent">Motion Planning</span>

### Sampling a path through a robot's own configuration space

<div class="pt-6">
<div class="search-bar">🤖&nbsp;&nbsp;Route from <b>this pose</b> to <b>that pose</b>&nbsp;&nbsp;&mdash;&nbsp;&nbsp;without hitting anything</div>
</div>

<div class="title-footer">
<div class="nav-hint">Press → to move forward through the deck</div>
<div class="credit">AutoRob (autorob.org) &#183; Chad Jenkins (ocj@umich.edu) &#183; ocj-dev.github.io/kineval/motion_planning &#183; Michigan Honor License &#183; 2026</div>
</div>

<!--
Title slide. A KinEval lab walkthrough of a fully working RRT-Connect and
collision-detection implementation: motion_planning/reference/ for the 2D
canvas, kineval/ for the full 3D viewer.
-->

---
layout: default
---

# What this lab builds

<div class="split-panel">
<div class="panel text-sm">

Two working planners, in one repository, running **the same algorithm over two different
configuration spaces**:

- **`motion_planning/reference/`** &mdash; the 2D canvas. A configuration is a point in the
  plane, `q = [x, y]`, and collision is a point against a rectangle.
- **`kineval/`** &mdash; the full 3D viewer. A configuration is the robot's own: six base
  degrees of freedom plus every non-fixed joint, and collision is every link's bounding box
  against every obstacle sphere.

Everything on every slide in this deck is quoted from code in this repository, and every
number is measured from running it.

</div>
<div class="panel text-sm">

### Where to click

<div class="link-list">

**[The 2D canvas &rarr;](https://ocj-dev.github.io/kineval/motion_planning/reference/)**
`search_alg=RRT-connect`, five scenes, A\* on the same worlds for comparison

**[The 3D viewer &rarr;](https://ocj-dev.github.io/kineval/kineval/?world=local_minima)**
six robots, fourteen worlds. `Shift+1` stores a goal pose, `1` loads it, `m` plans,
`n`/`b` step through the result

</div>

<div class="history-note mt-3">
<b>Reproducible by construction</b>
Every random choice in this module comes from one seeded generator, and the viewer's HUD
echoes the seed. Every picture in this deck carries the <code>?seed=</code> that produced
it, so you can reproduce it exactly.
</div>

</div>
</div>

---
layout: default
---

# Why not just search a grid?

<div class="split-panel">
<div class="panel text-sm">

The previous lab built **A\***, which searches a grid over the plane. It is resolution-complete
and optimal on that grid, and for a mobile base on a floor it is the right tool.

Now count the cells for a robot arm. The lecture does this directly: a robot with
**21 degrees of freedom**, each discretised to just **10 values**, has $10^{21}$ configurations
— and $n^d$ in general.

<div class="meaning">
<b>The curse of dimensionality</b>
C-space grows exponentially in the number of DOFs. You cannot enumerate it, you cannot store
it, and you certainly cannot search it exhaustively. You can only <em>sample</em> it.
</div>

That is the whole motivation for this lab. Sampling-based planners never build the space. They
build a graph of configurations they happened to try, and ask a collision checker about each
one.

</div>
<div class="panel text-sm">

### What we give up

| | A\* on a grid | RRT-Connect |
|---|---|---|
| Completeness | resolution-complete | **probabilistically** complete |
| Optimality | optimal on the grid | **none** |
| "No path exists" | can report it | **can never report it** |
| Scales to 30 DOF | no | yes |

A\* can tell you a problem is unsolvable, because it can exhaust the graph. RRT-Connect cannot
— it can only keep sampling. That asymmetry comes back at the end of this deck, in the
`world_sealed_enclosure` test case.

</div>
</div>

<!--
roadmaps_rrt p.31-32 is the curse-of-dimensionality slide: d=21 DOFs, n=10
values each, 10^21 configurations.
-->

---
layout: default
---

# A brief history: roadmaps, then trees

<div class="split-panel">
<div class="panel text-sm">

**1996 &mdash; PRM.** Kavraki, Švestka, Latombe & Overmars, *Probabilistic roadmaps for path
planning in high-dimensional configuration spaces*, **IEEE T-RA 12(4):566&ndash;580**. Sample
many configurations, discard the colliding ones, connect neighbours into a roadmap, then
answer *many* queries against it.

**1998 &mdash; RRT.** LaValle, *Rapidly-exploring random trees: a new tool for path planning*,
Iowa State **TR 98-11**; the journal version is LaValle & Kuffner, *Randomized Kinodynamic
Planning*, **IJRR 20(5):378&ndash;400, 2001**. One tree, grown from the start, for a *single*
query.

**2000 &mdash; RRT-Connect.** Kuffner & LaValle, *RRT-Connect: An Efficient Approach to
Single-Query Path Planning*, **ICRA 2000**. Two trees, grown toward each other, with a greedy
connect step. **This is what this lab implements.**

**2010/2011 &mdash; RRT\*.** Karaman & Frazzoli, **RSS 2010** / **IJRR 30(7):846&ndash;894,
2011**. Rewiring makes it asymptotically optimal.

Today these live in **OMPL**, and reach most roboticists through **MoveIt**.

</div>
<div class="panel text-sm">

### Citation corrections

Checked against the course lecture (`autorob_2026_11_roadmaps_rrt.pdf`) for this lab, and
confirmed on the rendered slides:

<div class="history-note">
<b>p.70 — PRM</b>
Reads <code>[Kavraki, Svetska, Latombe, Overmars, 95]</code>. The name is <b>Švestka</b>, and
the canonical year is <b>1996</b> (the earlier circulating version is a 1994 Stanford tech
report). There is no 1995 edition.
</div>

<div class="history-note mt-2">
<b>pp.83–89 — plain RRT</b>
The <code>BUILD_RRT</code> walkthrough is credited to <code>[Kuffner, LaValle 2000]</code>,
which is the <em>RRT-Connect</em> paper. Plain RRT is <b>LaValle 1998</b>. The deck gets it
right from p.95 on, where the same credit sits above <code>RRT_CONNECT_PLANNER</code> and
belongs there.
</div>

</div>
</div>

---
layout: default
---

# Two approaches to a roadmap

<div class="split-panel">
<div class="panel text-sm">

### Multi-query: PRM

Build a roadmap **once**, covering the free space, then answer many start/goal pairs against
it. Worth it when the world is static and you will plan repeatedly.

1. sample *N* configurations at random
2. discard the ones in collision
3. connect each to its neighbours — *k* nearest, or everything within ε — if a straight local
   path between them is collision-free
4. to query: attach start and goal to the roadmap, then **run a graph search** — A\*, Dijkstra,
   BFS. The previous lab's algorithm, reused here.

</div>
<div class="panel text-sm">

### Single-query: RRT, RRT-Connect

Build a tree **for this one problem**, rooted at the configurations you actually care about,
and stop as soon as they connect. Worth it when the world changes, or you plan once.

No roadmap is retained. No graph search is needed at the end: the tree already encodes exactly
one path from its root to each of its vertices, so extracting the answer is a walk up parent
pointers, not a search.

<div class="meaning">
<b>The shared idea</b>
Neither builds C-space. Both only ever ask a collision checker about one configuration, or one
short edge, at a time. That is the single assumption that makes high-dimensional planning
tractable at all.
</div>

</div>
</div>

---
layout: default
---

# RRT-Connect, in four steps

<div class="split-panel">
<div class="panel">

```
RRT_CONNECT(q_init, q_goal)
  T_a.init(q_init);  T_b.init(q_goal)
  for k = 1 to K do
    q_rand ← RANDOM_CONFIG()
    if EXTEND(T_a, q_rand) ≠ Trapped then
      if CONNECT(T_b, q_new) = Reached then
        return PATH(T_a, T_b)
    SWAP(T_a, T_b)
  return Failure
```

<div class="text-xs mt-2">

**EXTEND** takes *one* step from the nearest vertex toward a target, and returns
`Reached` / `Advanced` / `Trapped`.
**CONNECT** calls EXTEND *repeatedly* while it keeps saying `Advanced`.

</div>
</div>
<div class="panel text-sm">

Read the four lines as four decisions:

- **two trees**, one at each end. Neither has a privileged direction.
- **sample uniformly.** `RANDOM_CONFIG()` has no idea where the goal is.
- **extend one tree one step** toward that sample, then **greedily connect the other tree**
  toward wherever the first just arrived.
- **swap**, so both trees grow and neither does all the work.

<div class="meaning">
<b>There is no goal bias</b>
A common presentation of RRT adds "and with probability 0.1, sample the goal". RRT-Connect does
not do that, and does not need to: <b>the second tree is what replaces goal biasing</b>, and it
pulls from both ends at once.
</div>

</div>
</div>

<!--
The pseudocode matches roadmaps_rrt p.95's own RRT_CONNECT_PLANNER figure,
which the deck gets right.
-->

---
layout: default
---

# The whole algorithm, stepping

<div class="content-body">
<RrtConnectPanel scene="construction_detour" :seed="424242" />
</div>

<div class="slide-caption">
Reproduce in the live canvas:
<code>reference/search_canvas.html?search_alg=RRT-connect?planning_scene=scenes/construction_detour.js?seed=424242</code>
</div>

<!--
The trace comes from lib/planning/planner.ts, a function-for-function port
of reference/rrt.js, so the picture is produced by the algorithm being
taught rather than by a separate animation.
-->

---
layout: section
---

# The components

### In the order they depend on one another

---
layout: default
---

# Sampling

<div class="split-panel">
<div class="panel">

<<< ../reference/rrt.js#random-config {*}{lines:true}

</div>
<div class="panel text-sm">

Two lines, and almost all of the interesting content is in what they *do not* do.

- **uniform** over the sampling region. Not Gaussian around the goal, not biased toward
  obstacles, not adaptive.
- the region is the scene's own extent. The canvas worlds are walled in around
  $[-1.8, 5.8]$, so sampling well outside only wastes collision checks.
- `random()` is **not** `Math.random()` — see the next slide.

<div class="meaning">
<b>The sampler is not what finds the passage</b>
A uniform sampler lands inside a narrow gap with probability proportional to the gap's area,
which is tiny. RRT-Connect does not find passages by sampling into them. It finds them because
<code>CONNECT</code> drags a tree through, from a sample on the far side.
</div>

</div>
</div>

---
layout: default
---

# Voronoi bias: what the claim actually is

<div class="content-body">
<VoronoiBiasPanel scene="construction_detour" :seed="7" />
</div>

<div class="slide-caption">
Each shaded cell is the set of samples that would be routed to one tree vertex. Frontier
vertices own enormous cells; interior vertices own slivers.
</div>

<!--
The lecture's p.109 is titled "RRT Probabilistic Completeness" but its two
bullets are both about Voronoi bias and neither states completeness. Both
ideas are here, on separate slides, each under its own name.
-->

---
layout: default
---

# Voronoi bias, precisely

<div class="split-panel">
<div class="panel text-sm">

The sampler is uniform, and yet the tree visibly **reaches outward**. Why?

Every sample is routed to its **nearest** vertex. So the probability that a given vertex is the
one extended equals the measure of its **Voronoi cell**, clipped to the sampling region. A
vertex on the frontier owns a huge cell; a vertex buried among its siblings owns almost nothing.

The tree therefore grows toward unexplored space **as a consequence of uniform sampling and
nearest-neighbour routing**. Nothing is aiming it.

<div class="meaning">
<b>Say it as a distribution, not a guarantee</b>
This biases <em>where expansion happens</em>. It does not promise coverage in any finite number
of iterations, and it is not a property of the sampler.
</div>

</div>
<div class="panel text-sm">

### A correction worth making

The lecture (p.81) says RRT-Connect will

> *"Focus sampling in unexplored areas of C-space and moving towards start/goal"*

Both halves need care:

- sampling is **not** focused — it is uniform. What is biased toward unexplored regions is the
  **expansion**, by the Voronoi argument above. Stating the effect as the mechanism forecloses
  the explanation the deck gives twenty slides later.
- "moving towards start/goal" describes **goal biasing**, which RRT-Connect does not use.

Suggested wording: *sampling is uniform; tree growth is biased toward unexplored regions, and
toward the other tree.*

</div>
</div>

---
layout: default
---

# One seeded generator

<div class="split-panel">
<div class="panel">

<<< ../reference/random.js#random {*}{lines:true}

</div>
<div class="panel text-sm">

Every stochastic choice in this module — the sampler, the shortcut smoother, the world
generators — draws from **one** seeded generator. Nothing calls `Math.random()`.

This is a **reproducibility practice, not an algorithmic idea**, and it is load-bearing anyway:

- *"plan twice from the same start, goal and seed and get the same path"* is an assertion in
  the implementation's test suite. With `Math.random()` it would be untestable.
- every animation in this deck, and every screenshot, replays a specific run.
- `?seed=` on both pages selects it, and the viewer's HUD **echoes the active seed**, so a
  screenshot carries what is needed to reproduce itself.

The algorithm is mulberry32: 32 bits of state, three multiply-xor-shift rounds, period $2^{32}$.
Fine for simulation; **not** suitable for anything else, which is worth saying out loud.

</div>
</div>

---
layout: default
---

# Distance, and why it is not obvious

<div class="split-panel">
<div class="panel">

2D canvas — both components are metres:

<<< ../reference/rrt.js#configuration-distance {*}{lines:true,maxHeight:'130px'}

The viewer — metres *and* radians in one vector:

<<< ../reference/motion_plan.js#configuration-distance {*}{lines:true,maxHeight:'190px'}

</div>
<div class="panel text-sm">

In the plane there is nothing to decide: both components are the same unit, so plain Euclidean
distance is the answer and `configurationDistance` is three lines.

In the viewer, `q` mixes **metres** (base translation, prismatic joints) and **radians**
(base rotation, revolute and continuous joints). Adding their squares is adding metres to
radians: the metric's behaviour then depends on the arbitrary choice of unit.

<div class="meaning">
<b>The unit problem is a property of the C-space, not of the algorithm</b>
It appears the moment a robot has both a base and joints, and it does not exist in the 2D
module at all. That is exactly what the two files being different is <em>for</em>.
</div>

</div>
</div>

---
layout: default
---

# The weighted metric

<div class="split-panel">
<div class="panel">

<<< ../reference/motion_plan.js#configuration-weights {*}{lines:true,maxHeight:'330px'}

</div>
<div class="panel text-sm">

Each DOF is divided by **its own natural range**, so every component of a difference is a
*fraction of that DOF's travel* and the sum is dimensionless.

- translation, by the world's own extent
- base rotation, by $\pi$
- each joint, by its own declared limit

A full sweep of a revolute joint then counts the same as a full traverse of the world. In the
absence of a task-specific cost, that is the only defensible default.

<div class="history-note">
<b>Upstream left this undone</b>
<code>kineval_rrt_connect.js</code>'s <code>compute_configuration_distance</code> has the
weighting branch — and both sides of it compute the same thing, the second multiplied by a
literal <code>1.0</code>. The hook was written and never filled in.
</div>

</div>
</div>

---
layout: default
---

# Angles live on a real line

<div class="split-panel">
<div class="panel">

<<< ../reference/motion_plan.js#configuration-difference {*}{lines:true,maxHeight:'300px'}

</div>
<div class="panel text-sm">

Upstream's standing note at the top of `kineval_rrt_connect.js` asks for the opposite:

> *"Distance computation needs to consider modulo for joint angles"*

**This implementation deliberately does not wrap.** A joint at 0.1 rad and one at 6.3 rad are
**6.2 rad apart, not 0.08**.

- a **revolute joint with limits cannot wrap at all** — its short way round passes through a
  region the joint physically cannot enter.
- under a modulo difference, the edge from yaw $0$ to yaw $2\pi$ has length **zero**, so edge
  validation performs one check and the planner may splice in a full revolution it never
  examined. This was found by verification, not by argument.
- one rule then covers every DOF, and there is no second notion of distance to keep in sync
  with interpolation, sampling and limits.

</div>
</div>

---
layout: default
---

# Nearest neighbour

<div class="split-panel">
<div class="panel">

<<< ../reference/rrt.js#find-nearest-neighbor {*}{lines:true,maxHeight:'400px'}

</div>
<div class="panel text-sm">

A **linear scan**. $O(n)$ per call over a tree that grows every iteration, so quadratic over a
whole run.

It stays linear here on purpose: it is four lines a student can read, the implementation
*counts* its comparisons so the cost is measured rather than argued about, and — as the next
slide shows — collision checking still dominates it by orders of magnitude.

<div class="meaning">
<b>This is the second thing to optimise</b>
A k-d tree turns the scan into $O(\log n)$ and is what every production planner uses. The
lecture's p.79 lists "selecting neighbors (K-NN, kd-tree, cell hashing)" among PRM
considerations — without saying what comes <em>first</em>.
</div>

</div>
</div>

---
layout: default
---

# The cost model, measured

<div class="split-panel">
<div class="panel text-sm">

Neither course deck states it, so this module **measures** it. The planner counts collision
checks and nearest-neighbour comparisons per plan and shows both in the HUD.

Across all **6 robots × 14 worlds**, on the cluttered worlds:

| | collision checks | nn comparisons |
|---|---|---|
| by **count** | 1 125 532 | 62 830 393 |
| cost per call | **28.2 µs** | **0.029 µs** |
| by **time** | **31.8 s** | 1.8 s |

<div class="meaning">
<b>Count the wrong thing and you get the opposite answer</b>
By raw count, nearest-neighbour "wins" 56:1. By time, collision checking wins <b>17:1</b>. A
collision check is a whole forward-kinematics traversal plus a link-by-link,
obstacle-by-obstacle sweep; a comparison is one weighted sum. They differ by ~1000×.
</div>

</div>
<div class="panel text-sm">

### One world, read off the HUD

<img src="/images/viewer_narrow_passage.png" alt="The viewer solving the narrow-passage world, HUD showing 8052 collision checks against 724 nearest-neighbour comparisons" class="shot" />

`world=narrow_passage`, `seed=2024`: **8052** collision checks, **724** nearest-neighbour
comparisons, 33 iterations. Tight worlds push the ratio further toward collision checking,
because most extensions are rejected.

</div>
</div>

---
layout: default
---

# Taking a step

<div class="split-panel">
<div class="panel">

<<< ../reference/rrt.js#new-config {*}{lines:true}

</div>
<div class="panel text-sm">

Move from `q_near` toward `q_target` by at most one step, **clamping** so it never overshoots.

Two details that matter more than they look:

- **the zero guard.** Without it, a sample landing exactly on a vertex divides by zero and
  produces a configuration of `NaN`s that then propagates through the whole tree. Upstream's
  version has no guard.
- **clamping at the target** is what makes `"reached"` mean *arrived*, not *nearby* — which is
  what makes the final edge validated. More on that in two slides.

</div>
</div>

---
layout: default
---

# Step size has to scale

<div class="split-panel">
<div class="panel">

<<< ../reference/motion_plan.js#planner-step-size {*}{lines:true}

</div>
<div class="panel text-sm">

Upstream hardcodes `eps = 0.5`, in raw mixed units. The nine ported worlds make that
untenable — their boundaries run from

$$[-5,-5]\times[5,5] \quad\text{to}\quad [-50,-50]\times[2,2]$$

A constant that explores `world_basic` sensibly crawls across `world_random`, which is **52 m**
across; one that crosses `world_random` steps straight over `world_basic`'s obstacles.

<div class="meaning">
<b>The weighted metric pays for itself here</b>
Because every DOF is normalised to its own range, the C-space diameter is just $\sqrt{d}$. A
step can then be a fixed <em>fraction</em> of that, and scales automatically — with the world,
and with each joint's range.
</div>

</div>
</div>

---
layout: default
---

# Validate the edge, not the endpoint

<div class="split-panel">
<div class="panel">

<<< ../reference/rrt.js#edge-is-valid {*}{lines:true,maxHeight:'330px'}

</div>
<div class="panel text-sm">

Two collision-free endpoints **do not** imply a collision-free edge. A step can pass clean
through a thin obstacle with both ends in free space, and the planner will happily return a
path that walks the robot through a wall.

Upstream tests only the configuration an extension arrives at. That failure is invisible in the
viewer unless you step the plan frame by frame — which is exactly what `n`/`b` are for.

<div class="meaning">
<b>No resolution is provably safe</b>
This is <em>discrete</em> collision checking. A sampler with spacing $d$ will always miss a
collision window narrower than $d$. The honest alternative is continuous collision detection,
which is a different lecture.
</div>

</div>
</div>

---
layout: default
---

# The subtlest bug in the module

<div class="split-panel">
<div class="panel text-sm">

In the viewer, edge resolution is set in **metres** — and the first version set it in
configuration-space distance instead, which looks right and tunnels.

The weighted metric divides base translation by the **whole world span**. In a 21 m world, a
weighted edge length of 0.13 — one step — is **a full metre of base motion**. Four samples
along it steps the robot 0.25 m at a time past 0.6 m spheres.

<div class="history-note">
<b>How it surfaced</b>
A plan escaped <code>world_sealed_enclosure</code> — a world with no exit by construction.
Re-checking the offending edge at 100× the planner's resolution found <b>324 colliding
configurations</b> along a step the planner had called free.
</div>

The fix derives the spacing from the feature that can actually be tunnelled through: the
**smallest obstacle radius in the loaded world**.

</div>
<div class="panel">

<<< ../reference/motion_plan.js#workspace-motion-bound {*}{lines:true,maxHeight:'340px'}

</div>
</div>

<!--
The general lesson: a metric chosen to make a planner's SEARCH behave well
is not automatically the right metric for VALIDATING what it found. One is
about exploration, the other about geometry.
-->

---
layout: default
---

# EXTEND

<div class="split-panel">
<div class="panel">

<<< ../reference/rrt.js#extend-rrt {*}{lines:true,maxHeight:'340px'}

</div>
<div class="panel text-sm">

One step of the tree toward `q`, with the paper's own three-valued result:

| | |
|---|---|
| `reached` | the new vertex **is** `q` |
| `advanced` | a step was taken, `q` is still further on |
| `trapped` | blocked; nothing was added |

<div class="meaning">
<b>"reached" must mean arrived, not nearby</b>
The obvious version — "reached when <code>q</code> is within one step" — leaves the final
sub-step gap <b>unvalidated</b>. In RRT-Connect that gap is precisely the junction between the
two trees, so the planner splices two halves together across an edge nothing ever checked.
Because <code>newConfig</code> clamps, requiring exact arrival costs at most one extra
extension and makes the whole path validated by construction.
</div>

</div>
</div>

---
layout: default
---

# CONNECT — the greedy half

<div class="split-panel">
<div class="panel">

<<< ../reference/rrt.js#connect-rrt {*}{lines:true}

<div class="text-xs mt-2">

Four lines. **This is the "Connect" in RRT-Connect**, and the entire difference between it and
bidirectional RRT.

</div>
</div>
<div class="panel text-sm">

A uniform sampler almost never lands *inside* a narrow gap. What happens instead is that one
sample lands **beyond** it — and this loop drags the tree all the way through in a single
iteration, because each extension keeps returning `advanced`.

Replace `while` with a single `extendRRT` call and the planner becomes bidirectional RRT. That
one-line change is the next slide.

<div class="history-note">
<b>Why greedy is safe here</b>
Greediness costs nothing in correctness: every extension inside the loop validates its own edge
exactly as before. It can only ever stop early (<code>trapped</code>) or arrive
(<code>reached</code>).
</div>

</div>
</div>

---
layout: default
---

# One line, three planners

<div class="content-body">
<ExtendConnectPanel scene="spiral" :seed="909" :iterations="160" />
</div>

<div class="slide-caption">
Same scene, same seed, same 160-iteration budget. Only the algorithm differs: at 160
iterations only the greedy <code>CONNECT</code> has solved it, at iteration 124. Drag the
slider &mdash; <code>scene=spiral</code>, <code>seed=909</code>.
</div>

---
layout: default
---

# Two trees, and the bridge between them

<div class="split-panel">
<div class="panel">

<<< ../reference/rrt.js#iterate-rrt-connect {*}{lines:true,maxHeight:'350px'}

</div>
<div class="panel text-sm">

The two halves are joined at the meeting point. Each `parentPath` runs **root → meeting
point**, so one of them is reversed before concatenation.

`a_is_start_tree` is what remembers which tree is still rooted at `q_init` after however many
swaps — without it the halves concatenate in the wrong order and the drawn path runs
goal-to-start through the middle.

<div class="meaning">
<b>Swapping is not cosmetic</b>
It keeps the two trees balanced. Without it one tree does all the work and the algorithm
degenerates back toward single-tree RRT — which the previous slide's panel shows is markedly
worse on these scenes.
</div>

</div>
</div>

---
layout: default
---

# Extracting the path

<div class="split-panel">
<div class="panel">

<<< ../reference/rrt.js#parent-path {*}{lines:true}

</div>
<div class="panel text-sm">

A tree has **exactly one** path from its root to any vertex. There is nothing to search for:
walk parent pointers and reverse.

<div class="history-note">
<b>Upstream searches anyway — and the search is broken</b>
<code>kineval_rrt_connect.js#find_path</code> runs a recursive depth-first search to re-derive
a path the tree already knows. Worse, its visited-flag reset never executes: it loops over
<code>T.length</code> on an object that has no <code>length</code>, so the condition is
<code>0 &lt; undefined</code>. It works only because vertices are fresh and
<code>undefined</code> is falsy — until a second plan reuses a tree.
</div>

The `dfsPath` name is kept because that is what the stencil asks students for.

</div>
</div>

---
layout: default
---

# Smoothing

<div class="split-panel">
<div class="panel">

<<< ../reference/rrt.js#smooth-path {*}{lines:true,maxHeight:'330px'}

</div>
<div class="panel text-sm">

Pick two configurations on the path at random; if the straight edge between them is free,
delete everything in between. Repeat.

- it **cannot invalidate a path** — every accepted shortcut is validated by the same
  `edgeIsValid` the planner uses. The implementation's tests assert both that the smoothed path
  passes every check the raw one does, and that it is never longer.
- it **cannot make a path optimal**. It removes detours; it cannot move a path to the other
  side of an obstacle.

<div class="meaning">
<b>A path that went left stays left</b>
Shortcutting is a local improvement within one homotopy class. That is exactly the gap RRT\*
closes and smoothing does not.
</div>

</div>
</div>

---
layout: default
---

# What is guaranteed, and what is not

<div class="split-panel">
<div class="panel text-sm">

### Probabilistic completeness

> If a solution exists, the probability that the planner finds one tends to **1** as the number
> of samples tends to infinity.

Read the quantifiers carefully:

- **if a solution exists.** Nothing is promised when none does — the planner will sample
  forever, because it has no way to prove impossibility.
- **in the limit.** No bound at any finite number of iterations.
- **nothing about quality.** RRT and RRT-Connect are **not** asymptotically optimal.

RRT\* *is* asymptotically optimal: its `choose parent` and `rewire` steps drive path cost toward
the optimum as sampling continues. That is a different guarantee, and it costs more per
iteration.

</div>
<div class="panel text-sm">

### A gap in the lecture

Page 109 is titled **"RRT Probabilistic Completeness"**. Its two bullets are:

> *Probability a vertex is selected for extension is proportional to its area in Voronoi
> diagram* &mdash; *RRTs converge to a uniform coverage of C-space as the number of samples
> increases*

Both are about **Voronoi bias**. Neither is a completeness claim. A student reading that slide
learns Voronoi bias and is told its name is probabilistic completeness.

<div class="history-note">
<b>And RRT* is absent</b>
Across 124 planning slides and 145 collision slides, RRT\* appears exactly once — as a
grad-section assignment line — and asymptotic optimality is never defined. Since the
completeness/optimality conflation is the single most common misconception about RRT, it is
worth its own slide.
</div>

</div>
</div>

---
layout: default
---

# RRT\*, for contrast

<div class="split-panel">
<div class="panel">

<<< ../reference/rrt.js#iterate-rrt-star {*}{lines:true,maxHeight:'350px'}

</div>
<div class="panel text-sm">

Two steps buy the optimality, and both are visible above:

- **choose parent** — the new vertex attaches not to its nearest neighbour but to whichever
  nearby vertex gives it the lowest cost-from-root.
- **rewire** — every nearby vertex then checks whether routing *through* the new vertex would
  lower its own cost, and re-parents if so.

Rewiring is why RRT\* keeps improving a path it has already found, and it is also why RRT\* here
is **single-tree**: the cost-from-root bookkeeping has no natural meaning in a tree grown
backward from the goal. That is the honest reason there is no "RRT\*-Connect" in this file.

</div>
</div>

<!--
Implemented in reference/rrt.js and reachable live via
search_canvas.html?search_alg=RRT-star.
-->

---
layout: default
---

# A\* and RRT-Connect, same worlds

<div class="content-body">
<AstarVsRrtPanel scene="spiral" :seed="424242" />
</div>

<div class="slide-caption">
The five scenes the pathfinding deck already taught. Both planners here are ports of the code
carried into <code>motion_planning/reference/</code> — <code>graph_search.js</code> and
<code>rrt.js</code> — run on the same obstacle list, start and goal.
</div>

---
layout: default
---

# Reading that comparison

<div class="split-panel">
<div class="panel text-sm">

**A\*** expands a frontier outward from the start, cell by cell, and returns the shortest path
on its grid. The explored set is visibly a *region*. Its cost is bounded by the number of
cells, which is fixed by `eps` — and which is exactly what explodes in higher dimensions.

**RRT-Connect** scatters. Its explored set is a pair of spindly trees built from a few dozen
vertices rather than a few thousand cells, and its cost does not depend on a grid at all.

<div class="meaning">
<b>Read the two path lengths carefully</b>
On <code>spiral</code>, A\* returns <b>9.70</b> and RRT-Connect <b>9.57</b> — the sampling
planner's path is <em>shorter</em>. That does not make it better, and it does not make A\*
wrong: A\* is optimal <b>on its grid</b>, and this grid is four-connected, so it cannot move
diagonally and pays a Manhattan penalty a continuous-space path never incurs. Optimality is
always relative to the space you searched.
</div>

</div>
<div class="panel text-sm">

### Worth trying in the panel

- **`spiral`** — the corridor forces A\* to expand nearly everything, and gives RRT-Connect its
  cleanest win in explored-set size.
- **`cul_de_sac`** — a trap that costs greedy search dearly and that neither of these two
  notices much.
- **`downtown_gridlock`** — many short detours, and the grid penalty is smallest here because
  the obstacles are axis-aligned anyway.

<div class="meaning">
<b>The reason this lab exists</b>
On the plane, A\* is the better tool. At 11 degrees of freedom — let alone the crawler's 30 —
the grid it needs cannot be built at all, and RRT-Connect's cost barely notices the extra
dimensions.
</div>

<div class="history-note">
<b>A real defect in the defaults</b>
<code>search_canvas.html</code>'s own default goal, <code>q_goal = [4,4]</code>, is
<b>inside an obstacle</b> in <code>cul_de_sac</code>. RRT-Connect roots its second tree
<em>at the goal</em>, so it grows happily from inside a wall and returns a path ending there —
silently. <code>rrt.js#queryIsValid</code> now refuses the query instead.
</div>

</div>
</div>

---
layout: default
---

# Refusing an impossible query

<div class="split-panel">
<div class="panel">

<<< ../reference/rrt.js#query-is-valid {*}{lines:true,maxHeight:'400px'}

</div>
<div class="panel text-sm">

Two point-in-rectangle tests, every iteration. They cost nothing next to an extension, and they
turn the worst failure mode in the module — a confidently-drawn path that ends inside a wall —
into a sentence in the console.

<div class="meaning">
<b>Why every iteration, not just the first</b>
Gating on <code>search_iter_count === 0</code> misses the case entirely whenever anything has
already stepped the search — including the page's own animation loop, which starts on load.
That bug was in the first version of this function.
</div>

The viewer does the same thing at a different granularity: `robotRRTPlannerInit` checks both
ends once, up front, and reports *which link hit what*.

</div>
</div>

---
layout: section
---

# Collision detection

### The function that dominates everything above

---
layout: default
---

# Broad phase, narrow phase

<div class="split-panel">
<div class="panel text-sm">

The collision lecture builds a full **exact** triangle-triangle intersection test — Möller 1997
— over about sixty slides: plane equations, signed distances, the intersection line, interval
overlap, a coplanar fallback to 2D segment intersection by Cramer's rule.

Then it asks: *how many triangle tests must be performed? Can we reduce them?* — and answers
with **bounding volumes** and **sweep and prune**.

That is the broad-phase / narrow-phase split:

- **broad phase** — cheap, conservative, rejects most pairs. Bounding volumes, spatial hashing,
  sweep and prune.
- **narrow phase** — exact, expensive, run only on what survives.

</div>
<div class="panel text-sm">

<div class="meaning">
<b>KinEval stops after the broad phase</b>
From the assignment stencil onward, the bounding-volume approximation <em>is</em> the final
verdict. The triangle test built over sixty slides is never called.
</div>

That is a defensible teaching choice — a min/max comparison a student can read, in a course
whose point is the planner — **as long as it is labelled**, which the lecture never does. The
terms "broad phase" and "narrow phase" appear nowhere in 145 slides.

So, labelled: **the test on the next slide is a broad-phase test being used as the final
answer.** Everything that follows from that — including a deliberate class of false positives —
is a consequence of this choice, not a bug in the code.

</div>
</div>

---
layout: default
---

# The test

<div class="split-panel">
<div class="panel">

<<< ../reference/collision.js#link-sphere-collision {*}{lines:true,maxHeight:'340px'}

</div>
<div class="panel text-sm">

1. transform the obstacle sphere's **centre** into the link's own frame, by
   `matrix_invert_affine(link.xform)`
2. compare it against the link's bounding box **inflated by the sphere radius on every axis**

A pure min/max comparison. **No square roots, no distance computation anywhere.**

<div class="meaning">
<b>Two ways to write one test</b>
The lecture writes it as separation — <code>loc_x + radius &lt; x_min?</code> — and upstream
writes it as containment in an inflated box. They are the same arithmetic: one is the negation
of the other.
</div>

This is the function whose call count dominates planning time, so the inner loop allocates
nothing beyond the transformed centre.

</div>
</div>

---
layout: default
---

# Why the link frame

<div class="split-panel">
<div class="panel">

<<< ../reference/collision.js#pose-is-collision {*}{lines:true,maxHeight:'345px'}

</div>
<div class="panel text-sm">

The inverse is hoisted **out of the obstacle loop** — one per link, not one per obstacle.

<div class="meaning">
<b>Move the sphere, not the box</b>
A sphere is <b>rotation invariant</b>; a box is not. Transforming the sphere into the link's
frame means the box is axis-aligned there no matter how the link is rotated in the world.
</div>

The alternative — testing in the world frame — would force re-axis-aligning the box over its
transformed corners, and would be **looser still**. That is the alternative this code
*avoids*; the implementation's rotation-invariance test exists to prove it does not do it.

`matrix_invert_affine` exploits rigidity (transpose the rotation, negate the rotated
translation). Upstream calls a general 4×4 `numeric.inv` here — in the innermost loop of the
function that dominates planning time — with the affine version sitting right there, commented
out.

</div>
</div>

---
layout: default
---

# Conservatism, visualised

<div class="content-body">
<ConservatismPanel />
</div>

<div class="slide-caption">
Drag the obstacle. Shaded: where the inflated box says <em>collision</em> and the exact
sphere&ndash;box test says <em>free</em>. The exact test is implemented <b>in this panel only</b>.
</div>

---
layout: default
---

# The false positive is the lesson

<div class="split-panel">
<div class="panel text-sm">

Inflating a box by $r$ on all three axes replaces the sphere with a **box** near the corners. So
the test reports collisions that are not geometrically real.

The exact refinement is four lines: clamp the centre to the box, compare squared distance to
$r^2$. It is **deliberately absent** from `collision.js` — not even behind a flag — and lives
only in the panel on the previous slide.

<div class="history-note">
<b>The lecture already owns this</b>
Collision slides 131&ndash;136 walk a sphere to a corner, ask "Is this obstacle in collision?",
annotate <b>"True separating axis not tested"</b>, show all four comparisons firing anyway, and
close with <b>"umm… take comp geom"</b>. The false positive is taught, not overlooked.
</div>

</div>
<div class="panel text-sm">

### A test that asserts the bug

The implementation contains a unit test called *corner conservatism (deliberate false
positive)*. It places a sphere centre at $(0.8r, 0.8r, 0.8r)$ beyond a box corner — a true
distance of $\sqrt{3}\cdot 0.8r = 1.39r$, so the real sphere is **clear** — and asserts that
collision **is** reported.

<div class="meaning">
<b>If someone "finishes" the exact test, that test fails</b>
Which is the point. The approximation is a documented teaching choice, and the test suite is
what keeps it from being silently corrected into something the slides no longer describe.
</div>

Run over all 6 robots × 14 worlds, so it holds for every robot in the course.

</div>
</div>

---
layout: default
---

# Bounding boxes, derived not authored

<div class="split-panel">
<div class="panel">

<<< ../reference/collision.js#cache-link-bounding-boxes {*}{lines:true,maxHeight:'345px'}

</div>
<div class="panel text-sm">

`links_geom[linkName]` is a `THREE.BufferGeometry` that the robot files have **already
`.translate()`-ed into the link frame** — so its own bounding box *is* the link-frame bounding
box. No hand-entered numbers, and the `.translate()` offsets come out right for free.

Links with no geometry — fetch, baxter and sawyer are ported without their 20&ndash;35 MB meshes
— read back the skeleton `scene.js` already synthesised for them, so the box always matches
what is on screen.

<div class="meaning">
<b>AABB in which frame?</b>
The lecture defines an AABB as world-frame and an OBB as object-frame, then says "AABB link
tested in link frame". By its own definition <code>link.bbox</code> seen from the world is an
<em>OBB</em> — and KinEval never needs the Separating Axis Theorem machinery that usually
implies, because it moves the sphere instead.
</div>

</div>
</div>

---
layout: default
---

# The boundary test is a different thing

<div class="split-panel">
<div class="panel">

<<< ../reference/collision.js#link-boundary-violation {*}{lines:true,maxHeight:'330px'}

</div>
<div class="panel text-sm">

World boundaries are `[[xmin,0,zmin],[xmax,0,zmax]]` — a ground-plane extent with **no ceiling
and no floor**. So a robot lifted straight up in $+y$ never violates it. Height is governed by
obstacles, which are tested in full 3D.

<div class="history-note">
<b>A comment that is wrong in all nine world files</b>
Every upstream world carries <code>// collision only checked in x-z plane</code>. That is true
of <em>this</em> function and <b>false</b> of obstacle collision. <code>world_y.js</code> — with
its overhead gate at <code>y = 5.0</code> — exists to exercise the 3D case, so the comment
contradicts the file it sits in. All nine ports fix it.
</div>

This version also tests **every link's bounding-box corners**, where upstream tests only the
base *origin* — so upstream lets a robot drive until its arm is well past the wall.

</div>
</div>

---
layout: default
---

# Pure versus side-effecting

<div class="split-panel">
<div class="panel">

Rendering FK writes as it walks:

```js
// kineval/kinematics.js:117-119
function traverseFKLink(robot, linkName, xform) {
    var link = robot.links[linkName];
    link.xform = xform;          // ← side effect
```

Planning FK returns, and writes nothing:

<<< ../reference/collision.js#collision-forward-kinematics {*}{lines:true,maxHeight:'200px'}

</div>
<div class="panel text-sm">

Writing `link.xform` is exactly right for **rendering** — the viewer wants one current pose, and
`scene.js` reads it straight back out. It is fatal for **planning**, where every question is
about a configuration the robot is *not* in. Calling it to test a candidate would destroy the
pose on screen, and a planner evaluating thousands of candidates per second would make the robot
flicker through its own search.

<div class="meaning">
<b>A design property you can test</b>
Snapshot every <code>link.xform</code>, run a collision query on a different configuration,
compare. The implementation asserts exactly that, for every robot and every world — and it is
why the planner can evaluate thousands of hypothetical poses without the robot twitching.
</div>

</div>
</div>

---
layout: section
---

# Configuration space

### Where the viewer's `q` comes from

---
layout: default
---

# Workspace is not configuration space

<div class="content-body">
<ConfigSpacePanel />
</div>

<div class="slide-caption">
A planar 2R arm. One disc in the workspace becomes a complicated, disconnected region in
C-space &mdash; which nobody computes in practice. A planner only ever asks about one point in
it at a time.
</div>

---
layout: default
---

# The C-space of a 2R arm is a torus

<div class="split-panel">
<div class="panel text-sm">

Two continuous revolute joints, each living on a circle $S^1$. The configuration space is the
product:

$$Q = S^1 \times S^1 = T^2$$

the **2-torus** — the square $[0,2\pi)^2$ with *both* pairs of opposite edges glued. Walk off
the right edge of the panel and you reappear on the left.

<div class="history-note">
<b>Two errors on this point in the lecture</b>
<b>p.39</b> captions a picture of a <em>sphere</em> with <code>S¹ × S¹ = S² when torus axis on
surface</code>. A torus is not a sphere: different genus (1 vs 0), different Euler
characteristic (0 vs 2). No choice of axis changes that. The deck contradicts itself —
<b>p.36</b> says <code>S¹ × S¹ ≠ S²</code> and <b>p.40</b> correctly gives
<code>T² = S¹ × S¹</code>.
</div>

</div>
<div class="panel text-sm">

<div class="history-note">
<b>And p.43</b>
<code>SO(3) = S¹ × S¹ × S¹</code>. That product is the 3-torus $T^3$. SO(3) is diffeomorphic to
$\mathbb{RP}^3$ — the real projective 3-space, $S^3$ with antipodal points identified — and is
<em>not</em> homeomorphic to $T^3$: their fundamental groups are $\mathbb{Z}/2$ and
$\mathbb{Z}^3$. Euler angles give a $T^3$ <em>coordinate chart</em> on SO(3), and that chart has
singularities — gimbal lock is where it degenerates.
</div>

<div class="meaning">
<b>Why this matters for a planner, not just for topology</b>
The shape of C-space is what tells you whether a metric may wrap. This implementation decides
that angles <em>do not</em> wrap — so it is working on a covering space of the torus, the plane
$\mathbb{R}^2$, where "turn the long way round" is a real and distinct motion.
</div>

</div>
</div>

---
layout: default
---

# The viewer's q vector

<div class="split-panel">
<div class="panel">

<<< ../reference/collision.js#build-configuration-index {*}{lines:true,maxHeight:'340px'}

</div>
<div class="panel text-sm">

$$q = [\,x,\ y,\ z,\ \text{roll},\ \text{pitch},\ \text{yaw},\ \theta_1 \ldots \theta_n\,]$$

`q_names` (joint name → index) and `q_index` (index → joint name) are the **single source of
truth** for that layout. Nothing else in the planner is allowed to assume an ordering.

**No robot is named anywhere in `collision.js` or `motion_plan.js`.** The layout is derived from
whatever `ROBOT_FACTORIES` hands over, and every test loops over the cross product of registered
robots and worlds — so adding a seventh robot extends coverage with no code change.

| robot | DOFs in `q` |
|---|---|
| `urdf_example` | 9 |
| `mr2` | 11 |
| `sawyer` | 14 |
| `crawler` | **30** |

</div>
</div>

---
layout: default
---

# Fixed joints are not degrees of freedom

<div class="split-panel">
<div class="panel text-sm">

Upstream includes **every** joint in `q`, fixed ones among them. Fetch has five fixed joints out
of twenty.

That gives each one a dimension whose value can never matter. It wastes sampler effort in
proportion to how many there are, and — worse — it tells a student that a welded-shut joint is
part of the robot's configuration.

```js
for (x in robot.joints) {
    if (robot.joints[x].type === 'fixed') continue;
    names[x] = next;  index[next] = x;  next++;
}
```

<div class="meaning">
<b>A configuration is what you can choose</b>
If no choice about it exists, it is not a coordinate. This is a deliberate divergence from
upstream, and it is recorded in <code>kineval/IMPLEMENTATION_NOTES.md</code> rather than
silently applied.
</div>

</div>
<div class="panel text-sm">

### The base's six DOFs

The base is a free-floating rigid body in `q`, but all fourteen worlds are ground-plane worlds
and the boundary is an x-z extent. So the default samples `base_x`, `base_z` and `base_yaw`, and
**holds** `base_y`, roll and pitch at whatever the robot started with.

<div class="history-note">
<b>plan_base_dofs</b>
<code>'planar'</code> (default) &middot; <code>'free'</code>, the honest SE(3) problem &middot;
<code>'none'</code>, base pinned and arm-only — which is what
<code>world_arm_clutter</code> needs.
</div>

In a ground-plane world, sampling base roll and pitch spends almost every sample on a
configuration either driven into the floor or physically meaningless. The planner then looks
broken when it is merely being asked the wrong question.

</div>
</div>

---
layout: default
---

# Joint limits: a data artifact you have to check

<div class="split-panel">
<div class="panel">

<<< ../reference/collision.js#joint-limits {*}{lines:true,maxHeight:'345px'}

</div>
<div class="panel text-sm">

**The course's six robot files contained the string `limit` zero times.** Without limits,
"sample uniformly in `q`" is *undefined* for fetch's `torso_lift_joint` and its gripper fingers.

This module added them to every revolute and prismatic joint, taken **verbatim from the real
upstream URDFs**, with the source cited in each file's header. No value is estimated.

<div class="meaning">
<b>Fail loudly, never default</b>
A silently-invented interval produces a planner that works for every joint but one and gives no
clue which. That is how a student loses an afternoon.
</div>

Note `bounded: false` for continuous joints: $[0, 2\pi)$ is the **sampling interval, not a
constraint**. A value outside it is a joint wound further round.

</div>
</div>

---
layout: default
---

# Two things the robot files did not say

<div class="split-panel">
<div class="panel text-sm">

### Fetch has two prismatic DOFs, not one

The lecture works the Fetch example to **13 DOFs**: 3 base, 3 continuous, 6 revolute,
**1** prismatic.

`fetch.urdf` declares `bellows_joint` as a plain prismatic joint with limits $[0, 0.4]$ and
**no `<mimic>` element**. Physically the bellows is a passive cover that follows the torso lift
— so "1 prismatic" is the right *engineering* answer and the wrong *parsing* answer.

<div class="meaning">
<b>The correct answer by inspection and by parsing differ</b>
And the difference is a real joint on a real robot. Any program reading the description sees 14
DOFs, because that is what the description says.
</div>

</div>
<div class="panel text-sm">

### Three robots declare no joint types at all

`robot_crawler.js`, `robot_mr2.js` and `robot_urdf_example.js` have zero `type:` fields. Every
joint therefore falls to upstream's rule — *an absent type means continuous* — and is sampled
over $[0, 2\pi)$.

For the crawler's 24 leg joints that is physically generous: a real hip does not rotate through
its own body.

<div class="history-note">
<b>Left as-is, on purpose</b>
Inventing joint types and limits the upstream description does not contain would be
<em>fabricating robot data</em>, which is worse than planning in a larger space than necessary.
It is recorded in each file's header instead.
</div>

This is the third independent instance in this course of a data artifact being wrong — see the
appendix.

</div>
</div>

---
layout: default
---

# The same algorithm, two embeddings

<div class="split-panel">
<div class="panel">

2D canvas &mdash; `reference/rrt.js`:

<<< ../reference/rrt.js#connect-rrt {*}{lines:true,maxHeight:'150px'}

The viewer &mdash; `reference/motion_plan.js`:

<<< ../reference/motion_plan.js#rrt-connect {*}{lines:true,maxHeight:'170px'}

</div>
<div class="panel text-sm">

Line for line, the same function. The 2D version needs an obstacle list; the viewer's needs a
robot, a q-layout, a world, a metric, a step size, counters and a validation context.

**Everything that differs is the embedding, not the algorithm.**

| | 2D canvas | viewer |
|---|---|---|
| a configuration | point in a plane | 6 base DOFs + every joint |
| collision | point vs rectangle | every link's bbox vs every sphere |
| distance | plain Euclidean | weighted, per-DOF |
| step size | constant 0.5 | fraction of $\sqrt{d}$ |

<div class="meaning">
<b>The duplication is deliberate</b>
Two files, not one import. Reading them side by side is the clearest statement available of
what "configuration space" actually buys you.
</div>

</div>
</div>

---
layout: section
---

# Test cases

### Real worlds from the upstream stencil, and five more

---
layout: default
---

# The 2D scenes

<div class="split-panel">
<div class="panel text-sm">

The five scenes carried from the pathfinding module — not invented toy cases, and already
familiar from the A\* lab:

| scene | what it tests |
|---|---|
| `spiral` | a long corridor; the goal is at the centre |
| `cul_de_sac` | a trap that attracts greedy search |
| `construction_detour` | one wall, one gap |
| `diagonal_staircase` | three offset walls; the route doubles back |
| `downtown_gridlock` | nine blocks, many homotopy classes |

Each one runs under A\*, RRT, RRT-Connect **and** RRT\* from the same page, selected by
`?search_alg=`.

</div>
<div class="panel text-sm">

### What verification asserts, per scene

- a path is found
- **no path vertex is in collision**
- **every path edge is collision-free**, re-checked independently of the planner
- consecutive raw-path vertices are within one step — except for RRT\*, whose `choose parent`
  and `rewire` steps deliberately attach across the neighbourhood radius
- smoothing never lengthens a path
- the same seed reproduces the same tree; a different seed does not

<div class="history-note">
<b>Found by running it</b>
The step-size assertion originally applied to all three planners and failed on RRT\* — correctly.
A path edge longer than a step is <em>evidence that rewiring is working</em>.
</div>

</div>
</div>

---
layout: default
---

# The 3D worlds

<div class="split-panel">
<div class="panel text-sm">

**Nine ported** from the upstream stencil's `worlds/`: `empty`, `basic`, `local_minima`,
`barrier`, `s`, `s_higher`, `s_higher_tighter`, `y`, `random`.

**Five added**, and these are *parametric* — they size themselves to whichever robot is loaded,
from three different scales:

| scale | answers |
|---|---|
| widest single **link** | how wide must a gap be to pass through |
| whole-robot **reach** | how big must an enclosure be to stand inside |
| robot **height** | how tall must a wall be to not be stepped over |

`narrow_passage` &middot; `bug_trap` &middot; `two_sided_bug_trap` &middot; `arm_clutter`
&middot; `sealed_enclosure`

</div>
<div class="panel text-sm">

<img src="/images/viewer_local_minima.png" alt="The mr2 robot in world_local_minima, with the RRT-Connect search tree and the found path routing around the three-sphere pocket" class="shot" />

<div class="text-xs">

`world_local_minima` — three spheres forming a pocket that traps greedy and potential-field
methods. RRT-Connect walks out without noticing, because it never evaluates a gradient.
`?world=local_minima&seed=2024`

</div>

</div>
</div>

---
layout: default
---

# A world that is genuinely 3D

<div class="split-panel">
<div class="panel">

<img src="/images/viewer_y.png" alt="The mr2 robot passing under world_y's overhead gate, with the search tree visible" class="shot-tall" />

</div>
<div class="panel text-sm">

`world_y` is an **overhead gate** at $z = -10$: two spheres of radius 2.0 at $y = 1.0$ and
$x = \pm 3$, leaving a gap only at $|x| < 1$ — and a third at $y = 5.0$ directly above that gap.

A short robot drives through. A tall one, or one with its arm raised, does not.

<div class="meaning">
<b>This is the counterexample</b>
Every upstream world file says <code>// collision only checked in x-z plane</code>. If that were
true of obstacle collision, this world would be trivial. It is not: obstacle collision is fully
3D against each link's bounding box, and only the <em>boundary</em> test is planar.
</div>

143 iterations, 3095 collision checks. `?world=y&seed=2024`

</div>
</div>

---
layout: default
---

# Termination

<div class="split-panel">
<div class="panel">

<img src="/images/viewer_sealed_enclosure.png" alt="The mr2 robot inside world_sealed_enclosure's ring of spheres; the HUD reads planner = failed, iteration cap reached" class="shot-tall" />

</div>
<div class="panel text-sm">

`world_sealed_enclosure` has **no solution by construction**: a closed ring of spheres, stacked
to above the robot's own height, with the robot inside it.

RRT-Connect is probabilistically complete, which says what happens when a solution *exists* and
says **nothing at all** when one does not. The planner cannot prove impossibility. It will
sample forever.

<div class="meaning">
<b>A planner that fails cleanly is a correct planner; one that hangs is not</b>
The only thing standing between a student and a locked browser tab is
<code>PLANNER_MAX_ITERATIONS</code>. Verification asserts termination here on wall-clock
<em>and</em> on the reported state.
</div>

`failed` at 5000 iterations, 71 802 collision checks, 11.6 s. `?world=sealed_enclosure&seed=31337`

</div>
</div>

---
layout: default
---

# The limits of the algorithm, honestly

<div class="split-panel">
<div class="panel text-sm">

Not every (robot, world) pair solves within the cap, and the pattern is instructive:

| robot | DOFs | walled worlds |
|---|---|---|
| `urdf_example` | 9 | all solved |
| `mr2` | 11 | all solved |
| `crawler` | 30 | `s`, `s_higher` hit the cap |
| `fetch` / `baxter` / `sawyer` | 14&ndash;21 | several hit the cap |

<div class="meaning">
<b>This is the curse of dimensionality, measured in our own code</b>
Every arm joint is sampled uniformly even when a query only needs the base to move, so both
trees must random-walk back into agreement across all those extra dimensions before they can
connect.
</div>

</div>
<div class="panel text-sm">

### Why a bigger cap is not the answer

Raising it buys surprisingly little. Nearest-neighbour cost grows **quadratically** in tree
size, and collision checks get more expensive as the trees spread, so the last thousand
iterations cost several times the first thousand. Measured: going from 15 000 to 40 000
iterations took one world from **9 seconds to 51**.

<div class="history-note">
<b>The real answers</b>
A k-d tree for nearest neighbour, and a sampler that does not treat a base-only query as a
30-dimensional one. Both are later modules — not a bigger number in this one.
</div>

An experiment that failed, recorded so it is not repeated: insetting the sampling region by the
robot's reach *sounds* right, and made things **worse** — `world_barrier`'s gap is hard against
the boundary, which is exactly what the inset removes.

</div>
</div>

---
layout: default
---

# Collision detection in the viewer

<div class="split-panel">
<div class="panel">

<img src="/images/viewer_collision_highlight.png" alt="The crawler robot on page load, with leg5_lower highlighted red where it touches an obstacle sphere" class="shot-tall" />

</div>
<div class="panel text-sm">

The default page — `?robot=crawler&world=basic` — opens **in collision**, and says so: the HUD
reads `collision = link leg5_lower vs obstacle 1`, and `scene.js` colours that link red.

That is not a bug. The crawler is 2.3 m long with legs reaching further, and `world_basic`'s
spheres sit at $\pm 2$. It is left that way because it demonstrates collision detection working
the instant the page opens.

<div class="meaning">
<b>Return what collided, not a boolean</b>
<code>poseIsCollision</code> returns <code>{link, obstacle}</code> or <code>{link, boundary}</code>.
It costs nothing — we already know it at the point of return — it makes the red highlight nearly
free, and it turns "the planner never finds a path" from a mystery into a sentence.
</div>

</div>
</div>

---
layout: default
---

# Driving the planner

<div class="split-panel">
<div class="panel text-sm">

Upstream's own key scheme (`kineval_userinput.js:65-86`), verified against the source:

| key | what it does |
|---|---|
| `Shift`+`1`&ndash;`9` | store the current whole-robot pose in a slot |
| `1`&ndash;`9` | load a slot as the planning goal |
| `g` | print the slots to the console |
| `m` | plan from here to the loaded goal |
| `n` / `b` | step forward / back through the plan |
| `t` | frame the camera on the whole world |

So the workflow is: **drive somewhere, `Shift+1`, drive back, `1`, `m`, then `n`.**

</div>
<div class="panel">

<<< ../reference/motion_plan.js#plan-motion-rrt-connect {*}{lines:true,maxHeight:'345px'}

</div>
</div>

<!--
Planning is synchronous-in-frames: a 12ms budget per frame, so the viewer
never locks. A TIME budget rather than an iteration count, because one
iteration costs wildly different amounts across the six robots.
-->

---
layout: default
---

# Goal selection, and a divergence

<div class="split-panel">
<div class="panel">

<<< ../reference/motion_plan.js#robot-rrt-planner-init {*}{lines:true,maxHeight:'350px'}

</div>
<div class="panel text-sm">

The assignment, and upstream, plan to **the zero configuration at the world origin**.

This plans to **the active pose-setpoint slot** instead, because the zero configuration is
frequently itself in collision — and when it is, every world produces the same unsolvable query
and the planner looks broken.

<div class="meaning">
<b>Slots store the base pose too</b>
Upstream's setpoints are joint-only. Without the base, "plan to slot 1" would mean "reach that
arm configuration wherever you happen to be standing", and thirteen of the fourteen worlds would
be solvable without the base moving at all.
</div>

Both ends are checked for collision up front and refused with a readable reason, rather than
planned from and silently never solved.

</div>
</div>

---
layout: default
---

# What verification actually checks

<div class="split-panel">
<div class="panel text-sm">

**1469 assertions**, over the cross product of 6 robots and 14 worlds, driving the real pages
under headless Chromium. No test body names a robot.

- `collisionForwardKinematics` writes **nothing** — snapshot every `link.xform`, call it,
  compare
- **rotation invariance** — a sphere just outside a rotated link's true extent does not collide
- **monotone inflation** — collision is non-decreasing in sphere radius
- **corner conservatism** — the deliberate false positive, asserted
- the boundary test fires on x-z only
- every path vertex collision-free; every edge re-validates
- **no edge passes through an obstacle**, re-checked at 300 samples per edge
- determinism: same seed → identical path; different seed → different path
- `n`/`b` clamp at both ends, and the rendered pose equals `motion_plan[index]` exactly

</div>
<div class="panel text-sm">

### Two things verification found that reading did not

<div class="history-note">
<b>Edge validation tunnelled</b>
A plan escaped a sealed enclosure. 324 colliding configurations on an edge the planner had
called free. The resolution was in the wrong units.
</div>

<div class="history-note mt-2">
<b>A test that was wrong, not the code</b>
An audit rolled its own interpolation and reported 173 collisions on an edge whose endpoints
differed by exactly $2\pi$ of yaw — a zero-length edge, swept the long way round. The planner was
right. Audits now call the planner's own <code>interpolateConfiguration</code>, so the two cannot
drift.
</div>

<div class="meaning">
<b>"200 OK, no console errors" is not "correct"</b>
Every figure in this deck was looked at, not just loaded.
</div>

</div>
</div>

---
layout: default
---

# What to take away

<div class="split-panel">
<div class="panel text-sm">

**The algorithm is small.** `EXTEND` is fifteen lines, `CONNECT` is four, and the difference
between them is the whole of RRT-Connect.

**The embedding is where the work is.** What a configuration *is*, what distance *means* when
the units are mixed, how finely an edge must be checked, and what counts as a degree of freedom
— none of that is in the pseudocode, and all of it decides whether the planner works.

**Collision checking dominates.** Measured here at ~1000× the cost of a nearest-neighbour
comparison, and 17:1 of total planning time. Optimise it first, and the metric you count by
matters.

**Know what is guaranteed.** Probabilistically complete is not complete, and is not optimal.
A planner that fails cleanly is correct; one that hangs is not.

</div>
<div class="panel text-sm">

### Go run it

<div class="link-list">

**[The 2D canvas &rarr;](https://ocj-dev.github.io/kineval/motion_planning/reference/)**
Try `?search_alg=RRT` against `?search_alg=RRT-connect` on `scenes/diagonal_staircase.js`, then
`RRT-star` on the same scene and watch the path cost come down.

**[The 3D viewer &rarr;](https://ocj-dev.github.io/kineval/kineval/?world=local_minima)**
`Shift+1` to store a goal, `1` to load it, `m` to plan, `n` to walk it. Then try
`?robot=crawler` and watch 30 DOFs behave differently.

</div>

<div class="history-note mt-3">
<b>Everything here is reproducible</b>
One seeded generator, the seed in the HUD, and <code>?seed=</code> on both pages. Every number
and every picture in this deck can be regenerated from the URL in its caption.
</div>

</div>
</div>

---
layout: center
class: text-center
---

# Appendices

### Code and findings that did not fit the main thread

---
layout: default
---

# Appendix: reading a data source critically

<div class="split-panel">
<div class="panel text-sm">

This lab's plan asserted two defects in the upstream world files. **Verifying them was the
exercise, and both turned out to be wrong.**

<div class="history-note">
<b>Claim: <code>world_s.js</code> drops an obstacle</b>
Line 20 assigns <code>robot_obstacles[10]</code>; line 22 overwrites it. True — but <b>line 34
re-adds the displaced sphere at index 21</b>. The obstacle set is complete. There is no gap in
the S-wall.
</div>

<div class="history-note mt-2">
<b>Claim: <code>world_barrier.js</code> has the same defect</b>
Its block comment opens at <b>line 22</b>, <em>before</em> the line identified as a live
overwrite. The entire second wall is commented out. The file is clean.
</div>

So the dropped-obstacle defect is in **zero** files, not two.

</div>
<div class="panel text-sm">

### The defects that are real

- **`world_s_higher_tighter.js`** assigns `robot_boundary` twice — line 5, then line 9. The
  first is dead.
- **`world_random.js`**, four separate problems: it multiplies by the boundary *minimum*
  instead of interpolating, so it never samples the region out to $+2$; it puts sphere centres
  at $y \in [-0.6, 2.4)$, burying some below the floor; radii run to 3.0 and arbitrarily close
  to 0; and it uses `Math.random()`, so no two page loads agree.
- the **"x-z plane only" comment**, wrong in all nine files.

<div class="meaning">
<b>The appendix's real thesis</b>
Three independent data defects in one course — <code>baxter.urdf.js</code>'s double fixed joint
(FK deck), the missing joint limits, and these worlds — plus two <em>claimed</em> defects that
did not survive being checked. Read the data, and check the claims about the data too.
</div>

</div>
</div>

---
layout: default
---

# Appendix: the `reference/` snapshot convention

<div class="split-panel">
<div class="panel text-sm">

A student browsing this repository will notice that
`forward_kinematics/reference/kinematics.js` and `kineval/kinematics.js` are **not identical**,
and deserves to know why.

Slidev's `<<<` importer and Vite's `fs.allow` stop at the workspace root, which each module's
`pnpm-workspace.yaml` deliberately places at the module directory. So a deck can quote
`../reference/*` and **cannot** reach `../../kineval/*`.

Every module therefore carries its own copy of the viewer code it teaches. This one carries
dated snapshots of `collision.js` and `motion_plan.js`, and a byte-identical copy of
`random.js`.

</div>
<div class="panel text-sm">

### The cost, and the cheap check

Copies drift. The FK and IK modules' copies **already have**. That is the accepted price of a
per-module `reference/` in a repository with deliberately no build step.

<div class="meaning">
<b>So the drift is tested, not hoped for</b>
Each snapshot carries a header naming its source path and date, and the verification suite
<code>diff</code>s every one against its <code>kineval/</code> original and fails if they
differ. It caught a stale snapshot exactly once, correctly, when the source was edited after
copying.
</div>

Edits belong in `kineval/`, followed by a re-snapshot. Never the other way round.

</div>
</div>

---
layout: default
---

# Appendix: the tree, and what it is not

<div class="split-panel">
<div class="panel">

<<< ../reference/motion_plan.js#tree-init {*}{lines:true}

<<< ../reference/motion_plan.js#tree-add-vertex {*}{lines:true}

<<< ../reference/motion_plan.js#tree-add-edge {*}{lines:true}

</div>
<div class="panel text-sm">

Vertices carry their configuration, their adjacency, and a **parent index** — and it is the
parent that makes path extraction a walk rather than a search.

<div class="meaning">
<b>What the viewer draws is a shadow</b>
<code>updatePlannerMeshes</code> puts one marker per tree vertex <b>at that configuration's base
location</b>. A vertex is a point in an $n$-dimensional space and there is no honest way to draw
one in three dimensions, so two vertices with identical base poses and completely different arm
configurations land on top of each other.
</div>

Upstream's `add_config_origin_indicator_geom` does the same thing without saying so.

</div>
</div>

---
layout: default
---

# Appendix: remaining reference code

<div class="split-panel">
<div class="panel">

Per-joint sampling intervals, in the viewer:

<<< ../reference/motion_plan.js#random-config {*}{lines:true,maxHeight:'155px'}

Interpolation — one definition, shared:

<<< ../reference/motion_plan.js#interpolate-configuration {*}{lines:true,maxHeight:'155px'}

</div>
<div class="panel">

Reading a pose out, and driving one in:

<<< ../reference/collision.js#configuration-from-robot {*}{lines:true,maxHeight:'145px'}

<<< ../reference/collision.js#apply-configuration-to-robot {*}{lines:true,maxHeight:'165px'}

</div>
</div>

---
layout: default
---

# Appendix: the scales a generated world needs

<div class="split-panel">
<div class="panel">

<<< ../reference/collision.js#joint-lever-arms {*}{lines:true,maxHeight:'350px'}

</div>
<div class="panel text-sm">

The lever arm a joint acts through: the radius of everything downstream of it, **over every
configuration** — not just the current one.

Two earlier versions were wrong in instructive ways:

<div class="history-note">
<b>Too loose</b>
Using the whole robot's reach for every joint is a valid bound and a useless one. The crawler
has 24 joints, mostly knees and ankles that swing a foot; charging each with the full body
radius sent one planning run from <b>7 seconds to 55</b>.
</div>

<div class="history-note mt-2">
<b>Not a bound at all</b>
Measuring each subtree's <em>actual</em> radius at the start pose is tight and wrong: a folded
leg extends later in the path, so the spacing computed from it is too coarse there. Verification
found 173 colliding configurations on a path declared free.
</div>

</div>
</div>

---
layout: default
---

# Appendix: deriving a robot's own scales

<div class="split-panel">
<div class="panel">

<<< ../reference/collision.js#robot-extent {*}{lines:true,maxHeight:'130px'}

<<< ../reference/collision.js#robot-height {*}{lines:true,maxHeight:'215px'}

</div>
<div class="panel text-sm">

`robotExtent` — widest single **link**. The scale for *how wide must a gap be*.

`robotReach` — the whole **assembly's** ground-plane span. The scale for *how big must an
enclosure be*. Sizing `world_sealed_enclosure` by `extent` wedged several robots into their own
enclosure before planning started.

`robotHeight` — **a single course of spheres is not a wall.** Obstacle collision is fully 3D, so
a robot whose links sit above a knee-high ring simply walks over it, and the first version of
the "sealed" enclosure was not sealed.

<div class="history-note">
<b>Upstream hit this too</b>
<code>world_s_higher.js</code>'s two courses at <code>y = 0.5</code> and <code>y = 1.5</code>,
with its comment "obstacles near boundaries to force arm upright", is the same discovery —
hand-fixed, for one robot. Deriving it keeps it fixed for all of them.
</div>

</div>
</div>

---
layout: default
---

# Appendix: the 2D module's support code

<div class="split-panel">
<div class="panel">

The collision test the whole 2D module rests on, provided by the stencil:

<<< ../reference/infrastructure.js#appendix-collision {*}{lines:true,maxHeight:'190px'}

RRT\*'s neighbourhood query and cost bookkeeping:

<<< ../reference/rrt.js#find-neighbors-within {*}{lines:true,maxHeight:'120px'}

</div>
<div class="panel">

<<< ../reference/rrt.js#vertex-cost {*}{lines:true,maxHeight:'130px'}

<<< ../reference/rrt.js#path-length {*}{lines:true,maxHeight:'105px'}

<<< ../reference/rrt.js#dfs-path {*}{lines:true,maxHeight:'115px'}

</div>
</div>

---
layout: default
---

# Appendix: single-tree RRT, and the seeded generator

<div class="split-panel">
<div class="panel">

<<< ../reference/rrt.js#iterate-rrt {*}{lines:true,maxHeight:'350px'}

</div>
<div class="panel">

<<< ../reference/random.js#seed-random {*}{lines:true,maxHeight:'185px'}

<<< ../reference/random.js#random-in-range {*}{lines:true,maxHeight:'62px'}

<<< ../reference/random.js#random-integer {*}{lines:true,maxHeight:'62px'}

The getter the HUD calls, so a screenshot carries its own seed:

<<< ../reference/random.js#random-seed-value {*}{lines:true,maxHeight:'58px'}

</div>
</div>

---
layout: default
---

# Appendix: the viewer's remaining planner code

<div class="split-panel">
<div class="panel">

<<< ../reference/motion_plan.js#rrt-extend {*}{lines:true,maxHeight:'175px'}

<<< ../reference/motion_plan.js#nearest-neighbor {*}{lines:true,maxHeight:'165px'}

</div>
<div class="panel">

<<< ../reference/motion_plan.js#robot-rrt-planner-iterate {*}{lines:true,maxHeight:'250px'}

<<< ../reference/motion_plan.js#step-motion-plan-traversal {*}{lines:true,maxHeight:'95px'}

</div>
</div>

---
layout: default
---

# Appendix: the rest of collision.js

<div class="split-panel">
<div class="panel">

<<< ../reference/collision.js#collision-joint-transform {*}{lines:true,maxHeight:'165px'}

<<< ../reference/collision.js#collision-traverse-link {*}{lines:true,maxHeight:'165px'}

</div>
<div class="panel">

<<< ../reference/collision.js#collision-traverse-joint {*}{lines:true,maxHeight:'145px'}

<<< ../reference/collision.js#robot-is-collision {*}{lines:true,maxHeight:'85px'}

<<< ../reference/collision.js#joint-chains {*}{lines:true,maxHeight:'130px'}

</div>
</div>

---
layout: default
---

# Appendix: everything else that runs

<div class="split-panel">
<div class="panel">

The viewer's remaining helpers:

<<< ../reference/motion_plan.js#new-config {*}{lines:true,maxHeight:'140px'}

<<< ../reference/motion_plan.js#edge-is-valid {*}{lines:true,maxHeight:'140px'}

<<< ../reference/motion_plan.js#find-path {*}{lines:true,maxHeight:'95px'}

</div>
<div class="panel">

<<< ../reference/motion_plan.js#smooth-path {*}{lines:true,maxHeight:'150px'}

<<< ../reference/motion_plan.js#describe-collision {*}{lines:true,maxHeight:'80px'}

<<< ../reference/collision.js#robot-reach {*}{lines:true,maxHeight:'150px'}

</div>
</div>

---
layout: default
---

# Appendix: A\*'s own code, carried for the comparison

<div class="split-panel">
<div class="panel">

<<< ../reference/graph_search.js#priority-formula {*}{lines:true,maxHeight:'185px'}

<<< ../reference/draw.js#path-reconstruct {*}{lines:true,maxHeight:'150px'}

</div>
<div class="panel">

<<< ../reference/graph_search.js#insert-queue {*}{lines:true,maxHeight:'120px'}

<<< ../reference/graph_search.js#pop-min {*}{lines:true,maxHeight:'120px'}

The dispatch that runs whichever algorithm `?search_alg=` names:

<<< ../reference/draw.js#appendix-animate {*}{lines:true,maxHeight:'120px'}

</div>
</div>

---
layout: default
---

# Appendix: A\*'s grid and search loop

<div class="split-panel">
<div class="panel">

<<< ../reference/graph_search.js#grid-node-fields {*}{lines:true,maxHeight:'130px'}

<<< ../reference/graph_search.js#find-start-cell {*}{lines:true,maxHeight:'130px'}

<<< ../reference/graph_search.js#init-start {*}{lines:true,maxHeight:'115px'}

</div>
<div class="panel">

<<< ../reference/graph_search.js#main-loop {*}{lines:true,maxHeight:'195px'}

<<< ../reference/graph_search.js#neighbor-loop {*}{lines:true,maxHeight:'195px'}

</div>
</div>

<!--
Carried verbatim from pathfinding/reference/ and taught in full by the
pathfinding deck; reproduced here only so this module's code-coverage
pass covers every region of every file it carries, with no exemptions.
-->


<!--
graph_search.js and draw.js are carried into this module unchanged from
pathfinding/reference/, so the A*-versus-RRT-Connect panel runs the same
A* the previous lab taught. Fully covered in the pathfinding deck; quoted
here so this deck's own code-coverage pass reaches every region in every
file it carries.
-->

---
layout: center
class: text-center
---

# <span class="accent">Happy hacking</span>, and make progress

<div class="pt-4 text-sm">

`motion_planning/reference/` &middot; `kineval/` &middot;
[ocj-dev.github.io/kineval](https://ocj-dev.github.io/kineval/)

</div>

<div class="title-footer">
<div class="credit">AutoRob (autorob.org) &#183; Chad Jenkins (ocj@umich.edu) &#183; Michigan Honor License &#183; 2026</div>
</div>
