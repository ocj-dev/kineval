---
theme: default
title: Behavior Trees — KinEval Lab
base: /kineval/behavior_trees/
colorSchema: light
info: |
  ## Behavior Trees and Motion-Plan Execution
  A lab-session walkthrough of how a motion plan is represented, executed, and
  sequenced — taught from the Rust publish-subscribe system, for the AutoRob
  course (autorob.org).
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

# <span class="accent">Behavior Trees</span>

### and where a motion plan actually goes

<div class="pt-6">
<div class="search-bar">🌳&nbsp;&nbsp;<b>tick</b>&nbsp;&nbsp;&rarr;&nbsp;&nbsp;Success&nbsp;&middot;&nbsp;Failure&nbsp;&middot;&nbsp;<b>Running</b></div>
</div>

<div class="title-footer">
<div class="nav-hint">Press → to move forward through the deck</div>
<div class="credit">AutoRob (autorob.org) &#183; Chad Jenkins (ocj@umich.edu) &#183; ocj-dev.github.io/kineval/behavior_trees &#183; Michigan Honor License &#183; 2026</div>
</div>

<!--
This deck's subject is the Rust publish-subscribe system
(pubsub_testbed_rs), not the KinEval JavaScript viewer that the rest of
this site teaches. It is hosted here because this repository owns the
course's Slidev pipeline.
-->

---
layout: default
---

# Where the code is, and what is new here

<div class="split-panel">
<div class="panel text-sm">

**This deck is about the Rust publish-subscribe system**, not the KinEval viewer
the rest of this site teaches. The code lives in `pubsub_testbed_rs`: nodes that
talk over topics, services and actions, in the ROS 2 style the course uses.

It is hosted here because this repository owns the course's slide pipeline. Every
Rust file quoted is carried into `behavior_trees/reference/` as a **dated
snapshot**, with its source path, upstream length and commit hash in the header —
so a citation to "line 1087" means the line in the real repository, not in the
copy you are reading.

<div class="meaning">
<b>There is no behavior-tree lecture in this course</b>
All 89 pages of <code>autorob_2026_09_fsm_subsumption.pdf</code> cover finite
state machines, subsumption and behavior-based control — and never mention
behavior trees. So this deck carries its own formalism from scratch rather than
reinforcing a lecture.
</div>

</div>
<div class="panel text-sm">

### The arc

1. **what a behavior tree is** — ticks, the three-valued return, the node kinds,
   and the one rule everybody breaks
2. **a motion plan as data** — the `JointTrajectory` message, and why publishing
   one is not enough
3. **executing it** — the JointTrajectory *action*: feedback, result, cancel
4. **sequencing it** — the plan-and-execute leaf pair inside a tree, for tasks
   longer than one motion

then the comparison the course *does* teach: finite state machines, subsumption,
and behavior-based robotics.

<div class="history-note mt-2">
<b>Cross-link</b>
The geometry and the search — configuration space, collision detection,
RRT-Connect — are the
<a href="/kineval/motion_planning/">motion planning deck</a>. This deck takes a
plan as given and asks what happens to it next.
</div>

</div>
</div>

---
layout: section
---

# Part 1 — What a behavior tree is

### No lecture behind this; we build it from scratch

---
layout: default
---

# The problem: sequencing a robot

<div class="split-panel">
<div class="panel text-sm">

A robot that does one thing needs no architecture. A robot that must

> approach the object · plan a path · execute it · close the gripper ·
> plan a path back · open the gripper

needs something that decides **what to do next**, and that can cope when a step
fails, takes a long time, or has to be abandoned.

Three answers the course has already shown you:

- a **finite state machine** — states and transitions, a switch-case in a loop
- a **subsumption architecture** — layers of reactive behaviour, highest priority
  wins
- **behavior-based control** — behaviours as modules, sequence emerging rather
  than planned

<div class="meaning">
<b>A fourth answer</b>
A <b>behavior tree</b> keeps behaviour-as-module and restores explicit
sequencing — and gives a clean place to put a deliberative planner.
</div>

</div>
<div class="panel text-sm">

### Where they came from

Behavior trees were invented in **commercial game AI**, not robotics: the
canonical origin is *Halo 2* (Bungie, 2004), written up by **Damián Isla**,
*Handling Complexity in the Halo 2 AI*, GDC 2005. The problem was the same one
above — scripted FSMs for non-player characters had become unmaintainable.

Robotics adopted them roughly a decade later, and the standard reference is now
**Colledanchise & Ögren**, *Behavior Trees in Robotics and AI: An Introduction*,
CRC Press, 2018 — which also proves the equivalence results this deck cites
later.

Today they are the task-level layer in **MoveIt Pro**, **Nav2** (`BT Navigator`),
and most ROS 2 manipulation stacks, usually through the **BehaviorTree.CPP**
library.

</div>
</div>

---
layout: default
---

# The tick

<div class="split-panel">
<div class="panel text-sm">

A behavior tree is **not** a flowchart that runs once. It is a tree that gets
**ticked** — traversed from the root, at some fixed rate — over and over.

Every node, when ticked, returns exactly one of three things:

| | |
|---|---|
| **`Success`** | this node is done, and it worked |
| **`Failure`** | this node is done, and it did not work |
| **`Running`** | this node is **not** done; ask again next tick |

The parent reads that value and decides what to do. That is the entire
mechanism.

<div class="meaning">
<b>Two-way control transfer</b>
A parent passes control <i>down</i> by ticking a child, and the child passes a
result <i>back up</i>. An FSM's transition is a one-way <code>goto</code> — the
state it leaves never hears what happened next.
</div>

</div>
<div class="panel text-sm">

### The one rule everybody breaks

<div class="meaning">
<b>A tick is polling. A leaf must never block.</b>
A leaf that waits for a motion to finish before returning freezes the whole
tree: no other branch is ticked, no condition is re-checked, nothing can
pre-empt it. A leaf <b>starts</b> work on the tick that begins it, returns
<code>Running</code> immediately, and reports completion on some <i>later</i>
tick.
</div>

This is why `Running` exists at all, and it is the single most common mistake
when writing a behavior tree for the first time.

In this implementation every leaf is written that way, which is why
`bt_node.rs` has a region called `non-blocking-leaves` — the property is load
bearing enough to name.

<div class="history-note">
<b>What non-blocking does and does not buy</b>
It keeps the tree <i>tickable</i>. Whether a tick re-examines earlier children
is a separate choice — see "memory" two slides on, where this implementation
takes the non-reactive option.
</div>

</div>
</div>

---
layout: default
---

# Node kinds

<div class="split-panel">
<div class="panel text-sm">

**Composites** — interior nodes, which have children and arbitrate between them:

- **`sequence`** — an AND. Tick children in order; the first *non-Success*
  stops the walk and becomes the result. All succeed → `Success`.
- **`selector`** — an OR, and a **priority list**. Tick children in order; the
  first *non-Failure* stops the walk and becomes the result. All fail →
  `Failure`.

**Decorators** — exactly one child, whose result they transform:

- **`repeat`** — run the child *n* times
- **`invert`** — swap `Success` and `Failure` (and leave `Running` alone)

**Leaves** — the nodes that actually do or check something:

- **`action`** — drive the robot toward a pose
- **`condition`** — ask a question and answer `Success` / `Failure`

</div>
<div class="panel text-sm">

<div class="meaning">
<b>Running is decisive for every composite</b>
A sequence stops at a <code>Running</code> child, and so does a selector.
Neither "skips ahead" — if a child is still working, the tick ends there and
the composite reports <code>Running</code> upward. The next tick starts again
at the root and comes back down to that same child.
</div>

### Reset on a decisive result

A node that returns `Success` or `Failure` is **finished**, and its internal
state is cleared, so that if it is ticked again it starts afresh. A node that
returns `Running` keeps its state.

That rule is what makes a `repeat` loop work, and — as the next slide shows —
it is what makes a composite's cursor restart.

</div>
</div>

---
layout: default
---

# Memory: the choice that surprises everyone

<div class="split-panel">
<div class="panel text-sm">

A composite that is `Running` has a child part-way through. On the **next**
tick, does it re-tick its earlier children, or resume where it left off?

Both are legitimate, and the two have names:

- **with memory** (BehaviorTree.CPP's `Sequence`) — keep a cursor. Children
  that already returned `Success` are **not** ticked again.
- **reactive** (`ReactiveSequence`) — no cursor. Every tick re-ticks from the
  first child, so earlier conditions are re-evaluated continuously.

<div class="meaning">
<b>This implementation has memory</b>
<code>BtState::Sequence { current_index, child_states }</code> — the cursor is
right there in the state. A <code>condition</code> guard that succeeded on
tick 1 is <b>not</b> re-checked on tick 7.
</div>

</div>
<div class="panel text-sm">

You can see it in the trace. At tick 7 of the recorded run, the only nodes
visited are

```
follow_trajectory   running
sequence            running
```

The `condition` and the `plan` above it are not in the list at all — they
already succeeded, and the cursor has moved past them.

<div class="history-note">
<b>Why it matters, practically</b>
With memory, a guard is a <b>precondition</b>: checked once, on the way in.
Reactive, the same guard is an <b>invariant</b>: violated at any moment, the
subtree is abandoned. Writing an invariant and getting a precondition is a
real bug, and the only thing that distinguishes them is which composite you
chose.
</div>

The reset rule is what closes the loop: once the sequence finally returns
`Success` or `Failure`, its state — cursor included — is cleared, so the next
tick starts from the first child again.

</div>
</div>

---
layout: default
---

# Tick semantics, by hand

<div class="content-body">
<BtSemanticsPanel />
</div>

<div class="slide-caption">
Click a child to change what it returns, and switch the composite. The walk
rule is the same one <code>bt_node.rs</code> implements.
</div>

---
layout: default
---

# The node kinds, in Rust

<div class="split-panel">
<div class="panel">

<<< ../reference/bt_node.rs#bt-node-kinds {*}{lines:true,maxHeight:'345px'}

</div>
<div class="panel text-sm">

**Eight kinds, as a genuine Rust `enum`.** Five are the general-purpose ones
above; three — `Plan`, `FollowTrajectory`, `Condition` — are this module's
additions and are the subject of Part 4.

<div class="meaning">
<b>The payoff of an enum over a string</b>
<code>kind()</code> derives the name <i>from</i> the enum. Nothing in the
interpreter ever compares a raw <code>"type"</code> string — that happens in
exactly one place, the parser, and after it the compiler checks every match
site exhaustively. Add a ninth kind and the build fails in every place that
needs updating.
</div>

Note `ConditionCheck` is an enum with a **single** variant. That looks like
over-engineering until you add the second kind of check: with an enum it is a
compile error at every match site, with a string it is a silent fall-through.

</div>
</div>

---
layout: default
---

# Ticking the tree

<div class="split-panel">
<div class="panel">

<<< ../reference/bt_node.rs#hierarchical-dispatch {*}{lines:true,maxHeight:'345px'}

</div>
<div class="panel text-sm">

One recursive function over the enum. The composites are four or five lines
each, because the semantics really are that small once `Running` is handled
uniformly.

Two things worth noticing:

- **the `visited` and `running_paths` bookkeeping.** Each tick records which
  nodes it reached, by child-index path, and what each returned. That is what
  `/bt/status` publishes once per tick — and it is what the panels in this deck
  replay.
- **the path is a `Vec<usize>`**, the child indices from the root. `[]` is the
  root, `[2]` is its third child. A compact address for any node in the tree,
  which the trace uses throughout.

</div>
</div>

---
layout: section
---

# Part 2 — A motion plan is data

### `JointTrajectory`

---
layout: default
---

# Where does a motion plan go?

<div class="split-panel">
<div class="panel text-sm">

The [motion planning deck](/kineval/motion_planning/) ends with RRT-Connect
returning a **path**: collision-free configurations from start to goal.

That is not yet executable. It has no notion of **time** — nothing saying when
the arm should be where. Adding that is *trajectory generation*, and the result
rides in **`trajectory_msgs/JointTrajectory`**:

```
JointTrajectory
  header       { stamp, frame_id }
  joint_names  [ string, ... ]
  points       [ JointTrajectoryPoint, ... ]

JointTrajectoryPoint
  positions        [ float64, ... ]
  velocities       [ float64, ... ]
  accelerations    [ float64, ... ]
  time_from_start  { sec, nanosec }
```

</div>
<div class="panel text-sm">

### The positional-order contract

`joint_names` is a list, and every `positions` array is **parallel to it**.
`positions[i]` is the value for `joint_names[i]` — there is no name inside the
point itself.

<div class="meaning">
<b>Why that matters</b>
The ordering is a contract between producer and consumer, carried once in the
header rather than repeated per point. Reorder <code>joint_names</code> without
reordering every <code>positions</code> array and the robot moves — smoothly,
confidently, to entirely the wrong pose. Nothing will throw.
</div>

### `time_from_start`

Each point's time is measured from the **start of the trajectory**, not from
the epoch and not from the previous point. So the points are strictly
increasing in time, and the last point's `time_from_start` is the duration of
the whole motion.

</div>
</div>

---
layout: default
---

# A real one

<div class="split-panel">
<div class="panel text-sm">

This is the trajectory the `plan` leaf produced in the recorded trace this deck
replays — six joints, trimmed to its first two points:

```json
{
  "joint_names": [
    "shoulder_pan_joint", "shoulder_lift_joint",
    "elbow_joint",        "wrist_roll_joint",
    "wrist_pitch_joint",  "tool_roll_joint"
  ],
  "points": [
    { "positions":   [-1.885, -0.942, -1.571,
                      -1.885, -0.942, -1.885],
      "velocities":  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "time_from_start": { "sec": 0, "nanosec": 500000000 } },
    { "positions":   [ ... ],
      "time_from_start": { "sec": 1, "nanosec": 0 } }
  ]
}
```

</div>
<div class="panel text-sm">

Read the first point back against the contract:

- six names, six positions, in the same order
- `time_from_start` is **0.5 s** — the first waypoint is half a second in, not
  at zero
- velocities are present and zero, because this planner emits a geometric path
  retimed with stops at the waypoints rather than a dynamically smooth one

<div class="meaning">
<b>This is the whole plan, as data</b>
A value you can log, replay, diff, store in a blackboard, or send over a wire.
Nothing about it says who runs it, when it started, whether it finished, or how
to stop it. That is the subject of the next three slides.
</div>

</div>
</div>

---
layout: default
---

# Why publishing it is not enough

<div class="split-panel">
<div class="panel text-sm">

Suppose you publish that message on a topic — `/joint_trajectory` — and some
node subscribes and drives the arm. What have you given up?

- **Nobody knows when it finished.** A publisher gets no acknowledgement. The
  sender can poll joint states and guess, which is the beginning of
  reimplementing an action badly.
- **Nobody can cancel it.** There is no handle to the in-flight motion. The
  only way to stop it is to publish something else and hope the consumer
  prefers it.
- **Nobody knows if it was rejected.** Malformed message, wrong joint names,
  node not running — all indistinguishable from success at the sender.
- **There is no feedback.** No "I am at point 3 of 11".

<div class="meaning">
<b>A topic is fire-and-forget</b>
Which is exactly right for a sensor stream, and exactly wrong for a long-running
motion you need to supervise.
</div>

</div>
<div class="panel text-sm">

### And in this system, something worse

<div class="history-note">
<b>robot_sim_node.rs, upstream line 1087</b>
The simulator's <code>apply_trajectory</code> handler reads
<code>/joint_trajectory</code> and keeps <b>only the last point</b>. Every
intermediate waypoint is discarded, and the arm <b>teleports</b> to the final
pose in a single frame.
</div>

So publishing a whole trajectory to this topic does not execute it slowly and
badly — it does not execute it *at all*. It jumps to the end.

That is not a criticism of the simulator: `/joint_trajectory` is a setpoint
channel, and treating the last point as the setpoint is a reasonable reading of
a message with no execution semantics attached.

It is an argument for the **action**.

</div>
</div>

---
layout: default
---

# The proof, in fourteen lines

<div class="split-panel">
<div class="panel">

<<< ../reference/robot_sim_node.rs#apply-trajectory {*}{lines:true,maxHeight:'300px'}

<div class="text-xs mt-1">

`robot_sim_node.rs`, upstream **lines 1084–1097**. The `#region` marker is an
addition in this deck's snapshot — see the file header.

</div>
</div>
<div class="panel text-sm">

```rust
let Some(last_point) = points.last() else { return };
```

One line, and every intermediate waypoint in the message is gone. What follows
copies that point's positions straight into `self.setpoint`.

<div class="meaning">
<b>Read it as a contract, not a bug</b>
This handler implements "the trajectory's endpoint is the new setpoint". It is
honest about what a topic can promise. The mistake would be <i>believing</i>
that publishing a trajectory here executes it — and that mistake is invisible
unless you look at this function or watch the arm jump.
</div>

The next panel shows both behaviours side by side, driven by the same recorded
message.

</div>
</div>

---
layout: default
---

# Teleport versus execution

<div class="content-body">
<TrajectoryPanel />
</div>

<div class="slide-caption">
Both columns animate the <em>same</em> trajectory — the one the <code>plan</code>
leaf wrote to the blackboard in the recorded trace. Left: the topic. Right: the
action.
</div>

---
layout: section
---

# Part 3 — Executing it

### The JointTrajectory action

---
layout: default
---

# Topic, service, action

<div class="split-panel">
<div class="panel text-sm">

| | shape | right for |
|---|---|---|
| **topic** | fire-and-forget, many-to-many | streams: joint states, scans, images |
| **service** | request → reply, blocking, fast | questions: *is this configuration valid?* |
| **action** | goal → feedback\* → result, **cancellable** | long work: *move along this trajectory* |

An **action** is the one that fits a motion. It gives the caller four things a
topic cannot:

1. a **goal** that is accepted or rejected, with an id
2. **feedback** while the work proceeds
3. a **result** when it ends — succeeded, aborted, or cancelled
4. **cancel**, and therefore pre-emption

</div>
<div class="panel text-sm">

<div class="meaning">
<b>The distinction this repository already argued</b>
<code>pubsub_testbed_rs</code>'s README makes the service-versus-action case for
<code>ik_action_node</code>: a service call that takes seconds blocks its caller
and cannot be abandoned. Executing a trajectory has exactly the same shape, and
gets the same answer.
</div>

### The real name

In ROS 2 the standard interface for this is

> **`control_msgs/action/FollowJointTrajectory`**

served by `joint_trajectory_controller` in `ros2_control`. This system's
`joint_trajectory_action_node` is shaped after it deliberately. Learn the real
name — it is the one on the job.

</div>
</div>

---
layout: default
---

# Walking the points

<div class="split-panel">
<div class="panel">

<<< ../reference/joint_trajectory_action_node.rs#one-point-at-a-time {*}{lines:true,maxHeight:'345px'}

</div>
<div class="panel text-sm">

The action server holds the trajectory and advances through it **one point at a
time**, publishing each as a setpoint and waiting for the arm to arrive before
moving to the next.

That is precisely the behaviour `/joint_trajectory` alone cannot give you, and
it is why the action exists.

<div class="meaning">
<b>split_point, and a drift that cannot happen</b>
A trajectory may plan the base, the joints, or both. This splits a point into
its joint half and its base half by matching against
<code>BASE_DOF_NAMES</code> — <b>by position in a shared constant</b>, not
against string literals repeated here. The planner's notion of which
coordinates are the base and this node's are one definition, in
<code>planning_scene.rs</code>, so they cannot disagree.
</div>

</div>
</div>

---
layout: default
---

# Knowing you have arrived

<div class="split-panel">
<div class="panel">

<<< ../reference/joint_trajectory_action_node.rs#arrival-check {*}{lines:true,maxHeight:'345px'}

</div>
<div class="panel text-sm">

Each point is gated on **arrival within epsilon**: the maximum absolute error
across joint-name-matched pairs, compared against a tolerance carried on the
goal.

Three details worth the slide:

- **name-matched, not index-matched.** `/joint_states` need not list joints in
  the trajectory's order, and comparing by position would silently compare a
  shoulder against a wrist.
- **`None` on a mismatched set.** If the sample does not name exactly the
  expected joints, the function refuses to answer rather than comparing a
  subset — a missing joint is not an arrival.
- **the convention is shared with `bt_node.rs`'s own `max_joint_error`**,
  deliberately duplicated rather than abstracted, and the comment says so.

<div class="meaning">
<b>Epsilon is a contract with the controller</b>
Too tight and a trajectory never completes; too loose and the arm moves on
before it has arrived. It belongs on the goal, where the caller who knows the
task can set it.
</div>

</div>
</div>

---
layout: section
---

# Part 4 — Sequencing it

### The plan-and-execute pair, inside a tree

---
layout: default
---

# Three more leaf kinds

<div class="split-panel">
<div class="panel text-sm">

The five general-purpose kinds cannot express "plan a path and then follow it".
This module adds three:

```json
{ "type": "condition",
  "check": "state_validity",
  "positions": [ ... ] }

{ "type": "plan",
  "goal": [ ... ],
  "output_key": "traj",
  "seed": 1, "max_iterations": 120 }

{ "type": "follow_trajectory",
  "input_key": "traj",
  "epsilon": 0.02, "timeout": 60.0 }
```

</div>
<div class="panel text-sm">

- **`condition`** asks `planning_scene_node` whether a configuration is
  collision-free, and answers `Success` / `Failure`. A guard.
- **`plan`** calls the RRT-Connect planner, and **writes the resulting
  trajectory somewhere**.
- **`follow_trajectory`** **reads a trajectory from somewhere** and sends it to
  the action as a goal, returning `Running` until the action resolves.

<div class="meaning">
<b>"Somewhere" is the whole problem</b>
<code>plan</code> produces a value that a <i>sibling</i> needs. None of the
original five kinds ever did — every one of them carried an
already-resolved target in its own JSON. This is the one genuinely new idea in
the implementation, and it is what the next slides are about.
</div>

</div>
</div>

---
layout: default
---

# A leaf that does not block

<div class="split-panel">
<div class="panel">

<<< ../reference/bt_node.rs#non-blocking-leaves {*}{lines:true,maxHeight:'345px'}

</div>
<div class="panel text-sm">

`follow_trajectory` is an **action client**. On the tick that starts it, it
sends one `send_goal` and returns `Running` — it does not wait.

On every later tick it looks for its own goal id among the results seen on
`/joint_trajectory_action/result`:

- result not there yet → `Running`
- result says succeeded → `Success`
- result says aborted or cancelled, or the timeout elapsed → `Failure`

<div class="meaning">
<b>This is the polling rule, made concrete</b>
The leaf is ticked perhaps fifty times during one motion, and does almost
nothing on forty-nine of them. That is correct: the tick returns immediately,
so the tree is never stuck inside a leaf, and the motion can be abandoned
between any two ticks.
</div>

Which of its *siblings* get re-examined while it runs is a separate question,
and the answer in this implementation is "none" — the next slide.

</div>
</div>

---
layout: default
---

# The blackboard

<div class="split-panel">
<div class="panel text-sm">

A **blackboard** is a set of named slots that nodes in a tree read and write, so
one node's output can become another's input without the two referring to each
other.

Here it is deliberately minimal: `plan` writes to its `output_key`,
`follow_trajectory` reads from its `input_key`, and the names must match.

```json
{ "type": "sequence", "children": [
  { "type": "condition", "check": "state_validity",
    "positions": [-1.885, -0.942, -1.571,
                  -1.885, -0.942, -1.885] },
  { "type": "plan", "output_key": "traj",
    "goal": [1.885, 0.942, 1.571,
             1.885, 0.942, 1.885],
    "seed": 1, "max_iterations": 120 },
  { "type": "follow_trajectory", "input_key": "traj",
    "epsilon": 0.02, "timeout": 60.0 }
]}
```

That is the tree the recorded traces run — quoted from the trace's own header.

</div>
<div class="panel text-sm">

<div class="meaning">
<b>Why the original five kinds never needed one</b>
Every one of them carried its target inside its own JSON: an <code>action</code>
leaf knows its <code>target_positions</code> before the tree ever runs. Nothing
was computed <i>during</i> execution, so nothing had to be handed anywhere.
<code>plan</code> is the first leaf whose output is not known at load time.
</div>

### The guard in front of it

The `condition` is not decoration. `plan` from a start that is already in
collision cannot succeed, and the sequence's AND semantics mean the guard
returning `Failure` stops the walk before the planner is ever called.

Cheap check first, expensive work second — which is what a sequence is *for*.

</div>
</div>

---
layout: default
---

# A tree ticking, and a blackboard filling

<div class="content-body">
<BtTickPanel scenario="plan_and_execute" />
</div>

<div class="slide-caption">
Replayed from <code>bt_motion_plan_and_execute.json</code> — a real recording
from <code>tests/bt_node_motion.rs</code>, not an animation. Watch
<code>traj</code> appear in the blackboard the tick <code>plan</code> succeeds.
</div>

---
layout: default
---

# Reading that trace

<div class="split-panel">
<div class="panel text-sm">

The shape of the run, in order:

1. the **`condition`** returns `Success` on the first tick — the start pose is
   collision-free — and the sequence moves on.
2. **`plan`** returns `Running` for several ticks. It is not blocking; it is
   being asked once per tick whether the planner has answered yet. The sequence
   returns `Running` upward each time, and the root does too.
3. the tick `plan` returns `Success`, **`traj` appears in the blackboard** —
   a whole `JointTrajectory`, six joints, eleven points.
4. **`follow_trajectory`** reads it, sends one goal, and returns `Running` while
   the action walks the points.
5. the action's result arrives, the leaf returns `Success`, the sequence has no
   children left, and the **root returns `Success`**.

</div>
<div class="panel text-sm">

<div class="meaning">
<b>Deliberative work, inside a reactive structure</b>
Step 2 is a sampling-based motion planner running to completion. From the
tree's point of view it is a leaf that said <code>Running</code> for a while.
That is the whole trick: the tree does not care how long a leaf takes, so a
planner fits where a reflex would.
</div>

### What the trace records per tick

`tick`, `root_result`, `running_paths`, and a `visited` list giving every node
reached by child-index path with the result it returned — plus `blackboard` and
`blackboard_keys`.

That is exactly what `/bt/status` publishes live. The panel is replaying the
same structure a running system broadcasts.

</div>
</div>

---
layout: default
---

# Validating the blackboard at load time

<div class="split-panel">
<div class="panel">

<<< ../reference/bt_node.rs#blackboard-validation {*}{lines:true,maxHeight:'345px'}

</div>
<div class="panel text-sm">

The parser threads a set of **keys already written by nodes earlier in
pre-order** down the walk. When it reaches a `follow_trajectory`, it checks the
`input_key` against that set:

```rust
return Err(format!(
  "a \"follow_trajectory\" node reads input_key {input_key:?}, \
   which no earlier node writes as an \"output_key\""
));
```

<div class="meaning">
<b>Name the key, not the problem</b>
"a key is missing" sends someone to read the whole tree. <code>input_key
"trajecotry" is never written</code> sends them to the typo. The error message
is part of the design.
</div>

A tree with a dangling key is rejected by `/bt/update_logic` at load — before a
single tick. The alternative is a leaf that fails mysteriously, once, thirty
seconds into a task.

</div>
</div>

---
layout: default
---

# Where validation belongs

<div class="split-panel">
<div class="panel text-sm">

Two rules, the same shape:

- **the only place a raw `"type"` string is compared** against anything is
  `parse_and_validate_node`. Afterwards the tree is a Rust `enum` and the
  compiler checks every match exhaustively.
- **the only place a blackboard key is checked** is the same parse. Afterwards
  every `input_key` is known to have a producer.

<div class="meaning">
<b>Push the check to the boundary</b>
Validate once, at the edge where untrusted JSON becomes a typed value — then
the interior cannot be wrong. A tick-time check would have to run on every tick,
forever, to catch a mistake that was already present at load.
</div>

</div>
<div class="panel text-sm">

### What this buys at the tick

Look back at `hierarchical-dispatch`: there is no error handling in it. No
`"unknown node type"` branch, no `"missing blackboard key"` branch, no
`Result`. It cannot need them — a `BtNode` that exists is a `BtNode` that
parsed.

<div class="history-note">
<b>And it is tested</b>
The implementation has a test that loads a tree whose
<code>follow_trajectory</code> reads a key nothing writes, and asserts the load
is <b>rejected with the key named</b>. Without that test the rule would decay
the first time someone added a leaf kind.
</div>

</div>
</div>

---
layout: default
---

# Cancelling mid-motion

<div class="content-body">
<BtTickPanel scenario="preempt" :speed="950" />
</div>

<div class="slide-caption">
The preempt recording. Watch <code>traj</code> vanish from the blackboard and
reappear, and the root return <code>failure</code> once, as the in-flight goal
is abandoned and the tree re-plans.
</div>

---
layout: default
---

# Longer horizons

<div class="split-panel">
<div class="panel text-sm">

The three-node sequence is the smallest useful tree. A real task nests them:

```
sequence
├── condition   gripper_empty
├── sequence                      ← approach and grasp
│   ├── plan              → traj_a
│   ├── follow_trajectory ← traj_a
│   └── action            close_gripper
└── sequence                      ← carry and place
    ├── plan              → traj_b
    ├── follow_trajectory ← traj_b
    └── action            open_gripper
```

Each `plan` writes its own key; each `follow_trajectory` reads the one beside
it. The load-time rule checks all of them.

</div>
<div class="panel text-sm">

<div class="meaning">
<b>Adding a subtree touches nothing</b>
Wrap the whole thing in a <code>selector</code> with an emergency-stop branch
above it, and no existing node changes — not its JSON, not its code, not its
meaning. The new branch simply has higher priority.
</div>

<div class="history-note">
<b>With one caveat, from two slides ago</b>
Because this implementation's composites have <b>memory</b>, that
emergency-stop branch is only consulted while the outer
<code>selector</code> is choosing — not on every tick once a lower branch is
already <code>Running</code>. For a guard that must hold <i>throughout</i>,
you want a reactive composite, which is a change to the interpreter and not
to the tree.
</div>

### The comparison this sets up

That property — **composability without modification** — is the real argument
for behavior trees, and it is what the rest of this deck measures them against:
a finite state machine, a subsumption architecture, and behavior-based control.

It is *not* an argument about expressive power. That distinction matters, and
the next section makes it carefully.

</div>
</div>

---
layout: default
---

# Asking rather than knowing

<div class="split-panel">
<div class="panel text-sm">

The planner behind the `plan` leaf **holds no geometry at all**. It is a client
of `planning_scene_node`'s **`/check_state_validity`** — MoveIt's own service
name and shape.

<div class="meaning">
<b>One round trip per sample</b>
A sampling planner asks "is this configuration valid?" thousands of times per
plan. Over a service, that is thousands of round trips — which is why the
implementation added <code>/check_state_validity_batch</code>.
</div>

<div class="history-note">
<b>That one is not a MoveIt service</b>
<code>/check_state_validity</code> is real and you will meet it again.
<code>/check_state_validity_batch</code> is <b>this project's own addition</b>,
and the <code>_batch</code> suffix is deliberate so the two cannot be confused.
Students carry these names into jobs; it matters which is which.
</div>

</div>
<div class="panel text-sm">

### The counterweight, stated honestly

Linking the collision checker directly into the planner **would be faster**. No
serialisation, no round trip, no network at all.

The reasons not to:

- the course's concepts *are* nodes connected by topics and services, and a
  planner that asks over a service is the one that demonstrates them
- `rrt_connect.rs` stays **generic over a validity trait**, so the library is
  unit-testable with no networking, while the node supplies a service-calling
  implementation. Both properties at once.

<div class="meaning">
<b>Cross-link</b>
The geometry itself — the inflated-AABB test, the per-link bounding boxes, the
corner conservatism — belongs to the
<a href="/kineval/motion_planning/">motion planning deck</a>, which teaches it
in full. This deck only needs that the question is <i>asked</i>.
</div>

</div>
</div>

---
layout: section
---

# Part 5 — The comparison

### Against what the course already taught

---
layout: default
---

# Finite state machines

<div class="split-panel">
<div class="panel text-sm">

The course defines an FSM as *alphabet, states, transitions, stopping
condition*, and notes it is "commonly implemented as switch-case or if-else
within a while loop" — `autorob_2026_09_fsm_subsumption.pdf`, p.48.

`fsm_node.rs` is exactly that: a fixed interpreter whose own source never
changes, loading its state table live from `/fsm/update_logic`.

<div class="meaning">
<b>The comparison is unusually fair</b>
<code>fsm_node.rs</code> and <code>bt_node.rs</code> are <b>two interpreters in
one repository, driving the same robot, loaded the same way</b>. Neither is a
straw man written to lose an argument.
</div>

</div>
<div class="panel">

<<< ../reference/fsm_node.rs#fsm-transition-model {*}{lines:true,maxHeight:'120px'}

<div class="text-sm mt-2">

A transition is a **name** and a **`next_state`**. That is the whole data model,
and the whole difficulty: a transition points at a state, and the state it
leaves learns nothing about what happens afterwards.

</div>
</div>
</div>

---
layout: default
---

# FSM versus BT: what is actually claimed

<div class="split-panel">
<div class="panel text-sm">

### What is *not* claimed

<div class="meaning">
<b>BTs and FSMs are expressively equivalent</b>
Anything one can express, so can the other. Colledanchise &amp; Ögren prove this
constructively, in both directions. <b>The win is modularity, not power</b> —
and a deck that claims otherwise is wrong.
</div>

### What is claimed

- **two-way control transfer.** A BT child returns a status to its parent; an
  FSM transition is a one-way `goto`.
- **adding a behaviour.** A new BT subtree touches no existing node. A new FSM
  state must be wired into the existing ones — every state that should be able
  to reach it needs a transition, and every state it should be able to reach
  needs one from it.

</div>
<div class="panel text-sm">

### On the growth claim, carefully

It is often said that a flat FSM's transitions grow quadratically in the number
of states. The honest version:

- the **worst case** is quadratic — a fully connected table is `n(n−1)`
  transitions
- real choreographies are far sparser than that

<div class="history-note">
<b>A number this repository cannot supply</b>
<code>fsm_node.rs</code> is an <b>interpreter</b>: its transitions are data
loaded at runtime, not lines of source. "Count the transitions in
<code>fsm_node.rs</code>" has no answer — the count belongs to whichever table
you load. The structural argument above stands on its own and does not need the
number.
</div>

</div>
</div>

---
layout: default
---

# Subsumption architectures

<div class="split-panel">
<div class="panel text-sm">

**Brooks, 1986** — *A Robust Layered Control System for a Mobile Robot*, IEEE
Journal of Robotics and Automation, **RA-2(1):14–23**.

Competence in **layers**: each layer is a complete behaviour from sensing to
action, higher layers are added on top without modifying lower ones, and
higher-priority layers pre-empt lower ones.

The course's own slide (p.69) frames it as:

> Generalization of FSM-based control · Collection of modular reactive
> controllers in a **priority hierarchy** · **Large nested if-else statement**

and p.70 writes that cascade out literally.

<div class="meaning">
<b>That cascade is a priority selector</b>
Which is the cleanest possible bridge from what the course taught to what this
deck teaches — the next panel puts them side by side.
</div>

</div>
<div class="panel text-sm">

### Where they differ

Subsumption's arbitration is **wired into the network topology**: which
suppressor and inhibitor nodes sit on which connections. To see the priority
order you read the wiring diagram.

A behavior tree's arbitration is **the structure itself**: the order of a
selector's children. It is data, it is inspectable, and in this implementation
it is republished every tick on `/bt/status`.

<div class="history-note">
<b>And one thing subsumption has</b>
Layers run <b>concurrently</b> — every layer is always computing, and
arbitration picks among live outputs. A behavior tree ticks one path at a time.
Neither is strictly better; they answer different questions.
</div>

</div>
</div>

---
layout: default
---

# The same arbitration, written twice

<div class="content-body">
<SubsumptionPanel />
</div>

---
layout: default
---

# Inhibition and suppression

<div class="split-panel">
<div class="panel text-sm">

These are the two arbitration operations in Brooks's architecture, and they are
**not the same thing** — the difference is the mechanism of layered control.

<div class="meaning">
<b>Suppression acts on an INPUT</b>
A higher layer's suppressor node <b>replaces</b> the signal arriving at a lower
layer's input, for a specified period. The lower layer keeps running — it is
fed something else.
</div>

<div class="meaning">
<b>Inhibition acts on an OUTPUT</b>
An inhibitor node <b>blocks</b> a lower layer's output from reaching the
actuators, for a specified period. The lower layer keeps running — nothing it
says gets out.
</div>

Both are time-limited, and both are properties of the *wiring*.

</div>
<div class="panel text-sm">

<div class="history-note">
<b>A gap in the course lecture, not an error</b>
<code>autorob_2026_09_fsm_subsumption.pdf</code> names both words exactly once —
p.71, step 4 of the design process: <i>"Implement 'arbitration' processes for
suppression and inhibition..."</i> — and never defines or contrasts them. The
diagram on p.70 draws the layers but not the suppressor and inhibitor nodes.
This slide fills that gap; the attribution is Brooks 1986.
</div>

### The behavior-tree analogue

Neither has a direct one. A selector choosing a higher-priority child is closest
to **inhibition** — the lower behaviour's output never reaches the actuators —
but the lower behaviour is not running at all, so nothing is being blocked.

That difference is real: subsumption suppresses a *running* competitor; a
behavior tree simply does not tick it.

</div>
</div>

---
layout: default
---

# Behavior-based robotics

<div class="split-panel">
<div class="panel text-sm">

**Matarić** and **Arkin**: behaviours as the unit of composition, minimal world
model, and sequence that **emerges** from interaction rather than being planned
in advance.

The course's p.87 contrasts it with subsumption precisely: in a behavior-based
system *"all modules have equal priority, access to sensing, and output motor
commands"*, merged by arbitration or fusion — where subsumption is a strict
hierarchy.

And p.12 gives the two formulations:

> **Hybrid control** — "think and act separately & concurrently"
> **Behavior-based control** — "think the way you act"

<div class="history-note">
<b>Attribution</b>
Those two phrasings are <b>Matarić's</b> (<i>The Robotics Primer</i>, MIT Press,
2007); the course slide gives them unattributed. Arkin 1998 <i>is</i> cited, on
p.11.
</div>

</div>
<div class="panel text-sm">

### What a behavior tree keeps and what it restores

**Keeps:** behaviour as a module with a narrow interface — a leaf that can be
written, tested and reused without knowing what else is in the tree.

**Restores:** explicit sequencing. In a behavior-based system the order is
emergent; in a tree it is written down, inspectable, and editable as data.

<div class="meaning">
<b>And it adds an interface to deliberation</b>
The <code>plan</code> leaf is a motion planner — complete, slow, model-based —
sitting inside a reactive structure as just another child. Neither architecture
this deck compared against has an obvious place to put one.
</div>

</div>
</div>

---
layout: default
---

# The thesis

<div class="split-panel">
<div class="panel text-sm">

The course's p.86 describes **hybrid systems**: a deliberative planner on top,
reactive execution underneath, and an *interface layer to coordinate* — with
the open question, "how to balance long and short term".

<div class="meaning">
<b>A behavior tree is that interface layer, made executable</b>
Reactive, behaviour-based control and deliberative motion planning meet in one
structure you can load as data, inspect while it runs, and extend without
touching what is already there.
</div>

The `plan` leaf is the deliberative half. The `condition` guard above it and the
priority `selector` you can wrap around the whole thing are the reactive half.
The blackboard is how they exchange a value. None of that required a new
architecture — only one more node kind and a set of named slots.

</div>
<div class="panel text-sm">

### What to take away

**A tick is polling.** A leaf starts work, returns `Running`, and reports later.
Everything else follows from that.

**A plan is data; executing it is a process.** The message is inert — it says
nothing about when, whether, or how to stop. The action supplies all three.

**Validate at the boundary.** One place compares strings; after it, the
compiler and the load-time check make whole categories of failure impossible.

**The win is modularity, not power.** BTs and FSMs can express the same things.
One of them lets you add a behaviour without touching the others.

</div>
</div>

---
layout: default
---

# References

<div class="split-panel">
<div class="panel text-sm">

### Primary

- **Colledanchise, M. & Ögren, P.** *Behavior Trees in Robotics and AI: An
  Introduction.* CRC Press, 2018. — the standard reference, and the source of
  the FSM/BT equivalence results.
- **Isla, D.** *Handling Complexity in the Halo 2 AI.* GDC 2005. — the
  canonical origin of behavior trees.
- **Brooks, R. A.** *A Robust Layered Control System for a Mobile Robot.* IEEE
  Journal of Robotics and Automation, **RA-2(1):14–23**, 1986.
- **Arkin, R. C.** *Behavior-Based Robotics.* MIT Press, 1998.
- **Matarić, M. J.** *The Robotics Primer.* MIT Press, 2007.
- **Kuffner, J. J. & LaValle, S. M.** *RRT-Connect: An Efficient Approach to
  Single-Query Path Planning.* ICRA 2000. — the planner behind the `plan` leaf.

</div>
<div class="panel text-sm">

### Practical

- Sebastian Castro, *Introduction to Behavior Trees* —
  [roboticseabass.com](https://roboticseabass.com/2021/05/08/introduction-to-behavior-trees/)
- PickNik, *Behavior Trees* (MoveIt Pro) —
  [docs.picknik.ai](https://docs.picknik.ai/8/concepts/behavior_trees/)
- ROS Wiki, *decision_making / BehaviorTree* —
  [wiki.ros.org](https://wiki.ros.org/decision_making/Tutorials/BehaviorTree)
- **BehaviorTree.CPP** — the library most ROS 2 stacks use
- **`control_msgs/action/FollowJointTrajectory`** and
  `ros2_control`'s `joint_trajectory_controller`

### Course material

- `autorob_2026_09_fsm_subsumption.pdf` — FSMs, subsumption, behavior-based
  control. Quoted here at pp. 12, 48, 69, 70, 71, 86, 87.

</div>
</div>

---
layout: center
class: text-center
---

# Appendices

### The rest of the code, and the provenance

---
layout: default
---

# Appendix: the tick's runtime state

<div class="split-panel">
<div class="panel">

<<< ../reference/bt_node.rs#bt-runtime-state {*}{lines:true,maxHeight:'350px'}

</div>
<div class="panel text-sm">

What a node has to remember *between* ticks, which is exactly the state the
reset-on-decisive-result rule clears.

An `action` or `follow_trajectory` leaf that has started needs to know it
started — the `started_at` instant is what makes a timeout meaningful and what
stops the leaf re-sending its goal on every tick.

<div class="meaning">
<b>This is why Running has to be a return value</b>
If a leaf blocked instead, none of this would exist — and neither would the
ability to abandon it.
</div>

</div>
</div>

---
layout: default
---

# Appendix: snapshots and provenance

<div class="split-panel">
<div class="panel text-sm">

Every Rust file this deck quotes lives in **another repository**,
`pubsub_testbed_rs`. Slidev's `<<<` importer cannot reach outside its own
workspace root, so each file is carried into `behavior_trees/reference/` as a
dated snapshot.

Each snapshot header records:

- the **source repository and path**
- the **upstream length in lines**
- the **commit hash** it was taken at
- the **snapshot date**
- every `#region` marker **with its upstream line number**

<div class="meaning">
<b>Line numbers shift; citations must not</b>
The header pushes everything down, so a line number in the copy is not the line
number upstream. Where a slide cites a line — <code>robot_sim_node.rs</code>
line 1087 is the one that matters — it cites the <b>upstream</b> number, and
the header carries the mapping.
</div>

</div>
<div class="panel text-sm">

### Regions added in the snapshot only

The implementation placed `#region` markers in the files it **wrote**.
`robot_sim_node.rs` and `fsm_node.rs` pre-existed and the work was
additive-only, so they could not be marked upstream.

Their markers — `apply-trajectory` and `fsm-transition-model` — are **additions
in this deck's copy**, and each file's header says so explicitly. Without them
those two slides would have to quote a line range, which breaks silently the
next time the file changes.

### The traces

`bt_motion_plan_and_execute.json` and `bt_motion_preempt.json` are carried
unmodified, headers intact, under `reference/traces/`. Each names the `cargo
test` command that produced it, the commit, and the robot.

<div class="history-note">
<b>Nothing in this deck is mocked</b>
Every tick in every panel happened. The one failure mode a reader cannot check
for themselves is an animation that claims to be a real run and is not — so it
is worth saying plainly that none of these are.
</div>

</div>
</div>

---
layout: center
class: text-center
---

# <span class="accent">Tick</span>, and ask again

<div class="pt-4 text-sm">

`behavior_trees/reference/` &middot;
[the motion planning deck](/kineval/motion_planning/) &middot;
[ocj-dev.github.io/kineval](https://ocj-dev.github.io/kineval/)

</div>

<div class="title-footer">
<div class="credit">AutoRob (autorob.org) &#183; Chad Jenkins (ocj@umich.edu) &#183; Michigan Honor License &#183; 2026</div>
</div>
