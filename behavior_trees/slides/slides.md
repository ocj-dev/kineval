---
theme: default
title: Behavior Trees and Motion-Plan Execution — KinEval Lab
base: /kineval/behavior_trees/
colorSchema: light
info: |
  ## Behavior Trees and Motion-Plan Execution
  A KinEval lab-session walkthrough: what a behavior tree is, how an
  RRT-Connect motion plan becomes a JointTrajectory, how a joint-trajectory
  action executes it, and how a behavior tree sequences that execution into
  tasks longer than one motion. Built for the AutoRob course (autorob.org).
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

# Behavior Trees <span class="accent">and Motion-Plan Execution</span>

### Ticks, statuses, and what happens to a plan after the planner returns it

<div class="pt-8">
<div class="swatch-bar"><span class="swatch amber" /><span class="swatch indigo" />Plan &rarr; JointTrajectory &rarr; Action &rarr; Behavior Tree</div>
</div>

<div class="title-footer">
<div class="nav-hint">Press &rarr; to move forward through the deck</div>
<div class="credit">AutoRob (autorob.org) &#183; Chad Jenkins (ocj@umich.edu) &#183; ocj-dev.github.io/kineval/behavior_trees</div>
</div>

<!--
Title slide. This deck picks up exactly where the motion-planning deck
stopped: RRT-Connect has returned a path, and nothing has moved. The
foundational half follows Sebastian Castro's "Introduction to Behavior
Trees" (roboticseabass.com, 2021), credited on its own slide; the second
half is this module's own reference implementation of a JointTrajectory, a
joint-trajectory action, and the trees that sequence them.
-->

---
layout: default
---

# What this lab builds

<div class="panel text-sm" style="padding:0.6em 0.95em">

A **behavior tree tick engine**, a **motion plan expressed as a JointTrajectory**, a
**joint-trajectory action** that executes it under servo control, and the **trees** that sequence
all of it into tasks longer than one motion.

<table>
<tr><th style="text-align:left">file</th><th style="text-align:left">what is in it</th></tr>
<tr><td><code>behavior_tree.js</code></td><td>statuses, the six node kinds, the tick, halting, the blackboard</td></tr>
<tr><td><code>joint_trajectory.js</code></td><td>a planner path becomes a <code>trajectory_msgs/JointTrajectory</code></td></tr>
<tr><td><code>joint_trajectory_action.js</code></td><td>goal / feedback / result, a P servo, tolerances, cancel</td></tr>
<tr><td><code>bt_tasks.js</code></td><td>every leaf: conditions, actions, and what each reads and writes</td></tr>
<tr><td><code>trees.js</code></td><td>the eight trees this deck draws</td></tr>
<tr><td><code>plans/mr2_arm_workspace.json</code></td><td>a real RRT-Connect plan, with its seed and its counts</td></tr>
</table>

<span class="meaning"><b>Not in scope</b>
This module does not plan. The plan it executes came from <code>kineval/motion_plan.js</code> &mdash; the
RRT-Connect implementation the <a href="/kineval/motion_planning/">motion planning</a> module teaches
&mdash; and is committed to a file that records the seed, the world, and the iteration count.</span>

</div>

---
layout: default
---

# Where the code runs

<div class="panel text-sm">

Three places run the **same** functions, which is what makes every number on these slides checkable:

- the **buildless reference page**, <code>behavior_trees/reference/behavior_trees.html</code> &mdash; plain
  <code>&lt;script src&gt;</code> files, no build step, an orbitable three.js scene and a HUD showing the tick
  trace, the action feedback and the blackboard
- the **panels in this deck**, which use a hand-maintained TypeScript port kept structurally identical
  to the reference &mdash; same function names, same order, same control flow
- **<code>reference/tests/smoke.cjs</code>**, which loads the reference files into one Node `vm` context in
  the same order the page does and asserts on real numbers: 81 checks, covering every node kind, the
  memory/reactive difference, halt propagation, every action result code, and all eight trees

<span class="meaning"><b>The property that makes this possible</b>
Nothing in the tick engine measures time. The caller advances <code>blackboard.dt</code>, so a tree
behaves identically whether it is ticked by an animation loop at 20&nbsp;Hz, by a Step button, or by a
<code>for</code> loop in a test.</span>

</div>

---
layout: section
---

# Part 1 — What a behavior tree is

<div class="panel text-sm" style="max-width:46em;margin:0 auto">
Following <b>Sebastian Castro</b>, <i>"Introduction to Behavior Trees"</i>
(roboticseabass.com, 8 May 2021) &mdash; its definitions, its vocabulary, and its
object-searching example, step by step. Full credit, and a link, on the slide that closes this part.
</div>

---
layout: default
---

# Why a robot needs more than a loop

<div class="panel text-sm">

Abstraction in robotics has moved from **actuator commands** to **sensing** to reasoning about
**behaviors** &mdash; and the question of how to *arrange* behaviors is a separate problem from how to
implement any one of them.

The arrangement has to survive three things that a sequence of function calls does not:

- **actions take time.** "Drive to the pantry" is not a call that returns; it is seconds of motion
  during which other things are true and other things change
- **actions fail**, and failing is normal rather than exceptional. A grasp slips, a path is blocked,
  a battery runs down. Recovery is most of a real task
- **the arrangement gets edited constantly.** The structure that is easy to *extend* beats the one
  that was elegant when first written

<span class="meaning"><b>The claim this part makes</b>
A behavior tree is a way of writing that arrangement so that each of those three has a place to live:
<code>Running</code> for duration, <code>Failure</code> plus the fallback node for recovery, and a tree
for editability.</span>

</div>

---
layout: default
---

# A brief history: from game AI to the ROS 2 navigation stack

<div class="panel text-sm">

Behavior trees come from the **videogame industry**, where the problem was authoring and editing
non-player-character behaviour quickly, and where both **Unreal Engine** and **Unity** ship dedicated
tools for building them. Castro's framing: they caught on because they are "easy to compose and
modify, even at runtime".

They moved into robotics "as robots have become increasingly capable of doing more than simple
repetitive tasks". The reference text is **Colledanchise &amp; Ögren**, <i>Behavior Trees in Robotics
and AI: An Introduction</i> (CRC Press, 2018; arXiv:1709.00084), which is where the formal semantics
used in this deck come from.

In ROS specifically: **BehaviorTree.CPP** (Davide Faconti and Michele Colledanchise) is, in Castro's
words, "quickly gaining traction as **the** behavior tree library of the ROS developers' ecosystem",
and the **ROS 2 navigation stack's BT Navigator** is built on it.

<div class="history-note">
<b>Read the originals</b>
This deck follows Castro's tutorial for the foundational half and the Colledanchise &amp; Ögren text
for the semantics. Where this module's implementation differs from either &mdash; and it does, in a
couple of places &mdash; the slide says so.
</div>

</div>

---
layout: default
---

# The definition, in three properties

<div class="panel text-sm">

Castro defines a behavior tree by three properties, and everything else follows from them:

1. **It is a tree.** Traversal starts at a root node and proceeds in a defined order until a
   terminal state is reached.
2. **Every leaf does something.** "A simple check or a complex action" &mdash; and it outputs a
   **status**: Success, Failure, or Running.
3. **Every internal node dispatches.** It accepts the statuses its children return and applies
   **its own rule** to decide which node is expanded next.

</div>

<div class="panel text-sm" style="margin-top:0.4em">

So the vocabulary splits in two, and the split is worth fixing in mind now:

- **execution nodes** are the leaves &mdash; **conditions** and **actions** &mdash; and they are
  where the robot touches the world
- **control nodes** are the internal ones &mdash; **sequence**, **fallback**, **parallel** and
  **decorator** &mdash; and they contain no robot code at all. They only read statuses.

</div>

---
layout: default
---

# The tick

<div class="panel text-sm">

A behavior tree does nothing until it is **ticked**, and it is ticked repeatedly, at a stated rate.
A tick enters at the root and recurses down whichever children the control nodes select.

Each node a tick reaches returns one of exactly three statuses:

<table>
<tr><td><b>Success</b></td><td>it is done, and it worked</td></tr>
<tr><td><b>Failure</b></td><td>it is done, and it did not work</td></tr>
<tr><td><b>Running</b></td><td>it is not done; ask again next tick</td></tr>
</table>

**Running is the status that matters.** It is what lets a leaf span many ticks: an action that takes
four seconds, ticked at 20&nbsp;Hz, returns Running eighty times and then Success once. Nothing in
the tree ever blocks.

<span class="meaning"><b>Consequence</b>
A tick is a complete, finite traversal. "What is the robot doing?" is not answered by a program
counter sitting inside a function &mdash; it is answered by which leaf returned Running on the last
tick. That is also why this implementation records a <b>trace</b> of every tick.</span>

</div>

---
layout: default
---

# The three statuses, in code

```ts {*}{lines:false}
// behavior_tree.js
```

<<< ../reference/behavior_tree.js#bt-status {*}{lines:true}

<div class="panel text-sm" style="margin-top:0.5em">

There is no fourth value, and in particular there is no "idle". A node that has never been ticked has
**no status at all**, which is a different thing from a node that was ticked and is Running &mdash;
and the difference is visible in every tree drawing in this deck: pale means *not visited by the last
tick*.

</div>

---
layout: default
---

# Execution nodes: conditions and actions

<div class="panel text-sm">

A **condition** answers a question about the world *now*. Castro: it "can only return Success or
Failure within a single tick". Is the gripper open? Is the robot at the pantry? Is the battery above
20%?

An **action** does something, and "can span multiple ticks and can return Running until
[it reaches] a terminal state". Open the door. Drive to the pantry. Execute this trajectory.

</div>

<div class="content-body" style="margin-top:0.4em">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/behavior_tree.js#bt-action {*}{lines:true,maxHeight:'230px'}

</div>
</div>

<div class="slide-caption">
The <code>start</code>/<code>step</code> split is not decoration. "Send the goal" must happen once;
"has it arrived yet" must happen every tick. Collapsing them is how an action re-sends its goal
eighty times.
</div>

---
layout: default
---

# Control nodes: the sequence

<div class="panel text-sm" style="padding:0.5em 0.9em">

Castro: sequence nodes "execute children in order until one child returns Failure or all children
return Success". Click a child below to change what it returns.

</div>

<div class="content-body" style="margin-top:0.4em">
<div class="panel" style="height:100%;padding:0.6em 0.9em">
  <NodeKindPanel kind="sequence" />
</div>
</div>

<div class="slide-caption">
The panel runs <code>btTickSequence</code> from this module's own TypeScript port, so "not ticked" is
observed rather than asserted.
</div>

---
layout: default
---

# Control nodes: the fallback

<div class="panel text-sm" style="padding:0.5em 0.9em">

Fallback nodes "execute children in order until one of them returns Success or all children return
Failure". Castro: "These nodes are key in designing recovery behaviors" &mdash; if one behaviour does
not work, try the next one, and so on.

</div>

<div class="content-body" style="margin-top:0.4em">
<div class="panel" style="height:100%;padding:0.6em 0.9em">
  <NodeKindPanel kind="fallback" />
</div>
</div>

<div class="slide-caption">
Also called a <b>Selector</b> &mdash; which is py_trees' name for it, with slightly different
semantics. The textbook and BehaviorTree.CPP say Fallback, and so does this module.
</div>

---
layout: default
---

# Control nodes: the parallel

<div class="panel text-sm" style="padding:0.5em 0.9em">

Parallel nodes execute all their children "in parallel" &mdash; the quotation marks are Castro's.
Success when at least **M** of N children have succeeded; Failure when M successes are no longer
possible. Move the slider: M&nbsp;=&nbsp;1 is an "or", M&nbsp;=&nbsp;N is an "and".

</div>

<div class="content-body" style="margin-top:0.4em">
<div class="panel" style="height:100%;padding:0.6em 0.9em">
  <NodeKindPanel kind="parallel" :threshold="1" />
</div>
</div>

<div class="slide-caption">
There is no concurrency here. The children are ticked one after another <i>within</i> a tick, in
order; what is parallel is their progress <i>across</i> ticks. The failure threshold is derived, not
configured: once more than N&minus;M have failed, M successes are arithmetically out of reach.
</div>

---
layout: default
---

# Control nodes: the decorator

<div class="panel text-sm">

A decorator "modif[ies] a single child node with a custom policy". One child, exactly. This module
implements four policies:

<table>
<tr><td><code>invert</code></td><td>Success &harr; Failure; Running passes through</td></tr>
<tr><td><code>force-success</code></td><td>Failure &rarr; Success</td></tr>
<tr><td><code>repeat-until-success</code></td><td>Failure &rarr; Running, and tick the child again next tick, up to <code>limit</code> attempts</td></tr>
<tr><td><code>retry</code></td><td>a synonym &mdash; both names appear in the literature</td></tr>
</table>

</div>

<div class="content-body" style="margin-top:0.4em">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/behavior_tree.js#bt-tick-decorator {*}{lines:true,maxHeight:'250px'}

</div>
</div>

---
layout: default
---

# The tick, dispatched

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/behavior_tree.js#bt-tick {*}{lines:true,maxHeight:'330px'}

</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em;padding:0.5em 0.9em">

Six cases, one `switch`, and a `path` string that is the node's position in the tree
(`''` for the root, `'0.2'` for the third child of the first child). Every status is recorded into
`trace` **in visit order** &mdash; which is the thing worth seeing, because visit order is the entire
difference between a sequence and a parallel and is invisible in a drawing.

</div>

---
layout: default
---

# Sequence and fallback, in code

<div class="split-panel">
<div>

<<< ../reference/behavior_tree.js#bt-tick-sequence {*}{lines:true,maxHeight:'330px'}

</div>
<div>

<<< ../reference/behavior_tree.js#bt-tick-fallback {*}{lines:true,maxHeight:'330px'}

</div>
</div>

<div class="slide-caption">
Mirror images: the sequence stops at the first Failure, the fallback at the first Success. Both stop
at the first Running. The <code>haltChildrenFrom</code> calls are the subject of the slide after next.
</div>

---
layout: default
---

# Memory: the choice that surprises everyone

<div class="panel text-sm">

A sequence has to answer one more question that the definition does not: on the **next** tick, does
it resume at the child that was Running, or start again from the first child?

<table>
<tr><td><code>memory: true</code></td><td>the textbook's <b>Sequence</b> &mdash; resume. Children that already returned Success are not re-ticked.</td></tr>
<tr><td><code>memory: false</code></td><td>the textbook's <b>ReactiveSequence</b> &mdash; start again from the first child, re-evaluating every condition that came before the running action.</td></tr>
</table>

The difference only shows up **when something changes underneath a running action**, which is exactly
when it matters. A sequence of `[battery_ok?, follow_trajectory]` with memory executes the whole
trajectory even if the battery dies halfway, because `battery_ok?` already returned Success and is
never asked again.

<span class="meaning"><b>Neither one is the right default</b>
Which is why libraries ship both, and why this is a per-node flag rather than a global one. The two
trees are drawn identically. Part 4 runs both.</span>

</div>

---
layout: default
---

# Halting, and why it is not optional

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/behavior_tree.js#bt-halt {*}{lines:true,maxHeight:'300px'}

</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em;padding:0.5em 0.9em">

A running action **owns something** &mdash; a trajectory being executed, a goal sent to an action
server. When the tree stops ticking it, the action must be told, or the arm keeps following a
trajectory the tree has already abandoned. Leaving this out is not a simplification; it is a bug with
a physical consequence, and Part 4 shows exactly that consequence.

</div>

---
layout: default
---

# The robot example: searching for an object

<div class="panel text-sm">

Castro's running example is a mobile robot looking for coloured blocks around a simulated apartment.
This module implements it literally &mdash; the trees below are ticked, not drawn &mdash; over a
small world held on the blackboard:

<table>
<tr><th style="text-align:left">location</th><th style="text-align:left">what is there</th></tr>
<tr><td>kitchen</td><td>an orange</td></tr>
<tr><td>pantry</td><td><b>the apple</b></td></tr>
<tr><td>garage</td><td>nothing</td></tr>
</table>

The robot starts in the hallway. Each navigation takes three ticks, so Running is visible rather than
instantaneous. **The apple is in the pantry** &mdash; which is what makes each tree in the
progression behave differently from the one before it.

</div>

---
layout: default
---

# Step 1: a sequence, and its flaw

<<< ../reference/trees.js#tree-search-naive {*}{lines:false,maxHeight:'130px'}

<div class="content-body" style="margin-top:0.3em">
<div class="panel" style="height:100%;padding:0.6em 0.9em">
  <TickPanel tree="naive" />
</div>
</div>

<div class="slide-caption">
Correct, and wrong in a way that is invisible until it is ticked twice: it navigates
<b>unconditionally</b>. Ticked again with the robot already in the kitchen, it drives to the kitchen
again. It also fails here &mdash; the apple is in the pantry.
</div>

---
layout: default
---

# Step 2: check before you act

<div class="panel text-sm" style="padding:0.5em 0.9em">

Castro's rule: <b>"you should almost always check before you act"</b> &mdash; expressed as a fallback
whose first child is the condition and whose second is the action that satisfies it.

</div>

<div class="content-body" style="margin-top:0.3em">
<div class="panel" style="height:100%;padding:0.6em 0.9em">
  <TickPanel tree="explicit" start-location="kitchen" />
</div>
</div>

<div class="slide-caption">
Starting in the kitchen, the whole tree resolves in <b>one tick</b> and the navigation action is never
started. The subtree is now idempotent, which is what makes it safe to put under a decorator that
repeats &mdash; see step 5.
</div>

---
layout: default
---

# Step 3: several places to look

<<< ../reference/trees.js#tree-search-multiple-locations {*}{lines:true,maxHeight:'150px'}

<div class="content-body" style="margin-top:0.3em">
<div class="panel" style="height:100%;padding:0.6em 0.9em">
  <TickPanel tree="multiple" :speed-ms="420" />
</div>
</div>

<div class="slide-caption">
The fallback's "if that did not work, try the next one" <b>is</b> the search. Each child fails when
the apple is not in that room, and failure is what advances it &mdash; so the thing most likely to be
read as an error is the mechanism.
</div>

---
layout: default
---

# Step 4: an apple *or* an orange

<div class="panel text-sm" style="padding:0.5em 0.9em">

A parallel with M&nbsp;=&nbsp;1 is an "or"; with M&nbsp;=&nbsp;2 the same tree means "and". The robot
starts in the kitchen, which has an orange and no apple.

</div>

<div class="content-body" style="margin-top:0.3em">
<div class="panel" style="height:100%;padding:0.6em 0.9em">
  <TickPanel tree="parallel" :threshold="1" start-location="kitchen" />
</div>
</div>

<div class="slide-caption">
Both children are ticked on the same tick even after one has succeeded &mdash; the log shows two
lookups, not one. Change <code>:threshold</code> to 2 and the identical tree fails.
</div>

---
layout: default
---

# The blackboard

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/behavior_tree.js#bt-blackboard {*}{lines:true,maxHeight:'300px'}

</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em;padding:0.5em 0.9em">

Castro: "a common storage area where individual behaviors can read or write data". The point is that
**leaves do not know about each other**: the leaf that plans a motion writes `trajectory`, the leaf
that executes one reads `trajectory`, and neither names the other.

The cost, stated plainly: this is global mutable state with a nice name.

</div>

---
layout: default
---

# Step 5: one subtree and a queue

<<< ../reference/trees.js#tree-search-with-decorator {*}{lines:true,maxHeight:'165px'}

<div class="content-body" style="margin-top:0.3em">
<div class="panel" style="height:100%;padding:0.6em 0.9em">
  <TickPanel tree="decorator" :speed-ms="420" />
</div>
</div>

<div class="slide-caption">
Step 3 had one subtree per location and had to be <i>edited</i> to add a fourth. This one reads its
locations from the blackboard, so a fourth location is a <b>data</b> change. Castro credits the
pattern to Davide Faconti; BehaviorTree.CPP calls the decorator <code>RetryUntilSuccessful</code>.
</div>

---
layout: default
---

# How that tree terminates

<div class="panel text-sm">

Worth tracing, because it is the one piece of the refactoring that is not obvious.

The decorator repeats while its child **fails**. The child fails either because

- the object was not at this location &mdash; in which case repeating is exactly right, and
  "Get next location" supplies the next one; **or**
- the queue has run dry &mdash; in which case repeating is pointless.

Those two failures are **indistinguishable to the decorator**. The attempt limit is what separates
them, and this module sets it from the queue's own length, which is the honest version of a loop
bound. `taskNextLocation` returning Failure on an empty queue is the belt to that braces:

</div>

<<< ../reference/bt_tasks.js#task-next-location {*}{lines:true,maxHeight:'185px'}

---
layout: default
---

# Declaring what a leaf touches

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/behavior_tree.js#bt-blackboard-require {*}{lines:true,maxHeight:'300px'}

</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em;padding:0.5em 0.9em">

Every leaf in `bt_tasks.js` carries `reads` and `writes` arrays. This walks the tree before the first
tick and reports any key a leaf reads that nothing writes and the initial blackboard does not have.
It is a **linter, not a type system**: it cannot check that a key holds the right kind of value, and
it cannot know whether the branch that writes a key runs before the branch that reads it.

</div>

---
layout: default
---

# The libraries you would actually use

<div class="panel text-sm">

**py_trees** (Daniel Stonier, Python). Flexible because it is interpreted &mdash; modifying a tree at
runtime is practical. Actively developed, with its focus on ROS 2; its visualisation and logging
tools integrate well there. Note the vocabulary differs: it says **Selector** where the textbook says
Fallback, with slightly different behaviour.

**BehaviorTree.CPP** (Davide Faconti and Michele Colledanchise, C++). Follows the textbook's notation
closely &mdash; one of its authors wrote the book. Trees are declared in **XML** and node types
registered as C++ classes, and **Groot** is its visual editor and monitor. Excellent when the tree's
shape is known ahead of time; building one programmatically at runtime needs workarounds.

Castro's own summary: both are mature, both integrate with ROS, and the choice is mostly C++ versus
Python. The **ROS 2 navigation stack's BT Navigator** uses BehaviorTree.CPP.

<span class="meaning"><b>What this module is</b>
Not a competitor to either. It is the smallest implementation that is still honest about the three
things those libraries get asked about and toy implementations skip: memory, halting, and the tick
trace.</span>

</div>

---
layout: default
---

# Credit where it is due

<div class="panel">

The whole of Part 1 &mdash; its definitions, its vocabulary, its node-by-node progression and its
object-searching example &mdash; follows:

<div class="link-list" style="margin-top:0.5em">
<p><strong><a href="https://roboticseabass.com/2021/05/08/introduction-to-behavior-trees/">Sebastian Castro, &ldquo;Introduction to Behavior Trees&rdquo;</a></strong><br>
roboticseabass.com, 8 May 2021. Read it &mdash; it is better than this summary of it, and its
figures are worth the visit on their own.</p>
<p><strong><a href="https://github.com/sea-bass/turtlebot3_behavior_demos">github.com/sea-bass/turtlebot3_behavior_demos</a></strong><br>
Castro's companion repository: the same example implemented in both py_trees and BehaviorTree.CPP.</p>
<p><strong><a href="https://arxiv.org/abs/1709.00084">Colledanchise &amp; Ögren, <i>Behavior Trees in Robotics and AI: An Introduction</i></a></strong><br>
CRC Press, 2018. The formal semantics this module implements. Castro's verdict: "I cannot recommend
this one enough."</p>
</div>

<span class="meaning"><b>On the figures</b>
The diagrams in this deck are this module's own, drawn live from the trees being discussed rather
than reproduced from the tutorial. The pedagogy &mdash; which example, in which order, making which
point &mdash; is Castro's.</span>

</div>

---
layout: section
---

# Part 2 — A motion plan is data

<div class="panel text-sm" style="max-width:46em;margin:0 auto">
RRT-Connect has returned a path and nothing has moved. What has to be added to that path before a
controller will accept it &mdash; and what a <code>trajectory_msgs/JointTrajectory</code> can and
cannot carry.
</div>

---
layout: default
---

# Where the motion-planning module stopped

<div class="panel text-sm">

The <a href="/kineval/motion_planning/">motion planning</a> module's RRT-Connect returns
`planner.path`: an array of **configurations**, each a full `q` vector. This module's plan was
produced by running that planner &mdash; `kineval/motion_plan.js`, unmodified, in a Node `vm`
context &mdash; and committing the result:

<table>
<tr><td>robot</td><td><code>mr2</code>, right arm, base pinned (<code>plan_base_dofs: 'none'</code>)</td></tr>
<tr><td>world</td><td>24 obstacle spheres, r&nbsp;=&nbsp;0.3, sampled in the arm's own workspace</td></tr>
<tr><td>seed</td><td>7</td></tr>
<tr><td>search</td><td><b>46 iterations</b>, 3426 collision checks, 365 nearest-neighbour comparisons</td></tr>
<tr><td>result</td><td>an <b>18</b>-configuration raw path, <b>5</b> after shortcut smoothing</td></tr>
</table>

<span class="meaning"><b>Why it is committed rather than planned in the browser</b>
This module is about what happens to a plan <i>after</i> the planner returns one. Replanning in every
panel would make each picture depend on a search, and a slide could not say "five waypoints" and be
sure. The file records the seed, so it is reproducible rather than merely fixed.</span>

</div>

---
layout: default
---

# The world the plan was found in

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/worlds/world_arm_workspace.js#create-world-arm-workspace {*}{lines:true,maxHeight:'330px'}

</div>
</div>

<div class="slide-caption">
This module's own world, and the reason is measurable: with the base pinned, planning in the viewer's
<code>world_arm_clutter</code> &mdash; eighteen spheres over a 10&times;10 floor &mdash; solves in
<b>one iteration</b> with a two-configuration path, because almost nothing is within the arm's reach.
A plan with no intermediate waypoints teaches nothing about trajectories.
</div>

---
layout: default
---

# Rejection sampling, and the defect it answers

<div class="panel text-sm">

Every candidate sphere is tested against the robot at **both** the start and the goal configuration,
inflated by a margin, and resampled if it touches either.

That is a direct response to a defect recorded in the viewer's own `motion_plan.js`: the first
version of `world_sealed_enclosure` sized its ring so that, for several robots, the **start pose was
already in collision** with the enclosure, and the planner could not run at all. A generator that is
handed the query it is generating for can simply refuse to produce that situation.

<span class="meaning"><b>The cost, stated</b>
This world is defined relative to one query rather than standing alone, which is why its factory
takes the robot and both configurations, where the upstream generators take only
<code>{extent, reach}</code>.</span>

</div>

<div class="panel text-sm" style="margin-top:0.4em;padding:0.5em 0.9em">

Measured over a sweep of obstacle counts, radii and seeds, the chosen parameters
(24 spheres, r&nbsp;=&nbsp;0.3, seed&nbsp;7) were picked for being **genuinely constrained**: tens of
iterations rather than one, and waypoints in the middle rather than a straight line.

</div>

---
layout: default
---

# The plan, and the three things it is missing

<div class="panel text-sm">

A plan is a list of configurations. A controller will not accept one, because it does not say:

1. **TIME.** No statement about *when* the robot should be at each configuration. A trajectory is a
   function of time, and choosing that function is **time parameterization**.
2. **NAMES.** The plan is indexed by the planner's own `q` layout &mdash; six base DOFs, then one
   entry per non-fixed joint in `robot.joints` order. A trajectory carries `joint_names`, and
   `points[i].positions[j]` means *the position of* `joint_names[j]`. The numbers alone are
   meaningless.
3. **A CONTRACT ABOUT WHAT IS NOT MENTIONED.** A JointTrajectory says nothing about joints it does
   not name &mdash; which is not the same as commanding them to hold still.

</div>

<<< ../reference/joint_trajectory.js#plan-joint-columns {*}{lines:true,maxHeight:'195px'}

---
layout: default
---

# trajectory_msgs/JointTrajectory

<div class="split-panel">
<div>
<div class="panel text-sm" style="padding:0.6em 0.9em">

```
std_msgs/Header          header
string[]                 joint_names
JointTrajectoryPoint[]   points
```

and each point:

```
float64[]                    positions
float64[]                    velocities
float64[]                    accelerations
float64[]                    effort
builtin_interfaces/Duration  time_from_start
```

</div>
</div>
<div>
<div class="panel text-sm" style="padding:0.6em 0.9em">

Read from `ros2/common_interfaces`, not from memory. This module uses **the ROS field names, in
snake_case**, deliberately out of step with the rest of its JavaScript, so that
`point.time_from_start` here and `point.time_from_start` in a real message are the same words.

Two simplifications, named rather than glossed:

- `time_from_start` is a `Duration` &mdash; integer seconds plus integer nanoseconds. Here it is a
  float number of seconds.
- the message documents its units: **radians** for revolute and continuous joints, metres for
  prismatic. Every joint this module commands is continuous.

</div>
</div>
</div>

---
layout: default
---

# Time parameterization

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/joint_trajectory.js#time-parameterization {*}{lines:true,maxHeight:'330px'}

</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em;padding:0.5em 0.9em">

Constant speed, longest joint wins: each segment lasts `max_j |Δq_j| / v_max`, so every joint in a
segment starts and stops together and the joint with furthest to travel sets the pace. The duration
becomes a property of the **plan** rather than of a guess.

</div>

---
layout: default
---

# The plan, plotted against time

<div class="content-body">
<div class="panel" style="height:100%;padding:0.6em 0.9em">
  <TrajectoryPanel />
</div>
</div>

<div class="slide-caption">
Four joints, five waypoints, 9.83&nbsp;s at the default v<sub>max</sub>&nbsp;=&nbsp;0.6&nbsp;rad/s.
Drag <code>t</code> to sample. Switch to the <b>raw path</b> to see what smoothing removed: eighteen
waypoints of the same motion. Every curve is <code>trajectoryFromPlan</code>'s output on the
committed file.
</div>

---
layout: default
---

# What constant speed costs

<div class="panel text-sm">

Stated rather than hidden, because all three bite on real hardware:

- **the velocity profile is a square wave.** Each segment runs at constant speed and the speed
  changes discontinuously at every waypoint, which implies infinite acceleration. A real controller
  will not do that &mdash; it will lag, and the lag is visible as tracking error in Part 3.
  Trapezoidal and spline parameterizations (ROS's own `time_optimal_trajectory_generation`,
  Reflexxes, Ruckig) exist to fix exactly this.
- **per-joint velocity limits are ignored**, because `mr2`'s description declares none. A real
  controller reads them from the URDF's `<limit velocity="...">`.
- **acceleration limits are not mentioned at all**, and on real hardware that is usually the
  constraint that actually binds.

<span class="meaning"><b>The pattern worth taking away</b>
Every one of these is a place where a number has to come from somewhere. A named constant that says
where is better than one buried in a formula.</span>

</div>

---
layout: default
---

# The velocities field, and the trap in leaving it empty

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/joint_trajectory.js#trajectory-velocities {*}{lines:true,maxHeight:'300px'}

</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em;padding:0.5em 0.9em">

An empty `velocities` array means **unspecified**, and what a controller does with unspecified
velocities is controller-specific. One interpolates positions only; another assumes zero velocity at
each waypoint and therefore comes to a **complete stop at every one of them**. A five-waypoint
trajectory that stops four times is a common and baffling symptom &mdash; caused by a field nobody
filled in.

</div>

---
layout: default
---

# Why the interpolation is linear

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/joint_trajectory.js#sample-trajectory {*}{lines:true,maxHeight:'330px'}

</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em;padding:0.5em 0.9em">

The planner validated the **straight line in configuration space** between consecutive waypoints
&mdash; `edgeIsValid` checks samples along exactly that segment. Linear interpolation reproduces it,
so a sampled configuration is collision-free for the same reason the plan is.

</div>

---
layout: default
---

# A spline through the same waypoints

<div class="content-body">
<div class="panel" style="height:100%;padding:0.6em 0.9em">
  <TrajectoryPanel :show-raw-toggle="false" />
</div>
</div>

<div class="slide-caption">
Tick <b>overlay a spline</b>. It is smoother, and it leaves the validated segment by a measured
amount that no part of the pipeline downstream would report. Smoothness here is bought with validity
&mdash; which is a trade worth making sometimes, and worth making <i>knowingly</i> always.
</div>

---
layout: default
---

# The base is not a joint

<div class="panel text-sm">

KinEval's `q` begins with six base DOFs: `base_x, base_y, base_z, base_roll, base_pitch, base_yaw`.
A JointTrajectory's `positions` are **single-DOF joint values** &mdash; the message says so itself.
The base does not fit.

ROS's answer is a second message, carried alongside in the same goal:

```
trajectory_msgs/JointTrajectory          trajectory
trajectory_msgs/MultiDOFJointTrajectory  multi_dof_trajectory   # "for all planar or
                                                                #  floating joints"
```

So the base is not unrepresentable &mdash; it is representable in a **different field of a different
type**. This module implements only the single-DOF half, and **refuses** a plan whose base moves
rather than quietly dropping six columns.

<span class="meaning"><b>And that refusal is the hinge of this whole deck</b>
A motion that moves both base and arm has to be split into two goals, of two different kinds, that
something has to sequence. Sequencing exactly that is what a behavior tree is for.</span>

</div>

---
layout: default
---

# Refusing, rather than guessing

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/joint_trajectory.js#trajectory-from-plan {*}{lines:true,maxHeight:'335px'}

</div>
</div>

<div class="slide-caption">
Three refusals, each a real mistake with a quiet failure mode: a one-configuration "plan" (a pose,
not a motion), a plan whose base moves, and a joint name the layout does not contain.
</div>

---
layout: default
---

# The joints a trajectory says nothing about

<div class="split-panel">
<div>

<<< ../reference/joint_trajectory.js#trajectory-joints-not-named {*}{lines:true,maxHeight:'330px'}

</div>
<div>
<div class="panel text-sm" style="padding:0.6em 0.9em">

`mr2` has five non-fixed joints. This module's trajectories name **four** of them: the right arm's
chain. `clavicle_left_roll` &mdash; the stub of the left arm upstream leaves as an exercise &mdash;
is never mentioned.

An unmentioned joint is **not commanded to hold position**. It is simply not mentioned, and whatever
was driving it before carries on driving it.

<span class="meaning"><b>Why this is worth a function</b>
The natural reading of a JointTrajectory is "this is what the robot does". It is not. It is what the
<i>named</i> joints do. Someone who sees an unnamed joint drift during an otherwise correct execution
will go looking for a bug in the trajectory, which is the wrong place.</span>

</div>
</div>
</div>

---
layout: default
---

# Everything checkable without a robot

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/joint_trajectory.js#validate-trajectory {*}{lines:true,maxHeight:'335px'}

</div>
</div>

<div class="slide-caption">
Every one of these presents as <i>a motion that looks wrong</i> rather than as an error: a
non-monotonic time makes the arm jump backwards, a short <code>positions</code> array drives the
wrong joints, and one <code>NaN</code> propagates into every joint angle on the first interpolation.
</div>

---
layout: section
---

# Part 3 — Executing it

<div class="panel text-sm" style="max-width:46em;margin:0 auto">
A trajectory is still only data. What turns it into motion is an <b>action</b>: a goal that can be
accepted or rejected, feedback while it runs, a result when it stops, and the ability to change your
mind.
</div>

---
layout: default
---

# Topic, service, action

<div class="panel text-sm">

<table>
<tr><th style="text-align:left">pattern</th><th style="text-align:left">shape</th><th style="text-align:left">the robotics example</th></tr>
<tr><td><b>topic</b></td><td>one-way, anonymous, no reply</td><td><b>joint state</b> &mdash; a fact about now, published continuously. Nobody acknowledges a fact.</td></tr>
<tr><td><b>service</b></td><td>request/reply, blocking, no progress</td><td><b>collision checking</b> &mdash; ask, get an answer, done in milliseconds.</td></tr>
<tr><td><b>action</b></td><td>goal/feedback/result, non-blocking, cancellable</td><td><b>motion execution</b> &mdash; seconds long; the caller needs progress and the ability to stop it.</td></tr>
</table>

**Publishing a trajectory on a topic** is a fire-and-forget statement about what should happen. It
gives the sender nothing: no acknowledgement that the trajectory was accepted, no progress while it
runs, no verdict when it stops, and no way to change its mind. Executing a motion needs all four.

<span class="meaning"><b>The ROS answer</b>
"A long-running request that reports progress and can be cancelled" is precisely what an action is
for.</span>

</div>

---
layout: default
---

# control_msgs/FollowJointTrajectory

<div class="panel text-sm" style="padding:0.6em 0.9em">

```
#### GOAL ####
trajectory_msgs/JointTrajectory          trajectory
trajectory_msgs/MultiDOFJointTrajectory  multi_dof_trajectory
JointTolerance[]                         path_tolerance          # violated -> abort
JointTolerance[]                         goal_tolerance          # must hold at the end
builtin_interfaces/Duration              goal_time_tolerance     # leeway past the end

#### RESULT ####
int32   error_code        # 0 SUCCESSFUL   -1 INVALID_GOAL   -2 INVALID_JOINTS
string  error_string      # -3 OLD_HEADER_TIMESTAMP   -4 PATH_TOLERANCE_VIOLATED
                          # -5 GOAL_TOLERANCE_VIOLATED

#### FEEDBACK ####
std_msgs/Header  header     string[]  joint_names   int32  index
JointTrajectoryPoint  desired,  actual,  error
```

<div class="slide-caption" style="margin-top:0.2em">
In a real <code>.action</code> file those three blocks are separated by <code>---</code> lines; they
are spelled out here because a bare <code>---</code> inside this deck's markdown would start a new
slide.
</div>

</div>

<div class="slide-caption">
Read from <code>ros-controls/control_msgs</code>. Note the shape of the feedback: <b>desired, actual
and error</b> are three separate trajectory points. A controller that cannot tell them apart has
nothing to report.
</div>

---
layout: default
---

# The constants, and where they came from

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/joint_trajectory_action.js#action-constants {*}{lines:true,maxHeight:'335px'}

</div>
</div>

<div class="slide-caption">
The goal lifecycle in ROS 2's names. ROS 1's actionlib called these PENDING / ACTIVE / PREEMPTED /
SUCCEEDED / ABORTED / REJECTED, and "preempt" is still the word everyone says for a cancel.
</div>

---
layout: default
---

# The controller is a servo, not a teleport

<div class="panel text-sm">

This module does **not** assign the sampled trajectory position to `robot.joints[x].angle`. It writes
`robot.joints[x].control` and lets `applyControls` fold that into `.angle` &mdash; the same path every
other control input in KinEval takes:

```js
joint.control += server.gain * (sample.positions[i] - joint.angle);
```

That is the whole reason `error` is a field in the feedback message: **the desired position and the
actual position are different numbers**, and the difference is the only thing that makes the
tolerance fields meaningful.

<span class="meaning"><b>Contrast with the motion-planning module</b>
Its <code>n</code>/<code>b</code> plan traversal assigns <code>.angle</code> directly, deliberately
&mdash; it is <i>inspecting</i> plan vertices, not executing them. Assign <code>.angle</code> here and
every error is identically zero, every tolerance is unfalsifiable, and the arm snaps between
waypoints. The two look similar for about a second and are not the same claim.</span>

</div>

---
layout: default
---

# Accept, or reject, before anything moves

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/joint_trajectory_action.js#action-send-goal {*}{lines:true,maxHeight:'335px'}

</div>
</div>

<div class="slide-caption">
An action server that accepts every goal and discovers the problem while executing has converted a
programming error into a robot moving wrongly. A new goal <b>preempts</b> the running one, which is
ROS's own convention &mdash; queueing instead would let a tree that re-sends every tick accumulate
thousands of goals.
</div>

---
layout: default
---

# One control cycle, in order

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/joint_trajectory_action.js#action-update {*}{lines:true,maxHeight:'335px'}

</div>
</div>

<div class="slide-caption">
Advance the clock, sample, command, apply, <b>then</b> measure, then judge. Measuring before applying
makes the reported error consistently one cycle stale &mdash; at 20&nbsp;Hz, enough to make a
tolerance plot disagree with the arm on screen.
</div>

---
layout: default
---

# Watching it execute

<div class="content-body">
<div class="panel" style="height:100%;padding:0.6em 0.9em">
  <ExecutionPanel mode="action" :show-tree="false" />
</div>
</div>

<div class="slide-caption">
Magenta is <b>desired</b>, orange is <b>actual</b>; the gap between them is the feedback message's
<code>error</code>. Grey spheres are the plan's five waypoints, blue the start and green the goal.
Drag the <b>gain</b> down and the arm falls behind; drag the <b>path tolerance</b> below the tracking
error and the goal aborts with <code>error_code&nbsp;-4</code>.
</div>

---
layout: default
---

# The three tolerances

<div class="panel text-sm">

<table>
<tr><td><b>path tolerance</b></td><td>per-joint tracking error that aborts the goal <i>while it runs</i>. A statement about the <b>controller's health</b>, not about the plan: exceeding it means the arm is not where the trajectory says it should be, which is what hitting something looks like.</td></tr>
<tr><td><b>goal tolerance</b></td><td>per-joint error at the final waypoint that counts as arrived. Much tighter &mdash; ending up at that configuration was the point of the motion.</td></tr>
<tr><td><b>goal time tolerance</b></td><td>extra time past the trajectory's nominal duration in which to converge.</td></tr>
</table>

The third one is not a nicety. A P servo approaches its setpoint **asymptotically**, so it is still
converging when the clock runs out; without a grace period, every single goal would report
`GOAL_TOLERANCE_VIOLATED`. That is exactly the bug the constant prevents, and exactly the bug its
absence looks like.

<span class="meaning"><b>Measured, not guessed</b>
On the committed plan at gain 0.35 and 20&nbsp;Hz, the worst tracking error peaks at
<b>0.056&nbsp;rad</b> and the final error settles to <b>0.013&nbsp;rad</b> &mdash; comfortably inside
the 0.25 path tolerance and the 0.02 goal tolerance. <code>smoke.cjs</code> asserts both.</span>

</div>

---
layout: default
---

# Cancelling

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/joint_trajectory_action.js#action-cancel {*}{lines:true,maxHeight:'335px'}

</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em;padding:0.5em 0.9em">

"Stop" does **not** mean stop writing to the robot. It means stop advancing along the trajectory: the
last commanded setpoint stays where it was, so the arm holds the configuration it reached. A cancel
that simply stopped calling the controller would leave the joints wherever the last partial control
delta put them.

</div>

---
layout: default
---

# Checking before acting, with the tolerance that must match

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/joint_trajectory_action.js#action-at-configuration {*}{lines:true,maxHeight:'300px'}

</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em;padding:0.5em 0.9em">

This lives next to the action's goal tolerance because the two numbers **must agree**. If the
condition were tighter than the tolerance the action succeeds on, a fallback of
`[check, move]` would move, succeed, be re-ticked, find itself not arrived, and move again &mdash;
forever, with the arm visibly still.

</div>

---
layout: section
---

# Part 4 — Sequencing it

<div class="panel text-sm" style="max-width:46em;margin:0 auto">
One motion is a goal. A task is several motions, each of which can fail, with checks in between and
something watching the whole time. That is the tree's job.
</div>

---
layout: default
---

# A leaf that does not block

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/bt_tasks.js#task-follow-trajectory {*}{lines:true,maxHeight:'335px'}

</div>
</div>

<div class="slide-caption">
<code>start</code> sends the goal once; <code>step</code> advances the controller one cycle and asks.
Sending from <code>step</code> would re-send every tick &mdash; and because a new goal preempts the
running one, the arm would restart the trajectory from the top on every tick and never move.
</div>

---
layout: default
---

# Six states onto three statuses

<div class="panel text-sm">

<table>
<tr><th style="text-align:left">action state</th><th style="text-align:left">BT status</th></tr>
<tr><td><code>EXECUTING</code> / <code>ACCEPTED</code></td><td>Running</td></tr>
<tr><td><code>SUCCEEDED</code></td><td>Success</td></tr>
<tr><td><code>ABORTED</code></td><td>Failure</td></tr>
<tr><td><code>CANCELED</code></td><td>Failure</td></tr>
<tr><td>goal rejected</td><td>Failure</td></tr>
</table>

The two vocabularies are different sizes, and **that asymmetry is where information is lost**. Why a
motion failed &mdash; path tolerance, bad joint names, preempted by another goal &mdash; is not
visible to the tree. The tree can only see *that* it failed.

<span class="meaning"><b>So anything that wants to react differently to different failures</b>
has to read the result off the blackboard, which is why this leaf writes <code>last_result</code>.
That is a design decision with a cost, not a law of behavior trees.</span>

</div>

---
layout: default
---

# Plan, then execute

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/trees.js#tree-plan-and-execute {*}{lines:true,maxHeight:'300px'}

</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em;padding:0.5em 0.9em">

The smallest tree that is about this module's subject. The **sequence** is why planning and execution
are separable &mdash; the planner's output lands on the blackboard and the executor picks it up. The
**fallback** is why the tree can be ticked forever: once the arm has arrived, the condition succeeds
and nothing replans.

</div>

---
layout: default
---

# Plan, then execute &mdash; running

<div class="content-body">
<div class="panel" style="height:100%;padding:0.6em 0.9em">
  <ExecutionPanel mode="plan_execute" />
</div>
</div>

<div class="slide-caption">
Press <b>Play</b>. "Plan motion" runs for six ticks, then the execute leaf sends the goal and returns
Running for the whole motion. On the first tick <code>trajectory</code> is undefined, so the
check-before-you-act condition fails and the sequence runs &mdash; which is why that condition is
allowed to read a key the planner writes.
</div>

---
layout: default
---

# The planning leaf, and what it honestly is

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/bt_tasks.js#task-plan-motion {*}{lines:true,maxHeight:'335px'}

</div>
</div>

<div class="slide-caption">
In a system with a live planner this is the node that sends a planning request &mdash; in MoveIt, a
<code>/plan_kinematic_path</code> service call or a MoveGroup action goal. Here it <b>replays</b> a
committed plan, returning Running for a few ticks so the tree has the shape it would have with a real
planner: an action that takes time, can be waited on, and can be abandoned.
</div>

---
layout: default
---

# A longer horizon

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/trees.js#tree-inspection-queue {*}{lines:true,maxHeight:'300px'}

</div>
</div>

<div class="panel text-sm" style="margin-top:0.4em;padding:0.5em 0.9em">

Three inspection poses, visited in turn, stopping at the first one that has the part. One motion per
cycle, each a full plan-to-trajectory-to-action pipeline, and nothing here knows how many targets
there are except the queue and the decorator's limit.

</div>

---
layout: default
---

# A longer horizon &mdash; running

<div class="content-body">
<div class="panel" style="height:100%;padding:0.6em 0.9em">
  <ExecutionPanel mode="inspection" />
</div>
</div>

<div class="slide-caption">
The part is at <code>bin_c</code>, so all three are visited: two trajectories succeed and their
inspections fail, which is what makes the decorator repeat. Structurally <b>identical</b> to the
apple-searching tree of Part 1, with navigation replaced by trajectory execution.
</div>

---
layout: default
---

# The argument, made as a diff

<div class="panel text-sm">

Compare the two trees directly:

<table>
<tr><th style="text-align:left"></th><th style="text-align:left">searching an apartment</th><th style="text-align:left">inspecting three bins</th></tr>
<tr><td>decorator</td><td>repeat until success, limit = locations</td><td>repeat until success, limit = targets</td></tr>
<tr><td>first leaf</td><td>Get next location</td><td>Get next inspection target</td></tr>
<tr><td>fallback</td><td>At target? / Navigate to target</td><td>At the goal configuration? / Follow joint trajectory</td></tr>
<tr><td>last leaf</td><td>Found the object?</td><td>Part present?</td></tr>
</table>

The task got a great deal more real &mdash; a real planner's output, a real message, a real
controller with real tracking error &mdash; and **the tree barely changed**.

<span class="meaning"><b>That is the modularity claim</b>
Not "behavior trees are nice", but: the structure that sequences the work is independent of what the
work is, and swapping the work does not rewrite the structure.</span>

</div>

---
layout: default
---

# Honest about the hand-authored part

<div class="panel text-sm">

The three inspection poses are **hand-authored**, not planned. Each is reached by a straight
two-waypoint trajectory, so those motions are **not collision-checked** against the world the way the
committed plan is.

That is a real limitation and it is marked as one in the code, in
`infrastructure.js#inspection-poses`. The alternative would be either three planner runs on page load
or one pretend plan, and both are worse than saying which is which.

<span class="meaning"><b>Which slide is about which</b>
The <b>plan-and-execute</b> tree is about one motion being a real plan. The <b>inspection-queue</b>
tree is about sequencing several motions. Conflating them would make both claims weaker.</span>

</div>

---
layout: default
---

# Reactivity: the same tree, twice

<div class="content-body">
<div class="panel" style="height:100%;padding:0.5em 0.7em">

<<< ../reference/trees.js#tree-reactive-battery {*}{lines:true,maxHeight:'330px'}

</div>
</div>

<div class="slide-caption">
One flag &mdash; <code>memory: !reactive</code> on the outer sequence &mdash; is the entire
difference between the two runs on the next two slides. The drawing of the tree is identical either
way.
</div>

---
layout: default
---

# Reactive: the guard fires

<div class="content-body">
<div class="panel" style="height:100%;padding:0.6em 0.9em">
  <ExecutionPanel mode="reactive" :reactive="true" />
</div>
</div>

<div class="slide-caption">
Battery starts at 30% and drains 0.6% per tick &mdash; in the <b>world</b>, not in the condition, so
it drains whether or not the tree asks. The guard is re-evaluated every tick, the motion is cancelled
mid-flight, and the robot charges.
</div>

---
layout: default
---

# The same tree with memory

<div class="content-body">
<div class="panel" style="height:100%;padding:0.6em 0.9em">
  <ExecutionPanel mode="reactive" :reactive="false" />
</div>
</div>

<div class="slide-caption">
Identical drawing, identical leaves, one flag different. The battery is checked once at the start of
the traversal and never again, so the arm finishes the whole trajectory on a flat battery. The tree is
not wrong &mdash; it is just not reactive, and nothing in the picture says which one you are looking
at.
</div>

---
layout: default
---

# Four things had to line up

<div class="panel text-sm">

For that cancellation to actually stop the arm, every one of these had to exist:

1. the sequence's **no-memory flag**, so the guard is re-evaluated at all
2. **halt propagation** &mdash; `haltChildrenFrom` on the Failure path, so the abandoned subtree is
   told
3. the leaf's **halt handler**, which is the only thing that knows this leaf owns an action goal
4. the action server's **cancel**, which is what actually stops advancing along the trajectory

Miss any one and the tree takes the charging branch **while the arm carries on moving** &mdash; which
looks, on screen, exactly like a behavior tree that works.

</div>

<<< ../reference/behavior_tree.js#bt-halt-children-from {*}{lines:true,maxHeight:'180px'}

---
layout: default
---

# A plan is only valid from where it was planned

<div class="panel text-sm">

Watch the reactive panel past the first charge: the tree replans and re-executes, and the trajectory
it executes still **starts at the plan's original start configuration** &mdash; which is not where
the arm is, because the cancel left it part-way along.

The servo drives to that first waypoint anyway, smoothly and without complaint, because nothing in
the trajectory says where the robot is supposed to be when it begins.

<span class="meaning"><b>What a real system does about it</b>
Replans from the current configuration. "Plan motion" is a leaf that would call a planner with the
robot's present state as the start; this module replays one recorded plan, so it cannot. The
difference is invisible in the tree, which is exactly why it is worth saying here: a tree is only as
correct as its leaves' contracts with the world.</span>

</div>

---
layout: section
---

# Part 5 — Behavior trees and state machines

<div class="panel text-sm" style="max-width:46em;margin:0 auto">
Castro's comparison, and his conclusion: "There is a tradeoff between <b>modularity</b> and
<b>reactivity</b>."
</div>

---
layout: default
---

# The same task, written twice

<div class="content-body">
<div class="panel" style="height:100%;padding:0.6em 0.9em">
  <FsmVsBtPanel />
</div>
</div>

<div class="slide-caption">
A picking task: move to the object, close the gripper, return home. Both are clean. Turn on the
switch to add a pre-grasp alignment check &mdash; the <b>same</b> change to both.
</div>

---
layout: default
---

# The modularity argument

<div class="panel text-sm">

Castro: *"With a BT, we can directly insert a subtree along our desired sequence of actions, whereas
with a FSM we must rewire multiple transitions."*

The panel makes that countable. Inserting the check adds **three nodes at one point** to the tree and
changes nothing that was already there. In the machine it adds **two states and three transitions**,
and one existing transition &mdash; `move → close` &mdash; **stops existing**.

The asymmetry is structural rather than incidental: a tree node's contract is with its parent only,
so inserting one changes one relationship. A state's contract is with every state that can reach it.

<span class="meaning"><b>And the mirror image of it</b>
Those same any-state-to-any-state transitions are precisely what makes an FSM easier to make
reactive. The property is not a defect that BTs fixed; it is the other side of the same coin.</span>

</div>

---
layout: default
---

# The reactivity argument

<div class="panel text-sm">

Consider the battery again: the robot must charge when it runs low, **regardless of what it is doing**.

Castro: *"You can implement something like this with BTs, but a fully reactive behavior (that is, the
battery state causes the robot to go charge no matter where it is) is easier to implement with a FSM…
even if it looks a bit messy."*

In an FSM it is one transition out of every state. In a tree it is a **guard placed above everything
it guards**, which means:

- the guard has to be positioned correctly in the hierarchy, and a subtree added later underneath the
  wrong parent is silently unguarded
- the sequence holding it has to be reactive, which is the memory flag &mdash; invisible in any
  drawing of the tree
- and halting has to work all the way down, which is the four-piece chain from Part 4

</div>

<div class="panel text-sm" style="margin-top:0.4em;padding:0.5em 0.9em">

This module's implementation makes all three available. It does not make any of them automatic.

</div>

---
layout: default
---

# Hierarchical state machines, and the hybrid

<div class="panel text-sm">

Castro pushes back on the usual "spaghetti state machine" dismissal: *"The notion of a hierarchical
finite-state machine (HFSM) has been around for a long time and helps avoid this issue if you follow
good design practices."* States group into superstates &mdash; a "Nominal" mode and a "Charging" mode
&mdash; and the structure returns. Managing transitions is still harder than adding or removing a
subtree.

His actual recommendation is **not** to pick one:

> High-level mode switches are handled by a FSM and mode-specific behaviors are managed with BTs.

<span class="meaning"><b>And the caveat he closes on</b>
It is <i>"not a fair comparison"</i> to dismiss FSMs. Both have legitimate applications; which one
fits depends on the problem's characteristics, not on ideological preference. This deck has spent
fifty slides on behavior trees and that is not a claim that they win.</span>

</div>

---
layout: default
---

# Where AutoRob has met this before

<div class="panel text-sm">

This is not the course's first arbitration architecture. The **finite-state machines and subsumption**
lecture covers the other two points on the same spectrum:

- a **finite-state machine** arbitrates by *where you are* &mdash; one active state, transitions out
  of it
- **subsumption** arbitrates by *priority* &mdash; layered behaviours, with higher layers inhibiting
  and suppressing lower ones, and no central plan at all
- a **behavior tree** arbitrates by *position in a traversal* &mdash; which is priority (the fallback)
  and sequence (the sequence) made into the same mechanism

<span class="meaning"><b>The honest summary</b>
A fallback node is a priority arbiter whose layers are children. Subsumption's inhibition is a
reactive fallback with the guard conditions written as suppressor nodes. These are different
notations for overlapping ideas, and knowing all three is worth more than preferring one.</span>

</div>

---
layout: default
---

# Test cases

<div class="panel text-sm">

Run the reference page, which is configured entirely by URL parameter:

<table class="test-case-table">
<tr><th style="text-align:left">what it shows</th><th style="text-align:left">link</th></tr>
<tr><td>plan, then execute</td><td><a href="/kineval/behavior_trees/reference/?tree=plan_execute">?tree=plan_execute</a></td></tr>
<tr><td>three inspection poses in turn</td><td><a href="/kineval/behavior_trees/reference/?tree=inspection">?tree=inspection</a></td></tr>
<tr><td>a reactive battery guard</td><td><a href="/kineval/behavior_trees/reference/?tree=reactive">?tree=reactive</a></td></tr>
<tr><td>the same tree, with memory</td><td><a href="/kineval/behavior_trees/reference/?tree=reactive&reactive=0">?tree=reactive&amp;reactive=0</a></td></tr>
<tr><td>ticking slowly enough to read</td><td><a href="/kineval/behavior_trees/reference/?tree=plan_execute&rate=3">?rate=3</a></td></tr>
<tr><td>no obstacles, no waypoint ghosts</td><td><a href="/kineval/behavior_trees/reference/?tree=plan_execute&obstacles=0&ghosts=0">?obstacles=0&amp;ghosts=0</a></td></tr>
</table>

Press **Step one tick** to advance the tree one tick at a time; the HUD shows the tick's trace, the
action's feedback and the blackboard's log.

</div>

---
layout: default
---

# What verification actually checks

<div class="panel text-sm">

`reference/tests/smoke.cjs` loads the reference files into one Node `vm` context, in the same order
the page's `<script>` tags do, and asserts **81 checks**:

- that the four snapshots in `reference/` are still byte-identical to their sources in `kineval/`
- every node kind's semantics, including that children after a Failure are never ticked and that a
  parallel has no early exit
- the memory/reactive difference, by halt count: with memory, zero halts and the action stepped on
  both ticks; without, exactly one halt
- the plan's own numbers &mdash; 46 iterations, 18 &rarr; 5 waypoints, 24 obstacles
- that the duration equals the sum of segment durations under the stated parameterization, computed
  independently of the function being tested
- every action outcome: accepted, `INVALID_JOINTS`, `INVALID_GOAL`, cancelled, preempted,
  `PATH_TOLERANCE_VIOLATED`, `GOAL_TOLERANCE_VIOLATED`
- all eight trees, including that re-ticking an arrived tree sends no new goal

<span class="meaning"><b>Why this slide exists</b>
Every number on these slides is read from that output rather than typed from memory. If a check
fails, a slide is wrong.</span>

</div>

---
layout: default
---

# References

<div class="panel text-sm">

<div class="link-list">
<p><b>Sebastian Castro</b>, <a href="https://roboticseabass.com/2021/05/08/introduction-to-behavior-trees/">"Introduction to Behavior Trees"</a>, roboticseabass.com, 8 May 2021. The source for Part 1 entire, and for the FSM comparison in Part 5.</p>
<p><b>Michele Colledanchise &amp; Petter Ögren</b>, <i>Behavior Trees in Robotics and AI: An Introduction</i>, CRC Press, 2018. <a href="https://arxiv.org/abs/1709.00084">arXiv:1709.00084</a>.</p>
<p><b>BehaviorTree.CPP</b> &mdash; <a href="https://www.behaviortree.dev/">behaviortree.dev</a>, with the Groot editor. Used by the ROS 2 navigation stack's BT Navigator.</p>
<p><b>py_trees</b> &mdash; <a href="https://py-trees.readthedocs.io/">py-trees.readthedocs.io</a> (Daniel Stonier).</p>
<p><b>trajectory_msgs</b> and <b>control_msgs</b> &mdash; message and action definitions read from <a href="https://github.com/ros2/common_interfaces">ros2/common_interfaces</a> and <a href="https://github.com/ros-controls/control_msgs">ros-controls/control_msgs</a>.</p>
<p><b>Kuffner &amp; LaValle</b>, "RRT-Connect: An Efficient Approach to Single-Query Path Planning", ICRA 2000 &mdash; the planner whose output this module executes; taught in the <a href="/kineval/motion_planning/">motion planning</a> module.</p>
</div>

</div>

---
layout: center
class: text-center
---

# Appendices

<div class="panel text-sm" style="max-width:40em;margin:0 auto">
The rest of the reference implementation: the node constructors not shown above, the remaining leaves,
the robot and world descriptions, the scene, and the page infrastructure.
</div>

---
layout: default
---

# Appendix: the control-node constructors

<div class="split-panel">
<div>

<<< ../reference/behavior_tree.js#bt-sequence {*}{lines:true,maxHeight:'155px'}

<<< ../reference/behavior_tree.js#bt-parallel {*}{lines:true,maxHeight:'155px'}

</div>
<div>

<<< ../reference/behavior_tree.js#bt-fallback {*}{lines:true,maxHeight:'155px'}

<<< ../reference/behavior_tree.js#bt-decorator {*}{lines:true,maxHeight:'155px'}

</div>
</div>

---
layout: default
---

# Appendix: conditions, and the two leaf ticks

<div class="split-panel">
<div>

<<< ../reference/behavior_tree.js#bt-condition {*}{lines:true,maxHeight:'155px'}

<<< ../reference/behavior_tree.js#bt-tick-condition {*}{lines:true,maxHeight:'155px'}

</div>
<div>

<<< ../reference/behavior_tree.js#bt-tick-action {*}{lines:true,maxHeight:'155px'}

<<< ../reference/behavior_tree.js#bt-tick-parallel {*}{lines:true,maxHeight:'155px'}

</div>
</div>

---
layout: default
---

# Appendix: ticking a whole tree, and walking one

<div class="split-panel">
<div>

<<< ../reference/behavior_tree.js#bt-root-tick {*}{lines:true,maxHeight:'175px'}

<<< ../reference/behavior_tree.js#bt-child-path {*}{lines:true,maxHeight:'130px'}

</div>
<div>

<<< ../reference/behavior_tree.js#bt-walk {*}{lines:true,maxHeight:'320px'}

</div>
</div>

---
layout: default
---

# Appendix: the remaining trajectory functions

<div class="split-panel">
<div>

<<< ../reference/joint_trajectory.js#plan-base-columns {*}{lines:true,maxHeight:'155px'}

<<< ../reference/joint_trajectory.js#trajectory-duration {*}{lines:true,maxHeight:'155px'}

</div>
<div>

<<< ../reference/joint_trajectory.js#base-is-stationary {*}{lines:true,maxHeight:'320px'}

</div>
</div>

---
layout: default
---

# Appendix: the rest of the action server

<div class="split-panel">
<div>

<<< ../reference/joint_trajectory_action.js#action-server-create {*}{lines:true,maxHeight:'320px'}

</div>
<div>

<<< ../reference/joint_trajectory_action.js#action-hold {*}{lines:true,maxHeight:'155px'}

<<< ../reference/joint_trajectory_action.js#action-index-of-largest {*}{lines:true,maxHeight:'155px'}

</div>
</div>

---
layout: default
---

# Appendix: the search leaves

<div class="split-panel">
<div>

<<< ../reference/bt_tasks.js#task-at-location {*}{lines:true,maxHeight:'130px'}

<<< ../reference/bt_tasks.js#task-navigate-to {*}{lines:true,maxHeight:'185px'}

</div>
<div>

<<< ../reference/bt_tasks.js#task-find-object {*}{lines:true,maxHeight:'150px'}

<<< ../reference/bt_tasks.js#task-at-target-location {*}{lines:true,maxHeight:'165px'}

</div>
</div>

---
layout: default
---

# Appendix: the remaining motion leaves

<div class="split-panel">
<div>

<<< ../reference/bt_tasks.js#task-at-configuration {*}{lines:true,maxHeight:'150px'}

<<< ../reference/bt_tasks.js#task-inspect {*}{lines:true,maxHeight:'165px'}

</div>
<div>

<<< ../reference/bt_tasks.js#task-next-inspection-target {*}{lines:true,maxHeight:'150px'}

<<< ../reference/bt_tasks.js#task-battery {*}{lines:true,maxHeight:'165px'}

</div>
</div>

---
layout: default
---

# Appendix: the remaining trees

<div class="split-panel">
<div>

<<< ../reference/trees.js#tree-search-explicit-success {*}{lines:true,maxHeight:'155px'}

<<< ../reference/trees.js#tree-search-parallel {*}{lines:true,maxHeight:'155px'}

</div>
<div>
<div class="panel text-sm" style="padding:0.6em 0.9em">

Five of the eight trees are quoted in full in Parts 1 and 4: the naive search, the multi-location
fallback, the decorator refactor, plan-and-execute, the inspection queue and the reactive battery
guard. These two complete the set.

Every tree in `trees.js` is one function, so a slide can quote the tree it is discussing and nothing
else &mdash; the same reason every function in this module sits inside its own `#region`.

</div>
</div>
</div>

---
layout: default
---

# Appendix: the robot descriptions

<div class="split-panel">
<div>

<<< ../reference/robots/robot_mr2.js#create-robot-mr2 {*}{lines:true,maxHeight:'250px'}

<<< ../reference/robots/robot_mr2.js#mr2-arm-chain {*}{lines:true,maxHeight:'110px'}

</div>
<div>

<<< ../reference/robots/urdf_example.js#create-urdf-example-robot {*}{lines:true,maxHeight:'250px'}

<<< ../reference/robots/urdf_example.js#urdf-example-arm-chain {*}{lines:true,maxHeight:'110px'}

</div>
</div>

---
layout: default
---

# Appendix: the scene

<div class="split-panel">
<div>

<<< ../reference/scene.js#make-link-geometry {*}{lines:true,maxHeight:'150px'}

<<< ../reference/scene.js#create-viewer {*}{lines:true,maxHeight:'165px'}

</div>
<div>

<<< ../reference/scene.js#robot-meshes {*}{lines:true,maxHeight:'165px'}

<<< ../reference/scene.js#obstacle-meshes {*}{lines:true,maxHeight:'150px'}

</div>
</div>

---
layout: default
---

# Appendix: markers, ghosts and resizing

<div class="split-panel">
<div>

<<< ../reference/scene.js#waypoint-ghosts {*}{lines:true,maxHeight:'320px'}

</div>
<div>

<<< ../reference/scene.js#marker-pair {*}{lines:true,maxHeight:'165px'}

<<< ../reference/scene.js#clear-extras {*}{lines:true,maxHeight:'75px'}

<<< ../reference/scene.js#resize-renderer {*}{lines:true,maxHeight:'75px'}

</div>
</div>

---
layout: default
---

# Appendix: the page's own infrastructure

<div class="split-panel">
<div>

<<< ../reference/infrastructure.js#bt-params {*}{lines:true,maxHeight:'155px'}

<<< ../reference/infrastructure.js#build-tree {*}{lines:true,maxHeight:'155px'}

</div>
<div>

<<< ../reference/infrastructure.js#inspection-poses {*}{lines:true,maxHeight:'155px'}

<<< ../reference/infrastructure.js#animation-loop {*}{lines:true,maxHeight:'155px'}

</div>
</div>

---
layout: default
---

# Appendix: the HUD

<div class="split-panel">
<div>

<<< ../reference/infrastructure.js#hud {*}{lines:true,maxHeight:'320px'}

</div>
<div>

<<< ../reference/infrastructure.js#hud-render {*}{lines:true,maxHeight:'155px'}

<<< ../reference/infrastructure.js#tree-ascii {*}{lines:true,maxHeight:'90px'}

<<< ../reference/infrastructure.js#load-plan {*}{lines:true,maxHeight:'75px'}

</div>
</div>

---
layout: default
---

# Appendix: the snapshots

<div class="split-panel">
<div>

<<< ../reference/kinematics.js#robot-init {*}{lines:true,maxHeight:'300px'}

</div>
<div>
<div class="panel text-sm" style="padding:0.6em 0.9em">

Four files in `reference/` are **snapshots**, byte-identical to their `kineval/` sources apart from
one header block &mdash; `smoke.cjs` asserts it:

- `matrix.js`, `quaternion.js` &mdash; the linear algebra
- `kinematics.js` &mdash; forward kinematics, whose `initRobot` is quoted here because the demo page
  calls it before anything else
- `controls.js` &mdash; `applyControls`, which the action's servo writes through

Their contents are taught by the
<a href="/kineval/forward_kinematics/">forward kinematics</a> module and are not re-explained here.
The copies exist because Slidev's `<<<` importer cannot read outside this module's workspace root.

A snapshot can drift from its source; the test is the mitigation, and re-snapshotting is a
`cat`.

</div>
</div>
</div>

---
layout: center
class: text-center
---

# <span class="accent">Tick</span>, and ask again

<div class="panel text-sm" style="max-width:42em;margin:1em auto">

A plan is a list of configurations. A trajectory adds time and names. An action adds a verdict and a
way to change your mind. A tree adds everything that happens before and after the motion, and the
means to abandon it.

<div class="credit" style="margin-top:1em">
ocj-dev.github.io/kineval/behavior_trees &#183; AutoRob (autorob.org)
</div>

</div>
