---
theme: default
title: A-star Path Planning — KinEval Lab
base: /kineval/pathfinding/
info: |
  ## A-star Path Planning
  A KinEval lab-session walkthrough of A-star graph search, built for the AutoRob course (autorob.org).
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

# A-star <span class="accent">Path Planning</span>

### Finding the shortest collision-free route through a 2D grid world

<div class="pt-8">
<div class="search-bar">🔍&nbsp;&nbsp;Directions from <b>Start</b> to <b>Goal</b>&nbsp;&nbsp;&mdash;&nbsp;&nbsp;via A-star</div>
</div>

<div class="title-footer">
<div class="nav-hint">Press → to move forward through the deck</div>
<div class="credit">AutoRob (autorob.org) &#183; ocj-dev.github.io/kineval/pathfinding</div>
</div>

<!--
Title slide. This deck complements the AutoRob lab sections: a fully working
KinEval reference implementation of A-star, walked through step by step.
-->

---
layout: default
---

# A brief history of graph search

<div class="split-panel">
<div class="panel text-sm">

In 1959, **Edsger Dijkstra** published an algorithm for finding shortest paths in a weighted
graph by always expanding the closest not-yet-visited node — optimal, but with no sense of
*direction* toward a particular goal. In 1968, **Peter Hart, Nils Nilsson, and Bertram
Raphael**, working on the [Shakey](https://en.wikipedia.org/wiki/Shakey_the_robot) robot project
at the Stanford Research Institute, generalized this into **A-star**: add an admissible
heuristic estimate of remaining distance to the goal, and the search remains provably optimal
while exploring far fewer nodes.

Shakey needed exactly this — a mobile robot planning routes through a room of obstacles —
making A-star, from its very first publication, a *robotics* algorithm.

</div>
<div>
<div class="side-image">
<img src="/images/shakey.jpg" alt="Shakey the robot on display at the Computer History Museum" />
</div>
<div class="side-caption">Shakey the Robot, SRI International, 1966–1972 &#183; Computer History Museum. Credit: Wikimedia Commons.</div>
</div>
</div>

---
layout: default
---

# Why it still matters

<div class="split-panel">
<div class="panel text-sm">

A-star (and its many descendants — D*, Anytime A*, Theta*) is the routing engine behind mapping
software, video-game pathfinding, and robot navigation stacks alike. This lab's 2D grid-world
version is a direct ancestor of the occupancy-grid path planner used in this course's own Rust
pubsub project (`astar_node.rs`, visualized in `frontend/astar.html`) — the same algorithm, the
same admissible-heuristic argument, just a different systems architecture around it.

The same idea now routes commuters, delivers packages, and steers robots across the University
of Michigan's own backyard: a **May Mobility** autonomous shuttle navigating Detroit streets, an
**Agility Robotics** humanoid delivering a package inside the Ford Robotics Building, an
**MBot** mobile-robotics education platform planning across a classroom floor, and — the
original use case — turn-by-turn directions across town.

</div>
<div class="collage">
<figure>
  <img src="/images/may-mobility.jpg" alt="A May Mobility autonomous shuttle in Detroit" />
  <figcaption>May Mobility autonomous shuttle, Detroit</figcaption>
</figure>
<figure>
  <img src="/images/agility-digit.jpg" alt="An Agility Robotics Digit humanoid carrying a package" />
  <figcaption>Agility Robotics Digit, Ford Robotics Building, UM</figcaption>
</figure>
<figure>
  <img src="/images/mbot-omni.jpg" alt="An MBot omni-wheel education robot" />
  <figcaption>MBot Omni education robot</figcaption>
</figure>
<figure>
  <img src="/images/route-map.jpg" alt="A driving route from the UM Robotics Building to Detroit Street Filling Station" />
  <figcaption>Robotics Building &rarr; Detroit St. Filling Station (OpenStreetMap route, standing in for an Apple Maps snapshot)</figcaption>
</figure>
</div>
</div>

---
layout: default
---

# The algorithmic process

<div class="grid grid-cols-2 gap-6 mt-2">
<div class="panel text-xs">

**Setup**

- A uniform grid of candidate locations is built over the 2D world, spaced `eps` apart.
- Each grid cell tracks: its distance-from-start so far (`distance`), a pointer back toward the
  start (`parent`), whether it has been `visited`, and its search `priority`.
- The **open queue** is a min-heap ordered by `priority`, so the cheapest-looking node to expand
  next is always at the top.

**The admissible heuristic**

A-star's priority is `f = g + h`: `g` is the actual distance traveled so far, and `h` is the
straight-line (Euclidean) distance remaining to the goal. Because straight-line distance can
never *overestimate* the true remaining grid distance, this heuristic is **admissible** — and an
admissible heuristic is exactly what guarantees A-star still finds the *shortest* path, not just
*a* path.

</div>
<div class="panel">

```text
 1  initialize the open queue with the start node (distance 0)
 2  while the open queue is not empty
 3      pop the node with minimum priority from the open queue
 4      if that node was already visited, discard it and continue
 5      mark the node visited
 6      if within one grid cell of the goal, reconstruct path and succeed
 7      for each of its 4 grid neighbors
 8          if the neighbor is off-grid or in collision, skip it
 9          tentative_distance = current.distance + eps
10          if tentative_distance < neighbor.distance
11              record this cheaper path: neighbor.distance, neighbor.parent
12              neighbor.priority = f(neighbor)   // g + h for A-star
13              insert the neighbor into the open queue
14  the open queue emptied out -- no path exists; fail
```

</div>
</div>

<div class="mt-2 text-xs opacity-60">
Next: a hand-worked example on a tiny grid, before diving into the real code.
</div>

---
layout: default
---

# Setup: a uniform grid, by hand

<div class="grid grid-cols-2 gap-4 mt-2">
<div class="panel text-sm">

A 3-column by 4-row grid, small enough to work through by hand:

- **Start (A)** is the lowermost-left cell.
- **Goal (B)** is the uppermost-right cell.
- The second-from-bottom row is entirely blocked, **except its leftmost cell** — the only way
  from the bottom half of the grid to the top half.

Every cell is a candidate node, exactly as `initSearchGraph()` builds one for the real
`[-2,7)×[-2,7)` kineval-stencil world — just three columns and four rows instead of forty-five
by forty-five.

</div>
<div class="panel" style="min-height: 300px; display:flex;">
<StaticMiniGrid />
</div>
</div>

---
layout: default
---

# What every grid cell tracks

<div class="panel text-xs mt-2">
Four fields per cell are all the search needs: <code>distance</code> (cost of the cheapest path
found so far), <code>parent</code> (the neighbor that path arrives from — this is what lets the
search reconstruct a route at the end), <code>visited</code> (finalized, never revisited), and
<code>priority</code> (what the open-queue heap sorts on).
</div>

<<< ../reference/graph_search.js#grid-node-fields {*}{lines:true,startLine:61}

---
layout: default
---

# The open queue: a first look

<div class="content-body">
<AStarPanel initial-scene="simple_3x4" :show-alg-picker="false" :show-scene-picker="false" :speed-ms="250" />
</div>

<!--
Step forward a few times: the start node is queued (line 1), the loop
checks the queue (line 2), pops the only entry (line 3), and marks it
visited (line 5) -- the open-queue block on the left shows the heap array
shrink from one entry to zero, right before the goal check and neighbor
expansion begin.
-->

---
layout: default
---

# The admissible heuristic

<div class="panel text-sm mt-4">

For a 4-connected grid, the true remaining cost from any cell to the goal is at least its
**Euclidean (straight-line) distance** — grid moves can only ever be as short as a straight
line, never shorter. That's exactly what makes `h = Euclidean distance to goal` **admissible**:
it never overestimates.

This is the whole argument for why A-star is optimal: as long as `h` never overestimates, the
first time A-star pops the goal off the open queue, `g` at that moment is guaranteed to be the
true shortest distance — no cheaper path could still be waiting in the queue, because anything
cheaper would have had a smaller `f = g + h` and been popped first.

Drop the `g` term entirely (priority `= h` alone) and you get **greedy-best-first** — fast, but
no longer guaranteed optimal, since it can be lured toward a dead end that merely *looks* close
to the goal. The Cul-de-Sac test case later in this deck shows exactly that happening.

</div>

---
layout: default
---

# Step through the simple grid

<div class="content-body">
<AStarPanel initial-scene="simple_3x4" :show-scene-picker="false" :speed-ms="180" />
</div>

<!--
The full search to completion on the 12-cell example: watch the open-queue
heap blocks, the distance labels inside each visited/queued cell, and the
bold parent-edge tree grow up through the gap in the obstacle row.
-->

---
layout: default
---

# Component breakdown

<div class="panel text-sm mt-4">

The next several slides walk the reference implementation
(`kineval/pathfinding/reference/graph_search.js`) one pseudocode line at a time, in dependency
order — the two heap primitives first (everything else calls them), then setup, then the
heuristic, then the two loop bodies, then how the result gets turned into a path:

1. **Line 3** — pop the minimum-priority node (`minheap_extract`)
2. **Line 13** — insert a node into the open queue (`minheap_insert`)
3. **Line 1** — initialize the open queue with the start node
4. **Line 12** — the priority formula, `f = g + h`
5. **Lines 2–6** — the main loop: pop, discard stale entries, visit, goal test
6. **Lines 7–13** — expanding neighbors: collision checks and edge relaxation
7. **Line 6**, revisited — reconstructing the path once the goal is reached

Every snippet is imported directly from the shipped reference file — not retyped — with line
numbers matching that file exactly.

</div>

---
layout: default
---

# Line 3: pop the minimum-priority node

<div class="panel text-xs mt-2">
<code>minheap_extract</code> swaps the root with the last element, pops the old root off the
end, then "sifts down" the new root by repeatedly swapping with its smaller child until the
heap invariant is restored — O(log n).
</div>

<<< ../reference/graph_search.js#pop-min {*}{lines:true,startLine:241}

---
layout: default
---

# Line 13: insert into the open queue

<div class="panel text-xs mt-2">
<code>minheap_insert</code> appends the new element, then "sifts up" by repeatedly swapping with
its parent while it is smaller — also O(log n). A node may be inserted more than once if a
cheaper path to it is found later; the main loop's stale-entry check (line 4) discards the
leftover duplicate lazily when it is eventually popped.
</div>

<<< ../reference/graph_search.js#insert-queue {*}{lines:true,startLine:222}

---
layout: default
---

# Line 1: initialize the open queue

<div class="panel text-xs mt-2">
The start location generally won't land exactly on a grid point, so <code>initSearchGraph()</code>
tracks the single closest grid node while building the grid, then queues that node with zero
distance — the seed the entire search grows from.
</div>

<<< ../reference/graph_search.js#init-start {*}{lines:true,startLine:83}

---
layout: default
---

# Line 12: the priority formula

<div class="panel text-xs mt-2">
<code>f = g + h</code> for A-star. Swapping in a different formula turns this very same search
loop into greedy-best-first, breadth-first, or a depth-first emulation — the reference
implementation supports all four via the <code>search_alg</code> switch.
</div>

<<< ../reference/graph_search.js#priority-formula {*}{lines:true,startLine:191}

---
layout: default
---

# Lines 2–6: the main loop

<div class="panel text-xs mt-2">
Each call to <code>iterateGraphSearch()</code> performs <i>one</i> queue-pop-and-visit step, then
returns control to the browser's animation loop — an explicit <code>while</code> loop here would
block the page. This per-call structure is exactly what makes Play/Pause/Step possible in this
deck's visualizations: one call is one step.
</div>

<<< ../reference/graph_search.js#main-loop {*}{lines:true,startLine:111}

---
layout: default
---

# Lines 7–13: expanding neighbors

<div class="panel text-xs mt-2">
For each of the 4-connected neighbors: skip anything off-grid or in collision, compute the cost
of stepping there, and <i>relax</i> the edge — update the neighbor's distance/parent/priority and
(re)queue it — only if this path to it is cheaper than any found before.
</div>

<<< ../reference/graph_search.js#neighbor-loop {*}{lines:true,startLine:143}

---
layout: default
---

# Line 6, revisited: reconstructing the path

<div class="panel text-xs mt-2">
Once the goal is reached, the path is just the chain of <code>parent</code> pointers back to the
start — walked once here to draw it, exactly the same chain this deck's visualizations walk to
render their blue route line and bold parent-edge tree.
</div>

<<< ../reference/draw.js#path-reconstruct {*}{lines:true,startLine:64}

---
layout: default
---

# Watch it search

<div class="content-body">
<AStarPanel initial-scene="misc" initial-alg="A-star" :speed-ms="4" />
</div>

<!--
Interactive: Start/Pause/Step Forward/Step Back/Reset, plus scene and
algorithm pickers. Every pseudocode line gets its own step, so stepping
forward walks the pseudocode one line at a time; Play runs the full trace.
The legend above the map explains the cell colors, the numbers inside
visited/queued cells are their current distance, the bold blue lines are
the search tree of parent edges, and the dashed red line from a
just-queued cell to the goal shows its f-score.
-->

---
layout: default
---

# Test cases

<div class="panel text-xs mt-2">

| Scene | New? | What it tests |
|---|---|---|
| Empty | | Baseline: unobstructed Manhattan-distance path length |
| Misc | | Scattered obstacles with a narrow opening (stencil default) |
| Narrow 1 | | A single narrow corridor between two large blocks |
| Narrow 2 | | A staggered pair of corridors, forcing a longer route |
| Three Sections | | Three compartments connected by alternating passages |
| Downtown Gridlock | ✅ | A 3x3 grid of blocks — many turns, many equally-short routes |
| Diagonal Staircase | ✅ | Alternating full-width bands force a genuinely longer detour |
| Spiral | ✅ | A two-ring spiral corridor from the outside in to a center goal |
| Cul-de-Sac | ✅ | A dead end lures greedy-best-first into a suboptimal route |
| Construction Detour | ✅ | A single "road closure" wall, gapped past the goal |

</div>

---
layout: default
---

# Try every test case

<div class="content-body">
<AStarPanel initial-scene="downtown_gridlock" :show-alg-picker="true" :speed-ms="3" />
</div>

<!--
Same interactive panel, defaulted to one of the five new scenes. Use the
Scene picker to step through all ten, and the Algorithm picker to compare
A-star against greedy-best-first, breadth-first, and depth-first on the
same world -- Cul-de-Sac is the clearest side-by-side case: A-star still
finds the optimal route while greedy-best-first gets lured into the dead
end and settles for a longer one.
-->

---
layout: default
---

# Try this yourself

<div class="panel text-sm mt-2">

The reference implementation runs standalone, no build step, straight from `search_canvas.html`.
Every run is configured entirely through URL parameters:

| Parameter | Values | Meaning |
|---|---|---|
| `search_alg` | `A-star`, `greedy-best-first`, `breadth-first`, `depth-first` | which priority formula drives the search |
| `planning_scene` | a built-in name, or a path like `scenes/spiral.js` | which obstacle layout to load |
| `q_init` | `[x,y]` | start location |
| `q_goal` | `[x,y]` | goal location |
| `eps` | a number | grid spacing (smaller = finer, slower) |
| `color_scheme` | `default`, `light`, `blue` | canvas color palette |

```text
search_canvas.html?search_alg=A-star?planning_scene=scenes/spiral.js?q_init=[0,0]?q_goal=[4,4]?eps=0.2
```

**Run it live:** [ocj-dev.github.io/kineval/kineval/pathplanning](https://ocj-dev.github.io/kineval/kineval/pathplanning/)

</div>

---
layout: default
class: text-center
---

<div class="content-body image-slide">
<a href="https://robots.engin.umich.edu/Projects/NGV" target="_blank" class="side-image" style="display:inline-block;max-height:76%;">
<img src="/images/ford-fusion-ngv.jpg" class="hero-image" alt="A Ford Fusion Next Generation Vehicle autonomous research car" />
</a>
<div class="image-caption">Go Blue!</div>
<div class="image-credit">Ford Fusion Next Generation Vehicle &#183; <a href="https://robots.engin.umich.edu/Projects/NGV">robots.engin.umich.edu/Projects/NGV</a></div>
</div>
