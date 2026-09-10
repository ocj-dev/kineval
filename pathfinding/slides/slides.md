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

<div class="grid grid-cols-2 gap-4 mt-4 text-sm">
<div class="panel">

In 1959, **Edsger Dijkstra** published an algorithm for finding shortest paths in a weighted
graph by always expanding the closest not-yet-visited node — optimal, but with no sense of
*direction* toward a particular goal. In 1968, **Peter Hart, Nils Nilsson, and Bertram
Raphael**, working on the Shakey robot project at the Stanford Research Institute, generalized
this into **A-star**: add an admissible heuristic estimate of remaining distance to the goal, and
the search remains provably optimal while exploring far fewer nodes. Shakey needed exactly this —
a mobile robot planning routes through a room of obstacles — making A-star, from its very first
publication, a *robotics* algorithm.

</div>
<div class="panel">

**Why it still matters**

A-star (and its many descendants — D*, Anytime A*, Theta*) is the routing engine behind mapping
software, video-game pathfinding, and robot navigation stacks alike. This lab's 2D grid-world
version is a direct ancestor of the occupancy-grid path planner used in this course's own Rust
pubsub project (`astar_node.rs`, visualized in `frontend/astar.html`) — the same algorithm, the
same admissible-heuristic argument, just a different systems architecture around it.

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

---
layout: default
---

# Component breakdown

<div class="panel text-sm mt-4">

Following the pseudocode's own dependency order, the reference implementation
(`kineval/pathfinding/reference/graph_search.js`) breaks down into:

1. **The priority queue** — a binary min-heap over each node's `priority` field (pseudocode lines 3, 13)
2. **Building the search graph** — laying out the grid and locating the start node (line 1)
3. **The admissible heuristic and priority formula** — `f = g + h` (line 12)
4. **The main search loop** — pop, discard stale entries, visit, goal test (lines 2&ndash;6)
5. **Expanding neighbors** — collision checks and edge relaxation (lines 7&ndash;13)
6. **Reconstructing the path** — walking `parent` pointers back to the start (line 6)

Every snippet on the following slides is imported directly from the shipped reference file — not
retyped — so what you read here is exactly what runs.

</div>

---
layout: default
---

# Component: the priority queue

<div class="grid grid-cols-1 gap-2 mt-2 text-xs">
<div class="panel">
A standard array-backed binary min-heap, keyed on each node's <code>priority</code>. Insert
appends and "sifts up"; extract swaps the root with the last element and "sifts down" &mdash;
both O(log n). A node may be pushed more than once if a cheaper path to it is found later; the
main loop discards stale duplicates lazily when they're popped (next slide).
</div>
</div>

<<< ../reference/graph_search.js#L214-L256 {maxHeight:'330px'}

---
layout: default
---

# Component: building the search graph

<div class="panel text-xs mt-2">
Lays a uniform grid of candidate locations over the 2D world, spaced <code>eps</code> apart, and
locates the grid node closest to <code>q_init</code> as the search's start &mdash; since the true
start location generally won't land exactly on a grid point.
</div>

<<< ../reference/graph_search.js#L37-L87 {maxHeight:'330px'}

---
layout: default
---

# Component: the heuristic and priority formula

<div class="panel text-xs mt-2">
<code>f = g + h</code> for A-star: <code>g</code> is <code>node.distance</code> (actual cost so
far), <code>h</code> is the Euclidean distance remaining to the goal. Swapping in a different
formula turns the very same search loop into greedy-best-first, breadth-first, or a depth-first
emulation &mdash; try the <b>Algorithm</b> picker on the next-but-one slide.
</div>

<<< ../reference/graph_search.js#L182-L202 {maxHeight:'330px'}

---
layout: default
---

# Component: the main search loop

<div class="panel text-xs mt-2">
Each call to <code>iterateGraphSearch()</code> performs <i>one</i> queue-pop-and-visit step, then
returns control to the browser's animation loop &mdash; an explicit <code>while</code> loop here
would block the page and make it unresponsive. This per-call structure is exactly what makes
Play/Pause/Step possible in the visualization on the next slide: one call is one step.
</div>

<<< ../reference/graph_search.js#L89-L134 {maxHeight:'310px'}

---
layout: default
---

# Component: expanding neighbors

<div class="panel text-xs mt-2">
For each of the 4-connected neighbors: skip anything off-grid or in collision, compute the cost
of stepping there, and <i>relax</i> the edge &mdash; update the neighbor's distance/parent/priority
and (re)queue it &mdash; only if this path to it is cheaper than any found before.
</div>

<<< ../reference/graph_search.js#L136-L177 {maxHeight:'330px'}

---
layout: default
---

# Component: reconstructing the path

<div class="panel text-xs mt-2">
Once the goal is reached, the path is just the chain of <code>parent</code> pointers back to the
start &mdash; walked once here to draw it, exactly the same chain the visualization walks to
render its blue route line.
</div>

<<< ../reference/draw.js#L63-L112 {maxHeight:'330px'}

---
layout: default
---

# Watch it search

<div class="content-body">
<AStarPanel initial-scene="misc" initial-alg="A-star" :speed-ms="10" />
</div>

<!--
Interactive: Start/Pause/Step Forward/Step Back/Reset, plus scene and
algorithm pickers. The pseudocode line highlighted on the left always
matches the step being drawn on the map.
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
<AStarPanel initial-scene="downtown_gridlock" :show-alg-picker="true" :speed-ms="8" />
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
class: text-center
---

<div class="content-body image-slide">
<CampusMapCloser />
<div class="image-caption">Go Blue!</div>
<div class="image-credit">A course robot, parked at the Diag &#183; illustration coded for this deck (no image-generation tool was available in this environment)</div>
</div>
