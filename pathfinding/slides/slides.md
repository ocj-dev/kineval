---
theme: default
title: A-star Path Planning — KinEval Lab
base: /kineval/pathfinding/
colorSchema: light
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
<div class="credit">AutoRob (autorob.org) &#183; Chad Jenkins (ocj@umich.edu) &#183; ocj-dev.github.io/kineval/pathfinding</div>
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
<img src="/images/shakey.jpg" alt="Shakey the robot, with callouts labeling its parts" />
</div>
<div class="side-caption">Shakey the Robot, SRI International, 1966–1972 &#183; Credit: Wikimedia Commons.</div>
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
  <img src="/images/route-map.jpg" alt="A driving route from the UM Robotics Building to Detroit Street Filling Station" />
  <figcaption>Robotics Building &rarr; Detroit St. Filling Station (OpenStreetMap route, standing in for an Apple Maps snapshot)</figcaption>
</figure>
<figure>
  <img src="/images/mbot-omni.jpg" alt="An MBot omni-wheel education robot" />
  <figcaption>MBot Omni education robot</figcaption>
</figure>
<figure>
  <img src="/images/agility-digit.jpg" alt="An Agility Robotics Digit humanoid carrying a package" />
  <figcaption>Agility Robotics Digit, Ford Robotics Building, UM</figcaption>
</figure>
</div>
</div>

---
layout: default
---

# KinEval Path Planning Stencil <a class="accent" href="https://ocj-dev.github.io/kineval/kineval/pathplanning/search_canvas.html?search_alg=A-star?planning_scene=narrow2?q_init=[0,0]?q_goal=[4,4]?eps=0.1" target="_blank">(link)</a>

<div class="split-panel">
<div class="side-image">
<img src="/images/kineval-narrow2.jpg" alt="KinEval search canvas interface showing a completed A-star path through the narrow2 scene" />
</div>
<div class="panel text-xs">

The reference implementation runs standalone, no build step, straight from `search_canvas.html`.
Every run is configured entirely through URL parameters:

| Parameter | Values |
|---|---|
| `search_alg` | `A-star`, `greedy-best-first`, `breadth-first`, `depth-first`<br><span class="meaning">which priority formula drives the search</span> |
| `planning_scene` | `empty`, `misc`, `narrow1`, `narrow2`, `three_sections` (the original kineval-stencil scenes), or a custom path like `scenes/spiral.js`<br><span class="meaning">which obstacle layout to load</span> |
| `q_init` | `[x,y]`<br><span class="meaning">start location</span> |
| `q_goal` | `[x,y]`<br><span class="meaning">goal location</span> |
| `eps` | a number<br><span class="meaning">grid spacing (smaller = finer, slower)</span> |
| `color_scheme` | `default`, `light`, `blue`<br><span class="meaning">canvas color palette</span> |

```text
search_canvas.html
  ?search_alg=A-star
  ?planning_scene=narrow2
  ?q_init=[0,0]?q_goal=[4,4]?eps=0.1
```

**Run it live:** [ocj-dev.github.io/kineval/kineval/pathplanning](https://ocj-dev.github.io/kineval/kineval/pathplanning/search_canvas.html?search_alg=A-star?planning_scene=narrow2?q_init=[0,0]?q_goal=[4,4]?eps=0.1)

**Previous version (Winter 2023):** [autorob.org/archive/assignment-1-path-planning](https://autorob.org/archive/assignment-1-path-planning/)

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
 0  find the start node for the search from the q_init user parameter
 1  initialize the open queue with the start node (distance 0)
 2  while the open queue is not empty
 3      pop the node with minimum priority from the open queue
 4      if that node was already visited, discard it and continue
 5      mark the node visited
 6      if within one grid cell of the goal, reconstruct path, stop, and succeed
 7      for each of its 4 grid neighbors
 8          if the neighbor is off-grid or in collision, skip it
 9          tentative_distance = current.distance + eps
10          if tentative_distance < neighbor.distance
11              record this cheaper path: neighbor.distance, neighbor.parent
12              neighbor.priority = f(neighbor)   // g + h for A-star
13              insert the neighbor into the open queue
14  the open queue emptied out -- stop; no path exists; fail
```

</div>
</div>

<div class="mt-2 text-xs opacity-60">
Next: which components implement which lines, then a hand-worked example on a tiny grid.
</div>

---
layout: default
---

# Component breakdown

<div class="panel text-sm mt-4">

The upcoming slides walk the reference implementation
(`kineval/pathfinding/reference/graph_search.js`) one pseudocode line at a time. **Line 13**
(inserting into the open queue) comes first, right after the hand-worked grid setup below, since
it's needed to make sense of the very first step in that example; the remaining components follow
afterward, in dependency order — the other heap primitive, then the rest of setup, the heuristic,
the two loop bodies, and finally how the result gets turned into a path:

1. **Line 13** — insert a node into the open queue (`minheap_insert`)
2. **Line 3** — pop the minimum-priority node (`minheap_extract`)
3. **Line 1** — initialize the open queue with the start node
4. **Line 12** — the priority formula, `f = g + h`
5. **Lines 2–6** — the main loop: pop, discard stale entries, visit, goal test
6. **Lines 7–13** — expanding neighbors: collision checks and edge relaxation
7. **Line 6**, revisited — reconstructing the path once the goal is reached

Every snippet is imported directly from the shipped reference file — not retyped — with line
numbers matching that file exactly. A short appendix after the code walkthrough covers the
handful of supporting functions none of these slides touch directly.

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

<<< ../reference/graph_search.js#grid-node-fields {*}{lines:true,startLine:64,maxHeight:'380px'}

---
layout: default
---

# Which cell is the start cell?

<div class="panel text-xs mt-2">
<code>q_init</code> generally won't land exactly on a grid point, so <code>initSearchGraph()</code>
tracks the single closest grid node to it (line 0) while it builds the grid — the same
double loop that lays out every cell's fields above also does this bookkeeping along the way.
</div>

<<< ../reference/graph_search.js#find-start-cell {*}{lines:true,startLine:53,maxHeight:'420px'}

---
layout: default
---

# Line 13: insert into the open queue

<div class="panel text-xs mt-2">
<code>minheap_insert</code> appends the new element, then "sifts up" by repeatedly swapping with
its parent while it is smaller — O(log n). A node may be inserted more than once if a
cheaper path to it is found later; the main loop's stale-entry check (line 4) discards the
leftover duplicate lazily when it is eventually popped.
</div>

<<< ../reference/graph_search.js#insert-queue {*}{lines:true,startLine:235,maxHeight:'420px'}

---
layout: default
---

# The open queue: a first look

<div class="content-body">
<AStarPanel initial-scene="simple_3x4" :show-alg-picker="false" :show-scene-picker="false" :speed-ms="250" />
</div>

<!--
Step forward a few times: the start node is found (line 0) and queued
(line 1), the loop checks the queue (line 2), pops the only entry (line 3),
and marks it visited (line 5) -- the open-queue block on the left shows the
heap array shrink from one entry to zero, right before the goal check and
neighbor expansion begin.
-->

---
layout: default
---

# The admissible heuristic

<div class="split-panel">
<div class="panel text-sm">

For a 4-connected grid, the true remaining cost from any cell to the goal is at least its
**Euclidean (straight-line) distance** — grid moves can only ever be as short as a straight
line, never shorter. That's what makes Euclidean distance **admissible**: it never overestimates,
which is the whole argument for why A-star is optimal — the first time A-star pops the goal off
the open queue, `g` at that moment is guaranteed to be the true shortest distance, since anything
cheaper would have had a smaller `f = g + h` and been popped first.

**Manhattan (taxicab) distance**, `|dx| + |dy|`, is *also* admissible on a 4-connected grid —
diagonal movement isn't allowed, so the true remaining cost can never be less than it either. It's
a **tighter** bound than Euclidean (Euclidean ≤ Manhattan ≤ true distance always), making it a
more informed estimate that typically visits fewer nodes while remaining just as optimal. Try the
**Manhattan** checkbox on the visualizations ahead to see the difference.

Drop the `g` term entirely (priority `= h` alone) and you get **greedy-best-first** — fast, but
no longer guaranteed optimal, since it can be lured toward a dead end that merely *looks* close
to the goal. The Cul-de-Sac test case later in this deck shows exactly that happening.

</div>
<div>
<div class="side-image">
<img src="/images/admissible-heuristic-diagram.png" alt="Diagram showing g(N), h(N), and h*(N) for a node N on a path from A to B around an obstacle, illustrating h(N) <= h*(N)" />
</div>
<div class="side-caption">Admissibility: h(N) never overestimates the true remaining cost h*(N). Widely used course-material diagram; original author unconfirmed.</div>
</div>
</div>

---
layout: default
---

# Same cell, two heuristics

<div class="split-panel">
<div class="panel" style="display:flex; flex-direction:column;">
<HeuristicExampleGrid mode="euclidean" />
<div class="text-xs mt-2">
Euclidean: <code>h = &radic;(2&sup2; + 1&sup2;) = &radic;5 &asymp; 2.24</code>, so
<code>f = g + h = 2 + 2.24 = <b>4.24</b></code>
</div>
</div>
<div class="panel" style="display:flex; flex-direction:column;">
<HeuristicExampleGrid mode="manhattan" />
<div class="text-xs mt-2">
Manhattan: <code>h = |2| + |1| = <b>3</b></code>, so
<code>f = g + h = 2 + 3 = <b>5.00</b></code>
</div>
</div>
</div>

<div class="mt-2 text-xs opacity-60">
A real step from the simple grid above: the cell in the leftmost column, second-from-top row,
being relaxed by the cell directly below it. Same <code>g</code> either way (the actual distance
traveled); only the heuristic estimate of the remaining distance to the goal changes.
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
bold parent-edge tree grow up through the gap in the obstacle row. Toggle
Manhattan to see the f-score line bend into an L-shape and compare the
resulting node count.
-->

---
layout: default
---

# Line 3: pop the minimum-priority node

<div class="panel text-xs mt-2">
<code>minheap_extract</code> swaps the root with the last element, pops the old root off the
end, then "sifts down" the new root by repeatedly swapping with its smaller child until the
heap invariant is restored — O(log n).
</div>

<<< ../reference/graph_search.js#pop-min {*}{lines:true,startLine:254,maxHeight:'420px'}

---
layout: default
---

# Line 1: initialize the open queue

<div class="panel text-xs mt-2">
Once the closest grid node to <code>q_init</code> has been found (line 0), <code>initSearchGraph()</code>
queues that node with zero distance — the seed the entire search grows from.
</div>

<<< ../reference/graph_search.js#init-start {*}{lines:true,startLine:87,maxHeight:'420px'}

---
layout: default
---

# Line 12: the priority formula

<div class="panel text-xs mt-2">
<code>f = g + h</code> for A-star. Swapping in a different formula turns this very same search
loop into greedy-best-first, breadth-first, or a depth-first emulation — the reference
implementation supports all four via the <code>search_alg</code> switch.
</div>

<<< ../reference/graph_search.js#priority-formula {*}{lines:true,startLine:204,maxHeight:'420px'}

---
layout: default
---

# Lines 2–6: the main loop

<div class="panel text-xs mt-2">
Each call to <code>iterateGraphSearch()</code> performs <i>one</i> queue-pop-and-visit step, then
returns control to the browser's animation loop — an explicit <code>while</code> loop here would
block the page. This per-call structure is exactly what makes Play/Pause/Step possible in this
deck's visualizations: one call is one step. Both terminal cases set
<code>search_iterate = false</code> before returning, so the animation loop stops calling this
function the moment the search succeeds or fails, instead of keeping the search running well
past the point it found (or ruled out) a path.
</div>

<<< ../reference/graph_search.js#main-loop {*}{lines:true,startLine:115,maxHeight:'380px'}

---
layout: default
---

# Lines 7–13: expanding neighbors

<div class="panel text-xs mt-2">
For each of the 4-connected neighbors: skip anything off-grid or in collision, compute the cost
of stepping there, and <i>relax</i> the edge — update the neighbor's distance/parent/priority and
(re)queue it — only if this path to it is cheaper than any found before.
</div>

<<< ../reference/graph_search.js#neighbor-loop {*}{lines:true,startLine:156,maxHeight:'420px'}

---
layout: default
---

# Line 6, revisited: reconstructing the path

<div class="panel text-xs mt-2">
Once the goal is reached, the path is just the chain of <code>parent</code> pointers back to the
start — walked here to draw it, plus one extra segment out to the exact goal coordinate (the
nearest visited node is only guaranteed to be within <code>eps</code> of the goal, not on top of
it) so the drawn path always fully connects start to goal, exactly like this deck's own
visualizations do.
</div>

<<< ../reference/draw.js#path-reconstruct {*}{lines:true,startLine:64,maxHeight:'420px'}

---
layout: default
---

# Appendix: the rest of the reference implementation

<div class="panel text-sm mt-4">

The slides above cover every line of the pseudocode, but the reference implementation
(`graph_search.js`, `draw.js`, `infrastructure.js`, `search_canvas.html`) includes a handful of
supporting pieces none of them touch directly. For completeness, the next few slides cover:

1. **Collision testing** (`infrastructure.js`) — the provided helper every scene's obstacle
   layout is checked against
2. **The animation loop** (`draw.js`) — how `iterateGraphSearch()` actually gets called once per
   frame, and how it's told to stop
3. **Reading the URL parameters** (`search_canvas.html`) — how `search_alg`, `planning_scene`,
   `q_init`, `q_goal`, and `eps` get parsed into the globals every function above reads

</div>

---
layout: default
---

# Appendix: collision testing

<div class="panel text-xs mt-2">
Every scene's obstacles are axis-aligned boxes, given as a <code>[x-range, y-range]</code> pair.
A configuration is in collision if it falls inside <i>every</i> dimension's range for
<i>any</i> obstacle — this is the function <code>testCollision()</code> that lines 8 and neighbor
relaxation call on every candidate neighbor.
</div>

<<< ../reference/infrastructure.js#appendix-collision {*}{lines:true,startLine:40,maxHeight:'420px'}

---
layout: default
---

# Appendix: the animation loop

<div class="panel text-xs mt-2">
Called once per rendered frame. <code>search_iterate</code> gates whether it does anything at
all — this is the flag <code>iterateGraphSearch()</code> sets to <code>false</code> to stop the
search once it succeeds or fails (see "Lines 2–6" earlier). While it's still true, this is the
dispatch that calls <code>iterateGraphSearch()</code> once per frame for every graph-search
algorithm this deck covers.
</div>

<<< ../reference/draw.js#appendix-animate {*}{lines:true,startLine:254,maxHeight:'420px'}

---
layout: default
---

# Appendix: reading the URL parameters

<div class="panel text-xs mt-2">
<code>search_canvas.html</code>'s inline script sets defaults, then overwrites them from the
page's own URL — this is what makes every example in this deck (and the <code>(link)</code> on
the "KinEval Path Planning Stencil" slide) just a plain hyperlink, no server or build step
involved.
</div>

<<< ../reference/search_canvas.html#appendix-url-params {*}{lines:true,startLine:65,maxHeight:'420px'}

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
just-queued cell to the goal shows its f-score (toggle Manhattan to bend
it into an L-shape and compare g/h/f for the currently examined neighbor).
-->

---
layout: default
---

# Test cases

<div class="panel text-xs mt-2">

| Scene | New? | What it tests | Run it |
|---|---|---|---|
| Empty | | Baseline: unobstructed Manhattan-distance path length | [▶](https://ocj-dev.github.io/kineval/kineval/pathplanning/search_canvas.html?search_alg=A-star?planning_scene=empty?q_init=[0,0]?q_goal=[4,4]?eps=0.1) |
| Misc | | Scattered obstacles with a narrow opening (stencil default) | [▶](https://ocj-dev.github.io/kineval/kineval/pathplanning/search_canvas.html?search_alg=A-star?planning_scene=misc?q_init=[0,0]?q_goal=[4,4]?eps=0.1) |
| Narrow 1 | | A single narrow corridor between two large blocks | [▶](https://ocj-dev.github.io/kineval/kineval/pathplanning/search_canvas.html?search_alg=A-star?planning_scene=narrow1?q_init=[0,0]?q_goal=[4,4]?eps=0.1) |
| Narrow 2 | | A staggered pair of corridors, forcing a longer route | [▶](https://ocj-dev.github.io/kineval/kineval/pathplanning/search_canvas.html?search_alg=A-star?planning_scene=narrow2?q_init=[0,0]?q_goal=[4,4]?eps=0.1) |
| Three Sections | | Three compartments connected by alternating passages | [▶](https://ocj-dev.github.io/kineval/kineval/pathplanning/search_canvas.html?search_alg=A-star?planning_scene=three_sections?q_init=[0,0]?q_goal=[4,4]?eps=0.1) |
| Downtown Gridlock | ✅ | A 3x3 grid of blocks — many turns, many equally-short routes | [▶](https://ocj-dev.github.io/kineval/kineval/pathplanning/search_canvas.html?search_alg=A-star?planning_scene=scenes/downtown_gridlock.js?q_init=[0,0]?q_goal=[4,4]?eps=0.1) |
| Diagonal Staircase | ✅ | Alternating full-width bands force a genuinely longer detour | [▶](https://ocj-dev.github.io/kineval/kineval/pathplanning/search_canvas.html?search_alg=A-star?planning_scene=scenes/diagonal_staircase.js?q_init=[0,0]?q_goal=[4,4]?eps=0.1) |
| Spiral | ✅ | A two-ring spiral corridor from the outside in to a center goal | [▶](https://ocj-dev.github.io/kineval/kineval/pathplanning/search_canvas.html?search_alg=A-star?planning_scene=scenes/spiral.js?q_init=[-0.65,2]?q_goal=[2,2]?eps=0.1) |
| Cul-de-Sac | ✅ | A dead end lures greedy-best-first into a suboptimal route | [▶](https://ocj-dev.github.io/kineval/kineval/pathplanning/search_canvas.html?search_alg=A-star?planning_scene=scenes/cul_de_sac.js?q_init=[0,0]?q_goal=[4,4]?eps=0.1) |
| Construction Detour | ✅ | A single "road closure" wall, gapped past the goal | [▶](https://ocj-dev.github.io/kineval/kineval/pathplanning/search_canvas.html?search_alg=A-star?planning_scene=scenes/construction_detour.js?q_init=[0,0]?q_goal=[4,4]?eps=0.1) |

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
class: text-center
---

<div class="split-panel">
<div>
<a href="https://robots.engin.umich.edu/Projects/NGV" target="_blank" class="side-image">
<img src="/images/ford-fusion-ngv.jpg" alt="A Ford Fusion Next Generation Vehicle autonomous research car" />
</a>
</div>
<div>
<a href="https://robots.engin.umich.edu/Projects/NGV" target="_blank" class="side-image">
<img src="/images/ford-fusion-ngv2.jpg" alt="LIDAR point-cloud view from a Ford Fusion Next Generation Vehicle, detecting a car, trees, and lane markings" />
</a>
</div>
</div>
<div class="image-credit mt-2">Ford Fusion Next Generation Vehicle &#183; <a href="https://robots.engin.umich.edu/Projects/NGV">robots.engin.umich.edu/Projects/NGV</a></div>
<div class="image-caption">Go Blue!</div>
<div class="credit">AutoRob (autorob.org) &#183; Chad Jenkins (ocj@umich.edu) &#183; ocj-dev.github.io/kineval/pathfinding</div>
