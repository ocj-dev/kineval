# motion_planning/reference/ — implementation notes

The 2D half of the motion-planning module: RRT, RRT-Connect and RRT* over
the `project_pathplan` HTML5 canvas, where configuration space *is* the
plane. Written 2026-11-22.

Open `search_canvas.html` to run it:

```
search_canvas.html?search_alg=RRT-connect?planning_scene=scenes/spiral.js?q_init=[-0.65,2]?q_goal=[2,2]?seed=424242
```

---

## What is in this directory

| File | Origin |
|---|---|
| `rrt.js` | **new** — completes the upstream `project_pathplan/rrt.js` stencil |
| `random.js` | **new** — byte-identical copy of `kineval/random.js` |
| `collision.js`, `motion_plan.js` | **snapshots** of the `kineval/` files (see below) |
| `infrastructure.js` | carried from `pathfinding/reference/`, one addition |
| `search_canvas.html` | carried from `pathfinding/reference/`, **edited** |
| `draw.js`, `graph_search.js`, `scenes/` | carried unchanged |

`graph_search.js` is carried because three separate things need it: the
carried `search_canvas.html` loads it, the verification compares RRT
against A\* on the same scenes, and the deck's strongest interactive panel
is A\* and RRT-Connect side by side on the same five worlds.

Each module's `reference/` is self-contained by design — Slidev's `<<<`
importer cannot read outside the workspace root that
`motion_planning/pnpm-workspace.yaml` pins at `motion_planning/`.

---

## `#region` inventory

Every function is wrapped in `// #region name` / `// #endregion name` so
the deck can `<<<`-quote by region.

`rrt.js`: `query-is-valid`, `iterate-rrt`, `iterate-rrt-connect`,
`iterate-rrt-star`, `random-config`, `configuration-distance`,
`new-config`, `edge-is-valid`, `find-nearest-neighbor`,
`find-neighbors-within`, `extend-rrt`, `connect-rrt`, `dfs-path`,
`vertex-cost`, `path-length`, `smooth-path`.

`random.js`: `seed-random`, `random-seed-value`, `random`,
`random-in-range`, `random-integer`.

The carried files keep the regions they already had — notably
`appendix-collision` in `infrastructure.js` (the `testCollision` the whole
2D module is built on) and `appendix-animate` in `draw.js` (the dispatch
that calls `iterateRRT` / `iterateRRTConnect` / `iterateRRTStar`).

---

## The contract `rrt.js` has to meet

`draw.js`'s animation loop calls one iterate function per frame and reads
the returned string:

- `"failed"` — no path **this iteration**; keep going. This does *not*
  mean "give up", and the distinction matters: `draw.js`'s RRT case loops
  `while (search_result == "failed")` to speed up the visualization, so a
  function returning `"failed"` to mean "stop" would hang the browser.
- `"succeeded"` — a path was found.
- `"extended"` — a step was taken.

`drawHighlightedPath(path)` reads `path[q].vertex` (`draw.js:152`), so
**`rrt_path` is an array of tree vertex objects, not of bare
configurations**. Every path function in `rrt.js` reaches through
`.vertex` for coordinates. This was found by running it, not by reading
it; a path of raw `[x, y]` arrays renders as nothing at all, silently.

---

## Edits to the carried files

Two, both necessary, both noted in the files themselves.

**`search_canvas.html`** — adds `<script src="rrt.js">` and
`<script src="random.js">`, seeds the generator before `init()`, and
documents the `seed` URL parameter and the three new `search_alg` values.
Its line-13 comment used to say that `rrt.js` is intentionally absent
"pending the planned motion_planning/ deck"; this *is* that module, so the
comment now describes the current state. The `pathfinding/` copy is
untouched and still omits `rrt.js`, which is correct for that deck.

**`infrastructure.js`** — resets `a_is_start_tree = true` wherever `T_a`
and `T_b` are initialized, in both `initSearch` and `restartSearch`.
RRT-Connect swaps the two trees every iteration and has to know which one
is still rooted at `q_init` when it joins the two halves of a found path;
without the reset, clicking a new goal (which calls `restartSearch`) can
leave the flag stale and the drawn path runs goal-to-start through the
middle. Nothing else is changed.

---

## What the Phase 0 findings this code illustrates

- **Plain RRT is LaValle 1998, not Kuffner & LaValle 2000.** The course
  deck credits `[Kuffner, LaValle 2000]` on pp.83–89, which is the
  RRT-Connect paper (review findings R-3). `iterateRRT`'s header carries
  the correct attribution, and `iterateRRTConnect`'s carries the one the
  deck gets right.
- **RRT-Connect has no goal bias.** The deck is clean on this; the
  commented-out `if (random() < 0.2) q_rand = q_goal` in `iterateRRT` is
  there precisely so the contrast can be run rather than described.
- **Probabilistic completeness is not optimality.** The deck has a slide
  titled "RRT Probabilistic Completeness" that never states the property
  and never mentions that RRT is *not* asymptotically optimal; RRT\* is
  absent from 269 slides except as a grad-section assignment line (review
  findings R-4, G-2). `iterateRRTStar` is implemented here as the
  contrast, with choose-parent and rewire both visible, so the difference
  can be shown rather than asserted.
- **Collision checking dominates.** `edgeIsValid`'s resolution is the knob;
  see `kineval/IMPLEMENTATION_NOTES.md` for the measured numbers.

---

## Things that surprised us

**`search_canvas.html`'s own default goal is inside an obstacle in two of
the five scenes.** `q_goal = [4,4]` sits inside `cul_de_sac`'s obstacle
(`x ∈ [2.2,4.1]`, `y ∈ [3.7,4.0]`), and `[4.6,4.6]` — the first goal tried
when building the verification — sits inside `spiral`'s outer ring. Neither
scene is wrong; the default predates them.

This matters more than it sounds, because of how RRT-Connect fails at it.
`T_b` is rooted **at the goal**, so the planner grows a tree happily from a
configuration inside a wall and returns a path whose last vertex is inside
that wall. No error, no warning, a drawn path. That is a silent wrong
answer, and it is the single easiest way to lose an afternoon in this
module.

`rrt.js#queryIsValid` now checks both ends on every iteration and refuses
the query with a named reason. It is checked every iteration rather than
only the first, because gating on `search_iter_count === 0` misses the case
entirely whenever anything has already stepped the search — including the
page's own animation loop, which starts on load. Two point-in-rectangle
tests per iteration cost nothing.

**RRT\*'s path edges are legitimately longer than one step.** The
verification initially asserted that consecutive path configurations are
within `rrt_step_size` for all three algorithms. True for RRT and
RRT-Connect, which build a path one extension at a time; false for RRT\*,
whose choose-parent step deliberately attaches a vertex to a cheaper
neighbour up to `rrt_star_radius` steps away, and whose rewire step
re-parents across the same neighbourhood. A path edge longer than a step
is *evidence that rewiring is working*. The check is now per-algorithm.

**Shortcut smoothing cannot fix a suboptimal homotopy class.** It removes
detours; it cannot move a path to the other side of an obstacle. A path
that went left stays left however long it is smoothed. That is precisely
the gap RRT\* closes and shortcutting does not, and it is visible by
running the two on `diagonal_staircase`: RRT-Connect smooths 17.8 to 13.5,
and RRT\* converges to about 10.5.

---

## Snapshots

`collision.js` and `motion_plan.js` here are **dated snapshots** of
`kineval/collision.js` and `kineval/motion_plan.js`, each with a
provenance header. They exist so the deck can quote the viewer's planner,
which it otherwise cannot reach. **Edit the `kineval/` originals and
re-snapshot**; do not edit these.

`random.js` is a byte-identical copy rather than a snapshot — it is a
dependency of `rrt.js` here and of `motion_plan.js` there, and the Phase 4
harness asserts the two files are identical.

Known and accepted: `forward_kinematics/reference/` and
`inverse_kinematics/reference/` carry per-module copies of viewer code for
the same reason, and those **have already drifted** from `kineval/`. That
is the cost of a per-module `reference/` convention in a repository with
no build step. A `diff` is the cheap check, and the verification harness
runs it for this module's copies.
