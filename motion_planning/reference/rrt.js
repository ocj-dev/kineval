/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | RRT Methods

    COMPLETED REFERENCE IMPLEMENTATION of the kineval-stencil
    project_pathplan module's rrt.js stencil: RRT, RRT-Connect and
    RRT* over the 2D canvas.

    CONFIGURATION SPACE HERE IS THE PLANE. q is [x, y], the robot is a
    point, and testCollision(q) (infrastructure.js, #region
    appendix-collision) is a point-against-axis-aligned-rectangle test
    over the scene's obstacle list. That is the entire difference between
    this file and kineval/motion_plan.js, which runs the same algorithm
    over a robot's own configuration space -- six base DOFs plus every
    non-fixed joint, with a full forward-kinematics traversal behind every
    collision query. Reading the two side by side is the point: the
    algorithm does not change at all, and everything around it does.

    The duplication is therefore deliberate and is not an oversight. It is
    also structurally required -- each module's reference/ directory must
    be self-contained so the Slidev deck can quote it, and the static
    deploy copies these trees separately.

    THE THREE-VALUED RETURN. search_canvas.html's animation loop
    (draw.js, #region appendix-animate) calls iterateRRT/iterateRRTConnect
    once per frame and reads the returned string, so these functions must
    return exactly:

      "failed"    -- no path this iteration; keep going (NOT "gave up")
      "succeeded" -- a path was found; stop
      "extended"  -- a step was taken

    "failed" meaning "not yet" rather than "give up" is upstream's own
    vocabulary and is worth reading twice, because draw.js's RRT case
    loops `while (search_result == "failed")` to speed up the
    visualization -- so a function that returned "failed" to mean "stop"
    would hang the browser.

    WHAT RRT-CONNECT GUARANTEES. Probabilistic completeness: if a path
    exists, the probability of finding one tends to 1 as samples tend to
    infinity. It is not complete -- it can never report that no path
    exists -- and it is not asymptotically optimal, which is why
    iterateRRTStar is here as the contrast rather than as an alternative.
    search_max_iterations (infrastructure.js) is what bounds the search.

    All randomness comes from random.js's seeded generator, never
    Math.random(), so ?seed= reproduces a tree exactly.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// Step length for one extension, in world units. The canvas worlds all
// span roughly [-2, 6], so 0.5 crosses a world in about sixteen steps.
var rrt_step_size = 0.5;

// How finely an edge is checked for collision, as a fraction of a step.
// See edgeIsValid for why checking only the endpoint is not enough.
var rrt_edge_resolution = 0.2;

// Shortcut-smoothing attempts once a path is found.
var rrt_smoothing_attempts = 150;

// RRT*'s neighbourhood radius, as a multiple of the step size.
var rrt_star_radius = 2.5;

// The found path, as an array of tree VERTEX OBJECTS -- the form
// draw.js#drawHighlightedPath reads (`path[q].vertex`). Declared here
// rather than left as an implicit global assigned on success, so that
// `rrt_path` is a defined empty array before any path exists.
var rrt_path = [];

// The unsmoothed path, kept alongside the smoothed rrt_path that draw.js
// renders. Smoothing replaces runs of step-sized edges with single long
// shortcuts, so "consecutive path vertices are within one step" is a
// property of the RAW path only -- verification needs both.
var rrt_path_raw = [];

//////////////////////////////////////////////////
/////     ITERATION ENTRY POINTS
//////////////////////////////////////////////////

// #region query-is-valid
// Is this planning query answerable at all?
//
// If q_init or q_goal is itself inside an obstacle, no path exists and no
// amount of sampling will find one -- but the planner has no way to
// notice: RRT-Connect's second tree is rooted AT the goal, so it will
// happily grow from a configuration in collision and return a path whose
// last vertex is inside a wall. That is a silent wrong answer, and it is
// easy to produce: search_canvas.html's default q_goal of [4,4] lands
// inside an obstacle in the cul_de_sac scene.
//
// Checked on EVERY iteration rather than only the first. Two point-in-
// rectangle tests cost nothing next to an extension, and gating on
// `search_iter_count === 0` would miss the case entirely whenever
// anything has already stepped the search -- including the page's own
// animation loop, which starts on load. The warning is printed once.
var query_validity_reported = false;
function queryIsValid() {
    var bad = null;
    if (testCollision(q_init)) bad = 'start configuration [' + q_init + ']';
    else if (testCollision(q_goal)) bad = 'goal configuration [' + q_goal + ']';
    if (!bad) return true;

    if (!query_validity_reported) {
        query_validity_reported = true;
        console.warn('rrt: ' + bad + ' is inside an obstacle; no path can exist');
    }
    return false;
}
// #endregion query-is-valid

// #region iterate-rrt
// One iteration of single-tree RRT: sample, extend the tree toward the
// sample, and check whether the goal has come within reach.
//
// This is the algorithm from LaValle's 1998 tech report (TR 98-11), NOT
// from Kuffner & LaValle 2000, which is RRT-Connect -- an attribution the
// course deck currently gets wrong on its pp.83-89 (see the Phase 0
// review findings, R-3/K-4).
//
// Kept here as the contrast that makes iterateRRTConnect worth having.
// Run them on the same scene: single-tree RRT fills the space more or
// less uniformly before stumbling on the goal, because nothing pulls it
// there. The commented-out goal bias below is the usual patch, and
// RRT-Connect's second tree is the better one.
function iterateRRT() {
    if (!queryIsValid()) { search_iterate = false; return "failed"; }
    if (search_iter_count > search_max_iterations) { search_iterate = false; return "failed"; }
    search_iter_count++;

    var q_rand = randomConfig();

    // Goal biasing: sample the goal itself some fraction of the time.
    // Deliberately left off. It is a different algorithm, it is what the
    // deck's RRT-Connect slides are sometimes wrongly illustrated with,
    // and RRT-Connect does not use it. Uncomment to see the difference.
    // if (random() < 0.2) q_rand = q_goal.slice();

    var result = extendRRT(T_a, q_rand);
    if (result === "trapped") return "failed";

    // has the newest vertex brought the goal within one step?
    var q_new = T_a.vertices[T_a.newest].vertex;
    if (configurationDistance(q_new, q_goal) < rrt_step_size &&
        edgeIsValid(q_new, q_goal)) {

        var q_new_index = T_a.newest;
        insertTreeVertex(T_a, q_goal.slice());
        insertTreeEdge(T_a, T_a.newest, q_new_index);
        T_a.vertices[T_a.newest].parent = q_new_index;

        rrt_path_raw = dfsPath(T_a, T_a.vertices[0], T_a.vertices[T_a.newest]);
        rrt_path = smoothPath(rrt_path_raw);
        path_length = pathLength(rrt_path);
        draw_path = true;
        search_iterate = false;
        return "succeeded";
    }

    return "extended";
}
// #endregion iterate-rrt

// #region iterate-rrt-connect
// One iteration of RRT-Connect [Kuffner & LaValle, ICRA 2000], in the
// paper's own four steps:
//
//   0. two trees, T_a from q_init and T_b from q_goal
//      (initialized by infrastructure.js#initSearch)
//   1. extend T_a one step toward a uniformly random configuration
//   2. GREEDILY connect T_b toward T_a's newest vertex -- extend
//      repeatedly until blocked or arrived
//   3. swap the trees and repeat
//
// THERE IS NO GOAL BIAS, and there is no need for one: the second tree
// does the job a goal bias is trying to do, and does it from both ends.
//
// Step 2's greediness is the whole point of the "Connect" in the name.
// A uniform sampler almost never lands inside the narrow1/narrow2
// scenes' gaps; what happens instead is that one sample lands beyond a
// gap and connectRRT drags the tree all the way through it in a single
// iteration. Replace connectRRT with a single extendRRT and the planner
// becomes bidirectional RRT, which is markedly slower on exactly those
// scenes -- a one-line experiment worth running.
function iterateRRTConnect() {
    if (!queryIsValid()) { search_iterate = false; return "failed"; }
    if (search_iter_count > search_max_iterations) { search_iterate = false; return "failed"; }
    search_iter_count++;

    var q_rand = randomConfig();

    // 1. extend the first tree toward the sample
    if (extendRRT(T_a, q_rand) !== "trapped") {

        // 2. greedily connect the second tree toward where the first
        //    just arrived
        var q_new = T_a.vertices[T_a.newest].vertex;
        if (connectRRT(T_b, q_new) === "reached") {

            // the trees met. Each half runs from its own root to the
            // meeting point, so one is reversed and they are joined.
            var path_a = dfsPath(T_a, T_a.vertices[0], T_a.vertices[T_a.newest]);
            var path_b = dfsPath(T_b, T_b.vertices[0], T_b.vertices[T_b.newest]);

            // a_is_start_tree tracks which tree is still the one rooted
            // at q_init after however many swaps have happened
            if (a_is_start_tree) {
                rrt_path_raw = path_a.concat(path_b.slice().reverse());
            } else {
                rrt_path_raw = path_b.concat(path_a.slice().reverse());
            }

            rrt_path = smoothPath(rrt_path_raw);
            path_length = pathLength(rrt_path);
            draw_path = true;
            search_iterate = false;
            return "succeeded";
        }
    }

    // 3. swap, so both trees grow. Without this one tree does all the
    //    work and the algorithm degenerates toward single-tree RRT.
    var temp = T_a;
    T_a = T_b;
    T_b = temp;
    a_is_start_tree = !a_is_start_tree;

    return "extended";
}
// #endregion iterate-rrt-connect

// #region iterate-rrt-star
// One iteration of RRT* [Karaman & Frazzoli, RSS 2010 / IJRR 2011].
//
// Here as the answer to the question RRT-Connect provokes: the path it
// returns is visibly not the shortest one, and smoothing improves it but
// cannot fix it. RRT* is ASYMPTOTICALLY OPTIMAL -- its path cost
// converges to the optimum as samples tend to infinity -- where RRT and
// RRT-Connect are only probabilistically complete. That distinction is
// the single most common misconception about RRT and neither course deck
// currently states it (Phase 0 review findings, G-2).
//
// Two steps are what buy the optimality, and both are visible below:
//
//   CHOOSE PARENT: the new vertex attaches not to its nearest neighbour
//     but to whichever nearby vertex gives it the lowest cost-from-root.
//   REWIRE: every nearby vertex then checks whether routing through the
//     new vertex would lower ITS cost, and re-parents if so.
//
// Rewiring is why RRT* keeps improving a path it has already found, and
// why it is single-tree: the cost-from-root bookkeeping that makes it
// work has no natural meaning in a tree grown backward from the goal.
// That is the honest reason there is no "RRT*-Connect" here.
function iterateRRTStar() {
    if (!queryIsValid()) { search_iterate = false; return "failed"; }
    if (search_iter_count > search_max_iterations) { search_iterate = false; return "failed"; }
    search_iter_count++;

    var q_rand = randomConfig();
    var near = findNearestNeighbor(q_rand, T_a);
    var q_new = newConfig(near.config, q_rand, rrt_step_size);

    if (!edgeIsValid(near.config, q_new)) return "failed";

    var radius = rrt_star_radius * rrt_step_size;
    var neighbors = findNeighborsWithin(q_new, T_a, radius);

    // CHOOSE PARENT: start with the nearest, then take any neighbour that
    // reaches q_new more cheaply by a validated edge
    var best_idx = near.index;
    var best_cost = vertexCost(T_a, near.index) + configurationDistance(near.config, q_new);
    var i;
    for (i = 0; i < neighbors.length; i++) {
        var idx = neighbors[i];
        var cost = vertexCost(T_a, idx) + configurationDistance(T_a.vertices[idx].vertex, q_new);
        if (cost < best_cost && edgeIsValid(T_a.vertices[idx].vertex, q_new)) {
            best_cost = cost;
            best_idx = idx;
        }
    }

    insertTreeVertex(T_a, q_new);
    var new_idx = T_a.newest;
    insertTreeEdge(T_a, new_idx, best_idx);
    T_a.vertices[new_idx].parent = best_idx;
    T_a.vertices[new_idx].cost = best_cost;

    // REWIRE: now that q_new exists, does it give any neighbour a cheaper
    // route than the one it has?
    for (i = 0; i < neighbors.length; i++) {
        var n_idx = neighbors[i];
        if (n_idx === best_idx) continue;
        var through = best_cost + configurationDistance(q_new, T_a.vertices[n_idx].vertex);
        if (through < vertexCost(T_a, n_idx) &&
            edgeIsValid(q_new, T_a.vertices[n_idx].vertex)) {
            T_a.vertices[n_idx].parent = new_idx;
            T_a.vertices[n_idx].cost = through;
        }
    }

    // RRT* does not stop at the first path it finds -- that is the whole
    // idea -- so the goal check records a path and keeps going, replacing
    // it whenever a cheaper one appears.
    if (configurationDistance(q_new, q_goal) < rrt_step_size && edgeIsValid(q_new, q_goal)) {
        insertTreeVertex(T_a, q_goal.slice());
        var goal_idx = T_a.newest;
        insertTreeEdge(T_a, goal_idx, new_idx);
        T_a.vertices[goal_idx].parent = new_idx;
        T_a.vertices[goal_idx].cost = best_cost + configurationDistance(q_new, q_goal);

        var candidate = parentPath(T_a, goal_idx);
        var candidate_length = pathLength(candidate);
        if (!draw_path || candidate_length < path_length) {
            rrt_path_raw = candidate;
            rrt_path = candidate;
            path_length = candidate_length;
            draw_path = true;
        }
        return "succeeded";
    }

    return "extended";
}
// #endregion iterate-rrt-star

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

// #region random-config
// A uniformly random configuration in the plane.
//
// Drawn over the scene's own extent rather than a fixed box. All the
// canvas scenes are walled in by four obstacle rectangles spanning about
// [-1.8, 5.8] (infrastructure.js#setPlanningScene), so sampling well
// outside that only wastes collision checks.
//
// Uniform, with no goal bias: see iterateRRT's note.
function randomConfig() {
    return [randomInRange(-2, 6), randomInRange(-2, 6)];
}
// #endregion random-config

// #region configuration-distance
// Euclidean distance in the plane.
//
// Worth noticing how short this is compared with
// kineval/motion_plan.js#configurationDistance, which has to weight each
// DOF by its own range because that q mixes metres and radians. Here both
// components are the same unit, so there is nothing to weight -- the unit
// problem is a property of the robot's configuration space, not of the
// algorithm.
function configurationDistance(q1, q2) {
    var dx = q1[0] - q2[0];
    var dy = q1[1] - q2[1];
    return Math.sqrt(dx * dx + dy * dy);
}
// #endregion configuration-distance

// #region new-config
// A configuration one step from q_near toward q_target, never
// overshooting it.
function newConfig(q_near, q_target, step) {
    var distance = configurationDistance(q_target, q_near);

    // the sample landed on the vertex; without this guard the division
    // below produces a configuration of NaNs
    if (distance < 1e-9) return q_near.slice();

    var ratio = Math.min(1, step / distance);
    return [
        q_near[0] + ratio * (q_target[0] - q_near[0]),
        q_near[1] + ratio * (q_target[1] - q_near[1])
    ];
}
// #endregion new-config

// #region edge-is-valid
// Is every configuration along the straight edge from qA to qB free?
//
// Checking only the endpoint is not enough, and the failure is a quiet
// one: two free endpoints a step apart can straddle a thin obstacle, and
// the planner will return a path that walks straight through a wall. The
// three_sections and narrow1 scenes have walls thin enough for a 0.5 step
// to cross, so this is reachable here, not hypothetical.
//
// Finer resolution is safer and costs proportionally more collision
// checks -- which are the dominant cost of any sampling-based planner,
// a point neither course deck makes (Phase 0 findings, G-1). There is no
// resolution that is provably safe; this is discrete collision checking,
// and the alternative is continuous collision detection.
function edgeIsValid(qA, qB) {
    var distance = configurationDistance(qA, qB);
    var steps = Math.max(1, Math.ceil(distance / (rrt_edge_resolution * rrt_step_size)));

    var s;
    // from 1, because qA is already on the tree and therefore known free
    for (s = 1; s <= steps; s++) {
        var t = s / steps;
        if (testCollision([
            qA[0] + t * (qB[0] - qA[0]),
            qA[1] + t * (qB[1] - qA[1])
        ])) return false;
    }
    return true;
}
// #endregion edge-is-valid

// #region find-nearest-neighbor
// The tree vertex closest to q, by linear scan.
//
// O(n) per call over a tree that grows every iteration, so quadratic over
// a whole search. A k-d tree fixes it; this stays linear because it is
// four lines a student can read and because collision checking still
// costs more. Nearest-neighbour cost is the SECOND thing to optimize in a
// sampling-based planner, and the deck's p.79 lists it without saying
// what comes first.
function findNearestNeighbor(q, tree) {
    var best_index = 0;
    var best_distance = Infinity;
    var j;
    for (j = 0; j < tree.vertices.length; j++) {
        var d = configurationDistance(q, tree.vertices[j].vertex);
        if (d < best_distance) {
            best_distance = d;
            best_index = j;
        }
    }
    return {
        index: best_index,
        config: tree.vertices[best_index].vertex,
        distance: best_distance
    };
}
// #endregion find-nearest-neighbor

// #region find-neighbors-within
// Every tree vertex within `radius` of q. RRT*'s choose-parent and rewire
// steps both work over this set; plain RRT and RRT-Connect never call it.
function findNeighborsWithin(q, tree, radius) {
    var found = [];
    var j;
    for (j = 0; j < tree.vertices.length; j++) {
        if (configurationDistance(q, tree.vertices[j].vertex) <= radius) found.push(j);
    }
    return found;
}
// #endregion find-neighbors-within

// #region extend-rrt
// One step of the tree toward q, with the paper's three-valued result:
//
//   "reached"  -- the new vertex IS q (within a step)
//   "advanced" -- a step was taken, q is still further on
//   "trapped"  -- blocked; nothing added
//
// connectRRT loops while this says "advanced". That loop is the entire
// difference between RRT and RRT-Connect.
function extendRRT(tree, q) {
    var near = findNearestNeighbor(q, tree);
    var q_new = newConfig(near.config, q, rrt_step_size);

    // validate the whole edge, not just where it lands
    if (!edgeIsValid(near.config, q_new)) return "trapped";

    insertTreeVertex(tree, q_new);
    insertTreeEdge(tree, tree.newest, near.index);
    tree.vertices[tree.newest].parent = near.index;

    // "reached" means the new vertex IS q, not merely within a step of
    // it. The looser version leaves the final sub-step gap unvalidated,
    // and in RRT-Connect that gap is exactly the junction between the
    // two trees -- so the planner splices the two halves together across
    // an edge nothing ever collision-checked. newConfig clamps at the
    // target, so exact arrival costs at most one extra extension and
    // makes the whole path validated by construction. (The same bug, and
    // the same fix, in kineval/motion_plan.js#rrtExtend.)
    if (configurationDistance(q_new, q) < 1e-9) return "reached";
    return "advanced";
}
// #endregion extend-rrt

// #region connect-rrt
// The greedy half of RRT-Connect: extend toward q over and over until
// something stops it. See iterateRRTConnect's note for why this, and not
// bidirectionality alone, is what crosses a narrow passage.
function connectRRT(tree, q) {
    var result = "advanced";
    while (result === "advanced") {
        result = extendRRT(tree, q);
    }
    return result;
}
// #endregion connect-rrt

//////////////////////////////////////////////////
/////     PATH EXTRACTION AND SMOOTHING
//////////////////////////////////////////////////

// #region dfs-path
// The configurations from a tree's root to a given vertex.
//
// Named dfsPath because that is the name upstream's stencil asks for, but
// it does not search: it walks parent pointers, which extendRRT records.
// A tree has exactly one path from its root to any vertex, so there is
// nothing to search for, and a depth-first search would re-derive by
// exploration what the tree already knows. (Upstream's own kineval
// version really does recurse, and its visited-flag reset silently never
// runs -- see the Phase 0 findings, K-6.1.)
//
// Returns VERTEX OBJECTS, not bare configurations, because that is what
// draw.js#drawHighlightedPath reads (`path[q].vertex`, draw.js:152). Every
// path function below therefore reaches through `.vertex` for coordinates.
function dfsPath(tree, q_start_vertex, q_end_vertex) {
    var end_index = tree.vertices.indexOf(q_end_vertex);
    if (end_index < 0) return [];
    return parentPath(tree, end_index);
}

// #region parent-path
// Walk parent pointers from a vertex back to the root, then reverse.
// dfsPath is a thin wrapper that finds the index first.
function parentPath(tree, index) {
    var path = [];
    var i = index;
    var guard = 0;
    while (i !== undefined && i >= 0 && guard++ < tree.vertices.length + 1) {
        path.push(tree.vertices[i]);
        i = tree.vertices[i].parent;
    }
    path.reverse(); // walked root-ward, wanted root-first
    return path;
}
// #endregion parent-path
// #endregion dfs-path

// #region vertex-cost
// Cost-from-root of a tree vertex, for RRT*. Cached on the vertex by
// iterateRRTStar; the root is free.
function vertexCost(tree, index) {
    var v = tree.vertices[index];
    if (v.cost === undefined) v.cost = (v.parent === undefined || v.parent < 0)
        ? 0
        : vertexCost(tree, v.parent) + configurationDistance(tree.vertices[v.parent].vertex, v.vertex);
    return v.cost;
}
// #endregion vertex-cost

// #region path-length
function pathLength(path) {
    var total = 0;
    var i;
    for (i = 1; i < path.length; i++) {
        total += configurationDistance(path[i - 1].vertex, path[i].vertex);
    }
    return total;
}
// #endregion path-length

// #region smooth-path
// Shortcut smoothing: pick two configurations on the path at random and,
// if the straight edge between them is free, delete everything between.
//
// RRT paths wander visibly even across open space, because the tree is
// grown by random extension. Smoothing cannot make the path invalid --
// every shortcut is validated by the same edgeIsValid the planner uses --
// and cannot make it optimal either. It removes detours; it cannot move
// the path to the other side of an obstacle. A path that went left stays
// left however long you smooth it, which is exactly the gap RRT* closes
// and shortcutting does not.
//
// Candidate pairs come from the shared seeded generator, so smoothing is
// as reproducible as the search.
function smoothPath(path) {
    if (path.length < 3) return path.slice();

    var smoothed = path.slice();
    var attempt;
    for (attempt = 0; attempt < rrt_smoothing_attempts; attempt++) {
        if (smoothed.length < 3) break;

        var a = randomInteger(smoothed.length);
        var b = randomInteger(smoothed.length);
        if (a > b) { var t = a; a = b; b = t; }
        if (b - a < 2) continue; // nothing between them to remove

        if (edgeIsValid(smoothed[a].vertex, smoothed[b].vertex)) {
            smoothed.splice(a + 1, b - a - 1);
        }
    }
    return smoothed;
}
// #endregion smooth-path
