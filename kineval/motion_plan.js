/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | RRT-Connect Motion Planning

    Completes the upstream kineval-stencil's kineval_rrt_connect.js:
    bidirectional RRT-Connect over the robot's own configuration space.

    This is the SECOND implementation of RRT-Connect in this repository,
    and the duplication is deliberate rather than an oversight. The other
    one, ../motion_planning/reference/rrt.js, plans over a 2D canvas where
    configuration space IS the plane and the collision test is a point
    against an axis-aligned box. This one plans over an n-dimensional
    space -- six base DOFs plus every non-fixed joint -- where the
    collision test is a full-robot forward-kinematics traversal. Same
    algorithm, same five function names, and almost nothing else in
    common. Reading them side by side is the clearest available statement
    of what "configuration space" actually buys you. (It is also required
    structurally: each module's reference/ directory has to be
    self-contained for the Slidev deck to quote it.)

    WHAT RRT-CONNECT IS, in the four steps the lecture gives
    (autorob_2026_11_roadmaps_rrt, pp.95-99, [Kuffner & LaValle, ICRA
    2000]):

      0. two trees, rooted at q_start and q_goal
      1. extend tree A one step toward a uniformly random configuration
      2. GREEDILY connect tree B toward A's newest vertex -- extend
         repeatedly, not once, until blocked or arrived
      3. swap the roles of A and B, and repeat

    THERE IS NO GOAL BIAS. Sampling is uniform over the configuration
    space; the second tree is what replaces goal biasing, and adding a
    bias on top would be a different algorithm. The deck is correct on
    this (review findings N-1/K-4); the common lecture error of showing
    goal-biased single-tree RRT under an RRT-Connect heading is not
    present and is not reproduced here.

    WHAT IS GUARANTEED, AND WHAT IS NOT. RRT-Connect is PROBABILISTICALLY
    COMPLETE: if a path exists, the probability of finding one tends to 1
    as samples tend to infinity. It is NOT complete -- it can never report
    "no path exists", only "not yet" -- and it is NOT asymptotically
    optimal, so the path it returns is not the shortest and smoothing it
    does not make it so. world_sealed_enclosure.js is the world where the
    difference bites, and PLANNER_MAX_ITERATIONS below is the only thing
    that keeps it from running forever.

    WHAT THIS DOES THAT UPSTREAM DOES NOT (all recorded in
    IMPLEMENTATION_NOTES.md, all traceable to the review findings' K-6.1):

      - validates EDGES, not just endpoints. Upstream's new_config tests
        only the configuration it arrives at, so an eps-length step can
        pass clean through a thin obstacle with both ends free.
      - weights the distance metric per DOF. Upstream's
        compute_configuration_distance has the weighting branch but both
        sides compute the same thing, so metres and radians are summed
        raw.
      - treats angles as lying on a continuous real line with NO modulo
        wrap, which is a deliberate reversal of upstream's own line-29
        note; see configurationDifference for the three reasons.
      - scales the step size with the configuration space's extent rather
        than hardcoding 0.5, so world_random (52x52) and world_basic
        (10x10) both work.
      - draws from one seeded generator (../random.js), so a plan is
        reproducible.
      - counts collision checks and nearest-neighbour comparisons, which
        is how this module answers the cost-model question neither lecture
        deck addresses (review findings G-1).
      - iterates under a cap and reports failure, rather than looping on
        a flag.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// Hard cap on iterations before the planner gives up and reports 'failed'.
// This is what makes world_sealed_enclosure terminate: no amount of
// sampling can prove a path does not exist, so something has to stop.
//
// 5000 is chosen from measurement, and it is a COMPROMISE rather than a
// value that makes every query solvable. Measured across the six robots
// and fourteen worlds:
//
//   urdf_example / mr2, most worlds        1 to ~100 iterations
//   sawyer (14 DOF), open worlds           1 to ~150
//   crawler (30 DOF), world_basic          ~450, about 9 seconds
//   crawler (30 DOF), world_s              unsolved
//   fetch / baxter / sawyer, walled worlds often unsolved
//
// That spread is not a defect to tune away; it is the curse of
// dimensionality, measured in our own code. Every arm joint is sampled
// uniformly even when a query only needs the base to move, so both trees
// must random-walk back into agreement across all of those extra
// dimensions before they can connect. The lecture's "10^21
// configurations" slide (roadmaps_rrt p.31) is about exactly this, and
// the crawler -- thirty DOFs, twenty-five links -- is where it bites
// hardest.
//
// Raising the cap buys surprisingly little. Nearest-neighbour cost grows
// QUADRATICALLY in tree size, and collision checks get more expensive as
// the trees spread, so the last thousand iterations cost several times
// what the first thousand did; going from 15000 to 40000 took one world
// from 9 seconds to 51. 5000 solves every world for the smaller robots,
// keeps the unsolvable case (world_sealed_enclosure) reporting failure
// in a few seconds, and bounds the worst case. The honest answer for the
// hard pairs is a k-d tree and a smarter sampler, which is a later
// module -- not a bigger number here.
var PLANNER_MAX_ITERATIONS = 5000;

// Planning budget per animation frame, in milliseconds. Planning is
// synchronous-in-frames: the main loop plans for at most this long and
// then yields, so the viewer never locks and the HUD can show progress.
// This is upstream's own intent -- its "asynch timing mechanism ... to
// avoid blocking and non-responsiveness in the browser".
//
// A TIME budget rather than an iteration count, which is what the first
// version used. One iteration costs wildly different amounts across the
// six robots: an iteration for urdf_example (9 DOF, 5 links) and one for
// the crawler (30 DOF, 25 links, every collision check an FK traversal
// over 24 joints) differ by more than an order of magnitude. A fixed 40
// iterations per frame is imperceptible on one and a visible freeze on
// the other. 12ms leaves room inside a 16ms frame for rendering.
var PLANNER_MS_PER_FRAME = 12;

// Step size as a fraction of the configuration space's own diameter under
// the weighted metric -- see plannerStepSize.
var PLANNER_STEP_FRACTION = 0.04;

// How finely an edge is checked for collision, as a fraction of the
// SMALLEST OBSTACLE RADIUS in the loaded world -- a distance in metres,
// not in configuration-space units. See edgeIsValid for why that
// distinction is load-bearing rather than pedantic. Smaller is safer and
// slower; this is the knob the cost-model discussion is about, together
// with nearestNeighbor.
var PLANNER_EDGE_RESOLUTION = 0.4;

// Ceiling on collision checks for a single edge, so one pathological
// edge cannot stall a frame. Clamping here means the edge is checked
// more coarsely than the resolution asks for, so it warns loudly rather
// than silently reintroducing the tunnelling bug.
var PLANNER_MAX_EDGE_SAMPLES = 400;

// Shortcut-smoothing attempts after a path is found.
var PLANNER_SMOOTHING_ATTEMPTS = 200;

//////////////////////////////////////////////////
/////   THE METRIC
//////////////////////////////////////////////////

// #region configuration-weights
// Per-DOF scale factors that make a configuration-space distance mean
// something.
//
// THE UNIT PROBLEM: q mixes metres (base translation, prismatic joints)
// and radians (base rotation, revolute and continuous joints) in one
// vector. Summing their squares raw -- which is what upstream does -- is
// adding metres to radians, so the metric's behaviour depends on the
// arbitrary choice of unit. In world_random, whose boundary is 52 metres
// across, translation then dominates so completely that the planner
// effectively ignores the arm; in a 1-metre world the reverse happens.
// Neither is a planner anyone can reason about.
//
// The fix is to divide each DOF by its own natural range, so every
// component of the difference is a FRACTION OF THAT DOF'S TRAVEL and the
// sum is dimensionless. A full sweep of a revolute joint and a full
// traverse of the world then count the same, which is the only
// defensible default in the absence of a task-specific cost.
//
// Returns, per q index: { scale }.
function configurationWeights(robot, qmap, world) {
    var weights = new Array(qmap.length);

    // translation DOFs: normalize by the world's own extent
    var spanX = Math.abs(world.boundary[1][0] - world.boundary[0][0]) || 1;
    var spanZ = Math.abs(world.boundary[1][2] - world.boundary[0][2]) || 1;
    // the boundary has no y extent (it is a ground-plane rectangle), so
    // use the larger horizontal span as a stand-in scale for height
    var spanY = Math.max(spanX, spanZ);

    weights[Q_BASE_X] = { scale: 1 / spanX };
    weights[Q_BASE_Y] = { scale: 1 / spanY };
    weights[Q_BASE_Z] = { scale: 1 / spanZ };

    // base orientation: a half-turn is the largest rotation worth
    // normalizing against in any one Euler angle
    weights[Q_BASE_ROLL] = { scale: 1 / Math.PI };
    weights[Q_BASE_PITCH] = { scale: 1 / Math.PI };
    weights[Q_BASE_YAW] = { scale: 1 / Math.PI };

    var x;
    for (x in qmap.names) {
        var i = qmap.names[x];
        var range = jointLimitRange(robot, x);
        if (!range) { weights[i] = { scale: 1 / Math.PI }; continue; }

        // each joint normalized by its own travel, so a full sweep of any
        // joint counts the same as a full sweep of any other. Nothing
        // here is treated as circular -- see configurationDifference.
        weights[i] = { scale: 1 / (Math.abs(range.upper - range.lower) || 1) };
    }

    return weights;
}
// #endregion configuration-weights

// #region configuration-difference
// The signed difference along one DOF. Plain subtraction, for every DOF
// including every angular one.
//
// ANGLES ARE TREATED AS LYING ON A CONTINUOUS REAL LINE, WITH NO MODULO
// WRAP. A joint at 0.1 radians and one at 6.3 radians are 6.2 radians
// apart, not 0.08: reaching the second from the first means winding the
// joint almost all the way round, and the planner must plan, validate
// and execute that motion rather than pretend it is a twitch.
//
// This is a deliberate reversal of what both upstream's own note
// ("Distance computation needs to consider modulo for joint angles",
// kineval_rrt_connect.js:29) and this module's implementation plan asked
// for, and it is the right call for three reasons:
//
//   1. A revolute joint with limits CANNOT wrap. Its short way round
//      passes through a region the joint physically cannot enter, so a
//      modulo metric reports a distance the robot cannot travel and an
//      interpolation that drives the joint through its own hard stop.
//
//   2. Even for a genuinely continuous joint, the wrap makes the metric
//      disagree with the motion. Under a modulo difference the edge
//      between yaw 0 and yaw 2*PI has length ZERO, so edge validation
//      performs a single check and the planner is free to splice in a
//      full revolution it never examined. That is not hypothetical --
//      it showed up as a path through a wall during verification.
//
//   3. It makes one rule cover every DOF. Metres and radians are then
//      treated alike: a difference is a difference, the weighting
//      (configurationWeights) is the only thing that makes them
//      comparable, and there is no second notion of distance to keep in
//      sync with interpolation, sampling and limit enforcement.
//
// The cost is that two configurations which LOOK identical on screen can
// be far apart in configuration space, and the planner may wind a
// continuous joint the long way round. That is a real and visible
// behaviour, and it is the honest one: the robot really does have to
// turn that far.
function configurationDifference(a, b) {
    return a - b;
}
// #endregion configuration-difference

// #region configuration-distance
// Weighted Euclidean distance between two configurations, dimensionless.
// Every call to this is one "nearest-neighbour comparison" for counting
// purposes; nearestNeighbor does a linear scan, so this runs once per tree
// vertex per iteration and is the planner's second-largest cost after
// collision checking.
function configurationDistance(q1, q2, weights) {
    var sum = 0;
    var i;
    for (i = 0; i < q1.length; i++) {
        var d = configurationDifference(q1[i], q2[i]) * weights[i].scale;
        sum += d * d;
    }
    return Math.sqrt(sum);
}
// #endregion configuration-distance

// #region planner-step-size
// How far one extension travels, under the weighted metric.
//
// Because the metric is normalized, the configuration space's own diameter
// is sqrt(number of DOFs) -- each DOF contributes at most 1. So a step is
// a fixed fraction of that diameter and automatically scales: the same
// fraction is a bigger number of metres in world_random than in
// world_basic, and a bigger number of radians for a joint with a wide
// range than a narrow one. Upstream hardcodes eps = 0.5 in raw mixed
// units, which is why its planner behaves completely differently from one
// world to the next.
function plannerStepSize(qmap) {
    return PLANNER_STEP_FRACTION * Math.sqrt(qmap.length);
}
// #endregion planner-step-size

//////////////////////////////////////////////////
/////   SAMPLING
//////////////////////////////////////////////////

// #region random-config
// A uniformly random configuration. Every draw comes from the shared
// seeded generator in random.js, never Math.random().
//
// BASE DOFs. The six base DOFs describe a free-floating rigid body, but
// all fourteen worlds are ground-plane worlds and the boundary test is an
// x-z extent. So the default, plan_base_dofs = 'planar', samples base_x,
// base_z and base_yaw over the world and HOLDS base_y, roll and pitch at
// whatever the robot started with.
//
// Why that is the default and not a shortcut: in a ground-plane world,
// sampling base roll and pitch spends almost every sample on a
// configuration that is either driven into the floor or physically
// meaningless, so the planner's apparent failure rate goes up by an order
// of magnitude while it is merely being asked the wrong question. The two
// other settings are available and are the interesting comparison:
//
//   'planar' (default) : x, z, yaw sampled; y, roll, pitch held
//   'free'             : all six sampled -- the honest SE(3) problem
//   'none'             : base pinned, arm-only planning. This is what
//                        world_arm_clutter needs.
function randomConfig(robot, qmap, world, params, qStart) {
    var mode = params.plan_base_dofs || 'planar';
    var q = new Array(qmap.length);
    var i;

    // start from the reference configuration, then overwrite whichever
    // base DOFs this mode samples -- so "held" means "held at the value
    // the robot actually had", not "held at zero" as upstream does
    for (i = 0; i < Q_BASE_DOF_COUNT; i++) q[i] = qStart[i];

    if (mode !== 'none') {
        // Sampled over the world's FULL boundary.
        //
        // An inset was tried here and removed. The reasoning for it was
        // that, since the boundary test checks every link's bounding-box
        // corners rather than upstream's base origin alone, a base within
        // one robot-reach of the wall usually puts some link outside it,
        // so those samples are wasted. That reasoning is correct and the
        // change still made things worse: world_barrier's gap is at
        // z < -8, hard against the boundary, and world_s's openings are
        // at the ends of its walls. Insetting by the robot's reach
        // removes exactly the region the passage is in, and sawyer went
        // from solving world_barrier in ~12000 iterations to not solving
        // it in 15000. Recorded so it is not re-attempted.
        q[Q_BASE_X] = randomInRange(world.boundary[0][0], world.boundary[1][0]);
        q[Q_BASE_Z] = randomInRange(world.boundary[0][2], world.boundary[1][2]);
        q[Q_BASE_YAW] = randomInRange(0, 2 * Math.PI);
    }
    if (mode === 'free') {
        // the boundary is planar, so height gets the horizontal span as
        // its range -- stated here rather than hidden, because it is the
        // one sampling bound no world file supplies
        var spanY = Math.max(
            Math.abs(world.boundary[1][0] - world.boundary[0][0]),
            Math.abs(world.boundary[1][2] - world.boundary[0][2])
        );
        q[Q_BASE_Y] = randomInRange(0, spanY);
        q[Q_BASE_ROLL] = randomInRange(0, 2 * Math.PI);
        q[Q_BASE_PITCH] = randomInRange(0, 2 * Math.PI);
    }

    // joint DOFs: each over its own interval, derived from its type and
    // declared limits. No joint is named here.
    var x;
    for (x in qmap.names) {
        var range = jointLimitRange(robot, x);
        q[qmap.names[x]] = range ? randomInRange(range.lower, range.upper) : 0;
    }

    return q;
}
// #endregion random-config

// #region new-config
// A configuration one step from qNear toward qTarget, under the weighted
// metric, clamped so it never overshoots the target.
//
// Returns the configuration only. Unlike upstream's new_config it does NOT
// also run the collision test and bundle the verdict -- that conflation is
// what makes upstream check only the endpoint, because by the time the
// caller sees the result the intermediate configurations are gone. Here
// the caller validates the whole edge (see edgeIsValid).
function newConfig(qNear, qTarget, weights, step) {
    var distance = configurationDistance(qTarget, qNear, weights);

    // the sample landed on the vertex: upstream divides by this and
    // returns a configuration of NaNs
    if (distance < 1e-9) return qNear.slice();

    return interpolateConfiguration(qNear, qTarget, Math.min(1, step / distance), weights);
}
// #endregion new-config

//////////////////////////////////////////////////
/////   VALIDITY
//////////////////////////////////////////////////

// #region interpolate-configuration
// The configuration a fraction t of the way along the straight edge from
// qA to qB. THE one definition of what "along an edge" means in this
// planner; edgeIsValid walks it, and anything auditing a path must use
// it too.
//
// Plain linear interpolation of every DOF, angular ones included, since
// angles lie on a continuous real line here with no wrap (see
// configurationDifference). An edge from yaw 0 to yaw 2*PI really is a
// full revolution, and every intermediate configuration of it is
// checked.
//
// Kept as one named function, rather than inlined at each site, so that
// the planner and anything auditing a path share a single definition of
// what "along an edge" means. An early version of the Phase 4
// verification rolled its own interpolation, disagreed with the
// planner's, and reported collisions on a path the planner had never
// produced -- the worst way for a test to fail.
function interpolateConfiguration(qA, qB, t, weights) {
    var q = new Array(qA.length);
    var i;
    for (i = 0; i < qA.length; i++) {
        q[i] = qA[i] + t * configurationDifference(qB[i], qA[i]);
    }
    return q;
}
// #endregion interpolate-configuration

// #region workspace-motion-bound
// An upper bound, in METRES, on how far any point on the robot can move
// when the configuration goes from qA to qB in a straight line.
//
// This is the quantity edge validation actually needs, and it is NOT the
// configuration-space distance -- see edgeIsValid for what goes wrong
// when the two are confused.
//
// The base's own translation contributes directly. Every rotational DOF
// contributes at most its angle change times the lever arm it acts
// through -- the POSE-INDEPENDENT radius of everything downstream of
// that joint (collision.js#jointLeverArms), since a wrist moves a hand
// and not a torso. Prismatic DOFs contribute their displacement. Base
// rotation swings the whole robot, so it gets the whole robot's radius.
//
// Still an upper bound, and loose in the SAFE direction, which is the
// direction that matters.
function workspaceMotionBound(robot, qmap, qA, qB, weights, validation) {
    var levers = validation.levers;
    var reach = levers.__base;
    // base translation, in metres, exactly
    var dx = qB[Q_BASE_X] - qA[Q_BASE_X];
    var dy = qB[Q_BASE_Y] - qA[Q_BASE_Y];
    var dz = qB[Q_BASE_Z] - qA[Q_BASE_Z];
    var motion = Math.sqrt(dx * dx + dy * dy + dz * dz);

    // base rotation: each Euler angle swings the whole robot about the base
    var i;
    for (i = Q_BASE_ROLL; i <= Q_BASE_YAW; i++) {
        motion += Math.abs(configurationDifference(qB[i], qA[i])) * reach;
    }

    // joints: prismatic in metres, everything else swinging through the
    // radius of its OWN subtree
    var x;
    for (x in qmap.names) {
        var k = qmap.names[x];
        var d = Math.abs(configurationDifference(qB[k], qA[k]));
        motion += (robot.joints[x].type === 'prismatic') ? d : d * (levers[x] || 0);
    }

    return motion;
}
// #endregion workspace-motion-bound

// #region edge-is-valid
// Is every configuration along the straight edge from qA to qB free?
//
// THIS IS THE FIX FOR UPSTREAM'S MOST CONSEQUENTIAL BUG. Upstream tests
// only the configuration an extension arrives at. Two collision-free
// endpoints do not imply a collision-free edge: with eps = 0.5 and
// radius-0.5 obstacles, a step can pass clean through a sphere and the
// planner will happily return a path that drives the robot through a wall.
// The failure is invisible in the viewer unless you step the plan
// frame-by-frame, which is exactly what n/b is for.
//
// RESOLUTION IS SET IN METRES, NOT IN CONFIGURATION-SPACE DISTANCE, and
// getting that wrong is the subtlest bug this module produced. The first
// version sampled the edge at a fixed fraction of the step size under
// the weighted metric -- which looks right, and tunnels. The weighted
// metric divides base translation by the WHOLE WORLD'S SPAN, so in a
// 21-metre world a weighted edge length of 0.13 can be a base movement
// of a full metre. Verification caught it exactly where it should: a
// plan escaped world_sealed_enclosure, and re-checking the offending
// edge at 100x resolution found 324 colliding configurations along a
// step the planner had declared free.
//
// So the spacing is derived from the world's smallest obstacle instead:
// no sample may let any point on the robot move further than a fraction
// of the smallest sphere's radius, which is the feature that could be
// passed through. workspaceMotionBound supplies the motion in metres.
//
// That is a choice with teeth. Halve PLANNER_EDGE_RESOLUTION and
// collision checks roughly double, and collision checking is the
// dominant cost (review findings G-1). Coarser is faster and tunnels;
// finer is slower and does not. NO resolution is provably safe -- this
// is discrete collision checking, and the honest alternative is
// continuous collision detection, which is a different lecture.
//
// Counts every check into counters.collisionChecks.
function edgeIsValid(robot, qmap, world, qA, qB, weights, counters, validation) {
    // how many samples the workspace criterion demands
    var motion = workspaceMotionBound(robot, qmap, qA, qB, weights, validation);
    var steps = Math.max(1, Math.ceil(motion / validation.metres));

    // Guard against a pathological edge demanding a runaway number of
    // checks. Clamping reintroduces the tunnelling risk, so it warns
    // rather than doing it quietly -- if this fires, the world has
    // obstacles far smaller than the robot and the step size wants
    // lowering, which is a real finding and not something to absorb.
    if (steps > PLANNER_MAX_EDGE_SAMPLES) {
        console.warn('kineval motion_plan: edge needs ' + steps + ' collision checks ' +
            '(workspace motion ' + motion.toFixed(2) + 'm, resolution ' +
            validation.metres.toFixed(3) + 'm); clamping to ' + PLANNER_MAX_EDGE_SAMPLES +
            '. Lower PLANNER_STEP_FRACTION for this world.');
        steps = PLANNER_MAX_EDGE_SAMPLES;
    }

    var s;
    // start at 1: qA is already known free (it is on the tree), and
    // checking it again would double the cost of a connect sweep
    for (s = 1; s <= steps; s++) {
        counters.collisionChecks++;
        if (poseIsCollision(robot, qmap,
                interpolateConfiguration(qA, qB, s / steps, weights), world)) {
            return false;
        }
    }
    return true;
}
// #endregion edge-is-valid

//////////////////////////////////////////////////
/////   THE TREE
//////////////////////////////////////////////////

// #region tree-init
// A tree holding one configuration. Vertices carry their configuration,
// their adjacency, and a parent index -- the parent is what makes path
// extraction a walk rather than a search (see findPath).
function treeInit(q) {
    return {
        vertices: [{ vertex: q, edges: [], parent: -1 }],
        newest: 0
    };
}
// #endregion tree-init

// #region tree-add-vertex
function treeAddVertex(tree, q, parentIndex) {
    tree.vertices.push({ vertex: q, edges: [], parent: parentIndex });
    tree.newest = tree.vertices.length - 1;
    return tree.newest;
}
// #endregion tree-add-vertex

// #region tree-add-edge
function treeAddEdge(tree, indexA, indexB) {
    tree.vertices[indexA].edges.push(indexB);
    tree.vertices[indexB].edges.push(indexA);
}
// #endregion tree-add-edge

// #region nearest-neighbor
// The tree vertex closest to q, by linear scan.
//
// Linear scan is O(n) per call and the tree grows every iteration, so this
// is quadratic in the number of vertices over a whole plan -- the second
// of the planner's two real costs, and the one a k-d tree fixes. It is
// kept linear here on purpose: it is six lines a student can read, the
// counter below measures exactly how much it costs, and in these worlds
// collision checking still dominates it by a wide margin. That inequality
// is asserted in Phase 4 rather than asserted in prose.
function nearestNeighbor(q, tree, weights, counters) {
    var bestIndex = 0;
    var bestDistance = Infinity;
    var j;
    for (j = 0; j < tree.vertices.length; j++) {
        counters.nearestNeighborComparisons++;
        var d = configurationDistance(q, tree.vertices[j].vertex, weights);
        if (d < bestDistance) {
            bestDistance = d;
            bestIndex = j;
        }
    }
    return { index: bestIndex, config: tree.vertices[bestIndex].vertex, distance: bestDistance };
}
// #endregion nearest-neighbor

//////////////////////////////////////////////////
/////   EXTEND AND CONNECT
//////////////////////////////////////////////////

// #region rrt-extend
// One step of the tree toward q. The three-valued return is the heart of
// the algorithm and is upstream's and the paper's vocabulary exactly:
//
//   'reached'  -- the new vertex IS q (within a step), so the tree has
//                 arrived at what it was aiming for
//   'advanced' -- a step was taken and added, but q is still further on
//   'trapped'  -- the step was blocked, nothing was added
//
// `connect` loops while this says 'advanced', which is the whole of the
// difference between RRT and RRT-Connect.
function rrtExtend(robot, qmap, world, tree, q, weights, step, counters, validation) {
    var near = nearestNeighbor(q, tree, weights, counters);
    var qNew = newConfig(near.config, q, weights, step);

    // validate the whole edge, not just where it lands
    if (!edgeIsValid(robot, qmap, world, near.config, qNew, weights, counters, validation)) {
        return 'trapped';
    }

    var newIndex = treeAddVertex(tree, qNew, near.index);
    treeAddEdge(tree, newIndex, near.index);

    // 'reached' means the new vertex IS q, not merely close to it.
    //
    // That distinction is load-bearing and was a real bug. The obvious
    // version -- "reached when q is within one step" -- leaves the final
    // sub-step gap between the new vertex and q UNVALIDATED, and in
    // RRT-Connect that gap is precisely the junction between the two
    // trees. So the planner would splice two halves together across an
    // edge nothing had ever collision-checked, and with a step size
    // worth about a metre of base motion, the robot walked through
    // walls. Verification found it by re-checking path edges at 300
    // samples each and turning up 173 colliding configurations on one.
    //
    // newConfig clamps at the target, so when the target is within a
    // step the new vertex equals it exactly and the edge validated just
    // above runs all the way to it. Requiring exact arrival therefore
    // costs at most one extra extension and makes the whole path
    // validated by construction.
    if (configurationDistance(qNew, q, weights) < 1e-9) return 'reached';
    return 'advanced';
}
// #endregion rrt-extend

// #region rrt-connect
// The greedy half of RRT-Connect: extend toward q over and over until
// something stops it.
//
// This is the "Connect heuristic" of Kuffner & LaValle 2000 and the reason
// the algorithm crosses a narrow passage at all. A uniform sampler almost
// never lands IN world_narrow_passage's gap; what happens instead is that
// one sample lands beyond it, and this loop drags the tree all the way
// through in a single iteration. Replace the loop with one extension and
// the planner becomes bidirectional RRT, which is markedly worse on
// exactly those worlds -- a one-line experiment worth running.
function rrtConnect(robot, qmap, world, tree, q, weights, step, counters, validation) {
    var result = 'advanced';
    while (result === 'advanced') {
        result = rrtExtend(robot, qmap, world, tree, q, weights, step, counters, validation);
    }
    return result;
}
// #endregion rrt-connect

//////////////////////////////////////////////////
/////   PATH EXTRACTION
//////////////////////////////////////////////////

// #region find-path
// The configurations from a tree's root to one of its vertices.
//
// Iterative, by walking parent pointers. Upstream does a recursive
// depth-first search instead, which is both asymptotically worse (it
// explores the tree to find a path it already knows) and bounded by the
// JavaScript call stack -- and whose visited-flag reset silently never
// runs, because it loops over `T.length` on an object that has no length
// (review findings K-6.1). A tree has exactly one path from root to any
// vertex; there is nothing to search for.
function findPath(tree, index) {
    var path = [];
    var i = index;
    while (i !== -1) {
        path.push(tree.vertices[i].vertex);
        i = tree.vertices[i].parent;
    }
    path.reverse(); // walked root-ward, wanted root-first
    return path;
}
// #endregion find-path

// #region smooth-path
// Shortcut smoothing: repeatedly pick two configurations on the path at
// random and, if the straight edge between them is collision-free, delete
// everything in between.
//
// RRT paths are visibly ragged -- the tree is grown by random extension,
// so the route wanders even where the space is wide open. Smoothing fixes
// the appearance and shortens the path, and it CANNOT make the path
// invalid, because every shortcut it accepts has been validated by the
// same edgeIsValid the planner uses. Phase 4 asserts both: the smoothed
// path passes every check the raw one does, and is no longer.
//
// What it does not do is make the path optimal. RRT-Connect is not
// asymptotically optimal and shortcutting a suboptimal path is a local
// improvement, not a global one -- it can only remove detours, never find
// a different homotopy class. The path through the left side of an
// obstacle stays on the left however long you smooth it. That distinction
// is worth a slide (review findings G-2).
//
// Candidate pairs come from the shared seeded generator, so smoothing is
// as reproducible as planning.
function smoothPath(robot, qmap, world, path, weights, counters, validation) {
    if (path.length < 3) return path.slice();

    var smoothed = path.slice();
    var attempt;
    for (attempt = 0; attempt < PLANNER_SMOOTHING_ATTEMPTS; attempt++) {
        if (smoothed.length < 3) break;

        var a = randomInteger(smoothed.length);
        var b = randomInteger(smoothed.length);
        if (a > b) { var t = a; a = b; b = t; }
        if (b - a < 2) continue; // nothing between them to remove

        if (edgeIsValid(robot, qmap, world, smoothed[a], smoothed[b], weights, counters, validation)) {
            smoothed.splice(a + 1, b - a - 1);
        }
    }
    return smoothed;
}
// #endregion smooth-path

//////////////////////////////////////////////////
/////   THE PLANNER
//////////////////////////////////////////////////

// #region zero-configuration
// The zero configuration at the world origin: base at (0,0,0) with no
// rotation, every joint at 0.
//
// THIS IS THE DEFAULT PLANNING GOAL, matching upstream
// (kineval_rrt_connect.js:128-131, `for (i=0;...) q_goal_config[i] = 0`)
// and matching the assignment the collision lecture states: "generate a
// collision free motion plan to the world origin and zero joint angle
// configuration".
//
// An earlier version of this file defaulted the goal to the active pose
// setpoint instead, reasoning that the zero configuration is sometimes
// itself in collision. That reasoning was right about the risk and wrong
// about the consequence: with no setpoint stored, the goal silently
// became the START configuration, so pressing `m` "solved" a zero-length
// problem in one iteration and the planner appeared never to run at all.
// A default that is occasionally unsolvable -- and says so -- is far
// better than one that is always trivially solved.
function zeroConfiguration(qmap) {
    var q = new Array(qmap.length);
    var i;
    for (i = 0; i < qmap.length; i++) q[i] = 0;
    return q;
}
// #endregion zero-configuration

// #region robot-rrt-planner-init
// Prepares a plan from the robot's current pose to the goal, and resets
// everything a run owns.
//
// GOAL CONVENTION. The goal is params.goal_config when one has been set
// -- by the "Set Goal" button, or by loading a pose-setpoint slot with
// [1-9] -- and otherwise the ZERO CONFIGURATION AT THE WORLD ORIGIN,
// which is upstream's own convention. See zeroConfiguration above.
//
// Re-seeds the generator, so planning twice from the same start, goal and
// seed replays the identical run. Phase 4 asserts it.
function robotRRTPlannerInit(robot, qmap, world, params) {
    seedRandom(params.seed);

    var qStart = configurationFromRobot(robot, qmap);
    var qGoal = (params.goal_config && params.goal_config.length === qmap.length)
        ? params.goal_config.slice()
        : zeroConfiguration(qmap);

    var weights = configurationWeights(robot, qmap, world);
    var counters = { collisionChecks: 0, nearestNeighborComparisons: 0 };

    // Edge-validation spacing, in metres. The feature that can be
    // tunnelled through is the smallest obstacle, so the spacing is a
    // fraction of its radius. An obstacle-free world has nothing to
    // tunnel through, and falls back to the robot's own scale so the
    // number is still finite and sane.
    var smallestObstacle = Infinity;
    var oi;
    for (oi = 0; oi < (world.obstacles || []).length; oi++) {
        smallestObstacle = Math.min(smallestObstacle, world.obstacles[oi].radius);
    }
    var extent = robotExtent(robot);
    if (!isFinite(smallestObstacle) || smallestObstacle <= 0) smallestObstacle = extent;

    var validation = {
        metres: PLANNER_EDGE_RESOLUTION * smallestObstacle,
        // each joint swings only what is downstream of it; levers.__base
        // is the whole robot, for base rotation. Derived from the
        // kinematics rather than from a pose, so the bound holds
        // everywhere along an edge -- see collision.js#jointLeverArms.
        levers: jointLeverArms(robot, qmap),
        // which joints carry each link, so the bound can be per-link
        // rather than a sum over the whole robot
        chains: jointChains(robot, qmap)
    };

    var planner = {
        qStart: qStart,
        qGoal: qGoal,
        weights: weights,
        validation: validation,
        step: plannerStepSize(qmap),
        counters: counters,
        iterations: 0,
        state: 'planning',
        // The outcome of the most recent EXTEND and CONNECT, in the
        // algorithm's own three-valued vocabulary. `state` says what the
        // whole run is doing (planning / found / failed); these say what
        // the last step did, which is what makes the search legible
        // while it is running rather than only after it stops.
        lastExtend: '-',
        lastConnect: '-',
        // T_a always holds the tree currently being EXTENDED. The two swap
        // every iteration, so this flag is what remembers which end of the
        // final path is the start.
        aIsStartTree: true,
        startedAt: Date.now(),
        elapsedMs: 0
    };

    // A goal identical to the start is not an error, but it is almost
    // always a mistake, and silently returning a two-configuration path
    // is exactly how this module once appeared not to be running at all.
    // Say so instead.
    if (configurationDistance(qStart, qGoal, weights) < 1e-9) {
        planner.state = 'failed';
        planner.failureReason = 'goal is identical to the start configuration -- drive the robot away (w/s/a/d) and plan again, or press Set Goal somewhere else';
        return planner;
    }

    // refuse to start from or aim at a configuration that is already in
    // collision. Both are ordinary user mistakes -- drive into a wall,
    // then press m -- and both otherwise present as "the planner never
    // finds anything", which is a much worse error message.
    counters.collisionChecks += 2;
    var startHit = poseIsCollision(robot, qmap, qStart, world);
    if (startHit) {
        planner.state = 'failed';
        planner.failureReason = 'start configuration is in collision (' +
            describeCollision(startHit) + ')';
        return planner;
    }
    var goalHit = poseIsCollision(robot, qmap, qGoal, world);
    if (goalHit) {
        planner.state = 'failed';
        planner.failureReason = 'goal configuration is in collision (' +
            describeCollision(goalHit) + ')';
        return planner;
    }

    planner.T_a = treeInit(qStart);
    planner.T_b = treeInit(qGoal);
    return planner;
}
// #endregion robot-rrt-planner-init

// #region describe-collision
// Turns poseIsCollision's verdict into something readable in the HUD.
function describeCollision(hit) {
    if (!hit) return 'none';
    if (hit.boundary) return 'link ' + hit.link + ' outside world boundary';
    return 'link ' + hit.link + ' vs obstacle ' + hit.obstacle;
}
// #endregion describe-collision

// #region robot-rrt-planner-iterate
// EXACTLY ONE iteration of RRT-Connect. Returns the planner's state.
//
// One iteration per call, with no internal loop, is a deliberate shape
// rather than an accident of style. It lets the caller spend a frame
// budget instead of a time budget, it makes the iteration counter
// meaningful in the HUD (which is upstream's own unaddressed "Display
// planning iteration number in UI" note), and it is what the deck's
// step-through panels reuse to advance a planner one click at a time.
function robotRRTPlannerIterate(robot, qmap, world, params, planner) {
    if (planner.state !== 'planning') return planner.state;

    if (planner.iterations >= PLANNER_MAX_ITERATIONS) {
        planner.state = 'failed';
        planner.failureReason = 'iteration cap (' + PLANNER_MAX_ITERATIONS + ') reached';
        planner.elapsedMs = Date.now() - planner.startedAt;
        return planner.state;
    }
    planner.iterations++;

    var weights = planner.weights;
    var step = planner.step;
    var counters = planner.counters;

    // 1. extend tree A one step toward a uniformly random configuration
    var qRand = randomConfig(robot, qmap, world, params, planner.qStart);
    var extendResult = rrtExtend(robot, qmap, world, planner.T_a, qRand, weights, step, counters, planner.validation);
    planner.lastExtend = extendResult;

    if (extendResult !== 'trapped') {
        // 2. greedily connect tree B toward A's newest vertex
        var qNew = planner.T_a.vertices[planner.T_a.newest].vertex;
        var connectResult = rrtConnect(robot, qmap, world, planner.T_b, qNew, weights, step, counters, planner.validation);
        planner.lastConnect = connectResult;

        if (connectResult === 'reached') {
            // the trees met: A's newest vertex and B's newest vertex are
            // within one step of each other, and that step has been
            // validated by the extension that produced it
            var pathA = findPath(planner.T_a, planner.T_a.newest);
            var pathB = findPath(planner.T_b, planner.T_b.newest);

            // pathA runs root->meeting point, pathB likewise. Whichever
            // tree is rooted at the start supplies the first half as-is;
            // the other supplies the second half reversed.
            var raw;
            if (planner.aIsStartTree) {
                raw = pathA.concat(pathB.slice().reverse());
            } else {
                raw = pathB.concat(pathA.slice().reverse());
            }

            planner.rawPath = raw;
            planner.path = smoothPath(robot, qmap, world, raw, weights, counters, planner.validation);
            planner.state = 'found';
            planner.elapsedMs = Date.now() - planner.startedAt;
            return planner.state;
        }
    }

    // 3. swap roles and repeat, so both trees grow. Swapping is what keeps
    // the two trees balanced; without it one tree does all the work and
    // the algorithm degenerates toward single-tree RRT.
    var temp = planner.T_a;
    planner.T_a = planner.T_b;
    planner.T_b = temp;
    planner.aIsStartTree = !planner.aIsStartTree;

    planner.elapsedMs = Date.now() - planner.startedAt;
    return planner.state;
}
// #endregion robot-rrt-planner-iterate

// #region plan-motion-rrt-connect
// The per-frame driver, called once from the main animation loop -- the
// counterpart to upstream's kineval.planMotionRRTConnect.
//
// Reads two one-shot request flags that userinput.js sets:
//   params.update_motion_plan           ('m')      start a new plan
//   params.update_motion_plan_traversal ('n'/'b')  show plan[index]
//
// Planning runs a bounded batch of iterations per frame so the browser
// stays responsive; traversal is pure index navigation over a finished
// plan and does no planning at all.
function planMotionRRTConnect(robot, qmap, world, params) {
    if (params.update_motion_plan) {
        params.update_motion_plan = false;
        params.planner = robotRRTPlannerInit(robot, qmap, world, params);
        params.motion_plan_traversal_index = 0;
    }

    var planner = params.planner;

    if (planner && planner.state === 'planning') {
        // at least one iteration per frame even on a machine slow enough
        // that the budget is already spent, so the planner always makes
        // progress rather than livelocking at zero iterations per frame
        var frameStart = Date.now();
        do {
            if (robotRRTPlannerIterate(robot, qmap, world, params, planner) !== 'planning') break;
        } while (Date.now() - frameStart < PLANNER_MS_PER_FRAME);
    }

    if (params.update_motion_plan_traversal) {
        params.update_motion_plan_traversal = false;
        if (planner && planner.state === 'found' && planner.path.length > 0) {
            applyConfigurationToRobot(robot, qmap, planner.path[params.motion_plan_traversal_index]);
        }
    }
}
// #endregion plan-motion-rrt-connect

// #region step-motion-plan-traversal
// Move one configuration forward (+1) or backward (-1) along a finished
// plan, clamped at both ends rather than wrapping.
//
// Clamped, not modular, on purpose: upstream's persist-traversal mode
// wraps with `% motion_plan.length`, which teleports the robot from the
// goal back to the start in one frame and looks like the plan is broken.
// Stepping with n/b should run out of plan at the end and stay there.
function stepMotionPlanTraversal(params, direction) {
    var planner = params.planner;
    if (!planner || planner.state !== 'found' || !planner.path.length) return;

    var index = (params.motion_plan_traversal_index || 0) + direction;
    index = Math.max(0, Math.min(planner.path.length - 1, index));
    params.motion_plan_traversal_index = index;
    params.update_motion_plan_traversal = true;
}
// #endregion step-motion-plan-traversal
