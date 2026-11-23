/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | Seeded Pseudorandom Numbers

    One seeded generator, shared by every stochastic choice in the motion
    planning module: the RRT sampler (motion_plan.js#randomConfig), the
    shortcut smoother's candidate-pair selection, and the world generators
    (worlds/world_random.js and the five parametric worlds). Nothing in
    this module calls Math.random().

    This is not a stylistic preference. Two claims the module makes depend
    on it. "Plan twice from the same start, goal and seed and you get the
    same path" is a Phase 4 assertion, and the presentation deck's
    animations are built on replaying a specific run. Math.random() cannot
    be seeded in JavaScript, so both would be false with it.

    The algorithm is mulberry32: a 32-bit state, three multiply-xor-shift
    rounds, no dependency, and a period of 2^32 -- far more than a planner
    run needs, and short enough to state honestly rather than imply
    cryptographic quality. It is NOT suitable for anything but simulation.

    Upstream's kineval_rrt_connect.js#random_config calls Math.random()
    directly; this file is a deliberate divergence, recorded in
    IMPLEMENTATION_NOTES.md.

    THIS FILE EXISTS TWICE, BYTE FOR BYTE IDENTICALLY, at
    `kineval/random.js` and `motion_planning/reference/random.js`. Each
    module's reference/ directory has to be self-contained -- Slidev's
    `<<<` snippet importer cannot read outside the deck's workspace root,
    which `motion_planning/pnpm-workspace.yaml` pins at `motion_planning/`
    -- and the viewer cannot import from a module's reference/ either. A
    `diff` between the two is the cheap check, and Phase 4 runs it; edit
    one and copy, never edit both.

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

// the seed applied when ?seed= is absent -- documented rather than
// arbitrary, so a screenshot taken with no query string is reproducible
var RANDOM_DEFAULT_SEED = 20261122;

// module-level state, deliberately shared: every caller draws from the same
// stream, so "the seed" means one thing for a whole page load
var random_state = 0;
var random_seed_value = RANDOM_DEFAULT_SEED;

// #region seed-random
// Seeds the shared generator. Called at world/robot load and again at the
// start of every robotRRTPlannerInit, so re-planning from an unchanged
// start and goal replays the identical sequence of samples.
//
// The seed is hashed rather than used as the state directly: mulberry32
// started at a small integer spends its first few outputs in a visibly
// correlated corner of the range, and seeds like 0, 1, 2 are exactly what
// a person types. The hash is a single round of the 32-bit integer
// finalizer from MurmurHash3, which decorrelates them.
function seedRandom(seed) {
    var s = (typeof seed === 'number' && isFinite(seed)) ? Math.floor(seed) : RANDOM_DEFAULT_SEED;
    random_seed_value = s;

    var h = s >>> 0;
    h = Math.imul(h ^ (h >>> 16), 0x85ebca6b);
    h = Math.imul(h ^ (h >>> 13), 0xc2b2ae35);
    h = (h ^ (h >>> 16)) >>> 0;

    random_state = h;
    return random_seed_value;
}
// #endregion seed-random

// #region random-seed-value
// The seed currently in effect, so the HUD can echo it and a screenshot
// carries the information needed to reproduce itself.
function randomSeedValue() {
    return random_seed_value;
}
// #endregion random-seed-value

// #region random
// One draw, uniform on [0, 1). Same contract as Math.random().
function random() {
    // advance the state by an odd increment (the golden-ratio constant), so
    // the state sequence itself is a full-period Weyl sequence and the
    // generator can never get stuck on a fixed point
    random_state = (random_state + 0x6d2b79f5) >>> 0;

    // then scramble the state into the output: two multiply-xor-shift
    // rounds mix the high bits down into the low ones, which is what makes
    // consecutive outputs independent rather than merely increasing
    var t = random_state;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);

    // >>> 0 reads the result as unsigned before scaling, and 4294967296 is
    // 2^32, so the result lands in [0, 1) and never reaches exactly 1
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
}
// #endregion random

// #region random-in-range
// Uniform on [low, high). Used wherever a sampler needs an interval rather
// than a unit draw -- joint limits, world boundaries, obstacle radii.
function randomInRange(low, high) {
    return low + (high - low) * random();
}
// #endregion random-in-range

// #region random-integer
// Uniform integer on [0, count), for picking an index -- the shortcut
// smoother's two candidate path vertices, for instance.
function randomInteger(count) {
    return Math.floor(random() * count);
}
// #endregion random-integer

// seed once at load, so a caller that forgets to seed still gets a
// deterministic stream rather than an uninitialized one
seedRandom(RANDOM_DEFAULT_SEED);
