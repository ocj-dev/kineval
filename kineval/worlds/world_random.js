/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | World: random

    Ported from upstream worlds/world_random.js, with four defects fixed
    and one deliberate behaviour change. 100 spheres scattered over a
    52x52 world -- by far the largest boundary in the set, and therefore
    the stress case for the two things that must scale with world extent:
    the sampler's range and the RRT step size. A planner with a hardcoded
    step size looks fine on world_basic and hangs here.

    UPSTREAM DEFECTS FIXED (see the review findings' section K-5.3):

    1. Lopsided sampling. Upstream writes
       `Math.random()*robot_boundary[0][0]`, which with
       robot_boundary[0][0] = -50 samples uniformly over [-50, 0] -- it
       multiplies by the minimum instead of interpolating between the
       minimum and the maximum, so the entire region from 0 out to +2 is
       never populated. Same bug in z. Fixed: sample across the full
       boundary in both axes.

    2. Spheres buried in the floor. Upstream's `(Math.random()-0.2)*3`
       puts centres at y in [-0.6, 2.4), so a sphere of radius up to 3.0
       can sit mostly or entirely below the ground plane.

    3. Unbounded radii. Upstream's `Math.random()*3.0` admits radii
       arbitrarily close to zero (invisible) and up to 3.0 (a sixth of the
       world across). Fixed: an explicit [r_min, r_max] range.

    4. Unseeded. Upstream calls Math.random(), so no two page loads agree
       and no screenshot can be reproduced. Fixed: draws from the shared
       seeded generator in ../random.js, so ?seed= reproduces a world
       exactly and the default seed is documented there.

    BEHAVIOUR CHANGE, not a bug fix, stated here because it is one:
    every sphere is placed at y = radius, so it RESTS ON the ground plane
    rather than floating above it or sinking into it. Upstream's y was
    independent of radius. A sphere buried in the floor is not a teaching
    obstacle -- it is invisible and it blocks the base for no visible
    reason -- and a floating one is not either. This makes every obstacle
    in the world both visible and relevant.

    NOT FIXED, and worth knowing: nothing here guarantees the robot's
    start pose is collision-free in the generated world. With 100 spheres
    the origin is sometimes occupied. The planner reports that honestly
    (robotIsCollision colours the offending link) rather than this file
    quietly clearing a disc around the origin, which would make the world
    less random than its name claims.

    See world_empty.js's header for the two notes that apply to all nine
    ported worlds.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// #region create-world-random
function createWorldRandom() {
    var boundary = [[-50, 0, -50], [2, 0, 2]];
    var count = 100;
    var radiusMin = 0.4;
    var radiusMax = 2.0;

    var obstacles = [];
    var i;
    for (i = 0; i < count; i++) {
        var radius = randomInRange(radiusMin, radiusMax);
        obstacles.push({
            location: [
                [randomInRange(boundary[0][0], boundary[1][0])],
                // centre at exactly one radius above the ground plane, so
                // the sphere is tangent to it -- see the header
                [radius],
                [randomInRange(boundary[0][2], boundary[1][2])],
                [1]
            ],
            radius: radius
        });
    }

    return { boundary: boundary, obstacles: obstacles };
}
// #endregion create-world-random
