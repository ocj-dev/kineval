/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | World: narrow_passage (generated)

    Not an upstream world. One of five PARAMETRIC generators: the feature
    sizes are computed from the robot's own extent rather than written
    down, so the world stays meaningful when the robot changes. Load
    narrow_passage with the crawler and with fetch and the gap is a
    different number of metres in each, but the same multiple of the
    robot.

    A single wall across the world with one gap in it, where the gap is a
    small multiple of the robot's widest link dimension
    (collision.js#robotExtent). This is the world that exercises the
    GREEDINESS of RRT-Connect's connect step. A uniform sampler lands in
    the gap with probability proportional to the gap's measure, which is
    tiny -- so the planner does not find the passage by sampling into it.
    It finds it because `connect` extends repeatedly from the nearest
    neighbour until it is blocked, so a single sample on the far side
    drags the tree all the way through the gap in one iteration.

    Turn the gap multiplier down and watch the iteration count climb
    nonlinearly; that curve is the narrow-passage problem, and it is the
    reason PRM and RRT research spent a decade on sampling strategies.

    All five generators take the same (context) argument -- {extent,
    reach} -- and draw any randomness from ../random.js, so ?seed=
    reproduces them exactly. The two scales are not interchangeable:
    `extent` is the widest single link, which is what has to fit through a
    gap, and `reach` is the whole robot's ground-plane span from its base,
    which is what an enclosure or a pocket has to clear. Sizing an
    enclosure by `extent` builds one the robot is already wedged into.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// #region create-world-narrow-passage
// context: { extent, reach } -- the robot's widest single link, and its
//   whole-assembly ground-plane reach (collision.js#robotExtent / #robotReach)
function createWorldNarrowPassage(context) {
    var extent = context.extent;
    var reach = context.reach;

    // size the world to the robot rather than fixing it, using the whole
    // robot's reach so there is open space either side of the wall for a
    // start and a goal. The GAP, below, is sized by the widest single
    // link instead -- that is what has to fit through.
    // See collision.js#robotReach vs robotExtent.
    var half = Math.max(5, reach * 5);
    var boundary = [[-half, 0, -half], [half, 0, half]];

    // the gap is 3 robot-widths across: wide enough that a plan certainly
    // exists, narrow enough that uniform sampling rarely lands in it
    var gapHalfWidth = Math.max(extent * 1.5, reach * 0.75);

    // spheres sized so consecutive centres overlap, leaving no seam a
    // thin link could slip through between them
    var radius = extent * 0.5;
    var spacing = radius * 1.4;

    // The wall is STACKED to the robot's own height. One course of
    // spheres is not a wall: obstacle collision is fully 3D, so a robot
    // whose links all sit above a knee-high row simply walks over it and
    // the passage stops being narrow. Courses are spaced so consecutive
    // spheres overlap vertically too, leaving no seam.
    // (See collision.js#robotHeight, and world_s_higher.js, where
    // upstream hit the same thing and hand-fixed it for one robot.)
    var wallTop = context.height * 1.4 + radius;
    var obstacles = [];
    var x, y;
    for (y = radius; y <= wallTop; y += radius * 1.4) {
        for (x = -half; x <= half; x += spacing) {
            if (Math.abs(x) < gapHalfWidth) continue; // the passage itself
            obstacles.push({ location: [[x], [y], [0], [1]], radius: radius });
        }
    }

    return { boundary: boundary, obstacles: obstacles };
}
// #endregion create-world-narrow-passage
