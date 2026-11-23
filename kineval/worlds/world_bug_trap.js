/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | World: bug_trap (generated)

    The classic motivation for bidirectional search. A three-sided pocket
    opening away from the goal, with the start placed inside it.

    A single tree rooted at the start spends almost all of its samples
    inside the pocket, because that is where its vertices are and the
    nearest neighbour of most samples is a vertex on the pocket wall. It
    escapes only by sampling the narrow mouth, which is rare. A tree
    rooted at the GOAL has no such problem -- it is outside, in open
    space, and grows toward the mouth freely. RRT-Connect's second tree is
    what converts a hard problem into an easy one here, and running the
    same world with rrt_alg set to single-tree is the demonstration.

    The trap's mouth is sized to the robot (1.8 robot-widths) so it stays
    passable but not inviting whichever robot is loaded.

    See world_narrow_passage.js's header for what the five parametric
    generators have in common: sizes derived from the robot's own extent,
    randomness from ../random.js, and the same (context) argument.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// #region create-world-bug-trap
function createWorldBugTrap(context) {
    var extent = context.extent;
    // the pocket is sized by the robot's whole reach, so the robot fits
    // inside it; the mouth is sized by its widest link, so squeezing out
    // is what is being tested. See collision.js#robotReach vs robotExtent.
    var reach = context.reach;
    var half = Math.max(6, reach * 6);
    var boundary = [[-half, 0, -half], [half, 0, half]];

    var radius = extent * 0.45;
    var spacing = radius * 1.4;
    var trapHalf = reach * 1.6;           // the pocket's half-width
    var mouthHalf = extent * 1.1;         // the opening, facing +z
    // Walls are STACKED to the robot's own height -- one course of
    // spheres is not a wall, because obstacle collision is fully 3D and a
    // tall robot would simply step over a knee-high trap. See
    // collision.js#robotHeight.
    var wallTop = context.height * 1.4 + radius;
    var obstacles = [];
    var t, y;

    for (y = radius; y <= wallTop; y += radius * 1.4) {
        // back wall of the pocket, at -z, solid
        for (t = -trapHalf; t <= trapHalf; t += spacing) {
            obstacles.push({ location: [[t], [y], [-trapHalf], [1]], radius: radius });
        }
        // the two side walls, running from the back wall toward the opening
        for (t = -trapHalf; t <= trapHalf; t += spacing) {
            obstacles.push({ location: [[-trapHalf], [y], [t], [1]], radius: radius });
            obstacles.push({ location: [[trapHalf], [y], [t], [1]], radius: radius });
        }
        // the front wall is mostly closed: only `mouthHalf` either side of
        // centre is open, so the pocket is a trap rather than a corridor
        for (t = -trapHalf; t <= trapHalf; t += spacing) {
            if (Math.abs(t) < mouthHalf) continue;
            obstacles.push({ location: [[t], [y], [trapHalf], [1]], radius: radius });
        }
    }

    return { boundary: boundary, obstacles: obstacles };
}
// #endregion create-world-bug-trap
