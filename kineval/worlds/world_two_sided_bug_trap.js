/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | World: two_sided_bug_trap (generated)

    An honest "limits of the algorithm" case: two bug traps facing each
    other, with the start inside one and the goal inside the other.

    Bidirectionality is no longer a rescue, because BOTH trees are now
    trapped. Each one spends its samples inside its own pocket, and the
    two mouths have to be found independently before connect can bridge
    them. RRT-Connect still terminates -- and often still succeeds -- but
    the iteration count is an order of magnitude higher than
    world_bug_trap's and the variance across seeds is large. That variance
    is the lesson: a probabilistically complete planner gives no bound on
    how long it takes.

    REQUIREMENT, enforced by Phase 4: this world must still TERMINATE
    under the iteration cap, reporting failure cleanly rather than
    hanging. A planner that hangs here has no iteration cap, which is a
    bug the easy worlds cannot reveal.

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

// #region create-world-two-sided-bug-trap
function createWorldTwoSidedBugTrap(context) {
    var extent = context.extent;
    // as world_bug_trap: pocket by reach, mouth by widest link
    var reach = context.reach;
    var half = Math.max(9, reach * 9);
    var boundary = [[-half, 0, -half], [half, 0, half]];

    var radius = extent * 0.45;
    var spacing = radius * 1.4;
    var trapHalf = reach * 1.5;
    var mouthHalf = extent * 1.1;
    var offset = reach * 3.2;    // how far apart the two pockets sit, in z

    // stacked to the robot's height, as world_bug_trap.js
    var wallTop = context.height * 1.4 + radius;
    var obstacles = [];

    // one pocket, built around (0, centreZ), opening toward `facing`
    // (+1 or -1 in z). Called twice, mirrored, which is the whole world.
    function addTrap(centreZ, facing) {
        var t, y;
        for (y = radius; y <= wallTop; y += radius * 1.4) {
            // closed back
            for (t = -trapHalf; t <= trapHalf; t += spacing) {
                obstacles.push({ location: [[t], [y], [centreZ - facing * trapHalf], [1]], radius: radius });
            }
            // sides
            for (t = -trapHalf; t <= trapHalf; t += spacing) {
                obstacles.push({ location: [[-trapHalf], [y], [centreZ + t], [1]], radius: radius });
                obstacles.push({ location: [[trapHalf], [y], [centreZ + t], [1]], radius: radius });
            }
            // front, with the mouth
            for (t = -trapHalf; t <= trapHalf; t += spacing) {
                if (Math.abs(t) < mouthHalf) continue;
                obstacles.push({ location: [[t], [y], [centreZ + facing * trapHalf], [1]], radius: radius });
            }
        }
    }

    addTrap(-offset, 1);   // opens toward +z
    addTrap(offset, -1);   // opens toward -z, facing the first

    return { boundary: boundary, obstacles: obstacles };
}
// #endregion create-world-two-sided-bug-trap
