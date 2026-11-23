/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | World: s_higher_tighter

    Ported from upstream worlds/world_s_higher_tighter.js.

    world_s_higher.js with the walls moved closer together (x = 1.5 and
    x = 4 rather than 2 and 5), the boundary pulled in to match, and four
    extra cap spheres at each opening -- the hardest of the nine ported
    worlds, and the one where the step size and the edge-validation
    granularity start to matter.

    DEFECT FIXED: upstream assigns robot_boundary twice, at line 5 and
    again at line 9 in terms of wall2_x. The first assignment is dead. The
    second is kept here and the first is deleted. See the review findings'
    section K-5.3.

    Verified clean on the index-10 double-assignment: assigned once.

    See world_empty.js's header for the two notes that apply to all nine
    ported worlds: the factory-instead-of-globals port, and the corrected
    "collision only checked in x-z plane" comment.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// #region create-world-s-higher-tighter
function createWorldSHigherTighter() {
    var wall1_x = 1.5;
    var wall2_x = 4;
    var obstacles = [];
    var z, y;
    var heights = [0.5, 1.5];

    for (y = 0; y < heights.length; y++) {
        for (z = 2; z >= -8; z--) {
            obstacles.push({ location: [[wall1_x], [heights[y]], [z], [1]], radius: 0.5 });
        }
        for (z = 0; z >= -10; z--) {
            obstacles.push({ location: [[wall2_x], [heights[y]], [z], [1]], radius: 0.5 });
        }
        // three cap spheres past each opening rather than one, which is
        // what makes this world "tighter" than world_s_higher
        obstacles.push({ location: [[wall1_x], [heights[y]], [-10.5], [1]], radius: 0.5 });
        obstacles.push({ location: [[wall1_x + 1], [heights[y]], [-10.5], [1]], radius: 0.5 });
        obstacles.push({ location: [[wall1_x + 2], [heights[y]], [-10.5], [1]], radius: 0.5 });
        obstacles.push({ location: [[wall2_x], [heights[y]], [2.5], [1]], radius: 0.5 });
        obstacles.push({ location: [[wall2_x - 1], [heights[y]], [2.5], [1]], radius: 0.5 });
        obstacles.push({ location: [[wall2_x - 2], [heights[y]], [2.5], [1]], radius: 0.5 });
    }

    // upstream's second, live assignment -- the dead one at its line 5 is
    // deliberately not carried over
    return { boundary: [[-0.1, 0, -10], [wall2_x + 1.6, 0, 2]], obstacles: obstacles };
}
// #endregion create-world-s-higher-tighter
