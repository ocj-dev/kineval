/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | World: s

    Ported from upstream worlds/world_s.js. Two offset walls forming an
    S-shaped corridor: a wall at x = -2 open at its far (-z) end, and a
    wall at x = -6 open at its near (+z) end, so the only route doubles
    back on itself.

    DEFECT CLAIM PARTLY RETRACTED. The implementation plan asserted that
    line 20's robot_obstacles[10] is overwritten at line 22, leaving a gap
    in the S-wall. The overwrite is real, but line 34 re-adds the
    displaced sphere at index 21, so the net obstacle set is complete and
    there is no gap. What was actually wrong was readability: an index
    collision repaired fourteen lines later at a non-obvious index. The
    loop below assigns each sphere once, which produces the identical
    geometry. See the review findings' section K-5.1.

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

// #region create-world-s
function createWorldS() {
    var obstacles = [];
    var z;
    // near wall at x = -2, z from 2 down to -8 -- open at the far end
    for (z = 2; z >= -8; z--) {
        obstacles.push({ location: [[-2], [0.5], [z], [1]], radius: 0.5 });
    }
    // far wall at x = -6, z from 0 down to -10 -- open at the near end,
    // so the two openings are at opposite ends and the path is an S
    for (z = 0; z >= -10; z--) {
        obstacles.push({ location: [[-6], [0.5], [z], [1]], radius: 0.5 });
    }
    return { boundary: [[-10, 0, -10], [2, 0, 2]], obstacles: obstacles };
}
// #endregion create-world-s
