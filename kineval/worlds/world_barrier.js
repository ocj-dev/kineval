/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | World: barrier

    Ported from upstream worlds/world_barrier.js, unchanged. A single
    wall of eleven r = 0.5 spheres at x = -5 spanning z in [-8, 2], inside
    a 12x12 boundary -- the wall stops short of z = -10, so there is a way
    around its far end and a plan always exists.

    DEFECT CLAIM RETRACTED. The implementation plan asserted this file
    overwrites robot_obstacles[10] at line 23 and silently drops a sphere.
    It does not: the block comment opens at line 22, BEFORE line 23, so
    the entire second wall (indices 10-20 at x = -6) is commented out and
    line 20's assignment to index 10 is the only live one. The file is
    clean and is carried over as-is. See the review findings' section
    K-5.2; the commented-out second wall is omitted here rather than
    carried as dead code.

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

// #region create-world-barrier
function createWorldBarrier() {
    var obstacles = [];
    var z;
    // one wall at x = -5, from z = 2 down to z = -8; the gap at the far
    // end (z < -8) is the only way through
    for (z = 2; z >= -8; z--) {
        obstacles.push({ location: [[-5], [0.5], [z], [1]], radius: 0.5 });
    }
    return { boundary: [[-10, 0, -10], [2, 0, 2]], obstacles: obstacles };
}
// #endregion create-world-barrier
