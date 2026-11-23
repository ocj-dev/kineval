/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | World: y

    Ported from upstream worlds/world_y.js, unchanged.

    THE ONE GENUINELY 3D WORLD. Two r = 2.0 spheres at y = 1.0 sit at
    x = +-3 and span y in [-1, 3], leaving a gap only at |x| < 1. A third
    r = 2.0 sphere at y = 5.0 sits directly over that gap, spanning
    y in [3, 7]. So the world is an overhead gate at z = -10: a low robot
    drives straight through, and a tall one -- or one with its arm raised --
    does not.

    This is the world that disproves every upstream world file's
    "collision only checked in x-z plane" comment. Obstacle collision is
    fully 3D (collision.js#linkSphereCollision tests all three axes); only
    the boundary test is planar. Screenshot a traversal here.

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

// #region create-world-y
function createWorldY() {
    return {
        boundary: [[-5, 0, -20], [5, 0, 5]],
        obstacles: [
            // the overhead bar of the gate, clear of the ground
            { location: [[0], [5.0], [-10], [1]], radius: 2.0 },
            // its two posts, leaving a gap at |x| < 1
            { location: [[3], [1.0], [-10], [1]], radius: 2.0 },
            { location: [[-3], [1.0], [-10], [1]], radius: 2.0 }
        ]
    };
}
// #endregion create-world-y
