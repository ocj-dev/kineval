/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | World: basic

    Ported from upstream worlds/world_basic.js, unchanged. Three
    r = 0.5 spheres in a 10x10 world -- the smallest world with anything to
    avoid, and the one to debug a new robot's bounding boxes against before
    trying a wall.

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

// #region create-world-basic
function createWorldBasic() {
    return {
        boundary: [[-5, 0, -5], [5, 0, 5]],
        obstacles: [
            { location: [[0], [0.5], [2], [1]], radius: 0.5 },
            { location: [[2], [0.5], [0], [1]], radius: 0.5 },
            { location: [[-2], [0.5], [0], [1]], radius: 0.5 }
        ]
    };
}
// #endregion create-world-basic
