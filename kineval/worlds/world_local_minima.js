/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | World: local_minima

    Ported from upstream worlds/world_local_minima.js, unchanged -- and it
    earns its keep. Three r = 1.0 spheres form a pocket around the origin
    that traps greedy best-first search and potential-field methods: every
    locally-downhill move leads further into the pocket. RRT-Connect walks
    out of it without noticing, because it never evaluates a gradient.

    This is the cleanest available motivation for sampling-based planning
    in the whole set, and the best world to run A* and RRT-Connect side by
    side on.

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

// #region create-world-local-minima
function createWorldLocalMinima() {
    return {
        boundary: [[-5, 0, -5], [5, 0, 5]],
        obstacles: [
            { location: [[0], [0.5], [-2], [1]], radius: 1.0 },
            { location: [[2], [0.5], [0], [1]], radius: 1.0 },
            { location: [[-2], [0.5], [0], [1]], radius: 1.0 }
        ]
    };
}
// #endregion create-world-local-minima
