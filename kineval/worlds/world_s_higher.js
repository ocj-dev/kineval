/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | World: s_higher

    Ported from upstream worlds/world_s_higher.js, unchanged in geometry.

    world_s.js's two offset walls, each stacked two spheres high (y = 0.5
    and y = 1.5), plus four spheres just outside the boundary ends to stop
    a robot slipping around them. The extra height is what forces the arm
    upright rather than letting it swing through at waist level -- this is
    the first world in the set where the ARM's configuration, not just the
    base's, has to change for a plan to exist.

    Verified clean on the index-10 double-assignment the plan asked about:
    index 10 is assigned exactly once here.

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

// #region create-world-s-higher
function createWorldSHigher() {
    var wall1_x = 2;
    var wall2_x = 5;
    var obstacles = [];
    var z, y;
    var heights = [0.5, 1.5]; // two courses, so the walls are too tall to reach over

    for (y = 0; y < heights.length; y++) {
        // near wall, open at the far end
        for (z = 2; z >= -8; z--) {
            obstacles.push({ location: [[wall1_x], [heights[y]], [z], [1]], radius: 0.5 });
        }
        // far wall, open at the near end
        for (z = 0; z >= -10; z--) {
            obstacles.push({ location: [[wall2_x], [heights[y]], [z], [1]], radius: 0.5 });
        }
        // caps just past each wall's opening, so the route is through the
        // S rather than around the outside of it
        obstacles.push({ location: [[wall1_x], [heights[y]], [-10.5], [1]], radius: 0.5 });
        obstacles.push({ location: [[wall2_x], [heights[y]], [2.5], [1]], radius: 0.5 });
    }

    return { boundary: [[-1, 0, -10], [9, 0, 2]], obstacles: obstacles };
}
// #endregion create-world-s-higher
