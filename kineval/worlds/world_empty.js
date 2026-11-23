/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | World: empty

    Ported from the upstream kineval-stencil's worlds/world_empty.js. A
    bare 10x10 ground-plane world with no obstacles -- the control case.
    Every planner bug that is not about obstacles shows up here first, and
    a plan that fails in this world is broken in the sampler, the metric or
    the boundary test rather than in collision detection.

    PORT NOTE (applies to all nine ported worlds): upstream world files are
    straight-line scripts that assign two globals, `robot_boundary` and
    `robot_obstacles`. Here each is a factory returning plain data, so two
    worlds can exist at once, switching worlds cannot leave half the old
    one behind, and nothing in worlds/ touches three.js -- scene.js stays
    the only file that turns data into meshes.

    COMMENT FIX (applies to all nine): upstream's "collision only checked
    in x-z plane" is true of the BOUNDARY test and false of obstacle
    collision, which is fully 3D against each link's bounding box
    (collision.js#linkSphereCollision). world_y.js is built to prove it.
    The comment is corrected in every port.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// #region create-world-empty
function createWorldEmpty() {
    return {
        // [[xmin, y, zmin], [xmax, y, zmax]] -- an x-z ground-plane extent
        // with no ceiling; obstacles govern height, the boundary does not
        boundary: [[-5, 0, -5], [5, 0, 5]],
        obstacles: []
    };
}
// #endregion create-world-empty
