/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | Robot Init + Forward Kinematics

    The same completed forward-kinematics/quaternion STENCIL work as
    ../forward_kinematics/reference/kinematics.js -- robot init (building
    each link's parent/child joint cross-references) and the matrix-stack +
    quaternion DFS traversal (traverseFKBase -> traverseFKLink ->
    traverseFKJoint) -- reused here rather than re-derived, since this
    viewer depends on that module's completed work (see ../README.md's
    build-order table).

    One naming difference from that module: each link/joint's fully-composed
    world transform is stored as `.xform` here, not `.matrix` -- matching the
    upstream kineval-stencil's own kineval_threejs.js (robot.links[x].xform),
    since scene.js in this directory ports that file's rendering approach
    directly. Every joint here is always revolute/continuous (quaternion
    axis-angle) -- none of this viewer's three ported test robots use a
    prismatic or fixed joint, matching upstream's own kineval_forward_kinematics.js
    STENCIL, which doesn't address joint type at all.

    This file (like every other reference implementation in this project)
    keeps every function as a plain top-level function rather than adopting
    upstream's "kineval = {}" namespace-object convention -- consistent with
    this project's own house style (see pathfinding/, simulation/,
    forward_kinematics/), not a claim that the namespace itself was ported.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// #region robot-init
function initRobotLinks(robot) {
    var x;
    for (x in robot.links) {
        robot.links[x].name = x;
        robot.links[x].children = [];
        robot.links[x].parent = null;
    }
    robot.control = { xyz: [0, 0, 0], rpy: [0, 0, 0] };
}

function initRobotJoints(robot) {
    var x;
    for (x in robot.joints) {
        robot.joints[x].name = x;
        if (robot.joints[x].angle === undefined) robot.joints[x].angle = 0;
        if (robot.joints[x].control === undefined) robot.joints[x].control = 0;

        var parentLink = robot.links[robot.joints[x].parent];
        var childLink = robot.links[robot.joints[x].child];

        parentLink.children.push(x);
        childLink.parent = x;
    }
}

function initRobot(robot) {
    initRobotLinks(robot);
    initRobotJoints(robot);
}
// #endregion robot-init

// #region forward-kinematics
function matrix_from_rpy(rpy) {
    return matrix_multiply(
        generate_rotation_matrix_Z(rpy[2]),
        matrix_multiply(generate_rotation_matrix_Y(rpy[1]), generate_rotation_matrix_X(rpy[0]))
    );
}

function matrix_from_origin(origin) {
    return matrix_multiply(generate_translation_matrix(origin.xyz[0], origin.xyz[1], origin.xyz[2]), matrix_from_rpy(origin.rpy));
}

function traverseFKJoint(robot, jointName, parentXform) {
    var joint = robot.joints[jointName];

    var jointOrigin = matrix_multiply(parentXform, matrix_from_origin(joint.origin));

    var q = quaternion_normalize(quaternion_from_axisangle(joint.axis, joint.angle));
    var jointXform = matrix_multiply(jointOrigin, quaternion_to_rotation_matrix(q));

    joint.xform = jointXform;
    traverseFKLink(robot, joint.child, jointXform);
}

function traverseFKLink(robot, linkName, xform) {
    var link = robot.links[linkName];
    link.xform = xform;

    var i;
    for (i = 0; i < link.children.length; i++) {
        traverseFKJoint(robot, link.children[i], xform);
    }
}

function traverseFKBase(robot) {
    var baseOrigin = matrix_from_origin(robot.origin);

    robot.heading = matrix_multiply(baseOrigin, [[0], [0], [1], [0]]);
    robot.lateral = matrix_multiply(baseOrigin, [[1], [0], [0], [0]]);

    traverseFKLink(robot, robot.base, baseOrigin);
}

function buildFKTransforms(robot) {
    traverseFKBase(robot);
}

function robotForwardKinematics(robot) {
    buildFKTransforms(robot);
}
// #endregion forward-kinematics
