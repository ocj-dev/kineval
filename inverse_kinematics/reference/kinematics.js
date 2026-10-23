/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Forward Kinematics and Quaternions | Robot Init + Forward Kinematics

    COMPLETED REFERENCE IMPLEMENTATION for the AutoRob (autorob.org) lab-session
    slides. Completes every "STENCIL" section of three upstream kineval-stencil
    (github.com/autorob/kineval-stencil) modules:

      kineval_robot_init.js        -- initRobotLinks
      kineval_robot_init_joints.js -- initRobotJoints: cross-reference each
                                       link's parent/child JOINTS (the robot
                                       description only states each joint's
                                       parent/child LINK)
      kineval_forward_kinematics.js -- buildFKTransforms, via a depth-first
                                       traversal over the kinematic tree
                                       (traverseFKBase -> traverseFKLink ->
                                       traverseFKJoint -> traverseFKLink -> ...)
                                       that composes a matrix stack: each
                                       joint's FIXED origin offset (xyz/rpy,
                                       exactly as authored in the robot
                                       description) is applied as ordinary
                                       4x4 rotation/translation matrices from
                                       matrix.js, then each joint's VARIABLE
                                       one-DOF motion (the thing that actually
                                       changes as the robot moves) is applied
                                       as a quaternion built from its rotation
                                       axis and current angle, converted to a
                                       matrix, and folded into the same stack
                                       -- i.e. the matrix stack and
                                       quaternions are not two competing
                                       techniques here, but used together:
                                       matrices for the tree structure,
                                       quaternions for each joint's own motion.

    A robot description (see robots/mobile_arm.urdf.js) is a plain JS object:
        robot = {
          name: "...",
          origin: {xyz:[x,y,z], rpy:[roll,pitch,yaw]},  // world -> base
          base: "link_name",
          links: { linkName: {}, ... },
          joints: {
            jointName: {
              parent: "linkName", child: "linkName",
              type: "revolute" | "continuous" | "prismatic" | "fixed",
              origin: {xyz:[...], rpy:[...]},  // parent link -> joint frame
              axis: [x,y,z],
              angle: 0  // the one variable DOF value, in radians (revolute/
                        // continuous) or meters (prismatic); unused if fixed
            }, ...
          }
        }
    This is KinEval's convenience JSON representation of the URDF <visual>
    tree (see the deck's "URDF <visual> <-> KinEval JSON" slide) -- geometry
    itself lives in a parallel links_geom[name] map of threejs geometry,
    built alongside robot.links in the same robot description file.

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
// #region init-robot-links
function initRobotLinks(robot) {
    var x;
    for (x in robot.links) {
        robot.links[x].name = x;
        robot.links[x].children = [];  // joint names for which this link is the parent
        robot.links[x].parent = null;  // joint name for which this link is the child (base link has none)
    }
    robot.control = { xyz: [0, 0, 0], rpy: [0, 0, 0] };
}
// #endregion init-robot-links

// #region init-robot-joints
function initRobotJoints(robot) {
    // build the kinematic hierarchy by looping over every joint and
    // cross-referencing it onto its parent/child links, since the robot
    // description only states each JOINT's parent/child link, not the
    // reverse: a link doesn't yet know which joint(s) hang off it, or which
    // joint connects it to its own parent. Both directions are needed by
    // the DFS traversal below (traverseFKLink walks robot.links[x].children
    // to find the next joints to descend into).
    var x;
    for (x in robot.joints) {
        robot.joints[x].name = x;
        if (robot.joints[x].angle === undefined) robot.joints[x].angle = 0;
        if (robot.joints[x].type === undefined) robot.joints[x].type = 'revolute';

        var parentLink = robot.links[robot.joints[x].parent];
        var childLink = robot.links[robot.joints[x].child];

        parentLink.children.push(x);
        childLink.parent = x;
    }
}
// #endregion init-robot-joints

// #region init-robot
function initRobot(robot) {
    initRobotLinks(robot);
    initRobotJoints(robot);
}
// #endregion init-robot
// #endregion robot-init

// #region forward-kinematics
// #region matrix-from-rpy
function matrix_from_rpy(rpy) {
    // fixed-axis (extrinsic) roll-pitch-yaw composition, ROS/URDF convention:
    // R = Rz(yaw) * Ry(pitch) * Rx(roll). This is the one place <origin
    // rpy="..."> values (always constant, authored once in the robot
    // description) become a matrix; a joint's own VARIABLE motion is never
    // expressed this way -- see traverseFKJoint below.
    return matrix_multiply(
        generate_rotation_matrix_Z(rpy[2]),
        matrix_multiply(generate_rotation_matrix_Y(rpy[1]), generate_rotation_matrix_X(rpy[0]))
    );
}
// #endregion matrix-from-rpy

// #region matrix-from-origin
function matrix_from_origin(origin) {
    // a URDF-style <origin xyz="..." rpy="..."/> as a single 4x4 matrix
    return matrix_multiply(generate_translation_matrix(origin.xyz[0], origin.xyz[1], origin.xyz[2]), matrix_from_rpy(origin.rpy));
}
// #endregion matrix-from-origin

// #region traverse-fk-joint
function traverseFKJoint(robot, jointName, parentMatrix) {
    var joint = robot.joints[jointName];

    // 1. the joint's fixed origin offset from its parent link, as authored
    //    in the robot description -- ordinary matrix-stack composition
    var jointOrigin = matrix_multiply(parentMatrix, matrix_from_origin(joint.origin));

    // 2. the joint's own variable one-DOF motion, built as a quaternion from
    //    its rotation axis and current angle, then converted to a matrix and
    //    folded into the same stack (prismatic joints translate along the
    //    axis instead; fixed joints contribute no motion at all)
    var jointMatrix;
    if (joint.type === 'prismatic') {
        var unitAxis = vector_normalize(joint.axis);
        jointMatrix = matrix_multiply(jointOrigin, generate_translation_matrix(
            unitAxis[0] * joint.angle, unitAxis[1] * joint.angle, unitAxis[2] * joint.angle
        ));
    } else if (joint.type === 'fixed') {
        jointMatrix = jointOrigin;
    } else { // 'revolute' or 'continuous'
        var q = quaternion_normalize(quaternion_from_axisangle(joint.axis, joint.angle));
        jointMatrix = matrix_multiply(jointOrigin, quaternion_to_rotation_matrix(q));
    }

    joint.matrix = jointMatrix;
    traverseFKLink(robot, joint.child, jointMatrix);
}
// #endregion traverse-fk-joint

// #region traverse-fk-link
function traverseFKLink(robot, linkName, matrix) {
    var link = robot.links[linkName];
    link.matrix = matrix;

    var i;
    for (i = 0; i < link.children.length; i++) {
        traverseFKJoint(robot, link.children[i], matrix);
    }
}
// #endregion traverse-fk-link

// #region traverse-fk-base
function traverseFKBase(robot) {
    var baseOrigin = matrix_from_origin(robot.origin);

    // heading (local +z) and lateral (local +x) of the robot base, as 4x1
    // homogeneous POINTS one unit ahead of / beside the base in world
    // coordinates (w=1, so the base's own translation IS included). The
    // upstream stencil comment calls these "directions", but its own
    // consumer -- kineval_userinput.js's w/s/q/e base driving -- reads them
    // as `heading[2][0] - robot.origin.xyz[2]`, subtracting the base
    // position back off to recover the direction. That subtraction only
    // yields a heading if these are points: with w=0 the base's position
    // leaks into the result and "forward" stops pointing forward as soon
    // as the robot drives away from the world origin.
    robot.heading = matrix_multiply(baseOrigin, [[0], [0], [1], [1]]);
    robot.lateral = matrix_multiply(baseOrigin, [[1], [0], [0], [1]]);

    traverseFKLink(robot, robot.base, baseOrigin);
}
// #endregion traverse-fk-base

// #region build-fk-transforms
function buildFKTransforms(robot) {
    traverseFKBase(robot);
}
// #endregion build-fk-transforms

function robotForwardKinematics(robot) {
    if (typeof buildFKTransforms === 'undefined') {
        return;
    }
    buildFKTransforms(robot);
}
// #endregion forward-kinematics
