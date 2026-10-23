/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | Inverse Kinematics

    The upstream kineval-stencil's own kineval_inverse_kinematics.js
    (kineval.robotInverseKinematics / kineval.iterateIK) is itself an
    unfinished STENCIL -- it defines the gate function but leaves the
    Jacobian-based iteration as "STENCIL: implement inverse kinematics
    iteration". This file is this project's own completed reference
    implementation of that iteration, following the same Jacobian-transpose
    / pseudoinverse manipulator-Jacobian approach the lab-session slides
    describe, ported into this viewer's plain-function house style (see
    kinematics.js's header for why this project doesn't adopt upstream's
    "kineval = {}" namespace convention).

    Jacobian construction: for every joint in the end-effector's arm chain
    (see #region arm-chain below), column i of the Jacobian is built from
    that joint's already-computed world .xform (set by traverseFKJoint in
    kinematics.js on the most recent forward-kinematics pass):
      - axis_world = rotation block of joint.xform times the joint's local
        .axis (its rotation/slide axis expressed in the joint's own frame)
      - p_joint = translation part of joint.xform (the joint's world position)
      - for a revolute/continuous joint, the linear part of the column is
        axis_world x (p_ee - p_joint) (angular velocity's effect on a point
        p_ee rigidly attached out along the chain), and the angular part
        (only used if orientation error is included in dx) is axis_world
        itself
      - for a prismatic joint (fetch's torso_lift_joint, the only one any
        ported robot's arm chain actually passes through), sliding along
        the axis contributes axis_world directly to linear velocity and
        nothing to angular velocity -- no cross product, since translating
        along a line doesn't rotate anything downstream

    One subtlety worth flagging: joint.xform already bakes in that joint's
    OWN variable-angle rotation (see traverseFKJoint), not just its ancestors'
    rotations, which looks at first like it would corrupt axis_world (the
    axis a joint rotates ABOUT should intuitively come from before that
    joint's own motion is applied). It doesn't: a rotation leaves its own
    axis fixed (R(axis, angle) * axis == axis for any angle), so
    ancestor_rotation * own_rotation * axis_local == ancestor_rotation *
    axis_local regardless of the current joint angle -- reading axis_world
    out of the post-rotation .xform gives exactly the same vector as reading
    it out of the pre-rotation jointOrigin would. Only ancestor joints'
    rotations actually matter.

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

// #region arm-chain
// General arm-chain derivation: walk backward from the end-effector's joint
// to the robot base via .parent links (not hardcoded per robot), then
// reverse so the chain reads base-to-tip. For mr2 this produces exactly
// ['clavicle_right_yaw','shoulder_right_yaw','upperarm_right_pitch',
// 'forearm_right_yaw'] -- NOT clavicle_left_roll, which is a disconnected
// sibling branch off the base link that this backward walk never visits,
// since it only ever follows the single parent link/joint pointer chain
// initRobotJoints (kinematics.js) already set up, never a child list.
function deriveArmChain(robot) {
    var chain = [];
    var jointName = robot.endeffector.frame;

    while (true) {
        chain.unshift(jointName);
        var parentLink = robot.joints[jointName].parent;
        if (parentLink === robot.base) break;
        // the joint whose child is parentLink is the one link.parent points to
        jointName = robot.links[parentLink].parent;
    }

    return chain;
}
// #endregion arm-chain

// #region jacobian
function jacobianColumnForJoint(joint, eeWorld, includeOrientation) {
    // rotation block of the joint's world xform (already includes this
    // joint's own variable-angle rotation -- see header comment for why
    // that's fine for axis_world)
    var xf = joint.xform;
    var axisLocal = vector_normalize(joint.axis);
    var axisWorld = [
        xf[0][0] * axisLocal[0] + xf[0][1] * axisLocal[1] + xf[0][2] * axisLocal[2],
        xf[1][0] * axisLocal[0] + xf[1][1] * axisLocal[1] + xf[1][2] * axisLocal[2],
        xf[2][0] * axisLocal[0] + xf[2][1] * axisLocal[1] + xf[2][2] * axisLocal[2]
    ];
    axisWorld = vector_normalize(axisWorld);

    var pJoint = [xf[0][3], xf[1][3], xf[2][3]];

    var linear, angular;
    if (joint.type === 'prismatic') {
        // sliding along the axis moves the end-effector directly along that
        // axis and rotates nothing downstream
        linear = axisWorld;
        angular = [0, 0, 0];
    } else {
        // revolute/continuous: a unit angular velocity about axis_world at
        // p_joint moves the end-effector at axis_world x (p_ee - p_joint)
        var armVector = [eeWorld[0] - pJoint[0], eeWorld[1] - pJoint[1], eeWorld[2] - pJoint[2]];
        linear = vector_cross(axisWorld, armVector);
        angular = axisWorld;
    }

    return includeOrientation
        ? [[linear[0]], [linear[1]], [linear[2]], [angular[0]], [angular[1]], [angular[2]]]
        : [[linear[0]], [linear[1]], [linear[2]]];
}

// builds the manipulator Jacobian (3xN, or 6xN if orientation rows are
// included) over the given arm chain, columns in base-to-tip order
function buildJacobian(robot, chain, eeWorld, includeOrientation) {
    var rows = includeOrientation ? 6 : 3;
    var J = [];
    var r, i;
    for (r = 0; r < rows; r++) J[r] = [];

    for (i = 0; i < chain.length; i++) {
        var col = jacobianColumnForJoint(robot.joints[chain[i]], eeWorld, includeOrientation);
        for (r = 0; r < rows; r++) J[r][i] = col[r][0];
    }
    return J;
}
// #endregion jacobian

// #region iterate-ik
// one Jacobian-based IK step: moves every joint in the arm chain's .control
// (not .angle directly -- same one-frame-of-indirection every other control
// input in this viewer goes through, see controls.js#applyControls) toward
// closing the gap between the end-effector's current world position and
// params.ik_target.xyz.
function iterateIK(robot, params) {
    if (!robot.ik_chain) robot.ik_chain = deriveArmChain(robot);
    var chain = robot.ik_chain;

    var eeJoint = robot.joints[robot.endeffector.frame];
    var eeWorldH = matrix_multiply(eeJoint.xform, robot.endeffector.position);
    var eeWorld = [eeWorldH[0][0], eeWorldH[1][0], eeWorldH[2][0]];

    var includeOrientation = !!params.ik_include_orientation;
    var target = params.ik_target.xyz;

    // error term: how far the end-effector needs to move to reach the target
    var dx = [[target[0] - eeWorld[0]], [target[1] - eeWorld[1]], [target[2] - eeWorld[2]]];

    var J = buildJacobian(robot, chain, eeWorld, includeOrientation);

    // [NOTICE]: exposed on the robot object, matching the upstream stencil's
    // own grading hooks (robot.dx / robot.jacobian / robot.dq), kept here as
    // a convenient place to inspect IK state from the console or tests.
    robot.dx = dx;
    robot.jacobian = J;

    // dq = T(J) * dx, where T(J) is either the transpose or the
    // Moore-Penrose pseudoinverse (matrix_pseudoinverse, matrix.js), picked
    // by params.ik_method
    var dq;
    if (params.ik_method === 'pseudoinverse') {
        dq = matrix_multiply(matrix_pseudoinverse(J), dx);
    } else {
        dq = matrix_multiply(matrix_transpose(J), dx);
    }
    robot.dq = dq;

    var steplength = params.ik_steplength !== undefined ? params.ik_steplength : 0.1;
    var i;
    for (i = 0; i < chain.length; i++) {
        robot.joints[chain[i]].control += steplength * dq[i][0];
    }
}
// #endregion iterate-ik

// #region robot-inverse-kinematics
// gate function called every frame from the main loop: only actually steps
// IK when the user has requested it (params.update_ik, a one-shot flag set
// by userinput.js's 'p' key and cleared here once consumed -- mirroring
// upstream's own kineval.robotInverseKinematics), or continuously if
// params.persist_ik is set.
function robotInverseKinematics(robot, params) {
    if (!params.update_ik && !params.persist_ik) return;

    iterateIK(robot, params);

    params.update_ik = false; // clear one-shot IK request for next frame
}
// #endregion robot-inverse-kinematics
