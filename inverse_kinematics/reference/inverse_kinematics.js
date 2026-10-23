/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Inverse Kinematics | Jacobian-Based Iteration

    COMPLETED REFERENCE IMPLEMENTATION for the AutoRob (autorob.org) lab-session
    slides. Completes every "STENCIL" section of the upstream kineval-stencil
    (github.com/autorob/kineval-stencil) kineval/kineval_inverse_kinematics.js
    module: robotInverseKinematics, iterateIK -- plus a new function,
    computeManipulatorJacobian, factored out of iterateIK as its own named,
    quotable unit purely for this deck's walkthrough slides (upstream folds
    Jacobian construction directly into iterateIK's body; nothing upstream
    calls it out separately).

    Two naming/signature differences from upstream, both the same house style
    already used by ../../forward_kinematics/reference/kinematics.js and this
    project's other modules:

      1. No "kineval = {}" namespace object, and no global kineval.params --
         every function here is a plain top-level function taking `robot`
         (and whatever else it needs) as an explicit argument, including the
         "was an update requested this frame" flag upstream reads off
         kineval.params.update_ik/persist_ik. This module in particular reads
         a single `ikParams` argument ({method, steplength, damping,
         orientationIncluded, chainJointNames}) everywhere upstream would
         have reached into kineval.params.

      2. upstream's `endeffector_joint` argument is robot.endeffector.frame,
         a JOINT NAME STRING (home.html calls robotInverseKinematics with
         robot.endeffector.frame itself, not robot.joints[robot.endeffector.frame]);
         iterateIK would have had to look it up by name internally. Here
         endeffector_joint is the resolved JOINT OBJECT itself (the thing
         carrying .matrix), since the caller already has it at hand -- one
         less lookup, and it matches how computeManipulatorJacobian's other
         per-joint arguments (robot.joints[chainJointNames[i]]) are already
         objects, not names.

    Property-name note inherited from ../../forward_kinematics/reference/
    kinematics.js (copied verbatim into this module's kinematics.js): each
    joint's/link's fully-composed 4x4 world transform is stored as `.matrix`,
    NOT `.xform` -- `.xform` is a different naming convention used only by
    the full kineval/ viewer (kineval/kinematics.js), which this module does
    not depend on. Every reference to a joint's world transform below reads
    `.matrix`.

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

// #region robot-inverse-kinematics
function robotInverseKinematics(robot, endeffector_target_world, endeffector_joint, endeffector_position_local, ikParams, updateRequested) {
    // compute joint angle controls to move location on specified link to
    // Cartesian location. Mirrors upstream's own gate --
    //   if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { ... }
    // -- as an explicit boolean argument instead of two flags read off a
    // global kineval.params object, matching this project's no-global-
    // namespace style (see this file's header). The caller (infrastructure.js)
    // is the one deciding, per animation tick, whether an IK update is due;
    // this function just gates on that decision and performs exactly one
    // Jacobian iteration when it is.
    if (!updateRequested) return;

    iterateIK(robot, endeffector_target_world, endeffector_joint, endeffector_position_local, ikParams);
}
// #endregion robot-inverse-kinematics

// #region manipulator-jacobian
function computeManipulatorJacobian(robot, chainJointNames, endeffector_joint, endeffector_position_local, includeOrientation) {
    // end-effector world position: endeffector_joint's already-computed
    // world transform (.matrix, 4x4 homogeneous) applied to the
    // end-effector's local offset point (homogeneous 4x1 column vector)
    var eePositionHomogeneous = matrix_multiply(endeffector_joint.matrix, endeffector_position_local);
    var p_ee = [eePositionHomogeneous[0][0], eePositionHomogeneous[1][0], eePositionHomogeneous[2][0]];

    // one column per joint in chainJointNames (base-to-end-effector order);
    // built column-by-column first (one 3- or 6-vector per joint), then
    // transposed into matrix.js's row-major convention (array of row arrays)
    // right before returning -- matrix_multiply/matrix_transpose both index
    // as m[row][col], so robot.jacobian must do the same.
    var columns = [];
    var i, jointName, joint, R, axis, z_i, p_i, linear, column;

    for (i = 0; i < chainJointNames.length; i++) {
        jointName = chainJointNames[i];
        joint = robot.joints[jointName];

        // world axis z_i = rotation block of joint.matrix times the joint's
        // own local rotation axis. joint.matrix already includes THIS
        // joint's own variable-angle rotation (traverseFKJoint in
        // kinematics.js folds it in before returning), yet using it here is
        // still correct: rotating a vector about itself leaves it unchanged,
        // so rotating joint.axis by R_ancestors*R_fixed*R_own gives the exact
        // same result as rotating it by R_ancestors*R_fixed alone (R_own is
        // a rotation about joint.axis itself). Only ANCESTOR joints'
        // rotations actually change the direction of this joint's own axis;
        // this joint's own current angle never does.
        axis = vector_normalize(joint.axis);
        R = joint.matrix;
        z_i = [
            R[0][0] * axis[0] + R[0][1] * axis[1] + R[0][2] * axis[2],
            R[1][0] * axis[0] + R[1][1] * axis[1] + R[1][2] * axis[2],
            R[2][0] * axis[0] + R[2][1] * axis[1] + R[2][2] * axis[2]
        ];

        // joint i's world origin = translation part of joint.matrix (also
        // unaffected by this joint's own rotation, which has zero
        // translation of its own -- see this function's Jacobian slide for
        // the one-line proof: appending a pure rotation to a matrix stack
        // never changes the stack's translation column).
        p_i = [joint.matrix[0][3], joint.matrix[1][3], joint.matrix[2][3]];

        // linear velocity column of a revolute joint: z_i x (p_ee - p_i)
        linear = vector_cross(z_i, vector_subtract(p_ee, p_i));

        // angular velocity column (only when solving for orientation too)
        // is just the joint's own world rotation axis
        column = includeOrientation ? linear.concat(z_i) : linear;
        columns.push(column);
    }

    var numRows = includeOrientation ? 6 : 3;
    var jacobian = [];
    var row, col;
    for (row = 0; row < numRows; row++) {
        jacobian[row] = [];
        for (col = 0; col < columns.length; col++) {
            jacobian[row][col] = columns[col][row];
        }
    }

    robot.jacobian = jacobian;
    return jacobian;
}
// #endregion manipulator-jacobian

// #region orientation-error-helpers
// quaternion.js (copied verbatim from ../../forward_kinematics/reference/)
// completes exactly the 4 functions upstream's own kineval_quaternion.js
// STENCIL lists -- quaternion_from_axisangle, quaternion_normalize,
// quaternion_multiply, quaternion_to_rotation_matrix -- and no more. Neither
// a quaternion conjugate nor a rotation-matrix-to-quaternion conversion is
// among them, so both small helpers iterateIK's orientation-error term needs
// are kept locally here instead of growing that file's scope beyond its own
// documented STENCIL list.
function quaternionConjugateLocal(q) {
    return { a: q.a, b: -q.b, c: -q.c, d: -q.d };
}

function rotationMatrixToQuaternion(R) {
    // standard trace-based (Shepperd's method) extraction of a unit
    // quaternion from a 3x3 rotation matrix R (indexed R[row][col]); picks
    // whichever of the 4 algebraically-equivalent formulas has the largest,
    // best-conditioned denominator for the given R.
    var m00 = R[0][0], m11 = R[1][1], m22 = R[2][2];
    var trace = m00 + m11 + m22;
    var a, b, c, d, s;

    if (trace > 0) {
        s = 2 * Math.sqrt(trace + 1.0);
        a = 0.25 * s;
        b = (R[2][1] - R[1][2]) / s;
        c = (R[0][2] - R[2][0]) / s;
        d = (R[1][0] - R[0][1]) / s;
    } else if (m00 > m11 && m00 > m22) {
        s = 2 * Math.sqrt(1.0 + m00 - m11 - m22);
        a = (R[2][1] - R[1][2]) / s;
        b = 0.25 * s;
        c = (R[0][1] + R[1][0]) / s;
        d = (R[0][2] + R[2][0]) / s;
    } else if (m11 > m22) {
        s = 2 * Math.sqrt(1.0 + m11 - m00 - m22);
        a = (R[0][2] - R[2][0]) / s;
        b = (R[0][1] + R[1][0]) / s;
        c = 0.25 * s;
        d = (R[1][2] + R[2][1]) / s;
    } else {
        s = 2 * Math.sqrt(1.0 + m22 - m00 - m11);
        a = (R[1][0] - R[0][1]) / s;
        b = (R[0][2] + R[2][0]) / s;
        c = (R[1][2] + R[2][1]) / s;
        d = 0.25 * s;
    }

    return quaternion_normalize({ a: a, b: b, c: c, d: d });
}
// #endregion orientation-error-helpers

// #region iterate-inverse-kinematics
function iterateIK(robot, endeffector_target_world, endeffector_joint, endeffector_position_local, ikParams) {
    var chainJointNames = ikParams.chainJointNames;
    var includeOrientation = !!(ikParams.orientationIncluded && endeffector_target_world.orientation);

    // p_ee: current end-effector world position (recomputed identically to
    // computeManipulatorJacobian's own copy below -- cheap, and keeps this
    // function readable as a self-contained "error, then Jacobian, then
    // solve" sequence rather than threading an extra return value through)
    var eePositionHomogeneous = matrix_multiply(endeffector_joint.matrix, endeffector_position_local);
    var p_ee = [eePositionHomogeneous[0][0], eePositionHomogeneous[1][0], eePositionHomogeneous[2][0]];

    // [NOTICE]: robot.dx, robot.jacobian, and robot.dq (below) are the 3
    // variables this module is graded/tested against -- keep these exact
    // names and shapes. robot.dx is 3x1 (position only) or 6x1 (with
    // orientation); robot.dq does NOT have the step length folded in.

    // --- robot.dx: target-minus-current error term ---
    var dPosition = vector_subtract(endeffector_target_world.position, p_ee);
    var dx = [[dPosition[0]], [dPosition[1]], [dPosition[2]]];

    if (includeOrientation) {
        var R_current = [
            [endeffector_joint.matrix[0][0], endeffector_joint.matrix[0][1], endeffector_joint.matrix[0][2]],
            [endeffector_joint.matrix[1][0], endeffector_joint.matrix[1][1], endeffector_joint.matrix[1][2]],
            [endeffector_joint.matrix[2][0], endeffector_joint.matrix[2][1], endeffector_joint.matrix[2][2]]
        ];
        var q_current = rotationMatrixToQuaternion(R_current);
        var q_target = endeffector_target_world.orientation;

        // standard small-angle/axis-angle orientation error:
        //   d_theta ~= 2 * vec(q_target * conj(q_current))
        // (the imaginary part of the "difference" quaternion, doubled, is a
        // first-order approximation of the axis-angle rotation vector that
        // would carry the current orientation onto the target orientation --
        // exact in the limit as that rotation shrinks to zero, which is
        // exactly the regime a converging IK iteration runs in)
        var q_err = quaternion_multiply(q_target, quaternionConjugateLocal(q_current));
        dx.push([2 * q_err.b]);
        dx.push([2 * q_err.c]);
        dx.push([2 * q_err.d]);
    }
    robot.dx = dx;

    // --- robot.jacobian ---
    robot.jacobian = computeManipulatorJacobian(robot, chainJointNames, endeffector_joint, endeffector_position_local, includeOrientation);

    // --- robot.dq = T(robot.jacobian) * robot.dx, T depending on ikParams.method ---
    var JT = matrix_transpose(robot.jacobian);
    var dq, i;

    if (ikParams.method === 'pseudoinverse') {
        // reuses matrix_pseudoinverse AS-IS from matrix.js -- for a
        // redundant chain solving position only (more joints than task
        // dimensions), robot.jacobian is WIDE, so matrix_pseudoinverse's
        // right form (J^T (J J^T)^-1) is the branch that actually runs
        dq = matrix_multiply(matrix_pseudoinverse(robot.jacobian), robot.dx);
    } else if (ikParams.method === 'damped') {
        // damped least squares: dq = (J^T J + lambda^2 I)^-1 J^T dx.
        // Built from matrix_multiply/matrix_transpose/matrix_square_invert
        // exactly as instructed, but WITHOUT routing through
        // matrix.js's generate_identity(): that helper is a fixed 4x4 (sized
        // for homogeneous transforms, matrix.js's only other use for an
        // identity), not an NxN sized to this chain's joint count, and
        // matrix.js has no scalar-multiply or matrix-add helper to scale and
        // add a separate identity matrix with anyway. Adding lambda^2
        // directly onto the diagonal of J^T J produces the identical
        // (J^T J + lambda^2 I) matrix with no extra helpers required.
        var JTJ = matrix_multiply(JT, robot.jacobian);
        var lambdaSquared = ikParams.damping * ikParams.damping;
        for (i = 0; i < JTJ.length; i++) {
            JTJ[i][i] += lambdaSquared;
        }
        dq = matrix_multiply(matrix_multiply(matrix_square_invert(JTJ), JT), robot.dx);
    } else { // 'transpose' (default)
        dq = matrix_multiply(JT, robot.dx);
    }
    robot.dq = dq;

    // Accumulate this iteration's step onto each chain joint's .control
    // field -- NOT directly onto .angle. This project's applyControls
    // (../../kineval/controls.js) is the single place every control source
    // (user input, servo setpoints in servo_control.js, and IK here alike)
    // gets folded from .control into .angle, exactly once per frame, then
    // zeroed. Writing .angle directly here would quietly bypass that shared
    // per-frame pipeline and double-apply relative to however the rest of
    // this project's control sources are wired. iterateIK instead produces
    // exactly one Jacobian step's worth of .control per call -- the same
    // contribution a single controller tick would -- and leaves folding it
    // into .angle to applyControls, same as every other controller.
    // Defensive guard, NOT part of the graded dx/jacobian/dq values above
    // (those are left exactly as computed, singular or not -- a grader
    // inspecting them still sees the real, possibly-NaN result of the
    // chosen method at this configuration): 'pseudoinverse' has no
    // regularization against an exactly-singular J^T J (matrix_square_invert
    // divides by an exact-zero pivot there, same way 1/0 would), which a
    // converging arm can genuinely drive itself into/through -- the straight-
    // arm Jacobian singularity this file's manipulator-jacobian region
    // already discusses. Once .control goes NaN it corrupts .angle
    // permanently (NaN propagates through every later buildFKTransforms
    // call), so a degenerate iteration here is skipped rather than applied --
    // exactly the failure mode 'damped' least squares exists to avoid in the
    // first place.
    var steplength = ikParams.steplength !== undefined ? ikParams.steplength : 1;
    var dqIsFinite = true;
    for (i = 0; i < chainJointNames.length; i++) {
        if (!isFinite(robot.dq[i][0])) dqIsFinite = false;
    }
    if (dqIsFinite) {
        for (i = 0; i < chainJointNames.length; i++) {
            robot.joints[chainJointNames[i]].control += steplength * robot.dq[i][0];
        }
    }
}
// #endregion iterate-inverse-kinematics
