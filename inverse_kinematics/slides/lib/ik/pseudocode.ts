// Two shared master pseudocode listings, reused with different active-line
// highlighting across this deck's panels -- same pattern
// forward_kinematics/slides/lib/fk/pseudocode.ts establishes for that
// module. MASTER_IK_LOOP mirrors robotInverseKinematics/iterateIK
// line-for-line (reference/inverse_kinematics.js#robot-inverse-kinematics,
// #iterate-inverse-kinematics); MASTER_JACOBIAN_BUILD mirrors
// computeManipulatorJacobian's own per-joint loop
// (reference/inverse_kinematics.js#manipulator-jacobian).

export const MASTER_IK_LOOP: string[] = [
  /* 0 */ 'robotInverseKinematics(robot, target, eeJoint, eeLocalPos, ikParams, updateRequested):',
  /* 1 */ '    if (!updateRequested) return',
  /* 2 */ '    iterateIK(robot, target, eeJoint, eeLocalPos, ikParams)',
  /* 3 */ 'iterateIK(robot, target, eeJoint, eeLocalPos, ikParams):',
  /* 4 */ '    p_ee = eeJoint.matrix * eeLocalPos',
  /* 5 */ '    robot.dx = target.position - p_ee        -- (+ 3 orientation rows if included)',
  /* 6 */ '    robot.jacobian = computeManipulatorJacobian(robot, chainJointNames, eeJoint, eeLocalPos, ...)',
  /* 7 */ '    JT = matrix_transpose(robot.jacobian)',
  /* 8 */ '    robot.dq = solve(ikParams.method, JT, robot.jacobian, robot.dx)   -- transpose|pseudoinverse|damped',
  /* 9 */ '    for each joint i in chainJointNames:',
  /* 10 */ '        joint[i].control += ikParams.steplength * robot.dq[i]',
]
export const IK_LOOP_LINE_GATE = 1
export const IK_LOOP_LINE_PEE = 4
export const IK_LOOP_LINE_DX = 5
export const IK_LOOP_LINE_JACOBIAN = 6
export const IK_LOOP_LINE_TRANSPOSE = 7
export const IK_LOOP_LINE_SOLVE = 8
export const IK_LOOP_LINE_SCALE_LOOP = 9
export const IK_LOOP_LINE_SCALE_APPLY = 10

export const MASTER_JACOBIAN_BUILD: string[] = [
  /* 0 */ 'computeManipulatorJacobian(robot, chainJointNames, eeJoint, eeLocalPos, includeOrientation):',
  /* 1 */ '    p_ee = eeJoint.matrix * eeLocalPos      -- fixed for the whole build',
  /* 2 */ '    columns = []',
  /* 3 */ '    for i in chainJointNames:   -- base-to-end-effector order',
  /* 4 */ '        axis = vector_normalize(joint[i].axis)',
  /* 5 */ '        z_i  = rotation(joint[i].matrix) * axis     -- world axis',
  /* 6 */ '        p_i  = translation(joint[i].matrix)          -- world origin',
  /* 7 */ '        linear  = vector_cross(z_i, p_ee - p_i)',
  /* 8 */ '        column  = includeOrientation ? linear ++ z_i : linear',
  /* 9 */ '        columns.push(column)',
  /* 10 */ '    jacobian = transpose(columns)   -- one COLUMN per joint, row-major storage',
  /* 11 */ '    return jacobian',
]
export const JAC_LINE_PEE = 1
export const JAC_LINE_LOOP = 3
export const JAC_LINE_AXIS = 4
export const JAC_LINE_Z = 5
export const JAC_LINE_P = 6
export const JAC_LINE_LINEAR = 7
export const JAC_LINE_COLUMN = 8
export const JAC_LINE_PUSH = 9
export const JAC_LINE_DONE = 10
