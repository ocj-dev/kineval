// TS port of ../../../reference/inverse_kinematics.js's
// computeManipulatorJacobian + iterateIK, in two forms:
//   - computeJacobianPlain / solveDq / runIkIteration: plain functions, used
//     by IkConvergePanel.vue and the test-case panel so the picture is
//     driven by the SAME math being taught, never a separately-derived
//     approximation ("derive overlays by calling the same function being
//     taught" -- see this deck's draggable-target slide).
//   - iterateIkSteps: a step GENERATOR over the same sequence, for
//     IkStepPanel.vue's walkthrough of iterateIK itself (distinct from
//     jacobianSteps.ts's per-JOINT-COLUMN walkthrough -- this one steps
//     through the solve-and-apply algorithm once the Jacobian already
//     exists, mirroring iterateIK's own gate/error/jacobian/solve/scale
//     sequence line-for-line).
import type { IkMethod, IkParams, IterateIkStep, Mat4, RobotDesc } from './types'
import {
  matrixMultiply, matrixTranspose, matrixSquareInvert, matrixPseudoinverse, vectorSubtract,
} from './matrixMath'
import { quaternionConjugate, quaternionMultiply, rotationMatrixToQuaternion } from './quaternionMath'
import type { Quat } from './types'
import { assembleJacobian, jacobianSteps } from './jacobianSteps'
import {
  IK_LOOP_LINE_GATE, IK_LOOP_LINE_PEE, IK_LOOP_LINE_DX, IK_LOOP_LINE_JACOBIAN,
  IK_LOOP_LINE_TRANSPOSE, IK_LOOP_LINE_SOLVE, IK_LOOP_LINE_SCALE_LOOP, IK_LOOP_LINE_SCALE_APPLY,
  JAC_LINE_PUSH,
} from './pseudocode'

export function computeJacobianPlain(robot: RobotDesc, chainJointNames: string[], includeOrientation: boolean): number[][] {
  let columns: number[][] = []
  for (const step of jacobianSteps(robot, chainJointNames, includeOrientation)) {
    if (step.line === JAC_LINE_PUSH) columns = step.columnsSoFar
  }
  return assembleJacobian(columns)
}

// dq, WITHOUT the step length folded in -- same contract as iterateIK's own
// robot.dq (its own [NOTICE] comment: robot.dx/robot.jacobian/robot.dq are
// the 3 values this module is graded against; steplength scaling happens
// only when writing .control).
export function solveDq(method: IkMethod, jacobian: number[][], dx: number[][], damping: number): number[][] {
  const JT = matrixTranspose(jacobian)
  if (method === 'pseudoinverse') {
    return matrixMultiply(matrixPseudoinverse(jacobian), dx)
  } else if (method === 'damped') {
    const JTJ = matrixMultiply(JT, jacobian)
    const lambdaSquared = damping * damping
    for (let i = 0; i < JTJ.length; i++) JTJ[i][i] += lambdaSquared
    return matrixMultiply(matrixMultiply(matrixSquareInvert(JTJ), JT), dx)
  }
  return matrixMultiply(JT, dx)
}

function orientationErrorRows(robot: RobotDesc, targetOrientation: Quat): number[][] {
  const m = robot.joints[robot.endeffector.frame].matrix as Mat4
  const R = [[m[0][0], m[0][1], m[0][2]], [m[1][0], m[1][1], m[1][2]], [m[2][0], m[2][1], m[2][2]]]
  const qCurrent = rotationMatrixToQuaternion(R)
  const qErr = quaternionMultiply(targetOrientation, quaternionConjugate(qCurrent))
  return [[2 * qErr.b], [2 * qErr.c], [2 * qErr.d]]
}

// One full iterateIK call: error term, Jacobian, solve, scale+write
// .control -- but does NOT fold .control into .angle or re-run FK; the
// caller (IkConvergePanel.vue's animation loop, same three-stage pipeline
// infrastructure.js's own createAnimationLoop runs) does that afterwards,
// exactly like the reference's own inverse_kinematics.html.
export function runIkIteration(
  robot: RobotDesc, targetPosition: [number, number, number], targetOrientation: Quat | null, ikParams: IkParams,
): { dx: number[][]; jacobian: number[][]; dq: number[][] } {
  const eeJoint = robot.joints[robot.endeffector.frame]
  const eeHom = [0, 1, 2].map((r) =>
    robot.endeffector.position.reduce((s, row, c) => s + (eeJoint.matrix as Mat4)[r][c] * row[0], 0),
  )
  const includeOrientation = !!(ikParams.orientationIncluded && targetOrientation)
  const dPosition = vectorSubtract(targetPosition, eeHom)
  const dx: number[][] = dPosition.map((v) => [v])
  if (includeOrientation) dx.push(...orientationErrorRows(robot, targetOrientation as Quat))

  const jacobian = computeJacobianPlain(robot, ikParams.chainJointNames, includeOrientation)
  robot.jacobian = jacobian
  robot.dx = dx
  const dq = solveDq(ikParams.method, jacobian, dx, ikParams.damping)
  robot.dq = dq

  let dqIsFinite = true
  for (let i = 0; i < ikParams.chainJointNames.length; i++) if (!isFinite(dq[i][0])) dqIsFinite = false
  if (dqIsFinite) {
    for (let i = 0; i < ikParams.chainJointNames.length; i++) {
      const joint = robot.joints[ikParams.chainJointNames[i]]
      joint.control = (joint.control || 0) + ikParams.steplength * dq[i][0]
    }
  }
  return { dx, jacobian, dq }
}

export function applyControls(robot: RobotDesc) {
  for (const name in robot.joints) {
    const joint = robot.joints[name]
    joint.angle += joint.control || 0
    joint.control = 0
  }
}

export function* iterateIkSteps(
  robot: RobotDesc, targetPosition: [number, number, number], targetOrientation: Quat | null, ikParams: IkParams, updateRequested: boolean,
): Generator<IterateIkStep, void, unknown> {
  yield { line: IK_LOOP_LINE_GATE, action: 'gate', note: updateRequested ? 'update requested -- proceed' : 'no update requested -- return' }
  if (!updateRequested) return

  const eeJoint = robot.joints[robot.endeffector.frame]
  const eeHom = [0, 1, 2].map((r) =>
    robot.endeffector.position.reduce((s, row, c) => s + (eeJoint.matrix as Mat4)[r][c] * row[0], 0),
  )
  yield { line: IK_LOOP_LINE_PEE, action: 'error', note: `p_ee = [${eeHom.map((v) => v.toFixed(2)).join(', ')}]` }

  const includeOrientation = !!(ikParams.orientationIncluded && targetOrientation)
  const dPosition = vectorSubtract(targetPosition, eeHom)
  const dx: number[][] = dPosition.map((v) => [v])
  if (includeOrientation) dx.push(...orientationErrorRows(robot, targetOrientation as Quat))
  yield { line: IK_LOOP_LINE_DX, action: 'error', dx }

  const jacobian = computeJacobianPlain(robot, ikParams.chainJointNames, includeOrientation)
  yield { line: IK_LOOP_LINE_JACOBIAN, action: 'jacobian', dx, jacobian }

  const JT = matrixTranspose(jacobian)
  yield { line: IK_LOOP_LINE_TRANSPOSE, action: 'transpose', dx, jacobian, jacobianT: JT }

  const dq = solveDq(ikParams.method, jacobian, dx, ikParams.damping)
  yield { line: IK_LOOP_LINE_SOLVE, action: 'solve', dx, jacobian, jacobianT: JT, dq, note: `method = ${ikParams.method}` }

  yield { line: IK_LOOP_LINE_SCALE_LOOP, action: 'scale-apply', dx, jacobian, dq, note: `steplength = ${ikParams.steplength}` }

  let dqIsFinite = true
  for (let i = 0; i < ikParams.chainJointNames.length; i++) if (!isFinite(dq[i][0])) dqIsFinite = false
  if (dqIsFinite) {
    for (let i = 0; i < ikParams.chainJointNames.length; i++) {
      const joint = robot.joints[ikParams.chainJointNames[i]]
      joint.control = (joint.control || 0) + ikParams.steplength * dq[i][0]
    }
  }
  yield { line: IK_LOOP_LINE_SCALE_APPLY, action: 'scale-apply', dx, jacobian, dq, note: dqIsFinite ? 'control written' : 'skipped -- dq not finite' }
}
