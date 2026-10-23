// A step-generator port of computeManipulatorJacobian's own per-joint loop
// (../../../reference/inverse_kinematics.js#manipulator-jacobian), for
// driving JacobianBuildPanel.vue's interactive joint-by-joint walkthrough --
// same role forward_kinematics/slides/lib/fk/fkSteps.ts plays for that
// module's DFS traversal. Kept structurally identical (same per-joint
// z_i/p_i/linear/column computation, same row-major transpose-at-the-end
// assembly) so the two never disagree; the deck's own manipulator-Jacobian
// slide embeds the real reference/inverse_kinematics.js code directly via
// Slidev snippet-import.
import type { JacobianColumnStep, Mat4, RobotDesc } from './types'
import { vectorCross, vectorNormalize, vectorSubtract } from './matrixMath'
import {
  JAC_LINE_PEE, JAC_LINE_LOOP, JAC_LINE_AXIS, JAC_LINE_Z, JAC_LINE_P,
  JAC_LINE_LINEAR, JAC_LINE_COLUMN, JAC_LINE_PUSH, JAC_LINE_DONE,
} from './pseudocode'

export function* jacobianSteps(
  robot: RobotDesc, chainJointNames: string[], includeOrientation: boolean,
): Generator<JacobianColumnStep, void, unknown> {
  const eeJoint = robot.joints[robot.endeffector.frame]
  const eeHom = [0, 1, 2].map((r) =>
    robot.endeffector.position.reduce((s, row, c) => s + (eeJoint.matrix as Mat4)[r][c] * row[0], 0),
  )
  const p_ee: [number, number, number] = [eeHom[0], eeHom[1], eeHom[2]]

  yield { line: JAC_LINE_PEE, jointIndex: -1, jointName: '', axisWorld: [0, 0, 0], originWorld: [0, 0, 0], eeWorld: p_ee, linear: [0, 0, 0], angular: [0, 0, 0], columnsSoFar: [] }

  const columnsSoFar: number[][] = []
  for (let i = 0; i < chainJointNames.length; i++) {
    const jointName = chainJointNames[i]
    const joint = robot.joints[jointName]
    const R = joint.matrix as Mat4

    yield { line: JAC_LINE_LOOP, jointIndex: i, jointName, axisWorld: [0, 0, 0], originWorld: [0, 0, 0], eeWorld: p_ee, linear: [0, 0, 0], angular: [0, 0, 0], columnsSoFar: columnsSoFar.map((c) => c.slice()) }

    const axis = vectorNormalize(joint.axis)
    yield { line: JAC_LINE_AXIS, jointIndex: i, jointName, axisWorld: [0, 0, 0], originWorld: [0, 0, 0], eeWorld: p_ee, linear: [0, 0, 0], angular: [0, 0, 0], columnsSoFar: columnsSoFar.map((c) => c.slice()) }

    const z_i: [number, number, number] = [
      R[0][0] * axis[0] + R[0][1] * axis[1] + R[0][2] * axis[2],
      R[1][0] * axis[0] + R[1][1] * axis[1] + R[1][2] * axis[2],
      R[2][0] * axis[0] + R[2][1] * axis[1] + R[2][2] * axis[2],
    ]
    yield { line: JAC_LINE_Z, jointIndex: i, jointName, axisWorld: z_i, originWorld: [0, 0, 0], eeWorld: p_ee, linear: [0, 0, 0], angular: z_i, columnsSoFar: columnsSoFar.map((c) => c.slice()) }

    const p_i: [number, number, number] = [R[0][3], R[1][3], R[2][3]]
    yield { line: JAC_LINE_P, jointIndex: i, jointName, axisWorld: z_i, originWorld: p_i, eeWorld: p_ee, linear: [0, 0, 0], angular: z_i, columnsSoFar: columnsSoFar.map((c) => c.slice()) }

    const linear = vectorCross(z_i, vectorSubtract(p_ee, p_i)) as [number, number, number]
    yield { line: JAC_LINE_LINEAR, jointIndex: i, jointName, axisWorld: z_i, originWorld: p_i, eeWorld: p_ee, linear, angular: z_i, columnsSoFar: columnsSoFar.map((c) => c.slice()) }

    const column = includeOrientation ? [...linear, ...z_i] : [...linear]
    yield { line: JAC_LINE_COLUMN, jointIndex: i, jointName, axisWorld: z_i, originWorld: p_i, eeWorld: p_ee, linear, angular: z_i, columnsSoFar: columnsSoFar.map((c) => c.slice()) }

    columnsSoFar.push(column)
    yield { line: JAC_LINE_PUSH, jointIndex: i, jointName, axisWorld: z_i, originWorld: p_i, eeWorld: p_ee, linear, angular: z_i, columnsSoFar: columnsSoFar.map((c) => c.slice()) }
  }

  yield { line: JAC_LINE_DONE, jointIndex: chainJointNames.length, jointName: '', axisWorld: [0, 0, 0], originWorld: [0, 0, 0], eeWorld: p_ee, linear: [0, 0, 0], angular: [0, 0, 0], columnsSoFar: columnsSoFar.map((c) => c.slice()) }
}

// The finished Jacobian as a plain row-major matrix (columnsSoFar transposed),
// same assembly computeManipulatorJacobian's own tail loop performs.
export function assembleJacobian(columns: number[][]): number[][] {
  const numRows = columns.length ? columns[0].length : 0
  const jac: number[][] = []
  for (let row = 0; row < numRows; row++) {
    jac[row] = []
    for (let col = 0; col < columns.length; col++) jac[row][col] = columns[col][row]
  }
  return jac
}
