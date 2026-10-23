// A plain (non-generator) TS port of ../../../reference/kinematics.js's
// buildFKTransforms/traverseFKBase/traverseFKLink/traverseFKJoint -- every
// IK panel needs each joint's/link's current world .matrix before it can
// build a Jacobian or compute dx, exactly the same prerequisite iterateIK
// itself leans on (robot.joints[name].matrix, already composed by the time
// iterateIK runs). forward_kinematics/slides/lib/fk/fkSteps.ts is a step
// *generator* for that module's own DFS-traversal-order walkthrough; this
// IK module's walkthroughs step through JACOBIAN COLUMNS and IK
// ITERATIONS instead (see jacobianSteps.ts / solveSteps.ts), so this file
// stays a plain recursive function, run once per pose, not a generator.
import type { RobotDesc, Mat4 } from './types'
import { matrixMultiply, matrixFromOrigin, generateTranslationMatrix, vectorNormalize } from './matrixMath'
import { quaternionFromAxisAngle, quaternionNormalize, quaternionToRotationMatrix } from './quaternionMath'

function traverseJoint(robot: RobotDesc, jointName: string, parentMatrix: Mat4) {
  const joint = robot.joints[jointName]
  const jointOrigin = matrixMultiply(parentMatrix, matrixFromOrigin(joint.origin)) as Mat4

  let jointMatrix: Mat4
  if (joint.type === 'prismatic') {
    const unitAxis = vectorNormalize(joint.axis)
    jointMatrix = matrixMultiply(jointOrigin, generateTranslationMatrix(unitAxis[0] * joint.angle, unitAxis[1] * joint.angle, unitAxis[2] * joint.angle)) as Mat4
  } else if (joint.type === 'fixed') {
    jointMatrix = jointOrigin
  } else {
    const q = quaternionNormalize(quaternionFromAxisAngle(joint.axis, joint.angle))
    jointMatrix = matrixMultiply(jointOrigin, quaternionToRotationMatrix(q)) as Mat4
  }

  joint.matrix = jointMatrix
  traverseLink(robot, joint.child, jointMatrix)
}

function traverseLink(robot: RobotDesc, linkName: string, matrix: Mat4) {
  const link = robot.links[linkName]
  link.matrix = matrix
  for (const jointName of Object.keys(robot.joints).filter((j) => robot.joints[j].parent === linkName)) {
    traverseJoint(robot, jointName, matrix)
  }
}

export function buildFK(robot: RobotDesc) {
  const baseMatrix = matrixFromOrigin(robot.origin)
  traverseLink(robot, robot.base, baseMatrix)
}

export function endEffectorWorldPosition(robot: RobotDesc): [number, number, number] {
  const joint = robot.joints[robot.endeffector.frame]
  const p = matrixMultiply(joint.matrix as Mat4, robot.endeffector.position)
  return [p[0][0], p[1][0], p[2][0]]
}
