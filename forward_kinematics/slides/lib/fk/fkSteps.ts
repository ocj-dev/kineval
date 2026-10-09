// A step-generator port of ../../../reference/kinematics.js's
// buildFKTransforms/traverseFKBase/traverseFKLink/traverseFKJoint, for
// driving the interactive step-through visualization -- same role
// pathfinding's aStarSteps.ts plays for graph_search.js. Kept structurally
// identical (same recursion shape, same matrix composition order) so the
// two never disagree; the "Component breakdown" slides embed the actual
// reference/kinematics.js code directly via Slidev snippet-import, so
// what's read there is always the real shipped code, not this port.
// `line` on every yielded step matches pseudocode.ts's LINE_* constants.

import type { FKStep, FKFrame, RobotDesc, Mat4 } from './types'
import {
  matrixMultiply, matrixFromOrigin, generateTranslationMatrix, vectorNormalize,
} from './matrixMath'
import { quaternionFromAxisAngle, quaternionNormalize, quaternionToRotationMatrix } from './quaternionMath'
import {
  LINE_BUILD, LINE_BASE, LINE_LINK_HEADER, LINE_LINK_SET, LINE_LINK_LOOP,
  LINE_JOINT_HEADER, LINE_JOINT_ORIGIN, LINE_JOINT_MOTION, LINE_JOINT_RECURSE,
} from './pseudocode'

// Deep-clones robot.joints/links angle state is never mutated here -- the
// tracer only READS joint.angle, so a single RobotDesc can be stepped
// against repeatedly as its sliders change.
export function* fkSteps(robot: RobotDesc): Generator<FKStep, void, unknown> {
  const stack: FKFrame[] = []
  yield { line: LINE_BUILD, action: 'build', depth: 0, stack: snap(stack) }

  const baseMatrix = matrixFromOrigin(robot.origin)
  stack.push({ kind: 'base', name: robot.base, matrix: baseMatrix })
  yield { line: LINE_BASE, action: 'base', linkName: robot.base, depth: 0, matrix: baseMatrix, stack: snap(stack) }

  yield* traverseLink(robot, robot.base, baseMatrix, 1, stack)
  stack.pop()
}

// the panel renders frames live, so each step needs its own copy
function snap(stack: FKFrame[]): FKFrame[] {
  return stack.map((f) => ({ ...f }))
}

function* traverseLink(robot: RobotDesc, name: string, matrix: Mat4, depth: number, stack: FKFrame[]): Generator<FKStep, void, unknown> {
  stack.push({ kind: 'link', name, matrix })
  yield { line: LINE_LINK_HEADER, action: 'link-enter', linkName: name, depth, matrix, stack: snap(stack) }
  yield { line: LINE_LINK_SET, action: 'link-enter', linkName: name, depth, matrix, stack: snap(stack) }

  const childJoints = Object.keys(robot.joints).filter((j) => robot.joints[j].parent === name)
  yield { line: LINE_LINK_LOOP, action: 'link-enter', linkName: name, depth, matrix, stack: snap(stack) }

  for (const jointName of childJoints) {
    yield* traverseJoint(robot, jointName, matrix, depth, stack)
  }
  stack.pop()   // the pop half of the matrix stack: this link's frame goes away
}

function* traverseJoint(robot: RobotDesc, jointName: string, parentMatrix: Mat4, depth: number, stack: FKFrame[]): Generator<FKStep, void, unknown> {
  const joint = robot.joints[jointName]
  stack.push({ kind: 'joint', name: jointName, matrix: parentMatrix })
  yield { line: LINE_JOINT_HEADER, action: 'joint-origin', jointName, depth, matrix: parentMatrix, stack: snap(stack) }

  const jointOrigin = matrixMultiply(parentMatrix, matrixFromOrigin(joint.origin))
  stack[stack.length - 1].matrix = jointOrigin
  yield { line: LINE_JOINT_ORIGIN, action: 'joint-origin', jointName, depth: depth + 1, matrix: jointOrigin, stack: snap(stack) }

  let jointMatrix: Mat4
  if (joint.type === 'prismatic') {
    const unitAxis = vectorNormalize(joint.axis)
    jointMatrix = matrixMultiply(jointOrigin, generateTranslationMatrix(unitAxis[0] * joint.angle, unitAxis[1] * joint.angle, unitAxis[2] * joint.angle))
  } else if (joint.type === 'fixed') {
    jointMatrix = jointOrigin
  } else {
    const q = quaternionNormalize(quaternionFromAxisAngle(joint.axis, joint.angle))
    jointMatrix = matrixMultiply(jointOrigin, quaternionToRotationMatrix(q))
  }
  stack[stack.length - 1].matrix = jointMatrix
  yield { line: LINE_JOINT_MOTION, action: 'joint-motion', jointName, depth: depth + 1, matrix: jointMatrix, stack: snap(stack) }

  yield { line: LINE_JOINT_RECURSE, action: 'joint-recurse', jointName, linkName: joint.child, depth: depth + 1, matrix: jointMatrix, stack: snap(stack) }

  yield* traverseLink(robot, joint.child, jointMatrix, depth + 1, stack)
  stack.pop()   // joint frame popped on the way back up
}
