// A plain (non-generator) per-joint snapshot of the exact same math
// jacobianSteps.ts's own generator computes (z_i, p_i, lever arm, column),
// for JacobianExpressionPanel.vue's static "math expression per joint"
// listing. Deliberately does NOT re-derive computeManipulatorJacobian's
// per-joint quantities a second time -- it just drains jacobianSteps.ts's
// generator and keeps each joint's fully-computed JAC_LINE_PUSH snapshot,
// reshaped into a flat array the listing can render directly. Pose is
// whatever robot.joints[*].matrix already holds (buildFK already run by
// the caller), same precondition every other panel in this module has.
import type { RobotDesc } from './types'
import { jacobianSteps } from './jacobianSteps'
import { JAC_LINE_PUSH } from './pseudocode'

export interface JointJacobianExpression {
  jointName: string
  jointIndex: number
  axisLocal: [number, number, number] // joint.axis, as authored (pre-normalize)
  axisWorld: [number, number, number] // z_i = rotation(joint.matrix) * axis_i
  originWorld: [number, number, number] // p_i = translation(joint.matrix)
  eeWorld: [number, number, number] // p_ee -- fixed across every joint
  lever: [number, number, number] // p_ee - p_i
  column: [number, number, number] // z_i x (p_ee - p_i)
}

export function allJointJacobianExpressions(robot: RobotDesc, chainJointNames: string[]): JointJacobianExpression[] {
  const out: JointJacobianExpression[] = []
  for (const step of jacobianSteps(robot, chainJointNames, false)) {
    if (step.line !== JAC_LINE_PUSH) continue
    const lever: [number, number, number] = [
      step.eeWorld[0] - step.originWorld[0],
      step.eeWorld[1] - step.originWorld[1],
      step.eeWorld[2] - step.originWorld[2],
    ]
    out.push({
      jointName: step.jointName,
      jointIndex: step.jointIndex,
      axisLocal: robot.joints[step.jointName].axis,
      axisWorld: step.axisWorld,
      originWorld: step.originWorld,
      eeWorld: step.eeWorld,
      lever,
      column: step.linear,
    })
  }
  return out
}
