// The committed RRT-Connect plan, imported straight from
// ../../../reference/plans/, so every panel and every number on a slide
// reads the same file the buildless demo page fetches and the smoke test
// asserts on.
//
// The import reaches outside slides/ on purpose. Vite allows it because
// behavior_trees/pnpm-workspace.yaml marks behavior_trees/ as the
// workspace root -- the same file, and the same reason, that lets
// slides.md's `<<< ../reference/...` snippet imports resolve. A copy of
// the JSON under slides/ would be a second source of truth for numbers
// that appear on slides, which is exactly what this module is trying not
// to have.
import planJson from '../../../reference/plans/mr2_arm_workspace.json'
import type { JointTrajectory, PlanRecord } from './types'
import { trajectoryFromPlan } from './trajectory'

export const PLAN = planJson as unknown as PlanRecord

// The joint positions of the smoothed path, as committed -- the five
// configurations RRT-Connect returned, projected onto the four arm joints.
export const PLAN_POSITIONS: number[][] = (PLAN.smoothed_positions || [])

export function planTrajectory(options?: { useRawPath?: boolean; maxVelocity?: number }): JointTrajectory {
  return trajectoryFromPlan(PLAN, PLAN.joint_names, options)!
}

// The three hand-authored inspection poses the longer-horizon tree
// visits, matching ../../../reference/infrastructure.js#inspection-poses.
// Hand-authored, NOT planned: each is reached by a straight two-waypoint
// trajectory and is not collision-checked, which the slides say out loud.
export const INSPECTION_POSES: Record<string, number[]> = {
  bin_a: [0.4, 0.1, 0.5, 0.0],
  bin_b: [0.9, -0.3, 0.9, 0.4],
  bin_c: [1.1, -0.8, 1.2, 0.9],
}

export const PARTS_PRESENT: Record<string, boolean> = { bin_a: false, bin_b: false, bin_c: true }

export function inspectionTrajectories(): Record<string, JointTrajectory> {
  const names = PLAN.joint_names
  const trajectories: Record<string, JointTrajectory> = {}
  let from = PLAN_POSITIONS[0]
  for (const target of Object.keys(INSPECTION_POSES)) {
    trajectories[target] = trajectoryFromPlan({
      q_layout: names,
      smoothed_path: [from, INSPECTION_POSES[target]],
      joint_names: names,
      robot: PLAN.robot,
      seed: PLAN.seed,
      provenance: { generated_by: 'lib/bt/plan.ts#inspectionTrajectories (hand-authored two-waypoint hop)' },
    }, names)!
    from = INSPECTION_POSES[target]
  }
  return trajectories
}
