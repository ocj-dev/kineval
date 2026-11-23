// A TS port of ../../../reference/joint_trajectory.js. See types.ts's
// header for why a port exists; the reference file carries the reasoning.
import type { JointTrajectory, PlanRecord, RobotDesc } from './types'

export const TRAJECTORY_MAX_JOINT_VELOCITY = 0.6
export const TRAJECTORY_MIN_SEGMENT_SECONDS = 0.05

export function planJointColumns(path: number[][], qLayout: string[], jointNames: string[]): number[][] | null {
  const column: Record<string, number> = {}
  for (let i = 0; i < qLayout.length; i++) column[qLayout[i]] = i
  for (const name of jointNames) if (column[name] === undefined) return null

  return path.map((q) => jointNames.map((name) => q[column[name]]))
}

const BASE_NAMES = ['base_x', 'base_y', 'base_z', 'base_roll', 'base_pitch', 'base_yaw']

export function planBaseColumns(path: number[][], qLayout: string[]): number[][] | null {
  for (const name of BASE_NAMES) if (qLayout.indexOf(name) === -1) return null
  return planJointColumns(path, qLayout, BASE_NAMES)
}

export function baseIsStationary(path: number[][], qLayout: string[], tolerance = 1e-9): boolean {
  const base = planBaseColumns(path, qLayout)
  if (!base || base.length === 0) return true
  for (let i = 1; i < base.length; i++) {
    for (let j = 0; j < base[i].length; j++) {
      if (Math.abs(base[i][j] - base[0][j]) > tolerance) return false
    }
  }
  return true
}

export function trajectoryTimesFromPositions(positions: number[][], maxVelocity = TRAJECTORY_MAX_JOINT_VELOCITY): number[] {
  const times = [0]
  for (let i = 1; i < positions.length; i++) {
    let largest = 0
    for (let j = 0; j < positions[i].length; j++) {
      largest = Math.max(largest, Math.abs(positions[i][j] - positions[i - 1][j]))
    }
    times.push(times[i - 1] + Math.max(largest / maxVelocity, TRAJECTORY_MIN_SEGMENT_SECONDS))
  }
  return times
}

export function trajectoryVelocities(positions: number[][], times: number[]): number[][] {
  return positions.map((row, i) =>
    row.map((_, j) => {
      if (i === 0 || i === positions.length - 1) return 0
      const span = times[i + 1] - times[i - 1]
      return span > 0 ? (positions[i + 1][j] - positions[i - 1][j]) / span : 0
    }),
  )
}

export function trajectoryFromPlan(
  plan: PlanRecord,
  jointNames?: string[],
  options?: { useRawPath?: boolean; maxVelocity?: number },
): JointTrajectory | null {
  const opts = options || {}
  const path = opts.useRawPath ? plan.raw_path! : plan.smoothed_path
  const names = jointNames || plan.joint_names

  if (!path || path.length < 2) return null
  if (!baseIsStationary(path, plan.q_layout)) return null

  const positions = planJointColumns(path, plan.q_layout, names)
  if (!positions) return null

  const times = trajectoryTimesFromPositions(positions, opts.maxVelocity)
  const velocities = trajectoryVelocities(positions, times)

  return {
    header: { stamp: 0, frame_id: '' },
    joint_names: names.slice(),
    points: positions.map((p, i) => ({
      positions: p,
      velocities: velocities[i],
      accelerations: [],
      effort: [],
      time_from_start: times[i],
    })),
    provenance: {
      plan: plan.provenance ? plan.provenance.generated_by : 'unknown',
      robot: plan.robot,
      seed: plan.seed,
      path_used: opts.useRawPath ? 'raw_path' : 'smoothed_path',
      max_velocity: opts.maxVelocity === undefined ? TRAJECTORY_MAX_JOINT_VELOCITY : opts.maxVelocity,
    },
  }
}

export function trajectoryDuration(trajectory: JointTrajectory | null): number {
  if (!trajectory || !trajectory.points.length) return 0
  return trajectory.points[trajectory.points.length - 1].time_from_start
}

export function sampleTrajectory(
  trajectory: JointTrajectory, t: number,
): { positions: number[]; segment: number; alpha: number } {
  const points = trajectory.points
  const n = points.length

  if (t <= points[0].time_from_start) return { positions: points[0].positions.slice(), segment: 0, alpha: 0 }
  if (t >= points[n - 1].time_from_start) {
    return { positions: points[n - 1].positions.slice(), segment: n - 2, alpha: 1 }
  }

  for (let i = 0; i < n - 1; i++) {
    const t0 = points[i].time_from_start
    const t1 = points[i + 1].time_from_start
    if (t < t0 || t > t1) continue
    const alpha = (t - t0) / (t1 - t0)
    const positions = points[i].positions.map((p, j) => p + alpha * (points[i + 1].positions[j] - p))
    return { positions, segment: i, alpha }
  }
  return { positions: points[n - 1].positions.slice(), segment: n - 2, alpha: 1 }
}

// NOT in the reference implementation, and deliberately so: a Catmull-Rom
// spline through the same waypoints, used by the interpolation panel to
// show what "smoother" buys and what it costs. The reference interpolates
// linearly because the planner validated the straight line between
// consecutive configurations and a spline leaves it; this exists to make
// the amount it leaves by visible, not to be used.
export function sampleTrajectorySpline(trajectory: JointTrajectory, t: number): number[] {
  const points = trajectory.points
  const n = points.length
  const linear = sampleTrajectory(trajectory, t)
  if (n < 3 || t <= 0 || t >= trajectory.points[n - 1].time_from_start) return linear.positions

  const i = linear.segment
  const a = linear.alpha
  const p = (k: number) => points[Math.max(0, Math.min(n - 1, k))].positions

  const p0 = p(i - 1), p1 = p(i), p2 = p(i + 1), p3 = p(i + 2)
  return p1.map((_, j) =>
    0.5 * ((2 * p1[j]) +
      (-p0[j] + p2[j]) * a +
      (2 * p0[j] - 5 * p1[j] + 4 * p2[j] - p3[j]) * a * a +
      (-p0[j] + 3 * p1[j] - 3 * p2[j] + p3[j]) * a * a * a))
}

export function validateTrajectory(trajectory: JointTrajectory | null): string[] {
  const problems: string[] = []
  if (!trajectory) return ['trajectory is null']
  if (!trajectory.joint_names || !trajectory.joint_names.length) {
    problems.push('joint_names is empty: the positions arrays have no meaning without it')
  }
  if (!trajectory.points || trajectory.points.length < 2) {
    problems.push('a trajectory needs at least two points')
    return problems
  }

  const width = trajectory.joint_names.length
  trajectory.points.forEach((p, i) => {
    if (p.positions.length !== width) {
      problems.push(`point ${i} has ${p.positions.length} positions for ${width} joint_names`)
    }
    if (p.velocities.length && p.velocities.length !== width) {
      problems.push(`point ${i} has ${p.velocities.length} velocities for ${width} joint_names`)
    }
    p.positions.forEach((v, j) => {
      if (!isFinite(v)) problems.push(`point ${i} position ${j} is not finite`)
    })
    if (!isFinite(p.time_from_start) || p.time_from_start < 0) {
      problems.push(`point ${i} has time_from_start ${p.time_from_start}`)
    }
    if (i > 0 && p.time_from_start <= trajectory.points[i - 1].time_from_start) {
      problems.push(`point ${i} time_from_start does not increase past point ${i - 1}`)
    }
  })

  const seen: Record<string, boolean> = {}
  for (const name of trajectory.joint_names) {
    if (seen[name]) problems.push(`joint_names contains "${name}" twice`)
    seen[name] = true
  }
  return problems
}

export function trajectoryJointsNotNamed(trajectory: JointTrajectory, robot: RobotDesc): string[] {
  const named: Record<string, boolean> = {}
  for (const name of trajectory.joint_names) named[name] = true
  return Object.keys(robot.joints).filter((x) => !named[x])
}
