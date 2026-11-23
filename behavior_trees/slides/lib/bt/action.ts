// A TS port of ../../../reference/joint_trajectory_action.js. See
// types.ts's header for why a port exists; the reference file carries the
// reasoning, including why the controller is a servo and not a teleport.
import type { ActionServer, ActionState, JointTrajectory, RobotDesc } from './types'
import { sampleTrajectory, trajectoryDuration, validateTrajectory } from './trajectory'

export const JTA_ACCEPTED: ActionState = 'accepted'
export const JTA_EXECUTING: ActionState = 'executing'
export const JTA_CANCELING: ActionState = 'canceling'
export const JTA_SUCCEEDED: ActionState = 'succeeded'
export const JTA_CANCELED: ActionState = 'canceled'
export const JTA_ABORTED: ActionState = 'aborted'

export const JTA_SUCCESSFUL = 0
export const JTA_INVALID_GOAL = -1
export const JTA_INVALID_JOINTS = -2
export const JTA_OLD_HEADER_TIMESTAMP = -3
export const JTA_PATH_TOLERANCE_VIOLATED = -4
export const JTA_GOAL_TOLERANCE_VIOLATED = -5

export const JTA_SERVO_GAIN = 0.35
export const JTA_PATH_TOLERANCE = 0.25
export const JTA_GOAL_TOLERANCE = 0.02
export const JTA_GOAL_TIME_TOLERANCE = 2.0

// applyControls, from ../../../reference/controls.js: every joint's
// accumulated per-frame .control delta is folded into its .angle and then
// zeroed. The one frame of indirection between "what the controller wants"
// and "the robot's configuration" is upstream KinEval's own structure.
export function applyControls(robot: RobotDesc) {
  for (const x in robot.joints) {
    robot.joints[x].angle += robot.joints[x].control || 0
    robot.joints[x].control = 0
  }
}

export function jointTrajectoryActionServer(robot: RobotDesc, options?: { gain?: number }): ActionServer {
  return {
    robot,
    gain: options && options.gain !== undefined ? options.gain : JTA_SERVO_GAIN,
    goal: null,
    state: null,
    time: 0,
    result: null,
    feedback: null,
    hold: null,
    stats: { goals: 0, canceled: 0, aborted: 0, succeeded: 0, rejected: 0 },
  }
}

export function jointTrajectoryActionSendGoal(
  server: ActionServer,
  goal: {
    trajectory: JointTrajectory
    path_tolerance?: number
    goal_tolerance?: number
    goal_time_tolerance?: number
  },
) {
  const problems = validateTrajectory(goal.trajectory)
  if (problems.length) {
    server.stats.rejected++
    server.result = { error_code: JTA_INVALID_GOAL, error_string: 'invalid goal: ' + problems.join('; ') }
    server.state = JTA_ABORTED
    return { accepted: false, result: server.result }
  }

  for (const name of goal.trajectory.joint_names) {
    if (!server.robot.joints[name]) {
      server.stats.rejected++
      server.result = {
        error_code: JTA_INVALID_JOINTS,
        error_string: `robot "${server.robot.name}" has no joint "${name}"`,
      }
      server.state = JTA_ABORTED
      return { accepted: false, result: server.result }
    }
  }

  if (server.goal && (server.state === JTA_EXECUTING || server.state === JTA_ACCEPTED)) {
    jointTrajectoryActionCancel(server, 'preempted by a new goal')
  }

  server.goal = {
    trajectory: goal.trajectory,
    path_tolerance: goal.path_tolerance !== undefined ? goal.path_tolerance : JTA_PATH_TOLERANCE,
    goal_tolerance: goal.goal_tolerance !== undefined ? goal.goal_tolerance : JTA_GOAL_TOLERANCE,
    goal_time_tolerance: goal.goal_time_tolerance !== undefined ? goal.goal_time_tolerance : JTA_GOAL_TIME_TOLERANCE,
  }
  server.state = JTA_ACCEPTED
  server.time = 0
  server.result = null
  server.feedback = null
  server.stats.goals++
  return { accepted: true }
}

export function jointTrajectoryActionCancel(server: ActionServer, reason?: string) {
  if (!server.goal) return null
  server.state = JTA_CANCELING
  server.hold = server.feedback ? server.feedback.desired.slice() : null
  server.result = {
    error_code: JTA_SUCCESSFUL,
    error_string: 'canceled: ' + (reason || 'no reason given'),
    canceled_at_time: server.time,
    canceled_at_segment: server.feedback ? server.feedback.segment : 0,
  }
  server.state = JTA_CANCELED
  server.goal = null
  server.stats.canceled++
  return server.result
}

export function jointTrajectoryActionUpdate(server: ActionServer, dt: number): ActionState | null {
  if (!server.goal) return server.state

  const goal = server.goal
  const trajectory = goal.trajectory
  const names = trajectory.joint_names

  if (server.state === JTA_ACCEPTED) server.state = JTA_EXECUTING

  server.time += dt
  const sample = sampleTrajectory(trajectory, server.time)

  for (let i = 0; i < names.length; i++) {
    const joint = server.robot.joints[names[i]]
    joint.control = (joint.control || 0) + server.gain * (sample.positions[i] - joint.angle)
  }
  applyControls(server.robot)

  const actual: number[] = []
  const error: number[] = []
  let worst = 0
  for (let i = 0; i < names.length; i++) {
    const angle = server.robot.joints[names[i]].angle
    actual.push(angle)
    const e = sample.positions[i] - angle
    error.push(e)
    worst = Math.max(worst, Math.abs(e))
  }

  const duration = trajectoryDuration(trajectory)
  server.feedback = {
    header: { stamp: server.time },
    joint_names: names.slice(),
    desired: sample.positions.slice(),
    actual,
    error,
    index: Math.min(sample.segment + 1, trajectory.points.length - 1),
    segment: sample.segment,
    alpha: sample.alpha,
    worst_error: worst,
    duration,
  }

  if (server.time < duration && worst > goal.path_tolerance) {
    const j = indexOfLargest(error)
    server.result = {
      error_code: JTA_PATH_TOLERANCE_VIOLATED,
      error_string: `joint "${names[j]}" is ${worst.toFixed(4)} rad from its desired position, ` +
        `past the ${goal.path_tolerance} rad path tolerance, at t = ${server.time.toFixed(2)}s`,
    }
    server.state = JTA_ABORTED
    server.goal = null
    server.stats.aborted++
    return server.state
  }

  if (server.time >= duration) {
    if (worst <= goal.goal_tolerance) {
      server.result = {
        error_code: JTA_SUCCESSFUL, error_string: '',
        finished_at_time: server.time, worst_error: worst,
      }
      server.state = JTA_SUCCEEDED
      server.goal = null
      server.stats.succeeded++
      return server.state
    }
    if (server.time > duration + goal.goal_time_tolerance) {
      server.result = {
        error_code: JTA_GOAL_TOLERANCE_VIOLATED,
        error_string: `still ${worst.toFixed(4)} rad from the final waypoint ` +
          `${goal.goal_time_tolerance}s after the trajectory ended`,
      }
      server.state = JTA_ABORTED
      server.goal = null
      server.stats.aborted++
      return server.state
    }
  }

  return server.state
}

export function jointTrajectoryActionHold(server: ActionServer) {
  if (server.goal || !server.hold) return
  const names = server.feedback ? server.feedback.joint_names : []
  for (let i = 0; i < names.length; i++) {
    const joint = server.robot.joints[names[i]]
    joint.control = (joint.control || 0) + server.gain * (server.hold[i] - joint.angle)
  }
  applyControls(server.robot)
}

function indexOfLargest(values: number[]): number {
  let best = 0
  for (let i = 1; i < values.length; i++) if (Math.abs(values[i]) > Math.abs(values[best])) best = i
  return best
}

export function robotAtConfiguration(
  robot: RobotDesc, jointNames: string[], positions: number[], tolerance = JTA_GOAL_TOLERANCE,
): boolean {
  for (let i = 0; i < jointNames.length; i++) {
    if (Math.abs(robot.joints[jointNames[i]].angle - positions[i]) > tolerance) return false
  }
  return true
}
