// A TS port of ../../../reference/bt_tasks.js. See types.ts's header for
// why a port exists; the reference file carries the reasoning.
import type { ActionServer, Blackboard, BtNode, JointTrajectory, PlanRecord } from './types'
import { BT_FAILURE, BT_RUNNING, BT_SUCCESS, btAction, btCondition } from './behaviorTree'
import { trajectoryDuration, trajectoryFromPlan } from './trajectory'
import {
  JTA_ACCEPTED, JTA_EXECUTING, JTA_SUCCEEDED,
  jointTrajectoryActionCancel, jointTrajectoryActionSendGoal, jointTrajectoryActionUpdate,
  robotAtConfiguration,
} from './action'

export const SEARCH_NAVIGATION_TICKS = 3
export const PLAN_ITERATIONS_PER_TICK = 8

//////////////////////////////////////////////////
/////   THE SEARCH TASKS
//////////////////////////////////////////////////

export function taskAtLocation(location: string): BtNode {
  const node = btCondition(`At ${location}?`, (bb) => bb.robot_location === location)
  node.reads = ['robot_location']
  node.writes = []
  return node
}

export function taskNavigateTo(location: string): BtNode {
  const node = btAction(`Navigate to ${location}`, {
    start: (bb) => {
      bb.nav_ticks = 0
      bb.nav_goal = location
      bb.robot_location = null
      bb.log.push('navigate: heading for ' + location)
    },
    step: (bb) => {
      bb.nav_ticks++
      if (bb.nav_ticks < SEARCH_NAVIGATION_TICKS) return BT_RUNNING
      bb.robot_location = location
      bb.log.push('navigate: arrived at ' + location)
      return BT_SUCCESS
    },
    halt: (bb) => {
      bb.log.push('navigate: halted en route to ' + location)
      bb.nav_goal = null
    },
  })
  node.reads = ['robot_location']
  node.writes = ['robot_location', 'nav_ticks', 'nav_goal']
  return node
}

export function taskFindObject(object: string): BtNode {
  const node = btCondition(`Found ${object}?`, (bb) => {
    const here = bb.robot_location
    const objects = (bb.objects_at || {})[here] || []
    const found = objects.indexOf(object) !== -1
    bb.log.push(`look for ${object} at ${here}: ` + (found ? 'found' : 'not here'))
    if (found) bb.found = (bb.found || []).concat([object])
    return found
  })
  node.reads = ['robot_location', 'objects_at']
  node.writes = ['found']
  return node
}

export function taskNextLocation(): BtNode {
  const node = btAction('Get next location', {
    step: (bb) => {
      const queue: string[] = bb.location_queue || []
      if (!queue.length) {
        bb.log.push('get next location: queue empty')
        return BT_FAILURE
      }
      bb.target_location = queue.shift()
      bb.log.push('get next location: ' + bb.target_location)
      return BT_SUCCESS
    },
  })
  node.reads = ['location_queue']
  node.writes = ['target_location']
  return node
}

export function taskAtTargetLocation(): BtNode {
  const node = btCondition('At target?', (bb) => bb.robot_location === bb.target_location)
  node.reads = ['robot_location', 'target_location']
  node.writes = []
  return node
}

export function taskNavigateToTarget(): BtNode {
  const node = btAction('Navigate to target', {
    start: (bb) => {
      bb.nav_ticks = 0
      bb.robot_location = null
      bb.log.push('navigate: heading for ' + bb.target_location)
    },
    step: (bb) => {
      bb.nav_ticks++
      if (bb.nav_ticks < SEARCH_NAVIGATION_TICKS) return BT_RUNNING
      bb.robot_location = bb.target_location
      bb.log.push('navigate: arrived at ' + bb.robot_location)
      return BT_SUCCESS
    },
    halt: (bb) => { bb.log.push('navigate: halted en route to ' + bb.target_location) },
  })
  node.reads = ['target_location']
  node.writes = ['robot_location', 'nav_ticks']
  return node
}

export function taskFindObjectHere(): BtNode {
  const node = btCondition('Found the object?', (bb) => {
    const here = bb.robot_location
    const objects = (bb.objects_at || {})[here] || []
    const found = objects.indexOf(bb.wanted_object) !== -1
    bb.log.push(`look for ${bb.wanted_object} at ${here}: ` + (found ? 'found' : 'not here'))
    if (found) bb.found = (bb.found || []).concat([bb.wanted_object])
    return found
  })
  node.reads = ['robot_location', 'objects_at', 'wanted_object']
  node.writes = ['found']
  return node
}

//////////////////////////////////////////////////
/////   THE MOTION TASKS
//////////////////////////////////////////////////

export function taskPlanMotion(plan: PlanRecord, jointNames?: string[]): BtNode {
  const node = btAction('Plan motion', {
    start: (bb) => {
      bb.plan_iterations = 0
      bb.log.push(`plan: replaying a recorded RRT-Connect plan (${plan.search!.iterations} iterations, ` +
        `${plan.search!.smoothed_path_length} waypoints)`)
    },
    step: (bb) => {
      bb.plan_iterations += PLAN_ITERATIONS_PER_TICK
      if (bb.plan_iterations < plan.search!.iterations) return BT_RUNNING
      const trajectory = trajectoryFromPlan(plan, jointNames)
      if (!trajectory) {
        bb.log.push('plan: FAILED to build a trajectory from the plan')
        return BT_FAILURE
      }
      bb.trajectory = trajectory
      bb.log.push(`plan: trajectory with ${trajectory.points.length} points, ` +
        `${trajectoryDuration(trajectory).toFixed(2)}s long`)
      return BT_SUCCESS
    },
    halt: (bb) => { bb.log.push('plan: halted after ' + bb.plan_iterations + ' iterations') },
  })
  node.reads = []
  node.writes = ['trajectory', 'plan_iterations']
  return node
}

export function taskFollowTrajectory(server: ActionServer): BtNode {
  const node = btAction('Follow joint trajectory', {
    start: (bb) => {
      const trajectory: JointTrajectory | undefined = bb.trajectory
      if (!trajectory) { bb.goal_accepted = false; return }
      const reply = jointTrajectoryActionSendGoal(server, { trajectory })
      bb.goal_accepted = reply.accepted
      bb.log.push('execute: goal ' +
        (reply.accepted ? 'accepted' : 'REJECTED -- ' + reply.result!.error_string))
    },
    step: (bb) => {
      if (!bb.trajectory) {
        bb.log.push('execute: nothing on the blackboard to execute')
        return BT_FAILURE
      }
      if (bb.goal_accepted === false) {
        bb.last_result = server.result
        return BT_FAILURE
      }

      const state = jointTrajectoryActionUpdate(server, bb.dt)
      bb.action_state = state
      bb.action_feedback = server.feedback

      if (state === JTA_EXECUTING || state === JTA_ACCEPTED) return BT_RUNNING

      bb.last_result = server.result
      if (state === JTA_SUCCEEDED) {
        bb.log.push(`execute: succeeded at t = ${server.result!.finished_at_time!.toFixed(2)}s, ` +
          `worst error ${server.result!.worst_error!.toFixed(4)} rad`)
        return BT_SUCCESS
      }
      bb.log.push(`execute: ${state} -- ${server.result!.error_string}`)
      return BT_FAILURE
    },
    halt: (bb) => {
      const result = jointTrajectoryActionCancel(server, 'behavior tree halted the leaf')
      if (result) {
        bb.last_result = result
        bb.log.push(`execute: canceled at t = ${result.canceled_at_time!.toFixed(2)}s, ` +
          `in segment ${result.canceled_at_segment}`)
      }
    },
  })
  node.reads = ['trajectory']
  node.writes = ['action_state', 'action_feedback', 'last_result', 'goal_accepted']
  return node
}

export function taskAtTrajectoryEnd(server: ActionServer): BtNode {
  const node = btCondition('At the goal configuration?', (bb) => {
    const trajectory: JointTrajectory | undefined = bb.trajectory
    if (!trajectory) return false
    const last = trajectory.points[trajectory.points.length - 1]
    return robotAtConfiguration(server.robot, trajectory.joint_names, last.positions)
  })
  node.reads = ['trajectory']
  node.writes = []
  return node
}

export function taskInspect(): BtNode {
  const node = btCondition('Part present?', (bb) => {
    const target = bb.inspection_target
    const present = (bb.parts_present || {})[target] === true
    bb.log.push(`inspect ${target}: ` + (present ? 'part present' : 'empty'))
    bb.inspected = (bb.inspected || []).concat([target])
    return present
  })
  node.reads = ['inspection_target', 'parts_present']
  node.writes = ['inspected']
  return node
}

export function taskNextInspectionTarget(targets: Record<string, JointTrajectory>): BtNode {
  const node = btAction('Get next inspection target', {
    step: (bb) => {
      if (!bb.target_queue) bb.target_queue = Object.keys(targets).slice()
      if (!bb.target_queue.length) {
        bb.log.push('targets: queue empty, nothing left to inspect')
        return BT_FAILURE
      }
      const name = bb.target_queue.shift()
      bb.inspection_target = name
      bb.trajectory = targets[name]
      bb.log.push(`targets: next is ${name} (${targets[name].points.length}-point trajectory)`)
      return BT_SUCCESS
    },
  })
  node.reads = ['target_queue']
  node.writes = ['inspection_target', 'trajectory', 'target_queue']
  return node
}

export function taskBatteryAbove(threshold: number): BtNode {
  const node = btCondition(`Battery above ${threshold}%?`, (bb) => {
    const level = bb.battery === undefined ? 100 : bb.battery
    return level > threshold
  })
  node.reads = ['battery']
  node.writes = []
  return node
}

export function simulateBattery(bb: Blackboard, drainPerTick = 0.6) {
  if (bb.battery === undefined) bb.battery = 100
  if (bb.charging) return
  bb.battery = Math.max(0, bb.battery - drainPerTick)
}

export function taskCharge(ratePerTick = 12): BtNode {
  const node = btAction('Go charge', {
    start: (bb) => {
      bb.charging = true
      bb.log.push('charge: battery at ' + bb.battery.toFixed(1) + '%, docking')
    },
    step: (bb) => {
      bb.battery = Math.min(100, bb.battery + ratePerTick)
      if (bb.battery < 100) return BT_RUNNING
      bb.charging = false
      bb.log.push('charge: full')
      return BT_SUCCESS
    },
    halt: (bb) => {
      bb.charging = false
      bb.log.push('charge: halted at ' + bb.battery.toFixed(1) + '%')
    },
  })
  node.reads = ['battery']
  node.writes = ['battery', 'charging']
  return node
}
