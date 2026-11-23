// A TS port of ../../../reference/trees.js. See types.ts's header for why
// a port exists; the reference file carries the reasoning for each tree.
import type { ActionServer, BtNode, JointTrajectory, PlanRecord } from './types'
import { btDecorator, btFallback, btParallel, btSequence } from './behaviorTree'
import {
  taskAtLocation, taskAtTargetLocation, taskAtTrajectoryEnd, taskBatteryAbove, taskCharge,
  taskFindObject, taskFindObjectHere, taskFollowTrajectory, taskInspect, taskNavigateTo,
  taskNavigateToTarget, taskNextInspectionTarget, taskNextLocation, taskPlanMotion,
} from './tasks'

export function treeSearchNaive(): BtNode {
  return btSequence('Search the kitchen', [
    taskNavigateTo('kitchen'),
    taskFindObject('apple'),
  ])
}

export function treeSearchExplicitSuccess(): BtNode {
  return btSequence('Search the kitchen', [
    btFallback('Be at the kitchen', [taskAtLocation('kitchen'), taskNavigateTo('kitchen')]),
    taskFindObject('apple'),
  ])
}

export function treeSearchMultipleLocations(locations?: string[]): BtNode {
  const places = locations || ['kitchen', 'pantry', 'garage']
  return btFallback('Search everywhere', places.map((place) =>
    btSequence(`Search the ${place}`, [
      btFallback(`Be at the ${place}`, [taskAtLocation(place), taskNavigateTo(place)]),
      taskFindObject('apple'),
    ])))
}

export function treeSearchParallel(threshold = 1): BtNode {
  return btSequence('Search the kitchen for fruit', [
    btFallback('Be at the kitchen', [taskAtLocation('kitchen'), taskNavigateTo('kitchen')]),
    btParallel('Apple or orange', [taskFindObject('apple'), taskFindObject('orange')], threshold),
  ])
}

export function treeSearchWithDecorator(locationCount = 3): BtNode {
  return btDecorator('Try every location', 'repeat-until-success',
    btSequence('Search one location', [
      taskNextLocation(),
      btFallback('Be at the target', [taskAtTargetLocation(), taskNavigateToTarget()]),
      taskFindObjectHere(),
    ]),
    { limit: locationCount })
}

export function treePlanAndExecute(server: ActionServer, plan: PlanRecord, jointNames?: string[]): BtNode {
  return btFallback('Reach the goal configuration', [
    taskAtTrajectoryEnd(server),
    btSequence('Plan and execute', [
      taskPlanMotion(plan, jointNames),
      taskFollowTrajectory(server),
    ]),
  ])
}

export function treeInspectionQueue(
  server: ActionServer, trajectoriesByTarget: Record<string, JointTrajectory>,
): BtNode {
  const count = Object.keys(trajectoriesByTarget).length
  return btDecorator('Inspect every target', 'repeat-until-success',
    btSequence('Inspect one target', [
      taskNextInspectionTarget(trajectoriesByTarget),
      btFallback('Be at the inspection pose', [taskAtTrajectoryEnd(server), taskFollowTrajectory(server)]),
      taskInspect(),
    ]),
    { limit: count })
}

export function treeReactiveBattery(
  server: ActionServer, plan: PlanRecord, jointNames?: string[], options?: { reactive?: boolean },
): BtNode {
  const reactive = !options || options.reactive !== false
  return btFallback('Work, or charge', [
    btSequence('Work while charged', [
      taskBatteryAbove(20),
      btFallback('Reach the goal configuration', [
        taskAtTrajectoryEnd(server),
        btSequence('Plan and execute', [
          taskPlanMotion(plan, jointNames),
          taskFollowTrajectory(server),
        ]),
      ]),
    ], { memory: !reactive }),
    taskCharge(),
  ])
}
