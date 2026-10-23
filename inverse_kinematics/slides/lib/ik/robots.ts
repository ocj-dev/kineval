// TS mirrors of ../../../reference/robots/urdf_example.js and
// robot_mr2.js, plus the chain/initial-pose/default-target tables
// ../../../reference/infrastructure.js keeps -- same numbers, same
// structure, kept in sync by hand (see matrixMath.ts's header for why a TS
// port exists at all alongside the snippet-embedded originals).
//
// robot_mr2 here is reduced to plain {type:'box',size,offset} link
// descriptors (this module's own geometry-as-plain-data convention, same as
// urdf_example.js/mobile_arm.urdf.js) instead of the reference's literal
// THREE.BoxGeometry calls, since these panels build their own three.js
// meshes from plain data -- same translate() numbers, just expressed as
// data instead of an imperative THREE.BoxGeometry().translate() call.
import type { RobotDesc } from './types'

export function createUrdfExampleRobot(): RobotDesc {
  return {
    name: 'urdf_example',
    origin: { xyz: [0, 0.1, 0], rpy: [0, 0, 0] },
    base: 'link1',
    links: {
      link1: { geometry: { type: 'box', size: [0.9, 0.7, 0.2], offset: { xyz: [0.15, 0.25, 0], rpy: [0, 0, 0] } }, color: 0x3b6ea5 },
      link2: { geometry: { type: 'box', size: [0.7, 0.2, 0.2], offset: { xyz: [0.25, 0, 0], rpy: [0, 0, 0] } }, color: 0xd98236 },
      link3: { geometry: { type: 'box', size: [0.7, 0.2, 0.2], offset: { xyz: [0.25, 0, 0], rpy: [0, 0, 0] } }, color: 0x27966b },
      link4: { geometry: { type: 'box', size: [0.5, 0.2, 0.2], offset: { xyz: [0.15, 0, 0], rpy: [0, 0, 0] } }, color: 0xc0392b },
    },
    joints: {
      joint1: { parent: 'link1', child: 'link2', type: 'revolute', origin: { xyz: [0.5, 0.3, 0.0], rpy: [0, 0, 0] }, axis: [-1.0, 0.0, 0.0], angle: 0 },
      joint2: { parent: 'link1', child: 'link3', type: 'revolute', origin: { xyz: [-0.2, 0.5, 0.0], rpy: [0, 0, Math.PI / 2] }, axis: [-Math.cos(Math.PI / 4), Math.cos(Math.PI / 4), 0], angle: 0 },
      joint3: { parent: 'link3', child: 'link4', type: 'revolute', origin: { xyz: [0.5, 0.0, 0.0], rpy: [0, 0, -Math.PI / 2] }, axis: [Math.cos(Math.PI / 4), -Math.cos(Math.PI / 4), 0], angle: 0 },
    },
    endeffector: { frame: 'joint3', position: [[0.5], [0], [0], [1]] },
  }
}

// Upstream's own right-arm-only humanoid stencil exercise -- clavicle_left
// is a disconnected sibling branch off `base` (left arm deliberately
// unfinished upstream), excluded from every chain below. See
// reference/robots/robot_mr2.js's own header.
export function createRobotMr2(): RobotDesc {
  return {
    name: 'mr2',
    origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
    base: 'base',
    links: {
      base: { geometry: { type: 'box', size: [1, 0.4, 1], offset: { xyz: [0, 0.2, 0], rpy: [0, 0, 0] } }, color: 0x3b6ea5 },
      clavicle_right: { geometry: { type: 'box', size: [0.3, 0.3, 1], offset: { xyz: [0, 0, 0.5], rpy: [0, 0, 0] } }, color: 0x27966b },
      clavicle_left: { geometry: { type: 'box', size: [0.3, 0.3, 1], offset: { xyz: [0, 0, 0.5], rpy: [0, 0, 0] } }, color: 0x8a8f98 },
      shoulder_right: { geometry: { type: 'box', size: [0.3, 0.3, 0.7], offset: { xyz: [0, 0, 0.35], rpy: [0, 0, 0] } }, color: 0xd98236 },
      upperarm_right: { geometry: { type: 'box', size: [0.3, 0.3, 0.7], offset: { xyz: [0, 0, 0.35], rpy: [0, 0, 0] } }, color: 0x27966b },
      forearm_right: { geometry: { type: 'box', size: [0.3, 0.3, 0.5], offset: { xyz: [0, 0, 0.25], rpy: [0, 0, 0] } }, color: 0xc0392b },
    },
    joints: {
      clavicle_right_yaw: { parent: 'base', child: 'clavicle_right', type: 'revolute', origin: { xyz: [0.3, 0.4, 0.0], rpy: [-Math.PI / 2, 0, 0] }, axis: [0.0, 0.0, -1.0], angle: 0 },
      shoulder_right_yaw: { parent: 'clavicle_right', child: 'shoulder_right', type: 'revolute', origin: { xyz: [0.0, -0.15, 0.85], rpy: [Math.PI / 2, 0, 0] }, axis: [0.0, 0.707, 0.707], angle: 0 },
      upperarm_right_pitch: { parent: 'shoulder_right', child: 'upperarm_right', type: 'revolute', origin: { xyz: [0.0, 0.0, 0.7], rpy: [0, 0, 0] }, axis: [0.0, 1.0, 0.0], angle: 0 },
      forearm_right_yaw: { parent: 'upperarm_right', child: 'forearm_right', type: 'revolute', origin: { xyz: [0.0, 0.0, 0.7], rpy: [0, 0, 0] }, axis: [1.0, 0.0, 0.0], angle: 0 },
      clavicle_left_roll: { parent: 'base', child: 'clavicle_left', type: 'revolute', origin: { xyz: [-0.3, 0.4, 0.0], rpy: [-Math.PI / 2, 0, 0] }, axis: [0.0, 0.0, 1.0], angle: 0 },
    },
    endeffector: { frame: 'forearm_right_yaw', position: [[0], [0], [0.5], [1]] },
  }
}

export type RobotName = 'urdf_example' | 'mr2'

export const ROBOT_FACTORIES: Record<RobotName, () => RobotDesc> = {
  urdf_example: createUrdfExampleRobot,
  mr2: createRobotMr2,
}

// The ordered base-to-end-effector chain of ACTUAL ANCESTOR joints driving
// each robot's end-effector -- see infrastructure.js#appendix-url-params's
// own CHAIN_JOINT_NAMES comment for why joint1 (urdf_example) and
// clavicle_left_roll (mr2) are excluded: neither is an ancestor of the
// end-effector frame, so a Jacobian column for either would be identically
// zero.
export const CHAIN_JOINT_NAMES: Record<RobotName, string[]> = {
  urdf_example: ['joint2', 'joint3'],
  mr2: ['clavicle_right_yaw', 'shoulder_right_yaw', 'upperarm_right_pitch', 'forearm_right_yaw'],
}

export const INITIAL_ANGLES: Record<RobotName, Record<string, number>> = {
  urdf_example: { joint2: 0.3, joint3: -0.5 },
  mr2: { clavicle_right_yaw: 0.2, shoulder_right_yaw: 0.3, upperarm_right_pitch: 0.4, forearm_right_yaw: -0.3 },
}

export const DEFAULT_TARGETS: Record<RobotName, [number, number, number]> = {
  urdf_example: [0.7, 0.75, 0.25],
  mr2: [0.4, 1.15, 0.45],
}

export function edgesOf(robot: RobotDesc): { parent: string; child: string }[] {
  return Object.values(robot.joints).map((j) => ({ parent: j.parent, child: j.child }))
}

export function initJointAngles(robot: RobotDesc, robotName: RobotName) {
  const initial = INITIAL_ANGLES[robotName]
  for (const n in initial) robot.joints[n].angle = initial[n]
  for (const n in robot.joints) if (robot.joints[n].control === undefined) robot.joints[n].control = 0
}
