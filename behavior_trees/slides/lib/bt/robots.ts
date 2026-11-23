// TS mirrors of ../../../reference/robots/robot_mr2.js and
// urdf_example.js -- same numbers, same structure, kept in sync by hand.
// See types.ts's header for why a port exists at all.
import type { RobotDesc } from './types'

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
      clavicle_right_yaw: { parent: 'base', child: 'clavicle_right', origin: { xyz: [0.3, 0.4, 0.0], rpy: [-Math.PI / 2, 0, 0] }, axis: [0.0, 0.0, -1.0], angle: 0, control: 0 },
      shoulder_right_yaw: { parent: 'clavicle_right', child: 'shoulder_right', origin: { xyz: [0.0, -0.15, 0.85], rpy: [Math.PI / 2, 0, 0] }, axis: [0.0, 0.707, 0.707], angle: 0, control: 0 },
      upperarm_right_pitch: { parent: 'shoulder_right', child: 'upperarm_right', origin: { xyz: [0.0, 0.0, 0.7], rpy: [0, 0, 0] }, axis: [0.0, 1.0, 0.0], angle: 0, control: 0 },
      forearm_right_yaw: { parent: 'upperarm_right', child: 'forearm_right', origin: { xyz: [0.0, 0.0, 0.7], rpy: [0, 0, 0] }, axis: [1.0, 0.0, 0.0], angle: 0, control: 0 },
      clavicle_left_roll: { parent: 'base', child: 'clavicle_left', origin: { xyz: [-0.3, 0.4, 0.0], rpy: [-Math.PI / 2, 0, 0] }, axis: [0.0, 0.0, 1.0], angle: 0, control: 0 },
    },
    endeffector: { frame: 'forearm_right_yaw', position: [[0], [0], [0.5], [1]] },
  }
}

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
      joint1: { parent: 'link1', child: 'link2', type: 'revolute', origin: { xyz: [0.5, 0.3, 0.0], rpy: [0, 0, 0] }, axis: [-1.0, 0.0, 0.0], angle: 0, control: 0 },
      joint2: { parent: 'link1', child: 'link3', type: 'revolute', origin: { xyz: [-0.2, 0.5, 0.0], rpy: [0, 0, Math.PI / 2] }, axis: [-Math.cos(Math.PI / 4), Math.cos(Math.PI / 4), 0], angle: 0, control: 0 },
      joint3: { parent: 'link3', child: 'link4', type: 'revolute', origin: { xyz: [0.5, 0.0, 0.0], rpy: [0, 0, -Math.PI / 2] }, axis: [Math.cos(Math.PI / 4), -Math.cos(Math.PI / 4), 0], angle: 0, control: 0 },
    },
    endeffector: { frame: 'joint3', position: [[0.5], [0], [0], [1]] },
  }
}

// The joint_names of every trajectory this module builds for each robot,
// in order. The order is load-bearing: points[i].positions[j] means
// joint_names[j], so names and numbers are only correct together.
export const MR2_ARM_CHAIN = [
  'clavicle_right_yaw', 'shoulder_right_yaw', 'upperarm_right_pitch', 'forearm_right_yaw',
]
export const URDF_EXAMPLE_ARM_CHAIN = ['joint2', 'joint3']

export const ROBOT_FACTORIES: Record<string, () => RobotDesc> = {
  mr2: createRobotMr2,
  urdf_example: createUrdfExampleRobot,
}
