// TS mirrors of ../../../reference/robots/mobile_arm.urdf.js and
// urdf_example.js -- same numbers, same structure, kept in sync by hand
// (see matrixMath.ts's header for why a TS port exists at all alongside
// the snippet-embedded originals). fkSteps.ts doesn't need the
// link.children/link.parent bookkeeping kinematics.js's initRobot builds
// (it derives each link's child joints with a filter, on the fly), so
// these are plain RobotDesc literals, nothing to initialize.
import type { RobotDesc } from './types'

export function createMobileArmRobot(): RobotDesc {
  return {
    name: 'mobile_arm',
    origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
    base: 'base_link',
    links: {
      base_link: { geometry: { type: 'box', size: [0.6, 0.2, 0.6], offset: { xyz: [0, 0, 0], rpy: [0, 0, 0] } }, color: 0x3b6ea5 },
      mast_link: { geometry: { type: 'box', size: [0.12, 0.3, 0.12], offset: { xyz: [0, 0.15, 0], rpy: [0, 0, 0] } }, color: 0x27966b },
      arm_link1: { geometry: { type: 'box', size: [0.5, 0.08, 0.08], offset: { xyz: [0.25, 0, 0], rpy: [0, 0, 0] } }, color: 0xd98236 },
      arm_link2: { geometry: { type: 'box', size: [0.4, 0.06, 0.06], offset: { xyz: [0.2, 0, 0], rpy: [0, 0, 0] } }, color: 0xc0392b },
    },
    // all three joints turn about the lateral +z axis -- the arm is planar in
    // a VERTICAL plane, and joint_pitch tilts that whole plane
    joints: {
      joint_pitch: { parent: 'base_link', child: 'mast_link', type: 'revolute', origin: { xyz: [0, 0.1, 0], rpy: [0, 0, 0] }, axis: [0, 0, 1], angle: 0 },
      joint1: { parent: 'mast_link', child: 'arm_link1', type: 'revolute', origin: { xyz: [0, 0.3, 0], rpy: [0, 0, 0] }, axis: [0, 0, 1], angle: 0 },
      joint2: { parent: 'arm_link1', child: 'arm_link2', type: 'revolute', origin: { xyz: [0.5, 0, 0], rpy: [0, 0, 0] }, axis: [0, 0, 1], angle: 0 },
    },
    endeffector: { frame: 'joint2', position: [[0.4], [0], [0], [1]] },
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
      joint1: { parent: 'link1', child: 'link2', type: 'revolute', origin: { xyz: [0.5, 0.3, 0.0], rpy: [0, 0, 0] }, axis: [-1.0, 0.0, 0.0], angle: 0 },
      joint2: { parent: 'link1', child: 'link3', type: 'revolute', origin: { xyz: [-0.2, 0.5, 0.0], rpy: [0, 0, Math.PI / 2] }, axis: [-Math.cos(Math.PI / 4), Math.cos(Math.PI / 4), 0], angle: 0 },
      joint3: { parent: 'link3', child: 'link4', type: 'revolute', origin: { xyz: [0.5, 0.0, 0.0], rpy: [0, 0, -Math.PI / 2] }, axis: [Math.cos(Math.PI / 4), -Math.cos(Math.PI / 4), 0], angle: 0 },
    },
    endeffector: { frame: 'joint3', position: [[0.5], [0], [0], [1]] },
  }
}

export function edgesOf(robot: RobotDesc): { parent: string; child: string }[] {
  return Object.values(robot.joints).map((j) => ({ parent: j.parent, child: j.child }))
}
