// Shared types for the forward-kinematics step tracer and its panels. Mirrors
// the plain-object shapes ../../../reference/kinematics.js and
// ../../../reference/robots/*.urdf.js use -- a TS port needs real types, the
// vanilla JS reference implementation deliberately doesn't.

export type Mat4 = number[][] // 4x4, row-major: m[row][col]
export type Quat = { a: number; b: number; c: number; d: number }

export interface JointDesc {
  parent: string
  child: string
  type: 'revolute' | 'continuous' | 'prismatic' | 'fixed'
  origin: { xyz: [number, number, number]; rpy: [number, number, number] }
  axis: [number, number, number]
  angle: number
}

export interface LinkDesc {
  geometry: { type: 'box' | 'cylinder' | 'sphere'; size: number[]; offset: { xyz: [number, number, number]; rpy: [number, number, number] } }
  color?: number
}

export interface RobotDesc {
  name: string
  origin: { xyz: [number, number, number]; rpy: [number, number, number] }
  base: string
  links: Record<string, LinkDesc>
  joints: Record<string, JointDesc>
  endeffector?: { frame: string; position: number[][] }
}

export type FKAction = 'build' | 'base' | 'link-enter' | 'joint-origin' | 'joint-motion' | 'joint-recurse'

export interface FKStep {
  line: number
  action: FKAction
  linkName?: string
  jointName?: string
  depth: number // matrix-stack depth at this step, for the push/pop visualization
  matrix?: Mat4 // the transform just computed, when action produces one
}
