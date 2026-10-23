// Shared types for the inverse-kinematics panels' TS ports. Mirrors the
// plain-object shapes ../../../reference/inverse_kinematics.js,
// kinematics.js and robots/*.js use -- a TS port needs real types, the
// vanilla JS reference implementation deliberately doesn't (see
// forward_kinematics/slides/lib/fk/types.ts's own header for why these
// ports exist alongside the snippet-embedded originals at all).

export type Mat4 = number[][] // 4x4, row-major: m[row][col]
export type Mat = number[][] // general 2D array, row-major: m[row][col]
export type Quat = { a: number; b: number; c: number; d: number }

export interface JointDesc {
  parent: string
  child: string
  type: 'revolute' | 'continuous' | 'prismatic' | 'fixed'
  origin: { xyz: [number, number, number]; rpy: [number, number, number] }
  axis: [number, number, number]
  angle: number
  matrix?: Mat4 // filled in by buildFK, same as reference kinematics.js's joint.matrix
  control?: number
}

export interface LinkDesc {
  geometry?: { type: 'box' | 'cylinder' | 'sphere'; size: number[]; offset: { xyz: [number, number, number]; rpy: [number, number, number] } }
  color?: number
  matrix?: Mat4
  children?: string[]
  parent?: string | null
}

export interface RobotDesc {
  name: string
  origin: { xyz: [number, number, number]; rpy: [number, number, number] }
  base: string
  links: Record<string, LinkDesc>
  joints: Record<string, JointDesc>
  endeffector: { frame: string; position: number[][] } // 4x1 homogeneous local point
  // the 3 values iterateIK is graded against (see its own [NOTICE] comment)
  dx?: number[][]
  jacobian?: number[][]
  dq?: number[][]
}

// One column of the manipulator Jacobian under construction -- one joint's
// contribution, exactly the quantities computeManipulatorJacobian's loop
// body computes (../../../reference/inverse_kinematics.js#manipulator-jacobian).
export interface JacobianColumnStep {
  line: number
  jointIndex: number
  jointName: string
  axisWorld: [number, number, number] // z_i
  originWorld: [number, number, number] // p_i
  eeWorld: [number, number, number] // p_ee (fixed for the whole build)
  linear: [number, number, number] // z_i x (p_ee - p_i)
  angular: [number, number, number] // z_i (only meaningful if orientation included)
  columnsSoFar: number[][] // each entry a [lin0,lin1,lin2,(ang0,ang1,ang2)] column, in joint order
}

export type IkMethod = 'transpose' | 'pseudoinverse' | 'damped'

export interface IkParams {
  method: IkMethod
  steplength: number
  damping: number
  orientationIncluded: boolean
  chainJointNames: string[]
}

// One step of the iterateIK step-through panel -- mirrors iterateIK's own
// sequence: gate check, error term, Jacobian, solve, scale+apply.
export type IterateIkAction = 'gate' | 'error' | 'jacobian' | 'transpose' | 'solve' | 'scale-apply'
export interface IterateIkStep {
  line: number
  action: IterateIkAction
  dx?: number[][]
  jacobian?: number[][]
  jacobianT?: number[][]
  dq?: number[][]
  note?: string
}
