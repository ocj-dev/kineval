// Shared types for the behavior-trees panels' TS ports. Mirrors the
// plain-object shapes ../../../reference/*.js use.
//
// WHY A TS PORT EXISTS AT ALL alongside the snippet-embedded originals:
// the reference implementation is a set of classic <script src> files
// defining globals, which is what lets it run with no build step. Vite
// cannot import a global-defining script as a module, so the panels carry
// a port. It is maintained by hand and kept structurally identical --
// same function names, same order, same control flow -- so that a slide
// can show the reference code and the panel beside it without the two
// being different algorithms. tests/smoke.cjs tests the reference; the
// deck's own verifier checks that every reference #region appears in the
// deck.
//
// One naming difference, inherited and deliberate: FK here writes each
// link's world transform to `.matrix` (the forward_kinematics/ and
// inverse_kinematics/ convention, since fk.ts is ported from that
// module), while the reference's snapshot of the viewer's kinematics.js
// writes `.xform`. See reference/scene.js#update-robot-meshes.

export type Mat4 = number[][] // 4x4, row-major: m[row][col]
export type Mat = number[][]
export type Quat = { a: number; b: number; c: number; d: number }

export interface JointDesc {
  parent: string
  child: string
  type?: 'revolute' | 'continuous' | 'prismatic' | 'fixed'
  origin: { xyz: [number, number, number]; rpy: [number, number, number] }
  axis: [number, number, number]
  angle: number
  matrix?: Mat4
  control?: number
}

export interface LinkDesc {
  geometry?: {
    type: 'box' | 'cylinder' | 'sphere'
    size: number[]
    offset: { xyz: [number, number, number]; rpy: [number, number, number] }
  }
  color?: number
  matrix?: Mat4
}

export interface RobotDesc {
  name: string
  origin: { xyz: [number, number, number]; rpy: [number, number, number] }
  base: string
  links: Record<string, LinkDesc>
  joints: Record<string, JointDesc>
  endeffector: { frame: string; position: number[][] }
}

//////////////////////////////////////////////////
/////   BEHAVIOR TREE
//////////////////////////////////////////////////

export type Status = 'success' | 'failure' | 'running'
export type NodeKind = 'sequence' | 'fallback' | 'parallel' | 'decorator' | 'condition' | 'action'
export type DecoratorPolicy = 'invert' | 'force-success' | 'repeat-until-success' | 'retry'

export interface Blackboard {
  now: number
  dt: number
  log: string[]
  [key: string]: any
}

export interface BtNode {
  kind: NodeKind
  name: string
  children: BtNode[]
  status?: Status
  state: {
    current?: number
    attempts?: number
    started?: boolean
    childStatus?: (Status | undefined)[]
  }
  memory?: boolean
  successThreshold?: number
  policy?: DecoratorPolicy
  limit?: number
  predicate?: (bb: Blackboard) => boolean | Status
  start?: ((bb: Blackboard) => void) | null
  step?: (bb: Blackboard) => Status
  halt?: ((bb: Blackboard) => void) | null
  reads?: string[]
  writes?: string[]
}

export interface TraceEntry {
  path: string
  name: string
  kind: NodeKind
  status: Status
}

export interface WalkEntry {
  node: BtNode
  path: string
  depth: number
  index: number
  kind: NodeKind
  name: string
  childCount: number
}

//////////////////////////////////////////////////
/////   TRAJECTORY AND ACTION
//////////////////////////////////////////////////

export interface TrajectoryPoint {
  positions: number[]
  velocities: number[]
  accelerations: number[]
  effort: number[]
  time_from_start: number
}

export interface JointTrajectory {
  header: { stamp: number; frame_id: string }
  joint_names: string[]
  points: TrajectoryPoint[]
  provenance?: Record<string, unknown>
}

export interface PlanRecord {
  provenance?: { generated_by: string; [key: string]: unknown }
  robot: string
  seed: number
  q_layout: string[]
  joint_names: string[]
  world?: { boundary: number[][]; obstacles: { location: number[][]; radius: number }[] }
  search?: {
    iterations: number
    collision_checks: number
    nearest_neighbor_comparisons: number
    step_size: number
    raw_path_length: number
    smoothed_path_length: number
  }
  raw_path?: number[][]
  smoothed_path: number[][]
  smoothed_positions?: number[][]
}

export type ActionState = 'accepted' | 'executing' | 'canceling' | 'succeeded' | 'canceled' | 'aborted'

export interface ActionFeedback {
  header: { stamp: number }
  joint_names: string[]
  desired: number[]
  actual: number[]
  error: number[]
  index: number
  segment: number
  alpha: number
  worst_error: number
  duration: number
}

export interface ActionResult {
  error_code: number
  error_string: string
  finished_at_time?: number
  worst_error?: number
  canceled_at_time?: number
  canceled_at_segment?: number
}

export interface ActionServer {
  robot: RobotDesc
  gain: number
  goal: {
    trajectory: JointTrajectory
    path_tolerance: number
    goal_tolerance: number
    goal_time_tolerance: number
  } | null
  state: ActionState | null
  time: number
  result: ActionResult | null
  feedback: ActionFeedback | null
  hold?: number[] | null
  stats: { goals: number; canceled: number; aborted: number; succeeded: number; rejected: number }
}
