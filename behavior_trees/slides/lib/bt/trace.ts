// Replay types for the recorded behavior-tree traces.
//
// THESE TRACES ARE REAL RECORDINGS, not animation data written for the
// slides. Each was emitted by `tests/bt_node_motion.rs` in
// pubsub_testbed_rs, carries its own header naming the command, the
// commit and the robot, and is carried into this module unmodified under
// ../../reference/traces/. The panels replay them; they do not simulate
// anything. That distinction is the one thing a reader downstream cannot
// check for themselves, so it is worth stating plainly: if a panel here
// shows a tick, that tick happened.
//
// The JSON is IMPORTED rather than fetched. A runtime `fetch()` of a
// trace would be the most likely thing in this deck to work locally and
// 404 under the real `/kineval/behavior_trees/` base path -- the plan
// that commissioned this deck flags exactly that risk. An import is
// resolved by Vite at build time and bundled, so the failure mode cannot
// occur.

import planAndExecute from '../../../reference/traces/bt_motion_plan_and_execute.json'
import preempt from '../../../reference/traces/bt_motion_preempt.json'

export type TickResult = 'success' | 'failure' | 'running'

/** One node as recorded in the trace's own tree snapshot. */
export interface TreeNode {
  type: string
  children?: TreeNode[]
  child?: TreeNode
  // leaf payloads, present per kind
  goal?: number[]
  start?: number[]
  positions?: number[]
  output_key?: string
  input_key?: string
  check?: string
  epsilon?: number
  timeout?: number
  seed?: number
  max_iterations?: number
  count?: number
  invert?: boolean
}

/** One node the tick actually reached, with the result it returned. */
export interface VisitedNode {
  /** child indices from the root; [] is the root itself */
  path: number[]
  type: string
  result: TickResult
}

export interface Tick {
  tick: number
  root_result: TickResult
  visited: VisitedNode[]
  running_paths: number[][]
  blackboard: Record<string, unknown>
  blackboard_keys: string[]
}

export interface TraceHeader {
  command: string
  commit: string
  date: string
  produced_by: string
  robot: string
  scenario: string
  schema: string
  tree: TreeNode
}

export interface Trace {
  header: TraceHeader
  ticks: Tick[]
}

export const TRACES: Record<string, Trace> = {
  plan_and_execute: planAndExecute as unknown as Trace,
  preempt: preempt as unknown as Trace,
}

// ------------------------------------------------------------- flattening

/** A node of the tree, flattened for drawing, with its path and depth. */
export interface FlatNode {
  path: number[]
  depth: number
  node: TreeNode
  /** a short label: the kind, plus the blackboard key it reads or writes */
  label: string
  /** the row index, so a parent can draw a connector to its children */
  row: number
  parentRow: number
}

function labelOf(n: TreeNode): string {
  if (n.type === 'plan') return `plan  → ${n.output_key}`
  if (n.type === 'follow_trajectory') return `follow_trajectory  ← ${n.input_key}`
  if (n.type === 'condition') return `condition  ${n.check}${n.invert ? ' (inverted)' : ''}`
  if (n.type === 'repeat') return `repeat ×${n.count}`
  return n.type
}

/** Pre-order flatten, which is also the order a tick visits children. */
export function flatten(root: TreeNode): FlatNode[] {
  const out: FlatNode[] = []
  const walk = (n: TreeNode, path: number[], depth: number, parentRow: number) => {
    const row = out.length
    out.push({ path, depth, node: n, label: labelOf(n), row, parentRow })
    const kids = n.children || (n.child ? [n.child] : [])
    kids.forEach((c, i) => walk(c, path.concat(i), depth + 1, row))
  }
  walk(root, [], 0, -1)
  return out
}

const samePath = (a: number[], b: number[]) =>
  a.length === b.length && a.every((v, i) => v === b[i])

/** The result this node returned on this tick, or null if never reached. */
export function resultAt(tick: Tick, path: number[]): TickResult | null {
  const hit = tick.visited.find((v) => samePath(v.path, path))
  return hit ? hit.result : null
}

/** Is this node on the currently-running path (an ancestor of, or, the running leaf)? */
export function onRunningPath(tick: Tick, path: number[]): boolean {
  return tick.running_paths.some(
    (rp) => rp.length >= path.length && path.every((v, i) => v === rp[i]),
  )
}

// --------------------------------------------------------- the blackboard

/**
 * A blackboard value rendered short enough for a slide.
 *
 * The `traj` value in these traces is a whole JointTrajectory message --
 * joint names, and a points array with positions, velocities,
 * accelerations and a time_from_start per point. Printing it raw fills
 * the panel, so this summarises it in the shape the deck teaches, and
 * the full message is one slide earlier as quoted JSON.
 */
export function summariseValue(v: unknown): string {
  if (v === null || v === undefined) return 'null'
  if (typeof v !== 'object') return String(v)
  const o = v as Record<string, unknown>
  if (Array.isArray(o)) return `[${o.length} items]`
  if (Array.isArray(o.points) && Array.isArray(o.joint_names)) {
    const pts = o.points as Array<Record<string, unknown>>
    const last = pts[pts.length - 1] as { time_from_start?: { sec?: number; nanosec?: number } }
    const t = last && last.time_from_start
      ? (last.time_from_start.sec || 0) + (last.time_from_start.nanosec || 0) / 1e9
      : 0
    return `JointTrajectory · ${(o.joint_names as string[]).length} joints · ` +
      `${pts.length} points · ends t+${t.toFixed(1)}s`
  }
  return `{${Object.keys(o).slice(0, 3).join(', ')}}`
}
