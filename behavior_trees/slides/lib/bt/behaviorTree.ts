// A TS port of ../../../reference/behavior_tree.js, function for
// function and in the same order. The reference file carries the
// explanations; this carries only what the port needs to be a port. See
// types.ts's header for why a port exists.
import type { Blackboard, BtNode, DecoratorPolicy, Status, TraceEntry, WalkEntry } from './types'

export const BT_SUCCESS: Status = 'success'
export const BT_FAILURE: Status = 'failure'
export const BT_RUNNING: Status = 'running'

export function btSequence(name: string, children: BtNode[], options?: { memory?: boolean }): BtNode {
  return { kind: 'sequence', name, children, memory: !options || options.memory !== false, state: { current: 0 } }
}

export function btFallback(name: string, children: BtNode[], options?: { memory?: boolean }): BtNode {
  return { kind: 'fallback', name, children, memory: !options || options.memory !== false, state: { current: 0 } }
}

export function btParallel(name: string, children: BtNode[], successThreshold?: number): BtNode {
  return {
    kind: 'parallel', name, children,
    successThreshold: successThreshold === undefined ? children.length : successThreshold,
    state: {},
  }
}

export function btDecorator(
  name: string, policy: DecoratorPolicy, child: BtNode, options?: { limit?: number },
): BtNode {
  return {
    kind: 'decorator', name, policy, children: [child],
    limit: options && options.limit !== undefined ? options.limit : 16,
    state: { attempts: 0 },
  }
}

export function btCondition(name: string, predicate: (bb: Blackboard) => boolean | Status): BtNode {
  return { kind: 'condition', name, children: [], predicate, state: {} }
}

export function btAction(
  name: string,
  handlers: { start?: (bb: Blackboard) => void; step: (bb: Blackboard) => Status; halt?: (bb: Blackboard) => void },
): BtNode {
  return {
    kind: 'action', name, children: [],
    start: handlers.start || null, step: handlers.step, halt: handlers.halt || null,
    state: { started: false },
  }
}

export function btTick(node: BtNode, blackboard: Blackboard, trace: TraceEntry[] | null, path: string): Status {
  let status: Status
  switch (node.kind) {
    case 'sequence': status = btTickSequence(node, blackboard, trace, path); break
    case 'fallback': status = btTickFallback(node, blackboard, trace, path); break
    case 'parallel': status = btTickParallel(node, blackboard, trace, path); break
    case 'decorator': status = btTickDecorator(node, blackboard, trace, path); break
    case 'condition': status = btTickCondition(node, blackboard); break
    case 'action': status = btTickAction(node, blackboard); break
    default: status = BT_FAILURE
  }
  node.status = status
  if (trace) trace.push({ path, name: node.name, kind: node.kind, status })
  return status
}

function btTickSequence(node: BtNode, bb: Blackboard, trace: TraceEntry[] | null, path: string): Status {
  const start = node.memory ? node.state.current! : 0
  for (let i = start; i < node.children.length; i++) {
    const status = btTick(node.children[i], bb, trace, childPath(path, i))
    if (status === BT_RUNNING) {
      node.state.current = i
      haltChildrenFrom(node, i + 1, bb)
      return BT_RUNNING
    }
    if (status === BT_FAILURE) {
      node.state.current = 0
      haltChildrenFrom(node, i + 1, bb)
      return BT_FAILURE
    }
  }
  node.state.current = 0
  return BT_SUCCESS
}

function btTickFallback(node: BtNode, bb: Blackboard, trace: TraceEntry[] | null, path: string): Status {
  const start = node.memory ? node.state.current! : 0
  for (let i = start; i < node.children.length; i++) {
    const status = btTick(node.children[i], bb, trace, childPath(path, i))
    if (status === BT_RUNNING) {
      node.state.current = i
      haltChildrenFrom(node, i + 1, bb)
      return BT_RUNNING
    }
    if (status === BT_SUCCESS) {
      node.state.current = 0
      haltChildrenFrom(node, i + 1, bb)
      return BT_SUCCESS
    }
  }
  node.state.current = 0
  return BT_FAILURE
}

function btTickParallel(node: BtNode, bb: Blackboard, trace: TraceEntry[] | null, path: string): Status {
  if (!node.state.childStatus) node.state.childStatus = []
  let successes = 0
  let failures = 0

  for (let i = 0; i < node.children.length; i++) {
    const prior = node.state.childStatus[i]
    let status: Status
    if (prior === BT_SUCCESS || prior === BT_FAILURE) {
      status = prior
    } else {
      status = btTick(node.children[i], bb, trace, childPath(path, i))
      node.state.childStatus[i] = status
    }
    if (status === BT_SUCCESS) successes++
    else if (status === BT_FAILURE) failures++
  }

  const needed = node.successThreshold!
  if (successes >= needed) {
    btHalt(node, bb)
    node.state.childStatus = []
    return BT_SUCCESS
  }
  if (failures > node.children.length - needed) {
    btHalt(node, bb)
    node.state.childStatus = []
    return BT_FAILURE
  }
  return BT_RUNNING
}

function btTickDecorator(node: BtNode, bb: Blackboard, trace: TraceEntry[] | null, path: string): Status {
  const status = btTick(node.children[0], bb, trace, childPath(path, 0))

  switch (node.policy) {
    case 'invert':
      if (status === BT_SUCCESS) return BT_FAILURE
      if (status === BT_FAILURE) return BT_SUCCESS
      return BT_RUNNING
    case 'force-success':
      if (status === BT_FAILURE) return BT_SUCCESS
      return status
    case 'repeat-until-success':
    case 'retry':
      if (status === BT_SUCCESS) { node.state.attempts = 0; return BT_SUCCESS }
      if (status === BT_FAILURE) {
        node.state.attempts = (node.state.attempts || 0) + 1
        if (node.state.attempts >= node.limit!) { node.state.attempts = 0; return BT_FAILURE }
        return BT_RUNNING
      }
      return BT_RUNNING
    default:
      return BT_FAILURE
  }
}

function btTickCondition(node: BtNode, bb: Blackboard): Status {
  const answer = node.predicate!(bb)
  if (answer === BT_RUNNING) return BT_FAILURE
  return answer ? BT_SUCCESS : BT_FAILURE
}

function btTickAction(node: BtNode, bb: Blackboard): Status {
  if (!node.state.started) {
    node.state.started = true
    if (node.start) node.start(bb)
  }
  const status = node.step!(bb)
  if (status !== BT_RUNNING) node.state.started = false
  return status
}

export function btHalt(node: BtNode, bb: Blackboard) {
  for (let i = 0; i < node.children.length; i++) btHalt(node.children[i], bb)
  if (node.kind === 'action' && node.state.started) {
    if (node.halt) node.halt(bb)
    node.state.started = false
  }
  if (node.state.current !== undefined) node.state.current = 0
  if (node.state.attempts !== undefined) node.state.attempts = 0
  if (node.state.childStatus) node.state.childStatus = []
  node.status = undefined
}

function haltChildrenFrom(node: BtNode, from: number, bb: Blackboard) {
  for (let i = from; i < node.children.length; i++) btHalt(node.children[i], bb)
}

export function btTickTree(root: BtNode, blackboard: Blackboard): { status: Status; trace: TraceEntry[] } {
  const trace: TraceEntry[] = []
  const status = btTick(root, blackboard, trace, '')
  return { status, trace }
}

export function childPath(path: string, index: number): string {
  return path === '' ? String(index) : path + '.' + index
}

export function btWalk(root: BtNode): WalkEntry[] {
  const nodes: WalkEntry[] = []
  function visit(node: BtNode, path: string, depth: number, index: number) {
    nodes.push({
      node, path, depth, index,
      kind: node.kind, name: node.name, childCount: node.children.length,
    })
    for (let i = 0; i < node.children.length; i++) visit(node.children[i], childPath(path, i), depth + 1, i)
  }
  visit(root, '', 0, 0)
  return nodes
}

export function btBlackboard(initial?: Record<string, unknown>): Blackboard {
  const blackboard: Blackboard = { now: 0, dt: 0, log: [] }
  for (const key in initial || {}) blackboard[key] = (initial as any)[key]
  return blackboard
}

export function btBlackboardRequire(root: BtNode, blackboard: Blackboard) {
  const nodes = btWalk(root)
  const written: Record<string, string> = {}
  const problems: { path: string; name: string; key: string; message: string }[] = []

  for (const entry of nodes) {
    for (const key of entry.node.writes || []) written[key] = entry.path
  }
  for (const entry of nodes) {
    for (const key of entry.node.reads || []) {
      if (blackboard[key] !== undefined) continue
      if (written[key] !== undefined) continue
      problems.push({
        path: entry.path, name: entry.name, key,
        message: `leaf "${entry.name}" (path ${entry.path}) reads blackboard key "${key}", which nothing in this tree writes`,
      })
    }
  }
  return problems
}
