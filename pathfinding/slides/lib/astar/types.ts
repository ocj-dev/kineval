export interface GridNode {
  i: number
  j: number
  x: number
  y: number
  parent: GridNode | null
  distance: number
  visited: boolean
  priority: number | null
  queued: boolean
}

export type SearchAlg = 'A-star' | 'greedy-best-first' | 'breadth-first' | 'depth-first'

export type StepAction = 'visit' | 'discard-stale' | 'queue' | 'succeed' | 'fail'

// `line` matches the pseudocode line numbers used both in the "Algorithmic
// process" slide and in the code comments of ../../reference/graph_search.js,
// so the pseudocode panel, the code panel, and this trace all highlight the
// same step in lockstep.
export interface AStarStep {
  line: number
  action: StepAction
  node?: { x: number; y: number }
  neighbor?: { x: number; y: number }
  path?: { x: number; y: number }[]
}

export interface Scene {
  id: string
  name: string
  description: string
  obstacles: [[number, number], [number, number]][]
  qInit: [number, number]
  qGoal: [number, number]
  isNew: boolean
}
