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

// One action per pseudocode line (see lib/astar/pseudocode.ts), so every
// line gets its own step during the search -- some lines (8, 10) fork into
// two actions depending on which branch is taken.
export type StepAction =
  | 'check-queue'       // line 2
  | 'fail'              // line 14 (queue was empty)
  | 'pop'               // line 3
  | 'discard-stale'     // line 4 (popped node already visited)
  | 'visit'             // line 5
  | 'not-goal'          // line 6, false branch
  | 'succeed'           // line 6, true branch
  | 'consider-neighbor' // line 7
  | 'skip-neighbor'     // line 8, off-grid or in collision
  | 'neighbor-ok'       // line 8, passable
  | 'tentative'         // line 9
  | 'no-relax'          // line 10, false branch
  | 'relax-check'       // line 10, true branch (about to update)
  | 'relax'             // line 11, true branch (distance/parent updated)
  | 'priority'          // line 12
  | 'enqueue'           // line 13
  | 'init'              // line 1 (start node queued)

export interface HeapEntry {
  i: number
  j: number
  x: number
  y: number
  priority: number
}

// `line` matches the pseudocode line numbers used both in the "Algorithmic
// process" slide and in the code comments of ../../reference/graph_search.js,
// so the pseudocode panel, the code panel, and this trace all highlight the
// same step in lockstep.
export interface AStarStep {
  line: number
  action: StepAction
  current?: { x: number; y: number }
  neighbor?: { x: number; y: number }
  tentativeDistance?: number
  neighborPriority?: number
  path?: { x: number; y: number }[]
  heap: HeapEntry[]
}

export interface GridBounds {
  xMin: number
  xMax: number
  yMin: number
  yMax: number
  eps: number
}

export interface Scene {
  id: string
  name: string
  description: string
  obstacles: [[number, number], [number, number]][]
  qInit: [number, number]
  qGoal: [number, number]
  isNew: boolean
  /** The actual search grid extent/resolution. */
  gridBounds: GridBounds
  /**
   * Overrides MapCanvas's default kineval-stencil-matching visual crop
   * (world x,y in [-2,6)) for small, purpose-built teaching grids that
   * should render at their exact extent instead.
   */
  renderBounds?: { xMin: number; xMax: number; yMin: number; yMax: number }
}
