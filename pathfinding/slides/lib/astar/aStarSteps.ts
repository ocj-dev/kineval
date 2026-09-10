import type { AStarStep, GridNode, HeapEntry, Scene, SearchAlg } from './types'

// A step-generator port of ../../reference/graph_search.js for driving the
// interactive visualization in the browser. It is kept structurally
// identical to that file (same grid construction, same heap, same neighbor
// order, same priority formula) so the two never disagree on behavior --
// but the "Component breakdown" slides embed the actual reference/*.js
// files directly via Slidev snippet-import, so what you read there is
// always the real shipped code, not this port. `line` on every yielded
// step matches the pseudocode line numbers used in both that slide and the
// comments in graph_search.js -- every pseudocode line gets its own step,
// so stepping through the trace one entry at a time walks the pseudocode
// one line at a time.

// shared key format for looking up a world coordinate in a visited/queued
// Set -- used by useAStarTracer.ts (to build the sets) and MapCanvas.vue
// (to look positions up in them)
export function nodeKey(x: number, y: number): string {
  return `${x.toFixed(3)},${y.toFixed(3)}`
}

export function testCollision(scene: Scene, q: [number, number]): boolean {
  for (const [xRange, yRange] of scene.obstacles) {
    if (q[0] >= xRange[0] && q[0] <= xRange[1] && q[1] >= yRange[0] && q[1] <= yRange[1]) return true
  }
  return false
}

function heuristic(node: GridNode, goal: [number, number]): number {
  return Math.hypot(node.x - goal[0], node.y - goal[1])
}

// mirrors computeNodePriority() in reference/graph_search.js
function computePriority(node: GridNode, goal: [number, number], alg: SearchAlg, expansionCounter: number): number {
  switch (alg) {
    case 'greedy-best-first':
      return heuristic(node, goal)
    case 'breadth-first':
      return node.distance
    case 'depth-first':
      return -expansionCounter
    case 'A-star':
    default:
      return node.distance + heuristic(node, goal)
  }
}

// mirrors minheap_insert() in reference/graph_search.js
function heapInsert(heap: GridNode[], element: GridNode) {
  heap.push(element)
  let idx = heap.length - 1
  while (idx > 0) {
    const parentIdx = Math.floor((idx - 1) / 2)
    if (heap[parentIdx].priority! <= heap[idx].priority!) break
    ;[heap[parentIdx], heap[idx]] = [heap[idx], heap[parentIdx]]
    idx = parentIdx
  }
}

// mirrors minheap_extract() in reference/graph_search.js
function heapExtract(heap: GridNode[]): GridNode {
  const minElement = heap[0]
  const lastElement = heap.pop()!
  if (heap.length > 0) {
    heap[0] = lastElement
    let idx = 0
    for (;;) {
      const left = 2 * idx + 1
      const right = 2 * idx + 2
      let smallest = idx
      if (left < heap.length && heap[left].priority! < heap[smallest].priority!) smallest = left
      if (right < heap.length && heap[right].priority! < heap[smallest].priority!) smallest = right
      if (smallest === idx) break
      ;[heap[idx], heap[smallest]] = [heap[smallest], heap[idx]]
      idx = smallest
    }
  }
  return minElement
}

function heapSnapshot(heap: GridNode[]): HeapEntry[] {
  return heap.map((n) => ({ i: n.i, j: n.j, x: n.x, y: n.y, priority: n.priority! }))
}

export function buildGrid(scene: Scene): { grid: GridNode[][]; start: GridNode } {
  const { xMin, xMax, yMin, yMax, eps } = scene.gridBounds
  const grid: GridNode[][] = []
  let closestStart: GridNode | null = null
  let closestDist = Infinity

  let iind = 0
  for (let xpos = xMin; xpos < xMax; xpos += eps) {
    grid[iind] = []
    let jind = 0
    for (let ypos = yMin; ypos < yMax; ypos += eps) {
      const node: GridNode = {
        i: iind, j: jind, x: xpos, y: ypos,
        parent: null, distance: 10000, visited: false, priority: null, queued: false,
      }
      grid[iind][jind] = node
      const d = Math.hypot(xpos - scene.qInit[0], ypos - scene.qInit[1])
      if (d < closestDist) {
        closestDist = d
        closestStart = node
      }
      jind++
    }
    iind++
  }

  return { grid, start: closestStart! }
}

// pseudocode:
//  1  initialize the open queue with the start node (distance 0)
//  2  while the open queue is not empty
//  3      pop the node with minimum priority from the open queue
//  4      if that node was already visited, discard it and continue
//  5      mark the node visited
//  6      if within one grid cell of the goal, reconstruct the path and succeed
//  7      for each of its 4 grid neighbors
//  8          if the neighbor is off the grid or in collision, skip it
//  9          tentative_distance = current.distance + eps
// 10          if tentative_distance < neighbor.distance
// 11              update routing through visited cell
// 12              neighbor.priority = f(neighbor)
// 13              insert the neighbor into the open queue
// 14  the open queue emptied out -- no path exists; fail
export function* aStarSteps(scene: Scene, searchAlg: SearchAlg = 'A-star'): Generator<AStarStep, void, unknown> {
  const { grid, start } = buildGrid(scene)
  const eps = scene.gridBounds.eps

  start.distance = 0
  start.priority = 0
  start.queued = true
  const visitQueue: GridNode[] = []
  heapInsert(visitQueue, start)
  let expansionCounter = 0

  yield { line: 1, action: 'init', current: { x: start.x, y: start.y }, heap: heapSnapshot(visitQueue) }

  for (;;) {
    yield { line: 2, action: 'check-queue', heap: heapSnapshot(visitQueue) }
    if (visitQueue.length === 0) {
      yield { line: 14, action: 'fail', heap: [] }
      return
    }

    const current = heapExtract(visitQueue)
    yield { line: 3, action: 'pop', current: { x: current.x, y: current.y }, heap: heapSnapshot(visitQueue) }

    if (current.visited) {
      yield { line: 4, action: 'discard-stale', current: { x: current.x, y: current.y }, heap: heapSnapshot(visitQueue) }
      continue
    }
    yield { line: 4, action: 'discard-stale', current: { x: current.x, y: current.y }, heap: heapSnapshot(visitQueue) }

    current.visited = true
    yield { line: 5, action: 'visit', current: { x: current.x, y: current.y }, heap: heapSnapshot(visitQueue) }

    const distToGoal = Math.hypot(current.x - scene.qGoal[0], current.y - scene.qGoal[1])
    if (distToGoal <= eps) {
      const path: { x: number; y: number }[] = []
      let ref: GridNode | null = current
      while (ref) {
        path.push({ x: ref.x, y: ref.y })
        ref = ref.parent
      }
      yield { line: 6, action: 'succeed', current: { x: current.x, y: current.y }, path, heap: heapSnapshot(visitQueue) }
      return
    }
    yield { line: 6, action: 'not-goal', current: { x: current.x, y: current.y }, heap: heapSnapshot(visitQueue) }

    const neighborOffsets: [number, number][] = [[1, 0], [-1, 0], [0, 1], [0, -1]]
    for (const [di, dj] of neighborOffsets) {
      const ni = current.i + di
      const nj = current.j + dj
      const inBounds = ni >= 0 && ni < grid.length && nj >= 0 && nj < grid[ni].length
      const neighborCoords = inBounds ? { x: grid[ni][nj].x, y: grid[ni][nj].y } : { x: current.x + di * eps, y: current.y + dj * eps }

      yield { line: 7, action: 'consider-neighbor', current: { x: current.x, y: current.y }, neighbor: neighborCoords, heap: heapSnapshot(visitQueue) }

      if (!inBounds || testCollision(scene, [neighborCoords.x, neighborCoords.y])) {
        yield { line: 8, action: 'skip-neighbor', current: { x: current.x, y: current.y }, neighbor: neighborCoords, heap: heapSnapshot(visitQueue) }
        continue
      }
      yield { line: 8, action: 'neighbor-ok', current: { x: current.x, y: current.y }, neighbor: neighborCoords, heap: heapSnapshot(visitQueue) }

      const neighbor = grid[ni][nj]
      const tentativeDistance = current.distance + eps
      yield {
        line: 9, action: 'tentative', current: { x: current.x, y: current.y },
        neighbor: neighborCoords, tentativeDistance, heap: heapSnapshot(visitQueue),
      }

      if (tentativeDistance < neighbor.distance) {
        yield {
          line: 10, action: 'relax-check', current: { x: current.x, y: current.y },
          neighbor: neighborCoords, tentativeDistance, heap: heapSnapshot(visitQueue),
        }

        neighbor.distance = tentativeDistance
        neighbor.parent = current
        yield {
          line: 11, action: 'relax', current: { x: current.x, y: current.y },
          neighbor: neighborCoords, tentativeDistance, heap: heapSnapshot(visitQueue),
        }

        neighbor.priority = computePriority(neighbor, scene.qGoal, searchAlg, expansionCounter)
        expansionCounter++
        yield {
          line: 12, action: 'priority', current: { x: current.x, y: current.y },
          neighbor: neighborCoords, neighborPriority: neighbor.priority, heap: heapSnapshot(visitQueue),
        }

        neighbor.queued = true
        heapInsert(visitQueue, neighbor)
        yield {
          line: 13, action: 'enqueue', current: { x: current.x, y: current.y },
          neighbor: neighborCoords, neighborPriority: neighbor.priority, heap: heapSnapshot(visitQueue),
        }
      } else {
        yield {
          line: 10, action: 'no-relax', current: { x: current.x, y: current.y },
          neighbor: neighborCoords, tentativeDistance, heap: heapSnapshot(visitQueue),
        }
      }
    }
  }
}
