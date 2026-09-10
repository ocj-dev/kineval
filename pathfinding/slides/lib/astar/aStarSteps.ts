import type { AStarStep, GridNode, Scene, SearchAlg } from './types'

// A step-generator port of ../../reference/graph_search.js for driving the
// interactive visualization in the browser. It is kept structurally
// identical to that file (same grid construction, same heap, same neighbor
// order, same priority formula) so the two never disagree on behavior --
// but the "Component breakdown" slides embed the actual reference/*.js
// files directly via Slidev snippet-import, so what you read there is
// always the real shipped code, not this port. `line` on every yielded
// step matches the pseudocode line numbers used in both that slide and the
// comments in graph_search.js.

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

export function buildGrid(scene: Scene, eps: number): { grid: GridNode[][]; start: GridNode } {
  const grid: GridNode[][] = []
  let closestStart: GridNode | null = null
  let closestDist = Infinity

  let iind = 0
  for (let xpos = -2; xpos < 7; xpos += eps) {
    grid[iind] = []
    let jind = 0
    for (let ypos = -2; ypos < 7; ypos += eps) {
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
// 11              record this cheaper path: neighbor.distance, neighbor.parent
// 12              neighbor.priority = f(neighbor)
// 13              insert the neighbor into the open queue
// 14  the open queue emptied out -- no path exists; fail
export function* aStarSteps(scene: Scene, searchAlg: SearchAlg = 'A-star', eps = 0.2): Generator<AStarStep, void, unknown> {
  const { grid, start } = buildGrid(scene, eps)

  start.distance = 0
  start.priority = 0
  start.queued = true
  const visitQueue: GridNode[] = []
  heapInsert(visitQueue, start)
  let expansionCounter = 0

  yield { line: 1, action: 'queue', node: { x: start.x, y: start.y } }

  for (;;) {
    if (visitQueue.length === 0) {
      yield { line: 14, action: 'fail' }
      return
    }

    const current = heapExtract(visitQueue)

    if (current.visited) {
      yield { line: 4, action: 'discard-stale', node: { x: current.x, y: current.y } }
      continue
    }

    current.visited = true
    yield { line: 5, action: 'visit', node: { x: current.x, y: current.y } }

    const distToGoal = Math.hypot(current.x - scene.qGoal[0], current.y - scene.qGoal[1])
    if (distToGoal <= eps) {
      const path: { x: number; y: number }[] = []
      let ref: GridNode | null = current
      while (ref) {
        path.push({ x: ref.x, y: ref.y })
        ref = ref.parent
      }
      yield { line: 6, action: 'succeed', node: { x: current.x, y: current.y }, path }
      return
    }

    const neighborOffsets: [number, number][] = [[1, 0], [-1, 0], [0, 1], [0, -1]]
    for (const [di, dj] of neighborOffsets) {
      const ni = current.i + di
      const nj = current.j + dj
      if (ni < 0 || ni >= grid.length || nj < 0 || nj >= grid[ni].length) continue

      const neighbor = grid[ni][nj]
      if (testCollision(scene, [neighbor.x, neighbor.y])) continue

      const tentativeDistance = current.distance + eps
      if (tentativeDistance < neighbor.distance) {
        neighbor.distance = tentativeDistance
        neighbor.parent = current
        neighbor.priority = computePriority(neighbor, scene.qGoal, searchAlg, expansionCounter)
        expansionCounter++
        neighbor.queued = true
        heapInsert(visitQueue, neighbor)
        yield {
          line: 13,
          action: 'queue',
          node: { x: current.x, y: current.y },
          neighbor: { x: neighbor.x, y: neighbor.y },
        }
      }
    }
  }
}
