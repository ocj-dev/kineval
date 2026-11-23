// Panel-side port of motion_planning/reference/graph_search.js, enough of
// it to run A* on the same five scenes the RRT panels use.
//
// Carried into this module for exactly one slide -- A* and RRT-Connect on
// the same world, side by side -- which is the comparison the pathfinding
// deck's groundwork makes available and nothing else in this course does.
// Same grid (eps = 0.1 over [-2, 7)), same four-connected neighbours, same
// f = g + h priority with Euclidean h.

import { testCollision, type Config, type Obstacle } from './planner'

export const EPS = 0.1

interface Node {
  i: number; j: number
  x: number; y: number
  parent: Node | null
  distance: number
  visited: boolean
  queued: boolean
  priority: number
}

export interface AstarResult {
  visited: number
  path: Config[] | null
  length: number
  /** every expanded node, for drawing the explored set */
  expanded: Config[]
}

export function runAstar(ranges: Obstacle[], qInit: Config, qGoal: Config, maxIterations = 200000): AstarResult {
  const G: Node[][] = []
  let startNode: Node | null = null
  let bestStart = Infinity

  let iind = 0
  for (let xpos = -2; xpos < 7; iind++, xpos += EPS) {
    G[iind] = []
    let jind = 0
    for (let ypos = -2; ypos < 7; jind++, ypos += EPS) {
      G[iind][jind] = {
        i: iind, j: jind, x: xpos, y: ypos,
        parent: null, distance: 10000, visited: false, queued: false, priority: 0,
      }
      const d = Math.hypot(xpos - qInit[0], ypos - qInit[1])
      if (d < bestStart) { bestStart = d; startNode = G[iind][jind] }
    }
  }
  if (!startNode) return { visited: 0, path: null, length: 0, expanded: [] }

  const h = (n: Node) => Math.hypot(n.x - qGoal[0], n.y - qGoal[1])

  startNode.distance = 0
  startNode.priority = h(startNode)
  startNode.queued = true

  // a plain array used as a priority queue; the reference uses a real
  // binary min-heap (graph_search.js #insert-queue / #pop-min), which
  // matters for its runtime and not for the picture this panel draws
  const queue: Node[] = [startNode]
  const expanded: Config[] = []
  let visited = 0

  for (let it = 0; it < maxIterations; it++) {
    if (queue.length === 0) return { visited, path: null, length: 0, expanded }

    let bi = 0
    for (let k = 1; k < queue.length; k++) if (queue[k].priority < queue[bi].priority) bi = k
    const node = queue.splice(bi, 1)[0]
    if (node.visited) continue
    node.visited = true
    visited++
    expanded.push([node.x, node.y])

    if (Math.hypot(node.x - qGoal[0], node.y - qGoal[1]) < EPS * 1.5) {
      const path: Config[] = []
      let cur: Node | null = node
      let length = 0
      while (cur) {
        path.push([cur.x, cur.y])
        if (cur.parent) length += Math.hypot(cur.x - cur.parent.x, cur.y - cur.parent.y)
        cur = cur.parent
      }
      path.reverse()
      return { visited, path, length, expanded }
    }

    const neighbours = [
      G[node.i - 1]?.[node.j], G[node.i + 1]?.[node.j],
      G[node.i]?.[node.j - 1], G[node.i]?.[node.j + 1],
    ]
    for (const nb of neighbours) {
      if (!nb || nb.visited) continue
      if (testCollision([nb.x, nb.y], ranges)) continue
      const tentative = node.distance + EPS
      if (tentative < nb.distance) {
        nb.distance = tentative
        nb.parent = node
        nb.priority = tentative + h(nb)
        if (!nb.queued) { nb.queued = true; queue.push(nb) }
        else queue.push(nb)
      }
    }
  }
  return { visited, path: null, length: 0, expanded }
}
