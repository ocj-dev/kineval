// Panel-side port of motion_planning/reference/{rrt,random}.js.
//
// Structurally identical to the reference implementation, function for
// function, so a panel cannot drift from the code the slides quote: same
// names (randomConfig, newConfig, findNearestNeighbor, edgeIsValid,
// extendRRT, connectRRT, smoothPath), same three-valued extend result,
// same seeded generator. The only addition is that the iterate functions
// are GENERATORS -- they yield one step per pseudocode line, carrying the
// active line and enough state to draw -- which is what makes
// Play/Pause/Step/Reset index navigation over a precomputed trace.
//
// Keeping it a port rather than a reimplementation is the point: the
// picture on the slide is produced by the same algorithm the slide is
// teaching.

export type Config = [number, number]
// an obstacle is [[xmin, xmax], [ymin, ymax]] -- exactly the `range`
// format infrastructure.js#setPlanningScene builds and
// #appendix-collision tests against
export type Obstacle = [[number, number], [number, number]]

export interface Vertex {
  vertex: Config
  parent: number
}
export interface Tree {
  vertices: Vertex[]
  newest: number
}

// ---------------------------------------------------------------- random
// mulberry32, byte-for-byte the algorithm in reference/random.js
export class Rng {
  private state = 0
  constructor(public seed: number) {
    this.reseed(seed)
  }
  reseed(seed: number) {
    this.seed = seed
    let h = seed >>> 0
    h = Math.imul(h ^ (h >>> 16), 0x85ebca6b)
    h = Math.imul(h ^ (h >>> 13), 0xc2b2ae35)
    this.state = (h ^ (h >>> 16)) >>> 0
  }
  next(): number {
    this.state = (this.state + 0x6d2b79f5) >>> 0
    let t = this.state
    t = Math.imul(t ^ (t >>> 15), t | 1)
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61)
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
  inRange(lo: number, hi: number) {
    return lo + (hi - lo) * this.next()
  }
  integer(n: number) {
    return Math.floor(this.next() * n)
  }
}

// ------------------------------------------------------------- the world
// The four walls infrastructure.js#setPlanningScene always installs,
// before any scene's own obstacles are concatenated on.
export const WORLD_WALLS: Obstacle[] = [
  [[-1.8, 5.8], [-1.8, -1]],
  [[-1.8, 5.8], [5, 5.8]],
  [[-1.8, -1], [-1.8, 5.8]],
  [[5, 5.8], [-1.8, 5.8]],
]

// The five scenes carried into this module at
// reference/scenes/*.js, verbatim.
export const SCENES: Record<string, { label: string; obstacles: Obstacle[]; init: Config; goal: Config }> = {
  spiral: {
    label: 'spiral',
    obstacles: [
      [[-0.8, 4.8], [-0.8, -0.5]],
      [[-0.8, 4.8], [4.5, 4.8]],
      [[4.5, 4.8], [-0.8, 4.8]],
      [[-0.8, -0.5], [-0.8, 1.5]],
      [[-0.8, -0.5], [2.5, 4.8]],
      [[0.2, 3.8], [0.2, 0.5]],
      [[0.2, 3.8], [3.5, 3.8]],
      [[0.2, 0.5], [0.2, 3.8]],
      [[3.5, 3.8], [0.2, 1.5]],
      [[3.5, 3.8], [2.5, 3.8]],
    ],
    init: [-0.65, 2],
    goal: [2, 2],
  },
  cul_de_sac: {
    label: 'cul_de_sac',
    obstacles: [
      [[2.2, 4.1], [3.7, 4.0]],
      [[3.7, 4.0], [2.2, 4.0]],
    ],
    init: [0, 0],
    goal: [4.6, 2],
  },
  construction_detour: {
    label: 'construction_detour',
    obstacles: [
      [[-1.8, 4.6], [2.0, 2.3]],
      [[5.1, 5.8], [2.0, 2.3]],
    ],
    init: [0, 0],
    goal: [4.6, 4.6],
  },
  diagonal_staircase: {
    label: 'diagonal_staircase',
    obstacles: [
      [[-1.8, 3.0], [0.9, 1.2]],
      [[1.0, 5.8], [2.1, 2.4]],
      [[-1.8, 3.0], [3.3, 3.6]],
    ],
    init: [0, 0],
    goal: [4.6, 4.6],
  },
  downtown_gridlock: {
    label: 'downtown_gridlock',
    obstacles: [
      [[0.3, 0.9], [0.3, 0.9]], [[0.3, 0.9], [1.7, 2.3]], [[0.3, 0.9], [3.1, 3.7]],
      [[1.7, 2.3], [0.3, 0.9]], [[1.7, 2.3], [1.7, 2.3]], [[1.7, 2.3], [3.1, 3.7]],
      [[3.1, 3.7], [0.3, 0.9]], [[3.1, 3.7], [1.7, 2.3]], [[3.1, 3.7], [3.1, 3.7]],
    ],
    init: [0, 0],
    goal: [4.6, 4.6],
  },
}

export function sceneRanges(name: string): Obstacle[] {
  return WORLD_WALLS.concat(SCENES[name].obstacles)
}

// ------------------------------------------------- the reference functions

// reference/infrastructure.js #appendix-collision
export function testCollision(q: Config, ranges: Obstacle[]): boolean {
  for (let j = 0; j < ranges.length; j++) {
    let inCollision = true
    for (let i = 0; i < q.length; i++) {
      if (q[i] < ranges[j][i][0] || q[i] > ranges[j][i][1]) inCollision = false
    }
    if (inCollision) return true
  }
  return false
}

export const STEP_SIZE = 0.5
export const EDGE_RESOLUTION = 0.2

// reference/rrt.js #configuration-distance
export function configurationDistance(a: Config, b: Config): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1])
}

// reference/rrt.js #random-config
export function randomConfig(rng: Rng): Config {
  return [rng.inRange(-2, 6), rng.inRange(-2, 6)]
}

// reference/rrt.js #new-config
export function newConfig(qNear: Config, qTarget: Config, step: number): Config {
  const d = configurationDistance(qTarget, qNear)
  if (d < 1e-9) return [qNear[0], qNear[1]]
  const ratio = Math.min(1, step / d)
  return [qNear[0] + ratio * (qTarget[0] - qNear[0]), qNear[1] + ratio * (qTarget[1] - qNear[1])]
}

// reference/rrt.js #edge-is-valid -- the whole edge, not just its endpoint
export function edgeIsValid(a: Config, b: Config, ranges: Obstacle[]): boolean {
  const d = configurationDistance(a, b)
  const steps = Math.max(1, Math.ceil(d / (EDGE_RESOLUTION * STEP_SIZE)))
  for (let s = 1; s <= steps; s++) {
    const t = s / steps
    if (testCollision([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])], ranges)) return false
  }
  return true
}

// reference/rrt.js #find-nearest-neighbor -- linear scan, deliberately
export function findNearestNeighbor(q: Config, tree: Tree) {
  let bestIndex = 0
  let best = Infinity
  for (let j = 0; j < tree.vertices.length; j++) {
    const d = configurationDistance(q, tree.vertices[j].vertex)
    if (d < best) { best = d; bestIndex = j }
  }
  return { index: bestIndex, config: tree.vertices[bestIndex].vertex, distance: best }
}

export function treeInit(q: Config): Tree {
  return { vertices: [{ vertex: q, parent: -1 }], newest: 0 }
}

export type ExtendResult = 'reached' | 'advanced' | 'trapped'

// reference/rrt.js #extend-rrt
export function extendRRT(tree: Tree, q: Config, ranges: Obstacle[]): ExtendResult {
  const near = findNearestNeighbor(q, tree)
  const qNew = newConfig(near.config, q, STEP_SIZE)
  if (!edgeIsValid(near.config, qNew, ranges)) return 'trapped'
  tree.vertices.push({ vertex: qNew, parent: near.index })
  tree.newest = tree.vertices.length - 1
  // "reached" means the new vertex IS q, so the validated edge runs all
  // the way to it -- see reference/rrt.js for why the looser "within one
  // step" version leaves the tree junction unchecked
  if (configurationDistance(qNew, q) < 1e-9) return 'reached'
  return 'advanced'
}

// reference/rrt.js #connect-rrt -- the greedy half
export function connectRRT(tree: Tree, q: Config, ranges: Obstacle[]): ExtendResult {
  let result: ExtendResult = 'advanced'
  let guard = 0
  while (result === 'advanced' && guard++ < 500) {
    result = extendRRT(tree, q, ranges)
  }
  return result
}

// reference/rrt.js #parent-path
export function parentPath(tree: Tree, index: number): Config[] {
  const path: Config[] = []
  let i = index
  let guard = 0
  while (i >= 0 && guard++ < tree.vertices.length + 1) {
    path.push(tree.vertices[i].vertex)
    i = tree.vertices[i].parent
  }
  path.reverse()
  return path
}

export function pathLength(path: Config[]): number {
  let total = 0
  for (let i = 1; i < path.length; i++) total += configurationDistance(path[i - 1], path[i])
  return total
}

// reference/rrt.js #smooth-path
export function smoothPath(path: Config[], ranges: Obstacle[], rng: Rng, attempts = 150): Config[] {
  if (path.length < 3) return path.slice()
  const smoothed = path.slice()
  for (let a = 0; a < attempts; a++) {
    if (smoothed.length < 3) break
    let i = rng.integer(smoothed.length)
    let j = rng.integer(smoothed.length)
    if (i > j) { const t = i; i = j; j = t }
    if (j - i < 2) continue
    if (edgeIsValid(smoothed[i], smoothed[j], ranges)) smoothed.splice(i + 1, j - i - 1)
  }
  return smoothed
}

// ------------------------------------------------------------ the traces

export interface Step {
  line: number
  note: string
  treeA: Config[][]      // [from, to] segments
  treeB: Config[][]
  qRand: Config | null
  qNew: Config | null
  highlight: Config[] | null   // the edge just added
  path: Config[] | null
  smoothed: Config[] | null
  aIsStart: boolean
  iteration: number
  collisionChecks: number
}

function edgesOf(tree: Tree): Config[][] {
  const out: Config[][] = []
  for (let i = 1; i < tree.vertices.length; i++) {
    const p = tree.vertices[i].parent
    if (p >= 0) out.push([tree.vertices[p].vertex, tree.vertices[i].vertex])
  }
  return out
}

// The pseudocode the step-through highlights, one entry per line.
export const RRT_CONNECT_LINES = [
  'RRT_CONNECT(q_init, q_goal)',
  '  T_a.init(q_init);  T_b.init(q_goal)',
  '  for k = 1 to K do',
  '    q_rand <- RANDOM_CONFIG()',
  '    if EXTEND(T_a, q_rand) != Trapped then',
  '      if CONNECT(T_b, q_new) = Reached then',
  '        return PATH(T_a, T_b)',
  '    SWAP(T_a, T_b)',
  '  return Failure',
]

// RRT-Connect as a generator, one yield per pseudocode line.
export function* rrtConnectTrace(
  sceneName: string,
  seed: number,
  maxIterations = 400,
): Generator<Step> {
  const scene = SCENES[sceneName]
  const ranges = sceneRanges(sceneName)
  const rng = new Rng(seed)

  let Ta = treeInit(scene.init)
  let Tb = treeInit(scene.goal)
  let aIsStart = true
  let checks = 0

  const snap = (line: number, note: string, extra: Partial<Step> = {}): Step => ({
    line,
    note,
    treeA: edgesOf(aIsStart ? Ta : Tb),
    treeB: edgesOf(aIsStart ? Tb : Ta),
    qRand: null, qNew: null, highlight: null, path: null, smoothed: null,
    aIsStart, iteration: iter, collisionChecks: checks,
    ...extra,
  })

  let iter = 0
  yield snap(1, 'Two trees: one rooted at the start, one at the goal.')

  for (iter = 1; iter <= maxIterations; iter++) {
    const qRand = randomConfig(rng)
    yield snap(3, 'Sample a configuration uniformly. No goal bias.', { qRand })

    const beforeA = Ta.vertices.length
    const result = extendRRT(Ta, qRand, ranges)
    checks += 4
    const qNewA = Ta.vertices[Ta.newest].vertex

    if (result === 'trapped') {
      yield snap(4, 'EXTEND is Trapped — the step hit an obstacle. Swap and resample.', { qRand })
    } else {
      yield snap(4, `EXTEND ${result === 'reached' ? 'Reached' : 'Advanced'} — one step toward the sample.`, {
        qRand,
        qNew: qNewA,
        highlight: beforeA < Ta.vertices.length
          ? [Ta.vertices[Ta.vertices[Ta.newest].parent].vertex, qNewA]
          : null,
      })

      const beforeB = Tb.vertices.length
      const connect = connectRRT(Tb, qNewA, ranges)
      checks += 4 * Math.max(1, Tb.vertices.length - beforeB)

      if (connect === 'reached') {
        yield snap(5, 'CONNECT Reached — the two trees have met.', { qRand, qNew: qNewA })
        const pa = parentPath(Ta, Ta.newest)
        const pb = parentPath(Tb, Tb.newest)
        const raw = aIsStart ? pa.concat(pb.slice().reverse()) : pb.concat(pa.slice().reverse())
        const smoothed = smoothPath(raw, ranges, rng)
        yield snap(6, `Path found after ${iter} iterations: ${raw.length} configurations, ${smoothed.length} after smoothing.`, {
          qRand, qNew: qNewA, path: raw, smoothed,
        })
        return
      }
      yield snap(5, 'CONNECT extended greedily until blocked, but did not arrive.', { qRand, qNew: qNewA })
    }

    const t = Ta; Ta = Tb; Tb = t
    aIsStart = !aIsStart
    yield snap(7, 'Swap the trees, so both grow.', {})
  }

  yield snap(8, `No path within ${maxIterations} iterations.`, {})
}

// Single-tree RRT, for the extend-versus-connect comparison: identical
// except that it never calls CONNECT.
export function growSingleTree(sceneName: string, seed: number, iterations: number) {
  const scene = SCENES[sceneName]
  const ranges = sceneRanges(sceneName)
  const rng = new Rng(seed)
  const T = treeInit(scene.init)
  let reached = -1
  for (let k = 1; k <= iterations; k++) {
    const qRand = randomConfig(rng)
    if (extendRRT(T, qRand, ranges) !== 'trapped') {
      const qNew = T.vertices[T.newest].vertex
      if (reached < 0 && configurationDistance(qNew, scene.goal) < STEP_SIZE
          && edgeIsValid(qNew, scene.goal, ranges)) {
        reached = k
      }
    }
  }
  return { tree: T, edges: edgesOf(T), reached }
}

// Bidirectional with EXTEND only (no greedy CONNECT) -- the one-line
// change that turns RRT-Connect back into bidirectional RRT.
export function growBidirectionalExtendOnly(sceneName: string, seed: number, iterations: number) {
  const scene = SCENES[sceneName]
  const ranges = sceneRanges(sceneName)
  const rng = new Rng(seed)
  let Ta = treeInit(scene.init)
  let Tb = treeInit(scene.goal)
  let met = -1
  for (let k = 1; k <= iterations && met < 0; k++) {
    const qRand = randomConfig(rng)
    if (extendRRT(Ta, qRand, ranges) !== 'trapped') {
      const qNew = Ta.vertices[Ta.newest].vertex
      if (extendRRT(Tb, qNew, ranges) === 'reached') met = k
    }
    const t = Ta; Ta = Tb; Tb = t
  }
  return { a: edgesOf(Ta), b: edgesOf(Tb), met, vertices: Ta.vertices.length + Tb.vertices.length }
}

export function growRrtConnect(sceneName: string, seed: number, maxIterations = 2000) {
  const scene = SCENES[sceneName]
  const ranges = sceneRanges(sceneName)
  const rng = new Rng(seed)
  let Ta = treeInit(scene.init)
  let Tb = treeInit(scene.goal)
  let aIsStart = true
  for (let k = 1; k <= maxIterations; k++) {
    const qRand = randomConfig(rng)
    if (extendRRT(Ta, qRand, ranges) !== 'trapped') {
      const qNew = Ta.vertices[Ta.newest].vertex
      if (connectRRT(Tb, qNew, ranges) === 'reached') {
        const pa = parentPath(Ta, Ta.newest)
        const pb = parentPath(Tb, Tb.newest)
        const raw = aIsStart ? pa.concat(pb.slice().reverse()) : pb.concat(pa.slice().reverse())
        return {
          a: edgesOf(aIsStart ? Ta : Tb),
          b: edgesOf(aIsStart ? Tb : Ta),
          raw,
          smoothed: smoothPath(raw, ranges, rng),
          iterations: k,
          vertices: Ta.vertices.length + Tb.vertices.length,
        }
      }
    }
    const t = Ta; Ta = Tb; Tb = t
    aIsStart = !aIsStart
  }
  return { a: edgesOf(Ta), b: edgesOf(Tb), raw: null, smoothed: null, iterations: maxIterations, vertices: Ta.vertices.length + Tb.vertices.length }
}
