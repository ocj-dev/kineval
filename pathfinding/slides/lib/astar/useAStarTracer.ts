import { ref, computed, onBeforeUnmount, type Ref } from 'vue'
import { aStarSteps, nodeKey as key } from './aStarSteps'
import type { AStarStep, HeapEntry, Scene, SearchAlg } from './types'

interface TraceEntry {
  step: AStarStep
  visited: Set<string>
  queued: Set<string>
  distances: Map<string, number>
  edges: Map<string, string>
  path: { x: number; y: number }[]
}

function buildTrace(scene: Scene, alg: SearchAlg): TraceEntry[] {
  const trace: TraceEntry[] = []
  let visited = new Set<string>()
  let queued = new Set<string>()
  let distances = new Map<string, number>()
  let edges = new Map<string, string>()
  let path: { x: number; y: number }[] = []

  for (const step of aStarSteps(scene, alg)) {
    if (step.action === 'init' && step.current) {
      distances = new Map(distances)
      distances.set(key(step.current.x, step.current.y), 0)
    }
    if (step.action === 'visit' && step.current) {
      const k = key(step.current.x, step.current.y)
      visited = new Set(visited)
      visited.add(k)
      queued = new Set(queued)
      queued.delete(k)
    }
    if (step.action === 'relax' && step.current && step.neighbor && step.tentativeDistance !== undefined) {
      const nk = key(step.neighbor.x, step.neighbor.y)
      distances = new Map(distances)
      distances.set(nk, step.tentativeDistance)
      edges = new Map(edges)
      edges.set(nk, key(step.current.x, step.current.y))
    }
    if (step.action === 'enqueue' && step.neighbor) {
      queued = new Set(queued)
      queued.add(key(step.neighbor.x, step.neighbor.y))
    }
    if (step.action === 'succeed' && step.path) {
      path = step.path
    }
    trace.push({ step, visited, queued, distances, edges, path })
  }
  return trace
}

// Every scene/algorithm combination is precomputed into a full trace, so
// Play/Pause/Step Forward/Step Back/Reset are all just index navigation over
// an array that already exists -- mirrors useSorter.ts's approach in the
// sortingthespiderverse deck. Every pseudocode line gets its own trace
// entry (see aStarSteps.ts), so stepping one entry at a time walks the
// pseudocode one line at a time.
export function useAStarTracer(speedMs: Ref<number> = ref(4)) {
  const activeLine = ref(-1)
  const visited = ref<Set<string>>(new Set())
  const queued = ref<Set<string>>(new Set())
  const distances = ref<Map<string, number>>(new Map())
  const edges = ref<Map<string, string>>(new Map())
  const path = ref<{ x: number; y: number }[]>([])
  const currentNode = ref<{ x: number; y: number } | null>(null)
  const neighborNode = ref<{ x: number; y: number } | null>(null)
  const neighborPriority = ref<number | null>(null)
  const heap = ref<HeapEntry[]>([])
  const status = ref<'iterating' | 'succeeded' | 'failed' | 'idle'>('idle')
  const isRunning = ref(false)
  const pos = ref(-1)
  const stepCount = ref(0)

  let trace: TraceEntry[] = []
  let timer: ReturnType<typeof setTimeout> | null = null

  const isDone = computed(() => trace.length > 0 && pos.value >= trace.length - 1)
  const isAtStart = computed(() => pos.value <= -1)
  const visitedCount = computed(() => (pos.value >= 0 && trace[pos.value] ? trace[pos.value].visited.size : 0))
  const queuedCount = computed(() => (pos.value >= 0 && trace[pos.value] ? trace[pos.value].queued.size : 0))

  function applyPos(p: number) {
    pos.value = p
    if (p < 0) {
      activeLine.value = -1
      visited.value = new Set()
      queued.value = new Set()
      distances.value = new Map()
      edges.value = new Map()
      path.value = []
      currentNode.value = null
      neighborNode.value = null
      neighborPriority.value = null
      heap.value = []
      status.value = 'idle'
      return
    }
    const entry = trace[p]
    activeLine.value = entry.step.line
    visited.value = entry.visited
    queued.value = entry.queued
    distances.value = entry.distances
    edges.value = entry.edges
    path.value = entry.path
    currentNode.value = entry.step.current ?? null
    // neighborNode covers the whole lines 7-13 span for a given neighbor
    // (every step in that span carries a `neighbor`), so the neighbor cell
    // stays highlighted throughout its consideration; neighborPriority is
    // only set once it's actually computed (lines 12-13), which gates the
    // full tentative-route line drawn back to start and out to the goal.
    neighborNode.value = entry.step.neighbor ?? null
    neighborPriority.value = entry.step.neighborPriority ?? null
    heap.value = entry.step.heap
    if (entry.step.action === 'succeed') status.value = 'succeeded'
    else if (entry.step.action === 'fail') status.value = 'failed'
    else status.value = 'iterating'
  }

  function load(scene: Scene, alg: SearchAlg) {
    pause()
    trace = buildTrace(scene, alg)
    stepCount.value = trace.length
    applyPos(-1)
  }

  function reset() {
    pause()
    applyPos(-1)
  }

  function stepForward(): boolean {
    if (pos.value >= trace.length - 1) return false
    applyPos(pos.value + 1)
    return pos.value < trace.length - 1
  }

  function stepBack() {
    if (pos.value <= -1) return
    applyPos(pos.value - 1)
  }

  function scheduleNext() {
    timer = setTimeout(() => {
      const canContinue = stepForward()
      if (canContinue && isRunning.value) scheduleNext()
      else pause()
    }, speedMs.value)
  }

  function play() {
    if (isRunning.value || isDone.value) return
    isRunning.value = true
    scheduleNext()
  }

  function pause() {
    isRunning.value = false
    if (timer) {
      clearTimeout(timer)
      timer = null
    }
  }

  onBeforeUnmount(() => pause())

  return {
    activeLine, visited, queued, distances, edges, path, currentNode,
    neighborNode, neighborPriority, heap, status,
    isRunning, isDone, isAtStart, pos, stepCount, visitedCount, queuedCount,
    load, play, pause, reset, stepForward, stepBack,
  }
}
