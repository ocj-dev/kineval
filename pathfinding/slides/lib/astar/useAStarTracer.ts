import { ref, computed, onBeforeUnmount, type Ref } from 'vue'
import { aStarSteps, nodeKey as key } from './aStarSteps'
import type { AStarStep, Scene, SearchAlg } from './types'

interface TraceEntry {
  step: AStarStep
  visited: Set<string>
  queued: Set<string>
  path: { x: number; y: number }[]
}

function buildTrace(scene: Scene, alg: SearchAlg, eps: number): TraceEntry[] {
  const trace: TraceEntry[] = []
  let visited = new Set<string>()
  let queued = new Set<string>()
  let path: { x: number; y: number }[] = []

  for (const step of aStarSteps(scene, alg, eps)) {
    if (step.action === 'visit' && step.node) {
      visited = new Set(visited)
      visited.add(key(step.node.x, step.node.y))
      queued = new Set(queued)
      queued.delete(key(step.node.x, step.node.y))
    }
    if (step.action === 'queue' && step.neighbor) {
      queued = new Set(queued)
      queued.add(key(step.neighbor.x, step.neighbor.y))
    }
    if (step.action === 'succeed' && step.path) {
      path = step.path
    }
    trace.push({ step, visited, queued, path })
  }
  return trace
}

// Every scene/algorithm/eps combination is precomputed into a full trace, so
// Play/Pause/Step Forward/Step Back/Reset are all just index navigation over
// an array that already exists -- mirrors useSorter.ts's approach in the
// sortingthespiderverse deck.
export function useAStarTracer(speedMs: Ref<number> = ref(15)) {
  const activeLine = ref(-1)
  const visited = ref<Set<string>>(new Set())
  const queued = ref<Set<string>>(new Set())
  const path = ref<{ x: number; y: number }[]>([])
  const currentNode = ref<{ x: number; y: number } | null>(null)
  const status = ref<'iterating' | 'succeeded' | 'failed' | 'idle'>('idle')
  const isRunning = ref(false)
  const pos = ref(-1)
  const stepCount = ref(0)

  let trace: TraceEntry[] = []
  let timer: ReturnType<typeof setTimeout> | null = null

  const isDone = computed(() => trace.length > 0 && pos.value >= trace.length - 1)
  const isAtStart = computed(() => pos.value <= -1)
  const visitedCount = computed(() => (pos.value >= 0 && trace[pos.value] ? trace[pos.value].visited.size : 0))

  function applyPos(p: number) {
    pos.value = p
    if (p < 0) {
      activeLine.value = -1
      visited.value = new Set()
      queued.value = new Set()
      path.value = []
      currentNode.value = null
      status.value = 'idle'
      return
    }
    const entry = trace[p]
    activeLine.value = entry.step.line
    visited.value = entry.visited
    queued.value = entry.queued
    path.value = entry.path
    currentNode.value = entry.step.node ?? entry.step.neighbor ?? null
    if (entry.step.action === 'succeed') status.value = 'succeeded'
    else if (entry.step.action === 'fail') status.value = 'failed'
    else status.value = 'iterating'
  }

  function load(scene: Scene, alg: SearchAlg, eps: number) {
    pause()
    trace = buildTrace(scene, alg, eps)
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
    activeLine, visited, queued, path, currentNode, status,
    isRunning, isDone, isAtStart, pos, stepCount, visitedCount,
    load, play, pause, reset, stepForward, stepBack,
  }
}
