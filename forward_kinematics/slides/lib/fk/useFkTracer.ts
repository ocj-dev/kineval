// Precomputes the full DFS trace for a robot pose and exposes Play/Pause/
// Step Forward/Step Back/Reset as index navigation over it -- same
// approach pathfinding's useAStarTracer.ts and simulation's
// usePhaseTracer.ts both take. Call load(robot) again whenever a joint
// angle slider changes, to re-trace under the new pose.
import { ref, computed, onBeforeUnmount, type Ref } from 'vue'
import { fkSteps } from './fkSteps'
import type { FKStep, FKFrame, Mat4, RobotDesc } from './types'

interface TraceEntry {
  step: FKStep
  revealed: Map<string, Mat4> // every link whose matrix has been finalized by this point in the trace
}

function buildTrace(robot: RobotDesc): TraceEntry[] {
  const trace: TraceEntry[] = []
  let revealed = new Map<string, Mat4>()

  for (const step of fkSteps(robot)) {
    if ((step.action === 'base' || step.action === 'link-enter') && step.linkName && step.matrix) {
      revealed = new Map(revealed)
      revealed.set(step.linkName, step.matrix)
    }
    trace.push({ step, revealed })
  }
  return trace
}

export function useFkTracer(speedMs: Ref<number> = ref(400)) {
  const activeLine = ref(-1)
  const currentLinkName = ref<string | null>(null)
  const currentJointName = ref<string | null>(null)
  const depth = ref(0)
  const revealedLinks = ref<Map<string, Mat4>>(new Map())
  const stack = ref<FKFrame[]>([])
  const currentMatrix = ref<Mat4 | null>(null)
  const isRunning = ref(false)
  const pos = ref(-1)
  const stepCount = ref(0)

  let trace: TraceEntry[] = []
  let timer: ReturnType<typeof setTimeout> | null = null

  const isDone = computed(() => trace.length > 0 && pos.value >= trace.length - 1)
  const isAtStart = computed(() => pos.value <= -1)

  function applyPos(p: number) {
    pos.value = p
    if (p < 0) {
      activeLine.value = -1
      currentLinkName.value = null
      currentJointName.value = null
      depth.value = 0
      revealedLinks.value = new Map()
      stack.value = []
      currentMatrix.value = null
      return
    }
    const entry = trace[p]
    activeLine.value = entry.step.line
    currentLinkName.value = entry.step.linkName ?? null
    currentJointName.value = entry.step.jointName ?? null
    depth.value = entry.step.depth
    revealedLinks.value = entry.revealed
    stack.value = entry.step.stack
    currentMatrix.value = entry.step.matrix ?? null
  }

  function load(robot: RobotDesc) {
    pause()
    trace = buildTrace(robot)
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
    activeLine, currentLinkName, currentJointName, depth, revealedLinks, stack, currentMatrix,
    isRunning, isDone, isAtStart, pos, stepCount,
    load, play, pause, reset, stepForward, stepBack,
  }
}
