import { ref, computed, onBeforeUnmount, type Ref, type ComputedRef } from 'vue'

export interface PhaseTracer<T> {
  trace: T[]
  pos: Ref<number>
  current: ComputedRef<T>
  isRunning: Ref<boolean>
  smooth: Ref<boolean>
  isDone: ComputedRef<boolean>
  isAtStart: ComputedRef<boolean>
  play: () => void
  pause: () => void
  reset: () => void
  stepForward: () => boolean
  stepBack: () => void
}

// safety cap on lazily-generated phase entries, since these small systems
// (a hanging constraint, a settling blob) run indefinitely rather than
// terminating -- this just stops an idle, endlessly-Play()ing tab from
// growing the trace array forever
const MAX_ENTRIES = 4000

// Generic play/pause/step/reset engine shared by every interactive panel in
// this deck. `generateFrame()` advances the caller's own physics state by
// exactly one simulation frame and returns that frame's phase snapshots
// (e.g. accumulate/integrate/relax/collide); this composable only owns the
// bookkeeping -- growing the trace lazily, and navigating it -- not the
// physics itself.
//
// The `smooth` flag (bound to the "Smooth (skip step breakdown)" checkbox)
// changes what Play() does: stepped through one phase at a time (220ms
// apart, each rendered so its vectors are visible) when off, or jumping
// straight to the last phase of each newly generated frame (~30ms apart, so
// only the final per-frame state ever renders) when on -- the same
// underlying per-frame phase data either way, just consumed at a different
// granularity.
export function usePhaseTracer<T extends { frame: number }>(
  generateFrame: () => T[],
  resetSim: () => void,
): PhaseTracer<T> {

  const trace: T[] = []
  const pos = ref(0)
  const isRunning = ref(false)
  const smooth = ref(false)
  let timer: ReturnType<typeof setTimeout> | null = null

  const current = computed(() => trace[Math.min(pos.value, trace.length - 1)])
  const isDone = computed(() => trace.length >= MAX_ENTRIES && pos.value >= trace.length - 1)
  const isAtStart = computed(() => pos.value <= 0)

  function ensureNext(): boolean {
    if (pos.value + 1 < trace.length) return true
    if (trace.length >= MAX_ENTRIES) return false
    trace.push(...generateFrame())
    return pos.value + 1 < trace.length
  }

  function stepForward(): boolean {
    if (!ensureNext()) return false
    pos.value++
    return true
  }
  function stepBack() { if (pos.value > 0) pos.value-- }

  function stepFrameJump(): boolean {
    if (pos.value + 1 >= trace.length) {
      if (trace.length >= MAX_ENTRIES) return false
      trace.push(...generateFrame())
    }
    pos.value = trace.length - 1
    return true
  }

  function reset() {
    pause()
    trace.length = 0
    resetSim()
    trace.push(...generateFrame())
    pos.value = 0
  }
  reset()

  function tick() {
    const more = smooth.value ? stepFrameJump() : stepForward()
    if (!more) { pause(); return }
    if (isRunning.value) timer = setTimeout(tick, smooth.value ? 30 : 220)
  }
  function play() {
    if (isRunning.value || isDone.value) return
    isRunning.value = true
    timer = setTimeout(tick, smooth.value ? 30 : 220)
  }
  function pause() {
    isRunning.value = false
    if (timer) { clearTimeout(timer); timer = null }
  }

  onBeforeUnmount(() => pause())

  return { trace, pos, current, isRunning, smooth, isDone, isAtStart, play, pause, reset, stepForward, stepBack }
}
