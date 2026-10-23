// A GENERIC version of forward_kinematics/slides/lib/fk/useFkTracer.ts's
// precompute-the-whole-trace-then-navigate-by-index pattern: load(steps)
// takes an already-materialized step array (jacobianSteps/iterateIkSteps
// are generators -- callers spread them into an array first, same reason
// FK's useFkTracer pre-runs its own generator into a trace[] before
// exposing Play/Pause/Step/Reset). Reused by both JacobianBuildPanel.vue
// and IkStepPanel.vue rather than writing two near-identical tracers, since
// neither panel needs FK's extra "revealed links" bookkeeping -- just "which
// step am I on".
import { ref, computed, onBeforeUnmount, type Ref } from 'vue'

export function useStepTracer<T>(speedMs: Ref<number> = ref(500)) {
  const pos = ref(-1)
  const steps = ref<T[]>([]) as Ref<T[]>
  const isRunning = ref(false)
  let timer: ReturnType<typeof setTimeout> | null = null

  const stepCount = computed(() => steps.value.length)
  const isDone = computed(() => steps.value.length > 0 && pos.value >= steps.value.length - 1)
  const isAtStart = computed(() => pos.value <= -1)
  const current = computed<T | null>(() => (pos.value >= 0 && pos.value < steps.value.length ? steps.value[pos.value] : null))

  function load(newSteps: T[]) {
    pause()
    steps.value = newSteps
    pos.value = -1
  }

  function reset() {
    pause()
    pos.value = -1
  }

  function stepForward(): boolean {
    if (pos.value >= steps.value.length - 1) return false
    pos.value++
    return pos.value < steps.value.length - 1
  }

  function stepBack() {
    if (pos.value <= -1) return
    pos.value--
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
    if (timer) { clearTimeout(timer); timer = null }
  }

  onBeforeUnmount(() => pause())

  return { pos, steps, current, stepCount, isRunning, isDone, isAtStart, load, play, pause, reset, stepForward, stepBack }
}
