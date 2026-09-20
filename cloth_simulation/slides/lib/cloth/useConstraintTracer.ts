import { ref, computed, onBeforeUnmount } from 'vue'
import {
  makeParticle, accumulateForces, verletIntegrateParticle, satisfyConstraintParticle,
  satisfyCollisionParticle, type ParticleState, type Vec2,
} from './clothPhysics'

export type ConstraintPhase = 'accumulate' | 'integrate' | 'relax' | 'collide'

export interface ConstraintTraceEntry {
  phase: ConstraintPhase
  frame: number
  p1: Vec2
  p2: Vec2
  vectors: { from: Vec2; to: Vec2; color: string }[]
}

const GRAVITY = 0.5
const FRICTION = 0.98
const REST_LENGTH = 70
const SCALE = 26

// Builds a fixed-length trace: one anchored particle (p1) and one free
// particle (p2), stepped frame by frame through the same
// accumulate/integrate/relax(/collide) phases as the reference
// implementation's simulateStep(), so Play/Step/Reset are just index
// navigation over a precomputed array -- the same approach as
// pathfinding/slides/lib/astar/useAStarTracer.ts.
export function buildConstraintTrace(opts: { collision: boolean; bounds: { minX: number; maxX: number; minY: number; maxY: number } }): ConstraintTraceEntry[] {

  const p1 = makeParticle(140, 40, true)   // anchor
  const p2 = makeParticle(210, 90, false)  // free, starts stretched past rest length

  const trace: ConstraintTraceEntry[] = []
  const frames = opts.collision ? 46 : 30

  for (let frame = 0; frame < frames; frame++) {

    accumulateForces(p2, { gravityOn: true, gravity: GRAVITY, windOn: false, wind: { x: 0, y: 0 } })
    trace.push({
      phase: 'accumulate', frame, p1: { ...p1 }, p2: { ...p2 },
      vectors: [{ from: { x: p2.x, y: p2.y }, to: { x: p2.x, y: p2.y + GRAVITY * SCALE }, color: '#00274C' }],
    })

    const forceVec = { from: { x: p2.x, y: p2.y }, to: { x: p2.x, y: p2.y + GRAVITY * SCALE }, color: '#00274C' }
    verletIntegrateParticle(p2, FRICTION)
    trace.push({ phase: 'integrate', frame, p1: { ...p1 }, p2: { ...p2 }, vectors: [forceVec] })

    const correction = satisfyConstraintParticle(p1, p2, REST_LENGTH, 1.0)
    trace.push({
      phase: 'relax', frame, p1: { ...p1 }, p2: { ...p2 },
      vectors: [{
        from: { x: p2.x + correction.x, y: p2.y + correction.y }, to: { x: p2.x, y: p2.y },
        color: '#FFCB05',
      }],
    })

    if (opts.collision) {
      const response = satisfyCollisionParticle(p2, { ...opts.bounds, groundY: opts.bounds.maxY }, 0.5)
      trace.push({
        phase: 'collide', frame, p1: { ...p1 }, p2: { ...p2 },
        vectors: (response.x !== 0 || response.y !== 0)
          ? [{ from: { x: p2.x - response.x, y: p2.y - response.y }, to: { x: p2.x, y: p2.y }, color: '#7d3ac1' }]
          : [],
      })
    }
  }

  return trace
}

export function useConstraintTracer(collision: boolean, bounds: { minX: number; maxX: number; minY: number; maxY: number }) {

  const trace = buildConstraintTrace({ collision, bounds })
  const pos = ref(0)
  const isRunning = ref(false)
  let timer: ReturnType<typeof setTimeout> | null = null

  const current = computed(() => trace[pos.value])
  const isDone = computed(() => pos.value >= trace.length - 1)
  const isAtStart = computed(() => pos.value <= 0)

  function stepForward() { if (pos.value < trace.length - 1) pos.value++ }
  function stepBack() { if (pos.value > 0) pos.value-- }
  function reset() { pause(); pos.value = 0 }

  function scheduleNext() {
    timer = setTimeout(() => {
      stepForward()
      if (!isDone.value && isRunning.value) scheduleNext()
      else pause()
    }, 220)
  }
  function play() { if (isRunning.value || isDone.value) return; isRunning.value = true; scheduleNext() }
  function pause() { isRunning.value = false; if (timer) { clearTimeout(timer); timer = null } }

  onBeforeUnmount(() => pause())

  return { trace, pos, current, isRunning, isDone, isAtStart, play, pause, reset, stepForward, stepBack }
}
