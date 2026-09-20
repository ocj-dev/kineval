import { ref, computed, onBeforeUnmount } from 'vue'
import {
  makeRigid, verletIntegrateRigid, satisfyConstraintRigid, worldCorner, type RigidState, type Vec2,
} from './clothPhysics'

export type ConstraintPhase = 'accumulate' | 'integrate' | 'relax'

export interface ConstraintRigidTraceEntry {
  phase: ConstraintPhase
  frame: number
  bodyA: RigidState
  bodyB: RigidState
  vectors: { from: Vec2; to: Vec2; color: string }[]
}

const GRAVITY = 0.5
const FRICTION = 0.97
const HALF = 26
const SCALE = 24

// Two rigid squares sharing an edge: bodyA is pinned (fixed position AND
// orientation, since verletIntegrateRigid() and applyCornerCorrection() both
// skip pinned bodies entirely), bodyB starts displaced and rotated away from
// the edge-aligned rest configuration so relaxation visibly pulls it back
// into both position and orientation agreement over several frames.
export function buildConstraintRigidTrace(): ConstraintRigidTraceEntry[] {

  const bodyA = makeRigid(140, 60, HALF, true)
  const bodyB = makeRigid(230, 110, HALF, false)
  bodyB.theta = 0.5

  const trace: ConstraintRigidTraceEntry[] = []
  const frames = 34

  for (let frame = 0; frame < frames; frame++) {

    bodyB.force_x = 0; bodyB.force_y = GRAVITY * bodyB.mass; bodyB.torque = 0
    trace.push({
      phase: 'accumulate', frame,
      bodyA: { ...bodyA }, bodyB: { ...bodyB },
      vectors: [{ from: { x: bodyB.x, y: bodyB.y }, to: { x: bodyB.x, y: bodyB.y + GRAVITY * SCALE }, color: '#00274C' }],
    })

    const forceVec = { from: { x: bodyB.x, y: bodyB.y }, to: { x: bodyB.x, y: bodyB.y + GRAVITY * SCALE }, color: '#00274C' }
    verletIntegrateRigid(bodyB, FRICTION)
    trace.push({ phase: 'integrate', frame, bodyA: { ...bodyA }, bodyB: { ...bodyB }, vectors: [forceVec] })

    const beforeA1 = worldCorner(bodyA, 1), beforeA2 = worldCorner(bodyA, 2)
    const c1 = satisfyConstraintRigid(bodyA, 1, bodyB, 0, 1.0)
    const c2 = satisfyConstraintRigid(bodyA, 2, bodyB, 3, 1.0)
    trace.push({
      phase: 'relax', frame, bodyA: { ...bodyA }, bodyB: { ...bodyB },
      vectors: [
        { from: beforeA1, to: { x: beforeA1.x + c1.x, y: beforeA1.y + c1.y }, color: '#FFCB05' },
        { from: beforeA2, to: { x: beforeA2.x + c2.x, y: beforeA2.y + c2.y }, color: '#FFCB05' },
      ],
    })
  }

  return trace
}

export function useConstraintRigidTracer() {

  const trace = buildConstraintRigidTrace()
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
