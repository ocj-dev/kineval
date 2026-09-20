import { ref, onBeforeUnmount } from 'vue'
import {
  makeParticle, accumulateForces, verletIntegrateParticle, satisfyConstraintParticle,
  satisfyCollisionParticle, type ParticleState,
} from './clothPhysics'

const GRAVITY = 0.55
const FRICTION = 0.985
const ACCURACY = 5
const STIFFNESS = 1.0
const DRAG_PICK_RADIUS = 28

export interface BlobConstraint { i: number; j: number; rest: number }

// Five unanchored particles at the corners of a pentagon, fully connected
// (every pair gets a distance constraint -- 5 choose 2 = 10 constraints),
// falling under gravity onto a ground plane inside the panel's own walls.
export function useBlobSimulation(width: number, height: number) {

  const centerX = width / 2, centerY = height * 0.38, radius = 46
  const particles: ParticleState[] = []
  const constraints: BlobConstraint[] = []
  const groundY = height - 24

  function build() {
    particles.length = 0
    for (let k = 0; k < 5; k++) {
      const angle = -Math.PI / 2 + (k * 2 * Math.PI) / 5
      particles.push(makeParticle(centerX + radius * Math.cos(angle), centerY + radius * Math.sin(angle)))
    }
    constraints.length = 0
    for (let i = 0; i < 5; i++) {
      for (let j = i + 1; j < 5; j++) {
        const dx = particles[i].x - particles[j].x, dy = particles[i].y - particles[j].y
        constraints.push({ i, j, rest: Math.sqrt(dx * dx + dy * dy) })
      }
    }
  }
  build()

  const tick_count = ref(0)
  const isRunning = ref(false)
  const smooth = ref(true)
  let timer: ReturnType<typeof setTimeout> | null = null

  // #region mouse-drag-constraint
  // While the mouse is down near a node, that node is pinned directly to
  // the mouse position every frame -- an (infinitely) stiff location
  // constraint -- rather than nudged like the reference implementation's
  // velocity-imparting drag. Releasing un-pins it and zeroes its implicit
  // Verlet velocity so it doesn't snap away from wherever it was dropped.
  const dragIndex = ref(-1)

  function mouseDown(x: number, y: number) {
    let best = -1, bestDist = DRAG_PICK_RADIUS
    particles.forEach((p, i) => {
      const d = Math.hypot(p.x - x, p.y - y)
      if (d < bestDist) { bestDist = d; best = i }
    })
    if (best >= 0) {
      dragIndex.value = best
      particles[best].pinned = true
      particles[best].x = x; particles[best].y = y
    }
  }
  function mouseMove(x: number, y: number) {
    if (dragIndex.value < 0) return
    const p = particles[dragIndex.value]
    p.x = x; p.y = y
  }
  function mouseUp() {
    if (dragIndex.value < 0) return
    const p = particles[dragIndex.value]
    p.pinned = false
    p.px = p.x; p.py = p.y
    dragIndex.value = -1
  }
  // #endregion mouse-drag-constraint

  function stepFrame() {
    for (const p of particles) {
      accumulateForces(p, { gravityOn: true, gravity: GRAVITY, windOn: false, wind: { x: 0, y: 0 } })
      verletIntegrateParticle(p, FRICTION)
    }
    for (let pass = 0; pass < ACCURACY; pass++) {
      for (const c of constraints) satisfyConstraintParticle(particles[c.i], particles[c.j], c.rest, STIFFNESS)
      for (const p of particles)
        satisfyCollisionParticle(p, { minX: 12, maxX: width - 12, minY: 12, maxY: height - 12, groundY }, 0.45)
    }
    tick_count.value++
  }

  function tick() {
    stepFrame()
    if (isRunning.value) timer = setTimeout(tick, smooth.value ? 16 : 300)
  }
  function play() { if (isRunning.value) return; isRunning.value = true; timer = setTimeout(tick, smooth.value ? 16 : 300) }
  function pause() { isRunning.value = false; if (timer) { clearTimeout(timer); timer = null } }
  function reset() { pause(); dragIndex.value = -1; build(); tick_count.value = 0 }
  function stepOnce() { if (!isRunning.value) stepFrame() }

  onBeforeUnmount(() => pause())

  return {
    particles, constraints, groundY, tick_count, isRunning, smooth,
    play, pause, reset, stepOnce, mouseDown, mouseMove, mouseUp, dragIndex,
  }
}
