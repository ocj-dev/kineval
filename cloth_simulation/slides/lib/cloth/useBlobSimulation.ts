import { ref, onBeforeUnmount } from 'vue'
import {
  makeParticle, accumulateForces, verletIntegrateParticle, satisfyConstraintParticle,
  satisfyCollisionParticle, type ParticleState,
} from './clothPhysics'

const GRAVITY = 0.55
const FRICTION = 0.985
const ACCURACY = 5
const STIFFNESS = 1.0

export interface BlobConstraint { i: number; j: number; rest: number }

// Five unanchored particles at the corners of a pentagon, fully connected
// (every pair gets a distance constraint -- 5 choose 2 = 10 constraints),
// falling under gravity onto a ground plane inside the panel's own walls.
// Unlike the constraint-example tracers above, this runs live (Play/Pause
// advance real frames rather than replaying a precomputed trace) since the
// point here is watching the whole system settle, not single pseudocode lines.
export function useBlobSimulation(width: number, height: number) {

  const centerX = width / 2, centerY = height * 0.38, radius = 46
  let particles: ParticleState[] = []
  let constraints: BlobConstraint[] = []
  const groundY = height - 24

  // mutate the arrays in place (length = 0, then push) rather than
  // reassigning `particles`/`constraints` -- the object returned below
  // captures a reference to these two arrays once, at composable-creation
  // time, so reset() must keep writing into that same reference rather than
  // pointing the local variable at a brand-new array the caller never sees.
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
  let raf = 0

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

  function loop() {
    stepFrame()
    if (isRunning.value) raf = requestAnimationFrame(loop)
  }

  function play() { if (isRunning.value) return; isRunning.value = true; raf = requestAnimationFrame(loop) }
  function pause() { isRunning.value = false; cancelAnimationFrame(raf) }
  function reset() { pause(); build(); tick_count.value = 0 }
  function stepOnce() { if (!isRunning.value) stepFrame() }

  onBeforeUnmount(() => cancelAnimationFrame(raf))

  return { particles, constraints, groundY, tick_count, isRunning, play, pause, reset, stepOnce }
}
