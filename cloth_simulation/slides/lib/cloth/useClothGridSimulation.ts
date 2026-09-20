import { ref, watch, onBeforeUnmount, type Ref } from 'vue'
import {
  makeParticle, makeRigid, accumulateForces, verletIntegrateParticle, verletIntegrateRigid,
  satisfyConstraintParticle, satisfyConstraintRigid, satisfyCollisionParticle, satisfyCollisionRigid,
  type ParticleState, type RigidState,
} from './clothPhysics'

const GRAVITY = 0.45
const FRICTION = 0.98
const ACCURACY = 4
const STIFFNESS = 1.0
const SPACING = 46
const GRID_N = 3

interface PConstraint { i: number; j: number; rest: number }
interface RConstraint { a: number; cornerA: number; b: number; cornerB: number }

// A small 3x3 grid, same construction as reference/cloth.js's Cloth(): top
// row pinned, particle nodes with distance constraints or rigid-square nodes
// with 2 corner constraints per shared edge, depending on nodeType.
export function useClothGridSimulation(width: number, height: number, nodeType: Ref<'particle' | 'rigid'>) {

  const startX = width / 2 - (SPACING * (GRID_N - 1)) / 2
  const startY = 30

  // declared once and mutated in place by build() (length = 0, then push) --
  // never reassigned -- so the arrays captured by the object this composable
  // returns stay valid across reset()/nodeType changes instead of going stale
  const particles: ParticleState[] = []
  const pConstraints: PConstraint[] = []
  const rigids: RigidState[] = []
  const rConstraints: RConstraint[] = []

  function build() {
    particles.length = 0
    pConstraints.length = 0
    rigids.length = 0
    rConstraints.length = 0

    const grid: number[][] = []
    for (let row = 0; row < GRID_N; row++) {
      grid[row] = []
      for (let col = 0; col < GRID_N; col++) {
        const x = startX + col * SPACING
        const y = startY + row * SPACING
        const pinned = row === 0

        if (nodeType.value === 'rigid') {
          const body = makeRigid(x, y, (SPACING - 1) / 2, pinned)
          rigids.push(body)
          grid[row][col] = rigids.length - 1
          if (col > 0) {
            rConstraints.push({ a: grid[row][col - 1], cornerA: 1, b: grid[row][col], cornerB: 0 })
            rConstraints.push({ a: grid[row][col - 1], cornerA: 2, b: grid[row][col], cornerB: 3 })
          }
          if (row > 0) {
            rConstraints.push({ a: grid[row - 1][col], cornerA: 3, b: grid[row][col], cornerB: 0 })
            rConstraints.push({ a: grid[row - 1][col], cornerA: 2, b: grid[row][col], cornerB: 1 })
          }
        } else {
          const p = makeParticle(x, y, pinned)
          particles.push(p)
          grid[row][col] = particles.length - 1
          if (col > 0) pConstraints.push({ i: grid[row][col - 1], j: particles.length - 1, rest: SPACING })
          if (row > 0) pConstraints.push({ i: grid[row - 1][col], j: particles.length - 1, rest: SPACING })
        }
      }
    }
  }
  build()

  const tick_count = ref(0)
  const isRunning = ref(false)
  let raf = 0

  function stepFrame() {
    if (nodeType.value === 'rigid') {
      for (const b of rigids) {
        b.force_x = 0; b.force_y = GRAVITY * b.mass; b.torque = 0
        verletIntegrateRigid(b, FRICTION)
      }
      for (let pass = 0; pass < ACCURACY; pass++) {
        for (const c of rConstraints) satisfyConstraintRigid(rigids[c.a], c.cornerA, rigids[c.b], c.cornerB, STIFFNESS)
        for (const b of rigids) satisfyCollisionRigid(b, { minX: 12, maxX: width - 12, minY: 12, maxY: height - 12 }, 0.4)
      }
    } else {
      for (const p of particles) {
        accumulateForces(p, { gravityOn: true, gravity: GRAVITY, windOn: false, wind: { x: 0, y: 0 } })
        verletIntegrateParticle(p, FRICTION)
      }
      for (let pass = 0; pass < ACCURACY; pass++) {
        for (const c of pConstraints) satisfyConstraintParticle(particles[c.i], particles[c.j], c.rest, STIFFNESS)
        for (const p of particles) satisfyCollisionParticle(p, { minX: 12, maxX: width - 12, minY: 12, maxY: height - 12 }, 0.4)
      }
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

  watch(nodeType, () => reset())

  onBeforeUnmount(() => cancelAnimationFrame(raf))

  return { particles, pConstraints, rigids, rConstraints, tick_count, isRunning, play, pause, reset, stepOnce, build }
}
