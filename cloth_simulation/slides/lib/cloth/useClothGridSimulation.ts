import { ref, watch, onBeforeUnmount, type Ref } from 'vue'
import {
  makeParticle, makeRigid, accumulateForces, verletIntegrateParticle, verletIntegrateRigid,
  satisfyConstraintParticle, satisfyConstraintRigid, satisfyCollisionParticle, satisfyCollisionRigid,
  type ParticleState, type RigidState,
} from './clothPhysics'
import { michiganNodeColor } from './michiganColor'

const GRAVITY = 0.45
const FRICTION = 0.98
const ACCURACY = 4
// rigid squares default to a lower stiffness than particles: each interior
// square carries up to 8 corner constraints at once, so full-strength
// relaxation with only ACCURACY passes is visibly jittery (see cloth.js's
// build-cloth region for the corner_rest fix that removed the worse,
// diverging half of this -- the remaining jitter at high stiffness is just
// inherent to a densely-coupled Gauss-Seidel solve with few iterations)
const PARTICLE_STIFFNESS = 1.0
const RIGID_STIFFNESS = 0.4
const LOW_STIFFNESS = 0.25
const SPACING = 46
const GRID_N = 3
const DRAG_PICK_RADIUS = 30
const HALF_SIZE = SPACING * 0.75 / 2   // rigid squares fill 75% of spacing
// the actual gap between adjacent squares' touching corners at
// construction -- used as the corner constraints' rest length so relaxation
// isn't fighting a pre-stretched grid
const CORNER_REST = SPACING - 2 * HALF_SIZE

interface PConstraint { i: number; j: number; rest: number }
interface RConstraint { a: number; cornerA: number; b: number; cornerB: number }
export interface GridParticle extends ParticleState { color: string }

// A small 3x3 grid, same construction as reference/cloth.js's Cloth(): top
// row pinned, particle nodes with distance constraints or rigid-square nodes
// with 2 corner constraints per shared edge, depending on nodeType. Rigid
// squares are sized 75% of the grid spacing (25% gap between neighbors),
// with the corner constraints' rest length set to that same gap (CORNER_REST)
// so the grid isn't pre-stretched from the moment it's built.
export function useClothGridSimulation(width: number, height: number, nodeType: Ref<'particle' | 'rigid'>) {

  const startX = width / 2 - (SPACING * (GRID_N - 1)) / 2
  const startY = 30

  const particles: GridParticle[] = []
  const pConstraints: PConstraint[] = []
  const rigids: RigidState[] = []
  const rConstraints: RConstraint[] = []
  const pinnedFlags: boolean[] = []   // parallel to whichever array is active, so a drag can restore pin state
  const bounds = { minX: 12, maxX: width - 12, minY: 12, maxY: height - 12 }

  function build() {
    particles.length = 0
    pConstraints.length = 0
    rigids.length = 0
    rConstraints.length = 0
    pinnedFlags.length = 0

    const grid: number[][] = []
    for (let row = 0; row < GRID_N; row++) {
      grid[row] = []
      for (let col = 0; col < GRID_N; col++) {
        const x = startX + col * SPACING
        const y = startY + row * SPACING
        const pinned = row === 0

        if (nodeType.value === 'rigid') {
          const body = makeRigid(x, y, HALF_SIZE, pinned)
          rigids.push(body)
          pinnedFlags.push(pinned)
          grid[row][col] = rigids.length - 1
          // adjacent squares' nearest corners are held CORNER_REST apart
          // (the real gap between them at construction), not coincident
          if (col > 0) {
            rConstraints.push({ a: grid[row][col - 1], cornerA: 1, b: grid[row][col], cornerB: 0 })
            rConstraints.push({ a: grid[row][col - 1], cornerA: 2, b: grid[row][col], cornerB: 3 })
          }
          if (row > 0) {
            rConstraints.push({ a: grid[row - 1][col], cornerA: 3, b: grid[row][col], cornerB: 0 })
            rConstraints.push({ a: grid[row - 1][col], cornerA: 2, b: grid[row][col], cornerB: 1 })
          }
        } else {
          const p = makeParticle(x, y, pinned) as GridParticle
          p.color = michiganNodeColor(col, row, GRID_N, GRID_N)
          particles.push(p)
          pinnedFlags.push(pinned)
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
  const smooth = ref(true)
  const lowStiffness = ref(false)
  const enforceConstraints = ref(true)
  let timer: ReturnType<typeof setTimeout> | null = null

  // #region mouse-drag-constraint
  // Same idea as the blob panel: while the mouse is down near a node, that
  // node is pinned directly to the mouse position every frame (a stiff
  // location constraint), for either node type -- for a rigid square this
  // also freezes its orientation while dragged, the same way any other
  // pinned rigid body does.
  const dragIndex = ref(-1)

  function activeNodes(): (ParticleState | RigidState)[] {
    return nodeType.value === 'rigid' ? rigids : particles
  }

  function mouseDown(x: number, y: number) {
    const nodes = activeNodes()
    let best = -1, bestDist = DRAG_PICK_RADIUS
    nodes.forEach((n, i) => {
      const d = Math.hypot(n.x - x, n.y - y)
      if (d < bestDist) { bestDist = d; best = i }
    })
    if (best >= 0) {
      dragIndex.value = best
      nodes[best].pinned = true
      nodes[best].x = x; nodes[best].y = y
    }
  }
  function mouseMove(x: number, y: number) {
    if (dragIndex.value < 0) return
    const n = activeNodes()[dragIndex.value]
    n.x = x; n.y = y
  }
  function mouseUp() {
    if (dragIndex.value < 0) return
    const n = activeNodes()[dragIndex.value] as ParticleState | RigidState
    n.pinned = pinnedFlags[dragIndex.value]   // restore this node's original pin state, not always "free"
    n.px = n.x; n.py = n.y
    if ('ptheta' in n) n.ptheta = n.theta
    dragIndex.value = -1
  }
  // #endregion mouse-drag-constraint

  function stepFrame() {
    const baseStiffness = nodeType.value === 'rigid' ? RIGID_STIFFNESS : PARTICLE_STIFFNESS
    const stiffness = lowStiffness.value ? LOW_STIFFNESS : baseStiffness
    if (nodeType.value === 'rigid') {
      for (const b of rigids) {
        b.force_x = 0; b.force_y = GRAVITY * b.mass; b.torque = 0
        verletIntegrateRigid(b, FRICTION)
      }
      for (let pass = 0; pass < ACCURACY; pass++) {
        if (enforceConstraints.value)
          for (const c of rConstraints) satisfyConstraintRigid(rigids[c.a], c.cornerA, rigids[c.b], c.cornerB, stiffness, CORNER_REST)
        for (const b of rigids) satisfyCollisionRigid(b, bounds, 0.4)
      }
    } else {
      for (const p of particles) {
        accumulateForces(p, { gravityOn: true, gravity: GRAVITY, windOn: false, wind: { x: 0, y: 0 } })
        verletIntegrateParticle(p, FRICTION)
      }
      for (let pass = 0; pass < ACCURACY; pass++) {
        if (enforceConstraints.value)
          for (const c of pConstraints) satisfyConstraintParticle(particles[c.i], particles[c.j], c.rest, stiffness)
        for (const p of particles) satisfyCollisionParticle(p, bounds, 0.4)
      }
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

  watch(nodeType, () => reset())

  onBeforeUnmount(() => pause())

  return {
    particles, pConstraints, rigids, rConstraints, bounds, tick_count, isRunning, smooth, lowStiffness, enforceConstraints,
    play, pause, reset, stepOnce, build, mouseDown, mouseMove, mouseUp, dragIndex,
  }
}
