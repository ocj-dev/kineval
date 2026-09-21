<script setup lang="ts">
import { ref, onMounted, onBeforeUnmount, watch } from 'vue'
import PhaseStepperShell from './PhaseStepperShell.vue'
import JoystickControl from './JoystickControl.vue'
import { usePhaseTracer } from '../lib/cloth/usePhaseTracer'
import { MASTER_PSEUDOCODE, LINE_ACCUMULATE, LINE_INTEGRATE, LINE_RELAX } from '../lib/cloth/pseudocode'
import {
  makeRigid, verletIntegrateRigid, accumulateTorque, satisfyConstraintRigid, worldCorner,
  type RigidState, type Vec2,
} from '../lib/cloth/clothPhysics'
import { drawArrow, drawRigidSquare } from '../lib/cloth/drawUtils'

const WIDTH = 380, HEIGHT = 260
const GRAVITY = 0.5
const FRICTION = 0.975
// same non-grid exception as ConstraintRigidPanel
const HALF = 26
const SIDE = 2 * HALF
const REST_LENGTH = SIDE   // half of the original 2x-side rest length

type Phase = 'accumulate' | 'integrate' | 'relax'
interface Entry { frame: number; phase: Phase; bodyA: RigidState; bodyB: RigidState; vectors: { from: Vec2; to: Vec2; color: string }[] }

const gravityOn = ref(true)
const windOn = ref(false)
const wind = ref({ x: 0, y: 0 })

function initial() {
  const a = makeRigid(120, 50, HALF, true)
  const b = makeRigid(230, 150, HALF, false)
  b.theta = 0.2
  return { a, b }
}
let { a: bodyA, b: bodyB } = initial()
let frameCount = 0

function generateFrame(): Entry[] {
  const entries: Entry[] = []

  bodyB.force_x = 0; bodyB.force_y = 0; bodyB.torque = 0
  const vectors: Entry['vectors'] = []
  if (gravityOn.value) {
    bodyB.force_y += GRAVITY * bodyB.mass
    vectors.push({ from: { x: bodyB.x, y: bodyB.y }, to: { x: bodyB.x, y: bodyB.y + GRAVITY * 24 }, color: '#00274C' })
  }
  if (windOn.value) {
    const c = worldCorner(bodyB, 1)
    accumulateTorque(bodyB, c.x, c.y, wind.value.x, wind.value.y)
    bodyB.force_x += wind.value.x
    bodyB.force_y += wind.value.y
    vectors.push({ from: c, to: { x: c.x + wind.value.x * 20, y: c.y + wind.value.y * 20 }, color: '#1a9e6b' })
  }
  entries.push({ frame: frameCount, phase: 'accumulate', bodyA: { ...bodyA }, bodyB: { ...bodyB }, vectors })

  verletIntegrateRigid(bodyB, FRICTION)
  entries.push({ frame: frameCount, phase: 'integrate', bodyA: { ...bodyA }, bodyB: { ...bodyB }, vectors })

  // a SINGLE constraint (not 2) between the bottom-right corners (index 2),
  // with a nonzero rest length -- so unlike the shared-edge rigid weld, both
  // squares remain free to rotate about this one connection point
  const before = worldCorner(bodyA, 2)
  const c = satisfyConstraintRigid(bodyA, 2, bodyB, 2, 1.0, REST_LENGTH)
  entries.push({
    frame: frameCount, phase: 'relax', bodyA: { ...bodyA }, bodyB: { ...bodyB },
    vectors: [{ from: before, to: { x: before.x + c.x, y: before.y + c.y }, color: '#FFCB05' }],
  })

  frameCount++
  return entries
}

function resetSim() { ({ a: bodyA, b: bodyB } = initial()); frameCount = 0 }

const tracer = usePhaseTracer<Entry>(generateFrame, resetSim)
const { current, trace, pos } = tracer

const activeLineFor: Record<Phase, number> = { accumulate: LINE_ACCUMULATE, integrate: LINE_INTEGRATE, relax: LINE_RELAX }
const labelFor: Record<Phase, string> = {
  accumulate: 'Accumulate forces', integrate: 'Verlet integrate (pos + orientation)', relax: 'Satisfy 1 corner constraint (relax)',
}

const canvasRef = ref<HTMLCanvasElement | null>(null)

// #region mouse-drag-anywhere
// Unlike the particle/node drags elsewhere in this deck (a small pick
// radius around a point), a click anywhere within bodyB's rotated bounds
// picks it up -- the click point is rotated into the body's local frame and
// tested against its half_size square. Only the free body (bodyB) is
// draggable; bodyA stays the fixed anchor throughout.
let dragging = false

function pointInBody(body: RigidState, x: number, y: number): boolean {
  const cos_t = Math.cos(body.theta), sin_t = Math.sin(body.theta)
  const dx = x - body.x, dy = y - body.y
  const localX = dx * cos_t + dy * sin_t
  const localY = -dx * sin_t + dy * cos_t
  return Math.abs(localX) <= body.half_size && Math.abs(localY) <= body.half_size
}

function canvasPos(e: MouseEvent) {
  const canvas = canvasRef.value!
  const rect = canvas.getBoundingClientRect()
  return { x: (e.clientX - rect.left) * (WIDTH / rect.width), y: (e.clientY - rect.top) * (HEIGHT / rect.height) }
}
function onDown(e: MouseEvent) {
  const { x, y } = canvasPos(e)
  if (!pointInBody(bodyB, x, y)) return
  dragging = true
  bodyB.pinned = true
  bodyB.x = x; bodyB.y = y
  current.value.bodyB = { ...bodyB }
}
function onMove(e: MouseEvent) {
  if (!dragging) return
  const { x, y } = canvasPos(e)
  bodyB.x = x; bodyB.y = y
  current.value.bodyB = { ...bodyB }
}
function onUp() {
  if (!dragging) return
  dragging = false
  bodyB.pinned = false
  bodyB.px = bodyB.x; bodyB.py = bodyB.y; bodyB.ptheta = bodyB.theta
}
// #endregion mouse-drag-anywhere

function render() {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  ctx.clearRect(0, 0, WIDTH, HEIGHT)

  const cornersA = [0, 1, 2, 3].map(i => worldCorner(current.value.bodyA, i))
  const cornersB = [0, 1, 2, 3].map(i => worldCorner(current.value.bodyB, i))

  // the rope: a line between the two connected corners
  const ropeA = cornersA[2], ropeB = cornersB[2]
  ctx.strokeStyle = '#7a5c3a'
  ctx.lineWidth = 2
  ctx.setLineDash([6, 4])
  ctx.beginPath(); ctx.moveTo(ropeA.x, ropeA.y); ctx.lineTo(ropeB.x, ropeB.y); ctx.stroke()
  ctx.setLineDash([])

  drawRigidSquare(ctx, cornersA, '#00274Ccc')
  drawRigidSquare(ctx, cornersB, '#FFCB05cc')
  for (const v of current.value.vectors) drawArrow(ctx, v.from.x, v.from.y, v.to.x, v.to.y, v.color)
}

let raf = 0
function loop() { render(); raf = requestAnimationFrame(loop) }
onMounted(() => { raf = requestAnimationFrame(loop) })
onBeforeUnmount(() => cancelAnimationFrame(raf))
watch(current, render)
</script>

<template>
  <PhaseStepperShell
    v-model:smooth="tracer.smooth.value"
    :pseudocode-lines="MASTER_PSEUDOCODE"
    :active-line="activeLineFor[current.phase]"
    :is-running="tracer.isRunning.value" :is-done="tracer.isDone.value" :is-at-start="tracer.isAtStart.value"
    :step-label="labelFor[current.phase]"
    :frame-info="`frame ${current.frame} · step ${pos + 1}/${trace.length}`"
    @play="tracer.play" @pause="tracer.pause" @step-forward="tracer.stepForward" @step-back="tracer.stepBack" @reset="tracer.reset"
  >
    <template #controls>
      <label class="check"><input v-model="gravityOn" type="checkbox"> Gravity</label>
      <label class="check"><input v-model="windOn" type="checkbox"> Wind</label>
      <JoystickControl v-model="wind" :max-magnitude="1.2" label="Wind (mag + dir)" />
    </template>
    <div class="vector-canvas-wrap">
      <canvas
        ref="canvasRef" :width="WIDTH" :height="HEIGHT"
        @mousedown="onDown" @mousemove="onMove" @mouseup="onUp" @mouseleave="onUp"
      />
    </div>
    <div class="legend">
      <span><i style="background:#00274C" /> anchor square</span>
      <span>rope: bottom-right corners, rest length = side length &middot; drag anywhere on the maize square</span>
    </div>
  </PhaseStepperShell>
</template>

<style scoped>
.check { font-family: var(--font-mono, monospace); font-size: 0.72em; display: flex; align-items: center; gap: 0.3em; cursor: pointer; }
.vector-canvas-wrap { flex: 1 1 auto; min-height: 0; border: 1px solid #e3ddc9; border-radius: 10px; background: #fff; }
.vector-canvas-wrap canvas { width: 100%; height: 100%; display: block; cursor: grab; }
.legend { display: flex; gap: 1em; flex-wrap: wrap; font-family: var(--font-mono, monospace); font-size: 0.6em; opacity: 0.75; }
.legend i { display: inline-block; width: 0.8em; height: 0.8em; border-radius: 2px; margin-right: 0.3em; vertical-align: -0.1em; }
</style>
