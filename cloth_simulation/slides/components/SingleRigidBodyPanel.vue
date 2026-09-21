<script setup lang="ts">
import { ref, onMounted, onBeforeUnmount, watch } from 'vue'
import PhaseStepperShell from './PhaseStepperShell.vue'
import JoystickControl from './JoystickControl.vue'
import { usePhaseTracer } from '../lib/cloth/usePhaseTracer'
import { MASTER_PSEUDOCODE, LINE_ACCUMULATE, LINE_INTEGRATE } from '../lib/cloth/pseudocode'
import {
  makeRigid, verletIntegrateRigid, accumulateTorque, satisfyCollisionRigid, worldCorner,
  type RigidState, type Vec2,
} from '../lib/cloth/clothPhysics'
import { drawArrow, drawRigidSquare, drawCollisionAreas } from '../lib/cloth/drawUtils'

const WIDTH = 380, HEIGHT = 240
const GRAVITY = 0.6
const FRICTION = 0.99
const SCALE = 20
const HALF = 22
const bounds = { minX: 10, maxX: WIDTH - 10, minY: 10, maxY: HEIGHT - 10 }

interface Entry { frame: number; phase: 'accumulate' | 'integrate'; body: RigidState; vectors: { from: Vec2; to: Vec2; color: string }[] }

const gravityOn = ref(true)
const windOn = ref(false)
const wind = ref({ x: 0, y: 0 })

let body: RigidState = makeRigid(WIDTH / 2, 50, HALF)
let frameCount = 0

function generateFrame(): Entry[] {
  body.force_x = 0; body.force_y = 0; body.torque = 0
  const vectors: Entry['vectors'] = []

  if (gravityOn.value) {
    body.force_y += GRAVITY * body.mass
    vectors.push({ from: { x: body.x, y: body.y }, to: { x: body.x, y: body.y + GRAVITY * SCALE }, color: '#00274C' })
  }
  if (windOn.value) {
    const c = worldCorner(body, 1)
    accumulateTorque(body, c.x, c.y, wind.value.x, wind.value.y)
    body.force_x += wind.value.x
    body.force_y += wind.value.y
    vectors.push({ from: c, to: { x: c.x + wind.value.x * SCALE, y: c.y + wind.value.y * SCALE }, color: '#1a9e6b' })
  }

  const accEntry: Entry = { frame: frameCount, phase: 'accumulate', body: { ...body }, vectors }

  verletIntegrateRigid(body, FRICTION)
  satisfyCollisionRigid(body, bounds, 0.4)
  const intEntry: Entry = { frame: frameCount, phase: 'integrate', body: { ...body }, vectors }

  frameCount++
  return [accEntry, intEntry]
}

function resetSim() { body = makeRigid(WIDTH / 2, 50, HALF); frameCount = 0 }

const tracer = usePhaseTracer<Entry>(generateFrame, resetSim)
const { current, trace, pos } = tracer

const canvasRef = ref<HTMLCanvasElement | null>(null)

function render() {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  ctx.clearRect(0, 0, WIDTH, HEIGHT)
  drawCollisionAreas(ctx, WIDTH, HEIGHT, bounds)

  const corners = [0, 1, 2, 3].map(i => worldCorner(current.value.body, i))
  drawRigidSquare(ctx, corners, '#00274C55')
  for (const v of current.value.vectors) drawArrow(ctx, v.from.x, v.from.y, v.to.x, v.to.y, v.color)
  drawRigidSquare(ctx, corners, '#FFCB05cc')
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
    :active-line="current.phase === 'accumulate' ? LINE_ACCUMULATE : LINE_INTEGRATE"
    :is-running="tracer.isRunning.value" :is-done="tracer.isDone.value" :is-at-start="tracer.isAtStart.value"
    :step-label="current.phase === 'accumulate' ? 'Accumulate forces + torque' : 'Verlet integrate (pos + orientation)'"
    :frame-info="`frame ${current.frame} · step ${pos + 1}/${trace.length}`"
    @play="tracer.play" @pause="tracer.pause" @step-forward="tracer.stepForward" @step-back="tracer.stepBack" @reset="tracer.reset"
  >
    <template #controls>
      <label class="check"><input v-model="gravityOn" type="checkbox"> Gravity (center, no torque)</label>
      <label class="check"><input v-model="windOn" type="checkbox"> Wind (corner, + torque)</label>
      <JoystickControl v-model="wind" :max-magnitude="1.2" label="Wind (mag + dir)" />
    </template>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasRef" :width="WIDTH" :height="HEIGHT" />
    </div>
    <div class="legend">
      <span><i style="background:#00274C" /> gravity (F = ma)</span>
      <span><i style="background:#1a9e6b" /> wind (produces torque about center)</span>
      <span><i style="background:#9a9a9a" /> collision area (wall)</span>
    </div>
  </PhaseStepperShell>
</template>

<style scoped>
.check { font-family: var(--font-mono, monospace); font-size: 0.72em; display: flex; align-items: center; gap: 0.3em; cursor: pointer; }
.vector-canvas-wrap { flex: 1 1 auto; min-height: 0; border: 1px solid #e3ddc9; border-radius: 10px; background: #fff; }
.vector-canvas-wrap canvas { width: 100%; height: 100%; display: block; }
.legend { display: flex; gap: 1em; flex-wrap: wrap; font-family: var(--font-mono, monospace); font-size: 0.62em; opacity: 0.75; }
.legend i { display: inline-block; width: 0.8em; height: 0.8em; border-radius: 2px; margin-right: 0.3em; vertical-align: -0.1em; }
</style>
