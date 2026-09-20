<script setup lang="ts">
import { ref, onMounted, onBeforeUnmount } from 'vue'
import JoystickControl from './JoystickControl.vue'
import {
  makeRigid, verletIntegrateRigid, accumulateTorque, satisfyCollisionRigid, worldCorner,
  type RigidState,
} from '../lib/cloth/clothPhysics'
import { drawArrow, drawRigidSquare, VECTOR_COLORS } from '../lib/cloth/drawUtils'

const WIDTH = 420, HEIGHT = 260
const GRAVITY = 0.6
const FRICTION = 0.99
const SCALE = 20
const HALF = 22

const canvasRef = ref<HTMLCanvasElement | null>(null)
const gravityOn = ref(true)
const windOn = ref(false)
const wind = ref({ x: 0, y: 0 })

let body: RigidState = makeRigid(WIDTH / 2, 50, HALF)
let raf = 0

function reset() { body = makeRigid(WIDTH / 2, 50, HALF) }

function render() {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  ctx.clearRect(0, 0, WIDTH, HEIGHT)

  const corners = [0, 1, 2, 3].map(i => worldCorner(body, i))
  drawRigidSquare(ctx, corners, '#00274C55')

  // gravity acts at the center of mass -- no torque
  if (gravityOn.value)
    drawArrow(ctx, body.x, body.y, body.x, body.y + GRAVITY * SCALE, VECTOR_COLORS.gravity)

  // wind is modeled as striking the top-right corner (off-center), so it
  // visibly produces a torque as well as a translation
  const windCorner = corners[1]
  if (windOn.value) {
    drawArrow(
      ctx, windCorner.x, windCorner.y,
      windCorner.x + wind.value.x * SCALE, windCorner.y + wind.value.y * SCALE,
      VECTOR_COLORS.wind,
    )
  }

  drawRigidSquare(ctx, corners, '#FFCB05cc')
}

function tick() {
  body.force_x = 0; body.force_y = 0; body.torque = 0
  if (gravityOn.value) body.force_y += GRAVITY * body.mass
  if (windOn.value) {
    const c = worldCorner(body, 1)
    accumulateTorque(body, c.x, c.y, wind.value.x, wind.value.y)
    body.force_x += wind.value.x
    body.force_y += wind.value.y
  }
  verletIntegrateRigid(body, FRICTION)
  satisfyCollisionRigid(body, { minX: 10, maxX: WIDTH - 10, minY: 10, maxY: HEIGHT - 10 }, 0.4)
  render()
  raf = requestAnimationFrame(tick)
}

onMounted(() => { render(); raf = requestAnimationFrame(tick) })
onBeforeUnmount(() => cancelAnimationFrame(raf))
</script>

<template>
  <div class="single-rigid-panel">
    <div class="controls">
      <label class="check"><input v-model="gravityOn" type="checkbox"> Gravity (center, no torque)</label>
      <label class="check"><input v-model="windOn" type="checkbox"> Wind (corner, + torque)</label>
      <JoystickControl v-model="wind" :max-magnitude="1.2" label="Wind (mag + dir)" />
      <button class="btn" @click="reset">Reset</button>
    </div>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasRef" :width="WIDTH" :height="HEIGHT" />
    </div>
    <div class="legend">
      <span><i style="background:#00274C" /> gravity (F = ma)</span>
      <span><i style="background:#1a9e6b" /> wind (produces torque about center)</span>
    </div>
  </div>
</template>

<style scoped>
.single-rigid-panel { display: flex; flex-direction: column; gap: 0.5em; height: 100%; }
.controls { display: flex; align-items: center; gap: 1em; flex-wrap: wrap; }
.check { font-family: var(--font-mono, monospace); font-size: 0.72em; display: flex; align-items: center; gap: 0.3em; cursor: pointer; }
.vector-canvas-wrap { flex: 1 1 auto; min-height: 0; border: 1px solid #e3ddc9; border-radius: 10px; background: #fff; }
.vector-canvas-wrap canvas { width: 100%; height: 100%; display: block; }
.legend { display: flex; gap: 1em; font-family: var(--font-mono, monospace); font-size: 0.65em; opacity: 0.75; }
.legend i { display: inline-block; width: 0.8em; height: 0.8em; border-radius: 2px; margin-right: 0.3em; vertical-align: -0.1em; }
</style>
