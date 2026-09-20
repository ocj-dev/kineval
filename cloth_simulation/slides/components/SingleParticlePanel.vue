<script setup lang="ts">
import { ref, onMounted, onBeforeUnmount } from 'vue'
import JoystickControl from './JoystickControl.vue'
import { makeParticle, accumulateForces, verletIntegrateParticle, satisfyCollisionParticle, type ParticleState } from '../lib/cloth/clothPhysics'
import { drawArrow, drawParticleDot, VECTOR_COLORS } from '../lib/cloth/drawUtils'

const WIDTH = 420, HEIGHT = 260
const GRAVITY = 0.6
const FRICTION = 0.995
const SCALE = 20   // pixels per unit of force, for the vector overlay

const canvasRef = ref<HTMLCanvasElement | null>(null)
const gravityOn = ref(true)
const windOn = ref(false)
const wind = ref({ x: 0, y: 0 })

let p: ParticleState = makeParticle(WIDTH / 2, 40)
let raf = 0

function reset() { p = makeParticle(WIDTH / 2, 40) }

function render() {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  ctx.clearRect(0, 0, WIDTH, HEIGHT)

  const gvx = 0, gvy = gravityOn.value ? GRAVITY * SCALE : 0
  const wvx = windOn.value ? wind.value.x * SCALE : 0
  const wvy = windOn.value ? wind.value.y * SCALE : 0

  if (gravityOn.value) drawArrow(ctx, p.x, p.y, p.x + gvx, p.y + gvy, VECTOR_COLORS.gravity)
  if (windOn.value) drawArrow(ctx, p.x, p.y, p.x + wvx, p.y + wvy, VECTOR_COLORS.wind)
  if (gravityOn.value && windOn.value)
    drawArrow(ctx, p.x, p.y, p.x + gvx + wvx, p.y + gvy + wvy, VECTOR_COLORS.resultant, 1.5)

  drawParticleDot(ctx, p.x, p.y, '#00274C', 7)
}

function tick() {
  accumulateForces(p, { gravityOn: gravityOn.value, gravity: GRAVITY, windOn: windOn.value, wind: wind.value })
  verletIntegrateParticle(p, FRICTION)
  satisfyCollisionParticle(p, { minX: 10, maxX: WIDTH - 10, minY: 10, maxY: HEIGHT - 10 }, 0.5)
  render()
  raf = requestAnimationFrame(tick)
}

onMounted(() => { render(); raf = requestAnimationFrame(tick) })
onBeforeUnmount(() => cancelAnimationFrame(raf))
</script>

<template>
  <div class="single-particle-panel">
    <div class="controls">
      <label class="check"><input v-model="gravityOn" type="checkbox"> Gravity</label>
      <label class="check"><input v-model="windOn" type="checkbox"> Wind</label>
      <JoystickControl v-model="wind" :max-magnitude="1.2" label="Wind (mag + dir)" />
      <button class="btn" @click="reset">Reset</button>
    </div>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasRef" :width="WIDTH" :height="HEIGHT" />
    </div>
    <div class="legend">
      <span><i style="background:#00274C" /> gravity</span>
      <span><i style="background:#1a9e6b" /> wind</span>
      <span><i style="background:#c1272d" /> resultant (F = ma)</span>
    </div>
  </div>
</template>

<style scoped>
.single-particle-panel { display: flex; flex-direction: column; gap: 0.5em; height: 100%; }
.controls { display: flex; align-items: center; gap: 1em; flex-wrap: wrap; }
.check { font-family: var(--font-mono, monospace); font-size: 0.75em; display: flex; align-items: center; gap: 0.3em; cursor: pointer; }
.vector-canvas-wrap { flex: 1 1 auto; min-height: 0; border: 1px solid #e3ddc9; border-radius: 10px; background: #fff; }
.vector-canvas-wrap canvas { width: 100%; height: 100%; display: block; }
.legend { display: flex; gap: 1em; font-family: var(--font-mono, monospace); font-size: 0.65em; opacity: 0.75; }
.legend i { display: inline-block; width: 0.8em; height: 0.8em; border-radius: 2px; margin-right: 0.3em; vertical-align: -0.1em; }
</style>
