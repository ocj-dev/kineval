<script setup lang="ts">
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { useClothGridSimulation } from '../lib/cloth/useClothGridSimulation'
import { worldCorner } from '../lib/cloth/clothPhysics'
import { drawParticleDot, drawRigidSquare, drawCollisionAreas } from '../lib/cloth/drawUtils'

const WIDTH = 420, HEIGHT = 300
const nodeType = ref<'particle' | 'rigid'>('particle')

const {
  particles, pConstraints, rigids, rConstraints, bounds, tick_count, isRunning, smooth, lowStiffness, enforceConstraints,
  play, pause, reset, stepOnce, mouseDown, mouseMove, mouseUp, dragIndex,
} = useClothGridSimulation(WIDTH, HEIGHT, nodeType)

const canvasRef = ref<HTMLCanvasElement | null>(null)
let drawRaf = 0

function canvasPos(e: MouseEvent) {
  const canvas = canvasRef.value!
  const rect = canvas.getBoundingClientRect()
  return { x: (e.clientX - rect.left) * (WIDTH / rect.width), y: (e.clientY - rect.top) * (HEIGHT / rect.height) }
}
function onDown(e: MouseEvent) { const { x, y } = canvasPos(e); mouseDown(x, y) }
function onMove(e: MouseEvent) { const { x, y } = canvasPos(e); mouseMove(x, y) }
function onUp() { mouseUp() }

function render() {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  ctx.clearRect(0, 0, WIDTH, HEIGHT)
  drawCollisionAreas(ctx, WIDTH, HEIGHT, bounds)

  if (nodeType.value === 'rigid') {
    for (const body of rigids) {
      const corners = [0, 1, 2, 3].map(i => worldCorner(body, i))
      drawRigidSquare(ctx, corners, body.pinned ? '#00274Ccc' : '#FFCB05cc')
    }
  } else {
    ctx.strokeStyle = enforceConstraints.value ? '#999' : '#ddd'
    ctx.lineWidth = 1.5
    ctx.beginPath()
    for (const c of pConstraints) {
      ctx.moveTo(particles[c.i].x, particles[c.i].y)
      ctx.lineTo(particles[c.j].x, particles[c.j].y)
    }
    ctx.stroke()
    particles.forEach((p, i) => drawParticleDot(ctx, p.x, p.y, i === dragIndex.value ? '#FFCB05' : p.color, i === dragIndex.value ? 8 : 6))
  }

  drawRaf = requestAnimationFrame(render)
}

onMounted(() => { drawRaf = requestAnimationFrame(render) })
onBeforeUnmount(() => cancelAnimationFrame(drawRaf))
</script>

<template>
  <div class="grid-panel">
    <div class="controls">
      <label class="picker">
        <span>Node type</span>
        <select v-model="nodeType">
          <option value="particle">Particle</option>
          <option value="rigid">Rigid square</option>
        </select>
      </label>
      <label class="check smooth-check">
        <input v-model="smooth" type="checkbox">
        Smooth (skip step breakdown)
      </label>
      <label class="check">
        <input v-model="lowStiffness" type="checkbox">
        Low stiffness
      </label>
      <label class="check">
        <input v-model="enforceConstraints" type="checkbox">
        Enforce constraints
      </label>
      <button v-if="!isRunning" class="btn primary" @click="play">&#9654; Play</button>
      <button v-else class="btn primary" @click="pause">&#10074;&#10074; Pause</button>
      <button class="btn" :disabled="isRunning" @click="stepOnce">Step &raquo;</button>
      <button class="btn" @click="reset">&#8634; Reset</button>
      <span class="frame-count">frame {{ tick_count }}</span>
    </div>
    <div class="vector-canvas-wrap">
      <canvas
        ref="canvasRef" :width="WIDTH" :height="HEIGHT"
        @mousedown="onDown" @mousemove="onMove" @mouseup="onUp" @mouseleave="onUp"
      />
    </div>
    <div class="legend">Click and drag a node -- it's pinned to the mouse (a stiff location constraint) until released.</div>
  </div>
</template>

<style scoped>
.grid-panel { display: flex; flex-direction: column; gap: 0.4em; height: 100%; }
.controls { display: flex; align-items: center; gap: 0.6em; flex-wrap: wrap; }
.picker { display: flex; align-items: center; gap: 0.4em; font-family: var(--font-mono, monospace); font-size: 0.68em; color: #5f6368; }
.picker select { font-family: var(--font-body, sans-serif); font-size: 1em; padding: 0.25em 0.5em; border-radius: 8px; border: 1px solid #ddd3ad; background: #fff; }
.check { font-family: var(--font-mono, monospace); font-size: 0.68em; display: flex; align-items: center; gap: 0.3em; cursor: pointer; }
.smooth-check { color: var(--blue, #00274C); font-weight: 700; }
.frame-count { font-family: var(--font-mono, monospace); font-size: 0.62em; opacity: 0.6; }
.vector-canvas-wrap { flex: 1 1 auto; min-height: 0; border: 1px solid #e3ddc9; border-radius: 10px; background: #fff; }
.vector-canvas-wrap canvas { width: 100%; height: 100%; display: block; cursor: grab; }
.legend { font-family: var(--font-mono, monospace); font-size: 0.58em; opacity: 0.7; }
</style>
