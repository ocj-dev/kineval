<script setup lang="ts">
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { useClothGridSimulation } from '../lib/cloth/useClothGridSimulation'
import { worldCorner } from '../lib/cloth/clothPhysics'
import { drawParticleDot, drawRigidSquare } from '../lib/cloth/drawUtils'

const WIDTH = 420, HEIGHT = 300
const nodeType = ref<'particle' | 'rigid'>('particle')

const { particles, pConstraints, rigids, rConstraints, tick_count, isRunning, play, pause, reset, stepOnce } =
  useClothGridSimulation(WIDTH, HEIGHT, nodeType)

const canvasRef = ref<HTMLCanvasElement | null>(null)
let drawRaf = 0

function render() {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  ctx.clearRect(0, 0, WIDTH, HEIGHT)

  if (nodeType.value === 'rigid') {
    for (const body of rigids) {
      const corners = [0, 1, 2, 3].map(i => worldCorner(body, i))
      drawRigidSquare(ctx, corners, body.pinned ? '#00274Ccc' : '#FFCB05cc')
    }
  } else {
    ctx.strokeStyle = '#999'
    ctx.lineWidth = 1.5
    ctx.beginPath()
    for (const c of pConstraints) {
      ctx.moveTo(particles[c.i].x, particles[c.i].y)
      ctx.lineTo(particles[c.j].x, particles[c.j].y)
    }
    ctx.stroke()
    for (const p of particles) drawParticleDot(ctx, p.x, p.y, p.pinned ? '#00274C' : '#FFCB05', 6)
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
      <button v-if="!isRunning" class="btn primary" @click="play">&#9654; Play</button>
      <button v-else class="btn primary" @click="pause">&#10074;&#10074; Pause</button>
      <button class="btn" :disabled="isRunning" @click="stepOnce">Step &raquo;</button>
      <button class="btn" @click="reset">&#8634; Reset</button>
      <span class="frame-count">frame {{ tick_count }}</span>
    </div>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasRef" :width="WIDTH" :height="HEIGHT" />
    </div>
  </div>
</template>

<style scoped>
.grid-panel { display: flex; flex-direction: column; gap: 0.4em; height: 100%; }
.controls { display: flex; align-items: center; gap: 0.6em; flex-wrap: wrap; }
.picker { display: flex; align-items: center; gap: 0.4em; font-family: var(--font-mono, monospace); font-size: 0.68em; color: #5f6368; }
.picker select { font-family: var(--font-body, sans-serif); font-size: 1em; padding: 0.25em 0.5em; border-radius: 8px; border: 1px solid #ddd3ad; background: #fff; }
.frame-count { font-family: var(--font-mono, monospace); font-size: 0.62em; opacity: 0.6; }
.vector-canvas-wrap { flex: 1 1 auto; min-height: 0; border: 1px solid #e3ddc9; border-radius: 10px; background: #fff; }
.vector-canvas-wrap canvas { width: 100%; height: 100%; display: block; }
</style>
