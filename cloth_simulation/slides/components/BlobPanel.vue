<script setup lang="ts">
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { useBlobSimulation } from '../lib/cloth/useBlobSimulation'
import { drawParticleDot } from '../lib/cloth/drawUtils'

const WIDTH = 420, HEIGHT = 300

const { particles, constraints, groundY, tick_count, isRunning, play, pause, reset, stepOnce } =
  useBlobSimulation(WIDTH, HEIGHT)

const canvasRef = ref<HTMLCanvasElement | null>(null)
let drawRaf = 0

function render() {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  ctx.clearRect(0, 0, WIDTH, HEIGHT)

  ctx.strokeStyle = '#00274C'
  ctx.lineWidth = 2
  ctx.beginPath()
  ctx.moveTo(0, groundY)
  ctx.lineTo(WIDTH, groundY)
  ctx.stroke()

  ctx.strokeStyle = '#999'
  ctx.lineWidth = 1.5
  ctx.beginPath()
  for (const c of constraints) {
    ctx.moveTo(particles[c.i].x, particles[c.i].y)
    ctx.lineTo(particles[c.j].x, particles[c.j].y)
  }
  ctx.stroke()

  particles.forEach((p, i) => drawParticleDot(ctx, p.x, p.y, i === 0 ? '#FFCB05' : '#00274C', 7))

  drawRaf = requestAnimationFrame(render)
}

onMounted(() => { drawRaf = requestAnimationFrame(render) })
onBeforeUnmount(() => cancelAnimationFrame(drawRaf))
</script>

<template>
  <div class="blob-panel">
    <div class="controls">
      <button v-if="!isRunning" class="btn primary" @click="play">&#9654; Play</button>
      <button v-else class="btn primary" @click="pause">&#10074;&#10074; Pause</button>
      <button class="btn" :disabled="isRunning" @click="stepOnce">Step &raquo;</button>
      <button class="btn" @click="reset">&#8634; Reset</button>
      <span class="frame-count">frame {{ tick_count }} &middot; 10 constraints, full connectivity</span>
    </div>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasRef" :width="WIDTH" :height="HEIGHT" />
    </div>
  </div>
</template>

<style scoped>
.blob-panel { display: flex; flex-direction: column; gap: 0.4em; height: 100%; }
.controls { display: flex; align-items: center; gap: 0.5em; flex-wrap: wrap; }
.frame-count { font-family: var(--font-mono, monospace); font-size: 0.62em; opacity: 0.6; }
.vector-canvas-wrap { flex: 1 1 auto; min-height: 0; border: 1px solid #e3ddc9; border-radius: 10px; background: #fff; }
.vector-canvas-wrap canvas { width: 100%; height: 100%; display: block; }
</style>
