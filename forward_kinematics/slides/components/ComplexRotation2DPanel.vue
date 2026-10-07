<script setup lang="ts">
import { reactive, ref, onMounted, watch } from 'vue'
import { rotatePoint2D } from '../lib/fk/complexMath'
import { clearCanvas, INDIGO, AMBER } from '../lib/fk/drawUtils'

// 2D rotation of a point via complex-number multiplication -- the deck's
// conceptual stepping-stone to 3D quaternion rotation in
// QuaternionRotation3DPanel.vue (same drag-an-angle interaction, one
// dimension down).

const theta = reactive({ value: Math.PI / 6 })
const point = reactive({ x: 1.4, y: 0.6 })
const canvasEl = ref<HTMLCanvasElement | null>(null)

function redraw() {
  const canvas = canvasEl.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  const rect = canvas.getBoundingClientRect()
  canvas.width = rect.width; canvas.height = rect.height
  clearCanvas(ctx, canvas.width, canvas.height)

  const scale = 70
  const origin: [number, number] = [canvas.width / 2, canvas.height / 2]
  const toScreen = (x: number, y: number): [number, number] => [origin[0] + x * scale, origin[1] - y * scale]

  ctx.strokeStyle = '#00000020'
  ctx.beginPath(); ctx.moveTo(0, origin[1]); ctx.lineTo(canvas.width, origin[1])
  ctx.moveTo(origin[0], 0); ctx.lineTo(origin[0], canvas.height); ctx.stroke()

  const [rx, ry] = rotatePoint2D([point.x, point.y], theta.value)

  function drawVec(x: number, y: number, color: string) {
    const [sx, sy] = toScreen(x, y)
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2.5
    ctx.beginPath(); ctx.moveTo(origin[0], origin[1]); ctx.lineTo(sx, sy); ctx.stroke()
    ctx.beginPath(); ctx.arc(sx, sy, 5, 0, Math.PI * 2); ctx.fill()
  }
  drawVec(point.x, point.y, '#9aa0a6')
  drawVec(rx, ry, AMBER)

  // angle arc between the two vectors
  const r0 = Math.atan2(-point.y, point.x) // screen-space angle (y flipped)
  ctx.strokeStyle = INDIGO
  ctx.beginPath()
  ctx.arc(origin[0], origin[1], 28, r0, r0 - theta.value, theta.value < 0)
  ctx.stroke()

  ctx.font = '12px "Roboto Mono", monospace'
  ctx.fillStyle = '#202124'
  ctx.textAlign = 'left'
  ctx.fillText(`p = (${point.x.toFixed(2)}, ${point.y.toFixed(2)})`, 10, 18)
  ctx.fillStyle = AMBER
  ctx.fillText(`p' = (${rx.toFixed(2)}, ${ry.toFixed(2)})  =  p · (cos θ + i sin θ)`, 10, 36)
}

watch([() => theta.value, () => point.x, () => point.y], redraw)
onMounted(redraw)

let dragging = false
function onPointerDown() { dragging = true }
function onPointerUp() { dragging = false }
function onPointerMove(e: PointerEvent) {
  if (!dragging || !canvasEl.value) return
  const rect = canvasEl.value.getBoundingClientRect()
  const x = e.clientX - rect.left - rect.width / 2
  const y = -(e.clientY - rect.top - rect.height / 2)
  theta.value = Math.atan2(y, x) - Math.atan2(point.y, point.x)
}
</script>

<template>
  <div class="complex-panel">
    <div class="controls">
      <label>θ <input v-model.number="theta.value" type="range" min="-3.14159" max="3.14159" step="0.01"> {{ theta.value.toFixed(2) }} rad</label>
    </div>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasEl" @pointerdown="onPointerDown" @pointerup="onPointerUp" @pointermove="onPointerMove" @pointerleave="onPointerUp" />
    </div>
  </div>
</template>

<style scoped>
.complex-panel { display: flex; flex-direction: column; height: 100%; gap: 0.5em; }
.controls { display: flex; gap: 1em; font-size: 0.8em; font-family: var(--font-mono, monospace); }
.controls input[type="range"] { width: 12em; }
</style>
