<script setup lang="ts">
import { reactive, ref, onMounted, watch } from 'vue'
import { quaternionFromAxisAngle, quaternionNormalize, quaternionRotatePoint } from '../lib/fk/quaternionMath'
import { vectorNormalize } from '../lib/fk/matrixMath'
import { clearCanvas, project, INDIGO, AMBER } from '../lib/fk/drawUtils'

// 3D rotation of a point via a unit quaternion built from an axis and
// angle -- the 3D counterpart to ComplexRotation2DPanel.vue's 2D complex-
// number rotation, and the exact two-step pipeline (axis-angle ->
// quaternion -> apply) traverseFKJoint uses for each joint's own motion.

const axis = reactive({ x: 0.3, y: 1, z: 0.2 })
const angle = reactive({ value: Math.PI / 3 })
const point = reactive({ x: 1.1, y: 0.4, z: 0.6 })
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
  const origin: [number, number] = [canvas.width / 2, canvas.height * 0.6]
  const toScreen = (x: number, y: number, z: number): [number, number] => {
    const [px, py] = project(x, y, z, scale)
    return [origin[0] + px, origin[1] + py]
  }

  const unitAxis = vectorNormalize([axis.x, axis.y, axis.z])
  const q = quaternionNormalize(quaternionFromAxisAngle(unitAxis, angle.value))
  const rotated = quaternionRotatePoint(q, [point.x, point.y, point.z])

  // rotation axis, drawn through the origin both directions
  ctx.strokeStyle = '#00000030'
  ctx.setLineDash([4, 5])
  const [ax1, ay1] = toScreen(unitAxis[0] * 1.6, unitAxis[1] * 1.6, unitAxis[2] * 1.6)
  const [ax2, ay2] = toScreen(-unitAxis[0] * 1.6, -unitAxis[1] * 1.6, -unitAxis[2] * 1.6)
  ctx.beginPath(); ctx.moveTo(ax1, ay1); ctx.lineTo(ax2, ay2); ctx.stroke()
  ctx.setLineDash([])

  function drawVec(x: number, y: number, z: number, color: string) {
    const [sx, sy] = toScreen(x, y, z)
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2.5
    ctx.beginPath(); ctx.moveTo(origin[0], origin[1]); ctx.lineTo(sx, sy); ctx.stroke()
    ctx.beginPath(); ctx.arc(sx, sy, 5, 0, Math.PI * 2); ctx.fill()
  }
  drawVec(point.x, point.y, point.z, '#9aa0a6')
  drawVec(rotated[0], rotated[1], rotated[2], AMBER)

  ctx.font = '12px "Roboto Mono", monospace'
  ctx.fillStyle = '#202124'
  ctx.textAlign = 'left'
  ctx.fillText(`v = (${point.x.toFixed(2)}, ${point.y.toFixed(2)}, ${point.z.toFixed(2)})`, 10, 18)
  ctx.fillStyle = AMBER
  ctx.fillText(`v' = (${rotated[0].toFixed(2)}, ${rotated[1].toFixed(2)}, ${rotated[2].toFixed(2)})  =  q v q⁻¹`, 10, 36)
  ctx.fillStyle = INDIGO
  ctx.fillText(`q = (${q.a.toFixed(2)}, ${q.b.toFixed(2)}, ${q.c.toFixed(2)}, ${q.d.toFixed(2)})`, 10, 54)
}

watch([() => axis.x, () => axis.y, () => axis.z, () => angle.value, () => point.x, () => point.y, () => point.z], redraw)
onMounted(redraw)
</script>

<template>
  <div class="quat-panel">
    <div class="controls">
      <label>axis x <input v-model.number="axis.x" type="range" min="-1" max="1" step="0.01"></label>
      <label>axis y <input v-model.number="axis.y" type="range" min="-1" max="1" step="0.01"></label>
      <label>axis z <input v-model.number="axis.z" type="range" min="-1" max="1" step="0.01"></label>
      <label>θ <input v-model.number="angle.value" type="range" min="-3.14159" max="3.14159" step="0.01"> {{ angle.value.toFixed(2) }} rad</label>
    </div>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasEl" />
    </div>
  </div>
</template>

<style scoped>
.quat-panel { display: flex; flex-direction: column; height: 100%; gap: 0.5em; }
.controls { display: flex; flex-wrap: wrap; gap: 1em; font-size: 0.72em; font-family: var(--font-mono, monospace); }
.controls input[type="range"] { width: 7em; }
</style>
