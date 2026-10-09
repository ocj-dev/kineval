<script setup lang="ts">
import { reactive, ref, onMounted, onBeforeUnmount, watch } from 'vue'
import { quaternionFromAxisAngle, quaternionNormalize, quaternionRotatePoint } from '../lib/fk/quaternionMath'
import { vectorNormalize } from '../lib/fk/matrixMath'
import { clearCanvas, INDIGO, AMBER } from '../lib/fk/drawUtils'

// 3D rotation of a point via a unit quaternion built from an axis and
// angle -- the 3D counterpart to ComplexRotation2DPanel.vue's 2D complex-
// number rotation, and the exact two-step pipeline (axis-angle ->
// quaternion -> apply) traverseFKJoint uses for each joint's own motion.

const axis = reactive({ x: 0.15, y: 1, z: 0.3 })
const angle = reactive({ value: Math.PI / 3 })
const point = reactive({ x: 1.2, y: 0.45, z: 0.2 })
const view = reactive({ yaw: 0.6 })
const canvasEl = ref<HTMLCanvasElement | null>(null)

// An orbitable orthographic camera, rather than drawUtils' fixed oblique
// projection. That projection is linear R3->R2, so it has a 1-D kernel, and
// any circle whose plane contains that kernel collapses to a straight
// sliver -- which is exactly what the sweep ring below did from the old
// fixed viewpoint (it sat within ~2 degrees of edge-on). A yaw the viewer
// controls means no single orientation can hide the geometry.
const PITCH = 0.42
function makeProject(scale: number) {
  const cy = Math.cos(view.yaw), sy = Math.sin(view.yaw)
  const cp = Math.cos(PITCH), sp = Math.sin(PITCH)
  return (x: number, y: number, z: number): [number, number] => {
    const X = x * cy + z * sy
    const Zc = -x * sy + z * cy
    const Y = y * cp - Zc * sp
    return [X * scale, -Y * scale]
  }
}

function redraw() {
  const canvas = canvasEl.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  const rect = canvas.getBoundingClientRect()
  // Nothing can be drawn into a zero-sized backing store -- bail rather than
  // committing a 0x0 canvas, which the browser then stretches over the
  // element's full CSS size as an uninitialized (garbage-coloured) block.
  // The ResizeObserver below calls back once real layout exists.
  if (rect.width === 0 || rect.height === 0) return
  canvas.width = rect.width; canvas.height = rect.height
  clearCanvas(ctx, canvas.width, canvas.height)

  // fill the space available rather than assuming a fixed panel height
  const scale = Math.min(canvas.width * 0.17, canvas.height * 0.34)
  const project = makeProject(scale)
  const origin: [number, number] = [canvas.width * 0.56, canvas.height * 0.55]
  const toScreen = (x: number, y: number, z: number): [number, number] => {
    const [px, py] = project(x, y, z)
    return [origin[0] + px, origin[1] + py]
  }

  const unitAxis = vectorNormalize([axis.x, axis.y, axis.z])
  const v: [number, number, number] = [point.x, point.y, point.z]
  const q = quaternionNormalize(quaternionFromAxisAngle(unitAxis, angle.value))
  const rotated = quaternionRotatePoint(q, v)

  // The sweep: where this point lands for EVERY angle, not just the one on
  // the slider. Sampled by actually rotating v through a full turn with the
  // same quaternion routine that drives the live vector -- so the ring is
  // the function's own output, not a separately-derived circle. Geometrically
  // it's the circle of latitude v traces about the axis: centred on v's
  // projection onto the axis, with the perpendicular remainder as radius.
  const SWEEP_STEPS = 160
  ctx.strokeStyle = AMBER + '70'
  ctx.lineWidth = 1.5
  ctx.beginPath()
  for (let i = 0; i <= SWEEP_STEPS; i++) {
    const t = (i / SWEEP_STEPS) * Math.PI * 2
    const qt = quaternionNormalize(quaternionFromAxisAngle(unitAxis, t))
    const p = quaternionRotatePoint(qt, v)
    const [sx, sy] = toScreen(p[0], p[1], p[2])
    if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy)
  }
  ctx.closePath(); ctx.stroke()

  // rotation axis, through the origin both ways
  ctx.strokeStyle = 'rgba(32, 43, 78, 0.6)'
  ctx.lineWidth = 1.5
  ctx.setLineDash([5, 5])
  const [ax1, ay1] = toScreen(unitAxis[0] * 1.7, unitAxis[1] * 1.7, unitAxis[2] * 1.7)
  const [ax2, ay2] = toScreen(-unitAxis[0] * 1.7, -unitAxis[1] * 1.7, -unitAxis[2] * 1.7)
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
  ctx.textAlign = 'left'
  ctx.fillStyle = '#202124'
  ctx.fillText(`v  = (${point.x.toFixed(2)}, ${point.y.toFixed(2)}, ${point.z.toFixed(2)})`, 10, 18)
  ctx.fillStyle = AMBER
  ctx.fillText(`v' = (${rotated[0].toFixed(2)}, ${rotated[1].toFixed(2)}, ${rotated[2].toFixed(2)})  =  q v q⁻¹`, 10, 36)
  ctx.fillStyle = INDIGO
  ctx.fillText(`q  = (${q.a.toFixed(2)}, ${q.b.toFixed(2)}, ${q.c.toFixed(2)}, ${q.d.toFixed(2)})`, 10, 54)
  ctx.fillStyle = AMBER + 'bb'
  ctx.fillText('ring: where v lands for every θ', 10, 72)
}

watch([
  () => axis.x, () => axis.y, () => axis.z, () => angle.value,
  () => point.x, () => point.y, () => point.z, () => view.yaw,
], redraw)

// Drives both the first paint and every resize: onMounted alone fires before
// the browser has laid this element out, so the canvas would otherwise sit
// at 0x0 until some unrelated reactive change happened to repaint it.
let resizeObserver: ResizeObserver | null = null
onMounted(() => {
  if (!canvasEl.value) return
  resizeObserver = new ResizeObserver(() => redraw())
  resizeObserver.observe(canvasEl.value)
  redraw()
})
onBeforeUnmount(() => resizeObserver?.disconnect())
</script>

<template>
  <div class="quat-panel">
    <div class="controls">
      <span class="group-label">rotation axis</span>
      <label>x <input v-model.number="axis.x" type="range" min="-1" max="1" step="0.01"></label>
      <label>y <input v-model.number="axis.y" type="range" min="-1" max="1" step="0.01"></label>
      <label>z <input v-model.number="axis.z" type="range" min="-1" max="1" step="0.01"></label>
      <label>&theta; <input v-model.number="angle.value" type="range" min="-3.14159" max="3.14159" step="0.01"> {{ angle.value.toFixed(2) }} rad</label>
    </div>
    <div class="controls">
      <span class="group-label">vector <b>v</b></span>
      <label>x <input v-model.number="point.x" type="range" min="-1.5" max="1.5" step="0.01"></label>
      <label>y <input v-model.number="point.y" type="range" min="-1.5" max="1.5" step="0.01"></label>
      <label>z <input v-model.number="point.z" type="range" min="-1.5" max="1.5" step="0.01"></label>
      <label class="view-ctl">orbit view <input v-model.number="view.yaw" type="range" min="-3.14159" max="3.14159" step="0.01"></label>
    </div>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasEl" />
    </div>
  </div>
</template>

<style scoped>
.quat-panel { display: flex; flex-direction: column; height: 100%; gap: 0.3em; }
.controls { display: flex; flex-wrap: wrap; gap: 0.85em; align-items: center; font-size: 0.72em; font-family: var(--font-mono, monospace); }
.controls input[type="range"] { width: 6.2em; }
.group-label { font-weight: 700; color: var(--indigo, #202B4E); min-width: 7.5em; }
.view-ctl { color: var(--amber-dark, #b85a08); font-weight: 700; }
</style>
