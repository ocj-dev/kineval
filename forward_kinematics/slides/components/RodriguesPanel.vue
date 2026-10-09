<script setup lang="ts">
import { reactive, ref, onMounted, onBeforeUnmount, watch } from 'vue'
import { vectorNormalize, vectorCross } from '../lib/fk/matrixMath'
import { clearCanvas, INDIGO, AMBER } from '../lib/fk/drawUtils'

// Breaks the Rodrigues rotation formula into the three terms the slide
// writes out, and draws each one as its own vector:
//
//   b' = (1 - cos t)(a.b) a  +  b cos t  +  (a x b) sin t
//        \__ along the axis __/  \_ shrunk _/  \_ perpendicular _/
//
// The point of the picture is that the axis-parallel part of b never moves:
// only the part of b perpendicular to a gets spun, and (a x b) supplies the
// second perpendicular direction needed to sweep it round.

const axisIn = reactive({ x: 0.1, y: 1, z: 0.25 })
const bIn = reactive({ x: 1.15, y: 0.35, z: 0.1 })
const theta = reactive({ value: Math.PI / 3 })
const view = reactive({ yaw: 0.6 })
const show = reactive({ parts: true })
const canvasEl = ref<HTMLCanvasElement | null>(null)

const PITCH = 0.4
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
  if (rect.width === 0 || rect.height === 0) return
  canvas.width = rect.width; canvas.height = rect.height
  clearCanvas(ctx, canvas.width, canvas.height)

  const scale = Math.min(canvas.width * 0.20, canvas.height * 0.30)
  const project = makeProject(scale)
  const ox = canvas.width * 0.52, oy = canvas.height * 0.58
  const S = (v: number[]): [number, number] => {
    const [px, py] = project(v[0], v[1], v[2])
    return [ox + px, oy + py]
  }

  const a = vectorNormalize([axisIn.x, axisIn.y, axisIn.z])
  const b = [bIn.x, bIn.y, bIn.z]
  const t = theta.value
  const adotb = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
  const axb = vectorCross(a, b)

  // the three Rodrigues terms
  const term1 = a.map((c) => (1 - Math.cos(t)) * adotb * c)       // (1-cos t)(a.b) a
  const term2 = b.map((c) => c * Math.cos(t))                      // b cos t
  const term3 = axb.map((c) => c * Math.sin(t))                    // (a x b) sin t
  const bPrime = [0, 1, 2].map((i) => term1[i] + term2[i] + term3[i])

  // the axis-parallel component of b -- the part the rotation cannot touch
  const bPar = a.map((c) => adotb * c)

  function arrow(from: number[], to: number[], color: string, width: number, dash: number[] = []) {
    const [x1, y1] = S(from), [x2, y2] = S(to)
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width
    if (dash.length) ctx.setLineDash(dash)
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke()
    ctx.setLineDash([])
    const ang = Math.atan2(y2 - y1, x2 - x1), h = 7
    ctx.beginPath()
    ctx.moveTo(x2, y2)
    ctx.lineTo(x2 - h * Math.cos(ang - Math.PI / 6), y2 - h * Math.sin(ang - Math.PI / 6))
    ctx.lineTo(x2 - h * Math.cos(ang + Math.PI / 6), y2 - h * Math.sin(ang + Math.PI / 6))
    ctx.closePath(); ctx.fill()
  }

  // the circle b' sweeps as theta runs over a full turn
  ctx.strokeStyle = AMBER + '55'; ctx.lineWidth = 1.3
  ctx.beginPath()
  for (let i = 0; i <= 140; i++) {
    const s = (i / 140) * Math.PI * 2
    const p = [0, 1, 2].map((k) =>
      (1 - Math.cos(s)) * adotb * a[k] + b[k] * Math.cos(s) + axb[k] * Math.sin(s))
    const [sx, sy] = S(p)
    if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy)
  }
  ctx.closePath(); ctx.stroke()

  // rotation axis a
  ctx.strokeStyle = 'rgba(32,43,78,0.6)'; ctx.lineWidth = 1.5
  ctx.setLineDash([5, 5])
  const A1 = S(a.map((c) => c * 1.7)), A2 = S(a.map((c) => -c * 1.7))
  ctx.beginPath(); ctx.moveTo(A1[0], A1[1]); ctx.lineTo(A2[0], A2[1]); ctx.stroke()
  ctx.setLineDash([])
  arrow([0, 0, 0], a, INDIGO, 2)

  if (show.parts) {
    // the component of b along a -- fixed by the rotation
    arrow([0, 0, 0], bPar, '#27966b', 2, [4, 3])
    // (a x b) sin t, the second perpendicular direction
    arrow([0, 0, 0], term3, '#9b59b6', 2, [4, 3])
    // b cos t
    arrow([0, 0, 0], term2, '#8a8f98', 1.8, [3, 3])
    // show the terms summing head-to-tail into b'
    ctx.strokeStyle = '#00000035'; ctx.lineWidth = 1; ctx.setLineDash([2, 3])
    const s1 = S(term2), s2 = S([term2[0] + term3[0], term2[1] + term3[1], term2[2] + term3[2]]), s3 = S(bPrime)
    ctx.beginPath(); ctx.moveTo(s1[0], s1[1]); ctx.lineTo(s2[0], s2[1]); ctx.lineTo(s3[0], s3[1]); ctx.stroke()
    ctx.setLineDash([])
  }

  arrow([0, 0, 0], b, '#5f6368', 2.5)
  arrow([0, 0, 0], bPrime, AMBER, 3)

  // legend
  const L = [
    ['a  (unit axis)', INDIGO],
    ['b  (input)', '#5f6368'],
    ["b' (rotated)", AMBER],
  ]
  const P = [
    ['(a·b)a  fixed part', '#27966b'],
    ['(a×b) sin θ', '#9b59b6'],
    ['b cos θ', '#8a8f98'],
  ]
  ctx.font = '11px "Roboto Mono", monospace'; ctx.textAlign = 'left'
  let y = 16
  for (const [label, col] of L) { ctx.fillStyle = col; ctx.fillText(label, 8, y); y += 14 }
  if (show.parts) for (const [label, col] of P) { ctx.fillStyle = col; ctx.fillText(label, 8, y); y += 14 }
}

watch([
  () => axisIn.x, () => axisIn.y, () => axisIn.z,
  () => bIn.x, () => bIn.y, () => bIn.z,
  () => theta.value, () => view.yaw, () => show.parts,
], redraw)

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
  <div class="rod-panel">
    <div class="controls">
      <span class="group-label">axis <b>a</b></span>
      <label>x <input v-model.number="axisIn.x" type="range" min="-1" max="1" step="0.01"></label>
      <label>y <input v-model.number="axisIn.y" type="range" min="-1" max="1" step="0.01"></label>
      <label>z <input v-model.number="axisIn.z" type="range" min="-1" max="1" step="0.01"></label>
    </div>
    <div class="controls">
      <span class="group-label">vector <b>b</b></span>
      <label>x <input v-model.number="bIn.x" type="range" min="-1.5" max="1.5" step="0.01"></label>
      <label>y <input v-model.number="bIn.y" type="range" min="-1.5" max="1.5" step="0.01"></label>
      <label>z <input v-model.number="bIn.z" type="range" min="-1.5" max="1.5" step="0.01"></label>
    </div>
    <div class="controls">
      <label>&theta; <input v-model.number="theta.value" type="range" min="-3.14159" max="3.14159" step="0.01"> {{ theta.value.toFixed(2) }}</label>
      <label class="view-ctl">orbit <input v-model.number="view.yaw" type="range" min="-3.14159" max="3.14159" step="0.01"></label>
      <label class="chk"><input v-model="show.parts" type="checkbox"> show terms</label>
    </div>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasEl" />
    </div>
  </div>
</template>

<style scoped>
.rod-panel { display: flex; flex-direction: column; height: 100%; gap: 0.25em; min-height: 0; }
.controls { display: flex; flex-wrap: wrap; align-items: center; gap: 0.6em; font-size: 0.66em; font-family: var(--font-mono, monospace); }
.controls input[type="range"] { width: 4.6em; }
.group-label { font-weight: 700; color: var(--indigo, #202B4E); min-width: 5.5em; }
.view-ctl { color: var(--amber-dark, #b85a08); font-weight: 700; }
.chk { display: flex; align-items: center; gap: 0.3em; font-weight: 700; }
</style>
