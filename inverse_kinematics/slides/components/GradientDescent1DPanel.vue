<script setup lang="ts">
import { reactive, ref, watch, onMounted, onBeforeUnmount } from 'vue'
import { clearCanvas, INDIGO, AMBER } from '../lib/ik/drawUtils'
import { useStepTracer } from '../lib/ik/useStepTracer'

// Gradient descent on an asymmetric double-well -- exactly ONE global
// minimum, ONE local minimum, and the local maximum separating them:
//   f(x) = (x^2 - 1)^2 + c*x,  c = 0.5, on -2 <= x <= 2
// Derivative and second derivative:
//   f'(x)  = 4x^3 - 4x + c
//   f''(x) = 12x^2 - 4
// c = 0.5 was chosen after root-finding f'(x) = 0 over [-2, 2] and
// classifying each root via f''(x): EXACTLY three critical points, in
// order min / max / min --
//   x ~= -1.057  (f ~= -0.5148)  GLOBAL minimum, f'' = 9.40 > 0
//   x ~=  0.127  (f ~=  1.0315)  local maximum,  f'' = -3.81 < 0
//   x ~=  0.930  (f ~=  0.4833)  local minimum,  f'' = 6.38 > 0
// -- two clearly-distinct minima (f-values ~1.0 apart) with both endpoints
// (f(-2)=8, f(2)=10) well above either, confirming -1.057 is the global
// minimum over the whole domain, not just locally. f'(x) also verified
// against a central finite-difference check at several sample points in
// Node before shipping this panel (agreement to ~1e-10) -- same "verify
// every quantitative claim before publishing" discipline this project's
// Whitney-citation slide names.
//
// The point of this panel: which well gradient descent settles into
// depends deterministically on which side of the local max (x ~= 0.127)
// the start x sits on -- drag "start x" across the domain and watch it
// always fall into the SAME well from the SAME side, a cleaner
// illustration of start-point-dependence than an oscillating function
// gives. This deck's Jacobian-based IK (ahead) inherits the same idea:
// multiple solutions / local minima in configuration space, not just in
// this toy 1D example.

const C = 0.5
function f(x: number): number {
  return (x * x - 1) * (x * x - 1) + C * x
}
function fprime(x: number): number {
  return 4 * x * x * x - 4 * x + C
}

const DOMAIN_MIN = -2
const DOMAIN_MAX = 2

interface GdStep { x: number; fx: number; slope: number }

const params = reactive({ startX: 1.5, gamma: 0.01 })
const speedMs = ref(220)
const tracer = useStepTracer<GdStep>(speedMs)

function buildTrace(): GdStep[] {
  const steps: GdStep[] = []
  let x = params.startX
  for (let i = 0; i < 150; i++) {
    const fx = f(x)
    const slope = fprime(x)
    steps.push({ x, fx, slope })
    if (Math.abs(slope) < 1e-3) break
    const next = x - params.gamma * slope
    if (!isFinite(next) || next < DOMAIN_MIN || next > DOMAIN_MAX) break
    if (Math.abs(next - x) < 1e-7) break
    x = next
  }
  return steps
}

function retrace() {
  tracer.load(buildTrace())
  redraw()
}

function doReset() {
  tracer.reset()
  redraw()
}

const canvasEl = ref<HTMLCanvasElement | null>(null)

function redraw() {
  const canvas = canvasEl.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  const rect = canvas.getBoundingClientRect()
  if (rect.width === 0 || rect.height === 0) return
  canvas.width = rect.width
  canvas.height = rect.height
  clearCanvas(ctx, canvas.width, canvas.height)

  const N = 500
  const xs: number[] = []
  const ys: number[] = []
  for (let i = 0; i <= N; i++) {
    const x = DOMAIN_MIN + (DOMAIN_MAX - DOMAIN_MIN) * (i / N)
    xs.push(x)
    ys.push(f(x))
  }
  const yMin = Math.min(...ys)
  const yMax = Math.max(...ys)
  const pad = 0.1 * (yMax - yMin || 1)
  const yLo = yMin - pad
  const yHi = yMax + pad

  const marginL = 42, marginR = 14, marginT = 14, marginB = 24
  const plotW = canvas.width - marginL - marginR
  const plotH = canvas.height - marginT - marginB
  const X = (x: number) => marginL + ((x - DOMAIN_MIN) / (DOMAIN_MAX - DOMAIN_MIN)) * plotW
  const Y = (y: number) => marginT + (1 - (y - yLo) / (yHi - yLo)) * plotH

  // zero line
  ctx.strokeStyle = '#00000022'
  ctx.lineWidth = 1
  ctx.beginPath()
  ctx.moveTo(marginL, Y(0))
  ctx.lineTo(marginL + plotW, Y(0))
  ctx.stroke()

  // curve
  ctx.strokeStyle = INDIGO
  ctx.lineWidth = 2
  ctx.beginPath()
  xs.forEach((x, i) => {
    const px = X(x), py = Y(ys[i])
    if (i === 0) ctx.moveTo(px, py)
    else ctx.lineTo(px, py)
  })
  ctx.stroke()

  // trajectory of visited points so far
  const visited = tracer.steps.value.slice(0, Math.max(0, tracer.pos.value + 1))
  ctx.fillStyle = '#9b59b699'
  for (const s of visited) {
    const px = X(s.x), py = Y(s.fx)
    ctx.beginPath()
    ctx.arc(px, py, 3, 0, Math.PI * 2)
    ctx.fill()
  }

  const cur = tracer.current.value
  if (cur) {
    const half = 0.16
    const x0 = Math.max(DOMAIN_MIN, cur.x - half)
    const x1 = Math.min(DOMAIN_MAX, cur.x + half)
    const y0 = cur.fx + cur.slope * (x0 - cur.x)
    const y1 = cur.fx + cur.slope * (x1 - cur.x)
    ctx.strokeStyle = AMBER
    ctx.lineWidth = 2
    ctx.beginPath()
    ctx.moveTo(X(x0), Y(y0))
    ctx.lineTo(X(x1), Y(y1))
    ctx.stroke()

    ctx.fillStyle = AMBER
    ctx.beginPath()
    ctx.arc(X(cur.x), Y(cur.fx), 6, 0, Math.PI * 2)
    ctx.fill()
  }
}

watch(() => tracer.pos.value, redraw)
watch(() => [params.startX, params.gamma], retrace)

let ro: ResizeObserver | null = null
onMounted(() => {
  if (!canvasEl.value) return
  retrace()
  ro = new ResizeObserver(() => redraw())
  ro.observe(canvasEl.value)
})
onBeforeUnmount(() => ro?.disconnect())

const fmt = (v: number) => (v >= 0 ? ' ' : '') + v.toFixed(4)
</script>

<template>
  <div class="gd1d-shell">
    <div class="controls">
      <label>start x <input v-model.number="params.startX" type="range" min="-2" max="2" step="0.01"></label>
      <span class="val">{{ params.startX.toFixed(2) }}</span>
      <label>step size &gamma; <input v-model.number="params.gamma" type="range" min="0.001" max="0.05" step="0.001"></label>
      <span class="val">{{ params.gamma.toFixed(3) }}</span>
      <button class="btn" :disabled="tracer.isAtStart.value" @click="tracer.stepBack">&laquo; Step</button>
      <button v-if="!tracer.isRunning.value" class="btn primary" :disabled="tracer.isDone.value" @click="tracer.play">&#9654; Play</button>
      <button v-else class="btn primary" @click="tracer.pause">&#10074;&#10074; Pause</button>
      <button class="btn" :disabled="tracer.isDone.value" @click="tracer.stepForward">Step &raquo;</button>
      <button class="btn" @click="doReset">&#8634; Reset</button>
      <span v-if="tracer.current.value" class="status-tag">
        x = {{ fmt(tracer.current.value.x) }} &middot; f(x) = {{ fmt(tracer.current.value.fx) }} &middot; f&prime;(x) = {{ fmt(tracer.current.value.slope) }}
      </span>
    </div>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasEl" />
    </div>
    <div class="hint">amber: current point &amp; tangent line (actual f&prime;) &middot; purple dots: trajectory so far &middot; step {{ Math.max(0, tracer.pos.value + 1) }} / {{ tracer.stepCount.value }}</div>
  </div>
</template>

<style scoped>
.gd1d-shell { display: flex; flex-direction: column; height: 100%; min-height: 0; gap: 0.4em; }
.controls { display: flex; flex-wrap: wrap; align-items: center; gap: 0.5em; font-size: 0.64em; font-family: var(--font-mono, monospace); }
.controls input[type="range"] { width: 5.5em; }
.val { opacity: 0.7; min-width: 2.6em; }
.status-tag {
  font-weight: 700; background: #fdecd9; border: 1px solid #202B4E; color: #202B4E;
  border-radius: 999px; padding: 0.15em 0.6em; margin-left: auto;
}
.hint { font-size: 0.6em; opacity: 0.65; font-family: var(--font-mono, monospace); }
</style>
