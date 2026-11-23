<script setup lang="ts">
// Workspace versus configuration space, for a planar 2R arm.
//
// Left: the arm in its workspace, with a circular obstacle. Right: the
// same situation in C-space, which for two continuous revolute joints is
// the 2-torus T2 = S1 x S1 -- drawn as the square [0, 2pi)^2 with its
// opposite edges identified, because that is the only way to put a torus
// on a slide.
//
// The C-space obstacle is computed by sampling: for each (t1, t2), does
// either link intersect the disc? That is a brute-force version of
// exactly what collision.js#poseIsCollision answers one configuration at
// a time, and it makes the point that a C-space obstacle is not a
// transformed copy of the workspace obstacle -- it is a different shape
// entirely, and in general nobody computes it.
import { ref, onMounted, onBeforeUnmount, watch } from 'vue'

const t1 = ref(0.6)
const t2 = ref(1.1)
const obstacle = ref({ x: 1.15, y: 0.55, r: 0.33 })
const cw = ref<HTMLCanvasElement | null>(null)
const cc = ref<HTMLCanvasElement | null>(null)
const inCollision = ref(false)
let observer: ResizeObserver | undefined
let cspaceCache: ImageData | null = null
let cacheKey = ''

const L1 = 0.95, L2 = 0.8
const TAU = Math.PI * 2

function joints(a: number, b: number) {
  const p1 = { x: L1 * Math.cos(a), y: L1 * Math.sin(a) }
  const p2 = { x: p1.x + L2 * Math.cos(a + b), y: p1.y + L2 * Math.sin(a + b) }
  return { p0: { x: 0, y: 0 }, p1, p2 }
}

function segmentHitsDisc(ax: number, ay: number, bx: number, by: number, cx: number, cy: number, r: number) {
  const dx = bx - ax, dy = by - ay
  const len2 = dx * dx + dy * dy
  let t = len2 > 0 ? ((cx - ax) * dx + (cy - ay) * dy) / len2 : 0
  t = Math.max(0, Math.min(1, t))
  return Math.hypot(ax + t * dx - cx, ay + t * dy - cy) <= r
}

function configCollides(a: number, b: number) {
  const j = joints(a, b)
  const o = obstacle.value
  return segmentHitsDisc(j.p0.x, j.p0.y, j.p1.x, j.p1.y, o.x, o.y, o.r)
      || segmentHitsDisc(j.p1.x, j.p1.y, j.p2.x, j.p2.y, o.x, o.y, o.r)
}

function ctxOf(c: HTMLCanvasElement | null) {
  if (!c) return null
  const rect = c.getBoundingClientRect()
  if (rect.width < 2 || rect.height < 2) return null
  const dpr = window.devicePixelRatio || 1
  c.width = Math.round(rect.width * dpr)
  c.height = Math.round(rect.height * dpr)
  const ctx = c.getContext('2d')
  if (!ctx) return null
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx.fillStyle = '#fff'
  ctx.fillRect(0, 0, rect.width, rect.height)
  return { ctx, w: rect.width, h: rect.height }
}

function drawWorkspace() {
  const g = ctxOf(cw.value)
  if (!g) return
  const { ctx, w, h } = g
  const s = Math.min(w, h) / 4.2
  const X = (x: number) => w / 2 + x * s
  const Y = (y: number) => h / 2 - y * s

  // reachable annulus, for context
  ctx.strokeStyle = '#eceff3'
  ctx.lineWidth = 1
  ctx.beginPath(); ctx.arc(X(0), Y(0), (L1 + L2) * s, 0, TAU); ctx.stroke()
  ctx.beginPath(); ctx.arc(X(0), Y(0), Math.abs(L1 - L2) * s, 0, TAU); ctx.stroke()

  const o = obstacle.value
  ctx.beginPath(); ctx.arc(X(o.x), Y(o.y), o.r * s, 0, TAU)
  ctx.fillStyle = '#4a7ebb'; ctx.fill()

  const j = joints(t1.value, t2.value)
  const hit = configCollides(t1.value, t2.value)
  inCollision.value = hit
  ctx.strokeStyle = hit ? '#c5221f' : '#3c4043'
  ctx.lineWidth = 7
  ctx.lineCap = 'round'
  ctx.beginPath()
  ctx.moveTo(X(j.p0.x), Y(j.p0.y)); ctx.lineTo(X(j.p1.x), Y(j.p1.y)); ctx.lineTo(X(j.p2.x), Y(j.p2.y))
  ctx.stroke()
  for (const p of [j.p0, j.p1, j.p2]) {
    ctx.beginPath(); ctx.arc(X(p.x), Y(p.y), 4.5, 0, TAU)
    ctx.fillStyle = '#202B4E'; ctx.fill()
  }
}

function drawCspace() {
  const g = ctxOf(cc.value)
  if (!g) return
  const { ctx, w, h } = g
  const side = Math.min(w, h)
  const ox = (w - side) / 2, oy = (h - side) / 2

  const key = `${obstacle.value.x},${obstacle.value.y},${obstacle.value.r},${Math.round(side)}`
  if (key !== cacheKey) {
    const N = 150
    const cell = side / N
    ctx.fillStyle = '#eceff3'
    ctx.fillRect(ox, oy, side, side)
    ctx.fillStyle = '#c5221f'
    for (let i = 0; i < N; i++) {
      for (let j = 0; j < N; j++) {
        const a = (i / N) * TAU
        const b = (j / N) * TAU
        if (configCollides(a, b)) ctx.fillRect(ox + i * cell, oy + side - (j + 1) * cell, cell + 0.6, cell + 0.6)
      }
    }
    cspaceCache = ctx.getImageData(0, 0, Math.round(w * (window.devicePixelRatio || 1)), Math.round(h * (window.devicePixelRatio || 1)))
    cacheKey = key
  } else if (cspaceCache) {
    ctx.putImageData(cspaceCache, 0, 0)
  }

  ctx.strokeStyle = '#9aa0a6'
  ctx.lineWidth = 1
  ctx.strokeRect(ox, oy, side, side)

  // the current configuration
  const px = ox + (((t1.value % TAU) + TAU) % TAU) / TAU * side
  const py = oy + side - (((t2.value % TAU) + TAU) % TAU) / TAU * side
  ctx.beginPath(); ctx.arc(px, py, 5, 0, TAU)
  ctx.fillStyle = '#E8710A'; ctx.fill()
  ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke()

  ctx.fillStyle = '#5f6368'
  ctx.font = '600 10px Roboto Mono, monospace'
  ctx.fillText('θ₁ →', ox + side - 34, oy + side + 12)
  ctx.save(); ctx.translate(ox - 6, oy + 30); ctx.rotate(-Math.PI / 2)
  ctx.fillText('θ₂ →', 0, 0); ctx.restore()
}

function redraw() { drawWorkspace(); drawCspace() }
watch([t1, t2], () => { drawWorkspace(); drawCspace() })
onMounted(() => {
  observer = new ResizeObserver(() => { cacheKey = ''; redraw() })
  if (cw.value) observer.observe(cw.value)
  if (cc.value) observer.observe(cc.value)
  redraw()
})
onBeforeUnmount(() => observer?.disconnect())
</script>

<template>
  <div class="cs-panel">
    <div class="cs-grid">
      <div class="cs-cell">
        <div class="cs-title">Workspace &mdash; <span class="cs-sub">where the arm is</span></div>
        <div class="vector-canvas-wrap"><canvas ref="cw"></canvas></div>
      </div>
      <div class="cs-cell">
        <div class="cs-title">C-space T² = S¹×S¹ &mdash; <span class="cs-sub">opposite edges identified</span></div>
        <div class="vector-canvas-wrap"><canvas ref="cc"></canvas></div>
      </div>
    </div>
    <div class="cs-controls">
      <span class="cs-label">θ₁</span>
      <input type="range" min="0" max="6.283" step="0.01" v-model.number="t1" />
      <span class="cs-label">θ₂</span>
      <input type="range" min="0" max="6.283" step="0.01" v-model.number="t2" />
      <span class="cs-stat" :class="{ hit: inCollision }">{{ inCollision ? 'in collision' : 'free' }}</span>
      <span class="cs-stat">one disc in the workspace &rarr; a C-space obstacle that wraps around the torus</span>
    </div>
  </div>
</template>

<style scoped>
.cs-panel { display: flex; flex-direction: column; height: 100%; min-height: 0; gap: 0.4em; }
.cs-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 0.7em; flex: 1 1 auto; min-height: 0; }
.cs-cell { display: flex; flex-direction: column; min-height: 0; gap: 0.25em; }
.cs-title { font-size: 0.62em; font-weight: 700; font-family: var(--font-mono, monospace); color: var(--indigo, #202B4E); }
.cs-sub { font-weight: 400; opacity: 0.75; }
.vector-canvas-wrap {
  flex: 1 1 auto; height: 100%; min-height: 0;
  border: 1px solid #dcdfe6; border-radius: 10px; overflow: hidden; background: #fff;
}
.vector-canvas-wrap canvas { display: block; width: 100%; height: 100%; }
.cs-controls { display: flex; align-items: center; gap: 0.4em; flex-wrap: wrap; }
.cs-label { font-family: var(--font-mono, monospace); font-size: 0.68em; color: #5f6368; }
.cs-controls input[type=range] { width: 7.5em; }
.cs-stat {
  font-family: var(--font-mono, monospace); font-size: 0.58em; color: #5f6368;
  background: #eef0f4; border-radius: 999px; padding: 0.15em 0.6em;
}
.cs-stat.hit { background: #fadcd9; color: #c5221f; font-weight: 700; }
</style>
