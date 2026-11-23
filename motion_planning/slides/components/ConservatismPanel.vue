<script setup lang="ts">
// The conservatism panel: the inflated-AABB test KinEval actually uses,
// side by side with the exact sphere-vs-box test it is an approximation
// of, with the region where the two verdicts disagree shaded.
//
// THE EXACT TEST IS IMPLEMENTED HERE AND NOWHERE ELSE. It is deliberately
// absent from motion_planning/reference/collision.js, and a unit test
// there asserts the CONSERVATIVE verdict at a box corner precisely so
// that adding it to the implementation fails loudly. The conservatism is
// teaching material, not an oversight -- the collision lecture itself
// walks a sphere to a corner, annotates "True separating axis not
// tested", and closes with "umm... take comp geom".
//
// Drag the sphere, or change its radius, and watch the corner regions.
import { ref, onMounted, onBeforeUnmount, watch } from 'vue'

const radius = ref(0.55)
// Default position chosen to land IN the disagreement region, because a
// panel that opens on "the two agree" hides the thing it exists to show.
// With the box corner at (1.2, 0.8) and r = 0.55: offset (0.42, 0.42) is
// inside the inflated box on both axes (0.42 < 0.55) yet sqrt(2)*0.42 =
// 0.594 from the corner, which is further than r. Inflated says
// COLLISION, exact says free.
const centre = ref({ x: 1.62, y: 1.22 })
const canvas = ref<HTMLCanvasElement | null>(null)
const verdict = ref({ inflated: false, exact: false, trueDist: 0 })
let observer: ResizeObserver | undefined
let dragging = false

// the link's bounding box, in the link's own frame
const BOX = { minx: -1.2, maxx: 1.2, miny: -0.8, maxy: 0.8 }

// ---- the test collision.js uses: inflate the box, compare the centre ----
// (reference/collision.js #link-sphere-collision, in 2D)
function inflatedAabbCollision(cx: number, cy: number, r: number) {
  if (cx < BOX.minx - r || cx > BOX.maxx + r) return false
  if (cy < BOX.miny - r || cy > BOX.maxy + r) return false
  return true
}

// ---- the exact test, which collision.js does NOT implement ----
// clamp the centre to the box, then compare squared distance to r^2
function exactSphereBoxCollision(cx: number, cy: number, r: number) {
  const qx = Math.max(BOX.minx, Math.min(BOX.maxx, cx))
  const qy = Math.max(BOX.miny, Math.min(BOX.maxy, cy))
  const dx = cx - qx, dy = cy - qy
  return { hit: dx * dx + dy * dy <= r * r, dist: Math.hypot(dx, dy) }
}

const SPAN = 4.2
function redraw() {
  const c = canvas.value
  if (!c) return
  const rect = c.getBoundingClientRect()
  if (rect.width < 2 || rect.height < 2) return
  const dpr = window.devicePixelRatio || 1
  c.width = Math.round(rect.width * dpr)
  c.height = Math.round(rect.height * dpr)
  const ctx = c.getContext('2d')
  if (!ctx) return
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)

  const s = Math.min(rect.width, rect.height) / SPAN
  const ox = rect.width / 2, oy = rect.height / 2
  const X = (x: number) => ox + x * s
  const Y = (y: number) => oy - y * s

  ctx.fillStyle = '#fff'
  ctx.fillRect(0, 0, rect.width, rect.height)

  const r = radius.value

  // the disagreement region: inside the inflated box, outside the true
  // sphere-reachable region. Shaded by sampling, which is honest about
  // it being four rounded corners rather than a shape with a nice name.
  const stepPx = 3
  ctx.fillStyle = '#fde0d2'
  for (let px = 0; px < rect.width; px += stepPx) {
    for (let py = 0; py < rect.height; py += stepPx) {
      const wx = (px - ox) / s, wy = (oy - py) / s
      if (inflatedAabbCollision(wx, wy, r) && !exactSphereBoxCollision(wx, wy, r).hit) {
        ctx.fillRect(px, py, stepPx, stepPx)
      }
    }
  }

  // the inflated box outline
  ctx.strokeStyle = '#E8710A'
  ctx.setLineDash([5, 4])
  ctx.lineWidth = 1.5
  ctx.strokeRect(X(BOX.minx - r), Y(BOX.maxy + r),
    (BOX.maxx - BOX.minx + 2 * r) * s, (BOX.maxy - BOX.miny + 2 * r) * s)
  ctx.setLineDash([])

  // the link's own box
  ctx.fillStyle = '#8a8f98'
  ctx.fillRect(X(BOX.minx), Y(BOX.maxy), (BOX.maxx - BOX.minx) * s, (BOX.maxy - BOX.miny) * s)
  ctx.strokeStyle = '#3c4043'
  ctx.lineWidth = 1.5
  ctx.strokeRect(X(BOX.minx), Y(BOX.maxy), (BOX.maxx - BOX.minx) * s, (BOX.maxy - BOX.miny) * s)

  // the obstacle sphere
  const ex = exactSphereBoxCollision(centre.value.x, centre.value.y, r)
  const infl = inflatedAabbCollision(centre.value.x, centre.value.y, r)
  verdict.value = { inflated: infl, exact: ex.hit, trueDist: ex.dist }

  ctx.beginPath()
  ctx.arc(X(centre.value.x), Y(centre.value.y), r * s, 0, 2 * Math.PI)
  ctx.fillStyle = infl !== ex.hit ? '#c5221f33' : '#4a7ebb44'
  ctx.fill()
  ctx.strokeStyle = infl !== ex.hit ? '#c5221f' : '#4a7ebb'
  ctx.lineWidth = 2
  ctx.stroke()

  ctx.beginPath()
  ctx.arc(X(centre.value.x), Y(centre.value.y), 3, 0, 2 * Math.PI)
  ctx.fillStyle = '#202124'
  ctx.fill()

  // the closest point on the box, which is what the exact test clamps to
  const qx = Math.max(BOX.minx, Math.min(BOX.maxx, centre.value.x))
  const qy = Math.max(BOX.miny, Math.min(BOX.maxy, centre.value.y))
  ctx.strokeStyle = '#202124'
  ctx.setLineDash([3, 3])
  ctx.lineWidth = 1
  ctx.beginPath()
  ctx.moveTo(X(centre.value.x), Y(centre.value.y))
  ctx.lineTo(X(qx), Y(qy))
  ctx.stroke()
  ctx.setLineDash([])
}

function toWorld(ev: MouseEvent) {
  const c = canvas.value!
  const rect = c.getBoundingClientRect()
  const s = Math.min(rect.width, rect.height) / SPAN
  return {
    x: (ev.clientX - rect.left - rect.width / 2) / s,
    y: (rect.height / 2 - (ev.clientY - rect.top)) / s,
  }
}
function onDown(ev: MouseEvent) { dragging = true; centre.value = toWorld(ev); redraw() }
function onMove(ev: MouseEvent) { if (dragging) { centre.value = toWorld(ev); redraw() } }
function onUp() { dragging = false }

watch(radius, redraw)
onMounted(() => {
  observer = new ResizeObserver(() => redraw())
  if (canvas.value) observer.observe(canvas.value)
  window.addEventListener('mouseup', onUp)
  redraw()
})
onBeforeUnmount(() => { observer?.disconnect(); window.removeEventListener('mouseup', onUp) })
</script>

<template>
  <div class="cv-panel">
    <div class="vector-canvas-wrap">
      <canvas ref="canvas" @mousedown="onDown" @mousemove="onMove"></canvas>
    </div>
    <div class="cv-side">
      <div class="cv-legend">
        <div><span class="sw sw-box"></span> link bounding box, in the link frame</div>
        <div><span class="sw sw-infl"></span> box inflated by <code>r</code> on every axis</div>
        <div><span class="sw sw-dis"></span> where the two tests disagree</div>
      </div>
      <table class="cv-table">
        <tr><th>inflated AABB <span class="cv-which">(collision.js)</span></th>
            <td :class="verdict.inflated ? 'hit' : 'miss'">{{ verdict.inflated ? 'COLLISION' : 'free' }}</td></tr>
        <tr><th>exact sphere&ndash;box <span class="cv-which">(this panel only)</span></th>
            <td :class="verdict.exact ? 'hit' : 'miss'">{{ verdict.exact ? 'COLLISION' : 'free' }}</td></tr>
        <tr><th>true distance to the box</th>
            <td>{{ verdict.trueDist.toFixed(3) }} &nbsp;vs&nbsp; r = {{ radius.toFixed(2) }}</td></tr>
      </table>
      <div class="cv-verdict" :class="{ differ: verdict.inflated !== verdict.exact }">
        {{ verdict.inflated !== verdict.exact
          ? 'The two disagree here — a false positive, at a corner.'
          : 'The two agree here.' }}
      </div>
      <div class="cv-controls">
        <span class="cv-label">radius</span>
        <input type="range" min="0.15" max="1.2" step="0.05" v-model.number="radius" />
        <span class="cv-stat">{{ radius.toFixed(2) }}</span>
      </div>
      <div class="cv-hint">Drag inside the canvas to move the obstacle.</div>
    </div>
  </div>
</template>

<style scoped>
.cv-panel { display: grid; grid-template-columns: 1.25fr 1fr; gap: 0.8em; height: 100%; min-height: 0; }
.vector-canvas-wrap {
  height: 100%; min-height: 0;
  border: 1px solid #dcdfe6; border-radius: 10px; overflow: hidden; background: #fff; cursor: crosshair;
}
.vector-canvas-wrap canvas { display: block; width: 100%; height: 100%; }
.cv-side { display: flex; flex-direction: column; gap: 0.5em; justify-content: center; font-size: 0.68em; }
.cv-legend { display: flex; flex-direction: column; gap: 0.25em; }
.sw { display: inline-block; width: 0.9em; height: 0.9em; border-radius: 3px; vertical-align: -0.1em; margin-right: 0.35em; }
.sw-box { background: #8a8f98; border: 1px solid #3c4043; }
.sw-infl { background: #fff; border: 1.5px dashed #E8710A; }
.sw-dis { background: #fde0d2; border: 1px solid #e0b89e; }
.cv-table { border-collapse: collapse; width: 100%; }
.cv-table th { text-align: left; font-weight: 600; padding: 0.2em 0; }
.cv-table td { text-align: right; font-family: var(--font-mono, monospace); font-weight: 700; }
.cv-which { font-weight: 400; opacity: 0.6; font-size: 0.85em; }
.hit { color: #c5221f; }
.miss { color: #188038; }
.cv-verdict {
  border-radius: 8px; padding: 0.4em 0.6em; background: #eef0f4; font-weight: 600;
}
.cv-verdict.differ { background: #fde0d2; color: #a8390f; }
.cv-controls { display: flex; align-items: center; gap: 0.4em; }
.cv-label { font-family: var(--font-mono, monospace); color: #5f6368; }
.cv-stat { font-family: var(--font-mono, monospace); }
.cv-hint { opacity: 0.6; font-size: 0.85em; }
</style>
