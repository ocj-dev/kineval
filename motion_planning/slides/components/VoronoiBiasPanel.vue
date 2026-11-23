<script setup lang="ts">
// Voronoi bias, stated as what it actually is.
//
// The claim is about the DISTRIBUTION OF EXPANSION, not a guarantee and
// not a property of the sampler. Sampling is uniform. What follows from
// that is: a sample is routed to whichever vertex is nearest, so the
// probability a given vertex is chosen for extension equals the measure
// of its Voronoi cell clipped to the sampling region. Frontier vertices
// own enormous cells, so the tree reaches outward -- without anything
// anywhere "aiming" it.
//
// The cells are computed by brute force over pixels (nearest vertex per
// pixel), which is slow and exactly right for a picture.
import { ref, onMounted, onBeforeUnmount, watch } from 'vue'
import { growSingleTree, SCENES, sceneRanges } from '../lib/planning/planner'
import { prepareCanvas, clear, drawObstacles, drawEdges, drawDots, vx, vy, WORLD_MIN, WORLD_MAX } from '../lib/planning/draw'

const props = withDefaults(defineProps<{ scene?: string; seed?: number }>(),
  { scene: 'construction_detour', seed: 7 })

const iterations = ref(40)
const showCells = ref(true)
const canvas = ref<HTMLCanvasElement | null>(null)
const biggest = ref('')
let observer: ResizeObserver | undefined

function redraw() {
  const c = canvas.value
  if (!c) return
  const v = prepareCanvas(c)
  if (!v) return
  clear(v)

  const r = growSingleTree(props.scene, props.seed, iterations.value)
  const pts = r.tree.vertices.map(x => x.vertex)

  if (showCells.value && pts.length) {
    // nearest-vertex colouring over the sampling region [-2, 6)^2,
    // which is the region randomConfig actually draws from
    const step = 4
    const counts = new Array(pts.length).fill(0)
    for (let px = 0; px < v.w; px += step) {
      for (let py = 0; py < v.h; py += step) {
        const wx = WORLD_MIN + (px - v.ox) / v.scale
        const wy = WORLD_MAX - (py - v.oy) / v.scale
        if (wx < -2 || wx > 6 || wy < -2 || wy > 6) continue
        let best = 0, bd = Infinity
        for (let i = 0; i < pts.length; i++) {
          const d = (pts[i][0] - wx) ** 2 + (pts[i][1] - wy) ** 2
          if (d < bd) { bd = d; best = i }
        }
        counts[best]++
        const hue = (best * 47) % 360
        v.ctx.fillStyle = `hsl(${hue} 62% 88%)`
        v.ctx.fillRect(px, py, step, step)
      }
    }
    const total = counts.reduce((a, b) => a + b, 0) || 1
    const top = counts.indexOf(Math.max(...counts))
    biggest.value = `largest cell: vertex ${top}, ${(100 * counts[top] / total).toFixed(1)}% of the sampling region`
  } else {
    biggest.value = ''
  }

  drawObstacles(v, sceneRanges(props.scene), '#5f6368')
  drawEdges(v, r.edges, '#202B4E', 1.2)
  drawDots(v, pts, '#E8710A', 2.2)
}

watch([iterations, showCells], redraw)
onMounted(() => {
  observer = new ResizeObserver(() => redraw())
  if (canvas.value) observer.observe(canvas.value)
  redraw()
})
onBeforeUnmount(() => observer?.disconnect())
</script>

<template>
  <div class="vb-panel">
    <div class="vector-canvas-wrap"><canvas ref="canvas"></canvas></div>
    <div class="vb-controls">
      <span class="vb-label">iterations</span>
      <input type="range" min="3" max="160" step="1" v-model.number="iterations" />
      <span class="vb-stat">{{ iterations }}</span>
      <label class="vb-check"><input type="checkbox" v-model="showCells" /> Voronoi cells</label>
      <span class="vb-stat">{{ biggest }}</span>
    </div>
  </div>
</template>

<style scoped>
.vb-panel { display: flex; flex-direction: column; height: 100%; min-height: 0; gap: 0.4em; }
.vector-canvas-wrap {
  flex: 1 1 auto; height: 100%; min-height: 0;
  border: 1px solid #dcdfe6; border-radius: 10px; overflow: hidden; background: #fff;
}
.vector-canvas-wrap canvas { display: block; width: 100%; height: 100%; }
.vb-controls { display: flex; align-items: center; gap: 0.5em; flex-wrap: wrap; }
.vb-label, .vb-check { font-family: var(--font-mono, monospace); font-size: 0.62em; color: #5f6368; }
.vb-controls input[type=range] { width: 12em; }
.vb-stat {
  font-family: var(--font-mono, monospace); font-size: 0.58em; color: #5f6368;
  background: #eef0f4; border-radius: 999px; padding: 0.15em 0.6em;
}
</style>
