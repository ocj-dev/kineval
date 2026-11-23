<script setup lang="ts">
// Extend versus connect, on one world, at one seed, for one iteration
// budget -- the panel that carries the algorithm's actual content.
//
// Three planners that differ by exactly one line:
//   1. single-tree RRT         EXTEND only, one tree
//   2. bidirectional RRT       EXTEND only, two trees
//   3. RRT-Connect             EXTEND + greedy CONNECT, two trees
//
// Same scene, same seed, same iteration cap, so the difference on screen
// is the algorithm and nothing else.
import { ref, onMounted, onBeforeUnmount, watch } from 'vue'
import { growSingleTree, growBidirectionalExtendOnly, growRrtConnect, SCENES, sceneRanges } from '../lib/planning/planner'
import { prepareCanvas, clear, drawObstacles, drawEdges, drawPath, drawMarker } from '../lib/planning/draw'

const props = withDefaults(defineProps<{ scene?: string; seed?: number; iterations?: number }>(),
  { scene: 'diagonal_staircase', seed: 20261122, iterations: 160 })

const c1 = ref<HTMLCanvasElement | null>(null)
const c2 = ref<HTMLCanvasElement | null>(null)
const c3 = ref<HTMLCanvasElement | null>(null)
const iterations = ref(props.iterations)
const stats = ref({ single: '', bidir: '', connect: '' })
let observer: ResizeObserver | undefined

function base(canvas: HTMLCanvasElement | null) {
  if (!canvas) return null
  const v = prepareCanvas(canvas)
  if (!v) return null
  clear(v)
  drawObstacles(v, sceneRanges(props.scene))
  return v
}

function redraw() {
  const scene = SCENES[props.scene]
  const n = iterations.value

  const v1 = base(c1.value)
  if (v1) {
    const r = growSingleTree(props.scene, props.seed, n)
    drawEdges(v1, r.edges, '#1a73e8', 1)
    drawMarker(v1, scene.init, '#1a73e8', 4)
    drawMarker(v1, scene.goal, '#188038', 4)
    stats.value.single = `${r.tree.vertices.length} vertices · goal ${r.reached > 0 ? 'reached at iter ' + r.reached : 'not reached'}`
  }

  const v2 = base(c2.value)
  if (v2) {
    const r = growBidirectionalExtendOnly(props.scene, props.seed, n)
    drawEdges(v2, r.a, '#1a73e8', 1)
    drawEdges(v2, r.b, '#188038', 1)
    drawMarker(v2, scene.init, '#1a73e8', 4)
    drawMarker(v2, scene.goal, '#188038', 4)
    stats.value.bidir = `${r.vertices} vertices · trees ${r.met > 0 ? 'met at iter ' + r.met : 'have not met'}`
  }

  const v3 = base(c3.value)
  if (v3) {
    const r = growRrtConnect(props.scene, props.seed, n)
    drawEdges(v3, r.a, '#1a73e8', 1)
    drawEdges(v3, r.b, '#188038', 1)
    if (r.raw) drawPath(v3, r.raw, '#c5221f', 2)
    if (r.smoothed) drawPath(v3, r.smoothed, '#E8710A', 3)
    drawMarker(v3, scene.init, '#1a73e8', 4)
    drawMarker(v3, scene.goal, '#188038', 4)
    stats.value.connect = r.raw
      ? `${r.vertices} vertices · solved at iter ${r.iterations}`
      : `${r.vertices} vertices · not solved`
  }
}

watch([iterations, () => props.scene, () => props.seed], redraw)
onMounted(() => {
  observer = new ResizeObserver(() => redraw())
  for (const c of [c1.value, c2.value, c3.value]) if (c) observer.observe(c)
  redraw()
})
onBeforeUnmount(() => observer?.disconnect())
</script>

<template>
  <div class="ec-panel">
    <div class="ec-grid">
      <div class="ec-cell">
        <div class="ec-title">1 &middot; RRT &mdash; <span class="ec-sub">EXTEND, one tree</span></div>
        <div class="vector-canvas-wrap"><canvas ref="c1"></canvas></div>
        <div class="ec-stat">{{ stats.single }}</div>
      </div>
      <div class="ec-cell">
        <div class="ec-title">2 &middot; Bidirectional &mdash; <span class="ec-sub">EXTEND, two trees</span></div>
        <div class="vector-canvas-wrap"><canvas ref="c2"></canvas></div>
        <div class="ec-stat">{{ stats.bidir }}</div>
      </div>
      <div class="ec-cell">
        <div class="ec-title">3 &middot; RRT-Connect &mdash; <span class="ec-sub">EXTEND + greedy CONNECT</span></div>
        <div class="vector-canvas-wrap"><canvas ref="c3"></canvas></div>
        <div class="ec-stat">{{ stats.connect }}</div>
      </div>
    </div>
    <div class="ec-controls">
      <span class="ec-label">iterations</span>
      <input type="range" min="10" max="400" step="10" v-model.number="iterations" />
      <span class="ec-stat">{{ iterations }}</span>
      <span class="ec-stat">scene {{ props.scene }}</span>
      <span class="ec-stat">seed {{ props.seed }}</span>
    </div>
  </div>
</template>

<style scoped>
.ec-panel { display: flex; flex-direction: column; height: 100%; min-height: 0; gap: 0.4em; }
.ec-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 0.6em; flex: 1 1 auto; min-height: 0; }
.ec-cell { display: flex; flex-direction: column; min-height: 0; gap: 0.25em; }
.ec-title { font-size: 0.6em; font-weight: 700; font-family: var(--font-mono, monospace); color: var(--indigo, #202B4E); }
.ec-sub { font-weight: 400; opacity: 0.75; }
.vector-canvas-wrap {
  flex: 1 1 auto; height: 100%; min-height: 0;
  border: 1px solid #dcdfe6; border-radius: 10px; overflow: hidden; background: #fff;
}
.vector-canvas-wrap canvas { display: block; width: 100%; height: 100%; }
.ec-stat {
  font-family: var(--font-mono, monospace); font-size: 0.56em; color: #5f6368;
  background: #eef0f4; border-radius: 999px; padding: 0.15em 0.6em;
}
.ec-controls { display: flex; align-items: center; gap: 0.4em; }
.ec-label { font-family: var(--font-mono, monospace); font-size: 0.6em; color: #5f6368; }
.ec-controls input[type=range] { width: 11em; }
</style>
