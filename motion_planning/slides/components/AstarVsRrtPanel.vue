<script setup lang="ts">
// A* and RRT-Connect on the same five scenes.
//
// The strongest comparison available in this course: the pathfinding deck
// already taught these exact worlds with A*, and graph_search.js is
// carried into motion_planning/reference/ for precisely this slide. Both
// planners here are ports of the carried reference code, run on the same
// `range` array, from the same start to the same goal.
import { ref, onMounted, onBeforeUnmount, watch } from 'vue'
import { SCENES, sceneRanges, growRrtConnect } from '../lib/planning/planner'
import { runAstar } from '../lib/planning/astar'
import { prepareCanvas, clear, drawObstacles, drawEdges, drawPath, drawDots, drawMarker } from '../lib/planning/draw'

const props = withDefaults(defineProps<{ scene?: string; seed?: number }>(),
  { scene: 'spiral', seed: 424242 })

const sceneName = ref(props.scene)
const names = Object.keys(SCENES)
const ca = ref<HTMLCanvasElement | null>(null)
const cr = ref<HTMLCanvasElement | null>(null)
const aStat = ref('')
const rStat = ref('')
let observer: ResizeObserver | undefined

function redraw() {
  const scene = SCENES[sceneName.value]
  const ranges = sceneRanges(sceneName.value)

  const va = prepareCanvas(ca.value!)
  if (va) {
    clear(va); drawObstacles(va, ranges)
    const r = runAstar(ranges, scene.init, scene.goal)
    drawDots(va, r.expanded, '#c7d7f5', 1.1)
    if (r.path) drawPath(va, r.path, '#1a73e8', 3)
    drawMarker(va, scene.init, '#1a73e8', 5)
    drawMarker(va, scene.goal, '#188038', 5)
    aStat.value = r.path
      ? `${r.visited} nodes expanded · path ${r.length.toFixed(2)}`
      : `${r.visited} nodes expanded · no path`
  }

  const vr = prepareCanvas(cr.value!)
  if (vr) {
    clear(vr); drawObstacles(vr, ranges)
    const r = growRrtConnect(sceneName.value, props.seed, 4000)
    drawEdges(vr, r.a, '#f0b8b6', 1)
    drawEdges(vr, r.b, '#bfe0c5', 1)
    if (r.raw) drawPath(vr, r.raw, '#c5221f', 1.8)
    if (r.smoothed) drawPath(vr, r.smoothed, '#E8710A', 3.2)
    drawMarker(vr, scene.init, '#1a73e8', 5)
    drawMarker(vr, scene.goal, '#188038', 5)
    const len = r.smoothed ? r.smoothed.reduce((s, p, i, arr) =>
      i ? s + Math.hypot(p[0] - arr[i - 1][0], p[1] - arr[i - 1][1]) : 0, 0) : 0
    rStat.value = r.raw
      ? `${r.vertices} vertices · ${r.iterations} iterations · path ${len.toFixed(2)}`
      : `${r.vertices} vertices · no path in ${r.iterations}`
  }
}

watch([sceneName, () => props.seed], redraw)
onMounted(() => {
  observer = new ResizeObserver(() => redraw())
  if (ca.value) observer.observe(ca.value)
  if (cr.value) observer.observe(cr.value)
  redraw()
})
onBeforeUnmount(() => observer?.disconnect())
</script>

<template>
  <div class="av-panel">
    <div class="av-grid">
      <div class="av-cell">
        <div class="av-title">A* &mdash; <span class="av-sub">resolution-complete, optimal on its grid</span></div>
        <div class="vector-canvas-wrap"><canvas ref="ca"></canvas></div>
        <div class="av-stat">{{ aStat }}</div>
      </div>
      <div class="av-cell">
        <div class="av-title">RRT-Connect &mdash; <span class="av-sub">probabilistically complete, not optimal</span></div>
        <div class="vector-canvas-wrap"><canvas ref="cr"></canvas></div>
        <div class="av-stat">{{ rStat }}</div>
      </div>
    </div>
    <div class="av-controls">
      <span class="av-label">scene</span>
      <button v-for="n in names" :key="n" class="btn av-btn"
              :class="{ on: n === sceneName }" @click="sceneName = n">{{ n }}</button>
      <span class="av-stat">seed {{ props.seed }}</span>
    </div>
  </div>
</template>

<style scoped>
.av-panel { display: flex; flex-direction: column; height: 100%; min-height: 0; gap: 0.4em; }
.av-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 0.7em; flex: 1 1 auto; min-height: 0; }
.av-cell { display: flex; flex-direction: column; min-height: 0; gap: 0.25em; }
.av-title { font-size: 0.62em; font-weight: 700; font-family: var(--font-mono, monospace); color: var(--indigo, #202B4E); }
.av-sub { font-weight: 400; opacity: 0.75; }
.vector-canvas-wrap {
  flex: 1 1 auto; height: 100%; min-height: 0;
  border: 1px solid #dcdfe6; border-radius: 10px; overflow: hidden; background: #fff;
}
.vector-canvas-wrap canvas { display: block; width: 100%; height: 100%; }
.av-stat {
  font-family: var(--font-mono, monospace); font-size: 0.58em; color: #5f6368;
  background: #eef0f4; border-radius: 999px; padding: 0.15em 0.6em;
}
.av-controls { display: flex; align-items: center; gap: 0.3em; flex-wrap: wrap; }
.av-label { font-family: var(--font-mono, monospace); font-size: 0.6em; color: #5f6368; }
.av-btn { font-size: 0.72em; padding: 0.15em 0.6em; }
.av-btn.on { background: var(--amber, #E8710A); color: #fff; border-color: var(--amber, #E8710A); }
</style>
