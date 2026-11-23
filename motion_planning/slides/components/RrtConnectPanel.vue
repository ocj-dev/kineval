<script setup lang="ts">
// Step-through of RRT-Connect, with the executing pseudocode line
// highlighted. The trace comes from lib/planning/planner.ts's generator,
// which is a function-for-function port of
// motion_planning/reference/rrt.js -- so what is drawn is produced by the
// same algorithm the surrounding slides quote.
//
// The whole trace is precomputed on mount, which makes Play / Pause /
// Step / Reset nothing more than index navigation.
import { ref, computed, onMounted, onBeforeUnmount, watch } from 'vue'
import PseudocodePanel from './PseudocodePanel.vue'
import { rrtConnectTrace, RRT_CONNECT_LINES, SCENES, sceneRanges, type Step } from '../lib/planning/planner'
import { prepareCanvas, clear, drawObstacles, drawEdges, drawPath, drawMarker, drawLabel } from '../lib/planning/draw'

const props = withDefaults(defineProps<{
  scene?: string
  seed?: number
  speed?: number
  autoplay?: boolean
}>(), { scene: 'construction_detour', seed: 424242, speed: 110, autoplay: false })

const canvas = ref<HTMLCanvasElement | null>(null)
const steps = ref<Step[]>([])
const index = ref(0)
const playing = ref(false)
let timer: number | undefined
let observer: ResizeObserver | undefined

const current = computed<Step | undefined>(() => steps.value[index.value])
const activeLine = computed(() => current.value?.line ?? 0)

function buildTrace() {
  const out: Step[] = []
  for (const s of rrtConnectTrace(props.scene, props.seed)) {
    out.push(s)
    if (out.length > 1200) break
  }
  steps.value = out
  index.value = 0
}

function redraw() {
  const c = canvas.value
  if (!c) return
  const v = prepareCanvas(c)
  if (!v) return

  const ranges = sceneRanges(props.scene)
  const scene = SCENES[props.scene]
  clear(v)
  drawObstacles(v, ranges)

  const s = current.value
  if (!s) return

  drawEdges(v, s.treeA, '#1a73e8', 1)
  drawEdges(v, s.treeB, '#188038', 1)
  if (s.highlight) drawEdges(v, [s.highlight], '#E8710A', 2.5)

  if (s.path) drawPath(v, s.path, '#c5221f', 2)
  if (s.smoothed) drawPath(v, s.smoothed, '#E8710A', 3.5)

  if (s.qRand) { drawMarker(v, s.qRand, '#9334e6', 4, true); drawLabel(v, s.qRand, 'q_rand', '#9334e6') }
  if (s.qNew) drawMarker(v, s.qNew, '#E8710A', 3.5)

  drawMarker(v, scene.init, '#1a73e8', 5)
  drawLabel(v, scene.init, 'start', '#1a73e8')
  drawMarker(v, scene.goal, '#188038', 5)
  drawLabel(v, scene.goal, 'goal', '#188038')
}

function step(d = 1) {
  index.value = Math.max(0, Math.min(steps.value.length - 1, index.value + d))
  if (index.value >= steps.value.length - 1) pause()
  redraw()
}
function play() {
  if (playing.value) return
  if (index.value >= steps.value.length - 1) index.value = 0
  playing.value = true
  timer = window.setInterval(() => step(1), props.speed)
}
function pause() {
  playing.value = false
  if (timer !== undefined) { window.clearInterval(timer); timer = undefined }
}
function reset() { pause(); index.value = 0; redraw() }

watch(() => [props.scene, props.seed], () => { pause(); buildTrace(); redraw() })

onMounted(() => {
  buildTrace()
  if (canvas.value) {
    // first paint and every resize are driven from the observer, since
    // onMounted runs before layout and the rect is 0x0 there
    observer = new ResizeObserver(() => redraw())
    observer.observe(canvas.value)
  }
  redraw()
  if (props.autoplay) play()
})
onBeforeUnmount(() => { pause(); observer?.disconnect() })
</script>

<template>
  <div class="rrt-panel">
    <div class="rrt-left">
      <PseudocodePanel :lines="RRT_CONNECT_LINES" :active-line="activeLine" />
      <div class="rrt-note">{{ current?.note }}</div>
    </div>
    <div class="rrt-right">
      <div class="vector-canvas-wrap"><canvas ref="canvas"></canvas></div>
      <div class="rrt-controls">
        <button class="btn" @click="playing ? pause() : play()">{{ playing ? 'Pause' : 'Play' }}</button>
        <button class="btn" @click="pause(); step(-1)">&larr;</button>
        <button class="btn" @click="pause(); step(1)">Step &rarr;</button>
        <button class="btn" @click="reset()">Reset</button>
        <span class="rrt-stat">iter {{ current?.iteration ?? 0 }}</span>
        <span class="rrt-stat">step {{ index + 1 }}/{{ steps.length }}</span>
        <span class="rrt-stat">seed {{ props.seed }}</span>
      </div>
    </div>
  </div>
</template>

<style scoped>
.rrt-panel {
  display: grid;
  grid-template-columns: 1.05fr 1fr;
  gap: 0.7em;
  height: 100%;
  min-height: 0;
}
.rrt-left, .rrt-right {
  display: flex;
  flex-direction: column;
  min-height: 0;
  gap: 0.4em;
}
.rrt-left :deep(.pseudocode) { flex: 1 1 auto; min-height: 0; }
.rrt-note {
  font-size: 0.62em;
  line-height: 1.35;
  background: #fff;
  border: 1px solid #dcdfe6;
  border-radius: 8px;
  padding: 0.45em 0.6em;
  min-height: 3.2em;
}
.vector-canvas-wrap {
  flex: 1 1 auto;
  height: 100%;
  min-height: 0;
  border: 1px solid #dcdfe6;
  border-radius: 10px;
  overflow: hidden;
  background: #fff;
}
.vector-canvas-wrap canvas { display: block; width: 100%; height: 100%; }
.rrt-controls {
  display: flex;
  align-items: center;
  gap: 0.35em;
  flex-wrap: wrap;
}
.rrt-stat {
  font-family: var(--font-mono, monospace);
  font-size: 0.6em;
  color: #5f6368;
  background: #eef0f4;
  border-radius: 999px;
  padding: 0.15em 0.6em;
}
</style>
