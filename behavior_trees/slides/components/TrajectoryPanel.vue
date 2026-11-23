<script setup lang="ts">
import { ref, computed, onMounted, onBeforeUnmount, watch } from 'vue'
import { PLAN } from '../lib/bt/plan'
import {
  sampleTrajectory, sampleTrajectorySpline, trajectoryDuration, trajectoryFromPlan,
} from '../lib/bt/trajectory'
import type { JointTrajectory } from '../lib/bt/types'

// The committed plan, as a trajectory, plotted against time.
//
// Every curve here is the output of trajectoryFromPlan on
// reference/plans/mr2_arm_workspace.json -- the same function the slides
// quote and the same file the demo page fetches. The two toggles are the
// two decisions the file documents: which path (smoothed or raw) and
// which interpolation (linear, as shipped, or a spline, which is shown
// only so the cost of smoothing it can be seen).
const props = withDefaults(defineProps<{
  showSpline?: boolean
  showRawToggle?: boolean
  maxVelocity?: number
}>(), { showSpline: true, showRawToggle: true, maxVelocity: 0.6 })

const useRawPath = ref(false)
const spline = ref(false)
const maxVelocity = ref(props.maxVelocity)
const t = ref(0)

const trajectory = computed<JointTrajectory>(() =>
  trajectoryFromPlan(PLAN, PLAN.joint_names, {
    useRawPath: useRawPath.value, maxVelocity: maxVelocity.value,
  })!)

const duration = computed(() => trajectoryDuration(trajectory.value))
const sample = computed(() => sampleTrajectory(trajectory.value, t.value))
const splineSample = computed(() => sampleTrajectorySpline(trajectory.value, t.value))

const COLORS = ['#3b6ea5', '#d98236', '#27966b', '#c0392b']

const canvas = ref<HTMLCanvasElement | null>(null)
let observer: ResizeObserver | null = null

function redraw() {
  const el = canvas.value
  if (!el) return
  const rect = el.getBoundingClientRect()
  // never size a backing store from a zero rect: onMounted runs before
  // layout, and an uninitialised buffer stretched over the element
  // renders as a solid block on some machines and blank on others
  if (!rect.width || !rect.height) return

  const dpr = window.devicePixelRatio || 1
  el.width = Math.round(rect.width * dpr)
  el.height = Math.round(rect.height * dpr)

  const ctx = el.getContext('2d')!
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx.clearRect(0, 0, rect.width, rect.height)

  const padL = 34, padR = 8, padT = 8, padB = 18
  const w = rect.width - padL - padR
  const h = rect.height - padT - padB

  const points = trajectory.value.points
  let lo = Infinity, hi = -Infinity
  for (const p of points) for (const v of p.positions) { lo = Math.min(lo, v); hi = Math.max(hi, v) }
  const pad = (hi - lo) * 0.08 || 0.1
  lo -= pad; hi += pad

  const X = (time: number) => padL + (time / duration.value) * w
  const Y = (value: number) => padT + h - ((value - lo) / (hi - lo)) * h

  // axes
  ctx.strokeStyle = '#c7cad1'
  ctx.lineWidth = 1
  ctx.beginPath()
  ctx.moveTo(padL, padT); ctx.lineTo(padL, padT + h); ctx.lineTo(padL + w, padT + h)
  ctx.stroke()

  ctx.fillStyle = '#5f6368'
  ctx.font = '9px ui-monospace, monospace'
  ctx.fillText(hi.toFixed(1), 4, padT + 8)
  ctx.fillText(lo.toFixed(1), 4, padT + h)
  ctx.fillText('0s', padL, rect.height - 5)
  ctx.fillText(duration.value.toFixed(1) + 's', padL + w - 24, rect.height - 5)

  // waypoint times, as vertical rules: the discrete plan under the
  // continuous curve
  ctx.strokeStyle = '#e4e6ea'
  for (const p of points) {
    ctx.beginPath()
    ctx.moveTo(X(p.time_from_start), padT)
    ctx.lineTo(X(p.time_from_start), padT + h)
    ctx.stroke()
  }

  const steps = 220
  for (let j = 0; j < trajectory.value.joint_names.length; j++) {
    // the shipped linear interpolation
    ctx.strokeStyle = COLORS[j % COLORS.length]
    ctx.lineWidth = 1.6
    ctx.beginPath()
    for (let s = 0; s <= steps; s++) {
      const time = (s / steps) * duration.value
      const v = sampleTrajectory(trajectory.value, time).positions[j]
      if (s === 0) ctx.moveTo(X(time), Y(v)); else ctx.lineTo(X(time), Y(v))
    }
    ctx.stroke()

    // the spline, for comparison only
    if (spline.value) {
      ctx.setLineDash([3, 3])
      ctx.lineWidth = 1.2
      ctx.beginPath()
      for (let s = 0; s <= steps; s++) {
        const time = (s / steps) * duration.value
        const v = sampleTrajectorySpline(trajectory.value, time)[j]
        if (s === 0) ctx.moveTo(X(time), Y(v)); else ctx.lineTo(X(time), Y(v))
      }
      ctx.stroke()
      ctx.setLineDash([])
    }

    // the waypoints themselves
    ctx.fillStyle = COLORS[j % COLORS.length]
    for (const p of points) {
      ctx.beginPath()
      ctx.arc(X(p.time_from_start), Y(p.positions[j]), useRawPath.value ? 1.6 : 2.6, 0, Math.PI * 2)
      ctx.fill()
    }
  }

  // the cursor
  ctx.strokeStyle = '#202B4E'
  ctx.lineWidth = 1.4
  ctx.beginPath()
  ctx.moveTo(X(t.value), padT); ctx.lineTo(X(t.value), padT + h)
  ctx.stroke()
  for (let j = 0; j < sample.value.positions.length; j++) {
    ctx.fillStyle = '#202B4E'
    ctx.beginPath()
    ctx.arc(X(t.value), Y(sample.value.positions[j]), 2.4, 0, Math.PI * 2)
    ctx.fill()
  }
}

onMounted(() => {
  if (!canvas.value) return
  observer = new ResizeObserver(() => redraw())
  observer.observe(canvas.value)
  redraw()
})
onBeforeUnmount(() => observer?.disconnect())
watch([t, spline, useRawPath, maxVelocity], () => redraw())
watch(duration, (d) => { if (t.value > d) t.value = d })

const worstSplineGap = computed(() => {
  let worst = 0
  for (let s = 0; s <= 200; s++) {
    const time = (s / 200) * duration.value
    const a = sampleTrajectory(trajectory.value, time).positions
    const b = sampleTrajectorySpline(trajectory.value, time)
    for (let j = 0; j < a.length; j++) worst = Math.max(worst, Math.abs(a[j] - b[j]))
  }
  return worst
})
</script>

<template>
  <div class="traj-shell">
    <div class="controls">
      <label v-if="showRawToggle"><input v-model="useRawPath" type="checkbox"> raw path ({{ PLAN.search!.raw_path_length }} waypoints)</label>
      <label v-if="showSpline"><input v-model="spline" type="checkbox"> overlay a spline</label>
      <label>v<sub>max</sub> <input v-model.number="maxVelocity" type="range" min="0.2" max="2" step="0.1"> {{ maxVelocity.toFixed(1) }} rad/s</label>
      <span class="tag">{{ trajectory.points.length }} points &middot; {{ duration.toFixed(2) }}s</span>
    </div>

    <canvas ref="canvas" class="plot" />

    <div class="scrub">
      <label>t = <input v-model.number="t" type="range" min="0" :max="duration" step="0.01"> {{ t.toFixed(2) }}s</label>
      <span class="tag">segment {{ sample.segment }}, {{ (sample.alpha * 100).toFixed(0) }}% through</span>
    </div>

    <table class="sample-table">
      <tr>
        <th></th>
        <th v-for="(n, j) in trajectory.joint_names" :key="n" :style="{ color: COLORS[j % 4] }">{{ n.replace(/_/g, ' ') }}</th>
      </tr>
      <tr>
        <td>linear</td>
        <td v-for="(v, j) in sample.positions" :key="j">{{ v.toFixed(3) }}</td>
      </tr>
      <tr v-if="spline" class="spline-row">
        <td>spline</td>
        <td v-for="(v, j) in splineSample" :key="j">{{ v.toFixed(3) }}</td>
      </tr>
    </table>

    <p v-if="spline" class="note">
      The dashed spline leaves the straight line between waypoints by up to
      <b>{{ worstSplineGap.toFixed(3) }} rad</b>. That straight line is what the planner
      validated against the obstacles; the spline is not.
    </p>
  </div>
</template>

<style scoped>
.traj-shell { display: flex; flex-direction: column; height: 100%; min-height: 0; gap: 0.3em; }
.controls, .scrub {
  display: flex; flex-wrap: wrap; align-items: center; gap: 0.8em;
  font-size: 0.6em; font-family: var(--font-mono, monospace);
}
.controls input[type="range"], .scrub input[type="range"] { width: 6em; vertical-align: middle; }
.tag {
  font-weight: 700; background: #fdecd9; border: 1px solid #202B4E; color: #202B4E;
  border-radius: 999px; padding: 0.1em 0.6em;
}
.plot { flex: 1 1 auto; min-height: 0; height: 100%; width: 100%; display: block; }
.sample-table { font-size: 0.58em; font-family: var(--font-mono, monospace); border-collapse: collapse; }
.sample-table th, .sample-table td { padding: 0.08em 0.6em 0.08em 0; text-align: left; font-weight: 400; }
.sample-table th { opacity: 0.8; font-weight: 700; }
.sample-table td:first-child { opacity: 0.6; }
.spline-row td { opacity: 0.7; font-style: italic; }
.note { margin: 0; font-size: 0.56em; opacity: 0.85; }
</style>
