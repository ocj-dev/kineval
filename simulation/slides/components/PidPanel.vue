<script setup lang="ts">
import { reactive, ref, watch } from 'vue'
import PhaseStepperShell from './PhaseStepperShell.vue'
import { usePhaseTracer } from '../lib/pendulum/usePhaseTracer'
import { useCanvasRenderer } from '../lib/pendulum/useCanvasRenderer'
import { pendulumAcceleration, integrateRK4 } from '../lib/pendulum/pendulumPhysics'
import { drawPendulumChain, drawVerticalReference, drawTimeSeries, clearCanvas, MAIZE } from '../lib/pendulum/drawUtils'
import { MASTER_PSEUDOCODE, LINE_ERROR, LINE_PID, LINE_ACCEL, LINE_INTEGRATE } from '../lib/pendulum/pseudocode'

// P -> D -> I, tuned in that order (per the AutoRob "Motion Control and
// PID" lecture): defaults are the known-good kp=150/kd=60/ki=4 for this
// mass=2/length=2/gravity=9.81 pendulum (the same gains the TA's own lab
// overview slide cites). Drag any gain to 0 to see the lecture's own
// tuning steps in reverse -- kd=0 stops damping the overshoot; ki=0 leaves
// a small steady-state gap gravity never lets a P+D-only controller close.

const GRAVITY = 9.81, MASS = 2.0, LENGTH = 2.0, DT = 0.02
const desired = reactive({ value: -1.0 })
const gains = reactive({ kp: 150, kd: 60, ki: 4 })
const releaseAngle = reactive({ value: Math.PI / 2 })

type Phase = 'error' | 'pid' | 'accelerate' | 'integrate'
interface Entry {
  frame: number; phase: Phase; angle: number; error: number; control: number
  pTerm: number; iTerm: number; dTerm: number; errorSeries: number[]
}

let angle = [releaseAngle.value], angle_dot = [0]
let previous_error = [0], accumulated_error = [0], control = [0]
let pTerm = 0, iTerm = 0, dTerm = 0
let frame = 0
const errorSeries: number[] = []

function resetSim() {
  angle = [releaseAngle.value]; angle_dot = [0]
  previous_error = [0]; accumulated_error = [0]; control = [0]
  pTerm = 0; iTerm = 0; dTerm = 0
  frame = 0
  errorSeries.length = 0
}

function generateFrame(): Entry[] {
  frame++
  const error = desired.value - angle[0]
  const e1: Entry = { frame, phase: 'error', angle: angle[0], error, control: control[0], pTerm, iTerm, dTerm, errorSeries: errorSeries.slice() }

  // Same formula PID() in pendulumPhysics.ts uses, computed inline so each
  // term's individual contribution to `control` can be displayed alongside
  // the slider that tunes it.
  const derivative = (error - previous_error[0]) / DT
  accumulated_error = [accumulated_error[0] + error * DT]
  previous_error = [error]
  pTerm = gains.kp * error
  dTerm = gains.kd * derivative
  iTerm = gains.ki * accumulated_error[0]
  control = [pTerm + iTerm + dTerm]
  const e2: Entry = { frame, phase: 'pid', angle: angle[0], error, control: control[0], pTerm, iTerm, dTerm, errorSeries: errorSeries.slice() }

  const accel = pendulumAcceleration(angle, angle_dot, control, GRAVITY, [MASS], [LENGTH])
  const e3: Entry = { frame, phase: 'accelerate', angle: angle[0], error, control: control[0], pTerm, iTerm, dTerm, errorSeries: errorSeries.slice() }

  const result = integrateRK4(angle, angle_dot, (a, w) => pendulumAcceleration(a, w, control, GRAVITY, [MASS], [LENGTH]), DT)
  angle = result.angle; angle_dot = result.angle_dot
  errorSeries.push(error)
  const e4: Entry = { frame, phase: 'integrate', angle: angle[0], error, control: control[0], pTerm, iTerm, dTerm, errorSeries: errorSeries.slice() }

  return [e1, e2, e3, e4]
}

const tracer = usePhaseTracer<Entry>(generateFrame, resetSim)
const activeLineFor: Record<Phase, number> = { error: LINE_ERROR, pid: LINE_PID, accelerate: LINE_ACCEL, integrate: LINE_INTEGRATE }
const labelFor: Record<Phase, string> = { error: 'Compute error', pid: 'PID control torque', accelerate: 'Accelerate', integrate: 'Integrate' }

const canvasEl = ref<HTMLCanvasElement | null>(null)
const plotEl = ref<HTMLCanvasElement | null>(null)

const { redraw: redrawPendulum } = useCanvasRenderer(canvasEl, (ctx, w, h) => {
  clearCanvas(ctx, w, h)
  const e = tracer.current.value
  if (!e) return
  const pivotX = w / 2, pivotY = h * 0.15, linkLen = h * 0.55
  drawVerticalReference(ctx, pivotX, pivotY, linkLen)
  const desiredX = pivotX + linkLen * Math.sin(desired.value), desiredY = pivotY + linkLen * Math.cos(desired.value)
  ctx.strokeStyle = '#0d652d'; ctx.setLineDash([3, 4]); ctx.lineWidth = 1.5
  ctx.beginPath(); ctx.moveTo(pivotX, pivotY); ctx.lineTo(desiredX, desiredY); ctx.stroke(); ctx.setLineDash([])
  drawPendulumChain(ctx, pivotX, pivotY, [{ angleAbs: e.angle, length: linkLen, color: MAIZE }])

  ctx.font = '12px "Roboto Mono", monospace'
  ctx.fillStyle = '#202124'
  ctx.textAlign = 'left'
  ctx.fillText(`error = ${e.error.toFixed(3)}   control = ${e.control.toFixed(1)}`, 12, h - 14)
})

const { redraw: redrawPlot } = useCanvasRenderer(plotEl, (ctx, w, h) => {
  const e = tracer.current.value
  if (!e) return
  drawTimeSeries(ctx, w, h, [{ label: 'error (rad)', color: '#c0392b', points: e.errorSeries }], { yLabel: 'error', zeroLine: true })
})

function redraw() { redrawPendulum(); redrawPlot() }
watch(tracer.current, redraw)
function onGainInput() { tracer.reset() }
</script>

<template>
  <PhaseStepperShell
    :pseudocode-lines="MASTER_PSEUDOCODE"
    :active-line="tracer.current.value ? activeLineFor[tracer.current.value.phase] : -1"
    :is-running="tracer.isRunning.value" :is-done="tracer.isDone.value" :is-at-start="tracer.isAtStart.value"
    v-model:smooth="tracer.smooth.value"
    :step-label="tracer.current.value ? labelFor[tracer.current.value.phase] : ''"
    :frame-info="`frame ${tracer.current.value?.frame ?? 0}`"
    @play="tracer.play" @pause="tracer.pause" @step-forward="tracer.stepForward" @step-back="tracer.stepBack" @reset="tracer.reset"
  >
    <template #controls>
      <div class="gain-control">
        <label class="check">kp <input type="range" min="0" max="220" step="5" v-model.number="gains.kp" @input="onGainInput"> {{ gains.kp }}</label>
        <div class="term-readout">P: kp&times;error = {{ (tracer.current.value?.pTerm ?? 0).toFixed(1) }}</div>
      </div>
      <div class="gain-control">
        <label class="check">kd <input type="range" min="0" max="120" step="2" v-model.number="gains.kd" @input="onGainInput"> {{ gains.kd }}</label>
        <div class="term-readout">D: kd&times;d(error)/dt = {{ (tracer.current.value?.dTerm ?? 0).toFixed(1) }}</div>
      </div>
      <div class="gain-control">
        <label class="check">ki <input type="range" min="0" max="20" step="0.5" v-model.number="gains.ki" @input="onGainInput"> {{ gains.ki }}</label>
        <div class="term-readout">I: ki&times;sum(error*dt) = {{ (tracer.current.value?.iTerm ?? 0).toFixed(1) }}</div>
      </div>
      <label class="check">desired <input type="range" min="-2.5" max="2.5" step="0.1" v-model.number="desired.value" @input="onGainInput"> {{ desired.value.toFixed(1) }}</label>
      <label class="check">release angle <input type="range" min="-3.0" max="3.0" step="0.05" v-model.number="releaseAngle.value" @input="onGainInput"> {{ releaseAngle.value.toFixed(2) }} rad</label>
    </template>
    <div class="pid-layout">
      <div class="vector-canvas-wrap"><canvas ref="canvasEl" /></div>
      <div class="vector-canvas-wrap plot"><canvas ref="plotEl" /></div>
    </div>
  </PhaseStepperShell>
</template>

<style scoped>
.check { font-family: var(--font-mono, monospace); font-size: 0.7em; display: flex; align-items: center; gap: 0.4em; }
input[type="range"] { width: 5.5em; }
.gain-control { display: flex; flex-direction: column; align-items: flex-start; gap: 0.15em; }
.term-readout { font-family: var(--font-mono, monospace); font-size: 0.62em; opacity: 0.7; }
.pid-layout { display: flex; gap: 0.5em; height: 100%; min-height: 0; }
.pid-layout .vector-canvas-wrap { flex: 1 1 50%; }
.pid-layout .plot { flex-basis: 40%; }
</style>
