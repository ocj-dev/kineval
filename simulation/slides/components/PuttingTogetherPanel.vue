<script setup lang="ts">
import { reactive, ref, watch } from 'vue'
import PhaseStepperShell from './PhaseStepperShell.vue'
import { usePhaseTracer } from '../lib/pendulum/usePhaseTracer'
import { useCanvasRenderer } from '../lib/pendulum/useCanvasRenderer'
import { pendulumAcceleration, integrateRK4, PID, setPIDParameters } from '../lib/pendulum/pendulumPhysics'
import { drawPendulumChain, drawVerticalReference, clearCanvas, MAIZE } from '../lib/pendulum/drawUtils'
import {
  MASTER_PSEUDOCODE, LINE_SERVO_CHECK, LINE_ERROR, LINE_PID, LINE_ELSE, LINE_ZERO_CONTROL,
  LINE_ACCEL, LINE_INTEGRATE, LINE_ADVANCE_TIME,
} from '../lib/pendulum/pseudocode'

// simulateStep() end to end, exactly as reference/dynamics.js orchestrates
// it: check whether the servo is active, compute PID control torque (or
// zero it out), compute acceleration from the equations of motion, then
// integrate -- one visible phase per pseudocode line this slide's code
// walkthrough covers, single pendulum, RK4, toggleable servo.

const GRAVITY = 9.81, MASS = 2.0, LENGTH = 2.0, DT = 0.02
const servoOn = reactive({ value: true })
const releaseAngle = reactive({ value: Math.PI / 2 })
const desired = -1.0

type Phase = 'servo-check' | 'error' | 'pid' | 'zero-control' | 'accelerate' | 'integrate' | 'advance-time'
interface Entry { frame: number; phase: Phase; t: number; angle: number; control: number }

let angle = [releaseAngle.value], angle_dot = [0], previous_error = [0], accumulated_error = [0], control = [0], t = 0, frame = 0
const servo = setPIDParameters(1)

function resetSim() {
  angle = [releaseAngle.value]; angle_dot = [0]
  previous_error = [0]; accumulated_error = [0]; control = [0]
  t = 0; frame = 0
}

function generateFrame(): Entry[] {
  frame++
  const entries: Entry[] = []
  const snap = (phase: Phase): Entry => ({ frame, phase, t, angle: angle[0], control: control[0] })

  entries.push(snap('servo-check'))
  if (servoOn.value) {
    const error = desired - angle[0]
    entries.push(snap('error'))
    const pid = PID(angle, [desired], previous_error, accumulated_error, DT, servo)
    control = pid.control; previous_error = pid.previous_error; accumulated_error = pid.accumulated_error
    entries.push(snap('pid'))
  } else {
    control = [0]
    entries.push(snap('zero-control'))
  }

  entries.push(snap('accelerate'))
  const result = integrateRK4(angle, angle_dot, (a, w) => pendulumAcceleration(a, w, control, GRAVITY, [MASS], [LENGTH]), DT)
  angle = result.angle; angle_dot = result.angle_dot
  entries.push(snap('integrate'))

  t += DT
  entries.push(snap('advance-time'))

  return entries
}

const tracer = usePhaseTracer<Entry>(generateFrame, resetSim)
const activeLineFor: Record<Phase, number> = {
  'servo-check': LINE_SERVO_CHECK, error: LINE_ERROR, pid: LINE_PID, 'zero-control': LINE_ZERO_CONTROL,
  accelerate: LINE_ACCEL, integrate: LINE_INTEGRATE, 'advance-time': LINE_ADVANCE_TIME,
}
const labelFor: Record<Phase, string> = {
  'servo-check': 'Check servo', error: 'Compute error', pid: 'PID control torque', 'zero-control': 'Zero control (servo off)',
  accelerate: 'Accelerate', integrate: 'Integrate', 'advance-time': 'Advance time',
}

const canvasEl = ref<HTMLCanvasElement | null>(null)

const { redraw } = useCanvasRenderer(canvasEl, (ctx, w, h) => {
  clearCanvas(ctx, w, h)

  const pivotX = w / 2, pivotY = h * 0.18, linkLen = h * 0.6
  drawVerticalReference(ctx, pivotX, pivotY, linkLen)
  const e = tracer.current.value
  if (!e) return
  if (servoOn.value) {
    const dx = pivotX + linkLen * Math.sin(desired), dy = pivotY + linkLen * Math.cos(desired)
    ctx.strokeStyle = '#0d652d'; ctx.setLineDash([3, 4]); ctx.lineWidth = 1.5
    ctx.beginPath(); ctx.moveTo(pivotX, pivotY); ctx.lineTo(dx, dy); ctx.stroke(); ctx.setLineDash([])
  }
  drawPendulumChain(ctx, pivotX, pivotY, [{ angleAbs: e.angle, length: linkLen, color: MAIZE }])

  ctx.font = '12px "Roboto Mono", monospace'
  ctx.fillStyle = '#202124'
  ctx.textAlign = 'left'
  ctx.fillText(`t = ${e.t.toFixed(2)}s   angle = ${e.angle.toFixed(3)}   control = ${e.control.toFixed(1)}`, 12, h - 14)
})

watch(tracer.current, redraw)
function onControlInput() { tracer.reset() }
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
      <label class="check">
        <input type="checkbox" v-model="servoOn.value" @change="onControlInput">
        PID servo active (desired = {{ desired.toFixed(1) }})
      </label>
      <label class="check">
        release angle <input type="range" min="-3.0" max="3.0" step="0.05" v-model.number="releaseAngle.value" @input="onControlInput"> {{ releaseAngle.value.toFixed(2) }} rad
      </label>
    </template>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasEl" />
    </div>
  </PhaseStepperShell>
</template>

<style scoped>
.check { font-family: var(--font-mono, monospace); font-size: 0.72em; display: flex; align-items: center; gap: 0.4em; }
input[type="range"] { width: 7em; }
</style>
