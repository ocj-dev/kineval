<script setup lang="ts">
import { reactive, ref, onMounted, watch } from 'vue'
import PhaseStepperShell from './PhaseStepperShell.vue'
import { usePhaseTracer } from '../lib/pendulum/usePhaseTracer'
import { pendulumAcceleration, integrateVelocityVerlet } from '../lib/pendulum/pendulumPhysics'
import { drawPendulumChain, drawVerticalReference, drawArrow, clearCanvas } from '../lib/pendulum/drawUtils'
import { MASTER_PSEUDOCODE, LINE_ACCEL, LINE_INTEGRATE } from '../lib/pendulum/pseudocode'

// A slider-driven, undriven single pendulum (no PID, gravity only) --
// released from whatever initial angle the slider sets. The point of this
// panel isn't the integrator (Velocity Verlet, fixed) but the equation of
// motion itself: line 7's theta_dot_dot = -(g/l)*sin(theta) computed fresh
// every accelerate phase, read out numerically alongside the gravity-torque
// arrow.

const GRAVITY = 9.81, MASS = 2.0, LENGTH = 2.0, DT = 0.02
const initialAngle = reactive({ value: Math.PI / 2 })

interface Entry { frame: number; phase: 'accelerate' | 'integrate'; angle: number; angle_dot: number; accel: number }

let angle = [initialAngle.value], angle_dot = [0], frame = 0

function resetSim() {
  angle = [initialAngle.value]
  angle_dot = [0]
  frame = 0
}

function generateFrame(): Entry[] {
  frame++
  const accel = pendulumAcceleration(angle, angle_dot, [0], GRAVITY, [MASS], [LENGTH])
  const afterAccel: Entry = { frame, phase: 'accelerate', angle: angle[0], angle_dot: angle_dot[0], accel: accel[0] }
  const result = integrateVelocityVerlet(angle, angle_dot, accel, (a, w) => pendulumAcceleration(a, w, [0], GRAVITY, [MASS], [LENGTH]), DT)
  angle = result.angle; angle_dot = result.angle_dot
  const afterIntegrate: Entry = { frame, phase: 'integrate', angle: angle[0], angle_dot: angle_dot[0], accel: accel[0] }
  return [afterAccel, afterIntegrate]
}

const tracer = usePhaseTracer<Entry>(generateFrame, resetSim)

const activeLineFor: Record<Entry['phase'], number> = { accelerate: LINE_ACCEL, integrate: LINE_INTEGRATE }
const labelFor: Record<Entry['phase'], string> = { accelerate: 'Accelerate', integrate: 'Integrate' }

const canvasEl = ref<HTMLCanvasElement | null>(null)
let ctx: CanvasRenderingContext2D | null = null

function render() {
  const canvas = canvasEl.value
  if (!ctx || !canvas) return
  const w = canvas.clientWidth, h = canvas.clientHeight
  if (canvas.width !== w) canvas.width = w
  if (canvas.height !== h) canvas.height = h
  clearCanvas(ctx, w, h)

  const pivotX = w / 2, pivotY = h * 0.18, linkLen = h * 0.62
  drawVerticalReference(ctx, pivotX, pivotY, linkLen)
  const e = tracer.current.value
  if (!e) return
  drawPendulumChain(ctx, pivotX, pivotY, [{ angleAbs: e.angle, length: linkLen, color: '#B3261E' }])

  const tipX = pivotX + linkLen * Math.sin(e.angle), tipY = pivotY + linkLen * Math.cos(e.angle)
  const gravityTorque = -MASS * GRAVITY * LENGTH * Math.sin(e.angle)
  drawArrow(ctx, tipX, tipY, Math.sign(gravityTorque) * 40, 0, '#1a73e8')

  ctx.font = '13px "Roboto Mono", monospace'
  ctx.fillStyle = '#202124'
  ctx.textAlign = 'left'
  ctx.fillText(`theta = ${e.angle.toFixed(3)} rad`, 12, h - 46)
  ctx.fillText(`theta_dot = ${e.angle_dot.toFixed(3)} rad/s`, 12, h - 30)
  ctx.fillText(`theta_dot_dot = -(g/l)*sin(theta) = ${e.accel.toFixed(3)} rad/s^2`, 12, h - 14)
}

watch(tracer.current, render)
onMounted(() => {
  ctx = canvasEl.value!.getContext('2d')
  render()
})

function onSliderInput() { tracer.reset() }
</script>

<template>
  <PhaseStepperShell
    :pseudocode-lines="MASTER_PSEUDOCODE"
    :active-line="tracer.current.value ? activeLineFor[tracer.current.value.phase] : -1"
    :is-running="tracer.isRunning.value" :is-done="tracer.isDone.value" :is-at-start="tracer.isAtStart.value"
    v-model:smooth="tracer.smooth.value"
    :step-label="tracer.current.value ? labelFor[tracer.current.value.phase] : ''"
    :frame-info="`frame ${tracer.current.value?.frame ?? 0} &middot; step ${tracer.pos.value + 1}/${tracer.trace.length}`"
    @play="tracer.play" @pause="tracer.pause" @step-forward="tracer.stepForward" @step-back="tracer.stepBack" @reset="tracer.reset"
  >
    <template #controls>
      <label class="check">
        release angle
        <input type="range" min="-3.0" max="3.0" step="0.05" v-model.number="initialAngle.value" @input="onSliderInput">
        {{ initialAngle.value.toFixed(2) }} rad
      </label>
    </template>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasEl" />
    </div>
  </PhaseStepperShell>
</template>

<style scoped>
.check { font-family: var(--font-mono, monospace); font-size: 0.72em; display: flex; align-items: center; gap: 0.5em; }
input[type="range"] { width: 8em; }
</style>
