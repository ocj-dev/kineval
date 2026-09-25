<script setup lang="ts">
import { reactive, ref, watch } from 'vue'
import PhaseStepperShell from './PhaseStepperShell.vue'
import { usePhaseTracer } from '../lib/pendulum/usePhaseTracer'
import { useCanvasRenderer } from '../lib/pendulum/useCanvasRenderer'
import { pendulumAcceleration } from '../lib/pendulum/pendulumPhysics'
import { drawPendulumChain, drawVerticalReference, clearCanvas, MAIZE } from '../lib/pendulum/drawUtils'
import { MASTER_PSEUDOCODE, LINE_ACCEL, LINE_INTEGRATE } from '../lib/pendulum/pseudocode'

// RK4's four stages, each its own phase so the probe angle each stage
// evaluates acceleration at is visible as a ghost arm before the final
// weighted combine -- k_x{1..4} are the velocity-stage estimates that
// advance angle, k_v{1..4} the acceleration-stage estimates that advance
// angle_dot, matching the AutoRob dynamics lecture's own notation.

const GRAVITY = 9.81, MASS = 2.0, LENGTH = 2.0, DT = 0.06
const releaseAngle = reactive({ value: Math.PI / 2 })

type Phase = 'k1' | 'k2' | 'k3' | 'k4' | 'combine'
interface Entry { frame: number; phase: Phase; angle: number; probeAngle: number; angle_dot: number }

let angle = [releaseAngle.value], angle_dot = [0], frame = 0

function accelFn(a: number[], w: number[]) { return pendulumAcceleration(a, w, [0], GRAVITY, [MASS], [LENGTH]) }
function resetSim() { angle = [releaseAngle.value]; angle_dot = [0]; frame = 0 }

function generateFrame(): Entry[] {
  frame++
  const n = angle.length, dt = DT

  const k_v1 = accelFn(angle, angle_dot), k_x1 = angle_dot
  const e1: Entry = { frame, phase: 'k1', angle: angle[0], probeAngle: angle[0], angle_dot: angle_dot[0] }

  const a2: number[] = [], v2: number[] = []
  for (let i = 0; i < n; i++) { a2[i] = angle[i] + 0.5 * dt * k_x1[i]; v2[i] = angle_dot[i] + 0.5 * dt * k_v1[i] }
  const k_v2 = accelFn(a2, v2), k_x2 = v2
  const e2: Entry = { frame, phase: 'k2', angle: angle[0], probeAngle: a2[0], angle_dot: angle_dot[0] }

  const a3: number[] = [], v3: number[] = []
  for (let i = 0; i < n; i++) { a3[i] = angle[i] + 0.5 * dt * k_x2[i]; v3[i] = angle_dot[i] + 0.5 * dt * k_v2[i] }
  const k_v3 = accelFn(a3, v3), k_x3 = v3
  const e3: Entry = { frame, phase: 'k3', angle: angle[0], probeAngle: a3[0], angle_dot: angle_dot[0] }

  const a4: number[] = [], v4: number[] = []
  for (let i = 0; i < n; i++) { a4[i] = angle[i] + dt * k_x3[i]; v4[i] = angle_dot[i] + dt * k_v3[i] }
  const k_v4 = accelFn(a4, v4), k_x4 = v4
  const e4: Entry = { frame, phase: 'k4', angle: angle[0], probeAngle: a4[0], angle_dot: angle_dot[0] }

  const next_angle: number[] = [], next_angle_dot: number[] = []
  for (let i = 0; i < n; i++) {
    next_angle[i] = angle[i] + (dt / 6) * (k_x1[i] + 2 * k_x2[i] + 2 * k_x3[i] + k_x4[i])
    next_angle_dot[i] = angle_dot[i] + (dt / 6) * (k_v1[i] + 2 * k_v2[i] + 2 * k_v3[i] + k_v4[i])
  }
  angle = next_angle; angle_dot = next_angle_dot
  const e5: Entry = { frame, phase: 'combine', angle: angle[0], probeAngle: angle[0], angle_dot: angle_dot[0] }

  return [e1, e2, e3, e4, e5]
}

const tracer = usePhaseTracer<Entry>(generateFrame, resetSim)
const activeLineFor: Record<Phase, number> = { k1: LINE_ACCEL, k2: LINE_ACCEL, k3: LINE_ACCEL, k4: LINE_ACCEL, combine: LINE_INTEGRATE }
const labelFor: Record<Phase, string> = { k1: 'Stage k1 (start)', k2: 'Stage k2 (midpoint)', k3: 'Stage k3 (midpoint)', k4: 'Stage k4 (endpoint)', combine: 'Combine (Simpson weights 1:2:2:1)' }

const canvasEl = ref<HTMLCanvasElement | null>(null)

const { redraw } = useCanvasRenderer(canvasEl, (ctx, w, h) => {
  clearCanvas(ctx, w, h)

  const pivotX = w / 2, pivotY = h * 0.18, linkLen = h * 0.6
  drawVerticalReference(ctx, pivotX, pivotY, linkLen)
  const e = tracer.current.value
  if (!e) return
  if (e.phase !== 'combine') drawPendulumChain(ctx, pivotX, pivotY, [{ angleAbs: e.probeAngle, length: linkLen, color: '#9aa0a660' }], 8)
  drawPendulumChain(ctx, pivotX, pivotY, [{ angleAbs: e.angle, length: linkLen, color: MAIZE }])

  ctx.font = '12px "Roboto Mono", monospace'
  ctx.fillStyle = '#202124'
  ctx.textAlign = 'left'
  ctx.fillText(`probe angle = ${e.probeAngle.toFixed(3)}${e.phase === 'combine' ? ' (final)' : ' (faint arm)'}`, 12, h - 14)
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
    :frame-info="`step ${tracer.pos.value + 1}/${tracer.trace.length}`"
    @play="tracer.play" @pause="tracer.pause" @step-forward="tracer.stepForward" @step-back="tracer.stepBack" @reset="tracer.reset"
  >
    <template #controls>
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
.check { font-family: var(--font-mono, monospace); font-size: 0.72em; display: flex; align-items: center; gap: 0.5em; }
input[type="range"] { width: 7em; }
</style>
