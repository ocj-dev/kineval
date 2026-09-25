<script setup lang="ts">
import { ref, watch } from 'vue'
import PhaseStepperShell from './PhaseStepperShell.vue'
import { usePhaseTracer } from '../lib/pendulum/usePhaseTracer'
import { useCanvasRenderer } from '../lib/pendulum/useCanvasRenderer'
import { pendulumAcceleration } from '../lib/pendulum/pendulumPhysics'
import { drawPendulumChain, drawVerticalReference, clearCanvas, MAIZE } from '../lib/pendulum/drawUtils'
import { MASTER_PSEUDOCODE, LINE_ACCEL, LINE_INTEGRATE } from '../lib/pendulum/pseudocode'

// Velocity Verlet, split into its two real sub-steps (unlike
// pendulumPhysics.ts's single-call integrateVelocityVerlet, which does both
// internally) so "predict" and "correct" are each their own visible phase:
// predict advances POSITION using the OLD acceleration only (angle_dot is
// untouched here); correct then re-evaluates acceleration at the new
// position and averages old+new to advance velocity. Predicting with the
// old acceleration, unlike basic Verlet, does keep an explicit angle_dot the
// whole time -- it's just not updated until the correct phase.

const GRAVITY = 9.81, MASS = 2.0, LENGTH = 2.0, DT = 0.03, RELEASE_ANGLE = Math.PI / 2

interface Entry { frame: number; phase: 'accelerate' | 'predict' | 'correct'; angle: number; angle_dot: number; accel: number }

let angle = [RELEASE_ANGLE], angle_dot = [0], accelOld = [0], frame = 0

function accelFn(a: number[], w: number[]) { return pendulumAcceleration(a, w, [0], GRAVITY, [MASS], [LENGTH]) }

function resetSim() { angle = [RELEASE_ANGLE]; angle_dot = [0]; accelOld = [0]; frame = 0 }

function generateFrame(): Entry[] {
  frame++
  accelOld = accelFn(angle, angle_dot)
  const afterAccel: Entry = { frame, phase: 'accelerate', angle: angle[0], angle_dot: angle_dot[0], accel: accelOld[0] }

  const predictedAngle = angle.map((a, i) => a + angle_dot[i] * DT + 0.5 * accelOld[i] * DT * DT)
  const afterPredict: Entry = { frame, phase: 'predict', angle: predictedAngle[0], angle_dot: angle_dot[0], accel: accelOld[0] }

  const accelNew = accelFn(predictedAngle, angle_dot)
  const correctedVel = angle_dot.map((w, i) => w + 0.5 * (accelOld[i] + accelNew[i]) * DT)
  angle = predictedAngle; angle_dot = correctedVel
  const afterCorrect: Entry = { frame, phase: 'correct', angle: angle[0], angle_dot: angle_dot[0], accel: accelNew[0] }

  return [afterAccel, afterPredict, afterCorrect]
}

const tracer = usePhaseTracer<Entry>(generateFrame, resetSim)
const activeLineFor: Record<Entry['phase'], number> = { accelerate: LINE_ACCEL, predict: LINE_INTEGRATE, correct: LINE_INTEGRATE }
const labelFor: Record<Entry['phase'], string> = { accelerate: 'Accelerate', predict: 'Predict position', correct: 'Correct velocity' }

const canvasEl = ref<HTMLCanvasElement | null>(null)

const { redraw } = useCanvasRenderer(canvasEl, (ctx, w, h) => {
  clearCanvas(ctx, w, h)

  const pivotX = w / 2, pivotY = h * 0.18, linkLen = h * 0.6
  drawVerticalReference(ctx, pivotX, pivotY, linkLen)
  const e = tracer.current.value
  if (!e) return
  drawPendulumChain(ctx, pivotX, pivotY, [{ angleAbs: e.angle, length: linkLen, color: e.phase === 'predict' ? '#9aa0a6' : MAIZE }])

  ctx.font = '12px "Roboto Mono", monospace'
  ctx.fillStyle = '#202124'
  ctx.textAlign = 'left'
  ctx.fillText(`angle = ${e.angle.toFixed(3)}   angle_dot = ${e.angle_dot.toFixed(3)}`, 12, h - 14)
})

watch(tracer.current, redraw)
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
    <div class="vector-canvas-wrap">
      <canvas ref="canvasEl" />
    </div>
  </PhaseStepperShell>
</template>
