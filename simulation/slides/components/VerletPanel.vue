<script setup lang="ts">
import { reactive, ref, watch } from 'vue'
import PhaseStepperShell from './PhaseStepperShell.vue'
import { usePhaseTracer } from '../lib/pendulum/usePhaseTracer'
import { useCanvasRenderer } from '../lib/pendulum/useCanvasRenderer'
import { pendulumAcceleration, initVerletIntegrator, integrateVerlet } from '../lib/pendulum/pendulumPhysics'
import { drawPendulumChain, drawVerticalReference, clearCanvas, MAIZE } from '../lib/pendulum/drawUtils'
import { MASTER_PSEUDOCODE, LINE_ACCEL, LINE_INTEGRATE } from '../lib/pendulum/pseudocode'

// Basic (position-only) Verlet: the position update never reads angle_dot,
// only the current and PREVIOUS angle plus the current acceleration -- the
// three-point readout below makes that explicit. The one-time init step
// (bootstrapping angle_previous from a Taylor expansion, since there's no
// real "previous frame" at t=0) runs inside resetSim(), not as a traced
// phase, matching the pseudocode note that it's setup, not part of the loop.

const GRAVITY = 9.81, MASS = 2.0, LENGTH = 2.0, DT = 0.03
const releaseAngle = reactive({ value: Math.PI / 2 })

interface Entry { frame: number; phase: 'accelerate' | 'integrate'; angle: number; angle_previous: number; angle_dot: number; accel: number }

let angle = [releaseAngle.value], angle_previous = [releaseAngle.value], angle_dot = [0], accel = [0], frame = 0

function accelFn(a: number[], w: number[]) { return pendulumAcceleration(a, w, [0], GRAVITY, [MASS], [LENGTH]) }

function resetSim() {
  angle = [releaseAngle.value]; angle_dot = [0]
  accel = accelFn(angle, angle_dot)
  angle_previous = initVerletIntegrator(angle, angle_dot, accel, DT)   // one-time setup, not a traced phase
  frame = 0
}

function generateFrame(): Entry[] {
  frame++
  accel = accelFn(angle, angle_dot)
  const afterAccel: Entry = { frame, phase: 'accelerate', angle: angle[0], angle_previous: angle_previous[0], angle_dot: angle_dot[0], accel: accel[0] }

  const result = integrateVerlet(angle, angle_previous, angle_dot, accel, accelFn, DT)
  angle_previous = result.angle_previous!; angle = result.angle; angle_dot = result.angle_dot

  const afterIntegrate: Entry = { frame, phase: 'integrate', angle: angle[0], angle_previous: angle_previous[0], angle_dot: angle_dot[0], accel: accel[0] }
  return [afterAccel, afterIntegrate]
}

const tracer = usePhaseTracer<Entry>(generateFrame, resetSim)
const activeLineFor: Record<Entry['phase'], number> = { accelerate: LINE_ACCEL, integrate: LINE_INTEGRATE }
const labelFor: Record<Entry['phase'], string> = { accelerate: 'Accelerate', integrate: 'Verlet integrate' }

const canvasEl = ref<HTMLCanvasElement | null>(null)

const { redraw } = useCanvasRenderer(canvasEl, (ctx, w, h) => {
  clearCanvas(ctx, w, h)

  const pivotX = w / 2, pivotY = h * 0.18, linkLen = h * 0.6
  drawVerticalReference(ctx, pivotX, pivotY, linkLen)
  const e = tracer.current.value
  if (!e) return
  drawPendulumChain(ctx, pivotX, pivotY, [{ angleAbs: e.angle_previous, length: linkLen, color: '#9aa0a660' }], 7)
  drawPendulumChain(ctx, pivotX, pivotY, [{ angleAbs: e.angle, length: linkLen, color: MAIZE }])

  ctx.font = '12px "Roboto Mono", monospace'
  ctx.fillStyle = '#202124'
  ctx.textAlign = 'left'
  ctx.fillText(`angle_previous = ${e.angle_previous.toFixed(3)}  (faint arm)`, 12, h - 46)
  ctx.fillText(`angle          = ${e.angle.toFixed(3)}`, 12, h - 30)
  ctx.fillText(`angle_next = 2*angle - angle_previous + accel*dt^2`, 12, h - 14)
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
