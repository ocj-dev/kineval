<script setup lang="ts">
import { reactive, ref, watch } from 'vue'
import PhaseStepperShell from './PhaseStepperShell.vue'
import { usePhaseTracer } from '../lib/pendulum/usePhaseTracer'
import { useCanvasRenderer } from '../lib/pendulum/useCanvasRenderer'
import { pendulumAcceleration, integrateEuler, integrateRK4 } from '../lib/pendulum/pendulumPhysics'
import { drawPendulumChain, drawVerticalReference, clearCanvas, MAIZE } from '../lib/pendulum/drawUtils'
import { MASTER_PSEUDOCODE, LINE_ACCEL, LINE_INTEGRATE } from '../lib/pendulum/pseudocode'

// Explicit (forward) Euler against an RK4 "ghost" reference at the SAME dt,
// released from the same initial angle -- both undriven, gravity only.
// Dragging dt up makes Euler's energy gain visibly overtake RK4's within a
// few swings, the same "naive integrator is unstable at large dt" lesson
// the AutoRob dynamics lecture opens with.

const GRAVITY = 9.81, MASS = 2.0, LENGTH = 2.0, RELEASE_ANGLE = Math.PI / 2
const dtSetting = reactive({ value: 0.05 })

interface Entry { frame: number; phase: 'accelerate' | 'integrate'; angle: number; angle_dot: number; ghostAngle: number }

let angle = [RELEASE_ANGLE], angle_dot = [0], accel = [0]
let ghostAngle = [RELEASE_ANGLE], ghostAngleDot = [0]
let frame = 0

function accelFn(a: number[], w: number[]) { return pendulumAcceleration(a, w, [0], GRAVITY, [MASS], [LENGTH]) }

function resetSim() {
  angle = [RELEASE_ANGLE]; angle_dot = [0]; accel = [0]
  ghostAngle = [RELEASE_ANGLE]; ghostAngleDot = [0]
  frame = 0
}

function generateFrame(): Entry[] {
  frame++
  const dt = dtSetting.value
  accel = accelFn(angle, angle_dot)
  const afterAccel: Entry = { frame, phase: 'accelerate', angle: angle[0], angle_dot: angle_dot[0], ghostAngle: ghostAngle[0] }

  const result = integrateEuler(angle, angle_dot, accel, dt)
  angle = result.angle; angle_dot = result.angle_dot
  const ghost = integrateRK4(ghostAngle, ghostAngleDot, accelFn, dt)
  ghostAngle = ghost.angle; ghostAngleDot = ghost.angle_dot

  const afterIntegrate: Entry = { frame, phase: 'integrate', angle: angle[0], angle_dot: angle_dot[0], ghostAngle: ghostAngle[0] }
  return [afterAccel, afterIntegrate]
}

const tracer = usePhaseTracer<Entry>(generateFrame, resetSim)
const activeLineFor: Record<Entry['phase'], number> = { accelerate: LINE_ACCEL, integrate: LINE_INTEGRATE }
const labelFor: Record<Entry['phase'], string> = { accelerate: 'Accelerate', integrate: 'Euler integrate' }

const canvasEl = ref<HTMLCanvasElement | null>(null)

const { redraw } = useCanvasRenderer(canvasEl, (ctx, w, h) => {
  clearCanvas(ctx, w, h)

  const pivotX = w / 2, pivotY = h * 0.18, linkLen = h * 0.6
  drawVerticalReference(ctx, pivotX, pivotY, linkLen)
  const e = tracer.current.value
  if (!e) return
  drawPendulumChain(ctx, pivotX, pivotY, [{ angleAbs: e.ghostAngle, length: linkLen, color: '#9aa0a680' }], 8)
  drawPendulumChain(ctx, pivotX, pivotY, [{ angleAbs: e.angle, length: linkLen, color: MAIZE }])

  ctx.font = '12px "Roboto Mono", monospace'
  ctx.fillStyle = '#202124'
  ctx.textAlign = 'left'
  ctx.fillText(`dt = ${dtSetting.value.toFixed(3)} s`, 12, h - 46)
  ctx.fillText(`Euler theta = ${e.angle.toFixed(3)}`, 12, h - 30)
  ctx.fillText(`RK4 (ghost) theta = ${e.ghostAngle.toFixed(3)}`, 12, h - 14)
})

watch(tracer.current, redraw)
function onDtInput() { tracer.reset() }
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
        dt <input type="range" min="0.01" max="0.35" step="0.01" v-model.number="dtSetting.value" @input="onDtInput"> {{ dtSetting.value.toFixed(2) }}s
      </label>
      <span class="legend"><span class="dot maize" /> Euler &nbsp; <span class="dot ghost" /> RK4 (reference)</span>
    </template>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasEl" />
    </div>
  </PhaseStepperShell>
</template>

<style scoped>
.check { font-family: var(--font-mono, monospace); font-size: 0.72em; display: flex; align-items: center; gap: 0.5em; }
input[type="range"] { width: 7em; }
.legend { font-family: var(--font-mono, monospace); font-size: 0.68em; }
.dot { display: inline-block; width: 0.7em; height: 0.7em; border-radius: 50%; }
.dot.maize { background: #FFCB05; }
.dot.ghost { background: #9aa0a6; }
</style>
