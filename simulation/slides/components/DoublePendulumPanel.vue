<script setup lang="ts">
import { ref, onMounted, watch } from 'vue'
import PhaseStepperShell from './PhaseStepperShell.vue'
import { usePhaseTracer } from '../lib/pendulum/usePhaseTracer'
import { doublePendulumAcceleration, integrateRK4 } from '../lib/pendulum/pendulumPhysics'
import { drawPendulumChain, drawVerticalReference, clearCanvas } from '../lib/pendulum/drawUtils'
import { MASTER_PSEUDOCODE, LINE_ACCEL, LINE_INTEGRATE } from '../lib/pendulum/pseudocode'

// The coupled double-pendulum equations of motion, run TWICE from almost
// (but not quite) the same initial angles -- 0.001 rad apart on joint 2 --
// both undriven, both integrated with RK4. Sensitive dependence on initial
// conditions is the double pendulum's defining feature: watch the two
// traces track each other closely at first, then visibly diverge within a
// few swings.

const GRAVITY = 9.81, MASS = [2.0, 2.0], LENGTH = [2.0, 2.0], DT = 0.02
const START_A = [1.55, 1.55], START_B = [1.55, 1.551]

interface Entry { frame: number; phase: 'accelerate' | 'integrate'; angleA: number[]; angleB: number[] }

let angleA = START_A.slice(), angleDotA = [0, 0]
let angleB = START_B.slice(), angleDotB = [0, 0]
let frame = 0

function accelFn(a: number[], w: number[]) { return doublePendulumAcceleration(a, w, [0, 0], GRAVITY, MASS, LENGTH) }

function resetSim() {
  angleA = START_A.slice(); angleDotA = [0, 0]
  angleB = START_B.slice(); angleDotB = [0, 0]
  frame = 0
}

function generateFrame(): Entry[] {
  frame++
  const e1: Entry = { frame, phase: 'accelerate', angleA: angleA.slice(), angleB: angleB.slice() }

  const resultA = integrateRK4(angleA, angleDotA, accelFn, DT)
  angleA = resultA.angle; angleDotA = resultA.angle_dot
  const resultB = integrateRK4(angleB, angleDotB, accelFn, DT)
  angleB = resultB.angle; angleDotB = resultB.angle_dot

  const e2: Entry = { frame, phase: 'integrate', angleA: angleA.slice(), angleB: angleB.slice() }
  return [e1, e2]
}

const tracer = usePhaseTracer<Entry>(generateFrame, resetSim)
const activeLineFor: Record<Entry['phase'], number> = { accelerate: LINE_ACCEL, integrate: LINE_INTEGRATE }
const labelFor: Record<Entry['phase'], string> = { accelerate: 'Accelerate (both copies)', integrate: 'Integrate (both copies)' }

const canvasEl = ref<HTMLCanvasElement | null>(null)
let ctx: CanvasRenderingContext2D | null = null

function render() {
  const canvas = canvasEl.value
  if (!ctx || !canvas) return
  const w = canvas.clientWidth, h = canvas.clientHeight
  if (canvas.width !== w) canvas.width = w
  if (canvas.height !== h) canvas.height = h
  clearCanvas(ctx, w, h)

  const pivotX = w / 2, pivotY = h * 0.12, linkLen = h * 0.36
  drawVerticalReference(ctx, pivotX, pivotY, linkLen * 2)
  const e = tracer.current.value
  if (!e) return

  ctx.globalAlpha = 0.55
  drawPendulumChain(ctx, pivotX, pivotY, [
    { angleAbs: e.angleB[0], length: linkLen, color: '#1a73e880' },
    { angleAbs: e.angleB[1], length: linkLen, color: '#1a73e8' },
  ], 9)
  ctx.globalAlpha = 1
  drawPendulumChain(ctx, pivotX, pivotY, [
    { angleAbs: e.angleA[0], length: linkLen, color: '#E8710A80' },
    { angleAbs: e.angleA[1], length: linkLen, color: '#E8710A' },
  ], 9)

  ctx.font = '11px "Roboto Mono", monospace'
  ctx.fillStyle = '#202124'
  ctx.textAlign = 'left'
  const divergence = Math.hypot(e.angleA[0] - e.angleB[0], e.angleA[1] - e.angleB[1])
  ctx.fillText(`start: theta2 differs by 0.001 rad  |  now differs by ${divergence.toFixed(4)} rad`, 12, h - 14)
}

watch(tracer.current, render)
onMounted(() => { ctx = canvasEl.value!.getContext('2d'); render() })
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
      <span class="legend"><span class="dot orange" /> theta2(0)=1.550 &nbsp; <span class="dot blue" /> theta2(0)=1.551</span>
    </template>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasEl" />
    </div>
  </PhaseStepperShell>
</template>

<style scoped>
.legend { font-family: var(--font-mono, monospace); font-size: 0.68em; }
.dot { display: inline-block; width: 0.7em; height: 0.7em; border-radius: 50%; }
.dot.orange { background: #E8710A; }
.dot.blue { background: #1a73e8; }
</style>
