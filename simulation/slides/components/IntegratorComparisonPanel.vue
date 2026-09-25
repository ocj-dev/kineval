<script setup lang="ts">
import { reactive, ref, watch } from 'vue'
import PhaseStepperShell from './PhaseStepperShell.vue'
import { usePhaseTracer } from '../lib/pendulum/usePhaseTracer'
import { useCanvasRenderer } from '../lib/pendulum/useCanvasRenderer'
import { pendulumAcceleration, integrateEuler, integrateVerlet, integrateVelocityVerlet, integrateRK4, initVerletIntegrator } from '../lib/pendulum/pendulumPhysics'
import { drawTimeSeries } from '../lib/pendulum/drawUtils'
import { MASTER_PSEUDOCODE, LINE_INTEGRATE } from '../lib/pendulum/pseudocode'

// All four integrators, run in parallel on the SAME undriven pendulum,
// plotting total mechanical energy over time. Euler's energy runs away
// almost immediately; Verlet and Velocity Verlet (symplectic methods) hold
// energy in a small bounded band for as long as you let it run; RK4 --
// despite being higher-order and far more accurate PER STEP than either --
// can still show a slow secular energy drift over MANY periods at a coarse
// dt, since it isn't symplectic. Raising dt makes all of this visible much
// faster.

const GRAVITY = 9.81, MASS = 2.0, LENGTH = 2.0, RELEASE_ANGLE = Math.PI / 2
const dtSetting = reactive({ value: 0.05 })

interface Entry { frame: number; phase: 'step'; energies: { euler: number; verlet: number; velVerlet: number; rk4: number } }

function accelFn(a: number[], w: number[]) { return pendulumAcceleration(a, w, [0], GRAVITY, [MASS], [LENGTH]) }
function energyOf(a: number[], w: number[]) {
  return 0.5 * MASS * LENGTH * LENGTH * w[0] * w[0] - MASS * GRAVITY * LENGTH * Math.cos(a[0])
}

let eAngle: number[], eAngleDot: number[]
let vAngle: number[], vAnglePrev: number[]
let vvAngle: number[], vvAngleDot: number[]
let rAngle: number[], rAngleDot: number[]
let frame = 0

const eulerSeries: number[] = [], verletSeries: number[] = [], velVerletSeries: number[] = [], rk4Series: number[] = []

function resetSim() {
  eAngle = [RELEASE_ANGLE]; eAngleDot = [0]
  vAngle = [RELEASE_ANGLE]; vvAngle = [RELEASE_ANGLE]; vvAngleDot = [0]
  rAngle = [RELEASE_ANGLE]; rAngleDot = [0]
  const a0 = accelFn(vAngle, [0])
  vAnglePrev = initVerletIntegrator(vAngle, [0], a0, dtSetting.value)
  frame = 0
  eulerSeries.length = 0; verletSeries.length = 0; velVerletSeries.length = 0; rk4Series.length = 0
}

function generateFrame(): Entry[] {
  frame++
  const dt = dtSetting.value

  const eAccel = accelFn(eAngle, eAngleDot)
  const eResult = integrateEuler(eAngle, eAngleDot, eAccel, dt)
  eAngle = eResult.angle; eAngleDot = eResult.angle_dot

  const vAccel = accelFn(vAngle, [0])
  const vResult = integrateVerlet(vAngle, vAnglePrev, [0], vAccel, accelFn, dt)
  vAnglePrev = vResult.angle_previous!; vAngle = vResult.angle

  const vvAccel = accelFn(vvAngle, vvAngleDot)
  const vvResult = integrateVelocityVerlet(vvAngle, vvAngleDot, vvAccel, accelFn, dt)
  vvAngle = vvResult.angle; vvAngleDot = vvResult.angle_dot

  const rResult = integrateRK4(rAngle, rAngleDot, accelFn, dt)
  rAngle = rResult.angle; rAngleDot = rResult.angle_dot

  const energies = {
    euler: energyOf(eAngle, eAngleDot),
    verlet: energyOf(vAngle, vResult.angle_dot),
    velVerlet: energyOf(vvAngle, vvAngleDot),
    rk4: energyOf(rAngle, rAngleDot),
  }
  eulerSeries.push(energies.euler); verletSeries.push(energies.verlet)
  velVerletSeries.push(energies.velVerlet); rk4Series.push(energies.rk4)

  return [{ frame, phase: 'step', energies }]
}

const tracer = usePhaseTracer<Entry>(generateFrame, resetSim)

const canvasEl = ref<HTMLCanvasElement | null>(null)

const { redraw } = useCanvasRenderer(canvasEl, (ctx, w, h) => {
  const upto = tracer.pos.value + 1
  drawTimeSeries(ctx, w, h, [
    { label: 'Euler', color: '#c0392b', points: eulerSeries.slice(0, upto) },
    { label: 'RK4', color: '#E8710A', points: rk4Series.slice(0, upto) },
    { label: 'Verlet', color: '#1a73e8', points: verletSeries.slice(0, upto) },
    { label: 'Velocity Verlet', color: '#0d652d', points: velVerletSeries.slice(0, upto) },
  ], { yLabel: 'total energy (J)' })
})

watch(tracer.current, redraw)
function onDtInput() { tracer.reset() }
</script>

<template>
  <PhaseStepperShell
    :pseudocode-lines="MASTER_PSEUDOCODE"
    :active-line="LINE_INTEGRATE"
    :is-running="tracer.isRunning.value" :is-done="tracer.isDone.value" :is-at-start="tracer.isAtStart.value"
    v-model:smooth="tracer.smooth.value"
    step-label="Integrate (all four, in parallel)"
    :frame-info="`frame ${tracer.current.value?.frame ?? 0}`"
    @play="tracer.play" @pause="tracer.pause" @step-forward="tracer.stepForward" @step-back="tracer.stepBack" @reset="tracer.reset"
  >
    <template #controls>
      <label class="check">
        dt <input type="range" min="0.01" max="0.2" step="0.01" v-model.number="dtSetting.value" @input="onDtInput"> {{ dtSetting.value.toFixed(2) }}s
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
