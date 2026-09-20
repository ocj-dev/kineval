<script setup lang="ts">
import { ref, onMounted, onBeforeUnmount, watch } from 'vue'
import PhaseStepperShell from './PhaseStepperShell.vue'
import JoystickControl from './JoystickControl.vue'
import { usePhaseTracer } from '../lib/cloth/usePhaseTracer'
import { makeParticle, accumulateForces, verletIntegrateParticle, satisfyCollisionParticle, type ParticleState, type Vec2 } from '../lib/cloth/clothPhysics'
import { drawArrow, drawParticleDot, VECTOR_COLORS } from '../lib/cloth/drawUtils'

const WIDTH = 380, HEIGHT = 240
const GRAVITY = 0.6
const FRICTION = 0.995
const SCALE = 20

const PSEUDOCODE = [
  'every frame:',
  '    for each node',
  '        accumulate forces (gravity, wind)',
  '        Verlet-integrate the node\'s position',
]
const LINE_ACCUMULATE = 2, LINE_INTEGRATE = 3

interface Entry { frame: number; phase: 'accumulate' | 'integrate'; p: Vec2; vectors: { from: Vec2; to: Vec2; color: string }[] }

const gravityOn = ref(true)
const windOn = ref(false)
const wind = ref({ x: 0, y: 0 })

let p: ParticleState = makeParticle(WIDTH / 2, 40)
let frameCount = 0

function generateFrame(): Entry[] {
  accumulateForces(p, { gravityOn: gravityOn.value, gravity: GRAVITY, windOn: windOn.value, wind: wind.value })
  const gvx = 0, gvy = gravityOn.value ? GRAVITY * SCALE : 0
  const wvx = windOn.value ? wind.value.x * SCALE : 0
  const wvy = windOn.value ? wind.value.y * SCALE : 0
  const vectors: Entry['vectors'] = []
  if (gravityOn.value) vectors.push({ from: { x: p.x, y: p.y }, to: { x: p.x + gvx, y: p.y + gvy }, color: VECTOR_COLORS.gravity })
  if (windOn.value) vectors.push({ from: { x: p.x, y: p.y }, to: { x: p.x + wvx, y: p.y + wvy }, color: VECTOR_COLORS.wind })
  if (gravityOn.value && windOn.value)
    vectors.push({ from: { x: p.x, y: p.y }, to: { x: p.x + gvx + wvx, y: p.y + gvy + wvy }, color: VECTOR_COLORS.resultant })

  const accEntry: Entry = { frame: frameCount, phase: 'accumulate', p: { x: p.x, y: p.y }, vectors }

  verletIntegrateParticle(p, FRICTION)
  satisfyCollisionParticle(p, { minX: 10, maxX: WIDTH - 10, minY: 10, maxY: HEIGHT - 10 }, 0.5)
  const intEntry: Entry = { frame: frameCount, phase: 'integrate', p: { x: p.x, y: p.y }, vectors }

  frameCount++
  return [accEntry, intEntry]
}

function resetSim() { p = makeParticle(WIDTH / 2, 40); frameCount = 0 }

const tracer = usePhaseTracer<Entry>(generateFrame, resetSim)
const { current, trace, pos } = tracer

const canvasRef = ref<HTMLCanvasElement | null>(null)

function render() {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  ctx.clearRect(0, 0, WIDTH, HEIGHT)
  for (const v of current.value.vectors) drawArrow(ctx, v.from.x, v.from.y, v.to.x, v.to.y, v.color)
  drawParticleDot(ctx, current.value.p.x, current.value.p.y, '#00274C', 7)
}

// re-render on every navigation step, AND continuously while playing in
// smooth mode (so dragging the joystick or toggling checkboxes updates the
// picture immediately even between generated frames)
let raf = 0
function loop() { render(); raf = requestAnimationFrame(loop) }
onMounted(() => { raf = requestAnimationFrame(loop) })
onBeforeUnmount(() => cancelAnimationFrame(raf))
watch(current, render)
</script>

<template>
  <PhaseStepperShell
    v-model:smooth="tracer.smooth.value"
    :pseudocode-lines="PSEUDOCODE"
    :active-line="current.phase === 'accumulate' ? LINE_ACCUMULATE : LINE_INTEGRATE"
    :is-running="tracer.isRunning.value" :is-done="tracer.isDone.value" :is-at-start="tracer.isAtStart.value"
    :step-label="current.phase === 'accumulate' ? 'Accumulate forces' : 'Verlet integrate'"
    :frame-info="`frame ${current.frame} · step ${pos + 1}/${trace.length}`"
    @play="tracer.play" @pause="tracer.pause" @step-forward="tracer.stepForward" @step-back="tracer.stepBack" @reset="tracer.reset"
  >
    <template #controls>
      <label class="check"><input v-model="gravityOn" type="checkbox"> Gravity</label>
      <label class="check"><input v-model="windOn" type="checkbox"> Wind</label>
      <JoystickControl v-model="wind" :max-magnitude="1.2" label="Wind (mag + dir)" />
    </template>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasRef" :width="WIDTH" :height="HEIGHT" />
    </div>
  </PhaseStepperShell>
</template>

<style scoped>
.check { font-family: var(--font-mono, monospace); font-size: 0.72em; display: flex; align-items: center; gap: 0.3em; cursor: pointer; }
.vector-canvas-wrap { flex: 1 1 auto; min-height: 0; border: 1px solid #e3ddc9; border-radius: 10px; background: #fff; }
.vector-canvas-wrap canvas { width: 100%; height: 100%; display: block; }
</style>
