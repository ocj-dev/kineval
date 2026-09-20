<script setup lang="ts">
import { ref, onMounted, onBeforeUnmount, watch } from 'vue'
import PhaseStepperShell from './PhaseStepperShell.vue'
import { usePhaseTracer } from '../lib/cloth/usePhaseTracer'
import { MASTER_PSEUDOCODE, LINE_ACCUMULATE, LINE_INTEGRATE, LINE_RELAX, LINE_COLLIDE } from '../lib/cloth/pseudocode'
import {
  makeParticle, accumulateForces, verletIntegrateParticle, satisfyConstraintParticle,
  satisfyCollisionParticle, type ParticleState, type Vec2,
} from '../lib/cloth/clothPhysics'
import { drawArrow, drawParticleDot } from '../lib/cloth/drawUtils'

const props = withDefaults(defineProps<{ collision?: boolean }>(), { collision: false })

const WIDTH = 380
const HEIGHT = props.collision ? 160 : 240
const GRAVITY = 0.5
const FRICTION = 0.98
const REST_LENGTH = 70
const SCALE = 26
const bounds = { minX: 20, maxX: WIDTH - 20, minY: 20, maxY: props.collision ? 90 : HEIGHT - 20 }

type Phase = 'accumulate' | 'integrate' | 'relax' | 'collide'
interface Entry { frame: number; phase: Phase; p1: Vec2; p2: Vec2; vectors: { from: Vec2; to: Vec2; color: string }[] }

let p1: ParticleState = makeParticle(140, 40, true)
let p2: ParticleState = makeParticle(210, 90, false)
let frameCount = 0

function generateFrame(): Entry[] {
  const entries: Entry[] = []

  accumulateForces(p2, { gravityOn: true, gravity: GRAVITY, windOn: false, wind: { x: 0, y: 0 } })
  const forceVec = { from: { x: p2.x, y: p2.y }, to: { x: p2.x, y: p2.y + GRAVITY * SCALE }, color: '#00274C' }
  entries.push({ frame: frameCount, phase: 'accumulate', p1: { ...p1 }, p2: { ...p2 }, vectors: [forceVec] })

  verletIntegrateParticle(p2, FRICTION)
  entries.push({ frame: frameCount, phase: 'integrate', p1: { ...p1 }, p2: { ...p2 }, vectors: [forceVec] })

  const correction = satisfyConstraintParticle(p1, p2, REST_LENGTH, 1.0)
  entries.push({
    frame: frameCount, phase: 'relax', p1: { ...p1 }, p2: { ...p2 },
    vectors: [{ from: { x: p2.x + correction.x, y: p2.y + correction.y }, to: { x: p2.x, y: p2.y }, color: '#FFCB05' }],
  })

  if (props.collision) {
    const response = satisfyCollisionParticle(p2, { ...bounds, groundY: bounds.maxY }, 0.5)
    entries.push({
      frame: frameCount, phase: 'collide', p1: { ...p1 }, p2: { ...p2 },
      vectors: (response.x !== 0 || response.y !== 0)
        ? [{ from: { x: p2.x - response.x, y: p2.y - response.y }, to: { x: p2.x, y: p2.y }, color: '#7d3ac1' }]
        : [],
    })
  }

  frameCount++
  return entries
}

function resetSim() {
  p1 = makeParticle(140, 40, true)
  p2 = makeParticle(210, 90, false)
  frameCount = 0
}

const tracer = usePhaseTracer<Entry>(generateFrame, resetSim)
const { current, trace, pos } = tracer

const activeLineFor: Record<Phase, number> = {
  accumulate: LINE_ACCUMULATE, integrate: LINE_INTEGRATE, relax: LINE_RELAX, collide: LINE_COLLIDE,
}
const labelFor: Record<Phase, string> = {
  accumulate: 'Accumulate forces', integrate: 'Verlet integrate', relax: 'Satisfy constraint (relax)',
  collide: 'Collision detect & respond',
}

const canvasRef = ref<HTMLCanvasElement | null>(null)

function render() {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  ctx.clearRect(0, 0, WIDTH, HEIGHT)

  if (props.collision) {
    ctx.strokeStyle = '#00274C'; ctx.lineWidth = 2
    ctx.beginPath(); ctx.moveTo(0, bounds.maxY); ctx.lineTo(WIDTH, bounds.maxY); ctx.stroke()
  }

  ctx.strokeStyle = '#888'; ctx.lineWidth = 2
  ctx.beginPath(); ctx.moveTo(current.value.p1.x, current.value.p1.y); ctx.lineTo(current.value.p2.x, current.value.p2.y); ctx.stroke()

  ctx.fillStyle = '#00274C'
  ctx.fillRect(current.value.p1.x - 7, current.value.p1.y - 7, 14, 14)
  drawParticleDot(ctx, current.value.p2.x, current.value.p2.y, '#FFCB05', 7)

  for (const v of current.value.vectors) drawArrow(ctx, v.from.x, v.from.y, v.to.x, v.to.y, v.color)
}

let raf = 0
function loop() { render(); raf = requestAnimationFrame(loop) }
onMounted(() => { raf = requestAnimationFrame(loop) })
onBeforeUnmount(() => cancelAnimationFrame(raf))
watch(current, render)
</script>

<template>
  <PhaseStepperShell
    v-model:smooth="tracer.smooth.value"
    :pseudocode-lines="MASTER_PSEUDOCODE"
    :active-line="activeLineFor[current.phase]"
    :is-running="tracer.isRunning.value" :is-done="tracer.isDone.value" :is-at-start="tracer.isAtStart.value"
    :step-label="labelFor[current.phase]"
    :frame-info="`frame ${current.frame} · step ${pos + 1}/${trace.length}`"
    @play="tracer.play" @pause="tracer.pause" @step-forward="tracer.stepForward" @step-back="tracer.stepBack" @reset="tracer.reset"
  >
    <div class="vector-canvas-wrap">
      <canvas ref="canvasRef" :width="WIDTH" :height="HEIGHT" />
    </div>
    <div class="legend">
      <span>anchor = pinned square</span>
      <span v-if="current.phase === 'relax'"><i style="background:#FFCB05" /> constraint correction</span>
      <span v-else-if="current.phase === 'collide'"><i style="background:#7d3ac1" /> collision response</span>
      <span v-else><i style="background:#00274C" /> gravity force</span>
    </div>
  </PhaseStepperShell>
</template>

<style scoped>
.vector-canvas-wrap { flex: 1 1 auto; min-height: 0; border: 1px solid #e3ddc9; border-radius: 10px; background: #fff; }
.vector-canvas-wrap canvas { width: 100%; height: 100%; display: block; }
.legend { display: flex; gap: 1em; font-family: var(--font-mono, monospace); font-size: 0.6em; opacity: 0.75; }
.legend i { display: inline-block; width: 0.8em; height: 0.8em; border-radius: 2px; margin-right: 0.3em; vertical-align: -0.1em; }
</style>
