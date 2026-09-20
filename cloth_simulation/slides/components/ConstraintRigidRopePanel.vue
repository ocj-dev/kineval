<script setup lang="ts">
import { ref, onMounted, onBeforeUnmount, watch } from 'vue'
import PhaseStepperShell from './PhaseStepperShell.vue'
import { usePhaseTracer } from '../lib/cloth/usePhaseTracer'
import { MASTER_PSEUDOCODE, LINE_ACCUMULATE, LINE_INTEGRATE, LINE_RELAX } from '../lib/cloth/pseudocode'
import { makeRigid, verletIntegrateRigid, satisfyConstraintRigid, worldCorner, type RigidState, type Vec2 } from '../lib/cloth/clothPhysics'
import { drawArrow, drawRigidSquare } from '../lib/cloth/drawUtils'

const WIDTH = 380, HEIGHT = 260
const GRAVITY = 0.5
const FRICTION = 0.975
// same non-grid exception as ConstraintRigidPanel
const HALF = 26
const SIDE = 2 * HALF
const REST_LENGTH = 2 * SIDE   // rest length = 2x the square's side length, per spec

type Phase = 'accumulate' | 'integrate' | 'relax'
interface Entry { frame: number; phase: Phase; bodyA: RigidState; bodyB: RigidState; vectors: { from: Vec2; to: Vec2; color: string }[] }

function initial() {
  const a = makeRigid(120, 50, HALF, true)
  const b = makeRigid(230, 150, HALF, false)
  b.theta = 0.2
  return { a, b }
}
let { a: bodyA, b: bodyB } = initial()
let frameCount = 0

function generateFrame(): Entry[] {
  const entries: Entry[] = []

  bodyB.force_x = 0; bodyB.force_y = GRAVITY * bodyB.mass; bodyB.torque = 0
  const forceVec = { from: { x: bodyB.x, y: bodyB.y }, to: { x: bodyB.x, y: bodyB.y + GRAVITY * 24 }, color: '#00274C' }
  entries.push({ frame: frameCount, phase: 'accumulate', bodyA: { ...bodyA }, bodyB: { ...bodyB }, vectors: [forceVec] })

  verletIntegrateRigid(bodyB, FRICTION)
  entries.push({ frame: frameCount, phase: 'integrate', bodyA: { ...bodyA }, bodyB: { ...bodyB }, vectors: [forceVec] })

  // a SINGLE constraint (not 2) between the bottom-right corners (index 2),
  // with a nonzero rest length -- so unlike the shared-edge rigid weld, both
  // squares remain free to rotate about this one connection point
  const before = worldCorner(bodyA, 2)
  const c = satisfyConstraintRigid(bodyA, 2, bodyB, 2, 1.0, REST_LENGTH)
  entries.push({
    frame: frameCount, phase: 'relax', bodyA: { ...bodyA }, bodyB: { ...bodyB },
    vectors: [{ from: before, to: { x: before.x + c.x, y: before.y + c.y }, color: '#FFCB05' }],
  })

  frameCount++
  return entries
}

function resetSim() { ({ a: bodyA, b: bodyB } = initial()); frameCount = 0 }

const tracer = usePhaseTracer<Entry>(generateFrame, resetSim)
const { current, trace, pos } = tracer

const activeLineFor: Record<Phase, number> = { accumulate: LINE_ACCUMULATE, integrate: LINE_INTEGRATE, relax: LINE_RELAX }
const labelFor: Record<Phase, string> = {
  accumulate: 'Accumulate forces', integrate: 'Verlet integrate (pos + orientation)', relax: 'Satisfy 1 corner constraint (relax)',
}

const canvasRef = ref<HTMLCanvasElement | null>(null)

function render() {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  ctx.clearRect(0, 0, WIDTH, HEIGHT)

  const cornersA = [0, 1, 2, 3].map(i => worldCorner(current.value.bodyA, i))
  const cornersB = [0, 1, 2, 3].map(i => worldCorner(current.value.bodyB, i))
  drawRigidSquare(ctx, cornersA, '#00274Ccc')
  drawRigidSquare(ctx, cornersB, '#FFCB05cc')
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
      <span><i style="background:#00274C" /> anchor square</span>
      <span>single constraint, bottom-right corners, rest length = 2&times; side</span>
    </div>
  </PhaseStepperShell>
</template>

<style scoped>
.vector-canvas-wrap { flex: 1 1 auto; min-height: 0; border: 1px solid #e3ddc9; border-radius: 10px; background: #fff; }
.vector-canvas-wrap canvas { width: 100%; height: 100%; display: block; }
.legend { display: flex; gap: 1em; font-family: var(--font-mono, monospace); font-size: 0.6em; opacity: 0.75; }
.legend i { display: inline-block; width: 0.8em; height: 0.8em; border-radius: 2px; margin-right: 0.3em; vertical-align: -0.1em; }
</style>
