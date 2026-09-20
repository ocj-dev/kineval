<script setup lang="ts">
import { ref, computed, onMounted, watch } from 'vue'
import { useConstraintRigidTracer } from '../lib/cloth/useConstraintRigidTracer'
import { worldCorner } from '../lib/cloth/clothPhysics'
import { drawArrow, drawRigidSquare } from '../lib/cloth/drawUtils'

const WIDTH = 420, HEIGHT = 260

const { current, pos, trace, isRunning, isDone, isAtStart, play, pause, reset, stepForward, stepBack } =
  useConstraintRigidTracer()

const canvasRef = ref<HTMLCanvasElement | null>(null)

const phaseLabel = computed(() => ({
  accumulate: 'Accumulate forces',
  integrate: 'Verlet integrate (position + orientation)',
  relax: 'Satisfy 2 corner constraints (relax)',
}[current.value.phase]))

function render() {
  const canvas = canvasRef.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  ctx.clearRect(0, 0, WIDTH, HEIGHT)

  const cornersA = [0, 1, 2, 3].map(i => worldCorner(current.value.bodyA, i))
  const cornersB = [0, 1, 2, 3].map(i => worldCorner(current.value.bodyB, i))
  drawRigidSquare(ctx, cornersA, '#00274Ccc')   // anchor: solid blue
  drawRigidSquare(ctx, cornersB, '#FFCB05cc')   // free body: maize

  for (const v of current.value.vectors) drawArrow(ctx, v.from.x, v.from.y, v.to.x, v.to.y, v.color)
}

onMounted(render)
watch(current, render)
</script>

<template>
  <div class="constraint-rigid-panel">
    <div class="controls">
      <button class="btn" :disabled="isAtStart" @click="stepBack">&laquo; Step</button>
      <button v-if="!isRunning" class="btn primary" :disabled="isDone" @click="play">&#9654; Play</button>
      <button v-else class="btn primary" @click="pause">&#10074;&#10074; Pause</button>
      <button class="btn" :disabled="isDone" @click="stepForward">Step &raquo;</button>
      <button class="btn" @click="reset">&#8634; Reset</button>
      <span class="phase-tag">{{ phaseLabel }}</span>
      <span class="frame-count">frame {{ current.frame }} &middot; step {{ pos + 1 }}/{{ trace.length }}</span>
    </div>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasRef" :width="WIDTH" :height="HEIGHT" />
    </div>
    <div class="legend">
      <span><i style="background:#00274C" /> anchor square (fixed position + orientation)</span>
      <span v-if="current.phase === 'relax'"><i style="background:#FFCB05" /> corner corrections (2 constraints)</span>
      <span v-else><i style="background:#00274C" /> gravity force</span>
    </div>
  </div>
</template>

<style scoped>
.constraint-rigid-panel { display: flex; flex-direction: column; gap: 0.4em; height: 100%; }
.controls { display: flex; align-items: center; gap: 0.5em; flex-wrap: wrap; }
.phase-tag {
  font-family: var(--font-mono, monospace); font-size: 0.68em; font-weight: 700;
  background: #fff6d6; border: 1px solid #00274C; color: #00274C; border-radius: 999px; padding: 0.2em 0.7em;
}
.frame-count { font-family: var(--font-mono, monospace); font-size: 0.62em; opacity: 0.6; }
.vector-canvas-wrap { flex: 1 1 auto; min-height: 0; border: 1px solid #e3ddc9; border-radius: 10px; background: #fff; }
.vector-canvas-wrap canvas { width: 100%; height: 100%; display: block; }
.legend { display: flex; gap: 1em; font-family: var(--font-mono, monospace); font-size: 0.62em; opacity: 0.75; }
.legend i { display: inline-block; width: 0.8em; height: 0.8em; border-radius: 2px; margin-right: 0.3em; vertical-align: -0.1em; }
</style>
