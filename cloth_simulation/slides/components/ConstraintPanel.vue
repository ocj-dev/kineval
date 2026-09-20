<script setup lang="ts">
import { computed } from 'vue'
import { useConstraintTracer } from '../lib/cloth/useConstraintTracer'

const props = withDefaults(defineProps<{ collision?: boolean }>(), { collision: false })

const WIDTH = 420
const HEIGHT = props.collision ? 170 : 260
const bounds = { minX: 20, maxX: WIDTH - 20, minY: 20, maxY: props.collision ? 95 : HEIGHT - 20 }

const { current, pos, trace, isRunning, isDone, isAtStart, play, pause, reset, stepForward, stepBack } =
  useConstraintTracer(props.collision, bounds)

const phaseLabel = computed(() => ({
  accumulate: 'Accumulate forces',
  integrate: 'Verlet integrate',
  relax: 'Satisfy constraint (relax)',
  collide: 'Collision detect & respond',
}[current.value.phase]))
</script>

<template>
  <div class="constraint-panel">
    <div class="controls">
      <button class="btn" :disabled="isAtStart" @click="stepBack">&laquo; Step</button>
      <button v-if="!isRunning" class="btn primary" :disabled="isDone" @click="play">&#9654; Play</button>
      <button v-else class="btn primary" @click="pause">&#10074;&#10074; Pause</button>
      <button class="btn" :disabled="isDone" @click="stepForward">Step &raquo;</button>
      <button class="btn" @click="reset">&#8634; Reset</button>
      <span class="phase-tag">{{ phaseLabel }}</span>
      <span class="frame-count">frame {{ current.frame }} &middot; step {{ pos + 1 }}/{{ trace.length }}</span>
    </div>
    <svg class="constraint-canvas" :viewBox="`0 0 ${WIDTH} ${HEIGHT}`" :width="WIDTH" :height="HEIGHT">
      <line v-if="collision" :x1="0" :y1="bounds.maxY" :x2="WIDTH" :y2="bounds.maxY" stroke="#00274C" stroke-width="2" />
      <line :x1="current.p1.x" :y1="current.p1.y" :x2="current.p2.x" :y2="current.p2.y" stroke="#888" stroke-width="2" />
      <rect :x="current.p1.x - 7" :y="current.p1.y - 7" width="14" height="14" fill="#00274C" />
      <circle :cx="current.p2.x" :cy="current.p2.y" r="7" fill="#FFCB05" stroke="#00274C" stroke-width="1.5" />
      <g v-for="(v, i) in current.vectors" :key="i">
        <line :x1="v.from.x" :y1="v.from.y" :x2="v.to.x" :y2="v.to.y" :stroke="v.color" stroke-width="2.5" marker-end="url(#arrowhead)" />
      </g>
      <defs>
        <marker id="arrowhead" markerWidth="8" markerHeight="8" refX="6" refY="4" orient="auto">
          <path d="M0,0 L8,4 L0,8 Z" fill="context-stroke" />
        </marker>
      </defs>
    </svg>
    <div class="legend">
      <span>anchor = pinned square (fixed position)</span>
      <span v-if="current.phase === 'relax'"><i style="background:#FFCB05" /> constraint correction</span>
      <span v-else-if="current.phase === 'collide'"><i style="background:#7d3ac1" /> collision response</span>
      <span v-else><i style="background:#00274C" /> gravity force</span>
    </div>
  </div>
</template>

<style scoped>
.constraint-panel { display: flex; flex-direction: column; gap: 0.4em; height: 100%; }
.controls { display: flex; align-items: center; gap: 0.5em; flex-wrap: wrap; }
.phase-tag {
  font-family: var(--font-mono, monospace); font-size: 0.68em; font-weight: 700;
  background: #fff6d6; border: 1px solid #00274C; color: #00274C; border-radius: 999px; padding: 0.2em 0.7em;
}
.frame-count { font-family: var(--font-mono, monospace); font-size: 0.62em; opacity: 0.6; }
.constraint-canvas { border: 1px solid #e3ddc9; border-radius: 10px; background: #fff; width: 100%; height: auto; flex: 1 1 auto; }
.legend { display: flex; gap: 1em; font-family: var(--font-mono, monospace); font-size: 0.62em; opacity: 0.75; }
.legend i { display: inline-block; width: 0.8em; height: 0.8em; border-radius: 2px; margin-right: 0.3em; vertical-align: -0.1em; }
</style>
