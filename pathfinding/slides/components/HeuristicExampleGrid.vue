<script setup lang="ts">
import { computed } from 'vue'
import MapCanvas from './MapCanvas.vue'
import { miniGridScene } from '../lib/astar/scenes'
import { nodeKey } from '../lib/astar/aStarSteps'
import type { HeuristicMode } from '../lib/astar/types'

// A static, hand-worked example reusing the same simple 3x4 grid as the
// walkthrough slides: the cell in the leftmost column, second-from-top row
// (0,2), being relaxed by the cell directly below it (0,1) -- a real step
// from that grid's own A-star trace, not a fabricated one. Values below
// are the actual computed g/h/f for this exact transition (verified
// against aStarSteps.ts): g=2, h(euclidean)=sqrt(5)=2.24, h(manhattan)=3.
const props = defineProps<{ mode: HeuristicMode }>()

const current = { x: 0, y: 1 }
const neighbor = { x: 0, y: 2 }
const goal = miniGridScene.qGoal

const g = 2
const h = computed(() => (props.mode === 'manhattan'
  ? Math.abs(neighbor.x - goal[0]) + Math.abs(neighbor.y - goal[1])
  : Math.hypot(neighbor.x - goal[0], neighbor.y - goal[1])))
const f = computed(() => g + h.value)

const visited = new Set([nodeKey(0, 0), nodeKey(0, 1)])
const distances = new Map([[nodeKey(0, 0), 0], [nodeKey(0, 1), 1]])
const edges = new Map([[nodeKey(0, 1), nodeKey(0, 0)]])
</script>

<template>
  <div class="heuristic-example">
    <MapCanvas
      :scene="miniGridScene"
      :visited="visited"
      :distances="distances"
      :edges="edges"
      :current-node="current"
      :neighbor-node="neighbor"
      :neighbor-priority="f"
      :neighbor-g-score="g"
      :neighbor-h-score="h"
      :heuristic-mode="mode"
    />
  </div>
</template>

<style scoped>
.heuristic-example {
  width: 100%;
  flex: 1 1 auto;
  min-height: 0;
  min-width: 0;
  display: flex;
}
</style>
