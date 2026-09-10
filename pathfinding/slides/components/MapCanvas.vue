<script setup lang="ts">
import { computed } from 'vue'
import type { Scene } from '../lib/astar/types'
import { testCollision, nodeKey } from '../lib/astar/aStarSteps'

const props = withDefaults(
  defineProps<{
    scene: Scene
    eps: number
    visited?: Set<string>
    queued?: Set<string>
    path?: { x: number; y: number }[]
    currentNode?: { x: number; y: number } | null
  }>(),
  {
    visited: () => new Set(),
    queued: () => new Set(),
    path: () => [],
    currentNode: null,
  },
)

// crop to the same world window the standalone reference canvas draws
// (xformWorldViewX/Y map world [-2,6) to canvas [0,800) -- see reference/draw.js)
const WORLD_MIN = -2
const WORLD_MAX = 6

const cells = computed(() => {
  const rows: { x: number; y: number; obstacle: boolean }[][] = []
  for (let y = WORLD_MIN; y < WORLD_MAX; y += props.eps) {
    const row: { x: number; y: number; obstacle: boolean }[] = []
    for (let x = WORLD_MIN; x < WORLD_MAX; x += props.eps) {
      row.push({ x, y, obstacle: testCollision(props.scene, [x, y]) })
    }
    rows.push(row)
  }
  // reverse so larger y renders toward the top of the map
  return rows.reverse()
})

const cols = computed(() => cells.value[0]?.length ?? 0)

const pathKeys = computed(() => new Set(props.path.map((p) => nodeKey(p.x, p.y))))

function cellClass(cell: { x: number; y: number; obstacle: boolean }) {
  const k = nodeKey(cell.x, cell.y)
  return {
    obstacle: cell.obstacle,
    visited: !cell.obstacle && props.visited.has(k),
    queued: !cell.obstacle && props.queued.has(k),
    path: !cell.obstacle && pathKeys.value.has(k),
    current: !cell.obstacle && props.currentNode
      && nodeKey(props.currentNode.x, props.currentNode.y) === k,
  }
}

function isNear(a: [number, number], b: [number, number], tol: number) {
  return Math.hypot(a[0] - b[0], a[1] - b[1]) <= tol
}
</script>

<template>
  <div class="map-wrap">
    <div class="map-grid" :style="{ gridTemplateColumns: `repeat(${cols}, 1fr)` }">
      <template v-for="row in cells" :key="row[0]?.y">
        <div
          v-for="cell in row"
          :key="cell.x"
          class="cell"
          :class="cellClass(cell)"
        >
          <div v-if="isNear([cell.x, cell.y], scene.qInit, eps / 2)" class="pin start-pin"><span>A</span></div>
          <div v-else-if="isNear([cell.x, cell.y], scene.qGoal, eps / 2)" class="pin goal-pin"><span>B</span></div>
        </div>
      </template>
    </div>
  </div>
</template>

<style scoped>
.map-wrap {
  width: 100%;
  height: 100%;
  min-width: 0;
  min-height: 0;
  display: flex;
  border-radius: 10px;
  overflow: hidden;
  box-shadow: 0 1px 2px #00000026, 0 1px 3px 1px #00000014;
}
.map-grid {
  display: grid;
  grid-auto-rows: 1fr;
  width: 100%;
  height: 100%;
  background: var(--road, #fff);
}
.cell {
  position: relative;
  background: var(--road, #fff);
  outline: 0.5px solid #00000008;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: background-color 0.12s ease;
}
.cell.obstacle {
  background: var(--building, #dadce0);
  outline: 0.5px solid var(--building-border, #bdc1c6);
}
.cell.visited {
  background: var(--visited, #aecbfa);
}
.cell.queued {
  background: var(--queued, #fef3c4);
}
.cell.path {
  background: var(--gmaps-blue, #1a73e8);
}
.cell.current {
  box-shadow: inset 0 0 0 2px var(--gmaps-red, #ea4335);
  z-index: 2;
}
.pin {
  font-family: var(--font-display, 'Roboto', sans-serif);
  font-weight: 900;
  font-size: 0.55em;
  color: #fff;
  border-radius: 50% 50% 50% 0;
  transform: rotate(-45deg);
  width: 1.6em;
  height: 1.6em;
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 3;
  box-shadow: 0 1px 3px #00000059;
}
.pin span { transform: rotate(45deg); }
.start-pin { background: var(--gmaps-green, #34a853); }
.goal-pin { background: var(--gmaps-red, #ea4335); }
</style>
