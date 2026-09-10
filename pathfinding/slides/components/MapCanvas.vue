<script setup lang="ts">
import { computed } from 'vue'
import type { Scene } from '../lib/astar/types'
import { testCollision, nodeKey } from '../lib/astar/aStarSteps'

const props = withDefaults(
  defineProps<{
    scene: Scene
    visited?: Set<string>
    queued?: Set<string>
    distances?: Map<string, number>
    edges?: Map<string, string>
    path?: { x: number; y: number }[]
    currentNode?: { x: number; y: number } | null
    neighborNode?: { x: number; y: number } | null
    neighborPriority?: number | null
  }>(),
  {
    visited: () => new Set(),
    queued: () => new Set(),
    distances: () => new Map(),
    edges: () => new Map(),
    path: () => [],
    currentNode: null,
    neighborNode: null,
    neighborPriority: null,
  },
)

const eps = computed(() => props.scene.gridBounds.eps)

// crop to the kineval-stencil reference canvas's own visible world window
// (xformWorldViewX/Y map world [-2,6) to canvas [0,800) -- see
// reference/draw.js) unless the scene supplies its own exact render bounds
// (small, purpose-built teaching grids).
const bounds = computed(() => props.scene.renderBounds ?? { xMin: -2, xMax: 6, yMin: -2, yMax: 6 })

const cells = computed(() => {
  const { xMin, xMax, yMin, yMax } = bounds.value
  const rows: { x: number; y: number; obstacle: boolean }[][] = []
  for (let y = yMin; y < yMax; y += eps.value) {
    const row: { x: number; y: number; obstacle: boolean }[] = []
    for (let x = xMin; x < xMax; x += eps.value) {
      row.push({ x, y, obstacle: testCollision(props.scene, [x, y]) })
    }
    rows.push(row)
  }
  // reverse so larger y renders toward the top of the map
  return rows.reverse()
})

const numRows = computed(() => cells.value.length)
const numCols = computed(() => cells.value[0]?.length ?? 0)

function cellCenter(x: number, y: number) {
  const { xMin, yMin } = bounds.value
  const col = Math.round((x - xMin) / eps.value)
  const rowFromBottom = Math.round((y - yMin) / eps.value)
  const row = numRows.value - 1 - rowFromBottom
  return { cx: col + 0.5, cy: row + 0.5 }
}

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

function cellDistanceText(cell: { x: number; y: number; obstacle: boolean }): string {
  if (cell.obstacle) return ''
  const k = nodeKey(cell.x, cell.y)
  if (!props.visited.has(k) && !props.queued.has(k)) return ''
  const d = props.distances.get(k)
  return d === undefined ? '' : d.toFixed(1)
}

function isNear(a: [number, number], b: [number, number], tol: number) {
  return Math.hypot(a[0] - b[0], a[1] - b[1]) <= tol
}

// Persistent search-tree edges: every child->parent relaxation recorded so
// far, drawn as bold lines -- reflects the current state of the search.
const treeEdges = computed(() => {
  const lines: { x1: number; y1: number; x2: number; y2: number }[] = []
  for (const [childKey, parentKey] of props.edges.entries()) {
    const [cx, cy] = childKey.split(',').map(Number)
    const [px, py] = parentKey.split(',').map(Number)
    const a = cellCenter(cx, cy)
    const b = cellCenter(px, py)
    lines.push({ x1: a.cx, y1: a.cy, x2: b.cx, y2: b.cy })
  }
  return lines
})

// Transient line from a just-enqueued neighbor to the goal, labeled with
// its f-score -- only shown for the step that enqueues it.
const goalLine = computed(() => {
  if (!props.neighborNode) return null
  const from = cellCenter(props.neighborNode.x, props.neighborNode.y)
  const to = cellCenter(props.scene.qGoal[0], props.scene.qGoal[1])
  return { x1: from.cx, y1: from.cy, x2: to.cx, y2: to.cy, mx: (from.cx + to.cx) / 2, my: (from.cy + to.cy) / 2 }
})
</script>

<template>
  <div class="map-outer">
    <div class="legend">
      <span class="swatch obstacle" /> Obstacle
      <span class="swatch free" /> Free
      <span class="swatch visited" /> Visited
      <span class="swatch queued" /> Queued
    </div>
    <div class="map-wrap">
      <div class="map-grid" :style="{ gridTemplateColumns: `repeat(${numCols}, 1fr)` }">
        <template v-for="row in cells" :key="row[0]?.y">
          <div
            v-for="cell in row"
            :key="cell.x"
            class="cell"
            :class="cellClass(cell)"
          >
            <div v-if="isNear([cell.x, cell.y], scene.qInit, eps / 2)" class="pin start-pin"><span>A</span></div>
            <div v-else-if="isNear([cell.x, cell.y], scene.qGoal, eps / 2)" class="pin goal-pin"><span>B</span></div>
            <span v-else class="cell-distance">{{ cellDistanceText(cell) }}</span>
          </div>
        </template>
      </div>
      <svg class="edge-overlay" :viewBox="`0 0 ${numCols} ${numRows}`" preserveAspectRatio="none">
        <line
          v-for="(e, i) in treeEdges" :key="'edge' + i"
          :x1="e.x1" :y1="e.y1" :x2="e.x2" :y2="e.y2"
          class="tree-edge"
        />
        <g v-if="goalLine">
          <line :x1="goalLine.x1" :y1="goalLine.y1" :x2="goalLine.x2" :y2="goalLine.y2" class="goal-line" />
          <rect :x="goalLine.mx - 0.32" :y="goalLine.my - 0.11" width="0.64" height="0.22" rx="0.05" class="goal-line-label-bg" />
          <text :x="goalLine.mx" :y="goalLine.my" class="goal-line-label" text-anchor="middle" dominant-baseline="middle">
            {{ neighborPriority?.toFixed(1) }}
          </text>
        </g>
      </svg>
    </div>
  </div>
</template>

<style scoped>
.map-outer {
  width: 100%;
  height: 100%;
  min-width: 0;
  min-height: 0;
  display: flex;
  flex-direction: column;
  gap: 0.4em;
}
.legend {
  flex: 0 0 auto;
  display: flex;
  align-items: center;
  gap: 0.5em;
  font-size: 0.6em;
  font-family: var(--font-body, sans-serif);
  color: #5f6368;
}
.swatch {
  display: inline-block;
  width: 0.9em;
  height: 0.9em;
  border-radius: 3px;
  margin-left: 0.6em;
}
.swatch:first-child { margin-left: 0; }
.swatch.obstacle { background: var(--building, #dadce0); border: 1px solid var(--building-border, #bdc1c6); }
.swatch.free { background: var(--road, #fff); border: 1px solid #dadce0; }
.swatch.visited { background: var(--visited, #aecbfa); }
.swatch.queued { background: var(--queued, #fef3c4); }

.map-wrap {
  position: relative;
  flex: 1 1 auto;
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
.edge-overlay {
  position: absolute;
  inset: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
}
.tree-edge {
  stroke: var(--gmaps-blue-dark, #174ea6);
  stroke-width: 0.12;
  stroke-linecap: round;
}
.goal-line {
  stroke: var(--gmaps-red, #ea4335);
  stroke-width: 0.08;
  stroke-dasharray: 0.12 0.1;
}
.goal-line-label-bg {
  fill: #fff;
  opacity: 0.9;
}
.goal-line-label {
  font-size: 0.16px;
  font-family: var(--font-mono, monospace);
  fill: var(--gmaps-red, #ea4335);
  font-weight: 700;
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
.cell-distance {
  font-size: 0.42em;
  font-family: var(--font-mono, monospace);
  color: var(--ink, #202124);
  opacity: 0.75;
  pointer-events: none;
  white-space: nowrap;
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
