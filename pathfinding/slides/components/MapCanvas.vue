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
    showRouting?: boolean
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
    showRouting: true,
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

function pointsAttr(points: { x: number; y: number }[]): string {
  return points.map((p) => { const c = cellCenter(p.x, p.y); return `${c.cx},${c.cy}` }).join(' ')
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
    neighbor: !cell.obstacle && props.neighborNode
      && nodeKey(props.neighborNode.x, props.neighborNode.y) === k,
  }
}

function cellDistanceText(cell: { x: number; y: number; obstacle: boolean }): string {
  if (!props.scene.isTeachingExample) return ''
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

// Walks the recorded parent chain from (x,y) back to the start node,
// using the same edges map the persistent search tree is drawn from.
// Returns points ordered [start, ..., (x,y)].
function routeToStart(x: number, y: number): { x: number; y: number }[] {
  const points: { x: number; y: number }[] = [{ x, y }]
  let key = nodeKey(x, y)
  let guard = 0
  while (props.edges.has(key) && guard < 10000) {
    const parentKey = props.edges.get(key)!
    const [px, py] = parentKey.split(',').map(Number)
    points.push({ x: px, y: py })
    key = parentKey
    guard++
  }
  return points.reverse()
}

// The tentative route "through" the neighbor currently being enqueued:
// the actual (solid) path back to the start via the current node, plus the
// (dashed) straight-line heuristic estimate from the neighbor to the goal.
// Only shown once the neighbor's f-score has actually been computed
// (lines 12-13), not merely while it's being considered (lines 7-10).
const tentativeRoute = computed(() => {
  if (!props.neighborNode || props.neighborPriority === null || !props.currentNode) return null
  const solidPoints = [...routeToStart(props.currentNode.x, props.currentNode.y), props.neighborNode]
  return {
    solid: pointsAttr(solidPoints),
    dashed: `${pointsAttr([props.neighborNode])} ${pointsAttr([{ x: props.scene.qGoal[0], y: props.scene.qGoal[1] }])}`,
  }
})

// The final route once the search succeeds, extended with explicit
// segments to the exact start/goal coordinates so it always visually
// reaches both pins even when the nearest visited node isn't exactly on
// top of them.
const finalRouteLine = computed(() => {
  if (props.path.length === 0) return null
  const points = [
    { x: props.scene.qGoal[0], y: props.scene.qGoal[1] },
    ...props.path,
    { x: props.scene.qInit[0], y: props.scene.qInit[1] },
  ]
  return pointsAttr(points)
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
    <div class="map-wrap" :class="{ 'scene-teaching': scene.isTeachingExample }">
      <div v-if="showRouting && neighborPriority !== null" class="fscore-label">f = {{ neighborPriority.toFixed(1) }}</div>
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
      <svg v-if="showRouting" class="edge-overlay" :viewBox="`0 0 ${numCols} ${numRows}`" preserveAspectRatio="none">
        <line
          v-for="(e, i) in treeEdges" :key="'edge' + i"
          :x1="e.x1" :y1="e.y1" :x2="e.x2" :y2="e.y2"
          class="tree-edge"
        />
        <polyline v-if="finalRouteLine" :points="finalRouteLine" class="final-route-line" />
        <g v-if="tentativeRoute">
          <polyline :points="tentativeRoute.solid" class="route-solid" />
          <polyline :points="tentativeRoute.dashed" class="route-dashed" />
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
.fscore-label {
  position: absolute;
  top: 0.5em;
  left: 0.5em;
  z-index: 4;
  background: #fffffff0;
  border: 1px solid var(--gmaps-red, #ea4335);
  color: var(--gmaps-red, #ea4335);
  font-family: var(--font-mono, monospace);
  font-weight: 700;
  font-size: 0.65em;
  padding: 0.15em 0.5em;
  border-radius: 6px;
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
  /* routing lines always render above every cell/pin in the grid */
  z-index: 10;
}
.tree-edge {
  stroke: var(--gmaps-blue-dark, #174ea6);
  stroke-width: 0.12;
  stroke-linecap: round;
}
.final-route-line {
  fill: none;
  stroke: var(--gmaps-red, #ea4335);
  stroke-width: 0.16;
  stroke-linecap: round;
  stroke-linejoin: round;
}
.route-solid {
  fill: none;
  stroke: var(--gmaps-red, #ea4335);
  stroke-width: 0.08;
  stroke-linecap: round;
  stroke-linejoin: round;
}
.route-dashed {
  fill: none;
  stroke: var(--gmaps-red, #ea4335);
  stroke-width: 0.08;
  stroke-dasharray: 0.12 0.1;
}
.cell {
  position: relative;
  background: var(--road, #fff);
  outline: 0.5px solid #00000030;
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
.cell.path.visited,
.cell.path.queued,
.cell.path {
  /* deliberately higher specificity than .cell.visited/.cell.queued alone
     (a path cell is necessarily also visited) so the final route always
     shows red regardless of stylesheet rule order */
  background: var(--gmaps-red, #ea4335);
}
.cell.current {
  box-shadow: inset 0 0 0 2px var(--gmaps-red, #ea4335);
  z-index: 2;
}
.cell.neighbor {
  box-shadow: inset 0 0 0 2px var(--gmaps-purple, #a142f4);
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
.scene-teaching .cell-distance {
  position: absolute;
  left: 0.15em;
  bottom: 0.05em;
  font-size: 1.05em;
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
