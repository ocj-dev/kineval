<script setup lang="ts">
import { computed } from 'vue'
import { layoutTree, KIND_GLYPH, STATUS_FILL, STATUS_STROKE } from '../lib/bt/layout'
import type { BtNode, TraceEntry } from '../lib/bt/types'

// Draws a behavior tree, and recolours it from a tick's trace.
//
// NOTE: opacity is set through `:style`, never through an
// `opacity="..."` attribute. Slidev runs UnoCSS in attributify mode,
// which reads a bare `opacity="1"` on an SVG element as the utility class
// `opacity-1` -- one per cent -- so every node rendered at 1% opacity and
// the diagram looked like a faint watermark while every attribute in the
// DOM inspector read exactly right.
//
// Font size goes through `:style` for the same class of reason: a
// `font-size="9.5"` presentation attribute lost to a stylesheet rule in
// one of the two panels that use this component and not the other, so the
// same diagram rendered with 9.5px labels in one place and 38px labels in
// the other. An inline style wins against both.
//
// The drawing comes from layoutTree (structure only) and the colouring
// from `trace` (one tick's visit record), so the picture cannot drift
// from the tree and the tree cannot move while it runs. A node the last
// tick did not visit is drawn pale -- which is information, not an
// omission: it is how a sequence short-circuiting is visible.
const props = withDefaults(defineProps<{
  root: BtNode
  trace?: TraceEntry[]
  columnWidth?: number
  rowHeight?: number
  fontSize?: number
  maxScale?: number
}>(), { columnWidth: 152, rowHeight: 62, fontSize: 10.5, maxScale: 1.45 })

const layout = computed(() => layoutTree(props.root, {
  columnWidth: props.columnWidth, rowHeight: props.rowHeight,
}))

const statusByPath = computed<Record<string, string>>(() => {
  const map: Record<string, string> = {}
  for (const entry of props.trace || []) map[entry.path] = entry.status
  return map
})

// the node most recently ticked that is still Running: the "you are here"
// marker, and the single most useful thing to see while a tree runs
const activePath = computed(() => {
  const running = (props.trace || []).filter((e) => e.status === 'running' && !e.kind.match(/sequence|fallback|parallel|decorator/))
  return running.length ? running[running.length - 1].path : null
})

const BOX_W = 132
const BOX_H = 30

function boxLabel(name: string) {
  return name.length > 26 ? name.slice(0, 25) + '…' : name
}
</script>

<template>
  <!-- The wrapper, and the max-width/max-height on the svg, are what stop
       a small tree in a large pane from being scaled up until its labels
       are the size of headings. An SVG with width:100% and height:100%
       scales its text with everything else, which is correct and is not
       what a diagram wants past about 1.5x. -->
  <div class="tree-wrap">
  <svg
    class="tree-svg"
    :viewBox="`0 0 ${layout.width} ${layout.height}`"
    preserveAspectRatio="xMidYMid meet"
    :style="{ maxWidth: layout.width * maxScale + 'px', maxHeight: layout.height * maxScale + 'px' }"
  >
    <!-- edges first, so boxes sit on top of them -->
    <g>
      <template v-for="n in layout.nodes" :key="'e' + n.path">
        <path
          v-if="n.parentPath !== null"
          :d="`M ${layout.byPath[n.parentPath].x} ${layout.byPath[n.parentPath].y + BOX_H}
               C ${layout.byPath[n.parentPath].x} ${layout.byPath[n.parentPath].y + BOX_H + 18},
                 ${n.x} ${n.y - 18},
                 ${n.x} ${n.y}`"
          class="edge"
          :class="{ 'edge-active': statusByPath[n.path] }"
        />
      </template>
    </g>

    <g v-for="n in layout.nodes" :key="n.path">
      <rect
        :x="n.x - BOX_W / 2" :y="n.y" :width="BOX_W" :height="BOX_H"
        :rx="n.kind === 'condition' ? BOX_H / 2 : 6"
        :fill="statusByPath[n.path] ? STATUS_FILL[statusByPath[n.path]] : '#ffffff'"
        :stroke="statusByPath[n.path] ? STATUS_STROKE[statusByPath[n.path]] : '#c7cad1'"
        :stroke-width="n.path === activePath ? 2.6 : 1.2"
        :style="{ opacity: statusByPath[n.path] ? 1 : 0.55 }"
      />
      <text
        :x="n.x" :y="n.y + BOX_H / 2 + 3.6" text-anchor="middle"
        class="label"
        :style="{ fontSize: fontSize + 'px', opacity: statusByPath[n.path] ? 1 : 0.6 }"
      >
        <tspan v-if="KIND_GLYPH[n.kind]" class="glyph">{{ KIND_GLYPH[n.kind] }}&#160;</tspan>{{ boxLabel(n.name) }}
      </text>
      <text
        v-if="statusByPath[n.path]"
        :x="n.x + BOX_W / 2 - 4" :y="n.y - 3" text-anchor="end"
        class="status-label"
        :style="{ fontSize: (fontSize - 1.5) + 'px' }"
        :fill="STATUS_STROKE[statusByPath[n.path]]"
      >{{ statusByPath[n.path] }}</text>
    </g>
  </svg>
  </div>
</template>

<style scoped>
.tree-wrap { width: 100%; height: 100%; display: flex; align-items: center; justify-content: center; min-height: 0; }
.tree-svg { width: 100%; height: 100%; display: block; }
.edge { fill: none; stroke: #c7cad1; stroke-width: 1.2; }
.edge-active { stroke: #202B4E; stroke-width: 1.6; }
.label { font-family: var(--font-mono, monospace); fill: #202B4E; }
.glyph { font-weight: 700; }
.status-label { font-family: var(--font-mono, monospace); font-weight: 700; }
</style>
