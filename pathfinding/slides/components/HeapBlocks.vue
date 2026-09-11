<script setup lang="ts">
import { computed } from 'vue'
import type { HeapEntry } from '../lib/astar/types'

const props = defineProps<{
  heap: HeapEntry[]
}>()

const shown = computed(() => props.heap.slice(0, 3))
const overflow = computed(() => Math.max(0, props.heap.length - 3))
</script>

<template>
  <div class="heap-blocks">
    <div class="heap-title">Open queue (heap array, first 3 of {{ heap.length }})</div>
    <div class="blocks-box">
      <div class="blocks">
        <div v-for="(entry, idx) in shown" :key="idx" class="block" :class="{ root: idx === 0 }">
          <div class="idx">[{{ entry.i }},{{ entry.j }}]</div>
          <div class="f">f={{ entry.priority.toFixed(1) }}</div>
        </div>
        <div v-if="overflow > 0" class="block more">+{{ overflow }}</div>
        <div v-if="heap.length === 0" class="empty">empty</div>
      </div>
    </div>
  </div>
</template>

<style scoped>
.heap-blocks {
  font-size: 0.6em;
}
.heap-title {
  font-family: var(--font-mono, monospace);
  color: #5f6368;
  margin-bottom: 0.3em;
}
.blocks-box {
  border: 1.5px solid #dadce0;
  border-radius: 8px;
  padding: 0.4em;
  display: inline-block;
}
.blocks {
  display: flex;
  gap: 0.35em;
  flex-wrap: wrap;
}
.block {
  border: 1px solid #dadce0;
  background: #fff;
  border-radius: 6px;
  padding: 0.25em 0.5em;
  font-family: var(--font-mono, monospace);
  text-align: center;
  min-width: 3.4em;
  box-shadow: 0 1px 2px #00000014;
}
.block.root {
  border-color: var(--gmaps-blue, #1a73e8);
  background: #e8f0fe;
}
.block .idx {
  font-weight: 700;
  color: var(--ink, #202124);
}
.block .f {
  color: var(--gmaps-blue-dark, #174ea6);
}
.block.more {
  display: flex;
  align-items: center;
  justify-content: center;
  color: #5f6368;
  font-weight: 700;
}
.empty {
  font-family: var(--font-mono, monospace);
  color: #5f6368;
  font-style: italic;
}
</style>
