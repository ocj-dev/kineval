<script setup lang="ts">
import { scenes } from '../lib/astar/scenes'
import type { SearchAlg } from '../lib/astar/types'

withDefaults(
  defineProps<{
    isRunning: boolean
    isDone?: boolean
    isAtStart?: boolean
    sceneId: string
    searchAlg: SearchAlg
    showRouting: boolean
    skipQueueing: boolean
    showAlgPicker?: boolean
    showScenePicker?: boolean
    compact?: boolean
  }>(),
  { isDone: false, isAtStart: false, showAlgPicker: true, showScenePicker: true, compact: false },
)

const emit = defineEmits<{
  play: []
  pause: []
  reset: []
  stepForward: []
  stepBack: []
  'update:sceneId': [string]
  'update:searchAlg': [SearchAlg]
  'update:showRouting': [boolean]
  'update:skipQueueing': [boolean]
}>()

const algOptions: { id: SearchAlg; label: string }[] = [
  { id: 'A-star', label: 'A-star' },
  { id: 'greedy-best-first', label: 'Greedy Best-First' },
  { id: 'breadth-first', label: 'Breadth-First' },
  { id: 'depth-first', label: 'Depth-First' },
]
</script>

<template>
  <div class="astar-controls" :class="{ compact }">
    <div class="row buttons">
      <button class="btn-maps" :disabled="isAtStart" title="Step back" @click="emit('stepBack')">« Step</button>
      <button v-if="!isRunning" class="btn-maps primary" :disabled="isDone" @click="emit('play')">▶ Start</button>
      <button v-else class="btn-maps primary" @click="emit('pause')">⏸ Pause</button>
      <button class="btn-maps" :disabled="isDone" title="Step forward" @click="emit('stepForward')">Step »</button>
      <button class="btn-maps" @click="emit('reset')">⟲ Reset</button>
    </div>
    <div class="row pickers">
      <label v-if="showScenePicker" class="picker">
        <span>Scene</span>
        <select :value="sceneId" @change="emit('update:sceneId', ($event.target as HTMLSelectElement).value)">
          <option v-for="s in scenes" :key="s.id" :value="s.id">{{ s.name }}{{ s.isNew ? ' (new)' : '' }}</option>
        </select>
      </label>
      <label v-if="showScenePicker" class="checkbox-picker">
        <input
          type="checkbox" :checked="showRouting"
          @change="emit('update:showRouting', ($event.target as HTMLInputElement).checked)"
        />
        Routing
      </label>
      <label v-if="showAlgPicker" class="picker">
        <span>Algorithm</span>
        <select :value="searchAlg" @change="emit('update:searchAlg', ($event.target as HTMLSelectElement).value as SearchAlg)">
          <option v-for="a in algOptions" :key="a.id" :value="a.id">{{ a.label }}</option>
        </select>
      </label>
      <label v-if="showAlgPicker" class="checkbox-picker">
        <input
          type="checkbox" :checked="skipQueueing"
          @change="emit('update:skipQueueing', ($event.target as HTMLInputElement).checked)"
        />
        Skip Queueing
      </label>
    </div>
  </div>
</template>

<style scoped>
.astar-controls {
  display: flex;
  flex-direction: column;
  gap: 0.5em;
}
.row {
  display: flex;
  gap: 0.5em;
  align-items: center;
  flex-wrap: wrap;
}
.picker {
  display: flex;
  align-items: center;
  gap: 0.4em;
  font-family: var(--font-mono, monospace);
  font-size: 0.72em;
  color: #5f6368;
}
.picker select {
  font-family: var(--font-body, sans-serif);
  font-size: 1em;
  padding: 0.3em 0.5em;
  border-radius: 8px;
  border: 1px solid #dadce0;
  background: #fff;
  color: var(--ink, #202124);
}
.checkbox-picker {
  display: flex;
  align-items: center;
  gap: 0.3em;
  font-family: var(--font-mono, monospace);
  font-size: 0.72em;
  color: #5f6368;
  cursor: pointer;
  white-space: nowrap;
}
.checkbox-picker input {
  cursor: pointer;
}
.astar-controls.compact .btn-maps {
  font-size: 0.7em;
  padding: 0.4em 0.7em;
}
.astar-controls.compact .picker,
.astar-controls.compact .checkbox-picker {
  font-size: 0.62em;
}
</style>
