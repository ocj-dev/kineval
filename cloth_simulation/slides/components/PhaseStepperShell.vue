<script setup lang="ts">
import PseudocodePanel from './PseudocodePanel.vue'

withDefaults(defineProps<{
  pseudocodeLines: string[]
  activeLine: number
  isRunning: boolean
  isDone: boolean
  isAtStart: boolean
  smooth: boolean
  stepLabel?: string
  frameInfo?: string
}>(), { stepLabel: '', frameInfo: '' })

const emit = defineEmits<{
  play: []
  pause: []
  stepForward: []
  stepBack: []
  reset: []
  'update:smooth': [boolean]
}>()
</script>

<template>
  <div class="stepper-shell">
    <div class="pseudocode-pane">
      <PseudocodePanel :lines="pseudocodeLines" :active-line="smooth ? -1 : activeLine" />
    </div>
    <div class="sim-pane">
      <div class="controls">
        <slot name="controls" />
      </div>
      <div class="playback">
        <label class="check smooth-check">
          <input
            type="checkbox" :checked="smooth"
            @change="emit('update:smooth', ($event.target as HTMLInputElement).checked)"
          >
          Smooth (skip step breakdown)
        </label>
        <button class="btn" :disabled="isAtStart || smooth" @click="emit('stepBack')">&laquo; Step</button>
        <button v-if="!isRunning" class="btn primary" :disabled="isDone" @click="emit('play')">&#9654; Play</button>
        <button v-else class="btn primary" @click="emit('pause')">&#10074;&#10074; Pause</button>
        <button class="btn" :disabled="isDone || smooth" @click="emit('stepForward')">Step &raquo;</button>
        <button class="btn" @click="emit('reset')">&#8634; Reset</button>
        <span v-if="!smooth && stepLabel" class="phase-tag">{{ stepLabel }}</span>
        <span v-if="frameInfo" class="frame-count">{{ frameInfo }}</span>
      </div>
      <div class="vis-slot">
        <slot />
      </div>
    </div>
  </div>
</template>

<style scoped>
.stepper-shell {
  display: grid;
  grid-template-columns: 30% 70%;
  gap: 0.8em;
  height: 100%;
  min-height: 0;
}
.pseudocode-pane { min-height: 0; display: flex; }
.sim-pane { display: flex; flex-direction: column; gap: 0.4em; min-height: 0; min-width: 0; }
.controls { display: flex; align-items: center; gap: 0.8em; flex-wrap: wrap; }
.playback { display: flex; align-items: center; gap: 0.5em; flex-wrap: wrap; }
.check { font-family: var(--font-mono, monospace); font-size: 0.72em; display: flex; align-items: center; gap: 0.3em; cursor: pointer; }
.smooth-check { color: var(--blue, #00274C); font-weight: 700; }
.phase-tag {
  font-family: var(--font-mono, monospace); font-size: 0.66em; font-weight: 700;
  background: #fff6d6; border: 1px solid #00274C; color: #00274C; border-radius: 999px; padding: 0.2em 0.7em;
}
.frame-count { font-family: var(--font-mono, monospace); font-size: 0.6em; opacity: 0.6; }
.vis-slot { flex: 1 1 auto; min-height: 0; }
</style>
