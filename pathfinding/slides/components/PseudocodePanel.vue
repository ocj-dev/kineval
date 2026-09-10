<script setup lang="ts">
withDefaults(
  defineProps<{
    lines: string[]
    activeLine: number
  }>(),
  {},
)
</script>

<template>
  <div class="pseudocode">
    <div
      v-for="(line, i) in lines"
      :key="i"
      class="pc-line"
      :class="{ active: i + 1 === activeLine, blank: !line }"
    >
      <span class="pc-num">{{ i + 1 }}</span>
      <span class="pc-text">{{ line || ' ' }}</span>
    </div>
  </div>
</template>

<style scoped>
.pseudocode {
  font-family: var(--font-mono, 'Roboto Mono', monospace);
  font-size: 0.6em;
  background: #202124;
  color: #e8eaed;
  border-radius: 10px;
  padding: 0.5em 0.7em;
  line-height: 1.25;
  height: 100%;
  overflow-y: auto;
  box-sizing: border-box;
}
.pc-line {
  display: grid;
  grid-template-columns: 1.4em 1fr;
  column-gap: 0.7em;
  padding: 0.05em 0.4em;
  white-space: pre-wrap;
  word-break: break-word;
  border-radius: 5px;
}
.pc-num {
  color: #9aa0a6;
  text-align: right;
  user-select: none;
}
.pc-text {
  white-space: pre-wrap;
  word-break: break-word;
  min-width: 0;
}
.pc-line.active {
  background: var(--gmaps-yellow, #fbbc04);
  color: #202124;
  font-weight: 700;
}
.pc-line.active .pc-num {
  color: #202124;
  opacity: 0.6;
}
</style>
