<script setup lang="ts">
withDefaults(
  defineProps<{
    visited: number
    queued?: number | null
    pathLength?: number | null
    status?: 'idle' | 'iterating' | 'succeeded' | 'failed'
  }>(),
  { queued: null, pathLength: null, status: undefined },
)
</script>

<template>
  <div class="stats">
    <span class="stat">visited <b>{{ visited }}</b></span>
    <span v-if="queued !== null" class="stat">queued <b>{{ queued }}</b></span>
    <span v-if="pathLength !== null" class="stat">path length <b>{{ pathLength.toFixed(2) }}</b></span>
    <span v-if="status" class="status" :class="status">{{ status }}</span>
  </div>
</template>

<style scoped>
.stats {
  display: flex;
  gap: 0.8em;
  align-items: center;
  font-family: var(--font-mono, 'Roboto Mono', monospace);
  font-size: 0.72em;
  color: var(--ink, #202124);
  flex-wrap: wrap;
}
.stat b {
  color: var(--gmaps-blue, #1a73e8);
}
.status {
  border: 1px solid #dadce0;
  text-transform: uppercase;
  border-radius: 999px;
  padding: 0.15em 0.7em;
  font-size: 0.85em;
  font-weight: 700;
  background: #fff;
}
.status.iterating { background: var(--gmaps-yellow, #fbbc04); }
.status.succeeded { background: var(--gmaps-green, #34a853); color: #fff; }
.status.failed { background: var(--gmaps-red, #ea4335); color: #fff; }
</style>
