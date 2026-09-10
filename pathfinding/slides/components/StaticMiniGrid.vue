<script setup lang="ts">
import { miniGridScene } from '../lib/astar/scenes'
import MapCanvas from './MapCanvas.vue'
</script>

<template>
  <div class="static-grid">
    <MapCanvas :scene="miniGridScene" />
  </div>
</template>

<style scoped>
.static-grid {
  width: 100%;
  height: 100%;
  min-height: 260px;
}
</style>
