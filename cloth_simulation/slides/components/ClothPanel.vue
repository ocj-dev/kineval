<script setup lang="ts">
withDefaults(defineProps<{ query?: string }>(), { query: '' })
</script>

<template>
  <div class="cloth-embed">
    <iframe :src="`/kineval/cloth_simulation/reference/cloth_canvas.html${query}`" frameborder="0" />
  </div>
</template>

<style scoped>
.cloth-embed { flex: 1 1 auto; min-height: 0; border-radius: 10px; overflow: hidden; box-shadow: 0 2px 6px #00000033; }
.cloth-embed iframe { width: 100%; height: 100%; border: 0; background: #f2efe9; }
</style>
