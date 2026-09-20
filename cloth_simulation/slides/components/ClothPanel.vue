<script setup lang="ts">
withDefaults(defineProps<{ query?: string }>(), { query: '' })
</script>

<template>
  <div class="cloth-embed">
    <iframe :src="`/kineval/cloth_simulation/reference/cloth_canvas.html${query}`" frameborder="0" />
  </div>
</template>

<style scoped>
/* fill the full presentation space available to this slide, not just a
   flex remainder -- cloth_canvas.html's own canvas is itself responsive
   (see reference/infrastructure.js's resizeCanvasToContainer()), so this
   iframe growing to fill the slide is what makes the embedded canvas grow too */
.cloth-embed { flex: 1 1 auto; height: 100%; min-height: 500px; border-radius: 10px; overflow: hidden; box-shadow: 0 2px 6px #00000033; }
.cloth-embed iframe { display: block; width: 100%; height: 100%; border: 0; background: #f2efe9; }
</style>
