<script setup lang="ts">
import { ref, computed, watch, onMounted } from 'vue'
import { sceneById } from '../lib/astar/scenes'
import { astarPseudocode } from '../lib/astar/pseudocode'
import { useAStarTracer } from '../lib/astar/useAStarTracer'
import type { SearchAlg } from '../lib/astar/types'
import MapCanvas from './MapCanvas.vue'
import AStarControls from './AStarControls.vue'
import PseudocodePanel from './PseudocodePanel.vue'
import StatsBadge from './StatsBadge.vue'

const props = withDefaults(
  defineProps<{
    initialScene?: string
    initialAlg?: SearchAlg
    eps?: number
    speedMs?: number
    showPseudocode?: boolean
    showAlgPicker?: boolean
    compact?: boolean
  }>(),
  {
    initialScene: 'misc',
    initialAlg: 'A-star',
    eps: 0.2,
    speedMs: 12,
    showPseudocode: true,
    showAlgPicker: true,
    compact: false,
  },
)

const sceneId = ref(props.initialScene)
const searchAlg = ref<SearchAlg>(props.initialAlg)
const scene = computed(() => sceneById(sceneId.value))

const speed = ref(props.speedMs)
const tracer = useAStarTracer(speed)

function reload() {
  tracer.load(scene.value, searchAlg.value, props.eps)
}

onMounted(reload)
watch([sceneId, searchAlg], reload)

const pathLength = computed(() => {
  if (tracer.path.value.length < 2) return null
  let total = 0
  for (let i = 1; i < tracer.path.value.length; i++) {
    const a = tracer.path.value[i - 1]
    const b = tracer.path.value[i]
    total += Math.hypot(a.x - b.x, a.y - b.y)
  }
  return total
})
</script>

<template>
  <div class="stage" :class="{ compact }">
    <div class="stage-left">
      <div v-if="showPseudocode" class="pseudo-wrap">
        <PseudocodePanel :lines="astarPseudocode" :active-line="tracer.activeLine.value" />
      </div>
      <div class="controls-row">
        <AStarControls
          :is-running="tracer.isRunning.value"
          :is-done="tracer.isDone.value"
          :is-at-start="tracer.isAtStart.value"
          :scene-id="sceneId"
          :search-alg="searchAlg"
          :show-alg-picker="showAlgPicker"
          :compact="compact"
          @play="tracer.play"
          @pause="tracer.pause"
          @reset="tracer.reset"
          @step-forward="tracer.stepForward"
          @step-back="tracer.stepBack"
          @update:scene-id="sceneId = $event"
          @update:search-alg="searchAlg = $event"
        />
        <StatsBadge :visited="tracer.visitedCount.value" :path-length="pathLength" :status="tracer.status.value" />
      </div>
      <div class="scene-note">{{ scene.description }}</div>
    </div>

    <div class="stage-right panel">
      <MapCanvas
        :scene="scene"
        :eps="props.eps"
        :visited="tracer.visited.value"
        :queued="tracer.queued.value"
        :path="tracer.path.value"
        :current-node="tracer.currentNode.value"
      />
    </div>
  </div>
</template>

<style scoped>
.stage {
  display: grid;
  grid-template-columns: 0.85fr 1fr;
  gap: 1em;
  height: 100%;
  min-width: 0;
}
.stage-left {
  display: flex;
  flex-direction: column;
  gap: 0.5em;
  height: 100%;
  min-width: 0;
  min-height: 0;
}
.stage-right {
  min-width: 0;
  min-height: 0;
  display: flex;
  padding: 0.5em;
}
.pseudo-wrap {
  flex: 1 1 auto;
  min-height: 0;
}
.controls-row {
  flex: 0 0 auto;
  display: flex;
  flex-direction: column;
  gap: 0.35em;
}
.scene-note {
  flex: 0 0 auto;
  font-size: 0.6em;
  line-height: 1.3;
  color: #5f6368;
  padding: 0.1em 0.1em;
}
.stage.compact .scene-note {
  display: none;
}
</style>
