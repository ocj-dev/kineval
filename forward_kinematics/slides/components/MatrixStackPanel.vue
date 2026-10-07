<script setup lang="ts">
import { reactive, ref, watch, onMounted, computed } from 'vue'
import PhaseStepperShell from './PhaseStepperShell.vue'
import { useFkTracer } from '../lib/fk/useFkTracer'
import { createMobileArmRobot, createUrdfExampleRobot, edgesOf } from '../lib/fk/robots'
import { MASTER_PSEUDOCODE } from '../lib/fk/pseudocode'
import { drawFkTree } from '../lib/fk/drawUtils'
import type { RobotDesc } from '../lib/fk/types'

// Drives the DFS traversal of buildFKTransforms/traverseFKLink/
// traverseFKJoint one node at a time, revealing each link as the real
// traversal would reach it (see fkSteps.ts) and keeping the pseudocode
// panel's active line in lock-step -- the "step-by-step walkthrough" this
// lab's component-breakdown slides exist to give.

const props = withDefaults(defineProps<{ robotName?: 'mobile_arm' | 'urdf_example'; smoothDefault?: boolean }>(), {
  robotName: 'mobile_arm',
  smoothDefault: false,
})

const smooth = ref(props.smoothDefault)
const speedMs = ref(500)
const tracer = useFkTracer(speedMs)

let robot: RobotDesc = props.robotName === 'urdf_example' ? createUrdfExampleRobot() : createMobileArmRobot()
const edges = computed(() => edgesOf(robot))
const jointAngles = reactive<Record<string, number>>(
  Object.fromEntries(Object.keys(robot.joints).map((name) => [name, robot.joints[name].angle])),
)

function retrace() {
  for (const name in jointAngles) robot.joints[name].angle = jointAngles[name]
  tracer.load(robot)
  if (smooth.value) while (tracer.stepForward()) { /* fast-forward to the fully-posed robot */ }
}

watch(jointAngles, retrace, { deep: true })
watch(smooth, retrace)
onMounted(retrace)

const canvasEl = ref<HTMLCanvasElement | null>(null)
function redraw() {
  const canvas = canvasEl.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  const rect = canvas.getBoundingClientRect()
  canvas.width = rect.width; canvas.height = rect.height
  drawFkTree(ctx, canvas.width, canvas.height, tracer.revealedLinks.value, edges.value, tracer.currentLinkName.value)
}
watch(() => tracer.revealedLinks.value, redraw)
onMounted(redraw)

const stepLabel = computed(() => {
  if (tracer.currentJointName.value) return 'joint: ' + tracer.currentJointName.value
  if (tracer.currentLinkName.value) return 'link: ' + tracer.currentLinkName.value
  return ''
})
</script>

<template>
  <PhaseStepperShell
    :pseudocode-lines="MASTER_PSEUDOCODE" :active-line="tracer.activeLine.value"
    :is-running="tracer.isRunning.value" :is-done="tracer.isDone.value" :is-at-start="tracer.isAtStart.value"
    :smooth="smooth" :step-label="stepLabel"
    :frame-info="'step ' + Math.max(0, tracer.pos.value + 1) + ' / ' + tracer.stepCount.value"
    @play="tracer.play" @pause="tracer.pause" @step-forward="tracer.stepForward" @step-back="tracer.stepBack"
    @reset="tracer.reset" @update:smooth="(v: boolean) => (smooth = v)"
  >
    <template #controls>
      <label v-for="name in Object.keys(jointAngles)" :key="name" class="joint-slider">
        <span>{{ name }}</span>
        <input v-model.number="jointAngles[name]" type="range" min="-3.14159" max="3.14159" step="0.01">
      </label>
    </template>
    <div class="vector-canvas-wrap">
      <canvas ref="canvasEl" />
    </div>
  </PhaseStepperShell>
</template>

<style scoped>
.joint-slider { display: flex; align-items: center; gap: 0.4em; font-size: 0.72em; font-family: var(--font-mono, monospace); }
.joint-slider input[type="range"] { width: 6em; }
</style>
