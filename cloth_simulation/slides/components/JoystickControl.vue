<script setup lang="ts">
import { ref, computed } from 'vue'

const props = withDefaults(defineProps<{
  modelValue: { x: number; y: number }
  maxMagnitude?: number
  label?: string
}>(), { maxMagnitude: 1, label: 'Wind' })

const emit = defineEmits<{ 'update:modelValue': [{ x: number; y: number }] }>()

const baseEl = ref<HTMLDivElement | null>(null)
const dragging = ref(false)
const radius = 44

const knob = computed(() => ({
  x: (props.modelValue.x / props.maxMagnitude) * radius,
  y: (props.modelValue.y / props.maxMagnitude) * radius,
}))

function setFromEvent(e: PointerEvent) {
  if (!baseEl.value) return
  const rect = baseEl.value.getBoundingClientRect()
  const cx = rect.left + rect.width / 2
  const cy = rect.top + rect.height / 2
  let dx = e.clientX - cx
  let dy = e.clientY - cy
  const mag = Math.sqrt(dx * dx + dy * dy)
  if (mag > radius) { dx = (dx / mag) * radius; dy = (dy / mag) * radius }
  emit('update:modelValue', { x: (dx / radius) * props.maxMagnitude, y: (dy / radius) * props.maxMagnitude })
}

function onDown(e: PointerEvent) {
  dragging.value = true
  ;(e.target as HTMLElement).setPointerCapture(e.pointerId)
  setFromEvent(e)
}
function onMove(e: PointerEvent) { if (dragging.value) setFromEvent(e) }
function onUp() { dragging.value = false }
</script>

<template>
  <div class="joystick-wrap">
    <span class="joystick-label">{{ label }}</span>
    <div
      ref="baseEl" class="joystick-base"
      @pointerdown="onDown" @pointermove="onMove" @pointerup="onUp" @pointercancel="onUp"
    >
      <div class="joystick-knob" :style="{ transform: `translate(${knob.x}px, ${knob.y}px)` }" />
    </div>
  </div>
</template>

<style scoped>
.joystick-wrap {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.3em;
}
.joystick-label {
  font-family: var(--font-mono, monospace);
  font-size: 0.68em;
  color: var(--blue, #00274C);
  font-weight: 700;
}
.joystick-base {
  width: 100px;
  height: 100px;
  border-radius: 50%;
  background: #ffffffcc;
  border: 2px solid var(--blue, #00274C);
  position: relative;
  touch-action: none;
  cursor: grab;
}
.joystick-knob {
  position: absolute;
  left: 50%;
  top: 50%;
  width: 26px;
  height: 26px;
  margin-left: -13px;
  margin-top: -13px;
  border-radius: 50%;
  background: var(--maize, #FFCB05);
  border: 2px solid var(--blue, #00274C);
  pointer-events: none;
}
</style>
