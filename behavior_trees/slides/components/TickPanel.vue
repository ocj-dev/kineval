<script setup lang="ts">
import { ref, shallowRef, computed, onMounted, onBeforeUnmount } from 'vue'
import TreeDiagram from './TreeDiagram.vue'
import { btTickTree, btBlackboard } from '../lib/bt/behaviorTree'
import {
  treeSearchNaive, treeSearchExplicitSuccess, treeSearchMultipleLocations,
  treeSearchParallel, treeSearchWithDecorator,
} from '../lib/bt/trees'
import type { Blackboard, BtNode, Status, TraceEntry } from '../lib/bt/types'

// Ticks one of the search trees, one tick at a time, and draws what the
// tick did.
//
// These are the trees from Castro's object-searching example, running --
// not pictures of them. The apple is in the pantry and the kitchen has an
// orange, which is the arrangement that makes each tree in the
// progression behave differently from the last: the naive tree fails,
// the multi-location fallback succeeds on its second branch, and the
// 1-of-2 parallel succeeds on the orange alone.
const props = withDefaults(defineProps<{
  tree: 'naive' | 'explicit' | 'multiple' | 'parallel' | 'decorator'
  threshold?: number
  startLocation?: string
  speedMs?: number
  showLog?: boolean
}>(), { threshold: 1, startLocation: 'hallway', speedMs: 650, showLog: true })

const WORLD: Record<string, string[]> = {
  kitchen: ['orange'],
  pantry: ['apple'],
  garage: [],
}

function buildTree(): BtNode {
  switch (props.tree) {
    case 'naive': return treeSearchNaive()
    case 'explicit': return treeSearchExplicitSuccess()
    case 'multiple': return treeSearchMultipleLocations()
    case 'parallel': return treeSearchParallel(props.threshold)
    case 'decorator': return treeSearchWithDecorator(3)
  }
}

function buildBlackboard(): Blackboard {
  return btBlackboard({
    robot_location: props.startLocation,
    objects_at: WORLD,
    wanted_object: 'apple',
    location_queue: ['kitchen', 'pantry', 'garage'],
  })
}

const root = shallowRef<BtNode>(buildTree())
const blackboard = shallowRef<Blackboard>(buildBlackboard())
const trace = shallowRef<TraceEntry[]>([])
const status = ref<Status | null>(null)
const tickCount = ref(0)
const isRunning = ref(false)
let timer: ReturnType<typeof setTimeout> | null = null

const done = computed(() => status.value === 'success' || status.value === 'failure')
// `void tickCount.value` is the dependency that makes this re-evaluate:
// the log is an array mutated in place inside a shallowRef, so nothing
// about pushing to it is reactive on its own.
const logTail = computed(() => { void tickCount.value; return blackboard.value.log.slice(-2) })

function tickOnce() {
  if (done.value) return
  blackboard.value.dt = 0.05
  blackboard.value.now += 0.05
  const run = btTickTree(root.value, blackboard.value)
  trace.value = run.trace
  status.value = run.status
  tickCount.value++
  // a terminal status means the traversal is over; stop rather than
  // starting the whole tree again, so the final tick stays on screen
  if (done.value) pause()
}

function play() {
  if (isRunning.value || done.value) return
  isRunning.value = true
  const step = () => {
    tickOnce()
    if (isRunning.value && !done.value) timer = setTimeout(step, props.speedMs)
    else pause()
  }
  timer = setTimeout(step, 0)
}

function pause() {
  isRunning.value = false
  if (timer) { clearTimeout(timer); timer = null }
}

function reset() {
  pause()
  root.value = buildTree()
  blackboard.value = buildBlackboard()
  trace.value = []
  status.value = null
  tickCount.value = 0
}

// Tick once on mount. A panel showing an uncoloured tree and "not yet
// ticked" is a picture of a tree, which is the thing this panel exists not
// to be -- and on a slide nobody has clicked yet, that is all anyone sees.
onMounted(() => tickOnce())

onBeforeUnmount(pause)
</script>

<template>
  <div class="tick-shell">
    <div class="controls">
      <button class="btn" @click="isRunning ? pause() : play()" :disabled="done">
        {{ isRunning ? 'Pause' : 'Play' }}
      </button>
      <button class="btn" @click="pause(); tickOnce()" :disabled="done">Tick once</button>
      <button class="btn" @click="reset">&#8634; Reset</button>
      <span class="readout">tick {{ tickCount }}</span>
      <span class="status-tag" :class="status || 'idle'">
        {{ status ? 'root: ' + status : 'not yet ticked' }}
      </span>
      <span class="readout">robot: {{ blackboard.robot_location ?? 'in transit' }}</span>
    </div>

    <div class="diagram">
      <TreeDiagram :root="root" :trace="trace" />
    </div>

    <pre v-if="showLog" class="log">{{ logTail.join('\n') || 'press Tick once' }}</pre>
  </div>
</template>

<style scoped>
.tick-shell { display: flex; flex-direction: column; height: 100%; min-height: 0; gap: 0.35em; }
.controls {
  display: flex; flex-wrap: wrap; align-items: center; gap: 0.5em;
  font-size: 0.62em; font-family: var(--font-mono, monospace);
}
.readout { opacity: 0.75; }
.status-tag {
  font-weight: 700; border-radius: 999px; padding: 0.1em 0.6em;
  border: 1px solid #c7cad1; background: #fff;
}
.status-tag.success { background: #e3f3ea; border-color: #27966b; color: #1d6e4f; }
.status-tag.failure { background: #fdecd9; border-color: #c0392b; color: #a3302a; }
.status-tag.running { background: #dce9f8; border-color: #3b6ea5; color: #2d5580; }
.diagram { flex: 1 1 auto; min-height: 0; }
.log {
  margin: 0; font-size: 0.56em; line-height: 1.5; opacity: 0.8;
  font-family: var(--font-mono, monospace); white-space: pre-wrap;
  flex: 0 0 auto; max-height: 3.2em; overflow: hidden;
}
</style>
