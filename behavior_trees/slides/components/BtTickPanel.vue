<script setup lang="ts">
// The panel the whole deck is built around: a behavior tree ticking,
// with the active node highlighted and THE BLACKBOARD CONTENTS SHOWN
// CHANGING.
//
// Replayed from a recorded trace -- not simulated. Each tick in
// ../../reference/traces/ was emitted by tests/bt_node_motion.rs in
// pubsub_testbed_rs and carries the commit it was recorded at. Stepping
// is index navigation over that recording, which is why Play / Pause /
// Step / Reset are four lines each.
import { ref, computed, onMounted, onBeforeUnmount, watch } from 'vue'
import { TRACES, flatten, resultAt, onRunningPath, summariseValue } from '../lib/bt/trace'

const props = withDefaults(defineProps<{
  scenario?: 'plan_and_execute' | 'preempt'
  speed?: number
  autoplay?: boolean
}>(), { scenario: 'plan_and_execute', speed: 850, autoplay: false })

const index = ref(0)
const playing = ref(false)
let timer: number | undefined

const trace = computed(() => TRACES[props.scenario])
const nodes = computed(() => flatten(trace.value.header.tree))
const tick = computed(() => trace.value.ticks[index.value])
const total = computed(() => trace.value.ticks.length)

const blackboardRows = computed(() => {
  const bb = tick.value.blackboard || {}
  return Object.keys(bb).map((k) => ({ key: k, value: summariseValue(bb[k]) }))
})

/** which blackboard keys appeared or vanished since the previous tick */
const bbDelta = computed(() => {
  if (index.value === 0) return { added: [] as string[], removed: [] as string[] }
  const prev = trace.value.ticks[index.value - 1].blackboard_keys || []
  const now = tick.value.blackboard_keys || []
  return {
    added: now.filter((k) => prev.indexOf(k) < 0),
    removed: prev.filter((k) => now.indexOf(k) < 0),
  }
})

function cls(path: number[]) {
  const r = resultAt(tick.value, path)
  return {
    'bt-running': r === 'running',
    'bt-success': r === 'success',
    'bt-failure': r === 'failure',
    'bt-unvisited': r === null,
    'bt-active': onRunningPath(tick.value, path) && r === 'running',
  }
}
function resultLabel(path: number[]) {
  const r = resultAt(tick.value, path)
  return r === null ? '·' : r
}

function step(d = 1) {
  index.value = Math.max(0, Math.min(total.value - 1, index.value + d))
  if (index.value >= total.value - 1) pause()
}
function play() {
  if (playing.value) return
  if (index.value >= total.value - 1) index.value = 0
  playing.value = true
  timer = window.setInterval(() => step(1), props.speed)
}
function pause() {
  playing.value = false
  if (timer !== undefined) { window.clearInterval(timer); timer = undefined }
}
function reset() { pause(); index.value = 0 }

watch(() => props.scenario, () => { pause(); index.value = 0 })
onMounted(() => { if (props.autoplay) play() })
onBeforeUnmount(pause)
</script>

<template>
  <div class="btp">
    <div class="btp-tree">
      <div class="btp-head">the tree, as loaded</div>
      <div
        v-for="n in nodes" :key="n.row"
        class="bt-node" :class="cls(n.path)"
        :style="{ marginLeft: (n.depth * 1.5) + 'em' }"
      >
        <span class="bt-kind">{{ n.label }}</span>
        <span class="bt-result">{{ resultLabel(n.path) }}</span>
      </div>

      <div class="btp-head mt">root returned</div>
      <div class="bt-root" :class="'bt-' + tick.root_result">{{ tick.root_result }}</div>
    </div>

    <div class="btp-side">
      <div class="btp-head">the blackboard</div>
      <div class="bb">
        <div v-if="!blackboardRows.length" class="bb-empty">— empty —</div>
        <div v-for="r in blackboardRows" :key="r.key" class="bb-row"
             :class="{ 'bb-new': bbDelta.added.indexOf(r.key) >= 0 }">
          <span class="bb-key">{{ r.key }}</span>
          <span class="bb-val">{{ r.value }}</span>
        </div>
        <div v-for="k in bbDelta.removed" :key="'rm-' + k" class="bb-row bb-gone">
          <span class="bb-key">{{ k }}</span>
          <span class="bb-val">cleared</span>
        </div>
      </div>

      <div class="btp-controls">
        <button class="btn" @click="playing ? pause() : play()">{{ playing ? 'Pause' : 'Play' }}</button>
        <button class="btn" @click="pause(); step(-1)">&larr;</button>
        <button class="btn" @click="pause(); step(1)">Step &rarr;</button>
        <button class="btn" @click="reset()">Reset</button>
      </div>
      <div class="btp-meta">
        <div><b>tick {{ tick.tick }}</b> &middot; {{ index + 1 }}/{{ total }} recorded</div>
        <div class="btp-prov">{{ trace.header.scenario }} &middot; {{ trace.header.produced_by }}</div>
        <div class="btp-prov">commit {{ trace.header.commit.slice(0, 10) }} &middot; {{ trace.header.date }}</div>
      </div>
    </div>
  </div>
</template>

<style scoped>
.btp { display: grid; grid-template-columns: 1.35fr 1fr; gap: 0.8em; height: 100%; min-height: 0; }
.btp-tree, .btp-side { display: flex; flex-direction: column; min-height: 0; gap: 0.2em; }
.btp-head {
  font-family: var(--font-mono, monospace); font-size: 0.56em; font-weight: 700;
  text-transform: uppercase; letter-spacing: 0.06em; color: #5f6368;
}
.btp-head.mt { margin-top: 0.5em; }

.bt-node {
  display: flex; justify-content: space-between; align-items: center;
  font-family: var(--font-mono, monospace); font-size: 0.62em;
  border-radius: 6px; padding: 0.22em 0.6em; margin-bottom: 0.15em;
  border: 1px solid #dcdfe6; background: #fff;
}
.bt-kind { font-weight: 600; }
.bt-result { font-size: 0.85em; opacity: 0.8; }
.bt-unvisited { opacity: 0.4; }
.bt-running { background: #fff4e5; border-color: #e8a33d; }
.bt-success { background: #e6f4ea; border-color: #188038; }
.bt-failure { background: #fce8e6; border-color: #c5221f; }
.bt-active { box-shadow: 0 0 0 2px var(--amber, #E8710A); }

.bt-root {
  font-family: var(--font-mono, monospace); font-size: 0.7em; font-weight: 700;
  border-radius: 6px; padding: 0.25em 0.7em; display: inline-block; align-self: flex-start;
}

.bb {
  flex: 1 1 auto; min-height: 0; overflow-y: auto;
  border: 1px solid #dcdfe6; border-radius: 8px; background: #fff; padding: 0.4em 0.5em;
}
.bb-empty { font-family: var(--font-mono, monospace); font-size: 0.6em; opacity: 0.5; }
.bb-row {
  display: flex; gap: 0.5em; font-family: var(--font-mono, monospace); font-size: 0.58em;
  padding: 0.2em 0.3em; border-radius: 5px; align-items: baseline;
}
.bb-key { font-weight: 700; color: var(--indigo, #202B4E); min-width: 3.5em; }
.bb-val { opacity: 0.85; }
.bb-new { background: #e6f4ea; }
.bb-gone { background: #fce8e6; text-decoration: line-through; opacity: 0.75; }

.btp-controls { display: flex; gap: 0.3em; flex-wrap: wrap; margin-top: 0.4em; }
.btp-meta { font-family: var(--font-mono, monospace); font-size: 0.55em; color: #5f6368; margin-top: 0.35em; }
.btp-prov { opacity: 0.7; }
</style>
