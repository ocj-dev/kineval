<script setup lang="ts">
import { ref, computed } from 'vue'
import {
  BT_FAILURE, BT_RUNNING, BT_SUCCESS, btAction, btDecorator, btFallback, btParallel, btSequence, btTickTree, btBlackboard,
} from '../lib/bt/behaviorTree'
import type { NodeKind, Status } from '../lib/bt/types'

// Set each child's status by hand and see what the control node returns,
// and -- just as important -- which children were ticked at all.
//
// Driven by the real tick functions from lib/bt/behaviorTree.ts, which
// are a port of the reference implementation this deck quotes. The table
// is therefore not a table of what the semantics are supposed to be; it
// is the output of the code on the facing slide.
const props = withDefaults(defineProps<{
  kind: NodeKind
  childCount?: number
  threshold?: number
}>(), { childCount: 3, threshold: 1 })

const CYCLE: Status[] = [BT_SUCCESS, BT_FAILURE, BT_RUNNING]
const childStatuses = ref<Status[]>(
  Array.from({ length: props.childCount }, (_, i) => (i === 0 ? BT_SUCCESS : BT_FAILURE)),
)
const threshold = ref(props.threshold)

function cycle(i: number) {
  const next = (CYCLE.indexOf(childStatuses.value[i]) + 1) % CYCLE.length
  childStatuses.value = childStatuses.value.map((s, j) => (j === i ? CYCLE[next] : s))
}

const run = computed(() => {
  const ticked: boolean[] = childStatuses.value.map(() => false)
  const children = childStatuses.value.map((status, i) =>
    btAction('child ' + (i + 1), { step: () => { ticked[i] = true; return status } }))

  let root
  if (props.kind === 'sequence') root = btSequence('sequence', children)
  else if (props.kind === 'fallback') root = btFallback('fallback', children)
  else if (props.kind === 'parallel') root = btParallel('parallel', children, threshold.value)
  else root = btDecorator('decorator', 'invert', children[0])

  const result = btTickTree(root, btBlackboard({}))
  return { status: result.status, ticked }
})

const explanation = computed(() => {
  switch (props.kind) {
    case 'sequence': return 'stops at the first Failure or Running; every child after it is never ticked'
    case 'fallback': return 'stops at the first Success or Running; the children after it are the untried recoveries'
    case 'parallel': return `ticks every child, every tick; Success at ${threshold.value} success(es), Failure once ${threshold.value} is out of reach`
    default: return 'one child, one policy -- this one inverts Success and Failure, and passes Running through'
  }
})
</script>

<template>
  <div class="kind-shell">
    <div class="row">
      <div class="children">
        <div class="col-head">children (click to change what each returns)</div>
        <button
          v-for="(s, i) in childStatuses" :key="i"
          class="child" :class="[s, { untouched: !run.ticked[i] }]"
          @click="cycle(i)"
          :disabled="kind === 'decorator' && i > 0"
        >
          <span class="child-name">child {{ i + 1 }}</span>
          <span class="child-status">{{ s }}</span>
          <span class="child-ticked">{{ run.ticked[i] ? 'ticked' : 'not ticked' }}</span>
        </button>
      </div>

      <div class="result">
        <div class="col-head">the {{ kind }} returns</div>
        <div class="verdict" :class="run.status">{{ run.status }}</div>
        <label v-if="kind === 'parallel'" class="threshold">
          success threshold M =
          <input v-model.number="threshold" type="range" min="1" :max="childCount" step="1">
          <b>{{ threshold }}</b> of {{ childCount }}
          <span class="aside">{{ threshold === childCount ? '(an "and")' : threshold === 1 ? '(an "or")' : '' }}</span>
        </label>
      </div>
    </div>

    <p class="explain">{{ explanation }}</p>
  </div>
</template>

<style scoped>
.kind-shell { display: flex; flex-direction: column; height: 100%; min-height: 0; gap: 0.5em; }
.row { display: flex; gap: 1em; align-items: flex-start; flex: 1 1 auto; min-height: 0; }
.children { display: flex; flex-direction: column; gap: 0.35em; flex: 1 1 auto; }
.result { flex: 0 0 11em; display: flex; flex-direction: column; gap: 0.4em; }
.col-head { font-size: 0.56em; opacity: 0.65; font-family: var(--font-mono, monospace); }
.child {
  display: flex; align-items: center; gap: 0.7em; font: inherit; font-size: 0.62em;
  font-family: var(--font-mono, monospace); text-align: left; cursor: pointer;
  border: 1.4px solid #c7cad1; background: #fff; border-radius: 6px; padding: 0.3em 0.7em;
}
.child:disabled { cursor: default; opacity: 0.35; }
.child-name { flex: 0 0 5em; }
.child-status { font-weight: 700; flex: 0 0 5em; }
.child-ticked { opacity: 0.6; }
.child.untouched { opacity: 0.45; border-style: dashed; }
.child.success { border-color: #27966b; background: #e3f3ea; }
.child.failure { border-color: #c0392b; background: #fdecd9; }
.child.running { border-color: #3b6ea5; background: #dce9f8; }
.verdict {
  font-family: var(--font-mono, monospace); font-weight: 700; font-size: 0.85em;
  border: 2px solid #c7cad1; border-radius: 8px; padding: 0.45em 0.6em; text-align: center;
}
.verdict.success { border-color: #27966b; background: #e3f3ea; color: #1d6e4f; }
.verdict.failure { border-color: #c0392b; background: #fdecd9; color: #a3302a; }
.verdict.running { border-color: #3b6ea5; background: #dce9f8; color: #2d5580; }
.threshold { font-size: 0.58em; font-family: var(--font-mono, monospace); display: block; }
.threshold input { width: 100%; }
.aside { opacity: 0.65; }
.explain { margin: 0; font-size: 0.62em; opacity: 0.8; }
</style>
