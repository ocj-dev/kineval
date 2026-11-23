<script setup lang="ts">
// The formalism, from scratch, because no lecture in this course covers
// behavior trees.
//
// A tiny tree whose LEAF RESULTS THE READER SETS, so sequence and
// selector semantics are discovered rather than asserted. Change a leaf
// to Failure and watch a sequence stop at it; change the same leaf under
// a selector and watch the selector move on.
//
// The tick walk here is the same contract bt_node.rs implements -- visit
// children in order, stop on the decisive result, propagate it upward --
// written in twenty lines so it can be read beside the Rust.
import { ref, computed } from 'vue'

type Result = 'success' | 'failure' | 'running'

const composite = ref<'sequence' | 'selector'>('sequence')
const leaves = ref<Result[]>(['success', 'failure', 'success'])
const names = ['approach', 'grasp', 'lift']
const cycle: Result[] = ['success', 'failure', 'running']

function bump(i: number) {
  const cur = leaves.value[i]
  leaves.value[i] = cycle[(cycle.indexOf(cur) + 1) % cycle.length]
}

/**
 * One tick of the composite, recording which children were visited.
 *
 * sequence: run children in order; the FIRST non-success stops the walk
 *           and becomes the result. All success -> Success.
 * selector: run children in order; the FIRST non-failure stops the walk
 *           and becomes the result. All failure -> Failure.
 *
 * `Running` is decisive for BOTH: the tick stops there and returns
 * Running, so the tree never blocks waiting for a child to finish.
 */
const walk = computed(() => {
  const visited: { i: number; result: Result }[] = []
  let root: Result = composite.value === 'sequence' ? 'success' : 'failure'
  for (let i = 0; i < leaves.value.length; i++) {
    const r = leaves.value[i]
    visited.push({ i, result: r })
    const decisive = composite.value === 'sequence' ? r !== 'success' : r !== 'failure'
    if (decisive) { root = r; break }
  }
  return { visited, root }
})

const visitedSet = computed(() => new Set(walk.value.visited.map((v) => v.i)))
const explanation = computed(() => {
  const w = walk.value
  const stopped = w.visited[w.visited.length - 1]
  const all = w.visited.length === leaves.value.length
  const decisive = composite.value === 'sequence' ? 'non-Success' : 'non-Failure'
  if (all && !((composite.value === 'sequence') ? stopped.result !== 'success' : stopped.result !== 'failure')) {
    return `Every child was visited and none was ${decisive}, so the ${composite.value} returns ${w.root}.`
  }
  return `${names[stopped.i]} returned ${stopped.result} — the first ${decisive} result — ` +
    `so the walk stops there and the ${composite.value} returns ${w.root}. ` +
    `The remaining ${leaves.value.length - w.visited.length} child(ren) are not ticked at all.`
})
</script>

<template>
  <div class="sem">
    <div class="sem-left">
      <div class="sem-switch">
        <button class="btn" :class="{ on: composite === 'sequence' }" @click="composite = 'sequence'">sequence</button>
        <button class="btn" :class="{ on: composite === 'selector' }" @click="composite = 'selector'">selector</button>
      </div>

      <div class="sem-root" :class="'r-' + walk.root">
        {{ composite }} → {{ walk.root }}
      </div>

      <div
        v-for="(r, i) in leaves" :key="i"
        class="sem-leaf" :class="['r-' + r, { skipped: !visitedSet.has(i) }]"
        @click="bump(i)"
      >
        <span class="sem-name">{{ names[i] }}</span>
        <span class="sem-res">{{ r }}</span>
        <span class="sem-tag">{{ visitedSet.has(i) ? 'ticked' : 'not ticked' }}</span>
      </div>
      <div class="sem-hint">click a child to change what it returns</div>
    </div>

    <div class="sem-right">
      <div class="sem-rule">
        <b>sequence</b> — an AND. Visit children in order; the first
        <i>non-Success</i> stops the walk and becomes the result.
      </div>
      <div class="sem-rule">
        <b>selector</b> — an OR, and a priority list. Visit children in
        order; the first <i>non-Failure</i> stops the walk and becomes the
        result.
      </div>
      <div class="sem-rule emph">
        <b>Running is decisive for both.</b> A child that is still working
        stops the tick where it is. Nothing below it runs, nothing after it
        runs, and the tick returns immediately — which is what keeps a tree
        responsive.
      </div>
      <div class="sem-now">{{ explanation }}</div>
    </div>
  </div>
</template>

<style scoped>
.sem { display: grid; grid-template-columns: 1fr 1fr; gap: 0.9em; height: 100%; min-height: 0; }
.sem-left, .sem-right { display: flex; flex-direction: column; gap: 0.35em; min-height: 0; }
.sem-switch { display: flex; gap: 0.3em; margin-bottom: 0.2em; }
.sem-switch .btn { font-family: var(--font-mono, monospace); font-size: 0.72em; }
.sem-switch .btn.on { background: var(--amber, #E8710A); color: #fff; border-color: var(--amber, #E8710A); }

.sem-root {
  font-family: var(--font-mono, monospace); font-weight: 700; font-size: 0.72em;
  border-radius: 7px; padding: 0.3em 0.7em; border: 1px solid #dcdfe6;
}
.sem-leaf {
  display: grid; grid-template-columns: 1fr auto auto; gap: 0.6em; align-items: baseline;
  font-family: var(--font-mono, monospace); font-size: 0.65em; cursor: pointer;
  border: 1px solid #dcdfe6; border-radius: 7px; padding: 0.28em 0.7em; margin-left: 1.4em;
  user-select: none;
}
.sem-leaf:hover { border-color: var(--amber, #E8710A); }
.sem-name { font-weight: 600; }
.sem-res { font-weight: 700; }
.sem-tag { font-size: 0.8em; opacity: 0.6; min-width: 5.5em; text-align: right; }
.skipped { opacity: 0.4; border-style: dashed; }
.r-success { background: #e6f4ea; border-color: #188038; }
.r-failure { background: #fce8e6; border-color: #c5221f; }
.r-running { background: #fff4e5; border-color: #e8a33d; }
.sem-hint { font-size: 0.58em; opacity: 0.55; margin-top: 0.2em; }

.sem-rule {
  font-size: 0.68em; line-height: 1.4; border: 1px solid #dcdfe6;
  border-radius: 8px; padding: 0.45em 0.7em; background: #fff;
}
.sem-rule.emph { border-left: 3px solid var(--amber, #E8710A); background: #fff7ef; }
.sem-now {
  font-size: 0.66em; line-height: 1.4; background: #eef0f4;
  border-radius: 8px; padding: 0.5em 0.7em; margin-top: auto;
}
</style>
