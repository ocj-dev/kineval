<script setup lang="ts">
// Subsumption's if-else cascade and a behavior-tree priority selector,
// side by side, driven by the same conditions.
//
// The left column is the course's own slide: autorob_2026_09_fsm_
// subsumption.pdf p.70 draws Brooks's layered diagram beside exactly
// this cascade. The right column is the same arbitration written as a
// selector. Toggle a level's condition and watch both pick the same
// winner -- because they ARE the same arbitration.
//
// The point of the panel is the difference that remains once that is
// established: where the priority order is written down. In the cascade
// it is the order of the `else if`s. In the tree it is the order of the
// children, which is data the tree can report at runtime -- and in
// bt_node.rs it is exactly what /bt/status publishes.
import { ref, computed } from 'vue'

const LEVELS = [
  { level: 3, name: 'seek and grab object', cond: 'object_visible' },
  { level: 2, name: 'wander', cond: 'path_clear' },
  { level: 1, name: 'avoid obstacles', cond: 'obstacle_near' },
  { level: 0, name: 'halt', cond: '(always)' },
]

// index 3 is level 0, the default layer, always true
const active = ref<boolean[]>([false, false, true, true])

const winner = computed(() => active.value.findIndex((a) => a))
</script>

<template>
  <div class="sub">
    <div class="sub-col">
      <div class="sub-head">subsumption &mdash; arbitration in the wiring</div>
      <div class="sub-note">the course's own slide, p.70</div>
      <pre class="sub-code"><code><span
        v-for="(l, i) in LEVELS" :key="i"
        :class="{ hot: winner === i, cold: winner !== i }"
      >{{ i === 0 ? 'if (' + l.cond + ')' : (i === LEVELS.length - 1 ? 'else' : 'else if (' + l.cond + ')') }}
  u &#8592; level{{ l.level }}_control
</span></code></pre>
      <div class="sub-why">
        Priority is the <b>order of the branches</b>, and the arbitration
        between layers is wired into the network &mdash; suppressor and
        inhibitor nodes on the connections themselves.
      </div>
    </div>

    <div class="sub-col">
      <div class="sub-head">behavior tree &mdash; arbitration in the structure</div>
      <div class="sub-note">a priority <code>selector</code></div>
      <div class="sub-tree">
        <div class="sub-sel" :class="{ hot: winner >= 0 }">selector</div>
        <div
          v-for="(l, i) in LEVELS" :key="i"
          class="sub-child"
          :class="{ hot: winner === i, skipped: winner >= 0 && i > winner, tried: winner >= 0 && i < winner }"
          @click="i < LEVELS.length - 1 && (active[i] = !active[i])"
        >
          <span class="sub-name">{{ l.name }}</span>
          <span class="sub-res">{{
            winner === i ? 'Running' : (winner >= 0 && i < winner ? 'Failure' : 'not ticked')
          }}</span>
        </div>
      </div>
      <div class="sub-why">
        Priority is the <b>order of the children</b>. Each child that
        cannot run returns <code>Failure</code> and the selector moves on
        &mdash; so the arbitration is explicit, inspectable, and reported
        per tick on <code>/bt/status</code>.
      </div>
    </div>

    <div class="sub-foot">
      Click a behavior to flip its condition. Both columns always choose
      the same winner: <b>a priority selector is that cascade</b>. What
      differs is whether you can read the priority order off the running
      system &mdash; and whether adding a layer means re-wiring.
    </div>
  </div>
</template>

<style scoped>
.sub { display: grid; grid-template-columns: 1fr 1fr; grid-template-rows: 1fr auto; gap: 0.8em; height: 100%; min-height: 0; }
.sub-col { display: flex; flex-direction: column; gap: 0.25em; min-height: 0; }
.sub-head { font-family: var(--font-mono, monospace); font-size: 0.6em; font-weight: 700; color: var(--indigo, #202B4E); }
.sub-note { font-size: 0.56em; opacity: 0.6; margin-bottom: 0.2em; }

.sub-code {
  font-family: var(--font-mono, monospace); font-size: 0.6em; line-height: 1.45;
  background: #202124; color: #e8eaed; border-radius: 8px; padding: 0.6em 0.8em; margin: 0;
}
.sub-code .hot { color: #ffd34d; font-weight: 700; }
.sub-code .cold { opacity: 0.45; }

.sub-tree { display: flex; flex-direction: column; gap: 0.2em; }
.sub-sel {
  font-family: var(--font-mono, monospace); font-size: 0.64em; font-weight: 700;
  border: 1px solid #dcdfe6; border-radius: 6px; padding: 0.22em 0.6em; background: #fff;
}
.sub-sel.hot { background: #fff4e5; border-color: #e8a33d; }
.sub-child {
  display: flex; justify-content: space-between; align-items: baseline;
  font-family: var(--font-mono, monospace); font-size: 0.6em;
  border: 1px solid #dcdfe6; border-radius: 6px; padding: 0.22em 0.6em;
  margin-left: 1.3em; background: #fff; cursor: pointer; user-select: none;
}
.sub-child:hover { border-color: var(--amber, #E8710A); }
.sub-child.hot { background: #fff4e5; border-color: #e8a33d; font-weight: 700; }
.sub-child.tried { background: #fce8e6; border-color: #e3a9a6; }
.sub-child.skipped { opacity: 0.4; border-style: dashed; }
.sub-res { font-size: 0.85em; opacity: 0.8; }

.sub-why { font-size: 0.6em; line-height: 1.4; margin-top: 0.35em; }
.sub-foot {
  grid-column: 1 / -1; font-size: 0.62em; line-height: 1.4;
  background: #fff7ef; border-left: 3px solid var(--amber, #E8710A);
  border-radius: 0 8px 8px 0; padding: 0.5em 0.75em;
}
</style>
