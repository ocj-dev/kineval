<script setup lang="ts">
import { ref, computed } from 'vue'

// The same picking task written twice -- as a behavior tree and as a
// finite-state machine -- with a switch that applies the SAME change to
// both: check the grasp alignment before closing the gripper, and correct
// it if it is off.
//
// This is Castro's modularity argument, made countable rather than
// asserted. The counts below are derived from the two structures, not
// typed in: adding a step to the tree inserts one subtree and leaves
// every existing node alone, while adding it to the machine adds a state
// and rewires transitions that already existed. Neither is "better" in
// the abstract -- the FSM's rewiring is also what gives it the direct,
// any-state-to-any-state transitions that make it the easier of the two
// to make reactive, which is the next slide.

const modified = ref(false)

// --- the behavior tree, as (depth, label, kind) rows --------------------
interface BtRow { depth: number; label: string; kind: 'sequence' | 'fallback' | 'action' | 'condition'; added?: boolean }

const btRows = computed<BtRow[]>(() => {
  const rows: BtRow[] = [
    { depth: 0, label: 'Pick the object', kind: 'sequence' },
    { depth: 1, label: 'Move to the object', kind: 'action' },
  ]
  if (modified.value) {
    rows.push({ depth: 1, label: 'Grasp aligned', kind: 'fallback', added: true })
    rows.push({ depth: 2, label: 'Aligned?', kind: 'condition', added: true })
    rows.push({ depth: 2, label: 'Correct the pre-grasp', kind: 'action', added: true })
  }
  rows.push({ depth: 1, label: 'Close the gripper', kind: 'action' })
  rows.push({ depth: 1, label: 'Return home', kind: 'action' })
  return rows
})

// --- the state machine --------------------------------------------------
interface FsmState { id: string; label: string; added?: boolean }
interface FsmEdge { from: string; to: string; label: string; added?: boolean; changed?: boolean }

const fsmStates = computed<FsmState[]>(() => {
  const states: FsmState[] = [
    { id: 'move', label: 'MoveToObject' },
  ]
  if (modified.value) {
    states.push({ id: 'check', label: 'CheckAlignment', added: true })
    states.push({ id: 'correct', label: 'CorrectPreGrasp', added: true })
  }
  states.push({ id: 'close', label: 'CloseGripper' })
  states.push({ id: 'home', label: 'ReturnHome' })
  return states
})

const fsmEdges = computed<FsmEdge[]>(() => {
  if (!modified.value) {
    return [
      { from: 'move', to: 'close', label: 'arrived' },
      { from: 'close', to: 'home', label: 'closed' },
    ]
  }
  return [
    { from: 'move', to: 'check', label: 'arrived', changed: true },
    { from: 'check', to: 'close', label: 'aligned', added: true },
    { from: 'check', to: 'correct', label: 'not aligned', added: true },
    { from: 'correct', to: 'check', label: 'corrected', added: true },
    { from: 'close', to: 'home', label: 'closed' },
  ]
})

const btChanges = computed(() => btRows.value.filter((r) => r.added).length)
const fsmChanges = computed(() =>
  fsmStates.value.filter((s) => s.added).length +
  fsmEdges.value.filter((e) => e.added || e.changed).length)
</script>

<template>
  <div class="cmp-shell">
    <label class="toggle">
      <input v-model="modified" type="checkbox">
      add a pre-grasp alignment check before closing the gripper
    </label>

    <div class="columns">
      <div class="column">
        <div class="col-head">behavior tree</div>
        <div class="bt">
          <div
            v-for="(row, i) in btRows" :key="i"
            class="bt-row" :class="[row.kind, { added: row.added }]"
            :style="{ marginLeft: row.depth * 1.6 + 'em' }"
          >
            <span class="glyph">{{ row.kind === 'sequence' ? '→' : row.kind === 'fallback' ? '?' : row.kind === 'condition' ? '○' : '▭' }}</span>
            {{ row.label }}
          </div>
        </div>
      </div>

      <div class="column">
        <div class="col-head">finite-state machine</div>
        <div class="fsm">
          <div v-for="s in fsmStates" :key="s.id" class="fsm-state" :class="{ added: s.added }">
            {{ s.label }}
          </div>
          <div class="edges">
            <div v-for="(e, i) in fsmEdges" :key="i" class="edge" :class="{ added: e.added, changed: e.changed }">
              {{ e.from }} &rarr; {{ e.to }}<span class="edge-label">&nbsp;on {{ e.label }}</span>
            </div>
          </div>
        </div>
      </div>
    </div>

    <p class="verdict">
      <template v-if="modified">
        The tree gained <b>{{ btChanges }} nodes</b> inserted at one point and changed nothing that was
        already there. The machine gained <b>{{ fsmChanges }} states and transitions</b>, one of which
        is an existing transition redirected &mdash; <code>move &rarr; close</code> no longer exists.
      </template>
      <template v-else>
        Both are clean, and both are easy to read. Turn the switch on to make the same change to each.
      </template>
    </p>
  </div>
</template>

<style scoped>
.cmp-shell { display: flex; flex-direction: column; height: 100%; min-height: 0; gap: 0.5em; }
.toggle { font-size: 0.62em; font-family: var(--font-mono, monospace); }
.columns { display: flex; gap: 1em; flex: 1 1 auto; min-height: 0; }
.column { flex: 1 1 50%; display: flex; flex-direction: column; gap: 0.3em; }
.col-head { font-size: 0.56em; opacity: 0.65; font-family: var(--font-mono, monospace); }
.bt-row, .fsm-state, .edge {
  font-size: 0.6em; font-family: var(--font-mono, monospace);
  border: 1.2px solid #c7cad1; background: #fff; border-radius: 6px;
  padding: 0.18em 0.6em; margin-bottom: 0.22em; display: inline-block;
}
.bt-row { display: block; }
.bt-row.sequence { border-color: #3b6ea5; }
.bt-row.fallback { border-color: #d98236; }
.bt-row.condition { border-radius: 999px; }
.glyph { font-weight: 700; margin-right: 0.4em; }
.fsm-state { display: inline-block; margin-right: 0.3em; border-color: #27966b; }
.edges { margin-top: 0.4em; }
.edge { display: block; border: none; background: none; padding: 0.05em 0; opacity: 0.85; }
.edge-label { opacity: 0.6; }
.added { background: #e3f3ea; border-color: #27966b; font-weight: 700; }
.edge.added { background: none; color: #1d6e4f; }
.edge.changed { background: none; color: #a3302a; font-weight: 700; }
.verdict { margin: 0; font-size: 0.62em; }
</style>
