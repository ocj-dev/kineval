<script setup lang="ts">
// Publishing a JointTrajectory versus running it through the action.
//
// The trajectory drawn here is the REAL one: the `traj` value the `plan`
// leaf wrote to the blackboard in the recorded plan_and_execute trace,
// read straight out of the JSON. Both columns animate the same message.
//
//   left  -- what /joint_trajectory does today. robot_sim_node.rs's
//            apply_trajectory keeps points.last() and nothing else, so
//            the arm teleports: one jump, intermediate points discarded.
//   right -- what the action does. joint_trajectory_action_node walks
//            the points in order, gating each on arrival within epsilon.
//
// This is the slide the plan calls "a far better slide than any
// diagram", and it is only convincing because the trajectory is the
// recorded one rather than something drawn to make the point.
import { ref, computed, onMounted, onBeforeUnmount } from 'vue'
import { TRACES } from '../lib/bt/trace'

// the trajectory the plan leaf produced, from the recorded trace
const traj = computed(() => {
  for (const t of TRACES.plan_and_execute.ticks) {
    const v = (t.blackboard || {}).traj as
      | { joint_names: string[]; points: Array<{ positions: number[]; time_from_start: { sec: number; nanosec: number } }> }
      | undefined
    if (v && v.points && v.points.length) return v
  }
  return null
})

const points = computed(() => traj.value ? traj.value.points : [])
const nJoints = computed(() => traj.value ? traj.value.joint_names.length : 0)

const t = ref(0)          // 0..1 along the execution
const playing = ref(false)
let raf: number | undefined
let t0 = 0
const DURATION = 4200

function frame(now: number) {
  if (!t0) t0 = now
  t.value = Math.min(1, (now - t0) / DURATION)
  if (t.value >= 1) { playing.value = false; raf = undefined; return }
  raf = requestAnimationFrame(frame)
}
function play() {
  if (playing.value) return
  if (t.value >= 1) t.value = 0
  t0 = 0; playing.value = true
  raf = requestAnimationFrame(frame)
}
function pause() { playing.value = false; if (raf) { cancelAnimationFrame(raf); raf = undefined } }
function reset() { pause(); t.value = 0; t0 = 0 }
onBeforeUnmount(pause)
onMounted(reset)

/** index of the point the ACTION is currently driving toward */
const actionIndex = computed(() =>
  Math.min(points.value.length - 1, Math.floor(t.value * points.value.length)))

/** the action interpolates between consecutive points as it walks them */
const actionPose = computed(() => {
  const p = points.value
  if (!p.length) return []
  const f = t.value * (p.length - 1)
  const i = Math.min(p.length - 2, Math.floor(f))
  const a = p[i].positions, b = p[Math.min(p.length - 1, i + 1)].positions
  const u = Math.min(1, f - i)
  return a.map((v, k) => v + u * (b[k] - v))
})

/** the topic publishes once and the sim jumps straight to the last point */
const teleportPose = computed(() => {
  const p = points.value
  if (!p.length) return []
  return t.value > 0.02 ? p[p.length - 1].positions : p[0].positions
})

const SPAN = Math.PI * 1.25
function bar(v: number) { return 50 + (v / SPAN) * 50 }
</script>

<template>
  <div class="tj" v-if="traj">
    <div class="tj-col">
      <div class="tj-head tj-bad">publish <code>/joint_trajectory</code></div>
      <div class="tj-sub">robot_sim_node.rs keeps <code>points.last()</code></div>
      <div class="tj-joints">
        <div v-for="(name, j) in traj.joint_names" :key="j" class="tj-joint">
          <span class="tj-name">{{ name.replace('_joint', '') }}</span>
          <div class="tj-track"><div class="tj-fill bad" :style="{ left: Math.min(bar(teleportPose[j]), 50) + '%', width: Math.abs(bar(teleportPose[j]) - 50) + '%' }"></div></div>
        </div>
      </div>
      <div class="tj-state bad">
        {{ t > 0.02 ? 'arrived — in one frame' : 'start' }}
        <span class="tj-count">1 of {{ points.length }} points used</span>
      </div>
    </div>

    <div class="tj-col">
      <div class="tj-head tj-good">send a goal to the <b>action</b></div>
      <div class="tj-sub">joint_trajectory_action_node walks every point</div>
      <div class="tj-joints">
        <div v-for="(name, j) in traj.joint_names" :key="j" class="tj-joint">
          <span class="tj-name">{{ name.replace('_joint', '') }}</span>
          <div class="tj-track"><div class="tj-fill good" :style="{ left: Math.min(bar(actionPose[j]), 50) + '%', width: Math.abs(bar(actionPose[j]) - 50) + '%' }"></div></div>
        </div>
      </div>
      <div class="tj-state good">
        {{ t >= 1 ? 'result: succeeded' : 'feedback: executing' }}
        <span class="tj-count">point {{ actionIndex + 1 }} of {{ points.length }}</span>
      </div>
    </div>

    <div class="tj-foot">
      <button class="btn" @click="playing ? pause() : play()">{{ playing ? 'Pause' : 'Play' }}</button>
      <button class="btn" @click="reset()">Reset</button>
      <span class="tj-meta">
        the real trajectory from the recorded trace &middot;
        {{ nJoints }} joints &middot; {{ points.length }} points &middot;
        commit {{ TRACES.plan_and_execute.header.commit.slice(0, 10) }}
      </span>
    </div>
  </div>
</template>

<style scoped>
.tj { display: grid; grid-template-columns: 1fr 1fr; grid-template-rows: 1fr auto; gap: 0.8em; height: 100%; min-height: 0; }
.tj-col { display: flex; flex-direction: column; gap: 0.2em; min-height: 0; }
.tj-head { font-family: var(--font-mono, monospace); font-size: 0.64em; font-weight: 700; }
.tj-bad { color: #c5221f; }
.tj-good { color: #188038; }
.tj-sub { font-size: 0.56em; opacity: 0.65; margin-bottom: 0.35em; }
.tj-joints { display: flex; flex-direction: column; gap: 0.28em; }
.tj-joint { display: grid; grid-template-columns: 7.5em 1fr; gap: 0.5em; align-items: center; }
.tj-name { font-family: var(--font-mono, monospace); font-size: 0.55em; opacity: 0.8; }
.tj-track { position: relative; height: 0.85em; background: #eef0f4; border-radius: 999px; overflow: hidden; }
.tj-track::after { content: ''; position: absolute; left: 50%; top: 0; bottom: 0; width: 1px; background: #c7cad1; }
.tj-fill { position: absolute; top: 0; bottom: 0; border-radius: 999px; }
.tj-fill.bad { background: #c5221f; }
.tj-fill.good { background: #188038; transition: none; }
.tj-state {
  font-family: var(--font-mono, monospace); font-size: 0.58em; font-weight: 700;
  margin-top: auto; padding: 0.3em 0.6em; border-radius: 6px;
  display: flex; justify-content: space-between; align-items: baseline;
}
.tj-state.bad { background: #fce8e6; color: #a8130f; }
.tj-state.good { background: #e6f4ea; color: #0d652d; }
.tj-count { font-weight: 400; opacity: 0.75; }
.tj-foot { grid-column: 1 / -1; display: flex; align-items: center; gap: 0.4em; }
.tj-meta { font-family: var(--font-mono, monospace); font-size: 0.55em; color: #5f6368; }
</style>
