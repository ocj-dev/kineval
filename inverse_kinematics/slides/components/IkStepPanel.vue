<script setup lang="ts">
import { reactive, ref, watch, onMounted, onBeforeUnmount, computed } from 'vue'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import PseudocodePanel from './PseudocodePanel.vue'
import { useStepTracer } from '../lib/ik/useStepTracer'
import { iterateIkSteps, runIkIteration, applyControls } from '../lib/ik/solveSteps'
import { buildFK, endEffectorWorldPosition } from '../lib/ik/fkForIk'
import { ROBOT_FACTORIES, CHAIN_JOINT_NAMES, INITIAL_ANGLES, DEFAULT_TARGETS, type RobotName } from '../lib/ik/robots'
import { MASTER_IK_LOOP } from '../lib/ik/pseudocode'
import type { IkMethod, IkParams, IterateIkStep, Mat4, RobotDesc } from '../lib/ik/types'

// Steps iterateIK ITSELF, one algorithm stage at a time -- gate check,
// error term dx, Jacobian, transpose, solve (method-dependent), scale +
// write .control -- distinct from JacobianBuildPanel.vue's per-JOINT-COLUMN
// walkthrough: that panel builds the Jacobian; this one picks up once a
// Jacobian already exists and walks the rest of iterateIK's own body
// (reference/inverse_kinematics.js#iterate-inverse-kinematics), via the
// iterateIkSteps generator (solveSteps.ts), which performs the identical
// sequence of matrix.js operations.
//
// "Run to convergence" (below) is a SEPARATE loop from the single-call
// step-through above: it repeatedly calls the full iterateIK-equivalent
// pipeline (runIkIteration + applyControls + buildFK -- same three-stage
// pipeline IkConvergePanel.vue's own animation loop runs, the exact
// TS port, not a re-derived approximation) once per animation frame, until
// |dx| drops under a convergence tolerance OR a max-iteration safety cap
// is hit. This deck's own "Test cases" slide already documents that
// urdf_example's 2-joint chain against a 3D target settles to a
// persistent nonzero residual rather than reaching zero -- so this control
// terminates honestly on the cap and reports which way it stopped, rather
// than silently pretending every run converges.

const props = withDefaults(defineProps<{ robotName?: RobotName }>(), { robotName: 'urdf_example' })
const robotName = props.robotName

const robot: RobotDesc = ROBOT_FACTORIES[robotName]()
for (const n in INITIAL_ANGLES[robotName]) robot.joints[n].angle = INITIAL_ANGLES[robotName][n]
buildFK(robot)

const method = ref<IkMethod>('transpose')
const target = DEFAULT_TARGETS[robotName]
const speedMs = ref(900)
const tracer = useStepTracer<IterateIkStep>(speedMs)

function currentIkParams(): IkParams {
  return { method: method.value, steplength: 0.3, damping: 0.15, orientationIncluded: false, chainJointNames: CHAIN_JOINT_NAMES[robotName] }
}

function retrace() {
  tracer.load([...iterateIkSteps(robot, target, null, currentIkParams(), true)])
}
watch(method, () => { stopConvergenceRun(); retrace() })

const fmt = (v: number) => (v >= 0 ? ' ' : '') + v.toFixed(3)

/* ---------------- "Run to convergence": loop full iterateIK calls ---------------- */
const CONVERGE_TOL = 0.01 // same |dx| threshold IkConvergePanel.vue's own "converged" tag uses
const MAX_CONVERGE_ITERS = 500 // honest safety cap -- urdf_example's 2-joint chain vs. a 3D target never reaches CONVERGE_TOL

const isConverging = ref(false)
const convergeIterCount = ref(0)
const convergeDxNorm = ref(0)
const convergeOutcome = ref<string | null>(null)
let convergeRaf = 0

function resetRobotPose() {
  for (const n in INITIAL_ANGLES[robotName]) robot.joints[n].angle = INITIAL_ANGLES[robotName][n]
  for (const n in robot.joints) robot.joints[n].control = 0
  buildFK(robot)
}

function stopConvergenceRun() {
  if (convergeRaf) cancelAnimationFrame(convergeRaf)
  convergeRaf = 0
  isConverging.value = false
}

function runToConvergence() {
  if (isConverging.value) return
  tracer.pause()
  stopConvergenceRun()
  resetRobotPose()
  syncScene()
  isConverging.value = true
  convergeIterCount.value = 0
  convergeDxNorm.value = Infinity
  convergeOutcome.value = null
  const ikParams = currentIkParams()

  const step = () => {
    const result = runIkIteration(robot, target, null, ikParams)
    applyControls(robot)
    buildFK(robot)
    let sumSq = 0
    for (const row of result.dx) sumSq += row[0] * row[0]
    const dxNorm = Math.sqrt(sumSq)
    convergeIterCount.value++
    convergeDxNorm.value = dxNorm
    syncScene()

    if (dxNorm < CONVERGE_TOL) {
      convergeOutcome.value = `converged in ${convergeIterCount.value} iterations (|dx| = ${dxNorm.toFixed(4)})`
      isConverging.value = false
      convergeRaf = 0
      return
    }
    if (convergeIterCount.value >= MAX_CONVERGE_ITERS) {
      convergeOutcome.value = `stopped after ${convergeIterCount.value} iterations -- residual still |dx| = ${dxNorm.toFixed(4)}`
      isConverging.value = false
      convergeRaf = 0
      return
    }
    convergeRaf = requestAnimationFrame(step)
  }
  convergeRaf = requestAnimationFrame(step)
}

function doReset() {
  stopConvergenceRun()
  convergeOutcome.value = null
  convergeIterCount.value = 0
  convergeDxNorm.value = 0
  resetRobotPose()
  tracer.reset()
  retrace()
  syncScene()
}

/* ---------------- small 3D context view ---------------- */
const viewport = ref<HTMLDivElement | null>(null)
let renderer: THREE.WebGLRenderer | null = null
let scene: THREE.Scene, camera: THREE.PerspectiveCamera, controls: OrbitControls
let raf = 0
const linkMeshes: Record<string, THREE.Mesh> = {}
let targetMarker: THREE.Mesh, eeMarker: THREE.Mesh, dxLine: THREE.Line

function mat4(m: Mat4) {
  return new THREE.Matrix4().set(
    m[0][0], m[0][1], m[0][2], m[0][3],
    m[1][0], m[1][1], m[1][2], m[1][3],
    m[2][0], m[2][1], m[2][2], m[2][3],
    m[3][0], m[3][1], m[3][2], m[3][3],
  )
}

function buildScene() {
  const el = viewport.value!
  scene = new THREE.Scene()
  scene.background = new THREE.Color(0xfbfbfa)
  camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100)
  camera.position.set(1.9, 1.6, 2.1)
  renderer = new THREE.WebGLRenderer({ antialias: true })
  renderer.setPixelRatio(window.devicePixelRatio || 1)
  el.appendChild(renderer.domElement)
  controls = new OrbitControls(camera, renderer.domElement)
  controls.enableDamping = true
  controls.target.set(0, 0.4, 0)
  scene.add(new THREE.HemisphereLight(0xffffff, 0x555555, 1.25))
  const sun = new THREE.DirectionalLight(0xffffff, 0.75)
  sun.position.set(3, 5, 2)
  scene.add(sun)
  scene.add(new THREE.GridHelper(6, 12, 0xc7cad1, 0xe2e4e8))

  for (const name in robot.links) {
    const g = robot.links[name].geometry!
    const geom = new THREE.BoxGeometry(g.size[0], g.size[1], g.size[2])
    geom.translate(g.offset.xyz[0], g.offset.xyz[1], g.offset.xyz[2])
    const mesh = new THREE.Mesh(geom, new THREE.MeshLambertMaterial({ color: robot.links[name].color ?? 0x8a8f98 }))
    mesh.matrixAutoUpdate = false
    scene.add(mesh)
    linkMeshes[name] = mesh
  }
  targetMarker = new THREE.Mesh(new THREE.SphereGeometry(0.05, 16, 16), new THREE.MeshLambertMaterial({ color: 0xe8349a }))
  scene.add(targetMarker)
  eeMarker = new THREE.Mesh(new THREE.SphereGeometry(0.045, 16, 16), new THREE.MeshLambertMaterial({ color: 0xe8710a }))
  scene.add(eeMarker)
  const dxGeom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()])
  dxLine = new THREE.Line(dxGeom, new THREE.LineBasicMaterial({ color: 0xe8710a }))
  scene.add(dxLine)

  const loop = () => { raf = requestAnimationFrame(loop); controls.update(); renderer!.render(scene, camera) }
  loop()
}

function syncScene() {
  for (const name in linkMeshes) {
    const m = robot.links[name].matrix as Mat4 | undefined
    if (m) linkMeshes[name].matrix.copy(mat4(m))
  }
  targetMarker.position.set(target[0], target[1], target[2])
  const ee = endEffectorWorldPosition(robot)
  eeMarker.position.set(...ee)
  const pos = dxLine.geometry.attributes.position as THREE.BufferAttribute
  pos.setXYZ(0, ee[0], ee[1], ee[2])
  pos.setXYZ(1, target[0], target[1], target[2])
  pos.needsUpdate = true
}

let ro: ResizeObserver | null = null
onMounted(() => {
  if (!viewport.value) return
  buildScene()
  ro = new ResizeObserver((entries) => {
    const { width, height } = entries[0].contentRect
    if (!width || !height || !renderer) return
    camera.aspect = width / height
    camera.updateProjectionMatrix()
    renderer.setSize(width, height)
  })
  ro.observe(viewport.value)
  retrace()
  syncScene()
})
onBeforeUnmount(() => { cancelAnimationFrame(raf); stopConvergenceRun(); ro?.disconnect(); renderer?.dispose() })
</script>

<template>
  <div class="step-shell">
    <div class="left">
      <div class="code-pane">
        <PseudocodePanel :lines="MASTER_IK_LOOP" :active-line="tracer.current.value?.line ?? -1" />
      </div>
      <div class="controls">
        <label>method
          <select v-model="method" :disabled="isConverging">
            <option value="transpose">transpose</option>
            <option value="pseudoinverse">pseudoinverse</option>
            <option value="damped">damped</option>
          </select>
        </label>
        <button class="btn" :disabled="tracer.isAtStart.value || isConverging" @click="tracer.stepBack">&laquo; Step</button>
        <button v-if="!tracer.isRunning.value" class="btn primary" :disabled="tracer.isDone.value || isConverging" @click="tracer.play">&#9654; Play</button>
        <button v-else class="btn primary" @click="tracer.pause">&#10074;&#10074; Pause</button>
        <button class="btn" :disabled="tracer.isDone.value || isConverging" @click="tracer.stepForward">Step &raquo;</button>
        <button class="btn" :disabled="isConverging" @click="doReset">&#8634; Reset</button>
      </div>
      <div class="controls converge-row">
        <button class="btn primary" :disabled="isConverging" @click="runToConvergence">&#9654;&#9654; Run to convergence</button>
        <span v-if="isConverging" class="status-tag running">iter {{ convergeIterCount }} &middot; |dx| = {{ convergeDxNorm.toFixed(4) }} &middot; running&hellip;</span>
        <span v-else-if="convergeOutcome" class="status-tag" :class="{ ok: convergeOutcome.startsWith('converged') }">{{ convergeOutcome }}</span>
      </div>
    </div>

    <div class="right">
      <div ref="viewport" class="viewport" />
      <div class="values-pane">
        <div v-if="tracer.current.value?.note" class="note">{{ tracer.current.value.note }}</div>
        <div class="value-row" v-if="tracer.current.value?.dx">
          <div class="vlabel">robot.dx</div>
          <table class="vt"><tr v-for="(row, i) in tracer.current.value.dx" :key="i"><td>{{ fmt(row[0]) }}</td></tr></table>
        </div>
        <div class="value-row" v-if="tracer.current.value?.jacobian">
          <div class="vlabel">robot.jacobian</div>
          <table class="vt"><tr v-for="(row, i) in tracer.current.value.jacobian" :key="i"><td v-for="(v,j) in row" :key="j">{{ fmt(v) }}</td></tr></table>
        </div>
        <div class="value-row" v-if="tracer.current.value?.dq">
          <div class="vlabel">robot.dq</div>
          <table class="vt"><tr v-for="(row, i) in tracer.current.value.dq" :key="i"><td>{{ fmt(row[0]) }}</td></tr></table>
        </div>
      </div>
    </div>
  </div>
</template>

<style scoped>
.step-shell { display: grid; grid-template-columns: 0.85fr 1.15fr; gap: 0.7em; height: 100%; min-height: 0; }
.left, .right { display: flex; flex-direction: column; min-height: 0; min-width: 0; gap: 0.4em; }
.code-pane { flex: 1 1 auto; min-height: 0; display: flex; }
.controls { display: flex; flex-wrap: wrap; align-items: center; gap: 0.4em; font-size: 0.6em; font-family: var(--font-mono, monospace); }
.controls select { font: inherit; }
.converge-row { gap: 0.6em; }
.status-tag {
  font-weight: 700; background: #fdecd9; border: 1px solid #202B4E; color: #202B4E;
  border-radius: 999px; padding: 0.15em 0.6em;
}
.status-tag.ok { background: #e3f3ea; border-color: #27966b; color: #1d6e4f; }
.status-tag.running { background: #fdecd9; }
.viewport { flex: 1 1 auto; min-height: 0; border-radius: 8px; overflow: hidden; border: 1px solid #dcdfe6; }
.viewport :deep(canvas) { display: block; }
.values-pane { flex: 0 0 auto; display: flex; flex-wrap: wrap; gap: 0.6em; max-height: 40%; overflow: auto; }
.note {
  font-family: var(--font-mono, monospace); font-size: 0.6em; background: #fdecd9; border: 1px solid #202B4E;
  color: #202B4E; border-radius: 6px; padding: 0.25em 0.6em; width: 100%;
}
.value-row { border: 1px solid #dcdfe6; border-radius: 8px; background: #fff; padding: 0.3em 0.5em; }
.vlabel { font-family: var(--font-mono, monospace); font-size: 0.58em; font-weight: 700; color: var(--indigo, #202B4E); margin-bottom: 0.2em; }
.vt { border-collapse: collapse; font-family: var(--font-mono, monospace); font-size: 0.6em; }
.vt td { padding: 0.08em 0.4em; text-align: right; }
</style>
