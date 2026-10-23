<script setup lang="ts">
import { ref, reactive, watch, onMounted, onBeforeUnmount } from 'vue'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { useStepTracer } from '../lib/ik/useStepTracer'

// The explicit bridge from Step 0's 1D scalar-function gradient descent
// (GradientDescent1DPanel.vue) to the real vector-valued manipulator
// Jacobian the rest of this deck uses: g: R^2 -> R^2,
//   g(x,y) = (x^2 - y, x + y^2)
// with analytic Jacobian
//   J(x,y) = [[2x, -1], [1, 2y]]
// verified against a central finite-difference check at several (x,y)
// samples in Node before shipping (agreement to ~1e-9) -- same discipline
// this deck's every other quantitative claim gets.
//
// g_target = (0, 2) is reachable exactly: g(1,1) = (1-1, 1+1) = (0,2), so
// cost C(x,y) = ||g_target - g(x,y)||^2 has an exact zero at (x,y) = (1,1).
// Starting from (x0,y0) (default (-1, 1.5), cost 0.8125), gradient descent
// on C takes the SAME update shape iterateIK uses: form the real Jacobian
// at the current point, compute error e = g_target - g(x,y), update via
// dq = J^T e (transpose method, to keep this bridge slide simple and
// consistent with this deck's gradient-descent framing), scaled by a step
// length -- and the marker below descends the real C(x,y) surface along
// its actual trajectory while the left panel prints the numeric Jacobian
// and update at every step.
//
// Start point (x0, y0) is settable two ways: the sliders below, or
// clicking directly on the ground plane in the 3D view -- same
// raycast-a-pointer-against-a-plane technique IkConvergePanel.vue uses for
// its drag-to-set-target interaction, just against the fixed y=0 ground
// plane the cost surface is built over (not a camera-facing plane through
// a marker, since this plane never has to face the camera -- it's always
// horizontal), and a single click rather than a full drag state machine,
// since this panel only needs the one point before retracing, not a
// continuously-followed live target the way IkConvergePanel's IS.

function g(x: number, y: number): [number, number] {
  return [x * x - y, x + y * y]
}
function jacobianAt(x: number, y: number): [[number, number], [number, number]] {
  return [[2 * x, -1], [1, 2 * y]]
}
const TARGET: [number, number] = [0, 2]
function cost(x: number, y: number): number {
  const [gx, gy] = g(x, y)
  const ex = TARGET[0] - gx, ey = TARGET[1] - gy
  return ex * ex + ey * ey
}

interface Gd2Step {
  x: number; y: number; c: number
  e: [number, number]
  J: [[number, number], [number, number]]
  dq: [number, number]
}

const params = reactive({ gamma: 0.05, x0: -1, y0: 1.5 })
const speedMs = ref(180)
const tracer = useStepTracer<Gd2Step>(speedMs)

function buildTrace(): Gd2Step[] {
  const steps: Gd2Step[] = []
  let x = params.x0, y = params.y0
  for (let i = 0; i < 80; i++) {
    const [gx, gy] = g(x, y)
    const e: [number, number] = [TARGET[0] - gx, TARGET[1] - gy]
    const c = e[0] * e[0] + e[1] * e[1]
    const J = jacobianAt(x, y)
    // dq = J^T e
    const dq: [number, number] = [J[0][0] * e[0] + J[1][0] * e[1], J[0][1] * e[0] + J[1][1] * e[1]]
    steps.push({ x, y, c, e, J, dq })
    if (c < 1e-6) break
    x += params.gamma * dq[0]
    y += params.gamma * dq[1]
  }
  return steps
}

function retrace() {
  tracer.load(buildTrace())
}

/* ---------------- 3D cost-surface viewport ---------------- */
const viewport = ref<HTMLDivElement | null>(null)
let renderer: THREE.WebGLRenderer | null = null
let scene: THREE.Scene, camera: THREE.PerspectiveCamera, controls: OrbitControls
let raf = 0
let marker: THREE.Mesh
let trajLine: THREE.Line
let targetMarker: THREE.Mesh
let startMarker: THREE.Mesh
const raycaster = new THREE.Raycaster()
const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0) // y = 0, the same ground the surface is built over

const GRID_X_MIN = -2.0, GRID_X_MAX = 1.2
const GRID_Y_MIN = -1.2, GRID_Y_MAX = 2.2
const GRID_N = 26
let HEIGHT_SCALE = 1

function surfaceY(x: number, y: number): number {
  return cost(x, y) * HEIGHT_SCALE
}

function buildScene() {
  const el = viewport.value!
  scene = new THREE.Scene()
  scene.background = new THREE.Color(0xfbfbfa)

  camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100)
  camera.position.set(4.2, 3.4, 4.6)

  renderer = new THREE.WebGLRenderer({ antialias: true })
  renderer.setPixelRatio(window.devicePixelRatio || 1)
  el.appendChild(renderer.domElement)

  controls = new OrbitControls(camera, renderer.domElement)
  controls.enableDamping = true
  controls.target.set((GRID_X_MIN + GRID_X_MAX) / 2, 0, (GRID_Y_MIN + GRID_Y_MAX) / 2)

  scene.add(new THREE.HemisphereLight(0xffffff, 0x555555, 1.25))
  const sun = new THREE.DirectionalLight(0xffffff, 0.75)
  sun.position.set(3, 5, 2)
  scene.add(sun)

  // find the max cost over the grid to scale height into a sane range
  let maxC = 1e-6
  for (let i = 0; i <= GRID_N; i++) {
    for (let j = 0; j <= GRID_N; j++) {
      const x = GRID_X_MIN + (GRID_X_MAX - GRID_X_MIN) * (i / GRID_N)
      const y = GRID_Y_MIN + (GRID_Y_MAX - GRID_Y_MIN) * (j / GRID_N)
      maxC = Math.max(maxC, cost(x, y))
    }
  }
  HEIGHT_SCALE = 3.0 / maxC

  // wireframe cost surface: one line strip per grid row, one per grid column
  const positions: number[] = []
  for (let i = 0; i <= GRID_N; i++) {
    const x = GRID_X_MIN + (GRID_X_MAX - GRID_X_MIN) * (i / GRID_N)
    for (let j = 0; j < GRID_N; j++) {
      const y0 = GRID_Y_MIN + (GRID_Y_MAX - GRID_Y_MIN) * (j / GRID_N)
      const y1 = GRID_Y_MIN + (GRID_Y_MAX - GRID_Y_MIN) * ((j + 1) / GRID_N)
      positions.push(x, surfaceY(x, y0), y0, x, surfaceY(x, y1), y1)
    }
  }
  for (let j = 0; j <= GRID_N; j++) {
    const y = GRID_Y_MIN + (GRID_Y_MAX - GRID_Y_MIN) * (j / GRID_N)
    for (let i = 0; i < GRID_N; i++) {
      const x0 = GRID_X_MIN + (GRID_X_MAX - GRID_X_MIN) * (i / GRID_N)
      const x1 = GRID_X_MIN + (GRID_X_MAX - GRID_X_MIN) * ((i + 1) / GRID_N)
      positions.push(x0, surfaceY(x0, y), y, x1, surfaceY(x1, y), y)
    }
  }
  const surfGeom = new THREE.BufferGeometry()
  surfGeom.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3))
  const surfMat = new THREE.LineBasicMaterial({ color: 0xc7cad1 })
  scene.add(new THREE.LineSegments(surfGeom, surfMat))

  // ground-plane grid for spatial reference, at height 0
  scene.add(new THREE.GridHelper(Math.max(GRID_X_MAX - GRID_X_MIN, GRID_Y_MAX - GRID_Y_MIN) * 1.4, 12, 0xdddfe4, 0xeceef1))

  // exact zero-cost target point, (x,y) = (1,1)
  targetMarker = new THREE.Mesh(new THREE.SphereGeometry(0.06, 16, 16), new THREE.MeshLambertMaterial({ color: 0x27966b }))
  targetMarker.position.set(1, surfaceY(1, 1), 1)
  scene.add(targetMarker)

  marker = new THREE.Mesh(new THREE.SphereGeometry(0.07, 16, 16), new THREE.MeshLambertMaterial({ color: 0xe8710a }))
  scene.add(marker)

  // start marker: sits ON the ground plane (y=0) at (x0, y0) -- draggable
  // target for "click the ground to set the start point", distinct from
  // the orange "current point on the surface" marker above.
  startMarker = new THREE.Mesh(new THREE.ConeGeometry(0.06, 0.14, 20), new THREE.MeshLambertMaterial({ color: 0x2e6fd9 }))
  startMarker.rotation.x = Math.PI // point tip down, into the ground
  scene.add(startMarker)

  const trajGeom = new THREE.BufferGeometry()
  trajGeom.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(3 * 200), 3))
  trajLine = new THREE.Line(trajGeom, new THREE.LineBasicMaterial({ color: 0x9b59b6 }))
  scene.add(trajLine)

  renderer.domElement.addEventListener('click', onCanvasClick)

  const loop = () => { raf = requestAnimationFrame(loop); controls.update(); renderer!.render(scene, camera) }
  loop()
}

function pointerNdc(e: PointerEvent | MouseEvent): THREE.Vector2 {
  const rect = (renderer!.domElement as HTMLCanvasElement).getBoundingClientRect()
  return new THREE.Vector2(((e.clientX - rect.left) / rect.width) * 2 - 1, -((e.clientY - rect.top) / rect.height) * 2 + 1)
}

// Click anywhere on the 3D view -> raycast against the ground plane (the
// same plane the cost surface sits over) -> set (x0, y0) to the clicked
// location (clamped to the grid this panel actually draws) -> retrace.
// Same raycast-a-pointer-against-a-plane technique IkConvergePanel.vue
// uses to drag its target; here one click is enough since there's no
// continuously-followed live point to track.
function onCanvasClick(e: MouseEvent) {
  const ndc = pointerNdc(e)
  raycaster.setFromCamera(ndc, camera)
  const hit = new THREE.Vector3()
  if (!raycaster.ray.intersectPlane(groundPlane, hit)) return
  params.x0 = Math.min(GRID_X_MAX, Math.max(GRID_X_MIN, hit.x))
  params.y0 = Math.min(GRID_Y_MAX, Math.max(GRID_Y_MIN, hit.z))
  retrace()
  syncScene()
}

function syncScene() {
  startMarker.position.set(params.x0, 0.07, params.y0)

  const visited = tracer.steps.value.slice(0, Math.max(0, tracer.pos.value + 1))
  const pos = trajLine.geometry.attributes.position as THREE.BufferAttribute
  const cap = Math.min(visited.length, pos.count)
  for (let i = 0; i < cap; i++) pos.setXYZ(i, visited[i].x, surfaceY(visited[i].x, visited[i].y) + 0.01, visited[i].y)
  trajLine.geometry.setDrawRange(0, cap)
  pos.needsUpdate = true

  const cur = tracer.current.value
  if (cur) {
    marker.visible = true
    marker.position.set(cur.x, surfaceY(cur.x, cur.y) + 0.01, cur.y)
  } else {
    marker.visible = false
  }
}

watch(() => tracer.pos.value, syncScene)

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
onBeforeUnmount(() => {
  cancelAnimationFrame(raf)
  ro?.disconnect()
  renderer?.domElement.removeEventListener('click', onCanvasClick)
  renderer?.dispose()
})

watch(() => [params.gamma, params.x0, params.y0], () => { retrace(); syncScene() })

const fmt = (v: number) => (v >= 0 ? ' ' : '') + v.toFixed(4)
</script>

<template>
  <div class="gd2d-shell">
    <div class="left">
      <div class="controls">
        <label>step length &gamma; <input v-model.number="params.gamma" type="range" min="0.005" max="0.12" step="0.005"></label>
        <span class="val">{{ params.gamma.toFixed(3) }}</span>
      </div>
      <div class="controls">
        <label>start x<sub>0</sub> <input v-model.number="params.x0" type="range" :min="GRID_X_MIN" :max="GRID_X_MAX" step="0.05"></label>
        <span class="val">{{ params.x0.toFixed(2) }}</span>
        <label>start y<sub>0</sub> <input v-model.number="params.y0" type="range" :min="GRID_Y_MIN" :max="GRID_Y_MAX" step="0.05"></label>
        <span class="val">{{ params.y0.toFixed(2) }}</span>
      </div>
      <div class="hint" style="margin-top:0">or click the ground plane in the 3D view to set (x<sub>0</sub>, y<sub>0</sub>) directly (blue marker)</div>
      <div class="playback">
        <button class="btn" :disabled="tracer.isAtStart.value" @click="tracer.stepBack">&laquo; Step</button>
        <button v-if="!tracer.isRunning.value" class="btn primary" :disabled="tracer.isDone.value" @click="tracer.play">&#9654; Play</button>
        <button v-else class="btn primary" @click="tracer.pause">&#10074;&#10074; Pause</button>
        <button class="btn" :disabled="tracer.isDone.value" @click="tracer.stepForward">Step &raquo;</button>
        <button class="btn" @click="tracer.reset">&#8634; Reset</button>
      </div>
      <div class="readout" v-if="tracer.current.value">
        <div class="row"><span class="lbl">(x, y)</span> ({{ fmt(tracer.current.value.x) }}, {{ fmt(tracer.current.value.y) }})</div>
        <div class="row"><span class="lbl">g(x,y)</span> ({{ fmt(tracer.current.value.x * tracer.current.value.x - tracer.current.value.y) }}, {{ fmt(tracer.current.value.x + tracer.current.value.y * tracer.current.value.y) }})</div>
        <div class="row"><span class="lbl">e = g_target - g</span> ({{ fmt(tracer.current.value.e[0]) }}, {{ fmt(tracer.current.value.e[1]) }})</div>
        <div class="row"><span class="lbl">C(x,y) = |e|&sup2;</span> {{ fmt(tracer.current.value.c) }}</div>
        <div class="row jrow">
          <span class="lbl">J(x,y)</span>
          <table class="mat"><tr><td>{{ fmt(tracer.current.value.J[0][0]) }}</td><td>{{ fmt(tracer.current.value.J[0][1]) }}</td></tr><tr><td>{{ fmt(tracer.current.value.J[1][0]) }}</td><td>{{ fmt(tracer.current.value.J[1][1]) }}</td></tr></table>
        </div>
        <div class="row"><span class="lbl">dq = J<sup>T</sup> e</span> ({{ fmt(tracer.current.value.dq[0]) }}, {{ fmt(tracer.current.value.dq[1]) }})</div>
      </div>
      <div v-else class="readout muted">— not started —</div>
      <div class="hint">orange: current point &middot; green: exact target (x,y)=(1,1) &middot; purple: trajectory &middot; step {{ Math.max(0, tracer.pos.value + 1) }} / {{ tracer.stepCount.value }}</div>
    </div>
    <div ref="viewport" class="viewport" />
  </div>
</template>

<style scoped>
.gd2d-shell { display: grid; grid-template-columns: 0.95fr 1.05fr; gap: 0.7em; height: 100%; min-height: 0; }
.left { display: flex; flex-direction: column; gap: 0.5em; min-height: 0; min-width: 0; font-family: var(--font-mono, monospace); font-size: 0.64em; }
.controls { display: flex; align-items: center; gap: 0.5em; flex-wrap: wrap; }
.controls input[type="range"] { width: 6em; }
.val { opacity: 0.7; }
.playback { display: flex; align-items: center; gap: 0.4em; flex-wrap: wrap; }
.readout { border: 1px solid #dcdfe6; border-radius: 8px; background: #fff; padding: 0.4em 0.6em; display: flex; flex-direction: column; gap: 0.2em; }
.readout.muted { opacity: 0.5; }
.row { display: flex; gap: 0.5em; align-items: center; }
.lbl { font-weight: 700; color: var(--indigo, #202B4E); min-width: 7.5em; }
.jrow { align-items: flex-start; }
.mat { border-collapse: collapse; }
.mat td { padding: 0 0.4em; text-align: right; }
.hint { font-size: 0.92em; opacity: 0.65; margin-top: auto; }
.viewport { min-height: 0; border-radius: 8px; overflow: hidden; border: 1px solid #dcdfe6; }
.viewport :deep(canvas) { display: block; }
</style>
