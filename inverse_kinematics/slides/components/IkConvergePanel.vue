<script setup lang="ts">
import { reactive, ref, onMounted, onBeforeUnmount, computed } from 'vue'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { ROBOT_FACTORIES, CHAIN_JOINT_NAMES, INITIAL_ANGLES, DEFAULT_TARGETS, type RobotName } from '../lib/ik/robots'
import { buildFK, endEffectorWorldPosition } from '../lib/ik/fkForIk'
import { runIkIteration, applyControls } from '../lib/ik/solveSteps'
import type { IkMethod, IkParams, Mat4, RobotDesc } from '../lib/ik/types'

// Drag the magenta target anywhere in 3D; the arm converges toward it LIVE,
// using runIkIteration/applyControls -- the exact same TS port of
// iterateIK the step-through panel (IkStepPanel.vue) and the Jacobian
// walkthrough panels call, so this picture can never disagree with the
// math being taught (the "derive overlays by calling the same function
// being taught" pattern this deck's draggable-target slide names).
//
// Drag model: raycast the pointer against a plane through the target,
// facing the camera (the standard three.js-editor convention for dragging
// a point in 3D with a 2D pointer) -- orbit the view between drags to
// approach the target from a different depth.

const props = withDefaults(defineProps<{ robotName?: RobotName }>(), { robotName: 'mr2' })

const robotName = props.robotName
const robot: RobotDesc = ROBOT_FACTORIES[robotName]()
for (const n in INITIAL_ANGLES[robotName]) robot.joints[n].angle = INITIAL_ANGLES[robotName][n]
for (const n in robot.joints) if (robot.joints[n].control === undefined) robot.joints[n].control = 0
buildFK(robot)

const ikParams = reactive<IkParams>({
  method: 'transpose' as IkMethod,
  steplength: 0.3,
  damping: 0.15,
  orientationIncluded: false,
  chainJointNames: CHAIN_JOINT_NAMES[robotName],
})
const target = reactive<{ position: [number, number, number] }>({ position: [...DEFAULT_TARGETS[robotName]] })
const isRunning = ref(true)
const iterationCount = ref(0)
const dxNorm = ref(0)
const stepMs = 45

const viewport = ref<HTMLDivElement | null>(null)
let renderer: THREE.WebGLRenderer | null = null
let scene: THREE.Scene, camera: THREE.PerspectiveCamera, controls: OrbitControls
let raf = 0
const linkMeshes: Record<string, THREE.Mesh> = {}
let targetMarker: THREE.Mesh, eeMarker: THREE.Mesh
let dragging = false
const raycaster = new THREE.Raycaster()
const dragPlane = new THREE.Plane()
const dragPoint = new THREE.Vector3()

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
  scene.background = new THREE.Color(0xf5f5f5)

  camera = new THREE.PerspectiveCamera(48, 1, 0.05, 100)
  camera.position.set(2.3, 2.0, 2.6)

  renderer = new THREE.WebGLRenderer({ antialias: true })
  renderer.setPixelRatio(window.devicePixelRatio || 1)
  el.appendChild(renderer.domElement)

  controls = new OrbitControls(camera, renderer.domElement)
  controls.enableDamping = true
  controls.target.set(0, 0.5, 0)

  scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 1.2))
  const sun = new THREE.DirectionalLight(0xffffff, 0.85)
  sun.position.set(3, 5, 2)
  scene.add(sun)
  scene.add(new THREE.GridHelper(6, 12, 0xc7cad1, 0xdddfe4))

  for (const name in robot.links) {
    const g = robot.links[name].geometry!
    const geom = new THREE.BoxGeometry(g.size[0], g.size[1], g.size[2])
    geom.translate(g.offset.xyz[0], g.offset.xyz[1], g.offset.xyz[2])
    const mesh = new THREE.Mesh(geom, new THREE.MeshLambertMaterial({ color: robot.links[name].color ?? 0x8a8f98 }))
    mesh.matrixAutoUpdate = false
    scene.add(mesh)
    linkMeshes[name] = mesh
  }

  targetMarker = new THREE.Mesh(new THREE.SphereGeometry(0.06, 20, 20), new THREE.MeshLambertMaterial({ color: 0xe8349a }))
  scene.add(targetMarker)
  eeMarker = new THREE.Mesh(new THREE.SphereGeometry(0.05, 20, 20), new THREE.MeshLambertMaterial({ color: 0xe8710a }))
  scene.add(eeMarker)

  const dom = renderer.domElement
  dom.addEventListener('pointerdown', onPointerDown)
  dom.addEventListener('pointermove', onPointerMove)
  window.addEventListener('pointerup', onPointerUp)

  let lastStepTime = 0
  const loop = (now: number) => {
    raf = requestAnimationFrame(loop)
    const due = isRunning.value && now - lastStepTime >= stepMs
    if (due) {
      lastStepTime = now
      const result = runIkIteration(robot, target.position, null, ikParams)
      applyControls(robot)
      buildFK(robot)
      iterationCount.value++
      let n = 0
      for (const row of result.dx) n += row[0] * row[0]
      dxNorm.value = Math.sqrt(n)
    }
    syncScene()
    controls.update()
    renderer!.render(scene, camera)
  }
  raf = requestAnimationFrame(loop)
}

function syncScene() {
  for (const name in linkMeshes) {
    const m = robot.links[name].matrix as Mat4 | undefined
    if (m) linkMeshes[name].matrix.copy(mat4(m))
  }
  targetMarker.position.set(...target.position)
  const ee = endEffectorWorldPosition(robot)
  eeMarker.position.set(...ee)
}

function pointerNdc(e: PointerEvent): THREE.Vector2 {
  const rect = (renderer!.domElement as HTMLCanvasElement).getBoundingClientRect()
  return new THREE.Vector2(((e.clientX - rect.left) / rect.width) * 2 - 1, -((e.clientY - rect.top) / rect.height) * 2 + 1)
}

function onPointerDown(e: PointerEvent) {
  const ndc = pointerNdc(e)
  raycaster.setFromCamera(ndc, camera)
  const hit = raycaster.intersectObject(targetMarker)
  if (hit.length) {
    dragging = true
    controls.enabled = false
    const camDir = new THREE.Vector3()
    camera.getWorldDirection(camDir)
    dragPlane.setFromNormalAndCoplanarPoint(camDir, targetMarker.position)
  }
}
function onPointerMove(e: PointerEvent) {
  if (!dragging) return
  const ndc = pointerNdc(e)
  raycaster.setFromCamera(ndc, camera)
  if (raycaster.ray.intersectPlane(dragPlane, dragPoint)) {
    target.position = [dragPoint.x, dragPoint.y, dragPoint.z]
  }
}
function onPointerUp() {
  dragging = false
  controls.enabled = true
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
})
onBeforeUnmount(() => {
  cancelAnimationFrame(raf)
  ro?.disconnect()
  renderer?.domElement.removeEventListener('pointerdown', onPointerDown)
  renderer?.domElement.removeEventListener('pointermove', onPointerMove)
  window.removeEventListener('pointerup', onPointerUp)
  renderer?.dispose()
})

function resetPose() {
  for (const n in INITIAL_ANGLES[robotName]) robot.joints[n].angle = INITIAL_ANGLES[robotName][n]
  buildFK(robot)
  iterationCount.value = 0
}

const converged = computed(() => dxNorm.value < 0.01)
</script>

<template>
  <div class="converge-shell">
    <div class="controls">
      <label>method
        <select v-model="ikParams.method">
          <option value="transpose">transpose</option>
          <option value="pseudoinverse">pseudoinverse</option>
          <option value="damped">damped</option>
        </select>
      </label>
      <label>step length <input v-model.number="ikParams.steplength" type="range" min="0.02" max="0.8" step="0.02"></label>
      <label v-if="ikParams.method === 'damped'">damping &lambda; <input v-model.number="ikParams.damping" type="range" min="0" max="0.6" step="0.02"></label>
      <button class="btn" @click="isRunning = !isRunning">{{ isRunning ? 'Pause' : 'Resume' }}</button>
      <button class="btn" @click="resetPose">&#8634; Reset pose</button>
      <span class="status-tag" :class="{ ok: converged }">|dx| = {{ dxNorm.toFixed(4) }} &middot; iter {{ iterationCount }}</span>
    </div>
    <div ref="viewport" class="viewport" />
    <div class="hint">drag the magenta target sphere; the arm (orange end-effector marker) chases it live</div>
  </div>
</template>

<style scoped>
.converge-shell { display: flex; flex-direction: column; height: 100%; min-height: 0; gap: 0.4em; }
.controls { display: flex; flex-wrap: wrap; align-items: center; gap: 0.6em; font-size: 0.66em; font-family: var(--font-mono, monospace); }
.controls select { font: inherit; }
.controls input[type="range"] { width: 5em; }
.status-tag {
  font-weight: 700; background: #fdecd9; border: 1px solid #202B4E; color: #202B4E;
  border-radius: 999px; padding: 0.15em 0.6em; margin-left: auto;
}
.status-tag.ok { background: #e3f3ea; border-color: #27966b; color: #1d6e4f; }
.viewport { flex: 1 1 auto; min-height: 0; border-radius: 8px; overflow: hidden; border: 1px solid #dcdfe6; touch-action: none; }
.viewport :deep(canvas) { display: block; }
.hint { font-size: 0.6em; opacity: 0.65; font-family: var(--font-mono, monospace); }
</style>
