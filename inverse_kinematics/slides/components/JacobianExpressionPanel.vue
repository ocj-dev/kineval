<script setup lang="ts">
import { ref, computed, onMounted, onBeforeUnmount } from 'vue'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { ROBOT_FACTORIES, CHAIN_JOINT_NAMES, INITIAL_ANGLES, type RobotName } from '../lib/ik/robots'
import { buildFK } from '../lib/ik/fkForIk'
import { allJointJacobianExpressions } from '../lib/ik/jacobianExpressions'
import type { Mat4, RobotDesc } from '../lib/ik/types'

// A direct-selection companion to JacobianBuildPanel.vue's sequential
// walkthrough: instead of stepping through computeManipulatorJacobian's
// loop one joint at a time, this panel shows EVERY chain joint's
// instantiated math expression at once (left pane, real numbers at the
// robot's current pose) and lets a dropdown pick which one to highlight in
// 3D (right pane) -- same per-joint quantities (z_i, p_i, lever arm,
// column), same underlying math (lib/ik/jacobianSteps.ts via
// jacobianExpressions.ts's allJointJacobianExpressions), just reshaped for
// "pick one and look at it" instead of "watch the build happen".

const props = withDefaults(defineProps<{ robotName?: RobotName }>(), { robotName: 'urdf_example' })
const robotName = props.robotName

const robot: RobotDesc = ROBOT_FACTORIES[robotName]()
for (const n in INITIAL_ANGLES[robotName]) robot.joints[n].angle = INITIAL_ANGLES[robotName][n]
buildFK(robot)

const chainJointNames = CHAIN_JOINT_NAMES[robotName]
const expressions = allJointJacobianExpressions(robot, chainJointNames)
const selectedJoint = ref(chainJointNames[0])
const selectedExpr = computed(() => expressions.find((e) => e.jointName === selectedJoint.value) ?? expressions[0])
const eeWorld = expressions[0]?.eeWorld ?? [0, 0, 0]

const fmt = (v: number) => (v < 0 ? '' : ' ') + v.toFixed(2)
const fmtVec = (v: readonly number[]) => `(${v.map(fmt).join(', ')})`

/* ---------------- 3D scene: robot at 60% opacity, selected joint highlighted ---------------- */
const viewport = ref<HTMLDivElement | null>(null)
let renderer: THREE.WebGLRenderer | null = null
let scene: THREE.Scene, camera: THREE.PerspectiveCamera, controls: OrbitControls
let raf = 0
let axisArrow: THREE.ArrowHelper
let leverArrow: THREE.ArrowHelper
let originMarker: THREE.Mesh
let eeMarker: THREE.Mesh

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
  camera.position.set(2.1, 1.7, 2.3)

  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false })
  renderer.setPixelRatio(window.devicePixelRatio || 1)
  el.appendChild(renderer.domElement)

  controls = new OrbitControls(camera, renderer.domElement)
  controls.enableDamping = true
  controls.target.set(0, 0.5, 0)

  scene.add(new THREE.HemisphereLight(0xffffff, 0x555555, 1.25))
  const sun = new THREE.DirectionalLight(0xffffff, 0.75)
  sun.position.set(3, 5, 2)
  scene.add(sun)
  scene.add(new THREE.GridHelper(6, 12, 0xc7cad1, 0xe2e4e8))

  for (const name in robot.links) {
    const g = robot.links[name].geometry!
    const geom = new THREE.BoxGeometry(g.size[0], g.size[1], g.size[2])
    geom.translate(g.offset.xyz[0], g.offset.xyz[1], g.offset.xyz[2])
    const mat = new THREE.MeshLambertMaterial({
      color: robot.links[name].color ?? 0x8a8f98,
      transparent: true,
      opacity: 0.6,
    })
    const mesh = new THREE.Mesh(geom, mat)
    mesh.matrixAutoUpdate = false
    const m = robot.links[name].matrix as Mat4 | undefined
    if (m) mesh.matrix.copy(mat4(m))
    scene.add(mesh)
  }

  axisArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(), 0.55, 0xe8710a, 0.08, 0.05)
  scene.add(axisArrow)

  leverArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(), 0.1, 0x9b59b6, 0.08, 0.05)
  scene.add(leverArrow)

  originMarker = new THREE.Mesh(new THREE.SphereGeometry(0.035, 16, 16), new THREE.MeshLambertMaterial({ color: 0x202b4e }))
  scene.add(originMarker)

  eeMarker = new THREE.Mesh(new THREE.SphereGeometry(0.045, 16, 16), new THREE.MeshLambertMaterial({ color: 0xe8349a }))
  eeMarker.position.set(...(eeWorld as [number, number, number]))
  scene.add(eeMarker)

  syncHighlight()

  const loop = () => { raf = requestAnimationFrame(loop); controls.update(); renderer!.render(scene, camera) }
  loop()
}

function syncHighlight() {
  const s = selectedExpr.value
  if (!s) return
  originMarker.position.set(...s.originWorld)

  axisArrow.position.set(...s.originWorld)
  const axisDir = new THREE.Vector3(...s.axisWorld)
  if (axisDir.length() > 1e-6) {
    axisArrow.setDirection(axisDir.clone().normalize())
    axisArrow.setLength(0.55, 0.08, 0.05)
  }

  const lever = new THREE.Vector3(...s.lever)
  leverArrow.position.set(...s.originWorld)
  if (lever.length() > 1e-6) {
    leverArrow.setDirection(lever.clone().normalize())
    leverArrow.setLength(lever.length(), 0.07, 0.04)
  }
}

function onSelectJoint() {
  syncHighlight()
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
onBeforeUnmount(() => { cancelAnimationFrame(raf); ro?.disconnect(); renderer?.dispose() })
</script>

<template>
  <div class="expr-shell">
    <div class="left">
      <div class="ee-row">p<sub>ee</sub> = {{ fmtVec(eeWorld) }} <span class="muted">(fixed for every joint below)</span></div>
      <div
        v-for="e in expressions"
        :key="e.jointName"
        class="joint-expr"
        :class="{ active: e.jointName === selectedJoint }"
        @click="selectedJoint = e.jointName; onSelectJoint()"
      >
        <div class="jname">{{ e.jointIndex }}: {{ e.jointName }}</div>
        <div class="eline">axis<sub>{{ e.jointIndex }}</sub> (local) = {{ fmtVec(e.axisLocal) }}</div>
        <div class="eline">z<sub>{{ e.jointIndex }}</sub> = R<sub>{{ e.jointIndex }}</sub> &middot; axis<sub>{{ e.jointIndex }}</sub> = {{ fmtVec(e.axisWorld) }}</div>
        <div class="eline">p<sub>{{ e.jointIndex }}</sub> = translation(R<sub>{{ e.jointIndex }}</sub>) = {{ fmtVec(e.originWorld) }}</div>
        <div class="eline">lever = p<sub>ee</sub> &minus; p<sub>{{ e.jointIndex }}</sub> = {{ fmtVec(e.lever) }}</div>
        <div class="eline">J<sub>v{{ e.jointIndex }}</sub> = z<sub>{{ e.jointIndex }}</sub> &times; (p<sub>ee</sub> &minus; p<sub>{{ e.jointIndex }}</sub>) = {{ fmtVec(e.column) }}</div>
      </div>
    </div>
    <div class="right">
      <label class="picker">focus joint
        <select v-model="selectedJoint" @change="onSelectJoint">
          <option v-for="n in chainJointNames" :key="n" :value="n">{{ n }}</option>
        </select>
      </label>
      <div ref="viewport" class="viewport" />
    </div>
  </div>
</template>

<style scoped>
.expr-shell { display: grid; grid-template-columns: 1.05fr 0.95fr; gap: 0.7em; height: 100%; min-height: 0; }
.left { display: flex; flex-direction: column; gap: 0.4em; min-height: 0; overflow-y: auto; padding-right: 0.3em; }
.ee-row { font-family: var(--font-mono, monospace); font-size: 0.62em; font-weight: 700; color: var(--indigo, #202B4E); }
.ee-row .muted { font-weight: 400; opacity: 0.6; }
.joint-expr {
  border: 1px solid #dcdfe6; border-radius: 8px; background: #fff; padding: 0.35em 0.6em;
  font-family: var(--font-mono, monospace); font-size: 0.58em; cursor: pointer; transition: background 0.15s, border-color 0.15s;
}
.joint-expr:hover { background: #faf7ec; }
.joint-expr.active { border-color: #e8710a; background: #fdecd9; }
.jname { font-weight: 700; color: var(--indigo, #202B4E); margin-bottom: 0.2em; }
.joint-expr.active .jname { color: #b85a08; }
.eline { padding: 0.04em 0; white-space: nowrap; overflow-x: auto; }

.right { display: flex; flex-direction: column; min-height: 0; min-width: 0; gap: 0.4em; }
.picker { display: flex; align-items: center; gap: 0.4em; font-size: 0.64em; font-family: var(--font-mono, monospace); }
.picker select { font: inherit; }
.viewport { flex: 1 1 auto; min-height: 0; border-radius: 8px; overflow: hidden; border: 1px solid #dcdfe6; }
.viewport :deep(canvas) { display: block; }
</style>
