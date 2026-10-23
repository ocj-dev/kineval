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
//
// Two selection grains: the dropdown picks WHICH JOINT (coarse), then
// clicking one of that joint's three expression lines -- z_i, lever arm,
// or the Jacobian column J_vi itself (reusing the exact value
// allJointJacobianExpressions/jacobianSteps.ts already computed, not
// recomputed here) -- picks WHICH VECTOR to isolate at full saturation in
// the 3D view; the other two hide rather than merely dim, so exactly one
// arrow is ever on screen at a time.

const props = withDefaults(defineProps<{ robotName?: RobotName }>(), { robotName: 'urdf_example' })
const robotName = props.robotName

const robot: RobotDesc = ROBOT_FACTORIES[robotName]()
for (const n in INITIAL_ANGLES[robotName]) robot.joints[n].angle = INITIAL_ANGLES[robotName][n]
buildFK(robot)

const chainJointNames = CHAIN_JOINT_NAMES[robotName]
const expressions = allJointJacobianExpressions(robot, chainJointNames)
const selectedJoint = ref(chainJointNames[0])
type ExprLine = 'axis' | 'lever' | 'column'
const selectedLine = ref<ExprLine>('axis')
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
let columnArrow: THREE.ArrowHelper
let originMarker: THREE.Mesh
let eeMarker: THREE.Mesh

// Third arrow color for J_vi, distinct from axisArrow's amber (0xe8710a)
// and leverArrow's purple (0x9b59b6) -- same arrow-color convention this
// deck's JacobianBuildPanel.vue establishes, extended with one more hue.
const COLUMN_COLOR = 0x00a99d

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

  columnArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(), 0.1, COLUMN_COLOR, 0.08, 0.05)
  scene.add(columnArrow)

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

  const showAxis = selectedLine.value === 'axis'
  const showLever = selectedLine.value === 'lever'
  const showColumn = selectedLine.value === 'column'

  axisArrow.visible = showAxis
  if (showAxis) {
    axisArrow.position.set(...s.originWorld)
    const axisDir = new THREE.Vector3(...s.axisWorld)
    if (axisDir.length() > 1e-6) {
      axisArrow.setDirection(axisDir.clone().normalize())
      axisArrow.setLength(0.55, 0.08, 0.05)
    }
  }

  leverArrow.visible = showLever
  if (showLever) {
    const lever = new THREE.Vector3(...s.lever)
    leverArrow.position.set(...s.originWorld)
    if (lever.length() > 1e-6) {
      leverArrow.setDirection(lever.clone().normalize())
      leverArrow.setLength(lever.length(), 0.07, 0.04)
    }
  }

  // J_vi = z_i x (p_ee - p_i), the resulting linear-velocity column -- drawn
  // from the end-effector point since that's the velocity it describes,
  // not from the joint origin the other two vectors are anchored at.
  columnArrow.visible = showColumn
  if (showColumn) {
    const column = new THREE.Vector3(...s.column)
    columnArrow.position.set(...s.eeWorld)
    if (column.length() > 1e-6) {
      columnArrow.setDirection(column.clone().normalize())
      columnArrow.setLength(column.length(), 0.07, 0.04)
    }
  }
}

function selectJoint(name: string) {
  selectedJoint.value = name
  selectedLine.value = 'axis'
  syncHighlight()
}

function selectLine(name: string, line: ExprLine) {
  selectedJoint.value = name
  selectedLine.value = line
  syncHighlight()
}

function onDropdownChange() {
  selectedLine.value = 'axis'
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
      >
        <div class="jname" @click="selectJoint(e.jointName)">{{ e.jointIndex }}: {{ e.jointName }}</div>
        <div class="eline">axis<sub>{{ e.jointIndex }}</sub> (local) = {{ fmtVec(e.axisLocal) }}</div>
        <div
          class="eline selectable"
          :class="{ 'line-active': e.jointName === selectedJoint && selectedLine === 'axis' }"
          @click="selectLine(e.jointName, 'axis')"
        >z<sub>{{ e.jointIndex }}</sub> = R<sub>{{ e.jointIndex }}</sub> &middot; axis<sub>{{ e.jointIndex }}</sub> = {{ fmtVec(e.axisWorld) }}</div>
        <div class="eline">p<sub>{{ e.jointIndex }}</sub> = translation(R<sub>{{ e.jointIndex }}</sub>) = {{ fmtVec(e.originWorld) }}</div>
        <div
          class="eline selectable"
          :class="{ 'line-active': e.jointName === selectedJoint && selectedLine === 'lever' }"
          @click="selectLine(e.jointName, 'lever')"
        >lever = p<sub>ee</sub> &minus; p<sub>{{ e.jointIndex }}</sub> = {{ fmtVec(e.lever) }}</div>
        <div
          class="eline selectable"
          :class="{ 'line-active': e.jointName === selectedJoint && selectedLine === 'column' }"
          @click="selectLine(e.jointName, 'column')"
        >J<sub>v{{ e.jointIndex }}</sub> = z<sub>{{ e.jointIndex }}</sub> &times; (p<sub>ee</sub> &minus; p<sub>{{ e.jointIndex }}</sub>) = {{ fmtVec(e.column) }}</div>
      </div>
    </div>
    <div class="right">
      <label class="picker">focus joint
        <select v-model="selectedJoint" @change="onDropdownChange">
          <option v-for="n in chainJointNames" :key="n" :value="n">{{ n }}</option>
        </select>
      </label>
      <div class="legend">
        <span class="swatch-dot" style="background:#e8710a"></span> z<sub>i</sub> (axis)
        <span class="swatch-dot" style="background:#9b59b6"></span> lever arm
        <span class="swatch-dot" style="background:#00a99d"></span> J<sub>vi</sub> (column)
      </div>
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
.jname { font-weight: 700; color: var(--indigo, #202B4E); margin-bottom: 0.2em; cursor: pointer; }
.joint-expr.active .jname { color: #b85a08; }
.eline { padding: 0.04em 0; white-space: nowrap; overflow-x: auto; }
.eline.selectable { cursor: pointer; border-radius: 4px; padding: 0.08em 0.3em; margin: 0.02em -0.3em; transition: background 0.15s; }
.eline.selectable:hover { background: #f0f0f0; }
.eline.selectable.line-active { background: #e4f7f4; outline: 1px solid #00a99d; font-weight: 700; }

.right { display: flex; flex-direction: column; min-height: 0; min-width: 0; gap: 0.4em; }
.picker { display: flex; align-items: center; gap: 0.4em; font-size: 0.64em; font-family: var(--font-mono, monospace); }
.picker select { font: inherit; }
.legend { display: flex; align-items: center; gap: 0.4em; font-size: 0.58em; font-family: var(--font-mono, monospace); opacity: 0.8; }
.swatch-dot { display: inline-block; width: 0.7em; height: 0.7em; border-radius: 50%; margin-right: 0.15em; }
.viewport { flex: 1 1 auto; min-height: 0; border-radius: 8px; overflow: hidden; border: 1px solid #dcdfe6; }
.viewport :deep(canvas) { display: block; }
</style>
