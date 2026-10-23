<script setup lang="ts">
import { reactive, ref, watch, onMounted, onBeforeUnmount, computed } from 'vue'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import PseudocodePanel from './PseudocodePanel.vue'
import { useStepTracer } from '../lib/ik/useStepTracer'
import { jacobianSteps, assembleJacobian } from '../lib/ik/jacobianSteps'
import { buildFK } from '../lib/ik/fkForIk'
import { ROBOT_FACTORIES, CHAIN_JOINT_NAMES, INITIAL_ANGLES, type RobotName } from '../lib/ik/robots'
import { MASTER_JACOBIAN_BUILD } from '../lib/ik/pseudocode'
import type { JacobianColumnStep, Mat4, RobotDesc } from '../lib/ik/types'

// Steps computeManipulatorJacobian's own per-joint loop one column at a
// time, in 3D: each chain joint's world axis z_i (amber arrow) and its
// lever arm to the end-effector, p_ee - p_i (purple dashed arrow), appear
// exactly when the build reaches that joint, and the running partial
// Jacobian (one column per joint built so far) grows on the right -- the
// picture is driven by the SAME jacobianSteps.ts generator that is also
// quoted, unmodified logic-for-logic, from computeManipulatorJacobian
// itself (reference/inverse_kinematics.js#manipulator-jacobian).

const props = withDefaults(defineProps<{ robotName?: RobotName }>(), { robotName: 'urdf_example' })

const speedMs = ref(650)
const tracer = useStepTracer<JacobianColumnStep>(speedMs)

const robot: RobotDesc = ROBOT_FACTORIES[props.robotName]()
const chainJointNames = CHAIN_JOINT_NAMES[props.robotName]
const jointAngles = reactive<Record<string, number>>({ ...INITIAL_ANGLES[props.robotName] })
for (const n in robot.joints) if (jointAngles[n] === undefined) jointAngles[n] = robot.joints[n].angle

function retrace() {
  for (const n in jointAngles) robot.joints[n].angle = jointAngles[n]
  buildFK(robot)
  tracer.load([...jacobianSteps(robot, chainJointNames, false)])
}
watch(jointAngles, retrace, { deep: true })

const partialJacobian = computed(() => {
  const s = tracer.current.value
  if (!s) return [] as number[][]
  return assembleJacobian(s.columnsSoFar)
})

/* ---------------- 3D scene ---------------- */
const viewport = ref<HTMLDivElement | null>(null)
let renderer: THREE.WebGLRenderer | null = null
let scene: THREE.Scene, camera: THREE.PerspectiveCamera, controls: OrbitControls
let raf = 0
const linkMeshes: Record<string, THREE.Mesh> = {}
let axisArrow: THREE.ArrowHelper
let leverArrow: THREE.ArrowHelper
let eeMarker: THREE.Mesh
let originMarker: THREE.Mesh

const NORMAL = 0x8a8f98
const ACTIVE = 0xffd34d

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

  renderer = new THREE.WebGLRenderer({ antialias: true })
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
    const mesh = new THREE.Mesh(geom, new THREE.MeshLambertMaterial({ color: robot.links[name].color ?? NORMAL }))
    mesh.matrixAutoUpdate = false
    scene.add(mesh)
    linkMeshes[name] = mesh
  }

  axisArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(), 0.001, 0xe8710a, 0.08, 0.05)
  axisArrow.visible = false
  scene.add(axisArrow)

  leverArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(), 0.001, 0x9b59b6, 0.08, 0.05)
  leverArrow.visible = false
  scene.add(leverArrow)

  eeMarker = new THREE.Mesh(new THREE.SphereGeometry(0.045, 16, 16), new THREE.MeshLambertMaterial({ color: 0xe8349a }))
  eeMarker.visible = false
  scene.add(eeMarker)

  originMarker = new THREE.Mesh(new THREE.SphereGeometry(0.035, 16, 16), new THREE.MeshLambertMaterial({ color: 0x202b4e }))
  originMarker.visible = false
  scene.add(originMarker)

  const loop = () => { raf = requestAnimationFrame(loop); controls.update(); renderer!.render(scene, camera) }
  loop()
}

function syncScene() {
  for (const name in linkMeshes) {
    const m = robot.links[name].matrix as Mat4 | undefined
    const mesh = linkMeshes[name]
    if (m) mesh.matrix.copy(mat4(m))
    const active = chainJointNames.some((j) => robot.joints[j].child === name && j === tracer.current.value?.jointName)
    ;(mesh.material as THREE.MeshLambertMaterial).color.setHex(active ? ACTIVE : (robot.links[name].color ?? NORMAL))
  }

  const s = tracer.current.value
  if (!s || s.jointIndex < 0) {
    axisArrow.visible = false
    leverArrow.visible = false
    eeMarker.visible = false
    originMarker.visible = false
    return
  }
  eeMarker.visible = true
  eeMarker.position.set(...s.eeWorld)

  const hasAxis = s.axisWorld.some((v) => v !== 0)
  originMarker.visible = hasAxis
  if (hasAxis) originMarker.position.set(...s.originWorld)

  axisArrow.visible = hasAxis
  if (hasAxis) {
    axisArrow.position.set(...s.originWorld)
    axisArrow.setDirection(new THREE.Vector3(...s.axisWorld).normalize())
    axisArrow.setLength(0.55, 0.08, 0.05)
  }

  const hasLinear = s.linear.some((v) => v !== 0)
  leverArrow.visible = hasLinear
  if (hasLinear) {
    const lever = new THREE.Vector3(s.eeWorld[0] - s.originWorld[0], s.eeWorld[1] - s.originWorld[1], s.eeWorld[2] - s.originWorld[2])
    leverArrow.position.set(...s.originWorld)
    if (lever.length() > 1e-6) {
      leverArrow.setDirection(lever.clone().normalize())
      leverArrow.setLength(lever.length(), 0.07, 0.04)
    }
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
onBeforeUnmount(() => { cancelAnimationFrame(raf); ro?.disconnect(); renderer?.dispose() })

const stepLabel = computed(() => {
  const s = tracer.current.value
  if (!s) return '—'
  if (s.jointIndex < 0) return 'p_ee (fixed for the whole build)'
  if (s.jointIndex >= chainJointNames.length) return 'done -- transpose columns into robot.jacobian'
  return `joint ${s.jointIndex}: ${s.jointName}`
})
const fmt = (v: number) => (v < 0 ? '' : ' ') + v.toFixed(2)
</script>

<template>
  <div class="jac-shell">
    <div class="left">
      <div class="code-pane">
        <PseudocodePanel :lines="MASTER_JACOBIAN_BUILD" :active-line="tracer.current.value?.line ?? -1" />
      </div>

      <div class="jac-pane">
        <div class="jac-title">robot.jacobian <span class="muted">(so far)</span></div>
        <div v-if="!partialJacobian.length" class="jac-empty">— not started —</div>
        <table v-else class="jac-table">
          <tr v-for="(row, i) in partialJacobian" :key="i">
            <td v-for="(v, j) in row" :key="j">{{ fmt(v) }}</td>
          </tr>
        </table>
      </div>

      <div class="controls">
        <label v-for="n in chainJointNames" :key="n" class="js">
          <span>{{ n }}</span>
          <input v-model.number="jointAngles[n]" type="range" min="-3.14159" max="3.14159" step="0.01">
        </label>
      </div>

      <div class="playback">
        <button class="btn" :disabled="tracer.isAtStart.value" @click="tracer.stepBack">&laquo; Step</button>
        <button v-if="!tracer.isRunning.value" class="btn primary" :disabled="tracer.isDone.value" @click="tracer.play">&#9654; Play</button>
        <button v-else class="btn primary" @click="tracer.pause">&#10074;&#10074; Pause</button>
        <button class="btn" :disabled="tracer.isDone.value" @click="tracer.stepForward">Step &raquo;</button>
        <button class="btn" @click="tracer.reset">&#8634; Reset</button>
        <span class="phase-tag">{{ stepLabel }}</span>
        <span class="frame-count">step {{ Math.max(0, tracer.pos.value + 1) }} / {{ tracer.stepCount.value }}</span>
      </div>
    </div>

    <div class="right">
      <div ref="viewport" class="viewport" />
    </div>
  </div>
</template>

<style scoped>
.jac-shell { display: grid; grid-template-columns: 1.05fr 0.95fr; gap: 0.7em; height: 100%; min-height: 0; }
.left, .right { display: flex; flex-direction: column; min-height: 0; min-width: 0; gap: 0.35em; }
.left { overflow-y: auto; }
.code-pane { flex: 0 1 150px; min-height: 90px; display: flex; }

.jac-pane {
  flex: 0 0 auto; border: 1px solid #dcdfe6; border-radius: 8px; background: #fff;
  padding: 0.35em 0.5em; font-family: var(--font-mono, monospace); font-size: 0.6em; max-height: 30%;
  overflow: auto;
}
.jac-title { font-weight: 700; color: var(--indigo, #202B4E); margin-bottom: 0.3em; }
.jac-title .muted { font-weight: 400; opacity: 0.6; }
.jac-empty { opacity: 0.5; }
.jac-table { border-collapse: collapse; }
.jac-table td { padding: 0.1em 0.5em; text-align: right; border-bottom: 1px solid #eee; }

.controls { flex: 0 0 auto; display: flex; flex-wrap: wrap; gap: 0.5em; }
.js { display: flex; align-items: center; gap: 0.25em; font-size: 0.62em; font-family: var(--font-mono, monospace); }
.js input[type="range"] { width: 4.6em; }

.viewport { flex: 1 1 auto; min-height: 0; border-radius: 8px; overflow: hidden; border: 1px solid #dcdfe6; }
.viewport :deep(canvas) { display: block; }

.playback { flex: 0 0 auto; display: flex; align-items: center; gap: 0.4em; flex-wrap: wrap; }
.phase-tag {
  font-family: var(--font-mono, monospace); font-size: 0.62em; font-weight: 700;
  background: #fdecd9; border: 1px solid #202B4E; color: #202B4E; border-radius: 999px; padding: 0.15em 0.6em;
}
.frame-count { font-family: var(--font-mono, monospace); font-size: 0.58em; opacity: 0.6; }
</style>
