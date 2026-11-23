<script setup lang="ts">
import { ref, shallowRef, computed, onMounted, onBeforeUnmount } from 'vue'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import TreeDiagram from './TreeDiagram.vue'
import { createRobotMr2 } from '../lib/bt/robots'
import { buildFK } from '../lib/bt/fk'
import { PLAN, PLAN_POSITIONS, planTrajectory, inspectionTrajectories, PARTS_PRESENT } from '../lib/bt/plan'
import { trajectoryDuration } from '../lib/bt/trajectory'
import {
  JTA_ACCEPTED, JTA_EXECUTING,
  jointTrajectoryActionCancel, jointTrajectoryActionHold, jointTrajectoryActionSendGoal,
  jointTrajectoryActionUpdate, jointTrajectoryActionServer,
} from '../lib/bt/action'
import { btBlackboard, btTickTree } from '../lib/bt/behaviorTree'
import { treeInspectionQueue, treePlanAndExecute, treeReactiveBattery } from '../lib/bt/trees'
import { simulateBattery } from '../lib/bt/tasks'
import type { ActionServer, Blackboard, BtNode, Mat4, RobotDesc, Status, TraceEntry } from '../lib/bt/types'

// The arm executing the committed plan, either driven straight by the
// action server (mode "action") or by one of the behavior trees (the
// other three modes).
//
// Everything on screen is produced by the TS ports of the reference
// implementation -- the same tick engine, the same trajectory sampling,
// the same servo and the same tolerance checks the slides quote. The
// desired marker is where the trajectory says the endeffector should be;
// the actual marker is where it is. They are never in the same place
// while the arm is moving, and that gap is the feedback message's
// `error` field.
const props = withDefaults(defineProps<{
  mode?: 'action' | 'plan_execute' | 'inspection' | 'reactive'
  reactive?: boolean
  showTree?: boolean
  showObstacles?: boolean
  rate?: number
  gain?: number
  pathTolerance?: number
  autoplay?: boolean
}>(), {
  mode: 'action', reactive: true, showTree: true, showObstacles: true,
  rate: 20, gain: 0.35, pathTolerance: 0.25, autoplay: false,
})

const trajectory = planTrajectory()
const duration = trajectoryDuration(trajectory)

const gain = ref(props.gain)
const pathTolerance = ref(props.pathTolerance)
const isRunning = ref(props.autoplay)
const tickCount = ref(0)
const treeStatus = ref<Status | null>(null)
const trace = shallowRef<TraceEntry[]>([])

let robot: RobotDesc
let server: ActionServer
let blackboard: Blackboard
const root = shallowRef<BtNode | null>(null)

function freshRobot(): RobotDesc {
  const r = createRobotMr2()
  for (let j = 0; j < PLAN.joint_names.length; j++) {
    r.joints[PLAN.joint_names[j]].angle = PLAN_POSITIONS[0][j]
  }
  buildFK(r)
  return r
}

function setup() {
  robot = freshRobot()
  server = jointTrajectoryActionServer(robot, { gain: gain.value })
  tickCount.value = 0
  treeStatus.value = null
  trace.value = []

  if (props.mode === 'action') {
    blackboard = btBlackboard({ trajectory })
    root.value = null
    jointTrajectoryActionSendGoal(server, { trajectory, path_tolerance: pathTolerance.value })
  } else if (props.mode === 'inspection') {
    blackboard = btBlackboard({ parts_present: { ...PARTS_PRESENT } })
    root.value = treeInspectionQueue(server, inspectionTrajectories())
  } else if (props.mode === 'reactive') {
    blackboard = btBlackboard({ battery: 30 })
    root.value = treeReactiveBattery(server, PLAN, PLAN.joint_names, { reactive: props.reactive })
  } else {
    blackboard = btBlackboard({})
    root.value = treePlanAndExecute(server, PLAN, PLAN.joint_names)
  }
}

function tickOnce() {
  const dt = 1 / props.rate
  blackboard.dt = dt
  blackboard.now += dt
  server.gain = gain.value

  if (root.value) {
    if (props.mode === 'reactive') simulateBattery(blackboard)
    const run = btTickTree(root.value, blackboard)
    treeStatus.value = run.status
    trace.value = run.trace
    jointTrajectoryActionHold(server)
  } else {
    if (server.goal) server.goal.path_tolerance = pathTolerance.value
    jointTrajectoryActionUpdate(server, dt)
  }

  tickCount.value++
  buildFK(robot)
  syncScene()
}

const actionState = computed(() => { void tickCount.value; return server?.state ?? null })
const feedback = computed(() => {
  // depend on tickCount so the readout refreshes with the tick rather
  // than only when a shallowRef happens to change identity
  void tickCount.value
  return server?.feedback ?? null
})
const result = computed(() => { void tickCount.value; return server?.result ?? null })
const stats = computed(() => { void tickCount.value; return server?.stats })
const battery = computed(() => { void tickCount.value; return blackboard?.battery as number | undefined })
const logTail = computed(() => { void tickCount.value; return blackboard ? blackboard.log.slice(-3) : [] })

//////////////////////////////////////////////////
/////   THE THREE.JS VIEW
//////////////////////////////////////////////////

const viewport = ref<HTMLDivElement | null>(null)
let renderer: THREE.WebGLRenderer | null = null
let scene: THREE.Scene, camera: THREE.PerspectiveCamera, controls: OrbitControls
let raf = 0
let lastTickMs = 0
const linkMeshes: Record<string, THREE.Mesh> = {}
let desiredMarker: THREE.Mesh, actualMarker: THREE.Mesh
let ghostRobot: RobotDesc

function mat4(m: Mat4) {
  return new THREE.Matrix4().set(
    m[0][0], m[0][1], m[0][2], m[0][3],
    m[1][0], m[1][1], m[1][2], m[1][3],
    m[2][0], m[2][1], m[2][2], m[2][3],
    m[3][0], m[3][1], m[3][2], m[3][3],
  )
}

function endeffectorFor(positions: number[], names: string[]): [number, number, number] {
  if (!ghostRobot) ghostRobot = createRobotMr2()
  for (let i = 0; i < names.length; i++) ghostRobot.joints[names[i]].angle = positions[i]
  buildFK(ghostRobot)
  const frame = ghostRobot.joints[ghostRobot.endeffector.frame].matrix!
  const p = ghostRobot.endeffector.position
  return [
    frame[0][0] * p[0][0] + frame[0][1] * p[1][0] + frame[0][2] * p[2][0] + frame[0][3],
    frame[1][0] * p[0][0] + frame[1][1] * p[1][0] + frame[1][2] * p[2][0] + frame[1][3],
    frame[2][0] * p[0][0] + frame[2][1] * p[1][0] + frame[2][2] * p[2][0] + frame[2][3],
  ]
}

function buildScene() {
  const el = viewport.value!
  scene = new THREE.Scene()
  scene.background = new THREE.Color(0xf5f5f5)

  camera = new THREE.PerspectiveCamera(48, 1, 0.05, 100)
  camera.position.set(3.4, 2.6, 3.9)

  renderer = new THREE.WebGLRenderer({ antialias: true })
  renderer.setPixelRatio(window.devicePixelRatio || 1)
  el.appendChild(renderer.domElement)

  controls = new OrbitControls(camera, renderer.domElement)
  controls.enableDamping = true
  controls.target.set(0, 1.1, 0)

  scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 1.2))
  const sun = new THREE.DirectionalLight(0xffffff, 0.85)
  sun.position.set(3, 5, 2)
  scene.add(sun)
  scene.add(new THREE.GridHelper(8, 16, 0xc7cad1, 0xdddfe4))

  for (const name in robot.links) {
    const g = robot.links[name].geometry!
    const geom = new THREE.BoxGeometry(g.size[0], g.size[1], g.size[2])
    geom.translate(g.offset.xyz[0], g.offset.xyz[1], g.offset.xyz[2])
    const mesh = new THREE.Mesh(geom, new THREE.MeshLambertMaterial({ color: robot.links[name].color ?? 0x8a8f98 }))
    mesh.matrixAutoUpdate = false
    scene.add(mesh)
    linkMeshes[name] = mesh
  }

  if (props.showObstacles && PLAN.world) {
    const material = new THREE.MeshLambertMaterial({ color: 0xc0392b, transparent: true, opacity: 0.3 })
    for (const o of PLAN.world.obstacles) {
      const mesh = new THREE.Mesh(new THREE.SphereGeometry(o.radius, 18, 14), material)
      mesh.position.set(o.location[0][0], o.location[1][0], o.location[2][0])
      scene.add(mesh)
    }
  }

  // one ghost per plan waypoint, at that configuration's endeffector
  // position -- computed with the same FK the renderer uses, so the
  // ghosts cannot disagree with where the arm goes
  for (let i = 0; i < trajectory.points.length; i++) {
    const p = endeffectorFor(trajectory.points[i].positions, trajectory.joint_names)
    const mesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.05, 14, 10),
      new THREE.MeshLambertMaterial({
        color: i === 0 ? 0x3b6ea5 : i === trajectory.points.length - 1 ? 0x27966b : 0x8a8f98,
        transparent: true, opacity: 0.7,
      }),
    )
    mesh.position.set(p[0], p[1], p[2])
    scene.add(mesh)
  }

  desiredMarker = new THREE.Mesh(new THREE.SphereGeometry(0.055, 18, 14), new THREE.MeshLambertMaterial({ color: 0xe8349a }))
  actualMarker = new THREE.Mesh(new THREE.SphereGeometry(0.05, 18, 14), new THREE.MeshLambertMaterial({ color: 0xe8710a }))
  scene.add(desiredMarker)
  scene.add(actualMarker)

  const loop = (now: number) => {
    raf = requestAnimationFrame(loop)
    if (isRunning.value && now - lastTickMs >= 1000 / props.rate) {
      lastTickMs = now
      tickOnce()
    }
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
  const fb = server.feedback
  if (fb) {
    const d = endeffectorFor(fb.desired, fb.joint_names)
    const a = endeffectorFor(fb.actual, fb.joint_names)
    desiredMarker.position.set(d[0], d[1], d[2])
    actualMarker.position.set(a[0], a[1], a[2])
  }
}

let observer: ResizeObserver | null = null
onMounted(() => {
  setup()
  if (!viewport.value) return
  buildScene()
  syncScene()
  observer = new ResizeObserver((entries) => {
    const { width, height } = entries[0].contentRect
    if (!width || !height || !renderer) return
    camera.aspect = width / height
    camera.updateProjectionMatrix()
    renderer.setSize(width, height)
  })
  observer.observe(viewport.value)
  // one tick, so the tree arrives coloured and the readout has numbers in
  // it rather than dashes
  tickOnce()
})
onBeforeUnmount(() => {
  cancelAnimationFrame(raf)
  observer?.disconnect()
  renderer?.dispose()
})

function reset() {
  isRunning.value = false
  setup()
  buildFK(robot)
  syncScene()
}

function cancel() {
  const r = jointTrajectoryActionCancel(server, 'the Cancel button')
  if (r) blackboard.log.push('cancel button: ' + r.error_string)
  tickCount.value++
}

const executing = computed(() => actionState.value === JTA_EXECUTING || actionState.value === JTA_ACCEPTED)
</script>

<template>
  <div class="exec-shell">
    <div class="controls">
      <button class="btn" @click="isRunning = !isRunning">{{ isRunning ? 'Pause' : 'Play' }}</button>
      <button class="btn" @click="isRunning = false; tickOnce()">Tick once</button>
      <button class="btn" @click="reset">&#8634; Reset</button>
      <button class="btn" @click="cancel" :disabled="!executing">Cancel the goal</button>
      <label>gain <input v-model.number="gain" type="range" min="0.02" max="0.9" step="0.01"> {{ gain.toFixed(2) }}</label>
      <label v-if="mode === 'action'">
        path tol <input v-model.number="pathTolerance" type="range" min="0.005" max="0.4" step="0.005"> {{ pathTolerance.toFixed(3) }}
      </label>
    </div>

    <div class="body" :class="{ 'with-tree': showTree && root }">
      <div ref="viewport" class="viewport" />
      <div v-if="showTree && root" class="tree-pane">
        <TreeDiagram :root="root" :trace="trace" :column-width="156" :row-height="54" :font-size="9.5" />
      </div>
    </div>

    <div class="readout">
      <span class="tag" :class="actionState || ''">{{ actionState || 'no goal' }}</span>
      <span v-if="root">tree: <b>{{ treeStatus || '-' }}</b></span>
      <span>tick {{ tickCount }} &middot; t = {{ (blackboard?.now ?? 0).toFixed(2) }}s</span>
      <span v-if="feedback">
        point {{ feedback.index }}/{{ feedback.desired.length ? trajectory.points.length - 1 : 0 }}
        &middot; {{ feedback.header.stamp.toFixed(2) }}s of {{ feedback.duration.toFixed(2) }}s
      </span>
      <span v-if="feedback" class="err">worst error {{ feedback.worst_error.toFixed(4) }} rad</span>
      <span v-if="battery !== undefined">battery {{ battery.toFixed(0) }}%</span>
      <span v-if="stats">goals {{ stats.goals }} / ok {{ stats.succeeded }} / cancelled {{ stats.canceled }} / aborted {{ stats.aborted }}</span>
      <span v-if="result && result.error_code !== 0" class="bad">error_code {{ result.error_code }}: {{ result.error_string }}</span>
    </div>
    <pre class="log">{{ logTail.join('\n') }}</pre>
  </div>
</template>

<style scoped>
.exec-shell { display: flex; flex-direction: column; height: 100%; min-height: 0; gap: 0.3em; }
.controls, .readout {
  display: flex; flex-wrap: wrap; align-items: center; gap: 0.7em;
  font-size: 0.58em; font-family: var(--font-mono, monospace);
}
.controls input[type="range"] { width: 4.5em; vertical-align: middle; }
.body { flex: 1 1 auto; min-height: 0; display: flex; gap: 0.4em; }
.body.with-tree .viewport { flex: 1 1 46%; }
.body.with-tree .tree-pane { flex: 1 1 54%; min-width: 0; }
.viewport {
  flex: 1 1 auto; min-height: 0; border-radius: 8px; overflow: hidden;
  border: 1px solid #dcdfe6; touch-action: none;
}
.viewport :deep(canvas) { display: block; }
.tag {
  font-weight: 700; border-radius: 999px; padding: 0.1em 0.6em;
  border: 1px solid #c7cad1; background: #fff;
}
.tag.executing, .tag.accepted { background: #dce9f8; border-color: #3b6ea5; color: #2d5580; }
.tag.succeeded { background: #e3f3ea; border-color: #27966b; color: #1d6e4f; }
.tag.canceled, .tag.aborted { background: #fdecd9; border-color: #c0392b; color: #a3302a; }
.err { color: #a3660a; }
.bad { color: #a3302a; }
.log {
  margin: 0; font-size: 0.54em; line-height: 1.45; opacity: 0.75;
  font-family: var(--font-mono, monospace); white-space: pre-wrap; max-height: 3.4em; overflow: hidden;
}
</style>
