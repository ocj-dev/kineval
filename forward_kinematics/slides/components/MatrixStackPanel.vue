<script setup lang="ts">
import { reactive, ref, watch, onMounted, onBeforeUnmount, computed } from 'vue'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import PseudocodePanel from './PseudocodePanel.vue'
import { useFkTracer } from '../lib/fk/useFkTracer'
import { createMobileArmRobot, createUrdfExampleRobot } from '../lib/fk/robots'
import { MASTER_PSEUDOCODE } from '../lib/fk/pseudocode'
import type { Mat4, RobotDesc } from '../lib/fk/types'

// Steps the DFS of buildFKTransforms/traverseFKLink/traverseFKJoint one node
// at a time, in 3D: each link's box appears exactly when the traversal
// reaches it, the frame being worked on is highlighted, and the panel on the
// left shows the live call stack -- which IS the matrix stack, since the
// transforms are held in the recursion rather than in a separate structure.

const props = withDefaults(defineProps<{ robotName?: 'mobile_arm' | 'urdf_example' }>(), {
  robotName: 'urdf_example',
})

const speedMs = ref(420)
const tracer = useFkTracer(speedMs)

const robot: RobotDesc = props.robotName === 'mobile_arm' ? createMobileArmRobot() : createUrdfExampleRobot()
const jointAngles = reactive<Record<string, number>>(
  Object.fromEntries(Object.keys(robot.joints).map((n) => [n, robot.joints[n].angle])),
)

function retrace() {
  for (const n in jointAngles) robot.joints[n].angle = jointAngles[n]
  tracer.load(robot)
}
watch(jointAngles, retrace, { deep: true })

/* ---------------- 3D scene ---------------- */
const viewport = ref<HTMLDivElement | null>(null)
let renderer: THREE.WebGLRenderer | null = null
let scene: THREE.Scene, camera: THREE.PerspectiveCamera, controls: OrbitControls
let raf = 0
const linkMeshes: Record<string, THREE.Mesh> = {}
let frameHelper: THREE.AxesHelper

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
  camera.position.set(1.15, 0.95, 1.35)

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
    const g = robot.links[name].geometry
    const geom = new THREE.BoxGeometry(g.size[0], g.size[1], g.size[2])
    geom.translate(g.offset.xyz[0], g.offset.xyz[1], g.offset.xyz[2])
    const mesh = new THREE.Mesh(geom, new THREE.MeshLambertMaterial({ color: NORMAL }))
    mesh.matrixAutoUpdate = false
    mesh.visible = false
    scene.add(mesh)
    linkMeshes[name] = mesh
  }

  // marks the frame the traversal is holding right now
  frameHelper = new THREE.AxesHelper(0.33)
  frameHelper.matrixAutoUpdate = false
  frameHelper.visible = false
  scene.add(frameHelper)

  const loop = () => { raf = requestAnimationFrame(loop); controls.update(); renderer!.render(scene, camera) }
  loop()
}

function syncScene() {
  const revealed = tracer.revealedLinks.value
  for (const name in linkMeshes) {
    const m = revealed.get(name)
    const mesh = linkMeshes[name]
    mesh.visible = !!m
    if (m) mesh.matrix.copy(mat4(m))
    const active = name === tracer.currentLinkName.value
    ;(mesh.material as THREE.MeshLambertMaterial).color.setHex(active ? ACTIVE : NORMAL)
  }
  const cur = tracer.currentMatrix.value
  frameHelper.visible = !!cur
  if (cur) frameHelper.matrix.copy(mat4(cur))
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
  if (tracer.currentJointName.value) return 'joint: ' + tracer.currentJointName.value
  if (tracer.currentLinkName.value) return 'link: ' + tracer.currentLinkName.value
  return '—'
})
const fmt = (v: number) => (v < 0 ? '' : ' ') + v.toFixed(2)
</script>

<template>
  <div class="fk-shell">
    <!-- left: pseudocode + live call stack -->
    <div class="left">
      <div class="code-pane">
        <PseudocodePanel :lines="MASTER_PSEUDOCODE" :active-line="tracer.activeLine.value" />
      </div>

      <div class="stack-pane">
        <div class="stack-title">
          call stack <span class="muted">(the matrix stack)</span>
          <span class="depth">depth {{ tracer.stack.value.length }}</span>
        </div>
        <div v-if="!tracer.stack.value.length" class="stack-empty">— not started —</div>
        <div
          v-for="(f, i) in [...tracer.stack.value].reverse()" :key="i"
          class="frame" :class="{ top: i === 0, ['k-' + f.kind]: true }"
        >
          <span class="fn">{{ f.kind === 'joint' ? 'traverseFKJoint' : f.kind === 'link' ? 'traverseFKLink' : 'traverseFKBase' }}</span>
          <span class="nm">{{ f.name }}</span>
          <span class="tx">[{{ fmt(f.matrix[0][3]) }},{{ fmt(f.matrix[1][3]) }},{{ fmt(f.matrix[2][3]) }}]</span>
        </div>
      </div>
    </div>

    <!-- right: the robot in 3D -->
    <div class="right">
      <div class="controls">
        <label v-for="n in Object.keys(jointAngles)" :key="n" class="js">
          <span>{{ n }}</span>
          <input v-model.number="jointAngles[n]" type="range" min="-3.14159" max="3.14159" step="0.01">
        </label>
      </div>

      <div ref="viewport" class="viewport" />

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
  </div>
</template>

<style scoped>
.fk-shell { display: grid; grid-template-columns: 1.12fr 0.88fr; gap: 0.7em; height: 100%; min-height: 0; }
.left, .right { display: flex; flex-direction: column; min-height: 0; min-width: 0; gap: 0.4em; }
.code-pane { flex: 1 1 auto; min-height: 0; display: flex; }

.stack-pane {
  flex: 0 0 auto; border: 1px solid #dcdfe6; border-radius: 8px; background: #fff;
  padding: 0.35em 0.5em; font-family: var(--font-mono, monospace); font-size: 0.6em; max-height: 38%;
  overflow-y: auto;
}
.stack-title { font-weight: 700; color: var(--indigo, #202B4E); display: flex; align-items: center; gap: 0.4em; margin-bottom: 0.2em; }
.stack-title .muted { font-weight: 400; opacity: 0.6; }
.stack-title .depth { margin-left: auto; opacity: 0.7; }
.stack-empty { opacity: 0.5; }
.frame { display: flex; gap: 0.4em; padding: 0.1em 0.25em; border-left: 3px solid transparent; white-space: nowrap; }
.frame.k-joint { border-left-color: #E8710A; }
.frame.k-link { border-left-color: #3b6ea5; }
.frame.k-base { border-left-color: #27966b; }
.frame.top { background: #fdecd9; font-weight: 700; }
.frame .nm { color: var(--indigo, #202B4E); }
.frame .tx { margin-left: auto; opacity: 0.65; }

.controls { display: flex; flex-wrap: wrap; gap: 0.5em; }
.js { display: flex; align-items: center; gap: 0.25em; font-size: 0.62em; font-family: var(--font-mono, monospace); }
.js input[type="range"] { width: 4.6em; }

.viewport { flex: 1 1 auto; min-height: 0; border-radius: 8px; overflow: hidden; border: 1px solid #dcdfe6; }
.viewport :deep(canvas) { display: block; }

.playback { display: flex; align-items: center; gap: 0.4em; flex-wrap: wrap; }
.phase-tag {
  font-family: var(--font-mono, monospace); font-size: 0.62em; font-weight: 700;
  background: #fdecd9; border: 1px solid #202B4E; color: #202B4E; border-radius: 999px; padding: 0.15em 0.6em;
}
.frame-count { font-family: var(--font-mono, monospace); font-size: 0.58em; opacity: 0.6; }
</style>
