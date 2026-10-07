<script setup lang="ts">
import { ref, onMounted, onBeforeUnmount } from 'vue'
// Vite bundles this component's imports at build time, which can't follow
// the reference implementation's own browser-only <script type="importmap">
// trick -- so this one Vite-side component uses the npm `three` package
// directly (added to package.json) instead of reaching into
// ../../reference/vendor/three/ the way slides.md's plain-text code
// snippets do. The vendored copy there still backs the actual, buildless
// forward_kinematics.html reference page; this is a second, independent
// copy for this one interactive panel.
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js'
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js'

// Live three.js-rendered example geometry for each mesh-file-format slide
// (STL/Collada/OBJ) -- one small hand-authored sample asset per format
// under public/meshes/ (see that directory's own note), loaded with the
// matching three.js loader at runtime, not a screenshot.

const props = defineProps<{ format: 'stl' | 'obj' | 'collada'; meshUrl: string; color?: number }>()

const container = ref<HTMLDivElement | null>(null)
let renderer: THREE.WebGLRenderer | null = null
let animationHandle = 0

onMounted(() => {
  if (!container.value) return
  const el = container.value

  const scene = new THREE.Scene()
  scene.background = new THREE.Color(0xf5f5f5)
  // el.clientWidth/clientHeight are not yet reliable here -- onMounted fires
  // right after this element is inserted into the DOM, often before the
  // browser has run the layout pass that gives its flex/grid ancestors
  // (content-body -> split-panel -> side-image) their actual pixel sizes,
  // especially when mounting as part of a larger batch of slide-transition
  // DOM insertions. Seed the camera/renderer with a placeholder aspect and
  // let the ResizeObserver below (which fires once real layout is known,
  // and again on every subsequent resize) set the real size.
  const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50)
  camera.position.set(2, 1.6, 2.2)

  renderer = new THREE.WebGLRenderer({ antialias: true })
  renderer.setPixelRatio(window.devicePixelRatio || 1)
  el.appendChild(renderer.domElement)

  const controls = new OrbitControls(camera, renderer.domElement)
  controls.enableDamping = true

  scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 1.2))
  const sun = new THREE.DirectionalLight(0xffffff, 0.8)
  sun.position.set(2, 3, 2)
  scene.add(sun)

  const group = new THREE.Group()
  scene.add(group)
  const material = new THREE.MeshLambertMaterial({ color: props.color ?? 0xd98236 })

  if (props.format === 'stl') {
    new STLLoader().load(props.meshUrl, (geometry: any) => {
      geometry.center()
      group.add(new THREE.Mesh(geometry, material))
    })
  } else if (props.format === 'obj') {
    new OBJLoader().load(props.meshUrl, (obj: any) => {
      obj.traverse((child: any) => { if (child.isMesh) child.material = material })
      group.add(obj)
    })
  } else {
    new ColladaLoader().load(props.meshUrl, (result: any) => {
      group.add(result.scene)
    })
  }

  function frame() {
    animationHandle = requestAnimationFrame(frame)
    group.rotation.y += 0.006
    controls.update()
    renderer!.render(scene, camera)
  }
  frame()

  const resizeObserver = new ResizeObserver((entries) => {
    const { width, height } = entries[0].contentRect
    if (width === 0 || height === 0 || !renderer) return
    camera.aspect = width / height
    camera.updateProjectionMatrix()
    renderer.setSize(width, height)
  })
  resizeObserver.observe(el)

  onBeforeUnmount(() => {
    cancelAnimationFrame(animationHandle)
    resizeObserver.disconnect()
    renderer?.dispose()
  })
})
</script>

<template>
  <div ref="container" class="mesh-format-viewport" />
</template>

<style scoped>
.mesh-format-viewport { width: 100%; height: 100%; min-height: 0; border-radius: 10px; overflow: hidden; }
.mesh-format-viewport :deep(canvas) { display: block; }
</style>
