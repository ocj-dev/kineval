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
// (STL/Collada/OBJ). Each asset under public/meshes/ is a REAL robot mesh
// that some real URDF names in a <mesh filename="...">: Fetch's wheel and
// head-pan link, and Spot's lower leg (neither Fetch nor PR2 ships OBJ --
// see that directory's README). Loaded with the matching three.js loader at
// runtime, not a screenshot.

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

  // These are real robot meshes, so they arrive at real-world scale (a Fetch
  // wheel is ~0.1m, a Spot leg ~0.3m) and with arbitrary modelling origins --
  // Spot's leg in particular sits well off its own origin. Recentre and pull
  // the camera back to the bounding sphere so every format's example frames
  // itself identically regardless of its units or origin.
  //
  // Note this shifts the LOADED OBJECT within `group`, not `group` itself:
  // the animation loop spins `group`, so anything still offset from the
  // group's origin would swing around it instead of turning in place.
  function frameObject(object: any) {
    const box = new THREE.Box3().setFromObject(object)
    if (box.isEmpty()) return
    const centre = box.getCenter(new THREE.Vector3())
    object.position.sub(centre)

    const radius = box.getBoundingSphere(new THREE.Sphere()).radius || 1
    const dist = radius / Math.sin((camera.fov * Math.PI) / 180 / 2)
    camera.position.set(dist * 0.62, dist * 0.5, dist * 0.68)
    camera.near = dist / 100
    camera.far = dist * 100
    camera.updateProjectionMatrix()
    controls.target.set(0, 0, 0)
    controls.update()
  }

  // resolve against the deck's deployed base path (the built deck is served
  // from /kineval/forward_kinematics/, not the site root, so a bare
  // "/meshes/..." would 404 once deployed)
  const url = import.meta.env.BASE_URL.replace(/\/$/, '') + '/' + props.meshUrl.replace(/^\//, '')

  if (props.format === 'stl') {
    new STLLoader().load(url, (geometry: any) => {
      const mesh = new THREE.Mesh(geometry, material)
      group.add(mesh)
      frameObject(mesh)
    })
  } else if (props.format === 'obj') {
    new OBJLoader().load(url, (obj: any) => {
      obj.traverse((child: any) => { if (child.isMesh) child.material = material })
      group.add(obj)
      frameObject(obj)
    })
  } else {
    // Collada carries its own materials/textures -- deliberately left intact
    // here rather than overridden, since that capability is the point of the
    // format's slide.
    new ColladaLoader().load(url, (result: any) => {
      group.add(result.scene)
      frameObject(result.scene)
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
