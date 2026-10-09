<script setup lang="ts">
import { reactive, ref, onMounted, onBeforeUnmount, watch, computed } from 'vue'
import {
  matrixMultiply, generateRotationMatrixX, generateRotationMatrixY, generateRotationMatrixZ,
} from '../lib/fk/matrixMath'
import { clearCanvas, INDIGO, AMBER } from '../lib/fk/drawUtils'

// A three-ring gyroscope for the gimbal-lock slide. The rings are nested in
// the SAME order the deck's own convention composes them -- R = Rz Ry Rx,
// AutoRob's XYZ order -- so the outer ring carries yaw, the middle ring
// pitch, and the inner ring roll. The inner ring's own axis is therefore
// stacked through both of its parents, which is exactly why driving the
// middle ring to +/-90 degrees swings the inner axis onto the outer one and
// costs a degree of freedom.

const a = reactive({ roll: 0.0, pitch: 0.0, yaw: 0.0 })  // theta_x, theta_y, theta_z
const canvasEl = ref<HTMLCanvasElement | null>(null)

const LOCK_TOL = 0.12   // radians either side of +/-90deg that we call locked
const lockCloseness = computed(() => {
  // distance from the nearest gimbal-lock pitch (+/- pi/2)
  const d = Math.min(
    Math.abs(a.pitch - Math.PI / 2),
    Math.abs(a.pitch + Math.PI / 2),
  )
  return d
})
const isLocked = computed(() => lockCloseness.value < LOCK_TOL)

// R = Rz(yaw) Ry(pitch) Rx(roll), the same composition matrix_from_rpy uses
const R = computed(() =>
  matrixMultiply(
    generateRotationMatrixZ(a.yaw),
    matrixMultiply(generateRotationMatrixY(a.pitch), generateRotationMatrixX(a.roll)),
  ),
)

const PITCH_VIEW = 0.38
const YAW_VIEW = 0.62
function project(x: number, y: number, z: number, scale: number): [number, number] {
  const cy = Math.cos(YAW_VIEW), sy = Math.sin(YAW_VIEW)
  const cp = Math.cos(PITCH_VIEW), sp = Math.sin(PITCH_VIEW)
  const X = x * cy + z * sy
  const Zc = -x * sy + z * cy
  const Y = y * cp - Zc * sp
  return [X * scale, -Y * scale]
}

// apply a 4x4 (we only use the rotation block) to a 3-vector
function apply(m: number[][], v: number[]): [number, number, number] {
  return [
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
  ]
}

function redraw() {
  const canvas = canvasEl.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  const rect = canvas.getBoundingClientRect()
  if (rect.width === 0 || rect.height === 0) return
  canvas.width = rect.width; canvas.height = rect.height
  clearCanvas(ctx, canvas.width, canvas.height)

  const scale = Math.min(canvas.width, canvas.height) * 0.30
  const cx = canvas.width * 0.5
  const cy = canvas.height * 0.52
  const S = (p: [number, number]): [number, number] => [cx + p[0], cy + p[1]]

  // Each ring is a circle in a plane, carried by the rotations OUTSIDE it.
  // outer (yaw, about +y): carried by nothing
  // middle (pitch, about +z): carried by Rz(yaw)
  // inner (roll, about +x): carried by Rz(yaw) Ry(pitch)
  const Rz = generateRotationMatrixZ(a.yaw)
  const Rzy = matrixMultiply(Rz, generateRotationMatrixY(a.pitch))

  function drawRing(carrier: number[][], normal: [number, number, number], radius: number, color: string, width: number) {
    // basis for the ring plane (perpendicular to `normal`)
    const n = normal
    const tmp: [number, number, number] = Math.abs(n[1]) < 0.9 ? [0, 1, 0] : [1, 0, 0]
    const e1: [number, number, number] = [
      n[1] * tmp[2] - n[2] * tmp[1], n[2] * tmp[0] - n[0] * tmp[2], n[0] * tmp[1] - n[1] * tmp[0],
    ]
    const l1 = Math.hypot(...e1) || 1
    const u1: [number, number, number] = [e1[0] / l1, e1[1] / l1, e1[2] / l1]
    const u2: [number, number, number] = [
      n[1] * u1[2] - n[2] * u1[1], n[2] * u1[0] - n[0] * u1[2], n[0] * u1[1] - n[1] * u1[0],
    ]

    ctx.strokeStyle = color
    ctx.lineWidth = width
    ctx.beginPath()
    const N = 90
    for (let i = 0; i <= N; i++) {
      const t = (i / N) * Math.PI * 2
      const local: [number, number, number] = [
        (u1[0] * Math.cos(t) + u2[0] * Math.sin(t)) * radius,
        (u1[1] * Math.cos(t) + u2[1] * Math.sin(t)) * radius,
        (u1[2] * Math.cos(t) + u2[2] * Math.sin(t)) * radius,
      ]
      const w = apply(carrier, local)
      const [sx, sy] = S(project(w[0], w[1], w[2], scale))
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy)
    }
    ctx.closePath(); ctx.stroke()
  }

  function drawAxis(carrier: number[][], dir: [number, number, number], len: number, color: string, dashed: boolean, width = 2) {
    const p1 = apply(carrier, [dir[0] * len, dir[1] * len, dir[2] * len])
    const p2 = apply(carrier, [-dir[0] * len, -dir[1] * len, -dir[2] * len])
    const [x1, y1] = S(project(p1[0], p1[1], p1[2], scale))
    const [x2, y2] = S(project(p2[0], p2[1], p2[2], scale))
    ctx.strokeStyle = color; ctx.lineWidth = width
    if (dashed) ctx.setLineDash([5, 4])
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke()
    ctx.setLineDash([])
  }

  const lockRed = '#c0392b'
  const outerCol = isLocked.value ? lockRed : INDIGO
  const innerCol = isLocked.value ? lockRed : '#3b6ea5'

  // rings, outermost first
  drawRing([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 1, 0], 1.0, outerCol, isLocked.value ? 3 : 2)   // yaw
  drawRing(Rz, [0, 0, 1], 0.78, '#27966b', 2)                                                      // pitch
  drawRing(Rzy, [1, 0, 0], 0.56, innerCol, isLocked.value ? 3 : 2)                                 // roll

  // the three gimbal axes
  drawAxis([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 1, 0], 1.25, outerCol, true)   // yaw axis (world +y)
  drawAxis(Rz, [0, 0, 1], 1.0, '#27966b', true)                                   // pitch axis
  drawAxis(Rzy, [1, 0, 0], 0.8, innerCol, true)                                   // roll axis

  // the spinning rotor the whole rig is orienting: the body +x after full R
  const body = apply(R.value, [0.46, 0, 0])
  const [bx, by] = S(project(body[0], body[1], body[2], scale))
  ctx.strokeStyle = AMBER; ctx.fillStyle = AMBER; ctx.lineWidth = 3
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(bx, by); ctx.stroke()
  ctx.beginPath(); ctx.arc(bx, by, 6, 0, Math.PI * 2); ctx.fill()

  if (isLocked.value) {
    ctx.fillStyle = lockRed
    ctx.font = 'bold 13px "Roboto Mono", monospace'
    ctx.textAlign = 'center'
    ctx.fillText('GIMBAL LOCK — roll and yaw now turn the same axis', cx, canvas.height - 10)
  }
}

watch([() => a.roll, () => a.pitch, () => a.yaw], redraw)

let resizeObserver: ResizeObserver | null = null
onMounted(() => {
  if (!canvasEl.value) return
  resizeObserver = new ResizeObserver(() => redraw())
  resizeObserver.observe(canvasEl.value)
  redraw()
})
onBeforeUnmount(() => resizeObserver?.disconnect())

const fmt = (v: number) => (v >= 0 ? ' ' : '') + v.toFixed(3)
</script>

<template>
  <div class="gimbal-panel">
    <div class="controls">
      <label><span class="k" style="color:#3b6ea5">roll &theta;<sub>x</sub></span>
        <input v-model.number="a.roll" type="range" min="-3.14159" max="3.14159" step="0.01"></label>
      <label><span class="k" style="color:#27966b">pitch &theta;<sub>y</sub></span>
        <input v-model.number="a.pitch" type="range" min="-1.5708" max="1.5708" step="0.005"></label>
      <label><span class="k" :style="{ color: '#202B4E' }">yaw &theta;<sub>z</sub></span>
        <input v-model.number="a.yaw" type="range" min="-3.14159" max="3.14159" step="0.01"></label>
    </div>

    <div class="vector-canvas-wrap">
      <canvas ref="canvasEl" />
    </div>

    <div class="matrix-row" :class="{ locked: isLocked }">
      <div class="mlabel">R = R<sub>z</sub>(&theta;<sub>z</sub>) R<sub>y</sub>(&theta;<sub>y</sub>) R<sub>x</sub>(&theta;<sub>x</sub>)</div>
      <table class="mat">
        <tr v-for="(row, i) in [0, 1, 2]" :key="i">
          <td v-for="j in [0, 1, 2]" :key="j">{{ fmt(R[i][j]) }}</td>
        </tr>
      </table>
      <div class="status">
        <template v-if="isLocked">
          <b>singular:</b> &theta;<sub>y</sub> = &plusmn;90&deg;<br>3 DOF &rarr; 2
        </template>
        <template v-else>
          non-singular<br>{{ (lockCloseness * 180 / Math.PI).toFixed(0) }}&deg; from lock
        </template>
      </div>
    </div>
  </div>
</template>

<style scoped>
.gimbal-panel { display: flex; flex-direction: column; height: 100%; gap: 0.3em; min-height: 0; }
.controls { display: flex; flex-wrap: wrap; gap: 0.7em; font-size: 0.68em; font-family: var(--font-mono, monospace); }
.controls label { display: flex; align-items: center; gap: 0.3em; }
.controls .k { font-weight: 700; }
.controls input[type="range"] { width: 5.4em; }
.matrix-row {
  display: flex; align-items: center; gap: 0.8em;
  font-family: var(--font-mono, monospace); font-size: 0.66em;
  border: 1px solid #dcdfe6; border-radius: 8px; padding: 0.35em 0.6em; background: #fff;
}
.matrix-row.locked { border-color: #c0392b; background: #fdf2f1; color: #c0392b; }
.mlabel { font-weight: 700; color: var(--indigo, #202B4E); }
.matrix-row.locked .mlabel, .matrix-row.locked .status { color: #c0392b; }
.mat { border-collapse: collapse; }
.mat td { padding: 0 0.35em; text-align: right; }
.status { margin-left: auto; text-align: right; line-height: 1.3; }
</style>
