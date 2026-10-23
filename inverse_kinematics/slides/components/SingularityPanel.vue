<script setup lang="ts">
import { reactive, ref, onMounted, onBeforeUnmount, watch, computed } from 'vue'
import { clearCanvas, INDIGO, AMBER, LOCK_RED } from '../lib/ik/drawUtils'

// A 2-link planar arm approaching the classic straight-arm Jacobian
// singularity -- the SAME red-highlight treatment
// forward_kinematics/slides/components/GimbalLockPanel.vue uses for gimbal
// lock (lock-red #c0392b on the offending geometry, a .locked-class border
// on the matrix-row readout, canvas caption switching text when flagged),
// adapted from "two rotation rings align" to "two link vectors align".
//
// Position-only planar Jacobian for this arm (base at origin, link lengths
// L1/L2, joint angles theta1/theta2 measured as in the deck's manipulator-
// Jacobian slide):
//   J = [ -L1 s1 - L2 s12   -L2 s12 ]
//       [  L1 c1 + L2 c12    L2 c12 ]   (c12 = cos(theta1+theta2), etc.)
// det(J) = L1 L2 sin(theta2) -- zero exactly when the elbow (theta2) is at
// 0 (arm fully extended) or +/-pi (arm folded back on itself): the two
// columns go parallel, exactly the "Jacobian loses rank" story the deck's
// prose tells. For a SQUARE J, det(J J^T) = det(J)^2, which is what the
// readout below actually displays (matching the brief: "determinant of
// J J^T (or J^T J) crossing near zero").

const L1 = 0.65, L2 = 0.55
const a = reactive({ theta1: 0.5, theta2: 0.9 }) // shoulder, elbow
const canvasEl = ref<HTMLCanvasElement | null>(null)

const DET_TOL = 0.05 * L1 * L2 // ~3 degrees of theta2 either side of 0/pi
const detJ = computed(() => L1 * L2 * Math.sin(a.theta2))
const detJJT = computed(() => detJ.value * detJ.value)
const isLocked = computed(() => Math.abs(detJ.value) < DET_TOL)

const elbow = computed<[number, number]>(() => [L1 * Math.cos(a.theta1), L1 * Math.sin(a.theta1)])
const wrist = computed<[number, number]>(() => {
  const [ex, ey] = elbow.value
  return [ex + L2 * Math.cos(a.theta1 + a.theta2), ey + L2 * Math.sin(a.theta1 + a.theta2)]
})

const jacobian = computed(() => {
  const s1 = Math.sin(a.theta1), c1 = Math.cos(a.theta1)
  const s12 = Math.sin(a.theta1 + a.theta2), c12 = Math.cos(a.theta1 + a.theta2)
  return [
    [-L1 * s1 - L2 * s12, -L2 * s12],
    [L1 * c1 + L2 * c12, L2 * c12],
  ]
})

function redraw() {
  const canvas = canvasEl.value
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  const rect = canvas.getBoundingClientRect()
  if (rect.width === 0 || rect.height === 0) return
  canvas.width = rect.width; canvas.height = rect.height
  clearCanvas(ctx, canvas.width, canvas.height)

  const scale = Math.min(canvas.width, canvas.height) * 0.42
  const ox = canvas.width * 0.32, oy = canvas.height * 0.62
  const S = (x: number, y: number): [number, number] => [ox + x * scale, oy - y * scale]

  const [ex, ey] = elbow.value
  const [wx, wy] = wrist.value
  const armCol = isLocked.value ? LOCK_RED : INDIGO

  // reach circle (where the wrist could be for ANY theta2, this theta1 fixed)
  ctx.strokeStyle = '#00000018'
  ctx.setLineDash([4, 5])
  ctx.beginPath()
  const [cex, cey] = S(ex, ey)
  ctx.arc(cex, cey, L2 * scale, 0, Math.PI * 2)
  ctx.stroke()
  ctx.setLineDash([])

  // link 1 (shoulder -> elbow)
  ctx.strokeStyle = armCol
  ctx.lineWidth = isLocked.value ? 7 : 6
  ctx.beginPath()
  const [ox0, oy0] = S(0, 0)
  ctx.moveTo(ox0, oy0)
  ctx.lineTo(cex, cey)
  ctx.stroke()

  // link 2 (elbow -> wrist)
  ctx.beginPath()
  ctx.moveTo(cex, cey)
  const [wxs, wys] = S(wx, wy)
  ctx.lineTo(wxs, wys)
  ctx.stroke()

  // joints
  ctx.fillStyle = INDIGO
  ;[[ox0, oy0], [cex, cey]].forEach(([x, y]) => { ctx.beginPath(); ctx.arc(x, y, 6, 0, Math.PI * 2); ctx.fill() })
  ctx.fillStyle = AMBER
  ctx.beginPath(); ctx.arc(wxs, wys, 7, 0, Math.PI * 2); ctx.fill()

  // base ground line
  ctx.strokeStyle = '#00000022'
  ctx.beginPath(); ctx.moveTo(0, oy0); ctx.lineTo(canvas.width, oy0); ctx.stroke()

  if (isLocked.value) {
    ctx.fillStyle = LOCK_RED
    ctx.font = 'bold 13px "Roboto Mono", monospace'
    ctx.textAlign = 'center'
    const why = Math.abs(((a.theta2 % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)) < Math.PI / 2 ? 'fully extended' : 'folded back'
    ctx.fillText(`SINGULAR -- arm ${why}, columns of J parallel`, canvas.width / 2, canvas.height - 10)
  }
}

watch([() => a.theta1, () => a.theta2], redraw)

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
  <div class="sing-panel">
    <div class="controls">
      <label><span class="k" style="color:#202B4E">shoulder &theta;<sub>1</sub></span>
        <input v-model.number="a.theta1" type="range" min="-3.14159" max="3.14159" step="0.01"></label>
      <label><span class="k" style="color:#E8710A">elbow &theta;<sub>2</sub></span>
        <input v-model.number="a.theta2" type="range" min="-3.14159" max="3.14159" step="0.01"></label>
      <button class="btn" style="font-size:0.62em; padding:0.3em 0.7em" @click="a.theta2 = 0.9">reset</button>
      <button class="btn" style="font-size:0.62em; padding:0.3em 0.7em" @click="a.theta2 = 0.001">drive to singular</button>
    </div>

    <div class="vector-canvas-wrap">
      <canvas ref="canvasEl" />
    </div>

    <div class="matrix-row" :class="{ locked: isLocked }">
      <div class="mlabel">J(&theta;)</div>
      <table class="mat">
        <tr v-for="(row, i) in jacobian" :key="i">
          <td v-for="(v, j) in row" :key="j">{{ fmt(v) }}</td>
        </tr>
      </table>
      <div class="status">
        <template v-if="isLocked">
          <b>det(JJ<sup>T</sup>) &approx; {{ detJJT.toFixed(4) }}</b><br>rank-deficient
        </template>
        <template v-else>
          det(JJ<sup>T</sup>) = {{ detJJT.toFixed(4) }}<br>full rank
        </template>
      </div>
    </div>
  </div>
</template>

<style scoped>
.sing-panel { display: flex; flex-direction: column; height: 100%; gap: 0.3em; min-height: 0; }
.controls { display: flex; flex-wrap: wrap; align-items: center; gap: 0.7em; font-size: 0.68em; font-family: var(--font-mono, monospace); }
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
