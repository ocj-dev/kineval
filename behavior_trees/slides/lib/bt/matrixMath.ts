// TS port of ../../../reference/matrix.js, for the in-slide demos and the
// jacobianSteps/solve tracers. The deck's "Main components" slides embed
// matrix.js's actual code via Slidev snippet-import, so what's read there is
// always the real shipped reference code -- this port exists only so the
// interactive panels have the same math available client-side, kept
// structurally identical function-for-function so the two never disagree
// (same convention forward_kinematics/slides/lib/fk/matrixMath.ts
// established for this project).
import type { Mat, Mat4 } from './types'

export function matrixMultiply(m1: Mat, m2: Mat): Mat {
  const result: Mat = []
  for (let i = 0; i < m1.length; i++) {
    result[i] = []
    for (let j = 0; j < m2[0].length; j++) {
      let sum = 0
      for (let k = 0; k < m2.length; k++) sum += m1[i][k] * m2[k][j]
      result[i][j] = sum
    }
  }
  return result
}

export function matrixTranspose(m: Mat): Mat {
  const result: Mat = []
  for (let j = 0; j < m[0].length; j++) {
    result[j] = []
    for (let i = 0; i < m.length; i++) result[j][i] = m[i][j]
  }
  return result
}

// Gauss-Jordan elimination with partial pivoting -- the same small building
// block matrix_pseudoinverse (matrix.js) needs internally; see that file's
// own matrix-square-invert region comment for why this exists rather than
// an external linear-algebra dependency.
export function matrixSquareInvert(m: Mat): Mat {
  const n = m.length
  const aug: Mat = []
  for (let i = 0; i < n; i++) {
    aug[i] = m[i].slice()
    for (let j = 0; j < n; j++) aug[i].push(i === j ? 1 : 0)
  }
  for (let k = 0; k < n; k++) {
    let pivotRow = k
    let maxAbs = Math.abs(aug[k][k])
    for (let i = k + 1; i < n; i++) {
      if (Math.abs(aug[i][k]) > maxAbs) { maxAbs = Math.abs(aug[i][k]); pivotRow = i }
    }
    if (pivotRow !== k) { const tmp = aug[k]; aug[k] = aug[pivotRow]; aug[pivotRow] = tmp }
    const factor = aug[k][k]
    for (let j = 0; j < 2 * n; j++) aug[k][j] /= factor
    for (let i = 0; i < n; i++) {
      if (i === k) continue
      const f = aug[i][k]
      for (let j = 0; j < 2 * n; j++) aug[i][j] -= f * aug[k][j]
    }
  }
  const inv: Mat = []
  for (let i = 0; i < n; i++) inv[i] = aug[i].slice(n, 2 * n)
  return inv
}

// Moore-Penrose pseudoinverse via the normal equations -- TALL m (rows >=
// cols) uses the left form inv(m^T m) m^T; WIDE m (more columns than rows,
// the usual case for a redundant arm's end-effector Jacobian) uses the
// right form m^T inv(m m^T), since m^T m is singular there. See
// matrix.js#matrix-pseudoinverse's own comment -- this is the exact branch
// the deck's pseudoinverse slide walks through.
export function matrixPseudoinverse(m: Mat): Mat {
  const rows = m.length, cols = m[0].length
  const mT = matrixTranspose(m)
  if (rows >= cols) {
    return matrixMultiply(matrixSquareInvert(matrixMultiply(mT, m)), mT)
  } else {
    return matrixMultiply(mT, matrixSquareInvert(matrixMultiply(m, mT)))
  }
}

export function vectorNormalize(v: number[]): number[] {
  const len = Math.sqrt(v.reduce((s, x) => s + x * x, 0))
  return len === 0 ? v.slice() : v.map((x) => x / len)
}

export function vectorSubtract(a: number[], b: number[]): number[] {
  return a.map((x, i) => x - b[i])
}

export function vectorCross(a: number[], b: number[]): number[] {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

export function generateIdentity(): Mat4 {
  return [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
}

export function generateTranslationMatrix(tx: number, ty: number, tz: number): Mat4 {
  return [[1, 0, 0, tx], [0, 1, 0, ty], [0, 0, 1, tz], [0, 0, 0, 1]]
}

export function generateRotationMatrixX(angle: number): Mat4 {
  const c = Math.cos(angle), s = Math.sin(angle)
  return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]
}

export function generateRotationMatrixY(angle: number): Mat4 {
  const c = Math.cos(angle), s = Math.sin(angle)
  return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]
}

export function generateRotationMatrixZ(angle: number): Mat4 {
  const c = Math.cos(angle), s = Math.sin(angle)
  return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
}

// AutoRob's fixed-axis rpy order, matching kinematics.js's matrix_from_rpy: R = Rz(yaw) Ry(pitch) Rx(roll)
export function matrixFromRpy(rpy: [number, number, number]): Mat4 {
  return matrixMultiply(generateRotationMatrixZ(rpy[2]), matrixMultiply(generateRotationMatrixY(rpy[1]), generateRotationMatrixX(rpy[0]))) as Mat4
}

export function matrixFromOrigin(origin: { xyz: [number, number, number]; rpy: [number, number, number] }): Mat4 {
  return matrixMultiply(generateTranslationMatrix(...origin.xyz), matrixFromRpy(origin.rpy)) as Mat4
}
