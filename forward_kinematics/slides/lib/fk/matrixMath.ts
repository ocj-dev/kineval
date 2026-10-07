// TS port of ../../../reference/matrix.js, for the in-slide demos and
// fkSteps.ts tracer. The "Matrix stack & transforms" slides embed
// matrix.js's actual code via Slidev snippet-import, so what's read there
// is always the real shipped reference code -- this port exists only so
// the interactive panels have the same math available client-side, kept
// structurally identical function-for-function so the two never disagree.
import type { Mat4 } from './types'

export function matrixMultiply(m1: number[][], m2: number[][]): number[][] {
  const result: number[][] = []
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

export function matrixTranspose(m: number[][]): number[][] {
  const result: number[][] = []
  for (let j = 0; j < m[0].length; j++) {
    result[j] = []
    for (let i = 0; i < m.length; i++) result[j][i] = m[i][j]
  }
  return result
}

export function vectorNormalize(v: number[]): number[] {
  const len = Math.sqrt(v.reduce((s, x) => s + x * x, 0))
  return len === 0 ? v.slice() : v.map((x) => x / len)
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
  return matrixMultiply(generateRotationMatrixZ(rpy[2]), matrixMultiply(generateRotationMatrixY(rpy[1]), generateRotationMatrixX(rpy[0])))
}

export function matrixFromOrigin(origin: { xyz: [number, number, number]; rpy: [number, number, number] }): Mat4 {
  return matrixMultiply(generateTranslationMatrix(...origin.xyz), matrixFromRpy(origin.rpy))
}

export function matrixInvertAffine(m: Mat4): Mat4 {
  const rotT = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) rotT[i][j] = m[j][i]

  const t = [m[0][3], m[1][3], m[2][3]]
  const tInv = [0, 0, 0]
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) tInv[i] -= rotT[i][j] * t[j]

  return [
    [rotT[0][0], rotT[0][1], rotT[0][2], tInv[0]],
    [rotT[1][0], rotT[1][1], rotT[1][2], tInv[1]],
    [rotT[2][0], rotT[2][1], rotT[2][2], tInv[2]],
    [0, 0, 0, 1],
  ]
}
