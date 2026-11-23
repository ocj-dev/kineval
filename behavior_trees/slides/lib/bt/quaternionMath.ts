// TS port of ../../../reference/quaternion.js, plus the two local
// orientation-error helpers ../../../reference/inverse_kinematics.js adds
// itself (quaternionConjugateLocal, rotationMatrixToQuaternion) -- see that
// file's #orientation-error-helpers region comment for why those two live
// outside quaternion.js's own 4-function STENCIL scope.
import type { Mat4, Quat } from './types'
import { vectorNormalize } from './matrixMath'

export function quaternionFromAxisAngle(axis: number[], angle: number): Quat {
  const unitAxis = vectorNormalize(axis)
  const half = angle / 2
  const s = Math.sin(half)
  return { a: Math.cos(half), b: unitAxis[0] * s, c: unitAxis[1] * s, d: unitAxis[2] * s }
}

export function quaternionNormalize(q: Quat): Quat {
  const norm = Math.sqrt(q.a * q.a + q.b * q.b + q.c * q.c + q.d * q.d)
  if (norm === 0) return { a: 1, b: 0, c: 0, d: 0 }
  return { a: q.a / norm, b: q.b / norm, c: q.c / norm, d: q.d / norm }
}

export function quaternionMultiply(q1: Quat, q2: Quat): Quat {
  return {
    a: q1.a * q2.a - q1.b * q2.b - q1.c * q2.c - q1.d * q2.d,
    b: q1.a * q2.b + q1.b * q2.a + q1.c * q2.d - q1.d * q2.c,
    c: q1.a * q2.c - q1.b * q2.d + q1.c * q2.a + q1.d * q2.b,
    d: q1.a * q2.d + q1.b * q2.c - q1.c * q2.b + q1.d * q2.a,
  }
}

export function quaternionToRotationMatrix(q: Quat): Mat4 {
  const { a, b, c, d } = q
  return [
    [1 - 2 * (c * c + d * d), 2 * (b * c - a * d), 2 * (b * d + a * c), 0],
    [2 * (b * c + a * d), 1 - 2 * (b * b + d * d), 2 * (c * d - a * b), 0],
    [2 * (b * d - a * c), 2 * (c * d + a * b), 1 - 2 * (b * b + c * c), 0],
    [0, 0, 0, 1],
  ]
}

// quaternion.js only completes the 4 functions above; the conjugate and the
// rotation-matrix->quaternion extraction iterateIK's orientation-error term
// needs are kept local there too (inverse_kinematics.js), and ported here
// identically.
export function quaternionConjugate(q: Quat): Quat {
  return { a: q.a, b: -q.b, c: -q.c, d: -q.d }
}

// Standard trace-based (Shepperd's method) extraction of a unit quaternion
// from a 3x3 rotation matrix R (indexed R[row][col]); picks whichever of the
// 4 algebraically-equivalent formulas has the largest, best-conditioned
// denominator for the given R.
export function rotationMatrixToQuaternion(R: number[][]): Quat {
  const m00 = R[0][0], m11 = R[1][1], m22 = R[2][2]
  const trace = m00 + m11 + m22
  let a: number, b: number, c: number, d: number, s: number
  if (trace > 0) {
    s = 2 * Math.sqrt(trace + 1.0)
    a = 0.25 * s
    b = (R[2][1] - R[1][2]) / s
    c = (R[0][2] - R[2][0]) / s
    d = (R[1][0] - R[0][1]) / s
  } else if (m00 > m11 && m00 > m22) {
    s = 2 * Math.sqrt(1.0 + m00 - m11 - m22)
    a = (R[2][1] - R[1][2]) / s
    b = 0.25 * s
    c = (R[0][1] + R[1][0]) / s
    d = (R[0][2] + R[2][0]) / s
  } else if (m11 > m22) {
    s = 2 * Math.sqrt(1.0 + m11 - m00 - m22)
    a = (R[0][2] - R[2][0]) / s
    b = (R[0][1] + R[1][0]) / s
    c = 0.25 * s
    d = (R[1][2] + R[2][1]) / s
  } else {
    s = 2 * Math.sqrt(1.0 + m22 - m00 - m11)
    a = (R[1][0] - R[0][1]) / s
    b = (R[0][2] + R[2][0]) / s
    c = (R[1][2] + R[2][1]) / s
    d = 0.25 * s
  }
  return quaternionNormalize({ a, b, c, d })
}
