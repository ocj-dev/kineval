// TS port of ../../../reference/quaternion.js -- see matrixMath.ts's header
// for why this port exists alongside the snippet-embedded original.
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

// Rotates a 3D point by conjugation, v' = q v q^-1 (q unit, so q^-1 = conjugate)
// -- used by QuaternionRotation3DPanel.vue. Not one of the four STENCIL
// functions itself (the reference implementation only ever needs
// quaternion_to_rotation_matrix, folding the rotation into the matrix
// stack instead), but the textbook form of "rotate a point by a
// quaternion" the quaternions lecture itself introduces it as (see the
// deck's quaternion-rotation slide).
export function quaternionRotatePoint(q: Quat, v: [number, number, number]): [number, number, number] {
  const p: Quat = { a: 0, b: v[0], c: v[1], d: v[2] }
  const qConj: Quat = { a: q.a, b: -q.b, c: -q.c, d: -q.d }
  const rotated = quaternionMultiply(quaternionMultiply(q, p), qConj)
  return [rotated.b, rotated.c, rotated.d]
}
