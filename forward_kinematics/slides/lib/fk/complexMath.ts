// 2D rotation via complex-number multiplication -- the deck's conceptual
// stepping-stone from axis-angle to quaternions (a unit complex number
// a+bi = [cos(theta), sin(theta)] composes 2D rotations by multiplication
// exactly the way a unit quaternion composes 3D rotations by conjugation;
// see ComplexRotation2DPanel.vue and QuaternionRotation3DPanel.vue).

export type Complex = { re: number; im: number }

export function complexFromAngle(theta: number): Complex {
  return { re: Math.cos(theta), im: Math.sin(theta) }
}

export function complexMultiply(z1: Complex, z2: Complex): Complex {
  return { re: z1.re * z2.re - z1.im * z2.im, im: z1.re * z2.im + z1.im * z2.re }
}

// Rotates 2D point p by angle theta: treat p as a complex number and
// multiply by the unit complex number for theta.
export function rotatePoint2D(p: [number, number], theta: number): [number, number] {
  const z = complexMultiply({ re: p[0], im: p[1] }, complexFromAngle(theta))
  return [z.re, z.im]
}
