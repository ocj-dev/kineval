// TypeScript port of ../../../reference/dynamics.js, driving the small
// panel-local simulations in this deck's interactive components. Kept
// structurally identical to the shipped reference file by convention (same
// function names, same order, same derivation) -- NOT imported from it,
// exactly the same "parallel TS port" pattern the cloth_simulation and
// pathfinding decks both use for their own panels. The code-walkthrough
// slides embed the real reference/dynamics.js directly via Slidev's `<<<`
// snippet import; this file only drives what the panels visualize.

export interface PIDGains {
  kp: number[]
  kd: number[]
  ki: number[]
}

export interface IntegrateResult {
  angle: number[]
  angle_dot: number[]
  angle_previous?: number[]
}

export type AccelFn = (angle: number[], angle_dot: number[]) => number[]

export function pendulumAcceleration(
  angle: number[], angle_dot: number[], control: number[], gravity: number, mass: number[], length: number[],
): number[] {
  return [control[0] / (mass[0] * length[0] * length[0]) - (gravity / length[0]) * Math.sin(angle[0])]
}

export function doublePendulumAcceleration(
  angle: number[], angle_dot: number[], control: number[], gravity: number, mass: number[], length: number[],
): number[] {
  const m1 = mass[0], m2 = mass[1], l1 = length[0], l2 = length[1]
  const t1 = angle[0], t2 = angle[1], w1 = angle_dot[0], w2 = angle_dot[1]
  const dtheta = t1 - t2
  const cos_d = Math.cos(dtheta), sin_d = Math.sin(dtheta)

  const M11 = (m1 + m2) * l1 * l1
  const M12 = m2 * l1 * l2 * cos_d
  const M22 = m2 * l2 * l2

  const C1 = m2 * l1 * l2 * w2 * w2 * sin_d
  const C2 = -m2 * l1 * l2 * w1 * w1 * sin_d

  const G1 = -(m1 + m2) * gravity * l1 * Math.sin(t1)
  const G2 = -m2 * gravity * l2 * Math.sin(t2)

  const rhs1 = control[0] + G1 - C1
  const rhs2 = control[1] + G2 - C2
  const det = M11 * M22 - M12 * M12

  return [(M22 * rhs1 - M12 * rhs2) / det, (M11 * rhs2 - M12 * rhs1) / det]
}

export function integrateEuler(angle: number[], angle_dot: number[], angle_dot_dot: number[], dt: number): IntegrateResult {
  const next_angle: number[] = [], next_angle_dot: number[] = []
  for (let i = 0; i < angle.length; i++) {
    next_angle[i] = angle[i] + angle_dot[i] * dt
    next_angle_dot[i] = angle_dot[i] + angle_dot_dot[i] * dt
  }
  return { angle: next_angle, angle_dot: next_angle_dot }
}

export function initVerletIntegrator(angle: number[], angle_dot: number[], angle_dot_dot: number[], dt: number): number[] {
  const angle_previous: number[] = []
  for (let i = 0; i < angle.length; i++) {
    angle_previous[i] = angle[i] - angle_dot[i] * dt + 0.5 * angle_dot_dot[i] * dt * dt
  }
  return angle_previous
}

export function integrateVerlet(
  angle: number[], angle_previous: number[], angle_dot: number[], angle_dot_dot: number[], accelFn: AccelFn, dt: number,
): IntegrateResult {
  const next_angle: number[] = []
  for (let i = 0; i < angle.length; i++) {
    next_angle[i] = 2 * angle[i] - angle_previous[i] + angle_dot_dot[i] * dt * dt
  }
  const next_angle_dot_dot = accelFn(next_angle, angle_dot)
  const next_angle_dot: number[] = []
  for (let i = 0; i < angle.length; i++) {
    next_angle_dot[i] = angle_dot[i] + 0.5 * (angle_dot_dot[i] + next_angle_dot_dot[i]) * dt
  }
  return { angle: next_angle, angle_dot: next_angle_dot, angle_previous: angle.slice() }
}

export function integrateVelocityVerlet(
  angle: number[], angle_dot: number[], angle_dot_dot: number[], accelFn: AccelFn, dt: number,
): IntegrateResult {
  const next_angle: number[] = []
  for (let i = 0; i < angle.length; i++) {
    next_angle[i] = angle[i] + angle_dot[i] * dt + 0.5 * angle_dot_dot[i] * dt * dt
  }
  const next_angle_dot_dot = accelFn(next_angle, angle_dot)
  const next_angle_dot: number[] = []
  for (let i = 0; i < angle.length; i++) {
    next_angle_dot[i] = angle_dot[i] + 0.5 * (angle_dot_dot[i] + next_angle_dot_dot[i]) * dt
  }
  return { angle: next_angle, angle_dot: next_angle_dot }
}

export function integrateRK4(angle: number[], angle_dot: number[], accelFn: AccelFn, dt: number): IntegrateResult {
  const n = angle.length

  const k_v1 = accelFn(angle, angle_dot)
  const k_x1 = angle_dot

  const a2: number[] = [], v2: number[] = []
  for (let i = 0; i < n; i++) { a2[i] = angle[i] + 0.5 * dt * k_x1[i]; v2[i] = angle_dot[i] + 0.5 * dt * k_v1[i] }
  const k_v2 = accelFn(a2, v2)
  const k_x2 = v2

  const a3: number[] = [], v3: number[] = []
  for (let i = 0; i < n; i++) { a3[i] = angle[i] + 0.5 * dt * k_x2[i]; v3[i] = angle_dot[i] + 0.5 * dt * k_v2[i] }
  const k_v3 = accelFn(a3, v3)
  const k_x3 = v3

  const a4: number[] = [], v4: number[] = []
  for (let i = 0; i < n; i++) { a4[i] = angle[i] + dt * k_x3[i]; v4[i] = angle_dot[i] + dt * k_v3[i] }
  const k_v4 = accelFn(a4, v4)
  const k_x4 = v4

  const next_angle: number[] = [], next_angle_dot: number[] = []
  for (let i = 0; i < n; i++) {
    next_angle[i] = angle[i] + (dt / 6) * (k_x1[i] + 2 * k_x2[i] + 2 * k_x3[i] + k_x4[i])
    next_angle_dot[i] = angle_dot[i] + (dt / 6) * (k_v1[i] + 2 * k_v2[i] + 2 * k_v3[i] + k_v4[i])
  }
  return { angle: next_angle, angle_dot: next_angle_dot }
}

export function PID(
  angle: number[], desired: number[], previous_error: number[], accumulated_error: number[], dt: number, servo: PIDGains,
): { control: number[], previous_error: number[], accumulated_error: number[] } {
  const control: number[] = [], next_previous_error: number[] = [], next_accumulated_error: number[] = []
  for (let i = 0; i < angle.length; i++) {
    const error = desired[i] - angle[i]
    const derivative = (error - previous_error[i]) / dt
    next_accumulated_error[i] = accumulated_error[i] + error * dt
    next_previous_error[i] = error
    control[i] = servo.kp[i] * error + servo.kd[i] * derivative + servo.ki[i] * next_accumulated_error[i]
  }
  return { control, previous_error: next_previous_error, accumulated_error: next_accumulated_error }
}

export function setPIDParameters(links: number): PIDGains {
  return links === 2
    ? { kp: [150, 60], kd: [60, 20], ki: [4, 2] }
    : { kp: [150], kd: [60], ki: [4] }
}
