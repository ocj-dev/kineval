// Parameterized TypeScript port of reference/physics.js, used to drive the
// slide deck's interactive step-through panels. The reference implementation
// itself is shown to students via the `<<<` code-import snippets in
// slides.md; this file exists only so those same formulas can run inside a
// reactive Vue component (checkboxes, joystick, Play/Step/Reset), the same
// relationship pathfinding/slides/lib/astar/*.ts has to reference/graph_search.js.

export interface Vec2 { x: number; y: number }

export interface ParticleState {
  x: number; y: number
  px: number; py: number
  force_x: number; force_y: number
  mass: number
  pinned: boolean
}

export interface RigidState {
  x: number; y: number
  px: number; py: number
  theta: number; ptheta: number
  force_x: number; force_y: number
  torque: number
  mass: number
  I: number
  half_size: number
  pinned: boolean
}

export function makeParticle(x: number, y: number, pinned = false): ParticleState {
  return { x, y, px: x, py: y, force_x: 0, force_y: 0, mass: 1, pinned }
}

export function makeRigid(x: number, y: number, half_size: number, pinned = false): RigidState {
  const mass = 1
  return {
    x, y, px: x, py: y, theta: 0, ptheta: 0,
    force_x: 0, force_y: 0, torque: 0,
    mass, I: mass * (2 * half_size) * (2 * half_size) / 6,
    half_size, pinned,
  }
}

// #region demo-accumulate-forces
export function accumulateForces(
  node: { force_x: number; force_y: number; mass: number },
  opts: { gravityOn: boolean; gravity: number; windOn: boolean; wind: Vec2 },
) {
  node.force_x = 0
  node.force_y = 0
  if (opts.gravityOn) node.force_y += opts.gravity * node.mass
  if (opts.windOn) { node.force_x += opts.wind.x; node.force_y += opts.wind.y }
}
// #endregion demo-accumulate-forces

// #region demo-verlet-particle
export function verletIntegrateParticle(p: ParticleState, friction: number) {
  if (p.pinned) return
  const vx = (p.x - p.px) * friction
  const vy = (p.y - p.py) * friction
  p.px = p.x; p.py = p.y
  p.x += vx + p.force_x / p.mass
  p.y += vy + p.force_y / p.mass
}
// #endregion demo-verlet-particle

// #region demo-accumulate-torque
export function accumulateTorque(
  body: RigidState, world_point_x: number, world_point_y: number, force_x: number, force_y: number,
) {
  const rx = world_point_x - body.x
  const ry = world_point_y - body.y
  body.torque += rx * force_y - ry * force_x   // r x F, 2D scalar cross product
}
// #endregion demo-accumulate-torque

// #region demo-verlet-rigid
export function verletIntegrateRigid(body: RigidState, friction: number) {
  if (body.pinned) return
  const vx = (body.x - body.px) * friction
  const vy = (body.y - body.py) * friction
  body.px = body.x; body.py = body.y
  body.x += vx + body.force_x / body.mass
  body.y += vy + body.force_y / body.mass

  const omega = (body.theta - body.ptheta) * friction
  body.ptheta = body.theta
  body.theta += omega + body.torque / body.I
}
// #endregion demo-verlet-rigid

export function localCorner(body: RigidState, i: number): Vec2 {
  const h = body.half_size
  switch (i) {
    case 0: return { x: -h, y: -h }
    case 1: return { x: h, y: -h }
    case 2: return { x: h, y: h }
    default: return { x: -h, y: h }
  }
}

export function worldCorner(body: RigidState, i: number): Vec2 {
  const c = localCorner(body, i)
  const cos_t = Math.cos(body.theta), sin_t = Math.sin(body.theta)
  return { x: body.x + c.x * cos_t - c.y * sin_t, y: body.y + c.x * sin_t + c.y * cos_t }
}

// #region demo-satisfy-particle
export function satisfyConstraintParticle(
  p1: ParticleState, p2: ParticleState, rest_length: number, stiffness: number,
): Vec2 {
  const dx = p2.x - p1.x, dy = p2.y - p1.y
  const dist = Math.sqrt(dx * dx + dy * dy) || 1e-9
  const diff = (dist - rest_length) / dist
  const cx = dx * 0.5 * diff * stiffness
  const cy = dy * 0.5 * diff * stiffness
  if (!p1.pinned) { p1.x += cx; p1.y += cy }
  if (!p2.pinned) { p2.x -= cx; p2.y -= cy }
  return { x: cx, y: cy }   // correction applied to p1, for the relaxation-phase vector overlay
}
// #endregion demo-satisfy-particle

// #region demo-apply-corner-correction
export function applyCornerCorrection(body: RigidState, corner_index: number, dx: number, dy: number) {
  if (body.pinned) return
  if (dx === 0 && dy === 0) return

  const local = localCorner(body, corner_index)
  const cos_t = Math.cos(body.theta), sin_t = Math.sin(body.theta)
  const rx = local.x * cos_t - local.y * sin_t
  const ry = local.x * sin_t + local.y * cos_t
  const px = -ry, py = rx

  const inv_m = 1 / body.mass
  const inv_I = 1 / body.I

  const Kxx = inv_m + inv_I * px * px
  const Kxy = inv_I * px * py
  const Kyy = inv_m + inv_I * py * py
  const det = Kxx * Kyy - Kxy * Kxy

  const impulse_x = (Kyy * dx - Kxy * dy) / det
  const impulse_y = (Kxx * dy - Kxy * dx) / det

  body.x += inv_m * impulse_x
  body.y += inv_m * impulse_y
  body.theta += inv_I * (px * impulse_x + py * impulse_y)
}
// #endregion demo-apply-corner-correction

// #region demo-satisfy-rigid
export function satisfyConstraintRigid(
  bodyA: RigidState, cornerA: number, bodyB: RigidState, cornerB: number, stiffness: number,
): Vec2 {
  const wa = worldCorner(bodyA, cornerA)
  const wb = worldCorner(bodyB, cornerB)
  const dx = (wb.x - wa.x) * 0.5 * stiffness
  const dy = (wb.y - wa.y) * 0.5 * stiffness
  applyCornerCorrection(bodyA, cornerA, dx, dy)
  applyCornerCorrection(bodyB, cornerB, -dx, -dy)
  return { x: dx, y: dy }
}
// #endregion demo-satisfy-rigid

// #region demo-collide-rigid
export function satisfyCollisionRigid(
  body: RigidState, bounds: { minX: number; maxX: number; minY: number; maxY: number; groundY?: number },
  bounce: number,
) {
  if (body.pinned) return
  const floor = bounds.groundY ?? bounds.maxY
  let collided = false

  for (let i = 0; i < 4; i++) {
    let corner = worldCorner(body, i)
    if (corner.x < bounds.minX) { applyCornerCorrection(body, i, bounds.minX - corner.x, 0); collided = true; corner = worldCorner(body, i) }
    if (corner.x > bounds.maxX) { applyCornerCorrection(body, i, bounds.maxX - corner.x, 0); collided = true; corner = worldCorner(body, i) }
    if (corner.y < bounds.minY) { applyCornerCorrection(body, i, 0, bounds.minY - corner.y); collided = true; corner = worldCorner(body, i) }
    if (corner.y > floor) { applyCornerCorrection(body, i, 0, floor - corner.y); collided = true }
  }

  if (collided) {
    body.px = body.x + (body.x - body.px) * bounce
    body.py = body.y + (body.y - body.py) * bounce
  }
}
// #endregion demo-collide-rigid

// #region demo-collide-particle
export function satisfyCollisionParticle(
  p: ParticleState, bounds: { minX: number; maxX: number; minY: number; maxY: number; groundY?: number },
  bounce: number,
): Vec2 {
  if (p.pinned) return { x: 0, y: 0 }
  const before = { x: p.x, y: p.y }

  if (p.x < bounds.minX) { p.px = bounds.minX + (bounds.minX - p.px) * bounce; p.x = bounds.minX }
  if (p.x > bounds.maxX) { p.px = bounds.maxX + (bounds.maxX - p.px) * bounce; p.x = bounds.maxX }
  if (p.y < bounds.minY) { p.py = bounds.minY + (bounds.minY - p.py) * bounce; p.y = bounds.minY }
  const floor = bounds.groundY ?? bounds.maxY
  if (p.y > floor) { p.py = floor + (floor - p.py) * bounce; p.y = floor }

  return { x: p.x - before.x, y: p.y - before.y }
}
// #endregion demo-collide-particle
