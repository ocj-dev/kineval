// Small canvas drawing helpers shared by every interactive step-through
// panel: force/correction-vector arrows, and the two node renderings
// (particle dot, rigid square) mirroring reference/draw.js.

export function drawArrow(
  ctx: CanvasRenderingContext2D, x1: number, y1: number, x2: number, y2: number,
  color: string, width = 3,
) {
  const dx = x2 - x1, dy = y2 - y1
  const len = Math.sqrt(dx * dx + dy * dy)
  if (len < 1) return
  const ux = dx / len, uy = dy / len
  const head = Math.min(10, len * 0.4)

  ctx.strokeStyle = color
  ctx.fillStyle = color
  ctx.lineWidth = width
  ctx.beginPath()
  ctx.moveTo(x1, y1)
  ctx.lineTo(x2, y2)
  ctx.stroke()

  const leftx = x2 - head * (ux * 0.866 + uy * 0.5)
  const lefty = y2 - head * (uy * 0.866 - ux * 0.5)
  const rightx = x2 - head * (ux * 0.866 - uy * 0.5)
  const righty = y2 - head * (uy * 0.866 + ux * 0.5)
  ctx.beginPath()
  ctx.moveTo(x2, y2)
  ctx.lineTo(leftx, lefty)
  ctx.lineTo(rightx, righty)
  ctx.closePath()
  ctx.fill()
}

export function drawParticleDot(ctx: CanvasRenderingContext2D, x: number, y: number, color = '#3a3a3a', r = 6) {
  ctx.fillStyle = color
  ctx.beginPath()
  ctx.arc(x, y, r, 0, 2 * Math.PI)
  ctx.fill()
  ctx.strokeStyle = '#ffffff'
  ctx.lineWidth = 1.5
  ctx.stroke()
}

export function drawRigidSquare(
  ctx: CanvasRenderingContext2D, corners: { x: number; y: number }[], color = '#3a3a3a',
) {
  ctx.fillStyle = color
  ctx.strokeStyle = '#333333'
  ctx.lineWidth = 1.5
  ctx.beginPath()
  ctx.moveTo(corners[0].x, corners[0].y)
  for (let i = 1; i < corners.length; i++) ctx.lineTo(corners[i].x, corners[i].y)
  ctx.closePath()
  ctx.fill()
  ctx.stroke()
}

export const COLLISION_AREA_COLOR = '#9a9a9a'

// Draws every collision boundary -- the walls around `bounds`, and the
// ground plane at `groundY` when given -- as filled medium-gray rectangles
// covering the solid region beyond each one, mirroring reference/draw.js's
// drawCollisionAreas(). Call this before drawing the simulated nodes so they
// render on top.
export function drawCollisionAreas(
  ctx: CanvasRenderingContext2D, width: number, height: number,
  bounds: { minX: number; maxX: number; minY: number; maxY: number },
  groundY?: number,
) {
  ctx.fillStyle = COLLISION_AREA_COLOR
  ctx.fillRect(0, 0, bounds.minX, height)
  ctx.fillRect(bounds.maxX, 0, width - bounds.maxX, height)
  ctx.fillRect(bounds.minX, 0, bounds.maxX - bounds.minX, bounds.minY)
  const floor = groundY ?? bounds.maxY
  ctx.fillRect(bounds.minX, floor, bounds.maxX - bounds.minX, height - floor)
}

export const VECTOR_COLORS = {
  gravity: '#00274C',
  wind: '#1a9e6b',
  resultant: '#c1272d',
  correction: '#FFCB05',
  collision: '#7d3ac1',
}
