// Canvas draw helpers shared across this deck's interactive panels: a 2D
// side-view of the pendulum (pivot, rod(s), mass(es)) standing in for the
// reference implementation's full 3D scene, vector arrows for
// gravity/control torque, and a simple multi-series time-plot for the
// integrator-comparison and PID-convergence panels.

// Michigan maize -- the pendulum bob's color across every 2D panel in this deck.
export const MAIZE = '#FFCB05'

export interface LinkSpec {
  angleAbs: number   // absolute angle from the downward vertical
  length: number      // drawn length in canvas pixels
  color: string
}

export function clearCanvas(ctx: CanvasRenderingContext2D, width: number, height: number, bg = '#fbfbfa') {
  ctx.clearRect(0, 0, width, height)
  ctx.fillStyle = bg
  ctx.fillRect(0, 0, width, height)
}

export function drawArrow(ctx: CanvasRenderingContext2D, x: number, y: number, dx: number, dy: number, color: string, headSize = 8) {
  const len = Math.hypot(dx, dy)
  if (len < 1e-6) return
  const ex = x + dx, ey = y + dy
  ctx.strokeStyle = color
  ctx.fillStyle = color
  ctx.lineWidth = 2.5
  ctx.beginPath()
  ctx.moveTo(x, y)
  ctx.lineTo(ex, ey)
  ctx.stroke()

  const angle = Math.atan2(dy, dx)
  ctx.beginPath()
  ctx.moveTo(ex, ey)
  ctx.lineTo(ex - headSize * Math.cos(angle - Math.PI / 6), ey - headSize * Math.sin(angle - Math.PI / 6))
  ctx.lineTo(ex - headSize * Math.cos(angle + Math.PI / 6), ey - headSize * Math.sin(angle + Math.PI / 6))
  ctx.closePath()
  ctx.fill()
}

// Draws a chain of pendulum links from a fixed pivot, each angle absolute
// from the downward vertical (matching doublePendulumAcceleration()'s own
// convention) -- NOT relative to the previous link, unlike the three.js
// scene's nested-group hierarchy, since a flat 2D canvas draw can place each
// link's endpoint directly without composing transforms.
export function drawPendulumChain(
  ctx: CanvasRenderingContext2D, pivotX: number, pivotY: number, links: LinkSpec[], massRadius = 11,
) {
  ctx.fillStyle = '#9aa0a6'
  ctx.beginPath()
  ctx.arc(pivotX, pivotY, 5, 0, Math.PI * 2)
  ctx.fill()

  let x = pivotX, y = pivotY
  for (const link of links) {
    const ex = x + link.length * Math.sin(link.angleAbs)
    const ey = y + link.length * Math.cos(link.angleAbs)

    ctx.strokeStyle = '#3a3a3a'
    ctx.lineWidth = 4
    ctx.beginPath()
    ctx.moveTo(x, y)
    ctx.lineTo(ex, ey)
    ctx.stroke()

    ctx.fillStyle = link.color
    ctx.beginPath()
    ctx.arc(ex, ey, massRadius, 0, Math.PI * 2)
    ctx.fill()
    ctx.strokeStyle = '#00000033'
    ctx.lineWidth = 1
    ctx.stroke()

    x = ex; y = ey
  }
}

// Dashed reference line marking the downward vertical (theta = 0), so a
// panel's swing is visually anchored against the stable equilibrium.
export function drawVerticalReference(ctx: CanvasRenderingContext2D, pivotX: number, pivotY: number, length: number) {
  ctx.save()
  ctx.setLineDash([4, 5])
  ctx.strokeStyle = '#00000030'
  ctx.lineWidth = 1.5
  ctx.beginPath()
  ctx.moveTo(pivotX, pivotY)
  ctx.lineTo(pivotX, pivotY + length)
  ctx.stroke()
  ctx.restore()
}

export interface TimeSeries {
  label: string
  color: string
  points: number[]
}

// A small multi-series line plot (no axes library) used by the integrator-
// comparison and PID-convergence panels to show a value (energy, angle,
// error) over the simulated trace so far. `points` are drawn against a
// shared, auto-scaled y-range across all series.
export function drawTimeSeries(
  ctx: CanvasRenderingContext2D, width: number, height: number, series: TimeSeries[],
  options: { yLabel?: string, zeroLine?: boolean } = {},
) {
  clearCanvas(ctx, width, height)

  const padding = { left: 44, right: 12, top: 12, bottom: 22 }
  const plotW = width - padding.left - padding.right
  const plotH = height - padding.top - padding.bottom

  let minY = Infinity, maxY = -Infinity, maxN = 0
  for (const s of series) {
    for (const v of s.points) { if (v < minY) minY = v; if (v > maxY) maxY = v }
    maxN = Math.max(maxN, s.points.length)
  }
  if (!isFinite(minY) || !isFinite(maxY)) { minY = 0; maxY = 1 }
  if (options.zeroLine) { minY = Math.min(minY, 0); maxY = Math.max(maxY, 0) }
  if (maxY - minY < 1e-6) { maxY += 0.5; minY -= 0.5 }
  const pad = (maxY - minY) * 0.08
  minY -= pad; maxY += pad

  const xOf = (i: number) => padding.left + (maxN <= 1 ? 0 : (i / (maxN - 1)) * plotW)
  const yOf = (v: number) => padding.top + plotH - ((v - minY) / (maxY - minY)) * plotH

  ctx.strokeStyle = '#c7cad1'
  ctx.lineWidth = 1
  ctx.strokeRect(padding.left, padding.top, plotW, plotH)

  if (options.zeroLine) {
    ctx.strokeStyle = '#00000030'
    ctx.beginPath()
    ctx.moveTo(padding.left, yOf(0))
    ctx.lineTo(padding.left + plotW, yOf(0))
    ctx.stroke()
  }

  ctx.font = '10px "Roboto Mono", monospace'
  ctx.fillStyle = '#5f6368'
  ctx.textAlign = 'right'
  ctx.fillText(maxY.toFixed(1), padding.left - 6, padding.top + 8)
  ctx.fillText(minY.toFixed(1), padding.left - 6, padding.top + plotH)

  for (const s of series) {
    if (s.points.length < 2) continue
    ctx.strokeStyle = s.color
    ctx.lineWidth = 2
    ctx.beginPath()
    ctx.moveTo(xOf(0), yOf(s.points[0]))
    for (let i = 1; i < s.points.length; i++) ctx.lineTo(xOf(i), yOf(s.points[i]))
    ctx.stroke()
  }

  var legendX = padding.left + 8
  ctx.textAlign = 'left'
  for (const s of series) {
    ctx.fillStyle = s.color
    ctx.fillRect(legendX, padding.top + 2, 8, 8)
    ctx.fillStyle = '#202124'
    ctx.fillText(s.label, legendX + 12, padding.top + 10)
    legendX += 14 + ctx.measureText(s.label).width + 14
  }

  if (options.yLabel) {
    ctx.save()
    ctx.translate(12, padding.top + plotH / 2)
    ctx.rotate(-Math.PI / 2)
    ctx.textAlign = 'center'
    ctx.fillStyle = '#5f6368'
    ctx.fillText(options.yLabel, 0, 0)
    ctx.restore()
  }
}
