// Shared 2D canvas drawing for the planning panels, following
// motion_planning/reference/draw.js's own world-to-view mapping so the
// panels and the live reference canvas show the same picture.

import type { Config, Obstacle } from './planner'

export interface View {
  ctx: CanvasRenderingContext2D
  w: number
  h: number
  /** world units per device pixel, uniform in x and y */
  scale: number
  ox: number
  oy: number
}

// The reference canvas maps world [-2, 7) to 0..800 and flips y
// (draw.js#xformWorldViewY). Reproduced here, but fitted to whatever the
// panel's own element size turns out to be.
export const WORLD_MIN = -2
export const WORLD_MAX = 6.2

export function makeView(ctx: CanvasRenderingContext2D, w: number, h: number): View {
  const span = WORLD_MAX - WORLD_MIN
  const scale = Math.min(w, h) / span
  return {
    ctx, w, h, scale,
    ox: (w - span * scale) / 2,
    oy: (h - span * scale) / 2,
  }
}

export const vx = (v: View, x: number) => v.ox + (x - WORLD_MIN) * v.scale
export const vy = (v: View, y: number) => v.oy + (WORLD_MAX - y) * v.scale

export function clear(v: View, colour = '#ffffff') {
  v.ctx.fillStyle = colour
  v.ctx.fillRect(0, 0, v.w, v.h)
}

export function drawObstacles(v: View, ranges: Obstacle[], colour = '#5f6368') {
  v.ctx.fillStyle = colour
  for (const r of ranges) {
    const x0 = vx(v, r[0][0]), x1 = vx(v, r[0][1])
    const y0 = vy(v, r[1][1]), y1 = vy(v, r[1][0])
    v.ctx.fillRect(x0, y0, x1 - x0, y1 - y0)
  }
}

export function drawEdges(v: View, edges: Config[][], colour: string, width = 1) {
  v.ctx.strokeStyle = colour
  v.ctx.lineWidth = width
  v.ctx.beginPath()
  for (const e of edges) {
    v.ctx.moveTo(vx(v, e[0][0]), vy(v, e[0][1]))
    v.ctx.lineTo(vx(v, e[1][0]), vy(v, e[1][1]))
  }
  v.ctx.stroke()
}

export function drawPath(v: View, path: Config[], colour: string, width = 3) {
  if (!path || path.length < 2) return
  v.ctx.strokeStyle = colour
  v.ctx.lineWidth = width
  v.ctx.lineJoin = 'round'
  v.ctx.beginPath()
  v.ctx.moveTo(vx(v, path[0][0]), vy(v, path[0][1]))
  for (let i = 1; i < path.length; i++) v.ctx.lineTo(vx(v, path[i][0]), vy(v, path[i][1]))
  v.ctx.stroke()
}

export function drawDots(v: View, pts: Config[], colour: string, r = 1.2) {
  v.ctx.fillStyle = colour
  for (const p of pts) {
    v.ctx.beginPath()
    v.ctx.arc(vx(v, p[0]), vy(v, p[1]), r, 0, 2 * Math.PI)
    v.ctx.fill()
  }
}

export function drawMarker(v: View, q: Config, colour: string, r = 5, ring = false) {
  v.ctx.beginPath()
  v.ctx.arc(vx(v, q[0]), vy(v, q[1]), r, 0, 2 * Math.PI)
  if (ring) {
    v.ctx.strokeStyle = colour
    v.ctx.lineWidth = 2
    v.ctx.stroke()
  } else {
    v.ctx.fillStyle = colour
    v.ctx.fill()
  }
}

export function drawLabel(v: View, q: Config, text: string, colour: string) {
  v.ctx.fillStyle = colour
  v.ctx.font = '600 11px Roboto Mono, monospace'
  v.ctx.fillText(text, vx(v, q[0]) + 8, vy(v, q[1]) - 7)
}

/** sets up a hidpi backing store; returns null when the element has no layout yet */
export function prepareCanvas(canvas: HTMLCanvasElement): View | null {
  const rect = canvas.getBoundingClientRect()
  // Never commit a 0x0 backing store: onMounted fires before layout, and
  // an uninitialised buffer stretched across the element renders as a
  // solid block on some machines and blank on others.
  if (rect.width < 2 || rect.height < 2) return null
  const dpr = window.devicePixelRatio || 1
  canvas.width = Math.round(rect.width * dpr)
  canvas.height = Math.round(rect.height * dpr)
  const ctx = canvas.getContext('2d')
  if (!ctx) return null
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  return makeView(ctx, rect.width, rect.height)
}
