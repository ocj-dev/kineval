// Canvas draw helpers for this deck's 2D schematic DFS-traversal view --
// standing in for the reference implementation's full 3D three.js scene,
// same relationship simulation/slides/lib/pendulum/drawUtils.ts has to its
// own 3D pendulum rig. A link's pose is reduced to the single point its
// matrix's translation column gives, projected with a fixed oblique
// (2.5D) projection -- enough to show the kinematic TREE assembling itself
// link by link as the DFS proceeds, which is the lesson, without needing a
// real camera/perspective pipeline for what's a schematic, not a replica.
import type { Mat4 } from './types'

export const INDIGO = '#202B4E'
export const AMBER = '#E8710A'
export const HIGHLIGHT = '#ffd34d'

export function clearCanvas(ctx: CanvasRenderingContext2D, width: number, height: number, bg = '#fbfbfa') {
  ctx.clearRect(0, 0, width, height)
  ctx.fillStyle = bg
  ctx.fillRect(0, 0, width, height)
}

// Fixed oblique projection: x stays horizontal, world y (up) becomes
// screen-up, and world z leans the point diagonally -- a cheap stand-in
// for an isometric camera, not a real perspective/orthographic matrix.
export function project(x: number, y: number, z: number, scale: number): [number, number] {
  return [(x - z * 0.5) * scale, -(y + z * 0.3) * scale]
}

export function positionOf(m: Mat4): [number, number, number] {
  return [m[0][3], m[1][3], m[2][3]]
}

export interface FkEdge { parent: string; child: string }

// Draws every edge whose two endpoints are both already revealed, then
// every revealed node on top -- so a partially-built tree always shows
// complete parent-child segments, never a dangling line toward an
// unrevealed node.
export function drawFkTree(
  ctx: CanvasRenderingContext2D, width: number, height: number,
  revealed: Map<string, Mat4>, edges: FkEdge[], highlightName: string | null,
  scale = 220,
) {
  clearCanvas(ctx, width, height)
  const originPx: [number, number] = [width * 0.3, height * 0.55]

  const screenOf = (name: string): [number, number] | null => {
    const m = revealed.get(name)
    if (!m) return null
    const [x, y, z] = positionOf(m)
    const [px, py] = project(x, y, z, scale)
    return [originPx[0] + px, originPx[1] + py]
  }

  ctx.strokeStyle = '#9aa0a6'
  ctx.lineWidth = 3
  for (const edge of edges) {
    const p1 = screenOf(edge.parent)
    const p2 = screenOf(edge.child)
    if (!p1 || !p2) continue
    ctx.beginPath()
    ctx.moveTo(p1[0], p1[1])
    ctx.lineTo(p2[0], p2[1])
    ctx.stroke()
  }

  for (const name of revealed.keys()) {
    const p = screenOf(name)
    if (!p) continue
    const isCurrent = name === highlightName
    ctx.beginPath()
    ctx.arc(p[0], p[1], isCurrent ? 10 : 7, 0, Math.PI * 2)
    ctx.fillStyle = isCurrent ? HIGHLIGHT : INDIGO
    ctx.fill()
    ctx.strokeStyle = '#00000033'
    ctx.lineWidth = 1
    ctx.stroke()

    ctx.font = '11px "Roboto Mono", monospace'
    ctx.fillStyle = '#202124'
    ctx.textAlign = 'center'
    ctx.fillText(name, p[0], p[1] - 14)
  }

  // world-origin tick, so the base link's own offset from the world is legible
  const [ox, oy] = [originPx[0], originPx[1]]
  ctx.strokeStyle = '#00000030'
  ctx.setLineDash([3, 4])
  ctx.beginPath()
  ctx.moveTo(ox - 14, oy)
  ctx.lineTo(ox + 14, oy)
  ctx.moveTo(ox, oy - 14)
  ctx.lineTo(ox, oy + 14)
  ctx.stroke()
  ctx.setLineDash([])
}
