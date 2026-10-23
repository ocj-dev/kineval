// Canvas draw helpers for this deck's 2D schematic panels (SingularityPanel
// mainly) -- same relationship forward_kinematics/slides/lib/fk/drawUtils.ts
// has to that module's own 2D DFS-traversal view. Kept intentionally small:
// most of this module's panels (JacobianBuildPanel, IkConvergePanel) use a
// real three.js viewport instead, same as MatrixStackPanel.vue did for FK.
export const INDIGO = '#202B4E'
export const AMBER = '#E8710A'
export const LOCK_RED = '#c0392b'

export function clearCanvas(ctx: CanvasRenderingContext2D, width: number, height: number, bg = '#fbfbfa') {
  ctx.clearRect(0, 0, width, height)
  ctx.fillStyle = bg
  ctx.fillRect(0, 0, width, height)
}
