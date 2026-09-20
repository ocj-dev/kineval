// TypeScript mirror of reference/cloth.js's michigan-mask region, used by
// the deck's particle-node interactive panels (Blob, 3x3 grid) so their
// coloring follows the exact same buffer/aspect/fallback rules as the
// reference implementation. Kept as a hand-mirrored duplicate rather than a
// shared import, same as clothPhysics.ts's relationship to physics.js -- the
// reference implementation is a dependency-free plain-JS file, the slides
// are a bundled TS/Vue app; they can't literally share a module.

export const MICHIGAN_MAIZE = '#FFCB05'
export const MICHIGAN_BLUE = '#00274C'

const M_ASPECT = 1.15
const M_MIN_INNER = 5

function isMichiganM(u: number, v: number): boolean {
  const leg_width = 0.22
  const v_mid = 0.55
  const stroke_half = 0.09

  if (u < leg_width || u > 1 - leg_width) return true

  if (v <= v_mid) {
    const left_center = leg_width + (0.5 - leg_width) * (v / v_mid)
    if (Math.abs(u - left_center) < stroke_half) return true
    const right_center = (1 - leg_width) - (0.5 - leg_width) * (v / v_mid)
    if (Math.abs(u - right_center) < stroke_half) return true
  }
  return false
}

// col,row are 0-indexed node coordinates; cols,rows are the grid's node
// counts along each axis. Colors are inverted from a literal reading: the M
// glyph is maize, everything else (buffer, letterbox padding, background,
// and the too-small fallback) is blue.
export function michiganNodeColor(col: number, row: number, cols: number, rows: number): string {
  const margin = 1
  const inner_w = cols - 2 * margin
  const inner_h = rows - 2 * margin

  if (inner_w < M_MIN_INNER || inner_h < M_MIN_INNER) return MICHIGAN_BLUE
  if (col < margin || col >= cols - margin || row < margin || row >= rows - margin) return MICHIGAN_BLUE

  const u = (col - margin) / (inner_w - 1)
  const v = (row - margin) / (inner_h - 1)

  const inner_aspect = inner_w / inner_h
  let u_m = u, v_m = v
  if (inner_aspect > M_ASPECT) {
    const scale_x = M_ASPECT / inner_aspect
    u_m = 0.5 + (u - 0.5) / scale_x
  } else {
    const scale_y = inner_aspect / M_ASPECT
    v_m = 0.5 + (v - 0.5) / scale_y
  }
  if (u_m < 0 || u_m > 1 || v_m < 0 || v_m > 1) return MICHIGAN_BLUE

  return isMichiganM(u_m, v_m) ? MICHIGAN_MAIZE : MICHIGAN_BLUE
}
