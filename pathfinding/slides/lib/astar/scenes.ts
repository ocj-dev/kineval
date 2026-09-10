import type { Scene } from './types'

// World boundary walls, mirroring setPlanningScene() in
// ../../reference/infrastructure.js -- every scene includes these.
const WORLD_BOUNDARY: [[number, number], [number, number]][] = [
  [[-1.8, 5.8], [-1.8, -1]],
  [[-1.8, 5.8], [5, 5.8]],
  [[-1.8, -1], [-1.8, 5.8]],
  [[5, 5.8], [-1.8, 5.8]],
]

function withBoundary(obstacles: [[number, number], [number, number]][]) {
  return [...WORLD_BOUNDARY, ...obstacles]
}

// The five scenes built into the kineval-stencil (named cases hardcoded in
// setPlanningScene()), reproduced here as data so the visualization can
// offer the same test cases as the reference implementation.
export const scenes: Scene[] = [
  {
    id: 'empty',
    name: 'Empty',
    description: 'No interior obstacles -- a baseline for the unobstructed Manhattan-distance path length.',
    obstacles: withBoundary([]),
    qInit: [0, 0],
    qGoal: [4, 4],
    isNew: false,
  },
  {
    id: 'misc',
    name: 'Misc',
    description: 'A scattering of small obstacles with a narrow opening, the stencil\'s default scene.',
    obstacles: withBoundary([
      [[1, 2], [1, 2]],
      [[3, 3.3], [1, 4]],
      [[0.6, 0.7], [0.4, 0.7]],
      [[3.7, 3.9], [-0.8, 5]],
    ]),
    qInit: [0, 0],
    qGoal: [4, 4],
    isNew: false,
  },
  {
    id: 'narrow1',
    name: 'Narrow 1',
    description: 'A single narrow corridor between two large blocks.',
    obstacles: withBoundary([
      [[1, 3], [4, 5]],
      [[1, 3], [-1, 2]],
      [[1, 1.95], [2, 3.8]],
    ]),
    qInit: [0, 0],
    qGoal: [4, 4],
    isNew: false,
  },
  {
    id: 'narrow2',
    name: 'Narrow 2',
    description: 'A staggered pair of narrow corridors, forcing a longer route than Narrow 1.',
    obstacles: withBoundary([
      [[1, 3], [4, 5]],
      [[1, 3], [-1, 2]],
      [[1, 1.9], [2, 3.8]],
      [[2.1, 3], [2.2, 4]],
    ]),
    qInit: [0, 0],
    qGoal: [4, 4],
    isNew: false,
  },
  {
    id: 'three_sections',
    name: 'Three Sections',
    description: 'Three compartments connected by narrow passages on alternating sides.',
    obstacles: withBoundary([
      [[1, 1.3], [4, 5]],
      [[1, 1.3], [-1, 3.5]],
      [[2.7, 3], [-1, 0]],
      [[2.7, 3], [0.5, 5]],
    ]),
    qInit: [0, 0],
    qGoal: [4, 4],
    isNew: false,
  },

  // Five new scenes authored for this deck (also shipped as
  // ../../reference/scenes/*.js for use with the standalone search_canvas.html).
  {
    id: 'downtown_gridlock',
    name: 'Downtown Gridlock',
    description: 'A 3x3 grid of city-block obstacles forces Manhattan-style zig-zagging, though enough monotonic routes remain that the optimal path length ties the open-world baseline.',
    obstacles: withBoundary([
      [[0.3, 0.9], [0.3, 0.9]], [[0.3, 0.9], [1.7, 2.3]], [[0.3, 0.9], [3.1, 3.7]],
      [[1.7, 2.3], [0.3, 0.9]], [[1.7, 2.3], [1.7, 2.3]], [[1.7, 2.3], [3.1, 3.7]],
      [[3.1, 3.7], [0.3, 0.9]], [[3.1, 3.7], [1.7, 2.3]], [[3.1, 3.7], [3.1, 3.7]],
    ]),
    qInit: [0, 0],
    qGoal: [4, 4],
    isNew: true,
  },
  {
    id: 'diagonal_staircase',
    name: 'Diagonal Staircase',
    description: 'Three full-width bands with gaps on alternating sides force a genuine detour longer than any monotonic staircase route.',
    obstacles: withBoundary([
      [[-1.8, 3.0], [0.9, 1.2]],
      [[1.0, 5.8], [2.1, 2.4]],
      [[-1.8, 3.0], [3.3, 3.6]],
    ]),
    qInit: [0, 0],
    qGoal: [4, 4],
    isNew: true,
  },
  {
    id: 'spiral',
    name: 'Spiral',
    description: 'Two nested rings, gapped on opposite sides, form a spiral corridor from the outside in to a goal at the center.',
    obstacles: withBoundary([
      [[-0.8, 4.8], [-0.8, -0.5]],
      [[-0.8, 4.8], [4.5, 4.8]],
      [[4.5, 4.8], [-0.8, 4.8]],
      [[-0.8, -0.5], [-0.8, 1.5]],
      [[-0.8, -0.5], [2.5, 4.8]],
      [[0.2, 3.8], [0.2, 0.5]],
      [[0.2, 3.8], [3.5, 3.8]],
      [[0.2, 0.5], [0.2, 3.8]],
      [[3.5, 3.8], [0.2, 1.5]],
      [[3.5, 3.8], [2.5, 3.8]],
    ]),
    qInit: [-0.65, 2.0],
    qGoal: [2, 2],
    isNew: true,
  },
  {
    id: 'cul_de_sac',
    name: 'Cul-de-Sac',
    description: 'A dead-end pocket reaches in close to the goal, tempting greedy-best-first into a longer, suboptimal route -- A-star is not fooled.',
    obstacles: withBoundary([
      [[2.2, 4.1], [3.7, 4.0]],
      [[3.7, 4.0], [2.2, 4.0]],
    ]),
    qInit: [0, 0],
    qGoal: [4, 4],
    isNew: true,
  },
  {
    id: 'construction_detour',
    name: 'Construction Detour',
    description: 'A single long "road closure" wall with its only gap past the goal forces a real detour beyond the goal before doubling back.',
    obstacles: withBoundary([
      [[-1.8, 4.6], [2.0, 2.3]],
      [[5.1, 5.8], [2.0, 2.3]],
    ]),
    qInit: [0, 0],
    qGoal: [4, 4],
    isNew: true,
  },
]

export function sceneById(id: string): Scene {
  const scene = scenes.find((s) => s.id === id)
  if (!scene) throw new Error(`unknown scene: ${id}`)
  return scene
}
