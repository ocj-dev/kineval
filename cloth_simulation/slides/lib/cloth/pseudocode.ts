// The master simulation-loop pseudocode, exactly as shown on the "Simulation
// loop" overview slide -- reused here so every multi-phase interactive panel
// highlights its active line against the very same listing.
export const MASTER_PSEUDOCODE = [
  'build the cloth: a grid of nodes (particles or rigid squares), and',
  'the constraints between adjacent nodes',
  'every frame:',
  '    for each node',
  '        accumulate forces (gravity, wind, mouse) -- and torque, for rigid nodes',
  '        Verlet-integrate the node\'s position -- and orientation, for rigid nodes',
  '    repeat `accuracy` times:',
  '        for each constraint, satisfy it by relaxation',
  '        for each node, satisfy collisions (walls, and any ground plane)',
  '    draw every node and the constraints between them',
]

export const LINE_ACCUMULATE = 4
export const LINE_INTEGRATE = 5
export const LINE_RELAX = 7
export const LINE_COLLIDE = 8
