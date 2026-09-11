// Matches the pseudocode-line comments in ../../reference/graph_search.js
// and the `line` field yielded by aStarSteps.ts, line for line.
//
// This is the *animation-only* wording (used by PseudocodePanel inside
// AStarPanel): line 11's phrasing is simplified and line 12's inline
// comment is dropped to fit the panel's small footprint. The static
// "Algorithmic process" slide in slides.md keeps the original, fuller
// wording rather than importing this file.
export const astarPseudocode: string[] = [
  'initialize the open queue with the start node (distance 0)',
  'while the open queue is not empty',
  '    pop the node with minimum priority from the open queue',
  '    if that node was already visited, discard it and continue',
  '    mark the node visited',
  '    if within one grid cell of the goal, reconstruct path, stop, and succeed',
  '    for each of its 4 grid neighbors',
  '        if the neighbor is off-grid or in collision, skip it',
  '        tentative_distance = current.distance + eps',
  '        if tentative_distance < neighbor.distance',
  '            update routing through visited cell',
  '            neighbor.priority = f(neighbor)',
  '            insert the neighbor into the open queue',
  'the open queue emptied out -- stop; no path exists; fail',
]
