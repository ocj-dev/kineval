// Matches the pseudocode-line comments in ../../reference/graph_search.js
// and the `line` field yielded by aStarSteps.ts, line for line.
export const astarPseudocode: string[] = [
  'initialize the open queue with the start node (distance 0)',
  'while the open queue is not empty',
  '    pop the node with minimum priority from the open queue',
  '    if that node was already visited, discard it and continue',
  '    mark the node visited',
  '    if within one grid cell of the goal, reconstruct path and succeed',
  '    for each of its 4 grid neighbors',
  '        if the neighbor is off-grid or in collision, skip it',
  '        tentative_distance = current.distance + eps',
  '        if tentative_distance < neighbor.distance',
  '            record this cheaper path: neighbor.distance, neighbor.parent',
  '            neighbor.priority = f(neighbor)   // g + h for A-star',
  '            insert the neighbor into the open queue',
  'the open queue emptied out -- no path exists; fail',
]
