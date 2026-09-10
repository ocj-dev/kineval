// New test scene for the AutoRob KinEval A-star lab deck.
// A dead-end pocket opens toward the start and reaches in close to the goal
// before closing off -- every cell inside has a smaller straight-line
// (heuristic) distance to the goal than the correct bypass route around the
// outside, tempting greedy-best-first to dive all the way in before it can
// discover there is no way through. A-star's g-cost term keeps it from
// wasting nearly as much search effort in the trap.
obstacles = [
    [ [2.2,4.1],[3.7,4.0] ],
    [ [3.7,4.0],[2.2,4.0] ]
];
