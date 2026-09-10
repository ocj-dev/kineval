// New test scene for the AutoRob KinEval A-star lab deck.
// Two nested square rings, gapped on opposite sides, form a spiral
// corridor from the outside in -- forcing many more node expansions to
// reach the goal at the center than a direct-line scene, good for
// contrasting A-star against breadth-first and greedy-best-first
// (see the search_alg URL parameter) on the same world.
obstacles = [
    // outer ring, gapped on the left between y=1.5 and y=2.5
    [ [-0.8,4.8],[-0.8,-0.5] ],
    [ [-0.8,4.8],[4.5,4.8] ],
    [ [4.5,4.8],[-0.8,4.8] ],
    [ [-0.8,-0.5],[-0.8,1.5] ],
    [ [-0.8,-0.5],[2.5,4.8] ],
    // inner ring, gapped on the right between y=1.5 and y=2.5, so the only
    // route from the outer corridor to the goal at the center threads
    // through both gaps on opposite sides of the rings
    [ [0.2,3.8],[0.2,0.5] ],
    [ [0.2,3.8],[3.5,3.8] ],
    [ [0.2,0.5],[0.2,3.8] ],
    [ [3.5,3.8],[0.2,1.5] ],
    [ [3.5,3.8],[2.5,3.8] ]
];
