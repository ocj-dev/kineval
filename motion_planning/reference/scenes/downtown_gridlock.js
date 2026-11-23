// New test scene for the AutoRob KinEval A-star lab deck.
// A 3x3 grid of city-block obstacles forces a Manhattan-style zig-zag route,
// good for illustrating 4-connected expansion and tie-breaking between
// equal-cost paths.
obstacles = [
    [ [0.3,0.9],[0.3,0.9] ], [ [0.3,0.9],[1.7,2.3] ], [ [0.3,0.9],[3.1,3.7] ],
    [ [1.7,2.3],[0.3,0.9] ], [ [1.7,2.3],[1.7,2.3] ], [ [1.7,2.3],[3.1,3.7] ],
    [ [3.1,3.7],[0.3,0.9] ], [ [3.1,3.7],[1.7,2.3] ], [ [3.1,3.7],[3.1,3.7] ]
];
