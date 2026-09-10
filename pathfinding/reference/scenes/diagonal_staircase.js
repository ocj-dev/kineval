// New test scene for the AutoRob KinEval A-star lab deck.
// Three full-width bands with gaps on alternating sides force the search to
// zig-zag across the whole width of the world rather than following any of
// the many equal-length monotonic staircases available in an open grid --
// unlike an open grid, here every route is forced to be longer than the
// straight-line (Manhattan) distance between start and goal.
obstacles = [
    [ [-1.8,3.0],[0.9,1.2] ],
    [ [1.0,5.8],[2.1,2.4] ],
    [ [-1.8,3.0],[3.3,3.6] ]
];
