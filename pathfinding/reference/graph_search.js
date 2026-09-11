/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | Graph Search Methods

    COMPLETED REFERENCE IMPLEMENTATION for the AutoRob (autorob.org) lab-session
    slides. This file completes every "STENCIL" section of the upstream
    kineval-stencil (github.com/autorob/kineval-stencil) project_pathplan
    module. It supports four graph-search algorithms selected by the global
    `search_alg` string ("A-star", "greedy-best-first", "breadth-first",
    "depth-first"), all sharing the same priority-queue-driven search loop --
    they differ only in how a node's search priority is computed.

    The "Component breakdown" slides of the accompanying Slidev deck
    (../slides/slides.md) embed pieces of this exact file directly (via
    Slidev's `<<<` snippet-import feature), so what's shown in the
    walkthrough is always this shipped code, never a copy.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

function initSearchGraph() {

    // create the search queue (a binary min-heap keyed on each node's
    //   'priority' field -- see the MIN HEAP section below)
    visit_queue = [];

    // a monotonically increasing counter, used only to give depth-first
    //   search a well-defined "most recently queued" ordering (see
    //   computeNodePriority() below)
    search_expansion_counter = 0;

    // initialize search graph as 2D array over configuration space
    //   of 2D locations with specified spatial resolution
    G = [];

    // track the single closest grid node to q_init while building the grid,
    //   since q_init generally will not land exactly on a grid point
    var closest_start_node = null;
    var closest_start_dist = Infinity;

    for (iind=0,xpos=-2;xpos<7;iind++,xpos+=eps) {
        G[iind] = [];
        for (jind=0,ypos=-2;ypos<7;jind++,ypos+=eps) {
            // #region grid-node-fields
            G[iind][jind] = {
                i:iind,j:jind, // mapping to graph array
                x:xpos,y:ypos, // mapping to map coordinates
                parent:null, // pointer to parent in graph along motion path
                distance:10000, // distance to start via path through parent
                visited:false, // flag for whether the node has been visited
                priority:null, // visit priority based on fscore
                queued:false // flag for whether the node has been queued for visiting
            };
            // #endregion grid-node-fields

            // STENCIL: determine whether this graph node should be the start
            //   point for the search
            var d = Math.sqrt(Math.pow(xpos-q_init[0],2)+Math.pow(ypos-q_init[1],2));
            if (d < closest_start_dist) {
                closest_start_dist = d;
                closest_start_node = G[iind][jind];
            }
        }
    }

    // #region init-start
    // pseudocode line 1: the start node has zero distance from itself and
    //   is the first node queued for visiting
    closest_start_node.distance = 0;
    closest_start_node.priority = 0;
    closest_start_node.queued = true;
    minheap_insert(visit_queue, closest_start_node);
    node_start = closest_start_node;
    // #endregion init-start
}

function iterateGraphSearch() {

    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
    //   draw_2D_configuration - draws a square at a given location

    // #region main-loop
    // pseudocode line 2: search fails once the open queue is exhausted
    //   without having reached the goal. Stop the animate() loop from
    //   calling this function again (see infrastructure.js/draw.js) --
    //   without this, the search would keep iterating (and finding
    //   nothing, since the queue stays empty) forever.
    if (visit_queue.length === 0) {
        search_iterate = false;
        return "failed";
    }

    // pseudocode line 3: pop the node with minimum priority from the open
    //   queue (lazy deletion: a node may be enqueued more than once if its
    //   distance is later improved, so entries already visited are stale
    //   and simply discarded here rather than being removed from the heap
    //   up front)
    var current_node = minheap_extract(visit_queue);

    // pseudocode line 4: discard stale duplicate queue entries
    if (current_node.visited)
        return "iterating";

    // pseudocode line 5: mark this node visited -- its shortest distance
    //   from the start is now final
    current_node.visited = true;
    search_visited++;
    draw_2D_configuration([current_node.x, current_node.y], "visited");

    // pseudocode line 6: goal test -- succeed once we reach a node within
    //   one grid cell of the goal configuration
    var dist_to_goal = Math.sqrt(Math.pow(current_node.x-q_goal[0],2)+Math.pow(current_node.y-q_goal[1],2));
    if (dist_to_goal <= eps) {
        drawHighlightedPathGraph(current_node);
        // stop iterating -- the path is found, so further calls to this
        // function (from the animate() loop) would otherwise keep
        // expanding nodes well past the point the search is done.
        search_iterate = false;
        return "succeeded";
    }
    // #endregion main-loop

    // #region neighbor-loop
    // pseudocode line 7: expand the 4-connected neighbors of this node
    //   (grid indices are adjacent by construction, since eps is both the
    //   node spacing in world coordinates and the index step)
    var neighbor_offsets = [[1,0],[-1,0],[0,1],[0,-1]];
    for (var k=0; k<neighbor_offsets.length; k++) {
        var ni = current_node.i + neighbor_offsets[k][0];
        var nj = current_node.j + neighbor_offsets[k][1];

        // pseudocode line 8: skip neighbors that fall outside the graph
        if ((ni<0) || (ni>=G.length) || (nj<0) || (nj>=G[ni].length))
            continue;

        var neighbor = G[ni][nj];

        // pseudocode line 8: skip neighbors in collision with the world
        if (testCollision([neighbor.x, neighbor.y]))
            continue;

        // pseudocode line 9: cost of stepping from current_node to this
        //   neighbor is exactly one grid cell (eps), since the grid is
        //   uniformly spaced
        var tentative_distance = current_node.distance + eps;

        // pseudocode line 10-11: relax the edge if this path to the
        //   neighbor is shorter than any found so far
        if (tentative_distance < neighbor.distance) {
            neighbor.distance = tentative_distance;
            neighbor.parent = current_node;
            neighbor.priority = computeNodePriority(neighbor);

            // pseudocode line 13: queue the neighbor for visiting. if it is
            //   already queued, this pushes a second, cheaper-priority copy
            //   onto the heap; the stale first copy is discarded by the
            //   visited-check above when it is eventually popped.
            search_expansion_counter++;
            if (!neighbor.queued) {
                neighbor.queued = true;
                draw_2D_configuration([neighbor.x, neighbor.y], "queued");
            }
            minheap_insert(visit_queue, neighbor);
        }
    }
    // #endregion neighbor-loop

    return "iterating";
}

// #region priority-formula
// pseudocode line 12: a node's search priority is f = g + h for A-star
//   (g = distance-from-start so far, h = an admissible heuristic estimate of
//   the remaining distance to the goal); greedy-best-first uses only h;
//   breadth-first (uniform edge cost) reduces to Dijkstra's algorithm, i.e.
//   A-star with h = 0; depth-first has no natural priority-queue analogue,
//   so it is emulated by always preferring the most recently queued node
//   (a decreasing counter behaves like a LIFO stack under a min-heap)
function computeNodePriority(node) {
    var heuristic = Math.sqrt(Math.pow(node.x-q_goal[0],2)+Math.pow(node.y-q_goal[1],2));
    switch (search_alg) {
        case "greedy-best-first":
            return heuristic;
        case "breadth-first":
            return node.distance;
        case "depth-first":
            return -search_expansion_counter;
        case "A-star":
        default:
            return node.distance + heuristic;
    }
}
// #endregion priority-formula

//////////////////////////////////////////////////
/////     MIN HEAP IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement min heap functions for graph search priority queue.
    //   These functions work use the 'priority' field for elements in graph.

// #region insert-queue
// standard array-backed binary min-heap: insert appends the new element
//   then "sifts up" by repeatedly swapping with its parent while it is
//   smaller, restoring the heap invariant in O(log n)
function minheap_insert(heap, new_element) {
    heap.push(new_element);
    var idx = heap.length - 1;
    while (idx > 0) {
        var parent_idx = Math.floor((idx-1)/2);
        if (heap[parent_idx].priority <= heap[idx].priority)
            break;
        var tmp = heap[parent_idx];
        heap[parent_idx] = heap[idx];
        heap[idx] = tmp;
        idx = parent_idx;
    }
}
// #endregion insert-queue

// #region pop-min
// extract-min swaps the root with the last element, pops the (old) root off
//   the end, then "sifts down" the new root by repeatedly swapping with its
//   smaller child until the heap invariant is restored, also O(log n)
function minheap_extract(heap) {
    var min_element = heap[0];
    var last_element = heap.pop();

    if (heap.length > 0) {
        heap[0] = last_element;
        var idx = 0;
        while (true) {
            var left = 2*idx+1;
            var right = 2*idx+2;
            var smallest = idx;
            if ((left < heap.length) && (heap[left].priority < heap[smallest].priority))
                smallest = left;
            if ((right < heap.length) && (heap[right].priority < heap[smallest].priority))
                smallest = right;
            if (smallest === idx)
                break;
            var tmp = heap[idx];
            heap[idx] = heap[smallest];
            heap[smallest] = tmp;
            idx = smallest;
        }
    }

    return min_element;
}
// #endregion pop-min

// assign heap operations within a minheaper object, matching the
//   tutorial_heapsort convention used elsewhere in the KinEval stencil
minheaper = {};
minheaper.insert = minheap_insert;
minheaper.extract = minheap_extract;
