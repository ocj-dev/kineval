/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Cloth Simulation in HTML5 Canvas | Cloth Grid, Coloring, Mouse Interaction

    Builds the grid of cloth nodes (particles or rigid squares, per node_type),
    attaching the constraints between them exactly as described in Jakobsen's
    "Advanced Character Physics" and parameterized as in Adam Brooks' Tearable
    Cloth (see physics.js header for full citations). Also implements the
    "Michigan colors" block-M node coloring and the mouse drag/cut interaction
    carried over from Tearable Cloth.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see cloth_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

var MICHIGAN_MAIZE = '#FFCB05';
var MICHIGAN_BLUE = '#00274C';
var DEFAULT_NODE_COLOR = '#3a3a3a';

// #region michigan-mask
// A stylized block "M" over the unit square (u,v in [0,1], v=0 at the top):
// two solid vertical legs, plus two diagonal strokes running from the inside
// of each leg down to the horizontal center, generated analytically rather
// than from an image asset so the module stays fully self-contained.
function isMichiganM(u, v) {

    var leg_width = 0.22;
    var v_mid = 0.55;
    var stroke_half = 0.09;

    if (u < leg_width || u > 1 - leg_width) return true;

    if (v <= v_mid) {
        var left_center = leg_width + (0.5 - leg_width) * (v / v_mid);
        if (Math.abs(u - left_center) < stroke_half) return true;

        var right_center = (1 - leg_width) - (0.5 - leg_width) * (v / v_mid);
        if (Math.abs(u - right_center) < stroke_half) return true;
    }

    return false;
}

function nodeColor(col, row) {
    if (!michigan_colors) return DEFAULT_NODE_COLOR;
    var u = (cloth_x > 1) ? col / (cloth_x - 1) : 0;
    var v = (cloth_y > 1) ? row / (cloth_y - 1) : 0;
    return isMichiganM(u, v) ? MICHIGAN_BLUE : MICHIGAN_MAIZE;
}
// #endregion michigan-mask

// #region build-cloth
// Builds the node grid and the constraints between adjacent nodes: a particle
// grid with distance constraints (Tearable Cloth's own structure), or a
// rigid-square grid with 2 corner constraints per shared edge (one per
// corner pair along that edge) when node_type is "rigid". Either way, the
// top row is pinned in place -- for rigid squares this fixes both position
// and orientation, since verletIntegrateRigid() skips pinned bodies entirely.
function Cloth() {

    this.nodes = [];
    this.grid = [];
    this.constraints = [];
    this.corner_constraints = [];

    var start_x = canvas_width / 2 - (cloth_x - 1) * spacing / 2;
    var start_y = spacing;

    for (var row = 0; row < cloth_y; row++) {
        this.grid[row] = [];
        for (var col = 0; col < cloth_x; col++) {

            var x = start_x + col * spacing;
            var y = start_y + row * spacing;
            var node;

            if (node_type === 'rigid') {
                node = new RigidSquare(x, y, (spacing - 1) / 2);
            } else {
                node = new Particle(x, y);
                node.mass = 1;
            }
            node.color = nodeColor(col, row);
            node.pinned = (row === 0);

            this.grid[row][col] = node;
            this.nodes.push(node);

            if (node_type === 'rigid') {
                if (col > 0) {
                    var left = this.grid[row][col - 1];
                    this.corner_constraints.push(new CornerConstraint(left, 1, node, 0));
                    this.corner_constraints.push(new CornerConstraint(left, 2, node, 3));
                }
                if (row > 0) {
                    var above = this.grid[row - 1][col];
                    this.corner_constraints.push(new CornerConstraint(above, 3, node, 0));
                    this.corner_constraints.push(new CornerConstraint(above, 2, node, 1));
                }
            } else {
                if (col > 0) this.constraints.push(new Constraint(this.grid[row][col - 1], node, spacing));
                if (row > 0) this.constraints.push(new Constraint(this.grid[row - 1][col], node, spacing));
            }
        }
    }
}
// #endregion build-cloth

// #region mouse-interaction
// Carried over from Tearable Cloth: while the primary mouse button is held,
// nodes within `mouse.influence` are dragged along with the cursor (done by
// displacing the node's previous position, so Verlet integration reads it
// back as velocity); nodes within the smaller `mouse.cut` radius instead have
// their constraints deactivated, tearing the cloth by hand. Rigid-square
// nodes drag/cut the same way, just centered on the body instead of a point.
var mouse = { x: 0, y: 0, px: 0, py: 0, down: false, button: 1, influence: 28, cut: 10 };

function applyMouseInteraction(node) {

    if (!mouse.down || node.pinned) return;

    var dx = node.x - mouse.x;
    var dy = node.y - mouse.y;
    var dist = Math.sqrt(dx * dx + dy * dy);

    if (mouse.button === 1 && dist < mouse.influence) {
        node.px = node.x - (mouse.x - mouse.px);
        node.py = node.y - (mouse.y - mouse.py);
    } else if (dist < mouse.cut) {
        cutNodeConstraints(node);
    }
}

function cutNodeConstraints(node) {
    var list = (node_type === 'rigid') ? node.corner_constraints : node.constraints;
    for (var i = 0; i < list.length; i++) list[i].active = false;
}
// #endregion mouse-interaction
