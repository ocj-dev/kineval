/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Cloth Simulation in HTML5 Canvas | Rendering and Animation Loop

    Canvas rendering for both node types (particle points + constraint lines,
    or filled rigid squares), plus the requestAnimationFrame loop that drives
    simulateStep() once per rendered frame.

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

window.requestAnimFrame =
    window.requestAnimationFrame ||
    window.webkitRequestAnimationFrame ||
    window.mozRequestAnimationFrame ||
    function (callback) { window.setTimeout(callback, 1000 / 60); };

// #region draw-arrow
// Shared vector-arrow primitive (drawn with plain canvas path/line calls,
// no images) used for the wind indicator below, and mirrored in the slides
// deck's drawUtils.ts for the same visual language there.
function drawArrow(x1, y1, x2, y2, color) {
    var dx = x2 - x1, dy = y2 - y1;
    var len = Math.sqrt(dx * dx + dy * dy);
    if (len < 1) return;
    var ux = dx / len, uy = dy / len;
    var head = Math.min(10, len * 0.4);

    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();

    var leftx = x2 - head * (ux * 0.866 + uy * 0.5);
    var lefty = y2 - head * (uy * 0.866 - ux * 0.5);
    var rightx = x2 - head * (ux * 0.866 - uy * 0.5);
    var righty = y2 - head * (uy * 0.866 + ux * 0.5);
    ctx.beginPath();
    ctx.moveTo(x2, y2);
    ctx.lineTo(leftx, lefty);
    ctx.lineTo(rightx, righty);
    ctx.closePath();
    ctx.fill();
}
// #endregion draw-arrow

// #region draw-wind-indicator
// A fixed indicator (top-left corner) of the current gusting wind vector
// from physics.js's currentWind() -- drawn every frame it's non-zero, using
// the same drawArrow() primitive as every other vector in this module.
function drawWindIndicator() {
    if (!wind_enabled) return;
    var w = currentWind();
    var ox = 50, oy = 50, scale = 8;
    drawArrow(ox, oy, ox + w.x * scale, oy + w.y * scale, '#1a9e6b');
}
// #endregion draw-wind-indicator

// #region draw-particle-cloth
function drawParticleCloth() {

    ctx.strokeStyle = '#888888';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (var i = 0; i < cloth.constraints.length; i++) {
        var c = cloth.constraints[i];
        if (!c.active) continue;
        ctx.moveTo(c.p1.x, c.p1.y);
        ctx.lineTo(c.p2.x, c.p2.y);
    }
    ctx.stroke();

    for (i = 0; i < cloth.nodes.length; i++) {
        var p = cloth.nodes[i];
        ctx.fillStyle = p.color;
        ctx.beginPath();
        ctx.arc(p.x, p.y, 3.5, 0, 2 * Math.PI);
        ctx.fill();
    }
}
// #endregion draw-particle-cloth

// #region draw-rigid-cloth
function drawRigidCloth() {

    ctx.lineWidth = 1;
    ctx.strokeStyle = '#333333';

    for (var i = 0; i < cloth.nodes.length; i++) {
        var body = cloth.nodes[i];
        var corners = [body.worldCorner(0), body.worldCorner(1), body.worldCorner(2), body.worldCorner(3)];

        ctx.fillStyle = body.color;
        ctx.beginPath();
        ctx.moveTo(corners[0].x, corners[0].y);
        for (var k = 1; k < 4; k++) ctx.lineTo(corners[k].x, corners[k].y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
    }
}
// #endregion draw-rigid-cloth

// #region draw-collision-areas
// Every collision boundary -- the wall margin on all 4 canvas edges, and any
// ground_planes[] entry -- is drawn as a filled medium-gray rectangle
// covering the solid region beyond it, rather than a thin line at the
// boundary itself, so the collidable "walls" and "ground" read as solid.
var COLLISION_AREA_COLOR = '#9a9a9a';

function drawCollisionAreas() {
    ctx.fillStyle = COLLISION_AREA_COLOR;
    ctx.fillRect(0, 0, canvas_width, WALL_MARGIN);                                   // top
    ctx.fillRect(0, canvas_height - WALL_MARGIN, canvas_width, WALL_MARGIN);         // bottom
    ctx.fillRect(0, 0, WALL_MARGIN, canvas_height);                                  // left
    ctx.fillRect(canvas_width - WALL_MARGIN, 0, WALL_MARGIN, canvas_height);         // right

    for (var g = 0; g < ground_planes.length; g++)
        ctx.fillRect(0, ground_planes[g], canvas_width, canvas_height - ground_planes[g]);
}
// #endregion draw-collision-areas

function draw() {
    ctx.clearRect(0, 0, canvas_width, canvas_height);
    drawCollisionAreas();
    if (node_type === 'rigid') drawRigidCloth();
    else drawParticleCloth();
    drawWindIndicator();
}

// #region appendix-animate
// Called once per rendered frame. A fixed unit timestep (dt = 1) is used
// throughout this module -- gravity/wind/stiffness/etc. are all tuned in
// those units -- so simulateStep() takes no wall-clock delta.
function animate() {
    simulateStep(1);
    draw();
    window.requestAnimFrame(animate);
}
// #endregion appendix-animate
