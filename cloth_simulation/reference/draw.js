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

function drawGroundPlanes() {
    if (!ground_planes || ground_planes.length === 0) return;
    ctx.strokeStyle = '#00274C';
    ctx.lineWidth = 2;
    for (var g = 0; g < ground_planes.length; g++) {
        ctx.beginPath();
        ctx.moveTo(0, ground_planes[g]);
        ctx.lineTo(canvas_width, ground_planes[g]);
        ctx.stroke();
    }
}

function draw() {
    ctx.clearRect(0, 0, canvas_width, canvas_height);
    if (node_type === 'rigid') drawRigidCloth();
    else drawParticleCloth();
    drawGroundPlanes();
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
