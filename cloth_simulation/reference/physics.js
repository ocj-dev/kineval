/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Cloth Simulation in HTML5 Canvas | Physics Core

    COMPLETED REFERENCE IMPLEMENTATION for the AutoRob (autorob.org) lab-session
    slides. Implements maximal-coordinate, constraint-based physical simulation
    as described by Thomas Jakobsen, "Advanced Character Physics"
    (graphics.cs.cmu.edu/nsp/course/15-869/2006/papers/jakobsen.htm): force
    accumulation, Verlet integration, and satisfying multiple concurrent
    constraints by iterated relaxation. The particle/constraint/tear
    parameterization follows Adam Brooks' Tearable Cloth
    (codepen.io/dissimulate/pen/nYQrNP, github.com/dissimulate/Tearable-Cloth),
    extended here with a stiffness knob, an alternative rigid-square node type
    integrated via the Newton-Euler equations of motion, and wall/ground-plane
    collision resolved as just another constraint.

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

//////////////////////////////////////////////////
/////     PARTICLE NODE (node_type = "particle")
//////////////////////////////////////////////////

function Particle(x, y) {
    this.x = x;
    this.y = y;
    this.px = x;              // previous position (Verlet)
    this.py = y;
    this.force_x = 0;
    this.force_y = 0;
    this.pinned = false;
    this.constraints = [];    // Constraint objects touching this particle
}

// #region wind-time-varying
// A "windy flag" should feel like weather, not a fan: the configured `wind`
// URL parameter is treated as a base magnitude/direction that gusts and
// shifts over time, rather than a constant force. `sim_time` is a frame
// counter advanced once per simulateStep() call.
var sim_time = 0;

function currentWind() {
    if (!wind_enabled) return { x: 0, y: 0 };

    var base_mag = Math.sqrt(wind[0] * wind[0] + wind[1] * wind[1]);
    var base_angle = Math.atan2(wind[1], wind[0]);

    var gust_mag = base_mag * (0.6 + 0.4 * Math.sin(sim_time * 0.025));
    var gust_angle = base_angle + 0.35 * Math.sin(sim_time * 0.013);

    return { x: gust_mag * Math.cos(gust_angle), y: gust_mag * Math.sin(gust_angle) };
}
// #endregion wind-time-varying

// #region accumulate-forces
// Newton's Second Law: force accumulates on a node each timestep. Gravity is
// a fixed downward force; wind is a uniform (but, per currentWind() above,
// gusting) vector added while it's enabled; the mouse "influence" drag also
// enters here, as a spring-like pull toward the cursor.
function accumulateForces(node) {

    node.force_x = 0;
    node.force_y = 0;

    if (gravity_enabled)
        node.force_y += gravity * node.mass;

    if (wind_enabled) {
        var w = currentWind();
        node.force_x += w.x;
        node.force_y += w.y;
    }
}
// #endregion accumulate-forces

// #region verlet-integrate-particle
// Verlet integration recovers velocity implicitly from the difference between
// the current and previous position, so no explicit velocity state is kept.
// Multiplying that implicit velocity by `friction` (slightly less than 1)
// damps the system over time; acceleration (force / mass) is added scaled by
// dt^2, exactly as in Jakobsen's derivation.
function verletIntegrateParticle(p, dt) {

    if (p.pinned) return;

    var vx = (p.x - p.px) * friction;
    var vy = (p.y - p.py) * friction;

    p.px = p.x;
    p.py = p.y;

    p.x += vx + (p.force_x / p.mass) * dt * dt;
    p.y += vy + (p.force_y / p.mass) * dt * dt;
}
// #endregion verlet-integrate-particle

function Constraint(p1, p2, rest_length) {
    this.p1 = p1;
    this.p2 = p2;
    this.rest_length = (rest_length !== undefined) ? rest_length : distance(p1, p2);
    this.active = true;
    p1.constraints.push(this);
    p2.constraints.push(this);
}

function distance(a, b) {
    var dx = a.x - b.x, dy = a.y - b.y;
    return Math.sqrt(dx * dx + dy * dy);
}

// #region satisfy-constraint-particle
// One relaxation pass for a single distance constraint: measure how far the
// two particles are from the rest length, then move each halfway to close
// that gap (scaled by `stiffness`, a soft-constraint knob the original
// Tearable Cloth doesn't have). Iterating this over every constraint,
// `accuracy` times per frame, is Jakobsen's "solve multiple constraints by
// relaxation" -- no linear system is ever assembled or inverted.
function satisfyConstraintParticle(c) {

    if (!c.active) return;

    var dx = c.p2.x - c.p1.x;
    var dy = c.p2.y - c.p1.y;
    var dist = Math.sqrt(dx * dx + dy * dy);
    if (dist === 0) return;

    if (tear_enabled && dist > tear_dist) {
        c.active = false;
        return;
    }

    var diff = (dist - c.rest_length) / dist;
    var correction_x = dx * 0.5 * diff * stiffness;
    var correction_y = dy * 0.5 * diff * stiffness;

    if (!c.p1.pinned) { c.p1.x += correction_x; c.p1.y += correction_y; }
    if (!c.p2.pinned) { c.p2.x -= correction_x; c.p2.y -= correction_y; }
}
// #endregion satisfy-constraint-particle

//////////////////////////////////////////////////
/////     RIGID SQUARE NODE (node_type = "rigid")
//////////////////////////////////////////////////

function RigidSquare(x, y, half_size) {
    this.x = x;
    this.y = y;
    this.px = x;
    this.py = y;
    this.theta = 0;
    this.ptheta = 0;
    this.half_size = half_size;
    this.mass = 1;
    // moment of inertia of a uniform square plate about its center: m*side^2/6
    this.I = this.mass * (2 * half_size) * (2 * half_size) / 6;
    this.force_x = 0;
    this.force_y = 0;
    this.torque = 0;
    this.pinned = false;
    this.corner_constraints = [];
}

// local-frame corner offsets, in order: 0=top-left,1=top-right,2=bottom-right,3=bottom-left
RigidSquare.prototype.localCorner = function (i) {
    var h = this.half_size;
    switch (i) {
        case 0: return { x: -h, y: -h };
        case 1: return { x: h, y: -h };
        case 2: return { x: h, y: h };
        default: return { x: -h, y: h };
    }
};

// world-space position of corner i, given current center position and orientation
RigidSquare.prototype.worldCorner = function (i) {
    var c = this.localCorner(i);
    var cos_t = Math.cos(this.theta), sin_t = Math.sin(this.theta);
    return {
        x: this.x + c.x * cos_t - c.y * sin_t,
        y: this.y + c.x * sin_t + c.y * cos_t
    };
};

// #region accumulate-torque
// Torque accumulates from any force applied off-center; for the forces this
// module uses (gravity, wind, mouse drag) acting uniformly across the square,
// there is no net torque contribution here, but the accumulator is kept
// separate from force so per-corner effects (mouse drag on a corner,
// collision response) can add to it the same way accumulateForces() adds to
// force_x/force_y.
function accumulateTorque(body, world_point_x, world_point_y, force_x, force_y) {
    var rx = world_point_x - body.x;
    var ry = world_point_y - body.y;
    body.torque += rx * force_y - ry * force_x;   // r x F, 2D scalar cross product
}
// #endregion accumulate-torque

// #region verlet-integrate-rigid
// Newton-Euler equations of motion, integrated the same Verlet way as a
// particle: position from force/mass, orientation from torque/I. In 3D,
// Euler's rotation equation in vector form is I(domega/dt) + omega x (I
// omega) = tau; in 2D the gyroscopic term omega x (I omega) vanishes
// (omega and the inertia axis are both perpendicular to the plane), leaving
// the scalar special case used here: I (d omega/dt) = tau.
function verletIntegrateRigid(body, dt) {

    if (body.pinned) return;

    var vx = (body.x - body.px) * friction;
    var vy = (body.y - body.py) * friction;
    body.px = body.x;
    body.py = body.y;
    body.x += vx + (body.force_x / body.mass) * dt * dt;
    body.y += vy + (body.force_y / body.mass) * dt * dt;

    var omega = (body.theta - body.ptheta) * friction;
    body.ptheta = body.theta;
    body.theta += omega + (body.torque / body.I) * dt * dt;
}
// #endregion verlet-integrate-rigid

function CornerConstraint(bodyA, cornerA, bodyB, cornerB, rest_length) {
    this.bodyA = bodyA;
    this.cornerA = cornerA;
    this.bodyB = bodyB;
    this.cornerB = cornerB;
    // rest_length defaults to 0 (coincidence) -- the shared-edge grid case.
    // A single corner-to-corner constraint with a nonzero rest_length instead
    // behaves like a rigid-body "rope": it holds the two corners a fixed
    // distance apart but, unlike two coincidence constraints along a shared
    // edge, doesn't by itself prevent either body from rotating about it.
    this.rest_length = (rest_length !== undefined) ? rest_length : 0;
    this.active = true;
    bodyA.corner_constraints.push(this);
    bodyB.corner_constraints.push(this);
}

// #region apply-corner-correction
// Split a desired positional correction Δ=(dx,dy) at a corner (world-space
// offset r from the body's center) into a translation and a rotation, using
// the standard generalized-mass point-constraint solve from position-based
// rigid body dynamics: with p = perp(r), the 2x2 system
//   [ 1/m + p.x^2/I,   p.x*p.y/I    ] [impulse]   [dx]
//   [ p.x*p.y/I,       1/m + p.y^2/I] [       ] = [dy]
// gives the (fictitious) impulse whose translation (impulse/m) plus rotation
// (p . impulse / I) reproduce Δ at the corner exactly. This is what keeps the
// split well-scaled no matter how large a single correction is -- a naive
// "rotation proportional to r x Δ / I" without this normalization can produce
// wildly oversized single-step rotations and diverge.
function applyCornerCorrection(body, corner_index, dx, dy) {

    if (body.pinned) return;
    if (dx === 0 && dy === 0) return;

    var local = body.localCorner(corner_index);
    var cos_t = Math.cos(body.theta), sin_t = Math.sin(body.theta);
    var rx = local.x * cos_t - local.y * sin_t;
    var ry = local.x * sin_t + local.y * cos_t;
    var px = -ry, py = rx;   // p = perp(r)

    var inv_m = 1 / body.mass;
    var inv_I = 1 / body.I;

    var Kxx = inv_m + inv_I * px * px;
    var Kxy = inv_I * px * py;
    var Kyy = inv_m + inv_I * py * py;
    var det = Kxx * Kyy - Kxy * Kxy;

    var impulse_x = (Kyy * dx - Kxy * dy) / det;
    var impulse_y = (Kxx * dy - Kxy * dx) / det;

    body.x += inv_m * impulse_x;
    body.y += inv_m * impulse_y;
    body.theta += inv_I * (px * impulse_x + py * impulse_y);
}
// #endregion apply-corner-correction

// #region satisfy-constraint-rigid
// The rigid-body analog of satisfyConstraintParticle(): two corners (one on
// each of two squares) are relaxed toward being cc.rest_length apart, using
// exactly the same (dist - rest_length)/dist relaxation math as the particle
// constraint. For the grid's shared-edge corner pairs, rest_length is 0
// (coincidence); two of those per shared edge is what keeps neighboring
// squares from hinging freely about a single point. A single constraint with
// a nonzero rest_length, by contrast, still lets both squares rotate freely
// about it, like a rope between two corners.
function satisfyConstraintRigid(cc) {

    if (!cc.active) return;

    var wa = cc.bodyA.worldCorner(cc.cornerA);
    var wb = cc.bodyB.worldCorner(cc.cornerB);
    var dist = distance(wa, wb);

    if (tear_enabled && dist > tear_dist) {
        cc.active = false;
        return;
    }
    if (dist === 0) return;

    var diff = (dist - cc.rest_length) / dist;
    var dx = (wb.x - wa.x) * 0.5 * diff * stiffness;
    var dy = (wb.y - wa.y) * 0.5 * diff * stiffness;

    applyCornerCorrection(cc.bodyA, cc.cornerA, dx, dy);
    applyCornerCorrection(cc.bodyB, cc.cornerB, -dx, -dy);
}
// #endregion satisfy-constraint-rigid

//////////////////////////////////////////////////
/////     COLLISION: CANVAS WALLS + GROUND PLANE
//////////////////////////////////////////////////

// #region satisfy-collisions-particle
// Collision is treated as one more constraint, satisfied every relaxation
// pass alongside the distance constraints -- this is what keeps a fast-moving
// node from tunneling through a wall between one accuracy iteration and the
// next. On penetration the position is clamped to the boundary and the
// previous position is reflected across it and scaled by `bounce`, the same
// restitution trick Tearable Cloth uses for its canvas-edge bounce, just
// applied to any axis-aligned boundary (a canvas wall or a ground_planes[]
// entry) instead of only the canvas edges.
function resolveBoundaryAxis(pos, prev, boundary, outward_sign) {
    // outward_sign > 0 means the free region is pos > boundary (e.g. a floor);
    // outward_sign < 0 means the free region is pos < boundary (e.g. a right wall)
    if (outward_sign > 0 ? pos < boundary : pos > boundary) {
        return { pos: boundary, prev: boundary + (boundary - prev) * bounce };
    }
    return { pos: pos, prev: prev };
}

function satisfyCollisionsParticle(p) {

    if (p.pinned) return;

    var rx = resolveBoundaryAxis(p.x, p.px, 0, +1);
    p.x = rx.pos; p.px = rx.prev;
    rx = resolveBoundaryAxis(p.x, p.px, canvas_width, -1);
    p.x = rx.pos; p.px = rx.prev;

    var ry = resolveBoundaryAxis(p.y, p.py, 0, +1);
    p.y = ry.pos; p.py = ry.prev;
    ry = resolveBoundaryAxis(p.y, p.py, canvas_height, -1);
    p.y = ry.pos; p.py = ry.prev;

    for (var g = 0; g < ground_planes.length; g++) {
        ry = resolveBoundaryAxis(p.y, p.py, ground_planes[g], -1);
        p.y = ry.pos; p.py = ry.prev;
    }
}
// #endregion satisfy-collisions-particle

// #region satisfy-collisions-rigid
// Same wall/ground boundaries, but each of a rigid square's 4 corners is
// checked individually; a penetrating corner's needed correction is folded
// back onto the body's center position and orientation with the same
// applyCornerCorrection() split used for corner-to-corner constraints, using
// the boundary itself as the "other body". The body's center previous-
// position is reflected (scaled by `bounce`) whenever any corner collides, as
// a simplified stand-in for tracking each corner's own previous position.
function satisfyCollisionsRigid(body) {

    if (body.pinned) return;

    var boundaries = [
        { axis: 'x', value: 0, sign: +1 },
        { axis: 'x', value: canvas_width, sign: -1 },
        { axis: 'y', value: canvas_height, sign: -1 }
    ];
    for (var g = 0; g < ground_planes.length; g++)
        boundaries.push({ axis: 'y', value: ground_planes[g], sign: -1 });

    var collided = false;

    for (var i = 0; i < 4; i++) {
        var corner = body.worldCorner(i);
        for (var b = 0; b < boundaries.length; b++) {
            var bound = boundaries[b];
            var pos = (bound.axis === 'x') ? corner.x : corner.y;
            var penetrating = bound.sign > 0 ? pos < bound.value : pos > bound.value;
            if (!penetrating) continue;

            collided = true;
            var dx = (bound.axis === 'x') ? (bound.value - corner.x) : 0;
            var dy = (bound.axis === 'y') ? (bound.value - corner.y) : 0;
            applyCornerCorrection(body, i, dx, dy);
            corner = body.worldCorner(i);
        }
    }

    if (collided) {
        body.px = body.x + (body.x - body.px) * bounce;
        body.py = body.y + (body.y - body.py) * bounce;
    }
}
// #endregion satisfy-collisions-rigid

//////////////////////////////////////////////////
/////     SIMULATION STEP ORCHESTRATION
//////////////////////////////////////////////////

// #region simulate-step
// The full per-frame loop, following Jakobsen's structure exactly: accumulate
// forces, Verlet-integrate every node once, then relax every constraint --
// distance/corner constraints and collisions alike -- `accuracy` times so the
// whole concurrent system of constraints converges toward mutual
// satisfaction.
function simulateStep(dt) {

    var i, node;
    sim_time++;

    for (i = 0; i < cloth.nodes.length; i++) {
        node = cloth.nodes[i];
        accumulateForces(node);
        applyMouseInteraction(node);
        if (node_type === 'rigid') verletIntegrateRigid(node, dt);
        else verletIntegrateParticle(node, dt);
    }

    for (var pass = 0; pass < accuracy; pass++) {

        if (node_type === 'rigid') {
            for (i = 0; i < cloth.corner_constraints.length; i++)
                satisfyConstraintRigid(cloth.corner_constraints[i]);
            for (i = 0; i < cloth.nodes.length; i++)
                satisfyCollisionsRigid(cloth.nodes[i]);
        } else {
            for (i = 0; i < cloth.constraints.length; i++)
                satisfyConstraintParticle(cloth.constraints[i]);
            for (i = 0; i < cloth.nodes.length; i++)
                satisfyCollisionsParticle(cloth.nodes[i]);
        }
    }
}
// #endregion simulate-step
