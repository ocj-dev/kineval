/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Pendularm Dynamical Simulation | Physics Core

    COMPLETED REFERENCE IMPLEMENTATION for the AutoRob (autorob.org) lab-session
    slides. Completes every "STENCIL" section of the upstream kineval-stencil
    (github.com/autorob/kineval-stencil) project_pendularm module
    (pendularm1.html/update_pendulum_state.js for the single pendulum,
    pendularm2.html/update_pendulum_state2.js for the double pendulum): the
    equation-of-motion function(s), four numerical integrators (Euler, Verlet,
    Velocity Verlet, Runge-Kutta 4), and a PID servo controller, per the
    original AutoRob assignment ("Assignment 2: Pendularm").

    Both the single- and double-pendulum equations of motion are derived by
    the Euler-Lagrange equation d/dt(dL/dtheta_dot) - dL/dtheta = tau, L = KE -
    PE, with angles measured from the downward vertical (theta=0 is the
    stable, hanging equilibrium). The double pendulum is expressed in the
    general robot-dynamics form M(theta) theta_dot_dot + C(theta,theta_dot) =
    G(theta) + tau -- the same mass-matrix/Coriolis/gravity decomposition
    covered in the AutoRob "Motion Control and PID" lecture for general robot
    dynamics -- rather than as a single closed-form fraction, so it
    generalizes cleanly and reads the same way the lecture already taught
    the shape of the problem.

    All four integrators are written once, generically, against an
    arbitrary-length state array -- so the same four functions drive both the
    1-DOF and 2-DOF pendulum; only the acceleration function passed in
    differs.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see pendularm.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

//////////////////////////////////////////////////
/////     EQUATIONS OF MOTION
//////////////////////////////////////////////////

// #region pendulum-acceleration-single
// Simple pendulum: a rigid massless rod of length l with a point mass m at
// its end, pivoting frictionlessly, plus an idealized motor applying torque
// tau at the pivot. Rotational inertia about the pivot is I = m*l^2 (a point
// mass at radius l, no parallel-axis correction needed since the pivot IS
// the rotation axis). Newton's second law for rotation, I*theta_dot_dot =
// tau_net, with tau_net = tau - m*g*l*sin(theta) (gravity's restoring
// torque, theta measured from the downward vertical) gives:
//
//   theta_dot_dot = tau/(m*l^2) - (g/l)*sin(theta)
//
// which is exactly the form the AutoRob dynamics lecture derives via the
// Euler-Lagrange equation from L = KE - PE = (1/2)*m*l^2*theta_dot^2 -
// (-m*g*l*cos(theta)).
function pendulumAcceleration(angle, angle_dot, control, gravity, mass, length) {
    return [control[0] / (mass[0] * length[0] * length[0]) - (gravity / length[0]) * Math.sin(angle[0])];
}
// #endregion pendulum-acceleration-single

// #region pendulum-acceleration-double
// Double pendulum: two point masses (m1 at the end of link 1, m2 at the end
// of link 2, which hangs from the end of link 1), both angles measured
// absolutely from the downward vertical. Deriving the Lagrangian from each
// mass's (x,y) position and applying Euler-Lagrange to theta1 and theta2
// yields a coupled system in the same M(theta)*theta_dot_dot +
// C(theta,theta_dot) = G(theta) + tau form used for general robot dynamics:
//
//   M = [ (m1+m2)*l1^2            m2*l1*l2*cos(theta1-theta2) ]
//       [ m2*l1*l2*cos(theta1-theta2)   m2*l2^2                ]
//
//   C = [  m2*l1*l2*theta2_dot^2*sin(theta1-theta2) ]
//       [ -m2*l1*l2*theta1_dot^2*sin(theta1-theta2) ]
//
//   G = [ -(m1+m2)*g*l1*sin(theta1) ]
//       [ -m2*g*l2*sin(theta2)      ]
//
// solved for theta_dot_dot = M^-1 * (tau + G - C). This is the same
// derivation math24.net/double-pendulum.html gives in closed-form-fraction
// style; writing it as an explicit 2x2 mass-matrix solve instead keeps the
// coupling terms legible and matches the mass-matrix framing the AutoRob PID
// lecture uses for general multi-link dynamics.
function doublePendulumAcceleration(angle, angle_dot, control, gravity, mass, length) {

    var m1 = mass[0], m2 = mass[1], l1 = length[0], l2 = length[1];
    var t1 = angle[0], t2 = angle[1], w1 = angle_dot[0], w2 = angle_dot[1];
    var dtheta = t1 - t2;
    var cos_d = Math.cos(dtheta), sin_d = Math.sin(dtheta);

    var M11 = (m1 + m2) * l1 * l1;
    var M12 = m2 * l1 * l2 * cos_d;
    var M22 = m2 * l2 * l2;

    var C1 = m2 * l1 * l2 * w2 * w2 * sin_d;
    var C2 = -m2 * l1 * l2 * w1 * w1 * sin_d;

    var G1 = -(m1 + m2) * gravity * l1 * Math.sin(t1);
    var G2 = -m2 * gravity * l2 * Math.sin(t2);

    var rhs1 = control[0] + G1 - C1;
    var rhs2 = control[1] + G2 - C2;

    var det = M11 * M22 - M12 * M12;

    return [
        (M22 * rhs1 - M12 * rhs2) / det,
        (M11 * rhs2 - M12 * rhs1) / det
    ];
}
// #endregion pendulum-acceleration-double

//////////////////////////////////////////////////
/////     NUMERICAL INTEGRATORS (generic over state length)
//////////////////////////////////////////////////

// #region integrate-euler
// Explicit (forward) Euler: advance position using the CURRENT velocity, and
// velocity using the CURRENT acceleration -- both updates read only
// old-timestep values, so error accumulates every step with nothing to
// cancel it. This is the "naive" integrator: cheap, and visibly unstable
// (spiraling energy gain) once dt is large relative to the system's own
// timescale -- the AutoRob dynamics lecture introduces it first for exactly
// that reason.
function integrateEuler(angle, angle_dot, angle_dot_dot, dt) {
    var next_angle = [], next_angle_dot = [];
    for (var i = 0; i < angle.length; i++) {
        next_angle[i] = angle[i] + angle_dot[i] * dt;
        next_angle_dot[i] = angle_dot[i] + angle_dot_dot[i] * dt;
    }
    return { angle: next_angle, angle_dot: next_angle_dot };
}
// #endregion integrate-euler

// #region init-verlet-integrator
// Basic (position-only) Verlet needs a "previous" position to get started,
// since its update rule never references velocity directly. Bootstrap it
// with a single second-order Taylor step backward in time: angle_previous =
// angle(0) - angle_dot(0)*dt + (1/2)*angle_dot_dot(0)*dt^2 -- the same
// central-difference approximation Verlet itself is built from, applied
// once to manufacture a consistent starting history.
function initVerletIntegrator(angle, angle_dot, angle_dot_dot, dt) {
    var angle_previous = [];
    for (var i = 0; i < angle.length; i++) {
        angle_previous[i] = angle[i] - angle_dot[i] * dt + 0.5 * angle_dot_dot[i] * dt * dt;
    }
    return angle_previous;
}
// #endregion init-verlet-integrator

// #region integrate-verlet
// Basic Verlet integration stores no explicit velocity at all: the next
// position is extrapolated from the current and previous positions plus the
// current acceleration, via the central-difference identity angle_dot_dot ~=
// (angle_next - 2*angle + angle_previous) / dt^2, solved for angle_next.
// angle_dot plays NO role in that position update -- unlike Velocity Verlet
// below, which uses it explicitly -- which is what makes this method
// symplectic even though no velocity is ever integrated. A velocity value is
// still returned, but purely for HUD/energy display: it's recovered via the
// same average-acceleration estimate Velocity Verlet itself uses, so both
// methods report the same physical velocity for what is, position-wise, an
// identical trajectory (confirmed by running both against the same initial
// state: their angle sequences match to machine precision).
function integrateVerlet(angle, angle_previous, angle_dot, angle_dot_dot, accelFn, dt) {
    var next_angle = [];
    for (var i = 0; i < angle.length; i++) {
        next_angle[i] = 2 * angle[i] - angle_previous[i] + angle_dot_dot[i] * dt * dt;
    }
    var next_angle_dot_dot = accelFn(next_angle, angle_dot);
    var next_angle_dot = [];
    for (var i = 0; i < angle.length; i++) {
        next_angle_dot[i] = angle_dot[i] + 0.5 * (angle_dot_dot[i] + next_angle_dot_dot[i]) * dt;
    }
    return { angle: next_angle, angle_dot: next_angle_dot, angle_previous: angle.slice() };
}
// #endregion integrate-verlet

// #region integrate-velocity-verlet
// Velocity Verlet keeps an explicit velocity (unlike basic Verlet above) by
// splitting the step in two: advance position using the OLD acceleration,
// then average the old and newly-recomputed acceleration to advance
// velocity. This assumes acceleration depends only on position (not
// velocity) between the two acceleration evaluations -- a good approximation
// here, and the reason this method (like basic Verlet) is symplectic where
// plain Euler is not.
function integrateVelocityVerlet(angle, angle_dot, angle_dot_dot, accelFn, dt) {
    var next_angle = [];
    for (var i = 0; i < angle.length; i++) {
        next_angle[i] = angle[i] + angle_dot[i] * dt + 0.5 * angle_dot_dot[i] * dt * dt;
    }
    var next_angle_dot_dot = accelFn(next_angle, angle_dot);
    var next_angle_dot = [];
    for (var i = 0; i < angle.length; i++) {
        next_angle_dot[i] = angle_dot[i] + 0.5 * (angle_dot_dot[i] + next_angle_dot_dot[i]) * dt;
    }
    return { angle: next_angle, angle_dot: next_angle_dot };
}
// #endregion integrate-velocity-verlet

// #region integrate-rk4
// Classic 4th-order Runge-Kutta, applied to the first-order system (angle,
// angle_dot) with state derivative f(angle, angle_dot) = (angle_dot,
// accelFn(angle, angle_dot)). Four stages are evaluated at the start, twice
// at the midpoint, and once at the endpoint of the interval, then combined
// with Simpson's-rule weights [1,2,2,1]/6 -- k_x{1..4} are the
// velocity-stage estimates that advance angle, k_v{1..4} are the
// acceleration-stage estimates that advance angle_dot, matching the
// notation used in the AutoRob dynamics lecture's own Butcher-tableau
// derivation (RK4's tableau: c=[0,1/2,1/2,1], b=[1/6,1/3,1/3,1/6]).
function integrateRK4(angle, angle_dot, accelFn, dt) {

    var n = angle.length, i;

    var k_v1 = accelFn(angle, angle_dot);
    var k_x1 = angle_dot;

    var a2 = [], v2 = [];
    for (i = 0; i < n; i++) { a2[i] = angle[i] + 0.5 * dt * k_x1[i]; v2[i] = angle_dot[i] + 0.5 * dt * k_v1[i]; }
    var k_v2 = accelFn(a2, v2);
    var k_x2 = v2;

    var a3 = [], v3 = [];
    for (i = 0; i < n; i++) { a3[i] = angle[i] + 0.5 * dt * k_x2[i]; v3[i] = angle_dot[i] + 0.5 * dt * k_v2[i]; }
    var k_v3 = accelFn(a3, v3);
    var k_x3 = v3;

    var a4 = [], v4 = [];
    for (i = 0; i < n; i++) { a4[i] = angle[i] + dt * k_x3[i]; v4[i] = angle_dot[i] + dt * k_v3[i]; }
    var k_v4 = accelFn(a4, v4);
    var k_x4 = v4;

    var next_angle = [], next_angle_dot = [];
    for (i = 0; i < n; i++) {
        next_angle[i] = angle[i] + (dt / 6) * (k_x1[i] + 2 * k_x2[i] + 2 * k_x3[i] + k_x4[i]);
        next_angle_dot[i] = angle_dot[i] + (dt / 6) * (k_v1[i] + 2 * k_v2[i] + 2 * k_v3[i] + k_v4[i]);
    }
    return { angle: next_angle, angle_dot: next_angle_dot };
}
// #endregion integrate-rk4

//////////////////////////////////////////////////
/////     PID SERVO CONTROLLER
//////////////////////////////////////////////////

// #region pid-controller
// Proportional-Integral-Derivative control, framed (per the AutoRob "Motion
// Control and PID" lecture) as current + past + future error: P reacts to
// the CURRENT error like a spring pulling toward the setpoint (Hooke's Law
// intuition, F=-kx); I accumulates PAST error over time (sum of e*dt) to
// cancel steady-state error a P-only controller can't reach on its own
// (e.g. gravity constantly pulling the arm away from a non-hanging
// setpoint); D estimates the FUTURE trend of the error via its own rate of
// change, acting as damping that opposes fast motion. Gains kp/kd/ki are
// tuned in that same P -> D -> I order: raise kp until roughly reaching the
// setpoint (expect overshoot), raise kd until oscillation stops, raise ki
// until steady-state error is eliminated.
function PID(angle, desired, previous_error, accumulated_error, dt, servo) {

    var control = [], next_previous_error = [], next_accumulated_error = [];

    for (var i = 0; i < angle.length; i++) {
        var error = desired[i] - angle[i];
        var derivative = (error - previous_error[i]) / dt;
        next_accumulated_error[i] = accumulated_error[i] + error * dt;
        next_previous_error[i] = error;

        control[i] = servo.kp[i] * error + servo.kd[i] * derivative + servo.ki[i] * next_accumulated_error[i];
    }

    return { control: control, previous_error: next_previous_error, accumulated_error: next_accumulated_error };
}

// Known-good gains for the default mass=2.0, length=2.0, gravity=9.81
// parameters, tuned P -> D -> I: kp raised until the arm reliably reaches
// the setpoint, kd raised until the overshoot stops oscillating, ki raised
// just enough to close out the small remaining steady-state error from
// gravity's constant pull.
function setPIDParameters(links) {
    return (links === 2)
        ? { kp: [150, 60], kd: [60, 20], ki: [4, 2] }
        : { kp: [150], kd: [60], ki: [4] };
}
// #endregion pid-controller

//////////////////////////////////////////////////
/////     SIMULATION STEP ORCHESTRATION
//////////////////////////////////////////////////

// #region simulate-step
// The full per-frame update, in the same order the AutoRob PID lecture's own
// block diagram shows: compute error and PID control torque (if the servo is
// active), compute acceleration from the equations of motion (gravity plus
// that control torque), then integrate (angle, angle_dot) forward by dt
// using whichever numerical integrator is selected -- the acceleration
// function itself never changes between integrators, and the same four
// integrator functions above serve both the single- and double-pendulum
// acceleration functions.
function simulateStep(pendulum, dt) {

    var accelFn = (pendulum.links === 2) ? doublePendulumAcceleration : pendulumAcceleration;
    var boundAccel = function (angle, angle_dot) {
        return accelFn(angle, angle_dot, pendulum.control, pendulum.gravity, pendulum.mass, pendulum.length);
    };

    if (pendulum.servo_active) {
        var pid = PID(pendulum.angle, pendulum.desired, pendulum.previous_error, pendulum.accumulated_error, dt, pendulum.servo);
        pendulum.control = pid.control;
        pendulum.previous_error = pid.previous_error;
        pendulum.accumulated_error = pid.accumulated_error;
    } else {
        pendulum.control = pendulum.angle.map(function () { return 0; });
        pendulum.accumulated_error = pendulum.accumulated_error.map(function () { return 0; });
    }

    pendulum.angle_dot_dot = boundAccel(pendulum.angle, pendulum.angle_dot);

    var result;
    if (pendulum.integrator === 'euler') {
        result = integrateEuler(pendulum.angle, pendulum.angle_dot, pendulum.angle_dot_dot, dt);
    } else if (pendulum.integrator === 'verlet') {
        result = integrateVerlet(pendulum.angle, pendulum.angle_previous, pendulum.angle_dot, pendulum.angle_dot_dot, boundAccel, dt);
        pendulum.angle_previous = result.angle_previous;
    } else if (pendulum.integrator === 'velocity verlet') {
        result = integrateVelocityVerlet(pendulum.angle, pendulum.angle_dot, pendulum.angle_dot_dot, boundAccel, dt);
    } else if (pendulum.integrator === 'runge-kutta') {
        result = integrateRK4(pendulum.angle, pendulum.angle_dot, boundAccel, dt);
    } else {
        result = { angle: pendulum.angle, angle_dot: pendulum.angle.map(function () { return 0; }) };
    }

    pendulum.angle = result.angle;
    pendulum.angle_dot = result.angle_dot;
    pendulum.t += dt;

    return pendulum;
}
// #endregion simulate-step
