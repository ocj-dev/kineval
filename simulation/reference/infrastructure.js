/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Pendularm Dynamical Simulation | Infrastructure

    URL-parameter configuration (so every test case in the accompanying
    Slidev deck is a plain hyperlink, no build step or server involved),
    pendulum-state construction/reset, keyboard input (preserving the
    upstream stencil's own 0-4 / a-d / q-e / w-r / c-x-s bindings), the HUD
    status/control panel (Start/Pause/Reset/Step plus live parameter
    controls), and the fixed-timestep animation loop.

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

// #region appendix-url-params
// Every parameter is optional; defaults reproduce a single pendulum released
// from horizontal with a good general-purpose integrator, servo off. Comma-
// separated values set per-link parameters for the double pendulum
// (?links=2&mass=2,1.2); a single value applies to every link.
function parseListParam(searchParams, name, fallback) {
    var raw = searchParams.get(name);
    if (raw === null) return fallback.slice();
    var parts = raw.split(',').map(Number);
    return fallback.map(function (_, i) { return parts[i] !== undefined && !isNaN(parts[i]) ? parts[i] : fallback[i]; });
}

function readSimulationParams() {
    var q = new URLSearchParams(window.location.search);
    var links = q.get('links') === '2' ? 2 : 1;
    var fallbackPair = [2.0, 2.0];

    var integratorAliases = { 'velocity-verlet': 'velocity verlet' };
    var integrator = q.get('integrator') || 'runge-kutta';
    integrator = integratorAliases[integrator] || integrator;

    return {
        links: links,
        mass: parseListParam(q, 'mass', fallbackPair),
        length: parseListParam(q, 'length', fallbackPair),
        gravity: q.has('gravity') ? Number(q.get('gravity')) : 9.81,
        integrator: integrator,
        dt: q.has('dt') ? Number(q.get('dt')) : 0.01,
        initial_angle: parseListParam(q, 'angle0', [Math.PI / 2, 0]),
        desired: parseListParam(q, 'desired', [-1.0, 1.0]),
        servo_active: q.get('servo') === '1',
        kp: q.has('kp') ? parseListParam(q, 'kp', [150, 60]) : null,
        kd: q.has('kd') ? parseListParam(q, 'kd', [60, 20]) : null,
        ki: q.has('ki') ? parseListParam(q, 'ki', [4, 2]) : null,
        autoplay: q.get('autoplay') !== '0'
    };
}
// #endregion appendix-url-params

// #region create-pendulum
function createPendulum(params) {

    var n = params.links;
    var slice = function (arr) { return arr.slice(0, n); };

    var servo = setPIDParameters(n);
    if (params.kp) servo.kp = slice(params.kp);
    if (params.kd) servo.kd = slice(params.kd);
    if (params.ki) servo.ki = slice(params.ki);

    var pendulum = {
        links: n,
        mass: slice(params.mass),
        length: slice(params.length),
        gravity: params.gravity,
        integrator: params.integrator,
        angle: slice(params.initial_angle),
        angle_dot: new Array(n).fill(0),
        angle_previous: new Array(n).fill(0),
        angle_dot_dot: new Array(n).fill(0),
        previous_error: new Array(n).fill(0),
        accumulated_error: new Array(n).fill(0),
        control: new Array(n).fill(0),
        desired: slice(params.desired),
        servo: servo,
        servo_active: params.servo_active,
        t: 0
    };

    if (pendulum.integrator === 'verlet') {
        var a0 = pendulumAccelerationFor(pendulum);
        pendulum.angle_previous = initVerletIntegrator(pendulum.angle, pendulum.angle_dot, a0, params.dt);
    }

    return pendulum;
}

function pendulumAccelerationFor(pendulum) {
    var accelFn = (pendulum.links === 2) ? doublePendulumAcceleration : pendulumAcceleration;
    return accelFn(pendulum.angle, pendulum.angle_dot, pendulum.control, pendulum.gravity, pendulum.mass, pendulum.length);
}
// #endregion create-pendulum

// #region keyboard-state
// A minimal inline replacement for the upstream stencil's vendored
// THREEx.KeyboardState helper: a Set of currently-held key codes, checked
// each frame -- one dependency fewer, same continuous-hold behavior the
// stencil's own a/d "apply user force" binding relies on.
function createKeyboardState() {
    var held = new Set();
    window.addEventListener('keydown', function (e) { held.add(e.key.toLowerCase()); });
    window.addEventListener('keyup', function (e) { held.delete(e.key.toLowerCase()); });
    return { pressed: function (key) { return held.has(key.toLowerCase()); } };
}

// Preserves the stencil's own bindings: [0-4] integrator, a/d user-force
// impulse, q/e joint-1 desired angle, w/r joint-2 desired angle (double
// pendulum only), c|x servo toggle, s momentary disable.
function applyKeyboardBindings(keyboard, pendulum, app) {
    var integratorByKey = { '0': 'none', '1': 'euler', '2': 'verlet', '3': 'velocity verlet', '4': 'runge-kutta' };
    for (var key in integratorByKey) {
        if (keyboard.pressed(key)) pendulum.integrator = integratorByKey[key];
    }

    if (keyboard.pressed('d')) pendulum.control[0] += 50.0;
    else if (keyboard.pressed('a')) pendulum.control[0] += -50.0;

    if (keyboard.pressed('e')) pendulum.desired[0] += 0.02;
    if (keyboard.pressed('q')) pendulum.desired[0] += -0.02;
    if (pendulum.links === 2) {
        if (keyboard.pressed('r')) pendulum.desired[1] += 0.02;
        if (keyboard.pressed('w')) pendulum.desired[1] += -0.02;
    }

    if (keyboard.pressed('c') || keyboard.pressed('x')) {
        if (!app.servoKeyHeld) pendulum.servo_active = !pendulum.servo_active;
        app.servoKeyHeld = true;
    } else {
        app.servoKeyHeld = false;
    }
    if (keyboard.pressed('s')) pendulum.servo_active = false;
}
// #endregion keyboard-state

// #region hud-panel
function buildHud(container, app) {

    var hud = document.createElement('div');
    hud.className = 'pendularm-hud';
    container.appendChild(hud);

    var status = document.createElement('pre');
    status.className = 'pendularm-status';
    hud.appendChild(status);

    var controls = document.createElement('div');
    controls.className = 'pendularm-controls';
    hud.appendChild(controls);

    function button(label, onClick) {
        var b = document.createElement('button');
        b.textContent = label;
        b.addEventListener('click', onClick);
        controls.appendChild(b);
        return b;
    }

    var playBtn = button(app.isRunning ? 'Pause' : 'Start', function () {
        app.isRunning = !app.isRunning;
        playBtn.textContent = app.isRunning ? 'Pause' : 'Start';
    });
    button('Step', function () { app.isRunning = false; playBtn.textContent = 'Start'; app.stepOnce = true; });
    button('Reset', function () { app.reset(); playBtn.textContent = app.isRunning ? 'Pause' : 'Start'; });

    var servoLabel = document.createElement('label');
    var servoCheckbox = document.createElement('input');
    servoCheckbox.type = 'checkbox';
    servoCheckbox.checked = app.pendulum.servo_active;
    servoCheckbox.addEventListener('change', function () { app.pendulum.servo_active = servoCheckbox.checked; });
    servoLabel.appendChild(servoCheckbox);
    servoLabel.appendChild(document.createTextNode(' PID servo active'));
    controls.appendChild(servoLabel);

    var integratorSelect = document.createElement('select');
    ['none', 'euler', 'verlet', 'velocity verlet', 'runge-kutta'].forEach(function (name) {
        var opt = document.createElement('option');
        opt.value = name; opt.textContent = name;
        if (name === app.pendulum.integrator) opt.selected = true;
        integratorSelect.appendChild(opt);
    });
    integratorSelect.addEventListener('change', function () {
        app.pendulum.integrator = integratorSelect.value;
        if (integratorSelect.value === 'verlet') {
            var a0 = pendulumAccelerationFor(app.pendulum);
            app.pendulum.angle_previous = initVerletIntegrator(app.pendulum.angle, app.pendulum.angle_dot, a0, app.params.dt);
        }
    });
    controls.appendChild(integratorSelect);

    return { status: status, integratorSelect: integratorSelect, servoCheckbox: servoCheckbox };
}

function renderHudStatus(statusEl, pendulum) {
    var fmt = function (arr) { return arr.map(function (v) { return v.toFixed(3); }).join(', '); };
    statusEl.textContent =
        'Pendularm Dynamical Simulation\n' +
        'links = ' + pendulum.links + '   integrator = ' + pendulum.integrator + '   t = ' + pendulum.t.toFixed(2) + 's\n' +
        'angle       = [' + fmt(pendulum.angle) + ']\n' +
        'angle_dot   = [' + fmt(pendulum.angle_dot) + ']\n' +
        'desired     = [' + fmt(pendulum.desired) + ']\n' +
        'servo: ' + (pendulum.servo_active ? 'active' : 'disabled') +
        '   control = [' + fmt(pendulum.control) + ']\n' +
        'kp = [' + fmt(pendulum.servo.kp) + ']  kd = [' + fmt(pendulum.servo.kd) + ']  ki = [' + fmt(pendulum.servo.ki) + ']\n' +
        'mass = [' + fmt(pendulum.mass) + ']  length = [' + fmt(pendulum.length) + ']  gravity = ' + pendulum.gravity.toFixed(2) + '\n' +
        '\nKeys: [0-4] integrator  a/d force  q/e desired(1)  w/r desired(2)  c|x servo  s disable';
}
// #endregion hud-panel

// #region appendix-animate
// Advances the simulation by exactly one fixed-size timestep per call --
// decoupled from wall-clock frame time, so the physics is deterministic and
// reproducible regardless of the browser's actual framerate, exactly as
// every URL-parameter test case expects.
function createAnimationLoop(app, onFrame) {
    function frame() {
        requestAnimationFrame(frame);
        if (app.isRunning || app.stepOnce) {
            simulateStep(app.pendulum, app.params.dt);
            app.stepOnce = false;
        }
        onFrame();
    }
    requestAnimationFrame(frame);
}
// #endregion appendix-animate

// #region resize-canvas
function observeContainerResize(container, onResize) {
    var ro = new ResizeObserver(function () { onResize(); });
    ro.observe(container);
    window.addEventListener('resize', onResize);
}
// #endregion resize-canvas
