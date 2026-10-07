/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Forward Kinematics and Quaternions | Infrastructure

    URL-parameter configuration (every test case in the accompanying Slidev
    deck is a plain hyperlink, same convention as every other module's
    infrastructure.js), robot selection/pose, and the Start/Pause/Reset/Step
    controls -- which here step through the DFS TRAVERSAL ORDER of
    buildFKTransforms (link, joint, link, joint, ...) one node at a time,
    revealing each link's mesh (and highlighting the one just reached) as
    the traversal would visit it, rather than stepping through elapsed
    simulated time the way simulation/reference/infrastructure.js's
    equivalent loop does -- there is no "time" in a forward-kinematics
    evaluation, only tree order.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see forward_kinematics.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// #region appendix-url-params
function parseListParam(searchParams, name, fallback) {
    var raw = searchParams.get(name);
    if (raw === null) return fallback.slice();
    var parts = raw.split(',').map(Number);
    return fallback.map(function (_, i) { return parts[i] !== undefined && !isNaN(parts[i]) ? parts[i] : fallback[i]; });
}

function readFKParams() {
    var q = new URLSearchParams(window.location.search);
    return {
        robot: (q.get('robot') || 'mobile_arm').toLowerCase(),
        angles: q.has('angles') ? parseListParam(q, 'angles', [0, 0, 0, 0]) : null,
        base_x: q.has('base_x') ? Number(q.get('base_x')) : 0,
        base_z: q.has('base_z') ? Number(q.get('base_z')) : 0,
        base_yaw: q.has('base_yaw') ? Number(q.get('base_yaw')) : 0,
        autoplay: q.get('autoplay') !== '0',
        step_ms: q.has('step_ms') ? Number(q.get('step_ms')) : 500
    };
}

var ROBOT_FACTORIES = {
    mobile_arm: createMobileArmRobot,
    urdf_example: createUrdfExampleRobot
};

function createRobotFromParams(params) {
    var factory = ROBOT_FACTORIES[params.robot] || ROBOT_FACTORIES.mobile_arm;
    var robot = factory();
    initRobot(robot);

    robot.origin.xyz[0] = params.base_x;
    robot.origin.xyz[2] = params.base_z;
    robot.origin.rpy[1] = params.base_yaw;

    if (params.angles) {
        var jointNames = Object.keys(robot.joints);
        jointNames.forEach(function (name, i) {
            if (params.angles[i] !== undefined) robot.joints[name].angle = params.angles[i];
        });
    }

    return robot;
}
// #endregion appendix-url-params

// #region fk-traversal-order
// Records the exact node-visitation order traverseFKBase/traverseFKLink/
// traverseFKJoint in kinematics.js walks -- WITHOUT recomputing any of the
// transform math itself (buildFKTransforms, called separately, is the one
// and only place that happens). This is purely bookkeeping for the
// step-through UI, kept intentionally dumb: it mirrors the real traversal's
// recursion shape (a link, then each of its child joints in turn, each
// immediately followed by ITS child link) so that stepping through this
// list and stepping through the real algorithm never disagree.
function recordFKTraversalOrder(robot) {
    var order = [];

    function visitLink(name) {
        order.push({ type: 'link', name: name });
        var link = robot.links[name];
        var i;
        for (i = 0; i < link.children.length; i++) {
            var jointName = link.children[i];
            order.push({ type: 'joint', name: jointName });
            visitLink(robot.joints[jointName].child);
        }
    }

    order.push({ type: 'base', name: robot.base });
    visitLink(robot.base);
    return order;
}
// #endregion fk-traversal-order

// #region hud-panel
function buildHud(container, app) {

    var hud = document.createElement('div');
    hud.className = 'fk-hud';
    container.appendChild(hud);

    var status = document.createElement('pre');
    status.className = 'fk-status';
    hud.appendChild(status);

    var controls = document.createElement('div');
    controls.className = 'fk-controls';
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
    button('Reset', function () {
        app.stepIndex = -1;
        app.isRunning = app.params.autoplay;
        playBtn.textContent = app.isRunning ? 'Pause' : 'Start';
    });

    var robotSelect = document.createElement('select');
    Object.keys(ROBOT_FACTORIES).forEach(function (name) {
        var opt = document.createElement('option');
        opt.value = name; opt.textContent = name;
        if (name === app.params.robot) opt.selected = true;
        robotSelect.appendChild(opt);
    });
    robotSelect.addEventListener('change', function () {
        app.params.robot = robotSelect.value;
        app.reset();
    });
    controls.appendChild(robotSelect);

    var jointSlidersEl = document.createElement('div');
    jointSlidersEl.className = 'fk-joint-sliders';
    hud.appendChild(jointSlidersEl);

    return { status: status, robotSelect: robotSelect, jointSlidersEl: jointSlidersEl };
}

// Rebuilt whenever the robot changes (sliders depend on that robot's own
// joint set, which differs between mobile_arm and urdf_example).
function rebuildJointSliders(hud, app) {
    hud.jointSlidersEl.innerHTML = '';
    var jointNames = Object.keys(app.robot.joints);
    jointNames.forEach(function (name) {
        var joint = app.robot.joints[name];
        var row = document.createElement('label');
        row.className = 'fk-joint-row';

        var span = document.createElement('span');
        span.textContent = name + ' (' + joint.type + ')';
        row.appendChild(span);

        var slider = document.createElement('input');
        slider.type = 'range';
        slider.min = -Math.PI; slider.max = Math.PI; slider.step = 0.01;
        slider.value = joint.angle;
        slider.addEventListener('input', function () {
            joint.angle = Number(slider.value);
            if (app.recompute) app.recompute(); // re-run buildFKTransforms for the new angle
            app.stepIndex = -1; // replay the traversal under the new pose
            app.isRunning = app.params.autoplay;
        });
        row.appendChild(slider);

        hud.jointSlidersEl.appendChild(row);
    });
}

function renderHudStatus(statusEl, app) {
    var order = app.fkOrder;
    var current = app.stepIndex >= 0 && app.stepIndex < order.length ? order[app.stepIndex] : null;
    var line;
    if (!current) {
        line = 'traverseFKBase(robot)';
    } else if (current.type === 'link') {
        line = 'traverseFKLink("' + current.name + '")';
    } else {
        line = 'traverseFKJoint("' + current.name + '")';
    }

    statusEl.textContent =
        'Forward Kinematics -- DFS traversal\n' +
        'robot = ' + app.robot.name + '\n' +
        'step ' + Math.max(0, app.stepIndex + 1) + ' / ' + order.length + '   now executing: ' + line + '\n' +
        '\nDrag a joint slider, then Start/Step to replay the traversal under the new pose.';
}
// #endregion hud-panel

// #region appendix-animate
// Advances the step index by exactly one DFS-traversal node per call, same
// decoupled-from-framerate shape as every other module's animation loop,
// just stepping through tree order instead of physical time.
function createAnimationLoop(app, onFrame) {
    function frame() {
        requestAnimationFrame(frame);
        if (app.isRunning || app.stepOnce) {
            if (app.stepIndex < app.fkOrder.length - 1) {
                app.stepIndex++;
            } else {
                app.isRunning = false;
            }
            app.stepOnce = false;
        }
        onFrame();
    }
    requestAnimationFrame(frame);
}
// #endregion appendix-animate
