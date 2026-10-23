/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Inverse Kinematics | Infrastructure

    URL-parameter configuration (every test case in the accompanying Slidev
    deck is a plain hyperlink, same convention as every other module's
    infrastructure.js), robot/chain selection, and the Start/Pause/Reset/Step
    controls -- which here step through JACOBIAN ITERATIONS of iterateIK,
    one per step_ms interval while running (or exactly one on a "Step"
    click), rather than forward_kinematics/'s DFS-traversal-order stepping:
    there is no fixed "order" to an IK solve, only successive refinements of
    the same joint configuration.

    Each iteration runs the same three-stage per-frame pipeline a real
    controller tick would: robotInverseKinematics (writes .control deltas),
    applyControls (folds .control into .angle, ported below from
    ../../kineval/controls.js -- see that file's header), then
    buildFKTransforms (recomputes every joint's/link's world .matrix from
    the new angles) -- matching iterateIK's own comment in
    inverse_kinematics.js about why it writes to .control rather than .angle
    directly.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see inverse_kinematics.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// #region appendix-url-params
function readIKParams() {
    var q = new URLSearchParams(window.location.search);
    return {
        robot: (q.get('robot') || 'mobile_arm').toLowerCase(),
        method: (q.get('method') || 'transpose').toLowerCase(),
        steplength: q.has('steplength') ? Number(q.get('steplength')) : 0.3,
        damping: q.has('damping') ? Number(q.get('damping')) : 0.15,
        orientation: q.get('orientation') === '1',
        target_x: q.has('target_x') ? Number(q.get('target_x')) : null,
        target_y: q.has('target_y') ? Number(q.get('target_y')) : null,
        target_z: q.has('target_z') ? Number(q.get('target_z')) : null,
        autoplay: q.get('autoplay') !== '0',
        step_ms: q.has('step_ms') ? Number(q.get('step_ms')) : 50
    };
}

// Each entry's `create` is the robot's bare factory function. `needsThree`
// marks robots/robot_mr2.js's style of factory (ported verbatim from
// ../../kineval/robots/robot_mr2.js), which expects a THREE namespace
// passed in as an argument -- those run through scene.js's
// instantiateThreeRobot so THREE is still only ever imported there.
var ROBOT_FACTORIES = {
    mobile_arm: { create: createMobileArmRobot, needsThree: false },
    urdf_example: { create: createUrdfExampleRobot, needsThree: false },
    mr2: { create: createRobotMr2, needsThree: true },
    fetch: { create: createRobotFetch, needsThree: false }
};

// The ordered base-to-end-effector chain of ACTUAL ANCESTOR joints driving
// each robot's end-effector. Not simply "every joint": urdf_example's link1
// branches into joint1 (-> link2, a dead-end sibling) and joint2 (-> link3
// -> joint3 -> link4, which the end-effector sits on) -- joint1 is excluded.
// mr2's right arm (clavicle_right_yaw, shoulder_right_yaw,
// upperarm_right_pitch, forearm_right_yaw) is a complete, independent chain
// from base to end-effector; clavicle_left_roll is a disconnected sibling
// branch off `base` (mr2's left arm, left deliberately unfinished by
// upstream's own stencil) with zero effect on this end-effector, and is
// excluded for the same reason joint1 is.
// fetch's chain is this table's only one with a PRISMATIC joint
// (torso_lift_joint) -- see ../inverse_kinematics.js#manipulator-jacobian's
// prismatic branch, which exists specifically to drive this column. It also
// ends in gripper_axis, a `fixed`-type joint: walking base-to-end-effector
// via each joint's actual ancestor chain (robot_fetch.js: gripper_axis's
// parent link is wrist_roll_link, whose parent joint is wrist_roll_joint,
// ... up to torso_lift_joint's parent link base_link, the robot's base)
// necessarily ends AT the end-effector's own joint, same as urdf_example's
// chain ending in joint3 and mr2's ending in forearm_right_yaw above --
// gripper_axis is no different, it's just `fixed` rather than revolute.
// computeManipulatorJacobian has no `fixed`-type special case (only
// `prismatic` branches differently from everything else), so gripper_axis's
// own column runs through the revolute-shaped branch same as any other
// non-prismatic joint -- harmlessly: robot.endeffector.position sits 0.1m
// out along gripper_axis's own local +x, which is also gripper_axis's own
// rotation axis, so p_ee - p_gripper_axis is parallel to gripper_axis's
// world axis and that column's cross product comes out the zero vector
// (see robot_fetch.js's own header for this same point).
var CHAIN_JOINT_NAMES = {
    mobile_arm: ['joint_pitch', 'joint1', 'joint2'],
    urdf_example: ['joint2', 'joint3'],
    mr2: ['clavicle_right_yaw', 'shoulder_right_yaw', 'upperarm_right_pitch', 'forearm_right_yaw'],
    fetch: ['torso_lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint',
        'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint', 'gripper_axis']
};

// A non-singular starting pose per robot (every joint at angle 0 is a valid
// but visually uninteresting fully-extended arm) -- purely cosmetic, picked
// so the IK solve has visible work to do converging toward the default target.
var INITIAL_ANGLES = {
    mobile_arm: { joint_pitch: 0.3, joint1: 0.5, joint2: -0.4 },
    urdf_example: { joint2: 0.3, joint3: -0.5 },
    mr2: { clavicle_right_yaw: 0.2, shoulder_right_yaw: 0.3, upperarm_right_pitch: 0.4, forearm_right_yaw: -0.3 },
    fetch: {
        torso_lift_joint: 0.1, shoulder_pan_joint: 0.3, shoulder_lift_joint: 0.3, upperarm_roll_joint: 0.2,
        elbow_flex_joint: 0.6, forearm_roll_joint: 0, wrist_flex_joint: 0.3, wrist_roll_joint: 0
    }
};

// A reachable-ish default IK target per robot, overridden per-axis by
// ?target_x=/?target_y=/?target_z=. Kept comfortably INSIDE each chain's
// max reach (not just barely reachable) -- a target near full extension
// drives the solver toward the classic straight-arm Jacobian singularity
// (see inverse_kinematics.js's manipulator-jacobian region comment on
// parallel columns), which the 'pseudoinverse' method's plain
// normal-equations inverse (matrix_square_invert on a then-singular J^T J)
// has no regularization against -- 'damped' is the method built to survive
// that; these defaults are chosen so 'transpose'/'pseudoinverse' have a
// genuine, well-conditioned path to the target too.
var DEFAULT_TARGETS = {
    // mobile_arm: every joint (joint_pitch, joint1, joint2) rotates about
    // the SAME world axis -- all three origins have rpy=[0,0,0], and every
    // one of their own variable rotations is about local +z, which (per
    // computeManipulatorJacobian's own z_i comment) a joint's own rotation
    // never tilts out of alignment with its ancestors' -- so every link's
    // local z stays locked to world z however the arm moves, and every
    // translation (each link only ever extends along its own local +x)
    // stays confined to the world x-y plane. The end-effector's world z is
    // therefore IDENTICALLY 0 for every joint configuration: a target with
    // z != 0 would be permanently unreachable, not just far away.
    mobile_arm: [0.4, 0.55, 0],
    urdf_example: [0.7, 0.75, 0.25],
    mr2: [0.4, 1.15, 0.45],
    // fetch's gripper (chain's INITIAL_ANGLES pose) sits at roughly
    // [0.85, 0.30, 0.23]; fully extended (every joint at 0) it reaches
    // roughly [1.23, 0, 0.79] -- this target is comfortably inside that,
    // reachable by all three methods without nearing the straight-arm
    // singularity.
    fetch: [0.75, 0.2, 0.6]
};

function resolveRobotName(params) {
    return ROBOT_FACTORIES[params.robot] ? params.robot : 'mobile_arm';
}

function createRobotFromParams(params, instantiateWithThree) {
    var robotName = resolveRobotName(params);
    var entry = ROBOT_FACTORIES[robotName];

    var built = entry.needsThree
        ? instantiateWithThree(entry.create)       // -> {robot, links_geom}, THREE supplied by scene.js
        : { robot: entry.create(), links_geom: null };

    initRobot(built.robot);

    // kinematics.js here is copied VERBATIM from forward_kinematics/reference/
    // (see this project's module-mirroring convention), whose own initRobotJoints
    // never needed to initialize a per-joint .control field -- that module never
    // calls applyControls at all. The full kineval/ viewer's OWN (non-verbatim)
    // kinematics.js adds exactly this initialization for exactly this reason
    // (kineval/kinematics.js: "if (robot.joints[x].control === undefined)
    // robot.joints[x].control = 0;"). This module needs the same fix -- iterateIK
    // accumulates onto .control, and applyControls (below) folds it into .angle --
    // so it's done here instead, right after initRobot, rather than editing the
    // verbatim copy.
    var jointName;
    for (jointName in built.robot.joints) {
        if (built.robot.joints[jointName].control === undefined) built.robot.joints[jointName].control = 0;
    }

    var initialAngles = INITIAL_ANGLES[robotName] || {};
    for (jointName in initialAngles) {
        built.robot.joints[jointName].angle = initialAngles[jointName];
    }

    return {
        robot: built.robot,
        linksGeom: built.links_geom,
        chainJointNames: CHAIN_JOINT_NAMES[robotName]
    };
}

function targetPositionFromParams(params) {
    var robotName = resolveRobotName(params);
    var fallback = DEFAULT_TARGETS[robotName];
    return [
        params.target_x !== null ? params.target_x : fallback[0],
        params.target_y !== null ? params.target_y : fallback[1],
        params.target_z !== null ? params.target_z : fallback[2]
    ];
}

// A modest, fixed target orientation (0.4 rad about world +y) used whenever
// orientation IK is switched on -- there's no separate ?target_roll=/etc.
// URL param surface for this (position is this module's primary demo), but
// ?orientation=1 still needs SOME concrete target quaternion for iterateIK's
// 6x1 dx branch to have anything real to chase, rather than that branch
// staying permanently inert.
function defaultTargetOrientation() {
    return quaternion_normalize(quaternion_from_axisangle([0, 1, 0], 0.4));
}
// #endregion appendix-url-params

// #region apply-controls
// Ported from ../../kineval/controls.js's applyControls (itself ported from
// upstream's kineval_controls.js kineval.applyControls): folds every
// joint's accumulated per-frame .control delta into .angle (then zeroes it),
// same for the base's .control.xyz/.rpy into .origin.xyz/.rpy. The one place
// iterateIK's per-iteration .control contribution actually becomes a joint
// angle -- see inverse_kinematics.js's iterateIK for why it writes .control
// and leaves this step to run separately, once per iteration here.
function applyControls(robot) {
    var x;
    for (x in robot.joints) {
        robot.joints[x].angle += robot.joints[x].control;
        robot.joints[x].control = 0;
    }

    robot.origin.xyz[0] += robot.control.xyz[0];
    robot.origin.xyz[1] += robot.control.xyz[1];
    robot.origin.xyz[2] += robot.control.xyz[2];
    robot.origin.rpy[0] += robot.control.rpy[0];
    robot.origin.rpy[1] += robot.control.rpy[1];
    robot.origin.rpy[2] += robot.control.rpy[2];

    robot.control = { xyz: [0, 0, 0], rpy: [0, 0, 0] };
}
// #endregion apply-controls

// #region hud-panel
function buildHud(container, app) {

    var hud = document.createElement('div');
    hud.className = 'ik-hud';
    container.appendChild(hud);

    var status = document.createElement('pre');
    status.className = 'ik-status';
    hud.appendChild(status);

    var controls = document.createElement('div');
    controls.className = 'ik-controls';
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
    button('Step', function () {
        app.isRunning = false; playBtn.textContent = 'Start'; app.stepOnce = true;
    });
    button('Reset', function () {
        app.reset();
        playBtn.textContent = app.isRunning ? 'Pause' : 'Start';
    });

    var robotSelect = document.createElement('select');
    Object.keys(ROBOT_FACTORIES).forEach(function (name) {
        var opt = document.createElement('option');
        opt.value = name; opt.textContent = name;
        if (name === resolveRobotName(app.params)) opt.selected = true;
        robotSelect.appendChild(opt);
    });
    robotSelect.addEventListener('change', function () {
        app.params.robot = robotSelect.value;
        app.params.target_x = app.params.target_y = app.params.target_z = null; // adopt new robot's default target
        app.reset();
        playBtn.textContent = app.isRunning ? 'Pause' : 'Start';
    });
    controls.appendChild(robotSelect);

    var methodSelect = document.createElement('select');
    ['transpose', 'pseudoinverse', 'damped'].forEach(function (name) {
        var opt = document.createElement('option');
        opt.value = name; opt.textContent = name;
        if (name === app.ikParams.method) opt.selected = true;
        methodSelect.appendChild(opt);
    });
    methodSelect.addEventListener('change', function () { app.ikParams.method = methodSelect.value; });
    controls.appendChild(methodSelect);

    function numberField(labelText, value, step, onInput) {
        var row = document.createElement('label');
        row.className = 'ik-field';
        var span = document.createElement('span');
        span.textContent = labelText;
        row.appendChild(span);
        var input = document.createElement('input');
        input.type = 'number'; input.step = step; input.value = value;
        input.addEventListener('input', function () { onInput(Number(input.value)); });
        row.appendChild(input);
        controls.appendChild(row);
        return input;
    }

    numberField('step length', app.ikParams.steplength, 0.05, function (v) { app.ikParams.steplength = v; });
    numberField('damping', app.ikParams.damping, 0.05, function (v) { app.ikParams.damping = v; });

    var orientationCheckbox = document.createElement('input');
    orientationCheckbox.type = 'checkbox';
    orientationCheckbox.checked = app.ikParams.orientationIncluded;
    orientationCheckbox.addEventListener('change', function () {
        app.ikParams.orientationIncluded = orientationCheckbox.checked;
        // iterateIK's orientation branch only engages when the target
        // actually carries an .orientation quaternion (see its
        // includeOrientation gate) -- give it one the first time the
        // checkbox is switched on, rather than leaving the branch inert.
        if (orientationCheckbox.checked && !app.target.orientation) {
            app.target.orientation = defaultTargetOrientation();
        }
    });
    var orientationLabel = document.createElement('label');
    orientationLabel.className = 'ik-field';
    orientationLabel.appendChild(orientationCheckbox);
    orientationLabel.appendChild(document.createTextNode('orientation'));
    controls.appendChild(orientationLabel);

    var targetRow = document.createElement('div');
    targetRow.className = 'ik-controls';
    hud.appendChild(targetRow);

    ['x', 'y', 'z'].forEach(function (axis, i) {
        var row = document.createElement('label');
        row.className = 'ik-field';
        var span = document.createElement('span');
        span.textContent = 'target ' + axis;
        row.appendChild(span);
        var input = document.createElement('input');
        input.type = 'number'; input.step = 0.05; input.value = app.target.position[i].toFixed(2);
        input.addEventListener('input', function () {
            app.target.position[i] = Number(input.value);
        });
        row.appendChild(input);
        targetRow.appendChild(row);
        app.targetInputs = app.targetInputs || [];
        app.targetInputs[i] = input;
    });

    var hint = document.createElement('div');
    hint.className = 'ik-hint';
    hint.textContent = 'Arrow keys move target in X/Z, PageUp/PageDown move target in Y.';
    hud.appendChild(hint);

    return { status: status, robotSelect: robotSelect, methodSelect: methodSelect, playBtn: playBtn };
}

// Arrow-keyed target control (the "or arrow-keyed" half of the brief's
// "draggable-or-arrow-keyed target position readout" -- picked over
// pointer-drag/raycasting to keep this module's risk surface small).
function initTargetKeyControl(app, hud) {
    var STEP = 0.05;
    window.addEventListener('keydown', function (e) {
        var moved = true;
        if (e.key === 'ArrowLeft') app.target.position[0] -= STEP;
        else if (e.key === 'ArrowRight') app.target.position[0] += STEP;
        else if (e.key === 'ArrowUp') app.target.position[2] -= STEP;
        else if (e.key === 'ArrowDown') app.target.position[2] += STEP;
        else if (e.key === 'PageUp') app.target.position[1] += STEP;
        else if (e.key === 'PageDown') app.target.position[1] -= STEP;
        else moved = false;

        if (moved) {
            e.preventDefault();
            if (app.targetInputs) {
                app.targetInputs.forEach(function (input, i) { input.value = app.target.position[i].toFixed(2); });
            }
        }
    });
}

// buildHud's own Start/Pause/Reset click handlers keep playBtn's label in
// sync with app.isRunning as a side effect of handling the click itself,
// but the HTML page's very first app.reset() call (run once, right after
// buildHud, to load the initial robot) happens from OUTSIDE any of those
// handlers -- nothing else would otherwise sync the button's label to the
// autoplay-driven app.isRunning that reset() just set.
function syncPlayButtonLabel(hud, app) {
    hud.playBtn.textContent = app.isRunning ? 'Pause' : 'Start';
}

function renderHudStatus(statusEl, app) {
    var dxNorm = 0, i;
    if (app.robot.dx) {
        for (i = 0; i < app.robot.dx.length; i++) dxNorm += app.robot.dx[i][0] * app.robot.dx[i][0];
        dxNorm = Math.sqrt(dxNorm);
    }

    statusEl.textContent =
        'Inverse Kinematics -- Jacobian iteration\n' +
        'robot = ' + app.robot.name + '   method = ' + app.ikParams.method + '\n' +
        'chain = ' + app.ikParams.chainJointNames.join(', ') + '\n' +
        'iteration ' + app.iterationCount + '   |dx| = ' + dxNorm.toFixed(4) + '\n' +
        'target = [' + app.target.position.map(function (v) { return v.toFixed(2); }).join(', ') + ']' +
        '\n\nStart/Step to run iterateIK; drag the target fields or use arrow keys to move it.';
}
// #endregion hud-panel

// #region appendix-animate
// One iterateIK call per step_ms of elapsed wall time while running (or
// exactly one on a forced "Step"), decoupled from the monitor's own refresh
// rate -- unlike forward_kinematics/'s equivalent loop, which advances
// exactly one DFS-traversal node per rAF tick with no real notion of "time"
// to gate on, an IK iteration is a real, perceptible step, so step_ms gives
// control over the convergence rate's real-time pacing rather than tying it
// to however fast the browser renders.
function createAnimationLoop(app, onFrame) {
    var lastStepTime = 0;

    function frame(now) {
        requestAnimationFrame(frame);

        var due = app.stepOnce || (app.isRunning && (now - lastStepTime) >= app.params.step_ms);
        if (due) {
            lastStepTime = now;
            app.stepOnce = false;
            onFrame(true);
        } else {
            onFrame(false);
        }
    }
    requestAnimationFrame(frame);
}
// #endregion appendix-animate
