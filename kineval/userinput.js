/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | User Interaction

    The forward-kinematics-relevant subset of the upstream kineval-stencil's
    own kineval_userinput.js: active-joint navigation (j/k/l/h, walking the
    kinematic tree child/parent/sibling), per-joint angle control (u/i),
    base translation/turning (w/s/a/d/q/e), and camera zoom (z/x) -- every
    one of these ported with upstream's own key bindings, increment sizes,
    and (for w/s/q/e) its exact robot_heading/robot_lateral-based motion
    formula, unchanged.

    Also wired up, matching upstream's own key scheme exactly: `p`
    (one-shot, iterate inverse kinematics -- see inverse_kinematics.js),
    `r`/`f` (held, move the IK target up/down along world Y by upstream's
    own +-0.01 increment), and `o`/`c` (held, trigger the P-servo-to-
    setpoint and clock-driven setpoint controllers -- see servo_control.js).

    And now motion planning, per upstream's kineval_userinput.js:65-86:
    `m` plans, `n`/`b` step forward and back through a found plan, and
    `Shift+[1-9]` / `[1-9]` / `g` store, load and print pose-setpoint
    slots (see motion_plan.js, collision.js). Note those are the VERIFIED
    upstream semantics -- Shift to assign, unshifted to load -- rather
    than the "0-9 pose-setpoint slots" that kineval/README.md used to
    describe; the README has been corrected to match the source.

    Uses a minimal inline held-keys Set, the same "one dependency fewer"
    replacement for the upstream stencil's vendored THREEx.KeyboardState
    helper every other reference implementation in this project already
    uses (see e.g. simulation/reference/infrastructure.js).

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// #region keyboard-state
function createKeyboardState() {
    var held = new Set();
    window.addEventListener('keydown', function (e) { held.add(e.key.toLowerCase()); });
    window.addEventListener('keyup', function (e) { held.delete(e.key.toLowerCase()); });
    return { pressed: function (key) { return held.has(key.toLowerCase()); } };
}
// #endregion keyboard-state

// #region active-joint-navigation
// matches upstream kineval.changeActiveLinkDown exactly: descend past the
// CURRENT active joint into its child link, then re-focus on that link's
// own first child joint (one level further than changeActiveLinkUp's
// inverse) -- including upstream's own edge case of landing on
// `undefined` if that child link turns out to be a leaf.
function changeActiveLinkDown(robot, params) {
    var childLink = robot.joints[params.active_joint].child;
    if (robot.links[childLink].children !== undefined) {
        params.active_link = childLink;
        params.active_joint = robot.links[childLink].children[0];
    }
}

function changeActiveLinkUp(robot, params) {
    if (params.active_link !== robot.base) {
        params.active_joint = robot.links[params.active_link].parent;
        params.active_link = robot.joints[params.active_joint].parent;
    }
}

function changeActiveLinkSibling(robot, params, direction) {
    var siblings = robot.links[params.active_link].children;
    if (siblings.length === 0) return;
    var idx = siblings.indexOf(params.active_joint);
    params.active_joint = siblings[(siblings.length + idx + direction) % siblings.length];
}
// #endregion active-joint-navigation

// #region handle-user-input
function handleUserInput(robot, params, keyboard, camera) {
    if (keyboard.pressed('z')) {
        camera.position.x += 0.1 * (robot.origin.xyz[0] - camera.position.x);
        camera.position.y += 0.1 * (robot.origin.xyz[1] - camera.position.y);
        camera.position.z += 0.1 * (robot.origin.xyz[2] - camera.position.z);
    } else if (keyboard.pressed('x')) {
        camera.position.x -= 0.1 * (robot.origin.xyz[0] - camera.position.x);
        camera.position.y -= 0.1 * (robot.origin.xyz[1] - camera.position.y);
        camera.position.z -= 0.1 * (robot.origin.xyz[2] - camera.position.z);
    }

    if (keyboard.pressed('u')) {
        robot.joints[params.active_joint].control += 0.01;
    } else if (keyboard.pressed('i')) {
        robot.joints[params.active_joint].control += -0.01;
    }

    if (keyboard.pressed('a')) robot.control.rpy[1] += 0.1;
    if (keyboard.pressed('d')) robot.control.rpy[1] += -0.1;

    if (keyboard.pressed('w')) {
        robot.control.xyz[2] += 0.1 * (robot.heading[2][0] - robot.origin.xyz[2]);
        robot.control.xyz[0] += 0.1 * (robot.heading[0][0] - robot.origin.xyz[0]);
    }
    if (keyboard.pressed('s')) {
        robot.control.xyz[2] += -0.1 * (robot.heading[2][0] - robot.origin.xyz[2]);
        robot.control.xyz[0] += -0.1 * (robot.heading[0][0] - robot.origin.xyz[0]);
    }
    if (keyboard.pressed('q')) {
        robot.control.xyz[2] += 0.1 * (robot.lateral[2][0] - robot.origin.xyz[2]);
        robot.control.xyz[0] += 0.1 * (robot.lateral[0][0] - robot.origin.xyz[0]);
    }
    if (keyboard.pressed('e')) {
        robot.control.xyz[2] += -0.1 * (robot.lateral[2][0] - robot.origin.xyz[2]);
        robot.control.xyz[0] += -0.1 * (robot.lateral[0][0] - robot.origin.xyz[0]);
    }

    // move the IK target up/down along world Y, matching upstream's own
    // kineval.params.ik_target.position[1][0] +-0.01 held-key increment
    // (same xyz axis order as robot.origin.xyz/robot.control.xyz elsewhere
    // in this file -- index 1 is world Y, "up")
    if (keyboard.pressed('r')) params.ik_target.xyz[1] += 0.01;
    if (keyboard.pressed('f')) params.ik_target.xyz[1] -= 0.01;

    // execute P-servo-to-setpoint controller (servo_control.js), matching
    // upstream's own o/c held-key scheme: 'o' drives toward whatever is
    // currently in params.setpoint_target, 'c' additionally re-derives that
    // setpoint from the clock every frame first (setpointClockMovement)
    if (keyboard.pressed('o')) {
        params.update_pd = true;
        params.update_pd_clock = false;
        params.update_pd_dance = false;
    }
    if (keyboard.pressed('c')) {
        params.update_pd = true;
        params.update_pd_clock = true;
    }
}

// one-shot (keydown, not held) bindings for discrete active-joint navigation
function initKeyEvents(robot, params) {
    window.addEventListener('keydown', function (e) {
        // pose-setpoint slots and motion-plan traversal, matching upstream
        // kineval_userinput.js:65-86 exactly: Shift+[1-9] assigns the
        // current pose to a slot, [1-9] loads a slot as the setpoint
        // target (which is the planner's goal), g prints the slots.
        //
        // Checked before the lower-cased switch below because the digit
        // keys carry no case and Shift is the only thing distinguishing
        // assign from load. e.key is '1'..'9' with or without Shift on a
        // number row, so e.shiftKey is the discriminator rather than the
        // shifted character ('!' etc.), which differs by layout.
        if (e.key >= '1' && e.key <= '9') {
            var slot = parseInt(e.key, 10);
            if (e.shiftKey) assignPoseSetpoint(robot, params, slot);
            else loadPoseSetpoint(robot, params, slot);
            return;
        }

        switch (e.key.toLowerCase()) {
            case 'j': changeActiveLinkDown(robot, params); break;
            case 'k': changeActiveLinkUp(robot, params); break;
            case 'l': changeActiveLinkSibling(robot, params, 1); break;
            case 'h': changeActiveLinkSibling(robot, params, -1); break;
            // one-shot IK iteration request, consumed and cleared by
            // robotInverseKinematics (inverse_kinematics.js) on the next frame
            case 'p': params.update_ik = true; break;

            // one-shot motion-planning request, consumed and cleared by
            // planMotionRRTConnect (motion_plan.js) on the next frame.
            // Plans from the robot's CURRENT pose to whatever is in
            // params.setpoint_target, so the workflow is: drive the robot
            // to a goal pose, Shift+1 to store it, drive back, 1 to select
            // it, m to plan.
            case 'm': params.update_motion_plan = true; break;

            // n/b are pure index navigation over a COMPLETED plan -- they
            // do no planning and have no effect before one exists. Clamped
            // at both ends rather than wrapping (see
            // motion_plan.js#stepMotionPlanTraversal).
            case 'n': stepMotionPlanTraversal(params, 1); break;
            case 'b': stepMotionPlanTraversal(params, -1); break;

            case 'g': printPoseSetpoints(params); break;
        }
    });
}
// #endregion handle-user-input

// #region pose-setpoints
// Nine slots holding whole robot poses -- every joint angle plus the base
// pose. This is the module's goal-selection mechanism: the planner plans
// to whichever slot was last loaded.
//
// The base pose is stored alongside the joint angles, which upstream does
// not do (its setpoints are joint-only). Without it, "plan to slot 1"
// would mean "reach that arm configuration wherever you happen to be
// standing", and every plan in every ground-plane world would be solvable
// without the base moving at all -- which makes thirteen of the fourteen
// worlds pointless.
function assignPoseSetpoint(robot, params, slot) {
    if (!params.pose_setpoints) params.pose_setpoints = {};

    var pose = { joints: {}, base: { xyz: robot.origin.xyz.slice(), rpy: robot.origin.rpy.slice() } };
    var x;
    for (x in robot.joints) pose.joints[x] = robot.joints[x].angle;

    params.pose_setpoints[slot] = pose;
    params.setpoint_slot = slot;
    console.log('kineval: pose setpoint ' + slot + ' assigned from current pose');
}

function loadPoseSetpoint(robot, params, slot) {
    if (!params.pose_setpoints || !params.pose_setpoints[slot]) {
        console.warn('kineval: pose setpoint ' + slot + ' is empty; assign it first with Shift+' + slot);
        return;
    }
    var pose = params.pose_setpoints[slot];

    // these two are what robotRRTPlannerInit reads as its goal
    params.setpoint_target = {};
    var x;
    for (x in pose.joints) params.setpoint_target[x] = pose.joints[x];
    params.setpoint_base = { xyz: pose.base.xyz.slice(), rpy: pose.base.rpy.slice() };

    params.setpoint_slot = slot;
    console.log('kineval: pose setpoint ' + slot + ' loaded as planning goal / servo target');
}

function printPoseSetpoints(params) {
    console.log('kineval: pose setpoints', params.pose_setpoints || {});
}
// #endregion pose-setpoints

var HELP_TEXT =
    'KinEval full viewer -- forward kinematics controls' +
    '<br>mouse: orbit camera (drag), scroll to zoom' +
    '<br>z/x : camera zoom with respect to base' +
    '<br>w/s a/d q/e : move base forward/turn/strafe' +
    '<br>j/k/l/h : focus active joint to child/parent/next-sibling/previous-sibling' +
    '<br>u/i : rotate active joint' +
    '<br>p : iterate inverse kinematics motion toward IK target' +
    '<br>r/f : move inverse kinematics target up/down' +
    '<br>o : control robot arm to current setpoint target' +
    '<br>c : execute clock-driven setpoint controller' +
    '<br>&nbsp;' +
    '<br>motion planning (RRT-Connect)' +
    '<br>Shift+1..9 : store the current pose in a setpoint slot' +
    '<br>1..9 : load a slot as the planning goal' +
    '<br>g : print the setpoint slots to the console' +
    '<br>m : plan from the current pose to the loaded goal' +
    '<br>n/b : step forward/back through a found plan' +
    '<br>&nbsp;' +
    '<br>t : frame the camera on the whole world' +
    '<br>?robot= ?world= ?seed= select robot, world and random seed' +
    '<br>v : show this help';
