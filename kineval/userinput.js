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
    formula, unchanged. Out of scope: everything upstream's handleUserInput
    drives that isn't forward kinematics -- PID setpoints (o/c/0-9), inverse
    kinematics (p/r/f), motion planning (m/n/b) -- since none of those
    modules are built in this project yet (see ../README.md's build-order
    table).

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
}

// one-shot (keydown, not held) bindings for discrete active-joint navigation
function initKeyEvents(robot, params) {
    window.addEventListener('keydown', function (e) {
        switch (e.key.toLowerCase()) {
            case 'j': changeActiveLinkDown(robot, params); break;
            case 'k': changeActiveLinkUp(robot, params); break;
            case 'l': changeActiveLinkSibling(robot, params, 1); break;
            case 'h': changeActiveLinkSibling(robot, params, -1); break;
        }
    });
}
// #endregion handle-user-input

var HELP_TEXT =
    'KinEval full viewer -- forward kinematics controls' +
    '<br>mouse: orbit camera (drag), scroll to zoom' +
    '<br>z/x : camera zoom with respect to base' +
    '<br>w/s a/d q/e : move base forward/turn/strafe' +
    '<br>j/k/l/h : focus active joint to child/parent/next-sibling/previous-sibling' +
    '<br>u/i : rotate active joint' +
    '<br>v : show this help';
