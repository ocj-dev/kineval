/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | Servo / Setpoint Control

    The upstream kineval-stencil's own kineval_servo_control.js
    (kineval.setpointDanceSequence / kineval.setpointClockMovement /
    kineval.robotArmControllerSetpoint) is itself a partial STENCIL --
    setpointClockMovement is complete upstream, but setpointDanceSequence
    and robotArmControllerSetpoint are both left as
    "STENCIL: implement ...". This file is this project's own completed
    reference implementation of those two, ported into this viewer's plain-
    function house style (explicit robot/params arguments, no "kineval = {}"
    namespace -- see controls.js's header for the same convention).

    All three functions here act on a shared params.setpoint_target map
    (jointName -> target angle, radians) and write through robot.joints[x]
    .control, the same one-frame-of-indirection every other control input in
    this viewer uses before controls.js#applyControls folds it into .angle.

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

// #region setpoint-clock
// drives every joint's setpoint target from the wall clock's seconds hand --
// complete upstream, ported unchanged other than the explicit robot/params
// arguments.
function setpointClockMovement(robot, params) {
    if (!params.update_pd_clock) return;

    if (!params.setpoint_target) params.setpoint_target = {};

    var curdate = new Date();
    var x;
    for (x in robot.joints) {
        params.setpoint_target[x] = curdate.getSeconds() / 60 * 2 * Math.PI;
    }
}
// #endregion setpoint-clock

// #region setpoint-dance
// small FSM cycling the whole robot through a handful of hardcoded poses
// (not specific to any one robot's joint names -- built once per robot from
// whatever joints it actually has, so this works the same way for mr2's
// 4-joint arm, urdf_example's 3 joints, or fetch/baxter/sawyer's much larger
// joint sets), advancing one state every params.dance_pose_duration_ms
// (default 1.5s).
function setpointDanceSequence(robot, params) {
    if (!params.update_pd_dance) return;

    if (!params.dance_poses) {
        // build the pose table once: a handful of small alternating-sign
        // sweeps across every joint, so the whole robot visibly cycles
        // through distinct hardcoded postures
        var names = Object.keys(robot.joints);
        var poseCount = 4;
        var poses = [];
        var p, i;
        for (p = 0; p < poseCount; p++) {
            var pose = {};
            for (i = 0; i < names.length; i++) {
                var sign = ((i + p) % 2 === 0) ? 1 : -1;
                pose[names[i]] = sign * 0.4 * (p / (poseCount - 1));
            }
            poses.push(pose);
        }
        params.dance_poses = poses;
        params.dance_pose_index = 0;
        params.dance_last_switch = Date.now();
    }

    var duration = params.dance_pose_duration_ms !== undefined ? params.dance_pose_duration_ms : 1500;
    if (Date.now() - params.dance_last_switch > duration) {
        params.dance_pose_index = (params.dance_pose_index + 1) % params.dance_poses.length;
        params.dance_last_switch = Date.now();
    }

    params.setpoint_target = params.dance_poses[params.dance_pose_index];
}
// #endregion setpoint-dance

// #region servo-controller
// P servo over every joint with a defined setpoint target: nudges each
// joint's .control toward closing the gap between params.setpoint_target[x]
// and the joint's current .angle, scaled by a proportional gain.
function robotArmControllerSetpoint(robot, params) {
    if (!params.update_pd && !params.persist_pd) return;

    params.update_pd = false; // clear one-shot request, then process setpoint control

    if (!params.setpoint_target) return;

    var gain = params.pd_gain !== undefined ? params.pd_gain : 0.5;
    var x;
    for (x in robot.joints) {
        if (params.setpoint_target[x] === undefined) continue;
        robot.joints[x].control += gain * (params.setpoint_target[x] - robot.joints[x].angle);
    }
}
// #endregion servo-controller
