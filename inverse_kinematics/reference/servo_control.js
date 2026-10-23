/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Inverse Kinematics | Arm Servo Control

    COMPLETED REFERENCE IMPLEMENTATION for the AutoRob (autorob.org) lab-session
    slides. Completes every "STENCIL" section of the upstream kineval-stencil
    (github.com/autorob/kineval-stencil) kineval/kineval_servo_control.js
    module: setpointClockMovement (already complete upstream, just ported),
    setpointDanceSequence, robotArmControllerSetpoint.

    Same no-global-namespace, explicit-argument house style as
    ../../forward_kinematics/reference/kinematics.js and this module's own
    inverse_kinematics.js (see that file's header): every "was an update
    requested" flag upstream reads off a global kineval.params.update_pd*
    field is an explicit boolean argument here instead, and every piece of
    mutable state upstream stashes on kineval.params (setpoint_target, the
    dance sequence's own FSM bookkeeping) is passed in by the caller
    (infrastructure.js) instead of reached for on a global.

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

// #region setpoint-clock-movement
function setpointClockMovement(robot, setpointTarget, enabled) {
    // ported straight from upstream's own (already-complete, non-STENCIL)
    // kineval.setpointClockMovement: every joint's setpoint sweeps through
    // one full revolution per minute, driven off the wall clock rather than
    // any model of robot dynamics -- `enabled` here stands in for upstream's
    // `if (!kineval.params.update_pd_clock) return;` early-out.
    if (!enabled) return;

    var curdate = new Date();
    var x;
    for (x in robot.joints) {
        setpointTarget[x] = curdate.getSeconds() / 60 * 2 * Math.PI;
    }
}
// #endregion setpoint-clock-movement

// #region setpoint-dance-sequence
function setpointDanceSequence(robot, setpointTarget, poses, state, enabled) {
    // if update not requested, exit routine (upstream:
    // kineval.params.update_pd_dance)
    if (!enabled) return;
    if (!poses || poses.length === 0) return;

    // small FSM cycling through an array of named poses (each pose =
    // {jointName: angle, ...}), advancing to the next pose every
    // TICKS_PER_POSE calls. state.poseIndex/state.ticksInPose are owned and
    // persisted by the CALLER across calls (infrastructure.js keeps one
    // {poseIndex: 0, ticksInPose: 0} object alive across animation frames,
    // the same role kineval.params would have played upstream) -- this
    // function only reads and advances them.
    var TICKS_PER_POSE = 90; // ~1.5s of calls at a 60Hz animation loop

    if (state.poseIndex === undefined) state.poseIndex = 0;
    if (state.ticksInPose === undefined) state.ticksInPose = 0;

    var pose = poses[state.poseIndex % poses.length];
    var jointName;
    for (jointName in pose) {
        setpointTarget[jointName] = pose[jointName];
    }

    state.ticksInPose++;
    if (state.ticksInPose >= TICKS_PER_POSE) {
        state.ticksInPose = 0;
        state.poseIndex = (state.poseIndex + 1) % poses.length;
    }
}
// #endregion setpoint-dance-sequence

// #region robot-arm-controller-setpoint
function robotArmControllerSetpoint(robot, setpointTarget, chainJointNames, gain) {
    // if update not requested, exit routine -- folded into the caller here
    // (infrastructure.js only calls this when servo control is the active
    // mode), matching upstream's update_pd/persist_pd gate in spirit without
    // a global kineval.params to read it off of.

    // straightforward P (proportional) servo: drive each chain joint's
    // .control toward closing the gap between its setpoint and its current
    // angle -- same .control-not-.angle accumulation convention as
    // iterateIK in inverse_kinematics.js (applyControls, in
    // ../../kineval/controls.js, is still the one place .control gets
    // folded into .angle, once per frame)
    var i, name;
    for (i = 0; i < chainJointNames.length; i++) {
        name = chainJointNames[i];
        robot.joints[name].control += gain * (setpointTarget[name] - robot.joints[name].angle);
    }
}
// #endregion robot-arm-controller-setpoint
