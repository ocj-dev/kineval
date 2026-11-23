/*  SNAPSHOT -- DO NOT EDIT THIS COPY

    This file is a snapshot.
    Source path:    kineval/controls.js
    Snapshot date:  2026-11-23
    Source commit:  a5d61ec

    Edits belong in the source file above, and must then be
    re-snapshotted here. `diff` of the two, ignoring this block, is the
    cheap check; tests/smoke.cjs runs it.

    WHY THE COPY EXISTS. Slidev's `<<<` snippet importer cannot read
    outside the deck's workspace root, and behavior_trees/
    pnpm-workspace.yaml pins that root at behavior_trees/. So
    behavior_trees/slides/slides.md can reach ../reference/* and cannot
    reach ../../kineval/*. forward_kinematics/reference/,
    inverse_kinematics/reference/ and motion_planning/reference/ all carry
    per-module copies of viewer code for the same reason.

    WHAT THIS MODULE USES IT FOR. The buildless demo page needs forward
    kinematics to draw the robot, and the joint-trajectory action needs
    applyControls to fold its servo output into joint angles -- which is
    the whole reason its feedback has a nonzero `error` field. Nothing in
    this file is modified for this module.
*/

/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | Update Robot State From Controls

    Ported from the upstream kineval-stencil's own kineval_controls.js
    (kineval.applyControls): every joint's accumulated per-frame .control
    delta is added onto its .angle (then zeroed for the next frame), and
    the robot base's accumulated .control.xyz/.rpy delta is added onto
    .origin.xyz/.rpy (then zeroed) -- separating "what userinput.js wants to
    happen this frame" from "the robot's actual configuration" exactly the
    way upstream does, one frame of indirection before forward kinematics
    ever runs.

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
