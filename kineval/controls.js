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
