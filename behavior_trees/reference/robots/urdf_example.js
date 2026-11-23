/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Behavior Trees | Robot description: urdf_example

    Ported from the upstream kineval-stencil (github.com/autorob/kineval-stencil)
    robots/robot_urdf_example.js, identical to
    ../../../inverse_kinematics/reference/robots/urdf_example.js -- same
    geometry-as-plain-data convention, same numbers. See
    robot_mr2.js's header for why plain data rather than THREE.BoxGeometry.

    Kept as the second robot because its kinematic tree BRANCHES:

        link1 -- joint1 --> link2
          |
          +--- joint2 --> link3 -- joint3 --> link4

    which is what makes it the useful one for this module's joint-ordering
    slides. joint1 drives a branch the endeffector is not on, so a
    JointTrajectory for this robot has a real choice about which joints to
    include -- and `joint_names` is the only thing that records the choice.

    JOINT TYPES. All three joints are declared `revolute` but carry no
    `limit` field, which upstream's own description also omits. The planner
    in ../../../kineval/motion_plan.js refuses to invent a range for a
    revolute joint with no limit (collision.js#jointLimitRange logs and
    returns null), so the plan generator that produced plans/*.json plans
    over mr2 -- whose joints are untyped and therefore continuous -- rather
    than over this robot. This robot is used for the tree and trajectory
    panels, where a plan is supplied rather than planned.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay
                     (original robot_urdf_example.js)

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// #region create-urdf-example-robot
function createUrdfExampleRobot() {
    return {
        name: 'urdf_example',

        origin: { xyz: [0, 0.1, 0], rpy: [0, 0, 0] },

        base: 'link1',

        links: {
            link1: { geometry: { type: 'box', size: [0.9, 0.7, 0.2], offset: { xyz: [0.15, 0.25, 0], rpy: [0, 0, 0] } }, color: 0x3b6ea5 },
            link2: { geometry: { type: 'box', size: [0.7, 0.2, 0.2], offset: { xyz: [0.25, 0, 0], rpy: [0, 0, 0] } }, color: 0xd98236 },
            link3: { geometry: { type: 'box', size: [0.7, 0.2, 0.2], offset: { xyz: [0.25, 0, 0], rpy: [0, 0, 0] } }, color: 0x27966b },
            link4: { geometry: { type: 'box', size: [0.5, 0.2, 0.2], offset: { xyz: [0.15, 0, 0], rpy: [0, 0, 0] } }, color: 0xc0392b }
        },

        joints: {
            joint1: {
                parent: 'link1', child: 'link2', type: 'revolute',
                origin: { xyz: [0.5, 0.3, 0.0], rpy: [0, 0, 0] },
                axis: [-1.0, 0.0, 0.0], angle: 0
            },
            joint2: {
                parent: 'link1', child: 'link3', type: 'revolute',
                origin: { xyz: [-0.2, 0.5, 0.0], rpy: [0, 0, Math.PI / 2] },
                axis: [-Math.cos(Math.PI / 4), Math.cos(Math.PI / 4), 0], angle: 0
            },
            joint3: {
                parent: 'link3', child: 'link4', type: 'revolute',
                origin: { xyz: [0.5, 0.0, 0.0], rpy: [0, 0, -Math.PI / 2] },
                axis: [Math.cos(Math.PI / 4), -Math.cos(Math.PI / 4), 0], angle: 0
            }
        },

        endeffector: { frame: 'joint3', position: [[0.5], [0], [0], [1]] }
    };
}
// #endregion create-urdf-example-robot

// The ordered base-to-endeffector chain for urdf_example: joint2 then
// joint3. joint1 is excluded for the reason this file's header gives --
// it drives link2, which is on a branch the endeffector is not on.
//
// A JointTrajectory that omits a joint does not hold it still. It says
// nothing about it, and the controller keeps doing whatever it was already
// doing with it. That distinction gets its own slide.
// #region urdf-example-arm-chain
var URDF_EXAMPLE_ARM_CHAIN = ['joint2', 'joint3'];
// #endregion urdf-example-arm-chain
