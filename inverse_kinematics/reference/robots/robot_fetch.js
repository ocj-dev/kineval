/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Inverse Kinematics | Robot description: fetch

    Ported from ../../kineval/robots/robot_fetch.js (itself ported from the
    upstream kineval-stencil (github.com/autorob/kineval-stencil)
    robots/fetch/fetch.urdf.js) -- every link name, joint, origin, axis and
    joint type unchanged. This is this module's third test-case robot and
    its only one with a PRISMATIC joint (torso_lift_joint, axis +z): every
    other robot in this directory (mobile_arm, urdf_example, mr2) is an
    all-revolute chain, which is exactly why fetch is here -- it's what
    exercises computeManipulatorJacobian's prismatic branch (see
    ../inverse_kinematics.js#manipulator-jacobian).

    Two differences from the full-viewer kineval/robots/robot_fetch.js this
    was ported from, both just this module's own established conventions
    (see robots/urdf_example.js and robots/mobile_arm.urdf.js) rather than
    anything fetch-specific:

      1. Geometry: the full viewer's robot_fetch.js gives every link an
         empty {} (no geometry at all) and leans on kineval/scene.js's
         synthesizeSkeletonGeometry to build a stand-in stick figure AT
         RUNTIME from each link's own joint offsets, importing THREE
         directly to do it. This module's scene.js has no such runtime
         synthesis (and, per its own header, is deliberately the only file
         that ever imports three.js) -- so the equivalent stick-figure boxes
         are precomputed HERE instead, once, as the same plain
         {type:'box', size, offset} descriptors urdf_example.js/
         mobile_arm.urdf.js already use: each link's box is the axis-aligned
         bounding box of its own child joint origins (padded to a 0.05m
         minimum per axis for leaf links / links whose children sit at or
         near its own origin), centered at its own midpoint. Not a faithful
         Fetch mesh (this repo doesn't vendor the real 20-35MB meshes --
         same reasoning as the full viewer's own header on this point), just
         a recognizable placeholder proportioned from the robot's real link
         offsets.

      2. robot.endeffector.position is left exactly as upstream/the full
         viewer have it, [[0.1],[0],[0],[1]] -- a point offset 0.1m along
         gripper_axis's own local +x (gripper_axis's own axis), which is
         also why gripper_axis itself (a `fixed`-type joint, appended at the
         tail of CHAIN_JOINT_NAMES.fetch in ../infrastructure.js -- see that
         table's own comment) contributes a harmless all-zero Jacobian
         column: cross(axis, k*axis) is always the zero vector for any
         scalar k, since computeManipulatorJacobian's revolute-shaped branch
         runs for any non-prismatic joint.type (fixed included), and here
         p_ee - p_gripper_axis is exactly k*axis for k=0.1.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay
                     (original robots/fetch/fetch.urdf.js)

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// #region create-fetch-robot
function createRobotFetch() {
    return {
        name: 'fetch',

        origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },

        base: 'base_link',

        links: {
            base_link: {
                geometry: { type: 'box', size: [0.360, 0.426, 0.377], offset: { xyz: [0.055, 0.026, 0.189], rpy: [0, 0, 0] } },
                color: 0x3b6ea5
            },
            r_wheel_link: {
                geometry: { type: 'box', size: [0.06, 0.06, 0.06], offset: { xyz: [0, 0, 0], rpy: [0, 0, 0] } },
                color: 0x2b2f36
            },
            l_wheel_link: {
                geometry: { type: 'box', size: [0.06, 0.06, 0.06], offset: { xyz: [0, 0, 0], rpy: [0, 0, 0] } },
                color: 0x2b2f36
            },
            torso_lift_link: {
                geometry: { type: 'box', size: [0.120, 0.050, 0.603], offset: { xyz: [0.060, 0, 0.302], rpy: [0, 0, 0] } },
                color: 0x27966b
            },
            head_pan_link: {
                geometry: { type: 'box', size: [0.143, 0.050, 0.058], offset: { xyz: [0.071, 0, 0.029], rpy: [0, 0, 0] } },
                color: 0x5a6b7a
            },
            head_tilt_link: {
                geometry: { type: 'box', size: [0.06, 0.06, 0.06], offset: { xyz: [0, 0, 0], rpy: [0, 0, 0] } },
                color: 0x5a6b7a
            },
            shoulder_pan_link: {
                geometry: { type: 'box', size: [0.117, 0.050, 0.060], offset: { xyz: [0.059, 0, 0.030], rpy: [0, 0, 0] } },
                color: 0xd98236
            },
            shoulder_lift_link: {
                geometry: { type: 'box', size: [0.219, 0.050, 0.050], offset: { xyz: [0.110, 0, 0], rpy: [0, 0, 0] } },
                color: 0xd98236
            },
            upperarm_roll_link: {
                geometry: { type: 'box', size: [0.133, 0.050, 0.050], offset: { xyz: [0.067, 0, 0], rpy: [0, 0, 0] } },
                color: 0xd9a236
            },
            elbow_flex_link: {
                geometry: { type: 'box', size: [0.197, 0.050, 0.050], offset: { xyz: [0.099, 0, 0], rpy: [0, 0, 0] } },
                color: 0xd9a236
            },
            forearm_roll_link: {
                geometry: { type: 'box', size: [0.124, 0.050, 0.050], offset: { xyz: [0.062, 0, 0], rpy: [0, 0, 0] } },
                color: 0xc0862b
            },
            wrist_flex_link: {
                geometry: { type: 'box', size: [0.139, 0.050, 0.050], offset: { xyz: [0.069, 0, 0], rpy: [0, 0, 0] } },
                color: 0xc0862b
            },
            wrist_roll_link: {
                geometry: { type: 'box', size: [0.166, 0.050, 0.050], offset: { xyz: [0.083, 0, 0], rpy: [0, 0, 0] } },
                color: 0xc0392b
            },
            gripper_link: {
                geometry: { type: 'box', size: [0.05, 0.05, 0.05], offset: { xyz: [0, 0, 0], rpy: [0, 0, 0] } },
                color: 0xc0392b
            },
            r_gripper_finger_link: {
                geometry: { type: 'box', size: [0.06, 0.06, 0.06], offset: { xyz: [0, 0, 0], rpy: [0, 0, 0] } },
                color: 0x8a8f98
            },
            l_gripper_finger_link: {
                geometry: { type: 'box', size: [0.06, 0.06, 0.06], offset: { xyz: [0, 0, 0], rpy: [0, 0, 0] } },
                color: 0x8a8f98
            },
            bellows_link: {
                geometry: { type: 'box', size: [0.06, 0.06, 0.06], offset: { xyz: [0, 0, 0], rpy: [0, 0, 0] } },
                color: 0x8a8f98
            },
            bellows_link2: {
                geometry: { type: 'box', size: [0.06, 0.06, 0.06], offset: { xyz: [0, 0, 0], rpy: [0, 0, 0] } },
                color: 0x8a8f98
            },
            estop_link: {
                geometry: { type: 'box', size: [0.06, 0.06, 0.06], offset: { xyz: [0, 0, 0], rpy: [0, 0, 0] } },
                color: 0x8a8f98
            },
            laser_link: {
                geometry: { type: 'box', size: [0.06, 0.06, 0.06], offset: { xyz: [0, 0, 0], rpy: [0, 0, 0] } },
                color: 0x8a8f98
            },
            torso_fixed_link: {
                geometry: { type: 'box', size: [0.06, 0.06, 0.06], offset: { xyz: [0, 0, 0], rpy: [0, 0, 0] } },
                color: 0x8a8f98
            }
        },

        joints: {
            torso_lift_joint: {
                parent: 'base_link', child: 'torso_lift_link', type: 'prismatic',
                origin: { xyz: [-0.086875, 0, 0.37743], rpy: [0, 0, 0] },
                axis: [0, 0, 1], angle: 0
            },
            r_wheel_joint: {
                parent: 'base_link', child: 'r_wheel_link', type: 'continuous',
                origin: { xyz: [0.001291, -0.18738, 0.055325], rpy: [0, 0, 0] },
                axis: [0, 1, 0], angle: 0
            },
            l_wheel_joint: {
                parent: 'base_link', child: 'l_wheel_link', type: 'continuous',
                origin: { xyz: [0.001291, 0.18738, 0.055325], rpy: [0, 0, 0] },
                axis: [0, 1, 0], angle: 0
            },
            shoulder_pan_joint: {
                parent: 'torso_lift_link', child: 'shoulder_pan_link', type: 'revolute',
                origin: { xyz: [0.119525, 0, 0.34858], rpy: [0, 0, 0] },
                axis: [0, 0, 1], angle: 0
            },
            shoulder_lift_joint: {
                parent: 'shoulder_pan_link', child: 'shoulder_lift_link', type: 'revolute',
                origin: { xyz: [0.117, 0, 0.06], rpy: [0, 0, 0] },
                axis: [0, 1, 0], angle: 0
            },
            upperarm_roll_joint: {
                parent: 'shoulder_lift_link', child: 'upperarm_roll_link', type: 'continuous',
                origin: { xyz: [0.219, 0, 0], rpy: [0, 0, 0] },
                axis: [1, 0, 0], angle: 0
            },
            elbow_flex_joint: {
                parent: 'upperarm_roll_link', child: 'elbow_flex_link', type: 'revolute',
                origin: { xyz: [0.133, 0, 0], rpy: [0, 0, 0] },
                axis: [0, 1, 0], angle: 0
            },
            forearm_roll_joint: {
                parent: 'elbow_flex_link', child: 'forearm_roll_link', type: 'continuous',
                origin: { xyz: [0.197, 0, 0], rpy: [0, 0, 0] },
                axis: [1, 0, 0], angle: 0
            },
            wrist_flex_joint: {
                parent: 'forearm_roll_link', child: 'wrist_flex_link', type: 'revolute',
                origin: { xyz: [0.1245, 0, 0], rpy: [0, 0, 0] },
                axis: [0, 1, 0], angle: 0
            },
            wrist_roll_joint: {
                parent: 'wrist_flex_link', child: 'wrist_roll_link', type: 'continuous',
                origin: { xyz: [0.1385, 0, 0], rpy: [0, 0, 0] },
                axis: [1, 0, 0], angle: 0
            },
            gripper_axis: {
                parent: 'wrist_roll_link', child: 'gripper_link', type: 'fixed',
                origin: { xyz: [0.16645, 0, 0], rpy: [0, 0, 0] },
                axis: [1, 0, 0], angle: 0
            },
            head_pan_joint: {
                parent: 'torso_lift_link', child: 'head_pan_link', type: 'revolute',
                origin: { xyz: [0.053125, 0, 0.603001], rpy: [0, 0, 0] },
                axis: [0, 0, 1], angle: 0
            },
            head_tilt_joint: {
                parent: 'head_pan_link', child: 'head_tilt_link', type: 'revolute',
                origin: { xyz: [0.14253, 0, 0.057999], rpy: [0, 0, 0] },
                axis: [0, 1, 0], angle: 0
            },
            r_gripper_finger_joint: {
                parent: 'gripper_link', child: 'r_gripper_finger_link', type: 'prismatic',
                origin: { xyz: [0, 0.015425, 0], rpy: [0, 0, 0] },
                axis: [0, 1, 0], angle: 0
            },
            l_gripper_finger_joint: {
                parent: 'gripper_link', child: 'l_gripper_finger_link', type: 'prismatic',
                origin: { xyz: [0, -0.015425, 0], rpy: [0, 0, 0] },
                axis: [0, -1, 0], angle: 0
            },
            bellows_joint: {
                parent: 'torso_lift_link', child: 'bellows_link', type: 'prismatic',
                origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
                axis: [0, 0, -1], angle: 0
            },
            bellows_joint2: {
                parent: 'torso_lift_link', child: 'bellows_link2', type: 'fixed',
                origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
                axis: [0, 0, 1], angle: 0
            },
            estop_joint: {
                parent: 'base_link', child: 'estop_link', type: 'fixed',
                origin: { xyz: [-0.12465, 0.23892, 0.31127], rpy: [1.5708, 0, 0] },
                axis: [0, 0, 0], angle: 0
            },
            laser_joint: {
                parent: 'base_link', child: 'laser_link', type: 'fixed',
                origin: { xyz: [0.235, 0, 0.2878], rpy: [3.141593, 0, 0] },
                axis: [0, 0, 0], angle: 0
            },
            torso_fixed_joint: {
                parent: 'base_link', child: 'torso_fixed_link', type: 'fixed',
                origin: { xyz: [-0.086875, 0, 0.377425], rpy: [0, 0, 0] },
                axis: [0, 1, 0], angle: 0
            }
        },

        // 0.1m out along gripper_axis's own local +x (gripper_axis's own
        // axis) -- see this file's header for why that makes gripper_axis's
        // own Jacobian column (it's included at the tail of
        // CHAIN_JOINT_NAMES.fetch) harmlessly zero.
        endeffector: { frame: 'gripper_axis', position: [[0.1], [0], [0], [1]] }
    };
}
// #endregion create-fetch-robot
