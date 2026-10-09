/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | Robot: fetch

    Fetch's kinematic tree, ported from the upstream kineval-stencil
    (github.com/autorob/kineval-stencil) robots/fetch/fetch.urdf.js -- every link name,
    joint, origin, axis and joint type unchanged.

    What is NOT ported is the geometry. Upstream pairs this description with
    20-35MB of STL/Collada meshes per robot, which is impractical to vendor
    into this teaching repo (see ../README.md). Links here carry no geometry,
    so scene.js synthesizes a placeholder skeleton from the joint offsets
    instead. Forward kinematics -- the whole point of this module -- is
    unaffected: it reads origins, axes and angles, never meshes.

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
    var robot = {};
    robot.name = 'fetch';
    robot.origin = { xyz: [0, 0.1, 0], rpy: [0, 0, 0] };
    robot.base = 'base_link';

    robot.links = {
        'base_link': {},
        'r_wheel_link': {},
        'l_wheel_link': {},
        'torso_lift_link': {},
        'head_pan_link': {},
        'head_tilt_link': {},
        'shoulder_pan_link': {},
        'shoulder_lift_link': {},
        'upperarm_roll_link': {},
        'elbow_flex_link': {},
        'forearm_roll_link': {},
        'wrist_flex_link': {},
        'wrist_roll_link': {},
        'gripper_link': {},
        'r_gripper_finger_link': {},
        'l_gripper_finger_link': {},
        'bellows_link': {},
        'bellows_link2': {},
        'estop_link': {},
        'laser_link': {},
        'torso_fixed_link': {}
    };

    robot.joints = {};
    robot.joints['torso_lift_joint'] = {
        parent: 'base_link', child: 'torso_lift_link', type: 'prismatic',
        origin: { xyz: [-0.086875, 0, 0.37743], rpy: [0, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['r_wheel_joint'] = {
        parent: 'base_link', child: 'r_wheel_link', type: 'continuous',
        origin: { xyz: [0.001291, -0.18738, 0.055325], rpy: [0, 0, 0] },
        axis: [0, 1, 0], angle: 0
    };
    robot.joints['l_wheel_joint'] = {
        parent: 'base_link', child: 'l_wheel_link', type: 'continuous',
        origin: { xyz: [0.001291, 0.18738, 0.055325], rpy: [0, 0, 0] },
        axis: [0, 1, 0], angle: 0
    };
    robot.joints['shoulder_pan_joint'] = {
        parent: 'torso_lift_link', child: 'shoulder_pan_link', type: 'revolute',
        origin: { xyz: [0.119525, 0, 0.34858], rpy: [0, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['shoulder_lift_joint'] = {
        parent: 'shoulder_pan_link', child: 'shoulder_lift_link', type: 'revolute',
        origin: { xyz: [0.117, 0, 0.06], rpy: [0, 0, 0] },
        axis: [0, 1, 0], angle: 0
    };
    robot.joints['upperarm_roll_joint'] = {
        parent: 'shoulder_lift_link', child: 'upperarm_roll_link', type: 'continuous',
        origin: { xyz: [0.219, 0, 0], rpy: [0, 0, 0] },
        axis: [1, 0, 0], angle: 0
    };
    robot.joints['elbow_flex_joint'] = {
        parent: 'upperarm_roll_link', child: 'elbow_flex_link', type: 'revolute',
        origin: { xyz: [0.133, 0, 0], rpy: [0, 0, 0] },
        axis: [0, 1, 0], angle: 0
    };
    robot.joints['forearm_roll_joint'] = {
        parent: 'elbow_flex_link', child: 'forearm_roll_link', type: 'continuous',
        origin: { xyz: [0.197, 0, 0], rpy: [0, 0, 0] },
        axis: [1, 0, 0], angle: 0
    };
    robot.joints['wrist_flex_joint'] = {
        parent: 'forearm_roll_link', child: 'wrist_flex_link', type: 'revolute',
        origin: { xyz: [0.1245, 0, 0], rpy: [0, 0, 0] },
        axis: [0, 1, 0], angle: 0
    };
    robot.joints['wrist_roll_joint'] = {
        parent: 'wrist_flex_link', child: 'wrist_roll_link', type: 'continuous',
        origin: { xyz: [0.1385, 0, 0], rpy: [0, 0, 0] },
        axis: [1, 0, 0], angle: 0
    };
    robot.joints['gripper_axis'] = {
        parent: 'wrist_roll_link', child: 'gripper_link', type: 'fixed',
        origin: { xyz: [0.16645, 0, 0], rpy: [0, 0, 0] },
        axis: [1, 0, 0], angle: 0
    };
    robot.joints['head_pan_joint'] = {
        parent: 'torso_lift_link', child: 'head_pan_link', type: 'revolute',
        origin: { xyz: [0.053125, 0, 0.603001], rpy: [0, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['head_tilt_joint'] = {
        parent: 'head_pan_link', child: 'head_tilt_link', type: 'revolute',
        origin: { xyz: [0.14253, 0, 0.057999], rpy: [0, 0, 0] },
        axis: [0, 1, 0], angle: 0
    };
    robot.joints['r_gripper_finger_joint'] = {
        parent: 'gripper_link', child: 'r_gripper_finger_link', type: 'prismatic',
        origin: { xyz: [0, 0.015425, 0], rpy: [0, 0, 0] },
        axis: [0, 1, 0], angle: 0
    };
    robot.joints['l_gripper_finger_joint'] = {
        parent: 'gripper_link', child: 'l_gripper_finger_link', type: 'prismatic',
        origin: { xyz: [0, -0.015425, 0], rpy: [0, 0, 0] },
        axis: [0, -1, 0], angle: 0
    };
    robot.joints['bellows_joint'] = {
        parent: 'torso_lift_link', child: 'bellows_link', type: 'prismatic',
        origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
        axis: [0, 0, -1], angle: 0
    };
    robot.joints['bellows_joint2'] = {
        parent: 'torso_lift_link', child: 'bellows_link2', type: 'fixed',
        origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['estop_joint'] = {
        parent: 'base_link', child: 'estop_link', type: 'fixed',
        origin: { xyz: [-0.12465, 0.23892, 0.31127], rpy: [1.5708, 0, 0] },
        axis: [0, 0, 0], angle: 0
    };
    robot.joints['laser_joint'] = {
        parent: 'base_link', child: 'laser_link', type: 'fixed',
        origin: { xyz: [0.235, 0, 0.2878], rpy: [3.141593, 0, 0] },
        axis: [0, 0, 0], angle: 0
    };
    robot.joints['torso_fixed_joint'] = {
        parent: 'base_link', child: 'torso_fixed_link', type: 'fixed',
        origin: { xyz: [-0.086875, 0, 0.377425], rpy: [0, 0, 0] },
        axis: [0, 1, 0], angle: 0
    };

    robot.endeffector = {"frame":"gripper_axis","position":[[0.1],[0],[0],[1]]};

    return { robot: robot, links_geom: {} };
}
// #endregion create-fetch-robot
