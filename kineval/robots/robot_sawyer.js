/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | Robot: sawyer

    Sawyer's kinematic tree, ported from the upstream kineval-stencil
    (github.com/autorob/kineval-stencil) robots/sawyer/sawyer.urdf.js -- every link name,
    joint, origin, axis and joint type unchanged.

    What is NOT ported is the geometry. Upstream pairs this description with
    20-35MB of STL/Collada meshes per robot, which is impractical to vendor
    into this teaching repo (see ../README.md). Links here carry no geometry,
    so scene.js synthesizes a placeholder skeleton from the joint offsets
    instead. Forward kinematics -- the whole point of this module -- is
    unaffected: it reads origins, axes and angles, never meshes.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay
                     (original robots/sawyer/sawyer.urdf.js)

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// #region create-sawyer-robot
function createRobotSawyer() {
    var robot = {};
    robot.name = 'sawyer';
    robot.origin = { xyz: [0, 0.1, 0], rpy: [0, 0, 0] };
    robot.base = 'pedestal';

    robot.links = {
        'pedestal': {},
        'right_arm_base_link': {},
        'right_l0': {},
        'head': {},
        'right_l1': {},
        'right_l2': {},
        'right_l3': {},
        'right_l4': {},
        'right_l5': {},
        'right_l6': {}
    };

    robot.joints = {};
    robot.joints['pedestal_fixed'] = {
        parent: 'pedestal', child: 'right_arm_base_link', type: 'fixed',
        origin: { xyz: [0, 0, 0.86488], rpy: [0, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['right_j0'] = {
        parent: 'right_arm_base_link', child: 'right_l0', type: 'revolute',
        origin: { xyz: [0, 0, 0.08], rpy: [0, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['head_pan'] = {
        parent: 'right_l0', child: 'head', type: 'revolute',
        origin: { xyz: [0, 0, 0.2965], rpy: [0, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['right_j1'] = {
        parent: 'right_l0', child: 'right_l1', type: 'revolute',
        origin: { xyz: [0.081, 0.05, 0.237], rpy: [-1.570796, 1.570796, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['right_j2'] = {
        parent: 'right_l1', child: 'right_l2', type: 'revolute',
        origin: { xyz: [0, -0.14, 0.1425], rpy: [1.570796, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['right_j3'] = {
        parent: 'right_l2', child: 'right_l3', type: 'revolute',
        origin: { xyz: [0, -0.042, 0.26], rpy: [-1.570796, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['right_j4'] = {
        parent: 'right_l3', child: 'right_l4', type: 'revolute',
        origin: { xyz: [0, -0.125, -0.1265], rpy: [1.570796, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['right_j5'] = {
        parent: 'right_l4', child: 'right_l5', type: 'revolute',
        origin: { xyz: [0, 0.031, 0.275], rpy: [1.570796, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['right_j6'] = {
        parent: 'right_l5', child: 'right_l6', type: 'revolute',
        origin: { xyz: [0, -0.11, 0.1053], rpy: [-1.570796, -0.17453, 3.1416] },
        axis: [0, 0, 1], angle: 0
    };

    robot.endeffector = {"frame":"right_j6","position":[[0.1],[0],[0],[1]]};

    return { robot: robot, links_geom: {} };
}
// #endregion create-sawyer-robot
