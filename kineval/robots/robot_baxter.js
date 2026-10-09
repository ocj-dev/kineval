/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | Robot: baxter

    Baxter's kinematic tree, ported from the upstream kineval-stencil
    (github.com/autorob/kineval-stencil) robots/baxter/baxter.urdf.js -- every link name,
    joint, origin, axis and joint type unchanged.

    What is NOT ported is the geometry. Upstream pairs this description with
    20-35MB of STL/Collada meshes per robot, which is impractical to vendor
    into this teaching repo (see ../README.md). Links here carry no geometry,
    so scene.js synthesizes a placeholder skeleton from the joint offsets
    instead. Forward kinematics -- the whole point of this module -- is
    unaffected: it reads origins, axes and angles, never meshes.

    NOTE -- upstream data defect: baxter.urdf.js declares TWO fixed joints,
    'headnod' and 'display_joint', both joining head -> screen with different
    origins. That makes the description a DAG rather than a tree, so a
    depth-first traversal would reach 'screen' twice and the link's final pose
    would depend on visit order. 'display_joint' is dropped here, keeping
    'headnod' (declared first), so the tree is well formed and the DFS is
    well defined.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay
                     (original robots/baxter/baxter.urdf.js)

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// #region create-baxter-robot
function createRobotBaxter() {
    var robot = {};
    robot.name = 'baxter';
    robot.origin = { xyz: [0, 0.1, 0], rpy: [0, 0, 0] };
    robot.base = 'pedestal';

    robot.links = {
        'torso': {},
        'pedestal': {},
        'head': {},
        'screen': {},
        'right_arm_mount': {},
        'right_upper_shoulder': {},
        'right_lower_shoulder': {},
        'right_upper_elbow': {},
        'right_lower_elbow': {},
        'right_upper_forearm': {},
        'right_lower_forearm': {},
        'right_wrist': {},
        'left_arm_mount': {},
        'left_upper_shoulder': {},
        'left_lower_shoulder': {},
        'left_upper_elbow': {},
        'left_lower_elbow': {},
        'left_upper_forearm': {},
        'left_lower_forearm': {},
        'left_wrist': {}
    };

    robot.joints = {};
    robot.joints['torso_t0'] = {
        parent: 'pedestal', child: 'torso', type: 'fixed',
        origin: { xyz: [0, 0, 0.86488], rpy: [0, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['headpan'] = {
        parent: 'torso', child: 'head', type: 'revolute',
        origin: { xyz: [0.06, 0, 0.686], rpy: [0, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['headnod'] = {
        parent: 'head', child: 'screen', type: 'fixed',
        origin: { xyz: [0.1227, 0, 0], rpy: [1.75057, 0, 1.570796] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['right_torso_arm_mount'] = {
        parent: 'torso', child: 'right_arm_mount', type: 'fixed',
        origin: { xyz: [0.024645, -0.219645, 0.118588], rpy: [0, 0, -0.7854] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['right_s0'] = {
        parent: 'right_arm_mount', child: 'right_upper_shoulder', type: 'revolute',
        origin: { xyz: [0.055695, 0, 0.011038], rpy: [0, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['right_s1'] = {
        parent: 'right_upper_shoulder', child: 'right_lower_shoulder', type: 'revolute',
        origin: { xyz: [0.069, 0, 0.27035], rpy: [-1.570796, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['right_e0'] = {
        parent: 'right_lower_shoulder', child: 'right_upper_elbow', type: 'revolute',
        origin: { xyz: [0.102, 0, 0], rpy: [1.570796, 0, 1.570796] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['right_e1'] = {
        parent: 'right_upper_elbow', child: 'right_lower_elbow', type: 'revolute',
        origin: { xyz: [0.069, 0, 0.26242], rpy: [-1.570796, -1.570796, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['right_w0'] = {
        parent: 'right_lower_elbow', child: 'right_upper_forearm', type: 'revolute',
        origin: { xyz: [0.10359, 0, 0], rpy: [1.570796, 0, 1.570796] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['right_w1'] = {
        parent: 'right_upper_forearm', child: 'right_lower_forearm', type: 'revolute',
        origin: { xyz: [0.01, 0, 0.2707], rpy: [-1.570796, -1.570796, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['right_w2'] = {
        parent: 'right_lower_forearm', child: 'right_wrist', type: 'revolute',
        origin: { xyz: [0.115975, 0, 0], rpy: [1.570796, 0, 1.570796] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['left_torso_arm_mount'] = {
        parent: 'torso', child: 'left_arm_mount', type: 'fixed',
        origin: { xyz: [0.024645, 0.219645, 0.118588], rpy: [0, 0, 0.7854] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['left_s0'] = {
        parent: 'left_arm_mount', child: 'left_upper_shoulder', type: 'revolute',
        origin: { xyz: [0.055695, 0, 0.011038], rpy: [0, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['left_s1'] = {
        parent: 'left_upper_shoulder', child: 'left_lower_shoulder', type: 'revolute',
        origin: { xyz: [0.069, 0, 0.27035], rpy: [-1.570796, 0, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['left_e0'] = {
        parent: 'left_lower_shoulder', child: 'left_upper_elbow', type: 'revolute',
        origin: { xyz: [0.102, 0, 0], rpy: [1.570796, 0, 1.570796] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['left_e1'] = {
        parent: 'left_upper_elbow', child: 'left_lower_elbow', type: 'revolute',
        origin: { xyz: [0.069, 0, 0.26242], rpy: [-1.570796, -1.570796, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['left_w0'] = {
        parent: 'left_lower_elbow', child: 'left_upper_forearm', type: 'revolute',
        origin: { xyz: [0.10359, 0, 0], rpy: [1.570796, 0, 1.570796] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['left_w1'] = {
        parent: 'left_upper_forearm', child: 'left_lower_forearm', type: 'revolute',
        origin: { xyz: [0.01, 0, 0.2707], rpy: [-1.570796, -1.570796, 0] },
        axis: [0, 0, 1], angle: 0
    };
    robot.joints['left_w2'] = {
        parent: 'left_lower_forearm', child: 'left_wrist', type: 'revolute',
        origin: { xyz: [0.115975, 0, 0], rpy: [1.570796, 0, 1.570796] },
        axis: [0, 0, 1], angle: 0
    };

    robot.endeffector = {"frame":"right_w2","position":[[0.1],[0],[0],[1]]};

    return { robot: robot, links_geom: {} };
}
// #endregion create-baxter-robot
