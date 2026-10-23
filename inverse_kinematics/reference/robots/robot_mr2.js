/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | Robot: robot_mr2

    Ported from the upstream kineval-stencil (github.com/autorob/kineval-stencil)
    robots/robot_mr2.js -- see robot_urdf_example.js's header for what
    "ported" means here (factory-wrapped, CubeGeometry/applyMatrix
    modernized to BoxGeometry/translate, every link/joint/origin/axis value
    otherwise unchanged).

    This is upstream's OWN right-arm-only humanoid stencil exercise (its
    header comment literally reads "for you to do" above the left arm's
    links/joints) -- kept exactly that incomplete here too, deliberately:
    the 6 links it does define (base + right clavicle/shoulder/upperarm/
    forearm + left clavicle) form a complete, fully-connected kinematic
    tree on their own, so forward kinematics runs correctly over them
    without needing to invent the missing left-arm joints upstream leaves
    as an exercise.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay
                     (original robot_mr2.js)

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

function createRobotMr2(THREE) {
    var robot = {};
    robot.name = 'mr2';
    robot.origin = { xyz: [0, 0, 0], rpy: [0, 0, 0] };
    robot.base = 'base';

    robot.links = {
        base: {},
        clavicle_right: {},
        clavicle_left: {},
        shoulder_right: {},
        upperarm_right: {},
        forearm_right: {}
    };

    robot.joints = {};

    robot.joints.clavicle_right_yaw = { parent: 'base', child: 'clavicle_right' };
    robot.joints.clavicle_right_yaw.origin = { xyz: [0.3, 0.4, 0.0], rpy: [-Math.PI / 2, 0, 0] };
    robot.joints.clavicle_right_yaw.axis = [0.0, 0.0, -1.0];

    robot.joints.shoulder_right_yaw = { parent: 'clavicle_right', child: 'shoulder_right' };
    robot.joints.shoulder_right_yaw.origin = { xyz: [0.0, -0.15, 0.85], rpy: [Math.PI / 2, 0, 0] };
    robot.joints.shoulder_right_yaw.axis = [0.0, 0.707, 0.707];

    robot.joints.upperarm_right_pitch = { parent: 'shoulder_right', child: 'upperarm_right' };
    robot.joints.upperarm_right_pitch.origin = { xyz: [0.0, 0.0, 0.7], rpy: [0, 0, 0] };
    robot.joints.upperarm_right_pitch.axis = [0.0, 1.0, 0.0];

    robot.joints.forearm_right_yaw = { parent: 'upperarm_right', child: 'forearm_right' };
    robot.joints.forearm_right_yaw.origin = { xyz: [0.0, 0.0, 0.7], rpy: [0, 0, 0] };
    robot.joints.forearm_right_yaw.axis = [1.0, 0.0, 0.0];

    robot.joints.clavicle_left_roll = { parent: 'base', child: 'clavicle_left' };
    robot.joints.clavicle_left_roll.origin = { xyz: [-0.3, 0.4, 0.0], rpy: [-Math.PI / 2, 0, 0] };
    robot.joints.clavicle_left_roll.axis = [0.0, 0.0, 1.0];

    robot.endeffector = {};
    robot.endeffector.frame = 'forearm_right_yaw';
    robot.endeffector.position = [[0], [0], [0.5], [1]];

    var links_geom = {};

    links_geom['base'] = new THREE.BoxGeometry(1, 0.4, 1);
    links_geom['base'].translate(0, 0.2, 0);

    links_geom['clavicle_right'] = new THREE.BoxGeometry(0.3, 0.3, 1);
    links_geom['clavicle_right'].translate(0, 0, 0.5);

    links_geom['clavicle_left'] = new THREE.BoxGeometry(0.3, 0.3, 1);
    links_geom['clavicle_left'].translate(0, 0, 0.5);

    links_geom['shoulder_right'] = new THREE.BoxGeometry(0.3, 0.3, 0.7);
    links_geom['shoulder_right'].translate(0, 0, 0.35);

    links_geom['upperarm_right'] = new THREE.BoxGeometry(0.3, 0.3, 0.7);
    links_geom['upperarm_right'].translate(0, 0, 0.35);

    links_geom['forearm_right'] = new THREE.BoxGeometry(0.3, 0.3, 0.5);
    links_geom['forearm_right'].translate(0, 0, 0.25);

    return { robot: robot, links_geom: links_geom };
}
