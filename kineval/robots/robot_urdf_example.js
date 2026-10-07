/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | Robot: robot_urdf_example

    Ported from the upstream kineval-stencil (github.com/autorob/kineval-stencil)
    robots/robot_urdf_example.js -- same links, joints, origins, axes, and
    threejs box geometry, unchanged, just wrapped in a factory function
    (upstream's version is a plain script relying on a global `THREE` and
    assigning bare globals `robot`/`links_geom`; this viewer loads three.js
    as an ES module, so those globals are passed in and returned instead)
    and CubeGeometry/applyMatrix modernized to BoxGeometry/translate (same
    upstream OrbitControls-vendoring precedent as every other reference
    implementation in this project already follows for its three.js calls).

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

function createRobotUrdfExample(THREE) {
    var robot = {};
    robot.name = 'urdf_example';
    robot.origin = { xyz: [0, 0.1, 0], rpy: [0, 0, 0] };
    robot.base = 'link1';

    robot.links = { link1: {}, link2: {}, link3: {}, link4: {} };

    robot.joints = {};

    robot.joints.joint1 = { parent: 'link1', child: 'link2' };
    robot.joints.joint1.origin = { xyz: [0.5, 0.3, 0.0], rpy: [0, 0, 0] };
    robot.joints.joint1.axis = [-1.0, 0.0, 0.0];

    robot.joints.joint2 = { parent: 'link1', child: 'link3' };
    robot.joints.joint2.origin = { xyz: [-0.2, 0.5, 0], rpy: [0, 0, Math.PI / 2] };
    robot.joints.joint2.axis = [-Math.cos(Math.PI / 4), Math.cos(Math.PI / 4), 0];

    robot.joints.joint3 = { parent: 'link3', child: 'link4' };
    robot.joints.joint3.origin = { xyz: [0.5, 0, 0], rpy: [0, 0, -Math.PI / 2] };
    robot.joints.joint3.axis = [Math.cos(Math.PI / 4), -Math.cos(Math.PI / 4), 0];

    robot.endeffector = {};
    robot.endeffector.frame = 'joint3';
    robot.endeffector.position = [[0.5], [0], [0], [1]];

    var links_geom = {};

    links_geom['link1'] = new THREE.BoxGeometry(0.7 + 0.2, 0.5 + 0.2, 0.2);
    links_geom['link1'].translate((0.5 - 0.2) / 2, 0.5 / 2, 0);

    links_geom['link2'] = new THREE.BoxGeometry(0.5 + 0.2, 0.2, 0.2);
    links_geom['link2'].translate(0.5 / 2, 0, 0);

    links_geom['link3'] = new THREE.BoxGeometry(0.5 + 0.2, 0.2, 0.2);
    links_geom['link3'].translate(0.5 / 2, 0, 0);

    links_geom['link4'] = new THREE.BoxGeometry(0.5 + 0.2, 0.2, 0.2);
    links_geom['link4'].translate(0.5 / 2, 0, 0);

    return { robot: robot, links_geom: links_geom };
}
