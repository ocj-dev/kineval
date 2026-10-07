/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | Robot: robot_crawler

    Ported from the upstream kineval-stencil (github.com/autorob/kineval-stencil)
    robots/robot_crawler.js -- an 8-legged crawler, each leg an identical
    3-joint (hip/knee/ankle) chain off a shared base, making it this
    viewer's best exercise of a wide, uniformly-branching kinematic tree
    (robot_urdf_example's tree branches once; this one branches eight ways).
    See robot_urdf_example.js's header for what "ported" means here
    (factory-wrapped, CubeGeometry/applyMatrix modernized to
    BoxGeometry/translate). The 8 legs' hip origins/orientations and every
    link's box geometry are perfectly regular in the original source, so
    this port generates them from one per-leg table instead of writing out
    24 near-identical joint declarations by hand -- every resulting value is
    numerically identical to the original's explicit leg1..leg8 blocks.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay
                     (original robot_crawler.js)

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

function createRobotCrawler(THREE) {
    var robot = {};
    robot.name = 'crawler';
    robot.origin = { xyz: [0, 1, 0], rpy: [0, 0, 0] }; // held a bit over the ground plane
    robot.base = 'base';

    robot.links = { base: {} };
    robot.joints = {};

    // [hip_x, hip_z] per leg, matching upstream's explicit leg1..leg8 values;
    // hip_y (the Y_UP rotation sign) follows hip_x's own sign in every case.
    var legHipXZ = [
        [0.3, 0.9], [0.3, -0.9], [-0.3, 0.9], [-0.3, -0.9],
        [0.3, 0.3], [0.3, -0.3], [-0.3, 0.3], [-0.3, -0.3]
    ];

    var links_geom = {};
    links_geom['base'] = new THREE.BoxGeometry(1, 0.4, 2.3);

    var legIndex, legName, hipX, hipZ, hipYawSign;
    for (legIndex = 0; legIndex < legHipXZ.length; legIndex++) {
        legName = 'leg' + (legIndex + 1);
        hipX = legHipXZ[legIndex][0];
        hipZ = legHipXZ[legIndex][1];
        hipYawSign = hipX > 0 ? 1 : -1;

        robot.links[legName + '_upper'] = {};
        robot.links[legName + '_middle'] = {};
        robot.links[legName + '_lower'] = {};

        robot.joints[legName + '_hip'] = {
            parent: 'base', child: legName + '_upper',
            origin: { xyz: [hipX, 0.0, hipZ], rpy: [0, hipYawSign * Math.PI / 2, 0] },
            axis: [0.0, 1.0, 0.0]
        };
        robot.joints[legName + '_knee'] = {
            parent: legName + '_upper', child: legName + '_middle',
            origin: { xyz: [0.0, 0.0, 0.4], rpy: [-Math.PI / 4, 0, 0] },
            axis: [1.0, 0.0, 0.0]
        };
        robot.joints[legName + '_ankle'] = {
            parent: legName + '_middle', child: legName + '_lower',
            origin: { xyz: [0.0, 0.0, 0.6], rpy: [Math.PI / 2, 0, 0] },
            axis: [1.0, 0.0, 0.0]
        };

        links_geom[legName + '_upper'] = new THREE.BoxGeometry(0.3, 0.3, 0.3);
        links_geom[legName + '_upper'].translate(0, 0, 0.15);

        links_geom[legName + '_middle'] = new THREE.BoxGeometry(0.3, 0.3, 0.6);
        links_geom[legName + '_middle'].translate(0, 0, 0.3);

        links_geom[legName + '_lower'] = new THREE.BoxGeometry(0.3, 0.3, 1);
        links_geom[legName + '_lower'].translate(0, 0, 0.5);
    }

    robot.endeffector = {};
    robot.endeffector.frame = 'leg1_ankle';
    robot.endeffector.position = [[0], [0], [0.9], [1]];

    return { robot: robot, links_geom: links_geom };
}
