/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Forward Kinematics and Quaternions | Robot description: mobile_arm

    A new robot for this module: a single mobile-base link with a 2-joint
    planar arm mounted on top (3 links total -- base_link, arm_link1,
    arm_link2 -- matching the lab brief's "simple 3-link mobile robot with a
    single base link and a mounted planar arm"). Both arm joints rotate
    about the same (world "up", +y in kineval/three.js coordinates) axis, so
    the whole arm sweeps within one horizontal plane as it moves -- the
    "planar arm" part -- while the base link's own world pose (robot.origin)
    is free to translate/rotate anywhere in that same plane, the "mobile"
    part. See forward_kinematics.html's URL parameters for moving the base
    and driving each joint angle.

    This is KinEval's own JSON convenience representation of a URDF
    <visual> tree, NOT an XML parser input -- see the deck's "URDF <visual>
    <-> KinEval JSON" slide for the element-by-element correspondence, and
    robots/urdf_example.js for the second (branching) test-case robot this
    same representation describes.

    Each link's `geometry` field is plain data (a box's size and the fixed
    offset/orientation it sits at relative to its own link frame) -- scene.js
    is the only file that turns this into an actual three.js mesh, keeping
    this file (and the kinematic structure it describes) independent of the
    rendering engine, same as kinematics.js.

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

// #region create-mobile-arm-robot
function createMobileArmRobot() {
    return {
        name: 'mobile_arm',

        // world -> base_link; this is the robot's "mobile" freedom, driven
        // by the ?base_x=/?base_z=/?base_yaw= URL parameters
        origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },

        base: 'base_link',

        links: {
            base_link: {
                geometry: { type: 'box', size: [0.6, 0.2, 0.6], offset: { xyz: [0, 0, 0], rpy: [0, 0, 0] } },
                color: 0x3b6ea5
            },
            arm_link1: {
                // spans from joint1 (this link's own origin) to joint2, 0.5 along local +x
                geometry: { type: 'box', size: [0.5, 0.08, 0.08], offset: { xyz: [0.25, 0, 0], rpy: [0, 0, 0] } },
                color: 0xd98236
            },
            arm_link2: {
                // spans from joint2 to the end-effector tip, 0.4 along local +x
                geometry: { type: 'box', size: [0.4, 0.06, 0.06], offset: { xyz: [0.2, 0, 0], rpy: [0, 0, 0] } },
                color: 0xc0392b
            }
        },

        joints: {
            joint1: {
                parent: 'base_link', child: 'arm_link1', type: 'revolute',
                origin: { xyz: [0, 0.25, 0], rpy: [0, 0, 0] },
                axis: [0, 1, 0], angle: 0
            },
            joint2: {
                parent: 'arm_link1', child: 'arm_link2', type: 'revolute',
                origin: { xyz: [0.5, 0, 0], rpy: [0, 0, 0] },
                axis: [0, 1, 0], angle: 0
            }
        },

        // tool-tip point, expressed in the joint2/arm_link2 frame
        endeffector: { frame: 'joint2', position: [[0.4], [0], [0], [1]] }
    };
}
// #endregion create-mobile-arm-robot
