/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Forward Kinematics and Quaternions | Robot description: urdf_example

    Ported from the upstream kineval-stencil (github.com/autorob/kineval-stencil)
    robots/robot_urdf_example.js robot, onto this module's
    geometry-as-plain-data convention (see mobile_arm.urdf.js). Kept as a
    second test-case robot specifically because its kinematic tree BRANCHES
    -- link1 is the parent of two joints (joint1 -> link2, joint2 -> link3)
    -- which the simple single-chain mobile_arm robot can't exercise: it's
    what shows the DFS traversal's "for each child joint" loop in
    traverseFKLink actually iterating more than once.

        link1 -- joint1 --> link2
          |
          +--- joint2 --> link3 -- joint3 --> link4

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
            link1: {
                geometry: { type: 'box', size: [0.9, 0.7, 0.2], offset: { xyz: [0.15, 0.25, 0], rpy: [0, 0, 0] } },
                color: 0x3b6ea5
            },
            link2: {
                geometry: { type: 'box', size: [0.7, 0.2, 0.2], offset: { xyz: [0.25, 0, 0], rpy: [0, 0, 0] } },
                color: 0xd98236
            },
            link3: {
                geometry: { type: 'box', size: [0.7, 0.2, 0.2], offset: { xyz: [0.25, 0, 0], rpy: [0, 0, 0] } },
                color: 0x27966b
            },
            link4: {
                geometry: { type: 'box', size: [0.5, 0.2, 0.2], offset: { xyz: [0.15, 0, 0], rpy: [0, 0, 0] } },
                color: 0xc0392b
            }
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
