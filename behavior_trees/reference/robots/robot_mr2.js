/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Behavior Trees | Robot description: robot_mr2

    Ported from the upstream kineval-stencil (github.com/autorob/kineval-stencil)
    robots/robot_mr2.js, carrying the same link/joint/origin/axis numbers as
    ../../../kineval/robots/robot_mr2.js and
    ../../../inverse_kinematics/reference/robots/robot_mr2.js.

    GEOMETRY AS PLAIN DATA. The viewer's copy builds its link geometry with
    literal `new THREE.BoxGeometry(...).translate(...)` calls. This one
    expresses the identical boxes as {type:'box', size, offset} data, which
    is the convention forward_kinematics/ and inverse_kinematics/ already
    use for exactly this reason: four separate consumers have to agree on
    these numbers --

      - this module's own buildless demo page (reference/scene.js),
      - the deck's Vue panels (slides/lib/bt/robots.ts),
      - the Node plan generator that produced plans/*.json,
      - and collision.js, which normally reads each link's bounding box back
        off a built three.js mesh,

    and only the last of those has a three.js available. A box of `size`
    translated by `offset` has bounding box offset +/- size/2 exactly, so
    the plan generator can derive the same boxes cacheLinkBoundingBoxes
    would have computed without ever constructing a mesh.

    THE LEFT ARM IS DELIBERATELY MISSING. This is upstream's own
    right-arm-only humanoid stencil exercise -- its header reads "for you to
    do" above the left arm's links and joints -- and it is kept exactly that
    incomplete here. clavicle_left is a disconnected sibling branch off
    `base`: it is a real degree of freedom (the planner samples it, and this
    module's trajectories can command it) but it is not an ancestor of the
    endeffector frame.

    NO JOINT TYPES, SO EVERY JOINT IS CONTINUOUS. Upstream declares no
    `type` for any of these joints, and upstream's own rule is that an
    absent type means continuous, which is why no `limit` fields appear
    below -- a continuous joint has no limits by definition. That matters
    for the joint-trajectory slides: `trajectory_msgs/JointTrajectory`
    carries no limits either, so nothing downstream of the planner can
    check them. The controller is the only thing that ever enforces a
    limit, and this robot gives it none to enforce.

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

// #region create-robot-mr2
function createRobotMr2() {
    return {
        name: 'mr2',

        origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },

        base: 'base',

        links: {
            base: { geometry: { type: 'box', size: [1, 0.4, 1], offset: { xyz: [0, 0.2, 0], rpy: [0, 0, 0] } }, color: 0x3b6ea5 },
            clavicle_right: { geometry: { type: 'box', size: [0.3, 0.3, 1], offset: { xyz: [0, 0, 0.5], rpy: [0, 0, 0] } }, color: 0x27966b },
            clavicle_left: { geometry: { type: 'box', size: [0.3, 0.3, 1], offset: { xyz: [0, 0, 0.5], rpy: [0, 0, 0] } }, color: 0x8a8f98 },
            shoulder_right: { geometry: { type: 'box', size: [0.3, 0.3, 0.7], offset: { xyz: [0, 0, 0.35], rpy: [0, 0, 0] } }, color: 0xd98236 },
            upperarm_right: { geometry: { type: 'box', size: [0.3, 0.3, 0.7], offset: { xyz: [0, 0, 0.35], rpy: [0, 0, 0] } }, color: 0x27966b },
            forearm_right: { geometry: { type: 'box', size: [0.3, 0.3, 0.5], offset: { xyz: [0, 0, 0.25], rpy: [0, 0, 0] } }, color: 0xc0392b }
        },

        joints: {
            clavicle_right_yaw: {
                parent: 'base', child: 'clavicle_right',
                origin: { xyz: [0.3, 0.4, 0.0], rpy: [-Math.PI / 2, 0, 0] },
                axis: [0.0, 0.0, -1.0], angle: 0
            },
            shoulder_right_yaw: {
                parent: 'clavicle_right', child: 'shoulder_right',
                origin: { xyz: [0.0, -0.15, 0.85], rpy: [Math.PI / 2, 0, 0] },
                axis: [0.0, 0.707, 0.707], angle: 0
            },
            upperarm_right_pitch: {
                parent: 'shoulder_right', child: 'upperarm_right',
                origin: { xyz: [0.0, 0.0, 0.7], rpy: [0, 0, 0] },
                axis: [0.0, 1.0, 0.0], angle: 0
            },
            forearm_right_yaw: {
                parent: 'upperarm_right', child: 'forearm_right',
                origin: { xyz: [0.0, 0.0, 0.7], rpy: [0, 0, 0] },
                axis: [1.0, 0.0, 0.0], angle: 0
            },
            clavicle_left_roll: {
                parent: 'base', child: 'clavicle_left',
                origin: { xyz: [-0.3, 0.4, 0.0], rpy: [-Math.PI / 2, 0, 0] },
                axis: [0.0, 0.0, 1.0], angle: 0
            }
        },

        endeffector: { frame: 'forearm_right_yaw', position: [[0], [0], [0.5], [1]] }
    };
}
// #endregion create-robot-mr2

// The ordered base-to-endeffector chain of joints this module commands.
//
// This is the `joint_names` field of every JointTrajectory the module
// builds for mr2, and the order is load-bearing rather than cosmetic:
// a JointTrajectory's `points[i].positions[j]` means "the position of
// joint_names[j]", so the names and the numbers are only correct together.
// Reordering this array without reordering the points silently drives the
// wrong joint -- see joint_trajectory.js#trajectory-from-plan.
//
// clavicle_left_roll is excluded: it is not an ancestor of the
// endeffector frame, so no plan this module shows ever needs to move it.
// #region mr2-arm-chain
var MR2_ARM_CHAIN = [
    'clavicle_right_yaw',
    'shoulder_right_yaw',
    'upperarm_right_pitch',
    'forearm_right_yaw'
];
// #endregion mr2-arm-chain
