// One shared master pseudocode listing, reused with different active-line
// highlighting across every FK-traversal panel in this deck -- same pattern
// simulation's lib/pendulum/pseudocode.ts and pathfinding's
// lib/astar/aStarSteps.ts comment both establish. Mirrors kinematics.js's
// own buildFKTransforms/traverseFKBase/traverseFKLink/traverseFKJoint
// line-for-line.

export const MASTER_PSEUDOCODE: string[] = [
  /* 0 */ 'buildFKTransforms(robot):',
  /* 1 */ '    traverseFKBase(robot)   -- robot.origin -> world transform of the base link',
  /* 2 */ 'traverseFKLink(name, matrix):',
  /* 3 */ '    link.matrix = matrix   -- this link\'s fully-composed world transform',
  /* 4 */ '    for each child joint of this link:',
  /* 5 */ 'traverseFKJoint(jointName, parentMatrix):',
  /* 6 */ '    jointOrigin = parentMatrix * matrix_from_origin(joint.origin)   -- fixed offset',
  /* 7 */ '    jointMatrix = jointOrigin * quaternion_to_rotation_matrix(axisangle(joint.axis, joint.angle))',
  /* 8 */ '    traverseFKLink(joint.child, jointMatrix)   -- recurse into the child link',
]

export const LINE_BUILD = 0
export const LINE_BASE = 1
export const LINE_LINK_HEADER = 2
export const LINE_LINK_SET = 3
export const LINE_LINK_LOOP = 4
export const LINE_JOINT_HEADER = 5
export const LINE_JOINT_ORIGIN = 6
export const LINE_JOINT_MOTION = 7
export const LINE_JOINT_RECURSE = 8
