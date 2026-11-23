/*  SNAPSHOT -- DO NOT EDIT THIS COPY

    This file is a snapshot.
    Source path:    kineval/collision.js
    Snapshot date:  2026-11-22

    Edits belong in the source file above, and must then be
    re-snapshotted here. A `diff` of the two (ignoring this block) is the
    cheap check, and the Phase 4 verification harness runs it.

    WHY THE COPY EXISTS. Slidev's `<<<` snippet importer cannot read
    outside the deck's workspace root, and motion_planning/
    pnpm-workspace.yaml pins that root at motion_planning/. So
    motion_planning/slides/slides.md can reach ../reference/* and CANNOT
    reach ../../kineval/*. Every existing deck in this repository quotes
    only ../reference/*, and forward_kinematics/reference/ and
    inverse_kinematics/reference/ carry per-module copies of viewer code
    for exactly this reason.

    KNOWN COST: those earlier FK/IK copies have already drifted from
    kineval/. That is an accepted cost of the per-module reference/
    convention rather than a defect in it -- the alternative is a build
    step, and this repository deliberately has none. The diff is cheap;
    run it.
*/

/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | Collision Detection

    Completes the upstream kineval-stencil's kineval_collision.js: a
    configuration q is in collision if any link's bounding box overlaps any
    world obstacle sphere, or if the robot leaves the world boundary.

    Three things live here, in dependency order:

    1. The CONFIGURATION VECTOR. poseIsCollision(q) cannot be written
       without first fixing what q means, so the q_names/q_index mapping
       lives in this file rather than in motion_plan.js -- collision is its
       first consumer, and the planner imports the meaning rather than
       defining it. (The distance metric and the sampler do belong to the
       planner, and are there.)

    2. PURE FORWARD KINEMATICS. kinematics.js#robotForwardKinematics writes
       world transforms into robot.links[x].xform as a SIDE EFFECT
       (kinematics.js:117-119, `link.xform = xform`), so it cannot be asked
       "would this pose collide" without first destroying the pose the
       viewer is currently rendering. collisionForwardKinematics(robot, q)
       answers the same question by RETURNING a {linkName: xform} map and
       writing nothing. That contrast -- the same traversal, once
       side-effecting and once pure -- is the single best design
       observation this module produces, and it is why the planner can
       evaluate thousands of hypothetical poses per second without the
       robot on screen so much as twitching.

    3. THE COLLISION TEST ITSELF, in upstream's form exactly: transform the
       obstacle sphere's centre into the link's own frame, then compare it
       against the link's bounding box inflated by the sphere radius on
       every axis. A pure min/max comparison -- no square roots, no
       distance computation anywhere.

    WHY THE TEST IS DELIBERATELY WRONG AT THE CORNERS. Inflating a box by r
    on all three axes replaces the sphere with a box near the box's own
    corners, so this reports collisions that are not geometrically real.
    That is accepted on purpose. The collision lecture
    (autorob_17_collision_detection, slides 131-136) walks a sphere to a
    corner, annotates "True separating axis not tested", shows all four
    comparisons firing anyway, and closes with "umm... take comp geom". The
    false positive is taught, not overlooked.

    So: THE EXACT TEST -- clamp the centre to the box, compare squared
    distance to r^2 -- IS NOT IMPLEMENTED HERE, IN ANY FORM, NOT EVEN
    BEHIND A FLAG. It is deck material; the presentation plan builds it in
    a slide panel's own JavaScript to visualize where the two verdicts
    disagree. The Phase 2 tests assert the CONSERVATIVE verdict, including
    one test that asserts the corner false positive on purpose, so adding
    the exact test here as a second code path would break them. Please do
    not helpfully "finish" this.

    WHY THE LINK FRAME. Because the test runs in the link's own frame, a
    rotated link needs no re-axis-alignment of its box -- that is the whole
    point of inverting link.xform, and it is why a sphere (rotation
    invariant) is the thing that gets transformed rather than the box.
    Working in the world frame would force re-axis-aligning over the
    transformed corners, and would be looser still.

    DIVERGENCES FROM UPSTREAM, all recorded in IMPLEMENTATION_NOTES.md:
      - fixed joints are skipped in q (they are not degrees of freedom)
      - the boundary test uses each link's bounding box, not just the base
        origin as upstream's kineval_collision.js:53 does
      - matrix_invert_affine replaces upstream's numeric.inv
      - the visual-origin offset upstream computes and discards is applied
      - plain top-level functions with explicit arguments rather than
        upstream's "kineval = {}" namespace object, matching this project's
        house style (see servo_control.js's header for the same choice)

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

//////////////////////////////////////////////////
/////   THE CONFIGURATION VECTOR
//////////////////////////////////////////////////

// the first six entries of q are always the base's own 6 DOFs, in this
// order, matching upstream kineval_rrt_connect.js:99-106
var Q_BASE_X = 0, Q_BASE_Y = 1, Q_BASE_Z = 2;
var Q_BASE_ROLL = 3, Q_BASE_PITCH = 4, Q_BASE_YAW = 5;
var Q_BASE_DOF_COUNT = 6;

// #region build-configuration-index
// Derives the q-vector layout from whatever robot description it is handed.
// No robot is named anywhere in this file; adding a seventh robot to
// ROBOT_FACTORIES requires no change here.
//
//   q = [ base_x, base_y, base_z, base_roll, base_pitch, base_yaw,
//         ...one entry per non-fixed joint, in robot.joints order ]
//
// Returns the two mappings that are the single source of truth for that
// layout: names (jointName -> q index) and index (q index -> jointName).
//
// FIXED JOINTS ARE SKIPPED. Upstream includes them, giving every fixed
// joint a dimension whose value can never matter. That wastes sampler
// effort in proportion to how many there are -- fetch has five of twenty --
// and, worse, it tells a student that a welded-shut joint is part of the
// robot's configuration. It is not. This is a deliberate divergence.
function buildConfigurationIndex(robot) {
    var names = {};
    var index = [];
    var next = Q_BASE_DOF_COUNT;
    var x;

    for (x in robot.joints) {
        if (robot.joints[x].type === 'fixed') continue;
        names[x] = next;
        index[next] = x;
        next++;
    }

    return { names: names, index: index, length: next };
}
// #endregion build-configuration-index

// #region joint-limits
// The sampling interval for one joint, derived from its type and its
// limit field, with no per-robot special cases.
//
//   continuous / untyped -> sampled over [0, 2*PI); bounded: false,
//                            because it has no hard stops at all
//   revolute / prismatic -> the joint's own declared limit; bounded: true
//
// FAILS LOUDLY on a revolute or prismatic joint with no limit, naming the
// joint, and returns null. It never substitutes a plausible default at
// plan time: a silently-invented interval produces a planner that works
// for every joint but one and gives no clue which, which is how a student
// loses an afternoon. The robot files carry the limits (see each one's
// header for the source URDF).
function jointLimitRange(robot, jointName) {
    var joint = robot.joints[jointName];
    var type = joint.type;

    if (type === 'revolute' || type === 'prismatic') {
        if (!joint.limit || typeof joint.limit.lower !== 'number' || typeof joint.limit.upper !== 'number') {
            console.error(
                'kineval collision: joint "' + jointName + '" is ' + type +
                ' but declares no limit {lower, upper}. Add one to its robot file ' +
                '(cite the source URDF), or give it type continuous. Refusing to invent a range.'
            );
            return null;
        }
        return { lower: joint.limit.lower, upper: joint.limit.upper, bounded: true };
    }

    // continuous, or untyped (upstream treats an absent type as continuous,
    // and three of the six ported robots declare no types at all).
    //
    // `bounded: false` means the joint has no hard stops, so a
    // configuration outside [0, 2*PI) is not a limit violation -- it is a
    // joint that has been wound further round. [0, 2*PI) is the SAMPLING
    // interval, not a constraint. Angles are not wrapped anywhere in this
    // module; see motion_plan.js#configurationDifference.
    return { lower: 0, upper: 2 * Math.PI, bounded: false };
}
// #endregion joint-limits

// #region configuration-from-robot
// Reads the robot's current pose out as a q vector -- the planner's start
// configuration, and what robotIsCollision tests.
function configurationFromRobot(robot, qmap) {
    var q = new Array(qmap.length);

    q[Q_BASE_X] = robot.origin.xyz[0];
    q[Q_BASE_Y] = robot.origin.xyz[1];
    q[Q_BASE_Z] = robot.origin.xyz[2];
    q[Q_BASE_ROLL] = robot.origin.rpy[0];
    q[Q_BASE_PITCH] = robot.origin.rpy[1];
    q[Q_BASE_YAW] = robot.origin.rpy[2];

    var x;
    for (x in qmap.names) {
        q[qmap.names[x]] = robot.joints[x].angle;
    }

    return q;
}
// #endregion configuration-from-robot

// #region apply-configuration-to-robot
// The inverse: drives the robot to a configuration. Used by n/b traversal
// of a completed plan, so what the viewer renders is exactly
// motion_plan[index] and not an interpolation of it.
//
// Writes .angle directly rather than going through .control, because plan
// traversal is a teleport to a specific configuration, not a control
// input -- the servo would approach the configuration asymptotically and
// the rendered pose would never equal the path vertex under test.
function applyConfigurationToRobot(robot, qmap, q) {
    robot.origin.xyz = [q[Q_BASE_X], q[Q_BASE_Y], q[Q_BASE_Z]];
    robot.origin.rpy = [q[Q_BASE_ROLL], q[Q_BASE_PITCH], q[Q_BASE_YAW]];

    var x;
    for (x in qmap.names) {
        robot.joints[x].angle = q[qmap.names[x]];
        // keep the servo's integrator from immediately undoing the teleport
        robot.joints[x].control = 0;
    }
    robot.control = { xyz: [0, 0, 0], rpy: [0, 0, 0] };
}
// #endregion apply-configuration-to-robot

//////////////////////////////////////////////////
/////   PER-LINK BOUNDING BOXES
//////////////////////////////////////////////////

// #region cache-link-bounding-boxes
// Every link's axis-aligned bounding box IN ITS OWN FRAME, derived from the
// geometry rather than hand-entered.
//
// This is cheap and exactly correct because of a property of the robot
// files: links_geom[linkName] is a THREE.BufferGeometry that has already
// been .translate()-ed into the link frame (see robot_crawler.js's
// `links_geom[legName + '_upper'].translate(0, 0, 0.15)`), so its own
// bounding box IS the link-frame bounding box. No per-robot numbers, and
// the .translate() offsets come out right for free.
//
// Links with no geometry -- fetch, baxter and sawyer are ported without
// their meshes -- are handled by reading the mesh scene.js already built
// for them from synthesizeSkeletonGeometry. Reading back through
// link.geom.geometry rather than re-synthesizing guarantees the box
// matches what is on screen, which is what makes a collision highlight
// legible.
//
// Call once per loadRobot, after buildRobotMeshes. The cache is stored on
// the link objects, which are rebuilt by the robot factory on every robot
// switch, so it cannot go stale across one.
function cacheLinkBoundingBoxes(robot) {
    var x;
    for (x in robot.links) {
        var mesh = robot.links[x].geom;
        if (!mesh || !mesh.geometry) {
            console.error('kineval collision: link "' + x + '" has no geometry; ' +
                'call cacheLinkBoundingBoxes after buildRobotMeshes.');
            continue;
        }
        if (!mesh.geometry.boundingBox) mesh.geometry.computeBoundingBox();
        var bb = mesh.geometry.boundingBox;

        // copied out into plain numbers rather than held as a THREE.Box3:
        // the collision inner loop runs millions of times per plan and has
        // no other reason to touch three.js
        robot.links[x].bbox = {
            min: { x: bb.min.x, y: bb.min.y, z: bb.min.z },
            max: { x: bb.max.x, y: bb.max.y, z: bb.max.z }
        };
    }
}
// #endregion cache-link-bounding-boxes

// #region robot-extent
// The robot's largest bounding-box dimension across all links, in metres.
// The parametric world generators key their feature sizes off this -- a
// narrow passage is "two and a bit robot widths", not "0.8" -- so they
// stay meaningful when the robot changes.
function robotExtent(robot) {
    var largest = 0;
    var x;
    for (x in robot.links) {
        var bbox = robot.links[x].bbox;
        if (!bbox) continue;
        largest = Math.max(largest,
            bbox.max.x - bbox.min.x,
            bbox.max.y - bbox.min.y,
            bbox.max.z - bbox.min.z);
    }
    return largest > 0 ? largest : 1;
}
// #endregion robot-extent

// #region robot-reach
// How far the robot actually spans, in metres: the greatest distance from
// its base origin to any corner of any link's bounding box, at the pose
// described by q.
//
// This is NOT robotExtent. robotExtent is the widest single LINK, which
// is the right scale for "how wide must a gap be for this robot to fit
// through it". This is the whole ASSEMBLY, which is the right scale for
// "how big must an enclosure be for this robot to stand inside it". They
// differ by a lot on exactly the robots that matter: the crawler's base
// box is 2.3 long but its eight legs reach well past that, and sizing a
// bug trap by the base alone builds one the robot is already wedged into
// before planning starts.
//
// Found the hard way: the first version of world_sealed_enclosure sized
// its ring at 2.5 * robotExtent, and for several robots the start pose at
// the world origin was in collision with its own enclosure, so the
// termination test could not run at all.
function robotReach(robot, qmap, q) {
    var xforms = collisionForwardKinematics(robot, qmap, q);
    var furthest = 0;
    var x;
    for (x in robot.links) {
        var bbox = robot.links[x].bbox;
        var xform = xforms[x];
        if (!bbox || !xform) continue;

        var xs = [bbox.min.x, bbox.max.x];
        var ys = [bbox.min.y, bbox.max.y];
        var zs = [bbox.min.z, bbox.max.z];
        var i, j, k;
        for (i = 0; i < 2; i++) for (j = 0; j < 2; j++) for (k = 0; k < 2; k++) {
            var c = matrix_multiply(xform, [[xs[i]], [ys[j]], [zs[k]], [1]]);
            // measured in the ground plane from the base's own position,
            // since that is what an enclosure has to clear
            var dx = c[0][0] - q[Q_BASE_X];
            var dz = c[2][0] - q[Q_BASE_Z];
            furthest = Math.max(furthest, Math.sqrt(dx * dx + dz * dz));
        }
    }
    return furthest > 0 ? furthest : 1;
}
// #endregion robot-reach

// #region joint-lever-arms
// Per joint, the furthest any point downstream of that joint can EVER
// sit from the joint's own axis, in metres -- over every configuration,
// not just the current one.
//
// This is the lever arm a joint acts through. Rotating a joint by
// dtheta moves the points it carries by at most `lever * dtheta`, and
// moves nothing upstream of it at all. Edge validation needs it to bound
// workspace motion (motion_plan.js#workspaceMotionBound) without being
// absurdly pessimistic.
//
// POSE-INDEPENDENT, AND THAT IS THE WHOLE POINT. Two earlier versions
// were not, and both were wrong in instructive ways:
//
//   1. Using the whole robot's reach for every joint is a valid bound
//      and a uselessly loose one. The crawler has 24 joints, mostly
//      knees and ankles that swing a foot, and charging each with the
//      full body radius inflated the motion bound by more than an order
//      of magnitude -- an order of magnitude more collision checks per
//      edge, and one planning run went from 7 seconds to 55.
//
//   2. Measuring each subtree's actual radius AT THE START POSE is tight
//      and NOT A BOUND. A crawler leg folded at the start has a small
//      subtree radius; extended halfway along the path it is much
//      larger, so the validation spacing computed from it is too coarse
//      there and the edge tunnels. Verification caught this immediately:
//      a re-check at 300 samples per edge found 173 colliding
//      configurations on a path the planner had declared free.
//
// So it is computed from the kinematics alone -- link offsets, link
// bounding-box radii, and prismatic joints' declared travel -- which is
// the "fully extended" radius and is correct at every configuration.
// Rotation cannot change a distance from the axis it rotates about, so
// no joint angle appears anywhere below.
function jointLeverArms(robot, qmap) {
    var cache = {};

    // the furthest any point of `linkName`'s subtree can get from
    // `linkName`'s own frame origin, over all configurations
    function maxSubtreeRadius(linkName) {
        if (cache[linkName] !== undefined) return cache[linkName];
        cache[linkName] = 0; // guards against a malformed cyclic description

        var link = robot.links[linkName];
        var furthest = 0;

        // this link's own geometry, measured from its origin
        if (link.bbox) {
            var xs = [link.bbox.min.x, link.bbox.max.x];
            var ys = [link.bbox.min.y, link.bbox.max.y];
            var zs = [link.bbox.min.z, link.bbox.max.z];
            var i, j, k;
            for (i = 0; i < 2; i++) for (j = 0; j < 2; j++) for (k = 0; k < 2; k++) {
                furthest = Math.max(furthest,
                    Math.sqrt(xs[i] * xs[i] + ys[j] * ys[j] + zs[k] * zs[k]));
            }
        }

        // plus, for each child, how far that child's joint origin sits
        // from here and how far that child's own subtree can extend
        var children = link.children || [];
        var ci;
        for (ci = 0; ci < children.length; ci++) {
            var joint = robot.joints[children[ci]];
            var o = joint.origin.xyz;
            var offset = Math.sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2]);

            // a prismatic joint can also slide its whole subtree away
            var travel = 0;
            if (joint.type === 'prismatic' && joint.limit) {
                travel = Math.max(Math.abs(joint.limit.lower), Math.abs(joint.limit.upper));
            }

            furthest = Math.max(furthest,
                offset + travel + maxSubtreeRadius(joint.child));
        }

        cache[linkName] = furthest;
        return furthest;
    }

    var levers = {};
    var x;
    for (x in qmap.names) {
        // the joint's axis passes through its child link's frame origin,
        // so the subtree radius measured from there is the lever arm
        levers[x] = maxSubtreeRadius(robot.joints[x].child);
    }

    // and the whole robot, for base rotation, which swings everything
    levers.__base = maxSubtreeRadius(robot.base);
    return levers;
}
// #endregion joint-lever-arms

// #region joint-chains
// For each link, the joints between it and the base, and the pose-
// independent radius of the link's own geometry about its frame origin.
//
// This is what makes the edge-validation motion bound tight enough to
// be affordable. The obvious bound -- sum every joint's contribution --
// treats the robot as though one point were carried by all of them at
// once, which on the crawler means charging a single foot with the
// motion of all twenty-four leg joints. The real bound is per link:
// a point on leg 1 is moved only by leg 1's three joints and the base.
// Taking the maximum over links is correct and, for a wide robot,
// roughly (number of joints / chain depth) times smaller -- eight times,
// on the crawler, which is the difference between a 46-second planning
// run and a 6-second one.
function jointChains(robot, qmap) {
    var chains = {};
    var x;

    for (x in robot.links) {
        var chain = [];
        var linkName = x;
        var guard = 0;
        // walk parent joints back to the base
        while (linkName !== robot.base && guard++ < 1000) {
            var jointName = robot.links[linkName].parent;
            if (!jointName) break;
            // fixed joints have no q entry and contribute no motion
            if (qmap.names[jointName] !== undefined) chain.push(jointName);
            linkName = robot.joints[jointName].parent;
        }

        // the link's own geometry radius about its frame origin
        var radius = 0;
        var bbox = robot.links[x].bbox;
        if (bbox) {
            var xs = [bbox.min.x, bbox.max.x];
            var ys = [bbox.min.y, bbox.max.y];
            var zs = [bbox.min.z, bbox.max.z];
            var i, j, k;
            for (i = 0; i < 2; i++) for (j = 0; j < 2; j++) for (k = 0; k < 2; k++) {
                radius = Math.max(radius, Math.sqrt(xs[i] * xs[i] + ys[j] * ys[j] + zs[k] * zs[k]));
            }
        }

        chains[x] = { joints: chain, radius: radius };
    }

    return chains;
}
// #endregion joint-chains

// #region robot-height
// How tall the robot is, in metres: the greatest world-frame y of any
// corner of any link's bounding box, at the pose described by q.
//
// The parametric worlds need this because a wall one sphere high is not
// a wall. Obstacle collision is fully 3D, so a robot whose links all sit
// above a knee-high ring simply passes over it and the "sealed"
// enclosure is not sealed -- which is exactly what happened the first
// time world_sealed_enclosure was run against the taller robots, and
// what made the termination test report a path through a closed ring.
// The generators stack courses of spheres up to this height instead.
//
// This is why the ported world_s_higher.js has two courses at y = 0.5 and
// y = 1.5 with the comment "obstacles near boundaries to force arm
// upright": upstream hit the same thing and fixed it by hand for one
// robot. Deriving it means it stays fixed for all of them.
function robotHeight(robot, qmap, q) {
    var xforms = collisionForwardKinematics(robot, qmap, q);
    var tallest = 0;
    var x;
    for (x in robot.links) {
        var bbox = robot.links[x].bbox;
        var xform = xforms[x];
        if (!bbox || !xform) continue;

        var xs = [bbox.min.x, bbox.max.x];
        var ys = [bbox.min.y, bbox.max.y];
        var zs = [bbox.min.z, bbox.max.z];
        var i, j, k;
        for (i = 0; i < 2; i++) for (j = 0; j < 2; j++) for (k = 0; k < 2; k++) {
            var c = matrix_multiply(xform, [[xs[i]], [ys[j]], [zs[k]], [1]]);
            tallest = Math.max(tallest, c[1][0]);
        }
    }
    return tallest > 0 ? tallest : 1;
}
// #endregion robot-height

//////////////////////////////////////////////////
/////   PURE FORWARD KINEMATICS
//////////////////////////////////////////////////

// #region collision-joint-transform
// One joint's own variable transform for a given joint value. Mirrors
// kinematics.js#traverseFKJoint's branch exactly, by necessity: if these
// two ever disagree, the planner is checking collisions against a robot
// that is not the one on screen.
//
// Kept as a separate function rather than reusing traverseFKJoint because
// that one writes joint.xform (the side effect this whole file exists to
// avoid) and reads joint.angle rather than taking a value.
function collisionJointTransform(joint, value) {
    if (joint.type === 'prismatic') {
        var unitAxis = vector_normalize(joint.axis);
        return generate_translation_matrix(
            unitAxis[0] * value, unitAxis[1] * value, unitAxis[2] * value
        );
    }
    if (joint.type === 'fixed') {
        return generate_identity();
    }
    // revolute, continuous, or untyped -- all rotate about the joint axis
    return quaternion_to_rotation_matrix(
        quaternion_normalize(quaternion_from_axisangle(joint.axis, value))
    );
}
// #endregion collision-joint-transform

// #region collision-forward-kinematics
// Forward kinematics for a HYPOTHETICAL configuration q, returning
// {linkName: 4x4 world transform} and WRITING NOTHING.
//
// This is the counterpart to kinematics.js#robotForwardKinematics, which
// does the same traversal but assigns into robot.links[x].xform as it goes
// (kinematics.js:117-119). That side effect is correct for rendering -- the
// viewer wants exactly one current pose -- and fatal for planning, where
// the question is always about a configuration the robot is not in.
//
// kinematics.js is an already-taught deliverable of the forward-kinematics
// module and is deliberately not edited to accommodate this; the duplicated
// traversal is the cost of leaving it alone, and it is a small one.
//
// This is upstream's robot_collision_forward_kinematics with a cleaner
// signature (upstream threads a matrix stack through two mutually
// recursive functions and returns a collision verdict from the middle of
// the traversal; this returns transforms and lets the caller decide).
function collisionForwardKinematics(robot, qmap, q) {
    var xforms = {};

    // the base's own 6-DOF pose: translate to its position, then apply
    // roll-pitch-yaw in the same Z*Y*X order matrix_from_rpy uses
    var baseXform = matrix_multiply(
        generate_translation_matrix(q[Q_BASE_X], q[Q_BASE_Y], q[Q_BASE_Z]),
        matrix_from_rpy([q[Q_BASE_ROLL], q[Q_BASE_PITCH], q[Q_BASE_YAW]])
    );

    traverseCollisionLink(robot, qmap, q, robot.base, baseXform, xforms);
    return xforms;
}

// #region collision-traverse-link
// The link half of the mutual recursion: record this link's transform,
// then descend through each of its child joints.
function traverseCollisionLink(robot, qmap, q, linkName, xform, xforms) {
    var link = robot.links[linkName];
    xforms[linkName] = xform;

    var children = link.children || [];
    var i;
    for (i = 0; i < children.length; i++) {
        traverseCollisionJoint(robot, qmap, q, children[i], xform, xforms);
    }
}
// #endregion collision-traverse-link

// #region collision-traverse-joint
// The joint half: compose the joint's fixed origin offset with its
// variable motion for this configuration, then descend into its child
// link. A fixed joint has no q entry at all (buildConfigurationIndex
// skips it), so its value is never read -- it contributes only its
// origin.
function traverseCollisionJoint(robot, qmap, q, jointName, parentXform, xforms) {
    var joint = robot.joints[jointName];

    var jointOrigin = matrix_multiply(parentXform, matrix_from_origin(joint.origin));
    var value = (qmap.names[jointName] === undefined) ? 0 : q[qmap.names[jointName]];
    var jointXform = matrix_multiply(jointOrigin, collisionJointTransform(joint, value));

    traverseCollisionLink(robot, qmap, q, joint.child, jointXform, xforms);
}
// #endregion collision-traverse-joint
// #endregion collision-forward-kinematics

//////////////////////////////////////////////////
/////   THE COLLISION TEST
//////////////////////////////////////////////////

// #region link-sphere-collision
// Does one obstacle sphere overlap one link's bounding box?
//
// The sphere's centre arrives already transformed into the link's own
// frame, so the box is axis-aligned here no matter how the link is
// rotated in the world. The test is then four -- six, in 3D -- separating
// comparisons: if the sphere lies entirely beyond the box along ANY axis,
// there is no collision, and we can stop.
//
// Comparing `centre < min - radius` is the same thing as comparing the raw
// centre against a box grown by radius on that axis, which is where the
// name "inflated AABB" comes from. The collision lecture writes it the
// first way (slide 124, `loc_x + radius < x_min?`); upstream writes it the
// second (kineval_collision.js:171). They are the same test.
//
// See this file's header for why the corner false positive this produces
// is deliberate and must not be "fixed" here.
function linkSphereCollision(bbox, centre, radius) {
    // x: separated if the sphere is entirely left of, or entirely right of,
    // the box's extent along this axis
    if (centre[0] < bbox.min.x - radius || centre[0] > bbox.max.x + radius) return false;
    if (centre[1] < bbox.min.y - radius || centre[1] > bbox.max.y + radius) return false;
    if (centre[2] < bbox.min.z - radius || centre[2] > bbox.max.z + radius) return false;

    // no separating axis found among the three, so the inflated box
    // contains the centre: report collision
    return true;
}
// #endregion link-sphere-collision

// #region link-boundary-violation
// Is any corner of a link's bounding box outside the world boundary?
//
// THE BOUNDARY IS AN X-Z TEST ONLY. World boundaries are
// [[xmin,0,zmin],[xmax,0,zmax]]: a ground-plane extent with no ceiling and
// no floor, which is why a robot lifted straight up in +y never violates
// it. Height is governed by obstacles, which ARE tested in full 3D -- the
// "collision only checked in x-z plane" comment every upstream world file
// carries is true of this function and false of linkSphereCollision, and
// world_y.js exists to prove it. The ported worlds have that comment
// corrected.
//
// DIVERGENCE FROM UPSTREAM: kineval_collision.js:53 tests only the base
// ORIGIN -- a point -- so a robot can walk until its arm is well past the
// wall as long as its base origin is inside. Testing all eight corners of
// every link's box is what "the robot stays in the world" actually means,
// and it costs one transform per corner on a box we already have.
function linkBoundaryViolation(bbox, xform, boundary) {
    var xs = [bbox.min.x, bbox.max.x];
    var ys = [bbox.min.y, bbox.max.y];
    var zs = [bbox.min.z, bbox.max.z];

    var i, j, k;
    for (i = 0; i < 2; i++) {
        for (j = 0; j < 2; j++) {
            for (k = 0; k < 2; k++) {
                // the corner is in the link frame; push it out to the world
                var corner = matrix_multiply(xform, [[xs[i]], [ys[j]], [zs[k]], [1]]);
                if (corner[0][0] < boundary[0][0] || corner[0][0] > boundary[1][0]) return true;
                if (corner[2][0] < boundary[0][2] || corner[2][0] > boundary[1][2]) return true;
            }
        }
    }
    return false;
}
// #endregion link-boundary-violation

// #region pose-is-collision
// Is configuration q in collision with the world?
//
// Returns FALSE for no collision, or an object naming what collided:
//   { link: linkName, obstacle: index }   a link against an obstacle sphere
//   { link: linkName, boundary: true }    a link outside the world boundary
//
// Returning what collided rather than a bare boolean costs nothing (we
// already know it at the point of return), lets scene.js highlight the
// offending link in red, and turns "the planner never finds a path" from a
// mystery into a sentence. Upstream returns the link name for the same
// reason; this adds which obstacle.
//
// This is the function whose call count dominates planning time -- see
// motion_plan.js's counters and the Phase 0 findings' cost-model section --
// so the inner loop does no allocation beyond the transformed centre, and
// no square roots at all.
function poseIsCollision(robot, qmap, q, world) {
    if (!world) return false;

    var xforms = collisionForwardKinematics(robot, qmap, q);
    var obstacles = world.obstacles || [];
    var boundary = world.boundary;

    var x;
    for (x in robot.links) {
        var bbox = robot.links[x].bbox;
        if (!bbox) continue;
        var xform = xforms[x];
        // a link unreachable from the base has no transform this pose
        if (!xform) continue;

        if (boundary && linkBoundaryViolation(bbox, xform, boundary)) {
            return { link: x, boundary: true };
        }

        // one inverse per link, not per obstacle: the expensive step is
        // hoisted out of the obstacle loop. matrix_invert_affine exploits
        // the fact that this is a rigid transform (transpose the rotation,
        // negate the rotated translation) rather than running a general
        // 4x4 inversion -- upstream calls numeric.inv here, in this exact
        // innermost position.
        var inverse = matrix_invert_affine(xform);

        var j;
        for (j = 0; j < obstacles.length; j++) {
            var obstacle = obstacles[j];
            // the sphere's centre, carried from the world into this link's
            // own frame. The sphere needs no reorientation on the way --
            // it is rotation invariant -- which is precisely why we move
            // the sphere to the box and not the box to the sphere.
            var local = matrix_multiply(inverse, obstacle.location);
            if (linkSphereCollision(bbox, [local[0][0], local[1][0], local[2][0]], obstacle.radius)) {
                return { link: x, obstacle: j };
            }
        }
    }

    return false;
}
// #endregion pose-is-collision

// #region robot-is-collision
// The same test for the robot's CURRENT pose, called once per frame so the
// viewer can colour a colliding link. Stores the verdict on
// robot.collision, matching upstream's own kineval_collision.js:45.
function robotIsCollision(robot, qmap, world) {
    robot.collision = poseIsCollision(robot, qmap, configurationFromRobot(robot, qmap), world);
    return robot.collision;
}
// #endregion robot-is-collision
