/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Behavior Trees | A motion plan as a Joint Trajectory

    Turns the output of the RRT-Connect planner taught in the
    motion_planning module -- a list of configurations -- into the message
    a joint-trajectory controller will actually accept.

    THE SHAPE BEING MIRRORED is ROS's trajectory_msgs/JointTrajectory:

        std_msgs/Header            header
        string[]                   joint_names
        JointTrajectoryPoint[]     points

    and each trajectory_msgs/JointTrajectoryPoint:

        float64[]                    positions
        float64[]                    velocities
        float64[]                    accelerations
        float64[]                    effort
        builtin_interfaces/Duration  time_from_start

    checked against the definitions in ros2/common_interfaces rather than
    from memory.

    The field names here are the ROS field names, in snake_case, which is
    deliberately out of step with the rest of this JavaScript. A student
    who reads `point.time_from_start` here and then reads a real
    JointTrajectory should see the same words. (This module does not
    depend on ROS, run ROS, or claim to be ROS. It implements the same
    data structure so that the structure is what is being taught.)

    TWO PLACES WHERE THIS IS A SIMPLIFICATION, both stated rather than
    glossed:

      - `time_from_start` is a builtin_interfaces/Duration -- an integer
        count of seconds plus an integer count of nanoseconds -- and here
        it is a JavaScript number of seconds. Nothing in this module
        depends on the difference, but a real trajectory's times are
        exact rationals and a float's are not.
      - the real JointTrajectoryPoint documents the units: radians and
        radians/second for revolute and continuous joints, metres and
        metres/second for prismatic ones, in the same order as
        joint_names. Every joint this module commands is continuous, so
        everything below is in radians.

    THE THREE THINGS A PLAN IS MISSING, which is the whole reason this
    file exists:

      1. TIME. A plan is a sequence of configurations with no statement
         about when the robot should be at each one. A trajectory is a
         function of time. Choosing that function is TIME
         PARAMETERIZATION, and it is a choice -- see
         #time-parameterization for the one made here and what it costs.

      2. NAMES. A plan is an array indexed by the planner's own q layout
         (base_x, base_y, ..., then one entry per non-fixed joint, in
         robot.joints order). A trajectory carries `joint_names`, and
         `points[i].positions[j]` means "the position of joint_names[j]".
         The numbers alone are meaningless; a reordering that loses the
         names silently drives the wrong joint.

      3. A CONTRACT ABOUT WHAT IS NOT MENTIONED. A JointTrajectory says
         nothing about joints it does not name, which is NOT the same as
         commanding them to hold still. See #trajectory-joints-not-named.

    AND ONE THING A JointTrajectory GENUINELY CANNOT CARRY: a mobile
    base. Its `positions` are single-DOF joint values -- the message's own
    comment says so -- and KinEval's q vector begins with six base DOFs
    that are not joints at all. ROS's answer is a SECOND message type:
    control_msgs/FollowJointTrajectory's goal has both

        trajectory_msgs/JointTrajectory          trajectory
        trajectory_msgs/MultiDOFJointTrajectory  multi_dof_trajectory

    the second being, in its own words, "the trajectory for all planar or
    floating joints (i.e. individual joints with more than one DOF)".
    So the base is not unrepresentable, it is representable in a
    different field of a different type, carried alongside. This module
    implements only the single-DOF half and REFUSES a plan whose base
    moves (see #trajectory-from-plan) rather than quietly dropping six
    columns -- and the refusal is what motivates sequencing a base
    motion and an arm motion as two steps, which is a behavior tree's
    job.

    WHAT THIS FILE DOES NOT DO: it does not check collision. The plan it
    is handed was validated by the planner against a world; a trajectory
    built from it inherits that validation ONLY at the waypoints and only
    if the interpolation between them is the same straight line in
    configuration space that the planner validated. sampleTrajectory's
    linear interpolation is that same straight line, which is why it is
    linear -- a smoother cubic through the waypoints bulges off the
    validated edge, and nothing downstream would notice. That is a real
    trade and it gets its own slide.

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

// Cruise speed, radians per second, used by the default time
// parameterization. A real controller would read this off the robot's
// URDF joint limits (<limit velocity="..."/>); mr2's description declares
// none at all -- see robots/robot_mr2.js -- so the number has to come from
// somewhere, and a named constant that says so is better than one
// buried in a formula.
var TRAJECTORY_MAX_JOINT_VELOCITY = 0.6;

// Floor on a segment's duration, seconds. Two waypoints that differ by
// almost nothing would otherwise get a duration of almost zero, and
// `(t - t0) / (t1 - t0)` in sampleTrajectory would divide by it.
var TRAJECTORY_MIN_SEGMENT_SECONDS = 0.05;

//////////////////////////////////////////////////
/////   BUILDING ONE
//////////////////////////////////////////////////

// Project a planner path onto the joints a trajectory will name.
//
// `path` is an array of full q vectors as the planner produced them, and
// `qLayout` is the array of names for q's columns -- exactly the
// `q_layout` field the committed plan JSON carries, which came from
// buildConfigurationIndex. Returns one positions array per configuration,
// in jointNames order.
//
// FAILS LOUDLY on a joint name that is not in the layout. The alternative
// -- a quietly-undefined column -- becomes NaN in the first interpolation
// and a motionless robot three files later.
// #region plan-joint-columns
function planJointColumns(path, qLayout, jointNames) {
    var column = {};
    var i, j;
    for (i = 0; i < qLayout.length; i++) column[qLayout[i]] = i;

    for (j = 0; j < jointNames.length; j++) {
        if (column[jointNames[j]] === undefined) {
            console.error('joint_trajectory: joint "' + jointNames[j] +
                '" is not in the plan\'s q layout [' + qLayout.join(', ') + ']');
            return null;
        }
    }

    var positions = [];
    for (i = 0; i < path.length; i++) {
        var row = [];
        for (j = 0; j < jointNames.length; j++) {
            row.push(path[i][column[jointNames[j]]]);
        }
        positions.push(row);
    }
    return positions;
}
// #endregion plan-joint-columns

// The part of the plan a JointTrajectory cannot carry: the base's six
// DOFs, per configuration.
//
// This function exists to be looked at rather than used. If it returns
// anything that varies, the plan is not expressible as a single
// JointTrajectory: the motion has to be split into a base motion -- a
// navigation goal, or the goal's own multi_dof_trajectory field, which
// is a MultiDOFJointTrajectory and a different message type -- and an
// arm motion, and something has to sequence the two. Sequencing exactly
// that is what a behavior tree is for.
//
// The committed plans in plans/ were produced with plan_base_dofs:'none',
// so this returns a constant for them, and that is not luck: it is the
// condition under which the rest of this file is honest.
// #region plan-base-columns
function planBaseColumns(path, qLayout) {
    var baseNames = ['base_x', 'base_y', 'base_z', 'base_roll', 'base_pitch', 'base_yaw'];
    var i;
    for (i = 0; i < baseNames.length; i++) {
        if (qLayout.indexOf(baseNames[i]) === -1) return null;   // no base in this layout
    }
    return planJointColumns(path, qLayout, baseNames);
}
// #endregion plan-base-columns

// Does the plan hold the base still, to within a tolerance?
//
// Asked before a trajectory is built, so a plan whose base moves is
// rejected at the point where the mistake was made, rather than
// producing a trajectory that silently describes only part of the
// intended motion.
//
// A layout with no base columns at all -- a path given directly as joint
// positions, which is what the deck's hand-built two-waypoint hops are --
// vacuously holds the base still, and says so without complaining.
// #region base-is-stationary
function baseIsStationary(path, qLayout, tolerance) {
    var tol = tolerance === undefined ? 1e-9 : tolerance;
    var base = planBaseColumns(path, qLayout);
    if (!base || base.length === 0) return true;

    var i, j;
    for (i = 1; i < base.length; i++) {
        for (j = 0; j < base[i].length; j++) {
            if (Math.abs(base[i][j] - base[0][j]) > tol) return false;
        }
    }
    return true;
}
// #endregion base-is-stationary

// Assign a time to each waypoint: CONSTANT-SPEED, LONGEST-JOINT-WINS.
//
// For each segment, the duration is the largest per-joint displacement in
// that segment divided by the cruise speed:
//
//     dt_i = max_j |q_i+1,j - q_i,j| / v_max
//
// so every joint in a segment starts and stops together and the joint
// that has furthest to travel sets the pace. This is the simplest
// parameterization that is dimensionally sane, and it is what makes a
// trajectory's duration a property of the PLAN rather than of a guess.
//
// WHAT IT COSTS, stated rather than hidden:
//
//   - the velocity profile is a square wave. Each segment runs at
//     constant speed and the speed changes discontinuously at every
//     waypoint, which implies infinite acceleration. A real controller
//     will not do that; it will lag, and the lag is visible in the
//     execution panel's error plot. Trapezoidal or spline
//     parameterization (ROS's own time_optimal_trajectory_generation,
//     Reflexxes, Ruckig) exists precisely to fix this.
//   - it ignores joint velocity limits per joint, using one global cruise
//     speed, because mr2's description has no per-joint limits to read.
//   - it says nothing about acceleration limits, which is the constraint
//     that actually binds on real hardware.
//
// Returns cumulative times, one per waypoint, starting at 0.
// #region time-parameterization
function trajectoryTimesFromPositions(positions, maxVelocity) {
    var vmax = maxVelocity === undefined ? TRAJECTORY_MAX_JOINT_VELOCITY : maxVelocity;
    var times = [0];
    var i, j;

    for (i = 1; i < positions.length; i++) {
        var largest = 0;
        for (j = 0; j < positions[i].length; j++) {
            largest = Math.max(largest, Math.abs(positions[i][j] - positions[i - 1][j]));
        }
        var dt = Math.max(largest / vmax, TRAJECTORY_MIN_SEGMENT_SECONDS);
        times.push(times[i - 1] + dt);
    }
    return times;
}
// #endregion time-parameterization

// Per-waypoint velocities, by central difference, zero at both ends.
//
//     v_i = (q_i+1 - q_i-1) / (t_i+1 - t_i-1)
//
// WHY FILL THIS IN AT ALL, given that the controller in this module
// tracks positions? Because the field exists in the message and leaving
// it empty is a decision, not a default. In ROS, an empty `velocities`
// array means "unspecified", and what a controller does with
// unspecified velocities is controller-specific:
// joint_trajectory_controller interpolates positions only, while a
// controller that splines will assume zero velocity at each waypoint and
// therefore come to a complete stop at every one of them. A five-waypoint
// trajectory that stops four times is a common and baffling symptom, and
// it is caused by a field nobody filled in.
//
// The endpoints are zero because the robot is at rest there -- it is
// where the motion begins and ends.
// #region trajectory-velocities
function trajectoryVelocities(positions, times) {
    var velocities = [];
    var i, j;

    for (i = 0; i < positions.length; i++) {
        var row = [];
        for (j = 0; j < positions[i].length; j++) {
            if (i === 0 || i === positions.length - 1) {
                row.push(0);
            } else {
                var span = times[i + 1] - times[i - 1];
                row.push(span > 0 ? (positions[i + 1][j] - positions[i - 1][j]) / span : 0);
            }
        }
        velocities.push(row);
    }
    return velocities;
}
// #endregion trajectory-velocities

// The whole conversion: a planner path becomes a JointTrajectory.
//
// `plan` is a committed plan record from plans/*.json (or anything with
// the same three fields: q_layout, smoothed_path, joint_names).
//
// REFUSES rather than guesses, in three cases, each of which is a real
// mistake with a quiet failure mode:
//
//   - a plan with fewer than two configurations. One configuration is a
//     pose, not a motion; a controller handed it will either reject it or
//     jump.
//   - a plan whose base moves. There is no field for it. Splitting the
//     motion is the caller's decision and this function must not make it
//     silently by dropping six columns.
//   - a joint name the plan's layout does not contain (planJointColumns
//     reports it).
// #region trajectory-from-plan
function trajectoryFromPlan(plan, jointNames, options) {
    var opts = options || {};
    var path = opts.useRawPath ? plan.raw_path : plan.smoothed_path;
    var names = jointNames || plan.joint_names;

    if (!path || path.length < 2) {
        console.error('joint_trajectory: a plan needs at least two configurations ' +
            'to become a trajectory; got ' + (path ? path.length : 0));
        return null;
    }

    if (!baseIsStationary(path, plan.q_layout)) {
        console.error('joint_trajectory: this plan moves the robot base, and a ' +
            'JointTrajectory has no field that can carry a base pose -- its ' +
            'positions are single-DOF joint values. Split the motion into a base ' +
            'goal (a navigation action, or FollowJointTrajectory\'s own ' +
            'multi_dof_trajectory field) and a joint trajectory, and sequence the two.');
        return null;
    }

    var positions = planJointColumns(path, plan.q_layout, names);
    if (!positions) return null;

    var times = trajectoryTimesFromPositions(positions, opts.maxVelocity);
    var velocities = trajectoryVelocities(positions, times);

    var points = [];
    var i;
    for (i = 0; i < positions.length; i++) {
        points.push({
            positions: positions[i],
            velocities: velocities[i],
            // left empty exactly as a real message would be when nothing
            // computes them; see #trajectory-velocities for why "empty"
            // is a statement and not an omission
            accelerations: [],
            effort: [],
            time_from_start: times[i]
        });
    }

    return {
        header: {
            // a real header carries a stamp and a frame_id. A trajectory's
            // frame_id is conventionally empty -- joint positions are not
            // in any Cartesian frame -- and the stamp is what says WHEN
            // the trajectory starts: a zero stamp means "now", which is
            // the convention this module uses throughout.
            stamp: 0,
            frame_id: ''
        },
        joint_names: names.slice(),
        points: points,

        // not part of trajectory_msgs/JointTrajectory. Carried alongside
        // so a panel can say which plan a trajectory came from, and so a
        // claim on a slide is checkable against the file that produced it.
        provenance: {
            plan: plan.provenance ? plan.provenance.generated_by : 'unknown',
            robot: plan.robot,
            seed: plan.seed,
            path_used: opts.useRawPath ? 'raw_path' : 'smoothed_path',
            max_velocity: opts.maxVelocity === undefined ? TRAJECTORY_MAX_JOINT_VELOCITY : opts.maxVelocity
        }
    };
}
// #endregion trajectory-from-plan

//////////////////////////////////////////////////
/////   READING ONE
//////////////////////////////////////////////////

// #region trajectory-duration
// How long the trajectory takes: the last point's time_from_start.
function trajectoryDuration(trajectory) {
    if (!trajectory || !trajectory.points.length) return 0;
    return trajectory.points[trajectory.points.length - 1].time_from_start;
}
// #endregion trajectory-duration

// The desired position of every named joint at time t, by LINEAR
// interpolation between the bracketing waypoints.
//
// Clamped at both ends: before the start it returns the first point,
// after the end the last. A controller asks for samples until it has
// arrived, which can be after the trajectory's nominal duration, so
// "past the end" is a normal query and not an error.
//
// WHY LINEAR, when every graphics instinct says spline it. The planner
// validated the straight line in configuration space between consecutive
// waypoints -- edgeIsValid in motion_plan.js checks samples along exactly
// that segment. Linear interpolation reproduces that segment, so a
// sampled configuration is collision-free for the same reason the plan
// is. A cubic through the same waypoints leaves the segment, and the
// amount by which it leaves is unbounded by anything the planner checked.
// Smoothness here would be bought with validity, and nothing downstream
// would report the trade.
//
// Returns { positions, segment, alpha } -- the panel draws alpha, and the
// segment index is what the execution feedback reports.
// #region sample-trajectory
function sampleTrajectory(trajectory, t) {
    var points = trajectory.points;
    var n = points.length;

    if (t <= points[0].time_from_start) {
        return { positions: points[0].positions.slice(), segment: 0, alpha: 0 };
    }
    if (t >= points[n - 1].time_from_start) {
        return { positions: points[n - 1].positions.slice(), segment: n - 2, alpha: 1 };
    }

    var i;
    for (i = 0; i < n - 1; i++) {
        var t0 = points[i].time_from_start;
        var t1 = points[i + 1].time_from_start;
        if (t < t0 || t > t1) continue;

        var alpha = (t - t0) / (t1 - t0);
        var positions = [];
        var j;
        for (j = 0; j < points[i].positions.length; j++) {
            positions.push(points[i].positions[j] +
                alpha * (points[i + 1].positions[j] - points[i].positions[j]));
        }
        return { positions: positions, segment: i, alpha: alpha };
    }

    // unreachable given the clamps above, and worth saying so rather than
    // returning undefined if it ever becomes reachable
    console.error('joint_trajectory: no segment brackets t = ' + t);
    return { positions: points[n - 1].positions.slice(), segment: n - 2, alpha: 1 };
}
// #endregion sample-trajectory

// Everything that can be checked about a trajectory without a robot.
//
// Run before a trajectory is sent, because every one of these presents
// as a motion that looks wrong rather than as an error: a non-monotonic
// time makes the arm jump backwards, a short positions array drives the
// wrong joints, and a NaN propagates into every joint angle on the first
// interpolation and the robot simply disappears.
//
// Returns an array of problem strings; empty means valid.
// #region validate-trajectory
function validateTrajectory(trajectory) {
    var problems = [];

    if (!trajectory) return ['trajectory is null'];
    if (!trajectory.joint_names || !trajectory.joint_names.length) {
        problems.push('joint_names is empty: the positions arrays have no meaning without it');
    }
    if (!trajectory.points || trajectory.points.length < 2) {
        problems.push('a trajectory needs at least two points');
        return problems;
    }

    var width = trajectory.joint_names.length;
    var i, j;

    for (i = 0; i < trajectory.points.length; i++) {
        var p = trajectory.points[i];

        if (p.positions.length !== width) {
            problems.push('point ' + i + ' has ' + p.positions.length +
                ' positions for ' + width + ' joint_names');
        }
        if (p.velocities.length && p.velocities.length !== width) {
            problems.push('point ' + i + ' has ' + p.velocities.length +
                ' velocities for ' + width + ' joint_names');
        }
        for (j = 0; j < p.positions.length; j++) {
            if (!isFinite(p.positions[j])) {
                problems.push('point ' + i + ' position ' + j + ' is not finite');
            }
        }
        if (!isFinite(p.time_from_start) || p.time_from_start < 0) {
            problems.push('point ' + i + ' has time_from_start ' + p.time_from_start);
        }
        if (i > 0 && p.time_from_start <= trajectory.points[i - 1].time_from_start) {
            problems.push('point ' + i + ' time_from_start (' + p.time_from_start +
                ') does not increase past point ' + (i - 1) + ' (' +
                trajectory.points[i - 1].time_from_start + ')');
        }
    }

    // duplicated names are not a format error but they are never
    // intentional, and they make the trajectory ambiguous about which
    // column drives the joint
    var seen = {};
    for (i = 0; i < trajectory.joint_names.length; i++) {
        var name = trajectory.joint_names[i];
        if (seen[name]) problems.push('joint_names contains "' + name + '" twice');
        seen[name] = true;
    }

    return problems;
}
// #endregion validate-trajectory

// Which of the robot's joints this trajectory says NOTHING about.
//
// Worth a function and worth a slide, because the natural reading of a
// JointTrajectory is "this is what the robot does", and it is not. It is
// what the named joints do. An unnamed joint is not commanded to hold
// position -- it is simply not mentioned, and whatever was driving it
// before carries on driving it. For mr2 that unnamed joint is
// clavicle_left_roll, and an onlooker who sees it drift during an
// otherwise correct execution will go looking for a bug in the
// trajectory, which is the wrong place.
// #region trajectory-joints-not-named
function trajectoryJointsNotNamed(trajectory, robot) {
    var named = {};
    var i, x;
    for (i = 0; i < trajectory.joint_names.length; i++) named[trajectory.joint_names[i]] = true;

    var missing = [];
    for (x in robot.joints) {
        if (!named[x]) missing.push(x);
    }
    return missing;
}
// #endregion trajectory-joints-not-named
