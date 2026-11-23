/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Behavior Trees | The Joint Trajectory Action

    Executes a JointTrajectory on the robot, with the lifecycle of an
    ACTION rather than of a function call or a published message.

    WHY AN ACTION AND NOT A TOPIC. Publishing a trajectory is a
    fire-and-forget statement about what should happen. It gives the
    sender nothing: no acknowledgement that the trajectory was accepted,
    no progress while it runs, no verdict when it stops, and no way to
    change its mind. Executing a motion needs all four, and the ROS
    answer to "a long-running request that reports progress and can be
    cancelled" is an action.

    The three ROS communication patterns, in the order a student meets
    them:

      TOPIC    one-way, anonymous, no reply. Joint STATE is a topic -- it
               is a fact about now, published continuously, and nobody
               acknowledges a fact.
      SERVICE  request/reply, blocking, no progress. Collision checking
               is a service: ask, get an answer, done in milliseconds.
      ACTION   goal/feedback/result, non-blocking, cancellable. Motion
               execution is an action, because it takes seconds and the
               caller needs to know how it is going and be able to stop
               it.

    THE SHAPE BEING MIRRORED is control_msgs/FollowJointTrajectory, read
    from ros-controls/control_msgs rather than from memory:

      Goal      trajectory_msgs/JointTrajectory          trajectory
                trajectory_msgs/MultiDOFJointTrajectory  multi_dof_trajectory
                JointTolerance[]                         path_tolerance
                JointComponentTolerance[]                component_path_tolerance
                JointTolerance[]                         goal_tolerance
                JointComponentTolerance[]                component_goal_tolerance
                builtin_interfaces/Duration              goal_time_tolerance
      Result    int32   error_code
                string  error_string
      Feedback  std_msgs/Header            header
                string[]                   joint_names
                JointTrajectoryPoint       desired
                JointTrajectoryPoint       actual
                JointTrajectoryPoint       error
                int32                      index
                (and the multi_dof_* counterparts of all five)

    with error codes
      0 SUCCESSFUL, -1 INVALID_GOAL, -2 INVALID_JOINTS,
      -3 OLD_HEADER_TIMESTAMP, -4 PATH_TOLERANCE_VIOLATED,
      -5 GOAL_TOLERANCE_VIOLATED.

    THREE SIMPLIFICATIONS, named so that nobody has to discover them:

      - the real tolerances are JointTolerance[] -- one entry per joint,
        each carrying {name, position, velocity, acceleration} -- and
        here each is a single number applied to every joint's position.
        Worth knowing that the real message gives two special values a
        meaning: 0 means "unspecified, use the server's default" and -1
        means "erased, let this joint move without restriction".
      - the real feedback's desired/actual/error are each a whole
        JointTrajectoryPoint; here they are the `positions` arrays of
        those three points, which is the only part this module computes.
      - multi_dof_trajectory is not implemented at all, which is the
        same decision joint_trajectory.js makes about the base and for
        the same reason.

    THE CONTROLLER IS A P SERVO, not a teleport. This file does not
    assign the sampled trajectory position to robot.joints[x].angle. It
    writes robot.joints[x].control and lets applyControls fold that into
    .angle, which is the same path every other control input in KinEval
    takes, and it is the whole reason `error` is a field in the feedback
    message: the desired position and the actual position are different
    numbers, and the difference is the only thing that makes the
    tolerance fields meaningful.

    Assigning .angle directly -- which is what the motion_planning
    module's n/b plan traversal deliberately does, because it is
    inspecting plan vertices rather than executing them -- would make
    every error identically zero, every tolerance unfalsifiable, and the
    arm would snap between waypoints. "It teleports between waypoints" and
    "it executes the trajectory" look similar for about one second and
    are not the same claim.

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
/////   CONSTANTS
//////////////////////////////////////////////////

// The goal lifecycle, in ROS 2's names.
//
// ROS 1's actionlib called these PENDING / ACTIVE / PREEMPTED /
// SUCCEEDED / ABORTED / REJECTED, and "preempt" is still the word
// everyone says for what ROS 2 calls a cancel. Both vocabularies turn up
// in the same codebase; this module uses the ROS 2 names and says
// "preempt" in prose.
// #region action-constants
var JTA_ACCEPTED = 'accepted';
var JTA_EXECUTING = 'executing';
var JTA_CANCELING = 'canceling';
var JTA_SUCCEEDED = 'succeeded';
var JTA_CANCELED = 'canceled';
var JTA_ABORTED = 'aborted';

// control_msgs/FollowJointTrajectory's own result codes.
var JTA_SUCCESSFUL = 0;
var JTA_INVALID_GOAL = -1;
var JTA_INVALID_JOINTS = -2;
var JTA_OLD_HEADER_TIMESTAMP = -3;
var JTA_PATH_TOLERANCE_VIOLATED = -4;
var JTA_GOAL_TOLERANCE_VIOLATED = -5;

// Proportional gain of the joint servo, per update. Measured rather than
// picked: at 0.35, with the default 20 Hz update and the 0.6 rad/s cruise
// speed in joint_trajectory.js, the steady-state tracking error on the
// committed mr2 plan peaks around 0.09 rad -- large enough that the
// error plot shows a real lag, small enough to stay inside the default
// path tolerance below with room to spare.
var JTA_SERVO_GAIN = 0.35;

// Per-joint tracking error that aborts the goal, radians. A path
// tolerance is a statement about the CONTROLLER's health, not about the
// plan: exceeding it means the arm is not where the trajectory says it
// should be right now, which is what you would see if it had hit
// something.
var JTA_PATH_TOLERANCE = 0.25;

// Per-joint error at the final waypoint that counts as arrived, radians.
// Much tighter than the path tolerance, because the whole point of the
// motion was to end up at that configuration.
var JTA_GOAL_TOLERANCE = 0.02;

// Extra time past the trajectory's nominal duration in which to converge
// before giving up, seconds. A P servo approaches its setpoint
// asymptotically, so it is still converging when the clock runs out:
// without a grace period every single goal would report
// GOAL_TOLERANCE_VIOLATED, which is exactly the bug this constant exists
// to prevent and exactly the bug its absence looks like.
var JTA_GOAL_TIME_TOLERANCE = 2.0;
// #endregion action-constants

//////////////////////////////////////////////////
/////   THE SERVER
//////////////////////////////////////////////////

// A joint-trajectory action server bound to one robot.
//
// Holds no timer and starts no loop: `update(dt)` is called by whoever
// owns the clock -- the demo page's animation loop, a deck panel's step
// button, or a headless test's for-loop. Same code, same results, three
// very different clocks. That is the property that makes the
// step-through panels show the real controller rather than a
// reimplementation of it.
// #region action-server-create
function jointTrajectoryActionServer(robot, options) {
    var opts = options || {};
    return {
        robot: robot,

        gain: opts.gain !== undefined ? opts.gain : JTA_SERVO_GAIN,

        // the active goal, or null. A server with no goal is not idle in
        // some fourth state -- it simply has nothing to execute.
        goal: null,
        state: null,
        time: 0,            // seconds since the active goal started
        result: null,       // the last goal's result, kept for inspection
        feedback: null,     // the most recent feedback message

        // counters, so a panel can state what happened rather than
        // implying it
        stats: { goals: 0, canceled: 0, aborted: 0, succeeded: 0, rejected: 0 }
    };
}
// #endregion action-server-create

// Accept or reject a goal, in that order, before any motion happens.
//
// REJECTION IS A FEATURE. An action server that accepts every goal and
// discovers the problem while executing has converted a programming
// error into a robot moving wrongly. All three checks here are cheap and
// all three catch a real mistake:
//
//   INVALID_GOAL    the trajectory is malformed (validateTrajectory)
//   INVALID_JOINTS  it names a joint this robot does not have -- the
//                   classic symptom of a trajectory built for a
//                   different robot, or built from a plan whose q layout
//                   was reordered
//   (busy)          a goal is already executing. ROS's own convention is
//                   that a new goal PREEMPTS the current one, and that
//                   is what this does: cancel the old one, then accept
//                   the new. Quietly queueing the new goal instead would
//                   make a behavior tree that re-sends a goal every tick
//                   accumulate thousands of them.
// #region action-send-goal
function jointTrajectoryActionSendGoal(server, goal) {
    var problems = validateTrajectory(goal.trajectory);
    if (problems.length) {
        server.stats.rejected++;
        server.result = {
            error_code: JTA_INVALID_GOAL,
            error_string: 'invalid goal: ' + problems.join('; ')
        };
        server.state = JTA_ABORTED;
        return { accepted: false, result: server.result };
    }

    var i;
    for (i = 0; i < goal.trajectory.joint_names.length; i++) {
        var name = goal.trajectory.joint_names[i];
        if (!server.robot.joints[name]) {
            server.stats.rejected++;
            server.result = {
                error_code: JTA_INVALID_JOINTS,
                error_string: 'robot "' + server.robot.name + '" has no joint "' + name + '"'
            };
            server.state = JTA_ABORTED;
            return { accepted: false, result: server.result };
        }
    }

    if (server.goal && (server.state === JTA_EXECUTING || server.state === JTA_ACCEPTED)) {
        // preempted by the new goal: the old one's result is CANCELED,
        // and it is reported before the new goal starts so the record is
        // not lost
        jointTrajectoryActionCancel(server, 'preempted by a new goal');
    }

    server.goal = {
        trajectory: goal.trajectory,
        path_tolerance: goal.path_tolerance !== undefined ? goal.path_tolerance : JTA_PATH_TOLERANCE,
        goal_tolerance: goal.goal_tolerance !== undefined ? goal.goal_tolerance : JTA_GOAL_TOLERANCE,
        goal_time_tolerance: goal.goal_time_tolerance !== undefined ? goal.goal_time_tolerance : JTA_GOAL_TIME_TOLERANCE
    };
    server.state = JTA_ACCEPTED;
    server.time = 0;
    server.result = null;
    server.feedback = null;
    server.stats.goals++;

    return { accepted: true };
}
// #endregion action-send-goal

// Stop executing, now, and report CANCELED.
//
// WHAT "STOP" MEANS, and the mistake worth naming: it does NOT mean
// stop writing to the robot. It means stop advancing along the
// trajectory. The servo's last commanded setpoint stays where it was, so
// the arm holds the configuration it had reached. A cancel that simply
// stopped calling the controller would leave the joints wherever the
// last partial control delta put them, which is a different
// configuration from any waypoint and from where the arm appeared to be.
//
// This is the function a behavior tree's halt eventually calls. The
// chain is: a reactive sequence re-evaluates a condition -> it takes a
// different branch -> btHalt walks the abandoned subtree -> the execute
// leaf's halt handler runs -> this. Every link in that chain has to
// exist for the arm to actually stop, which is why btHalt is not
// optional.
// #region action-cancel
function jointTrajectoryActionCancel(server, reason) {
    if (!server.goal) return null;

    server.state = JTA_CANCELING;
    server.hold = server.feedback ? server.feedback.desired.slice() : null;

    server.result = {
        error_code: JTA_SUCCESSFUL,     // a cancel is not an error
        error_string: 'canceled: ' + (reason || 'no reason given'),
        canceled_at_time: server.time,
        canceled_at_segment: server.feedback ? server.feedback.segment : 0
    };
    server.state = JTA_CANCELED;
    server.goal = null;
    server.stats.canceled++;
    return server.result;
}
// #endregion action-cancel

// One control cycle: advance the clock, sample the trajectory, servo
// toward the sample, check the tolerances, and publish feedback.
//
// Returns the server's state, so a caller can branch on it without
// reaching into the server. Does nothing at all when no goal is active,
// which is what lets this be called unconditionally from an animation
// loop.
//
// THE ORDER MATTERS and is the order a real controller uses:
//
//   1. advance time         -- the trajectory is a function of it
//   2. sample               -- what SHOULD the joints be now
//   3. command              -- servo .control toward that
//   4. apply                -- fold .control into .angle (applyControls)
//   5. measure              -- error = desired - actual, AFTER applying,
//                              so the error reported is the error that
//                              exists now and not the one from last cycle
//   6. judge                -- path tolerance during, goal tolerance at
//                              the end
//
// Steps 5 and 6 after 4 rather than before is the subtle one. Reporting
// an error measured before the command was applied makes the feedback
// consistently one cycle stale, which at 20 Hz is enough to make a
// tolerance plot disagree with the arm on screen.
// #region action-update
function jointTrajectoryActionUpdate(server, dt) {
    if (!server.goal) return server.state;

    var goal = server.goal;
    var trajectory = goal.trajectory;
    var names = trajectory.joint_names;

    if (server.state === JTA_ACCEPTED) server.state = JTA_EXECUTING;

    // 1. the clock
    server.time += dt;

    // 2. where the trajectory says the joints should be
    var sample = sampleTrajectory(trajectory, server.time);

    // 3. and 4. command and apply
    var i;
    for (i = 0; i < names.length; i++) {
        var joint = server.robot.joints[names[i]];
        joint.control += server.gain * (sample.positions[i] - joint.angle);
    }
    applyControls(server.robot);

    // 5. measure
    var actual = [];
    var error = [];
    var worst = 0;
    for (i = 0; i < names.length; i++) {
        var angle = server.robot.joints[names[i]].angle;
        actual.push(angle);
        var e = sample.positions[i] - angle;
        error.push(e);
        worst = Math.max(worst, Math.abs(e));
    }

    server.feedback = {
        header: { stamp: server.time },
        joint_names: names.slice(),
        desired: sample.positions.slice(),
        actual: actual,
        error: error,

        // `index` IS a real field of the feedback message -- "the
        // currently used point from trajectory.points array" -- and the
        // message does not define which point that is when the robot is
        // between two of them. This module reports the point being
        // travelled TOWARD, which is the one whose time_from_start the
        // controller is currently working against.
        index: Math.min(sample.segment + 1, trajectory.points.length - 1),

        // these two are NOT in the real message. They are carried because
        // every panel in this module has to say where along the
        // trajectory the robot is, and "40% of the way through segment 2"
        // is the only phrasing of that a reader can check against the
        // picture.
        segment: sample.segment,
        alpha: sample.alpha,
        worst_error: worst,
        duration: trajectoryDuration(trajectory)
    };

    // 6a. the path tolerance, checked only while the trajectory is still
    //     running. After the end the arm is converging on the final
    //     waypoint and the goal tolerance is the test that applies.
    var duration = trajectoryDuration(trajectory);
    if (server.time < duration && worst > goal.path_tolerance) {
        server.result = {
            error_code: JTA_PATH_TOLERANCE_VIOLATED,
            error_string: 'joint "' + names[indexOfLargest(error)] + '" is ' +
                worst.toFixed(4) + ' rad from its desired position, past the ' +
                goal.path_tolerance + ' rad path tolerance, at t = ' +
                server.time.toFixed(2) + 's'
        };
        server.state = JTA_ABORTED;
        server.goal = null;
        server.stats.aborted++;
        return server.state;
    }

    // 6b. the goal tolerance, checked once the trajectory's own time is
    //     up. Arriving early is possible and is a success.
    if (server.time >= duration) {
        if (worst <= goal.goal_tolerance) {
            server.result = {
                error_code: JTA_SUCCESSFUL,
                error_string: '',
                finished_at_time: server.time,
                worst_error: worst
            };
            server.state = JTA_SUCCEEDED;
            server.goal = null;
            server.stats.succeeded++;
            return server.state;
        }

        if (server.time > duration + goal.goal_time_tolerance) {
            server.result = {
                error_code: JTA_GOAL_TOLERANCE_VIOLATED,
                error_string: 'still ' + worst.toFixed(4) + ' rad from the final ' +
                    'waypoint ' + goal.goal_time_tolerance + 's after the ' +
                    'trajectory ended (tolerance ' + goal.goal_tolerance + ' rad)'
            };
            server.state = JTA_ABORTED;
            server.goal = null;
            server.stats.aborted++;
            return server.state;
        }
    }

    return server.state;
}
// #endregion action-update

// Keep commanding the last setpoint after a goal ends.
//
// Called every cycle in which no goal is active, by a caller that wants
// the arm to STAY where the finished motion left it. Without it the
// joints hold their angles anyway in this simulation -- nothing decays --
// but on real hardware gravity exists, and a trajectory controller that
// stops writing is a limp arm. Stating it here makes the simulation's
// silence about gravity a choice rather than an oversight.
// #region action-hold
function jointTrajectoryActionHold(server) {
    if (server.goal || !server.hold) return;

    var names = server.feedback ? server.feedback.joint_names : [];
    var i;
    for (i = 0; i < names.length; i++) {
        var joint = server.robot.joints[names[i]];
        joint.control += server.gain * (server.hold[i] - joint.angle);
    }
    applyControls(server.robot);
}
// #endregion action-hold

// #region action-index-of-largest
// Which joint has the largest absolute error -- so an error message can
// name the joint rather than report a number with no referent.
function indexOfLargest(values) {
    var best = 0;
    var i;
    for (i = 1; i < values.length; i++) {
        if (Math.abs(values[i]) > Math.abs(values[best])) best = i;
    }
    return best;
}
// #endregion action-index-of-largest

// Is the robot already at this configuration, to within a tolerance?
//
// The condition half of the "check before you act" idiom: the fallback
// over [at-configuration?, move-to-configuration] in trees.js asks this
// first, and a tree that is re-ticked after the motion finished costs one
// comparison instead of a second execution.
//
// Lives in this file rather than in bt_tasks.js because the tolerance it
// uses has to be the action's goal tolerance. If the condition were
// tighter than the tolerance the action succeeds on, a sequence of
// [check, move] would move, succeed, be re-ticked, find itself not
// arrived, and move again -- forever, with the arm visibly still. Two
// numbers that must agree belong next to each other.
// #region action-at-configuration
function robotAtConfiguration(robot, jointNames, positions, tolerance) {
    var tol = tolerance === undefined ? JTA_GOAL_TOLERANCE : tolerance;
    var i;
    for (i = 0; i < jointNames.length; i++) {
        if (Math.abs(robot.joints[jointNames[i]].angle - positions[i]) > tol) return false;
    }
    return true;
}
// #endregion action-at-configuration
