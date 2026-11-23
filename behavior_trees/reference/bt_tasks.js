/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Behavior Trees | The leaves

    Every execution node this module's trees are built from, in two
    groups.

    GROUP ONE, the SEARCH tasks, are a runnable version of the
    object-searching example in Sebastian Castro's "Introduction to
    Behavior Trees" (roboticseabass.com, 2021) -- a mobile robot visiting
    locations looking for objects. They act on a tiny simulated world
    held entirely on the blackboard (where the robot is, which objects
    are where) and never touch the KinEval robot. They exist so that the
    foundational slides' trees can actually be ticked rather than only
    drawn: the difference between a Sequence and a ReactiveSequence, or
    between a Fallback with and without an explicit success condition, is
    a difference in what HAPPENS, and a static picture cannot show it.

    GROUP TWO, the MOTION tasks, drive the real thing: the KinEval robot,
    the committed RRT-Connect plan, and the joint-trajectory action
    server. These are what the rest of the module is about.

    WHAT EVERY LEAF HERE DECLARES. Each carries `reads` and `writes`
    arrays naming the blackboard keys it touches, which is what
    btBlackboardRequire checks before a tree is ever ticked. The
    declarations are maintained by hand and they are the only machine-
    readable record of a tree's data flow -- a blackboard is otherwise
    global mutable state, and a tree wired up wrong fails on whichever
    tick first reaches the bad branch, which may be thousands of ticks in.

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
/////   GROUP ONE: THE SEARCH TASKS
//////////////////////////////////////////////////

// How many ticks a simulated navigation takes. Fixed rather than
// distance-dependent: the foundational slides are about tree semantics,
// and a navigation whose duration varies would make two runs of the same
// tree produce different traces for a reason that has nothing to do with
// the tree.
var SEARCH_NAVIGATION_TICKS = 3;

// #region task-at-location
// Is the robot already at this location? The condition half of
// "check before you act".
function taskAtLocation(location) {
    var node = btCondition('At ' + location + '?', function (blackboard) {
        return blackboard.robot_location === location;
    });
    node.reads = ['robot_location'];
    node.writes = [];
    return node;
}
// #endregion task-at-location

// Drive to a location, over several ticks.
//
// Returns Running while it travels -- which is the single most important
// property of an action node, and the reason this is not a function
// call. The tree keeps ticking at its own rate the whole time, so
// anything above this node in a reactive branch is still being
// re-evaluated while the robot drives.
//
// While travelling, robot_location is set to null rather than left at
// the old value: a robot halfway between two rooms is in neither, and a
// condition that says otherwise is the kind of small lie that makes a
// tree work in the demo and fail in the building.
// #region task-navigate-to
function taskNavigateTo(location) {
    var node = btAction('Navigate to ' + location, {
        start: function (blackboard) {
            blackboard.nav_ticks = 0;
            blackboard.nav_goal = location;
            blackboard.robot_location = null;
            blackboard.log.push('navigate: heading for ' + location);
        },
        step: function (blackboard) {
            blackboard.nav_ticks++;
            if (blackboard.nav_ticks < SEARCH_NAVIGATION_TICKS) return BT_RUNNING;
            blackboard.robot_location = location;
            blackboard.log.push('navigate: arrived at ' + location);
            return BT_SUCCESS;
        },
        halt: function (blackboard) {
            // abandoned mid-drive. The robot stops where it is, which is
            // nowhere in particular -- and saying so is the point.
            blackboard.log.push('navigate: halted en route to ' + location);
            blackboard.nav_goal = null;
        }
    });
    node.reads = ['robot_location'];
    node.writes = ['robot_location', 'nav_ticks', 'nav_goal'];
    return node;
}
// #endregion task-navigate-to

// Look for an object at the robot's current location: Success if it is
// here, Failure if it is not.
//
// A CONDITION, not an action, and the distinction is doing work. Looking
// resolves within the tick, so a sequence of [navigate, look] fails
// immediately at a location that does not have the object, and a
// fallback over several such sequences moves straight on to the next
// location. Modelling the look as a multi-tick action instead would be
// defensible for a real perception pipeline and would change the tree's
// shape.
// #region task-find-object
function taskFindObject(object) {
    var node = btCondition('Found ' + object + '?', function (blackboard) {
        var here = blackboard.robot_location;
        var objects = (blackboard.objects_at || {})[here] || [];
        var found = objects.indexOf(object) !== -1;
        blackboard.log.push('look for ' + object + ' at ' + here + ': ' +
            (found ? 'found' : 'not here'));
        if (found) blackboard.found = (blackboard.found || []).concat([object]);
        return found;
    });
    node.reads = ['robot_location', 'objects_at'];
    node.writes = ['found'];
    return node;
}
// #endregion task-find-object

// Pop the next location off the queue and write it to the blackboard.
//
// This is Castro's "GetLoc" node, and it is the hinge of his refactoring
// argument. A fallback over one subtree per location is N copies of the
// same logic; this node plus a blackboard key plus a repeat decorator is
// one copy. Failure when the queue is empty is what terminates the
// decorator's repetition -- the tree's stopping condition is "ran out of
// places to look", expressed as a leaf returning Failure rather than as
// a counter somewhere.
// #region task-next-location
function taskNextLocation() {
    var node = btAction('Get next location', {
        step: function (blackboard) {
            var queue = blackboard.location_queue || [];
            if (!queue.length) {
                blackboard.log.push('get next location: queue empty');
                return BT_FAILURE;
            }
            blackboard.target_location = queue.shift();
            blackboard.log.push('get next location: ' + blackboard.target_location);
            return BT_SUCCESS;
        }
    });
    node.reads = ['location_queue'];
    node.writes = ['target_location'];
    return node;
}
// #endregion task-next-location

// The blackboard-parameterized versions of the two nodes above: same
// behaviour, but the location is read from `target_location` instead of
// being baked into the node when the tree was built.
//
// This is the whole trick. A node whose parameter is a blackboard key is
// reusable across every value that key can take, so the subtree
// containing it is written once.
// #region task-at-target-location
function taskAtTargetLocation() {
    var node = btCondition('At target?', function (blackboard) {
        return blackboard.robot_location === blackboard.target_location;
    });
    node.reads = ['robot_location', 'target_location'];
    node.writes = [];
    return node;
}

function taskNavigateToTarget() {
    var node = btAction('Navigate to target', {
        start: function (blackboard) {
            blackboard.nav_ticks = 0;
            blackboard.robot_location = null;
            blackboard.log.push('navigate: heading for ' + blackboard.target_location);
        },
        step: function (blackboard) {
            blackboard.nav_ticks++;
            if (blackboard.nav_ticks < SEARCH_NAVIGATION_TICKS) return BT_RUNNING;
            blackboard.robot_location = blackboard.target_location;
            blackboard.log.push('navigate: arrived at ' + blackboard.robot_location);
            return BT_SUCCESS;
        },
        halt: function (blackboard) {
            blackboard.log.push('navigate: halted en route to ' + blackboard.target_location);
        }
    });
    node.reads = ['target_location'];
    node.writes = ['robot_location', 'nav_ticks'];
    return node;
}

function taskFindObjectHere() {
    var node = btCondition('Found the object?', function (blackboard) {
        var here = blackboard.robot_location;
        var objects = (blackboard.objects_at || {})[here] || [];
        var wanted = blackboard.wanted_object;
        var found = objects.indexOf(wanted) !== -1;
        blackboard.log.push('look for ' + wanted + ' at ' + here + ': ' +
            (found ? 'found' : 'not here'));
        if (found) blackboard.found = (blackboard.found || []).concat([wanted]);
        return found;
    });
    node.reads = ['robot_location', 'objects_at', 'wanted_object'];
    node.writes = ['found'];
    return node;
}
// #endregion task-at-target-location

//////////////////////////////////////////////////
/////   GROUP TWO: THE MOTION TASKS
//////////////////////////////////////////////////

// How many ticks the simulated planner takes before it returns a plan.
// The committed plan took 46 RRT-Connect iterations (see
// plans/mr2_arm_workspace.json), and this leaf reports a tick's worth of
// them at a time so that "planning is not instantaneous" is visible in
// the tree rather than asserted in a comment. It is a REPLAY, not a
// search: the plan is already known.
var PLAN_ITERATIONS_PER_TICK = 8;

// Produce a JointTrajectory and write it to the blackboard.
//
// WHAT THIS LEAF REALLY IS. In a system with a live planner this is the
// node that sends a planning request and waits for the reply -- in
// MoveIt, a /plan_kinematic_path service call or a MoveGroup action
// goal. Here the plan is a committed artifact and the leaf replays it,
// returning Running for a few ticks first so that the tree's shape is
// the shape it would have with a real planner: an action that takes
// time, can be waited on, and can be abandoned.
//
// What it is NOT is a planner. Nothing in this module reimplements
// RRT-Connect; the plan came from kineval/motion_plan.js and the file it
// came from records the seed.
//
// It writes `trajectory`, and the execute leaf reads `trajectory`.
// Neither names the other. Swapping this leaf for one that calls a real
// planner changes nothing downstream, which is the property the
// blackboard is buying.
// #region task-plan-motion
function taskPlanMotion(plan, jointNames, options) {
    var node = btAction('Plan motion', {
        start: function (blackboard) {
            blackboard.plan_iterations = 0;
            blackboard.log.push('plan: replaying ' + plan.provenance.generated_by +
                ' (' + plan.search.iterations + ' iterations, ' +
                plan.search.smoothed_path_length + ' waypoints)');
        },
        step: function (blackboard) {
            blackboard.plan_iterations += PLAN_ITERATIONS_PER_TICK;
            if (blackboard.plan_iterations < plan.search.iterations) return BT_RUNNING;

            var trajectory = trajectoryFromPlan(plan, jointNames, options);
            if (!trajectory) {
                blackboard.log.push('plan: FAILED to build a trajectory from the plan');
                return BT_FAILURE;
            }
            blackboard.trajectory = trajectory;
            blackboard.log.push('plan: trajectory with ' + trajectory.points.length +
                ' points, ' + trajectoryDuration(trajectory).toFixed(2) + 's long');
            return BT_SUCCESS;
        },
        halt: function (blackboard) {
            // a real planning request would be cancelled here
            blackboard.log.push('plan: halted after ' + blackboard.plan_iterations + ' iterations');
        }
    });
    node.reads = [];
    node.writes = ['trajectory', 'plan_iterations'];
    return node;
}
// #endregion task-plan-motion

// Send the blackboard's trajectory to the joint-trajectory action server
// and report its progress as a BT status.
//
// THE MAPPING IS THE WHOLE SLIDE:
//
//     action EXECUTING  -> BT_RUNNING
//     action SUCCEEDED  -> BT_SUCCESS
//     action ABORTED    -> BT_FAILURE
//     action CANCELED   -> BT_FAILURE
//     goal rejected     -> BT_FAILURE
//
// The two vocabularies are different sizes, and that asymmetry is where
// information is lost: six action states collapse onto three BT
// statuses, so WHY a motion failed -- path tolerance, bad joint names,
// preempted by another goal -- is not visible to the tree. The tree can
// only see that it failed. Anything above this node that wants to react
// differently to different failures has to read the result off the
// blackboard, which is why this leaf writes `last_result`.
//
// A leaf that does not block. start() sends the goal once; step()
// advances the controller by one cycle and asks. Sending the goal from
// step() would re-send it every tick, and because a new goal preempts
// the current one (see #action-send-goal), the arm would restart the
// trajectory from the top on every tick and never move.
// #region task-follow-trajectory
function taskFollowTrajectory(server) {
    var node = btAction('Follow joint trajectory', {
        start: function (blackboard) {
            var trajectory = blackboard.trajectory;
            if (!trajectory) {
                blackboard.goal_accepted = false;
                return;
            }
            var reply = jointTrajectoryActionSendGoal(server, { trajectory: trajectory });
            blackboard.goal_accepted = reply.accepted;
            blackboard.log.push('execute: goal ' +
                (reply.accepted ? 'accepted' : 'REJECTED -- ' + reply.result.error_string));
        },
        step: function (blackboard) {
            if (!blackboard.trajectory) {
                blackboard.log.push('execute: nothing on the blackboard to execute');
                return BT_FAILURE;
            }
            if (blackboard.goal_accepted === false) {
                blackboard.last_result = server.result;
                return BT_FAILURE;
            }

            var state = jointTrajectoryActionUpdate(server, blackboard.dt);
            blackboard.action_state = state;
            blackboard.action_feedback = server.feedback;

            if (state === JTA_EXECUTING || state === JTA_ACCEPTED) return BT_RUNNING;

            blackboard.last_result = server.result;
            if (state === JTA_SUCCEEDED) {
                blackboard.log.push('execute: succeeded at t = ' +
                    server.result.finished_at_time.toFixed(2) + 's, worst error ' +
                    server.result.worst_error.toFixed(4) + ' rad');
                return BT_SUCCESS;
            }
            blackboard.log.push('execute: ' + state + ' -- ' + server.result.error_string);
            return BT_FAILURE;
        },
        halt: function (blackboard) {
            // The link in the chain that makes a cancelled motion
            // actually stop. Without this the arm would carry on
            // following a trajectory the tree has abandoned, because
            // nothing else ever tells the action server about the tree's
            // decision.
            var result = jointTrajectoryActionCancel(server, 'behavior tree halted the leaf');
            if (result) {
                blackboard.last_result = result;
                blackboard.log.push('execute: canceled at t = ' +
                    result.canceled_at_time.toFixed(2) + 's, in segment ' +
                    result.canceled_at_segment);
            }
        }
    });
    node.reads = ['trajectory'];
    node.writes = ['action_state', 'action_feedback', 'last_result', 'goal_accepted'];
    return node;
}
// #endregion task-follow-trajectory

// Is the robot already at the trajectory's final waypoint?
//
// Pairs with taskFollowTrajectory under a fallback, and the pairing is
// what keeps a tree that is ticked forever from executing the same
// motion forever. The tolerance comes from
// joint_trajectory_action.js#action-at-configuration, which is the same
// number the action succeeds on -- see that function's header for what
// goes wrong when the two disagree.
// #region task-at-configuration
function taskAtTrajectoryEnd(server) {
    var node = btCondition('At the goal configuration?', function (blackboard) {
        var trajectory = blackboard.trajectory;
        if (!trajectory) return false;
        var last = trajectory.points[trajectory.points.length - 1];
        return robotAtConfiguration(server.robot, trajectory.joint_names, last.positions);
    });
    node.reads = ['trajectory'];
    node.writes = [];
    return node;
}
// #endregion task-at-configuration

// The payload of the longer-horizon task: look at whatever the arm has
// been posed to look at, for one tick.
//
// Deterministic per target rather than random. A tree whose outcome
// changes between runs cannot be stepped through on a slide, and "the
// third target is the one with the defect" is a statement a reader can
// check against the panel.
// #region task-inspect
function taskInspect() {
    var node = btCondition('Part present?', function (blackboard) {
        var target = blackboard.inspection_target;
        var present = (blackboard.parts_present || {})[target] === true;
        blackboard.log.push('inspect ' + target + ': ' + (present ? 'part present' : 'empty'));
        blackboard.inspected = (blackboard.inspected || []).concat([target]);
        return present;
    });
    node.reads = ['inspection_target', 'parts_present'];
    node.writes = ['inspected'];
    return node;
}
// #endregion task-inspect

// Pop the next inspection target, and with it the plan that poses the
// arm to look at it. The motion-side counterpart of taskNextLocation.
//
// Writes BOTH the target name and the trajectory for reaching it, so the
// execute leaf downstream needs no knowledge of what a target is.
// #region task-next-inspection-target
function taskNextInspectionTarget(targets) {
    var node = btAction('Get next inspection target', {
        step: function (blackboard) {
            if (!blackboard.target_queue) {
                blackboard.target_queue = Object.keys(targets).slice();
            }
            if (!blackboard.target_queue.length) {
                blackboard.log.push('targets: queue empty, nothing left to inspect');
                return BT_FAILURE;
            }
            var name = blackboard.target_queue.shift();
            blackboard.inspection_target = name;
            blackboard.trajectory = targets[name];
            blackboard.log.push('targets: next is ' + name + ' (' +
                targets[name].points.length + '-point trajectory)');
            return BT_SUCCESS;
        }
    });
    node.reads = ['target_queue'];
    node.writes = ['inspection_target', 'trajectory', 'target_queue'];
    return node;
}
// #endregion task-next-inspection-target

// The reactivity example, in Castro's own terms: a battery level that
// drops while the robot works, a condition that guards the work, and a
// recovery action that refills it.
//
// THE BATTERY DRAINS IN THE WORLD, NOT IN THE QUESTION. simulateBattery
// below is called once per tick by whoever owns the tick loop -- the
// demo page, a panel, the smoke test -- and the condition only reads.
//
// The first version of this file drained the battery inside the
// condition, which was simpler and quietly destroyed the lesson: with
// memory, the condition is evaluated exactly once per traversal, so the
// battery drained once and then stayed put for the entire motion. The
// non-reactive tree therefore never had a flat battery to ignore, and
// the comparison it exists to demonstrate could not happen. A robot's
// battery drains whether or not anything asks about it, and the whole
// point of the non-reactive case is that nothing asks.
// #region task-battery
function taskBatteryAbove(threshold) {
    var node = btCondition('Battery above ' + threshold + '%?', function (blackboard) {
        var level = blackboard.battery === undefined ? 100 : blackboard.battery;
        return level > threshold;
    });
    node.reads = ['battery'];
    node.writes = [];
    return node;
}

// The world's clock, for the battery: called once per tick, outside the
// tree, before it is ticked.
function simulateBattery(blackboard, drainPerTick) {
    var drain = drainPerTick === undefined ? 0.6 : drainPerTick;
    if (blackboard.battery === undefined) blackboard.battery = 100;
    if (blackboard.charging) return;
    blackboard.battery = Math.max(0, blackboard.battery - drain);
}

// Charging sets blackboard.charging, which is what stops simulateBattery
// draining while the robot is on the dock. Clearing it in BOTH the
// success path and the halt path is not belt-and-braces: a charge that is
// abandoned half-way -- the fallback's higher-priority branch becoming
// viable again -- would otherwise leave the robot permanently plugged in
// as far as the simulation is concerned.
function taskCharge(ratePerTick) {
    var rate = ratePerTick === undefined ? 12 : ratePerTick;
    var node = btAction('Go charge', {
        start: function (blackboard) {
            blackboard.charging = true;
            blackboard.log.push('charge: battery at ' + blackboard.battery.toFixed(1) + '%, docking');
        },
        step: function (blackboard) {
            blackboard.battery = Math.min(100, blackboard.battery + rate);
            if (blackboard.battery < 100) return BT_RUNNING;
            blackboard.charging = false;
            blackboard.log.push('charge: full');
            return BT_SUCCESS;
        },
        halt: function (blackboard) {
            blackboard.charging = false;
            blackboard.log.push('charge: halted at ' + blackboard.battery.toFixed(1) + '%');
        }
    });
    node.reads = ['battery'];
    node.writes = ['battery', 'charging'];
    return node;
}
// #endregion task-battery
