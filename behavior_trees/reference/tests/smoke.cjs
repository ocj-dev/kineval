/*  Behavior Trees | Reference-implementation smoke tests

    Loads every file in ../ into one shared Node vm context -- exactly as
    the buildless demo page loads them with plain <script src> tags, in
    the same order -- and asserts on real numbers.

    Run from the repository root:

        node behavior_trees/reference/tests/smoke.cjs

    WHAT THIS IS FOR. Every claim this module puts on a slide about what
    the code does is checked here first: the tick semantics of each node
    kind, the memory/reactive difference, that halt propagation actually
    cancels an action goal, that a trajectory's duration and point count
    are what the slides say, and that the snapshots in ../ still match
    their sources in kineval/. A slide that states a number is only as
    good as a test that reads it.

    @author ohseejay / https://github.com/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License
*/

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const HERE = __dirname;
const REFERENCE = path.resolve(HERE, '..');
const REPO = path.resolve(HERE, '../../..');

let failures = 0;
let checks = 0;

function ok(condition, label, detail) {
    checks++;
    if (condition) {
        console.log('  ok   ' + label + (detail ? '   [' + detail + ']' : ''));
    } else {
        failures++;
        console.log('  FAIL ' + label + (detail ? '   [' + detail + ']' : ''));
    }
}

function near(a, b, tol, label) {
    ok(Math.abs(a - b) <= tol, label, a.toFixed(4) + ' vs ' + b.toFixed(4));
}

function section(name) {
    console.log('\n' + name);
}

//////////////////////////////////////////////////
/////   LOAD
//////////////////////////////////////////////////

const ctx = vm.createContext({
    console: { log: () => {}, warn: () => {}, error: (...a) => { ctx.__errors.push(a.join(' ')); } },
    Math, Date, JSON, Array, Object, Number, isFinite, isNaN
});
ctx.__errors = [];

// the same order, and the same files, as the demo page's <script> tags
const SOURCES = [
    'matrix.js',
    'quaternion.js',
    'kinematics.js',
    'controls.js',
    'robots/urdf_example.js',
    'robots/robot_mr2.js',
    'behavior_tree.js',
    'joint_trajectory.js',
    'joint_trajectory_action.js',
    'bt_tasks.js',
    'trees.js'
];

for (const file of SOURCES) {
    vm.runInContext(fs.readFileSync(path.join(REFERENCE, file), 'utf8'), ctx, { filename: file });
}

const plan = JSON.parse(fs.readFileSync(path.join(REFERENCE, 'plans/mr2_arm_workspace.json'), 'utf8'));

//////////////////////////////////////////////////
/////   SNAPSHOT FRESHNESS
//////////////////////////////////////////////////

section('snapshots match their sources in kineval/');

for (const name of ['matrix', 'quaternion', 'kinematics', 'controls']) {
    const copy = fs.readFileSync(path.join(REFERENCE, name + '.js'), 'utf8');
    const source = fs.readFileSync(path.join(REPO, 'kineval', name + '.js'), 'utf8');
    // the snapshot is the source with one header block prepended, so the
    // source must be a suffix of the copy
    ok(copy.endsWith(source), name + '.js is its source plus a snapshot header',
        copy.length - source.length + ' bytes of header');
}

//////////////////////////////////////////////////
/////   TICK SEMANTICS
//////////////////////////////////////////////////

section('tick semantics, one node kind at a time');

function leaf(name, statuses) {
    // an action that returns the given statuses in order, then repeats
    // the last one
    let i = 0;
    const node = ctx.btAction(name, {
        step: () => statuses[Math.min(i++, statuses.length - 1)]
    });
    node.__reset = () => { i = 0; };
    return node;
}

{
    const bb = ctx.btBlackboard({});
    const seq = ctx.btSequence('seq', [
        leaf('a', [ctx.BT_SUCCESS]),
        leaf('b', [ctx.BT_SUCCESS])
    ]);
    ok(ctx.btTickTree(seq, bb).status === ctx.BT_SUCCESS, 'sequence of two successes -> Success');
}
{
    const bb = ctx.btBlackboard({});
    const never = leaf('never', [ctx.BT_SUCCESS]);
    const seq = ctx.btSequence('seq', [leaf('a', [ctx.BT_FAILURE]), never]);
    const run = ctx.btTickTree(seq, bb);
    ok(run.status === ctx.BT_FAILURE, 'sequence stops at the first Failure');
    ok(run.trace.filter((e) => e.name === 'never').length === 0,
        'children after a Failure are never ticked', run.trace.map((e) => e.name).join(' '));
}
{
    const bb = ctx.btBlackboard({});
    const fb = ctx.btFallback('fb', [leaf('a', [ctx.BT_FAILURE]), leaf('b', [ctx.BT_SUCCESS])]);
    ok(ctx.btTickTree(fb, bb).status === ctx.BT_SUCCESS,
        'fallback recovers: first child fails, second succeeds');
}
{
    const bb = ctx.btBlackboard({});
    const fb = ctx.btFallback('fb', [leaf('a', [ctx.BT_FAILURE]), leaf('b', [ctx.BT_FAILURE])]);
    ok(ctx.btTickTree(fb, bb).status === ctx.BT_FAILURE, 'fallback with every child failing -> Failure');
}
{
    // parallel, 1-of-2 and 2-of-2 over the same children
    const bb = ctx.btBlackboard({});
    const any = ctx.btParallel('any', [leaf('a', [ctx.BT_SUCCESS]), leaf('b', [ctx.BT_FAILURE])], 1);
    ok(ctx.btTickTree(any, bb).status === ctx.BT_SUCCESS, 'parallel 1-of-2 with one success -> Success');

    const all = ctx.btParallel('all', [leaf('a', [ctx.BT_SUCCESS]), leaf('b', [ctx.BT_FAILURE])], 2);
    ok(ctx.btTickTree(all, ctx.btBlackboard({})).status === ctx.BT_FAILURE,
        'parallel 2-of-2 with one failure -> Failure');

    const both = ctx.btParallel('both', [leaf('a', [ctx.BT_SUCCESS]), leaf('b', [ctx.BT_SUCCESS])], 2);
    const run = ctx.btTickTree(both, ctx.btBlackboard({}));
    ok(run.trace.filter((e) => e.kind === 'action').length === 2,
        'parallel ticks every child in one tick, with no early exit');
}
{
    const bb = ctx.btBlackboard({});
    const inv = ctx.btDecorator('inv', 'invert', leaf('a', [ctx.BT_SUCCESS]));
    ok(ctx.btTickTree(inv, bb).status === ctx.BT_FAILURE, 'invert decorator turns Success into Failure');

    const force = ctx.btDecorator('force', 'force-success', leaf('a', [ctx.BT_FAILURE]));
    ok(ctx.btTickTree(force, ctx.btBlackboard({})).status === ctx.BT_SUCCESS,
        'force-success decorator turns Failure into Success');
}
{
    // the attempt limit is what stops a repeat decorator that can never succeed
    const bb = ctx.btBlackboard({});
    const rep = ctx.btDecorator('rep', 'repeat-until-success', leaf('a', [ctx.BT_FAILURE]), { limit: 3 });
    const statuses = [];
    for (let i = 0; i < 4; i++) statuses.push(ctx.btTickTree(rep, bb).status);
    ok(statuses.slice(0, 2).every((s) => s === ctx.BT_RUNNING) && statuses[2] === ctx.BT_FAILURE,
        'repeat-until-success returns Running until its limit, then Failure', statuses.join(' '));
}
{
    // a condition that returns Running is a category error and is reported
    ctx.__errors.length = 0;
    const bad = ctx.btCondition('bad', () => ctx.BT_RUNNING);
    ok(ctx.btTickTree(bad, ctx.btBlackboard({})).status === ctx.BT_FAILURE &&
        ctx.__errors.some((e) => e.indexOf('must resolve within one tick') !== -1),
        'a condition returning Running is rejected and reported');
}

//////////////////////////////////////////////////
/////   MEMORY, AND HALTING
//////////////////////////////////////////////////

section('memory vs reactive, and halt propagation');

function guardedTree(memory) {
    let guard = true;
    let halted = 0;
    let steps = 0;
    const condition = ctx.btCondition('guard', () => guard);
    const action = ctx.btAction('long action', {
        step: () => { steps++; return ctx.BT_RUNNING; },
        halt: () => { halted++; }
    });
    const seq = ctx.btSequence('seq', [condition, action], { memory: memory });
    return {
        seq: seq,
        drop: () => { guard = false; },
        halted: () => halted,
        steps: () => steps
    };
}

{
    const t = guardedTree(true);
    const bb = ctx.btBlackboard({});
    ctx.btTickTree(t.seq, bb);
    t.drop();
    const after = ctx.btTickTree(t.seq, bb).status;
    ok(after === ctx.BT_RUNNING, 'WITH memory: the guard is not re-evaluated, the action keeps running');
    ok(t.halted() === 0, 'WITH memory: nothing is halted', 'halts=' + t.halted());
    ok(t.steps() === 2, 'WITH memory: the action was stepped on both ticks', 'steps=' + t.steps());
}
{
    const t = guardedTree(false);
    const bb = ctx.btBlackboard({});
    ctx.btTickTree(t.seq, bb);
    t.drop();
    const after = ctx.btTickTree(t.seq, bb).status;
    ok(after === ctx.BT_FAILURE, 'WITHOUT memory: the guard is re-evaluated and the sequence fails');
    ok(t.halted() === 1, 'WITHOUT memory: the running action is halted exactly once',
        'halts=' + t.halted());
}

//////////////////////////////////////////////////
/////   BLACKBOARD VALIDATION
//////////////////////////////////////////////////

section('blackboard declarations are checked before the first tick');

{
    const orphan = ctx.btCondition('reads a key nobody writes', () => true);
    orphan.reads = ['nothing_writes_this'];
    const problems = ctx.btBlackboardRequire(orphan, ctx.btBlackboard({}));
    ok(problems.length === 1 && problems[0].key === 'nothing_writes_this',
        'a leaf reading an unwritten key is reported', problems.length + ' problem(s)');
}

//////////////////////////////////////////////////
/////   THE PLAN, AND THE TRAJECTORY
//////////////////////////////////////////////////

section('the committed plan');

ok(plan.search.iterations === 46, 'the plan took 46 RRT-Connect iterations',
    String(plan.search.iterations));
ok(plan.search.raw_path_length === 18 && plan.search.smoothed_path_length === 5,
    'raw path 18 configurations, smoothed 5',
    plan.search.raw_path_length + ' -> ' + plan.search.smoothed_path_length);
ok(plan.world.obstacles.length === 24, 'the world has 24 obstacles', String(plan.world.obstacles.length));
ok(ctx.baseIsStationary(plan.smoothed_path, plan.q_layout),
    'the plan holds the base still, so it is expressible as a JointTrajectory');

section('plan -> JointTrajectory');

const trajectory = ctx.trajectoryFromPlan(plan, plan.joint_names);
ok(trajectory !== null, 'a trajectory was built');
ok(ctx.validateTrajectory(trajectory).length === 0, 'the trajectory validates clean',
    ctx.validateTrajectory(trajectory).join('; '));
ok(trajectory.points.length === 5, 'five waypoints, one per smoothed configuration',
    String(trajectory.points.length));
ok(trajectory.joint_names.length === 4, 'four joint names', trajectory.joint_names.join(', '));
ok(trajectory.points[0].time_from_start === 0, 'the first point is at t = 0');

const duration = ctx.trajectoryDuration(trajectory);
ok(duration > 0, 'the trajectory has a positive duration', duration.toFixed(3) + 's');

// the duration must equal the sum of (largest per-joint displacement /
// cruise speed) over the segments -- the time parameterization, checked
// rather than trusted
{
    let expected = 0;
    for (let i = 1; i < plan.smoothed_positions.length; i++) {
        let largest = 0;
        for (let j = 0; j < plan.smoothed_positions[i].length; j++) {
            largest = Math.max(largest, Math.abs(plan.smoothed_positions[i][j] - plan.smoothed_positions[i - 1][j]));
        }
        expected += Math.max(largest / ctx.TRAJECTORY_MAX_JOINT_VELOCITY, ctx.TRAJECTORY_MIN_SEGMENT_SECONDS);
    }
    near(duration, expected, 1e-9, 'duration = sum of segment durations under the stated parameterization');
}

// monotonic times, and endpoints with zero velocity
{
    let monotonic = true;
    for (let i = 1; i < trajectory.points.length; i++) {
        if (trajectory.points[i].time_from_start <= trajectory.points[i - 1].time_from_start) monotonic = false;
    }
    ok(monotonic, 'time_from_start increases at every point');
    ok(trajectory.points[0].velocities.every((v) => v === 0) &&
        trajectory.points[4].velocities.every((v) => v === 0),
        'the first and last points have zero velocity');
}

// sampling reproduces the waypoints exactly, which is what makes the
// interpolation the same straight line the planner validated
{
    let exact = true;
    for (const point of trajectory.points) {
        const sample = ctx.sampleTrajectory(trajectory, point.time_from_start);
        for (let j = 0; j < point.positions.length; j++) {
            if (Math.abs(sample.positions[j] - point.positions[j]) > 1e-9) exact = false;
        }
    }
    ok(exact, 'sampling at a waypoint time returns that waypoint exactly');

    const before = ctx.sampleTrajectory(trajectory, -5);
    const after = ctx.sampleTrajectory(trajectory, duration + 5);
    ok(before.positions.every((v, j) => v === trajectory.points[0].positions[j]),
        'sampling before the start clamps to the first point');
    ok(after.positions.every((v, j) => v === trajectory.points[4].positions[j]),
        'sampling after the end clamps to the last point');

    const mid = ctx.sampleTrajectory(trajectory, trajectory.points[1].time_from_start * 0.5);
    ok(mid.segment === 0 && Math.abs(mid.alpha - 0.5) < 1e-9,
        'sampling halfway through the first segment reports segment 0, alpha 0.5');
}

// the joints a trajectory says nothing about
{
    const robot = ctx.createRobotMr2();
    ctx.initRobot(robot);
    const unnamed = ctx.trajectoryJointsNotNamed(trajectory, robot);
    ok(unnamed.length === 1 && unnamed[0] === 'clavicle_left_roll',
        'the trajectory names four of mr2\'s five joints; clavicle_left_roll is unmentioned',
        unnamed.join(', '));
}

// a trajectory built from a plan whose base moves must be refused
{
    ctx.__errors.length = 0;
    const moved = JSON.parse(JSON.stringify(plan));
    moved.smoothed_path[2][0] += 0.5;   // base_x
    const refused = ctx.trajectoryFromPlan(moved, plan.joint_names);
    ok(refused === null && ctx.__errors.some((e) => e.indexOf('no field that can carry a base pose') !== -1),
        'a plan whose base moves is refused, with the reason named');
}

//////////////////////////////////////////////////
/////   THE ACTION
//////////////////////////////////////////////////

section('the joint-trajectory action');

function freshRobot() {
    const robot = ctx.createRobotMr2();
    ctx.initRobot(robot);
    const start = plan.smoothed_positions[0];
    for (let j = 0; j < plan.joint_names.length; j++) {
        robot.joints[plan.joint_names[j]].angle = start[j];
    }
    return robot;
}

const DT = 1 / 20;

{
    const server = ctx.jointTrajectoryActionServer(freshRobot());
    const reply = ctx.jointTrajectoryActionSendGoal(server, { trajectory: trajectory });
    ok(reply.accepted === true, 'a valid goal is accepted');

    let cycles = 0;
    let worst = 0;
    while ((server.state === ctx.JTA_EXECUTING || server.state === ctx.JTA_ACCEPTED) && cycles < 10000) {
        ctx.jointTrajectoryActionUpdate(server, DT);
        if (server.feedback) worst = Math.max(worst, server.feedback.worst_error);
        cycles++;
    }
    ok(server.state === ctx.JTA_SUCCEEDED, 'the goal succeeds', server.state);
    ok(server.result.error_code === ctx.JTA_SUCCESSFUL, 'result error_code is SUCCESSFUL (0)',
        String(server.result.error_code));
    ok(worst > 0, 'the tracking error is nonzero -- this is a servo, not a teleport',
        'worst error ' + worst.toFixed(4) + ' rad');
    ok(worst < ctx.JTA_PATH_TOLERANCE, 'the worst tracking error stays inside the path tolerance',
        worst.toFixed(4) + ' < ' + ctx.JTA_PATH_TOLERANCE);
    ok(server.result.worst_error <= ctx.JTA_GOAL_TOLERANCE,
        'the final error is inside the goal tolerance',
        server.result.worst_error.toFixed(5) + ' <= ' + ctx.JTA_GOAL_TOLERANCE);
    console.log('       (took ' + cycles + ' cycles = ' + (cycles * DT).toFixed(2) +
        's of simulated time for a ' + duration.toFixed(2) + 's trajectory)');
}
{
    // a goal naming a joint the robot does not have
    const server = ctx.jointTrajectoryActionServer(freshRobot());
    const wrong = JSON.parse(JSON.stringify(trajectory));
    wrong.joint_names[0] = 'elbow_of_some_other_robot';
    const reply = ctx.jointTrajectoryActionSendGoal(server, { trajectory: wrong });
    ok(reply.accepted === false && reply.result.error_code === ctx.JTA_INVALID_JOINTS,
        'a goal naming an unknown joint is rejected with INVALID_JOINTS (-2)',
        String(reply.result.error_code));
}
{
    // a malformed trajectory
    const server = ctx.jointTrajectoryActionServer(freshRobot());
    const broken = JSON.parse(JSON.stringify(trajectory));
    broken.points[2].time_from_start = broken.points[1].time_from_start;
    const reply = ctx.jointTrajectoryActionSendGoal(server, { trajectory: broken });
    ok(reply.accepted === false && reply.result.error_code === ctx.JTA_INVALID_GOAL,
        'a non-monotonic trajectory is rejected with INVALID_GOAL (-1)',
        String(reply.result.error_code));
}
{
    // cancelling mid-motion stops the arm where it is
    const robot = freshRobot();
    const server = ctx.jointTrajectoryActionServer(robot);
    ctx.jointTrajectoryActionSendGoal(server, { trajectory: trajectory });
    for (let i = 0; i < 20; i++) ctx.jointTrajectoryActionUpdate(server, DT);

    const angleAtCancel = robot.joints[plan.joint_names[0]].angle;
    const result = ctx.jointTrajectoryActionCancel(server, 'smoke test');
    ok(server.state === ctx.JTA_CANCELED, 'the goal is CANCELED', server.state);
    ok(result.error_code === ctx.JTA_SUCCESSFUL, 'a cancel is not an error');

    for (let i = 0; i < 40; i++) ctx.jointTrajectoryActionUpdate(server, DT);
    ok(robot.joints[plan.joint_names[0]].angle === angleAtCancel,
        'updates after a cancel do not move the arm');
    ok(result.canceled_at_time > 0 && result.canceled_at_segment >= 0,
        'the cancel records when and where it happened',
        't=' + result.canceled_at_time.toFixed(2) + ' segment ' + result.canceled_at_segment);
}
{
    // a second goal preempts the first
    const server = ctx.jointTrajectoryActionServer(freshRobot());
    ctx.jointTrajectoryActionSendGoal(server, { trajectory: trajectory });
    for (let i = 0; i < 10; i++) ctx.jointTrajectoryActionUpdate(server, DT);
    const reply = ctx.jointTrajectoryActionSendGoal(server, { trajectory: trajectory });
    ok(reply.accepted === true && server.stats.canceled === 1 && server.time === 0,
        'a new goal preempts the running one and restarts the clock',
        'canceled=' + server.stats.canceled);
}
{
    // a path tolerance that cannot be met aborts the goal
    const server = ctx.jointTrajectoryActionServer(freshRobot());
    ctx.jointTrajectoryActionSendGoal(server, { trajectory: trajectory, path_tolerance: 0.001 });
    let cycles = 0;
    while ((server.state === ctx.JTA_EXECUTING || server.state === ctx.JTA_ACCEPTED) && cycles++ < 1000) {
        ctx.jointTrajectoryActionUpdate(server, DT);
    }
    ok(server.state === ctx.JTA_ABORTED &&
        server.result.error_code === ctx.JTA_PATH_TOLERANCE_VIOLATED,
        'an unmeetable path tolerance aborts with PATH_TOLERANCE_VIOLATED (-4)',
        String(server.result.error_code));
    ok(/clavicle_right_yaw|shoulder_right_yaw|upperarm_right_pitch|forearm_right_yaw/
        .test(server.result.error_string), 'the abort message names the offending joint');
}
{
    // a goal-time tolerance of zero, with a gain too low to converge in
    // time, reports GOAL_TOLERANCE_VIOLATED
    const server = ctx.jointTrajectoryActionServer(freshRobot(), { gain: 0.02 });
    ctx.jointTrajectoryActionSendGoal(server, {
        trajectory: trajectory, path_tolerance: 99, goal_time_tolerance: 0
    });
    let cycles = 0;
    while ((server.state === ctx.JTA_EXECUTING || server.state === ctx.JTA_ACCEPTED) && cycles++ < 10000) {
        ctx.jointTrajectoryActionUpdate(server, DT);
    }
    ok(server.state === ctx.JTA_ABORTED &&
        server.result.error_code === ctx.JTA_GOAL_TOLERANCE_VIOLATED,
        'failing to converge in time reports GOAL_TOLERANCE_VIOLATED (-5)',
        String(server.result.error_code));
}

//////////////////////////////////////////////////
/////   THE TREES
//////////////////////////////////////////////////

section('the search trees');

function searchBlackboard(extra) {
    const bb = ctx.btBlackboard({
        robot_location: 'hallway',
        objects_at: { kitchen: ['orange'], pantry: ['apple'], garage: [] },
        wanted_object: 'apple'
    });
    for (const key in (extra || {})) bb[key] = extra[key];
    return bb;
}

function runTree(tree, bb, maxTicks, worldUpdate) {
    const ticks = [];
    let status = ctx.BT_RUNNING;
    for (let i = 0; i < (maxTicks || 200) && status === ctx.BT_RUNNING; i++) {
        bb.dt = DT;
        bb.now += DT;
        if (worldUpdate) worldUpdate(bb);
        const run = ctx.btTickTree(tree, bb);
        status = run.status;
        ticks.push(run);
    }
    return { status: status, ticks: ticks };
}

{
    // the apple is in the pantry, so searching only the kitchen fails
    const run = runTree(ctx.treeSearchNaive(), searchBlackboard());
    ok(run.status === ctx.BT_FAILURE, 'the naive kitchen-only search fails: the apple is in the pantry');
    ok(run.ticks.length === ctx.SEARCH_NAVIGATION_TICKS,
        'it took ' + ctx.SEARCH_NAVIGATION_TICKS + ' ticks -- the navigation spans them',
        run.ticks.length + ' ticks');
}
{
    // already in the kitchen: the explicit success condition skips the drive
    const bb = searchBlackboard({ robot_location: 'kitchen', objects_at: { kitchen: ['apple'] } });
    const run = runTree(ctx.treeSearchExplicitSuccess(), bb);
    ok(run.status === ctx.BT_SUCCESS && run.ticks.length === 1,
        'check-before-you-act finds the apple in one tick when already in the kitchen',
        run.ticks.length + ' tick(s)');
    ok(!bb.log.some((l) => l.indexOf('heading for') !== -1),
        'the navigation action was never started');
}
{
    const bb = searchBlackboard();
    const run = runTree(ctx.treeSearchMultipleLocations(), bb);
    ok(run.status === ctx.BT_SUCCESS, 'the three-location fallback finds the apple in the pantry');
    ok(bb.found && bb.found.indexOf('apple') !== -1, 'and records it on the blackboard');
}
{
    // 1-of-2 succeeds (the kitchen has an orange); 2-of-2 does not
    const any = runTree(ctx.treeSearchParallel(1),
        searchBlackboard({ robot_location: 'kitchen' }));
    ok(any.status === ctx.BT_SUCCESS, 'parallel 1-of-2 succeeds on the orange alone');

    const all = runTree(ctx.treeSearchParallel(2),
        searchBlackboard({ robot_location: 'kitchen' }));
    ok(all.status === ctx.BT_FAILURE, 'parallel 2-of-2 fails without the apple');
}
{
    const bb = searchBlackboard({ location_queue: ['kitchen', 'pantry', 'garage'] });
    const run = runTree(ctx.treeSearchWithDecorator(3), bb);
    ok(run.status === ctx.BT_SUCCESS,
        'the refactored tree -- one subtree, a queue, a decorator -- finds the apple');
    ok(bb.location_queue.length === 1,
        'it stopped at the pantry with the garage still in the queue',
        'queue left: ' + JSON.stringify(bb.location_queue));
}

section('the motion trees');

{
    const robot = freshRobot();
    const server = ctx.jointTrajectoryActionServer(robot);
    const tree = ctx.treePlanAndExecute(server, plan, plan.joint_names);

    const bb = ctx.btBlackboard({});
    ok(ctx.btBlackboardRequire(tree, bb).length === 0,
        'the plan-and-execute tree passes blackboard validation');

    const run = runTree(tree, bb, 2000);
    ok(run.status === ctx.BT_SUCCESS, 'plan, then execute, succeeds', run.status);
    ok(bb.trajectory && bb.trajectory.points.length === 5,
        'the planner leaf left a five-point trajectory on the blackboard');
    ok(server.stats.succeeded === 1, 'the action server reports exactly one succeeded goal',
        JSON.stringify(server.stats));

    const last = bb.trajectory.points[4].positions;
    ok(ctx.robotAtConfiguration(robot, bb.trajectory.joint_names, last),
        'and the robot is at the plan\'s final configuration');

    // ticked again, the fallback's condition short-circuits the whole thing
    const before = server.stats.goals;
    bb.dt = DT;
    ok(ctx.btTickTree(tree, bb).status === ctx.BT_SUCCESS && server.stats.goals === before,
        're-ticking an arrived tree sends no new goal -- check before you act');
}
{
    // the longer-horizon tree: three inspection poses, the part is at the
    // third, so it executes three motions and stops
    const robot = freshRobot();
    const server = ctx.jointTrajectoryActionServer(robot);

    const poses = {
        bin_a: [0.4, 0.1, 0.5, 0.0],
        bin_b: [0.9, -0.3, 0.9, 0.4],
        bin_c: [1.1, -0.8, 1.2, 0.9]
    };
    const trajectories = {};
    const names = plan.joint_names;
    let fromAngles = plan.smoothed_positions[0];
    for (const target in poses) {
        trajectories[target] = ctx.trajectoryFromPlan({
            q_layout: names,
            smoothed_path: [fromAngles, poses[target]],
            joint_names: names,
            provenance: { generated_by: 'smoke test two-point hop' },
            robot: 'mr2', seed: 0
        }, names);
        fromAngles = poses[target];
    }

    const tree = ctx.treeInspectionQueue(server, trajectories);
    const bb = ctx.btBlackboard({ parts_present: { bin_a: false, bin_b: false, bin_c: true } });
    ok(ctx.btBlackboardRequire(tree, bb).length === 0,
        'the inspection-queue tree passes blackboard validation');

    const run = runTree(tree, bb, 5000);
    ok(run.status === ctx.BT_SUCCESS, 'the inspection task succeeds at the third target', run.status);
    ok(JSON.stringify(bb.inspected) === JSON.stringify(['bin_a', 'bin_b', 'bin_c']),
        'all three targets were visited in queue order', JSON.stringify(bb.inspected));
    ok(server.stats.succeeded === 3, 'three trajectories were executed',
        JSON.stringify(server.stats));
}
{
    // reactivity: the same tree with and without memory, and the battery
    // going flat mid-motion
    for (const reactive of [false, true]) {
        const robot = freshRobot();
        const server = ctx.jointTrajectoryActionServer(robot);
        const tree = ctx.treeReactiveBattery(server, plan, plan.joint_names, { reactive: reactive });
        const bb = ctx.btBlackboard({ battery: 30 });
        // the battery drains in the world, once per tick, whether or not
        // the tree asks about it
        const run = runTree(tree, bb, 3000, (blackboard) => ctx.simulateBattery(blackboard));

        if (reactive) {
            ok(server.stats.canceled >= 1,
                'REACTIVE: the flat battery cancels the motion mid-flight',
                'canceled=' + server.stats.canceled + ' succeeded=' + server.stats.succeeded);
            ok(bb.battery > 20, 'REACTIVE: and the robot charged instead',
                'battery=' + bb.battery.toFixed(1) + '%');
        } else {
            ok(server.stats.canceled === 0 && server.stats.succeeded === 1,
                'NON-REACTIVE: the same tree finishes the motion on a flat battery',
                'canceled=' + server.stats.canceled + ' succeeded=' + server.stats.succeeded);
            ok(bb.battery <= 20, 'NON-REACTIVE: the battery went below the threshold unnoticed',
                'battery=' + bb.battery.toFixed(1) + '%');
        }
        if (run.status === ctx.BT_RUNNING) {
            console.log('       (tree still running after the tick budget, which is fine: ' +
                'the charging branch repeats)');
        }
    }
}

//////////////////////////////////////////////////

console.log('\n' + (failures ? 'FAILED' : 'passed') + ': ' +
    (checks - failures) + '/' + checks + ' checks');
if (ctx.__errors.length) {
    console.log('\nconsole.error output captured during the run (expected, from the ' +
        'refusal paths above):');
    for (const e of ctx.__errors) console.log('  - ' + e.slice(0, 110));
}
process.exitCode = failures ? 1 : 0;
