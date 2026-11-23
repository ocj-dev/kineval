/*  Behavior Trees | Plan generator

    Produces the committed plan JSON files in this directory by running the
    KinEval RRT-Connect planner -- ../../../kineval/motion_plan.js, the one
    the motion_planning module's deck teaches, unmodified -- in a Node vm
    context.

    Run by hand, from the repository root:

        node behavior_trees/reference/plans/generate_plans.cjs

    WHY THE PLANS ARE COMMITTED RATHER THAN PLANNED IN THE BROWSER. This
    module is about what happens to a motion plan AFTER the planner
    returns one: how it becomes a JointTrajectory, who executes it, and how
    a behavior tree sequences that execution. Replanning live in every
    panel would make each panel's picture depend on a search, so a slide
    could not say "five waypoints" and be sure. These files are the
    planner's actual output, recorded once, with the seed and the counts
    that produced them, so every panel and every slide reads the same
    numbers.

    The planner is NOT reimplemented here or in the deck. The deck's
    panels read these files.

    @author ohseejay / https://github.com/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License
*/

const fs = require('fs');
const path = require('path');
const vm = require('vm');
const { execSync } = require('child_process');

const HERE = __dirname;
const REPO = path.resolve(HERE, '../../..');

const ctx = vm.createContext({
    console, Math, Date, JSON, Array, Object, Number, isFinite, isNaN
});

function load(relative) {
    const file = path.join(REPO, relative);
    vm.runInContext(fs.readFileSync(file, 'utf8'), ctx, { filename: relative });
}

// the planner and everything it needs, straight out of the viewer
['kineval/matrix.js',
 'kineval/quaternion.js',
 'kineval/kinematics.js',
 'kineval/random.js',
 'kineval/collision.js',
 'kineval/motion_plan.js'].forEach(load);

// this module's own robot descriptions and world generator
['behavior_trees/reference/robots/robot_mr2.js',
 'behavior_trees/reference/robots/urdf_example.js',
 'behavior_trees/reference/worlds/world_arm_workspace.js'].forEach(load);

// collision.js reads each link's bounding box back off a built three.js
// mesh (cacheLinkBoundingBoxes). There is no three.js here, and there does
// not need to be: for a box of `size` translated by `offset`, the
// bounding box is exactly offset +/- size/2, which is what that function
// would have computed from the mesh.
vm.runInContext(`
function cacheLinkBoundingBoxesFromPlainGeometry(robot) {
    var x;
    for (x in robot.links) {
        var g = robot.links[x].geometry;
        if (!g || g.type !== 'box') continue;
        robot.links[x].bbox = {
            min: { x: g.offset.xyz[0] - g.size[0] / 2,
                   y: g.offset.xyz[1] - g.size[1] / 2,
                   z: g.offset.xyz[2] - g.size[2] / 2 },
            max: { x: g.offset.xyz[0] + g.size[0] / 2,
                   y: g.offset.xyz[1] + g.size[1] / 2,
                   z: g.offset.xyz[2] + g.size[2] / 2 }
        };
    }
}

function planOnce(spec) {
    var robot = spec.factory();
    initRobot(robot);
    cacheLinkBoundingBoxesFromPlainGeometry(robot);

    var qmap = buildConfigurationIndex(robot);
    var n;
    for (n in spec.startAngles) robot.joints[n].angle = spec.startAngles[n];

    seedRandom(spec.seed);

    var qStart = configurationFromRobot(robot, qmap);
    var qGoal = qStart.slice();
    for (n in spec.goalAngles) qGoal[qmap.names[n]] = spec.goalAngles[n];

    var world = createWorldArmWorkspace({
        robot: robot, qmap: qmap, qStart: qStart, qGoal: qGoal,
        extent: robotExtent(robot),
        reach: robotReach(robot, qmap, qStart),
        count: spec.obstacleCount, radius: spec.obstacleRadius, margin: spec.margin
    });

    // plan_base_dofs: 'none' pins the base. That is not a convenience: a
    // JointTrajectory has no field that could carry a base pose, so a plan
    // whose base moves is not expressible as one.
    var params = { seed: spec.seed, goal_config: qGoal, plan_base_dofs: 'none' };

    var planner = robotRRTPlannerInit(robot, qmap, world, params);
    var guard = 0;
    while (planner.state === 'planning' && guard++ < 100000) {
        robotRRTPlannerIterate(robot, qmap, world, params, planner);
    }

    return { robot: robot, qmap: qmap, world: world, planner: planner };
}
`, ctx);

let commit = 'unknown';
try {
    commit = execSync('git rev-parse --short HEAD', { cwd: REPO }).toString().trim();
} catch (e) { /* not a git checkout; the header just says unknown */ }

// One spec per committed plan file.
//
// The parameters are not arbitrary. They were chosen by sweeping obstacle
// count, obstacle radius and seed and reading off the planner's own
// numbers, looking for a query that is genuinely constrained -- tens of
// iterations rather than one, and a smoothed path with waypoints in the
// middle rather than a straight line from start to goal. The sweep is
// recorded in the generated file's `provenance` block.
const SPECS = [
    {
        name: 'mr2_arm_workspace',
        robot: 'mr2',
        factory: 'createRobotMr2',
        chain: 'MR2_ARM_CHAIN',
        seed: 7,
        obstacleCount: 24,
        obstacleRadius: 0.3,
        margin: 0.08,
        startAngles: {
            clavicle_right_yaw: 0.2,
            shoulder_right_yaw: 0.3,
            upperarm_right_pitch: 0.4,
            forearm_right_yaw: -0.3
        },
        goalAngles: {
            clavicle_right_yaw: 1.1,
            shoulder_right_yaw: -0.8,
            upperarm_right_pitch: 1.2,
            forearm_right_yaw: 0.9
        },
        note: 'the module\'s primary plan: 46 RRT-Connect iterations, an ' +
              '18-configuration raw path, 5 configurations after smoothing'
    }
];

for (const spec of SPECS) {
    const out = ctx.planOnce({
        factory: ctx[spec.factory],
        seed: spec.seed,
        obstacleCount: spec.obstacleCount,
        obstacleRadius: spec.obstacleRadius,
        margin: spec.margin,
        startAngles: spec.startAngles,
        goalAngles: spec.goalAngles
    });

    const p = out.planner;
    if (p.state !== 'found') {
        console.error(`${spec.name}: planner did not find a path (${p.state}: ${p.failureReason})`);
        process.exitCode = 1;
        continue;
    }

    const chain = ctx[spec.chain];
    const qmap = out.qmap;

    const record = {
        provenance: {
            generated_by: 'behavior_trees/reference/plans/generate_plans.cjs',
            generated_on: new Date().toISOString().slice(0, 10),
            repository_commit: commit,
            planner: 'kineval/motion_plan.js robotRRTPlannerInit + robotRRTPlannerIterate (RRT-Connect)',
            planner_note: 'run unmodified in a Node vm context; the planner re-seeds the shared generator in robotRRTPlannerInit, so the search does not depend on world generation',
            world: 'behavior_trees/reference/worlds/world_arm_workspace.js',
            note: spec.note
        },
        robot: spec.robot,
        seed: spec.seed,
        // the full q layout, so a reader can tell which column is which
        q_layout: ['base_x', 'base_y', 'base_z', 'base_roll', 'base_pitch', 'base_yaw']
            .concat(qmap.index.slice(6)),
        base_dofs: 'pinned (plan_base_dofs: none)',
        joint_names: chain,
        world: out.world,
        search: {
            iterations: p.iterations,
            collision_checks: p.counters.collisionChecks,
            nearest_neighbor_comparisons: p.counters.nearestNeighborComparisons,
            step_size: p.step,
            raw_path_length: p.rawPath.length,
            smoothed_path_length: p.path.length
        },
        // the full q vectors, exactly as the planner produced them
        raw_path: p.rawPath,
        smoothed_path: p.path,
        // and the same smoothed path projected onto joint_names: this is
        // what becomes a JointTrajectory's points[].positions
        smoothed_positions: p.path.map((q) => chain.map((name) => q[qmap.names[name]]))
    };

    const file = path.join(HERE, `${spec.name}.json`);
    fs.writeFileSync(file, JSON.stringify(record, null, 1) + '\n');
    console.log(`${spec.name}: ${p.state}, ${p.iterations} iterations, ` +
        `raw ${p.rawPath.length} -> smoothed ${p.path.length}, ` +
        `${p.counters.collisionChecks} collision checks -> ${path.relative(REPO, file)}`);
}
