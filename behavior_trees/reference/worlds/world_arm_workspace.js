/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Behavior Trees | World: arm workspace clutter

    Obstacle spheres sampled inside the ARM's own workspace, with every
    candidate rejected if it would put either endpoint of the planning
    query in collision.

    WHY THIS MODULE HAS ITS OWN WORLD, rather than using one of the
    fourteen in ../../../kineval/worlds/. Those are all ground-plane
    worlds for a mobile base: world_arm_clutter, the one closest to what
    this module needs, scatters eighteen spheres uniformly over a boundary
    that is `max(5, reach * 4)` metres to a side. With the base pinned --
    which is what plan_base_dofs: 'none' does, and what a joint trajectory
    requires, since a JointTrajectory cannot carry a base pose -- the arm
    reaches barely two metres, so almost none of those spheres is
    reachable, and measurement bears that out: mr2 planning from this
    module's start pose to its goal pose in world_arm_clutter solves in
    ONE iteration with a two-configuration smoothed path. A plan with no
    intermediate waypoints is a perfectly correct plan and teaches nothing
    whatever about trajectories, which are a statement about the waypoints
    in between.

    So this world samples where the arm actually is.

    REJECTION SAMPLING, which the upstream generators do not do. Every
    candidate sphere is tested against the robot at BOTH the start and the
    goal configuration -- inflated by a margin, so a sphere may not merely
    avoid them but must clear them -- and resampled if it touches either.
    That is a direct response to a defect recorded in
    ../../../kineval/motion_plan.js: the first version of
    world_sealed_enclosure sized its ring so that, for several robots, the
    start pose was already in collision with the enclosure, and the
    planner could not run at all. A generator that is handed the query it
    is generating for can simply refuse to produce that situation. The
    cost is that this world is defined relative to one query rather than
    standing alone, which is why the factory takes the robot and both
    configurations rather than only the {extent, reach} context the
    upstream generators take.

    Randomness comes from ../../../kineval/random.js's shared seeded
    generator, like the upstream generators, so a world is reproducible
    from its seed. Note that the planner RE-SEEDS that generator in
    robotRRTPlannerInit, so the search itself does not depend on how many
    draws world generation consumed.

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

// context: { robot, qmap, qStart, qGoal, reach, count, radius, margin }
//
// `reach` is robotReach at the start configuration -- how far the assembly
// actually spans from its base origin -- so the sampling box is the arm's
// own workspace and not the floor plan of a world built for a mobile base.
// #region create-world-arm-workspace
function createWorldArmWorkspace(context) {
    var robot = context.robot;
    var qmap = context.qmap;
    var reach = context.reach;
    var count = context.count !== undefined ? context.count : 24;
    var radius = context.radius !== undefined ? context.radius : 0.3;
    var margin = context.margin !== undefined ? context.margin : 0.08;

    // The boundary still has to contain the robot with room to spare: the
    // boundary test checks every link's bounding-box corners, so a tight
    // boundary is itself an obstacle. Two reaches a side is loose enough
    // that no reachable configuration touches it.
    var half = Math.max(5, reach * 2);
    var boundary = [[-half, 0, -half], [half, 0, half]];

    // The band of heights the arm occupies: above the base's own box,
    // below the arm fully extended upward.
    var lowY = 0.6;
    var highY = 2.4;

    var obstacles = [];
    var tries = 0;
    var maxTries = count * 400;

    while (obstacles.length < count && tries++ < maxTries) {
        var location = [
            [randomInRange(-reach, reach)],
            [randomInRange(lowY, highY)],
            [randomInRange(-reach, reach)],
            [1]
        ];

        // the rejection test, run against a one-sphere world so it can use
        // the planner's own collision test rather than a second
        // approximation of it
        var probe = {
            boundary: boundary,
            obstacles: [{ location: location, radius: radius + margin }]
        };
        if (poseIsCollision(robot, qmap, context.qStart, probe)) continue;
        if (poseIsCollision(robot, qmap, context.qGoal, probe)) continue;

        obstacles.push({ location: location, radius: radius });
    }

    if (obstacles.length < count) {
        console.warn('world_arm_workspace: placed only ' + obstacles.length +
            ' of ' + count + ' obstacles in ' + maxTries + ' tries; the two ' +
            'endpoint poses may be leaving too little free workspace');
    }

    return { boundary: boundary, obstacles: obstacles };
}
// #endregion create-world-arm-workspace
