/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | World: arm_clutter (generated)

    The world that forces planning in the FULL configuration space rather
    than in the ground plane.

    Every obstacle floats at arm height -- above the base's own bounding
    box and below the robot's full reach -- so the base can translate
    anywhere it likes while the manipulator cannot. A planner that samples
    only base_x/base_z/base_yaw and holds the joints fixed will fail here
    whenever the arm starts raised, and succeed whenever it starts tucked,
    which looks like flakiness and is actually a C-space that is too
    small.

    This is the world the plan_base_dofs: 'none' setting exists for:
    pin the base and plan the arm alone, and the same obstacles become a
    pure manipulator problem.

    The clutter is randomly placed from the shared seeded generator, so
    ?seed= gives a repeatable instance and a different seed gives a
    different difficulty.

    See world_narrow_passage.js's header for what the five parametric
    generators have in common: sizes derived from the robot's own extent,
    randomness from ../random.js, and the same (context) argument.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// #region create-world-arm-clutter
function createWorldArmClutter(context) {
    var extent = context.extent;
    var reach = context.reach;
    var half = Math.max(5, reach * 4);
    var boundary = [[-half, 0, -half], [half, 0, half]];

    // the band of heights the obstacles occupy: clear of the base's own
    // box underneath, within the arm's reach above
    var lowY = extent * 1.1;
    var highY = extent * 2.6 + reach * 0.4;
    var radius = extent * 0.45;
    var count = 18;

    var obstacles = [];
    var i;
    for (i = 0; i < count; i++) {
        obstacles.push({
            location: [
                [randomInRange(boundary[0][0], boundary[1][0])],
                [randomInRange(lowY, highY)],
                [randomInRange(boundary[0][2], boundary[1][2])],
                [1]
            ],
            radius: radius
        });
    }

    return { boundary: boundary, obstacles: obstacles };
}
// #endregion create-world-arm-clutter
