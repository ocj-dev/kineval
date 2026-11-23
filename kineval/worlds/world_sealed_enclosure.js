/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | World: sealed_enclosure (generated)

    THE TERMINATION CASE. A closed ring of spheres around the start, with
    no gap anywhere. No path to anywhere outside it exists.

    This world cannot be solved, and that is the point. RRT-Connect is
    probabilistically complete, which says what happens when a solution
    EXISTS and says nothing at all when one does not -- the planner will
    sample forever without ever proving impossibility, because it has no
    way to prove impossibility. The only thing standing between a student
    and a locked browser tab is the iteration cap.

    So the requirement this world enforces (Phase 4 asserts it, on
    wall-clock as well as on the reported state) is: terminate, report
    state 'failed', leave the viewer responsive. Never hang.

    The ring is built with overlapping spheres and then checked for seams
    rather than assumed sealed -- a ring with an accidental gap is a
    perfectly ordinary world and would silently stop testing the thing
    this world exists to test.

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

// #region create-world-sealed-enclosure
function createWorldSealedEnclosure(context) {
    var extent = context.extent;
    // sized by the robot's whole REACH, not its widest link: the ring has
    // to be one the robot can stand inside without already touching it,
    // and on a robot like the crawler the legs span far more than the
    // base box does. See collision.js#robotReach.
    var reach = context.reach;
    var half = Math.max(6, reach * 5);
    var boundary = [[-half, 0, -half], [half, 0, half]];

    var ringRadius = reach * 1.8;
    var sphereRadius = extent * 0.6;

    // choose the sphere count so that the gap between adjacent centres is
    // comfortably less than two sphere radii -- i.e. consecutive spheres
    // overlap and the ring has no seam. Derived rather than guessed: the
    // chord between adjacent centres on a circle of radius R with n
    // spheres is 2*R*sin(pi/n), and we want that below 1.2*sphereRadius.
    var count = Math.ceil(Math.PI / Math.asin(Math.min(0.999, (1.2 * sphereRadius) / (2 * ringRadius))));

    // STACKED to above the robot's own height. A single ring of spheres
    // seals the ground plane and nothing else, and obstacle collision is
    // fully 3D -- so a robot whose links sit above a knee-high ring walks
    // straight over it. The first version of this world did exactly that,
    // and the Phase 4 termination test duly reported a path out of a
    // "sealed" enclosure. See collision.js#robotHeight.
    var wallTop = context.height * 1.5 + sphereRadius;

    var obstacles = [];
    var i, y;
    for (y = sphereRadius; y <= wallTop; y += sphereRadius * 1.4) {
        for (i = 0; i < count; i++) {
            var theta = (2 * Math.PI * i) / count;
            obstacles.push({
                location: [
                    [ringRadius * Math.cos(theta)],
                    [y],
                    [ringRadius * Math.sin(theta)],
                    [1]
                ],
                radius: sphereRadius
            });
        }
    }

    return { boundary: boundary, obstacles: obstacles };
}
// #endregion create-world-sealed-enclosure
