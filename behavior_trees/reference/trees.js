/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Behavior Trees | The example trees

    Every tree this module's slides draw, built from the leaves in
    bt_tasks.js. One function per tree, so a slide can quote the tree it
    is discussing and nothing else.

    THE FIRST FIVE follow the progression in Sebastian Castro's
    "Introduction to Behavior Trees" (roboticseabass.com, 2021) step by
    step -- a naive sequence, the same tree with an explicit success
    condition, several locations under a fallback, a parallel composing
    two object checks, and finally the refactoring that replaces N copies
    of a subtree with one copy plus a blackboard key and a decorator.
    Each is a tree the slides can tick, not a picture of one.

    THE REST are this module's own subject: a motion plan becoming a
    trajectory, a trajectory being executed by an action, and both being
    sequenced inside a tree for a task longer than one motion.

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
/////   THE SEARCH PROGRESSION
//////////////////////////////////////////////////

// STEP 1. Go to a room, look for the apple.
//
//     Sequence
//       |- Navigate to kitchen
//       +- Found apple?
//
// Correct, and wrong in a way that is invisible until the tree is
// ticked twice: it navigates unconditionally. Ticked again with the
// robot already in the kitchen, it drives to the kitchen again.
// #region tree-search-naive
function treeSearchNaive() {
    return btSequence('Search the kitchen', [
        taskNavigateTo('kitchen'),
        taskFindObject('apple')
    ]);
}
// #endregion tree-search-naive

// STEP 2. The same tree with an EXPLICIT SUCCESS CONDITION on the
// navigation.
//
//     Sequence
//       |- Fallback
//       |    |- At kitchen?
//       |    +- Navigate to kitchen
//       +- Found apple?
//
// "You should almost always check before you act" is Castro's phrasing,
// and the fallback is how it is expressed: if the condition already
// holds, the action is never ticked. The subtree is also now idempotent
// -- ticking it when the robot is already there costs one comparison --
// which is what makes it safe to put under a decorator that repeats.
// #region tree-search-explicit-success
function treeSearchExplicitSuccess() {
    return btSequence('Search the kitchen', [
        btFallback('Be at the kitchen', [
            taskAtLocation('kitchen'),
            taskNavigateTo('kitchen')
        ]),
        taskFindObject('apple')
    ]);
}
// #endregion tree-search-explicit-success

// STEP 3. Three places to look, under a FALLBACK.
//
//     Fallback
//       |- Sequence (kitchen)
//       |- Sequence (pantry)
//       +- Sequence (garage)
//
// The fallback's "if that did not work, try the next one" is doing the
// searching. Each child fails when the apple is not in that room, and
// failure is what advances the search -- so the thing a reader is most
// likely to call an error ("three of the four branches fail!") is the
// mechanism.
//
// THE COST, which is the setup for step 5: the three children are the
// same subtree three times. Change how arrival is checked and you change
// it in three places.
// #region tree-search-multiple-locations
function treeSearchMultipleLocations(locations) {
    var places = locations || ['kitchen', 'pantry', 'garage'];
    var children = [];
    var i;
    for (i = 0; i < places.length; i++) {
        children.push(btSequence('Search the ' + places[i], [
            btFallback('Be at the ' + places[i], [
                taskAtLocation(places[i]),
                taskNavigateTo(places[i])
            ]),
            taskFindObject('apple')
        ]));
    }
    return btFallback('Search everywhere', children);
}
// #endregion tree-search-multiple-locations

// STEP 4. Looking for an apple OR an orange, with a PARALLEL.
//
//     Sequence
//       |- Fallback (be at the kitchen)
//       +- Parallel (1 of 2)
//            |- Found apple?
//            +- Found orange?
//
// successThreshold = 1 is "or"; passing 2 makes the same tree mean
// "and". That one integer is the entire difference, which is the
// clearest argument for parallel existing as a node kind rather than
// being written as nested fallbacks.
//
// Both children are ticked every tick even after one has succeeded --
// see btTickParallel for why there is no early exit -- so the log shows
// two lookups, not one, and a reader who expected short-circuiting can
// see that it does not happen.
// #region tree-search-parallel
function treeSearchParallel(threshold) {
    return btSequence('Search the kitchen for fruit', [
        btFallback('Be at the kitchen', [
            taskAtLocation('kitchen'),
            taskNavigateTo('kitchen')
        ]),
        btParallel('Apple or orange', [
            taskFindObject('apple'),
            taskFindObject('orange')
        ], threshold === undefined ? 1 : threshold)
    ]);
}
// #endregion tree-search-parallel

// STEP 5. The refactoring: ONE subtree, a blackboard key, and a repeat
// decorator.
//
//     Decorator (repeat until success)
//       +- Sequence
//            |- Get next location        writes target_location
//            |- Fallback
//            |    |- At target?           reads  target_location
//            |    +- Navigate to target   reads  target_location
//            +- Found the object?         reads  wanted_object
//
// Compare with step 3: that tree had one subtree per location and had
// to be edited to add a fourth; this one takes its locations from
// blackboard.location_queue, so a fourth location is a data change.
//
// HOW IT TERMINATES, which is the part worth tracing. The decorator
// repeats while its child FAILS. The child fails either because the
// object was not at this location -- in which case repeating is exactly
// right, and "Get next location" supplies the next one -- or because the
// queue has run dry, in which case repeating is pointless. Those two
// failures are indistinguishable to the decorator. The attempt limit is
// what separates them, and it is set from the queue's own length by the
// caller, which is the honest version of a loop bound.
//
// Castro credits this pattern to Davide Faconti (BehaviorTree.CPP);
// BehaviorTree.CPP's own name for the decorator is RetryUntilSuccessful,
// with num_attempts.
// #region tree-search-with-decorator
function treeSearchWithDecorator(locationCount) {
    var attempts = locationCount === undefined ? 3 : locationCount;
    return btDecorator('Try every location', 'repeat-until-success',
        btSequence('Search one location', [
            taskNextLocation(),
            btFallback('Be at the target', [
                taskAtTargetLocation(),
                taskNavigateToTarget()
            ]),
            taskFindObjectHere()
        ]),
        { limit: attempts });
}
// #endregion tree-search-with-decorator

//////////////////////////////////////////////////
/////   THE MOTION TREES
//////////////////////////////////////////////////

// Plan a motion, then execute it -- with the check-before-you-act
// fallback wrapped around the whole thing.
//
//     Fallback
//       |- At the goal configuration?
//       +- Sequence
//            |- Plan motion               writes trajectory
//            +- Follow joint trajectory   reads  trajectory
//
// This is the smallest tree that is actually about this module's
// subject, and every piece of it is load-bearing:
//
//   - the SEQUENCE is why planning and execution are separable. The
//     planner's output lands on the blackboard and the executor picks it
//     up; neither names the other.
//   - the FALLBACK is why the tree can be ticked forever. Once the arm
//     has arrived, the condition succeeds and nothing replans.
//   - and the fact that "Follow joint trajectory" returns Running for
//     the whole motion is why this can be a tree at all rather than a
//     script. Everything above this node keeps being evaluated for the
//     several seconds the arm is moving.
//
// On the first tick `trajectory` is undefined, so the condition returns
// Failure and the sequence runs. That ordering is not an accident of
// evaluation order -- the condition reads a key the planner writes, and
// btBlackboardRequire accepts it for exactly that reason.
// #region tree-plan-and-execute
function treePlanAndExecute(server, plan, jointNames) {
    return btFallback('Reach the goal configuration', [
        taskAtTrajectoryEnd(server),
        btSequence('Plan and execute', [
            taskPlanMotion(plan, jointNames),
            taskFollowTrajectory(server)
        ])
    ]);
}
// #endregion tree-plan-and-execute

// The longer horizon: visit every inspection pose in turn, look at each,
// and stop at the first one that has the part.
//
//     Decorator (repeat until success, limit = number of targets)
//       +- Sequence
//            |- Get next inspection target   writes trajectory, inspection_target
//            |- Fallback
//            |    |- At the goal configuration?
//            |    +- Follow joint trajectory
//            +- Part present?
//
// Structurally identical to treeSearchWithDecorator -- the same
// decorator, the same queue-popping leaf, the same check-before-you-act
// fallback -- with navigation replaced by trajectory execution. That is
// the argument for behavior trees made as a diff rather than as a claim:
// the task got a great deal more real and the tree barely changed.
//
// One motion per cycle, each a full plan-to-trajectory-to-action
// pipeline, and the tree is what sequences them. Nothing here knows how
// many targets there are except the queue and the decorator's limit.
// #region tree-inspection-queue
function treeInspectionQueue(server, trajectoriesByTarget) {
    var count = Object.keys(trajectoriesByTarget).length;
    return btDecorator('Inspect every target', 'repeat-until-success',
        btSequence('Inspect one target', [
            taskNextInspectionTarget(trajectoriesByTarget),
            btFallback('Be at the inspection pose', [
                taskAtTrajectoryEnd(server),
                taskFollowTrajectory(server)
            ]),
            taskInspect()
        ]),
        { limit: count });
}
// #endregion tree-inspection-queue

// The reactivity example, as a tree, with the guard that makes it work
// and the flag that makes it fail.
//
//     Fallback
//       |- Sequence (memory: see below)
//       |    |- Battery above 20%?
//       |    +- Fallback
//       |         |- At the goal configuration?
//       |         +- Sequence [Plan motion, Follow joint trajectory]
//       +- Go charge
//
// WITH MEMORY (reactive: false) the battery is checked once, at the
// start of the traversal, and then never again -- the sequence resumes
// at the running motion on every subsequent tick. The arm finishes the
// whole trajectory on a battery that went flat two seconds in. The tree
// is not wrong, it is just not reactive, and the drawing of it is
// identical.
//
// WITHOUT MEMORY (reactive: true) the condition is re-evaluated every
// tick. When it fails, the sequence returns Failure, haltChildrenFrom
// halts the execute leaf, the leaf's halt handler cancels the action
// goal, and the arm stops where it is. Then the fallback's second child
// runs and the robot charges.
//
// Four pieces had to line up for that: the no-memory flag, the halt
// propagation, the leaf's halt handler, and the action server's cancel.
// Any one of them missing and the tree takes the charging branch while
// the arm carries on moving -- which looks, on screen, exactly like a
// behavior tree that works.
//
// This is also the place to be honest about the comparison Castro makes:
// a fully reactive battery response is easier to express in a
// finite-state machine, where it is one transition out of every state,
// than it is here, where it is a guard that has to be placed above
// everything it guards.
// #region tree-reactive-battery
function treeReactiveBattery(server, plan, jointNames, options) {
    var reactive = !options || options.reactive !== false;
    return btFallback('Work, or charge', [
        btSequence('Work while charged', [
            taskBatteryAbove(20),
            btFallback('Reach the goal configuration', [
                taskAtTrajectoryEnd(server),
                btSequence('Plan and execute', [
                    taskPlanMotion(plan, jointNames),
                    taskFollowTrajectory(server)
                ])
            ])
        ], { memory: !reactive }),
        taskCharge()
    ]);
}
// #endregion tree-reactive-battery
