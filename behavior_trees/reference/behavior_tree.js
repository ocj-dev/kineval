/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Behavior Trees | The tick engine

    A behavior tree, in the vocabulary of Colledanchise & Ogren's
    "Behavior Trees in Robotics and AI: An Introduction" (2018) as
    presented in Sebastian Castro's "Introduction to Behavior Trees"
    (roboticseabass.com, 2021), which this module's foundational slides
    follow.

    WHAT A BEHAVIOR TREE IS, in the three properties that definition
    turns on:

      1. it is a TREE, ticked from the root in a defined order;
      2. every LEAF does something -- a check or an action -- and returns
         one of three statuses: Success, Failure, or Running;
      3. every INTERNAL node reads its children's statuses and its own
         rule decides which child is expanded next.

    That is the whole idea. Everything below is bookkeeping in service of
    it.

    WHAT THIS FILE IS NOT. It is not a port of py_trees or
    BehaviorTree.CPP, and it is not a library. It is the smallest
    implementation that is still honest about the three things those
    libraries get asked about and toy implementations usually skip:

      - MEMORY. A sequence that remembers which child it got to is a
        different algorithm from one that re-ticks from the first child
        every tick, and the difference is the single most common source
        of surprise in a real tree. Both are here, selected per node, and
        the default matches the textbook's Sequence (memory) rather than
        its ReactiveSequence.
      - HALTING. A node that returns a terminal status while one of its
        children is still Running has to tell that child to stop. Without
        halt propagation an action keeps running after the tree has moved
        on, which is how a cancelled motion carries on moving.
      - THE TICK TRACE. Every tick records which nodes it visited, in
        order, with what each returned. The deck's panels highlight the
        active path from that trace, and the trace is also how you debug a
        tree -- the question is never "what does this tree do" but "what
        did it do on THAT tick".

    STATUS IS NOT A BOOLEAN, AND THAT IS THE POINT. Running is what
    lets a leaf span many ticks: an action that takes four seconds is
    ticked at, say, 20 Hz and returns Running eighty times before it
    returns Success once. Nothing in the tree blocks, ever. This is the
    property that makes the joint-trajectory execution in
    joint_trajectory_action.js embeddable at all.

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
/////   STATUS
//////////////////////////////////////////////////

// The three values a tick can return. There is no fourth, and in
// particular there is no "idle": a node that has not been ticked has no
// status at all, which is a different thing from a node that was ticked
// and is Running.
// #region bt-status
var BT_SUCCESS = 'success';
var BT_FAILURE = 'failure';
var BT_RUNNING = 'running';
// #endregion bt-status

//////////////////////////////////////////////////
/////   NODE CONSTRUCTORS
//////////////////////////////////////////////////

// Ticks its children in order until one returns Failure or all return
// Success.
//
//   any child Failure -> Failure immediately (the rest are not ticked)
//   all children Success -> Success
//   a child Running -> Running, and the sequence is where it was
//
// MEMORY (default true) is the choice this constructor exists to make
// explicit.
//
//   memory: true  -- the textbook's Sequence. On the next tick, resume at
//                    the child that was Running. Children that already
//                    returned Success are NOT re-ticked.
//   memory: false -- the textbook's ReactiveSequence. On the next tick,
//                    start again from the first child, re-evaluating
//                    every condition that came before the running action.
//
// The difference only shows up when something changes underneath a
// running action, which is exactly when it matters. A sequence of
// [battery_ok?, follow_trajectory] with memory executes the whole
// trajectory even if the battery dies halfway, because battery_ok?
// already returned Success and is never asked again. Without memory the
// same tree aborts the motion on the tick the battery goes flat -- and
// halting the trajectory action is what actually stops the arm (see
// btHalt).
//
// Neither is the right default for everything, which is why libraries
// ship both and why this is a per-node flag rather than a global one.
// #region bt-sequence
function btSequence(name, children, options) {
    return {
        kind: 'sequence',
        name: name,
        children: children,
        memory: !options || options.memory !== false,
        state: { current: 0 }
    };
}
// #endregion bt-sequence

// Ticks its children in order until one returns Success or all return
// Failure. The mirror image of a sequence, and the node that makes
// recovery behaviour expressible: "try this; if it does not work, try
// the next thing".
//
// Also called a Selector, which is py_trees' name for it (with slightly
// different semantics) -- the textbook and BehaviorTree.CPP say Fallback.
// This module says Fallback.
//
// THE "CHECK BEFORE YOU ACT" IDIOM. Castro's tutorial makes the point
// that a fallback whose first child is a condition and whose second is
// the action that satisfies it is almost always what you want:
//
//     Fallback
//       |- At target configuration?      (condition)
//       +- Move to target configuration  (action)
//
// The tree then costs one condition tick when the robot is already where
// it needs to be, instead of re-running a motion that has nothing to do.
// This is the shape of nearly every leaf pairing in trees.js.
// #region bt-fallback
function btFallback(name, children, options) {
    return {
        kind: 'fallback',
        name: name,
        children: children,
        memory: !options || options.memory !== false,
        state: { current: 0 }
    };
}
// #endregion bt-fallback

// Ticks ALL of its children every tick, and returns a status decided by
// counting.
//
//   successThreshold (M) children Success -> Success
//   (children - M + 1) children Failure   -> Failure
//   otherwise                             -> Running
//
// "PARALLEL" IS A MISNOMER AND THE TUTORIAL SAYS SO. There is no
// concurrency here. The children are ticked one after another within a
// single tick, in order, and every one of them gets ticked -- that is the
// whole difference from a sequence. What is parallel is the children's
// PROGRESS across ticks, not their execution within one.
//
// The failure threshold is derived rather than configured:
// once more than children - M have failed, M successes are no longer
// arithmetically possible, so the parallel can report Failure without
// waiting. M = children is "and"; M = 1 is "or". Those two cases are
// what the tutorial's apples-and-oranges example is about.
// #region bt-parallel
function btParallel(name, children, successThreshold) {
    return {
        kind: 'parallel',
        name: name,
        children: children,
        successThreshold: successThreshold === undefined ? children.length : successThreshold,
        state: {}
    };
}
// #endregion bt-parallel

// Wraps exactly ONE child and modifies its result, or how many times it
// is run, according to a named policy.
//
//   'invert'              Success <-> Failure; Running passes through
//   'force-success'       Failure -> Success (Running passes through)
//   'repeat-until-success' Failure -> Running, and tick the child again
//                          next tick, up to `limit` attempts; the limit
//                          is what stops a tree that can never succeed
//                          from looping forever
//   'retry'               a synonym for repeat-until-success, kept because
//                          both names appear in the literature
//
// Why a decorator rather than another control node: the tutorial's
// refactoring argument. A fallback over three near-identical subtrees,
// one per target, is three copies of the same logic; one subtree whose
// target is read from the blackboard, wrapped in a repeat decorator, is
// one copy. The decorator supplies the repetition and the blackboard
// supplies the thing that differs. See trees.js#tree-inspection-queue.
// #region bt-decorator
function btDecorator(name, policy, child, options) {
    return {
        kind: 'decorator',
        name: name,
        policy: policy,
        children: [child],
        limit: (options && options.limit !== undefined) ? options.limit : 16,
        state: { attempts: 0 }
    };
}
// #endregion bt-decorator

// A leaf that answers a question about the world, now, and cannot take
// more than one tick to do it: it returns Success or Failure and never
// Running. `predicate(blackboard)` returns a boolean.
//
// The single-tick restriction is enforced here rather than trusted,
// because a condition that returns Running is a category error that is
// very hard to see in a tree drawing -- it makes a sequence stall on
// something that looked instantaneous.
// #region bt-condition
function btCondition(name, predicate) {
    return {
        kind: 'condition',
        name: name,
        children: [],
        predicate: predicate,
        state: {}
    };
}
// #endregion bt-condition

// A leaf that does something, possibly over many ticks.
//
//   start(blackboard)   optional; called on the tick the action begins,
//                       which is the tick after it was last halted or
//                       last returned a terminal status
//   step(blackboard)    called every tick including the first; returns
//                       BT_RUNNING / BT_SUCCESS / BT_FAILURE
//   halt(blackboard)    optional; called when the tree abandons this
//                       action while it is still Running. THIS IS WHERE
//                       A ROBOT ACTUALLY STOPS MOVING -- see
//                       joint_trajectory_action.js#action-cancel.
//
// The start/step split is not decoration. "Send the goal" must happen
// once; "has it arrived yet" must happen every tick. Collapsing them
// into one function is how an action re-sends its goal eighty times.
// #region bt-action
function btAction(name, handlers) {
    return {
        kind: 'action',
        name: name,
        children: [],
        start: handlers.start || null,
        step: handlers.step,
        halt: handlers.halt || null,
        state: { started: false }
    };
}
// #endregion bt-action

//////////////////////////////////////////////////
/////   THE TICK
//////////////////////////////////////////////////

// ONE tick of one node, and the dispatch that makes the tree a tree.
//
// `path` is the node's position in the tree as a dotted index string
// ('' for the root, '0.2' for the third child of the first child). The
// trace keys on it, which is what lets a panel highlight the exact node
// that is running even when two nodes share a name.
//
// Every status that comes back out of here is also recorded into
// `trace`, in VISIT ORDER. Visit order is the thing worth seeing: it is
// the difference between a sequence and a parallel, and it is invisible
// in a drawing of the tree.
// #region bt-tick
function btTick(node, blackboard, trace, path) {
    var status;

    switch (node.kind) {
        case 'sequence':  status = btTickSequence(node, blackboard, trace, path); break;
        case 'fallback':  status = btTickFallback(node, blackboard, trace, path); break;
        case 'parallel':  status = btTickParallel(node, blackboard, trace, path); break;
        case 'decorator': status = btTickDecorator(node, blackboard, trace, path); break;
        case 'condition': status = btTickCondition(node, blackboard); break;
        case 'action':    status = btTickAction(node, blackboard); break;
        default:
            console.error('behavior_tree: unknown node kind "' + node.kind +
                '" at path "' + path + '"');
            status = BT_FAILURE;
    }

    node.status = status;
    if (trace) {
        trace.push({ path: path, name: node.name, kind: node.kind, status: status });
    }
    return status;
}
// #endregion bt-tick

// Children in order; stop at the first Failure or Running.
//
// The resume index lives in node.state.current and is reset to 0 on
// every terminal return, which is what makes "memory" mean "memory
// within one traversal" rather than "memory forever". A sequence that
// succeeded does not stay succeeded; ticked again, it runs again from
// the top.
// #region bt-tick-sequence
function btTickSequence(node, blackboard, trace, path) {
    var start = node.memory ? node.state.current : 0;
    var i;

    for (i = start; i < node.children.length; i++) {
        var status = btTick(node.children[i], blackboard, trace, childPath(path, i));

        if (status === BT_RUNNING) {
            node.state.current = i;
            haltChildrenFrom(node, i + 1, blackboard);
            return BT_RUNNING;
        }
        if (status === BT_FAILURE) {
            node.state.current = 0;
            // The children AFTER this one may well be running, and this
            // is where a reactive sequence earns its name: a guard
            // condition that fails on tick 40 has to stop the action
            // below it that has been running since tick 12. Without this
            // line the tree takes a different branch while the arm keeps
            // following the abandoned trajectory.
            haltChildrenFrom(node, i + 1, blackboard);
            return BT_FAILURE;
        }
        // Success: fall through to the next child
    }

    node.state.current = 0;
    return BT_SUCCESS;
}
// #endregion bt-tick-sequence

// #region bt-tick-fallback
// Children in order; stop at the first Success or Running.
function btTickFallback(node, blackboard, trace, path) {
    var start = node.memory ? node.state.current : 0;
    var i;

    for (i = start; i < node.children.length; i++) {
        var status = btTick(node.children[i], blackboard, trace, childPath(path, i));

        if (status === BT_RUNNING) {
            node.state.current = i;
            haltChildrenFrom(node, i + 1, blackboard);
            return BT_RUNNING;
        }
        if (status === BT_SUCCESS) {
            node.state.current = 0;
            // the mirror of the sequence's case: a recovery branch that
            // was running when a higher-priority child started
            // succeeding has to be stopped
            haltChildrenFrom(node, i + 1, blackboard);
            return BT_SUCCESS;
        }
        // Failure: try the next child -- this is the recovery branch
    }

    node.state.current = 0;
    return BT_FAILURE;
}
// #endregion bt-tick-fallback

// Every child, every tick, then count.
//
// Note what is NOT here: no early exit. A parallel whose success
// threshold is already met still has to tick its remaining children
// before returning, or children would get ticked a different number of
// times depending on their position, and "parallel" would be a lie about
// ordering as well as about concurrency. Children that have already
// finished ARE short-circuited, though -- a child that returned Success
// on an earlier tick is not re-ticked, because re-ticking it would
// restart it.
// #region bt-tick-parallel
function btTickParallel(node, blackboard, trace, path) {
    if (!node.state.childStatus) node.state.childStatus = [];

    var successes = 0;
    var failures = 0;
    var i;

    for (i = 0; i < node.children.length; i++) {
        var prior = node.state.childStatus[i];
        var status;
        if (prior === BT_SUCCESS || prior === BT_FAILURE) {
            status = prior;
        } else {
            status = btTick(node.children[i], blackboard, trace, childPath(path, i));
            node.state.childStatus[i] = status;
        }
        if (status === BT_SUCCESS) successes++;
        else if (status === BT_FAILURE) failures++;
    }

    var needed = node.successThreshold;
    if (successes >= needed) {
        btHalt(node, blackboard);          // stop the children still running
        node.state.childStatus = [];
        return BT_SUCCESS;
    }
    if (failures > node.children.length - needed) {
        btHalt(node, blackboard);
        node.state.childStatus = [];
        return BT_FAILURE;
    }
    return BT_RUNNING;
}
// #endregion bt-tick-parallel

// One child, one policy.
//
// repeat-until-success is the only policy that keeps state: it counts
// attempts so that `limit` can stop it. A decorator with no limit and a
// child that always fails is an infinite loop that presents as a frozen
// robot, and the tutorial's refactored tree -- repeat until the location
// queue runs dry -- relies on the child eventually returning Failure to
// terminate. The limit is the belt to that braces.
// #region bt-tick-decorator
function btTickDecorator(node, blackboard, trace, path) {
    var child = node.children[0];
    var status = btTick(child, blackboard, trace, childPath(path, 0));

    switch (node.policy) {
        case 'invert':
            if (status === BT_SUCCESS) return BT_FAILURE;
            if (status === BT_FAILURE) return BT_SUCCESS;
            return BT_RUNNING;

        case 'force-success':
            if (status === BT_FAILURE) return BT_SUCCESS;
            return status;

        case 'repeat-until-success':
        case 'retry':
            if (status === BT_SUCCESS) {
                node.state.attempts = 0;
                return BT_SUCCESS;
            }
            if (status === BT_FAILURE) {
                node.state.attempts++;
                if (node.state.attempts >= node.limit) {
                    console.warn('behavior_tree: decorator "' + node.name +
                        '" hit its ' + node.limit + '-attempt limit; reporting Failure');
                    node.state.attempts = 0;
                    return BT_FAILURE;
                }
                return BT_RUNNING;   // try again next tick
            }
            return BT_RUNNING;

        default:
            console.error('behavior_tree: unknown decorator policy "' +
                node.policy + '" on node "' + node.name + '"');
            return BT_FAILURE;
    }
}
// #endregion bt-tick-decorator

// #region bt-tick-condition
// A question, asked and answered within the tick.
function btTickCondition(node, blackboard) {
    var answer = node.predicate(blackboard);
    if (answer === BT_RUNNING) {
        console.error('behavior_tree: condition "' + node.name +
            '" returned Running; a condition must resolve within one tick');
        return BT_FAILURE;
    }
    return answer ? BT_SUCCESS : BT_FAILURE;
}
// #endregion bt-tick-condition

// #region bt-tick-action
// start() once, step() every tick, and reset when it finishes so the
// next traversal starts it cleanly.
function btTickAction(node, blackboard) {
    if (!node.state.started) {
        node.state.started = true;
        if (node.start) node.start(blackboard);
    }

    var status = node.step(blackboard);

    if (status !== BT_RUNNING) {
        node.state.started = false;   // finished: the next tick is a new run
    }
    return status;
}
// #endregion bt-tick-action

//////////////////////////////////////////////////
/////   HALTING
//////////////////////////////////////////////////

// Abandon everything still running underneath `node`.
//
// This is the half of a behavior tree that toy implementations leave
// out, and leaving it out is not a simplification -- it is a bug with a
// physical consequence. A running action owns something: a trajectory
// being executed, a goal sent to an action server. When the tree stops
// ticking that action -- because a reactive sequence re-evaluated a
// condition above it and took a different branch -- the action must be
// told, or the arm keeps following the trajectory the tree has already
// decided to abandon.
//
// Recurses the whole subtree rather than only its running children,
// because node.status is a record of the LAST tick and a node deep
// inside may hold resources even if its parent returned Success this
// tick. Cheap, and there is no correct way to be cleverer without
// tracking ownership explicitly.
// #region bt-halt
function btHalt(node, blackboard) {
    var i;
    for (i = 0; i < node.children.length; i++) {
        btHalt(node.children[i], blackboard);
    }

    if (node.kind === 'action' && node.state.started) {
        if (node.halt) node.halt(blackboard);
        node.state.started = false;
    }
    if (node.state.current !== undefined) node.state.current = 0;
    if (node.state.attempts !== undefined) node.state.attempts = 0;
    if (node.state.childStatus) node.state.childStatus = [];
    node.status = undefined;
}
// #endregion bt-halt

//////////////////////////////////////////////////
/////   DRIVING A TREE
//////////////////////////////////////////////////

// Halt children `from` onward, used by sequence and fallback when they
// return from a child that is not the last one.
//
// Cheap and unconditional: btHalt on a node that holds nothing does
// nothing. The alternative -- halting only children whose last status
// was Running -- is wrong in the one case that matters, because a child
// that returned Success on an earlier tick can still own a resource
// (a sequence that finished its motion still has an action server
// holding the goal) and node.status is a record of the last tick rather
// than of ownership.
// #region bt-halt-children-from
function haltChildrenFrom(node, from, blackboard) {
    var i;
    for (i = from; i < node.children.length; i++) {
        btHalt(node.children[i], blackboard);
    }
}
// #endregion bt-halt-children-from

// One tick of a whole tree, returning the status and the trace.
//
// THE TICK RATE IS THE TREE'S ONLY CLOCK. Nothing in this file measures
// time; the leaves that care about time read it off the blackboard
// (blackboard.now, blackboard.dt), which the caller advances. That is
// deliberate: it makes a tree's behaviour a function of the tick
// sequence alone, so the deck's step-through panels and the headless
// tests in this module drive exactly the same code with exactly the same
// results as the animated demo -- and so a trajectory's progress can be
// stepped one tick at a time rather than watched in real time.
// #region bt-root-tick
function btTickTree(root, blackboard) {
    var trace = [];
    var status = btTick(root, blackboard, trace, '');
    return { status: status, trace: trace };
}
// #endregion bt-root-tick

// #region bt-child-path
// The dotted index path of a child, given its parent's.
function childPath(path, index) {
    return path === '' ? String(index) : path + '.' + index;
}
// #endregion bt-child-path

// Every node in the tree, with its path, depth and child index, in
// depth-first order -- which is the order a drawing of the tree needs.
//
// Pure: it reads structure, never status. The panels draw the tree once
// from this and then recolour it from each tick's trace, so the layout
// cannot shift while the tree runs.
// #region bt-walk
function btWalk(root) {
    var nodes = [];

    function visit(node, path, depth, index) {
        nodes.push({
            node: node, path: path, depth: depth, index: index,
            kind: node.kind, name: node.name,
            childCount: node.children.length
        });
        var i;
        for (i = 0; i < node.children.length; i++) {
            visit(node.children[i], childPath(path, i), depth + 1, i);
        }
    }

    visit(root, '', 0, 0);
    return nodes;
}
// #endregion bt-walk

// The shared store every leaf reads and writes.
//
// "A common storage area where individual behaviors can read or write
// data" is Castro's definition, and the point of it is that leaves do
// not know about each other. The action that plans a motion writes
// `trajectory`; the action that executes one reads `trajectory`; neither
// names the other, so either can be replaced, and the same execute leaf
// can be driven by a planner, a recorded plan, or a teach pendant.
//
// THE COST, STATED PLAINLY: this is global mutable state with a nice
// name. Nothing above declares what a leaf will touch, so a tree can be
// wired up wrong in a way no drawing of it reveals -- the classic symptom
// is a subtree that works alone and fails in place, because something
// else in the tree writes the key it reads. btBlackboardRequire below is
// this module's answer: a leaf declares the keys it needs, and the tree
// is checked against them before it is ever ticked, rather than failing
// on tick 300 when a branch is first reached.
// #region bt-blackboard
function btBlackboard(initial) {
    var blackboard = { now: 0, dt: 0, log: [] };
    var key;
    for (key in (initial || {})) blackboard[key] = initial[key];
    return blackboard;
}
// #endregion bt-blackboard

// Check, before the first tick, that every key the tree's leaves declare
// they read is either already on the blackboard or written by some leaf
// in the same tree.
//
// Declarations are per leaf: `reads` and `writes` arrays on the node, set
// by the constructors in bt_tasks.js. A leaf with no declarations is
// skipped rather than assumed clean -- this validates what it is told,
// and says what it was told.
//
// Returns an array of problems, empty when there are none. It is a
// LINTER, not a type system: it cannot catch a key written with the
// wrong kind of value, and it cannot know whether the branch that writes
// a key will be reached before the branch that reads it.
// #region bt-blackboard-require
function btBlackboardRequire(root, blackboard) {
    var nodes = btWalk(root);
    var written = {};
    var problems = [];
    var i, j;

    for (i = 0; i < nodes.length; i++) {
        var writes = nodes[i].node.writes || [];
        for (j = 0; j < writes.length; j++) written[writes[j]] = nodes[i].path;
    }

    for (i = 0; i < nodes.length; i++) {
        var reads = nodes[i].node.reads || [];
        for (j = 0; j < reads.length; j++) {
            var key = reads[j];
            if (blackboard[key] !== undefined) continue;
            if (written[key] !== undefined) continue;
            problems.push({
                path: nodes[i].path,
                name: nodes[i].name,
                key: key,
                message: 'leaf "' + nodes[i].name + '" (path ' + nodes[i].path +
                    ') reads blackboard key "' + key + '", which nothing in ' +
                    'this tree writes and which the initial blackboard does not have'
            });
        }
    }

    return problems;
}
// #endregion bt-blackboard-require
