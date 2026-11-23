/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Behavior Trees | Page infrastructure

    Everything the demo page needs that is not the subject being taught:
    URL parameters, the HUD, the tick loop, and the tree picker.

    THE TICK LOOP IS THE ONLY CLOCK, and it is deliberately not the
    animation frame rate. A behavior tree is ticked at a stated rate --
    ?rate=20 by default, i.e. every 50ms -- and rendering happens every
    frame regardless. Tying ticks to frames would make the tree's
    behaviour depend on the machine's refresh rate, and the whole point of
    the tick abstraction is that it does not.

    `dt` handed to the tree is the NOMINAL tick period, not the measured
    elapsed time. That is a choice, and it is the one that makes a run
    reproducible: step the page a hundred times by hand and you get
    exactly the trajectory the smoke test gets. A real controller must use
    measured time, because its clock is the world's; a teaching
    environment that did so would show a slightly different motion on
    every machine and could not state a number on a slide.

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

// Every run of this page is configured by URL parameter, so a link is a
// complete description of what the reader is looking at.
//
//   ?tree=plan_execute | inspection | reactive    which tree to run
//   ?reactive=0|1      memory off/on for the battery tree's sequence
//   ?rate=20           tick rate, Hz
//   ?autoplay=0|1      start ticking immediately
//   ?obstacles=0|1     draw the planning world's spheres
//   ?ghosts=0|1        draw the waypoint markers
// #region bt-params
var BT_TREE_CHOICES = ['plan_execute', 'inspection', 'reactive'];

function readBtParams() {
    var search = new URLSearchParams(window.location.search);

    function number(name, fallback) {
        var raw = search.get(name);
        if (raw === null) return fallback;
        var value = parseFloat(raw);
        return isFinite(value) ? value : fallback;
    }
    function flag(name, fallback) {
        var raw = search.get(name);
        if (raw === null) return fallback;
        return raw !== '0' && raw !== 'false';
    }

    var tree = search.get('tree') || 'plan_execute';
    if (BT_TREE_CHOICES.indexOf(tree) === -1) {
        console.warn('behavior_trees: unknown ?tree=' + tree + '; using plan_execute. ' +
            'Choices: ' + BT_TREE_CHOICES.join(', '));
        tree = 'plan_execute';
    }

    return {
        tree: tree,
        reactive: flag('reactive', true),
        rate: Math.max(1, number('rate', 20)),
        autoplay: flag('autoplay', true),
        obstacles: flag('obstacles', true),
        ghosts: flag('ghosts', true),
        battery: number('battery', 30)
    };
}
// #endregion bt-params

// Three arm configurations for the longer-horizon tree to visit, and the
// deterministic answer to "is the part there" at each.
//
// Hand-authored rather than planned, and that is a real limitation worth
// stating: each is reached by a straight two-waypoint trajectory, so
// these motions are NOT collision-checked against the world the way the
// committed plan is. The inspection tree is about sequencing several
// motions; the plan-and-execute tree is about one motion being a real
// plan. Conflating them would mean either three planner runs on page
// load or one pretend plan, and both are worse than saying which is
// which.
// #region inspection-poses
var BT_INSPECTION_POSES = {
    bin_a: [0.4, 0.1, 0.5, 0.0],
    bin_b: [0.9, -0.3, 0.9, 0.4],
    bin_c: [1.1, -0.8, 1.2, 0.9]
};

var BT_PARTS_PRESENT = { bin_a: false, bin_b: false, bin_c: true };

function buildInspectionTrajectories(plan, startPositions) {
    var trajectories = {};
    var from = startPositions;
    var name;
    for (name in BT_INSPECTION_POSES) {
        trajectories[name] = trajectoryFromPlan({
            q_layout: plan.joint_names,
            smoothed_path: [from, BT_INSPECTION_POSES[name]],
            joint_names: plan.joint_names,
            provenance: { generated_by: 'infrastructure.js#inspection-poses (hand-authored two-waypoint hop)' },
            robot: plan.robot,
            seed: plan.seed
        }, plan.joint_names);
        from = BT_INSPECTION_POSES[name];
    }
    return trajectories;
}
// #endregion inspection-poses

// Builds the selected tree, and the blackboard that goes with it.
//
// Runs btBlackboardRequire before handing the tree back, and says so in
// the console either way. A tree that fails validation is still
// returned -- refusing to run it would hide the symptom the validator
// exists to explain -- but the reader is told before the first tick
// rather than after the three hundredth.
// #region build-tree
function buildBtTree(params, server, plan) {
    var tree, blackboard;

    if (params.tree === 'inspection') {
        var trajectories = buildInspectionTrajectories(plan, plan.smoothed_positions[0]);
        tree = treeInspectionQueue(server, trajectories);
        blackboard = btBlackboard({ parts_present: BT_PARTS_PRESENT });

    } else if (params.tree === 'reactive') {
        tree = treeReactiveBattery(server, plan, plan.joint_names, { reactive: params.reactive });
        blackboard = btBlackboard({ battery: params.battery });

    } else {
        tree = treePlanAndExecute(server, plan, plan.joint_names);
        blackboard = btBlackboard({});
    }

    var problems = btBlackboardRequire(tree, blackboard);
    if (problems.length) {
        var i;
        for (i = 0; i < problems.length; i++) console.error('blackboard: ' + problems[i].message);
    } else {
        console.log('blackboard: every key the "' + params.tree +
            '" tree reads is either preset or written by one of its own leaves');
    }

    return { tree: tree, blackboard: blackboard, problems: problems };
}
// #endregion build-tree

// Renders every frame; ticks the tree at the configured rate.
//
// `stepOnce` is a one-shot that survives being pressed while paused,
// which is what makes the page steppable: the reader can advance the
// tree one tick at a time and read the trace for that single tick.
// #region animation-loop
function createBtAnimationLoop(app, onTick, onFrame) {
    var lastTickTime = 0;

    function frame(now) {
        requestAnimationFrame(frame);

        var period = 1000 / app.params.rate;
        var due = app.stepOnce || (app.isRunning && now - lastTickTime >= period);

        if (due) {
            lastTickTime = now;
            app.stepOnce = false;
            onTick();
        }
        onFrame();
    }

    requestAnimationFrame(frame);
}
// #endregion animation-loop

// The on-page readout and controls.
//
// Everything the tree and the action know is on screen, because the
// alternative is a page where something visibly happens and nothing
// explains it: the tick count, the tree's status, the path of the node
// that is running, the action's state, the feedback's desired/actual/
// error, and the tail of the blackboard log.
// #region hud
function buildBtHud(container, app) {
    var hud = document.createElement('div');
    hud.className = 'bt-hud';

    var controls = document.createElement('div');
    controls.className = 'bt-controls';

    var playButton = document.createElement('button');
    var stepButton = document.createElement('button');
    stepButton.textContent = 'Step one tick';
    var resetButton = document.createElement('button');
    resetButton.textContent = '↺ Reset';
    var cancelButton = document.createElement('button');
    cancelButton.textContent = 'Cancel the goal';

    var treeSelect = document.createElement('select');
    var i;
    for (i = 0; i < BT_TREE_CHOICES.length; i++) {
        var option = document.createElement('option');
        option.value = BT_TREE_CHOICES[i];
        option.textContent = BT_TREE_CHOICES[i];
        treeSelect.appendChild(option);
    }
    treeSelect.value = app.params.tree;

    var reactiveLabel = document.createElement('label');
    reactiveLabel.className = 'bt-field';
    var reactiveBox = document.createElement('input');
    reactiveBox.type = 'checkbox';
    reactiveBox.checked = app.params.reactive;
    reactiveLabel.appendChild(reactiveBox);
    reactiveLabel.appendChild(document.createTextNode(' reactive (no memory)'));

    controls.appendChild(playButton);
    controls.appendChild(stepButton);
    controls.appendChild(resetButton);
    controls.appendChild(cancelButton);
    controls.appendChild(treeSelect);
    controls.appendChild(reactiveLabel);

    var status = document.createElement('pre');
    status.className = 'bt-status';

    var treeView = document.createElement('pre');
    treeView.className = 'bt-tree';

    var log = document.createElement('pre');
    log.className = 'bt-log';

    hud.appendChild(controls);
    hud.appendChild(status);
    hud.appendChild(treeView);
    hud.appendChild(log);
    container.appendChild(hud);

    function syncPlayLabel() {
        playButton.textContent = app.isRunning ? 'Pause' : 'Play';
    }

    playButton.addEventListener('click', function () {
        app.isRunning = !app.isRunning;
        syncPlayLabel();
    });
    stepButton.addEventListener('click', function () {
        app.isRunning = false;
        app.stepOnce = true;
        syncPlayLabel();
    });
    resetButton.addEventListener('click', function () { app.reset(); syncPlayLabel(); });
    cancelButton.addEventListener('click', function () {
        // A button that cancels the ACTION rather than the tree, which is
        // the interesting case: the tree finds out on its next tick, by
        // the execute leaf returning Failure, and whatever recovery
        // branch exists then runs. Nothing about the tree is special-cased.
        var result = jointTrajectoryActionCancel(app.server, 'the Cancel button');
        if (result) app.blackboard.log.push('cancel button: ' + result.error_string);
    });
    treeSelect.addEventListener('change', function () {
        app.params.tree = treeSelect.value;
        app.reset();
        syncPlayLabel();
    });
    reactiveBox.addEventListener('change', function () {
        app.params.reactive = reactiveBox.checked;
        app.reset();
        syncPlayLabel();
    });

    syncPlayLabel();
    return { root: hud, status: status, treeView: treeView, log: log, syncPlayLabel: syncPlayLabel };
}
// #endregion hud

// #region hud-render
// One frame's worth of readout.
function renderBtHud(hud, app) {
    var lines = [];
    var feedback = app.server.feedback;

    lines.push('tree      ' + app.params.tree +
        (app.params.tree === 'reactive' ? (app.params.reactive ? ' (reactive)' : ' (with memory)') : ''));
    lines.push('tick      ' + app.tickCount + '   at ' + app.params.rate + ' Hz' +
        '   t = ' + app.blackboard.now.toFixed(2) + 's');
    lines.push('status    ' + (app.lastStatus || '-'));
    lines.push('action    ' + (app.server.state || 'no goal') +
        '   goals ' + app.server.stats.goals +
        ' / succeeded ' + app.server.stats.succeeded +
        ' / canceled ' + app.server.stats.canceled +
        ' / aborted ' + app.server.stats.aborted);

    if (feedback) {
        lines.push('feedback  point index ' + feedback.index +
            ' of ' + (feedback.desired.length ? app.blackboard.trajectory.points.length - 1 : 0) +
            '   (segment ' + feedback.segment + ', ' + (feedback.alpha * 100).toFixed(0) + '% through)');
        lines.push('          t = ' + feedback.header.stamp.toFixed(2) + 's of ' +
            feedback.duration.toFixed(2) + 's');
        lines.push('desired   ' + feedback.desired.map(fixed3).join('  '));
        lines.push('actual    ' + feedback.actual.map(fixed3).join('  '));
        lines.push('error     ' + feedback.error.map(fixed3).join('  ') +
            '      worst ' + feedback.worst_error.toFixed(4) + ' rad');
    }
    if (app.blackboard.battery !== undefined) {
        lines.push('battery   ' + app.blackboard.battery.toFixed(1) + '%' +
            (app.blackboard.charging ? '  (charging)' : ''));
    }
    if (app.blackboard.last_result) {
        lines.push('result    error_code ' + app.blackboard.last_result.error_code +
            (app.blackboard.last_result.error_string ?
                '  ' + app.blackboard.last_result.error_string : ''));
    }

    hud.status.textContent = lines.join('\n');
    hud.treeView.textContent = renderTreeAscii(app.tree, app.lastTrace);

    var log = app.blackboard.log;
    hud.log.textContent = log.slice(Math.max(0, log.length - 7)).join('\n');
}

function fixed3(value) {
    return (value >= 0 ? ' ' : '') + value.toFixed(3);
}
// #endregion hud-render

// The tree, drawn as indented text, with each node's status from the
// most recent tick.
//
// Text rather than SVG because this page is the reference
// implementation, not the slides: the deck's panels draw the tree
// properly. What matters here is that the drawing comes from btWalk --
// structure only -- and the colouring from the tick trace, which is the
// same separation the panels use. A node not visited on the last tick
// shows a dot, and that is information: it is how you see a sequence
// short-circuiting.
// #region tree-ascii
function renderTreeAscii(root, trace) {
    var byPath = {};
    var i;
    for (i = 0; i < (trace || []).length; i++) byPath[trace[i].path] = trace[i].status;

    var symbol = { success: '[S]', failure: '[F]', running: '[R]' };
    var nodes = btWalk(root);
    var lines = [];

    for (i = 0; i < nodes.length; i++) {
        var entry = nodes[i];
        var status = byPath[entry.path];
        var mark = status ? symbol[status] : ' . ';
        var kind = entry.kind === 'action' ? 'action'
            : entry.kind === 'condition' ? 'cond  '
            : entry.kind;
        lines.push(mark + ' ' + new Array(entry.depth + 1).join('   ') +
            kind + '  ' + entry.name);
    }
    return lines.join('\n');
}
// #endregion tree-ascii

// Fetches the committed plan JSON.
//
// A fetch, so the page states where its plan came from and a reader can
// open the file. The alternative -- inlining the numbers in a .js file --
// would make the plan look authored rather than planned, which is the
// one impression this module must not give.
// #region load-plan
function loadCommittedPlan(url) {
    return fetch(url).then(function (response) {
        if (!response.ok) {
            throw new Error('could not load ' + url + ': HTTP ' + response.status);
        }
        return response.json();
    });
}
// #endregion load-plan
