// ===========================================================================
// SNAPSHOT -- DO NOT EDIT THIS COPY
//
//   Source repository: /ai/development/autorob_agentic/pubsub_testbed_rs
//   Source path:       src/bin/bt_node.rs
//   Upstream length:   1389 lines
//   Commit:            d9e34cd16c47c790856b1ccea0d8575620c66f89
//   Snapshot date:     2026-11-23
//
// Why this copy exists: Slidev's `<<<` snippet importer cannot read
// outside its own workspace root, which behavior_trees/pnpm-workspace.yaml
// pins at behavior_trees/. The deck quotes Rust that lives in a DIFFERENT
// REPOSITORY entirely, so the file has to be carried here to be quotable
// at all.
//
// Quoted here for: the behavior-tree interpreter: the eight node kinds, the blackboard and its load-time validation, the non-blocking leaves, and the hierarchical tick dispatch.
//
// LINE NUMBERS IN THIS FILE ARE NOT UPSTREAM LINE NUMBERS -- this header
// shifts everything down. Where a slide cites a line, it cites the
// UPSTREAM one, and the mapping is below.
//
// #region markers, with their upstream line numbers:
//   bt-node-kinds                upstream line 337
//   bt-runtime-state             upstream line 391
//   blackboard-validation        upstream line 481
//   non-blocking-leaves          upstream line 655
//   hierarchical-dispatch        upstream line 903
//
// Edits belong upstream, followed by a re-snapshot. Diff this file
// against the commit above to check it has not drifted.
// ===========================================================================

//! bt_node: a hierarchical-switch-case behavior tree interpreter -- the
//! recursive counterpart to `fsm_node.rs`'s flat one. Same "fixed,
//! hand-written Rust interpreter; only the *data* it interprets changes,
//! loaded live via a service call, no code generation, no rebuilds"
//! philosophy as that file's own module doc comment describes, just
//! applied to a tree instead of a flat table of states.
//!
//! ## Tree representation
//!
//! A behavior tree here is a plain, recursive `serde_json::Value` on the
//! wire -- no derived struct, matching this project's wire-message
//! philosophy everywhere else (see `ros_messages.rs`'s own module doc
//! comment). Each node is `{"type": <kind>, ...kind-specific fields}`:
//!   - `{"type": "sequence", "children": [<node>, ...]}` / `{"type":
//!     "selector", "children": [<node>, ...]}` -- at least one child each
//!     (an empty composite has no meaningful behavior and is rejected at
//!     load time, an added validation beyond what was strictly asked for,
//!     kept for the same reason `fsm_node.rs` added its own extra
//!     validations: it prevents an otherwise-nonsensical tree from ever
//!     being live).
//!   - `{"type": "repeat", "count": u32 (>= 1), "child": <node>}`.
//!   - `{"type": "invert", "child": <node>}`.
//!   - `{"type": "action", "target": {"joint_names": [...], "positions":
//!     [...]}, "epsilon": f64 (>= 0), "timeout": f64 (> 0), "slot": i64 |
//!     null}` -- a leaf. `slot` is pure round-trip bookkeeping (which
//!     memory-panel slot a front-end resolved this target from), never
//!     read by this node's own execution logic, exactly the same
//!     "already-resolved positions, `slot` is just metadata" division of
//!     responsibility `fsm_node.rs`'s own module doc comment explains at
//!     length for its states -- see that file for the full rationale (in
//!     short: this node must be able to run an entire tree without
//!     `pose_memory_node` staying reachable, so the *front-end* resolves
//!     "slot N" -> `{joint_names, positions}` once, at save time).
//!
//! `/bt/update_logic`'s own validation (see [`parse_and_validate_node`])
//! is the *only* place a raw `"type"` string is ever compared against
//! anything -- once a tree passes validation it is parsed into [`BtNode`],
//! a genuine Rust enum with exactly five variants, one per kind above.
//! Every tick thereafter dispatches on that enum, not the JSON string --
//! see "The hierarchical switch-case dispatch" below for why that is a
//! deliberate, and arguably *better*, way to teach the same lesson
//! `fsm_node.rs`'s flat dispatch teaches.
//!
//!
//! ## Three further node kinds, for planning and executing motion
//!
//! Beyond the five above, this interpreter has three kinds that exist to
//! drive `rrt_connect_node` and `joint_trajectory_action_node`:
//!
//!   - `{"type": "plan", "goal": [f64, ...], "output_key": str,
//!      "start": [f64, ...] (optional), "seed": int, "max_iterations":
//!      int, "base_dofs": bool, "timeout": f64}` -- asks
//!      `rrt_connect_node` for a motion plan over the robot's full
//!      configuration space and writes the resulting trajectory into the
//!      blackboard under `output_key`. `Success` once a plan arrives,
//!      `Failure` if the planner reports no path or the attempt exceeds
//!      `timeout`.
//!   - `{"type": "follow_trajectory", "input_key": str, "epsilon": f64,
//!      "timeout": f64}` -- reads a trajectory out of the blackboard and
//!      hands it to `joint_trajectory_action_node`, which walks it one
//!      point at a time. `Success` when that action succeeds, `Failure`
//!      if it aborts, is preempted, or does not finish within `timeout`.
//!   - `{"type": "condition", "check": "state_validity", "positions":
//!      [f64, ...], "invert": bool (optional)}` -- asks
//!      `planning_scene_node` whether a configuration is valid.
//!      `Success`/`Failure`, never `Running`. A `condition` under a
//!      `selector` is how a tree notices the world changed and takes its
//!      fallback branch.
//!
//! ## The blackboard, and why the five original kinds never needed one
//!
//! Every one of the five kinds above carries an **already-resolved**
//! target: a front end looks up "slot 3" once, at save time, and bakes
//! the joint positions into the tree. Nothing a leaf computes is ever
//! needed by another leaf, so there is nothing to pass and no mechanism
//! is required. That is a real design, not an omission -- see the
//! `action` leaf's own `slot` field above.
//!
//! `plan` breaks it. A plan is *computed at tick time*, cannot be baked
//! into the tree, and a sibling `follow_trajectory` needs it. So this
//! node has a **blackboard**: named slots that `plan` writes
//! (`output_key`) and `follow_trajectory` reads (`input_key`).
//!
//! The load-time-validation discipline extends to it exactly.
//! `/bt/update_logic` is still the only place a raw `"type"` string is
//! compared against anything, and it additionally **rejects a tree whose
//! `input_key` no earlier node writes**, with a message naming the key.
//! A blackboard that fails at tick time instead of load time is a
//! debugging trap: the tree loads, runs, appears fine, and then one
//! branch dies on a typo three minutes in.
//!
//! One honest limitation, stated rather than papered over: pre-order
//! "some earlier node writes it" is necessary but not sufficient. Under
//! a `selector`, an earlier branch may never actually run, so a key it
//! would have produced can genuinely be absent at tick time. Load-time
//! validation catches every typo and every never-written key -- the
//! overwhelming majority of real mistakes -- and the tick-time read
//! additionally returns a clear `Failure` naming the key rather than
//! panicking.
//!
//! ## A tick is polling: no leaf may block
//!
//! A behavior tree is **polled**. The root is ticked at a fixed rate and
//! every node must answer immediately, returning `Running` if it is not
//! done. A leaf that blocks stops the whole tree: no other branch is
//! evaluated, `/bt/status` stops updating, and preemption becomes
//! impossible for as long as the block lasts.
//!
//! This matters most for `plan`, because a motion plan takes **seconds**
//! -- thousands of service round trips (see `rrt_connect_node`'s doc
//! comment). So `plan` does not call `/plan_motion` on the tick thread.
//! On its first tick it spawns a worker thread that owns its own `Node`
//! and returns `Running` immediately; every later tick polls an `mpsc`
//! channel. This is the same "network I/O on its own thread, talking to
//! the ordinary `spin_once()` loop over channels" pattern
//! `robot_bridge_node` already uses, and for the same reason.
//!
//! `follow_trajectory` is an action **client**: it calls `send_goal`
//! once (a single bounded request/response), then returns `Running` on
//! every subsequent tick until a result arrives on
//! `/joint_trajectory_action/result`. It never waits for execution.
//!
//! `condition` *does* make its service call on the tick thread. That is
//! a deliberate exception and a bounded one: `/check_state_validity` is
//! a single round trip answering in about a millisecond, with an
//! explicit timeout as its ceiling. Spawning a thread per condition tick
//! would cost more than the call.
//!
//! ## Runtime state: the wrinkle `fsm_node.rs` didn't have
//!
//! `fsm_node`'s flat table only ever needed one integer
//! (`current_index`) to know where execution was. A tree needs much more:
//! a `sequence`/`selector` must remember *which child index it is
//! currently on* (so a later tick resumes there, not from 0), and a
//! `repeat` must remember *how many cumulative child successes it has
//! seen so far*. Both are properties of a *particular position in the
//! tree*, not of the tree's static shape -- so this node keeps a second,
//! parallel tree, [`BtState`], whose own shape exactly mirrors [`BtNode`]'s
//! (one [`BtState`] variant per [`BtNode`] variant, with the same nesting),
//! built once by [`default_state_for`] every time a tree is (re)loaded.
//! `tick_node`'s recursion always walks `(&BtNode, &mut BtState)` together,
//! one pair of trees in lockstep, rather than only the data tree.
//!
//! The one rule that keeps this whole design simple:  **whenever a node's
//! own tick result is decisive (`Success` or `Failure`, i.e. anything but
//! `Running`), that node's own [`BtState`] subtree is immediately reset
//! back to [`default_state_for`]'s fresh value**, applied uniformly by
//! `tick_node`'s outer wrapper for every node kind, not hand-implemented
//! separately per composite. This single rule is what makes a `repeat`'s
//! child come back fresh (re-publishing its `/joint_trajectory`, restarting
//! its own timeout clock) for each new cycle, and what makes a `sequence`/
//! `selector` that has just concluded (all children succeeded/failed) ready
//! to run from a clean slate the next time anything ticks it again --
//! without `repeat`, `sequence`, or `selector` ever needing their own
//! bespoke "now go clean up my children" code, since each child already
//! cleaned *itself* up the moment it returned its own decisive result, one
//! level down.
//!
//! ## The hierarchical switch-case dispatch, and its contrast with `fsm_node`
//!
//! [`fsm_node.rs`]'s dispatch is *flat and generated*: state identity is a
//! bare, meaningless `usize` index with no structure the compiler could
//! check ahead of time, so that file's `state_dispatch!` macro expands a
//! literal `match` with one mechanically-generated arm per index
//! `0..MAX_STATES`, checked at *tick* time. A behavior tree's node
//! *kind*, in contrast, is not arbitrary, unbounded data the way a state
//! index is -- it is drawn from a small, fixed, known-in-advance set
//! (exactly five kinds). That difference is why this node's dispatch looks
//! different, and is, if anything, a *better* illustration of "switch-case
//! over kind": [`parse_and_validate_node`] is the one place a raw JSON
//! `"type"` string is ever inspected, translating it *once*, at load time,
//! into [`BtNode`] -- a genuine Rust `enum` the compiler can (and does)
//! exhaustively check every `match` against, forever after, with no
//! `other => ...` catch-all arm ever needed or possible at tick time (an
//! unrecognized kind simply can never reach `tick_node` -- it was already
//! rejected by `/bt/update_logic` before ever becoming live). [`tick_node`]
//! is consequently a small, fixed, five-armed `match` on [`BtNode`],
//! recursing into children for the three composite/decorator kinds -- so
//! the *whole tree*, at every depth, is walked by exactly this one
//! function, making the interpreter genuinely hierarchical (unlike
//! `fsm_node`'s genuinely flat one) while staying just as fixed a piece of
//! Rust source, never regenerated or rebuilt per tree.
//!
//! ## Composite/decorator semantics chosen (and why)
//!
//!   - **`sequence`**: ticks children starting from its own remembered
//!     `current_index`. A child's `Success` immediately advances to and
//!     ticks the *next* child, in the same external tick (the standard
//!     "memory sequence" cascade most BT libraries, e.g.
//!     BehaviorTree.CPP's/py_trees' default `Sequence`, use -- chosen over
//!     an alternative "advance the index but wait for the next external
//!     tick to actually tick it" variant because that alternative would
//!     make an already-instantly-satisfied child stall the whole sequence
//!     for a full tick period for no reason, and would report a confusing
//!     "the sequence is Running" status with no child actually mid-flight
//!     underneath it). A child's `Failure` fails the sequence immediately.
//!     `Success` only once every child has succeeded in the same run.
//!   - **`selector`** (a.k.a. fallback): the exact dual -- a child's
//!     `Failure` advances to the next child (same-tick cascade, same
//!     rationale as above); a child's `Success` succeeds the selector
//!     immediately; `Failure` only once every child has failed.
//!   - **`repeat`**: ticks its child **at most once per external tick**
//!     (no cascading -- see below for why this one decorator does *not*
//!     cascade the way `sequence`/`selector` do). `Failure` from the child
//!     fails `repeat` immediately (resetting its own cycle count to 0).
//!     `Success` from the child increments a cumulative success counter;
//!     once that counter reaches `count`, `repeat` itself succeeds
//!     (counter resets to 0 via the uniform reset-on-decisive-result rule
//!     above); otherwise `repeat` reports `Running` for *this* tick and
//!     the child -- already reset fresh, per that same uniform rule, the
//!     instant it returned `Success` -- gets a brand new attempt (a fresh
//!     `/joint_trajectory` publish, a restarted timeout clock, for an
//!     `action` leaf) on the *next* external tick. This is exactly
//!     BehaviorTree.CPP's own `RepeatNode` semantics ("repeat N times,
//!     fail fast"), chosen specifically (over an equally-plausible
//!     "repeat until the child fails" variant some other BT libraries call
//!     `Repeat`) because "repeat" is genuinely ambiguous across BT
//!     libraries and this project needs exactly one, documented, meaning.
//!     Not cascading within one external tick (unlike `sequence`/
//!     `selector`) is deliberate: cascading here could otherwise spin an
//!     entire `count`-cycle loop of trivially-instant-success children to
//!     completion within a single tick, which is a much larger behavioral
//!     surprise for a decorator whose whole point is pacing repetition
//!     over *time*, not just over tree-evaluation order.
//!   - **`invert`**: ticks its child once; `Running` passes through
//!     unchanged; `Success`/`Failure` are swapped.
//!
//! ## `action` leaves: arrival convention (reused from `fsm_node`)
//!
//! Identical convention to `fsm_node.rs`'s own arrival check --
//! `max(|target_position_i - actual_position_i|)` over joint-name-matched
//! pairs (order-independent set match; a mismatched joint set is simply
//! ignored, not treated as arrived or as an error), no angle wrapping (see
//! that file's own module doc comment for the full justification, which
//! applies here verbatim -- this is an arrival *check*, not a control
//! law). On an `action` leaf's first tick since its own state was last
//! reset (`started_at` transitioning from `None` to `Some`), it publishes
//! one fresh one-point `/joint_trajectory` toward `target` -- never again
//! for the rest of that visit, the same "publish once per entry, not per
//! tick" discipline `arm_sim_node.rs`/`fsm_node.rs` both already establish.
//! Every following tick: `Success` the moment arrival is detected;
//! `Failure` if `timeout` **wall-clock** seconds have elapsed since that
//! first tick without arrival; otherwise `Running`. Wall-clock time (via
//! `std::time::Instant`, the same clock `ik_trial_node.rs`'s own trial
//! timer uses), not simulation time, is used deliberately: unlike
//! `ik_action_node.rs`'s dwell-time convergence check (which needs
//! simulation time so its behavior doesn't depend on real-time factor),
//! this timeout's whole job is "how long is this node personally willing
//! to wait, in real seconds, before giving up and letting a `selector`
//! try something else" -- a patience budget on the *caller's* clock, not a
//! property of the simulated system being observed.
//!
//! ## Communication
//!
//!   - Subscribes `/joint_states`, keeping only the latest sample -- same
//!     pattern as `pose_memory_node.rs`/`fsm_node.rs`.
//!   - Publishes `/joint_trajectory` -- see the `action` section above.
//!   - Offers `/bt/update_logic` (request `{"tree": <node>}`) -> standard
//!     `{values, result, status}`. Validates the *entire* tree recursively
//!     (unknown `type`; missing type-specific fields; `repeat.count < 1`;
//!     mismatched `target.joint_names`/`positions` lengths; a composite
//!     with zero children) before touching anything live -- on any
//!     failure the previously loaded tree keeps running completely
//!     untouched, the same guarantee `fsm_node.rs`'s own
//!     `/fsm/update_logic` makes. On success: the live tree and its
//!     runtime state are atomically replaced (state freshly built via
//!     [`default_state_for`]) and the new tree is ticked once immediately,
//!     so `/bt/status` and `values.root_result` both reflect real,
//!     current information rather than a stale "nothing has run yet."
//!   - Offers `/bt/get_logic` (empty request `{}`) -> `{"values": {"tree":
//!     <node>}, result, status}`, the current tree verbatim (including
//!     every `action` leaf's own `slot`). `result: false` if nothing has
//!     ever been loaded.
//!   - Publishes `/bt/status` once per tick (see `--tick-rate`):
//!     `{"tick": int, "root_result": "running"|"success"|"failure"|null,
//!     "running_paths": [[<child index>, ...], ...], "visited":
//!     [{"path": [...], "type": str, "result": str}, ...],
//!     "blackboard": {str: <value>}, "blackboard_keys": [str, ...]}`.
//!     `tick`, `visited`, `blackboard`, and `blackboard_keys` were added
//!     alongside the motion node kinds: an overall
//!     running/success/failure is not enough to watch the active node
//!     move through a tree while the blackboard fills in, which is the
//!     thing worth watching. They are purely additive -- every field the
//!     original five-kind interpreter published is still published,
//!     unchanged. A **path** is the list of child
//!     indices to follow from the root to reach some node (`[]` is the
//!     root itself; a `repeat`/`invert`'s single child is always index
//!     `0`). `running_paths` lists the path of **every** node (composite,
//!     decorator, *and* leaf) whose own tick this cycle evaluated to
//!     `Running` -- not just the leaf. Because exactly one child branch of
//!     any `sequence`/`selector` can ever be `Running` at a time,
//!     collecting every `Running` node this way always yields precisely
//!     the unbroken chain of paths from the root down to whichever leaf is
//!     currently in flight (often more than one leaf if the design ever
//!     nested independent subtrees -- not possible with only `sequence`/
//!     `selector`/`repeat`/`invert`/`action`, but the shape supports it
//!     without change). `root_result` is `null` before any tree has ever
//!     been loaded; `running_paths` is `[]` in that case too. A front-end
//!     can highlight the *entire* live path directly from this list with
//!     no further computation of its own -- the explicit design goal
//!     behind reporting every level, not just leaves.
//!
//! ## CLI
//!
//! `--tick-rate` (default [`DEFAULT_TICK_RATE_HZ`]) controls how often the
//! main loop calls into the root [`tick_node`] *and* how often
//! `/bt/status` is republished -- both are governed by the same knob here,
//! unlike `fsm_node.rs` (whose own arrival-checking ran every ~20ms
//! iteration unconditionally, with only its *status publish* throttled
//! separately). That difference is deliberate: re-evaluating an entire
//! tree from the root down is real, non-trivial work (unlike `fsm_node`'s
//! single flat arrival check), and nothing about an `action` leaf's
//! arrival-or-timeout check benefits from being re-evaluated any faster
//! than a human (or this node's own timeout budgets) would ever notice.

use pubsub_testbed::p2p::node::{Node, PendingRequest, Publisher, Service, Subscriber};
use pubsub_testbed::ros_messages as rm;
use serde_json::{json, Value};
use std::collections::{HashMap, HashSet};
use std::sync::mpsc::{Receiver, TryRecvError};
use std::time::{Duration, Instant};

const DEFAULT_TICK_RATE_HZ: f64 = 10.0;

/// Ceiling on one `/check_state_validity` call from a `condition` leaf,
/// and on `follow_trajectory`'s one `send_goal` call. Short: these are
/// the two places this node talks to another on the tick thread, so the
/// timeout *is* the bound on how long a tick can take.
const QUICK_SERVICE_TIMEOUT: Duration = Duration::from_secs(2);

/// Ceiling on the `/plan_motion` call a `plan` leaf's worker thread
/// makes. Generous, because it is off the tick thread and a real plan is
/// thousands of round trips; a `plan` node's own `timeout` is what the
/// tree uses to give up earlier.
const PLAN_SERVICE_TIMEOUT: Duration = Duration::from_secs(120);

// -- #region bt-node-kinds ------------------------------------------------

/// A fully validated behavior-tree node. Constructed only by
/// [`parse_and_validate_node`] -- the sole place a raw JSON `"type"`
/// string is compared against anything in this file, exactly as in
/// `bt_node.rs`.
#[derive(Clone, Debug)]
enum BtNode {
    Sequence { children: Vec<BtNode> },
    Selector { children: Vec<BtNode> },
    Repeat { count: u32, child: Box<BtNode> },
    Invert { child: Box<BtNode> },
    Action { target_joint_names: Vec<String>, target_positions: Vec<f64>, epsilon: f64, timeout: f64, slot: Option<i64> },
    Plan { start: Option<Vec<f64>>, goal: Vec<f64>, output_key: String, seed: u64, max_iterations: Option<u64>, base_dofs: bool, timeout: f64 },
    FollowTrajectory { input_key: String, epsilon: f64, timeout: f64 },
    Condition { check: ConditionCheck, positions: Vec<f64>, invert: bool },
}

/// What a `condition` leaf asks about. An enum with one variant rather
/// than a bare marker, so adding a second kind of check is a compile
/// error at every match site instead of a new string compared somewhere
/// at tick time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ConditionCheck {
    StateValidity,
}

impl ConditionCheck {
    fn as_str(self) -> &'static str {
        match self {
            ConditionCheck::StateValidity => "state_validity",
        }
    }
}

impl BtNode {
    /// The node's own kind as a string, for `/bt/status`'s
    /// per-node reporting. Derived from the enum, never compared against.
    fn kind(&self) -> &'static str {
        match self {
            BtNode::Sequence { .. } => "sequence",
            BtNode::Selector { .. } => "selector",
            BtNode::Repeat { .. } => "repeat",
            BtNode::Invert { .. } => "invert",
            BtNode::Action { .. } => "action",
            BtNode::Plan { .. } => "plan",
            BtNode::FollowTrajectory { .. } => "follow_trajectory",
            BtNode::Condition { .. } => "condition",
        }
    }
}

// -- #endregion bt-node-kinds ---------------------------------------------

// -- #region bt-runtime-state ---------------------------------------------

/// The mutable companion to [`BtNode`], shape for shape. See
/// `bt_node.rs`'s "Runtime state" section.
#[derive(Debug)]
enum BtState {
    Sequence { current_index: usize, child_states: Vec<BtState> },
    Selector { current_index: usize, child_states: Vec<BtState> },
    Repeat { successes: u32, child_state: Box<BtState> },
    Invert { child_state: Box<BtState> },
    Action { started_at: Option<Instant> },
    /// A `plan` in flight. `worker` is the channel its thread will send
    /// its one result down; dropping this state (which the uniform
    /// reset-on-decisive-result rule does) drops the receiver, and the
    /// orphaned thread's send simply fails -- which is the correct
    /// outcome for a plan nobody is waiting for any more.
    Plan { worker: Option<Receiver<Result<Value, String>>>, started_at: Option<Instant> },
    FollowTrajectory { goal_id: Option<String>, started_at: Option<Instant> },
    Condition,
}

fn default_state_for(node: &BtNode) -> BtState {
    match node {
        BtNode::Sequence { children } => BtState::Sequence { current_index: 0, child_states: children.iter().map(default_state_for).collect() },
        BtNode::Selector { children } => BtState::Selector { current_index: 0, child_states: children.iter().map(default_state_for).collect() },
        BtNode::Repeat { child, .. } => BtState::Repeat { successes: 0, child_state: Box::new(default_state_for(child)) },
        BtNode::Invert { child } => BtState::Invert { child_state: Box::new(default_state_for(child)) },
        BtNode::Action { .. } => BtState::Action { started_at: None },
        BtNode::Plan { .. } => BtState::Plan { worker: None, started_at: None },
        BtNode::FollowTrajectory { .. } => BtState::FollowTrajectory { goal_id: None, started_at: None },
        BtNode::Condition { .. } => BtState::Condition,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TickResult {
    Running,
    Success,
    Failure,
}

impl TickResult {
    fn as_str(self) -> &'static str {
        match self {
            TickResult::Running => "running",
            TickResult::Success => "success",
            TickResult::Failure => "failure",
        }
    }
}

// -- #endregion bt-runtime-state ------------------------------------------

#[derive(Clone, Debug)]
struct JointSample {
    joint_names: Vec<String>,
    positions: Vec<f64>,
}

fn parse_joint_state(msg: &Value) -> Option<JointSample> {
    let names = msg.get("name")?.as_array()?;
    let positions = msg.get("position")?.as_array()?;
    let joint_names: Vec<String> = names.iter().filter_map(|v| v.as_str().map(str::to_string)).collect();
    let positions: Vec<f64> = positions.iter().filter_map(Value::as_f64).collect();
    if joint_names.len() != names.len() || positions.len() != names.len() {
        return None;
    }
    Some(JointSample { joint_names, positions })
}

/// Same convention as `bt_node.rs` and `joint_trajectory_action_node.rs`.
fn max_joint_error(target_names: &[String], target_positions: &[f64], sample: &JointSample) -> Option<f64> {
    if target_names.len() != sample.joint_names.len() {
        return None;
    }
    let mut target_sorted = target_names.to_vec();
    let mut sample_sorted = sample.joint_names.clone();
    target_sorted.sort();
    sample_sorted.sort();
    if target_sorted != sample_sorted {
        return None;
    }
    let mut max_error = 0.0_f64;
    for (i, name) in target_names.iter().enumerate() {
        let sample_index = sample.joint_names.iter().position(|n| n == name)?;
        max_error = max_error.max((target_positions[i] - sample.positions[sample_index]).abs());
    }
    Some(max_error)
}

// -- #region blackboard-validation ----------------------------------------

fn read_f64_array(value: &Value, key: &str, context: &str) -> Result<Vec<f64>, String> {
    let array = value.get(key).and_then(Value::as_array).ok_or_else(|| format!("a {context:?} node is missing a {key:?} array of numbers"))?;
    let out: Vec<f64> = array.iter().filter_map(Value::as_f64).collect();
    if out.len() != array.len() {
        return Err(format!("a {context:?} node's {key:?} contains a non-number"));
    }
    Ok(out)
}

fn read_key(value: &Value, key: &str, context: &str) -> Result<String, String> {
    let text = value.get(key).and_then(Value::as_str).ok_or_else(|| format!("a {context:?} node is missing a string {key:?}"))?;
    if text.is_empty() {
        return Err(format!("a {context:?} node's {key:?} must not be empty"));
    }
    Ok(text.to_string())
}

fn parse_children(value: &Value, kind: &str, produced: &mut HashSet<String>) -> Result<Vec<BtNode>, String> {
    let raw = value.get("children").and_then(Value::as_array).ok_or_else(|| format!("a {kind:?} node is missing a \"children\" array"))?;
    if raw.is_empty() {
        return Err(format!("a {kind:?} node must have at least one child"));
    }
    raw.iter().map(|child| parse_and_validate_node(child, produced)).collect()
}

/// Parses and fully validates one node, threading `produced` -- the set
/// of blackboard keys written by nodes **earlier in pre-order** -- down
/// the walk. See this file's doc comment for why blackboard keys are
/// validated here rather than at tick time, and for the one limitation
/// of pre-order validation under a `selector`.
fn parse_and_validate_node(value: &Value, produced: &mut HashSet<String>) -> Result<BtNode, String> {
    let node_type = value.get("type").and_then(Value::as_str).ok_or("a node is missing a string \"type\"")?;
    match node_type {
        "sequence" => Ok(BtNode::Sequence { children: parse_children(value, "sequence", produced)? }),
        "selector" => Ok(BtNode::Selector { children: parse_children(value, "selector", produced)? }),
        "repeat" => {
            let count = value.get("count").and_then(Value::as_u64).ok_or("a \"repeat\" node is missing an integer \"count\"")?;
            if count < 1 {
                return Err("a \"repeat\" node's \"count\" must be at least 1".to_string());
            }
            let child = value.get("child").ok_or("a \"repeat\" node is missing a \"child\"")?;
            Ok(BtNode::Repeat { count: count as u32, child: Box::new(parse_and_validate_node(child, produced)?) })
        }
        "invert" => {
            let child = value.get("child").ok_or("an \"invert\" node is missing a \"child\"")?;
            Ok(BtNode::Invert { child: Box::new(parse_and_validate_node(child, produced)?) })
        }
        "action" => {
            let target = value.get("target").ok_or("an \"action\" node is missing \"target\"")?;
            let names = target.get("joint_names").and_then(Value::as_array).ok_or("an \"action\" node's target is missing a \"joint_names\" array")?;
            let target_joint_names: Vec<String> = names.iter().filter_map(|v| v.as_str().map(str::to_string)).collect();
            let target_positions = read_f64_array(target, "positions", "action")?;
            if target_joint_names.len() != target_positions.len() {
                return Err("an \"action\" node's target \"joint_names\" and \"positions\" must be the same length".to_string());
            }
            let epsilon = value.get("epsilon").and_then(Value::as_f64).ok_or("an \"action\" node is missing a numeric \"epsilon\"")?;
            if epsilon < 0.0 {
                return Err("an \"action\" node's \"epsilon\" must be non-negative".to_string());
            }
            let timeout = value.get("timeout").and_then(Value::as_f64).ok_or("an \"action\" node is missing a numeric \"timeout\"")?;
            if timeout <= 0.0 {
                return Err("an \"action\" node's \"timeout\" must be positive".to_string());
            }
            let slot = match value.get("slot") {
                None | Some(Value::Null) => None,
                Some(v) => Some(v.as_i64().ok_or("an \"action\" node's \"slot\", if present, must be an integer")?),
            };
            Ok(BtNode::Action { target_joint_names, target_positions, epsilon, timeout, slot })
        }
        "plan" => {
            let goal = read_f64_array(value, "goal", "plan")?;
            if goal.is_empty() {
                return Err("a \"plan\" node's \"goal\" must not be empty".to_string());
            }
            let start = match value.get("start") {
                None | Some(Value::Null) => None,
                Some(_) => Some(read_f64_array(value, "start", "plan")?),
            };
            let base_dofs = value.get("base_dofs").and_then(Value::as_bool).unwrap_or(false);
            if base_dofs && start.is_none() {
                // With base planning on, the start includes base
                // coordinates that /joint_states knows nothing about, so
                // there is no honest way to default it.
                return Err("a \"plan\" node with \"base_dofs\" must give an explicit \"start\" (joint states carry no base pose)".to_string());
            }
            let output_key = read_key(value, "output_key", "plan")?;
            let timeout = value.get("timeout").and_then(Value::as_f64).unwrap_or(60.0);
            if timeout <= 0.0 {
                return Err("a \"plan\" node's \"timeout\" must be positive".to_string());
            }
            produced.insert(output_key.clone());
            Ok(BtNode::Plan {
                start,
                goal,
                output_key,
                seed: value.get("seed").and_then(Value::as_u64).unwrap_or(0),
                max_iterations: value.get("max_iterations").and_then(Value::as_u64),
                base_dofs,
                timeout,
            })
        }
        "follow_trajectory" => {
            let input_key = read_key(value, "input_key", "follow_trajectory")?;
            if !produced.contains(&input_key) {
                // The load-time blackboard rule. Naming the key matters:
                // "a key is missing" sends someone reading the whole
                // tree, "input_key \"trajecotry\" is never written"
                // sends them to the typo.
                return Err(format!(
                    "a \"follow_trajectory\" node reads input_key {input_key:?}, which no earlier node writes as an \"output_key\""
                ));
            }
            let epsilon = value.get("epsilon").and_then(Value::as_f64).unwrap_or(0.05);
            if epsilon < 0.0 {
                return Err("a \"follow_trajectory\" node's \"epsilon\" must be non-negative".to_string());
            }
            let timeout = value.get("timeout").and_then(Value::as_f64).unwrap_or(60.0);
            if timeout <= 0.0 {
                return Err("a \"follow_trajectory\" node's \"timeout\" must be positive".to_string());
            }
            Ok(BtNode::FollowTrajectory { input_key, epsilon, timeout })
        }
        "condition" => {
            let check_name = value.get("check").and_then(Value::as_str).unwrap_or(ConditionCheck::StateValidity.as_str());
            let check = match check_name {
                "state_validity" => ConditionCheck::StateValidity,
                other => return Err(format!("a \"condition\" node's \"check\" must be \"state_validity\", got {other:?}")),
            };
            let positions = read_f64_array(value, "positions", "condition")?;
            Ok(BtNode::Condition { check, positions, invert: value.get("invert").and_then(Value::as_bool).unwrap_or(false) })
        }
        other => Err(format!(
            "unknown node type {other:?} (expected \"sequence\", \"selector\", \"repeat\", \"invert\", \"action\", \"plan\", \"follow_trajectory\", or \"condition\")"
        )),
    }
}

/// Validates a whole tree, starting from an empty set of produced
/// blackboard keys.
fn parse_and_validate_tree(value: &Value) -> Result<BtNode, String> {
    let mut produced = HashSet::new();
    parse_and_validate_node(value, &mut produced)
}

/// The inverse of [`parse_and_validate_node`] -- what `/bt/get_logic`
/// hands back.
fn node_to_json(node: &BtNode) -> Value {
    match node {
        BtNode::Sequence { children } => json!({ "type": "sequence", "children": children.iter().map(node_to_json).collect::<Vec<_>>() }),
        BtNode::Selector { children } => json!({ "type": "selector", "children": children.iter().map(node_to_json).collect::<Vec<_>>() }),
        BtNode::Repeat { count, child } => json!({ "type": "repeat", "count": count, "child": node_to_json(child) }),
        BtNode::Invert { child } => json!({ "type": "invert", "child": node_to_json(child) }),
        BtNode::Action { target_joint_names, target_positions, epsilon, timeout, slot } => json!({
            "type": "action",
            "target": { "joint_names": target_joint_names, "positions": target_positions },
            "epsilon": epsilon, "timeout": timeout, "slot": slot,
        }),
        BtNode::Plan { start, goal, output_key, seed, max_iterations, base_dofs, timeout } => json!({
            "type": "plan", "start": start, "goal": goal, "output_key": output_key,
            "seed": seed, "max_iterations": max_iterations, "base_dofs": base_dofs, "timeout": timeout,
        }),
        BtNode::FollowTrajectory { input_key, epsilon, timeout } => json!({
            "type": "follow_trajectory", "input_key": input_key, "epsilon": epsilon, "timeout": timeout,
        }),
        BtNode::Condition { check, positions, invert } => json!({
            "type": "condition", "check": check.as_str(), "positions": positions, "invert": invert,
        }),
    }
}

// -- #endregion blackboard-validation -------------------------------------

// -- #region non-blocking-leaves ------------------------------------------

/// Everything a tick needs, bundled so [`tick_node`] takes one context
/// rather than nine parameters.
struct TickContext<'a> {
    hub_endpoint: (String, u16),
    node: &'a Node,
    sample: Option<&'a JointSample>,
    trajectory_publisher: &'a mut Publisher,
    blackboard: &'a mut HashMap<String, Value>,
    /// Results seen on `/joint_trajectory_action/result`, keyed by goal
    /// id, so a `follow_trajectory` leaf can find its own.
    action_results: &'a HashMap<String, Value>,
    running_paths: &'a mut Vec<Vec<usize>>,
    visited: &'a mut Vec<Value>,
}

/// Ticks an `action` leaf -- identical to `bt_node.rs`'s.
fn tick_action(
    target_joint_names: &[String],
    target_positions: &[f64],
    epsilon: f64,
    timeout: f64,
    started_at: &mut Option<Instant>,
    context: &mut TickContext<'_>,
) -> TickResult {
    if started_at.is_none() {
        *started_at = Some(Instant::now());
        let n = target_positions.len();
        let point = rm::joint_trajectory_point(target_positions, &vec![0.0; n], &vec![0.0; n], 0, 0);
        context.trajectory_publisher.publish(&rm::joint_trajectory(target_joint_names, vec![point], ""));
    }
    if let Some(sample) = context.sample {
        if let Some(error) = max_joint_error(target_joint_names, target_positions, sample) {
            if error < epsilon {
                return TickResult::Success;
            }
        }
    }
    if started_at.expect("just set above if it was None").elapsed().as_secs_f64() >= timeout {
        return TickResult::Failure;
    }
    TickResult::Running
}

/// Ticks a `plan` leaf.
///
/// The first tick spawns a worker and returns `Running` **without
/// waiting**; later ticks poll. See this file's doc comment on why a
/// blocking plan would stop the whole tree.
#[allow(clippy::too_many_arguments)]
fn tick_plan(
    start: &Option<Vec<f64>>,
    goal: &[f64],
    output_key: &str,
    seed: u64,
    max_iterations: Option<u64>,
    base_dofs: bool,
    timeout: f64,
    worker: &mut Option<Receiver<Result<Value, String>>>,
    started_at: &mut Option<Instant>,
    context: &mut TickContext<'_>,
) -> TickResult {
    if worker.is_none() && started_at.is_none() {
        let mut request = json!({ "goal": goal, "seed": seed, "base_dofs": base_dofs });
        if let Some(iterations) = max_iterations {
            request["max_iterations"] = json!(iterations);
        }
        match resolve_plan_start(start, context.sample, goal.len()) {
            Ok(resolved) => request["start"] = json!(resolved),
            Err(message) => {
                eprintln!("[bt_node] plan leaf cannot determine a start configuration: {message}");
                return TickResult::Failure;
            }
        }

        let (sender, receiver) = std::sync::mpsc::channel();
        let hub_endpoint = context.hub_endpoint.clone();
        // The worker owns its own Node rather than borrowing this one:
        // a Node is just a name, an id, and the hub's address, and
        // call_service opens its own connections, so a second one costs
        // nothing and avoids sharing anything across the thread boundary.
        std::thread::spawn(move || {
            let worker_node = Node::new("bt_node_plan_worker", hub_endpoint);
            let outcome = match worker_node.call_service("/plan_motion", &request, PLAN_SERVICE_TIMEOUT) {
                Ok(response) if response.get("result").and_then(Value::as_bool) == Some(true) => Ok(response["values"].clone()),
                Ok(response) => Err(response.get("status").and_then(Value::as_str).unwrap_or("/plan_motion refused the request").to_string()),
                Err(e) => Err(format!("could not reach rrt_connect_node's /plan_motion: {e}")),
            };
            // A send failure means the tree moved on and dropped the
            // receiver -- expected, and nothing to report.
            let _ = sender.send(outcome);
        });

        *worker = Some(receiver);
        *started_at = Some(Instant::now());
        return TickResult::Running;
    }

    let Some(receiver) = worker.as_ref() else { return TickResult::Failure };
    match receiver.try_recv() {
        Ok(Ok(values)) => {
            if values.get("found").and_then(Value::as_bool) != Some(true) {
                eprintln!("[bt_node] plan leaf failed: {}", values.get("status").and_then(Value::as_str).unwrap_or("no path"));
                return TickResult::Failure;
            }
            let Some(trajectory) = values.get("trajectory") else {
                eprintln!("[bt_node] plan leaf: /plan_motion reported success with no trajectory");
                return TickResult::Failure;
            };
            println!(
                "[bt_node] plan leaf succeeded: {} points -> blackboard[{output_key:?}]",
                trajectory.get("points").and_then(Value::as_array).map(Vec::len).unwrap_or(0)
            );
            context.blackboard.insert(output_key.to_string(), trajectory.clone());
            TickResult::Success
        }
        Ok(Err(message)) => {
            eprintln!("[bt_node] plan leaf failed: {message}");
            TickResult::Failure
        }
        Err(TryRecvError::Empty) => {
            if started_at.is_some_and(|t| t.elapsed().as_secs_f64() >= timeout) {
                eprintln!("[bt_node] plan leaf gave up after {timeout}s");
                return TickResult::Failure;
            }
            TickResult::Running
        }
        Err(TryRecvError::Disconnected) => {
            eprintln!("[bt_node] plan leaf's worker thread vanished");
            TickResult::Failure
        }
    }
}

/// Works out what configuration a `plan` should start from: the explicit
/// one if the tree gave it, otherwise the latest `/joint_states` sample.
///
/// The sample is used **positionally** only when its width matches the
/// goal's, which is the honest limit of what this node can do: it has no
/// configuration space of its own (that lives in `planning_scene_node`),
/// so it cannot reorder a differently-ordered `/joint_states` into the
/// planner's degree-of-freedom order. A tree that cares should give an
/// explicit `start`.
fn resolve_plan_start(start: &Option<Vec<f64>>, sample: Option<&JointSample>, goal_len: usize) -> Result<Vec<f64>, String> {
    if let Some(explicit) = start {
        if explicit.len() != goal_len {
            return Err(format!("the plan leaf's start has {} values but its goal has {goal_len}", explicit.len()));
        }
        return Ok(explicit.clone());
    }
    let sample = sample.ok_or("no \"start\" was given and no /joint_states sample has arrived yet")?;
    if sample.positions.len() != goal_len {
        return Err(format!(
            "no \"start\" was given and the latest /joint_states has {} positions, which does not match the goal's {goal_len}",
            sample.positions.len()
        ));
    }
    Ok(sample.positions.clone())
}

/// Ticks a `follow_trajectory` leaf: an action **client**. Sends the
/// goal once, then reports `Running` until the action resolves.
fn tick_follow_trajectory(
    input_key: &str,
    epsilon: f64,
    timeout: f64,
    goal_id: &mut Option<String>,
    started_at: &mut Option<Instant>,
    context: &mut TickContext<'_>,
) -> TickResult {
    if goal_id.is_none() {
        // The tick-time half of the blackboard contract. Load-time
        // validation already proved some earlier node *writes* this key;
        // this catches the case it cannot -- an earlier branch under a
        // selector that never actually ran.
        let Some(trajectory) = context.blackboard.get(input_key) else {
            eprintln!("[bt_node] follow_trajectory: blackboard has no {input_key:?} yet (an earlier branch may not have run)");
            return TickResult::Failure;
        };
        let request = json!({ "trajectory": trajectory, "epsilon": epsilon, "timeout": timeout });
        match context.node.call_service("/joint_trajectory_action/send_goal", &request, QUICK_SERVICE_TIMEOUT) {
            Ok(response) if response.get("result").and_then(Value::as_bool) == Some(true) => {
                let id = response["values"]["goal_id"].as_str().unwrap_or_default().to_string();
                println!("[bt_node] follow_trajectory sent {id} from blackboard[{input_key:?}]");
                *goal_id = Some(id);
                *started_at = Some(Instant::now());
                return TickResult::Running;
            }
            Ok(response) => {
                eprintln!("[bt_node] follow_trajectory rejected: {}", response.get("status").and_then(Value::as_str).unwrap_or(""));
                return TickResult::Failure;
            }
            Err(e) => {
                eprintln!("[bt_node] follow_trajectory could not reach joint_trajectory_action_node: {e}");
                return TickResult::Failure;
            }
        }
    }

    let id = goal_id.as_ref().expect("just checked");
    if let Some(result) = context.action_results.get(id) {
        return match result.get("outcome").and_then(Value::as_str) {
            Some("succeeded") => TickResult::Success,
            // "preempted" and "aborted" are both failures for the tree,
            // but they are logged distinctly because they mean very
            // different things to whoever is debugging.
            Some(other) => {
                eprintln!("[bt_node] follow_trajectory goal {id} concluded as {other:?}");
                TickResult::Failure
            }
            None => TickResult::Failure,
        };
    }
    if started_at.is_some_and(|t| t.elapsed().as_secs_f64() >= timeout) {
        eprintln!("[bt_node] follow_trajectory goal {id} did not finish within {timeout}s; cancelling");
        let _ = context.node.call_service("/joint_trajectory_action/cancel_goal", &json!({ "goal_id": id }), QUICK_SERVICE_TIMEOUT);
        return TickResult::Failure;
    }
    TickResult::Running
}

/// Ticks a `condition` leaf. Never returns `Running` -- a condition is a
/// question with an answer, and a selector needs that answer in the
/// tick it asks.
fn tick_condition(check: ConditionCheck, positions: &[f64], invert: bool, context: &mut TickContext<'_>) -> TickResult {
    let valid = match check {
        ConditionCheck::StateValidity => match context.node.call_service("/check_state_validity", &json!({ "positions": positions }), QUICK_SERVICE_TIMEOUT) {
            Ok(response) if response.get("result").and_then(Value::as_bool) == Some(true) => response["values"]["valid"].as_bool().unwrap_or(false),
            Ok(_) => false,
            Err(e) => {
                // An unreachable checker reads as "not valid", never as
                // "valid" -- the same conservative choice rrt_connect_node
                // makes, and for the same reason.
                eprintln!("[bt_node] condition could not reach /check_state_validity: {e}");
                false
            }
        },
    };
    if valid != invert {
        TickResult::Success
    } else {
        TickResult::Failure
    }
}

// -- #endregion non-blocking-leaves ---------------------------------------

// -- #region hierarchical-dispatch ----------------------------------------

/// The one function that walks every level of the tree. See
/// `bt_node.rs`'s "hierarchical switch-case dispatch" section; the only
/// additions here are the three new leaf arms and the per-node record
/// pushed into `visited` for `/bt/status`.
fn tick_node(node: &BtNode, state: &mut BtState, path: &mut Vec<usize>, context: &mut TickContext<'_>) -> TickResult {
    let result = match (node, &mut *state) {
        (BtNode::Sequence { children }, BtState::Sequence { current_index, child_states }) => {
            let mut result = TickResult::Success;
            while *current_index < children.len() {
                path.push(*current_index);
                let child_result = tick_node(&children[*current_index], &mut child_states[*current_index], path, context);
                path.pop();
                match child_result {
                    TickResult::Running => {
                        result = TickResult::Running;
                        break;
                    }
                    TickResult::Failure => {
                        result = TickResult::Failure;
                        break;
                    }
                    TickResult::Success => *current_index += 1,
                }
            }
            result
        }
        (BtNode::Selector { children }, BtState::Selector { current_index, child_states }) => {
            let mut result = TickResult::Failure;
            while *current_index < children.len() {
                path.push(*current_index);
                let child_result = tick_node(&children[*current_index], &mut child_states[*current_index], path, context);
                path.pop();
                match child_result {
                    TickResult::Running => {
                        result = TickResult::Running;
                        break;
                    }
                    TickResult::Success => {
                        result = TickResult::Success;
                        break;
                    }
                    TickResult::Failure => *current_index += 1,
                }
            }
            result
        }
        (BtNode::Repeat { count, child }, BtState::Repeat { successes, child_state }) => {
            path.push(0);
            let child_result = tick_node(child, child_state, path, context);
            path.pop();
            match child_result {
                TickResult::Running => TickResult::Running,
                TickResult::Failure => TickResult::Failure,
                TickResult::Success => {
                    *successes += 1;
                    if *successes >= *count {
                        TickResult::Success
                    } else {
                        TickResult::Running
                    }
                }
            }
        }
        (BtNode::Invert { child }, BtState::Invert { child_state }) => {
            path.push(0);
            let child_result = tick_node(child, child_state, path, context);
            path.pop();
            match child_result {
                TickResult::Running => TickResult::Running,
                TickResult::Success => TickResult::Failure,
                TickResult::Failure => TickResult::Success,
            }
        }
        (BtNode::Action { target_joint_names, target_positions, epsilon, timeout, .. }, BtState::Action { started_at }) => {
            tick_action(target_joint_names, target_positions, *epsilon, *timeout, started_at, context)
        }
        (BtNode::Plan { start, goal, output_key, seed, max_iterations, base_dofs, timeout }, BtState::Plan { worker, started_at }) => {
            tick_plan(start, goal, output_key, *seed, *max_iterations, *base_dofs, *timeout, worker, started_at, context)
        }
        (BtNode::FollowTrajectory { input_key, epsilon, timeout }, BtState::FollowTrajectory { goal_id, started_at }) => {
            tick_follow_trajectory(input_key, *epsilon, *timeout, goal_id, started_at, context)
        }
        (BtNode::Condition { check, positions, invert }, BtState::Condition) => tick_condition(*check, positions, *invert, context),
        // BtState is always built from the very same BtNode tree by
        // default_state_for, so these can never disagree -- matched
        // explicitly rather than with `_` so adding a BtNode variant
        // without a matching BtState one is a compile error.
        (BtNode::Sequence { .. }, _)
        | (BtNode::Selector { .. }, _)
        | (BtNode::Repeat { .. }, _)
        | (BtNode::Invert { .. }, _)
        | (BtNode::Action { .. }, _)
        | (BtNode::Plan { .. }, _)
        | (BtNode::FollowTrajectory { .. }, _)
        | (BtNode::Condition { .. }, _) => unreachable!("BtState shape must always mirror BtNode shape (see default_state_for)"),
    };

    // Per-node reporting for /bt/status -- recorded for every
    // node the tick actually evaluated, not just leaves, so a front end
    // can colour the whole tree from one message.
    context.visited.push(json!({ "path": path.clone(), "type": node.kind(), "result": result.as_str() }));

    if result == TickResult::Running {
        context.running_paths.push(path.clone());
    } else {
        // The uniform reset-on-decisive-result rule, unchanged from
        // bt_node.rs. For a `plan` this drops the worker's receiver,
        // which is what makes a `repeat` over a plan re-plan rather than
        // reuse a stale trajectory.
        *state = default_state_for(node);
    }
    result
}

// -- #endregion hierarchical-dispatch -------------------------------------

struct BtNodeProcess {
    node: Node,
    hub_endpoint: (String, u16),
    joint_states_subscriber: Subscriber,
    action_result_subscriber: Subscriber,
    trajectory_publisher: Publisher,
    status_publisher: Publisher,
    update_logic_service: Service,
    get_logic_service: Service,

    tree: Option<BtNode>,
    state: Option<BtState>,
    blackboard: HashMap<String, Value>,
    action_results: HashMap<String, Value>,
    latest_joint_state: Option<JointSample>,

    tick_index: u64,
    last_root_result: Option<TickResult>,
    running_paths: Vec<Vec<usize>>,
    visited: Vec<Value>,

    tick_interval: Duration,
    last_tick: Instant,
}

impl BtNodeProcess {
    fn new(hub_endpoint: (String, u16), tick_rate_hz: f64) -> BtNodeProcess {
        let node = Node::new("bt_node", hub_endpoint.clone());
        BtNodeProcess {
            joint_states_subscriber: node.create_subscriber("/joint_states", "sensor_msgs/JointState").expect("failed to subscribe to /joint_states"),
            action_result_subscriber: node
                .create_subscriber("/joint_trajectory_action/result", "joint_trajectory_action/Result")
                .expect("failed to subscribe to /joint_trajectory_action/result"),
            trajectory_publisher: node.create_publisher("/joint_trajectory", "trajectory_msgs/JointTrajectory").expect("failed to register /joint_trajectory publisher"),
            status_publisher: node.create_publisher("/bt/status", "bt_msgs/Status").expect("failed to register /bt/status publisher"),
            update_logic_service: node.create_service("/bt/update_logic").expect("failed to register /bt/update_logic"),
            get_logic_service: node.create_service("/bt/get_logic").expect("failed to register /bt/get_logic"),
            tree: None,
            state: None,
            blackboard: HashMap::new(),
            action_results: HashMap::new(),
            latest_joint_state: None,
            tick_index: 0,
            last_root_result: None,
            running_paths: Vec::new(),
            visited: Vec::new(),
            tick_interval: Duration::from_secs_f64(1.0 / tick_rate_hz),
            last_tick: Instant::now() - Duration::from_secs(1),
            hub_endpoint,
            node,
        }
    }

    fn tick_root(&mut self) {
        let (Some(tree), Some(state)) = (self.tree.as_ref(), self.state.as_mut()) else { return };
        let mut path = Vec::new();
        let mut running_paths = Vec::new();
        let mut visited = Vec::new();
        let mut context = TickContext {
            hub_endpoint: self.hub_endpoint.clone(),
            node: &self.node,
            sample: self.latest_joint_state.as_ref(),
            trajectory_publisher: &mut self.trajectory_publisher,
            blackboard: &mut self.blackboard,
            action_results: &self.action_results,
            running_paths: &mut running_paths,
            visited: &mut visited,
        };
        let result = tick_node(tree, state, &mut path, &mut context);
        self.tick_index += 1;
        self.last_root_result = Some(result);
        self.running_paths = running_paths;
        self.visited = visited;
    }

    fn handle_update_logic(&mut self, pending: PendingRequest) {
        let response = match pending.request.get("tree") {
            None => json!({ "values": {}, "result": false, "status": "request must include a \"tree\"" }),
            Some(raw_tree) => match parse_and_validate_tree(raw_tree) {
                Ok(tree) => {
                    self.state = Some(default_state_for(&tree));
                    self.tree = Some(tree);
                    self.last_root_result = None;
                    self.running_paths = Vec::new();
                    self.visited = Vec::new();
                    // A fresh tree gets a fresh blackboard: carrying the
                    // previous tree's trajectories over would let a new
                    // tree's follow_trajectory pick up a plan made for a
                    // different goal, which is exactly the kind of
                    // invisible staleness load-time validation exists to
                    // prevent.
                    self.blackboard.clear();
                    self.tick_root();
                    json!({ "values": { "root_result": self.last_root_result.map(TickResult::as_str) }, "result": true, "status": "" })
                }
                Err(message) => json!({ "values": {}, "result": false, "status": message }),
            },
        };
        pending.respond(&response);
    }

    fn handle_get_logic(&self, pending: PendingRequest) {
        let response = match &self.tree {
            Some(tree) => json!({ "values": { "tree": node_to_json(tree) }, "result": true, "status": "" }),
            None => json!({ "values": {}, "result": false, "status": "no behavior tree has been loaded yet" }),
        };
        pending.respond(&response);
    }

    fn publish_status(&mut self) {
        let blackboard: serde_json::Map<String, Value> = self.blackboard.iter().map(|(k, v)| (k.clone(), v.clone())).collect();
        let msg = json!({
            "tick": self.tick_index,
            "root_result": self.last_root_result.map(TickResult::as_str),
            "running_paths": self.running_paths.iter().map(|p| json!(p)).collect::<Vec<Value>>(),
            "visited": self.visited,
            "blackboard": blackboard,
            "blackboard_keys": self.blackboard.keys().cloned().collect::<Vec<String>>(),
        });
        self.status_publisher.publish(&msg);
    }

    fn spin(&mut self) {
        loop {
            for (_topic, msg) in self.joint_states_subscriber.spin_once() {
                if let Some(sample) = parse_joint_state(&msg) {
                    self.latest_joint_state = Some(sample);
                }
            }
            for (_topic, msg) in self.action_result_subscriber.spin_once() {
                if let Some(goal_id) = msg.get("goal_id").and_then(Value::as_str) {
                    self.action_results.insert(goal_id.to_string(), msg.clone());
                }
            }
            self.trajectory_publisher.spin_once();
            self.status_publisher.spin_once();

            if let Some(pending) = self.update_logic_service.spin_once() {
                self.handle_update_logic(pending);
            }
            if let Some(pending) = self.get_logic_service.spin_once() {
                self.handle_get_logic(pending);
            }

            if self.last_tick.elapsed() >= self.tick_interval {
                self.tick_root();
                self.publish_status();
                self.last_tick = Instant::now();
            }

            std::thread::sleep(Duration::from_millis(10));
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mut hub_host = "127.0.0.1".to_string();
    let mut hub_port: u16 = 9090;
    let mut tick_rate_hz = DEFAULT_TICK_RATE_HZ;
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--host" => {
                hub_host = args[i + 1].clone();
                i += 2;
            }
            "--port" => {
                hub_port = args[i + 1].parse().expect("--port must be a valid u16");
                i += 2;
            }
            "--tick-rate" => {
                tick_rate_hz = args[i + 1].parse().expect("--tick-rate must be a positive number of Hz");
                i += 2;
            }
            other => {
                eprintln!("unrecognized argument: {other}");
                i += 1;
            }
        }
    }
    assert!(tick_rate_hz > 0.0, "--tick-rate must be a positive number of Hz");

    println!(
        "[bt_node] offering /bt/update_logic, /bt/get_logic; ticking at {tick_rate_hz}Hz and publishing /bt/status \
         with the active-node path and a blackboard snapshot"
    );
    let mut node = BtNodeProcess::new((hub_host, hub_port), tick_rate_hz);
    node.spin();
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plan_leaf(output_key: &str) -> Value {
        json!({ "type": "plan", "goal": [0.0, 0.0], "output_key": output_key })
    }

    fn follow_leaf(input_key: &str) -> Value {
        json!({ "type": "follow_trajectory", "input_key": input_key })
    }

    // -- the load-time blackboard rule ------------------------------------

    #[test]
    fn a_plan_then_follow_sequence_validates() {
        let tree = json!({ "type": "sequence", "children": [plan_leaf("traj"), follow_leaf("traj")] });
        assert!(parse_and_validate_tree(&tree).is_ok());
    }

    #[test]
    fn an_input_key_no_earlier_node_writes_is_rejected_at_load_naming_the_key() {
        let tree = json!({ "type": "sequence", "children": [plan_leaf("traj"), follow_leaf("trajecotry")] });
        let error = parse_and_validate_tree(&tree).unwrap_err();
        assert!(error.contains("trajecotry"), "the error must name the offending key: {error}");
        assert!(error.contains("output_key"), "and say what is missing: {error}");
    }

    #[test]
    fn reading_a_key_before_it_is_written_is_rejected_even_though_the_key_exists() {
        // The ordering half of the rule: `traj` *is* produced, but not
        // until after the node that reads it. A validator that only
        // collected every output_key in the tree first would wave this
        // through, and it would fail at tick time instead.
        let tree = json!({ "type": "sequence", "children": [follow_leaf("traj"), plan_leaf("traj")] });
        assert!(parse_and_validate_tree(&tree).unwrap_err().contains("traj"));
    }

    #[test]
    fn a_follow_trajectory_with_no_producer_anywhere_is_rejected() {
        assert!(parse_and_validate_tree(&follow_leaf("anything")).unwrap_err().contains("anything"));
    }

    #[test]
    fn keys_produced_deeper_in_the_tree_still_count_for_later_siblings() {
        let tree = json!({
            "type": "sequence",
            "children": [
                { "type": "repeat", "count": 2, "child": plan_leaf("traj") },
                follow_leaf("traj"),
            ],
        });
        assert!(parse_and_validate_tree(&tree).is_ok(), "pre-order means a nested producer counts for a later sibling");
    }

    // -- the rest of load-time validation ---------------------------------

    #[test]
    fn unknown_node_kinds_are_rejected_and_the_message_lists_the_real_ones() {
        let error = parse_and_validate_tree(&json!({ "type": "fly_to_the_moon" })).unwrap_err();
        assert!(error.contains("fly_to_the_moon"));
        assert!(error.contains("follow_trajectory") && error.contains("condition"), "the message should list the available kinds: {error}");
    }

    #[test]
    fn a_plan_leaf_needs_a_goal_and_a_non_empty_output_key() {
        assert!(parse_and_validate_tree(&json!({ "type": "plan", "output_key": "t" })).unwrap_err().contains("goal"));
        assert!(parse_and_validate_tree(&json!({ "type": "plan", "goal": [0.0] })).unwrap_err().contains("output_key"));
        assert!(parse_and_validate_tree(&json!({ "type": "plan", "goal": [0.0], "output_key": "" })).unwrap_err().contains("output_key"));
        assert!(parse_and_validate_tree(&json!({ "type": "plan", "goal": [], "output_key": "t" })).unwrap_err().contains("empty"));
    }

    #[test]
    fn base_planning_requires_an_explicit_start_because_joint_states_carry_no_base_pose() {
        let without = json!({ "type": "plan", "goal": [0.0, 0.0, 0.0, 0.0, 0.0], "output_key": "t", "base_dofs": true });
        assert!(parse_and_validate_tree(&without).unwrap_err().contains("start"));

        let with = json!({
            "type": "plan",
            "goal": [0.0, 0.0, 0.0, 0.0, 0.0],
            "start": [0.1, 0.1, 0.1, 0.1, 0.1],
            "output_key": "t",
            "base_dofs": true,
        });
        assert!(parse_and_validate_tree(&with).is_ok());
    }

    #[test]
    fn a_condition_leaf_needs_positions_and_a_known_check() {
        assert!(parse_and_validate_tree(&json!({ "type": "condition" })).unwrap_err().contains("positions"));
        assert!(parse_and_validate_tree(&json!({ "type": "condition", "check": "the_vibes", "positions": [0.0] }))
            .unwrap_err()
            .contains("state_validity"));
        // `check` defaults rather than being required, since there is
        // only one kind today.
        assert!(parse_and_validate_tree(&json!({ "type": "condition", "positions": [0.0] })).is_ok());
    }

    #[test]
    fn bt_nodes_five_original_kinds_still_validate_exactly_as_before() {
        let action = json!({ "type": "action", "target": { "joint_names": ["j"], "positions": [0.0] }, "epsilon": 0.1, "timeout": 1.0 });
        assert!(parse_and_validate_tree(&action).is_ok());
        assert!(parse_and_validate_tree(&json!({ "type": "sequence" })).unwrap_err().contains("children"));
        assert!(parse_and_validate_tree(&json!({ "type": "sequence", "children": [] })).unwrap_err().contains("at least one child"));
        assert!(parse_and_validate_tree(&json!({ "type": "repeat", "count": 0, "child": action })).unwrap_err().contains("at least 1"));
    }

    // -- round trip --------------------------------------------------------

    #[test]
    fn get_logic_round_trips_every_new_node_kind() {
        let tree = json!({
            "type": "selector",
            "children": [
                { "type": "sequence", "children": [
                    { "type": "condition", "check": "state_validity", "positions": [0.1, 0.2], "invert": false },
                    { "type": "plan", "goal": [1.0, 2.0], "output_key": "traj", "seed": 5, "max_iterations": 77, "base_dofs": false, "timeout": 30.0 },
                    { "type": "follow_trajectory", "input_key": "traj", "epsilon": 0.02, "timeout": 12.0 },
                ] },
                { "type": "action", "target": { "joint_names": ["j"], "positions": [0.0] }, "epsilon": 0.1, "timeout": 1.0, "slot": 4 },
            ],
        });
        let parsed = parse_and_validate_tree(&tree).expect("should validate");
        let round_tripped = node_to_json(&parsed);

        let branch = &round_tripped["children"][0]["children"];
        assert_eq!(branch[0]["type"], json!("condition"));
        assert_eq!(branch[0]["positions"], json!([0.1, 0.2]));
        assert_eq!(branch[1]["type"], json!("plan"));
        assert_eq!(branch[1]["output_key"], json!("traj"));
        assert_eq!(branch[1]["seed"], json!(5));
        assert_eq!(branch[1]["max_iterations"], json!(77));
        assert_eq!(branch[2]["input_key"], json!("traj"));
        assert_eq!(branch[2]["epsilon"], json!(0.02));
        assert_eq!(round_tripped["children"][1]["slot"], json!(4), "action slot metadata still round-trips verbatim");

        // And the round-tripped tree is itself loadable, which is the
        // property that makes /bt/get_logic useful rather than
        // merely informative.
        assert!(parse_and_validate_tree(&round_tripped).is_ok());
    }

    // -- the state tree mirrors the node tree ------------------------------

    #[test]
    fn every_node_kind_gets_a_matching_fresh_state() {
        let tree = parse_and_validate_tree(&json!({
            "type": "sequence",
            "children": [
                { "type": "plan", "goal": [0.0], "output_key": "t" },
                { "type": "follow_trajectory", "input_key": "t" },
                { "type": "condition", "positions": [0.0] },
            ],
        }))
        .unwrap();
        let state = default_state_for(&tree);
        let BtState::Sequence { current_index, child_states } = state else { panic!("expected a sequence state") };
        assert_eq!(current_index, 0);
        assert!(matches!(child_states[0], BtState::Plan { worker: None, started_at: None }));
        assert!(matches!(child_states[1], BtState::FollowTrajectory { goal_id: None, started_at: None }));
        assert!(matches!(child_states[2], BtState::Condition));
    }

    // -- resolving a plan's start -----------------------------------------

    #[test]
    fn a_plans_start_comes_from_the_tree_or_from_joint_states() {
        let sample = JointSample { joint_names: vec!["a".to_string(), "b".to_string()], positions: vec![0.3, 0.4] };

        assert_eq!(resolve_plan_start(&Some(vec![1.0, 2.0]), Some(&sample), 2).unwrap(), vec![1.0, 2.0], "an explicit start wins");
        assert_eq!(resolve_plan_start(&None, Some(&sample), 2).unwrap(), vec![0.3, 0.4], "otherwise the latest joint state");

        assert!(resolve_plan_start(&None, None, 2).unwrap_err().contains("joint_states"), "with neither, say so rather than guessing");
        assert!(resolve_plan_start(&Some(vec![1.0]), Some(&sample), 2).unwrap_err().contains("start has 1"));
        assert!(resolve_plan_start(&None, Some(&sample), 5).unwrap_err().contains("does not match"), "a width mismatch must not be silently padded");
    }
}
