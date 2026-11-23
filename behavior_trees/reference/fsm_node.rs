// ===========================================================================
// SNAPSHOT -- DO NOT EDIT THIS COPY
//
//   Source repository: /ai/development/autorob_agentic/pubsub_testbed_rs
//   Source path:       src/bin/fsm_node.rs
//   Upstream length:   982 lines
//   Commit:            d9e34cd16c47c790856b1ccea0d8575620c66f89
//   Snapshot date:     2026-11-23
//
// Why this copy exists: Slidev's `<<<` snippet importer cannot read
// outside its own workspace root, which behavior_trees/pnpm-workspace.yaml
// pins at behavior_trees/. The deck quotes Rust that lives in a DIFFERENT
// REPOSITORY entirely, so the file has to be carried here to be quotable
// at all.
//
// Quoted here for: the finite-state-machine interpreter, for the comparison section: the same robot, the same repository, a different architecture.
//
// LINE NUMBERS IN THIS FILE ARE NOT UPSTREAM LINE NUMBERS -- this header
// shifts everything down. Where a slide cites a line, it cites the
// UPSTREAM one, and the mapping is below.
//
// #region markers, with their upstream line numbers:
//   (none upstream)
//
// REGIONS ADDED IN THIS SNAPSHOT ONLY -- not present upstream.
// The implementation added #region markers to the six NEW files it
// wrote, but this file pre-existed and the implementation was
// additive-only, so it could not be marked there. Added here so the
// deck can quote by region rather than by a line range that breaks
// silently when the file changes:
//   fsm-transition-model
//
// Edits belong upstream, followed by a re-snapshot. Diff this file
// against the commit above to check it has not drifted.
// ===========================================================================

//! fsm_node: a flat-switch-case choreography interpreter -- a fixed,
//! hand-written Rust program whose own source never changes per
//! choreography. Only the *data* it interprets (which states exist, what
//! each one's target pose is, and which transitions connect them) changes,
//! loaded live via `/fsm/update_logic`. This is deliberately the opposite
//! of a code-generation pipeline that would emit a fresh `.rs` file (or a
//! fresh compiled binary) per choreography: the whole point of this node
//! is that a student -- or a front-end graph editor, see
//! `frontend/robot_fk.html`'s FSM panel -- can redefine an entire
//! multi-state pick-and-place-style choreography at runtime, with no
//! rebuild, by sending this node a new table.
//!
//! ## Why states carry already-resolved positions, not memory-slot indices
//!
//! Each state's `target` is a fully resolved `{joint_names, positions}`
//! pair, never a `pose_memory_node` slot index this node would have to
//! look up itself. That is a deliberate layering choice: this node must be
//! able to run an entire choreography start to finish even if
//! `pose_memory_node` is not reachable at all (crashed, never started,
//! whatever) -- a robot's active FSM should not depend on a *second*
//! node's availability just to know where it's going next. The
//! *front-end* is the one place that already needs both pieces at once
//! (it fetches every occupied memory slot for its own memory panel *and*
//! builds the FSM graph the user is editing), so it is also the natural,
//! and only necessary, place to resolve "slot N" -> `{joint_names,
//! positions}` once, at graph-save time, and embed the resolved values
//! directly into the `/fsm/update_logic` request. Each state's optional
//! `slot` field is kept purely as round-trip bookkeeping -- so a
//! `/fsm/get_logic` -> rebuild-the-graph-editor round trip can still show
//! "this state came from slot 7" -- and is never read by this node's own
//! execution logic at all.
//!
//! ## Communication
//!
//!   - Subscribes `/joint_states` (`sensor_msgs/JointState`), keeping only
//!     the latest sample -- the same "server holds the latest known pose"
//!     pattern `pose_memory_node.rs` uses, for the same reason (a
//!     transition decision needs "what did the robot last report,"
//!     independent of any particular caller's view of that).
//!   - Publishes `/joint_trajectory` (`trajectory_msgs/JointTrajectory`)
//!     exactly once per state **entry** -- never once per tick -- the same
//!     "publish a fresh one-point trajectory only when the desired
//!     configuration actually changes" discipline `arm_sim_node.rs`'s own
//!     module doc comment establishes and `robot_fk.html`'s sliders/memory
//!     panel already follow. Entering a state happens on a successful
//!     `/fsm/update_logic` (the new `initial_state`), a successful
//!     `/fsm/fire_transition`, or an automatic on-arrival transition (see
//!     below) -- all three funnel through the same [`FsmNode::enter_state`].
//!   - Publishes `/fsm/status` at `--status-rate` Hz (default
//!     [`DEFAULT_STATUS_RATE_HZ`], the same throttled-independent-of-the-
//!     main-loop convention `ik_trial_node.rs`'s own `DEFAULT_STATUS_RATE_HZ`
//!     doc comment explains): `{"current_state": <name> | null,
//!     "time_in_state": <seconds>, "last_transition": <name> | null,
//!     "running": bool}`. `current_state`/`time_in_state` are `null`/`0.0`
//!     before any logic has ever been loaded; `last_transition` is `null`
//!     until at least one transition (manual or automatic) has fired
//!     since the live table was last (re)loaded -- entering a fresh
//!     table's own `initial_state` is not itself a "transition." See
//!     `/fsm/set_running` below for `running`.
//!   - Offers `/fsm/update_logic` (request `{"states": [...], "initial_state":
//!     "<name>"}`) -> standard `{values, result, status}`. Each entry of
//!     `states` is:
//!     `{"name": str, "target": {"joint_names": [str, ...], "positions":
//!     [f64, ...]}, "epsilon": f64, "slot": i64 (optional, see above),
//!     "on_arrival": str (optional, must name one of this state's own
//!     `transitions`), "transitions": [{"name": str, "next_state": str},
//!     ...]}`. Validated *entirely* before anything is committed --
//!     state count at most [`MAX_STATES`], no duplicate state names, no
//!     duplicate transition names *within* a single state (transitions are
//!     looked up by name against "whichever state is currently active," so
//!     an ambiguous name within one state's own list would make both
//!     `/fsm/fire_transition` and on-arrival auto-firing ill-defined),
//!     every `next_state` and `on_arrival` reference resolves to a real
//!     name in *this* request's own `states` list, and `initial_state`
//!     does too. On any failure, `result: false` and the *previously
//!     loaded* table (if any) keeps running completely untouched -- the
//!     same "a bad update can never silently wipe out a working one"
//!     guarantee `robot_state_publisher.rs`'s own `"robot_description"`
//!     parsing documents for itself. On success: the live table is
//!     atomically replaced, the
//!     current state becomes `initial_state` (via `enter_state`, which
//!     also resets the "already auto-fired" flag and publishes that
//!     state's target), `last_transition` resets to `null` (a freshly
//!     loaded table has not yet had any transition fire in it), and
//!     `running` is set to `true` unconditionally -- loading fresh logic
//!     to run should, well, run, even if a previous `/fsm/set_running
//!     {"data": false}` had stopped execution; this is also what makes
//!     the front-end's own **Play** button (which calls this service
//!     directly, with whatever choreography is currently in its own
//!     in-progress graph editor -- see `frontend/robot_fk.html`'s own
//!     `playFsmLogic` doc comment, and this file's own "Named-file
//!     persistence" section below for why Play no longer reads a saved
//!     file the way it briefly used to) implicitly resume from a stop
//!     too, with no separate handling needed there.
//!   - Offers `/fsm/get_logic` (empty request `{}`) -> the current
//!     `states`/`initial_state` definition verbatim, including each
//!     state's own `slot` (`null` if it had none) -- exactly what a
//!     front-end needs to rebuild its graph editor from scratch, e.g.
//!     after a page reload. `result: false` (with `values: {}`) if
//!     `/fsm/update_logic` has never been called successfully.
//!   - Offers `/fsm/fire_transition` (request `{"transition": "<name>"}`):
//!     fires that named transition from whatever the *current* state is,
//!     looked up among that state's own `transitions`, **regardless of
//!     arrival status** -- this is what gives the graph real branching/
//!     testing value beyond a linear auto-advance chain (a student can
//!     force any edge at any time, e.g. to test a "fault" transition
//!     without actually driving the arm out of tolerance first).
//!     `result: false` (table and current state left untouched) if no
//!     transition by that name exists on the current state, if no
//!     logic/current state exists at all yet, **or if `running` is
//!     currently `false`** (a clear `"cannot fire a transition while
//!     stopped"`-style status message, not a silent no-op) -- see
//!     `/fsm/set_running` below. This is a deliberate "stop really means
//!     stop" choice: `running` gates *every* transition this node would
//!     otherwise fire, manual or automatic alike, not just automatic
//!     on-arrival ones.
//!   - Offers `/fsm/set_running` (request `{"data": <bool, optional>}`)
//!     -> `{"values": {"running": bool}, "result": true, "status": str}`
//!     -- mirrors `joint_state_publisher`'s own `/joint_state_publisher/
//!     servo_toggle` "field is optional; omit it to just query the
//!     current value" convention. While `running` is `false`: `tick_state`
//!     never checks arrival or auto-fires `on_arrival` (see below), and
//!     `/fsm/fire_transition` is rejected outright (see above) --
//!     `/joint_trajectory` simply stops being published at all, freezing
//!     the robot wherever it currently is (this node never itself commands
//!     a stop *position*; it just stops commanding a *new* one). `running`
//!     defaults to `true` on startup (unchanged behavior until something
//!     explicitly stops it) and is force-set back to `true` by every
//!     successful `/fsm/update_logic` (see above).
//!
//! ## Named-file persistence: Save/Load vs. Play/Stop
//!
//! Editing/persistence and execution are two entirely separate concerns,
//! each with its own pair of front-end buttons, and (as of this section)
//! this node keeps them genuinely decoupled rather than one implying the
//! other:
//!   - **Play/Stop drive *execution*** -- `/fsm/update_logic` (deploys a
//!     table and runs it, see above) and `/fsm/set_running` (pauses/
//!     resumes whatever is already deployed). Neither one ever touches
//!     disk.
//!   - **Save/Load drive *persistence*** -- `/fsm/save_to_file`/
//!     `/fsm/load_from_file` (below) write/read a named choreography as a
//!     flat JSON file under [`FSM_DIR`] (`fsm/`, project root, sibling to
//!     `robots/` -- see that directory's own `README.md` for the analogous
//!     "why a flat file, one per thing" convention this mirrors). Neither
//!     one ever touches `self.states`/`self.current_index`/anything else
//!     this node is actively running -- a save while a choreography is
//!     mid-execution does not need to (and does not) disturb it, and a
//!     load does not deploy anything either.
//!
//!   This split is a deliberate, real redesign, not just added surface: an
//!   earlier revision of this node/front-end pair had **Save** double as
//!   both (it called `/fsm/update_logic` directly with the front-end's own
//!   editor graph -- persistence and execution, conflated into one
//!   button), and **Load** read back whatever was *currently running* via
//!   `/fsm/get_logic` (a snapshot of live execution state, not a named
//!   saved file -- there was no durable, nameable "saved choreography"
//!   concept at all). Now: **Save** writes the editor's own in-progress
//!   graph to a named file and leaves whatever `fsm_node` is running
//!   completely alone; **Load** reads a named file back into the editor
//!   and, likewise, never touches what's running; **Play** deploys
//!   whatever is currently in the editor (exactly what the *old* Save used
//!   to do) via `/fsm/update_logic`, with no file involved at all. `/fsm/
//!   get_logic` itself is unchanged and still available (e.g. for a test
//!   or tool that genuinely wants "what is live right now," not a saved
//!   file) -- only the front-end's own Load button stopped using it.
//!
//!   - Offers `/fsm/save_to_file` (request `{"identifier": "<name>",
//!     "states": [...], "initial_state": "<name>"}`) -> standard
//!     `{values: {"path": str}, result, status}`. `identifier` is
//!     validated as a single safe path segment by
//!     [`validate_fsm_identifier`] -- the same empty/`/`/`\`/`.`/`..`
//!     rejection rule `joint_state_publisher.rs`'s own `validate_robot_name`
//!     uses for the same reason (a caller-controlled name that gets joined
//!     onto a directory to build a filesystem path, reachable from an
//!     untrusted browser via `rosbridge_server`) -- deliberately *not*
//!     `robot_state_publisher.rs`'s own more permissive `resolve_output_path`
//!     rule (which allows nested relative subdirectories), since an
//!     "identifier" here names one flat file, not an arbitrary output
//!     path. `states`/`initial_state` are written to `fsm/<identifier>.json`
//!     (pretty-printed) **verbatim, with no structural validation at
//!     all** -- deliberately: saving a work-in-progress graph the user
//!     hasn't finished wiring up yet (a dangling `next_state` reference, a
//!     duplicate name, whatever) must not be blocked the way actually
//!     *running* it (`/fsm/update_logic`, which does fully validate) is.
//!     Validation happens exactly once, at the one point it actually
//!     matters: when a table is about to be deployed.
//!   - Offers `/fsm/load_from_file` (request `{"identifier": "<name>"}`)
//!     -> `{"values": {"states": [...], "initial_state": "<name>"},
//!     "result": bool, "status": str}`. Reads `fsm/<identifier>.json` back
//!     verbatim (same non-validating spirit as save, and for the same
//!     reason -- a saved work-in-progress graph should load back into the
//!     editor exactly as saved, incomplete wiring included).
//!     `result: false` if `identifier` fails the same validation
//!     `save_to_file` applies, or if no such file exists.
//!   - Offers `/fsm/list_saved` (empty request `{}`) ->
//!     `{"values": {"identifiers": [...]}, "result": true, "status": str}`
//!     -- every `<name>` with an `fsm/<name>.json` file, sorted
//!     alphabetically. Mirrors `joint_state_publisher.rs`'s own
//!     `/joint_state_publisher/robot_description_list` pattern exactly (a
//!     directory scan, not a registry this node maintains itself) so the
//!     front-end can show what's already saved instead of requiring a
//!     user to type an exact remembered name blindly.
//!
//! ## Arrival detection: convention and why
//!
//! On every `/joint_states` message, if its `name` array names *exactly*
//! the same **set** of joints as the active state's `target.joint_names`
//! (order-independent, same compatibility-check spirit
//! `robot_fk.html`'s memory-recall code already applies before publishing
//! a recalled slot) this node computes, for each of that state's target
//! joints, `|target_position - actual_position|`, and calls the state
//! "arrived" for this tick iff the **maximum** of those per-joint errors is
//! `< epsilon`. A mismatched joint set (wrong robot loaded, stale message,
//! whatever) is simply ignored for arrival purposes, exactly as
//! `pose_memory_node.rs`'s own compatibility note recommends -- this node
//! never guesses at a partial/misaligned comparison.
//!
//! This deliberately differs from this project's two existing convergence
//! checks, and why:
//!   - `ik_action_node.rs`'s goal-reached check is Euclidean
//!     **end-effector** distance (`sqrt(dx^2 + dy^2)`) with a minimum
//!     **dwell time** before declaring victory (see that file's
//!     `advance_dwell`) -- appropriate there because it is judging
//!     *Cartesian* task-space convergence of a controller that can
//!     overshoot and oscillate through a target on its way to actually
//!     settling. This node instead judges **joint-space** arrival
//!     (`max(|error_i|)` over the same `joint_names`/`positions` shape
//!     every other node here already uses) with **no dwell time** at
//!     all: it is asking "is `/joint_states` currently reporting a pose
//!     within tolerance," a one-shot *check*, not a control law deciding
//!     when to declare success on a system it does not itself command
//!     moment-to-moment (that job belongs to whatever is driving toward
//!     the setpoint, e.g. `joint_state_publisher`). A momentary in-and-out
//!     wobble simply means "not currently arrived, check again next
//!     tick" here, which is a correct, simple, and sufficient behavior at
//!     this node's altitude of responsibility (see below for the residual
//!     risk this choice does accept).
//!   - `joint_state_publisher.rs`'s own servo law wraps its position error into
//!     `(-pi, pi]` before use (see that file's `shortest_angle_difference`)
//!     because a naive subtraction would otherwise send a revolute joint
//!     the "long way around" a full turn. That correction matters for a
//!     *control law* computing which direction to move; it does not matter
//!     for this node's plain arrival *check* -- the only consequence of
//!     skipping it here is that a target and an actual angle that are
//!     truly identical but expressed on opposite sides of a `+/-pi`
//!     wraparound (e.g. `3.14` vs `-3.14`, off by `2*pi` numerically but
//!     physically the same angle) would read as a large, non-arrived
//!     error instead of a tiny one. Documented here as a deliberate,
//!     narrow simplification rather than a silently missing feature: in
//!     practice every target this node ever receives came from a
//!     recalled `pose_memory_node` slot or a live `/joint_states` sample,
//!     both already expressed in whatever single convention the rest of
//!     this project's joint angles use, so the two sides of a comparison
//!     here are never expected to straddle that wraparound boundary.
//!
//! ## The auto-fire-once-per-visit rule
//!
//! A state's own `on_arrival` (if set) names one of its `transitions` to
//! fire automatically the *first* tick arrival is detected after entering
//! that state -- not every tick for as long as the robot happens to stay
//! within `epsilon`. This node tracks a single `already_fired_this_visit`
//! flag (there is only ever one "current" state, so one flag suffices),
//! cleared by [`FsmNode::enter_state`] on every entry and set the moment an
//! on-arrival auto-fire is *attempted*. In the ordinary case, firing a
//! transition to a genuinely different state already makes further
//! refiring impossible on its own: `tick_state` only ever runs against
//! `self.current_index`, and firing changes `current_index` away from the
//! old state immediately, in the same tick. Where the flag actually earns
//! its keep is: (1) it is what makes "at most once" *true by construction*
//! rather than "true only because nothing has gone wrong yet" -- a firing
//! attempt is never repeated even if `fire_transition_by_name` were to
//! fail (it shouldn't, given `/fsm/update_logic`'s own validation, but
//! this node does not depend on that never changing to stay correct), and
//! (2) a transition whose own `next_state` names its *own* state (a
//! literal self-loop) is, by this node's definition, a fresh **visit** on
//! every re-entry -- `enter_state` resets the flag then too, so a
//! self-looping `on_arrival` is expected to refire on every tick the robot
//! remains within `epsilon`, by design, not by omission.
//!
//! ## The flat switch-case dispatch
//!
//! Every loaded state is assigned a stable numeric index `0..N` (`N <=`
//! [`MAX_STATES`]) matching its position in `/fsm/update_logic`'s own
//! `states` array. Each main-loop tick, whichever index is currently
//! active is dispatched through [`state_dispatch`] -- a literal Rust
//! `match` with one arm per index `0..MAX_STATES`, generated by the
//! `state_dispatch!` macro (see that macro's own doc comment immediately
//! above its definition for the full rationale and an expansion example).
//! This is the concrete, load-bearing meaning of "flat switch-case
//! choreography interpreter" in this file's own name: dispatching on
//! which state is active is genuinely a `match` baked into this binary's
//! compiled code, not a `states[index]` array lookup with no `match` at
//! all, and not a single collapsed `0..MAX_STATES => ...` range arm either
//! (which would be indistinguishable, at the machine-code level, from
//! plain indexing) -- even though, in this implementation, every arm's
//! body happens to call the exact same generic per-tick function, since
//! the *actual* per-state behavior (which target to check arrival
//! against, which transition to auto-fire) is entirely a property of this
//! node's *data* (`self.states`), never of which arm of the `match`
//! happened to run. A hypothetical future code-generation pipeline
//! targeting this exact same "N states -> N switch-case arms" shape could
//! swap in per-arm bodies without touching this dispatch structure at
//! all -- which is exactly the intended seam.

use pubsub_testbed::p2p::node::{Node, PendingRequest, Publisher, Service, Subscriber};
use pubsub_testbed::ros_messages as rm;
use serde_json::{json, Value};
use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// The hard cap on how many states a single `/fsm/update_logic` call may
/// define, matching the "small, documented, load-time-checked cap" spirit
/// of this project's other `MAX_*` constants (`ik_trial_node.rs`'s
/// `MAX_SAMPLE_ATTEMPTS`, `arm_sim_node.rs`'s `MAX_INTEGRATION_STEPS`).
/// 64 is generous for any hand-built or hand-edited teaching choreography
/// (a graph a person is actually drawing/reasoning about node-by-node in
/// `robot_fk.html`'s FSM panel) while keeping [`state_dispatch`]'s
/// generated `match` a small, readable, genuinely-enumerable list of arms
/// rather than an unbounded one.
const MAX_STATES: usize = 64;

/// Default rate, in Hz, at which `/fsm/status` is published -- see
/// `--status-rate` and `ik_trial_node.rs`'s own `DEFAULT_STATUS_RATE_HZ`
/// doc comment for why this is deliberately decoupled from the main
/// loop's own tick rate.
const DEFAULT_STATUS_RATE_HZ: f64 = 10.0;

/// Where saved choreographies live -- see the module doc comment's
/// "Named-file persistence" section. Project root, sibling to `robots/`
/// (not nested under it -- a saved FSM is not a robot description).
const FSM_DIR: &str = "fsm";

/// Rejects anything that isn't a single, ordinary path segment --
/// `identifier` (from `/fsm/save_to_file`/`/fsm/load_from_file`) gets
/// joined onto [`FSM_DIR`] to build a filesystem path, and both services
/// are reachable from an untrusted browser via `rosbridge_server` exactly
/// like every other service in this project, so a caller-controlled
/// `identifier` containing a path separator or a `..` component is a real
/// path-escape risk, not a hypothetical one. Deliberately identical to
/// `joint_state_publisher.rs`'s own `validate_robot_name` (same threat
/// model, same fix) rather than `robot_state_publisher.rs`'s own more
/// permissive `resolve_output_path` rule -- an "identifier" here names one
/// flat file, not an arbitrary nested output path.
fn validate_fsm_identifier(identifier: &str) -> Result<(), String> {
    if identifier.is_empty() || identifier.contains('/') || identifier.contains('\\') || identifier == "." || identifier == ".." {
        return Err(format!("{identifier:?} is not a valid identifier (must be a single path segment, no \"/\", \"\\\\\", \".\", or \"..\")"));
    }
    Ok(())
}

/// The path `fsm/<identifier>.json` would resolve to, if `identifier` is
/// valid -- shared by `save_to_file`/`load_from_file`/`list_saved` so the
/// `<identifier>.json` naming convention lives in exactly one place.
fn fsm_file_path(identifier: &str) -> PathBuf {
    Path::new(FSM_DIR).join(format!("{identifier}.json"))
}

/// One transition out of some state: a name (looked up by
/// `/fsm/fire_transition` and by a state's own optional `on_arrival`) and
/// which state it leads to.
#[derive(Clone, Debug)]
// -- #region fsm-transition-model -----------------------------------------
struct Transition {
    name: String,
    next_state: String,
}
// -- #endregion fsm-transition-model --------------------------------------

/// One loaded state: everything from a single entry of
/// `/fsm/update_logic`'s `states` array, already validated (every
/// `next_state`/`on_arrival` reference is known to resolve, `target`'s two
/// arrays are known to be the same length) by the time it is ever
/// constructed. See the module doc comment for why `target` is already-
/// resolved positions rather than a `pose_memory_node` slot index, and
/// what `slot` is (and is not) used for.
#[derive(Clone, Debug)]
struct State {
    name: String,
    target_joint_names: Vec<String>,
    target_positions: Vec<f64>,
    epsilon: f64,
    /// Pure round-trip bookkeeping -- see module doc comment. Never read
    /// by anything in this file except `/fsm/get_logic`'s own response
    /// building.
    slot: Option<i64>,
    /// Names one of this state's own `transitions` to fire automatically
    /// the first tick arrival is detected after entering this state (see
    /// the module doc comment's "auto-fire-once-per-visit" section).
    on_arrival: Option<String>,
    transitions: Vec<Transition>,
}

/// The latest known `/joint_states` sample -- same "hold only the most
/// recent one" pattern as `pose_memory_node.rs`'s own `latest_joint_state`.
#[derive(Clone, Debug)]
struct JointSample {
    joint_names: Vec<String>,
    positions: Vec<f64>,
}

/// Generates a literal Rust `match` with one arm per listed index, each
/// arm identical except for which numeral it matches against `$index` --
/// e.g. `state_dispatch!(self, index, sample, [0, 1, 2])` expands to:
///
/// ```ignore
/// match index {
///     0 => self.tick_state(0, sample),
///     1 => self.tick_state(1, sample),
///     2 => self.tick_state(2, sample),
///     _ => {}
/// }
/// ```
///
/// The one invocation in [`FsmNode::dispatch_tick`] below lists every
/// index `0..MAX_STATES` explicitly (`[0, 1, 2, ..., 63]`) rather than a
/// range pattern, so the *expanded* code really is one arm per index --
/// see the module doc comment's "The flat switch-case dispatch" section
/// for why that distinction is the entire point of this file. Listing 64
/// literals once, here, is the tradeoff this macro exists to make: a
/// student reading the invocation sees exactly what expands (unlike a
/// recursive counting macro, whose generated arms are invisible at the
/// call site), while never hand-typing 64 near-identical lines of actual
/// match-arm code.
macro_rules! state_dispatch {
    ($self:expr, $index:expr, $sample:expr, [$($i:literal),+ $(,)?]) => {
        match $index {
            $($i => $self.tick_state($i, $sample),)+
            // Only reachable if `$index` somehow exceeds MAX_STATES - 1,
            // which /fsm/update_logic's own validation should never
            // allow through in the first place -- a silent no-op is a
            // safer failure mode here than a panic or an out-of-bounds
            // `self.states` index.
            _ => {}
        }
    };
}

struct FsmNode {
    joint_states_subscriber: Subscriber,
    trajectory_publisher: Publisher,
    status_publisher: Publisher,
    update_logic_service: Service,
    get_logic_service: Service,
    fire_transition_service: Service,
    set_running_service: Service,
    save_to_file_service: Service,
    load_from_file_service: Service,
    list_saved_service: Service,
    _node: Node,

    states: Vec<State>,
    name_to_index: HashMap<String, usize>,
    /// The `initial_state` name from the most recently *loaded* table --
    /// distinct from `current_index`, which moves as transitions fire.
    /// `/fsm/get_logic` echoes this, not wherever execution currently is.
    initial_state_name: Option<String>,

    /// See `/fsm/set_running`'s own module doc comment. `true` by
    /// default (unchanged behavior until something explicitly stops it);
    /// gates both `tick_state`'s own auto-fire logic and
    /// `/fsm/fire_transition` outright.
    running: bool,

    current_index: Option<usize>,
    state_entered_at: Instant,
    already_fired_this_visit: bool,
    last_transition: Option<String>,

    latest_joint_state: Option<JointSample>,

    status_publish_interval: Duration,
    last_status_publish: Instant,
}

impl FsmNode {
    fn new(hub_endpoint: (String, u16), status_rate_hz: f64) -> FsmNode {
        let node = Node::new("fsm_node", hub_endpoint);
        FsmNode {
            joint_states_subscriber: node
                .create_subscriber("/joint_states", "sensor_msgs/JointState")
                .expect("failed to subscribe to /joint_states"),
            trajectory_publisher: node
                .create_publisher("/joint_trajectory", "trajectory_msgs/JointTrajectory")
                .expect("failed to register /joint_trajectory publisher"),
            status_publisher: node.create_publisher("/fsm/status", "fsm_msgs/Status").expect("failed to register /fsm/status publisher"),
            update_logic_service: node.create_service("/fsm/update_logic").expect("failed to register /fsm/update_logic service"),
            get_logic_service: node.create_service("/fsm/get_logic").expect("failed to register /fsm/get_logic service"),
            fire_transition_service: node.create_service("/fsm/fire_transition").expect("failed to register /fsm/fire_transition service"),
            set_running_service: node.create_service("/fsm/set_running").expect("failed to register /fsm/set_running service"),
            save_to_file_service: node.create_service("/fsm/save_to_file").expect("failed to register /fsm/save_to_file service"),
            load_from_file_service: node.create_service("/fsm/load_from_file").expect("failed to register /fsm/load_from_file service"),
            list_saved_service: node.create_service("/fsm/list_saved").expect("failed to register /fsm/list_saved service"),
            states: Vec::new(),
            name_to_index: HashMap::new(),
            initial_state_name: None,
            running: true,
            current_index: None,
            state_entered_at: Instant::now(),
            already_fired_this_visit: false,
            last_transition: None,
            latest_joint_state: None,
            status_publish_interval: Duration::from_secs_f64(1.0 / status_rate_hz),
            last_status_publish: Instant::now() - Duration::from_secs(1), // publish immediately on the first tick
            _node: node,
        }
    }

    // -- /joint_states -----------------------------------------------

    fn parse_joint_state(msg: &Value) -> Option<JointSample> {
        let names = msg.get("name")?.as_array()?;
        let positions = msg.get("position")?.as_array()?;
        let joint_names: Vec<String> = names.iter().filter_map(|v| v.as_str().map(str::to_string)).collect();
        let positions: Vec<f64> = positions.iter().filter_map(Value::as_f64).collect();
        if joint_names.len() != names.len() || positions.len() != names.len() {
            return None; // malformed upstream message
        }
        Some(JointSample { joint_names, positions })
    }

    // -- Validation of a whole `/fsm/update_logic` request ------------

    /// Parses and fully validates a `/fsm/update_logic` request into a
    /// fresh `(states, name_to_index, initial_index)` triple *without*
    /// mutating `self` at all -- so that any validation failure anywhere
    /// leaves the live table completely untouched, per the module doc
    /// comment.
    fn parse_and_validate_logic(request: &Value) -> Result<(Vec<State>, HashMap<String, usize>, usize), String> {
        let raw_states = request.get("states").and_then(Value::as_array).ok_or("request must include a \"states\" array")?;
        if raw_states.len() > MAX_STATES {
            return Err(format!("at most {MAX_STATES} states are allowed (got {})", raw_states.len()));
        }
        let initial_state = request.get("initial_state").and_then(Value::as_str).ok_or("request must include a string \"initial_state\"")?;

        let mut states = Vec::with_capacity(raw_states.len());
        let mut name_to_index = HashMap::new();
        for (index, raw) in raw_states.iter().enumerate() {
            let name = raw.get("name").and_then(Value::as_str).ok_or_else(|| format!("state at index {index} is missing a string \"name\""))?.to_string();
            if name_to_index.insert(name.clone(), index).is_some() {
                return Err(format!("duplicate state name {name:?}"));
            }

            let target = raw.get("target").ok_or_else(|| format!("state {name:?} is missing \"target\""))?;
            let target_joint_names: Vec<String> = target
                .get("joint_names")
                .and_then(Value::as_array)
                .ok_or_else(|| format!("state {name:?}'s target is missing a \"joint_names\" array"))?
                .iter()
                .filter_map(|v| v.as_str().map(str::to_string))
                .collect();
            let target_positions: Vec<f64> = target
                .get("positions")
                .and_then(Value::as_array)
                .ok_or_else(|| format!("state {name:?}'s target is missing a \"positions\" array"))?
                .iter()
                .filter_map(Value::as_f64)
                .collect();
            if target_joint_names.len() != target_positions.len() {
                return Err(format!("state {name:?}'s target \"joint_names\" and \"positions\" must be the same length"));
            }

            let epsilon = raw.get("epsilon").and_then(Value::as_f64).ok_or_else(|| format!("state {name:?} is missing a numeric \"epsilon\""))?;
            if epsilon < 0.0 {
                return Err(format!("state {name:?}'s \"epsilon\" must be non-negative"));
            }

            let slot = match raw.get("slot") {
                None | Some(Value::Null) => None,
                Some(v) => Some(v.as_i64().ok_or_else(|| format!("state {name:?}'s \"slot\", if present, must be an integer"))?),
            };

            let on_arrival = match raw.get("on_arrival") {
                None | Some(Value::Null) => None,
                Some(v) => Some(v.as_str().ok_or_else(|| format!("state {name:?}'s \"on_arrival\", if present, must be a string"))?.to_string()),
            };

            let raw_transitions = raw.get("transitions").and_then(Value::as_array).ok_or_else(|| format!("state {name:?} is missing a \"transitions\" array"))?;
            let mut transitions = Vec::with_capacity(raw_transitions.len());
            let mut transition_names = HashSet::new();
            for t in raw_transitions {
                let t_name = t.get("name").and_then(Value::as_str).ok_or_else(|| format!("a transition on state {name:?} is missing a string \"name\""))?.to_string();
                let next_state = t
                    .get("next_state")
                    .and_then(Value::as_str)
                    .ok_or_else(|| format!("transition {t_name:?} on state {name:?} is missing a string \"next_state\""))?
                    .to_string();
                if !transition_names.insert(t_name.clone()) {
                    return Err(format!("state {name:?} has more than one transition named {t_name:?}"));
                }
                transitions.push(Transition { name: t_name, next_state });
            }

            states.push(State { name, target_joint_names, target_positions, epsilon, slot, on_arrival, transitions });
        }

        // Second pass: every next_state/on_arrival reference must resolve
        // against *this* request's own state names (forward references,
        // e.g. state 0 pointing at state 3, are fine -- all names are
        // already known from the first pass above).
        for state in &states {
            for transition in &state.transitions {
                if !name_to_index.contains_key(&transition.next_state) {
                    return Err(format!("state {:?}'s transition {:?} names unknown next_state {:?}", state.name, transition.name, transition.next_state));
                }
            }
            if let Some(on_arrival) = &state.on_arrival {
                if !state.transitions.iter().any(|t| &t.name == on_arrival) {
                    return Err(format!("state {:?}'s on_arrival {:?} does not name one of its own transitions", state.name, on_arrival));
                }
            }
        }

        let initial_index = *name_to_index.get(initial_state).ok_or_else(|| format!("initial_state {initial_state:?} is not one of the given states"))?;

        Ok((states, name_to_index, initial_index))
    }

    fn handle_update_logic_request(&mut self, pending: PendingRequest) {
        let response = match Self::parse_and_validate_logic(&pending.request) {
            Ok((states, name_to_index, initial_index)) => {
                let initial_name = states[initial_index].name.clone();
                self.states = states;
                self.name_to_index = name_to_index;
                self.initial_state_name = Some(initial_name);
                // A freshly loaded table has not had any transition fire
                // in it yet -- see module doc comment.
                self.last_transition = None;
                // Loading fresh logic to run should run -- see module doc
                // comment on why this is unconditional, and what it makes
                // Play implicitly do.
                self.running = true;
                self.enter_state(initial_index, None);
                json!({ "values": { "current_state": self.states[initial_index].name }, "result": true, "status": "" })
            }
            Err(message) => json!({ "values": {}, "result": false, "status": message }),
        };
        pending.respond(&response);
    }

    fn handle_get_logic_request(&self, pending: PendingRequest) {
        if self.states.is_empty() {
            pending.respond(&json!({ "values": {}, "result": false, "status": "no FSM logic has been loaded yet" }));
            return;
        }
        let states_json: Vec<Value> = self
            .states
            .iter()
            .map(|s| {
                json!({
                    "name": s.name,
                    "target": { "joint_names": s.target_joint_names, "positions": s.target_positions },
                    "epsilon": s.epsilon,
                    "slot": s.slot,
                    "on_arrival": s.on_arrival,
                    "transitions": s.transitions.iter().map(|t| json!({ "name": t.name, "next_state": t.next_state })).collect::<Vec<_>>(),
                })
            })
            .collect();
        let response = json!({
            "values": { "states": states_json, "initial_state": self.initial_state_name },
            "result": true,
            "status": "",
        });
        pending.respond(&response);
    }

    // -- Named-file persistence: /fsm/save_to_file, /fsm/load_from_file,
    // /fsm/list_saved -- see module doc comment's own section for the
    // full design (deliberately decoupled from execution/`self.states`).

    /// Writes `{"states": ..., "initial_state": ...}` to
    /// `fsm/<identifier>.json`, verbatim and unvalidated -- see module doc
    /// comment for why. Does not read or touch `self` at all (this is a
    /// pure, `&self`-only file write; the "no structural validation"
    /// choice is exactly what makes that possible -- there is nothing
    /// here that could need this node's own live state).
    fn handle_save_to_file(request: &Value) -> Result<PathBuf, String> {
        let identifier = request.get("identifier").and_then(Value::as_str).ok_or("request must include a string \"identifier\"")?;
        validate_fsm_identifier(identifier)?;
        let states = request.get("states").ok_or("request must include \"states\"")?.clone();
        let initial_state = request.get("initial_state").ok_or("request must include \"initial_state\"")?.clone();
        std::fs::create_dir_all(FSM_DIR).map_err(|e| format!("could not create {FSM_DIR}: {e}"))?;
        let path = fsm_file_path(identifier);
        let contents = json!({ "states": states, "initial_state": initial_state });
        let pretty = serde_json::to_string_pretty(&contents).expect("a Value built from already-parsed JSON should always re-serialize");
        std::fs::write(&path, pretty).map_err(|e| format!("could not write {}: {e}", path.display()))?;
        Ok(path)
    }

    fn handle_save_to_file_request(&self, pending: PendingRequest) {
        let response = match Self::handle_save_to_file(&pending.request) {
            Ok(path) => json!({ "values": { "path": path.display().to_string() }, "result": true, "status": "" }),
            Err(message) => json!({ "values": {}, "result": false, "status": message }),
        };
        pending.respond(&response);
    }

    /// Reads `fsm/<identifier>.json` back, verbatim -- see module doc
    /// comment. Also `&self`-only, for the same reason as `save_to_file`
    /// above.
    fn handle_load_from_file(request: &Value) -> Result<(Value, Value), String> {
        let identifier = request.get("identifier").and_then(Value::as_str).ok_or("request must include a string \"identifier\"")?;
        validate_fsm_identifier(identifier)?;
        let path = fsm_file_path(identifier);
        let text = std::fs::read_to_string(&path).map_err(|e| format!("could not read {}: {e}", path.display()))?;
        let parsed: Value = serde_json::from_str(&text).map_err(|e| format!("{} is not valid JSON: {e}", path.display()))?;
        let states = parsed.get("states").cloned().ok_or_else(|| format!("{} is missing \"states\"", path.display()))?;
        let initial_state = parsed.get("initial_state").cloned().ok_or_else(|| format!("{} is missing \"initial_state\"", path.display()))?;
        Ok((states, initial_state))
    }

    fn handle_load_from_file_request(&self, pending: PendingRequest) {
        let response = match Self::handle_load_from_file(&pending.request) {
            Ok((states, initial_state)) => json!({ "values": { "states": states, "initial_state": initial_state }, "result": true, "status": "" }),
            Err(message) => json!({ "values": {}, "result": false, "status": message }),
        };
        pending.respond(&response);
    }

    /// Every identifier `fsm/` currently offers -- every `<name>.json`
    /// direct child of [`FSM_DIR`], name sorted alphabetically -- mirrors
    /// `joint_state_publisher.rs`'s own `list_robots` pattern exactly (see
    /// module doc comment). An empty or entirely missing `FSM_DIR` simply
    /// yields an empty list, not an error, same reasoning as that
    /// function's own doc comment.
    fn list_saved() -> Vec<String> {
        let mut identifiers = Vec::new();
        if let Ok(entries) = std::fs::read_dir(FSM_DIR) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.extension().and_then(|e| e.to_str()) != Some("json") {
                    continue;
                }
                if let Some(stem) = path.file_stem().and_then(|s| s.to_str()) {
                    identifiers.push(stem.to_string());
                }
            }
        }
        identifiers.sort();
        identifiers
    }

    fn handle_list_saved_request(&self, pending: PendingRequest) {
        pending.respond(&json!({ "values": { "identifiers": Self::list_saved() }, "result": true, "status": "" }));
    }

    /// Fires the named transition from whatever state is currently
    /// active, regardless of arrival -- shared by `/fsm/fire_transition`
    /// (manual) and on-arrival auto-firing (see `tick_state`). Returns the
    /// new current state's name on success.
    fn fire_transition_by_name(&mut self, name: &str) -> Result<String, String> {
        let Some(current) = self.current_index else {
            return Err("no FSM logic is currently loaded/active".to_string());
        };
        let next_state_name = self.states[current]
            .transitions
            .iter()
            .find(|t| t.name == name)
            .map(|t| t.next_state.clone())
            .ok_or_else(|| format!("current state {:?} has no transition named {name:?}", self.states[current].name))?;
        // Resolved and validated at load time, so this index lookup
        // cannot fail -- but handled without panicking regardless.
        let next_index = *self.name_to_index.get(&next_state_name).ok_or_else(|| format!("internal error: unknown next_state {next_state_name:?}"))?;
        self.enter_state(next_index, Some(name.to_string()));
        Ok(self.states[next_index].name.clone())
    }

    fn handle_fire_transition_request(&mut self, pending: PendingRequest) {
        let response = if !self.running {
            // "Stop really means stop" -- see module doc comment: this
            // gates *every* transition, manual fires included, not just
            // automatic on-arrival ones.
            json!({ "values": {}, "result": false, "status": "cannot fire a transition while stopped -- call /fsm/set_running {\"data\": true} (or /fsm/update_logic) first" })
        } else {
            match pending.request.get("transition").and_then(Value::as_str) {
                None => json!({ "values": {}, "result": false, "status": "request must include a string \"transition\"" }),
                Some(name) => match self.fire_transition_by_name(name) {
                    Ok(new_state) => json!({ "values": { "current_state": new_state }, "result": true, "status": "" }),
                    Err(message) => json!({ "values": {}, "result": false, "status": message }),
                },
            }
        };
        pending.respond(&response);
    }

    /// `{"data": <bool, optional>}` -> `{"values": {"running": bool},
    /// "result": true, "status": str}` -- see module doc comment's
    /// `/fsm/set_running` section for the full contract. Mirrors
    /// `joint_state_publisher`'s own `/joint_state_publisher/servo_toggle`
    /// "field is optional; omit it to just query the current value"
    /// convention.
    fn handle_set_running_request(&mut self, pending: PendingRequest) {
        if let Some(data) = pending.request.get("data").and_then(Value::as_bool) {
            self.running = data;
        }
        let status = if self.running { "running" } else { "stopped" };
        pending.respond(&json!({ "values": { "running": self.running }, "result": true, "status": status }));
    }

    /// Makes `index` the active state: resets the per-visit clock and
    /// auto-fire flag, records `transition_name` as `last_transition`
    /// (`None` for the very first entry after a fresh
    /// `/fsm/update_logic`), and publishes a fresh one-point
    /// `/joint_trajectory` for its target -- the single funnel point every
    /// state change (initial load, manual fire, auto on-arrival) goes
    /// through, so "publish exactly once per entry" only has to be
    /// implemented once.
    fn enter_state(&mut self, index: usize, transition_name: Option<String>) {
        self.current_index = Some(index);
        self.state_entered_at = Instant::now();
        self.already_fired_this_visit = false;
        self.last_transition = transition_name;

        let state = &self.states[index];
        let n = state.target_positions.len();
        let point = rm::joint_trajectory_point(&state.target_positions, &vec![0.0; n], &vec![0.0; n], 0, 0);
        let msg = rm::joint_trajectory(&state.target_joint_names, vec![point], "");
        self.trajectory_publisher.publish(&msg);
    }

    /// Computes `max(|target_position_i - actual_position_i|)` for state
    /// `index`'s target against `sample`, or `None` if `sample` doesn't
    /// name *exactly* the same set of joints as that target -- see the
    /// module doc comment's "Arrival detection" section.
    fn max_joint_error(&self, index: usize, sample: &JointSample) -> Option<f64> {
        let target = &self.states[index];
        if target.target_joint_names.len() != sample.joint_names.len() {
            return None;
        }
        let mut target_sorted = target.target_joint_names.clone();
        let mut sample_sorted = sample.joint_names.clone();
        target_sorted.sort();
        sample_sorted.sort();
        if target_sorted != sample_sorted {
            return None;
        }

        let mut max_error = 0.0_f64;
        for (i, name) in target.target_joint_names.iter().enumerate() {
            // Guaranteed to be found: the set-equality check above already
            // confirmed `name` appears somewhere in `sample.joint_names`.
            let sample_index = sample.joint_names.iter().position(|n| n == name)?;
            let error = (target.target_positions[i] - sample.positions[sample_index]).abs();
            max_error = max_error.max(error);
        }
        Some(max_error)
    }

    /// The generic per-tick logic every [`state_dispatch`] arm calls: if
    /// `index` is (still) the active state and `sample` shows it has
    /// arrived (see `max_joint_error`) for the first time since entering,
    /// auto-fires its `on_arrival` transition (if it has one). A no-op in
    /// every other case (not arrived yet; already auto-fired this visit;
    /// no `on_arrival` configured at all; `sample`'s joint set doesn't
    /// match).
    fn tick_state(&mut self, index: usize, sample: &JointSample) {
        if !self.running {
            return; // stopped -- see module doc comment's /fsm/set_running section
        }
        if self.current_index != Some(index) {
            return; // stale dispatch -- current state moved on since this tick began
        }
        let Some(max_error) = self.max_joint_error(index, sample) else { return };
        if max_error >= self.states[index].epsilon {
            return; // not (yet) arrived this tick
        }
        if self.already_fired_this_visit {
            return; // already auto-fired once since entering -- see module doc comment
        }
        let Some(on_arrival) = self.states[index].on_arrival.clone() else { return };
        self.already_fired_this_visit = true;
        if let Err(message) = self.fire_transition_by_name(&on_arrival) {
            // Should be unreachable -- /fsm/update_logic's own validation
            // requires on_arrival to name one of this state's own
            // transitions -- but a stray warning beats a silent stall or
            // a panic if that invariant is ever violated some other way.
            eprintln!("[fsm_node] on_arrival transition {on_arrival:?} on state {:?} failed unexpectedly: {message}", self.states[index].name);
        }
    }

    /// Dispatches one tick's worth of arrival-checking work for whichever
    /// state is currently active through [`state_dispatch`]'s generated
    /// flat `match` -- see that macro's own doc comment.
    fn dispatch_tick(&mut self, index: usize, sample: &JointSample) {
        state_dispatch!(
            self, index, sample,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
             32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63]
        );
    }

    fn publish_status(&mut self) {
        let current_state = self.current_index.map(|i| self.states[i].name.clone());
        let time_in_state = if current_state.is_some() { self.state_entered_at.elapsed().as_secs_f64() } else { 0.0 };
        let msg = json!({
            "current_state": current_state,
            "time_in_state": time_in_state,
            "last_transition": self.last_transition,
            "running": self.running,
        });
        self.status_publisher.publish(&msg);
    }

    fn spin(&mut self) {
        loop {
            for (_topic, msg) in self.joint_states_subscriber.spin_once() {
                if let Some(sample) = Self::parse_joint_state(&msg) {
                    self.latest_joint_state = Some(sample);
                }
            }
            self.trajectory_publisher.spin_once();
            self.status_publisher.spin_once();

            if let Some(pending) = self.update_logic_service.spin_once() {
                self.handle_update_logic_request(pending);
            }
            if let Some(pending) = self.get_logic_service.spin_once() {
                self.handle_get_logic_request(pending);
            }
            if let Some(pending) = self.fire_transition_service.spin_once() {
                self.handle_fire_transition_request(pending);
            }
            if let Some(pending) = self.set_running_service.spin_once() {
                self.handle_set_running_request(pending);
            }
            if let Some(pending) = self.save_to_file_service.spin_once() {
                self.handle_save_to_file_request(pending);
            }
            if let Some(pending) = self.load_from_file_service.spin_once() {
                self.handle_load_from_file_request(pending);
            }
            if let Some(pending) = self.list_saved_service.spin_once() {
                self.handle_list_saved_request(pending);
            }

            if let (Some(index), Some(sample)) = (self.current_index, self.latest_joint_state.clone()) {
                self.dispatch_tick(index, &sample);
            }

            if self.last_status_publish.elapsed() >= self.status_publish_interval {
                self.publish_status();
                self.last_status_publish = Instant::now();
            }

            std::thread::sleep(Duration::from_millis(20));
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mut hub_host = "127.0.0.1".to_string();
    let mut hub_port: u16 = 9090;
    let mut status_rate_hz = DEFAULT_STATUS_RATE_HZ;
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
            "--status-rate" => {
                status_rate_hz = args[i + 1].parse().expect("--status-rate must be a positive number of Hz");
                i += 2;
            }
            other => {
                eprintln!("unrecognized argument: {other}");
                i += 1;
            }
        }
    }
    assert!(status_rate_hz > 0.0, "--status-rate must be a positive number of Hz");

    println!("[fsm_node] offering /fsm/update_logic, /fsm/get_logic, /fsm/fire_transition (max {MAX_STATES} states); publishing /fsm/status at {status_rate_hz}Hz");
    let mut node = FsmNode::new((hub_host, hub_port), status_rate_hz);
    node.spin();
}
