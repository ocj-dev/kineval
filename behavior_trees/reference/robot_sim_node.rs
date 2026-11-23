// ===========================================================================
// SNAPSHOT -- DO NOT EDIT THIS COPY
//
//   Source repository: /ai/development/autorob_agentic/pubsub_testbed_rs
//   Source path:       src/bin/robot_sim_node.rs
//   Upstream length:   1714 lines
//   Commit:            d9e34cd16c47c790856b1ccea0d8575620c66f89
//   Snapshot date:     2026-11-23
//
// Why this copy exists: Slidev's `<<<` snippet importer cannot read
// outside its own workspace root, which behavior_trees/pnpm-workspace.yaml
// pins at behavior_trees/. The deck quotes Rust that lives in a DIFFERENT
// REPOSITORY entirely, so the file has to be carried here to be quotable
// at all.
//
// Quoted here for: the simulator. Quoted for ONE function, apply_trajectory, which reads /joint_trajectory and keeps only points.last().
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
//   apply-trajectory
//
// Edits belong upstream, followed by a re-snapshot. Diff this file
// against the commit above to check it has not drifted.
// ===========================================================================

//! robot_sim_node: the sole `/joint_states` publisher for an arbitrary
//! URDF-described robot in the Project 4 (inverse-kinematics +
//! rigid-body-dynamics) deployment -- this node now owns **three**
//! interchangeable simulation methods, not two: the two real dynamics
//! backends below, *and* a third, purely kinematic P-only servo
//! (`"kinematics-only-JSP"`) that used to be `joint_state_publisher.rs`'s
//! own exclusive job. See "Why kinematics-only lives here now, not in a
//! separate process" below for why that moved.
//!
//! ## Why kinematics-only lives here now, not in a separate process
//!
//! An earlier revision of this node *stood down* entirely under
//! `"kinematics-only-JSP"` (`self.backend` stayed `None`, nothing
//! published), on the assumption that a separate, always-running
//! `joint_state_publisher.rs` process was the one actually driving
//! `/joint_states` in that mode -- `launch_ik_docker.bash` ran both nodes
//! side by side permanently, with only the live `simulation_method`
//! parameter deciding which one's publish was real. That assumption broke
//! the moment a user actually did what this node exists to let them do:
//! pick a real dynamics backend from `robot_fk.html`'s own Rigid-Body
//! Simulation pane. `joint_state_publisher.rs` has **no knowledge of
//! `simulation_method` at all** -- it kept independently subscribing
//! `/joint_trajectory` and publishing its own competing `/joint_states`
//! regardless, so selecting `"maximal-EPBD"`/`"generalized-newton_euler"`
//! put two independently-controlled authorities (this node's real PID +
//! dynamics, and `joint_state_publisher`'s own un-coordinated P-only
//! servo) racing to publish the same topic and servo the same
//! `/joint_trajectory` setpoint -- the actual, reported cause of visibly
//! unstable joint motion in that deployment. Implementing the kinematics-
//! only law directly in this node (see [`RobotSimNode::advance_kinematics_only`])
//! instead makes this node the one and only `/joint_states` publisher for
//! every value of `simulation_method`, so `launch_ik_docker.bash` no
//! longer needs to run `joint_state_publisher.rs` at all.
//! `joint_state_publisher.rs` itself is unmodified and keeps its original
//! role in `launch_fk_docker.bash`'s plain FK-only deployment, which has
//! no `robot_sim_node` to conflict with.
//!
//! This node also now offers four services under `joint_state_publisher`'s
//! *own* historical names -- `/joint_state_publisher/robot_description_list`,
//! `/joint_state_publisher/robot_description_select`,
//! `/joint_state_publisher/robot_get_visual_geometries`, and
//! `/joint_state_publisher/servo_toggle`/`/joint_state_publisher/set_kp`
//! (the last two controlling the kinematics-only backend specifically, as
//! distinct from and in addition to this node's own existing
//! `/robot_sim/servo_toggle`/`/robot_sim/set_gains`, which only ever
//! applied to the two PID-driven dynamics backends) -- see
//! "Communication" below. `robot_fk.html` calls these same URLs
//! unconditionally on every connect/robot-load (the robot picker dropdown,
//! link visual geometry, and the Joints pane's servo controls), regardless
//! of which launch script started the backend; keeping the exact same
//! service names here, re-implemented independently rather than shared
//! (this project's usual small-duplication-over-cross-binary-sharing
//! convention -- see e.g. `save_jrdf`'s own near-identical copies in
//! `robot_state_publisher.rs`/`joint_state_publisher.rs`), means
//! `robot_fk.html` needs no changes at all to keep working once
//! `joint_state_publisher.rs` is no longer running alongside it.
//!
//! ## Three interchangeable simulation methods
//!
//! This node can simulate the very same URDF robot with either of two
//! physically-independent dynamics engines, or with no dynamics at all,
//! selected live via the `simulation_method` parameter (see
//! "Communication" below):
//!
//! - **`"kinematics-only-JSP"`** (the default -- see
//!   [`DEFAULT_SIMULATION_METHOD`]) -- no PID, no dynamics model, no
//!   motor effort at all: [`RobotSimNode::advance_kinematics_only`] moves
//!   `q` directly toward `self.setpoint` with a single proportional gain
//!   (`self.jsp_kp`, live-settable via `/joint_state_publisher/set_kp`),
//!   or snaps it there instantly while
//!   `/joint_state_publisher/servo_toggle` has the servo bypassed (the
//!   default -- see that handler's own doc comment). This is a direct,
//!   independently-reimplemented port of `joint_state_publisher.rs`'s own
//!   `advance`/`shortest_angle_difference` -- see "Why kinematics-only
//!   lives here now" above for why it moved rather than staying a
//!   cross-process dependency. The name keeps its old `"-JSP"` suffix
//!   for backward compatibility with anything that already sets this
//!   parameter string, even though nothing named `joint_state_publisher`
//!   is involved in producing it anymore.
//! - **`"generalized-newton_euler"`** -- `rigid_body_dynamics.rs`'s
//!   Newton-Euler recursion in *generalized* (joint-space) coordinates,
//!   integrated forward with RK4 (`integrators::step`). Every joint is
//!   *always* exactly one scalar value by construction -- there is no
//!   maximal-coordinate pose to drift apart or need correcting, but the
//!   mass matrix this backend inverts every evaluation grows as an
//!   `O(n^3)` linear solve in the robot's own DOF count `n`.
//! - **`"maximal-EPBD"`** -- `xpbd_dynamics.rs`'s Extended Position-Based
//!   Dynamics, following Müller, Macklin, Chentanez, Jeschke & Kim,
//!   "Detailed Rigid Body Simulation with Extended Position Based
//!   Dynamics," *Computer Graphics Forum* (Proc. SCA 2020). Every link
//!   gets its own free 6-DOF maximal-coordinate pose, and each URDF
//!   joint becomes a positional + angular constraint (with an optional
//!   limit constraint) solved directly on positions/orientations every
//!   sub-step -- no linear system to assemble or invert at all, `O(n)`
//!   per sub-step in the number of joints, and unconditionally stable
//!   even through a joint limit collision (see that module's own doc
//!   comment for the two sign-convention bugs root-caused and fixed
//!   while bringing this backend up, both found by cross-validating
//!   against this very Newton-Euler backend's own already-tested steady
//!   states).
//!
//! Both dynamics backends are driven by the exact same PID controller
//! (`pid.rs`, unchanged) computing the exact same per-joint motor effort
//! from the exact same setpoint/trajectory/gain machinery below -- see
//! [`SimBackend`] for the small bridging surface (`dof_names`,
//! `q_values`/`qdot_values`, `step`) that lets the rest of this node stay
//! oblivious to which backend is currently active. That PID servo can
//! also be switched off independently of which dynamics backend is
//! active (see `/robot_sim/servo_toggle` below) -- a robot with the
//! servo off and a dynamics backend active simply falls under gravity
//! with zero commanded effort, rather than holding any setpoint.
//!
//! ## Scope, matching the Project 4 spec exactly
//!
//! - **The base link is never dynamically simulated.** `root_link` is
//!   treated as rigidly fixed to whatever `/global_pose` currently says
//!   (the same mechanism `robot_world_state_publisher` already uses for
//!   rendering -- see that node's own module doc comment) -- this is the
//!   classical *fixed-base manipulator* dynamics problem, just
//!   generalized from `arm_dynamics.rs`'s single planar chain to an
//!   arbitrary 3D tree. `/global_pose` can still change at any time (a
//!   user or another node teleporting/driving the robot); this node
//!   reacts to that only by re-deriving which direction gravity currently
//!   points in the base's own frame (see "Gravity and a tilted base"
//!   below) -- it never treats the base itself as a simulated degree of
//!   freedom.
//! - **No contact or friction.** Every joint's own motion is governed
//!   purely by its PID-driven actuator effort and whichever dynamics
//!   backend is active -- there is no ground plane, no self-collision, no
//!   joint friction model of any kind, in either backend.
//! - **Joints are enforced by construction (Newton-Euler) or by a solved
//!   positional constraint every sub-step (XPBD), never approximately.**
//!   Neither backend ever lets two linked bodies visibly drift apart.
//!
//! ## Gravity and a tilted base
//!
//! `/global_pose` is genuine ROS convention (see
//! `robot_world_state_publisher.rs`'s own module doc comment): a real
//! orientation quaternion, not just a ground-plane heading. If the robot
//! (or its mount) is tilted -- lying on its side, say -- gravity is
//! **not** straight down in `root_link`'s own frame anymore. Every
//! dynamics evaluation therefore rotates the fixed world gravity vector
//! `[0, 0, -g]` into `root_link`'s frame using `/global_pose`'s own
//! orientation (its *inverse*, since `/global_pose`'s rotation maps
//! `root_link`'s axes into world axes, and this needs the opposite
//! direction) before handing it to either backend -- see
//! [`RobotSimNode::gravity_in_root_frame`].
//!
//! ## One control update per integration step, held fixed across each
//! backend's own internal sub-stage evaluations
//!
//! Mirrors `arm_sim_node`'s own "one node, not two" architecture exactly:
//! the PID controller computes effort *once* per [`RobotSimNode::advance`]
//! call, from the state at the *start* of the step, and that one effort
//! vector is held fixed (a zero-order hold, the same assumption any real
//! digital control loop running faster than its own sensor update makes)
//! across the Newton-Euler backend's four RK4 sub-stages, or handed
//! directly to the XPBD backend's own single `substep` call as the
//! "motor torque" external force Müller et al. 2020 Section 3.4
//! describes -- never recomputed partway through either.
//!
//! ## Communication
//!
//!   - Polls `param_server_node` for `"robot_description"` (same
//!     poll-for-version-change pattern `joint_state_publisher.rs` already
//!     uses).
//!   - Polls `param_server_node` for `"simulation_method"` the same way:
//!     `"kinematics-only-JSP"`, `"generalized-newton_euler"`, or
//!     `"maximal-EPBD"` (any other value, or the parameter never having
//!     been set at all, keeps whatever method is already active --
//!     defaulting to [`DEFAULT_SIMULATION_METHOD`] if nothing has ever
//!     loaded). A live change rebuilds the active backend from scratch at
//!     the midpoint of every joint's own limits -- see
//!     [`RobotSimNode::rebuild_backend`].
//!   - Subscribes `/joint_trajectory` (`trajectory_msgs/JointTrajectory`):
//!     the *last* waypoint's `positions` become the held PID setpoint --
//!     the same "step-setpoint servo, not a trajectory follower" contract
//!     `arm_sim_node`/`joint_state_publisher` already use. A full
//!     `FollowJointTrajectory` *action* (interpolating through multiple
//!     waypoints over real time) is explicitly deferred to Project 5.
//!   - Subscribes `/global_pose` (`geometry_msgs/Pose`-shaped) -- see
//!     "Gravity and a tilted base" above. Defaults to the identity
//!     orientation if nothing has ever been published (matching
//!     `robot_world_state_publisher`'s own default).
//!   - Publishes `/joint_states` at `--publish-rate` Hz (default
//!     [`DEFAULT_PUBLISH_RATE`], 10 -- this project's now-standard rate
//!     for every render-consumed topic).
//!   - Offers `/robot_sim/reset` (request `{}`): snaps every joint to the
//!     midpoint of its own limits (`0.0` for `continuous`, which has
//!     none -- the same "known, unsurprising rest pose" rule
//!     `joint_state_publisher`'s own `/joint_state_publisher/reset`
//!     uses), velocity to zero, the PID setpoint to match (so it doesn't
//!     immediately get servoed away from the pose it was just reset to),
//!     and clears the PID integral accumulator.
//!   - Offers `/robot_sim/set_gains` (request `{"gains": {"<joint name>":
//!     {"kp": f64, "ki": f64, "kd": f64}, ...}}`, every inner field
//!     optional) -> `{"values": {"gains": {<joint name>: {"kp", "ki",
//!     "kd"}, ...}}, "result": bool, "status": str}` -- an empty request
//!     `{}` queries the current full set without changing anything, the
//!     same convention `arm_sim_node`'s own `/pid_controller/set_gains`
//!     uses, generalized from a fixed-size positional array to a
//!     joint-name-keyed map (this node's own joint set, unlike
//!     `arm_sim_node`'s fixed `--links`, is only known once a
//!     `robot_description` has actually loaded, and can vary robot to
//!     robot). The whole request is validated before anything is applied
//!     -- an unknown joint name or a negative gain anywhere in the
//!     request rejects the entire call (`result: false`, nothing
//!     changed) rather than silently applying part of it. Gains are
//!     shared across both backends (the same `PidController`, sized once
//!     per loaded robot, survives a live `simulation_method` switch).
//!   - Offers `/robot_sim/save_gains` / `/robot_sim/load_gains` /
//!     `/robot_sim/list_saved_gains` -- named-file persistence for the
//!     gains above, so a student doesn't have to re-tune by hand every
//!     time the same robot reloads. Same `<identifier>.json`-under-a-
//!     flat-directory convention `fsm_node.rs`'s own `/fsm/save_to_file`/
//!     `/fsm/load_from_file`/`/fsm/list_saved` already establish (see
//!     that file's own "Named-file persistence" module doc section and
//!     [`validate_gains_identifier`]'s own doc comment for the identical
//!     path-escape threat model) -- `gains/<identifier>.json`, sibling to
//!     `fsm/`, not nested under `robots/` (a saved gain set is not a
//!     robot description either). The one difference from `fsm_node.rs`'s
//!     own pair: `"identifier"` is *optional* here, defaulting to the
//!     currently-loaded robot's own `<robot name="...">` (`self.
//!     current_robot`'s own `name`) -- "save/load gains for each robot
//!     description" (the common case: one tuned gain set per robot, no
//!     typing required) falls out of that default for free, while an
//!     explicit `"identifier"` still lets a student keep multiple named
//!     presets for the same robot if they want to.
//!     - `/robot_sim/save_gains` (request `{"identifier": <str,
//!       optional>}` -> `{"values": {"path": str}, "result": bool,
//!       "status": str}`) writes the current gains (exactly
//!       [`RobotSimNode::gains_values`]'s own shape) plus the saving
//!       robot's own name (for [`RobotSimNode::handle_load_gains`]'s
//!       informational mismatch check below) to `gains/<identifier>.json`.
//!       Fails (`result: false`) if no robot is loaded yet (nothing to
//!       save, and no default identifier to fall back to).
//!     - `/robot_sim/load_gains` (request `{"identifier": <str,
//!       optional>}` -> `{"values": {"gains": {...}, "robot_name_matched":
//!       bool}, "result": bool, "status": str}`) reads the file back and
//!       applies it via the exact same all-or-nothing
//!       [`RobotSimNode::apply_set_gains`] validation `/robot_sim/
//!       set_gains` itself uses -- a saved file whose joint names don't
//!       match the currently-loaded robot fails cleanly with a clear
//!       per-joint error rather than silently applying a partial, wrong
//!       set. `robot_name_matched` is `false` whenever the file's own
//!       saved robot name differs from the currently-loaded robot's own
//!       name -- purely informational (the joint-name validation above is
//!       what actually protects correctness; two different robots could
//!       coincidentally share every joint name), surfaced so a front-end
//!       can warn even in that coincidental-match case.
//!     - `/robot_sim/list_saved_gains` (`{}` -> `{"values":
//!       {"identifiers": [...]}, "result": true, "status": ""}`) --
//!       every `<name>.json` direct child of `gains/`, sorted, mirroring
//!       `/fsm/list_saved`'s own identical `list_saved` helper.
//!   - Offers `/robot_sim/estimate_gains` (request `{"omega_n": <f64,
//!     optional, default [`DEFAULT_ESTIMATE_OMEGA_N`]>, "zeta": <f64,
//!     optional, default [`DEFAULT_ESTIMATE_ZETA`]>, "ti": <f64,
//!     optional -- integral time constant; omitted or absent means
//!     `ki = 0`>}` -> `{"values": {"gains": {...}}, "result": bool,
//!     "status": str}`, in exactly [`RobotSimNode::gains_values`]'s own
//!     shape) -- a **pure, read-only** suggestion, never applied on its
//!     own (pair it with `/robot_sim/set_gains` to actually use the
//!     result; this split lets a front-end show a preview, or a student
//!     inspect numbers, before trusting them). Ziegler-Nichols's own
//!     closed-loop ("ultimate gain") method was considered and rejected
//!     for this role: it needs a live, per-joint search that deliberately
//!     pushes each joint to the brink of sustained oscillation one at a
//!     time, which is both unsafe (this robot's joints are *not*
//!     dynamically independent -- `DynamicsModel::mass_matrix`'s own
//!     off-diagonal terms are real inertial coupling, so destabilizing one
//!     joint while hunting for its own ultimate gain can destabilize its
//!     neighbors too) and impractical at this project's own scale (PR2
//!     alone has 87 joints to search, sequentially, live). This service
//!     instead uses the plant model this project already has in closed
//!     form: for one joint modeled as a decoupled second-order system
//!     `I \ddot q + K_d \dot q + K_p q = 0` (the standard mass-spring-
//!     damper form a PID loop reduces to when only the P and D terms are
//!     considered), choosing a desired natural frequency `omega_n` and
//!     damping ratio `zeta` directly gives
//!     $$ K_p = I\,\omega_n^2, \qquad K_d = 2\zeta I\,\omega_n, \qquad
//!     K_i = K_p / T_i \text{ (if a } T_i \text{ was given).}$$
//!     `I`, each joint's own effective inertia, is read straight off the
//!     diagonal of a *fresh* `rigid_body_dynamics::DynamicsModel`'s own
//!     `mass_matrix` at the midpoint-of-limits pose ([`RobotSimNode::
//!     midpoint_q_map`] -- the same rest pose `rebuild_backend` already
//!     uses) -- built directly from `self.current_robot`, independent of
//!     whichever `simulation_method` backend happens to be active (XPBD
//!     exposes no mass matrix of its own, and this estimate is a property
//!     of the robot's mass distribution, not of either integrator).
//!     Critically damped (`zeta = 1.0`) is the default specifically
//!     because it is the fastest response with *no overshoot* -- a safer
//!     default than an underdamped one for a student who hasn't yet
//!     looked at what `omega_n` they asked for actually does to their
//!     specific robot.
//!   - Offers `/robot_sim/servo_toggle` (request `{"data": <bool,
//!     optional>}` -> `{"values": {"enabled": bool}, "result": true,
//!     "status": str}`) to turn the PID servo on/off live -- the same
//!     query-or-set convention `joint_state_publisher.rs`'s own
//!     `/joint_state_publisher/servo_toggle` uses (omit `data` to query
//!     the current state without changing it, which a front-end checkbox
//!     needs on connect/robot-load), not `arm_sim_node`'s own
//!     `/pid_controller/enable` (which always requires `data`). **Off by
//!     default** -- mirroring `arm_sim_node`'s own `/pid_controller/
//!     enable` default and its documented reasoning: a student should
//!     see the robot move under gravity/dynamics alone first, and turn
//!     control on as a deliberate second step. While off, [`Self::advance`]
//!     commands zero effort without ever calling `PidController::update`
//!     (so no integral accumulates against a setpoint while the servo
//!     isn't driving toward it); re-enabling (false -> true) clears the
//!     integral via `pid.reset()` regardless, the same defensive "nothing
//!     meaningful could have accumulated anyway, but clear it" rule
//!     `arm_sim_node`'s own handler follows.
//!   - Offers `/robot_sim/pause` (`std_srvs/SetBool`-shaped request
//!     `{"data": <bool>}` -> `{"values": {}, "result": true, "status":
//!     str}`) to freeze/resume the simulation clock, mirroring
//!     `arm_sim_node`'s own `/arm_sim/pause` exactly: while paused, this
//!     node keeps polling and publishing (so a front-end still sees live
//!     `/joint_states` traffic and every other service keeps responding),
//!     it just stops calling [`Self::advance`] -- no dynamics step, no
//!     PID update fed a frozen plant's state. Malformed/missing `data`
//!     defaults to `true` (pause), the same fail-safe `arm_sim_node`'s
//!     own handler uses.
//!   - Offers four services under `joint_state_publisher.rs`'s own
//!     historical names -- see "Why kinematics-only lives here now" above
//!     for why, each an independent reimplementation (not shared code) of
//!     that file's own handler of the same name:
//!     - `/joint_state_publisher/robot_description_list` /
//!       `/joint_state_publisher/robot_description_select` -- the
//!       `robots/` directory browsing pair `robot_fk.html`'s robot picker
//!       dropdown calls on every connect/refresh.
//!     - `/joint_state_publisher/robot_get_visual_geometries` -- the
//!       visual-relevant subset (plus a `generation` counter,
//!       incremented once per successful `robot_description` parse) of
//!       this node's own independently-parsed `current_robot`.
//!     - `/joint_state_publisher/servo_toggle` /
//!       `/joint_state_publisher/set_kp` -- enable/bypass and gain
//!       control for *this node's own* kinematics-only backend
//!       specifically (`self.jsp_servo_enabled`/`self.jsp_kp`), entirely
//!       independent of `/robot_sim/servo_toggle`/`/robot_sim/set_gains`
//!       above, which only ever control the two PID-driven dynamics
//!       backends. Bypassed (instant/direct position control) by
//!       default, exactly matching `joint_state_publisher.rs`'s own
//!       `--servo-enabled false` default.

use pubsub_testbed::p2p::node::{Node, PendingRequest, Publisher, Service, Subscriber};
use pubsub_testbed::pid::PidController;
use pubsub_testbed::ros_messages as rm;
use pubsub_testbed::rigid_body_dynamics::DynamicsModel;
use pubsub_testbed::transforms3d::{quat_conjugate, rotate_vector};
use pubsub_testbed::xpbd_dynamics::{RigidBody, XpbdModel};
use pubsub_testbed::{integrators, urdf};
use serde_json::{json, Value};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// Default PID gains applied to every joint whenever a robot is (re)loaded
/// -- generic starting points, not tuned to any particular robot's own
/// mass/inertia scale (which varies hugely across this project's bundled
/// robots, from a small student arm to PR2). Live-adjustable per joint via
/// `/robot_sim/set_gains`, which is the whole point of exposing them to
/// the front-end at all rather than hand-tuning them here.
const DEFAULT_KP: f64 = 50.0;
const DEFAULT_KI: f64 = 0.0;
const DEFAULT_KD: f64 = 5.0;
const INTEGRAL_LIMIT: f64 = 50.0;

/// `/robot_sim/estimate_gains`'s own default desired closed-loop natural
/// frequency, in rad/s, used whenever a request omits `"omega_n"` -- see
/// this file's own module doc comment for the full derivation. Unlike
/// [`DEFAULT_KP`]/[`DEFAULT_KD`] above, this is *not* itself a gain --
/// it's a bandwidth choice independent of any one robot's own mass scale
/// (the per-joint effective inertia `I` is what actually scales the
/// resulting `Kp`/`Kd` to the robot at hand), so one fixed default here
/// is far less arbitrary than one fixed default gain would be.
const DEFAULT_ESTIMATE_OMEGA_N: f64 = 10.0;

/// `/robot_sim/estimate_gains`'s own default damping ratio -- `1.0`
/// (critically damped: the fastest response with *no overshoot*) is the
/// safer default for a student who hasn't yet seen what their chosen
/// `omega_n` actually does to this specific robot; see this file's own
/// module doc comment.
const DEFAULT_ESTIMATE_ZETA: f64 = 1.0;

/// Default proportional gain for the kinematics-only backend specifically
/// (`/joint_state_publisher/set_kp`) -- deliberately a separate constant
/// from [`DEFAULT_KP`] above, which sizes the *PID* gains for the two
/// dynamics backends instead; the two are never used by the same backend
/// at the same time, but keeping them as distinct constants (rather than
/// reusing one) avoids ever implying a numeric coincidence that isn't
/// there. Identical value and derivation to `joint_state_publisher.rs`'s
/// own `DEFAULT_KP` -- see that file's own doc comment for the discrete-
/// step stability bound (`kp * dt < 2`) this value satisfies.
const DEFAULT_JSP_KP: f64 = 50.0;

/// Fixed integration timestep. **This is a starting point, not a value
/// provably stable for every robot this node might load** -- exactly the
/// same caveat `arm_sim_node.rs`'s own `DEFAULT_DT` doc comment documents
/// for the planar arm: a closed PID loop is numerically stiffer than the
/// open-loop dynamics alone, and a heavy, many-joint robot (PR2) and a
/// light student arm do not necessarily share one safe timestep. 1kHz is
/// a reasonable, commonly-used starting point for a real digital joint
/// controller; `--dt` exists specifically so this can be tightened for a
/// robot (or gain set) that diverges at the default. The XPBD backend
/// additionally treats this as its own sub-step length `h` (Müller et
/// al. 2020's own "small steps" philosophy -- see `xpbd_dynamics.rs`'s
/// module doc comment -- is already satisfied at 1kHz without this node
/// needing a *second*, separate sub-stepping parameter on top of `--dt`).
const DEFAULT_DT: f64 = 0.001;

/// The most *wall-clock* time one outer loop tick will ever spend inside
/// the dynamics catch-up loop before unconditionally yielding back to
/// this node's own `Service::spin_once` polling -- see `spin`'s own doc
/// comment for the real, measured bug this exists to prevent.
///
/// **Why a wall-clock budget, not a fixed sub-step *count*:** an earlier
/// revision of this node bounded the *count* of `advance()` calls per
/// tick instead (`MAX_SUBSTEPS_PER_TICK = 50`). That still starved this
/// node's own services for PR2 (87 joints): each `advance()` call costs
/// (for the Newton-Euler backend) four full `O(n)` Newton-Euler passes
/// plus an `O(n^3)` linear solve (see `rigid_body_dynamics.rs`), and a
/// *fixed* sub-step count silently assumes a roughly fixed per-step cost
/// -- which is wildly untrue across this project's own bundled robots
/// (PR2's 87 joints vs. a 2-DOF student arm can differ by orders of
/// magnitude). Measured directly against a real, running PR2 simulation:
/// even 50 bounded sub-steps still took long enough, repeatedly, that a
/// 5-second client timeout on `/robot_sim/set_gains` elapsed before this
/// node's own loop ever got back around to checking that service's
/// socket at all -- the *simulated*-time bound this constant's
/// predecessor enforced (50ms of sim time per tick) said nothing about
/// the *real* seconds that cost on this hardware. Budgeting by
/// wall-clock time directly fixes that for *any* robot, under *either*
/// backend: this loop always returns to service its sockets within
/// (approximately) this many milliseconds, no matter how expensive one
/// `advance()` call turns out to be -- the honest trade is that a robot
/// too expensive to simulate at `--dt` in real time runs in visible slow
/// motion (`self.sim_time` falls behind wall-clock) rather than ever
/// starving this node's own responsiveness.
const MAX_CATCHUP_BUDGET: Duration = Duration::from_millis(20);

/// See this file's own module doc comment -- this project's now-standard
/// publish rate for every render-consumed topic.
const DEFAULT_PUBLISH_RATE: f64 = 10.0;

/// Numerical-safety backstops against a transient PID overshoot ever
/// diverging to `NaN`/infinity -- generic, generous bounds (not realistic
/// per-robot actuator limits) in the same spirit as `arm_sim_node.rs`'s
/// own `--max-effort`/`--max-velocity`. **`--max-velocity` only applies to
/// the Newton-Euler backend** -- see [`SimBackend::step`]'s own doc
/// comment for why the XPBD backend needs no such backstop.
const DEFAULT_MAX_EFFORT: f64 = 1000.0;
const DEFAULT_MAX_VELOCITY: f64 = 50.0;

const DEFAULT_GRAVITY: f64 = 9.81;

const PARAM_POLL_INTERVAL: Duration = Duration::from_millis(500);
const PARAM_SERVICE_TIMEOUT: Duration = Duration::from_secs(2);

/// The `simulation_method` parameter's accepted string values -- see this
/// file's own module doc comment's "Three interchangeable simulation
/// methods" section.
const SIMULATION_METHOD_KINEMATICS_ONLY: &str = "kinematics-only-JSP";
const SIMULATION_METHOD_NEWTON_EULER: &str = "generalized-newton_euler";
const SIMULATION_METHOD_XPBD: &str = "maximal-EPBD";

/// Which simulation method is active. Defaults to
/// [`SimulationMethod::KinematicsOnly`] (`"kinematics-only-JSP"`) -- the
/// purely kinematic P-only servo this node now implements directly (see
/// this file's own module doc comment's "Why kinematics-only lives here
/// now" section), the behavior this project had before this node existed
/// at all, remains the default; a real dynamics backend is something a
/// user opts into live, not something that starts simulating by itself
/// the moment this process happens to be running.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SimulationMethod {
    KinematicsOnly,
    NewtonEuler,
    Epbd,
}

const DEFAULT_SIMULATION_METHOD: SimulationMethod = SimulationMethod::KinematicsOnly;

impl SimulationMethod {
    fn parse(text: &str) -> Option<SimulationMethod> {
        match text {
            SIMULATION_METHOD_KINEMATICS_ONLY => Some(SimulationMethod::KinematicsOnly),
            SIMULATION_METHOD_NEWTON_EULER => Some(SimulationMethod::NewtonEuler),
            SIMULATION_METHOD_XPBD => Some(SimulationMethod::Epbd),
            _ => None,
        }
    }
}

fn saturate(values: &mut [f64], limit: f64) {
    for v in values.iter_mut() {
        *v = v.clamp(-limit, limit);
    }
}

/// Computes `setpoint - current`, wrapped into `(-pi, pi]` before scaling
/// by a proportional gain -- an independent reimplementation of
/// `joint_state_publisher.rs`'s own function of the same name; see that
/// file's own doc comment for why a naive subtraction is wrong for a
/// revolute/continuous joint (it can servo the long way around a full
/// turn for two configurations that are actually nearly identical).
fn shortest_angle_difference(setpoint: f64, current: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let mut diff = (setpoint - current) % tau;
    if diff > std::f64::consts::PI {
        diff -= tau;
    } else if diff <= -std::f64::consts::PI {
        diff += tau;
    }
    diff
}

/// Where saved gain sets live -- see this file's own module doc
/// comment's `/robot_sim/save_gains` section. Project root, sibling to
/// `robots/`/`fsm/` (a saved gain set is not a robot description).
const GAINS_DIR: &str = "gains";

/// Rejects anything that isn't a single, ordinary path segment --
/// `identifier` gets joined onto [`GAINS_DIR`] to build a filesystem
/// path, and every service in this project (including `/robot_sim/
/// save_gains`/`/robot_sim/load_gains`) is reachable from an untrusted
/// browser via `rosbridge_server`, so a caller-controlled `identifier`
/// containing a path separator or a `..` component is a real path-escape
/// risk, not a hypothetical one. Deliberately identical to
/// `fsm_node.rs`'s own `validate_fsm_identifier` (same threat model, same
/// fix, same reasoning) rather than `robot_state_publisher.rs`'s own more
/// permissive `resolve_output_path` rule -- an "identifier" here names
/// one flat file, not an arbitrary nested output path.
fn validate_gains_identifier(identifier: &str) -> Result<(), String> {
    if identifier.is_empty() || identifier.contains('/') || identifier.contains('\\') || identifier == "." || identifier == ".." {
        return Err(format!("{identifier:?} is not a valid identifier (must be a single path segment, no \"/\", \"\\\\\", \".\", or \"..\")"));
    }
    Ok(())
}

/// The path `gains/<identifier>.json` would resolve to, if `identifier`
/// is valid -- shared by every `/robot_sim/..._gains` handler so the
/// `<identifier>.json` naming convention lives in exactly one place,
/// mirroring `fsm_node.rs`'s own `fsm_file_path`.
fn gains_file_path(identifier: &str) -> PathBuf {
    Path::new(GAINS_DIR).join(format!("{identifier}.json"))
}

/// Where this project's own bundled robot descriptions live -- see
/// `robots/README.md` for the directory convention. Identical to
/// `joint_state_publisher.rs`'s own [`ROBOTS_DIR`], duplicated here for
/// the same reason the four `/joint_state_publisher/...` services below
/// are: see this file's own module doc comment's "Why kinematics-only
/// lives here now" section.
const ROBOTS_DIR: &str = "robots";

/// The only file extensions counted as a loadable robot description by
/// the recursive `robots/` scan -- see `joint_state_publisher.rs`'s own
/// identical constant's doc comment for the full rationale (matched
/// case-insensitively; everything else in the tree is skipped).
const DESCRIPTION_EXTENSIONS: [&str; 2] = ["urdf", "json"];

/// How many directory levels below `robots/` the recursive scan will
/// descend -- a termination guarantee against a self-referential symlink,
/// not a meaningful limit on real content. See `joint_state_publisher.rs`'s
/// own identical constant.
const MAX_SCAN_DEPTH: usize = 8;

/// True iff `path`'s own extension is one of [`DESCRIPTION_EXTENSIONS`].
fn has_description_extension(path: &Path) -> bool {
    let Some(extension) = path.extension().and_then(|e| e.to_str()) else { return false };
    DESCRIPTION_EXTENSIONS.iter().any(|candidate| extension.eq_ignore_ascii_case(candidate))
}

/// Recursive worker behind [`list_robot_descriptions`] -- an independent
/// reimplementation of `joint_state_publisher.rs`'s own `collect_descriptions`;
/// see that file's own doc comment for the full walk semantics (an
/// unreadable directory contributes nothing rather than aborting the
/// whole scan, non-UTF-8 names are skipped, `prefix` is the `/`-separated
/// path built up so far).
fn collect_descriptions(dir: &Path, prefix: &str, depth: usize, out: &mut Vec<String>) {
    if depth > MAX_SCAN_DEPTH {
        return;
    }
    let Ok(entries) = std::fs::read_dir(dir) else { return };
    for entry in entries.flatten() {
        let path = entry.path();
        let Some(name) = path.file_name().and_then(|n| n.to_str()) else { continue };
        let relative = if prefix.is_empty() { name.to_string() } else { format!("{prefix}/{name}") };
        if path.is_dir() {
            collect_descriptions(&path, &relative, depth + 1, out);
        } else if has_description_extension(&path) {
            out.push(relative);
        }
    }
}

/// Every robot description `robots/` currently offers, as `/`-separated
/// paths relative to `robots/` itself, sorted alphabetically -- see
/// `joint_state_publisher.rs`'s own identical function for the full
/// contract (an empty/missing `ROBOTS_DIR` yields an empty list, not an
/// error; sorting the whole flat list at the end makes the result
/// filesystem-order-independent).
fn list_robot_descriptions() -> Vec<String> {
    let mut descriptions = Vec::new();
    collect_descriptions(Path::new(ROBOTS_DIR), "", 0, &mut descriptions);
    descriptions.sort();
    descriptions
}

/// Rejects anything that isn't a plain, strictly-downward relative path --
/// see `joint_state_publisher.rs`'s own identical function's doc comment
/// for the full threat model (this service is reachable from an
/// untrusted browser via `rosbridge_server`, so a caller-controlled
/// `name` is a real path-escape risk).
fn validate_description_name(name: &str) -> Result<(), String> {
    if name.is_empty() {
        return Err("request's \"name\" must not be empty".to_string());
    }
    if name.contains('\\') {
        return Err(format!("{name:?} is not a valid robot description path (no \"\\\\\" allowed; use \"/\" separators)"));
    }
    for segment in name.split('/') {
        if segment.is_empty() || segment == "." || segment == ".." {
            return Err(format!("{name:?} is not a valid robot description path (no empty, \".\", or \"..\" path segments)"));
        }
    }
    Ok(())
}

/// The description file `robot_description_select` should read for
/// `name`, or an explanatory `Err` -- see `joint_state_publisher.rs`'s own
/// identical function's doc comment for the two accepted spellings (a
/// relative path exactly as `robot_description_list` reports it, or a
/// bare directory name kept for backward compatibility).
fn find_description_file(name: &str) -> Result<PathBuf, String> {
    validate_description_name(name)?;
    let root = Path::new(ROBOTS_DIR);

    if has_description_extension(Path::new(name)) {
        let path = root.join(name);
        if path.is_file() {
            return Ok(path);
        }
        return Err(format!("no such robot description {name:?} under {ROBOTS_DIR}/ (see robot_description_list)"));
    }

    if name.contains('/') {
        return Err(format!("{name:?} is not a loadable robot description: expected one of the paths robot_description_list reports, each ending in {DESCRIPTION_EXTENSIONS:?}"));
    }

    for extension in DESCRIPTION_EXTENSIONS {
        let path = root.join(name).join(format!("{name}.{extension}"));
        if path.is_file() {
            return Ok(path);
        }
    }
    Err(format!("no such robot description {name:?} (expected either a path listed by robot_description_list, or a directory name with a matching {ROBOTS_DIR}/<name>/<name>.urdf)"))
}

/// One joint the kinematics-only backend tracks: just enough to replicate
/// `joint_state_publisher.rs`'s own P-only servo law (see
/// [`RobotSimNode::advance_kinematics_only`]) -- the joint's own type (so
/// the wraparound-aware [`shortest_angle_difference`] applies only to
/// revolute/continuous joints, never prismatic) and limit (for clamping).
/// `Fixed` joints are filtered out before this struct is ever built --
/// see `rebuild_backend`'s own `KinematicsOnly` arm.
struct KinematicJoint {
    joint_type: urdf::JointType,
    limit: Option<(f64, f64)>,
}

/// The small bridging surface that lets the rest of this node (PID
/// setpoint tracking, `/joint_states` publishing, limit bookkeeping) stay
/// oblivious to which dynamics backend is currently active -- see this
/// file's own module doc comment's "Two interchangeable dynamics
/// backends" section for what each variant actually computes.
enum SimBackend {
    NewtonEuler { model: DynamicsModel, q: Vec<f64>, qdot: Vec<f64> },
    Xpbd { model: XpbdModel, bodies: Vec<RigidBody> },
    /// No PID, no dynamics model -- see [`RobotSimNode::advance_kinematics_only`],
    /// which mutates `q`/`qdot` directly rather than going through
    /// [`SimBackend::step`] at all. `names`/`joints`/`q`/`qdot` are kept
    /// as parallel vectors, all indexed the same way (the same style
    /// `NewtonEuler`'s own `q`/`qdot` already use alongside
    /// `model.dof_names()`).
    KinematicsOnly { names: Vec<String>, joints: Vec<KinematicJoint>, q: Vec<f64>, qdot: Vec<f64> },
}

impl SimBackend {
    fn dof_names(&self) -> &[String] {
        match self {
            SimBackend::NewtonEuler { model, .. } => model.dof_names(),
            SimBackend::Xpbd { model, .. } => model.dof_names(),
            SimBackend::KinematicsOnly { names, .. } => names,
        }
    }

    fn dof_count(&self) -> usize {
        self.dof_names().len()
    }

    /// This backend's own joints, as a plain generalized-coordinate
    /// vector -- already true by construction for the Newton-Euler
    /// backend's own `q`, and recovered from the XPBD backend's live
    /// maximal-coordinate body poses via `XpbdModel::joint_value` (see
    /// that function's own doc comment) for the other.
    fn q_values(&self) -> Vec<f64> {
        match self {
            SimBackend::NewtonEuler { q, .. } => q.clone(),
            SimBackend::Xpbd { model, bodies } => (0..model.dof_count()).map(|i| model.joint_value(bodies, i)).collect(),
            SimBackend::KinematicsOnly { q, .. } => q.clone(),
        }
    }

    fn qdot_values(&self) -> Vec<f64> {
        match self {
            SimBackend::NewtonEuler { qdot, .. } => qdot.clone(),
            SimBackend::Xpbd { model, bodies } => (0..model.dof_count()).map(|i| model.joint_velocity(bodies, i)).collect(),
            SimBackend::KinematicsOnly { qdot, .. } => qdot.clone(),
        }
    }

    /// One physics step of length `dt`, given this step's already-computed
    /// (and already effort-saturated) motor torques/forces.
    ///
    /// For the **Newton-Euler** backend this is exactly what it always
    /// was: one RK4 step of the generalized-coordinate equations of
    /// motion, followed by `max_velocity` saturation and `joint_limits`
    /// clamping applied directly to its own `q`/`qdot` -- both are needed
    /// *here* because this backend's own integration has no other
    /// defense against a transient PID overshoot diverging numerically
    /// (see [`DEFAULT_MAX_EFFORT`]'s doc comment).
    ///
    /// For the **XPBD** backend, neither applies, and this is
    /// deliberate, not an oversight: `motor_effort` is supplied in
    /// exactly the role Müller et al. 2020 Section 3.4 describes (an
    /// external generalized force folded into the *prediction* step, not
    /// a constraint), and every joint's own position/angle limit (if
    /// any) is already enforced as a genuine positional constraint
    /// *inside* `XpbdModel::substep` itself, every sub-step, regardless
    /// of velocity -- clamping `q`/`qdot` again afterward here would be
    /// redundant at best. There is also no `qdot` *to* clamp in the
    /// first place: the only state this backend keeps is each link's own
    /// maximal-coordinate pose and velocity, and `max_velocity` has no
    /// well-defined meaning against a per-link 6-DOF velocity the way it
    /// does against a single scalar joint-space `qdot`.
    fn step(&mut self, motor_effort: &[f64], gravity: [f64; 3], dt: f64, max_velocity: f64, joint_limits: &HashMap<String, Option<(f64, f64)>>) {
        match self {
            SimBackend::NewtonEuler { model, q, qdot } => {
                let n = q.len();
                let (q_next, mut qdot_next) = integrators::step(integrators::Integrator::RungeKutta4, 0.0, q, qdot, dt, |_t, q, qdot| {
                    model.forward_dynamics(q, qdot, motor_effort, gravity).unwrap_or_else(|| vec![0.0; n])
                });
                saturate(&mut qdot_next, max_velocity);

                let mut q_next = q_next;
                for (i, name) in model.dof_names().iter().enumerate() {
                    if let Some(Some((lower, upper))) = joint_limits.get(name) {
                        q_next[i] = q_next[i].clamp(*lower, *upper);
                    }
                }

                *q = q_next;
                *qdot = qdot_next;
            }
            SimBackend::Xpbd { model, bodies } => {
                model.substep(bodies, motor_effort, gravity, dt);
            }
            SimBackend::KinematicsOnly { .. } => {
                unreachable!("the kinematics-only backend is advanced directly by RobotSimNode::advance_kinematics_only, never through SimBackend::step -- see RobotSimNode::advance's own branch on simulation_method")
            }
        }
    }
}

struct RobotSimNode {
    node: Node,
    trajectory_subscriber: Subscriber,
    global_pose_subscriber: Subscriber,
    joint_state_publisher: Publisher,
    pubsubout_publisher: Publisher,
    reset_service: Service,
    set_gains_service: Service,
    save_gains_service: Service,
    load_gains_service: Service,
    list_saved_gains_service: Service,
    estimate_gains_service: Service,
    servo_toggle_service: Service,
    pause_service: Service,
    /// See this file's own module doc comment's "Why kinematics-only
    /// lives here now" section -- these five services reproduce
    /// `joint_state_publisher.rs`'s own historical `/joint_state_publisher/...`
    /// surface (directory browsing, visual geometry, and the kinematics-
    /// only backend's own servo controls) so `robot_fk.html` needs no
    /// changes once that node is no longer run alongside this one. `jsp_`
    /// is this file's own shorthand for that compatibility surface.
    jsp_robot_description_list_service: Service,
    jsp_robot_description_select_service: Service,
    jsp_robot_get_visual_geometries_service: Service,
    jsp_servo_toggle_service: Service,
    jsp_set_kp_service: Service,

    /// The most recently successfully-parsed robot, kept around (beyond
    /// just the dynamics model the parse produces) specifically so that
    /// [`RobotSimNode::rebuild_backend`] can re-derive a fresh backend --
    /// of any of the three kinds -- on demand, without needing to
    /// re-fetch or re-parse `robot_description` from `param_server_node`
    /// again, and so `/joint_state_publisher/robot_get_visual_geometries`
    /// has something to read visuals back out of.
    current_robot: Option<urdf::Robot>,
    backend: Option<SimBackend>,
    joint_limits: HashMap<String, Option<(f64, f64)>>,
    last_seen_robot_description_version: u64,
    last_param_poll: Instant,
    /// Incremented once per *successful* `robot_description` parse --
    /// exposed by `/joint_state_publisher/robot_get_visual_geometries` so
    /// a caller can tell when this node's own view has caught up to a
    /// newly selected robot, mirroring `joint_state_publisher.rs`'s own
    /// identically-named, identically-reasoned counter.
    generation: u64,

    simulation_method: SimulationMethod,
    last_seen_simulation_method_version: u64,
    last_simulation_method_poll: Instant,

    setpoint: HashMap<String, f64>,
    pid: Option<PidController>,
    /// Whether the PID servo is actively driving toward `setpoint` --
    /// see `/robot_sim/servo_toggle`'s own doc comment above. Off by
    /// default. Only ever consulted by the two PID-driven dynamics
    /// backends -- see `jsp_servo_enabled` below for the kinematics-only
    /// backend's own, entirely independent equivalent.
    servo_enabled: bool,
    /// Whether the kinematics-only backend's own servo is bypassed
    /// (`false`, the default -- instant/direct position control) or
    /// gradually converging at `jsp_kp` -- see
    /// `/joint_state_publisher/servo_toggle`'s own doc comment above.
    /// Entirely independent of `servo_enabled` above, the same way
    /// `joint_state_publisher.rs`'s own `servo_enabled` field always was
    /// independent of `arm_sim_node`-style PID enable/disable.
    jsp_servo_enabled: bool,
    /// The kinematics-only backend's own proportional gain -- see
    /// `/joint_state_publisher/set_kp`'s own doc comment above.
    jsp_kp: f64,
    /// Whether the dynamics catch-up loop (and therefore `sim_time`) is
    /// frozen -- see `/robot_sim/pause`'s own doc comment above. Not
    /// paused by default.
    paused: bool,

    /// The robot's own live orientation in the world, from `/global_pose`
    /// -- identity (no tilt) until the first message arrives, matching
    /// `robot_world_state_publisher`'s own default.
    global_pose_rotation: [f64; 4],

    gravity: f64,
    dt: f64,
    max_effort: f64,
    max_velocity: f64,
    sim_time: f64,
}

impl RobotSimNode {
    fn new(hub_endpoint: (String, u16), gravity: f64, dt: f64, max_effort: f64, max_velocity: f64) -> RobotSimNode {
        let node = Node::new("robot_sim_node", hub_endpoint);
        RobotSimNode {
            trajectory_subscriber: node.create_subscriber("/joint_trajectory", "trajectory_msgs/JointTrajectory").expect("failed to subscribe to /joint_trajectory"),
            global_pose_subscriber: node.create_subscriber("/global_pose", "geometry_msgs/Pose").expect("failed to subscribe to /global_pose"),
            joint_state_publisher: node.create_publisher("/joint_states", "sensor_msgs/JointState").expect("failed to register /joint_states publisher"),
            pubsubout_publisher: node.create_publisher("/pubsubout", "pubsub_msgs/Log").expect("failed to register /pubsubout publisher"),
            reset_service: node.create_service("/robot_sim/reset").expect("failed to register /robot_sim/reset service"),
            set_gains_service: node.create_service("/robot_sim/set_gains").expect("failed to register /robot_sim/set_gains service"),
            save_gains_service: node.create_service("/robot_sim/save_gains").expect("failed to register /robot_sim/save_gains service"),
            load_gains_service: node.create_service("/robot_sim/load_gains").expect("failed to register /robot_sim/load_gains service"),
            list_saved_gains_service: node.create_service("/robot_sim/list_saved_gains").expect("failed to register /robot_sim/list_saved_gains service"),
            estimate_gains_service: node.create_service("/robot_sim/estimate_gains").expect("failed to register /robot_sim/estimate_gains service"),
            servo_toggle_service: node.create_service("/robot_sim/servo_toggle").expect("failed to register /robot_sim/servo_toggle service"),
            pause_service: node.create_service("/robot_sim/pause").expect("failed to register /robot_sim/pause service"),
            jsp_robot_description_list_service: node
                .create_service("/joint_state_publisher/robot_description_list")
                .expect("failed to register /joint_state_publisher/robot_description_list service"),
            jsp_robot_description_select_service: node
                .create_service("/joint_state_publisher/robot_description_select")
                .expect("failed to register /joint_state_publisher/robot_description_select service"),
            jsp_robot_get_visual_geometries_service: node
                .create_service("/joint_state_publisher/robot_get_visual_geometries")
                .expect("failed to register /joint_state_publisher/robot_get_visual_geometries service"),
            jsp_servo_toggle_service: node
                .create_service("/joint_state_publisher/servo_toggle")
                .expect("failed to register /joint_state_publisher/servo_toggle service"),
            jsp_set_kp_service: node.create_service("/joint_state_publisher/set_kp").expect("failed to register /joint_state_publisher/set_kp service"),
            current_robot: None,
            backend: None,
            joint_limits: HashMap::new(),
            last_seen_robot_description_version: 0,
            last_param_poll: Instant::now() - PARAM_POLL_INTERVAL,
            generation: 0,
            simulation_method: DEFAULT_SIMULATION_METHOD,
            last_seen_simulation_method_version: 0,
            last_simulation_method_poll: Instant::now() - PARAM_POLL_INTERVAL,
            setpoint: HashMap::new(),
            pid: None,
            servo_enabled: false,
            jsp_servo_enabled: false,
            jsp_kp: DEFAULT_JSP_KP,
            paused: false,
            global_pose_rotation: [0.0, 0.0, 0.0, 1.0],
            gravity,
            dt,
            max_effort,
            max_velocity,
            sim_time: 0.0,
            node,
        }
    }

    fn log_error(&mut self, msg: &str) {
        eprintln!("[robot_sim_node] {msg}");
        self.pubsubout_publisher.publish(&rm::pubsubout_log("error", "robot_sim_node", msg));
    }

    /// The midpoint of each joint's own limits (`0.0` for `continuous`,
    /// which has none) -- this backend's "known, unsurprising rest pose,"
    /// shared by [`RobotSimNode::rebuild_backend`] and (indirectly, since
    /// it calls that) `/robot_sim/reset`.
    fn midpoint_q_map(&self) -> HashMap<String, f64> {
        self.joint_limits.iter().map(|(name, limit)| (name.clone(), limit.map_or(0.0, |(lower, upper)| (lower + upper) / 2.0))).collect()
    }

    /// (Re)builds `self.backend` for `self.current_robot` under whichever
    /// `self.simulation_method` is currently selected, seeded at the
    /// midpoint of every joint's own limits -- shared by a freshly
    /// (re)parsed `robot_description`, an explicit `/robot_sim/reset`,
    /// and a live `simulation_method` switch, all of which need the same
    /// "known, unsurprising rest pose" backend (re)construction.
    /// `self.backend` is `Some` for all three methods now, including
    /// [`SimulationMethod::KinematicsOnly`] -- see this file's own module
    /// doc comment's "Why kinematics-only lives here now" section.
    /// Deliberately does **not** touch `self.pid`'s own gains (only its
    /// accumulated integral, via `pid.reset()`) -- a robot's own DOF
    /// count never changes just because the *backend* representing it
    /// did, so a live `simulation_method` switch (unlike an actual new
    /// `robot_description`) never needs to resize, let alone
    /// re-default, the gains a user may have already tuned via
    /// `/robot_sim/set_gains`. Also deliberately does not touch
    /// `self.servo_enabled`/`self.jsp_servo_enabled`/`self.paused` --
    /// those are independent of which dynamics backend is loaded, the
    /// same way `arm_sim_node`'s own `/arm_sim/reset` never touches its
    /// own `enabled`/`paused` locals either.
    fn rebuild_backend(&mut self) {
        let Some(robot) = self.current_robot.clone() else { return };
        let q0 = self.midpoint_q_map();

        self.backend = match self.simulation_method {
            SimulationMethod::KinematicsOnly => Some({
                let mut names = Vec::new();
                let mut joints = Vec::new();
                for j in robot.joints.iter().filter(|j| j.joint_type != urdf::JointType::Fixed) {
                    names.push(j.name.clone());
                    joints.push(KinematicJoint { joint_type: j.joint_type, limit: j.limit });
                }
                let q = names.iter().map(|name| q0.get(name).copied().unwrap_or(0.0)).collect();
                let qdot = vec![0.0; names.len()];
                SimBackend::KinematicsOnly { names, joints, q, qdot }
            }),
            SimulationMethod::NewtonEuler => Some({
                let model = DynamicsModel::build(&robot, None);
                let q = model.dof_names().iter().map(|name| q0.get(name).copied().unwrap_or(0.0)).collect();
                let qdot = vec![0.0; model.dof_count()];
                SimBackend::NewtonEuler { model, q, qdot }
            }),
            SimulationMethod::Epbd => Some({
                let (model, bodies) = XpbdModel::build_at(&robot, None, &q0);
                SimBackend::Xpbd { model, bodies }
            }),
        };

        self.setpoint = match &self.backend {
            Some(backend) => backend.dof_names().iter().map(|name| (name.clone(), q0.get(name).copied().unwrap_or(0.0))).collect(),
            None => HashMap::new(),
        };
        if let Some(pid) = &mut self.pid {
            pid.reset();
        }
        self.sim_time = 0.0;
    }

    fn sync_robot_description(&mut self) {
        if self.backend.is_some() && self.last_param_poll.elapsed() < PARAM_POLL_INTERVAL {
            return;
        }
        self.last_param_poll = Instant::now();

        let Ok(response) = self.node.call_service("/param_server/get_param", &json!({ "name": "robot_description" }), PARAM_SERVICE_TIMEOUT) else { return };
        if response.get("result").and_then(Value::as_bool) != Some(true) {
            return;
        }
        let Some(new_version) = response["values"]["version"].as_u64() else { return };
        if new_version == self.last_seen_robot_description_version {
            return;
        }
        self.last_seen_robot_description_version = new_version;

        let Some(text) = response["values"]["value"].as_str() else {
            self.log_error(&format!("robot_description (version {new_version}) is not a string -- ignoring"));
            return;
        };
        match urdf::parse_robot_description(text) {
            Ok(robot) => {
                // mesh_search_dir is None: this node has no bookkeeping of
                // which robots/<dir> (if any) the current robot_description
                // text came from -- see `rigid_body_dynamics::
                // default_inertial_for_link`'s own doc comment for why
                // that's a documented, deliberate scope limit (mesh-based
                // inertial estimation only ever fires when a known
                // directory is threaded through), not an oversight. Shared
                // by both backends via `DynamicsModel::
                // inertial_estimate_sources`/`XpbdModel::
                // inertial_estimate_sources` (the latter just delegates to
                // the former -- identical estimation, identical logging,
                // regardless of which backend ends up using it).
                self.joint_limits = robot.joints.iter().filter(|j| j.joint_type != urdf::JointType::Fixed).map(|j| (j.name.clone(), j.limit)).collect();
                let n = self.joint_limits.len();
                self.pid = Some(PidController::new(vec![DEFAULT_KP; n], vec![DEFAULT_KI; n], vec![DEFAULT_KD; n], INTEGRAL_LIMIT));
                self.current_robot = Some(robot.clone());
                self.rebuild_backend();
                // See `generation`'s own doc comment: incremented here,
                // on every *successful* parse, distinct from
                // `last_seen_robot_description_version` above (which
                // advances on every version looked at, whether or not it
                // actually parsed) -- mirrors `joint_state_publisher.rs`'s
                // own identical counter exactly.
                self.generation += 1;
                println!("[robot_sim_node] parsed robot_description version {new_version} -> generation {}: {n} joint(s), simulation_method={:?}", self.generation, self.simulation_method);

                for (link_name, source) in DynamicsModel::inertial_estimate_sources(&robot, None) {
                    if source != pubsub_testbed::rigid_body_dynamics::InertialEstimateSource::FromUrdfInertial {
                        println!("[robot_sim_node] link {link_name:?} has no <inertial> -- estimated via {source:?}");
                    }
                }
            }
            Err(message) => self.log_error(&format!("robot_description version {new_version} failed to parse, keeping the previously-loaded robot (if any) untouched: {message}")),
        }
    }

    /// Polls for a live `simulation_method` change -- see this file's own
    /// module doc comment's "Communication" section. The parameter never
    /// having been set at all (`result: false` from `get_param`, the same
    /// "doesn't exist yet" signal `param_server_node`'s own tests
    /// exercise) is not an error: it just means "keep using whichever
    /// backend is already active," which is [`DEFAULT_SIMULATION_METHOD`]
    /// until something else ever sets this parameter.
    fn sync_simulation_method(&mut self) {
        if self.last_simulation_method_poll.elapsed() < PARAM_POLL_INTERVAL {
            return;
        }
        self.last_simulation_method_poll = Instant::now();

        let Ok(response) = self.node.call_service("/param_server/get_param", &json!({ "name": "simulation_method" }), PARAM_SERVICE_TIMEOUT) else { return };
        if response.get("result").and_then(Value::as_bool) != Some(true) {
            return;
        }
        let Some(new_version) = response["values"]["version"].as_u64() else { return };
        if new_version == self.last_seen_simulation_method_version {
            return;
        }
        self.last_seen_simulation_method_version = new_version;

        let Some(text) = response["values"]["value"].as_str() else {
            self.log_error(&format!("simulation_method (version {new_version}) is not a string -- ignoring"));
            return;
        };
        let Some(new_method) = SimulationMethod::parse(text) else {
            self.log_error(&format!("simulation_method {text:?} is not one of {SIMULATION_METHOD_NEWTON_EULER:?}/{SIMULATION_METHOD_XPBD:?} -- keeping {:?}", self.simulation_method));
            return;
        };
        if new_method == self.simulation_method {
            return;
        }
        self.simulation_method = new_method;
        println!("[robot_sim_node] simulation_method (version {new_version}) switched to {text:?}");
        self.rebuild_backend();
    }

// -- #region apply-trajectory ---------------------------------------------
    fn apply_trajectory(&mut self, msg: &Value) {
        let Some(joint_names) = msg.get("joint_names").and_then(Value::as_array) else { return };
        let Some(points) = msg.get("points").and_then(Value::as_array) else { return };
        let Some(last_point) = points.last() else { return };
        let Some(positions) = last_point.get("positions").and_then(Value::as_array) else { return };

        for (name_value, position_value) in joint_names.iter().zip(positions.iter()) {
            let (Some(name), Some(position)) = (name_value.as_str(), position_value.as_f64()) else { continue };
            if self.setpoint.contains_key(name) {
                self.setpoint.insert(name.to_string(), position);
            }
        }
    }
// -- #endregion apply-trajectory ------------------------------------------

    fn ingest_global_pose(&mut self, msg: &Value) {
        let (Some(qx), Some(qy), Some(qz), Some(qw)) = (
            msg["orientation"]["x"].as_f64(),
            msg["orientation"]["y"].as_f64(),
            msg["orientation"]["z"].as_f64(),
            msg["orientation"]["w"].as_f64(),
        ) else {
            return;
        };
        self.global_pose_rotation = [qx, qy, qz, qw];
    }

    /// Rotates the fixed world gravity vector into `root_link`'s own,
    /// possibly-tilted frame -- see this file's own module doc comment's
    /// "Gravity and a tilted base" section for the full derivation.
    fn gravity_in_root_frame(&self) -> [f64; 3] {
        let gravity_world = [0.0, 0.0, -self.gravity];
        rotate_vector(quat_conjugate(self.global_pose_rotation), gravity_world)
    }

    /// One control-plus-dynamics update, branching first on whether the
    /// kinematics-only backend is active (see
    /// [`RobotSimNode::advance_kinematics_only`] -- no PID, no dynamics,
    /// no effort at all) before falling through to the PID-plus-dynamics
    /// path original to this node: computes this step's PID effort once
    /// from the current state (read generically from whichever dynamics
    /// backend is active via [`SimBackend::q_values`]/
    /// [`SimBackend::qdot_values`]) -- or, if `self.servo_enabled` is
    /// `false`, skips the PID entirely and commands zero effort, so the
    /// robot moves under the active dynamics backend alone (gravity plus
    /// whatever external forces it models) -- then advances that backend
    /// by exactly one step of length `self.dt` with that effort held
    /// fixed -- see this file's own module doc comment for why,
    /// mirroring `arm_sim_node.rs`'s own "one node, not two" architecture.
    fn advance(&mut self) {
        if self.simulation_method == SimulationMethod::KinematicsOnly {
            self.advance_kinematics_only();
            self.sim_time += self.dt;
            return;
        }

        let Some(backend) = self.backend.as_ref() else { return };
        let n = backend.dof_count();
        if n == 0 {
            return;
        }
        let dof_names = backend.dof_names().to_vec();
        let q_now = backend.q_values();
        let qdot_now = backend.qdot_values();

        let mut effort = if self.servo_enabled {
            let setpoint_pos: Vec<f64> = dof_names.iter().map(|name| self.setpoint.get(name).copied().unwrap_or(0.0)).collect();
            let setpoint_vel = vec![0.0; n];
            self.pid.as_mut().expect("pid is always Some whenever backend is Some").update(&setpoint_pos, &setpoint_vel, &q_now, &qdot_now, self.dt)
        } else {
            vec![0.0; n]
        };
        saturate(&mut effort, self.max_effort);

        let gravity_root = self.gravity_in_root_frame();
        self.backend
            .as_mut()
            .expect("checked Some above")
            .step(&effort, gravity_root, self.dt, self.max_velocity, &self.joint_limits);

        self.sim_time += self.dt;
    }

    /// The kinematics-only backend's own control law -- an independent
    /// reimplementation of `joint_state_publisher.rs`'s own `advance`
    /// (see this file's module doc comment's "Why kinematics-only lives
    /// here now" section for why it moved). No PID, no dynamics model, no
    /// motor effort: this backend has no plant to fight, so a single
    /// proportional gain (`self.jsp_kp`) drives `q` smoothly toward
    /// `self.setpoint` (`qdot = jsp_kp * error`, `q += qdot * self.dt`,
    /// using [`shortest_angle_difference`] for a revolute/continuous
    /// joint's wraparound-aware error and plain subtraction for a
    /// prismatic one) -- or, while `self.jsp_servo_enabled` is `false`
    /// (the default), snaps `q` to `setpoint` directly every tick with
    /// `qdot` reported as zero instead (see
    /// `/joint_state_publisher/servo_toggle`'s own doc comment). Either
    /// way, every joint's own limit clamp still applies exactly as it
    /// does under the two dynamics backends.
    fn advance_kinematics_only(&mut self) {
        let Some(SimBackend::KinematicsOnly { names, joints, q, qdot }) = self.backend.as_mut() else { return };
        for i in 0..names.len() {
            let setpoint_i = self.setpoint.get(&names[i]).copied().unwrap_or(0.0);
            if self.jsp_servo_enabled {
                let error = match joints[i].joint_type {
                    urdf::JointType::Prismatic => setpoint_i - q[i],
                    _ => shortest_angle_difference(setpoint_i, q[i]),
                };
                qdot[i] = self.jsp_kp * error;
                q[i] += qdot[i] * self.dt;
            } else {
                q[i] = setpoint_i;
                qdot[i] = 0.0;
            }
            if let Some((lower, upper)) = joints[i].limit {
                q[i] = q[i].clamp(lower, upper);
            }
        }
    }

    fn handle_reset(&mut self, pending: PendingRequest) {
        self.rebuild_backend();
        pending.respond(&json!({ "values": {}, "result": true, "status": "reset to the midpoint of every joint's limits" }));
    }

    fn gains_values(&self) -> Value {
        let Some((backend, pid)) = self.backend.as_ref().zip(self.pid.as_ref()) else { return json!({}) };
        let mut gains = serde_json::Map::new();
        for (i, name) in backend.dof_names().iter().enumerate() {
            gains.insert(name.clone(), json!({ "kp": pid.kp[i], "ki": pid.ki[i], "kd": pid.kd[i] }));
        }
        Value::Object(gains)
    }

    /// See `/robot_sim/estimate_gains`'s own doc comment above for the
    /// full derivation and rationale. Pure (`&self`, never mutates
    /// `self.pid`) -- builds a throwaway [`DynamicsModel`] straight from
    /// `self.current_robot` rather than reading `self.backend` at all, so
    /// this works identically regardless of which `simulation_method` is
    /// currently active (including `"kinematics-only-JSP"`, which has no
    /// dynamics model of its own).
    fn estimate_gains(&self, request: &Value) -> Result<Value, String> {
        let robot = self.current_robot.as_ref().ok_or("no robot loaded yet")?;

        let omega_n = request.get("omega_n").and_then(Value::as_f64).unwrap_or(DEFAULT_ESTIMATE_OMEGA_N);
        if omega_n <= 0.0 {
            return Err("\"omega_n\" must be positive".to_string());
        }
        let zeta = request.get("zeta").and_then(Value::as_f64).unwrap_or(DEFAULT_ESTIMATE_ZETA);
        if zeta < 0.0 {
            return Err("\"zeta\" must be zero or positive".to_string());
        }
        let ti = match request.get("ti") {
            None => None,
            Some(value) => {
                let ti = value.as_f64().ok_or("\"ti\" must be a number")?;
                if ti <= 0.0 {
                    return Err("\"ti\" must be positive (omit it entirely for ki = 0)".to_string());
                }
                Some(ti)
            }
        };

        let model = DynamicsModel::build(robot, None);
        let q0 = self.midpoint_q_map();
        let q: Vec<f64> = model.dof_names().iter().map(|name| q0.get(name).copied().unwrap_or(0.0)).collect();
        let mass_matrix = model.mass_matrix(&q);

        let mut gains = serde_json::Map::new();
        for (i, name) in model.dof_names().iter().enumerate() {
            let effective_inertia = mass_matrix[i][i];
            let kp = effective_inertia * omega_n * omega_n;
            let kd = 2.0 * zeta * effective_inertia * omega_n;
            let ki = ti.map_or(0.0, |ti| kp / ti);
            gains.insert(name.clone(), json!({ "kp": kp, "ki": ki, "kd": kd }));
        }
        Ok(Value::Object(gains))
    }

    fn handle_estimate_gains(&self, pending: PendingRequest) {
        let response = match self.estimate_gains(&pending.request) {
            Ok(gains) => json!({ "values": { "gains": gains }, "result": true, "status": "" }),
            Err(message) => json!({ "values": {}, "result": false, "status": message }),
        };
        pending.respond(&response);
    }

    /// Validates `request`'s `"gains"` map against the currently-loaded
    /// robot's own joint names before applying anything -- see this
    /// file's own module doc comment for why this is all-or-nothing
    /// rather than partial application.
    fn apply_set_gains(&mut self, request: &Value) -> Result<(), String> {
        let Some(gains) = request.get("gains").and_then(Value::as_object) else { return Ok(()) }; // no "gains" field -- query-only
        let Some(backend) = &self.backend else { return Err("no robot loaded yet".to_string()) };

        let mut updates: Vec<(usize, Option<f64>, Option<f64>, Option<f64>)> = Vec::new();
        for (joint_name, entry) in gains {
            let Some(index) = backend.dof_names().iter().position(|n| n == joint_name) else {
                return Err(format!("unknown joint {joint_name:?} (not one of this robot's own non-fixed joints)"));
            };
            let kp = entry.get("kp").and_then(Value::as_f64);
            let ki = entry.get("ki").and_then(Value::as_f64);
            let kd = entry.get("kd").and_then(Value::as_f64);
            for (label, value) in [("kp", kp), ("ki", ki), ("kd", kd)] {
                if let Some(v) = value {
                    if v < 0.0 {
                        return Err(format!("{joint_name:?}'s {label} must be zero or positive"));
                    }
                }
            }
            updates.push((index, kp, ki, kd));
        }

        let pid = self.pid.as_mut().expect("pid is always Some whenever backend is Some");
        for (index, kp, ki, kd) in updates {
            if let Some(v) = kp {
                pid.kp[index] = v;
            }
            if let Some(v) = ki {
                pid.ki[index] = v;
            }
            if let Some(v) = kd {
                pid.kd[index] = v;
            }
        }
        Ok(())
    }

    fn handle_set_gains(&mut self, pending: PendingRequest) {
        let request = pending.request.clone();
        let response = match self.apply_set_gains(&request) {
            Ok(()) => json!({ "values": { "gains": self.gains_values() }, "result": true, "status": "" }),
            Err(message) => json!({ "values": { "gains": self.gains_values() }, "result": false, "status": message }),
        };
        pending.respond(&response);
    }

    /// `request`'s own `"identifier"` if present, otherwise the
    /// currently-loaded robot's own `<robot name="...">` -- shared by
    /// `/robot_sim/save_gains`/`/robot_sim/load_gains` so "defaults to
    /// the loaded robot's own name" (see this file's own module doc
    /// comment) lives in exactly one place. `None` only when `request`
    /// omits `"identifier"` *and* no robot has ever loaded (nothing to
    /// default to).
    fn resolve_gains_identifier(&self, request: &Value) -> Option<String> {
        if let Some(identifier) = request.get("identifier").and_then(Value::as_str) {
            return Some(identifier.to_string());
        }
        self.current_robot.as_ref().map(|robot| robot.name.clone())
    }

    /// See `/robot_sim/save_gains`'s own doc comment above for the full
    /// contract.
    fn handle_save_gains(&self, pending: PendingRequest) {
        let response = (|| -> Result<Value, String> {
            let identifier = self.resolve_gains_identifier(&pending.request).ok_or("no robot loaded yet, and no explicit \"identifier\" given to save under")?;
            validate_gains_identifier(&identifier)?;
            let robot_name = self.current_robot.as_ref().ok_or("no robot loaded yet")?.name.clone();
            std::fs::create_dir_all(GAINS_DIR).map_err(|e| format!("could not create {GAINS_DIR}: {e}"))?;
            let path = gains_file_path(&identifier);
            let contents = json!({ "robot_name": robot_name, "gains": self.gains_values() });
            let pretty = serde_json::to_string_pretty(&contents).expect("a Value built from already-known fields should always re-serialize");
            std::fs::write(&path, pretty).map_err(|e| format!("could not write {}: {e}", path.display()))?;
            Ok(json!({ "path": path.display().to_string() }))
        })();
        let response = match response {
            Ok(values) => json!({ "values": values, "result": true, "status": "" }),
            Err(message) => json!({ "values": {}, "result": false, "status": message }),
        };
        pending.respond(&response);
    }

    /// See `/robot_sim/load_gains`'s own doc comment above for the full
    /// contract.
    fn handle_load_gains(&mut self, pending: PendingRequest) {
        let response = (|| -> Result<Value, String> {
            let identifier = self.resolve_gains_identifier(&pending.request).ok_or("no robot loaded yet, and no explicit \"identifier\" given to load")?;
            validate_gains_identifier(&identifier)?;
            let path = gains_file_path(&identifier);
            let text = std::fs::read_to_string(&path).map_err(|e| format!("could not read {}: {e}", path.display()))?;
            let parsed: Value = serde_json::from_str(&text).map_err(|e| format!("{} is not valid JSON: {e}", path.display()))?;
            let gains = parsed.get("gains").cloned().ok_or_else(|| format!("{} is missing \"gains\"", path.display()))?;
            let saved_robot_name = parsed.get("robot_name").and_then(Value::as_str);
            let robot_name_matched = saved_robot_name.is_some() && saved_robot_name == self.current_robot.as_ref().map(|r| r.name.as_str());
            self.apply_set_gains(&json!({ "gains": gains }))?;
            Ok(json!({ "gains": self.gains_values(), "robot_name_matched": robot_name_matched }))
        })();
        let response = match response {
            Ok(values) => json!({ "values": values, "result": true, "status": "" }),
            Err(message) => json!({ "values": { "gains": self.gains_values() }, "result": false, "status": message }),
        };
        pending.respond(&response);
    }

    /// Every identifier `gains/` currently offers -- every `<name>.json`
    /// direct child of [`GAINS_DIR`], sorted, mirroring `fsm_node.rs`'s
    /// own `list_saved` exactly.
    fn list_saved_gains() -> Vec<String> {
        let mut identifiers = Vec::new();
        if let Ok(entries) = std::fs::read_dir(GAINS_DIR) {
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

    fn handle_list_saved_gains_request(&self, pending: PendingRequest) {
        pending.respond(&json!({ "values": { "identifiers": Self::list_saved_gains() }, "result": true, "status": "" }));
    }

    /// See `/robot_sim/servo_toggle`'s own doc comment above for the
    /// full contract.
    fn handle_servo_toggle(&mut self, pending: PendingRequest) {
        if let Some(data) = pending.request.get("data").and_then(Value::as_bool) {
            if data && !self.servo_enabled {
                // Re-enabling: clear any integral accumulated while
                // disabled -- nothing meaningful could have accumulated
                // (advance() never calls pid.update() while disabled),
                // but this defensively matches arm_sim_node's own
                // /pid_controller/enable handler anyway.
                if let Some(pid) = &mut self.pid {
                    pid.reset();
                }
            }
            self.servo_enabled = data;
        }
        let status = if self.servo_enabled { "enabled" } else { "disabled" };
        pending.respond(&json!({ "values": { "enabled": self.servo_enabled }, "result": true, "status": status }));
    }

    /// See `/robot_sim/pause`'s own doc comment above for the full
    /// contract.
    fn handle_pause(&mut self, pending: PendingRequest) {
        self.paused = pending.request.get("data").and_then(Value::as_bool).unwrap_or(true);
        let status = if self.paused { "paused" } else { "running" };
        pending.respond(&json!({ "values": {}, "result": true, "status": status }));
    }

    /// See `/joint_state_publisher/robot_description_list`'s own doc
    /// comment above -- an independent reimplementation of
    /// `joint_state_publisher.rs`'s own handler of the same name.
    fn handle_jsp_robot_description_list_request(&self, pending: PendingRequest) {
        let robots = list_robot_descriptions();
        pending.respond(&json!({ "values": { "robots": robots }, "result": true, "status": "" }));
    }

    /// Reads `request`'s named description file and sets it as
    /// `"robot_description"` itself -- an independent reimplementation of
    /// `joint_state_publisher.rs`'s own `handle_robot_description_select`.
    fn handle_jsp_robot_description_select(&self, request: &Value) -> Result<(), String> {
        let name = request.get("name").and_then(Value::as_str).ok_or("request must include a string \"name\"")?;
        let path = find_description_file(name)?;
        let text = std::fs::read_to_string(&path).map_err(|e| format!("could not read {}: {e}", path.display()))?;
        let response = self
            .node
            .call_service("/param_server/set_param", &json!({ "name": "robot_description", "value": text }), PARAM_SERVICE_TIMEOUT)
            .map_err(|e| format!("could not reach param_server_node's /param_server/set_param: {e}"))?;
        if response.get("result").and_then(Value::as_bool) != Some(true) {
            return Err(format!("param_server_node rejected the set_param call: {}", response.get("status").and_then(Value::as_str).unwrap_or("")));
        }
        Ok(())
    }

    fn handle_jsp_robot_description_select_request(&self, pending: PendingRequest) {
        let response = match self.handle_jsp_robot_description_select(&pending.request) {
            Ok(()) => json!({ "values": {}, "result": true, "status": "" }),
            Err(message) => json!({ "values": {}, "result": false, "status": message }),
        };
        pending.respond(&response);
    }

    /// Builds the visual-relevant subset of this node's own
    /// independently-parsed `current_robot` -- an independent
    /// reimplementation of `joint_state_publisher.rs`'s own
    /// `handle_robot_get_visual_geometries_request`. `result: false`
    /// (empty `links`) if nothing has been parsed yet; `generation` is
    /// always included either way so a caller can distinguish "never
    /// loaded" from "loaded, no visuals" by watching it advance.
    fn handle_jsp_robot_get_visual_geometries_request(&self, pending: PendingRequest) {
        let response = match &self.current_robot {
            Some(robot) => {
                let links: Vec<Value> = robot.links.iter().map(urdf::link_to_jrdf).collect();
                json!({ "values": { "links": links, "generation": self.generation }, "result": true, "status": "" })
            }
            None => json!({ "values": { "links": [], "generation": self.generation }, "result": false, "status": "no robot loaded yet" }),
        };
        pending.respond(&response);
    }

    /// `{"data": <bool, optional>}` -> `{"values": {"enabled": bool},
    /// "result": true, "status": str}` -- controls *this node's own*
    /// kinematics-only backend specifically (`self.jsp_servo_enabled`),
    /// entirely independent of `/robot_sim/servo_toggle`'s own
    /// `self.servo_enabled` above (which only ever applies to the two
    /// PID-driven dynamics backends) -- see this file's module doc
    /// comment. Mirrors `joint_state_publisher.rs`'s own
    /// `/joint_state_publisher/servo_toggle` "field is optional; omit it
    /// to just query the current value" convention, since a front-end
    /// checkbox needs to learn the current state without changing it.
    fn handle_jsp_servo_toggle_request(&mut self, pending: PendingRequest) {
        if let Some(data) = pending.request.get("data").and_then(Value::as_bool) {
            self.jsp_servo_enabled = data;
        }
        let status = if self.jsp_servo_enabled { "enabled" } else { "disabled" };
        pending.respond(&json!({ "values": { "enabled": self.jsp_servo_enabled }, "result": true, "status": status }));
    }

    /// `{"kp": <f64, optional>}` -> `{"values": {"kp": f64}, "result":
    /// bool, "status": str}` -- the kinematics-only backend's own gain
    /// (`self.jsp_kp`), entirely independent of `/robot_sim/set_gains`'s
    /// per-joint PID gains above. Mirrors `joint_state_publisher.rs`'s
    /// own `/joint_state_publisher/set_kp`: a negative `kp` is rejected
    /// (`result: false`, current value unchanged), `kp = 0.0` is a
    /// legitimate "don't servo at all" gain.
    fn handle_jsp_set_kp_request(&mut self, pending: PendingRequest) {
        let response = match pending.request.get("kp").and_then(Value::as_f64) {
            Some(kp) if kp < 0.0 => json!({ "values": { "kp": self.jsp_kp }, "result": false, "status": "kp must be zero or positive" }),
            Some(kp) => {
                self.jsp_kp = kp;
                json!({ "values": { "kp": self.jsp_kp }, "result": true, "status": "" })
            }
            None => json!({ "values": { "kp": self.jsp_kp }, "result": true, "status": "" }),
        };
        pending.respond(&response);
    }

    fn joint_names(&self) -> Vec<String> {
        self.backend.as_ref().map(|b| b.dof_names().to_vec()).unwrap_or_default()
    }

    /// See [`MAX_CATCHUP_BUDGET`]'s own doc comment: this loop's own
    /// dynamics catch-up is deliberately bounded by wall-clock time per
    /// tick (not a fixed sub-step count -- that turned out not to be
    /// enough, see that constant's own doc comment) so that an expensive
    /// robot (many joints -- under the Newton-Euler backend, the
    /// per-step cost is dominated by an `O(n^3)` linear solve, see
    /// `rigid_body_dynamics.rs`) can never starve this same loop's
    /// `Service::spin_once` polling for `/robot_sim/reset`/
    /// `/robot_sim/set_gains` below -- confirmed via a real
    /// headless-browser run against PR2 (87 joints), where a client
    /// waiting on either service timed out outright while this node's own
    /// catch-up loop monopolized the thread trying to keep up with real
    /// time at the default `--dt`.
    fn spin(&mut self, publish_rate: f64) {
        let publish_period = Duration::from_secs_f64(1.0 / publish_rate);
        let mut last_publish = Instant::now() - publish_period;
        let mut last_step = Instant::now();

        loop {
            self.sync_robot_description();
            self.sync_simulation_method();

            for (_topic, msg) in self.trajectory_subscriber.spin_once() {
                self.apply_trajectory(&msg);
            }
            for (_topic, msg) in self.global_pose_subscriber.spin_once() {
                self.ingest_global_pose(&msg);
            }

            if let Some(pending) = self.reset_service.spin_once() {
                self.handle_reset(pending);
            }
            if let Some(pending) = self.set_gains_service.spin_once() {
                self.handle_set_gains(pending);
            }
            if let Some(pending) = self.save_gains_service.spin_once() {
                self.handle_save_gains(pending);
            }
            if let Some(pending) = self.load_gains_service.spin_once() {
                self.handle_load_gains(pending);
            }
            if let Some(pending) = self.list_saved_gains_service.spin_once() {
                self.handle_list_saved_gains_request(pending);
            }
            if let Some(pending) = self.estimate_gains_service.spin_once() {
                self.handle_estimate_gains(pending);
            }
            if let Some(pending) = self.servo_toggle_service.spin_once() {
                self.handle_servo_toggle(pending);
            }
            if let Some(pending) = self.pause_service.spin_once() {
                self.handle_pause(pending);
            }
            if let Some(pending) = self.jsp_robot_description_list_service.spin_once() {
                self.handle_jsp_robot_description_list_request(pending);
            }
            if let Some(pending) = self.jsp_robot_description_select_service.spin_once() {
                self.handle_jsp_robot_description_select_request(pending);
            }
            if let Some(pending) = self.jsp_robot_get_visual_geometries_service.spin_once() {
                self.handle_jsp_robot_get_visual_geometries_request(pending);
            }
            if let Some(pending) = self.jsp_servo_toggle_service.spin_once() {
                self.handle_jsp_servo_toggle_request(pending);
            }
            if let Some(pending) = self.jsp_set_kp_service.spin_once() {
                self.handle_jsp_set_kp_request(pending);
            }

            // Advance in fixed `self.dt`-sized sub-steps covering however
            // much wall-clock time actually elapsed since the last
            // iteration -- the same decoupled "physics runs at its own
            // fixed, numerically-stable rate; publishing is throttled
            // separately" pattern `arm_sim_node.rs` already uses, rather
            // than a single variable-size step per loop iteration (which
            // would make numerical stability depend on this process's own
            // scheduling jitter).
            //
            // Bounded by `MAX_CATCHUP_BUDGET` of *wall-clock* time actually
            // spent in this loop, checked after every single `advance()`
            // call -- see that constant's own doc comment for why a
            // wall-clock budget, not a count of sub-steps, is what
            // actually guarantees this node stays responsive regardless of
            // how expensive one robot's own dynamics evaluation is. A
            // robot too expensive to simulate at `--dt` in real time
            // simply falls behind (`self.sim_time` drifts behind wall
            // clock, visible as slow motion) rather than ever starving
            // this node's own `Service::spin_once` polling above.
            let elapsed = last_step.elapsed().as_secs_f64();
            last_step = Instant::now();
            let mut remaining = elapsed.min(0.25); // still cap a long stall (e.g. a debugger pause) from queuing up an enormous backlog of substeps to begin with
            let catchup_start = Instant::now();
            while remaining > 0.0 && !self.paused && self.backend.is_some() && catchup_start.elapsed() < MAX_CATCHUP_BUDGET {
                self.advance();
                remaining -= self.dt;
            }

            self.joint_state_publisher.spin_once();
            self.pubsubout_publisher.spin_once();
            // `self.backend` is `Some` for all three simulation methods
            // now (see this file's own module doc comment's "Why
            // kinematics-only lives here now" section) -- this node is
            // the sole /joint_states publisher regardless of which one
            // is active, gated only on "has a robot actually loaded yet"
            // rather than publishing an empty (zero-joint) message every
            // tick before that.
            if self.backend.is_some() && last_publish.elapsed() >= publish_period {
                let names = self.joint_names();
                let q = self.backend.as_ref().map(SimBackend::q_values).unwrap_or_default();
                let qdot = self.backend.as_ref().map(SimBackend::qdot_values).unwrap_or_default();
                let effort = vec![0.0; names.len()]; // not tracked across advance() calls at the publish boundary -- see note below
                // Stamped with this node's own accumulated simulation
                // clock (`self.sim_time`), not the wall clock -- the
                // `use_sim_time` convention this project already follows
                // for `arm_sim_node`'s own `/joint_states` (see
                // `ros_messages::header_at`'s doc comment). In this node's
                // case the two always advance 1:1 (there is no selectable
                // integrator/timestep-vs-real-time decoupling the way
                // `arm_sim_node` has), but stamping with the real
                // simulated clock still matters: a client computing an
                // elapsed-time-based dwell/timeout (the same role
                // `ik_action_node.rs`'s own dwell timer already plays)
                // should read it from here, not assume it is always wall
                // clock.
                let sim_sec = self.sim_time.floor().max(0.0) as u64;
                let sim_nanosec = ((self.sim_time - self.sim_time.floor()) * 1e9).round() as u32;
                let msg = rm::joint_state(&names, &q, &qdot, &effort, "", sim_sec, sim_nanosec);
                self.joint_state_publisher.publish(&msg);
                last_publish = Instant::now();
            }

            std::thread::sleep(Duration::from_millis(1));
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mut hub_host = "127.0.0.1".to_string();
    let mut hub_port: u16 = 9090;
    let mut gravity = DEFAULT_GRAVITY;
    let mut dt = DEFAULT_DT;
    let mut publish_rate = DEFAULT_PUBLISH_RATE;
    let mut max_effort = DEFAULT_MAX_EFFORT;
    let mut max_velocity = DEFAULT_MAX_VELOCITY;
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
            "--gravity" => {
                gravity = args[i + 1].parse().expect("--gravity must be a number");
                i += 2;
            }
            "--dt" => {
                dt = args[i + 1].parse().expect("--dt must be a positive number of seconds");
                i += 2;
            }
            "--publish-rate" => {
                publish_rate = args[i + 1].parse().expect("--publish-rate must be a positive number of Hz");
                i += 2;
            }
            "--max-effort" => {
                max_effort = args[i + 1].parse().expect("--max-effort must be a positive number");
                i += 2;
            }
            "--max-velocity" => {
                max_velocity = args[i + 1].parse().expect("--max-velocity must be a positive number");
                i += 2;
            }
            other => {
                eprintln!("unrecognized argument: {other}");
                i += 1;
            }
        }
    }
    assert!(gravity >= 0.0, "--gravity must be zero or positive");
    assert!(dt > 0.0, "--dt must be a positive number of seconds");
    assert!(publish_rate > 0.0, "--publish-rate must be a positive number of Hz");
    assert!(max_effort > 0.0, "--max-effort must be a positive number");
    assert!(max_velocity > 0.0, "--max-velocity must be a positive number");

    println!(
        "[robot_sim_node] rigid-body simulation (dt={dt}s, gravity={gravity} m/s^2), default simulation_method={SIMULATION_METHOD_KINEMATICS_ONLY:?}, polling param_server_node for \"robot_description\"/\"simulation_method\""
    );
    let mut node = RobotSimNode::new((hub_host, hub_port), gravity, dt, max_effort, max_velocity);
    node.spin(publish_rate);
}
