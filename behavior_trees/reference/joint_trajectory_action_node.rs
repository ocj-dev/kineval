// ===========================================================================
// SNAPSHOT -- DO NOT EDIT THIS COPY
//
//   Source repository: /ai/development/autorob_agentic/pubsub_testbed_rs
//   Source path:       src/bin/joint_trajectory_action_node.rs
//   Upstream length:   710 lines
//   Commit:            d9e34cd16c47c790856b1ccea0d8575620c66f89
//   Snapshot date:     2026-11-23
//
// Why this copy exists: Slidev's `<<<` snippet importer cannot read
// outside its own workspace root, which behavior_trees/pnpm-workspace.yaml
// pins at behavior_trees/. The deck quotes Rust that lives in a DIFFERENT
// REPOSITORY entirely, so the file has to be carried here to be quotable
// at all.
//
// Quoted here for: the JointTrajectory action server: walks a trajectory one point at a time, gating each on arrival within epsilon.
//
// LINE NUMBERS IN THIS FILE ARE NOT UPSTREAM LINE NUMBERS -- this header
// shifts everything down. Where a slide cites a line, it cites the
// UPSTREAM one, and the mapping is below.
//
// #region markers, with their upstream line numbers:
//   arrival-check                upstream line 149
//   one-point-at-a-time          upstream line 198
//
// Edits belong upstream, followed by a re-snapshot. Diff this file
// against the commit above to check it has not drifted.
// ===========================================================================

//! joint_trajectory_action_node: executes a whole multi-point joint
//! trajectory, **one point at a time**, as a `SimpleActionServer`.
//!
//! This project's analogue of real ROS's
//! `control_msgs/FollowJointTrajectory`, shaped exactly like
//! `ik_action_node` (see that file's "Why an *action*, not a service"
//! section, which applies here verbatim: long-running, progress-
//! reporting, preemptable).
//!
//! ## One point at a time is required, not a style choice
//!
//! `robot_sim_node.rs`'s `apply_trajectory` (**lines 1084-1097**, at
//! commit **`4a1b4fe604b139003a7599c8712c73361bba93bd`**) subscribes to
//! `/joint_trajectory` and does this:
//!
//! ```text
//!     fn apply_trajectory(&mut self, msg: &Value) {
//!         let Some(joint_names) = msg.get("joint_names").and_then(Value::as_array) else { return };
//!         let Some(points) = msg.get("points").and_then(Value::as_array) else { return };
//!         let Some(last_point) = points.last() else { return };            // <-- line 1087
//!         ...
//! ```
//!
//! **`points.last()`.** Every intermediate point is discarded. Publishing
//! a planned trajectory as a single `/joint_trajectory` message therefore
//! does not execute it -- it **teleports** the robot's setpoint to the
//! final configuration, straight through whatever the planner spent a
//! thousand collision checks routing around. The simulator is not wrong
//! to do this; `/joint_trajectory` is a *setpoint* topic, and a setpoint
//! topic has no way to express "and please pass through these forty
//! configurations on the way".
//!
//! That gap is precisely what an action is for, and it is the most
//! concrete answer this project can give to "why is this an action rather
//! than a message?" So this node publishes **one point**, waits until the
//! robot has actually arrived within `epsilon`, publishes the next, and
//! reports progress and accepts preemption throughout.
//!
//! Do not "optimize" this into a single publish. It would look like it
//! worked -- the robot would end up at the right final configuration --
//! and it would silently delete the entire path.
//!
//! ## A trajectory may move the base as well as the joints
//!
//! `rrt_connect_node` plans over the robot's **full** configuration
//! space, so a trajectory's `joint_names` may contain `base/x`,
//! `base/y`, and `base/yaw` (`planning_scene::BASE_DOF_NAMES`) alongside
//! real URDF joints. Those three are not joints and must not be sent to
//! a joint controller, so this node splits every point in two:
//!
//!   - the real joints go out on `/joint_trajectory`, as always;
//!   - the base coordinates go out on **`/global_pose`**, as a
//!     `geometry_msgs/Pose` -- the same topic and convention
//!     `robot_fk.html`'s WASD/navigation controls publish and
//!     `robot_world_state_publisher`/`robot_sim_node` consume.
//!
//! Only `x`, `y`, and the yaw are overwritten. The node **subscribes**
//! to `/global_pose` and carries the current `z` through untouched,
//! because the planner never planned a height (the base is constrained
//! to the ground plane) and inventing one here would drop a legged
//! robot's body to the floor.
//!
//! ### Why the base is treated as having arrived immediately
//!
//! `/joint_trajectory` is a *setpoint*: something else has to servo the
//! robot there, which takes time, which is what `epsilon` and the
//! per-point timeout are for. `/global_pose` is not a setpoint -- it is
//! the pose itself, and nothing in this project simulates base dynamics.
//! Publishing it *is* the motion. So the base half of a point is
//! considered reached the moment it is published, and only the joint
//! half is gated on `/joint_states`.
//!
//! That asymmetry is worth stating rather than hiding, because it is
//! visible: a base-only trajectory advances one point per tick and looks
//! like driving, while a trajectory with joints in it advances only as
//! fast as the joints actually converge. A point whose joint set is
//! empty is therefore always "arrived".
//!
//! **Exactly one authority per topic** (see `CLAUDE.md`): while this
//! node is executing a goal it is the thing writing `/global_pose`, so a
//! front end driving the base by hand must stop while a goal is active.
//! `frontend/robot_rrt.html` disables its WASD/navigation controls for
//! exactly that window.
//!
//! ## Arrival convention
//!
//! `max(|target_i - actual_i|)` over joint-name-matched pairs, order
//! independent, no angle wrapping -- identical to `fsm_node.rs`'s and
//! `bt_node.rs`'s own arrival checks (see `fsm_node.rs`'s module doc
//! comment for the full justification: this is an arrival *check*, not a
//! control law). A joint set that does not match the trajectory's is
//! ignored rather than treated as arrival.
//!
//! The timeout is **per point**, not for the whole trajectory, and it is
//! wall-clock. A forty-point trajectory with a two-second budget per
//! point is a far more useful thing to express than one with an eighty-
//! second budget overall, because it fails at the point that actually
//! stuck rather than at the end.
//!
//! ## Communication
//!
//!   - Subscribes `/joint_states`, keeping only the latest sample.
//!   - Publishes `/joint_trajectory` -- one point per message, gated on
//!     arrival. See above.
//!   - `/joint_trajectory_action/send_goal` (request
//!     `{"trajectory": {"joint_names": [...], "points": [{"positions":
//!     [...]}, ...]}, "epsilon": f64, "timeout": f64}`; a flat
//!     `{"joint_names": ..., "points": ...}` is accepted too) ->
//!     `values.goal_id` on acceptance. A new goal **preempts** whatever
//!     was active, publishing the old one's result as `"preempted"`
//!     first -- the `SimpleActionServer` convention `ik_action_node`
//!     already follows here, matched rather than reinvented.
//!   - `/joint_trajectory_action/cancel_goal` (request `{}` or
//!     `{"goal_id": str}`) preempts the active goal. A supplied
//!     `goal_id` must match, so a straggling cancel cannot kill the goal
//!     that replaced the one it was aimed at.
//!   - Publishes `/joint_trajectory_action/feedback` each tick while
//!     active: `{"goal_id", "point_index", "points_total", "error",
//!     "elapsed"}`. `point_index` advances monotonically.
//!   - Publishes `/joint_trajectory_action/result` exactly once per
//!     goal: `{"goal_id", "outcome": "succeeded"|"preempted"|"aborted",
//!     "point_index", "points_total", "final_error"}`.
//!   - Publishes `/joint_trajectory_action/status` every tick:
//!     `{"goal_id", "state": "idle"|"active", "point_index",
//!     "points_total"}`.
//!
//! ## CLI
//!
//! `--host` / `--port`, `--epsilon` (default arrival tolerance),
//! `--timeout` (default per-point patience, in seconds).

use pubsub_testbed::p2p::node::{Node, PendingRequest, Publisher, Service, Subscriber};
use pubsub_testbed::planning_scene::BASE_DOF_NAMES;
use pubsub_testbed::ros_messages as rm;
use serde_json::{json, Value};
use std::time::{Duration, Instant};

/// Default arrival tolerance, in each joint's own units (radians for a
/// revolute joint, metres for a prismatic one -- the same mixed-unit
/// convention `fsm_node`/`bt_node` already use for `epsilon`).
const DEFAULT_EPSILON: f64 = 0.05;

/// Default patience **per point**, in wall-clock seconds. See the module
/// doc comment on why this is per point rather than per trajectory.
const DEFAULT_POINT_TIMEOUT: f64 = 5.0;

const TICK_SLEEP: Duration = Duration::from_millis(20);

// -- #region arrival-check ------------------------------------------------

/// The latest `/joint_states` sample.
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
        return None; // malformed upstream message
    }
    Some(JointSample { joint_names, positions })
}

/// `max(|target_i - actual_i|)` over joint-name-matched pairs, or `None`
/// if `sample` does not name exactly the same set of joints as the
/// target. Identical in convention to `bt_node.rs`'s own
/// `max_joint_error` -- duplicated here rather than shared through a
/// library module, matching this project's existing habit of each node
/// binary carrying its own small, independently-readable copy of logic
/// like this.
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

// -- #endregion arrival-check ---------------------------------------------

// -- #region one-point-at-a-time ------------------------------------------

/// Splits one trajectory point into the joint half and the base half.
/// Returns `(joint_names, joint_positions, base_x, base_y, base_yaw)`,
/// with each base component `None` when the trajectory does not carry
/// it -- a trajectory may plan the base, the joints, or both.
fn split_point(joint_names: &[String], positions: &[f64]) -> (Vec<String>, Vec<f64>, Option<f64>, Option<f64>, Option<f64>) {
    let mut names = Vec::new();
    let mut values = Vec::new();
    let (mut x, mut y, mut yaw) = (None, None, None);
    for (name, value) in joint_names.iter().zip(positions.iter()) {
        // Matched against BASE_DOF_NAMES by position rather than against
        // string literals repeated here, so the planner's notion of
        // which coordinates are the base and this node's cannot drift
        // apart -- there is one definition, in planning_scene.rs.
        match BASE_DOF_NAMES.iter().position(|base| base == name) {
            Some(0) => x = Some(*value),
            Some(1) => y = Some(*value),
            Some(2) => yaw = Some(*value),
            _ => {
                names.push(name.clone());
                values.push(*value);
            }
        }
    }
    (names, values, x, y, yaw)
}

/// The one trajectory this `SimpleActionServer` is currently executing.
struct ActiveGoal {
    id: String,
    joint_names: Vec<String>,
    /// One entry per trajectory point: that point's joint positions.
    points: Vec<Vec<f64>>,
    /// Which point is currently being commanded. Only ever increases --
    /// the monotonicity `/joint_trajectory_action/feedback` promises.
    point_index: usize,
    /// Whether `points[point_index]` has been published yet. The
    /// "publish once per point, not once per tick" discipline
    /// `arm_sim_node`/`fsm_node`/`bt_node` all already establish.
    published: bool,
    epsilon: f64,
    point_timeout: f64,
    started: Instant,
    point_started: Instant,
    last_error: Option<f64>,
}

impl ActiveGoal {
    fn current_target(&self) -> &[f64] {
        &self.points[self.point_index.min(self.points.len() - 1)]
    }
}

/// Parses a trajectory out of a `send_goal` request, accepting it either
/// nested under `"trajectory"` (what `/plan_motion` hands back verbatim)
/// or flat (what a front end building one by hand is likely to send).
fn parse_trajectory(request: &Value) -> Result<(Vec<String>, Vec<Vec<f64>>), String> {
    let trajectory = request.get("trajectory").unwrap_or(request);
    let names = trajectory
        .get("joint_names")
        .and_then(Value::as_array)
        .ok_or("request must include a trajectory with a \"joint_names\" array")?;
    let joint_names: Vec<String> = names.iter().filter_map(|v| v.as_str().map(str::to_string)).collect();
    if joint_names.len() != names.len() {
        return Err("every entry of \"joint_names\" must be a string".to_string());
    }
    if joint_names.is_empty() {
        return Err("a trajectory needs at least one joint".to_string());
    }

    let raw_points = trajectory.get("points").and_then(Value::as_array).ok_or("request must include a trajectory with a \"points\" array")?;
    if raw_points.is_empty() {
        return Err("a trajectory needs at least one point".to_string());
    }
    let mut points = Vec::with_capacity(raw_points.len());
    for (i, point) in raw_points.iter().enumerate() {
        let positions = point.get("positions").and_then(Value::as_array).ok_or_else(|| format!("point {i} has no \"positions\" array"))?;
        let values: Vec<f64> = positions.iter().filter_map(Value::as_f64).collect();
        if values.len() != positions.len() {
            return Err(format!("point {i} has a non-numeric position"));
        }
        if values.len() != joint_names.len() {
            return Err(format!("point {i} has {} positions but the trajectory names {} joints", values.len(), joint_names.len()));
        }
        points.push(values);
    }
    Ok((joint_names, points))
}

// -- #endregion one-point-at-a-time ---------------------------------------

struct JointTrajectoryActionNode {
    joint_states_subscriber: Subscriber,
    global_pose_subscriber: Subscriber,
    trajectory_publisher: Publisher,
    global_pose_publisher: Publisher,
    feedback_publisher: Publisher,
    result_publisher: Publisher,
    status_publisher: Publisher,
    send_goal_service: Service,
    cancel_service: Service,
    _node: Node,

    default_epsilon: f64,
    default_point_timeout: f64,
    next_goal_id: u64,
    active_goal: Option<ActiveGoal>,
    latest_joint_state: Option<JointSample>,
    /// The robot's last known world pose. Only its `z` is ever read --
    /// carried through untouched when this node rewrites `x`/`y`/`yaw`,
    /// since the planner never planned a height.
    latest_base_z: f64,
}

impl JointTrajectoryActionNode {
    fn new(hub_endpoint: (String, u16), default_epsilon: f64, default_point_timeout: f64) -> JointTrajectoryActionNode {
        let node = Node::new("joint_trajectory_action_node", hub_endpoint);
        JointTrajectoryActionNode {
            joint_states_subscriber: node.create_subscriber("/joint_states", "sensor_msgs/JointState").expect("failed to subscribe to /joint_states"),
            global_pose_subscriber: node.create_subscriber("/global_pose", "geometry_msgs/Pose").expect("failed to subscribe to /global_pose"),
            trajectory_publisher: node.create_publisher("/joint_trajectory", "trajectory_msgs/JointTrajectory").expect("failed to register /joint_trajectory publisher"),
            global_pose_publisher: node.create_publisher("/global_pose", "geometry_msgs/Pose").expect("failed to register /global_pose publisher"),
            feedback_publisher: node
                .create_publisher("/joint_trajectory_action/feedback", "joint_trajectory_action/Feedback")
                .expect("failed to register /joint_trajectory_action/feedback publisher"),
            result_publisher: node
                .create_publisher("/joint_trajectory_action/result", "joint_trajectory_action/Result")
                .expect("failed to register /joint_trajectory_action/result publisher"),
            status_publisher: node
                .create_publisher("/joint_trajectory_action/status", "joint_trajectory_action/Status")
                .expect("failed to register /joint_trajectory_action/status publisher"),
            send_goal_service: node.create_service("/joint_trajectory_action/send_goal").expect("failed to register /joint_trajectory_action/send_goal"),
            cancel_service: node.create_service("/joint_trajectory_action/cancel_goal").expect("failed to register /joint_trajectory_action/cancel_goal"),
            default_epsilon,
            default_point_timeout,
            next_goal_id: 0,
            active_goal: None,
            latest_joint_state: None,
            latest_base_z: 0.0,
            _node: node,
        }
    }

    fn handle_send_goal(&mut self, pending: PendingRequest) {
        let (joint_names, points) = match parse_trajectory(&pending.request) {
            Ok(parsed) => parsed,
            Err(message) => {
                pending.respond(&json!({ "values": {}, "result": false, "status": message }));
                return;
            }
        };
        let epsilon = pending.request.get("epsilon").and_then(Value::as_f64).unwrap_or(self.default_epsilon);
        let point_timeout = pending.request.get("timeout").and_then(Value::as_f64).unwrap_or(self.default_point_timeout);
        if !(epsilon >= 0.0) || !(point_timeout > 0.0) {
            pending.respond(&json!({ "values": {}, "result": false, "status": "epsilon must be non-negative and timeout must be positive" }));
            return;
        }

        // A new goal always preempts whatever was active -- the
        // SimpleActionServer convention ik_action_node already follows:
        // no queue, the newest goal wins, and the old one is *reported*
        // as preempted rather than silently dropped.
        if let Some(old) = self.active_goal.take() {
            self.publish_result(&old, "preempted");
        }

        let goal_id = format!("goal_{}", self.next_goal_id);
        self.next_goal_id += 1;
        let points_total = points.len();
        self.active_goal = Some(ActiveGoal {
            id: goal_id.clone(),
            joint_names,
            points,
            point_index: 0,
            published: false,
            epsilon,
            point_timeout,
            started: Instant::now(),
            point_started: Instant::now(),
            last_error: None,
        });
        println!("[joint_trajectory_action_node] accepted {goal_id}: {points_total} points, epsilon {epsilon}, {point_timeout}s per point");
        pending.respond(&json!({ "values": { "goal_id": goal_id, "points_total": points_total }, "result": true, "status": "" }));
    }

    fn handle_cancel(&mut self, pending: PendingRequest) {
        let requested_id = pending.request.get("goal_id").and_then(Value::as_str).map(str::to_string);
        let Some(goal) = &self.active_goal else {
            pending.respond(&json!({ "values": {}, "result": false, "status": "no active goal to cancel" }));
            return;
        };
        if let Some(id) = &requested_id {
            if id != &goal.id {
                pending.respond(&json!({ "values": {}, "result": false, "status": "goal_id does not match the currently active goal" }));
                return;
            }
        }
        let finished = self.active_goal.take().expect("matched Some above");
        let at_point = finished.point_index;
        println!("[joint_trajectory_action_node] {} preempted at point {at_point}", finished.id);
        self.publish_result(&finished, "preempted");
        // Nothing further is published to /joint_trajectory: execution
        // stops where it stopped, which is what preemption means. The
        // robot holds whatever setpoint it was last given rather than
        // being yanked anywhere new.
        pending.respond(&json!({ "values": { "point_index": at_point }, "result": true, "status": "goal preempted" }));
    }

    /// The heart of this node: publish the current point if it has not
    /// been published yet, then decide whether the robot has arrived.
    fn advance_active_goal(&mut self) {
        let Some(goal) = &mut self.active_goal else { return };

        if !goal.published {
            let target = goal.current_target().to_vec();
            let (joint_names, joint_values, base_x, base_y, base_yaw) = split_point(&goal.joint_names, &target);
            goal.published = true;
            goal.point_started = Instant::now();

            if !joint_names.is_empty() {
                let n = joint_values.len();
                let point = rm::joint_trajectory_point(&joint_values, &vec![0.0; n], &vec![0.0; n], 0, 0);
                // One point, as its own single-point message. See the
                // module doc comment: robot_sim_node keeps only
                // `points.last()`, so a message carrying one point is
                // the only kind whose whole content survives the trip.
                self.trajectory_publisher.publish(&rm::joint_trajectory(&joint_names, vec![point], ""));
            }

            // The base half, as a pose rather than a setpoint. `z` is
            // whatever /global_pose last reported -- see the module doc
            // comment on why this node must not invent a height.
            if base_x.is_some() || base_y.is_some() || base_yaw.is_some() {
                let yaw = base_yaw.unwrap_or(0.0);
                let pose = rm::pose_3d(
                    base_x.unwrap_or(0.0),
                    base_y.unwrap_or(0.0),
                    self.latest_base_z,
                    0.0,
                    0.0,
                    (yaw * 0.5).sin(),
                    (yaw * 0.5).cos(),
                );
                self.global_pose_publisher.publish(&pose);
            }
            return; // give the simulator a tick to move before judging it
        }

        let target = goal.current_target().to_vec();
        let (joint_names, joint_values, _, _, _) = split_point(&goal.joint_names, &target);
        // A point with no real joints in it is base-only, and the base
        // has already moved by being published -- so it has arrived.
        let error = if joint_names.is_empty() {
            Some(0.0)
        } else {
            self.latest_joint_state.as_ref().and_then(|sample| max_joint_error(&joint_names, &joint_values, sample))
        };
        goal.last_error = error;

        if let Some(error) = error {
            if error < goal.epsilon {
                goal.point_index += 1;
                if goal.point_index >= goal.points.len() {
                    let finished = self.active_goal.take().expect("still active");
                    println!("[joint_trajectory_action_node] {} succeeded after {} points", finished.id, finished.points.len());
                    self.publish_result(&finished, "succeeded");
                    return;
                }
                goal.published = false; // the next point gets published on the next tick
                self.publish_feedback();
                return;
            }
        }

        if goal.point_started.elapsed().as_secs_f64() >= goal.point_timeout {
            let finished = self.active_goal.take().expect("still active");
            eprintln!(
                "[joint_trajectory_action_node] {} aborted: point {} of {} not reached within {}s (last error {:?})",
                finished.id,
                finished.point_index,
                finished.points.len(),
                finished.point_timeout,
                finished.last_error
            );
            self.publish_result(&finished, "aborted");
            return;
        }

        self.publish_feedback();
    }

    fn publish_feedback(&mut self) {
        let Some(goal) = &self.active_goal else { return };
        let msg = json!({
            "goal_id": goal.id,
            "point_index": goal.point_index,
            "points_total": goal.points.len(),
            "error": goal.last_error,
            "elapsed": goal.started.elapsed().as_secs_f64(),
        });
        self.feedback_publisher.publish(&msg);
    }

    fn publish_result(&mut self, goal: &ActiveGoal, outcome: &str) {
        let msg = json!({
            "goal_id": goal.id,
            "outcome": outcome,
            "point_index": goal.point_index,
            "points_total": goal.points.len(),
            "final_error": goal.last_error,
        });
        self.result_publisher.publish(&msg);
    }

    fn publish_status(&mut self) {
        let msg = match &self.active_goal {
            Some(goal) => json!({
                "goal_id": goal.id,
                "state": "active",
                "point_index": goal.point_index,
                "points_total": goal.points.len(),
            }),
            None => json!({ "goal_id": Value::Null, "state": "idle", "point_index": 0, "points_total": 0 }),
        };
        self.status_publisher.publish(&msg);
    }

    fn spin(&mut self) {
        loop {
            for (_topic, msg) in self.joint_states_subscriber.spin_once() {
                if let Some(sample) = parse_joint_state(&msg) {
                    self.latest_joint_state = Some(sample);
                }
            }
            for (_topic, msg) in self.global_pose_subscriber.spin_once() {
                if let Some(z) = msg["position"]["z"].as_f64() {
                    self.latest_base_z = z;
                }
            }
            self.trajectory_publisher.spin_once();
            self.global_pose_publisher.spin_once();
            self.feedback_publisher.spin_once();
            self.result_publisher.spin_once();
            self.status_publisher.spin_once();

            if let Some(pending) = self.send_goal_service.spin_once() {
                self.handle_send_goal(pending);
            }
            if let Some(pending) = self.cancel_service.spin_once() {
                self.handle_cancel(pending);
            }

            self.advance_active_goal();
            self.publish_status();

            std::thread::sleep(TICK_SLEEP);
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mut hub_host = "127.0.0.1".to_string();
    let mut hub_port: u16 = 9090;
    let mut epsilon = DEFAULT_EPSILON;
    let mut timeout = DEFAULT_POINT_TIMEOUT;
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
            "--epsilon" => {
                epsilon = args[i + 1].parse().expect("--epsilon must be a non-negative number");
                i += 2;
            }
            "--timeout" => {
                timeout = args[i + 1].parse().expect("--timeout must be a positive number of seconds");
                i += 2;
            }
            other => {
                eprintln!("unrecognized argument: {other}");
                i += 1;
            }
        }
    }
    assert!(epsilon >= 0.0, "--epsilon must be non-negative");
    assert!(timeout > 0.0, "--timeout must be a positive number of seconds");

    println!(
        "[joint_trajectory_action_node] offering /joint_trajectory_action/send_goal + cancel_goal; walking each trajectory one point at a time \
         (epsilon {epsilon}, {timeout}s per point). One point per /joint_trajectory message, deliberately -- robot_sim_node keeps only points.last()."
    );
    let mut node = JointTrajectoryActionNode::new((hub_host, hub_port), epsilon, timeout);
    node.spin();
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(names: &[&str], positions: &[f64]) -> JointSample {
        JointSample { joint_names: names.iter().map(|s| s.to_string()).collect(), positions: positions.to_vec() }
    }

    #[test]
    fn arrival_error_is_the_largest_per_joint_difference_and_is_order_independent() {
        let target_names = vec!["a".to_string(), "b".to_string()];
        let target = [1.0, 2.0];
        assert_eq!(max_joint_error(&target_names, &target, &sample(&["a", "b"], &[1.0, 2.0])), Some(0.0));

        let in_order = max_joint_error(&target_names, &target, &sample(&["a", "b"], &[0.9, 2.3])).unwrap();
        assert!((in_order - 0.3).abs() < 1e-12, "the largest of 0.1 and 0.3, got {in_order}");

        // The same sample with its joints listed in the other order must
        // give the *exactly* same answer -- /joint_states makes no
        // ordering promise, so a node that read positions positionally
        // instead of by name would pass the first assertion and fail
        // this one.
        let reordered = max_joint_error(&target_names, &target, &sample(&["b", "a"], &[2.3, 0.9])).unwrap();
        assert_eq!(in_order, reordered);
    }

    #[test]
    fn a_mismatched_joint_set_is_ignored_rather_than_counted_as_arrival() {
        let target_names = vec!["a".to_string(), "b".to_string()];
        assert_eq!(max_joint_error(&target_names, &[1.0, 2.0], &sample(&["a", "c"], &[1.0, 2.0])), None);
        assert_eq!(max_joint_error(&target_names, &[1.0, 2.0], &sample(&["a"], &[1.0])), None);
    }

    #[test]
    fn a_trajectory_parses_nested_or_flat() {
        let nested = json!({ "trajectory": { "joint_names": ["a"], "points": [{ "positions": [0.5] }] } });
        let flat = json!({ "joint_names": ["a"], "points": [{ "positions": [0.5] }] });
        assert_eq!(parse_trajectory(&nested).unwrap(), parse_trajectory(&flat).unwrap());
        let (names, points) = parse_trajectory(&flat).unwrap();
        assert_eq!(names, vec!["a".to_string()]);
        assert_eq!(points, vec![vec![0.5]]);
    }

    #[test]
    fn a_malformed_trajectory_is_rejected_with_a_reason_rather_than_truncated() {
        // Each of these is a shape a caller plausibly sends by mistake,
        // and each one would do something quietly wrong if accepted:
        // a width mismatch would command the wrong joints, and an empty
        // point list would succeed instantly having moved nothing.
        assert!(parse_trajectory(&json!({ "joint_names": ["a", "b"], "points": [{ "positions": [0.5] }] }))
            .unwrap_err()
            .contains("names 2 joints"));
        assert!(parse_trajectory(&json!({ "joint_names": ["a"], "points": [] })).unwrap_err().contains("at least one point"));
        assert!(parse_trajectory(&json!({ "joint_names": [], "points": [{ "positions": [] }] })).unwrap_err().contains("at least one joint"));
        assert!(parse_trajectory(&json!({ "joint_names": ["a"] })).unwrap_err().contains("points"));
        assert!(parse_trajectory(&json!({ "joint_names": ["a"], "points": [{ "positions": ["x"] }] })).unwrap_err().contains("non-numeric"));
    }

    #[test]
    fn a_point_splits_into_its_joint_half_and_its_base_half() {
        let names: Vec<String> = ["base/x", "shoulder", "base/yaw", "elbow", "base/y"].iter().map(|s| s.to_string()).collect();
        let (joint_names, joint_values, x, y, yaw) = split_point(&names, &[1.0, 0.5, 0.25, -0.5, 2.0]);

        assert_eq!(joint_names, vec!["shoulder".to_string(), "elbow".to_string()], "base coordinates must never reach a joint controller");
        assert_eq!(joint_values, vec![0.5, -0.5], "and the joint values must stay matched to their own names");
        assert_eq!((x, y, yaw), (Some(1.0), Some(2.0), Some(0.25)), "regardless of the order they appeared in");
    }

    #[test]
    fn a_joints_only_trajectory_reports_no_base_and_a_base_only_one_reports_no_joints() {
        let joints: Vec<String> = ["a", "b"].iter().map(|s| s.to_string()).collect();
        let (names, values, x, y, yaw) = split_point(&joints, &[0.1, 0.2]);
        assert_eq!(names.len(), 2);
        assert_eq!(values, vec![0.1, 0.2]);
        assert_eq!((x, y, yaw), (None, None, None), "a fixed-base plan must not publish a pose");

        let base: Vec<String> = BASE_DOF_NAMES.iter().map(|s| s.to_string()).collect();
        let (names, values, x, y, yaw) = split_point(&base, &[3.0, 4.0, 1.57]);
        assert!(names.is_empty() && values.is_empty(), "a base-only plan must not publish an empty joint trajectory");
        assert_eq!((x, y, yaw), (Some(3.0), Some(4.0), Some(1.57)));
    }

    #[test]
    fn a_joint_merely_named_like_a_base_coordinate_is_still_a_joint() {
        // Guards the obvious over-eager match: only the three exact
        // names are base coordinates, not anything starting with "base".
        let names: Vec<String> = ["base_link_joint", "base/xx", "basex"].iter().map(|s| s.to_string()).collect();
        let (joint_names, _, x, y, yaw) = split_point(&names, &[0.1, 0.2, 0.3]);
        assert_eq!(joint_names.len(), 3, "none of these are base coordinates: {joint_names:?}");
        assert_eq!((x, y, yaw), (None, None, None));
    }

    #[test]
    fn the_current_target_tracks_the_point_index() {
        let goal = ActiveGoal {
            id: "g".to_string(),
            joint_names: vec!["a".to_string()],
            points: vec![vec![0.0], vec![1.0], vec![2.0]],
            point_index: 1,
            published: false,
            epsilon: 0.01,
            point_timeout: 1.0,
            started: Instant::now(),
            point_started: Instant::now(),
            last_error: None,
        };
        assert_eq!(goal.current_target(), &[1.0]);
    }
}
