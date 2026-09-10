# KinEval Lab-Session Slides

Live site: https://ocj-dev.github.io/kineval/

This repository holds a **functioning reference implementation** of the JavaScript/HTML5
[KinEval](https://github.com/autorob/kineval-stencil) code stencil, plus a set of
[Slidev](https://sli.dev) presentations that walk through each concept step by step for the
AutoRob (autorob.org) course's laboratory sections. It's the published counterpart to the
`kineval/` subtree of the (private) `autorob_agentic` course-infrastructure repository, where
this content is developed.

These decks complement the main course projects (implemented in Rust as the pubsub-based
`autorob_agentic` rewrite) by showing a complete, correct, narrated version of the same
underlying ideas — A-star search, dynamical simulation, forward/inverse kinematics, and
RRT-Connect motion planning — originally taught through KinEval before that pubsub rewrite.

Each topic below is published at `https://ocj-dev.github.io/kineval/<topic>/`, and the completed
reference code lives alongside its deck at `<topic>/reference/`.

| Topic | Directory | Status |
|---|---|---|
| A-star path planning | [`pathfinding/`](pathfinding/) | Built |
| Pendularm dynamical simulation | [`simulation/`](simulation/) | Planned |
| Forward kinematics & quaternions | [`forward_kinematics/`](forward_kinematics/) | Planned |
| Inverse kinematics & optimization | [`inverse_kinematics/`](inverse_kinematics/) | Planned |
| RRT-Connect motion planning | [`motion_planning/`](motion_planning/) | Planned |
| Full working KinEval viewer | [`kineval/`](kineval/) | Planned (depends on the topics above) |

## License

The reference implementation in this tree is derived from `autorob/kineval-stencil`, copyright
Odest Chadwicke Jenkins at the University of Michigan, licensed under the Michigan Honor License
(see `LICENSE`). This code is an instructor-authored reference implementation, not a student
submission, so the honor-pledge clause in that license does not apply here.
