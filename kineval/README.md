# Full working KinEval reference viewer (planned)

The complete 3D KinEval viewer (robot loading, forward/inverse kinematics, RRT-Connect,
dynamics/control), with dependencies updated (e.g. a current three.js release in place of the
stencil's pinned r92/r73 builds) and every `STENCIL` section completed.

This depends on the `forward_kinematics/`, `inverse_kinematics/`, `motion_planning/`, and
`simulation/` decks being built first, since each completes the corresponding stencil module
this viewer needs. Not yet built — see `../README.md` for the overall plan and build order.

Will publish to `https://ocj-dev.github.io/kineval/kineval/`.

## Path planning reference, already live

The 2D path-planning reference implementation from `../pathfinding/reference/` is deployed here
as a standalone page, independent of the full 3D viewer above:
`https://ocj-dev.github.io/kineval/kineval/pathplanning/`. It's the same
`search_canvas.html`/`graph_search.js`/etc. shown in the `pathfinding/` deck's code-walkthrough
slides, just served at this URL too so it can be linked to directly (see that deck's "Try this
yourself" slide).
