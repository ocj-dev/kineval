// One shared master pseudocode listing, reused with different active-line
// highlighting across every panel in this deck -- exactly the pattern
// cloth_simulation's lib/cloth/pseudocode.ts establishes. Named line
// constants (rather than bare indices) keep each panel's active-line wiring
// readable and immune to renumbering.

export const MASTER_PSEUDOCODE: string[] = [
  /* 0 */ 'initialize pendulum state (angle, angle_dot), gravity, integrator, PID gains',
  /* 1 */ 'every frame:',
  /* 2 */ '    if servo is active:',
  /* 3 */ '        error = desired - angle',
  /* 4 */ '        control = PID(error, accumulated_error, dt)   -- kp*error + ki*sum(error*dt) + kd*d(error)/dt',
  /* 5 */ '    else:',
  /* 6 */ '        control = 0',
  /* 7 */ '    angle_dot_dot = equations_of_motion(angle, angle_dot, control, gravity)',
  /* 8 */ '    (angle, angle_dot) = integrate(angle, angle_dot, angle_dot_dot, dt)   -- Euler | Verlet | Velocity Verlet | RK4',
  /* 9 */ '    t = t + dt',
  /* 10 */ '    render pendulum at new angle',
]

export const LINE_INIT = 0
export const LINE_FRAME = 1
export const LINE_SERVO_CHECK = 2
export const LINE_ERROR = 3
export const LINE_PID = 4
export const LINE_ELSE = 5
export const LINE_ZERO_CONTROL = 6
export const LINE_ACCEL = 7
export const LINE_INTEGRATE = 8
export const LINE_ADVANCE_TIME = 9
export const LINE_RENDER = 10
