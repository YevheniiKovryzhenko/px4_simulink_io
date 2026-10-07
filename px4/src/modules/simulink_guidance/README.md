# Simulink guidance

`simulink_guidance` generates trajectory setpoints for PX4 controllers and
Simulink applications. It evaluates polynomial trajectories stored in `.traj`
files or receives trajectory samples from a companion computer. Outputs include
position, velocity, acceleration, jerk, snap, and yaw, depending on the selected
topic.

The module handles trajectory preparation, home capture, execution, and status
reporting. The integrating application manages arming, flight-mode changes,
offboard-control messages, takeoff, landing, and controller handoff. Starting
the module or a trajectory does not itself arm the vehicle or select offboard
mode.

## Configuration

The module must be included in the PX4 build through
`CONFIG_MODULES_SIMULINK_GUIDANCE`. Its parameters belong to the **SIM Guidance**
group.

| Parameter | Default | Values and behavior |
| --- | --- | --- |
| `SMG_EN` | `0` | `0`: disabled; `1`: local trajectory evaluation; `2`: companion guidance. |
| `SMG_IN_TYPE` | `0` | `0`: vehicle state from `vehicle_local_position`; `1`: state from the `simulink_inbound` debug-array topic. |
| `SMG_OUT_TYPE` | `1` | Output bitmask: `1` = `trajectory_setpoint`, `2` = `sim_guidance_trajectory`, `4` = `simulink_guidance` debug array. Add values to enable multiple outputs. |
| `SMG_TRAJ_DIR` | `1` | `0`: `./Trajectories/` relative to the PX4 working directory; `1`: `/fs/microsd/Trajectories/`. |
| `SMG_TRAJ_ID` | `0` | Selects a file with prefix `ID%04d_`, such as `ID0001_example.traj` for ID `1`. |

Select at least one output. For example, `SMG_OUT_TYPE=3` publishes both the
standard PX4 setpoint and the custom trajectory message.

The module attempts parameter-based file selection at startup and on parameter
updates when the directory or ID differs from the last successfully loaded
selection. The selected directory must contain exactly one regular `.traj`
file with the requested prefix. Missing or ambiguous matches are rejected.
In SITL, the relative directory is normally under the build's `rootfs` directory.

Changing `SMG_EN`, the state input, or the output selection clears execution and
home state. Enable the desired configuration, then issue a fresh start request.
Local trajectory data remains available across these transitions. Parameter
changes and CLI commands cannot replace a trajectory while it is executing.

## Command-line interface

Run these commands in the PX4 shell:

| Command | Action |
| --- | --- |
| `simulink_guidance start` | Start the module task and attempt parameter-based trajectory loading. |
| `simulink_guidance stop` | Stop the module task and clear guidance execution. |
| `simulink_guidance status` | Show the selected source, trajectory dimensions, and guidance flags. |
| `simulink_guidance ls [directory]` | List directories and files at the supplied location or selected source directory. |
| `simulink_guidance set_src filename` | Load a file from the selected source directory. |
| `simulink_guidance set_src directory filename` | Load a file from an explicit directory. |
| `simulink_guidance trajectory start` | Prepare a run and capture the current vehicle position and yaw as home. |
| `simulink_guidance trajectory set_home` | Capture and publish a stationary home while execution is inactive. |
| `simulink_guidance trajectory execute` | Begin a prepared trajectory at elapsed time zero. |
| `simulink_guidance trajectory stop` | End execution and mark the run finished. |
| `simulink_guidance trajectory reset` | Clear the run and home state. |

Trajectory commands require the module to be running and `SMG_EN` to be enabled.
`set_src` appends `.traj` when the extension is omitted. It validates the entire
file before replacing the selected source and coefficients; a failed load
preserves the selected trajectory. A CLI source selection stays active until a
successful parameter-based selection replaces it or the module restarts.

For example, with `ID0001_example.traj` present in the SITL trajectory directory:

```sh
param set SMG_TRAJ_DIR 0
param set SMG_TRAJ_ID 1
param set SMG_IN_TYPE 0
param set SMG_OUT_TYPE 1
param set SMG_EN 1
simulink_guidance start
simulink_guidance status
```

If the airframe startup script starts the module, use `status` to inspect that
instance. Once the integrating application is ready to consume setpoints:

```sh
simulink_guidance trajectory start
simulink_guidance trajectory execute
```

`trajectory start` captures home; `trajectory execute` starts evaluation. Use
`trajectory set_home` before execution when the reference must be captured at a
different location, such as after takeoff.

## Application interaction

### Requests

Publish `sim_guidance_request` messages to control the module programmatically.
The topic has a queue depth of eight messages. Requests are consumed in order,
and state is reported on `sim_guidance_status`.

| Request flag | Behavior |
| --- | --- |
| `start` | Prepare a run and capture home if guidance has not started or the previous run has finished. |
| `reset` | Clear started, executing, finished, valid, and home state. Local trajectory coefficients are retained. |
| `set_home` | Capture current XYZ/yaw and zero all home derivatives. Rejected during execution. |
| `start_execution` | Begin evaluation when started, loaded, unfinished, and supplied with valid home and vehicle state. |
| `stop` | Clear executing/valid state and mark the run finished. Takes precedence over all other flags in the same message. |

`reset + start` prepares a fresh run in one message. `set_home + start_execution`
captures home and begins evaluation in one message. A message containing
`start` or `reset` does not also begin execution. Duplicate start/execute
requests do not restart an active run unless a reset is explicitly included.

An application-managed takeoff sequence is:

1. Enable guidance and establish the application's arming and offboard control.
2. Publish `reset + start` to capture ground home and prepare guidance.
3. Generate and publish takeoff setpoints from the application.
4. At the desired takeoff altitude, publish `set_home + start_execution`.
5. Consume guidance setpoints during execution and monitor status.
6. On completion or termination, generate the next hold, landing, or other
   application-controlled setpoints.

### Status

| Status field | Meaning |
| --- | --- |
| `started` | A guidance run has been prepared. It can remain true after completion. |
| `loaded` | A local trajectory is loaded, or the companion reports readiness with a recent message. |
| `executing` | Trajectory execution is active. |
| `finished` | The run reached its endpoint, received a stop, or encountered a terminating error. |
| `trajectory_valid` | Guidance is started, loaded, has home and valid vehicle state, and is unfinished. This can be true while waiting to execute. |
| `timestamp` | Status publication time in PX4 microseconds. |

`finished` does not distinguish successful completion from termination; inspect
module logs for the reason. CLI trajectory commands enqueue requests, so their
return value confirms publication rather than completion of the requested action.

Local setpoints are published on home capture, on every execution update, and
at the final endpoint. There is no continuous setpoint publication while waiting
to execute or after completion. The application must provide any required
continuous offboard stream and handle setpoint ownership between flight phases.

## Coordinates and vehicle state

Axes are ordered `[x, y, z, yaw]`. Position uses the local North-East-Down frame:
X is north, Y is east, and Z is down. Position is in metres and yaw is in radians;
derivatives use seconds as the time unit.

Local-file positions are offsets from captured home:

```text
position_setpoint = home_position + polynomial_position
velocity_setpoint = polynomial_velocity
acceleration_setpoint = polynomial_acceleration
```

The same rule applies to yaw, jerk, and snap as appropriate: yaw position receives
the home yaw offset; its derivatives come from the polynomial. Home derivatives
are zero. Axes absent from the trajectory hold their home position with zero
derivatives. XY offsets are expressed in local NED axes and are not rotated by
home yaw. A zero initial polynomial position starts at home; a nonzero initial
value commands an offset immediately.

### Local-position input

With `SMG_IN_TYPE=0`, home capture and execution require:

- A `vehicle_local_position` sample no older than 500 ms.
- Valid XY and Z position flags.
- Finite X, Y, Z, and heading values.

`vehicle_local_position.heading` supplies Euler yaw. Ground startup does not
require `heading_good_for_control`, because EKF2 can leave that flag false until
in-flight magnetic alignment completes.

Single XY, Z, and heading estimator reset deltas shift the saved home and current
setpoint. If multiple resets were missed, a delta is invalid, or the coordinate
frame changes during companion guidance, the module terminates the run and
requires a fresh start. Invalid or stale vehicle state during execution also
terminates the run.

### Debug-array input

With `SMG_IN_TYPE=1`, the module subscribes to `simulink_inbound`, a `debug_array`
topic. Its timestamp must be nonzero and no older than 500 ms. Array indices are
zero-based:

| `data` indices | Contents |
| --- | --- |
| `18–20` | NED velocity XYZ. |
| `24–27` | Attitude quaternion in `[w, x, y, z]` order. |
| `32–34` | NED acceleration XYZ. |
| `35–37` | NED position XYZ. |

The quaternion must be finite with norm at least 0.5; it is normalized before yaw
extraction. Position and extracted yaw must be finite. Home capture uses XYZ/yaw
and zero derivatives.

## Output topics

| Topic | Contents |
| --- | --- |
| `trajectory_setpoint` | XYZ position, velocity, acceleration, jerk, plus yaw and yaw speed. Yaw is wrapped to `[-pi, pi)`. |
| `sim_guidance_trajectory` | Four-element arrays for position through snap, active DOF count, trajectory time in seconds, and publication timestamp. Yaw can be continuous/unwrapped. |
| `simulink_guidance` | `debug_array` with ID `SIMULINK_GUIDANCE_ID` and name `guidance`. Contains completion state and four-axis derivatives. |

The `simulink_guidance.data` layout is:

| Indices | Contents |
| --- | --- |
| `0` | Finished flag as a float. |
| `1–4` | Position `[x, y, z, yaw]`. |
| `5–8` | Velocity. |
| `9–12` | Acceleration. |
| `13–16` | Jerk. |
| `17–20` | Snap. |

At least one output must be enabled for home or trajectory publication to
succeed. Nonfinite setpoints and derivative overflow are rejected.

## Companion guidance

Set `SMG_EN=2` to exchange `debug_array` messages with a companion. Vehicle state
still comes from `SMG_IN_TYPE`. The companion supplies absolute NED XYZ
setpoints; local-file coefficients are not used for companion evaluation.
Companion output has three active axes and holds the captured yaw with zero yaw
rate.

### PX4 to companion: `companion_guidance_inbound`

The message uses ID `COMPANION_GUIDANCE_INBOUND_ID` and name `compg_in`.

| `data` indices | Contents |
| --- | --- |
| `0` | Start request. |
| `1` | Start-execution request. |
| `2` | Stop request. |
| `3–5` | Current reference position XYZ. |
| `6–8` | Current reference velocity XYZ. |
| `9–11` | Current reference acceleration XYZ. |
| `12–14` | Current reference jerk XYZ. |
| `15–17` | Current reference snap XYZ. |
| `18–20` | Measured vehicle position XYZ. |

Requests are emitted when processed. Reference/state updates are published while
guidance is started and unfinished.

### Companion to PX4: `companion_guidance_outbound`

| `data` indices | Contents |
| --- | --- |
| `0` | Alive/ready flag; a finite value greater than 0.1 means ready. |
| `1` | Finished flag; a finite value greater than 0.1 marks completion. |
| `2` | Finite, nonnegative trajectory time in seconds. |
| `3–5` | Absolute position XYZ. |
| `6–8` | Velocity XYZ. |
| `9–11` | Acceleration XYZ. |
| `12–14` | Jerk XYZ. |
| `15–17` | Snap XYZ. |

The received timestamp must be nonzero and current in the PX4 time domain.
During execution, all supplied trajectory values must be finite. The module
publishes a valid final sample even when that packet also sets `finished`.
Loss of readiness or a message age exceeding one second terminates execution.
A ready companion is required before a start-execution request can be accepted.

## Trajectory file format

A `.traj` file contains a packed three-byte header followed by one packed
46-byte record for each segment/axis pair. Supported PX4 targets use
little-endian IEEE-754 float32 values. There is no file-format version or magic
field.

Header fields, in order:

| Type | Field | Range |
| --- | --- | --- |
| `uint8` | `n_coeffs` | 1–10 coefficients, supporting polynomial degree up to 9. |
| `uint8` | `n_int` | 1–50 segments. |
| `uint8` | `n_dofs` | 1–4 axes, taken in `[x, y, z, yaw]` order. |

Each record contains:

| Type | Field | Description |
| --- | --- | --- |
| `uint8` | `i_int` | Zero-based segment index. |
| `uint8` | `i_dof` | Zero-based axis index. |
| `float32` | `t_int` | Segment duration in seconds. |
| `float32[10]` | `coefs` | Coefficients in ascending power order; the first `n_coeffs` entries are used. |

Records are ordered by segment, then by axis. All ten coefficient slots are
stored even when fewer coefficients are active. For segment duration `T` and
elapsed segment time `t`, the polynomial is:

```text
tau = t / T
p(tau) = c[0] + c[1]*tau + ... + c[n_coeffs-1]*tau^(n_coeffs-1)
```

Derivatives are scaled by `1/T^order`. The evaluator locates the active segment
once and computes position through snap using a differentiated Horner recurrence
with double intermediate arithmetic and float outputs. Evaluation at or beyond
the total duration uses the final endpoint and marks execution finished.

Loading rejects invalid dimensions or record indices, truncated data, nonfinite
active coefficients, and nonpositive or nonfinite durations. Axis durations
within a segment must agree within `1e-5` seconds, and accumulated duration must
remain finite and increase for each segment. Trajectory generation is responsible
for continuity between segments and suitable motion limits.

## Runtime and source layout

The module runs a dedicated task with a 4 KB stack. Each worker iteration handles
parameter updates and guidance, then sleeps for 5 ms; execution time contributes
to the actual update period. CLI and worker access are serialized with a mutex.
File loading stages approximately 8 KB on the heap before replacing the active
data. Normal trajectory evaluation performs no heap allocation.

| File | Responsibility |
| --- | --- |
| `simulink_guidance.cpp` / `.h` | Task lifecycle, CLI, parameters, and file selection. |
| `trajectory.cpp` / `.hpp` | Requests, state validation, home, execution, and topic interfaces. |
| `trajectory_math.hpp` | Polynomial and derivative evaluation. |
| `file_loader_backend.cpp` / `.hpp` | File access, packed file structures, and path handling. |
| `module.yaml` | Parameter definitions. |
| `tests/` | Host regression tests and runner. |

`waypoints.cpp` and the generated solver under `libs/` are excluded from the
module build. Runtime guidance consumes exported trajectories or companion
samples; it does not solve waypoint trajectories onboard.

## Tests

Build a lockstep SITL configuration containing the module and run the host tests
from the repository root. The runner requires a C++ compiler, Python 3, and
system GoogleTest headers/libraries (`libgtest-dev` on Debian/Ubuntu).

```sh
cmake --build build/px4_sitl_simulink --target px4 -j4
python3 src/modules/simulink_guidance/tests/run_host_tests.py
```

Use `--build-dir` to select another lockstep SITL build and `--asset-dir` to select
a trajectory directory. By default, asset checks use the build's
`rootfs/Trajectories` directory when available.

The runner links the build's uORB libraries, advances simulated time, and uses
temporary files in a separate process. It does not start Gazebo or connect to a
running SITL instance. Tests are also registered with PX4's functional GoogleTest
infrastructure; state-machine cases require lockstep time.

Coverage includes ground startup, takeoff handoff, yaw and derivative outputs,
home capture, request ordering, repeated runs, invalid files, state freshness,
estimator resets, debug-array input, companion completion/timeouts, segment
boundaries, and numerical overflow. Polynomial results are compared against
independent long-double reference calculations. Asset checks exercise valid
exports and rejection of nonfinite trajectory data.
