# Simulink I/O module

`simulink_io` runs the model exported by the `px4_simulink_io` toolbox. The
handwritten task owns scheduling and performance counters. The generated
`simulink_model_wrapper.h` owns model initialization, parameter refresh, stepping,
and termination. Model names, uORB topics, and sample periods require no edits to
the PX4 task.

## Runtime behavior

```sh
simulink_io start
simulink_io status
simulink_io stop
```

The task initializes the model once, calls its combined step function at the
compiled base period, and calls termination on a normal stop. `status` reports the
model name, configured period, observed intervals, execution time, and overruns.
An overrun resets the scheduling baseline without replaying missed steps.
Scheduling uses PX4's high-resolution clock; it does not guarantee hard real-time
execution or synchronize model steps with individual sensor publications.

The task imposes no arming, offboard, takeoff, or estimator-validity conditions.
Those decisions belong to the generated model and the PX4 modules it interacts
with. Task priority (`SCHED_PRIORITY_DEFAULT`) and stack size (8192 bytes) are
platform settings in the handwritten module; model computation must fit those
resources and its selected period.

## Generated model contract

The toolbox reads the compiled interface from `codeInfo.mat`, including the
resolved base period and entry-point names. It supports fixed-step,
single-tasking, nonreusable C models with one combined `void step(void)` entry
point; slower rates can execute inside that generated step. Periods must be
representable as whole microseconds. Unsupported interfaces fail during export.

## Model configuration and generation workflow

Configure the model manually in Simulink or use this optional settings helper once:

```matlab
px4io.deployment.configureModel('myModel');
```

The helper selects fixed-step, single-tasking, nonreusable C generation and enables
internal/external startup memory initialization, including initialization settings
on referenced models. It preserves the selected solver and sample time. It does
not save the model, generate code, or export anything. Save the affected model
settings in Simulink; ordinary model edits and PX4 updates do not require repeating
the configuration step.

Keep the model's Post-code generation command configured as:

```matlab
px4io.px4API.runPostCodeGen(buildInfo);
```

For everyday work, edit the model, press **Generate Code**, let the callback export,
and rebuild PX4. `slbuild('myModel')` is the command-line alternative to the button,
not another required step. The callback automatically collects parameter bindings,
reads the compiled sample period and entry points, generates the adapter and uORB
glue, and stages the complete export before replacing `generated_code`.

| Change | Action |
| --- | --- |
| Model logic, topics, parameters, or sample time | Generate Code, then rebuild PX4 |
| New model or changed code-generation settings | Configure and save settings, then Generate Code |
| PX4 code with unchanged message interfaces | Rebuild PX4 |
| PX4 message definitions | Refresh Simulink's API/buses before Generate Code, then rebuild PX4 |
| Generator/runner interface | Use matching versions, regenerate, then rebuild PX4 |

After editing PX4 `.msg` definitions in an existing MATLAB session, run
`px4io.px4API.forceGenerateAll()` before generating model code. This refreshes
message interfaces; it does not change model settings. The post-generation
callback cannot retroactively update bus layouts used to compile the model.

Export validates settings and the compiled interface without changing them. An
export failure leaves the previous deployed code in place. CMake detects changes
to the generated source file set. Generate persistent changes through the toolbox
rather than editing exported files. Export does not compile or flash PX4.

## Messages and parameters at runtime

The generated adapter refreshes cached parameters before each model step. PX4
parameter handles are resolved at initialization; values refresh on parameter
update notifications. Parameters must be defined in the selected firmware with
matching types. Model bindings alone do not create PX4 parameter definitions.

uORB reads return the last received message, initially zero, and retain it when
there is no new publication. Model logic should inspect timestamps when freshness
matters. Message initializers can set floating fields to NaN, and writers publish
the fields provided by the model. Read buffers reset on initialization, and normal
termination releases subscriptions. The generated model owns the meaning and use
of these messages, including requests sent to `simulink_guidance` or commander.

## Diagnostics

- Missing adapter members such as `period_us` indicate a mismatched generator and
  runner or stale generated code. Regenerate with the matching toolbox.
- Unsupported interface or restart-setting errors require correcting model
  configuration before generating again.
- Missing parameter warnings require checking the model binding and firmware's
  parameter definitions.
- Overruns indicate that execution did not finish before the next deadline. Use
  execution-time and interval counters to assess timing.
- SITL lockstep needs simulator time to advance. A standalone module lifecycle
  test does not measure flight-loop timing.

## Build and airframe selection

The `simulink` board variant retains the stock PX4 controllers and adds
`simulink_io` and `simulink_guidance`. The `simulinkctrl` variant removes the
stock controllers. Both variants are available for SITL, FMU v6c, Cube Orange,
and Cube Orange Plus.

Building a variant does not select an airframe. Choose a matching airframe:

| Platform | Stock controllers + Simulink (mode 1) | Full Simulink control (mode 2) |
| --- | --- | --- |
| Hardware | 4030_x500_v2_simulink | 4031_x500_v2_simulinkctrl |
| Gazebo S500 | 4040_gz_s500_simulink | 4050_gz_s500_simulinkctrl |
| Gazebo S500 with downward lidar | 4041_gz_s500_simulink_lidar_down | 4051_gz_s500_simulinkctrl_lidar_down |

On hardware, select the airframe through `SYS_AUTOSTART` and reboot. On SITL,
use the corresponding launch target, for example:

```sh
make px4_sitl_simulink gz_s500_simulink_lidar_down_baylands
make px4_sitl_simulinkctrl gz_s500_simulinkctrl_lidar_down_baylands
make cubepilot_cubeorangeplus_simulinkctrl
```

Mode 2 bypasses `rc.vehicle_setup`, including stock control allocator startup.
It requires generated code that implements the replacement control/output
path. A model that only publishes `trajectory_setpoint` and requests position
control through `offboard_control_mode` requires mode 1. Changing the build
variant cannot turn a position-setpoint generator into a motor controller.
When replacing generated code, ensure its outputs match the selected mode and
the hardware output drivers. The hardware control variants retain PX4IO and
DShot but disable the FMU `pwm_out` driver, following the Cube Orange profile.

`simulink_guidance` has separate enable, output-topic, and trajectory-location
parameters. `SMG_OUT_TYPE` is a bitmask: 1 publishes `trajectory_setpoint`, 2
publishes `sim_guidance_trajectory`, and 3 publishes both. Ensure that the
generated model subscribes to the selected topic and that independent modules
do not compete to publish the same setpoint. `SMG_TRAJ_DIR=0` selects local
SITL trajectories; 1 selects the hardware SD card. Starting the module alone
does not enable updates when `SMG_EN=0`.
