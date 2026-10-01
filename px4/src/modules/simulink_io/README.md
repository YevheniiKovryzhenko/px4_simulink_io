# Simulink build and airframe selection

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

For build directories configured before the exact configuration-name matching
fix, clear the cached selection once before rebuilding. For example:

```sh
cmake -S . -B build/px4_sitl_simulinkctrl -U PX4_CONFIG_FILE -DCONFIG=px4_sitl_simulinkctrl
```
