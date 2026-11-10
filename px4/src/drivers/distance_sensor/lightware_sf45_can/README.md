# LightWare SF45/B from a Matek CAN-G474 node

This guide covers the PX4 flight-controller side: building and loading the
custom Cube Orange+ firmware, setting PX4 parameters, and confirming reception.
ArduPilot-specific instructions for compiling, updating, and configuring the
Matek CAN-G474 are in the ArduPilot checkout's `Tools/AP_Periph/README.md`,
under “Matek CAN-G474 SF45/B to PX4”.

The system has two firmware images. AP_Periph on the Matek reads SF45 UART
samples and sends them over classic DroneCAN. This PX4 build receives those
samples and publishes the normal 72-bin, 5-degree `obstacle_distance` map.
Both ends must use the paired implementation described in their respective
repositories; stock firmware on either end does not provide this SF45 CAN
path.

## Build and upload this PX4 image

From the modified PX4 repository root, build and upload the Cube Orange+ image
for this repository's `surgeon` configuration:

```sh
cd /home/jack/PX4/v1.17.0-mod
make cubepilot_cubeorangeplus_surgeon -j4
make cubepilot_cubeorangeplus_surgeon upload
```

Connect the flight controller to the development computer by USB and follow
the PX4 bootloader upload prompt. The `.px4` file is also generated in
`build/cubepilot_cubeorangeplus_surgeon/` for upload with QGroundControl. This
is a custom build; a stock Cube Orange+ image does not contain
`lightware_sf45_can`.

AP_Periph build, update, recovery, and SLCAN adapter instructions are kept in
the ArduPilot checkout's `Tools/AP_Periph/README.md`, under “Matek CAN-G474
SF45/B to PX4”. That README includes the Cube Orange+ SD-card firmware-server
workflow and the ArduPilot-flight-controller SLCAN option. The Matek must
already have a working AP_Periph CAN bootloader for either CAN update path;
neither path installs or recovers a missing bootloader.

## Configure PX4

Use a unique PX4 node ID and match the bus bitrate used by the Matek node.
The SF45 CAN receiver is enabled by default and starts automatically at boot.
Its source-node parameter defaults to AP_Periph's node ID 42, while remaining
user-configurable if the node ID changes:

| Parameter | Example | Purpose |
|---|---:|---|
| `UAVCAN_ENABLE` | `1` | Enable manually configured DroneCAN sensors; does not switch motor outputs to CAN |
| `UAVCAN_BITRATE` | `1000000` | Match the Matek node's `CAN_BAUDRATE` |
| `UAVCAN_NODE_ID` | `1` | PX4 node ID; keep it distinct from the Matek node ID |
| `SENS_EN_SF45_CFG` | `0` | Disable the direct-UART SF45 path when using the same sensor over CAN |
| `SF45C_ENABLE` | `1` | Start the SF45 CAN receiver on boot (default) |
| `SF45C_NODE` | `42` | Match AP_Periph's configured `CAN_NODE` (default; change if needed) |
| `SF45C_SENSOR` | `0` | Match AP_Periph's `PRX1_ADDR` |
| `SF45C_ORIENT` | `24` or `25` | `24` upright; `25` downward |
| `SF45C_YAW` | `0`, `2`, `4`, or `6` | Forward, right, backward, or left mounting yaw |
| `SF45C_TIMEOUT` | `100` | Age in milliseconds before an observation becomes unknown |

Set any non-default values in QGroundControl's parameter editor or the PX4
MAVLink Console, save, and reboot. The startup script launches
`lightware_sf45_can` when `SF45C_ENABLE=1`; no separate driver command is
needed. The default mounting is upright and forward-facing. Set orientation
and yaw for the actual vehicle installation. Do not apply vehicle mounting
corrections on both PX4 and the Matek node.

If PX4 must also configure the Matek's remote parameters, enable
`UAVCAN_ENABLE=1` first and reboot. In the PX4 MAVLink Console,
`uavcan status` identifies the node's current ID and `uavcan param list`,
`get`, `set`, and `save` commands access its parameters. The exact commands
and AP parameter values belong to the ArduPilot README section named above.
The optional US-D1, when enabled on AP_Periph, arrives through PX4's normal
DroneCAN range-sensor bridge and does not use the SF45-specific parameters.

## Confirm that both ends are running the paired firmware

First confirm that `uavcan status` lists the Matek at the ID configured in
`SF45C_NODE`. In `lightware_sf45_can status`, the sample counter should
increase. `listener sf45_sample` should show messages with that source node ID
and the sensor ID configured in `SF45C_SENSOR`. These messages are emitted by
PX4's UAVCAN bridge only for the paired raw-SF45 packet marker; they confirm
that the CAN-G474 is sending the expected format, not just that a node is
electrically visible. Finally, `listener obstacle_distance` should show the
SF45 map.

```text
uavcan status
lightware_sf45_can status
listener sf45_sample
listener obstacle_distance
```

If the shell reports `lightware_sf45_can` as an unknown command, the running
PX4 image was not built with this module or is not the expected custom target.
If the module exists but is stopped, check `SF45C_ENABLE` and reboot. If the
node is visible but the sample counter does not move, check the AP_Periph
firmware and settings using the ArduPilot README section, the SF45 UART baud and
streaming, the CAN bitrate, and the matching node/sensor IDs. If samples arrive
but the obstacle map is wrong, check PX4's mounting parameters and the sensor's
physical scan limits.

Bench-check with the vehicle disarmed and props removed. Disconnect CAN or the
SF45 UART and confirm that readings expire to unknown after
`SF45C_TIMEOUT`; reconnect and check the front/left/right directions against
the installation before enabling vehicle-level collision prevention.

## What changed from stock PX4

This integration adds a `sf45_sample` uORB message and handling in PX4's
existing UAVCAN sensor bridge. It recognizes the paired raw-SF45 marker in the
existing DroneCAN `Proximity` message and delivers the sample to a separate
`lightware_sf45_can` module. That module filters by node and sensor ID, applies
PX4-side mounting/orientation and attitude projection, and publishes
`obstacle_distance`. The Cube Orange+ `surgeon` build configuration includes
the module, and the sensor startup script starts it when `SF45C_ENABLE=1`.

The direct-UART SF45 driver was refined separately: UART and CAN now use the
same obstacle-map implementation, and the UART baud rate and observation
timeout are exposed as parameters. The UART driver still reads a sensor
directly connected to PX4; it is not involved when the sensor is wired to the
Matek. Set `SENS_EN_SF45_CFG=0` for the CAN installation to avoid running both
input paths for one physical sensor.

The AP/PX4 message extension reuses DroneCAN `Proximity` with reserved
`flags=1` for raw sensor-frame SF45 data; `flags=0` remains the existing coarse
body-frame proximity form. This paired interpretation is not a standardized
SF45 DroneCAN profile. A generic receiver that ignores the flag may interpret
the yaw incorrectly. Deploy the modified AP_Periph and PX4 images together.

## Map behavior and limits

Both PX4 input paths use 72 bins spaced 5 degrees apart, centimetres,
body-FRD output, and the same mounting, projection, within-bin minimum, and
expiry logic. The CAN path transports each parsed SF45 sample; it does not
expand AP's coarse eight-sector boundary output. The scan's low/high limits
remain settings stored in the SF45. A narrower sweep fills fewer bins without
changing the map increment, and directions outside the sweep remain unknown.

CAN float16 encoding introduces small quantization, so a sample near a bin
edge may land in an adjacent bin compared with UART. PX4 timestamps samples on
reception; acquisition-time clock synchronization is not implemented. The
current AP_Periph backend requests SF45 update-rate code 6 (129
readings/s), which is approximately 39 kbit/s of classic CAN capacity at up to
150 bits per extended frame, before accounting for other traffic. Check the
actual bus load and data expiry on the bench.
