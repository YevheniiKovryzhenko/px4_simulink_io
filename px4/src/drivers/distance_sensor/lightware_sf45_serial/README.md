# LightWare SF45/B on a PX4 UART

The dedicated `lightware_sf45_serial` driver reads the SF45/B directly from a
PX4 serial port and publishes `obstacle_distance`. Its 72 bins remain spaced at
5 degrees. Sensor scan limits control which directions receive measurements;
they do not change the map increment. Directions without a recent measurement
remain unknown.

## Configure a direct UART connection

In QGroundControl's parameter editor, select the UART with `SENS_EN_SF45_CFG`
and set:

| Parameter | Setting |
|---|---|
| `SENS_EN_SF45_CFG` | PX4 serial port connected to SF45 |
| `SF45_BAUD_CFG` | The sensor's stored baud rate; default is 921600 |
| `SF45_UPDATE_CFG` | SF45 update-rate code appropriate for the scan |
| `SF45_ORIENT_CFG` | Upright or downward mounting |
| `SF45_YAW_CFG` | Forward, right, backward, or left mounting |
| `SF45_TIMEOUT` | Maximum age of a bin before it becomes unknown; default 100 ms |

The host baud parameter does not change the SF45's stored UART baud. Set the
sensor baud in LightWare Studio first if it differs from the PX4 setting. The
orientation and yaw values describe the physical installation. Increase the
timeout only when the scan revisit time requires it, since larger values retain
older obstacle readings for longer.

Reboot after changing the driver parameters. The scan limits remain settings
stored in the sensor; this driver does not configure them. At low update rates
or fast sweep speeds, some 5-degree bins may receive no sample and therefore
remain unknown. Adjust the sensor's stored scan and update settings to suit the
application.

For the CAN-G474 connection, configure the separate
[`lightware_sf45_can` path](../lightware_sf45_can/README.md). The UART parameters
above control only a sensor connected directly to a PX4 UART.
