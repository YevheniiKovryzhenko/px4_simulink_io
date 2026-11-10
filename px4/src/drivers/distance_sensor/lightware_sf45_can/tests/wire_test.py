#!/usr/bin/env python3
"""Check the AP_Periph generated C encoder against DroneCAN's DSDL decoder.

Run with the ArduPilot venv Python and the ArduPilot repository as argument.
Requires a MatekG474-Periph build and the existing dronecan Python package.
"""
import ctypes
import pathlib
import subprocess
import sys
import tempfile

import dronecan
from dronecan.transport import bits_from_bytes

ap = pathlib.Path(sys.argv[1]).resolve()
px4 = pathlib.Path(__file__).resolve().parents[5]
generated = ap / 'build/MatekG474-Periph/modules/DroneCAN/libcanard/dsdlc_generated'
canard = ap / 'modules/DroneCAN/libcanard'
message = 'ardupilot.equipment.proximity_sensor.Proximity'
relative = 'ardupilot/equipment/proximity_sensor/21910.Proximity.uavcan'
assert (ap / 'modules/DroneCAN/DSDL' / relative).read_text() == (
    px4 / 'src/drivers/uavcan/libdronecan/dsdl' / relative).read_text()
assert dronecan.ardupilot.equipment.proximity_sensor.Proximity()._type.get_data_type_signature() == 0x99DD3985FB3222CE


class Packet(ctypes.Structure):
    _fields_ = [('sensor_id', ctypes.c_uint8), ('reading_type', ctypes.c_uint8),
                ('flags', ctypes.c_uint8), ('yaw', ctypes.c_float),
                ('pitch', ctypes.c_float), ('distance', ctypes.c_float)]


with tempfile.TemporaryDirectory(prefix='sf45-wire-') as tmp:
    library = pathlib.Path(tmp) / 'wire.so'
    subprocess.run(['cc', '-shared', '-fPIC', '-I' + str(canard),
                    '-I' + str(generated / 'include'), str(canard / 'canard.c'),
                    str(generated / 'src' / (message + '.c')), '-o', str(library)], check=True)
    deadline_test = pathlib.Path(tmp) / 'deadline_test'
    subprocess.run(['cc', '-DCANARD_ENABLE_DEADLINE=1', '-I' + str(canard),
                    str(pathlib.Path(__file__).with_name('deadline_test.c')),
                    str(canard / 'canard.c'), '-o', str(deadline_test)], check=True)
    subprocess.run([str(deadline_test)], check=True)
    encoder = ctypes.CDLL(str(library)).ardupilot_equipment_proximity_sensor_Proximity_encode
    encoder.argtypes = [ctypes.POINTER(Packet), ctypes.POINTER(ctypes.c_uint8)]
    encoder.restype = ctypes.c_uint32
    for yaw, cm in [(-160, 0), (0, 100), (42.5, 5000), (160, 5001), (-2.49, 1234), (0, 65535)]:
        metres = 51.0 if cm > 5000 else cm * 0.01
        packet = Packet(7, 2, 1, yaw, 0, metres)
        buffer = (ctypes.c_uint8 * 8)()
        assert encoder(ctypes.byref(packet), buffer) == 8
        decoded = dronecan.ardupilot.equipment.proximity_sensor.Proximity()
        decoded._unpack(bits_from_bytes(bytes(buffer)))
        assert (decoded.sensor_id, decoded.reading_type, decoded.flags, decoded.pitch) == (7, 2, 1, 0)
        assert abs(decoded.yaw - yaw) <= 0.063
        assert abs(decoded.distance - metres) <= 0.016
        assert (decoded.distance > 50) == (cm > 5000)
print('AP C encoder / matching DSDL decoder wire test passed')
