from ForceSeatMI import *
import math
import time

PROFILE_NAME = 'SDK - Vehicle Telemetry ACE'
MAX_AMPLITUDE = 17.0   # Start small; increase gradually after validation.
RAMP_SECONDS = 5.0    # Ramp amplitude from 0 -> MAX_AMPLITUDE.
SLEEP_SEC = 0.01
ITER_STEP = 0.05
ITER_END = 50.0

fsmi = ForceSeatMI()

telemetry = FSMI_TelemetryACE()
telemetry.structSize = sizeof(FSMI_TelemetryACE)
telemetry.mask = FSMI_TelemetryMask()

platformInfo = FSMI_PlatformInfo()
platformInfo.structSize = sizeof(FSMI_PlatformInfo)

iterator = 0.0
start_time = time.perf_counter()

try:
    if not fsmi.activate_profile(PROFILE_NAME):
        raise RuntimeError('Failed to activate ForceSeat profile')

    if not fsmi.begin_motion_control():
        raise RuntimeError('Failed to begin motion control')

    while iterator < ITER_END:
        elapsed = time.perf_counter() - start_time
        ramp = min(elapsed / RAMP_SECONDS, 1.0)
        amplitude = MAX_AMPLITUDE * ramp

        telemetry.bodyLinearAcceleration.forward = math.sin(iterator) * amplitude

        if not fsmi.send_telemetry_ace(telemetry):
            raise RuntimeError('Failed to send telemetry packet')

        iterator += ITER_STEP

        # Optional status polling; continue even if this call fails.
        fsmi.get_platform_info_ex(platformInfo, sizeof(FSMI_PlatformInfo), 100)

        time.sleep(SLEEP_SEC)

except KeyboardInterrupt:
    print('Interrupted by user')

finally:
    try:
        fsmi.end_motion_control()
    finally:
        fsmi.delete()
