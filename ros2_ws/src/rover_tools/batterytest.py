from pymavlink import mavutil
import time

m = mavutil.mavlink_connection('/dev/ttyACM1', baud=115200)
m.wait_heartbeat()
print("Connected")

# Try setting GUIDED mode
mode_mapping = m.mode_mapping()
print(f'Available modes: {list(mode_mapping.keys())}')

if 'GUIDED' in mode_mapping:
    print('GUIDED mode is available')
else:
    print('GUIDED mode not available — check ArduPilot version')

# Check if GUIDED_NOGPS is available — this is what we need
if 'GUIDED_NOGPS' in mode_mapping:
    print('GUIDED_NOGPS is available — use this instead')