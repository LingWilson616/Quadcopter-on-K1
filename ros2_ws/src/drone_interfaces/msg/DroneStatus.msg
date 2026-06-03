# DroneStatus.msg
# Current state of the drone from ArduPilot
uint8 MODE_STABILIZE=0
uint8 MODE_ALT_HOLD=1
uint8 MODE_AUTO=2
uint8 MODE_GUIDED=3
uint8 MODE_LAND=4
uint8 MODE_RTL=5

uint8 mode
float32 roll
float32 pitch
float32 yaw
float32 altitude
float32 ground_speed
float32 battery_voltage
float32 battery_current
int32 latitude
int32 longitude
float32 heading
bool armed
