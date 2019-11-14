# This message represents Ridgeback's lower-frequency status updates from the MCU
# Default publish frequency is 1Hz.

Header header

# Commit of firmware source.
string hardware_id

# Times since MCU power-on and MCU rosserial connection, respectively.
duration mcu_uptime
duration connection_uptime

# Temperature of MCU's PCB in Celsius.
float32 pcb_temperature
float32 mcu_temperature

# Monitoring the run/stop loop. Changes in these values trigger an immediate
# publish, outside the ordinarily-scheduled 1Hz updates.
bool stop_power_status  # True indicates the stop loop is operational (ie. it is powered).
bool stop_engaged  # True when a stop has been pressed on the robot.
bool drivers_active  # False when a stop is asserted to the motor drivers.
bool external_stop_present  # True indicates a external stop has been plugged in.

# Indicates if AC power is connected.
bool charger_connected
bool charging_complete

# Voltage rails, in volts
# Averaged over the message period
float32 measured_battery
float32 measured_12v
float32 measured_5v
float32 measured_inverter
float32 measured_front_axle
float32 measured_rear_axle
float32 measured_light

# Current senses available on platform, in amps.
# Averaged over the message period
float32 total_current

# Highest total system current peak as measured in a 1ms window.
float32 total_current_peak

# Integration of all power consumption since MCU power-on, in watt-hours.
float64 total_power_consumed
