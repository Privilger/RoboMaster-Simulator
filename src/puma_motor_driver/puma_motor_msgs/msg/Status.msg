# Number on the bus (CAN ID).
uint8 device_number

# Name of joint controlled, or other identifier.
string device_name

# Input terminal voltage (volts).
float32 bus_voltage

# Internal driver temperature (degC).
float32 temperature

# Voltage as output to the motor (volts).
float32 output_voltage

# Value of the auxiliary ADC (volts).
float32 analog_input

# Available control modes, not all of which are broken out to
# this ROS driver.
uint8 MODE_VOLTAGE=0
uint8 MODE_CURRENT=1
uint8 MODE_SPEED=2
uint8 MODE_POSITION=3
uint8 MODE_VCOMP=4
uint8 mode

# Fault states which could cause the driver to be immobilized.
uint8 FAULT_CURRENT=1
uint8 FAULT_TEMPERATURE=2
uint8 FAULT_BUS_VOLTAGE=4
uint8 FAULT_BRIDGE_DRIVER=8
uint8 fault
