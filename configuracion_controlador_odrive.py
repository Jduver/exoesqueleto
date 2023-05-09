# -*- coding: utf-8 -*-
"""
Created on Sun Apr 10 17:04:28 2022

@author: DUVER
"""
#!/usr/bin/env python3

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
odrv0 = odrive.find_any()

#restore the motherboard default parameters
odrv0.erase_configuration()


#1 config dissipation resistor
odrv0.config.brake_resistance = 2.0

#2 config low voltage portection threshold
odrv0.config.dc_bus_undervoltage_trip_level= 8.0

#3 config overvoltage protection threshold
odrv0.config.dc_bus_overvoltage_trip_level= 56.0

#4 config overcurrent protection threshold
odrv0.config.dc_max_positive_current = 20.0

#5 config the reverse overcurrent protection threshold
odrv0.config.dc_max_negative_current =-1.0

#6 config the recharge current value
odrv0.config.max_regen_current =0

#7 save config parameters
odrv0.save_configuration()






#Part2: motor parameter configuration

#1 config the number of motor pole pairs
odrv0.axis0.motor.config.pole_pairs=7

#2 config the current when the motor is calibrated
odrv0.axis0.motor.config.calibration_current =5

#3 config the voltage when the motor is calibrated
odrv0.axis0.motor.config.resistance_calib_max_voltage= 2

#4 config motor type
odrv0.axis0.motor.config.motor_type=MOTOR_TYPE_HIGH_CURRENT

#5 Config the maximum current limit for motor operation
odrv0.axis0.motor.config.current_lim = 10 # [A]

#6 config the motor current sampling range
odrv0.axis0.motor.config.requested_current_range = 20

#******************************************************
#
odrv0.axis0.motor.config.current_control_bandwidth = 2000
odrv0.axis0.motor.config.torque_constant = 8.27 / 270
odrv0.axis0.motor.config.torque_lim= 1.3
#******************************************************


#7 save config parameters
odrv0.save_configuration()


# part 3
#encoder parameter config

#1 config encoder type
odrv0.axis0.encoder.config.mode= ENCODER_MODE_INCREMENTAL


#2 Config encoder resolution
#
odrv0.axis0.encoder.config.cpr =8192
#odrv0.axis0.encoder.config.cpr = 4096  16384
odrv0.axis0.encoder.config.enable_phase_interpolation = True

#3 config encoding bandwith
odrv0.axis0.encoder.config.bandwidth=3000

#4 motor running current when configuring encoder calibration
odrv0.axis0.config.calibration_lockin.current =3

#5 config the current rise time during encoder calibration
odrv0.axis0.config.calibration_lockin.ramp_time = 0.4

#6 config the motor rotation distance when the current rises
odrv0.axis0.config.calibration_lockin.ramp_distance = 3.1415927410125732
#7 motor acceletation when configuring encoder calibration
odrv0.axis0.config.calibration_lockin.accel=20

#8 motor speed when configuring encoder calibration
odrv0.axis0.config.calibration_lockin.vel=40

#hace que gire sin control si esta en true
#odrv0.axis0.config.enable_sensorless_mode = True
#9 save config parameters
odrv0.save_configuration()



#PART 4 
#1 config control mode
odrv0.axis0.controller.config.control_mode= CONTROL_MODE_POSITION_CONTROL

#2 config the maximum speed of the motor
odrv0.axis0.controller.config.vel_limit=5

#3 config position loop gain
odrv0.axis0.controller.config.pos_gain =30

#4 config speed loop gain
odrv0.axis0.controller.config.vel_gain = 0.2
odrv0.axis0.controller.config.vel_integrator_gain = 0.5 * 10 * 0.2 #0.2


#5 config input mode
odrv0.axis0.controller.config.input_mode= INPUT_MODE_TRAP_TRAJ

#6 Config the motor speed in trapezoidal mode
odrv0.axis0.trap_traj.config.vel_limit =30

#7 config the motor acceleration/deceleration in trapezoidal 
odrv0.axis0.trap_traj.config.accel_limit =5
odrv0.axis0.trap_traj.config.decel_limit =5


#odrv0.axis0.controller.config.anticogging.pre_calibrated=True

#8 save the config 
odrv0.save_configuration()

odrv0.reboot()



#-----------------------------------
#makes the sound
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE



#makes the sound
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION


odrv0.axis0.motor.config.pre_calibrated = True

#rotates 

odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

odrv0.axis0.config.startup_encoder_offset_calibration = True

odrv0.axis0.config.startup_closed_loop_control = True


odrv0.save_configuration()

#odrv0.reboot()
odrv0.clear_errors()
dump_errors(odrv0)
#odrv0.axis0.motor.config
#-------------------------------------------------
#da 50 vueltas
#odrv0.axis0.controller.config.input_mode = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.requested_state =AXIS_STATE_CLOSED_LOOP_CONTROL

odrv0.axis0.controller.input_pos =50

odrv0.axis0.controller.input_pos =0

#release the motor

odrv0.axis0.requested_state = AXIS_STATE_IDLE



odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_torque = 1




#***************************************************************************************
#velocity control

odrv0.axis0.encoder.config.use_index =True

#Run the calibration
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE


#At this point you can test with the following commands to see how the motor performs, it should be stuttery
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.controller.input_vel = 0.2


#***************************************************************************************


odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL


# Approximately 8.23 / Kv where Kv is in the units [rpm / V]
odrv0.axis0.motor.config.torque_constant = 8.27 / 150

odrv0.axis0.controller.input_torque = 1

odrv0.axis0.controller.input_torque = 0.01

start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])










