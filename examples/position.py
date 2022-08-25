import sys
from datetime import time

sys.path.append('..')
import logging

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

#Initialization IP address, Port and xml file
ROBOT_HOST = '134.28.124.104'
ROBOT_PORT = 30004
config_filename = 'control_loop_configuration.xml'
synchronization_running = True #boolean flag for running the program

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename) #get the xml file
state_names, state_types = conf.get_recipe('state') #return list of variable names and types associated to state in xml (send to the robot)
setp_names, setp_types = conf.get_recipe('setp') #return list of variable names and types associated to setp in xml (receive from the robot)
watchdog_names, watchdog_types = conf.get_recipe('watchdog')

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT) #create instance con by constucting a connection with RTDE class
con.connect() #initialize RTDE connection to host
con.get_controller_version() #get controller version

# setup recipes
con.send_output_setup(state_names, state_types) #call method of receiving from the robot at frequency 300HZ
setp = con.send_input_setup(setp_names, setp_types) #call method of sending to the robot variable names from the list of possible RTDE inputs
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

# Setpoints to move the robot to
setp1 = [0.968, -0.105, -0.037, 0.062, 0.242, -5.288]
setp2 = [-0.562, -0.100, 0.100, 0.280, -0.401, -1.851]

#Resetting the variables that should be sent to the robot controller
setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0

#Sending input registers values to list
def setp_to_list(setp):
    list = []
    for i in range(0, 6):
        print(list.append(setp.__dict__["input_double_register_%i" % i]))
    return list

#Receiving from the list to input registers
def list_to_setp(setp, list):
    for i in range(0, 6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp

#Sends a start command to the RTDE server to initiate the actual synchronization
con.send_start()

#exit the interpreter by raising SystemExit if synchronization is not started
if not con.send_start():
    sys.exit()

#convert points to list to put it in the register
def points_to_list(setp):
    list = []
    list.append(setp)
    return list

#######Execution loop######

while synchronization_running:

    # receive the current state
    state = con.receive()
    #print(state)
    if state is None:
        break

    print("ok")
    #not yet moved: so send input to list
    if state.output_int_register_0 == 0:

        setp_to_list(setp2)
        con.send(setp)
        print('Synchronization complete')

    #moved: retrieve input from list
    elif state.output_int_register_0 == 1:

        list_to_setp(setp, list)
        con.send_pause()
        con.disconnect()

con.send(watchdog)  # kick watchdog




