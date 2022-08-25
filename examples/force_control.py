import csv
import sys
import time
import numpy as np
import math
from examples.Automated_test import test_force_x_range, test_force_y_range, test_vel_x_range, test_vel_y_range, \
    test_impedance_x_range, test_impedance_y_range, test_torque_x_range, test_torque_y_range, \
    test_resultant_force_range, test_resultant_torque_range
from examples.DataBase_Automatedtesting import end_test_force_x_range, end_test_force_y_range, end_test_vel_x_range, \
    end_test_vel_y_range, end_test_torque_x_range, end_test_torque_y_range, end_test_resultant_force_range, \
    end_test_resultant_torque_range
from examples.Functions import get_Traj, pose_scal, get_pose, dampingWrench
from examples.impedance import WrenchForImpedance
from examples.plotting_Sensors import plot_TCP_pose_x_time, plot_TCP_force_x_time, plot_TCP_force_y_time, \
    plot_TCP_pose_y_time, plot_velocity_x_time, plot_velocity_y_time, plot_Impedance_x_time, plot_Impedance_y_time, \
    plot_torque_x_time, plot_torque_y_time, plot_resultant_force_time, plot_resultant_torque_time, \
    plot_resultant_impedance_time

sys.path.append('..')
import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

#Initialization IP address, Port and xml file

ROBOT_HOST = '134.28.124.104'
ROBOT_PORT = 30004
config_filename = 'control_loop_configuration.xml'

keep_running = True
Trajectory = False
program_time = 70
y = []
t = []
tcp_pose = []
tcp_speed = []
tcp_position_x = []
tcp_position_y = []
tcp_velocity_x = []
tcp_velocity_y = []
tcp_force = []
tcp_force_x = []
tcp_force_y = []
tcp_pose_x = []
execution_time = []
time_stamp = []
impedance_x = []
impedance_y = []
torque_x = []
torque_y = []
force_time_x = []
force_time_y = []
resultant_force = []
resultant_torque = []
resultant_impedance = []
resultant_force_time = []
resultant_torque_time = []
velocity_time_x = []
velocity_time_y = []
force_time_x = []
force_time_y = []
impedance_time_x = []
impedance_time_y = []
torque_time_x = []
torque_time_y = []

logging.getLogger().setLevel(logging.INFO)
conf = rtde_config.ConfigFile(config_filename) #configure the xml file
state_names, state_types = conf.get_recipe('state') #return list of variable names and types associated to state in xml (receive from the robot)
setp_names, setp_types = conf.get_recipe('setp') #return list of variable names and types associated to setp in xml (send to the robot)
watchdog_names, watchdog_types = conf.get_recipe('watchdog')

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT) #create instance con by constucting a connection with RTDE class
con.connect()#initialize RTDE connection to host
con.get_controller_version()#get controller version

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

#Resetting the variables that should be sent to the robot controller
setp.input_double_register_0 = 0 #wrench1
setp.input_double_register_1 = 0 #wrench2
setp.input_double_register_2 = 0 #wrench3
setp.input_double_register_3 = 0 #wrench4
setp.input_double_register_4 = 0 #wrench5
setp.input_double_register_5 = 0 #wrench6
setp.input_int_register_30 = 0 #flag to stop force mode
watchdog.input_int_register_0 = 0 #The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog

def list_to_setp_wrench(setp, list):
    setp.input_double_register_0 = list[0]
    setp.input_double_register_1 = list[1]
    setp.input_double_register_2 = list[2]
    setp.input_double_register_3 = list[3]
    setp.input_double_register_4 = list[4]
    setp.input_double_register_5 = list[5]
    return setp

# start data synchronization
if not con.send_start():
    sys.exit()
def export_tcp_pose_x_to_csv(list):

    file_to_output = open('TCP_pose_x.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_tcp_pose_y_to_csv(list):

    file_to_output = open('TCP_pose_y.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_tcp_speed_to_csv(list):

    file_to_output = open('TCP_speed.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_tcp_force_x_to_csv(list):

    file_to_output = open('TCP_force_x.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_tcp_force_y_to_csv(list):

    file_to_output = open('TCP_force_y.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_resultant_force_to_csv(list):
    file_to_output = open('resultant_force.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_resultant_torque_to_csv(list):
    file_to_output = open('resultant_torque.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_resultant_impedance_to_csv(list):
    file_to_output = open('resultant_impedance.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_tcp_velocity_x_to_csv(list):

    file_to_output = open('TCP_velocity_x.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_tcp_velocity_y_to_csv(list):

    file_to_output = open('TCP_velocity_y.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_execution_time_to_csv(list):

    file_to_output = open('Execution_time.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_impedance_forcex_to_csv(list):
    file_to_output = open('impedance_x.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_impedance_forcey_to_csv(list):
    file_to_output = open('impedance_y.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_torque_x_to_csv(list):
    file_to_output = open('torque_x.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_torque_y_to_csv(list):
    file_to_output = open('torque_y.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_time_to_csv(list):
    file_to_output = open('time.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def loop_rtde():
    current_time = 0
    Trajectory = False
    while keep_running and current_time <= program_time:
        state = con.receive()
        if Trajectory == True:
            setp.input_int_register_30 = 1  # Flag to terminate
            print("Flag is activated, Pronation point is reached")
            break
        if state.output_int_register_0 == 0:
            t2 = time.time()
            current_time = t2 - start_time
            damping = [damping_tx, damping_ty, damping_tz, damping_Rx, damping_Ry, damping_Rz]
            damping = np.diag(damping)  # damping matrix
            t_prev = 0
            i = 0
            t_start = time.time()
            poses = [start_pose, desired_pose_1] #, desired_pose_1, start_pose]
            if state is None:
                break
            traj = get_Traj(poses, trajectory_time, time_samples)  # trajectory S->1
            wrenchCalculation = WrenchForImpedance([stiffness_tx, stiffness_ty, stiffness_tz, stiffness_Rx, stiffness_Ry,
                                                    stiffness_Rz])
            while i < (len(poses) - 1) * N:
                state = con.receive()
                state_scaled = pose_scal(state.actual_TCP_pose)
                t_current = time.time() - t_start
                print(f"The current time is {t_current}")
                dt = t_current - t_prev
                if state.runtime_state > 1:
                    setp.input_int_register_30 = 0  # reset flag
                    Pose = get_pose(traj[i])
                    current_pose = state_scaled
                    current_speed = state.actual_TCP_speed
                    wrench_k = np.array(wrenchCalculation.WrenchCalculation(Pose, current_pose))
                    wrench_d = dampingWrench(current_speed, damping)
                    wrench = wrench_k - wrench_d
                    # wrench = wrench_k
                    wrench = np.ravel(wrench).tolist()
                    print(f"The wrench is {wrench}")
                    # print(current_time)
                    while dt < 0.005:
                        t_current = time.time() - t_start
                        dt = t_current - t_prev
                    t_prev = t_current
                    list_to_setp_wrench(setp, wrench)

                    ####Position x-direction####
                    tcp_position_x.append(state.actual_TCP_pose[0])
                    export_tcp_pose_x_to_csv(tcp_position_x)

                    ####Position y-direction####
                    tcp_position_y.append(state.actual_TCP_pose[1])
                    export_tcp_pose_y_to_csv(tcp_position_y)

                    ####Velocity x-direction####
                    velocity_x = state.actual_TCP_speed[0]
                    if test_vel_x_range(velocity_x):
                        current_time_velocity_x = [test_vel_x_range(velocity_x), current_time]
                        velocity_time_x.append(current_time_velocity_x)
                    test_vel_x_range(velocity_x)  ##Print values not in range in real-time
                    tcp_velocity_x.append(state.actual_TCP_speed[0])
                    export_tcp_velocity_x_to_csv(tcp_velocity_x)

                    ####Velocity y-direction####
                    velocity_y = state.actual_TCP_speed[1]
                    if test_vel_y_range(velocity_y):
                        current_time_velocity_y = [test_vel_y_range(velocity_y), current_time]
                        velocity_time_y.append(current_time_velocity_y)
                    test_vel_y_range(velocity_y)  ##Print values not in range in real-time
                    tcp_velocity_y.append(state.actual_TCP_speed[1])
                    export_tcp_velocity_y_to_csv(tcp_velocity_y)

                    ####Force x-direction####
                    force_x = state.actual_TCP_force[0]
                    if test_force_x_range(force_x):  ##if not in the range put it in 2D list
                        current_time_force_x = [test_force_x_range(force_x),
                                                current_time]  ##put in a changing variable 2D-list with forces and corresponding time
                        force_time_x.append(current_time_force_x)  ## 2D-list with forces and corresponding time
                    test_force_x_range(force_x)  ##Print values not in range
                    tcp_force_x.append(state.actual_TCP_force[0])  ##append in a list for export
                    export_tcp_force_x_to_csv(tcp_force_x)  ##export list to csv file

                    ####Force y-direction####
                    force_y = state.actual_TCP_force[1]
                    if test_force_y_range(force_y):
                        current_time_force_y = [test_force_y_range(force_y), current_time]
                        force_time_y.append(current_time_force_y)
                    test_force_y_range(force_y)  ##Print values not in range in real-time
                    tcp_force_y.append(state.actual_TCP_force[1])
                    export_tcp_force_y_to_csv(tcp_force_y)
                    # print(f"The actual TCP Force is {state.actual_TCP_force[0]}")

                    #### Resultant force ####
                    fx = pow(state.actual_TCP_force[0], 2)
                    fy = pow(state.actual_TCP_force[1], 2)
                    sum_force = fx + fy
                    result_force = math.sqrt(sum_force)
                    if test_resultant_force_range(result_force):
                        current_time_resultant_force = [test_resultant_force_range(result_force), current_time]
                        resultant_force_time.append(current_time_resultant_force)
                    test_resultant_force_range(result_force)  ##Print values not in range in real-time
                    resultant_force.append(result_force)
                    export_resultant_force_to_csv(resultant_force)

                    #### torque x-direction ####
                    torqx = state.actual_TCP_force[0] / 8.1
                    if test_torque_x_range(torqx):
                        current_time_torque_x = [test_torque_x_range(torqx), current_time]
                        torque_time_x.append(current_time_torque_x)
                    test_torque_x_range(torqx)  ##Print values not in range in real-time
                    torque_x.append(state.actual_TCP_force[3])
                    export_torque_x_to_csv(torque_x)

                    #### torque y-direction ####
                    torqy = state.actual_TCP_force[1] / 8.1
                    if test_torque_y_range(torqy):
                        current_time_torque_y = [test_torque_y_range(torqy), current_time]
                        torque_time_y.append(current_time_torque_y)
                    test_torque_y_range(torqy)  ##Print values not in range in real-time
                    torque_y.append(state.actual_TCP_force[4])
                    export_torque_y_to_csv(torque_y)

                    #### Resultant Torque ####
                    tx = pow(state.actual_TCP_force[3], 2)
                    ty = pow(state.actual_TCP_force[4], 2)
                    sum_torque = tx + ty
                    result_torque = math.sqrt(sum_torque)
                    if test_resultant_torque_range(result_torque):
                        current_time_resultant_torque = [test_resultant_torque_range(result_torque), current_time]
                        resultant_torque_time.append(current_time_resultant_torque)
                    test_resultant_torque_range(result_torque)  ##Print values not in range in real-time
                    resultant_torque.append(result_torque)
                    export_resultant_torque_to_csv(resultant_torque)

                    ####Impedance X-direction####
                    if state.actual_TCP_speed[0] == 0:
                        impedancex = state.actual_TCP_force[0]
                        # if test_impedance_x_range(impedancex):
                        #     current_time_imp_x = [test_impedance_x_range(impedancex), current_time]
                        #     impedance_time_x.append(current_time_imp_x)
                        test_impedance_x_range(impedancex)  ##Print values not in range in real-time
                        impedance_x.append(state.actual_TCP_force[0] / 100)
                        export_impedance_forcex_to_csv(impedance_x)
                    else:
                        impedancex = state.actual_TCP_force[0] / state.actual_TCP_speed[0]
                        # if test_impedance_x_range(impedancex):
                        #     current_time_imp_x = [test_impedance_x_range(impedancex), current_time]
                        #     impedance_time_x.append(current_time_imp_x)
                        test_impedance_x_range(impedancex)
                        impedance_x.append(state.actual_TCP_force[0] / 0.8 * state.actual_TCP_speed[0])
                        export_impedance_forcex_to_csv(impedance_x)

                    ####Impedance Y-direction####
                    if state.actual_TCP_speed[1] == 0:
                        impedancey = state.actual_TCP_force[1]
                        # if test_impedance_y_range(impedancey):
                        #     current_time_imp_y = [test_impedance_y_range(impedancey), current_time]
                        #     impedance_time_y.append(current_time_imp_y)
                        test_impedance_y_range(impedancey)
                        impedance_y.append(state.actual_TCP_force[1] / 100)
                        export_impedance_forcey_to_csv(impedance_y)
                    else:
                        impedancey = state.actual_TCP_force[1] / state.actual_TCP_speed[1]
                        # if test_impedance_y_range(impedancey):
                        #     current_time_imp_y = [test_impedance_y_range(impedancey), current_time]
                        #     impedance_time_y.append(current_time_imp_y)
                        test_impedance_y_range(impedancey)  ##Print values not in range in real-time
                        impedance_y.append(state.actual_TCP_force[1] / 0.8 * state.actual_TCP_speed[1])
                        export_impedance_forcey_to_csv(impedance_y)

                    #### Resultant Impedance ####
                    Ix = pow(state.actual_TCP_force[0]/ state.actual_TCP_speed[0], 2)
                    Iy = pow(state.actual_TCP_force[1]/ state.actual_TCP_speed[1], 2)
                    sum_impedance = Ix + Iy
                    result_impedance = math.sqrt(sum_impedance)
                    resultant_impedance.append(result_impedance)
                    export_resultant_impedance_to_csv(resultant_impedance)
                    #### time ####
                    execution_time.append(t_current)
                    export_execution_time_to_csv(execution_time)
                    # print(f"The execution time is {current_time}")
                    con.send(setp)
                    i += 1
                else:
                    setp.input_int_register_30 = 1  # Flag to terminate
                    print("Flag1 activated")
                    break
            Trajectory = True
            setp.input_int_register_30 = 1  # Flag to terminate
            con.send(setp)

        # kick watchdog
        con.send(watchdog)

if __name__ == "__main__":
    time_samples = 0.008  # 8 ms as a sampling time for the trajectory
    trajectory_time = 13 # 4 sec for the entire trajectory
    N = int(trajectory_time // time_samples)  # Number of samples, 0.008 sec sampling Time
    start_pose = [-0.39050, 0.01727, 0.35750, 0.393, 2.182, 1.048]
    # desired_pose_1 = [-0.30442, -0.02504, 0.35333, 0.961, 1.946, 0.822]
    desired_pose_1 = [-0.21856, -0.10774, 0.35580, 1.313, 1.607, 0.560]
    stiffness_tx = 5000  # stiffness for the translation along the x-Axis
    stiffness_ty = 5000  # stiffness for the translation along the y-Axis
    stiffness_tz = 5000  # stiffness for the translation along the z-Axis
    stiffness_Rx = 100  # stiffness for the rotation about the x-Axis
    stiffness_Ry = 100  # stiffness for the rotation about the y-Axis
    stiffness_Rz = 100  # stiffness for the rotation about the z-Axis
    damping_tx = 40  # damping for the translation along the x-Axis
    damping_ty = 40  # damping for the translation along the y-Axis
    damping_tz = 40  # damping for the translation along the z-Axis
    damping_Rx = 1  # damping for the rotation about the x-Axis
    damping_Ry = 1  # damping for the rotation about the y-Axis
    damping_Rz = 1  # damping for the rotation about the z-Axis
    # wrenchCalculation1 = WrenchForImpedance([1000, 1000, 1000, 0, 0, 0])  # unstable
    # wrenchCalculation1 = WrenchForImpedance([1500, 1500, 1500, 0, 0, 0])  # weaker in x-y
    # wrenchCalculation1 = WrenchForImpedance([2000, 2000, 2000, 0, 0, 0])  # weak stable
    # wrenchCalculation1 = WrenchForImpedance([2500, 2500, 2500, 0, 0, 0]) #weak in x-y
    # wrenchCalculation1 = WrenchForImpedance([3000, 3000, 3000, 50, 50, 50]) #stable in x-y ,try it with damping 0.01/0.04/0.05
    # wrenchCalculation1 = WrenchForImpedance([4000, 4000, 4000, 0, 0, 0])  #not stable in x-y
    # wrenchCalculation1 = WrenchForImpedance([5000, 5000, 5000, 0, 0, 0])  # stable stiff in x-y
    # wrenchCalculation1 = WrenchForImpedance([6000, 6000, 6000, 0, 0, 0])  # very stiff stable  in x-y
    start_time = time.time()
    loop_rtde()
    ### Plot ###
    plot_TCP_pose_x_time()
    plot_TCP_pose_y_time()
    plot_TCP_force_x_time()
    plot_TCP_force_y_time()
    plot_torque_x_time()
    plot_torque_y_time()
    plot_resultant_force_time()
    plot_resultant_torque_time()
    plot_resultant_impedance_time()
    plot_velocity_x_time()
    plot_velocity_y_time()
    plot_Impedance_x_time()
    plot_Impedance_y_time()

    ### display data not in the range at the specified time ###
    end_test_force_x_range(force_time_x)
    end_test_force_y_range(force_time_y)
    end_test_vel_x_range(velocity_time_x)
    end_test_vel_y_range(velocity_time_y)
    end_test_torque_x_range(torque_time_x)
    end_test_torque_y_range(torque_time_y)
    end_test_resultant_force_range(resultant_force_time)
    end_test_resultant_torque_range(resultant_torque_time)

    print("Program finished")

# con.send_pause()
# con.disconnect()
