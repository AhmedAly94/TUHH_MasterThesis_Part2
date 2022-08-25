# from examples.control_curved_path import force_time_x

def end_test_force_x_range(force_time_x):

    for i in range(len(force_time_x)):
        for j in range(len(force_time_x[i]) - 1):
            print(f" force {force_time_x[i][0]} in x is not in the range at time {force_time_x[i][1]}")

def end_test_force_y_range(force_time_y):

    for i in range(len(force_time_y)):
        for j in range(len(force_time_y[i]) - 1):
            print(f" force {force_time_y[i][0]} in y is not in the range at time {force_time_y[i][1]}")

def end_test_vel_x_range(velocity_time_x):

    for i in range(len(velocity_time_x)):
        for j in range(len(velocity_time_x[i]) - 1):
            print(f" velocity {velocity_time_x[i][0]} in x is not in the range at time {velocity_time_x[i][1]}")

def end_test_vel_y_range(velocity_time_y):

    for i in range(len(velocity_time_y)):
        for j in range(len(velocity_time_y[i]) - 1):
            print(f" velocity {velocity_time_y[i][0]} in y is not in the range at time {velocity_time_y[i][1]}")

def end_test_torque_x_range(torque_time_x):

    for i in range(len(torque_time_x)):
        for j in range(len(torque_time_x[i]) - 1):
            print(f" torque {torque_time_x[i][0]} in x is not in the range at time {torque_time_x[i][1]}")

def end_test_torque_y_range(torque_time_y):

    for i in range(len(torque_time_y)):
        for j in range(len(torque_time_y[i]) - 1):
            print(f" torque {torque_time_y[i][0]} in y is not in the range at time {torque_time_y[i][1]}")

def end_test_resultant_force_range(resultant_force_time):

    for i in range(len(resultant_force_time)):
        for j in range(len(resultant_force_time[i]) - 1):
            print(f" Resultant Force {resultant_force_time[i][0]} not in the range at time {resultant_force_time[i][1]}")

def end_test_resultant_torque_range(resultant_torque_time):

    for i in range(len(resultant_torque_time)):
        for j in range(len(resultant_torque_time[i]) - 1):
            print(f" Resultant Torque {resultant_torque_time[i][0]} not in the range at time {resultant_torque_time[i][1]}")
###############################################################


