import csv
import numpy as np
from matplotlib import pyplot as plt

### Plotting pose ####
def plot_TCP_pose_x_time():
    y = []
    t = []
    # reading the datapoints into lists
    with open('TCP_pose_x.csv') as file:

        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                y.append(float(tmp[i]))
                # print(float(tmp[i]))

    with open('Execution_time.csv') as file:
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                t.append(float(tmp[i]))

    t = np.array(t)
    y = np.array(y)

    # plot
    plt.figure()
    plt.plot(t, y, label="pose x data in m")
    plt.title('pose x vs time')
    plt.xlabel('Time in s')
    plt.ylabel('posx Amplitude')
    plt.legend()
    plt.show()

def plot_TCP_pose_y_time():
    y = []
    t = []
    # reading the datapoints into lists
    with open('TCP_pose_y.csv') as file:

        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                y.append(float(tmp[i]))
                # print(float(tmp[i]))

    with open('Execution_time.csv') as file:
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                t.append(float(tmp[i]))

    t = np.array(t)
    y = np.array(y)

    # plot
    plt.figure()
    plt.plot(t, y, label="pose y data in m")
    plt.title('pose y vs time')
    plt.xlabel('Time in s')
    plt.ylabel('posy Amplitude')
    plt.legend()
    plt.show()

### Plotting Force ####
def plot_TCP_force_x_time():
    y = []
    t = []
    # reading the datapoints into lists
    with open('TCP_force_x.csv') as file:

        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                y.append(float(tmp[i]))
                # print(float(tmp[i]))

    with open('Execution_time.csv') as file:
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                t.append(float(tmp[i]))

    t = np.array(t)
    y = np.array(y)

    # plot
    plt.figure()
    plt.plot(t, y, label="Force x data")
    plt.title('Force x vs time')
    plt.xlabel('Time in s')
    plt.ylabel('Fx Amplitude in N')
    plt.legend()
    plt.show()

def plot_TCP_force_y_time():
    y = []
    t = []
    # reading the datapoints into lists
    with open('TCP_force_y.csv') as file:

        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                y.append(float(tmp[i]))

    with open('Execution_time.csv') as file:
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                t.append(float(tmp[i]))

    t = np.array(t)
    y = np.array(y)

    # plot
    plt.figure()
    plt.plot(t, y, label="Force y data")
    plt.title('Force y vs time')
    plt.xlabel('Time in s')
    plt.ylabel('Fy Amplitude in N')
    plt.legend()
    plt.show()

def plot_resultant_force_time():
    y = []
    t = []
    # reading the datapoints into lists
    with open('resultant_force.csv') as file:

        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                y.append(float(tmp[i]))

    with open('Execution_time.csv') as file:
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                t.append(float(tmp[i]))

    t = np.array(t)
    y = np.array(y)

    # plot
    plt.figure()
    plt.plot(t, y, label="Resultant Force")
    plt.title('Resultant Force vs time')
    plt.xlabel('Time in s')
    plt.ylabel('Resultant Force in N ')
    plt.legend()
    plt.show()

### Plotting Velocity ####
def plot_velocity_x_time():
    y = []
    t = []
    # reading the datapoints into lists
    with open('TCP_velocity_x.csv') as file:

        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                y.append(float(tmp[i]))

    with open('Execution_time.csv') as file:
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                t.append(float(tmp[i]))

    t = np.array(t)
    y = np.array(y)

    # plot
    plt.figure()
    plt.plot(t, y, label="Velocity x data")
    plt.title('Vx vs time')
    plt.xlabel('Time in s')
    plt.ylabel('Vx in m/s')
    plt.legend()
    plt.show()

def plot_velocity_y_time():
    y = []
    t = []
    # reading the datapoints into lists
    with open('TCP_velocity_y.csv') as file:

        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                y.append(float(tmp[i]))

    with open('Execution_time.csv') as file:
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                t.append(float(tmp[i]))

    t = np.array(t)
    y = np.array(y)

    # plot
    plt.figure()
    plt.plot(t, y, label="Velocity y data")
    plt.title('Vy vs time')
    plt.xlabel('Time in s')
    plt.ylabel('Velocity y in m/s')
    plt.legend()
    plt.show()

### Plotting Impedance ####
def plot_Impedance_x_time():
    y = []
    t = []
    # reading the datapoints into lists
    with open('impedance_x.csv') as file:

        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                y.append(float(tmp[i]))

    with open('Execution_time.csv') as file:
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                t.append(float(tmp[i]))

    t = np.array(t)
    y = np.array(y)

    # plot
    plt.figure()
    plt.plot(t, y, label="Impedance x data")
    plt.title('Impedance x vs time')
    plt.xlabel('Time in s')
    plt.ylabel('Zx')
    plt.legend()
    plt.show()

def plot_Impedance_y_time():
    y = []
    t = []
    # reading the datapoints into lists
    with open('impedance_y.csv') as file:

        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                y.append(float(tmp[i]))

    with open('Execution_time.csv') as file:
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                t.append(float(tmp[i]))

    t = np.array(t)
    y = np.array(y)

    # plot
    plt.figure()
    plt.plot(t, y, label="Impedance y data")
    plt.title('Impedance y vs time')
    plt.xlabel('Time in s')
    plt.ylabel('Zy')
    plt.legend()
    plt.show()

def plot_torque_x_time():
    y = []
    t = []
    # reading the datapoints into lists
    with open('torque_x.csv') as file:

        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                y.append(float(tmp[i]))

    with open('Execution_time.csv') as file:
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                t.append(float(tmp[i]))

    t = np.array(t)
    y = np.array(y)

    # plot
    plt.figure()
    plt.plot(t, y, label="Torque x")
    plt.title('torque x vs time')
    plt.xlabel('Time in s')
    plt.ylabel('Tx in N.m')
    plt.legend()
    plt.show()

def plot_torque_y_time():
    y = []
    t = []
    # reading the datapoints into lists
    with open('torque_y.csv') as file:

        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                y.append(float(tmp[i]))

    with open('Execution_time.csv') as file:
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                t.append(float(tmp[i]))

    t = np.array(t)
    y = np.array(y)

    # plot
    plt.figure()
    plt.plot(t, y, label="Torquey")
    plt.title('torque y vs time')
    plt.xlabel('Time in s')
    plt.ylabel('Ty in N.m')
    plt.legend()
    plt.show()

def plot_resultant_torque_time():
    y = []
    t = []
    # reading the datapoints into lists
    with open('resultant_torque.csv') as file:

        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                y.append(float(tmp[i]))

    with open('Execution_time.csv') as file:
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                t.append(float(tmp[i]))

    t = np.array(t)
    y = np.array(y)

    # plot
    plt.figure()
    plt.plot(t, y, label=" Resultant Torque")
    plt.title('Resultant torque vs time')
    plt.xlabel('Time in s')
    plt.ylabel('Resultant Torque in N.m')
    plt.legend()
    plt.show()

def plot_resultant_impedance_time():
    y = []
    t = []
    # reading the datapoints into lists
    with open('resultant_impedance.csv') as file:

        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                y.append(float(tmp[i]))

    with open('Execution_time.csv') as file:
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                t.append(float(tmp[i]))

    t = np.array(t)
    y = np.array(y)

    # plot
    plt.figure()
    plt.plot(t, y, label=" Resultant Impedance")
    plt.title('Resultant Impedance vs time')
    plt.xlabel('Time in s')
    plt.ylabel('Resultant Impedance in N/(m/s)')
    plt.legend()
    plt.show()