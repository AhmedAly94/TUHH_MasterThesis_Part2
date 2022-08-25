import csv
import numpy as np
import pytest

minimum_vel_x = -0.01
maximum_vel_x = 0.01

minimum_vel_y = -0.01
maximum_vel_y = 0.01

minimum_force_x = -22
maximum_force_x = 22

minimum_force_y = -22
maximum_force_y = 22

minimum_torque_x = -1.8
maximum_torque_x = 1.8

minimum_torque_y = -1.8
maximum_torque_y = 1.8

minimum_resultant_force = -22
maximum_resultant_force = 22

minimum_resultant_torque = -1.8
maximum_resultant_torque = 1.8

# minimum_impedance_x = 0
# maximum_impedance_x  = 1
#
# minimum_impedance_y = 0
# maximum_impedance_y = 1

def test_total_force_x():
    with open('TCP_force_x.csv') as file:
        count = 0
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                try:
                    assert minimum_force_x< float(tmp[i]) < maximum_force_x
                    # print("OK")
                except:
                    count = count+1
                    print(f"force {float(tmp[i])} in x not in the range ")
        print(f"*************** Out of range numbers are : {count} / {len(tmp.T)} ***************")

def test_total_force_y():
    with open('TCP_force_y.csv') as file:
        count = 0
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                try:
                    assert minimum_force_y< float(tmp[i]) < maximum_force_y
                except:
                    count = count + 1
                    print(f"force {float(tmp[i])} in y not in the range ")
        print(f"*************** Out of range numbers are : {count} / {len(tmp.T)} ***************")

def test_total_velocity_x():
    with open('TCP_velocity_x.csv') as file:
        count = 0
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                try:
                    assert minimum_vel_x< float(tmp[i]) < maximum_vel_x
                except:
                    count = count + 1
                    print(f"velocity {float(tmp[i])} in x not in the range ")
        print(f"*************** Out of range numbers are : {count} / {len(tmp.T)} ***************")

def test_total_velocity_y():
    with open('TCP_velocity_y.csv') as file:
        count = 0
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                try:
                    assert minimum_vel_y< float(tmp[i]) < maximum_vel_y
                except:
                    count = count + 1
                    print(f"velocity {float(tmp[i])} in y not in the range ")
        print(f"*************** Out of range numbers are : {count} / {len(tmp.T)} ***************")

def test_total_torque_x():
    with open('torque_x.csv') as file:
        count = 0
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                try:
                    assert minimum_torque_x< float(tmp[i]) < maximum_torque_x
                except:
                    count = count + 1
                    print(f"torque {float(tmp[i])} in x not in the range ")
        print(f"*************** Out of range numbers are : {count} / {len(tmp.T)} ***************")

def test_total_torque_y():
    with open('torque_y.csv') as file:
        count = 0
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                try:
                    assert minimum_torque_y< float(tmp[i]) < maximum_torque_y
                except:
                    count = count + 1
                    print(f"torque {float(tmp[i])} in y not in the range ")
        print(f"*************** Out of range numbers are : {count} / {len(tmp.T)} ***************")

def test_total_resultant_force():
    with open('resultant_force.csv') as file:
        count = 0
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                try:
                    assert minimum_resultant_force< float(tmp[i]) < maximum_resultant_force
                except:
                    count = count + 1
                    print(f"Resultant force {float(tmp[i])} not in the range ")
        print(f"*************** Out of range numbers are : {count} / {len(tmp.T)} ***************")

def test_total_resultant_torque():
    with open('resultant_torque.csv') as file:
        count = 0
        reader = csv.reader(file, delimiter=",")
        for row in reader:
            tmp = np.array(row)
            ltmp = len(tmp.T)
            for i in range(ltmp):
                try:
                    assert minimum_resultant_torque< float(tmp[i]) < maximum_resultant_torque
                except:
                    count = count + 1
                    print(f"Resultant torque {float(tmp[i])} not in the range ")
        print(f"*************** Out of range numbers are : {count} / {len(tmp.T)} ***************")

# def test_total_impedance_x():
#     with open('impedance_x.csv') as file:
#         count = 0
#         reader = csv.reader(file, delimiter=",")
#         for row in reader:
#             tmp = np.array(row)
#             ltmp = len(tmp.T)
#             for i in range(ltmp):
#                 try:
#                     assert minimum_impedance_x< float(tmp[i]) < maximum_impedance_x
#                 except:
#                     count = count + 1
#                     print(f"Impedance {float(tmp[i])} in x not in the range ")
#         print(f"*************** Out of range numbers are : {count} / {len(tmp.T)} ***************")
#
# def test_total_impedance_y():
#     with open('impedance_y.csv') as file:
#         count = 0
#         reader = csv.reader(file, delimiter=",")
#         for row in reader:
#             tmp = np.array(row)
#             ltmp = len(tmp.T)
#             for i in range(ltmp):
#                 try:
#                     assert minimum_impedance_y< float(tmp[i]) < maximum_impedance_y
#                 except:
#                     count = count + 1
#                     print(f"Impedance {float(tmp[i])} in y not in the range ")
#         print(f"*************** Out of range numbers are : {count} / {len(tmp.T)} ***************")



