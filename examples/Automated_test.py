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

minimum_impedance_x = 0
maximum_impedance_x  = 23

minimum_impedance_y = 0
maximum_impedance_y = 23


def test_force_x_range(force_x):
    try:
        assert minimum_force_x< force_x<= maximum_force_x
    except:
        print("force in x not in the range")
        return force_x

def test_force_y_range(force_y):
    try:
        assert  minimum_force_y< force_y<= maximum_force_y
    except:
        print("force in y not in the range")
        return force_y

def test_vel_x_range(velocity_x):
    try:
        assert  minimum_vel_x< velocity_x<= maximum_vel_x
    except:
        print("velocity in x not in the range")
        return velocity_x

def test_vel_y_range(velocity_y):
    try:
        assert  minimum_vel_y< velocity_y<= maximum_vel_y
    except:
        print("velocity in y not in the range")
        return velocity_y

def test_torque_x_range(torque_x):
    try:
        assert  minimum_torque_x< torque_x<= maximum_torque_x
    except:
        print("torque in x not in the range")
        return torque_x

def test_torque_y_range(torque_y):
    try:
     assert  minimum_torque_y< torque_y<= maximum_torque_y
    except:
        print("torque in y not in the range")
        return torque_y

def test_impedance_x_range(impedancex):
    try:
        assert  minimum_impedance_x< impedancex<= maximum_impedance_x
    except:
        print("impedance in x not in the range")
        return impedancex

def test_impedance_y_range(impedancey):
    try:
        assert  minimum_impedance_y< impedancey<= maximum_impedance_y
    except:
        print("impedance in y not in the range")
        return impedancey

def test_resultant_force_range(resultantforce):
    try:
        assert  minimum_resultant_force< resultantforce<= maximum_resultant_force
    except:
        print("resultant force not in the range")
        return resultantforce

def test_resultant_torque_range(resultanttorque):
    try:
        assert  minimum_resultant_torque< resultanttorque<= maximum_resultant_torque
    except:
        print("resultant torque not in the range")
        return resultanttorque