from rtde_control import RTDEControlInterface as RTDEControl
import time

rtde_c = RTDEControl("127.0.0.1")

# Parameters
acceleration = 0.5
dt = 1.0/500  # 2ms
joint_q = [-1.54, -1.83, -2.28, -0.59, 1.60, 0.023]
joint_speed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Move to initial joint position with a regular moveJ
rtde_c.moveJ(joint_q)

# Execute 500Hz control loop for 2 seconds, each cycle is 2ms
for i in range(1000):
    start = time.time()
    rtde_c.speedJ(joint_speed, acceleration, dt)
    joint_speed[0] += 0.0005
    joint_speed[1] += 0.0005
    end = time.time()
    duration = end - start
    if duration < dt:
        time.sleep(dt - duration)

rtde_c.speedStop()
rtde_c.stopScript()
