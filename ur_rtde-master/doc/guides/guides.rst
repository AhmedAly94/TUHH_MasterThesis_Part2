******
Guides
******
This section contains guides for how to use the ur_rtde interface.

.. _realtime-setup-guide:

Realtime Setup Guide
====================
Not done yet.

.. _use-with-matlab:

Use with MATLAB
===============
MATLAB supports calling python library functions, please see
`this <https://se.mathworks.com/help/matlab/getting-started-with-python.html>`_ site for more information.

Here is an example of receiving the actual joint and tcp pose from the robot, and moving the robot
to some pre-defined cartesian position in MATLAB:

.. code-block:: matlab

    import py.rtde_receive.RTDEReceiveInterface
    import py.rtde_control.RTDEControlInterface

    rtde_r = RTDEReceiveInterface("localhost");
    rtde_c = RTDEControlInterface("localhost");

    actual_q = rtde_r.getActualQ();
    actual_tcp_pose = rtde_r.getActualTCPPose();

    % Convert to MATLAB array of double
    actual_q_array = cellfun(@double, cell(actual_q));
    actual_tcp_pose_array = cellfun(@double, cell(actual_tcp_pose));

    actual_q_array
    actual_tcp_pose_array

    position1 = [-0.343, -0.435, 0.50, -0.001, 3.12, 0.04];
    position2 = [-0.243, -0.335, 0.20, -0.001, 3.12, 0.04];

    rtde_c.moveL(position1);
    rtde_c.moveL(position2);
    rtde_c.stopRobot();
    clear

.. warning::
    Please notice, it is very important to include the 'clear' command and the end of execution, otherwise the ur_rtde
    threads will continue run in the background and you would not be able to execute the code again until the environment
    has been cleared.

.. note::
    Currently using the ur_rtde interface has only been tested with MATLAB R2019b using Python 2.7, since this seems
    to be the default interpreter of MATLAB R2019b. However, it should also work with Python 3.x


.. _use-with-robotiq-gripper:

Use with Robotiq Gripper
========================
There are currently 3 ways of using a Robotiq gripper with ur_rtde:

* **Option 1**: (Sending the robotiq preamble + function to be executed)

You can send the robotiq preamble script together with the function you want to run, using the
sendCustomScriptFunction() of the rtde_control interface. Unfortunately you have to send the preamble with
the gripper api functions everytime, which does give a bit of delay. You can download the preamble for
use with Python here: `robotiq_preamble.py <https://sdurobotics.gitlab.io/ur_rtde/_static/robotiq_preamble.py>`_,
and a python interface for using the robotiq gripper this way here:
`robotiq_gripper_control.py <https://sdurobotics.gitlab.io/ur_rtde/_static/robotiq_gripper_control.py>`_.

Example of this method:

.. code-block:: python

    from robotiq_gripper_control import RobotiqGripper
    from rtde_control import RTDEControlInterface
    import time

    rtde_c = RTDEControlInterface("<ROBOT_IP>")
    gripper = RobotiqGripper(rtde_c)

    # Activate the gripper and initialize force and speed
    gripper.activate()  # returns to previous position after activation
    gripper.set_force(50)  # from 0 to 100 %
    gripper.set_speed(100)  # from 0 to 100 %

    # Perform some gripper actions
    gripper.open()
    gripper.close()
    time.sleep(1)
    gripper.open()
    gripper.move(10)  # mm

    # Stop the rtde control script
    rtde_c.stopRobot()

.. admonition:: Pros
  :class: tip

    * Does not require any UR Cap to be installed.

.. admonition:: Cons
  :class: error

    * Slow execution, since the preamble is transmitted each time.
    * Simultaneous robot movements is not possible (since the rtde_control script is interrupted)

* **Option 2**: (Using the RS485 UR Cap)

Download the RS485 UR cap from here
`rs485-1.0.urcap <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/raw/master/ur_robot_driver/resources/rs485-1.0.urcap>`_,
install it on the robot and remember to remove the Robotiq_Grippers UR Cap as
these two cannot function together. It does not work with the Robotiq_Grippers UR Cap since this cap occupies the
RS485 port all the time.

You can then use the tool_communication script for making the robotiq serial port
available on your desktop. (eg. /tmp/ttyUR). Finally use a modbus RTU based driver to communicate through the serial
port. Alternatively you can avoid running the tool_communication script and just communicate directly to the socket at
the port specified in the RS485 cap (default is *54321*).

.. admonition:: Pros
  :class: tip

    * Allows you to communicate to the RS485 port on the robot.
    * This approach can be used with different grippers, that uses the UR RS485 connection.
    * Fast communication.

.. admonition:: Cons
  :class: error

    * Does not work together with the official Robotiq_Grippers UR Cap.
    * Requires you to install a UR Cap.

* **Option 3**: (Communicating directly with Robotiq_grippers UR Cap port)

A robotiq gripper can be controlled through a port (*63352*) that is opened by the Robotiq_grippers UR Cap. This
port provides direct communication to the gripper. So you simply connect to the robot IP at this port and you
can command it using the Robotiq string commands, see the 'Control' section of this
`manual <https://assets.robotiq.com/website-assets/support_documents/document/Hand-E_Manual_UniversalRobots_PDF_20191219.pdf>`_.

*C++*:

ur_rtde includes a C++ interface for robotiq grippers implemented by (Uwe Kindler). See the API here:
:ref:`Robotiq Gripper API <robotiq-gripper-api>`, and the example here: :ref:`Robotiq Gripper Example <robotiq-gripper-example>`

*Python*:

You can download an example Python class for controlling the gripper using this method here: `robotiq_gripper.py <https://sdurobotics.gitlab.io/ur_rtde/_static/robotiq_gripper.py>`_.
This class was implemented by Sam (Rasp) thanks! The class can be used like this:

.. code-block:: python

    import robotiq_gripper
    import time

    ip = "127.0.0.1"

    def log_info(gripper):
        print(f"Pos: {str(gripper.get_current_position()): >3}  "
              f"Open: {gripper.is_open(): <2}  "
              f"Closed: {gripper.is_closed(): <2}  ")

    print("Creating gripper...")
    gripper = robotiq_gripper.RobotiqGripper()
    print("Connecting to gripper...")
    gripper.connect(ip, 63352)
    print("Activating gripper...")
    gripper.activate()

    print("Testing gripper...")
    gripper.move_and_wait_for_pos(255, 255, 255)
    log_info(gripper)
    gripper.move_and_wait_for_pos(0, 255, 255)
    log_info(gripper)


.. admonition:: Pros
  :class: tip

    * Works with Robotiq_grippers UR Cap.
    * Fast communication.

.. admonition:: Cons
  :class: error

    * You might not be able to leverage existing robotiq drivers, depending on implementation.

My current recommendation is to use **Option 3** for controlling a Robotiq gripper, and if that does not suit your needs
go with **Option 2**. **Option 1** should only be used as a last resort.
