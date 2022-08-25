<div align="center">

<img width=30% src="https://gitlab.com/sdurobotics/ur_rtde/-/raw/master/doc/_static/ur_rtde_logo.png">
</div>
&nbsp;
<div align="center">

[![build status](https://gitlab.com/sdurobotics/ur_rtde/badges/master/pipeline.svg)](https://gitlab.com/sdurobotics/ur_rtde/-/pipelines/latest)
[![pypi](https://badgen.net/pypi/v/ur_rtde)](https://pypi.org/project/ur-rtde/)
[![pypi](https://gitlab.com/sdurobotics/ur_rtde/-/jobs/artifacts/master/raw/ps-5-11-1-108318.svg?job=polyscope-5.11.1.108318)](https://gitlab.com/sdurobotics/ur_rtde/-/pipelines/latest)
[![Gitter](https://badges.gitter.im/ur_rtde/community.svg)](https://gitter.im/ur_rtde/community?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge)
</div>

A C++ interface for controlling and receiving data from a UR robot using the 
[Real-Time Data Exchange (RTDE)](https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/real-time-data-exchange-rtde-guide-22229/)
 interface of the robot. The interface can also by used with python, through the provided python bindings.

### Key Features ###
 * Fast and lightweight interface for programming UR robots.
 * Uses the [Real-Time Data Exchange (RTDE)](https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/real-time-data-exchange-rtde-guide-22229/) of the robot.
 * Available on multiple platforms (Linux, Windows, macOS)
 * Can be used from C++ and Python.
 * Relies only on STL datatypes and can be used with various robot frameworks.
 * Switchable register range (FieldBus / PLC [0..23] or external clients range [24..47])
 * Use it with the UR's [ExternalControl UR Cap](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap),
    download it [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/resources/externalcontrol-1.0.5.urcap) see
    how to install it on the robot [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md). 
    In order to setup ur_rtde for using it see [here](https://sdurobotics.gitlab.io/ur_rtde/examples/examples.html#use-with-externalcontrol-ur-cap).
 * Easy to install and setup.
 
### Documentation ###
Documentation with installation and build instructions, examples and API resides at <https://sdurobotics.gitlab.io/ur_rtde/>

### Quick Install ##

#### From PPA: ####
If you are on Ubuntu, you can install ur_rtde with:

    sudo add-apt-repository ppa:sdurobotics/ur-rtde
    sudo apt-get update
    sudo apt install librtde librtde-dev

#### From PyPi: ####
If you only want to the use the Python interface, you can install ur_rtde through pip:

    pip install --user ur_rtde

:warning: Notice! Make sure your pip version >=**19.3**, otherwise the install might fail.

#### Prebuilt python wheels support matrix: ####
|   | Python 3.6 | Python 3.7 | Python 3.8 | Python 3.9 |
---------------|:----:|:----:|:-----:|:-----:|
| **Linux x86_64**        | ✅ | ✅ | ✅ | ✅ |
| **macOS Intel**         | ✅ | ✅ | ✅ | ✅ |
| **macOS Apple Silicon** | ✅ | ✅ | ✅ | ✅ | 
| **Windows 64bit**       | ✅ | ✅ | ✅ | ✅ |
| **Windows 32bit**       | :x: | :x: | :x: | :x: |

### Dependencies ###
*  [Boost](https://www.boost.org/)
*  [pybind11](https://github.com/pybind/pybind11) (Optional)

### Compatible Robots ###

*  All CB-Series from CB3/CB3.1 software 3.3
*  All e-Series

### Compatible Operating Systems ###
Currently tested on:

*  Ubuntu 16.04 (Xenial Xerus)
*  Ubuntu 18.04 (Bionic Beaver)
*  Ubuntu 20.04 (Focal Fossa)
*  macOS 10.14 (Mojave)
*  Windows 10 Pro x64

### Contact ###
If you have any questions or suggestions to the interface, feel free to contact Anders Prier Lindvig <anpl@mmmi.sdu.dk> or create an issue [here](https://gitlab.com/caro-sdu/ur_rtde/issues).
