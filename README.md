# leapAR10
Assignment for couse 'Software Architecture for Robotics' developped with EMARO lab in UNIGE, Italy.

## Introduction

Using [Leapmotion](https://www.leapmotion.com/) as hand tracking sensor to control [AR10 humanoid robotic hand](http://www.active-robots.com/ar10-humanoid-robotic-hand) of Active Robots

## Version and Dependencies
- ROS: Kinetic
- Leap SDK: 2.3.1+31549
- python: 2.7
- pyserial: 3.3

## Installation
- append ENV variable PYTHONPATH with path to Leap SDK
- run `catkin_make` to compile the project

### Remarks:

- From Ubuntu 15.04 the *upstart* service system has been switched to *systemd*. The launch of leapd service may fail due to absence of upstart. Checkout [here](https://wiki.ubuntu.com/SystemdForUpstartUsers) for further information. Instead of using service to auto-launch the daemon, you can run `sudo /usr/sbin/leapd --run` instead in a new console.

## Usage

### ros_ar10
- `rosrun ros_ar10 ar10_servo_position_node.py` : publish servo positions
- `rosrun ros_ar10 ar10_AR10_calibrate.py` : generate calibration file

### ros_leap

### ros_leap_ar10_controller
- `roscore` : launch roscore
- `rosrun ar10 ros_AR10_calibrate.py` : run calibrate script (you may need root permission to open serial port)
- copy the generated file *ros_calibration_file* and replace the file *src/ar10/ar10/ros_calibration_file*

## Contributing instructions

1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D

## Credits

### Author
#### Student:
- YANG Qiao <qiao.yang@etu.utc.fr>

#### Mentors:
- MASTROGIOVANNI Fulvio <fulvio.mastrogiovanni@unige.it>
- Alessandro CARFI <alessandro.carfi@dibris.unige.it>
