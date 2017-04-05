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
- you can add the user to group **dialout** so that the user can use the serial port without root previlage

### Remarks:

- From Ubuntu 15.04 the *upstart* service system has been switched to *systemd*. The launch of leapd service may fail due to absence of upstart. Checkout [here](https://wiki.ubuntu.com/SystemdForUpstartUsers) for further information. Instead of using service to auto-launch the daemon, you can run `sudo /usr/sbin/leapd --run` instead in a new console.

## Usage

### ros_ar10

#### Files
- **ros\_ar10\_calibrate.py** : calibrate the ar10 robotic hand nad generate calibration file
- **ros\_ar10\_check\_calibration.py** : check ar10 robotic hand calibration and calculate errors on each joint
- **ros\_ar10\_hand\_reset.py** : set hand to open or close state
- **ros\_ar10\_servo\_pos\_publisher.py** : publish servo actual positions
- **ros\_ar10\_servo\_pos\_listener.py** : listen to servo actual positions
- **ros\_ar10\_servo\_pos\_set\_listener.py** : listen to command to control servos

#### Usage

> Parameters in common:

>>  -d DEVICE, --device DEVICE:
                          serial device

>> -l, --left:            use left hand

>> -r, --right:           use right hand

> ros\_ar10\_calibrate.py [-h] -d DEVICE (-l | -r)

>> ex:`rosrun ros_ar10 ros_ar10_calibrate.py -d /dev/ttyACM0 -r`


> ros\_ar10\_check\_calibration.py [-h] -d DEVICE (-l | -r)

>> ex: `rosrun ros_ar10 ros_ar10_check_calibration.py -d /dev/ttyACM0 -l`

> ros\_ar10\_hand\_reset.py [-h] -d DEVICE (-l | -r) (-o | -c)

>> -o, --open:            open hand

>> -c, --close:           close hand

>> ex: `rosrun ros_ar10 ros_ar10_hand_reset.py -d /dev/ttyACM0 -l -o`

> ros\_ar10\_servo\_pos\_publisher.py [-h] -d DEVICE (-l | -r)

>> ex: `rosrun ros_ar10 ros_ar10_servo_pos_publisher.py -d /dev/ttyACM0 -l`

> ros\_ar10\_servo\_pos\_listener.py [-h]  (-l | -r)

>> ex: `rosrun ros_ar10 ros_ar10_servo_pos_set_listener.py -l`

> ros\_ar10\_servo\_pos\_publisher.py [-h] -d DEVICE (-l | -r)

>> ex: `rosrun ros_ar10 ros_ar10_servo_pos_set_publisher.py -d /dev/ttyACM0 -l`

### ros_leap

#### Files
- **ros\_leap\_publisher.py** : publish leapmotion frame
- **ros\_leap\_listener.py** : listen to leapmotion frame

#### Usage

> ros\_leap\_publisher.py

>> ex:`rosrun ros_leap ros_leap_publisher.py`

> ros\_leap\_listener.py

>> ex:`rosrun ros_leap ros_leap_listener.py`

### ros_leap_ar10_controller

#### Files
- **rlac\_mapper\_calibrate.py** : generate min and max calibration 
- **rlac\_controller.py** : controller to map leapmotion and ar10 robotic hand

#### Usage
- launch leapd daemon
- launch roscore
- calibrate two ar10 robotic hand
- calibrate rlac mapper
- `rosrun ros_leap ros_leap_publisher.py`
- `rosrun ros_ar10 ros_ar10_servo_pos_set_listener.py` on two serial port for left hand and right hand
- `rosrun rlac rlac_controller.py`

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
