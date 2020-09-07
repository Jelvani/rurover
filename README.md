# Rutgers Rover (RURover) User Guide (Under Develpoment)

RURover is an unmaned-ground-vehicke (UGV) based on the [Traxxas X-Maxx 8s](https://traxxas.com/products/landing/x-maxx/), that has been modified with the addition of wheel encoders, steering sensor, and an on-board embedded system.

## Hardware
***
A list of all additional hardware added to RURover can be found below:
- Sensors
    - [Wheel encoders (X4)](https://www.usdigital.com/products/encoders/incremental/shaft/S4T)
    - [Steering angle encoder](https://www.usdigital.com/products/encoders/absolute/magnetic/MA3)
    - [Inertial Measurement Unit (IMU)](https://www.adafruit.com/product/2472)
- Embedded System
    - [System on module, NVIDIA Jetson TX2](https://developer.nvidia.com/embedded/jetson-tx2-developer-kit)
    - [Jetson TX2 custom carrier board](https://www.uta.edu/utari/research/robotics/airborne/tutorials.php#TX2version2)
    - [Teensy 4.0 microcontroller](https://www.pjrc.com/store/teensy40.html)
- Batteries
    - [6500 mAh 4s LiPo (X2)](https://www.amazon.com/Youme-Battery-6500mAh-Traxxas-%EF%BC%8CHelicopter%EF%BC%8CAirplane/dp/B07PD2CNQM/ref=sr_1_7?dchild=1&keywords=4s+lipo+traxxas+connector&qid=1598924690&sr=8-7)

## Robot Operating System (ROS) Specifications
***

## Connecting to the platform
***
1. Choose the wifi network with SSID `rurover` and password is `rurover-ram-lab`.
2. SSH into the TX2 with `ssh rurover@10.42.0.1` and password is `12`.
