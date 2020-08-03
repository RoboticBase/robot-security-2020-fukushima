# ROS audit image

A ros package to audit image of camera.


# Usage

Launch the camera node

```
roslaunch ros_audit_image libuvc_camera.launch
```

Testing the time to generate a watermark

```
roslaunch ros_audit_image ros_audit_image.launch
```

Generate a hash including time, odometry and location info using **hashlib.sha224** algorithm

```
root@28114219c6f6:/opt/ros_ws# roslaunch ros_audit_image ros_audit_image.launch

~~~~~
~~~~~

process[audit_image-2]: started with pid [4294]
generate start
866515a8c012dd60699b3aa49de683ca884bb328aae9c8475854ff62
robot_01,2020-08-03 14:57:43,0.0_0.0,0.0_0.0_0.0_0.0
elapsed_time: 0.0569100379944[sec]
generate finish
===========================
generate start
866515a8c012dd60699b3aa49de683ca884bb328aae9c8475854ff62
robot_01,2020-08-03 14:57:43,0.0_0.0,0.0_0.0_0.0_0.0
elapsed_time: 0.0382528305054[sec]
generate finish
===========================
generate start
866515a8c012dd60699b3aa49de683ca884bb328aae9c8475854ff62
robot_01,2020-08-03 14:57:43,0.0_0.0,0.0_0.0_0.0_0.0
elapsed_time: 0.105615854263[sec]
generate finish
===========================
...

```

## Generated image

![e098732b-c293-48ad-8200-099ddabcb2b8](https://user-images.githubusercontent.com/6661165/89149808-566dc080-d598-11ea-8dc2-6b3ff15f5431.png)


## License

[Apache License 2.0](/LICENSE)

## Copyright
Copyright (c) 2020 TIS Inc.
