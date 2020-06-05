# ROS audit image

A ros package to audit image of camera.


# Usage

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
bc5ed4ed5a36def9895db455718f11c1835456f81a27545524197cd1
29.3572819233sec
```

## Generated image

![bc5ed4ed5a36def9895db455718f11c1835456f81a27545524197cd1](https://user-images.githubusercontent.com/6661165/83828732-537f5c80-a71c-11ea-9812-2bf76f1a48e7.png)


## License

[Apache License 2.0](/LICENSE)

## Copyright
Copyright (c) 2020 TIS Inc.
