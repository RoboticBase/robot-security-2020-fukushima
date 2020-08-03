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
bc5ed4ed5a36def9895db455718f11c1835456f81a27545524197cd1
29.3572819233sec
```

## Generated image

![bc5ed4ed5a36def9895db455718f11c1835456f81a27545524197cd1](https://user-images.githubusercontent.com/6661165/83828732-537f5c80-a71c-11ea-9812-2bf76f1a48e7.png)

### Resized image

Resize width to 600px.(600 × 450)

![3c928464-5748-44ad-b4d2-fa653363e218](https://user-images.githubusercontent.com/6661165/84484604-f2382a00-acd5-11ea-9db9-ab77297b79ce.png)

#### To shorten a generation time

```
root@28114219c6f6:/opt/ros_ws# roslaunch ros_audit_image ros_audit_image.launch

~~~~~
~~~~~

process[audit_image-2]: started with pid [5806]
generate start
4f5a699fbc18f07d148f4e75bc45c03afaa77ef95a91e1baae2d2b08
robot_01,2020-06-12 11:42:00,0.0_0.0,0.0_0.0_0.0_0.0
/opt/ros_ws/src/ros_audit_image/images/watermarked/3c928464-5748-44ad-b4d2-fa653363e218.png
0.617939949036sec
generate finish
```

## License

[Apache License 2.0](/LICENSE)

## Copyright
Copyright (c) 2020 TIS Inc.
