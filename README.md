# robot-security-2020-fukushima
The ROS & FIWARE softwares for building a secure robot system.

## Description
This repository contains the ROS & FIWARE softwares for building secure robotic systems.  
These softwares were developed by a subsidized project of the robot industries in Fukushima Prefecture.

このリポジトリには、セキュアなロボットシステムを構築するためのROSとFIWAREのソフトウェアが格納されています。  
これらのソフトウェアは、福島県の令和２年度ロボット関連産業基盤強化事業によって開発されました。

## Softwares
### fiware
* [Route Planner](/fiware/route-planner/)
    * A static route planner that gives a command list to the autonomous mobile robot.
* [Geo fence Checker](/fiware/security-geo-fence/)
    * A security geo-fence checker that detects when the autonomous mobile robot has gone away from the defined area.
* [Stack Checker](/fiware/security-stuck-checker/)
    * A security stack checker for detecting anomalous stucks of the autonomous mobile robot.
* [Schema Definitions](/fiware/security-schema-json/)
    * Schema definitions of json messages sent from the autonomous mobile robot.
### ros
* [ROS client](/ros/amqp-ros-client/)
    * A ROS package that consumes messages from FIWARE and publishes them to the autonomous mobile robot, and that subscribes messages from the robot and produces them to FIWARE.
* [Alert Player](/ros/ros_alert_player)
    * A ROS package that emits warning sounds from the autnomous mobile robot.
* [Audit Image Generator](/ros/ros_audit_image)
    * A ROS package that generates peripheral images for auditing and sends them to the management platform.
### secure\_box
* [Secure box](/secure_box/secure_box/)
    * A program that unlocks the secure box when a registered NFC card is touched.

## License
[Apache License 2.0](/LICENSE)

## Copyright
Copyright (c) 2021 [TIS Inc.](https://www.tis.co.jp/)

