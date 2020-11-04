#!/usr/bin/python
# -*- coding: utf-8 -*-
import hashlib
import datetime
import rospy
import pytz
import time
from sensor_msgs.msg import Image as ImageMSG
from watermark_generator import WatermarkGenerator
from azure_blob_storage_controller import AzureBlobStorageController

NODE_NAME = 'audit_image'


def callback(image_message):
    # TODO: Replace all information to ros topics.
    robot_id = 'robot_01'
    now = datetime.datetime.now(pytz.timezone('Asia/Tokyo')).strftime('%Y-%m-%d %H:%M:%S')
    location = '_'.join(['0.0', '0.0'])
    pos = '_'.join(['0.0', '0.0', '0.0', '0.0'])
    print("generate start")
    start = time.time()
    watermark_text = ','.join([robot_id, now, location, pos])

    hashed_watermark_text = hashlib.sha224(watermark_text.encode()).hexdigest()
    print(hashed_watermark_text)
    print(watermark_text)
    generator = WatermarkGenerator()
    file_name = generator.generate(image_message,
                                   '{}\n{}'.format(hashed_watermark_text, watermark_text))
    controller = AzureBlobStorageController(generator.output_path)
    controller.upload(file_name)
    elapsed_time = time.time() - start
    print("elapsed_time: " + str(elapsed_time) + "[sec]")
    print("generate finish")
    print("===========================")


def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber('image_raw', ImageMSG, callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
