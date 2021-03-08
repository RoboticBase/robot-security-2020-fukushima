#!/usr/bin/python
# -*- coding: utf-8 -*-
import hashlib
import datetime
import rospy
import pytz
import time
from eams_msgs.msg import ImageInfo
from sensor_msgs.msg import Image as ImageMSG
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from watermark_generator import WatermarkGenerator
from azure_blob_storage_controller import AzureBlobStorageController
import message_filters_py3 as message_filters

NODE_NAME = 'audit_image'


class ImageInformation:
    def __init__(self, params, image_info_pub):
        self.robot_id = params['robot_id']
        self.image_info_pub = image_info_pub

    def callback(self, position, compass, image_raw):
        rospy.loginfo('subscribe telemetries , position=%s, compass=%s', position, compass)
        rospy.loginfo("generate start")
        now = datetime.datetime.now(pytz.timezone('Asia/Tokyo')).strftime('%Y-%m-%d %H:%M:%S')
        location = '_'.join([str(position.latitude), str(position.longitude)])
        start = time.time()
        watermark_text = ','.join([self.robot_id, now, location, str(compass.data)])

        hashed_watermark_text = hashlib.sha224(watermark_text.encode()).hexdigest()
        rospy.loginfo(hashed_watermark_text)
        rospy.loginfo(watermark_text)
        generator = WatermarkGenerator()
        file_name = generator.generate(image_raw,
                                       '{}\n{}'.format(hashed_watermark_text, watermark_text))
        controller = AzureBlobStorageController(generator.output_path)
        result = controller.upload(file_name)
        rospy.loginfo(result)
        message = self.make_message(position, compass, now, hashed_watermark_text, file_name)
        self.image_info_pub.publish(message)

        elapsed_time = time.time() - start
        rospy.loginfo("elapsed_time: " + str(elapsed_time) + "[sec]")
        rospy.loginfo("generate finish")
        rospy.loginfo("===========================")

    def make_message(self, position, compass, time, hashed_watermark_text, file_path):
        image_info = ImageInfo()
        image_info.header.stamp = rospy.Time.now()
        image_info.header.frame_id = 'image_info'
        image_info.lat = str(position.latitude)
        image_info.lng = str(position.longitude)
        image_info.yaw = str(compass.data)
        image_info.time = time
        image_info.hash = hashed_watermark_text
        image_info.path = file_path
        return image_info


def main():
    try:
        rospy.init_node(NODE_NAME, anonymous=True, disable_signals=True)
        params = rospy.get_param('~')
        image_info_pub = rospy.Publisher(params['topic']['image_info'], ImageInfo, queue_size=1)

        image_information = ImageInformation(params, image_info_pub)
        position_sub = message_filters.Subscriber(params['topic']['position'], NavSatFix)
        compass_sub = message_filters.Subscriber(params['topic']['compass'], Float64)
        image_raw_sub = message_filters.Subscriber(params['topic']['image_raw'], ImageMSG)

        slop = float(params['thresholds']['slop_ms'])/1000.0
        ts = message_filters.ApproximateTimeSynchronizer([position_sub, compass_sub, image_raw_sub],
                                                         10, slop, allow_headerless=True)
        ts.registerCallback(image_information.callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
