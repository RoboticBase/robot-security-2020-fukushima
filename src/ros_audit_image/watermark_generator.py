# coding: utf-8
import rospy
import os
from cv_bridge import CvBridge, CvBridgeError
import cv2
import uuid


class WatermarkGenerator:
    def __init__(self):
        self.bridge = CvBridge()
        self.__params = rospy.get_param('~')
        self.output_path = self.__params['watermark_image']['output_path']
        self.font_size = self.__params['watermark_image']['font_size']
        text_color = self.__params['watermark_image']['text_color']
        self.color = (text_color['red'], text_color['green'], text_color['blue'])
        self.text_position_x = self.__params['watermark_image']['text_position']['x']
        self.text_position_y = self.__params['watermark_image']['text_position']['y']
        self.newline_size = self.__params['watermark_image']['newline_size']
        self.text_bold_size = self.__params['watermark_image']['text_bold_size']

    def generate(self, image_message, text):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
            self.put_text(cv_image, text)
            file_name = '{}.png'.format(str(uuid.uuid4()))
            path = os.path.join(self.output_path, file_name)
            cv2.imwrite(path, cv_image)
            return file_name
        except CvBridgeError as e:
            print(e)
            return ''

    def put_text(self, image, text):
        y0, dy = self.text_position_y, self.newline_size
        for i, line in enumerate(text.split('\n')):
            y = y0 + i*dy
            cv2.putText(image, line, (self.text_position_x, y), cv2.FONT_HERSHEY_SIMPLEX,
                        self.font_size, self.color, self.text_bold_size)
