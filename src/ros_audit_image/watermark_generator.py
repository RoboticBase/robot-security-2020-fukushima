# coding: utf-8
from PIL import Image, ImageDraw, ImageFont
import rospy
import os


class WatermarkGenerator:
    def __init__(self):
        self.__params = rospy.get_param('~')
        self.output_path = self.__params['watermark_image']['output_path']
        font_size = self.__params['watermark_image']['font_size']
        font_file_path = self.__params['watermark_image']['font_file_path']
        self.font = ImageFont.truetype(font=font_file_path,
                                       size=font_size)
        self.opacity = self.__params['watermark_image']['text_opacity']
        text_color = self.__params['watermark_image']['text_color']
        self.color = (text_color['red'], text_color['green'], text_color['blue'])
        self.quality = self.__params['watermark_image']['quality']

    def generate(self, image_path, text):
        self.image = Image.open(image_path).convert('RGBA')
        self.image_path = image_path
        text_layer = Image.new('RGBA', self.image.size, (255, 255, 255, 0))
        draw = ImageDraw.Draw(text_layer)
        textw, texth = draw.textsize(text, font=self.font)
        draw.text(((self.image.width - textw) / 2, (self.image.height - texth) / 2),
                  text, font=self.font, fill=self.color + (self.opacity,))
        watermarked_image = Image.alpha_composite(self.image, text_layer)
        watermarked_image.save(os.path.join(self.output_path, text),
                               'png', quality=self.quality, optimize=True)
