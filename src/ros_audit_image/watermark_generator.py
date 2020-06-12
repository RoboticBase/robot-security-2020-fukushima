# coding: utf-8
from PIL import Image, ImageDraw, ImageFont
import rospy
import os
import uuid


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
        self.output_width = self.__params['watermark_image']['output_width']

    def generate(self, image_path, text):
        self.image_path = image_path
        self.image = Image.open(image_path).convert('RGBA')
        self.image = self.scale_to_width(self.image, self.output_width)
        text_layer = Image.new('RGBA', self.image.size, (255, 255, 255, 0))
        draw = ImageDraw.Draw(text_layer)
        textw, texth = draw.textsize(text, font=self.font)
        draw.text(((self.image.width - textw) / 2, (self.image.height - texth) / 2),
                  text, font=self.font, fill=self.color + (self.opacity,))
        watermarked_image = Image.alpha_composite(self.image, text_layer)
        path = '{}.png'.format(os.path.join(self.output_path, str(uuid.uuid4())))
        watermarked_image.save(path,
                               'png', quality=self.quality, optimize=True)
        return path

    def scale_to_width(self, img, width):
        height = round(img.height * width / img.width)
        return img.resize((width, int(height)))
