import binascii
import nfc
import pygame.mixer
import RPi.GPIO as GPIO
import time

PORT = 4
GPIO_HIGH_TIME = 0.5
VOICE_FILE_PATH = 'voice/speech_20200710023858074.mp3'
ID_LIST_PATH = 'data/id_list.csv'


class SecurityBox:
    def __init__(self, id_list):
        self.id_list = id_list

    def on_connect(self, tag):
        print('【 Touched 】')
        print(tag)

        self.idm = binascii.hexlify(tag._nfcid)
        print("IDm : " + self.idm.decode())
        if self.idm.decode() in self.id_list:
            self.open()

        return True

    def open(self):
        GPIO.output(PORT, GPIO.HIGH)
        time.sleep(GPIO_HIGH_TIME)
        GPIO.output(PORT, GPIO.LOW)
        pygame.mixer.music.play()

    def read_id(self):
        clf = nfc.ContactlessFrontend('usb')
        try:
            clf.connect(rdwr={'on-connect': self.on_connect})
        finally:
            clf.close()


if __name__ == '__main__':
    id_list = []
    with open(ID_LIST_PATH, 'r') as f:
        id_list = [id.strip() for id in f.readlines()]
    security_box = SecurityBox(id_list)
    pygame.mixer.init()
    pygame.mixer.music.load(VOICE_FILE_PATH)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PORT, GPIO.OUT)
    while True:
        try:
            print('Please Touch')
            security_box.read_id()
            print('【 Released 】')
        except KeyboardInterrupt:
            GPIO.cleanup()
            break
