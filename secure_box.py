import binascii
import nfc
import pygame.mixer
import RPi.GPIO as GPIO
import time

PORT = 4
GPIO_HIGH_TIME = 0.5
VOICE_FILE_PATH = 'voice/speech_20200710023858074.mp3'


class CardReader(object):
    def on_connect(self, tag):
        print('【 Touched 】')
        print(tag)

        self.idm = binascii.hexlify(tag._nfcid)
        print("IDm : " + str(self.idm))
        GPIO.output(PORT, GPIO.HIGH)
        time.sleep(GPIO_HIGH_TIME)
        GPIO.output(PORT, GPIO.LOW)
        pygame.mixer.music.play()

        return True

    def read_id(self):
        clf = nfc.ContactlessFrontend('usb')
        try:
            clf.connect(rdwr={'on-connect': self.on_connect})
        finally:
            clf.close()


if __name__ == '__main__':
    cr = CardReader()
    pygame.mixer.init()
    pygame.mixer.music.load(VOICE_FILE_PATH)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PORT, GPIO.OUT)
    while True:
        try:
            print('Please Touch')
            cr.read_id()
            print('【 Released 】')
        except KeyboardInterrupt:
            print('test')
            GPIO.cleanup()
            break
