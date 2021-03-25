import os
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
        self.clf = nfc.ContactlessFrontend('usb')
        pygame.mixer.init()
        pygame.mixer.music.load(VOICE_FILE_PATH)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PORT, GPIO.OUT)

    def on_connect(self, tag):
        self.idm = binascii.hexlify(tag._nfcid)
        if self.idm.decode() in self.id_list:
            self.open()
        return True

    def open(self):
        GPIO.output(PORT, GPIO.HIGH)
        time.sleep(GPIO_HIGH_TIME)
        GPIO.output(PORT, GPIO.LOW)
        pygame.mixer.music.play()

    def read_id(self):
        self.clf.connect(rdwr={'on-connect': self.on_connect})

    def close(self):
        self.clf.close()


def main():
    id_list = []
    card_id = os.environ.get('CARD_ID')
    if card_id is None:
        with open(ID_LIST_PATH, 'r') as f:
            id_list = [id.strip() for id in f.readlines()]
    else:
        id_list.append(card_id)

    security_box = SecurityBox(id_list)
    while True:
        try:
            security_box.read_id()
            time.sleep(1)
        except KeyboardInterrupt:
            security_box.close()
            GPIO.cleanup()
            break


if __name__ == '__main__':
    main()
