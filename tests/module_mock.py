#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from mock import MagicMock, patch


MockNFC = MagicMock()
MockPyGame = MagicMock()
MockRPi = MagicMock()
MockTime = MagicMock()

PORT = 4
GPIO_HIGH_TIME = 0.5
VOICE_FILE_PATH = 'voice/speech_20200710023858074.mp3'
ID_LIST_PATH = 'data/id_list.csv'


modules = {
    'nfc': MockNFC,
    'pygame': MockPyGame,
    'pygame.mixer': MockPyGame.mixer,
    'RPi': MockRPi,
    'RPi.GPIO': MockRPi.GPIO,
}

patcher = patch.dict("sys.modules", modules)
patcher.start()
