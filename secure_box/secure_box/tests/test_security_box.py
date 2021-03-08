#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import unittest
from mock import MagicMock, patch
from tests.module_mock import (PORT, GPIO_HIGH_TIME,
                               VOICE_FILE_PATH, ID_LIST_PATH)
from security_box import SecurityBox, main


@patch('security_box.nfc')
@patch('security_box.GPIO')
@patch('security_box.time')
@patch('security_box.pygame.mixer')
class TestSecurityBox(unittest.TestCase):

    def setUp(self):
        self.id_list = ["0001", "0002"]

    @patch('security_box.SecurityBox.open')
    def test_success_on_connect(self, mocked_security_box_open,
                                mocked_pygame_mixer, mocked_time, mocked_gpio, mocked_nfc):
        security_box = SecurityBox(self.id_list)
        mocked_tag = MagicMock()
        mocked_tag._nfcid = b'\x00\x01'
        connection_result = security_box.on_connect(mocked_tag)
        self.assertTrue(connection_result)
        mocked_security_box_open.assert_called_once()

    @patch('security_box.SecurityBox.open')
    def test_not_open_on_connect(self, mocked_security_box_open,
                                 mocked_pygame_mixer, mocked_time, mocked_gpio, mocked_nfc):
        security_box = SecurityBox(self.id_list)
        mocked_tag = MagicMock()
        mocked_tag._nfcid = b'\x00\x03'
        connection_result = security_box.on_connect(mocked_tag)
        self.assertTrue(connection_result)
        mocked_security_box_open.assert_not_called()

    def test_open(self, mocked_pygame_mixer, mocked_time, mocked_gpio, mocked_nfc):
        security_box = SecurityBox(self.id_list)
        security_box.open()
        mocked_gpio.output.assert_any_call(PORT, mocked_gpio.HIGH)
        mocked_time.sleep.assert_called_once_with(GPIO_HIGH_TIME)
        mocked_gpio.output.assert_any_call(PORT, mocked_gpio.LOW)
        mocked_pygame_mixer.music.play.assert_called_once()

    def test_read_id(self, mocked_pygame_mixer, mocked_time, mocked_gpio, mocked_nfc):
        security_box = SecurityBox(self.id_list)
        mocked_clf = MagicMock()
        mocked_nfc.ContactlessFrontend.return_value = mocked_clf
        security_box.read_id()
        mocked_nfc.ContactlessFrontend.assert_called_once_with('usb')
        mocked_clf.connect.assert_called_once_with(rdwr={'on-connect': security_box.on_connect})
        mocked_clf.close.assert_called_once()


class TestMain(unittest.TestCase):

    @patch('builtins.open')
    @patch('security_box.pygame.mixer')
    @patch('security_box.SecurityBox.read_id', side_effect=KeyboardInterrupt)
    @patch('security_box.GPIO')
    def test_main(self, mocked_gpio, mocked_security_box_read_id,
                  mocked_pygame_mixer, mocked_open):
        main()
        mocked_open.called_once(ID_LIST_PATH, 'r')
        mocked_pygame_mixer.init.called_once()
        mocked_pygame_mixer.music.load.called_once_with(VOICE_FILE_PATH)
        mocked_gpio.setmode.called_once_with(mocked_gpio.BCM)
        mocked_gpio.setup.called_once_with(PORT, mocked_gpio.OUT)
        mocked_gpio.cleanup.called_once()

