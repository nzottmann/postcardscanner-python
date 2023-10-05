# Copyright 2022 Nils Zottmann
# Licensed under the EUPL-1.2-or-later

import logging
logger = logging.getLogger('scanner_v3')
import time
import threading
import RPi.GPIO as GPIO
from picamera2 import Picamera2, Preview
from RpiMotorLib import RpiMotorLib
from PIL import Image, ImageEnhance
from pyzbar.pyzbar import decode, ZBarSymbol
from .scanner import Scanner
from postcardscanner.states import PostcardScannerState

class ScannerV3(Scanner):
    clockwise = False
    steptype_1 = "Half"
    steptype_2 = "Half"
    stepdelay = .0003
    counter = 0
    last_timeout_reset = time.monotonic()
    postcard_accepted = False
    user_feedback_event = threading.Event()
    def __init__(self, callback, pins={
        'dir': 4,
        'step': 17,
        'mode': (22, 27, 18),
        's1': 16,
        's2': 19,
        's3': 20,
        's4': 21,
        'sleep': 23,
        'led': 12
    }):
        self.pins = pins
        self.callback = callback
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # GPIOs
        GPIO.setup((pins['s1'], pins['s2'], pins['s3'], pins['s4']), GPIO.IN)
        GPIO.setup((pins['sleep'], pins['led']), GPIO.OUT)

        self.motor = RpiMotorLib.A4988Nema(pins['dir'], pins['step'], pins['mode'], "DRV8825")
        
        Picamera2.set_logging(Picamera2.WARNING)
        self.picam2 = Picamera2()
        
        self._init_state()
        
    def _sensor_occupied(self, sensor):
        sensors = ('s1', 's2', 's3', 's4')
        return GPIO.input(self.pins[sensors[sensor]])
    
    def _all_sensors_occupied(self, sensors):
        return all(self._sensor_occupied(s) for s in sensors)

    def _no_sensor_occupied(self, sensors):
        return all(not self._sensor_occupied(s) for s in sensors)
        
    def _mot_sleep(self, state):
        GPIO.output(self.pins['sleep'], state)
        
    def _led(self, state):
        GPIO.output(self.pins['led'], state)

    def _reset_timeout(self):
        self.last_timeout_reset = time.monotonic()

    def _is_timeout_elapsed(self, timeout):
        return self.last_timeout_reset + timeout < time.monotonic()
        
    def _init_state(self):
        if self._no_sensor_occupied((0, 1)):
            self.pos = 0
        else:
            self.pos = 2

    
    def accept_postcard(self):
        self.postcard_accepted = True
        self.user_feedback_event.set()

    def reject_postcard(self):
        self.postcard_accepted = False
        self.user_feedback_event.set()

    # Idle waiting for postcard
    def state_0(self):
        if self._all_sensors_occupied((0, 1)): # Postcard inserted
            return 1
        self._mot_sleep(0)
        time.sleep(0.1)
        return 0
    
    # Sensor 0 and 1 occupied
    def state_1(self):
        self._reset_timeout()
        self._mot_sleep(1)
        while True:
            self.motor.motor_go(self.clockwise, self.steptype_1, 100, self.stepdelay, False, 0)

            if self._is_timeout_elapsed(5):
                return 99
            if not self._all_sensors_occupied((0, 1)):
                return 0
            if self._sensor_occupied(2):
                return 2
    
    # Sensor 0, 1 and 2 occupied
    def state_2(self):
        self._reset_timeout()
        self._mot_sleep(1)
        while True:
            self.motor.motor_go(self.clockwise, self.steptype_1, 100, self.stepdelay, False, 0)

            if self._is_timeout_elapsed(5):
                return 99
            if not self._sensor_occupied(2):
                return 1
            if self._sensor_occupied(3):
                return 3
    
    # Sensor 0, 1, 2 and 3 occupied, exact positioning and capture
    def state_3(self):
        self._reset_timeout()
        self._mot_sleep(1)
        while True: # Proceed until 0 and 1 not occupied anymore
            self.motor.motor_go(self.clockwise, self.steptype_1, 10, self.stepdelay, False, 0)

            if self._is_timeout_elapsed(10):
                return 99
            if self._no_sensor_occupied((0, 1)):
                break
        
        self.motor.motor_go(self.clockwise, self.steptype_1, 10, self.stepdelay, False, 0) # A bit more
        
        while True: # Slowly back to 1 to have an exact position
            self.motor.motor_go(not self.clockwise, self.steptype_1, 1, self.stepdelay, False, self.stepdelay)

            if self._is_timeout_elapsed(10):
                return 99
            if self._all_sensors_occupied((0, 1)):
                break
        
        # Move to capture position
        self.motor.motor_go(not self.clockwise, self.steptype_1, 20, self.stepdelay, False, 0)

        # Capture
        self._mot_sleep(0)
        self._led(1)
        try:
            self.picam2.start_preview(Preview.NULL) # Preview.DRM to see a preview
            still_config = self.picam2.create_still_configuration(display='main', main={"size": (4608, 2592)})
            self.picam2.configure(still_config)
            self.picam2.start()

            self.picam2.autofocus_cycle()
            img = self.picam2.capture_image("main")

            self.picam2.stop()
            self.picam2.stop_preview()
        except Exception as e:
            logger.error(f'Capture callback raised exception: {e}')
            return 6
        finally:
            self._led(0)

        try:
            position = (860, 300)
            dimensions = (2940, 2000)
            img = img.crop((position[0], position[1], position[0] + dimensions[0], position[1] + dimensions[1]))
            
            # Crop possible qr locations
            img_bottomleft = img.crop((0, img.height*0.7, img.width*0.2, img.height))
            img_topright = img.crop((img.width*0.8, 0, img.width, img.height*0.3))

            # Enhance contrast
            img_bottomleft = ImageEnhance.Contrast(img_bottomleft).enhance(3.0)
            img_topright = ImageEnhance.Contrast(img_topright).enhance(3.0)

            # Detect
            code_list_bottomleft = decode(img_bottomleft, symbols=[ZBarSymbol.QRCODE])
            code_list_topright = decode(img_topright, symbols=[ZBarSymbol.QRCODE])

            if code_list_bottomleft:
                img = img.transpose(Image.ROTATE_180)
                logger.info('Rotating image')
                qr = code_list_bottomleft[0]
            elif code_list_topright:
                qr = code_list_topright[0]
            else:
                logger.info('No qr found')
                self.callback(img, success=False)
                return 6

            self.user_feedback_event.clear() # Clear old events just before saving a new image
            self.callback(img, success=True)
        except Exception as e:
            logger.error(f'Error exracting qr code: {e}')
            return 6
        
        return 4

    # Wait for user feedback
    def state_4(self):
        logger.info('waiting for user feedback')
        if self.user_feedback_event.wait(timeout=120.0):
            self.user_feedback_event.clear()
            if self.postcard_accepted:
                logger.info('user accepted postcard')
                return 5
        else: # Timeout
            logger.info('timeout waiting for user feedback: collect postcard')
            return 5
        
        # User did not accept postcard -> return
        logger.info('user rejected postcard')
        return 6
    
    # Collect postcard
    def state_5(self):
        logger.info('5')
        self._reset_timeout()
        self._mot_sleep(1)
        while True: # Proceed until 2 not occupied anymore
            self.motor.motor_go(self.clockwise, self.steptype_2, 200, self.stepdelay, False, 0)

            if self._is_timeout_elapsed(5):
                return 99
            if not self._sensor_occupied(2):
                break

        self.motor.motor_go(self.clockwise, self.steptype_2, 3000, .00008, False, 0)
        self._mot_sleep(0)

        return 0
    
    # Eject
    def state_6(self):
        self._reset_timeout()
        self._mot_sleep(1)
        self.motor.motor_go(not self.clockwise, self.steptype_2, 900, self.stepdelay, False, 0)
        while True: # Eject until 2 not occupied anymore
            self.motor.motor_go(not self.clockwise, self.steptype_2, 10, self.stepdelay, False, 0)

            if self._is_timeout_elapsed(5):
                return 99
            if not self._sensor_occupied(2):
                break
        self.motor.motor_go(not self.clockwise, self.steptype_2, 100, self.stepdelay, False, 0) # Eject a little bit more, postcard just stuck
        self._mot_sleep(0)

        self._reset_timeout()
        while True: # Wait until postcard is removed
            if self._is_timeout_elapsed(60): # User did not remove postcard -> pull inside without scanning
                logger.info('timeout waiting for user to remove postcard')
                return 5
            if self._all_sensors_occupied((0, 1, 2)): # User pushed postcard back inside -> scan again
                logger.info('user pushed postcard inside instead of removing: scan again')
                return 2
            if self._no_sensor_occupied((0, 1, 2, 3)): # User removed postcard
                logger.info('user removed postcard')
                break

        return 0
    
    # Error state, wait until stuck postcard is removed manually
    def state_99(self):
        self._mot_sleep(0)
        while not self._no_sensor_occupied((0, 1, 2, 3)):
            time.sleep(0.1)
        
        return 0
    
    def loop(self):
        state_map = {
            0: self.state_0,
            1: self.state_1,
            2: self.state_2,
            3: self.state_3,
            4: self.state_4,
            5: self.state_5,
            6: self.state_6,
            99: self.state_99
        }

        # Execute current step, returns next step
        self.pos = state_map[self.pos]()

        # Disable motor after each step
        self._mot_sleep(0)
        
        if self.pos in [1, 2, 3]:
            return PostcardScannerState.scanning
        elif self.pos == 99:
            return PostcardScannerState.error
        else:
            return PostcardScannerState.enabled
