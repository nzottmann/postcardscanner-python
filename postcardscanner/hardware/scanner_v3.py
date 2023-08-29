# Copyright 2022 Nils Zottmann
# Licensed under the EUPL-1.2-or-later

import logging
logger = logging.getLogger('scanner_v3')
import time
from io import BytesIO
import subprocess
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
from .scanner import Scanner
from postcardscanner.states import PostcardScannerState

class ScannerV3(Scanner):
    clockwise = False
    steptype_1 = "Half"
    steptype_2 = "Half"
    stepdelay = .0003
    counter = 0
    last_timeout_reset = time.monotonic()
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
        
    def capture(self):
        process = subprocess.Popen(
            ['libcamera-still', '-n', '-t', '1', '-o', '-'],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )        
        return BytesIO(process.stdout.read())
    

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
            self.motor.motor_go(self.clockwise, "1/16", 100, self.stepdelay, False, 0)

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
            self.motor.motor_go(not self.clockwise, "1/16", 10, self.stepdelay, False, 0)

            if self._is_timeout_elapsed(10):
                return 99
            if self._all_sensors_occupied((0, 1)):
                break
        
        # Move to capture position
        self.motor.motor_go(not self.clockwise, "1/16", 20*8, self.stepdelay, False, 0)

        # Capture
        self._mot_sleep(0)
        self._led(1)
        try:
            self.callback(self.capture())
        except Exception as e:
            logger.error(f'Capture callback raised exception: {e}')
        
        return 4
    
    # Throw out
    def state_4(self):
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
    
    # Error state
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
            99: self.state_99
        }

        # Execute current step, returns next step
        self.pos = state_map[self.pos]()

        # Disable motor after each step
        self._mot_sleep(0)
        
        return PostcardScannerState.enabled
