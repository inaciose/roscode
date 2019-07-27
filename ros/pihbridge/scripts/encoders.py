#!/usr/bin/python
import time
import RPi.GPIO as GPIO

# left encoder pins
ELPIN1 = 25
ELPIN2 = 27
# right encodet pins
ERPIN1 = 28
ERPIN2 = 29

GPIO.setmode(GPIO.BCM)

# setup left encoder pins
GPIO.setup(ELPIN1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ELPIN2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# setup righ encoder pins
GPIO.setup(ERPIN1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ERPIN2, GPIO.IN, pull_up_down=GPIO.PUD_UP)

LENC = 0
RENC = 0

LLAST = "00"
RLAST = "00"

STATE = {"0001":1, "0010":-1, "0100":-1, "0111":1,
         "1000":1, "1011":-1, "1101":-1, "1110":1}

def encl_callback(chan):
  global LENC, LLAST, STATE
  curr = str(GPIO.input(ELPIN1)) + str(GPIO.input(ELPIN2))
  key = LLAST + curr
  if key in STATE:
    drctn = STATE[key]
    LLAST = curr
    LENC += drctn

def encr_callback(chan):
  global RENC, RLAST, STATE
  curr = str(GPIO.input(ERPIN1)) + str(GPIO.input(ERPIN2))
  key = RLAST + curr
  if key in STATE:
    drctn = STATE[key]
    RLAST = curr
    RENC += drctn

GPIO.add_event_detect(ELPIN1, GPIO.BOTH, callback=encl_callback)
GPIO.add_event_detect(ELPIN2, GPIO.BOTH, callback=encl_callback)
GPIO.add_event_detect(ERPIN1, GPIO.BOTH, callback=encr_callback)
GPIO.add_event_detect(ERPIN2, GPIO.BOTH, callback=encr_callback)

while(1):
    printf("(%i %i)" % (LENC, RENC))
    time.sleep(0.02)
