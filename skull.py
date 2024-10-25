# Halloween Skull
# (C) 2024, Daniel Quadros
# MIT License

import time
import random
import neopixel
import rp2 
from rp2 import PIO, asm_pio
from machine import Pin
from machine import UART

# Connections
pinHcsrTrigger = 19
pinHcsrEcho = 20
pinMp3Busy = 7
pinLedRGB = 3

# Sound Effects
mp3Voices = 10
mp3Laughter = 11
mp3Thunder = 12

# A few parameters
mp3Volume = 30
minDistance = 40

# Class for MP3 player control
class MP3(object):

    # send a command to the MP3 module
    def sendCmd(self, cmd, param):
        # put command and parameter in the buffer
        self.bufCmd[3] = cmd
        self.bufCmd[5] = param >> 8
        self.bufCmd[6] = param & 0xFF
        # calculate checksum and put it in the buffer
        check = 0
        for i in range(1, 7):
            check = check + self.bufCmd[i]
        check = -check
        self.bufCmd[7] = (check >> 8) & 0xFF
        self.bufCmd[8] = check & 0xFF
        # send buffer to module
        self.uart.write (self.bufCmd)

    # constructor
    def __init__(self, ioBusy):
        self.uart = UART(0, 9600)
        self.uart.init(9600, bits=8, parity=None, stop=1)
        self.pinBusy = Pin(ioBusy, Pin.IN)
        self.bufCmd = bytearray(10)
        self.bufCmd[0] = 0x7E  # start mark
        self.bufCmd[1] = 0xFF  # protocol version
        self.bufCmd[2] = 6     # size
        self.bufCmd[4] = 0     # no answer
        self.bufCmd[9] = 0xEF  # end mark

    # initialize the MP3 module
    def init(self, vol=16):
        self.sendCmd(0x06, vol)  # volume
        time.sleep(0.1)
        self.sendCmd(0x07, 1)    # pop equalization
        time.sleep(0.1)
        
    # play a track
    def play(self, track):
        self.sendCmd (0x12, track)
        # wait for playing to start
        timeout = time.ticks_add(time.ticks_ms(), 3000)
        while self.pinBusy.value() and time.ticks_diff(timeout, time.ticks_ms()) > 0:
            time.sleep(0.01)
            
    # wait for end of track
    def waitEndOfTrack(self):
        while not self.pinBusy.value():
            time.sleep(0.01)

    # abort playing
    def abortPlay(self):
        self.sendCmd (0x16, 0)
        self.waitEndOfTrack()


# PIO program for HC-SR04 sensor
@asm_pio(set_init=rp2.PIO.OUT_LOW,autopush=False)
def ULTRA_PIO():
    # wait for a request
    pull()
    mov (x,osr)
    
    # make a 10 us (20 cycles) pulse
    set(pins,1) [19]
    set(pins,0)
    
    # wait for the start of the echo pulse
    wait(1,pin,0)
    
    # wait for the end of the echo pulse
    # decrement X every two cycles (1us)
    label('waitEndOfPulse')
    jmp(pin,'continue')
    jmp('end')    
    label('continue')
    jmp(x_dec,'waitEndOfPulse')
    
    # return the pulses duration
    label('end')
    mov(isr,x)
    push()

# Class for distance sensor
class HCSR04(object):
    # constructor
    def __init__(self, trigger, echo):
        self.sm = rp2.StateMachine(0)
        self.pinTrigger = Pin(trigger, Pin.OUT)
        self.pinEcho = Pin(echo, Pin.IN)
        
    # measure distance
    def measure(self):
        self.sm.init(ULTRA_PIO,freq=2000000,set_base=self.pinTrigger,
                     in_base=self.pinEcho,jmp_pin=self.pinEcho)
        self.sm.active(1)
        self.sm.put(300000)
        val = self.sm.get()
        self.sm.active(0)
        tempo = 300000 - val
        return (tempo * 0.0343) / 2

# Initialize RGB LEDs
led = neopixel.NeoPixel(Pin(pinLedRGB), 3)

# Initialize MP3
mp3 = MP3(pinMp3Busy)
mp3.init(mp3Volume)

# Initialize distance sensor
sensor = HCSR04 (pinHcsrTrigger, pinHcsrEcho)

# Check if there is someone/something in front of the Skull
def presence():
    return sensor.measure() < minDistance

# Blink eyes
def blinkEyes():
    for color in range(4, 12, 2):
        if presence():
            return True
        led[1] = led[2] = (color, 0, 0)
        led.write()
        time.sleep(0.1)
    for color in range(10, -2, -2):
        if presence():
            return True
        led[1] = led[2] = (color, 0, 0)
        led.write()
        time.sleep(0.2)
    return False

# Light LEDs
def lightLEDs():
    for color in range(4, 40, 4):
        led[0] = (color//2, 0, 0)  # back
        led[1] = (color, 0, 0)  # left eye
        led[2] = (color, 0, 0)  # right eye
        led.write()
        time.sleep(0.1)

# Turn off the LEDs
def turnOffLEDs():
    led[0] = (0, 0, 0)  # back
    led[1] = (0, 0, 0)  # left eye
    led[2] = (0, 0, 0)  # right eye
    led.write()

# Generates a random time in the future
def randomTime(minTime, maxTime):
    return time.ticks_add(time.ticks_ms(),
                          minTime+random.randrange(0, maxTime-minTime))

# Sets initial state
turnOffLEDs()
timeToAction = time.ticks_ms()

# Main loop
while True:
    if presence():
        print ("Scares")
        mp3.abortPlay()
        lightLEDs()
        mp3.play(mp3Laughter)
        mp3.waitEndOfTrack()
        timeToAction = randomTime(5000, 7000)
        while presence():
            if time.ticks_diff(timeToAction, time.ticks_ms()) <= 0:
                if random.random() < 0.2:
                    mp3.play(mp3Laughter)
                else:
                    mp3.play(mp3Thunder)
                timeToAction = randomTime(10000, 20000)
            time.sleep(0.1)
        mp3.abortPlay()
        turnOffLEDs()
        timeToAction = randomTime(5000, 15000)
    elif time.ticks_diff(timeToAction, time.ticks_ms()) <= 0:
        print ("Warning")
        blinkEyes()
        if random.random() < 0.2:
            mp3.play(mp3Voices)
        timeToAction = randomTime(15000, 30000)
   
