from __future__ import print_function
import ftrobopy
import random
import time
import sys

txt=ftrobopy.ftrobopy('auto')

ml=txt.motor(1)
mr=txt.motor(2)
us=txt.ultrasonic(1)
battery=txt.voltage(8)

speed   = 400
mindist = 11

def go(dist, spd, turn=1):
  txt.SyncDataBegin()
  ml.setSpeed(spd)
  mr.setSpeed(turn*spd)
  ml.setDistance(dist, syncto=mr)
  mr.setDistance(dist, syncto=ml)
  txt.SyncDataEnd()
  txt.updateWait()

while True:
  go(random.randint(30,200), speed)
  while not ml.finished() and us.distance() > mindist:
    if us.distance() < 2*mindist:
      ml.setSpeed(300)
      mr.setSpeed(300)
    time.sleep(0.02)
  if battery.voltage() < 8500:
    print("Batteriespannung zu niedrig !")
    print("Programm wird beendet")
    print("Bitte die Batterie austauschen !")
    txt.stopOnline()
    time.sleep(0.5)
    sys.exit(0)
  ml.setSpeed(0)
  mr.setSpeed(0)
  time.sleep(0.5)
  if us.distance() <= mindist:
    go(15, -speed)
  time.sleep(0.2)
  while not ml.finished():
    d1=us.distance()
    time.sleep(0.02)
    d2=us.distance()
    time.sleep(0.02)
    if d1==d2:
      break
  time.sleep(0.5)
  go(random.randint(10,40), (random.randint(0,1)-1)*speed, -1)
  while not ml.finished() and not ml.getCurrentDistance() == 0:
    time.sleep(0.02)
  time.sleep(0.5)
