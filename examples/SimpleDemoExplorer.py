import sys
import ftrobopy

txt=ftrobopy.ftrobopy('auto')

Motor_rechts = txt.motor(1)
Motor_links  = txt.motor(2)
Ultraschall  = txt.ultrasonic(8)
Motor_rechts.setSpeed(512)
Motor_links.setSpeed(512)
Motor_rechts.setDistance(1000, syncto=Motor_links)
Motor_links.setDistance(1000, syncto=Motor_rechts)

while not Motor_rechts.finished(): 
  d=Ultraschall.distance()
  print d
  if d<10:
    Motor_rechts.stop()
    Motor_links.stop()
    if txt.sound_finished():
      txt.play_sound(3,1)

