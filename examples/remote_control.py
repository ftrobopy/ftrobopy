from __future__ import print_function
import time
import ftrobopy

txt=ftrobopy.ftrobopy('auto')

nr = 0 # 0:any, 1-4:remote1-4

joystickLeft  = txt.joystick(0, remote_number=nr)
joystickRight = txt.joystick(1) # remote_number=0 (any) is the default
buttonON      = txt.joybutton(0)
buttonOFF     = txt.joybutton(1)

while not buttonOFF.pressed():
  print(joystickLeft.leftright(), joystickLeft.updown(), buttonON.pressed())
  time.sleep(0.02)

print("fertig")
