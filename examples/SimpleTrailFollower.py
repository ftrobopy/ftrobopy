import ftrobopy
import time

txt=ftrobopy.ftrobopy('auto')

ml=txt.motor(1)
mr=txt.motor(2)
t1=txt.trailfollower(2)
t2=txt.trailfollower(3)

ml.setSpeed(512)
mr.setSpeed(512)
while True:
  if t1.state() < 1000 and t2.state() > 1000:
    ml.setSpeed(0)
  else:
    ml.setSpeed(512)
  if t2.state() < 1000 and t1.state() > 1000:
    mr.setSpeed(0)
  else:
    mr.setSpeed(512)
  time.sleep(0.02)

