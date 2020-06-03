import time
import ftrobopy

txt=ftrobopy.ftrobopy('auto', use_TransferAreaMode=True)
m1=txt.motor(1)
m1.setSpeed(300)
time.sleep(3)
m1.setSpeed(0)

print(txt.stopOnline())
time.sleep(2)
