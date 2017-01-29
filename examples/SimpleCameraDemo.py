import time
import ftrobopy

# Einlesen und Abspeichern eines TXT Camera Bildes (jpeg)

txt=ftrobopy.ftrobopy('auto')
time.sleep(2)
txt.startCameraOnline()
time.sleep(2.5)
pic = None
while pic == None:
  pic = txt.getCameraFrame()
  if pic != None:
    if len(pic) == 0:
      pic = None

with open('TXTCamPic.jpg', 'w') as f:
  f.write(''.join(pic))
