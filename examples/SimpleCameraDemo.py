import time
import ftrobopy

# Einlesen und Abspeichern eines TXT Camera Bildes (jpeg)

txt=ftrobopy.ftrobopy('auto')
time.sleep(2)
txt.startCameraOnline()
time.sleep(2.5)
pic = txt.getCameraFrame()

with open('TXTCamPic.jpg', 'wb') as f:
  f.write(bytearray(pic))
