import sys, time
import ftrobopy

# Einlesen und Abspeichern eines TXT Camera Bildes (jpeg)

txt=ftrobopy.ftrobopy('192.168.7.2', 65000)
txt.startCameraOnline()
time.sleep(2.5)
pic = None
while pic == None:
  pic = txt.getCameraFrame()

f=open('TXTCamPic.jpg','w')
f.write(''.join(pic))
f.close()

