######################################
# Example: ftDigiCam.py
#          Digital Camera with live
#          video stream to TXT display
#          and autofocus functionality
#          (c) 2016 by Torsten Stuehn
#          version 0.8 from 2016-02-12
######################################

# Python2/3 'print' compatibility
from __future__ import print_function

import ftrobopy
import ftrobopytools
from os import system
import time

txt = ftrobopy.ftrobopy(host                     ='127.0.0.1',
                        port                     = 65000,
                        update_interval          = 0.01,
                        keep_connection_interval = 1.0)

run_ave_contrast   = 8
hist_minlength     = 10
hist_maxlength     = 20
fname_prefix       = 'PICT'
displayLiveStream  = 1 # live video on TXT display, 0=no 1=yes

# definition of TXT outputs and inputs
FocusMotor       = txt.motor(1) # the focus motor is connected to M1
Switch           = txt.input(1) # the switch is connected to I1

if displayLiveStream:
  # make backup copy of TXT display contents
  with open('/dev/fb0', 'rb') as f:
    framebuffer_backup=f.read()

try:

  # initialize camera (/dev/video0) and 
  fps    = 15  # frames per second
  width  = 320 # width of camera image
  height = 240 # height of camera image
  videv  = ftrobopytools.camInit(fps, width, height, 0, 0)

  if displayLiveStream:
    # initialize Standard Display Library (SDL)
    # for access to display of TXT
    ftrobopytools.sdlInit()
    # reset text/cmd console (compromized by SDL)
    system('reset')

  contrast                = [0]
  hist_contrast           = [0]
  hist_counter            = [0]
  contrast_counter_shift  = 3
  ave_contrast            = 0

  xtopleft     = 10 # width / 2  - width / 4
  ytopleft     = 10 # height / 2 - height / 8
  xbottomright = 310 # width / 2  + width / 8
  ybottomright = 120 # height / 2 + height / 8
  
  state = 0
  dir   = -1
  
  for i in range(1000):
    contr = ftrobopytools.measureContrast(videv,
                                          width, height,
                                          xtopleft, ytopleft, xbottomright, ybottomright,
                                          displayLiveStream)
    if contr:
      contrast.append(contr)                                                                                       
      if len(contrast) > run_ave_contrast:
        contrast = contrast[1:]

    ave_contrast  = sum(contrast)/len(contrast)
    motor_counter = FocusMotor.getCurrentDistance()
    contrast_variation = 0
    for i in contrast:
      if i != ave_contrast:
        contrast_variation = 1
    
    hist_contrast.append(ave_contrast)
    if len(hist_contrast) > hist_maxlength:
      hist_contrast = hist_contrast[1:]
    
    hist_counter.append(motor_counter)
    if len(hist_counter) > hist_maxlength:
      hist_counter = hist_counter[1:]

    #if state == 2 or state == 3 or state == 4:
    if True:
      print(hist_contrast)
      #print(hist_counter)

    if state == 0:
      if Switch.state() != 0:
        # dir    = -dir
        state  = 1

    if state == 1:
      print("state 1: start focus motor")
      # start increasing focus
      FocusMotor.setDistance(3000)
      FocusMotor.setSpeed(512*dir)
      hist_contrast = [0]
      hist_counter  = [0]
      state = 2

    if state == 2:
      if len(hist_contrast) > hist_minlength and ( hist_contrast[-1] < hist_contrast[-2] or contrast_variation == 0 ):
        print("state 2: contrast_variation",contrast_variation)
        hist_contrast = [0]
        hist_counter  = [0]
        FocusMotor.stop()
        # start decreasing focus
        FocusMotor.setDistance(3000)
        FocusMotor.setSpeed(-512*dir)
        state = 3

    if state == 3:
      if len(hist_contrast) > hist_minlength and ( hist_contrast[-1] < hist_contrast[-2] or contrast_variation == 0 ):
        print("state 3: contrast_variation",contrast_variation)
        FocusMotor.stop()
        # increase focus to maximum contrast
        idx               = hist_contrast.index(max(hist_contrast))
        bestfocus_counter = hist_counter[idx]
        #FocusMotor.setDistance(hist_counter[-(1+contrast_counter_shift)] - bestfocus_counter)
        FocusMotor.setDistance(300)
        FocusMotor.setSpeed(512*dir)
        state = 4

    if state == 4:
      if FocusMotor.finished():
        # save jpeg in high resolution (1280x720)
        print("taking snapshot at high resolution ...")
        # close (low resolution) camera device
        ftrobopytools.camClose(videv, 0)
        # open (high resolution) camera device
        high_fps     = 5    # 5 is the lowest possible framerate of the TXT camera
        high_width   = 1280 # 1280 is the maximum horizontal resolution of the TXT camera
        high_height  = 720  # 720 is the maximum vertical resolution of the TXT camera
        videv = ftrobopytools.camInit(high_fps, high_width, high_height, 0, 0)
        # get high resolution snapshot as jpg image
        jpg   = ftrobopytools.getJPGImage(videv)
        # close (high resolution) camera device
        ftrobopytools.camClose(videv, 0)
        # restore resolution for liveStreaming
        videv = ftrobopytools.camInit(fps, width, height, 0, 0)
        # save jpeg to file and increment picture count index
        try:
          with open(fname_prefix+'IDX','r') as f:
            pict_number = int(f.read())
        except:
          pict_number = 0
        with open(fname_prefix + '%04i' % pict_number +'.JPG', 'wb') as f:
          f.write(jpg)
        with open(fname_prefix+'IDX','w') as f:
          f.write(str(pict_number + 1))
        
        # ready for the next picture
        hist_contrast = [0]
        hist_counter  = [0]
        state = 0

except ftrobopytools as error:
  print(error)

finally:
  # close camera device
  ftrobopytools.camClose(videv, 0)
  if displayLiveStream:
    # close Standard Display Library
    ftrobopytools.sdlClose()
    # restore TXT display
    with open('/dev/fb0', 'wb') as f:
      f.write(framebuffer_backup)

