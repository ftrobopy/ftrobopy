#######################################
# Example: ftDigiCam.py
#          Digital Camera with live
#          video stream to TXT display
#          and autofocus functionality
#          (c) 2016 by Torsten Stuehn
#          version 0.81 from 2016-02-19
#######################################

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

hist_minlength     = 10
hist_maxlength     = 30
fname_prefix       = 'PICT'
displayLiveStream  = 1 # live video on TXT display, 0=no 1=yes

# definition of TXT outputs and inputs
FocusMotor       = txt.motor(1) # the focus motor is connected to M1
Trigger          = txt.input(1) # the camera trigger with auto focus is connected to I1
ManualFocus1     = txt.input(2) # manual focus1 connected to I2
ManualFocus2     = txt.input(3) # manual focus2 connected to I3
ManualTrigger    = txt.input(4) # the camera trigger without auto focus is connected to I4

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

  contrast                = 0
  hist_contrast           = [0]
  hist_counter            = [0]
  ave_contrast            = 0
  counter_hist_shift      = 1

  xtopleft     =  width / 2  - width / 8
  ytopleft     =  height / 2 - height / 8
  xbottomright =  width / 2  + width / 8
  ybottomright =  height / 2 + height / 8
  
  state = 0
  dir   = -1

  ManualFocus1_prev = 0
  ManualFocus2_prev = 0
  
  for i in range(1000):
    contrast = ftrobopytools.measureContrast(videv,
                                          width, height,
                                          xtopleft, ytopleft, xbottomright, ybottomright,
                                          displayLiveStream)
    # print(ftrobopytools.sdlPollEvent())
    motor_counter = FocusMotor.getCurrentDistance()

    hist_contrast.append(contrast)
    if len(hist_contrast) > hist_maxlength:
      hist_contrast = hist_contrast[1:]
    
    hist_counter.append(motor_counter)
    if len(hist_counter) > hist_maxlength:
      hist_counter = hist_counter[1:]

    max_contrast       = max(hist_contrast)
    contrast_variation = 0
    for i in hist_contrast:
      if i != max_contrast:
        contrast_variation = 1
        break 

    if ManualTrigger.state() != 0:
      FocusMotor.stop()
      state = 4

    if ManualFocus1.state() != 0 and ManualFocus1_prev == 0 and ManualFocus2.state() == 0:
      FocusMotor.stop()
      FocusMotor.setDistance(1000)
      FocusMotor.setSpeed(512*dir)
      state = 0
      ManualFocus1_prev = 1

    if ManualFocus2.state() != 0 and ManualFocus2_prev == 0 and ManualFocus1.state() == 0:                                        
      FocusMotor.stop()                                                                                
      FocusMotor.setDistance(1000)                                                                       
      FocusMotor.setSpeed(-512*dir)                                                                     
      state = 0
      ManualFocus2_prev = 1

    if ManualFocus1.state() == 0 and ManualFocus1_prev == 1 and ManualFocus2.state() == 0:
      FocusMotor.stop()                                                                   
      state = 0
      ManualFocus1_prev = 0

    if ManualFocus2.state() == 0 and ManualFocus2_prev == 1 and ManualFocus1.state() == 0:
      FocusMotor.stop()                                                                   
      state = 0                                                                           
      ManualFocus2_prev = 0
                                                                                                       
    if state == 0:
      if Trigger.state() != 0:
        state  = 1

    if state == 1:
      #print("state 1: start focus motor")
      # start increasing focus
      FocusMotor.setDistance(3000)
      FocusMotor.setSpeed(512*dir)
      hist_contrast = [0]
      hist_counter  = [0]
      state = 2

    if state == 2:
      if len(hist_contrast) > hist_minlength and ( hist_contrast[-1] < hist_contrast[-2] or contrast_variation == 0 ):
        #print("state 2: contrast_variation",contrast_variation)
        hist_contrast = [0]
        hist_counter  = [0]
        FocusMotor.stop()
        # start decreasing focus
        FocusMotor.setDistance(3000)
        FocusMotor.setSpeed(-512*dir)
        state = 3

    if state == 3:
      if len(hist_contrast) > hist_minlength and ( hist_contrast[-1] < hist_contrast[-2] or contrast_variation == 0 ):
        #print("state 3: contrast_variation",contrast_variation)
        FocusMotor.stop()
        # increase focus to maximum contrast
        idx               = hist_contrast.index(max(hist_contrast))
        #print("index of highest contrast is ",idx)
        #print(hist_counter)
        bestfocus_counter = hist_counter[idx-counter_hist_shift]
        #print("FocusMotor.setDistance: ",abs(hist_counter[-1]-bestfocus_counter))
        FocusMotor.setDistance(abs(hist_counter[-1]-bestfocus_counter))
        FocusMotor.setSpeed(512*dir)
        state = 4

    if state == 4:
      if FocusMotor.finished():
        # save jpeg in high resolution (1280x720)
        #print("taking snapshot at high resolution ...")
        # close (low resolution) camera device
        ftrobopytools.camClose(videv, 0)
        # open (high resolution) camera device
        high_fps     = 5    # 5 is the lowest possible framerate of the TXT camera
        high_width   = 1280 # 1280 is the maximum horizontal resolution of the TXT camera
        high_height  = 720  # 720 is the maximum vertical resolution of the TXT camera
        videv = ftrobopytools.camInit(high_fps, high_width, high_height, 0, 0)
        # get high resolution snapshot as jpg image
        jpg   = ftrobopytools.getJPEGImage(videv)
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

