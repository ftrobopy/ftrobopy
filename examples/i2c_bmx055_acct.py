import ftrobopy
import time
import struct

txt=ftrobopy.ftrobopy('192.168.8.2', 65000)

res=txt.i2c_read(0x18, 0x00)                                                  
if res[0] == 250:
  print("Found BMX055 Acceleration Sensor at address 0x18")
  txt.i2c_write(0x18, 0x11, 0x00)  # set normal power mode
  txt.i2c_write(0x18, 0x3e, 0x80)  # set fifo stream mode
  txt.i2c_write(0x18, 0x0f, 0x05)  # set g-range to +-4g
  txt.i2c_write(0x18, 0x10, 0x0a)  # set filter to 15.63 Hz
                                                        
  for i in range(10000):                                   
    fifo_status=txt.i2c_read(0x18, 0x0e)[0]
    #temperature=txt.i2c_read(0x18, 0x08)[0]
    if fifo_status > 0: # and fifo_status < 128:
      res=txt.i2c_read(0x18, 0x3f, data_len=6)
      x,y,z=struct.unpack('<hhh', res)                    
      print(i, fifo_status, x >> 4 , y >> 4, z >> 4)
    #elif fifo_status > 128: # fifo overflow
    #  res=txt.i2c_read(0x18, 0x3f, data_len=6)
    #time.sleep(0.01)                                     
else:
  print("no bmx055 acceleration sensor found at address 0x18")
  
