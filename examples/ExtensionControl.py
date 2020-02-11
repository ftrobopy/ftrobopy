import ftrobopy

txt=ftrobopy.ftrobopy('auto', use_extension=True)

# One of the TXTs has be put in "Master"-mode and one in "Extension"-mode (see local TXT configuration menue)
# The two TXTs must be connected with a 10-pin extension cable

m1 = txt.motor(1, ext=0) # use motor output 1 on the Master
m2 = txt.motor(2) # if "ext" parameter is not given, use the default (ext=0), which is the master

m3 = txt.motor(1, ext=1) # use motor output 1 on the Extension

switch1 = txt.input(1, ext=1) # use input 1 on the Extension

