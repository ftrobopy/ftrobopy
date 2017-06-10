import ftrobopy
import sys

if (len(sys.argv) < 2):
    sys.exit('At least to args needed')

txt=ftrobopy.ftrobopy('auto')
txt.play_sound(int(sys.argv[1]),int(sys.argv[2]))
while not txt.sound_finished():
  pass
txt.stopOnline()
