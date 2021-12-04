import time
import ftrobopy

txt=ftrobopy.ftrobopy('auto')
txt.updateWait()

m1=txt.motor(1)
m2=txt.motor(2)

m1.setDistance(0, sn=6)
txt.updateWait()
m2.setDistance(0, sn=5)
txt.updateWait()

m1.setSpeed(450)
m2.setSpeed(-450)

txt.updateWait()
count = 1
inject = 12
inject_delta = inject
syncerror = 5
flip = True
while not m1.finished() or not m2.finished():
  txt.updateWait()
  c1 = m1.getCurrentDistance()
  c2 = m2.getCurrentDistance()
  # wenn das Error Injection Bit des Slave-Motors gesetzt ist (syncto-number: 5..8 anstelle von 1..4)
  # wird die Nulllinie (synczero) des Slave-Counters neu gesetzt nach der Formel:
  # synczero(Slave) = CurrentCounter(Slave) - distance
  #                   - CurrentCounter(Master)
  #                   + synczero(Master)   # auch die synczero Nullinie des Masters kann eingestellt werden
  # Grundsaetzlich wird die Synchronitaet geregelt, indem jeweils die Geschwindigkeit des Masters oder des
  # Slaves (je nachdem wer zu schnell oder zu langsam ist) staendig angepasst wird, um die Counter der beiden
  # in Uebereinstimmung zu bringen
  # Der synczero-Wert wird dabei jeweils vorher von dem Counter-Wert abgezogen
  # SyncErr = ( Counter(Slave) - synczero(Slave) ) - ( Counter(Master) - synczero(Master) )
  # if SyncErr < 0:
  #   slow down master
  #   speed up slave
  # else:
  #   slow down slave
  #   speed up master
  # die Staerke der Abbremsung richtet sich dabei jeweils nach der Groesse von SyncErr 
  # Wenn die Counter der beiden Motoren sich zu sehr unterscheiden (abs(c1-c2) > 16, synczero mit einbezogen),
  # wird einer der Motoren solange angehalten, bis der andere aufgeholt hat.
  # Mit dieser Methode kann man also zwei Motoren synchron halten und sie trotzdem
  # unterschiedliche Strecken zuruecklegen lassen.
  # Dabei kommt es jedoch zu einem leichten Geschwindigkeitsabhaengigen Jitter der Counter-Werte zueinander.

  # Beispiel: Beide Motoren liefen bisher synchron ==> Current(Master)=Current(Slave)=100
  #           nach m_slave.setDistance(10, sn=5)
  #           ist die neue Nulllinie des Slave-Counters bei -10
  # damit ist dann ein Unterschied von 10 das neue Ziel bei den Countern. Die Anpassung erfolgt
  # kontinuierlich und ist Geschwindigkeitsabhaengig.
  #
  # Wenn SyncErrors zu schnell hintereinander eingefuegt werden, koennen sich die Counter nicht mehr schnell
  # genug anpassen und es kommt zu einer Drift eines Counters
  #
  # Achtung: Beim Einguegen von SyncErrors werden die Motoren nicht mehr automatisch bei erreichen der eingestellten
  #          Distanz gestoppt. [motor.finished() ist dann immer False]

  if c1 > inject and c1 <= (inject * 5)+(inject_delta / 2):
    if flip:
      m1.setDistance(syncerror, sn=6)
      #flip = not flip
    else:
      m2.setDistance(syncerror, sn=5)
      #flip = not flip
    if syncerror > 0:
      print('+' * syncerror)
    elif syncerror < 0:
      print('-' * abs(syncerror))
    else:
      print('0')
    inject += inject_delta

  #print(f'{count:6d} {c1:6d} {c2:6d}','.'* abs(c1-c2), f'({c1-c2})')
  count += 1

print('m1 and m2 finished')

txt.stopAll()
txt.updateWait()

txt.stopOnline()
time.sleep(0.1)
