import time
import random
import numpy 
import arena



arena.init("oz.andrew.cmu.edu","/topic/render")

arena.start()

location=(5.0,6.0,7.0)
rotation=(0.0,0.0,0.0,0.0)
color=(0,255,0)
scale=(1.0,1.0,1.0)

myCube = arena.cube(location,rotation,scale,color)

x=1.0

while True: 
  myCube.location((x,2.0,3.0))
#  myCube = arena.cube((random.randrange(10),random.randrange(10),random.randrange(10)),rotation,scale,color)
  x=x+1
  time.sleep(.1)

# del myCube



arena.stop()

