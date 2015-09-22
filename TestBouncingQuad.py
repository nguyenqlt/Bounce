from visual import *

floor = box(size = (20,2,20),color=(0.6,0.3,0.9))
#create a composite object with f
f = frame()
## QUADCOPTER
ellipsoid(frame= f, pos=(0,40,0), size=(3, 1, 5))
ellipsoid(frame= f, pos=(1,40,0), size = (1.5, 0.5, 2.5), color = color.red)
ellipsoid(frame= f, pos=(1,40,1), size = (1.5, 0.5, 2.5), color = color.red)
ellipsoid(frame= f, pos=(-1,40,-1), size = (1.5, 0.5, 2.5), color = color.red)
ellipsoid(frame= f, pos=(-1,40,1), size = (1.5, 0.5, 2.5), color = color.red)


g = 9.8
finished = False
mass=15
velocity = vector(0,0,0)
dt = 0.01

for obj in f.objects:
while not finished:
    rate(100)
    force=vector(0,-mass*g,0)
    acceleration=force/mass
    velocity +=acceleration*dt
    f.pos += velocity*dt - 0.5*acceleration*dt**2
    if f.pos.y < mass*0.5*floor.height:
        velocity.y = -velocity.y