from __future__ import division
from visual import *
from visual.graph import *

## INITIAL CONDITIONS
r0 = 2 ## meters, relaxed length of spring
theta0 = math.radians(90) ## free angle
h = 3 ## meters, initial height of f
vx0 = 6 ## initial horizontal velocity
y_max = 3 ## initial max height


##create a composite object with f
f = frame()
## QUADCOPTER
ellipsoid(frame= f, pos=(0,h,0), size=(3, 1, 5))
ellipsoid(frame= f, pos=(1,h,0), size = (1.5, 0.5, 2.5), color = color.red)
ellipsoid(frame= f, pos=(1,h,1), size = (1.5, 0.5, 2.5), color = color.red)
ellipsoid(frame= f, pos=(-1,h,-1), size = (1.5, 0.5, 2.5), color = color.red)
ellipsoid(frame= f, pos=(-1,h,1), size = (1.5, 0.5, 2.5), color = color.red)

## DEFINE SPRING
return_axis = vector(r0*math.cos(theta0),r0*math.sin(theta0),0)
spring = helix(frame = f, pos=(return_axis.x,h - return_axis.y,0), axis=return_axis, radius=0.6,color=color.yellow)
spring.constant = 4000 ## N/m

f.axis = (0.5,-0.25,1)


