from visual import *

floor = box(size = (20,2,20),color=(0.6,0.3,0.9))

#create a composite object with f
ball = frame()
## QUADCOPTER
body = ellipsoid(frame= ball, pos=(0,50,0), size=(3, 2, 5))
part1=ellipsoid(frame= ball, pos=(1,50,0), size = (1.5, 0.5, 2.5), color = color.red)
part2=ellipsoid(frame= ball, pos=(1,50,1), size = (1.5, 0.5, 2.5), color = color.red)
part3=ellipsoid(frame= ball, pos=(-1,50,-1), size = (1.5, 0.5, 2.5), color = color.red)
part4=ellipsoid(frame= ball, pos=(-1,50,1), size = (1.5, 0.5, 2.5), color = color.red)

g = 9.8
finished = False
mass=15
velocity = vector(0,0,0)
dt = 0.01


while not finished:
    rate(100)
    force=vector(0,-mass*g,0)
    acceleration=force/mass
    velocity +=acceleration*dt
    ball.pos += velocity*dt - 0.5*acceleration*dt**2
	##does not work with body.height... works with negative and body.pos.y
    if ball.pos.y < -(body.pos.y+0.5*floor.height):
        velocity.y = -velocity.y
	print "this happened"