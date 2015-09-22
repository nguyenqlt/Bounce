from visual import *

floor = box(size = (20,2,20),color=(0.6,0.3,0.9))
ball = sphere(pos=(0,50,0),radius=2,color=(0.3,0.9,0.6))

g = 9.8
finished = False
mass=1
velocity = vector(0,0,0)
dt = 0.01

while not finished:
    rate(100)
    force=vector(0,-mass*g,0)
    acceleration=force/mass
    velocity +=acceleration*dt
    ball.pos += velocity*dt - 0.5*acceleration*dt**2
    if ball.pos.y < ball.radius+0.5*floor.height:
        velocity.y = -velocity.y