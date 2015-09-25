from visual import *

floor = box (pos=(0,0,0), length=4, height=0.5, width=4, color=color.blue)

#create a composite object with f
ball = frame()
## QUADCOPTER
body = ellipsoid(frame= ball, pos=(0,5,0), size=(3, 2, 5))
part1=ellipsoid(frame= ball, pos=(1,5,0), size = (1.5, 0.5, 2.5), color = color.red)
part2=ellipsoid(frame= ball, pos=(1,5,1), size = (1.5, 0.5, 2.5), color = color.red)
part3=ellipsoid(frame= ball, pos=(-1,5,-1), size = (1.5, 0.5, 2.5), color = color.red)
part4=ellipsoid(frame= ball, pos=(-1,5,1), size = (1.5, 0.5, 2.5), color = color.red)

ball.velocity = vector(0,-1,0)
dt = 0.01

while 1:
    rate (100)
    ball.pos = ball.pos + ball.velocity*dt
    if ball.pos.y < -(body.pos.y+0.5*floor.height):
        ball.velocity.y = abs(ball.velocity.y)
	print "please"
    else:
        ball.velocity.y = ball.velocity.y - 9.8*dt