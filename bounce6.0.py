from __future__ import division
from visual import *
from visual.graph import *

## INITIAL CONDITIONS
r0 = 2 ## meters, relaxed length of spring
theta0 = math.radians(90) ## free angle
h = 3 ## meters, initial height of f
vx0 = 6 ## initial horizontal velocity
y_max = 3 ## initial max height

## DEFINE SPRING
return_axis = vector(r0*math.cos(theta0),r0*math.sin(theta0),0)
spring = helix(pos=(return_axis.x,h - return_axis.y,0), axis=return_axis, radius=0.6,color=color.yellow)
spring.constant = 4000 ## N/m

##create a composite object with f
f = frame()
## QUADCOPTER
ellipsoid( pos=(0,h,0), size=(3, 1, 5))
ellipsoid(frame= f, pos=(1,h,0), size = (1.5, 0.5, 2.5), color = color.red)
ellipsoid(frame= f, pos=(1,h,1), size = (1.5, 0.5, 2.5), color = color.red)
ellipsoid(frame= f, pos=(-1,h,-1), size = (1.5, 0.5, 2.5), color = color.red)
ellipsoid(frame= f, pos=(-1,h,1), size = (1.5, 0.5, 2.5), color = color.red)
f.mass = 0.5 # kg
f.velocity = vector(vx0,0,0)
f.acceleration = vector(0,0,0)
f.force = vector(0,0,0)

floor = box(size=(50,.01,2),pos=(0,0,0))

## OTHER DEFINITIONS
accelOfGrav = vector(0,-9.8,0) ## acceleration of gravity

K0 = 0.5 * f.mass * vx0**2
Ugrav0 = f.mass * 9.8 * h
energy0 = K0 + Ugrav0 ## joules

dt = 0.0001
t = 0

tinySteps = 100
tiny_dt = dt/tinySteps

## GRAPHING
graph1 = gdisplay()
energygraph = gcurve(color=color.red)
Kgraph = gcurve(color=color.green)
Ugravgraph = gcurve(color=color.orange)
Uspringgraph = gcurve(color = color.yellow)
positiongraph = gcurve(color=color.cyan)

spring.previous_deflection=0

while True:
    rate(100)

    for i in range(100):
        t += tiny_dt
        f.force = f.mass * accelOfGrav
        
        ## STANCE PHASE
        if f.pos.y - return_axis.y <= 0.005 :## when spring touches floor
            previous_length = mag(spring.axis) ## for calcuating rate of deflection
            spring.axis = f.pos - spring.pos ##update spring length
            spring.deflection = return_axis.mag*norm(spring.axis) - spring.axis
            change_in_length = spring.deflection.mag - previous_length ## current - previous
            deflection_rate_of_change = change_in_length / tiny_dt
			
			
            ## ENERGY CONTROL LAW
            energyDifference = energy - energy0
            energyGain = 100000.0 ##Hz, this is proportional gain
            energy_error_tolerance = 0.0001
            if f.velocity.y > 0 and energyDifference < - energy_error_tolerance: ## spring is expanding and we need to add energy
                extraPower = energyGain * energyDifference
                extraForce = extraPower / deflection_rate_of_change
                print extraForce
                    
            elif f.velocity.y <0 and energyDifference > energy_error_tolerance: ##spring is contracting and we need to subtract energy
                extraPower = energyGain * energyDifference
                extraForce = extraPower / deflection_rate_of_change
              
            else: #no extra power necessary
                extraForce = 0

            springForce = spring.constant * spring.deflection - extraForce * norm(spring.axis)
            f.force += springForce ## update force on f
            
        ## FLIGHT PHASE
        else:

            ## FREE ANGLE CONTROL LAW
                                
            prev_y_max = y_max ## y_max will be stored from previous jump
            y_max = f.velocity.y ** 2 * (1 / (9.8 * 2))+ f.pos.y ## update y_max to current
            change_in_y_max = y_max - prev_y_max
            angle_gain = 0.1
            epsilon = 0.0001 

            if abs(change_in_y_max) > epsilon:
                print "Jump height change: " + str(change_in_y_max)
                theta0 += - change_in_y_max * angle_gain ## angle increases if y_max is decreasing or angle decreases if y_max increases
                print "Free angle: " + str(math.degrees(theta0))
                return_axis = vector(r0*math.cos(theta0),r0*math.sin(theta0),0) ## update return_axis with new angle

            spring.pos = f.pos - return_axis ##update spring position
            spring.axis = return_axis ##reset to fixed angle
            spring.deflection = vector(0,0,0) ##make sure spring is expanded

        f.acceleration = f.force/f.mass
        f.velocity += f.acceleration * dt ##update f velocity
        f.pos += f.velocity * dt ##update f position

        ## ENERGY ACCOUNTING
        K = 0.5 * f.mass * f.velocity.mag**2
        Ugrav = f.mass * 9.8 * f.pos.y
        Uspring = 0.5 * spring.constant * spring.deflection.mag**2
        energy = K + Ugrav + Uspring #calculate current energy

    ## UPDATE GRAPHS
    energygraph.plot(pos=(f.pos.x,energy))
    positiongraph.plot(pos=(f.pos.x, f.pos.y))
    Kgraph.plot(pos=(f.pos.x,K))
    Ugravgraph.plot(pos=(f.pos.x,Ugrav))
    Uspringgraph.plot(pos=(f.pos.x,Uspring))
