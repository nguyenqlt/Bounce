from __future__ import division
from visual import *
from visual.graph import *
import time


def setup_graphics():

    f = frame()
    # QUADCOPTER
    ellipsoid(frame=f, pos=(0, 0, 0), size=(3, 1, 3))
    ellipsoid(frame=f, pos=(1.2, 0.4, -1.2),
              size = (1.5, 0.5, 1.5), color = color.red)
    ellipsoid(frame=f, pos=(1.2, 0.4, 1.2), size = (
        1.5, 0.5, 1.5), color = color.red)
    ellipsoid(frame=f, pos=(-1.2, 0.4, -1.2),
              size = (1.5, 0.5, 1.5), color = color.red)
    ellipsoid(frame=f, pos=(-1.2, 0.4, 1.2),
              size = (1.5, 0.5, 1.5), color = color.red)
    f.velocity = vector(0.0, 0, 0)
    f.acceleration = vector(0, 0, 0)
    f.force = vector(0, 0, 0)
    f.pos = vector(0, 0, 0)
    # Special new internal graphics variable
    f.current_rot = 0.0

    # Initilize spring graphics
    spring = helix(pos=(0., 0.0, 0),
                   axis=vector(0, -2, 0), radius=0.6, color=color.yellow)
    f.spring = spring
    f.spring_velocity = 0

    floors = []
    for i in range(0, 5):
        if i % 2 == 0:
            colorz = color.yellow
        else:
            colorz = color.white
        floors.append(
            box(size=(5, .5, 2), pos=(5 * i, -0.25, 0), color=colorz))
    f.floors = floors
    return f

#params = np.array([3.17668374201,7.18850217025,-0.0786962870327,1.34667502859,1.57141426143])

def update_graphics(x, f, m, scoot_backwards = 0.0):
    rotate(x.p, f)
    f.pos = (-scoot_backwards, x.y, 0.0)
    f.spring.pos = f.pos
    f.spring.axis = (m.l_x, m.l_y, 0.0)
    for i in range(0, 5):
        f.floors[i].pos.x = (-x.x + 5 * i -scoot_backwards) % 20 - 10


def rotate(new_rot, f):
    f.rotate(angle=(new_rot - f.current_rot), axis = (0, 0, 1), origin=f.pos)
    f.current_rot = new_rot

GRAPHICS = True
if GRAPHICS:
    f = setup_graphics()
else:
    f = None

	
Second_flight_phase = False
#if second_flight_phase = True
#	stop_early =
# INITIAL CONDITIONS
# Boundary Conditions
h = 3.18994768009  # meters, initial height of f
vx0 = 7.20235479284  # initial horizontal velocity
p0 = 0.182398
pd0 = -0.00963917274952


# Parameters
mass = 0.5  # kg
r0 = 2  # meters, relaxed length of spring
theta_air = 1.39935049585
theta_contact = 1.57076188074
mom_inertia = (mass * (0.1) ** 2)
gravity = 9.8  # acceleration of gravity
K_l = 500  # N/m
K_o = 0.2 # Nm / rad [change me]

#params = np.array([3.17668374201,7.18850217025,-0.0786962870327,1.34667502859,1.57141426143])

# State (with initial conditions):
class Object(object):
    pass


def run_sim(params=[h, vx0,  pd0, theta_air, theta_contact], stop_early = False, stop_at_apex=True):
    h, vx0,  pd0, theta_air, theta_contact = params
    p0=0.0
    x = Object()
    x.x = 0.0
    x.y = h
    x.xd = vx0
    x.yd = 0.0
    x.p = p0
    x.pd = pd0
    x.contact = False
    x.foot_x = 0.0
    x.foot_y = 0.0
    x0 = Object()
    x0.x = 0.0
    x0.y = h
    x0.xd = vx0
    x0.yd = 0.0
    x0.p = p0
    x0.pd = pd0
    x0.contact = False
    x0.foot_x = 0.0
    x0.foot_y = 0.0
    
	
    # Dimensionless parameters
    h_pi = h / r0
    v_pi = r0 / (sqrt(2 * r0 * gravity))

    # Memory:
    m = Object()
    m.force_x = 0.0
    m.force_y = 0.0
    m.torque = 0.0
    m.delta_o = 0.0
    m.delta_l = 0.0
    m.l_x = 0.0
    m.l_y = 0.0
    m.o = theta_air
    m.gamma = x.p - m.o
    m.timer = 0.0
    m.has_bounced=False

    m0 = Object()
    m0.force_x = 0.0
    m0.force_y = 0.0
    m0.torque = 0.0
    m0.delta_o = 0.0
    m0.delta_l = 0.0
    m0.l_x = 0.0
    m0.l_y = 0.0
    m0.o = theta_air
    m0.gamma = x.p - m.o #should this be m0.o?
    m0.timer = 0.0
    m0.has_bounced=False

    x.leg_angle = x.p - m0.o #angle leg/spring relative to quadcopter
    x.leg_velocity = f.spring_velocity #leg velocity

    def norm(x, y):
        return sqrt(pow(x, 2) + pow(y, 2))

    # OTHER DEFINITIONS

    # K0 = 0.5 * f.mass * vx0**2
    # Ugrav0 = f.mass * 9.8 * h
    # energy0 = K0 + Ugrav0 ## joules

    # Solver parameters
    dt = 0.01
    t = 0
    tinySteps = 100
    tiny_dt = dt / tinySteps
	
    # main physics and graphics loop
    while True:
        rate(100)
        # physics only loop:
        for i in range(tinySteps):
            t += tiny_dt
            m.force_y = - mass * gravity
            m.force_x = 0.0
            m.torque = 0.0
            # f.force = f.mass * accelOfGrav

            # Recalculate foot location:
            if x.contact:
                pass
            else:
                m.o = theta_air
                m.gamma = x.p - m.o
                m.l_x = r0 * cos(m.gamma)
                m.l_y = r0 * sin(m.gamma)
                x.foot_x = x.x + m.l_x
                x.foot_y = x.y + m.l_y
                if stop_at_apex and m.has_bounced and x.yd<0:
                    f2 = setup_graphics()
                    update_graphics(x0,f2,m0, scoot_backwards = x.x)
                    print "time elapsed %.4f" %(t)
                    print "x1 %.11f" % (x.x)
                    print "y1 %.11f" % (x.y)
                    print "xd1 %.11f" % (x.xd)
                    return None

                if x.foot_y <= 0.0 and m.timer<0:
                    x.foot_y = 0.0
                    x.contact = True
                
            # STANCE PHASE
            if x.contact:  # when spring touches floor
                m.l_x = x.foot_x - x.x
                m.l_y = x.foot_y - x.y
                m.gamma = atan2(m.l_y, m.l_x)
                m.o = x.p - m.gamma
                m.delta_o = m.o - theta_contact

                m.torque = - K_o * m.delta_o

                temp_length = norm(m.l_x, m.l_y)
                m.delta_l = temp_length - r0

                fc_x = m.l_x / temp_length * K_l * m.delta_l
                fc_y = m.l_y / temp_length * K_l * m.delta_l

                ft_x = m.l_y * pow(temp_length, -2) * K_o * m.delta_o
                ft_y = -m.l_x * pow(temp_length, -2) * K_o * m.delta_o

                # spring.axis = f.pos - spring.pos ##update spring length
                # spring.deflection = return_axis.mag*norm(spring.axis) - spring.axis
                # deflection_rate_of_change = change_in_length / tiny_dt
                if fc_y > 0:
                    m.force_x = fc_x + ft_x
                    m.force_y = fc_y + ft_y - mass * gravity
                else:
                    # report_bounce()
                    m.force_x = 0.0
                    m.force_y = - mass * gravity
                    m.torque = 0.0
                    x.contact = False
                    m.timer = 0.3
                    m.has_bounced=True

                    result = x.y + ((0.5 * x.yd ** 2) / gravity)-h
                    print
                    # print stuff
                    print "velocity error %.4f" % (x.xd - vx0)
                    print "velocity error from acceleration %.4f" % (xdd*t - 0) #integration of acceleration; doesn't match??
                    print "y error %.4f" % result
                    print "pd error %.4f" % (x.pd-pd0)
                    print "p error %.4f" % ( x.p + ((x.pd * x.yd) / gravity)- p0 )

                    cost = sqrt((x.xd - vx0) ** 2 + 20 * result ** 2 + 100 * (pd0 - x.pd)
                                ** 2 + 200 * (p0 - x.p - ((x.pd * x.yd) / gravity)) ** 2)
                    print "cost function %.4f" % cost
                    if stop_early:
                        return cost
                    
                    # exit()  

            # compute physics loop stuff
            xdd = m.force_x / mass
            ydd = m.force_y / mass
            pdd = m.torque / mom_inertia

            x.xd += xdd * tiny_dt
            x.yd += ydd * tiny_dt
            x.pd += pdd * tiny_dt

            x.x += x.xd * tiny_dt
            x.y += x.yd * tiny_dt
            x.p += x.pd * tiny_dt
            m.timer -= tiny_dt

            if x.y < 0.5:
                exit()
            
        # update graphics:
        if f != None:
            update_graphics(x, f, m)
			
import numpy as np
#params = np.array([h, vx0,  pd0, theta_air, theta_contact])	
params = np.array([3.18994768009,7.20235479284,-0.00963917274952,1.39935049585,1.57076188074]) #boundary/initial conditions
run_sim(params)

#params = np.array([3.18994768009,7.20235479284,-0.00963917274952,1.39935049585,1.57076188074]) for k_l = 500 (0.0003 cost function) 
#params = np.array([3.17668374201,7.18850217025,-0.0786962870327,1.34667502859,1.57141426143]) for k_o = 1
#params = np.array([3.16639408661,7.17722284657,-0.020173195117,1.34934601458,1.57075573497]) for k_l = 300 (0.0003)
#params = np.array([3.17515209752,7.18678709533,-0.00652406796842,1.38286778214,1.57080788872]) for k_o = 0.1 (0.0003)
