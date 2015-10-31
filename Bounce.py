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


# INITIAL CONDITIONS
# Boundary Conditions
adjustment1 = [0.3049999999999997, 0.10399999999999965, -
               4.0000000000000036, 2.5999999999999357]
scale1 = 0.001
adjustment2 = [0.22999999999999687, 0.12000000000000899, -
               1.5000000000000568, 3.200000000000003]
scale2 = -0.12
adjustment3 = [-0.550000000000006, 0.032999999999999696,
               13.799999999999923, 6.49999999999995]
scale3 = -0.001
adjustment4 = [-5.39999999999996, 0.17000000000000348, -
               9.899999999999908, -1.3999999999999568]
scale4 = -0.00002
h = 2.973363  # meters, initial height of f
vx0 = 6.9856676  # initial horizontal velocity
p0 = 0.182398
pd0 = -0.287872


# adjustment1 = 1.0286 to 1.0204
# adjustment2 = 1.0204 to 0.9130
# adjustment3 = 0.9130 to 0.8210
# adjustment4 = 0.8210 to 0.8187


# floor = box(size=(50, .01, 2), pos=(0, 0, 0))


# create a composite object with f


# Parameters
mass = 0.5  # kg
r0 = 2  # meters, relaxed length of spring
theta_air = math.radians(80)
theta_contact = math.radians(90)
mom_inertia = (mass * (0.1) ** 2)
gravity = 9.8  # acceleration of gravity
K_l = 400  # N/m
K_o = 0.2  # Nm / rad [change me]


# changing k_o = pd error , slight p error
# changing p0 = small changes = change v and y error a lot
# changing pd0 = velocity and y error change and p error
# how to change both to tweak it?

# State (with initial conditions):
class Object(object):
    pass


def run_sim(params=[h, vx0,  pd0, theta_air, theta_contact], stop_early = True, stop_at_apex=False):
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
    #0.0005, 0.11365
    #0.0003, 0.01, 0.1115

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
    m0.gamma = x.p - m.o
    m0.timer = 0.0
    m0.has_bounced=False

    # counts = 1
    # def report_bounce():
    #     global counts
    #     print "bounced %d times!" % counts
    #     counts += 1

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
    # spring.previous_deflection=0

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
                    print "y error %.4f" % result
                    print "pd error %.4f" % (x.pd-pd0)
                    print "p error %.4f" % ( x.p + ((x.pd * x.yd) / gravity)- p0 )

                    cost = sqrt((x.xd - vx0) ** 2 + 20 * result ** 2 + 100 * (pd0 - x.pd)
                                ** 2 + 200 * (p0 - x.p - ((x.pd * x.yd) / gravity)) ** 2)
                    # important to weight it differently or else you barely get
                    # results
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
params = np.array([h, vx0,  pd0, theta_air, theta_contact])

print params


def get_gradient(params, scale=0.005):
    # scale = 0.005
    old_value = run_sim(params)
    gradient = []
    for i in range(0, params.size):
        params_prime = params
        params_prime[i] += scale
        new_value = run_sim(params_prime)
        diff = new_value - old_value
        gradient_element = diff / scale
        gradient.append(gradient_element)
    return np.array(gradient)


params = np.array([3.15137388 , 7.16348663,   -0.01344186 , 1.37993469 , 1.57083067])

# run_sim(params, False, stop_at_apex=True)
# print params
# exit()

for i in range(0, 200):
    print params
    grad = get_gradient(params, scale = 0.000001)
    delta = -grad * 0.000002
    params += delta
    print delta
    string = "params = np.array(["+",".join([str(p) for p in params])+"])"
    print string


# 92.95, 0.05, 0.1 - falls back
# 92.95, 0.06, 0.1 - bounces in place for a while, then forward
# 92.95, 0.075, 0.1 - falls forward
# 92.95, 0.0595, 0.1 - falls forward
# 92.95, 0.0595, 0.11 - falls back
# 92.95, 0.0593, 0.1 - falls forward 32.934s
# 92.95, 0.0592, 0.1 - falls back 36.377s
# 92.99, 0.0592, 0.1 - falls forward 28.834s
# 92.95, 0.0001, 0.01 - 1 minute 54 secs
# 92.93, 0.00092, 0 - ball bounces very fast / high in air 3 mins 4 secs
# 92.89, 0.00015, 0.04 - 30.669 secs

# [2.98270406  6.99535711  0.17750009 - 0.3059267   1.400896    1.57183688]
