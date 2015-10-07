from __future__ import division
from visual import *
from visual.graph import *


# INITIAL CONDITIONS
# Boundary Conditions
h = 3  # meters, initial height of f
vx0 = 6  # initial horizontal velocity

# floor = box(size=(50, .01, 2), pos=(0, 0, 0))
floors = []
for i in range(0, 5):
    if i % 2 == 0:
        colorz = color.yellow
    else:
        colorz = color.white
    floors.append(
        box(size=(5, .5, 2), pos=(5 * i, -0.25, 0), color=colorz))


# create a composite object with f

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
f.velocity = vector(vx0, 0, 0)
f.acceleration = vector(0, 0, 0)
f.force = vector(0, 0, 0)
f.pos = vector(0, h, 0)


# Parameters
mass = 0.5  # kg
r0 = 2  # meters, relaxed length of spring
theta0 = math.radians(92.95)  # free angle [change me]
mom_inertia = (mass * (0.1) ** 2)
gravity = 9.8  # acceleration of gravity
K_l = 4000  # N/m
K_o = 0.0001  # Nm / rad [change me]

# State (with initial conditions):
x = frame()
x.x = 0.0
x.y = h
x.xd = vx0
x.yd = 0.0
x.p = 0.0
x.pd = 0.1 # [change me]

x.contact = False
x.foot_x = 0.0
x.foot_y = 0.0

# Memory:
m = frame()
m.force_x = 0.0
m.force_y = 0.0
m.torque = 0.0
m.delta_o = 0.0
m.delta_l = 0.0
m.l_x = 0.0
m.l_y = 0.0
m.o = theta0
m.gamma = x.p - m.o


# Initilize spring graphics
return_axis = vector(r0 * math.cos(m.gamma), r0 * math.sin(m.gamma), 0)
spring = helix(pos=(return_axis.x, h + return_axis.y, 0),
               axis=return_axis, radius=0.6, color=color.yellow)

# Special new internal graphics variable
f.current_rot = 0.0

counts = 0
def report_bounce():
    global counts
    print "bounced %d times!" % counts
    counts += 1


def rotate(new_rot):
    f.rotate(angle=(new_rot - f.current_rot), axis = (0, 0, 1), origin=f.pos)
    f.current_rot = new_rot


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
            m.o = theta0
            m.gamma = x.p - m.o
            m.l_x = r0 * cos(m.gamma)
            m.l_y = r0 * sin(m.gamma)
            x.foot_x = x.x + m.l_x
            x.foot_y = x.y + m.l_y

            if x.foot_y <= 0.0:
                x.foot_y = 0.0
                x.contact = True

        # STANCE PHASE
        if x.contact:  # when spring touches floor
            m.l_x = x.foot_x - x.x
            m.l_y = x.foot_y - x.y
            m.gamma = atan2(m.l_y, m.l_x)
            m.o = x.p - m.gamma
            m.delta_o = m.o - theta0

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
                report_bounce()
                m.force_x = 0.0
                m.force_y = - mass*gravity
                m.torque = 0.0
                x.contact = False

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

        if x.y < 0.5:
            exit()

    # update graphics:
    rotate(x.p)
    f.pos = (0.0, x.y, 0.0)
    spring.pos = f.pos
    spring.axis = (m.l_x, m.l_y, 0.0)
    for i in range(0, 5):
        floors[i].pos.x = (-x.x + 5 * i) % 20 - 10
