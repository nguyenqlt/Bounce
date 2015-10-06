"""Gray's refactoring of the controlled SLIP bounce demo by Annie Ye."""

from __future__ import division
import visual.graph as vg
import math
import visual as vis
from collections import namedtuple


# define a state vector class, using namedtuple
StateVector = namedtuple("StateVector", ["t", "x", "y", "x_dot", "y_dot"])
# named tuples are useful, immutable data structures.
# namedtuple itself is a class factory, a function which returns a class
# the classes in python can be manipulated at runtime.
# they have a constructor which takes named args
for_example = StateVector(t=32.0, x=0.0, y=0.0, x_dot=21.0, y_dot=12)


def state_addition(state, state_update):
    """Addition overload for state vectors."""
    # return StateVector(*(a + b for a, b in zip(state, state_update))) #
    # slow?0.779
    return StateVector(  # faster: 0.385
        t=state.t + state_update.t,
        x=state.x + state_update.x,
        y=state.y + state_update.y,
        x_dot=state.x_dot + state_update.x_dot,
        y_dot=state.y_dot + state_update.y_dot)
# Here I add the addition method to the StateVector class. before I do this,
# addition is tuple concatenation:
print "just a size 10 tuple:", for_example + for_example
setattr(StateVector, "__add__", state_addition)
# now StateVector(stuff) + StateVector(stuff) returns a new StateVector
add_example = for_example + for_example
print "A new StateVector:", add_example

# define a discrete state vector class, using namedtuple
DiscreteStateVector = namedtuple(
    "DiscreteStateVector", ["is_contact", "x_foot", "y_foot"])

# define some physical parameters as a tuple class
PhysicalParams = namedtuple(
    "PhysicalParams", ["mass", "gravity", "spring_constant", "free_spring_length", "dt", "tiny_dt"])

# this is a datatype used by the energy_result variable, recalculated
# every time step.
EnergyResult = namedtuple(
    "EnergyResult", ["total_energy", "kinetic_energy", "grav_potential", "spring_potential"])


class SLIP_Physics(object):

    """Handles physics for SLIP models."""

    def __init__(self, physical_params):
        self.params = physical_params

    def contact_update(self, state, dstate, spring_monitor, extra_force):
        if spring_monitor.deflection < 0.0:
            # when the spring has stretched beyond its free length, contact has
            # ended
            dstate = dstate._replace(is_contact=False)
        force = (extra_force - self.params.spring_constant *
                 spring_monitor.deflection)
        x_double_dot = 1.0 / self.params.mass * spring_monitor.axis.x * force
        # negative because the axis points from COM towards the foot.
        y_double_dot = (self.params.gravity + spring_monitor.axis.y * force / self.params.mass)

        state = state + StateVector(
            #"t", "x", "y", "x_dot", "y_dot"
            t=self.params.tiny_dt,
            x=self.params.tiny_dt * state.x_dot,
            y=self.params.tiny_dt * state.y_dot,
            x_dot=self.params.tiny_dt * x_double_dot,
            y_dot=self.params.tiny_dt * y_double_dot)
        return state, dstate

    def flight_update(self, state, dstate, free_angle):
        state = state + StateVector(
            #"t", "x", "y", "x_dot", "y_dot"
            t=self.params.tiny_dt,
            x=self.params.tiny_dt * state.x_dot,
            y=self.params.tiny_dt * state.y_dot,
            x_dot=0.0,
            y_dot=self.params.tiny_dt * self.params.gravity)
        dstate = DiscreteStateVector(
            is_contact=False,
            x_foot=state.x +
            math.cos(free_angle) * self.params.free_spring_length,
            y_foot=state.y - math.sin(free_angle) * self.params.free_spring_length)
        if dstate.y_foot < 0.0:
            # if the foot is below the ground, contact is happening next
            # iteration
            dstate = dstate._replace(is_contact=True)
        return state, dstate


class SpringMonitor(object):

    def __init__(self, params):
        self.params = params
        # convention change: axis points from body to foot!
        # axis will now be a unit vector.
        self.axis = vis.vector(0.0, -1.0, 0.0)
        self.mag = params.free_spring_length
        self.deflection = 0.0
        self.mag_dot = 0.0

    def update(self, state, dstate):
        """Extract spring information from state."""
        x_spring = dstate.x_foot - state.x
        y_spring = dstate.y_foot - state.y
        previous_mag = self.mag
        self.mag = math.sqrt(x_spring ** 2 + y_spring ** 2)
        self.deflection = self.params.free_spring_length - self.mag
        self.mag_dot = (self.mag - previous_mag) / self.params.tiny_dt
        # axis stays a unit vector.
        self.axis.x = x_spring / self.mag
        self.axis.y = y_spring / self.mag


class AngleController(object):

    """
    A class to control the jump height via angle control.

    This class also calculates the max jump height, so all it 
    needs is the state vector. The class is callable, which
    just means it's objects can be used as if they were functions.
    This class also takes care of differentiating the previous
    y_max signal.
    """

    def __init__(self, angle_gain=-0.1, theta_0=math.radians(87.75), epsilon=0.0001, prev_y_max=3.0):
        """Initialize the internal state."""
        self.angle_gain = angle_gain
        self.prev_y_max = prev_y_max
        self.epsilon = epsilon
        self.theta_0 = theta_0

    def update(self, state):
        """Update the internal state."""
        y_max = state.y_dot ** 2 * (1 / (9.8 * 2)) + state.y
        change_in_y_max = y_max - self.prev_y_max
        # self.prev_y_max = y_max  # y_max will be stored from previous jump
        if abs(change_in_y_max) > self.epsilon:
            print "Jump height change: " + str(change_in_y_max)
            # angle increases if y_max is decreasing or angle decreases if
            # y_max increases
            self.theta_0 = math.radians(87.75) - change_in_y_max * self.angle_gain
            if self.theta_0>math.radians(89.0):
                self.theta_0 = math.radians(89.0)
            if self.theta_0<math.radians(70.0):
                self.theta_0 = math.radians(70.0)
            print "Free angle: " + str(math.degrees(self.theta_0))

    def __call__(self, state):
        """Return free angle."""
        return self.theta_0


class EnergyController(object):

    """Manages extra force, based on leap energy."""

    def __init__(self, gain=100.0, tolerance=0.0000001, desired=0.0):
        self.gain = gain
        self.tolerance = tolerance
        self.desired = desired

    def __call__(self, state, energy_result, spring_monitor):
        """Apply proportional control to energy."""
        # return 0.0
        error = energy_result.total_energy - self.desired
        if abs(error) < self.tolerance:
            # exit early.
            return 0.0
        if abs(spring_monitor.mag_dot) < 1e-7:
            return 0.0
        # spring is expanding and we need to add energy

        extraPower = self.gain * error

        extra_force = extraPower / spring_monitor.mag_dot
        print "error", error
        print "extra_force", extra_force

        # simple saturation is easier than the conditions.
        if extra_force < 0.0:
            extra_force = 0.0
        if extra_force > 20.0:
            extra_force = 20.0

        return extra_force


class EnergyTabulator(object):

    """Calculates Energy. Callable object (functor)."""

    def __init__(self, physical_params):
        self.params = physical_params
        self.total_energy = None
        self.kinetic_energy = None
        self.grav_potential = None
        self.spring_potential = None

    def __call__(self, state, dstate, spring_monitor):
        """Return a result object storing the energy components."""
        self.kinetic_energy = (0.5 * self.params.mass *
                               (state.x_dot ** 2 + state.y_dot ** 2))
        self.grav_potential = - (
            self.params.mass * self.params.gravity * state.y)
        self.spring_potential = 0.0
        if dstate.is_contact:
            self.spring_potential = (
                0.5 * self.params.spring_constant
                * spring_monitor.deflection ** 2)
        self.total_energy = self.kinetic_energy + \
            self.grav_potential + self.spring_potential
        # result_object = EnergyResult(
        #     total_energy, kinetic_energy, grav_potential, spring_potential)
        return self


class SLIPRunnerGraphics(object):

    """
    A class to handle the graphics.
    """

    def __init__(self):
        """Just create the objects."""
        self.spring = vis.helix(pos=(0.0, 0.0, 0.0),
                                axis=vis.vector(1.0, 1.0, 1.0), radius=0.6, color=vis.color.yellow)
        self.ball = vis.sphere(
            pos=(0.0, 0.0, 0.0), radius=1, make_trail = True)
        self.floors=[]
        for i in range(0,5):
            if i%2==0:
                color=vis.color.yellow
            else:
                color= vis.color.white
            self.floors.append(vis.box(size=(5, .01, 2), pos=(5*i, 0, 0),color=color))

    def __call__(self, state, dstate):
        """Position the objects according to the state and dstate."""
        self.spring.pos = (dstate.x_foot - state.x, dstate.y_foot, 0.0)
        self.spring.axis = (
            state.x - dstate.x_foot, state.y - dstate.y_foot, 0.0)
        self.ball.pos = (0.0, state.y, 0.0)
        for i in range(0,5):
            self.floors[i].pos.x = (-state.x+5*i)%20-10dstate


#class SLIPGraphs(object):

   # """A class to handle the graphs."""

 #   def __init__(self):
  #      graph1 = vg.gdisplay()
   #     self.energygraph = vg.gcurve(color=vis.color.red)
    #    self.Kgraph = vg.gcurve(color=vis.color.green)
     #   self.Ugravgraph = vg.gcurve(color=vis.color.orange)
      #  self.Uspringgraph = vg.gcurve(color=vis.color.yellow)
       # self.positiongraph = vg.gcurve(color=vis.color.cyan)
        #self.velocitygraph = vg.gcurve(color=vis.color.blue)

   # def __call__(self, state, energy_result):
    #    self.energygraph.plot(pos=(state.t, energy_result.total_energy))
     #   self.positiongraph.plot(pos=(state.t, state.y))
      #  self.Kgraph.plot(pos=(state.t, energy_result.kinetic_energy))
       # self.Ugravgraph.plot(pos=(state.t, energy_result.grav_potential))
        #self.Uspringgraph.plot(pos=(state.t, energy_result.spring_potential))
        #self.velocitygraph.plot(pos=(state.t, state.x_dot))


def main(iters=1000000):
    """Run the demo."""
    # INITIAL CONDITIONS
    r0 = 2  # meters, relaxed length of spring
    theta0 = math.radians(90)  # free angle
    h = 3  # meters, initial height of ball
    vx0 = 6  # initial horizontal velocity
    y_max = h  # initial max height
    dt = 0.01
    t = 0

    tinySteps = 100
    tiny_dt = dt / tinySteps

    physical_params = PhysicalParams(0.5, -9.81, 4000, 2.0, dt, tiny_dt)

    physics = SLIP_Physics(physical_params)
    energy_tabulator = EnergyTabulator(physical_params)
    spring_monitor = SpringMonitor(physical_params)

    # moving forward, at an initial height
    init_state = StateVector(t, 0.0, h, vx0, 0.0)
    init_dstate = DiscreteStateVector(False, 0.0, 0.0)  # not in contact

    angle_controller = AngleController(
        prev_y_max=y_max)  # use default settings

    K0 = 0.5 * physical_params.mass * vx0 ** 2
    Ugrav0 = physical_params.mass * 9.8 * h
    energy0 = K0 + Ugrav0  # joules
    energy_controller = EnergyController(gain=10.0, desired=energy0)

    graphics = SLIPRunnerGraphics()  

    graphs_object = SLIPGraphs()
	
	

    state = StateVector(*init_state)  # copy initial state
    dstate = DiscreteStateVector(*init_dstate)

    for i in range(0, iters):
        vis.rate(100)

        for i in range(100):
            t += tiny_dt

            spring_monitor.update(state, dstate)
            energy_result = energy_tabulator(state, dstate, spring_monitor)

            if dstate.is_contact:
                extra_force = energy_controller(
                    state, energy_result, spring_monitor)
                state, dstate = physics.contact_update(
                    state, dstate, spring_monitor, extra_force)
                if not dstate.is_contact:
                    angle_controller.update(state)
            else:
                free_angle = angle_controller(state)
                state, dstate = physics.flight_update(
                    state, dstate, free_angle)

        graphs_object(state, energy_result)
        graphics(state, dstate)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Run the slip model.')
    parser.add_argument(
        '-p', '--profile', help='run_profiler', action='store_true')

    args = parser.parse_args()
    if args.profile:
        print "Profiling!!!"
        import cProfile
        print cProfile.run("main(1000)")
    else:
        main()
