from math import sin, cos, atan2,pi

#...............................................................................................
class BaseRobot(object):
    """Base class of Robot containing the
    basic tools to operate a swarming robot.
    It requires a *body* instance that is
    inherits from BaseBody and a *network*
    instance that inherits from BaseNetwork.
    Initializing the robot will NOT open channels
    for broadcast or turn on the physical body
    of the robot (that's what turn_on() is for.)

    It is recommended that robot models inherit
    from BaseRobot and call its __init__ method
    if it has to be expanded, like this:

        class MyRobot(BaseRobot):
            def __init__(self, body, network, args):
                do_something(args)
                BaseRobot.__init__(self,body,network)
                return
    """

    def __init__(self, body, network):
        """As a way to make sure the body and network
        instances have the required method, this class
        only accept bodies that inherit from BaseBody
        and networks that inherit from BaseNetwork.
        """
        if isinstance(body, BaseBody):
            self.body = body
        else:
            raise Exception("The body provided is not an instance of BaseRobot.BaseBody()")
        if isinstance(network, BaseNetwork):
            self.network = network
        else:
            raise Exception("The network provided is not an instance of BaseRobot.BaseNetwork")
        self.working = False
        return

    def is_working(self):
        return self.working

    def turn_on(self):
        """Call the turn_on methods of body and network
        if they exist (not required) and start
        broadcasting. Calling this does nothing if the
        instance is already turned on.
        """
        if not self.is_working():
            if "turn_on" in dir(self.body):
                self.body.turn_on()
            if "turn_on" in dir(self.network):
                self.network.turn_on()
            self.network.start_broadcasting()
            self.working = True
        return

    def turn_off(self):
        """Call the turn_off methods of body and network
        if they exist (not required) and stop
        broadcasting. Calling this does nothing if the
        instance is already turned off.
        """
        if self.is_working():
            self.network.stop_broadcasting()
            if "turn_off" in dir(self.body):
                self.body.turn_off()
            if "turn_off" in dir(self.network):
                self.network.turn_off()
            self.working = False
        return

    # To use robots with the "with Robot(...) as robot:" statement
    def __enter__(self):
        self.turn_on()
        return self

    # To use robots with the "with Robot(...) as robot:" statement
    def __exit__(self, type, value, traceback):
        self.turn_off()
        return

# Kinematic methods:

    def move_forward(self, time,v=None):
        """If the robot is working, call body.move_forward.
        This will make the robot stop advancing if an
        obstacle is detected in front, as defined by obstacle_infront().
        """
        if self.is_working():
            if self.obstacle_infront():
                v = 0.0
            self.body.move_forward(time,v)
        return

    def rotate(self, dtheta):
        """Calls body.rotate is the robot is working.
        Return the time it took to rotate."""
        if self.is_working():
            time = self.body.rotate(dtheta)
        else:
            time = 0.
        return time

    def align(self, direction):
        """Align the heading of the robot to
        a given vector *direction*.
        """
        if "align" in dir(self.body):
            self.body.align(direction)
            dtheta = None
        else:
            dtheta = atan2( direction[1], direction[0]) - self.body.get_heading()
            if dtheta > pi:
                dtheta -= 2*pi
            elif dtheta < -pi:
                dtheta += 2*pi
            self.rotate(dtheta)
        return dtheta

# Communication methods:

    def get_agents(self):
        """Return a dictionary with the state of each robot.
        """
        return self.network.get_agents_state()

    def broadcast_state(self):
        """Broadcast current state (x,y,heading) over
        the network.
        """
        if self.is_working():
            self.network.send_state(self.body.get_position(), self.body.get_heading())
        return

# Obstacle avoidance methods:

    def obstacle_infront(self):
        """Return True if an obstacle is "in front", meaning
        that extreme measures such as stopping the movement
        have to be taken to avoid a collision.
        Used by move_forward() to stop the robot in case
        something is too close.
        """
        #TODO Explore the optimal ranges where of each sensor.
        us = self.body.get_ultrasound()
        return us[2]<0.31 or us[1]<0.29 or us[3]<0.29

    def obstacle_near(self):
        """Return True if an obstacle is "near" meaning
        that the robot should be aware of the existence
        of the obstacle even if it may not collide
        directly.
        """
        #TODO Explore the optimal ranges where of each sensor.
        return any( d<0.35 for d in self.body.get_ultrasound() if d)

    def correct_target(self, target):
        """Correct the target vector *target* so that if the
        robot moves in that direction it will not collide with
        obstacles.
        Current implementation: correct if obstacle_near(), not
        if obstacle_infront(). In case a correction is needed,
        choose the closest obstacle and project the target vector
        to be perpendicular to the obstacle position.
        """
        if self.obstacle_near():
            obstacles = self.body.obstacle_coordinates()
            # Find the nearest obstacle:
            obs = min(obstacles, key=lambda v: v[0]*v[0]+v[1]*v[1])
            projection = (obs[0]*target[0] + obs[1]*target[1])/(obs[0]*obs[0] + obs[1]*obs[1])
            if projection > 0:
                target[0] -= obs[0]*projection
                target[1] -= obs[1]*projection
        return target

#...............................................................................................
class BaseBody(object):
    """Minimal model of Body with the required methods
    for use as a body of a robot. Any body
    models should inherit from this class to
    be accepted by BaseRobot.
    All the methods below should be overwritten
    by each body model.
    """

    def get_position(self):
        """Returns the x,y coordinates of the robot.
        """
        raise Exception("The body class in use has not defined a get_position() method")
        return [0., 0.]

    def get_heading(self):
        """Returns the angle between the robot orientation
        and the x-axis in radians between [-pi/2,pi/2].
        """
        raise Exception("The body class in use has not defined a get_heading() method")
        return 0.

    def move_forward(self,time,v=None):
        """Move forward the robot during *time* seconds at
        a speed *v* (m/s). If no speed is given, use a
        predefined one from the body class.
        """
        raise Exception("The body class in use has not defined a move_forward() method")
        return

    def rotate(self,dtheta):
        """Rotate the robot an angle *dtheta* in radians between [-pi/2,pi/2].
        Return the time in seconds it took to perform the rotation.
        """
        raise Exception("The body class in use has not defined a rotate() method")
        return 0.

    def get_ultrasound(self):
        """Return an array with the distances of the obstacles
        detected. Right now, it is assumed this will return at least 3
        values (this may change).
        """
        raise Exception("The body class in use has not defined a get_ultrasound() method")
        return [0., 0., 0.]

    def obstacle_coordinates(self):
        """Return a list with the coordinates of the obstacles detected
        in relation to the robot. This coordinates are relative to the
        robot but using the global orientation of the axis, i.e. this
        returns the displacement vector from the robot to the obstacles
        using global coordinates. (in other words, sets the [0.,0.]
        at the position of the robot but ignores its heading.)
        """
        raise Exception("The body class in use has not defined a obstacle_coordinates() method")
        x1 = [0.,0.]
        x2 = [0.,0.]
        x3 = [0.,0.]
        return [x1, x2, x3]

#...............................................................................................
class BaseNetwork(object):
    """Minimal model of Network with the required methods
    for use as a network of a robot. Any network
    models should inherit from this class to
    be accepted by BaseRobot.
    All the methods below should be overwritten
    by each body model.
    """

    def start_broadcasting(self):
        """Open the channel and setup what is needed to start broadcasting (send AND receive).
        Return the 'channel' used for communication, typically a Serial.Serial instance or
        some other class with read() and write() methods.
        """
        raise Exception("The network class in use has not defined a start_broadcasting() method")
        return None

    def stop_broadcasting(self):
        """Stop the transmission and close the required channels.
        """
        raise Exception("The network class in use has not defined a stop_broadcasting() method")
        return

    def get_agents_state(self):
        """Return a dictionary with the state (x,y,heading) of each robot
        detected in the network."""
        raise Exception("The network class in use has not defined a get_agents_state() method")
        return { "robot1":[0.,0.,0.] , "robot2":[0.,0.,0.], "robotN":[0.,0.,0.] }

    def send_state(self,position, heading):
        """Broadcast the current state (position[0], position[1], heading) of the robot
        over the network.
        Return the message sent.
        """
        raise Exception("The network class in use has not defined a send_position() method")
        return "Robot1\t0.\t0.\t0."

    # To use network with the "with Network(...) as network:" statement
    def __enter__(self):
        self.start_broadcasting()
        return

    # To use network with the "with Network(...) as network:" statement
    def __exit__(self, type, value, traceback):
        self.stop_broadcasting()
        return


