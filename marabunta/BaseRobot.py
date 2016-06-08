from math import  *
import threading
from time import sleep, time
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
        self.printing = False
        self.last_target = [0.,0.]
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
            self.stop_printing()
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
        Looks for an align method in *self.body*
        but one is not required.
        """
        if "align" in dir(self.body):
            dtheta = self.body.align(direction)
        else:
            dtheta = atan2( direction[1], direction[0]) - self.body.get_heading()
            if dtheta > pi:
                dtheta -= 2*pi
            elif dtheta < -pi:
                dtheta += 2*pi
            self.rotate(dtheta)
        return dtheta

    def go_to(self, target, tol=0.8, max_time = 120):
        """Move the robot to *target*.
        Calling this will make the robot
        move in a straight line towards the
        target point and block the main
        thread until the point is reached
        within *tol* accuracy or *max_time*
        seconds have passed.
        """
        end_time = time() + max_time
        v = self.body.max_speed
        while time() < end_time:
            delta = [target[0] - self.body.get_position()[0] ,
                     target[1] - self.body.get_position()[1] ]
            distance = sqrt(delta[0]*delta[0] + delta[1]*delta[1])
            if distance > tol:
                delta = self.correct_target_projecting(delta)
                self.align( delta)
                self.move_forward( distance/v , v)
                sleep(0.1)
            else:
                self.move_forward( 0., 0.)
                break
        return

    def follow_path(self,targets, tol=0.8, max_time_ppt=120):
        """Move the robot along the path
        specified by *targets*.
        Uses the self.go_to method.
        """
        for target in targets:
            self.go_to(target, tol, max_time_ppt)
        return

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

    def broadcast_state_obstacles(self):
        """If no obstacles are detected then it calls the send_state method instead."""
        if self.is_working():
            obstacles =self.body.obstacle_global_coordinates()
            if obstacles:
                self.network.send_state_obstacles(self.body.get_position(), self.body.get_heading(), obstacles )
            else:
                self.network.send_state(self.body.get_position(), self.body.get_heading())
        return


    def broadcast_obstacles(self):
        """Only send if obstacles are detected."""
        if self.is_working():
            obstacles =self.body.obstacle_global_coordinates()
            if obstacles:
                self.network.send_obstacles( obstacles )
        return

    def broadcast_rendezvous(self):
        """Broadcast a signal to call everyone
        to the robot's current location.
        """
        if self.is_working():
            x, y = self.body.get_position()
            message = "goto %.2f %.2f\n"%(x, y)
            self.network.send_message(message)
        return

# Obstacle avoidance methods:

    def obstacle_infront(self):
        """Return True if an obstacle is "in front", meaning
        that extreme measures such as stopping the movement
        have to be taken to avoid a collision.
        Used by move_forward() to stop the robot in case
        something is too close.
        """
        return self.body.obstacle_infront()

    def obstacle_near(self):
        """Return True if an obstacle is "near" meaning
        that the robot should be aware of the existence
        of the obstacle even if it may not collide
        directly.
        """
        return self.body.obstacle_near()

    def frontal_obstacle_coordinates(self, obs_coords):
        """Return the frontal projection of a list of coordinates.
        Assuming the obstacles form a straight line, the
        frontal projection is the point in that line closer to
        the robot.
        """
        n = len(obs_coords)
        if n:
            x = sum(o[0] for o in obs_coords)/n
            y = sum(o[1] for o in obs_coords)/n
            x2 = sum(o[0]*o[0] for o in obs_coords)/n
            y2 = sum(o[1]*o[1] for o in obs_coords)/n
            xy = sum(o[0]*o[1] for o in obs_coords)/n
            sx2 = x2 - x*x
            sy2 = y2 - y*y
            cxy = (xy - x*y)
            try:
                x0 = cxy*(x*xy - x2*y) / (cxy*cxy + sx2*sx2)
            except:
                x0 = x
            try:
                y0 = cxy*(y*xy - y2*x) / (cxy*cxy + sy2*sy2)
            except:
                y0 = y
        else:
            x0 = None
            y0 = None
        return (x0,y0)

    def correct_target(self, target):
        """Correct the target vector *target* so that if the
        robot moves in that direction it will not collide with
        obstacles.
        Current implementation: correct if obstacle_near(), not
        if obstacle_infront(). In case a correction is needed,
        choose the closest obstacle and project the target vector
        to be perpendicular to the obstacle position.
        """
        if self.body.obstacle_near():
            obstacles = self.body.obstacle_coordinates()
            #obs = self.frontal_obstacle_coordinates(obstacles)
            # Find the nearest obstacle:
            dists = [ v[0]*v[0]+v[1]*v[1] for v in obstacles]
            idx = dists.index( min(dists) )
            obs = obstacles[idx]
            projection = (obs[0]*target[0] + obs[1]*target[1]) / (obs[0]*obs[0] + obs[1]*obs[1])
            if projection > 0:
                if projection < 0.80:
                    target[0] -= 1.1*obs[0]*projection
                    target[1] -= 1.1*obs[1]*projection
                else:
                    # Choose left or right depending on last_target:
                    if obs[0]*self.last_target[1] - obs[1]*self.last_target[0] > 0.:
                        theta =  0.55*pi
                    else:
                        theta = -0.55*pi
                    target[0] = 100.*obs[0]*cos(theta) - 100.*obs[1]*sin(theta)
                    target[1] = 100.*obs[0]*sin(theta) + 100.*obs[1]*cos(theta)
        self.last_target = target[:]
        return target

    def correct_target_projecting(self, target):
        """Correct the target vector *target* so that if the
        robot moves in that direction it will not collide with
        obstacles.
        Current implementation: correct if obstacle_near(), not
        if obstacle_infront(). In case a correction is needed,
        choose the closest obstacle and project the target vector
        to be perpendicular to the obstacle position.
        """
        if self.body.obstacle_near():
            obstacles = self.body.obstacle_coordinates()
            # Find the nearest obstacle:
            dists = [ v[0]*v[0]+v[1]*v[1] for v in obstacles]
            idx = dists.index( min(dists) )
            obs = obstacles[idx]
            dist = sqrt(dists[idx])
            projection = (obs[0]*target[0] + obs[1]*target[1]) / (obs[0]*obs[0] + obs[1]*obs[1])
            if projection > 0:
                # Purely phenomenological urgency parameter here.
                urgency = 0.6*( 1. + 4.*max(0., 0.4-dist) )
                target[0] -= urgency*obs[0]*projection
                target[1] -= urgency*obs[1]*projection
        return target


    def correct_target_rotating(self, target):
        """Correct the target vector *target* so that if the
        robot moves in that direction it will not collide with
        obstacles.
        Current implementation: correct if obstacle_near(), not
        if obstacle_infront(). In case a correction is needed,
        move perpendicular to the closest obstacle. The perpendicular
        direction is chosen depending on what is the second closest
        obstacle. If the closest obstacle is detected by the
        first or last sensor, add a bit to the rotation (+pi/10)
        to compensate drift towards obstacles and increase the
        security distance.
        """
        if self.body.obstacle_near():
            obstacles = self.body.obstacle_coordinates()
            # Find the nearest obstacle:
            dists = [ v[0]*v[0]+v[1]*v[1] for v in obstacles]
            idx = dists.index( min(dists) )
            obs = obstacles[idx]
            # Choose left or right:
            if idx == 0:
                theta = -0.6*pi
            elif idx==len(dists)-1:
                theta = 0.6*pi
            elif dists[idx-1] < dists[idx+1]:
                theta = -0.5*pi
            else:
                theta = 0.5*pi
            target[0] = obs[0]*cos(theta) - obs[1]*sin(theta)
            target[1] = obs[0]*sin(theta) + obs[1]*cos(theta)
        return target

# Data collection methods:

    def background_print(self,dt):
        """Print position + obtacles detected in global coordinates.
        Format:
            #pose   iter x   y   heading
            #wall   iter x   y   distance_to_robot
        It ignores any obstacle detected at less than 40cm from a known
        agent and any obstacle at more than 60cm from the robot.
        """
        print_iter = 0
        while self.printing:
            x , y = self.body.get_position()
            h = self.body.get_heading()
            obstacles = self.body.obstacle_coordinates()
            agents = self.get_agents().values()
            # Check if an obstacle is an agent.
            walls = []
            for o in obstacles:
                if all([(o[0]-a[0])**2 + (o[1]-a[1])**2 > 0.40**2 for a in agents]):
                    walls.append(o)
            # print poses and walls
            print("#pose\t%i\t%.3f\t%.3f\t%.3f"%(print_iter,x,y,h))
            for wall in walls:
                print("#wall\t%i\t%.3f\t%.3f\t%.3f"%(print_iter,wall[0]+x,wall[1]+y, sqrt(wall[0]**2 + wall[1]**2)))
            sleep(dt)
            print_iter += 1
        return

    def start_printing(self,dt):
        """Launch a new thread running background_print."""
        self.printing = True
        self.print_thread = threading.Thread(target = self.background_print , args = (dt,) )
        self.print_thread.daemon = True
        self.print_thread.start()
        return

    def stop_printing(self):
        """Stop the print thread."""
        if self.printing:
            self.printing = False
            self.print_thread.join(10)
            if self.print_thread.is_alive():
                raise Exception("marabunta.BaseRobot.stop_printing: Could not stop printing thread properly")
        return


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

    def obstacle_global_coordinates(self):
        """Same as obstacle_coordinates but setting
        the origin of coordinates to the ground truth
        origin and not relative to the robot position.
        """
        pos = self.get_position()
        return [ (ob[0]+pos[0], ob[1]+pos[1]) for ob in self.obstacle_coordinates()]

    def obstacle_infront(self):
        """Return True if an obstacle is "in front", meaning
        that extreme measures such as stopping the movement
        have to be taken to avoid a collision.
        Used by move_forward() to stop the robot in case
        something is too close.
        """
        raise Exception("The body class in use has not defined a obstacle_infront() method")
        return False

    def obstacle_near(self):
        """Return True if an obstacle is "near" meaning
        that the robot should be aware of the existence
        of the obstacle even if it may not collide
        directly.
        """
        raise Exception("The body class in use has not defined a obstacle_near() method")
        return False

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


