from marabunta import BaseRobot
from math import *

class PerimeterDefenseRobot(BaseRobot):
    """Robot model for perimeter defense.
    By iteratively calling the update() method,
    this robot will communicate with the rest
    of the swarm and move away from the others
    as far as possible. Takes a *threshold*
    parameter to determine when it has gone
    far enough and reached consensus. Can be
    set to 0.
    Obstacle avoidance (implemented in BaseRobot)
    will take precence over consensus reaching.
    """
    def __init__(self, body, network, threshold):
        BaseRobot.__init__(self, body, network)
        self.threshold = threshold
        self.rendezvous_point = None
        self.path = []
        self.known_lights = []
        self.num_lights = 0
        return

    
    def set_path(self, path):
        self.path = path[:]
        return self.path

    def spread_target(self):
        """Get the other agent's state and
        compute the direction of motion that
        will maximize distance with them.
        This is computed as a linear combination
        of the positions of each neighbor
        relative to the agent, where each
        position is weighted by the inverse
        of the distance**2 to that robot,

            t_i = sum_j (r_j - r_i)/|r_j - r_i|^2 ,

        so that higher priority is given to
        move away from the closer agents, but
        still taking all into account and
        allowing for neighbors to "cancel each
        other out."
        Returns a vector pointing to the
        mean heading. If no agents are
        detected, returns None.
        """
        neis = self.get_agents().values()
        pos = self.body.get_position()
        if neis:
            target = [0.,0.]
            for nei in neis:
                d2 = (nei[0]-pos[0])**2 + (nei[1]-pos[1])**2
                if d2>0:
                    target[0] += (pos[0] - nei[0])/d2
                    target[1] += (pos[1] - nei[1])/d2
        
            norm2 = target[0]*target[0] + target[1]*target[1]
            if norm2 < self.threshold:
                target = None
        else:
            target = None
        return target

    def rendezvous_target(self):
        """Compute the target direction of movement
        that allows the robot to reach the rendezvous point
        (stored in self.rendezvous_point).
        When the robot is close enough to the point this
        sets self.rendezvous_point to None and also returns
        None as the target.
        """
        if self.rendezvous_point:
            pos = self.body.get_position()
            target = [ self.rendezvous_point[0]-pos[0] , self.rendezvous_point[1]-pos[1] ]
            distance = sqrt(target[0]*target[0]+target[1]*target[1])
            if distance < 0.10: # rendezvous point reached
                try:
                    self.rendezvous_point = self.path.pop(0)
                    target = self.rendezvous_target()
                except:
                    target = [0., 0.]
                    self.rendezvous_point = None
        else:
            try:
                self.rendezvous_point = self.path.pop(0)
                target = self.rendezvous_target()
            except:
                target = None
                self.rendezvous_point = None
        return target



    def move_to_target(self, target, deltat, v, block=False):
        """If the norm2 of *target* is is larger
        than *threshold*, align the robot to
        *target* and move forward for *deltat*
        at a speed *v*.
        Else, stop for *deltat*.
        """
        if target[0]**2 + target[1]**2 > self.threshold*self.threshold:
            # Some robots allow for a block argument in
            # the align method.
            try:
                self.body.align(target, block)
            except (TypeError,AttributeError):
                self.align(target)
            self.move_forward(deltat, v)
        else:
            self.move_forward(deltat, 0)
        return

    def light_detected(self):
        """If light is detected and is a
        new light, broadcast its positon
        and add it to the list of known
        light sources.
        """
        try:
            light = self.body.light_detected()
        except AttributeError:
            light = False
        if light:
            x, y = self.body.get_position()
            self.add_light(x,y)
        return light

    def process_messages(self):
        messages = self.network.get_messages()
        for message in messages:
            if len(message)>3:
                mesdata = message.split()
                if mesdata[0]=="stop":
                    raise Exception("Stop!")
                elif mesdata[0]=="goto":
                    try:
                        self.rendezvous_point =  (float(mesdata[1]), float(mesdata[2]))
                    except:
                        print("#PerimenterDefenseRobot: Strange message received: ",message)
                elif mesdata[0]=="light":
                    try:
                        x, y = float(mesdata[1]), float(mesdata[2])
                    except:
                        x, y = None, None
                        print("#PerimenterDefenseRobot: Strange message received: ",message)
                    self.add_light(x,y)
        return messages

    def add_light(self, x, y):
        """Only add light to the list of known lights if 
        this new one is at least 0.8 from any other
        previously known light.
        """
        if all( (x-light[0])**2 + (y-light[1])**2 > 0.8 * 0.8 for light in self.known_lights):
            self.known_lights.append( (x,y) )
            self.num_lights += 1
            self.network.send_message("light\t%.2f\t%.2f\n"%(x,y))
        return

    def update(self, deltat, v=None):
        """Perform one step of the consensus
        protocol. This is the main "behavior"
        of the robot. It consists of 4 steps:
            1. Broadcast its state.
            2. Perform swarming. In practice,
               this means computing the desired
               target direction of motion.
               (in this case, perform perimeter
               defense)
            3. Correct the desired target
               in order to avoid obstacles.
            4. Move in the desired target direction.
        """
        self.broadcast_state()
        self.process_messages()
        # If goto message received, go there
        target = self.rendezvous_target()
        # check if rendezvous point has been reached
        if target and target[0]==0 and target[1]==0:
            return False, True # STOP HERE!
        if not target:
            # Perform swarming
            target = self.spread_target()
        if not target:
            h= self.body.get_heading()
            target = [10.*sqrt(self.threshold)*cos(h) ,10.*sqrt(self.threshold)*sin(h)]
        # Avoid obstacles
        target = self.correct_target(target)
        obstacle = self.obstacle_near()
        if obstacle and v:
            v *= 0.6
        self.move_to_target(target, deltat, v, obstacle)
        light = self.light_detected()
        return light, False
