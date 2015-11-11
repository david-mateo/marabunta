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
        return

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
        else:
            target= None
        return target

    def move_to_target(self, target, deltat, v, block=False):
        """If the norm2 of *target* is is larger
        than *threshold*, align the robot to
        *target* and move forward for *deltat*
        at a speed *v*.
        Else, stop for *deltat*.
        """
        d2 = target[0]*target[0] + target[1]*target[1]
        if d2 > self.threshold:
            # Some robots allow for a block argument in
            # the align method.
            try:
                self.body.align(target, block)
            except:
                self.align(target)
            self.move_forward(deltat, v)
        else:
            self.move_forward(deltat, 0.)
        return

    def light_detected(self):
        """If light is detected stop the robot
        and send the rendezvous signal.
        """
        try:
            light = self.body.light_detected()
        except AttributeError:
            light = False
        if light:
            self.move_forward(0.,0.)
            self.broadcast_rendezvous()
        return light

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
        # Perform swarming
        target = self.spread_target()
        if not target:
            h= self.body.get_heading()
            target = [1.5*sqrt(self.threshold)*cos(h) ,1.5*sqrt(self.threshold)*sin(h)]
        # Avoid obstacles
        target = self.correct_target_rotating(target)
        obstacle = self.obstacle_near()
        if obstacle and v:
            v *= 0.5
        self.move_to_target(target, deltat, v, obstacle)
        light = self.light_detected()
        return
