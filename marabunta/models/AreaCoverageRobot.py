from math import *
from marabunta.models import PerimeterDefenseRobot

class AreaCoverageRobot(PerimeterDefenseRobot):
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
        # Get both neighbors and obstacles, in relative coordinates
        points = [ [nei[0]-pos[0], nei[1]-pos[1]] for nei in neis] + self.body.obstacle_coordinates()
        if points:
            target = [0.,0.]
            for p in points:
                d2 = p[0]**2 + p[1]**2
                weight = (1.0/d2 )**1.5
                if d2>0:
                    target[0] -= p[0]*weight
                    target[1] -= p[1]*weight
        else:
            target= None
        return target

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
        # Perform swarming
        target = self.spread_target()
        if not target:
            h= self.body.get_heading()
            target = [10.*cos(h) ,10.*sin(h)]
        # Avoid obstacles
        target = self.correct_target(target)
        self.move_to_target(target, deltat, v)
        light = self.light_detected()
        return light
