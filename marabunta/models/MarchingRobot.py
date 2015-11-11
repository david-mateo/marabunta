from marabunta import BaseRobot
from math import *

class MarchingRobot(BaseRobot):
    """Robot model for marching algorithm.
    By iteratively calling the update() method,
    this robot will communicate with the rest
    of the swarm and move in a way that
    simulatenouslty tries to
        [Spread] stay away from the closest neighbor ,
        [Heading] achieve heading consensus , and
        [Group] stay close to the group of neighbors.
    The importance of each of these three aspects can
    be set with the variables S, H and G.
    Typical values are given by default but the optimal
    parameters can drastically change depending on the
    properties of the agents and the desired outcome.
    Takes a *threshold* parameter to determine when it
    has reached a "good enough" state. Can be set to 0.
    Obstacle avoidance (implemented in BaseRobot)
    will take precence over consensus reaching.
    """

    def __init__(self, body, network, threshold=0.5, w_spread=2., w_heading=1., w_group=0.2):
        BaseRobot.__init__(self, body, network)
        self.threshold = threshold
        self.S = w_spread
        self.H = w_heading
        self.G = w_group
        return

    def march_target(self):
        """Compute the target direction of motion.
        This is computed as a linear combination
        of three vectors:
            1. The *spread* vector is computed as the
            direction that will maximize the distance
            between agents.
            (same as in PerimeterDefenseRobot)
            2. The *heading* vector is computed as the
            mean of the swarm alignment.
            (same as in HeadingConsensusRobot)
            3. The *group* vector is computed as the
            distance to the center of mass of the
            swarm, normalized with the standard
            deviation of the positions. This means
            that the concept of being near or far
            from the swarm is relative to its size
            and spread.
        Returns a vector pointing to the target
        direction. If no agents are detected,
        returns None.
        """
        neis = self.get_agents().values()
        if not neis:
            return None
        n_neis = len(neis)
        pos = self.body.get_position()
        s2, group_x , group_y , spread_x, spread_y , heading_x , heading_y = 0, 0, 0, 0, 0, 0, 0
        R0 = 0.35
        for nei in neis:
            dx = nei[0]-pos[0]
            dy = nei[1]-pos[1]
            d2 = dx*dx + dy*dy
            s2 += d2
            group_x += dx
            group_y += dy
            spread_x -= dx * R0 / d2
            spread_y -= dy * R0 / d2
            heading_x += cos(nei[2])
            heading_y += sin(nei[2])
        heading_x /= n_neis
        heading_y /= n_neis
        if n_neis > 1:
            # Only consider the COM if two or more
            # neighbors are detected. (otherwise s2=0)
            s2 = sqrt( s2/n_neis - (group_x/n_neis)**2 - (group_y/n_neis)**2 )
            group_x /= s2
            group_y /= s2
        else:
            group_x = 0.0
            group_y = 0.0

        return [ self.S*spread_x + self.H*heading_x + self.G*group_x , \
                 self.S*spread_y + self.H*heading_y + self.G*group_y ]


    def move_to_target(self, target, deltat, v):
        """If the norm2 of *target* is is larger
        than *threshold*, align the robot to
        *target* and move forward for *deltat*
        at a speed *v*.
        Else, stop for *deltat*.
        """
        d2 = target[0]*target[0] + target[1]*target[1]
        if d2 > self.threshold:
            self.align(target)
            self.move_forward(deltat, v)
        else:
            self.move_forward(deltat, 0.)
        return

    def update(self, deltat, v=None):
        """Perform one step of the consensus
        protocol. This is the main "behavior"
        of the robot. It consists of 4 steps:
            1. Broadcast its state.
            2. Perform swarming. In practice,
               this means computing the desired
               target direction of motion.
               (in this case, march in formation)
            3. Correct the desired target
               in order to avoid obstacles.
            4. Move in the desired target direction.
        """
        self.broadcast_state()
        # Perform swarming
        target = self.march_target()
        if not target:
            h= self.body.get_heading()
            target = [1.5*sqrt(self.threshold)*cos(h) ,1.5*sqrt(self.threshold)*sin(h)]
        # Avoid obstacles
        target = self.correct_target(target)
        self.move_to_target(target, deltat, v)
        return
