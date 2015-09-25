from BaseRobot import BaseRobot
from math import sin,cos,pi

class HeadingConsensusRobot(BaseRobot):
    """Robot model for heading consensus.
    By iteratively calling the update() method,
    this robot will communicate with the rest
    of the swarm and align its heading to the
    swarm's mean heading.
    Obstacle avoidance (implemented in BaseRobot)
    will take precence over consensus reaching.
    """

    #def __init__(self, body, network):
    #    BaseRobot.__init__(self, body, network)
    #    return

    def heading_target(self):
        """Get the other agent's state and
        compute the mean heading. Note that
        for periodic quantities such as the
        heading, the mean is defined as

            < x_i > = atan( sum_i sin(x_i)/sum_i cos(x_i) )

        Returns a vector pointing to the
        mean heading. If no agents are
        detected, returns None.
        """
        neis = self.get_agents().values()
        if neis:
            sint = sum( [sin(nei[2]) for nei in neis])
            cost = sum( [cos(nei[2]) for nei in neis])
            target = [cost, sint]
        else:
            target = None
        return target

    def move_to_target(self, target, deltat, v):
        """Align the robot to *target* and
        move forward for *deltat* at a speed *v*.
        """
        self.align(target)
        self.move_forward(deltat, v)
        return

    def update(self, deltat, v=None):
        """Perform one step of the consensus
        protocol. This is the main "behavior"
        of the robot. It consists of 4 steps:
            1. Broadcast its state.
            2. Perform swarming. In practice,
               this means computing the desired
               target direction of motion.
               (in this case, perform heading
               consensus)
            3. Correct the desired target
               in order to avoid obstacles.
            4. Move in the desired target direction.
        """
        self.broadcast_state()
        # Perform swarming
        target = self.heading_target()
        if not target:
            h= self.body.get_heading()
            target = [cos(h) ,sin(h)]
        # Avoid obstacles
        target = self.correct_target(target)
        self.move_to_target(target, deltat, v)
        return
