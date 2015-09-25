from marabunta import MockBody, MockNetwork
from marabunta.models import PerimeterDefenseRobot

dt=1.0
log = open("the_cloud.dat","w+")

settings = []
settings.append( {"ID":1 , "pos":[ 0., 0.] , "heading":0.} )
settings.append( {"ID":2 , "pos":[ 2., 0.] , "heading":0.} )
settings.append( {"ID":3 , "pos":[ 0.,-1.] , "heading":0.} )

def new_robot(s, log):
    body = MockBody(s.get("pos"), s.get("heading") )
    network = MockNetwork(log,s.get("ID"))
    bot = PerimeterDefenseRobot( body, network, 0.02)
    bot.turn_on()
    return bot

robots = [ new_robot(s,log) for s in settings ]

for t in range(10):
    [robot.update(dt) for robot in robots]
robots[2].turn_off()
for t in range(10):
    [robot.update(dt) for robot in robots]
robots[0].turn_off()
for t in range(40):
    [robot.update(dt) for robot in robots]

[robot.turn_off() for robot in robots]
log.close()
