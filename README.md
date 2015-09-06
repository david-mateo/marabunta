#marabunta
A Python library for the design and control of artificial swarms.

##Overview
This package includes tools to either control or simulate specific hardware parts of a robot capable of locomotion and communication with the aim of performing experimental research in artificial swarming. It also includes some models of swarming behavior built on said tools.
The design of robots using this library consists of three main pieces:

1.  The body the robot uses to move and sense the environment.
2.  The network the robot uses to communicate with other robots.
3.  The behavior or protocol the robot follows, i.e. what it does with the information received from the body and the network.

The marabunta library follows this structure and provides the following classes:

* **BaseBody**: [[src/BaseRobot.py]](src/BaseRobot.py) Minimal model of Body with the required methods to use as a body of a robot. Any body models should inherit from this class to be accepted by BaseRobot.
    * **MockBody**:[[src/MockBody.py]](src/MockBody.py) Body implementation to simulate a robot body. Does not require any hardware to use.
    * **ebotBody**:[[src/ebotBody.py]](src/ebotBody.py) Body implementation to control an [eBot](http://edgebotix.com/). Requires bluetooth connection, an eBot, and the appropiate eBot-API installed.
* **BaseNetwork**: [[src/BaseRobot.py]](src/BaseRobot.py) Minimal model of Network with the required methods to use as a network of a robot. Any network models should inherit from this class to be accepted by BaseRobot.
    * **MockNetwork**: [[src/MockNetwork.py]](src/MockNetwork.py) Network implementation to simulate the communication using regular files (assumes the different robots are in the same computer, or at least can access the same files). Does not require any hardware to use.
    * **XBeeNetwork**: [[src/XBeeNetwork.py]](src/XBeeNetwork.py) Network implementation using a series 1 XBee. Requires an XBee connected through a serial port.
* **BaseRobot:** [[src/BaseRobot.py]](src/BaseRobot.py) Contains the basic tools to operate a robot. It requires a _body_ instance that inherits from `BaseBody` and a _network_ instance that inherits from `BaseNetwork`.
    * **HeadingConsensusRobot**: [[src/robot_models/HeadingConsensusRobot.py]](src/robot_models/HeadingConsensusRobot.py) Implementation of a robot following a heading consensus algorithm. Aligns its heading to the average heading of the swarm. 
    * **PerimeterDefenseRobot**: [[src/robot_models/PerimenterDefenseRobot.py]](src/robot_models/PerimenterDefenseRobot.py) Implementation of a robot performing perimeter defense. It moves away as far as possible from other robots.
    * **MarchingRobot**: [[src/robot_models/MarchingRobot.py]](src/robot_models/MarchingRobot.py) Implementation of a robot marching in formation. It simulataneously tries to keep a safe distance with the closests robot, keep close enough to the rest of the swarm, and keep its heading aligned to the swarm heading.

##Installation
To install the different modules included here, navigate to `src/` and type:
```Bash
make install
```
This will copy compiled versions of each module onto the user installation folder, as defined by
```Bash
python -m site --user-site
```

**Note:** this is a very rudimentary way of distribution. Assistance on cleaner ways to package several .py files in a single module is very welcomed and appreciated.

###eBot API
To control eBots through `ebotBody`, you will also need to install the eBot-API. You can find the official version at https://github.com/EdgeBotix/eBot-API.
A fork of this API that uses the host CPU to compute the localization of the robot by implementing a Kalman filter instead of relying on the eBot localization can be found at https://github.com/david-mateo/eBot-API. 

##Using this library

To design a robot behavior, one should define a new class that inherits from `BaseRobot`. The initialization of `BaseRobot` requires of a body, implemented as a class inheriting from `BaseBody`, and a network, a class inheriting from `BaseNetwork`.

To add support for new hardware, one should implement classes inheriting from `BaseBody` or `BaseNetwork`. These classes contain the minimal list of methods than any body or network should implement. 

To make a robot move following a particular behavior, say heading consensus, it should run a program like this:
To use the provided methods to make the robot move following a particular behavior, say heading consensus, one has to define the body, the network, the robot, turn it on, and iteratively call its `update` method. A minimal example code is:
```python
total_time = 60
ID="Walle"
init_pos = [0., 0.]
init_heading = 0.
communication_slot = 0.1
body = ebotBody( init_pos , init_heading)
network = XBeeNetwork( communication_slot, communication_slot+0.1, 1, ID )
robot = HeadingConsensusRobot( body, network )
robot.turn_on()

# MAIN LOOP
end_time = time() + total_time
while time() < end_time:
    robot.update(dt, speed)
    sleep(dt)

robot.turn_off()
```
One can find several ways to operate the robots in the scripts contained in `examples/`.


