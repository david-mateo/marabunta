##Examples
Two example uses of the library are provided: one where a small swarm of three robots spreading away from each other is simulated and one used to control the Taurus (eBot+Raspberry+XBee) platform used in SUTD's MEC lab.

###spreading
This demo uses `MockBody` and `MockNetwork` to simulate the swarming robots. It build three different robots, each with its own ID, initial position, and initial heading, and performs the Perimeter Defense algorithm (spreading away).
To simulate the reaction of the swarm to the loss of agents, the agents are turned off at different times: one after 10 iterations, another after 20, and the last one after 60.

To run the demo, navigate to `examples/spreading/` and type
```
python spreading.py
```
The main script itself does not print anything to the screen, but the information broadcasted by each agent is recorded in "radio_{XX}.dat" where {XX} is the agent's ID.
After running the script, one can plot the trajectories of the robots by typing
```
gnuplot trajectory.plt -
```

###dr3
This demo uses `ebotBody` and `XBeeNetwork` to control one Taurus robot. The folder `examples/dr3demo/` contains four Python files:
  1. **settings.py**: The settings that are specific to each robot, namely its ID, its initial position and heading, and its time slot to broadcast messages. This is the only file that should change from robot to robot, and the three other scripts read the information contained in this one.
  2. **dr3_demo.py**: The main demo shown in DR3, performed by the updated Taurus platform. This scripts reads the settings of the robot from `settigns.py` and defines a `PerimeterDefenseRobot` using a `ebotBody`and a `XBeeNetwork`. The robot turns on and broadcast its state. Then, it will wait until it detects a certain number of neighbors (with a timeout) to start performing the spreading / perimeter defense algorithm. The current position and heading estimate is printed every `dt` seconds.
  3. **heading.py**: Same as `dr3demo.py` but performing heading consensus protocol.
  4. **leader.py**: Same as `dr3demo.py` but following a pre-defined path. Intented to be used in one of the robots while the others execute `heading.py` to show a leader-follower example of swarming.

To run the demo navigate to `examples/dr3/`, write the proper values on `settings.py`, and type
```
python dr3_demo.py {N}
````
where `{N}` is the number of agents expected to be transmitted (set to 5 if omitted).

###dr4
This build on DR3 demo and adds some improvements to efficiently operate larger swarms. Upon execution of the  main demo (`dr4_demo.py`) the program will set up and connect all the pieces, and then wait to received a "wake up" signal through XBee. This allows for a synchronous start of all the robots and also eliminates the need to have a stable ssh-able connection to a central computer.

The demo can be simulated running `dr4_simula.py` script, provided a good mapping of the desired room is given. An example of such a map is provided in `map_data.dat`, generated with real-world recorded ultrasensor data of eBots moving in Lab 2.714 of SUTD.

The results of the simulation can be visualized using the Gnuplot script `plot_res.plt`.
