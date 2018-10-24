Author: Joshua Shaffer

-Off-board_ground
    - Fire_UAV_simulation

    ** NOTE: This section is not immediately relevant to the multi-UAV architecture to be developed. Can skip..
    These files make up two parts of the Off-board_ground/Fire_UAV_simulation directory. The first part is the controller synthesis, composed of:
        1. AutomaticGenerator_RevisedRH.py
        2. AutoGenFolders.py
        3. WaterControl.py

    1 and 3 are ran individually to generate the controllers, while 2 is ran before to create the necessary directory
    ** DO NOT DO THESE, instead request USB access from Joshua Shaffer **

    The second part is the simulator, composed of:
        1. Dynamics.py
        2. FireEnv.py
        3. FleetAndAgents.py
        4. Graph.py
        5. module_tester.py
        6. Parameters_dronekit_version.py
        7. SimFunctions.py
        8. Fire_Model_Eqns.py
        9. Allocation.py
        10. WaterControl_controller.py
        11. FireUAV_modules
        12. W_partitions (directory)

    FireUAV_modules.py is the primary module for manipulating all other modules and their respective classes, done through
    the creation of a "Simulation Object". Each have header comments
    describing their purpose, but not much else. The version of this simulation is a legacy version that doesn't work
    exactly as it should, but it's used primarily because it fits within the current iteration of the actual dronekit
    scripts and software. It serves as a synchronous test for now. Additionally, this work is/was overlapping with some other
    projects I have going on, so a few pieces (FleetAndAgents.py, Dynamics.py, Graph.py) are being created with modularity in
    mind.

    - OffBoard_simulator.py

        This is the ground-based script from when I assumed the project would utilize wifi communication to the drones.
        This script creates an instance of the Fire_UAV_Simulation simulation object to begin the synchronous, centralized
        controller. It sends updated graph nodes to the UAVs through TCP connections, and receives telemetry through
        such too, listening on a separate thread. Would work with real drones operating or through SITL (which is how
        we've been testing)

    - OffBoard_simulator_SITLtest.py

        This is the ground-based script from which I assumed that we could not use wifi communication. It is UNFINISHED,
        requiring a software bed to simulate the communication of Mavlink commands over radio. This requires a version
        of Ardupilot in which the vehicle parameters were modified to account for coordinated graphs between the UAVs
        and the ground


-On-board_UAV

    -Dynamics.py

        Equations of motion and integrator for the chosen dynamical system used in constructing the transitions for our
        graph

    -Graph.py

        Graph class that calculates allowable transitions between regions using the Dynamics class, builds parents
        and children for each node (need to set parameters so that they'll automatically match the OnBoard version)

    - OnBoard_controller.py

        DroneKit script that connects to the ground station over TCP (wifi assuming) and connects to an Ardupilot instance
        over TCP also (on-board). Currently setup to match the native SITL TCP instance. This utilizes queued nodes from
        the centralized controller at the base to implement a fixed duration transition across the nodes through the use
        of a feedforward/feedback control loop operating at a fixed frequency.

    -OnBoard_controller_SITLtest.py

        Version of the on-board controller that assumed queue updates were perceived through changes in the Ardupilot
        custom parameters. Unfinished (would work with the OffBoard_simulator_SITLtest.py in an SITL test)

- Initiate_clientUAVs_serverBase

    Bash script that opens 5 terminals, creating 2 separate ardupilot SITL instances, creating 2 separate onboard controllers
    to talk to these instances (OnBoard_controller.py), and creating an off board controller and simulator (OffBoard_simulator.py).
    It's written to work with my (Joshua Shaffer) directory setup on Mac, so you'd have to manually change those paths.

Send any questions my way to jshaffe9@terpmail.umd.edu