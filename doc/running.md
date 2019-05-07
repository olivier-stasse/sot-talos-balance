# Running a test

In the following, we quickly demonstrate how to run a test with sot-talos-balance.

## Start the simulation

First of all, you need to start the simulation.

To start Gazebo, load a scene and spawn Talos, the simplest way is to directly use the launch file provided by PAL, without een using sot-talos-balance
```
roslaunch talos_gazebo talos_gazebo.launch
```

Notice that this will spawn Talos at configuration zero. This is not always what you want.
The package sot-talos-balance offers different launch files to spawn it at different configurations. 

Most commonly, you might want to spawn the robot in the half-sitting position
```
roslaunch sot_talos_balance talos_gazebo_half_sitting.launch
```

If you ever need a different configuration, all you have to do is taking talos_gazebo_half_sitting.launch, copying it with a different name and modifying it.

## Start the SoT in position mode

To start the SoT in simulation in position mode: 
```
roslaunch roscontrol_sot_talos sot_talos_controller_gazebo.launch
```

## Run the test

First of all, you need to go to the folder where your script is.
For instance, for running the standard tests of sot-talos-balance,
assuming you are in the root directory:

```
cd python/sot_talos_balance/test
```

Then, you can just run the chosen test. For instance:

```
python test_dcmZmpControl_file.py
```

## Interacting with the dynamic graph

If you want to dynamically interact with the graph

```
rosrun dynamic_graph_bridge run_command
```

## Other

More information on how to use the SoT and how to work on Talos can be found <a href="https://wiki.laas.fr/robots/Pyrene">in the robot wiki page</a> (you need LAAS permissions to access this).


