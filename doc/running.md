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
python test_dcm_zmp_control.py
```
This will launch a test routine executing a sinusoid and raising the foot.

## Run the test with your own recorded movements

The test that needs to be run in order o execute your own prerecorded movement is

```
python test_dcm_zmp_control.py [testfolder]
```

where `[testfolder]` contains the following files:
```
CoM.dat
LeftFoot.dat
RightFoot.dat
WaistOrientation.dat
optionally, ZMP.dat (if not given, it is computed from the CoM)
```
where the CoM and the ZMP are in the following format:
```
[position3D velocity3D acceleration3D]
```
the feet are
```
[position6D velocity6D acceleration6D]
```
and the waist
```
[orientation3D angular_velocity3D angular_acceleration3D]
```

Pay attention that the lines of each file should *not* have trailing whitespace.
Actually, the velocity and acceleration information are only really needed for the CoM. For all other quantities, these values are not really employed, but they are needed due to how nd-trajectory-generator is implemented. You can set this quantities arbitrarily to zero.

If you are running a simulation, folder `[testfolder]` can be anyways in your computer.
If you are running an experiment on the robot, folder `[testfolder]` must be installed somewhere on the robot itself.

A quick-and-dirty way of installing new motions in the robot is putting them in sot-talos-balance.
In this case, you have to put it in
```
[sot-talos-balance-repo]/ros/sot_talos_balance/data
```
This way, when executing (see <a href="md_doc_installation.html">the installation page</a>)
```
make install
```
it will automatically be copied to
```
/opt/openrobots/share/sot_talos_balance/data
```
which will then be copied to the robot when you update sot-talos-balance on it.

If you which, you can access the motions in sot-talos-balance without specifying the full path, by doing
```
python test_dcm_zmp_control.py -0 [testfolder]
```
which will automatically look in
```
/opt/openrobots/share/sot_talos_balance/data/[testfolder]
```

## Interacting with the dynamic graph

If you want to dynamically interact with the graph

```
rosrun dynamic_graph_bridge run_command
```

## Other

More information on how to use the SoT and how to work on Talos can be found <a href="https://wiki.laas.fr/robots/Pyrene">in the robot wiki page</a> (you need LAAS permissions to access this).


