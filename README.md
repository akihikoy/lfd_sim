lfd_sim
==================
ODE simulation stuff including pouring, a hopping robot.
lfd_sim supports ROS (Robot Operating System) architecture.

ROS:
http://wiki.ros.org/

Author
==================
Akihiko Yamaguchi, http://akihikoy.net/


Requirements
==================
- ROS core system, rospy, roscpp, std_msgs, std_srvs, geometry_msgs, tf
- ODE: we assume that the ODE is built in `$HOME/prg/libode/ode-latest/` from source.
  - Use `./configure --enable-double-precision --disable-asserts` to setup.
  - Build drawstuff as well.
- Python: core, numpy


Build
==================
The repository directory should be in ROS workspace (e.g. ~/ros_ws/).
Build lfd_sim with rosmake.

```
$ rosmake lfd_sim
```

After rosmake, you will find some executables in bin/ directory.
There are some build directories made by ROS.


Programs
==================
Programs can be executed by "launch" files stored in launch/ directory.

```
$ roslaunch lfd_sim LAUNCH_FILE.launch
```

**launch/ode_grpour_sim1.launch**:
Pouring simulator with a gripper.

```
Space:  Pause/Resume
r: reset the simulator.
x,z: rotate the source container.
q,w,e,a,s,d: move the source container.
Ctrl+w: start to capture snapshots (files are saved in frame/ directory).
(on terminal) Ctrl+c: quit the program.
```

launch/ode_grpour_sim_video.launch, launch/ode_grpour_sim_video2.launch start the same program with different window sizes.


**launch/ode_hopper1_1.launch**:
A hopping robot simulator.

```
Space:  Pause/Resume
r: reset the simulator.
x,z: change the joint angles.
a,A,s,S,d,D,f,F: change each joint angle. a and A are opposite directions.
Ctrl+w: start to capture snapshots (files are saved in frame/ directory).
(on terminal) Ctrl+c: quit the program.
```


Troubles
==================
Send e-mails to the author.  I accept only from my students.
