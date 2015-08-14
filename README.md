cart_opt_ctrl
============

# Launch in Simulation

```bash
# Start Gazebo simulation
roslaunch rtt_lwr_gazebo lwr_gazebo.launch
# Start your controller in simulation
roslaunch cart_opt_ctrl run.launch sim:=true
```
# Launch on Hardware

```bash
# Start RTnet connection
rosrun lwr_scrits rtnet start
# Start your controller
roslaunch cart_opt_ctrl run.launch
```
