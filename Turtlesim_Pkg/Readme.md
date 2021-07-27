First, create a package named Turtle_Pkg, within your catkin workspace. Once done, compile and source the packages.
 
``` 
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

Within this package, you should have a scripts folder inside which you'll create a python script, named node_turtle_revolve.py.

After completing the python script. Make it executable, if it isn't already. To do that, enter the following code.

Before executing make sure that roscore is running along with turtlesim_node. You can either run them in separate terminals or simply create a node_turtle_revolve.launch file inside the ~/catkin_ws/src/Turtlesim_pkg/launch/ folder. Launch file can run multiple nodes unlike a python/cpp script. Run the launch file, enter,

```
roslaunch pkg_task0 node_turtle_revolve.launch 
```
