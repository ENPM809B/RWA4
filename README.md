# group2_rwa4
Build Instructions...

Change directory to "catkin_ws/src/" and run following command in the terminal.

```
git clone -b rachith https://github.com/ENPM809B/RWA4.git
cd ..
catkin_make --only-pkg-with-deps group2_rwa4
```

Run Instructions
...
1. Open terminal
2. Type following commands in the terminal
 ```
cd ~/catkin_ws
source devel/setup.bash
 ```

3. Run following commands in new terminals:
 ```
roslaunch group2_rwa4 group2_rwa4.launch
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1
rosrun group2_rwa4 main_node
```
