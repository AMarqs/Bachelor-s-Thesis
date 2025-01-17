# Autonomous tracking

### *Pure-pursuit reactive path planning with a scale car using ROS and Gazebo*

<!-- IDIOMAS ------------------------------------------------------------------------------------------------------------------->
<div>
    <p align="left">
        <a href="/README.md">Español<a> · <b>English</b>
    </p>
</div>

<br>

## Execution steps

1. Locate in the workspace:
```
cd catkin_racecar
```

2. Generate the working environment:
```
source devel/setup.bash:
```

3. Execute the control by command:
```
roslaunch racecar_control gazebo_sim_joy.launch
```

4. Execute the tracking algorithm:
```
rosrun path_tracker follower.py
```
