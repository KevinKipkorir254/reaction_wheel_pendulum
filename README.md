![REACTION WHEEL](/extra_resources/Screenshot%202024-11-30%20182851.png)

## HOW TO RUN THE PROJECT.
1. Go to the workspace and source the project;

```bash
reaction_wheel_pendulum_ws
source install/setup.bash
```

2. Launch the simulated gazebo robot;

```bash
ros2 launch  reaction_wheel_pendulum_description gazebo_complete.launch.xml
```

3. The run the controller on another bash:

```bash
colcon build --symlink-install
ros2 launch reaction_wheel_pendulum_control lqr_swinger.launch.py
```
The symlink install allows for changing the gain parameters in the yaml file and them taking effect in real time:

```yaml
/**:
  ros__parameters:
    K: [ -141.4214, -77.6558, -238.7684, -36.5906]

    kd: 1.0   #-->
    ke: 1.0   #-->
    kv: 1.0   #-->

    # 0.0 angle - upward position. When absolute angle is less than this threshold
    # transition from swing up to LQR
    lqr_transition_angle: 0.523599
```

4. I can also have the following plotter:

```bash
ros2 run plotjuggler plotjuggler
```

### Gazebo required

Here are some dependancies you will need.

```bash
sudo apt-get install gazebo
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
```

Here are is a command you will need

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```


TODO: I might add an initializer where I have a force to one direction at first to have a large initial oscillation before starting the project or I might just start at an angle to save time.
