The solution to running the it so that gazebo, the physics egnine works is including, add the following to my bashrc
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/kevo/ros_ws/src/ros_arm/src
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/kevo/arduino_bot/src
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/kevo/ws_moveit2/src
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/kevo/ros_ws/src/arduinobot_ws/src
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/kevo/ros_ws/src/ai_based_sorting_robot_arm/src
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/kevo/robotics_inc/under_actuated_ststems/cart_pole_ws/src/cart_pole
```