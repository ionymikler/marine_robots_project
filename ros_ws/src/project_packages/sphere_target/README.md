# Sphere Target

## Launch Environment with Sphere Target

Execute from the ROS workspace:
```bash
roslaunch bluerov2_gazebo start_project_env.launch
```

## Move Sphere Target via Keyboard

In a new terminal execute (remember to source first):
```bash
roslaunch uuv_teleop uuv_keyboard_teleop.launch uuv_name:=sphere_target
```

## Move Sphere Target via ROS Node

In a new terminal execute one of the following commands(remember to source first):
```bash
rosrun sphere_target_motion target_motion_line.py
```
```bash
rosrun sphere_target_motion target_motion_square.py
```
```bash
rosrun sphere_target_motion target_motion_random.py
```