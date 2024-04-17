# Perception

## Launch Node calculating the 3D position of the sphere-target in relation to the BlueROV2

In a new terminal execute one of the following commands(remember to source first):

```bash
rosrun sphere_target_tracking sphere_target_tracking.py
```

The results will be published to the `sphere_pos` topic which is of the message type Pose().