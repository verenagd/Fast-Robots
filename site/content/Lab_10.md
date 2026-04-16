+++
title = "Lab 10"
+++

## Prelab
In this lab, we will be setting up a virtual model of a robot navigating a space. 

## Open Loop Control

I started out with open loop control, making the robot follow a set of velocity commands to turn in a square shape.

Here is the code, along with the car running and the ground truth and odometry being plotted alongside it.

<video controls width="100%">
  <source src="/Fast-Robots/square.mp4" type="video/mp4">
</video>

```python
while cmdr.sim_is_running() and cmdr.plotter_is_running():
    pose, gt_pose = cmdr.get_pose()
    cmdr.set_vel(0,1.5)
    time.sleep(1.05)
    cmdr.set_vel(0.8,0)
    time.sleep(0.5)
    cmdr.plot_odom(pose[0], pose[1])
    cmdr.plot_gt(gt_pose[0], gt_pose[1])
    sensor_values = cmdr.get_sensor()
```

### What is the duration of a velocity command?

I made the robot turn for 1.05s, since the units are in radians/second and I set to turn at about 1.5 radians/second (~90 degrees). I then let the car run straight for 0.5s at 0.8m/s. Depending on how long each loop execution takes, it takes at least 1.55s for it to run.

### Does the robot always execute the exact same shape?

No, since it doesn't turn at exactly 90 degrees, overtime the error compounds and the robot starts to move in a different shape. 

## Closed Loop Control

I now designed a simple closed loop controller that took in the distance rom the wall and radomly decides to turn left or right.

```python
import random
cmdr.reset_plotter()
cmdr.reset_sim()
direction = [-4, 4]
while cmdr.sim_is_running() and cmdr.plotter_is_running():
    pose, gt_pose = cmdr.get_pose()
    if cmdr.get_sensor() < 0.5:
        cmdr.set_vel(0,random.choice(direction))
        time.sleep(1)
    cmdr.set_vel(2,0)
    cmdr.plot_odom(pose[0], pose[1])
    cmdr.plot_gt(gt_pose[0], gt_pose[1])
    sensor_values = cmdr.get_sensor()
```

<video controls width="100%">
  <source src="/Fast-Robots/avoiding_obs.mp4" type="video/mp4">
</video>

### By how much should the virtual robot turn when it is close to an obstacle?
I decided to turn by 90 degrees using the same method as before. However, I did increase the speed for the robot to turn 4 radians/s compared to the previous 1.5 rad/s.

### At what linear speed should the virtual robot move to minimize/prevent collisions? Can you make it go faster?

The faster I make the virtual robot go, the greater the distance it reads must be to turn in time. At a reading distance of 0.5m, the fastest I could make it reliably move was about 2m/s.

### How close can the virtual robot get to an obstacle without colliding?

At 2 m/s, 0.3m was approximately the shortest distance that let the car successfully go towards the wall and turn.

### Does your obstacle avoidance code always work? If not, what can you do to minimize crashes or (may be) prevent them completely?

Previously, I changed the car's velocity and minimum distance from the wall to turn.However, even after finding the right values for this, the robot would still graze past the boundary/objects.

One of the best ways to fix this would be if the distance measurement was not just a straight line, but a cone such that the robot does not graze by obstacles that is not detected in its limited stright line of sight.