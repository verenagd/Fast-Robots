+++
title = "Lab 11"
+++

## Prelab

Before beginning this lab, I reviewed the lectures on Sensor Models, Motion Models, and the Bayes Filter. I also revisited the simulation setup from Lab 10 to familiarize myself with the localization framework.

The Bayes filter estimates the robot's pose on a discrete grid map by combining a prior belief with sensor measurement updates. In this lab, only the **update step** is used — the prediction step is skipped because real robot motion is too noisy to be useful. The robot starts with a **uniform prior**, meaning it is assumed to be equally likely to be anywhere on the map before any sensor data is collected.

## Task 1 - Testing Localization in Simulation

Below is a screenshot of the final plot after having run the provided `lab11_sim.ipynb`.

<img src="/Fast-Robots/Test_Localization.png">

The blue represents the 'Belief', the red is the 'Odometry' readings, and the green is the 'Ground Truth'.

## Task 2 - Run the Update Step to Localize the Robot

In the provided `lab11_real.ipynb` file, I implemented the python code that I had used for Lab 9 that makes the car turn and take ToF measurements as it goes. In this case, I changed the sample size to be 18, such that the car performs a ToF sensor reading every 20 degrees. This is also different from Lab 9, as now the car only takes one measurement when it is at the desired orientation. Previously, the car would constantly be taking measurements without pause.

Since the output of the member function `perform_observation_loop()` needs to be a numpy column array, as required by the provided localization code, I added this in to the original python code:

``` python
for step in range(STEPS):
    target = target_for_step(step)
    idx = np.argmin(np.abs(yaws - target))
    sensor_ranges.append(tofs[idx] / 1000.0) 
    sensor_bearings.append(target)
    print(f"  target={target}°, actual={yaws[idx]}°, ToF={tofs[idx]}mm")

```

<div style="display: flex; gap: 16px; flex-wrap: wrap;">

  <figure style="flex: 1; min-width: 180px; margin: 0;">
    <img src="/Fast-Robots/PLOT_NEG3_NEG2.png" alt="A">
    <figcaption>Position 1: -3 ft, -2 ft, 0 deg</figcaption>
  </figure>

  <figure style="flex: 1; min-width: 180px; margin: 0;">
    <img src="/Fast-Robots/0_3.png" alt="B">
    <figcaption>Position 2: 0 ft, 3 ft, 0 deg</figcaption>
  </figure>

  <figure style="flex: 1; min-width: 180px; margin: 0;">
    <img src="/Fast-Robots/5neg3.png" alt="C">
    <figcaption>Position 3: 5 ft, -3 ft, 0 deg</figcaption>
  </figure>

  <figure style="flex: 1; min-width: 180px; margin: 0;">
    <img src="/Fast-Robots/5_3.png" alt="C">
    <figcaption>Position 4: 5 ft, 3 ft, 0 deg</figcaption>
  </figure>

</div>

The Bayes filter localized the robot poorly for position 1, as it is off by roughly 2' in x and 1.4' in y. This is in the lower-left region of the map, which is a relatively open and featureless area. The lack of nearby geometry, such as corners and obstacles makes it harder for the sensor model to uniquely identify the pose.

For position 2, the x error is about 0.5' and the y is ~2'. This position is near the notch in the upper wall, which provides some directional cues, but the robot may have been localized to a region with similar wall distances in the y direction, causing the offset.

Position 3 is the worst-performing position. The (5 ft, -3 ft) location is in the lower-right corner of the map, near the boundary a region where sensor readings may be ambiguous or where the ray-cast model has difficulty distinguishing it from other positions with similar wall profiles.

The robot localized most accurately at Position 2 (0 ft, 3 ft), where the belief was within roughly half a foot in x. Localization was worst at Positions 3 and 4 on the right side of the map. This likely reflects the geometry of the environment: the left and upper portions of the map feature the distinctive L-shaped notch in the outer wall, which produces unique distance signatures across different headings. The right side of the map, by contrast, is a more open rectangular region, where many poses yield similar sets of ToF readings, making it harder for the update step alone to disambiguate the robot's location. In general, poses near corners or irregular wall features tend to localize better than those in open, symmetric spaces.