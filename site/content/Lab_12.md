+++
title = "Lab 12"
+++


## Prelab

For this final lab, I decided to pursue the inverted pendulum challenge, where the car is upright and tries to balances itself using closed-loop PD control. This was achieved by continuously reading the car's pitch from the IMU's readings.

## Lab

For this lab, I created the following cases to call when performing a run on the Python side:

**START_BALANCE** takes Kp and Kd gain, and a left/right motor calibration factor to initialize the controller and begins the balacing sequence

**STOP_CAR** immediately sets both motor PWM values to 0.

**SEND_DATA** sends over logged pitch and PWM data from the run via BLE

**GET_PITCH** reads the current pitch angle and send it over BLE. This is primarly used to initalize the starting position and for debugging purposes.

My first attempt useded the ICM-20948's DMP to get quaternions and converted them to a pitch angle with influence from Stephan Wagner's:

``` c++
double sinp = 2.0 * (qw * qy - qx * qz);
double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
pitch_out = atan2(sinp, cosy) * 180.0 / PI + 90.0;
```

This worked sometimes, however, sometimes the car would read 0 degrees when upright, sometime 90, and sometimes 180. This made every run unpredictable and at times make the controller drive the motors in the wrong direction.

Instead, I decided to compute the gravity vector in the sensor frame directly from the quaternion, then take the angle of that vector. This meant that the DMP does not initiliaze differently upon every bootup. 

```c++
double gx = 2.0 * (qx*qz - qw*qy);
double gy = 2.0 * (qw*qx + qy*qz);
double gz = qw*qw - qx*qx - qy*qy + qz*qz;
pitch_out = atan2(gy, gz) * 180.0 / PI;
```

I empirically determined which axis to use, by selecting the one that gave a consistent value of -90 or 90 when horizontal and 0 degrees when vertical. 

Instead of hard-coding a target angle, the START_BALANCE case captures the pitch initially and uses that as the setpoint using the following code:

```c++
float p = 0;
for (int i = 0; i < 10; i++) {
    readDMPPitch(p);
    delay(10);
}
target_pitch = p;
```

I would first hold the car up, run **GET_PITCH** to see if I'm holding it upright, and then run **START_BALANCE**

I decided to use PD control using the following code:

```c++
float lean  = pitch_g - target_pitch;
float error = lean;
float derivative = (error - prev_error) / dt;

float u = (prop_gain * error) + (der_gain * derivative);
int pwm_out = (int)constrain(u, -150.0f, 150.0f);
```

The gains would be set on the Python side, allowing for an iterative process, adjusting the values for each run. I also added a safety cutoff to stop the motors once the angle reading is 25 degrees more than the target angle.

As I ran the car, I realized the mass was not perfectly balanced. I decided to compensate for this by adding a calibration factor for when the motor is driven forwards or backwards. This was also initialized in **START_BALANCE**. 

```python
ble.send_command(CMD.START_BALANCE, "3.0|-0.5|1.75|1.2|1")
```

This set-up made the entire process a very iterative one where I began with the same gain values as mentioned in Stephen Wagner's write-up and tuned them to my system. 

## Results

Here are the final values I settled on:
Kp: 6.7
Kd: -0.015
Motor Calibration: 1
Forward Calibration: 1.25
Reverse Calibration: 1

<video src="/Fast-Robots/RUN1.mp4" controls width="600"></video>

<video src="/Fast-Robots/RUN2.mp4" controls width="600"></video>

<video src="/Fast-Robots/RUN3.mp4" controls width="600"></video>

Runs With Their Respective Data:

<video src="/Fast-Robots/RUN4DATA.mp4" controls width="600"></video>

<img src="/Fast-Robots/RUN4DATAA.png">

<video src="/Fast-Robots/RUN5_DATA.mp4" controls width="600"></video>

<img src="/Fast-Robots/RUN5DATAA.png">


## Refernce

I reference Stephen Wagner's write up for a base for the calculations and control gains. Claude aided me in working out the math to compute the gravity vector to have a consistent starting angle. 