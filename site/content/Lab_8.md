+++
title = "Lab 8"
+++

## Prelab

The goal for this lab is to be able to run the car towards the wall, perform a flip, and drive backwards. 

## Kalman Filter Updating

In order to do this I decided to use the same Kalman Filter that I had developed in Lab 7. However, I modified it this lab to take a $dt$ argument computed from the running loop instead of using the hardcoded value of .103s used in Lab 7. I included this, as I wanted the Kalman Filter to estimate the position even when the ToF sensor had no new data, which meant that it had to update its estimates at a faster rate than the sensor itself. By passing in the loop-based $dt$, the filter is able to continuously perform prediction steps using the system model between measurements, and incorporate new ToF sensor readings when they are available. I created a flag for when the ToF had new data available called has_meas.

```c++
float kf_update(float u_scaled, float y_mm, bool has_meas, float dt){
    if (dt < 0.0005f) dt = 0.0005f;
    if (dt > 0.5f)   dt = 0.5f;
    ...
    if (!has_meas) {
        kf_mu0 = mp0;  
        kf_mu1 = mp1;
        ...
    }
}
```

## Flip Function

In order to run the function that allowed the car to perform a flip I created the following case that initiated the necessary variables and flags:

```c++
 case FLIP_RUN:{
    success = robot_cmd.get_next_value(pwm);
    if (!success)
    return;  
    success = robot_cmd.get_next_value(cal);
    if (!success)
    return;
    cal = cal;        
    last_pi_ms = millis();
    loop_samplecount = 0;
    samplecount = 0;
    last_kf_us = 0;
    tof_front.startRanging();
    while(!tof_front.checkForDataReady()) delay(1);
    last_tof_reading = tof_front.getDistance();  

    kf_mu0 = last_tof_reading;
    kf_mu1 = 0.0f;
    kf_P00 = SIG1_SQ;
    kf_P01 = 0.0f;
    kf_P11 = SIG2_SQ;
    
    flip_run = true;
    break;
}
```

The actual function that is run to perform this task is similar to that of Lab 7, however, I removed the PID control and instead made the car perform the flip when it was within a specific range. Since I decided to run the car at 225 PWM, the measured distance that the car should be from the wall in order to start the flip was empirically determined to be ~1200. This is due to the latency of the Artemis receiving the estimated (or measured from the ToF sensor) position, the loop execution time, and the motor response time compared to the speed at which the car is moving. 


The following are lines of code that differentiate the function from Lab 7 (once having removed the PID control):

```c++
if (last_kf_us == 0) {
    dt = 0.001f;   
} else {
    dt = (now_us - last_kf_us) * 1e-6f;
}
last_kf_us = now_us;

if (kf_estimated_pos <= 1200) {       
    reverse(255);
    delay(1000);
    hardStop();
    tx_characteristic_string.writeValue("Done :)");
    tof_front.stopRanging();
    flip_run = false;        
} 
```

Here is the run!

<video src="/Fast-Robots/FLIP.mp4" controls width="600"></video>

## Plots, Data & Conclusion

For the first runs, the car would not brake in time, and only stop after it had crashed into the wall. I decided to output both the estimated and measured distances to help troubleshoot, and originally the majority of the data looked like this:

Time (ms): 47142, Front (mm): 3043, Motor PWM: 150, KF Est (mm): 3043
Time (ms): 47246, Front (mm): 3032, Motor PWM: 150, KF Est (mm): 3032
Time (ms): 47346, Front (mm): 3025, Motor PWM: 150, KF Est (mm): 3025
Time (ms): 47450, Front (mm): 2983, Motor PWM: 150, KF Est (mm): 2983
Time (ms): 47556, Front (mm): 2902, Motor PWM: 150, KF Est (mm): 2904
Time (ms): 47654, Front (mm): 2795, Motor PWM: 150, KF Est (mm): 2797
Time (ms): 47761, Front (mm): 2668, Motor PWM: 150, KF Est (mm): 2671
Time (ms): 47865, Front (mm): 2510, Motor PWM: 150, KF Est (mm): 2513
Time (ms): 47965, Front (mm): 2331, Motor PWM: 150, KF Est (mm): 2337
Time (ms): 48072, Front (mm): 2135, Motor PWM: 150, KF Est (mm): 2139
Time (ms): 48171, Front (mm): 1932, Motor PWM: 150, KF Est (mm): 1937
Time (ms): 48275, Front (mm): 1703, Motor PWM: 150, KF Est (mm): 1708
Time (ms): 48376, Front (mm): 1459, Motor PWM: 150, KF Est (mm): 1465
Time (ms): 48479, Front (mm): 1213, Motor PWM: 150, KF Est (mm): 1221
Time (ms): 48581, Front (mm): 950, Motor PWM: 150, KF Est (mm): 957
Time (ms): 48692, Front (mm): 679, Motor PWM: 150, KF Est (mm): 687
Time (ms): 48797, Front (mm): 384, Motor PWM: 150, KF Est (mm): 395
Time (ms): 48899, Front (mm): 73, Motor PWM: 150, KF Est (mm): 81

<img src="/Fast-Robots/FLIP_ONE.png">

As you can see the motor did not stop when the car was well within 1 ft of the wall.

After having updated the distance to be about 1200mm, the following measurements are from the successful run shown in the video:

<img src="/Fast-Robots/FLIP_TWO.png">

The flip is triggered when the estimated distance reaches ~1200mm, which gives the system enough time to decelerate, and allowing the car to stop approximately 1 ft away from the wall.
