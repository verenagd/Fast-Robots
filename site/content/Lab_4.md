+++
title = "Lab 4"
+++

## Lab 4

### Prelab

In this lab, we wire up and run tests on our motor drivers. The following diagram shows how I decided to wire all the devices to the Artemis, including the batteries for the motor drivers and the Artemis itself. 

<img src="/Fast-Robots/PINOUT.png">

------ CAR LABELED WITH PIECES --------

I made sure to choose pins that supported PWM and that made physical sense with regard to the motor drivers' location. The two inputs and outputs are parallel coupled for each motor driver, which is only possible because they are controlled by the same chip running on its own clock circuitry. <- CAN CHANGE

As for the wiring, I attempted to cut the wires to lengths that were as short as possible to eliminate EMI, however, long enough to have slack such that connections remain intact. I decided to follow common practice, making sure that wires that supply voltage are red, and all grounding wires are black. The remaining colors I chose personally, and tried to remain consistent. To further reduce EMI between wires and components, I decided to place the Artemis at the far back, away from the motor drivers' battery, twisted wries pairs, and made sure that there were no unintential wire loops. 

The Artemis and the motor drivers are powered by two separate batteries in order to eliminate any possible fluctuations in current supplied, given that the motor drivers pull a lot more current than the Artemis, and may do so sporadically. This can cause voltage spikes, dips, and noise. It is in our best interest to ensure that the Artemis receives stable power given that it controls the motor drivers, all sensors, and overall communication. 

### Task 1 - Soldering the Motor drivers

I decided to solder the motor drivers as follows, again parallel-coupling the two input and outputs, and grounding the signal to the Artemis.

Before connecting the motor driver to the car, it is important to run some tests, which I did by power the motor drivers to a power supply to be able to control current. The following are general specifications for the motor drivers we're using:

<img src="/Fast-Robots/POLULU_SPEC.png">

Since the battery that we'll be using provides 3.7V, I set the power supply to just that, and set a current limit of about 2.5A, given that I parallel coupled the two input and outputs.

### Task 3 - Generating PWM Signals

To test if the Artemis pins are generating PWM signals, and that they are succesfully being transmitted by the motor drivers, I hooked up the motor driver's output for each of its output channels to an oscilloscope and recorded it:

<iframe width="560" height="315" src="https://www.youtube.com/embed/d6bkE2Y94Qc?si=uNlrwgpivWTg02Bd" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

I used the following code to generate the PWM signal, and altered the first argument to select the pin, and the second argument to control the duty cycle:

```c++
    void loop(){
        analogWrite(A1_PIN, 128);  // 50% speed     
    }
```

I ran two different duty cycles (50% and 75%) to see the difference in the outputted signal:

<div style="display: flex; gap: 16px; flex-wrap: wrap;">

  <figure style="flex: 1; min-width: 180px; margin: 0;">
    <img src="/Fast-Robots/50DUTY.png" alt="A">
    <figcaption>50% Duty Cycle</figcaption>
  </figure>

  <figure style="flex: 1; min-width: 180px; margin: 0;">
    <img src="/Fast-Robots/75DUTY.png" alt="B">
    <figcaption>75% Duty Cycle</figcaption>
  </figure>

</div>


### Task 4 - Testing the Motors

After having verified the code and signal outputs, I connected each motor driver to the car's actual motors to see if I could turn the wheels forwards and backwards.

<iframe width="560" height="315" src="https://www.youtube.com/embed/jX6RYujQzYs?si=BHrVsRKFLzN_1wAA" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

<iframe width="560" height="315" src="https://www.youtube.com/embed/-zHhpDWJS7w?si=XEPEHqVj_Yt9Wz01" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Task 5 - Battery-Powered Tests

Now, I decided to run the same tests with the motor drivers being powered by a single 850 mAh battery.

Using this code:
```c+++
  Right Wheel  
  analogWrite(RMD_F, 128); // 50% speed

  //Left Wheel
  analogWrite(LMD_F, 128); // 50% speed
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/CZvlztkX64I?si=fMPLPHSBigTJfuHn" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


### Task 6 - Testing All Motor Drivers

The above tests were run for both motor drivers, ensuring that all connections are properly soldered and wired, and there are no software or hardware issues.


### Task 7 - Running the Car

I placed everything onto the car, and starting running it untethered.

<iframe width="560" height="315" src="https://www.youtube.com/embed/TRTr8imcpfw?si=4B35cXZb3MsG_zOj" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Task 8 - Test PWM Limits

In order to test the lower PWM limit for which the robot moves forward and on-axis turns while on the ground I decided to test the lowest value for which it drive forward and on-axis turn while starting from rest.

The lower values that the car starts from rest is with a PWM signal of ~25, anything smaller results in the following behavior where the motor cannot overcome the static friction:

<iframe width="560" height="315" src="https://www.youtube.com/embed/aC39HMUdOSA?si=NdtEi8UWbuPlttxQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

The car requires more power to perform an on-axis spin. I found that it required at least ~125 of a PWM signal, if not it will do the following:

<iframe width="560" height="315" src="https://www.youtube.com/embed/aC39HMUdOSA?si=NdtEi8UWbuPlttxQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Task 9 - Motor Calibration

To see if both of my motors spin at the same rate, I decided to see if my robot could follow a straight line for at least 2m. 

Originally it did not:

<iframe width="560" height="315" src="https://www.youtube.com/embed/TMDn0Iq354k?si=-47eSzHHL8qn6HD1" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

This led me to empirically find a ratio between what duty cycle I should run one pair of wheels with respect to the other. The wheels on the left required an 80% greater duty cycle than the right-side of wheels. I tested this ratio over a variety of duty cycles to make sure it works with various speeds. Below are two videos of me running the calibrated motors at a 25% and 50% duty cycle.

<iframe width="560" height="315" src="https://www.youtube.com/embed/mE19yUAumGo?si=hBrGNGCnL62NBj5w" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

<iframe width="560" height="315" src="https://www.youtube.com/embed/kdm-MDQFwFc?si=XOMcGMSzjix8eU7H" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Task 10 - Run!

Now, I decided to run my car untethered, to see how it performs doing basic turns and manuevers using the following code:

```c++
  startMillis = millis();
  analogWrite(RMD_F, 128);
  analogWrite(LMD_F, 160);
  delay(1000);
  analogWrite(RMD_F, LOW);
  analogWrite(LMD_F, LOW);
  analogWrite(LMD_R, 160);
  analogWrite(RMD_F, 128);
  delay(1000);
  analogWrite(RMD_F, HIGH);
  analogWrite(RMD_R, HIGH);
  analogWrite(LMD_F, HIGH);
  analogWrite(LMD_R, HIGH);
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/JNI7jOArJLk?si=u5TLUkC5C62oNDY_" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Additional Tasks - 5190

In order to measure the frequency that the analogWrite generates, I turn to the oscilloscope's output for the 50% duty cycle. The image shows that the signal has a period of about 5.464 ms, which equates to about 183 Hz. The frequency is the same for all duty cycles. 

<img src="/Fast-Robots/50DUTY.png">

This frequency seems to work well, however, any lower fre