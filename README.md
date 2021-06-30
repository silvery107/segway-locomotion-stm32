# Segway Locomotion
This project is the locomotion control for a Segway robot with STM32F1.

In this project, we implemented ***balance control***, ***speed control***, ***steering control*** method for the Segway robot together with the ***Bluetooth*** debugging function.

## Segway Robot

<img src=images/image2.png width=300>

The structure of the Segway is divided into three layers. The top layer is equipped with STM32F1, MPU9250, motor driver module, and Bluetooth module. The medium layers is equipped with battery, power distribution board and 12V to 5V step-down module. The bottom layer contains two motors  and encoders mounted on them.

## Control Diagram

<img src=images/image6.png width=600>

Let `u` represent the system input commands (target speed and target steering angle), `x_k` represent the system state of the balance car at time `k`, and enter the control loop when the balance car is initialized. 

Bluetooth interrupts the input command `u_BT` through the serial port, converts the input command into the target system state at time `k` in the main function, and calculates the current system state through the IMU and the left and right wheel motor encoder readings. 

The control system makes the difference between the target system state and the current system state, and obtains the state error as the input of the **PID** feedback controller, and uses the **PD**, **PI** and **P** controllers to calculate the PWM values required for balance, speed and steering control. After the amplitude, it is input to the motor as the control quantity. 

After entering `k+1` time, the system state `x_k=x_(k+1)` is updated according to the sensor reading.