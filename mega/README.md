# Arduino Mega sketches for Herbie

Mega_count - Use this sketch if you are driving Herbie with RC control. 

drive_cmd_vel - Use this sketch if you want to drive Herbie with ROS, sending cmd_vel messages wirelessly to the Raspi and then to Mega over the USB cable. In this case, the sketch is set up to use the Sabertooth motor driver from Dimension Engineering and its "simplified serial" mode of commanding the motors. You'll see that code is also included for integrating the MPU6050 IMU board. If you don't have one, just comment out those lines in the sketch and it will work fine. The connections to Mega pins are explained in the code.

Good luck!


