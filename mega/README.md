# Arduino Mega sketches for Herbie

Mega_count - Use this sketch if you are driving Herbie with RC control and using Herbie v.2. 

drive_cmd_vel - Use this sketch if you want to drive Herbie with ROS, sending cmd_vel messages wirelessly to the Raspi and then to Mega over the USB cable. In this case, the sketch is set up to use the Sabertooth motor driver from Dimension Engineering and its "simplified serial" mode of commanding the motors. You'll see that code is also included for integrating the MPU6050 IMU board. If you don't have one, just comment out those lines in the sketch and it will work fine. The connections to Mega pins are explained in the code. Set Sabertooth's DIM switches thus: 1 ON OFF OFF OFF ON ON 6  A Bluetooth controller could also be used to connect to Raspi and do this type of control.

The Teensy4.0 folder was added since this little board performs better than Mega when loading up with tasks. The Mega couldn't publish the IMU and encoder messages quick enough (to satify discerning users) when it had to also handle cmd_vel. Teensy does it at 35/second with a substantial delay built into main loop! However, if you are driving your robot with RC control, the Mega is entirely sufficient. (I used joy and teleop_twist_joy on the laptop, with a large Logitech 3D controller. The X-box controller also was tested and worked well.)

Good luck!


