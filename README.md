# selfbalancing_robot
Remote controlled self balancing robot. 

https://user-images.githubusercontent.com/49548379/127552782-20ae32ec-ad17-4d8e-8578-7c62c85af341.mp4



## The Robot

The robot consists of a fully 3D printed chassis, two Nema 17 Stepper Motors + motor drivers, a 6DOF-MPU6050 Inertial Measurement Unit and one transceiver module NRF24L01.  Using an Arduino Nano the IMU data is fused with the transmitted controls to drive the stepper motors accordingly. Furthermore a voltage divider is utilized to keep track of the LiPo voltage. The L7805 is used as a stepdown converter to power the Nano from the battery directly.

### Assembly
![Robot_Schematics](https://user-images.githubusercontent.com/49548379/127550855-91db80ca-45e1-4865-a9db-fde33bf21caf.png)





## The Remote Controller

The RC uses a Joystick or another IMU to generate the controls for the selfbalancing robot. The controls are read by another Nano and send wirelessly using the second transceiver NRF24L01 module. 

### Assembly
![Transmitter_Circuit_diagram_Breadboard](https://user-images.githubusercontent.com/49548379/127552187-b2c73d63-f425-48ed-bd1f-ad29a4e3b3fb.png)




## Setting up the software
### Calibrate the IMUs
Especially the IMU used on the robot itself has to be calibrated nicely. Otherwise the robot will probably fall over. To calibrate your IMU it is suggested to put the robot on the table and run the provided calibration script. Open the Serial monitor and just wait for it to finish. The final values will show up eventually. Just copy them into the main script and you are ready to go. 

## RC robot in action


https://user-images.githubusercontent.com/49548379/127555006-d38b4bb5-38b9-4d4e-9c73-ccdcb0052809.mp4

