# Module
                                                                                  
All the math related parts, especially different type of controller and estimator.

## gimbal.c 
Controller. Written and maintained by Edward ZHANG, solider gimbal feedforward and feedback controller.

## attitude.c
Estimator. Written by Edward ZHANG, fused high percision gyro with onboard IMU

## chassis_velocity_generator.c
Trajectory generator. Written by Beck Pang, smooth out the velocity command without changing the controller.