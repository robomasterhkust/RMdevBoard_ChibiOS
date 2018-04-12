云台校准步骤：
0. before controller: 按照机械结构，调节gimbal.h里的gear_ratio。
1. before controller: 安装开发板前，用shell 输入 cal gyro, cal accel, 静置校准gyroscope，六面校准accelerometer;
    - 注意校准板载IMU时逻辑上需要先完成加热才能进入校准过程。
    - 高精ADIS16265 需要校准，cal adi-full，大约需要五分钟
    - 校准时不能comment out gimbal_init()，否则参数会被下次调参数时刷掉。
2. estimator: 检查坐标系，应该为ZYX Euler angle系统，X朝前，Y轴向左，Z朝上；
    - 先测IMU轴，看重力方向，z轴为负即正确；云台上播，x轴为负正确；向右侧翻云台，y轴为负正确。
3. position limit: 
    - 在 gimbal_attiCmd() function里查pitch_speed_limit，按pitch轴方向改加减号pitch speed limit;
    - 测方向结束，接下来是测数值。
4. initialization feedback controller: 
打开GIMBAL_INIT_TEST
    - 先调最简单的encoder的控制器，先调P，I再调D。encoder控制器会在温度达到的时候终结；
5. software limit: 加软件限位值，注意是大地坐标系；TODO: 180度会有break point. fixed at the quaternion calculation
6. attitude feedforward controller: 
关闭GIMBAL_INIT_TEST，打开GIMBAL_FF_TEST
姿态前馈控制器，只调pitch；
    - 位置：pitch_w, 增加到重力与前馈项抵消时停止；注意方向。
    - 重力方向：pitchX, pitchY, pitchZ, 找到枪口朝前时的重力方向，测完之后看加速度计；pitchY应为0。
    额命名命错了，应该填在YawX，YawY，YawZ。
7. velocity controller
关闭GIMBAL_FF_TEST
    - 拿手握着，先确定Kp方向，有阻尼感为正确。
8. attitude controller
    - 如果pitch轴高频震动，可能是由于pitch轴内环
   