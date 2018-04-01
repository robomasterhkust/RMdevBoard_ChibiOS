云台校准步骤：
1. before controller: 安装开发板前，用shell 输入 cal gyro, cal accel, 静置校准gyroscope，六面校准accelerometer;
    - 注意校准板载IMU时逻辑上需要先完成加热才能进入校准过程。
2. estimator: 检查坐标系，应该为ZYX Euler angle系统，x朝前，Y轴向右；
3. initialization feedback controller: 先调最简单的encoder的控制器，先调P，I再调D。encoder控制器会在温度达到的时候终结；
4. software limit: 加软件限位值，注意是大地坐标系；
5. attitude feedforward controller: 姿态前馈控制器，只调pitch；
    - 位置：pitch_w, 增加到重力与前馈项抵消时停止；
    - 重力方向：pitchX, pitchY, pitchZ, 找到枪口朝前时的重力方向；
6. attitude and velocity controller。