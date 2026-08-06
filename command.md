~/.platformio/penv/bin/platformio device monitor \
  -p /dev/ttyUSB0 -b 115200

x向上
=== MPU6050 1000-sample calibration result ===
Average accel raw: X=16859.85 Y=537.45 Z=1515.62
Average accel [g]: X=+1.029044 Y=+0.032803 Z=+0.092506
Accel bias raw (level, Z vertical): X=16859.85 Y=537.45 Z=-14868.38
Gyro bias raw: X=-92.05 Y=184.26 Z=26.65
Gyro bias [deg/s]: X=-0.702672 Y=+1.406588 Z=+0.203443
Average temperature: 29.21 C | failed reads: 0

-x向上
=== MPU6050 1000-sample calibration result ===
Average accel raw: X=-15864.38 Y=31.55 Z=1180.55
Average accel [g]: X=-0.968285 Y=+0.001926 Z=+0.072055
Accel bias raw (level, Z vertical): X=-15864.38 Y=31.55 Z=-15203.45
Gyro bias raw: X=-93.97 Y=186.50 Z=16.46
Gyro bias [deg/s]: X=-0.717321 Y=+1.423679 Z=+0.125626
Average temperature: 29.55 C | failed reads: 0

y向上
=== MPU6050 1000-sample calibration result ===
Average accel raw: X=42.99 Y=16459.97 Z=-1436.36
Average accel [g]: X=+0.002624 Y=+1.004637 Z=-0.087668
Accel bias raw (level, Z vertical): X=42.99 Y=16459.97 Z=14947.64
Gyro bias raw: X=-69.34 Y=175.46 Z=-14.61
Gyro bias [deg/s]: X=-0.529336 Y=+1.339405 Z=-0.111511
Average temperature: 30.80 C | failed reads: 0

y向下
=== MPU6050 1000-sample calibration result ===
Average accel raw: X=759.72 Y=-16305.59 Z=1051.03
Average accel [g]: X=+0.046369 Y=-0.995214 Z=+0.064150
Accel bias raw (level, Z vertical): X=759.72 Y=-16305.59 Z=-15332.97
Gyro bias raw: X=-105.44 Y=192.14 Z=58.24
Gyro bias [deg/s]: X=-0.804855 Y=+1.466733 Z=+0.444618
Average temperature: 31.91 C | failed reads: 0

z向上
=== MPU6050 1000-sample calibration result ===
Average accel raw: X=327.95 Y=190.55 Z=17546.20
Average accel [g]: X=+0.020016 Y=+0.011630 Z=+1.070935
Accel bias raw (level, Z vertical): X=327.95 Y=190.55 Z=1162.20
Gyro bias raw: X=-115.18 Y=201.35 Z=28.35
Gyro bias [deg/s]: X=-0.879237 Y=+1.537053 Z=+0.216412
Average temperature: 32.41 C | failed reads: 0

z向下
=== MPU6050 1000-sample calibration result ===
Average accel raw: X=370.21 Y=107.11 Z=-15919.67
Average accel [g]: X=+0.022596 Y=+0.006537 Z=-0.971660
Accel bias raw (level, Z vertical): X=370.21 Y=107.11 Z=464.33
Gyro bias raw: X=-108.89 Y=174.04 Z=-95.13
Gyro bias [deg/s]: X=-0.831221 Y=+1.328557 Z=-0.726214
Average temperature: 31.92 C | failed reads: 0


{-97.478f, 185.625f, 3.327f}

// deg/s
{-0.744109f, 1.416985f, 0.025394f}

// rad/s
Eigen::Vector3f gyro_bias{
  -0.0129872f,
   0.0247310f,
   0.0004432f
};

io       MPU、左右轮通信是否成功
acc      三轴加速度和模长
gyro     三轴角速度
wheel    左右轮速度和电流
ekf      theta、theta_dot、速度、偏航角速度
target   目标状态
torque   左右轮计算扭矩及SAT饱和标志
mode     STOP、COMPUTE_ONLY或RUN

mainbody 301.66g
calf link 53.55g
active 12.24g
passive 13.5g
wheel 37.77g
tire 87.2g
wheel all 124.97g



