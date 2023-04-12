# M031BSP_I2C_MPU6050
 M031BSP_I2C_MPU6050

update @ 2023/04/12

1. use I2C0 initial MPU6050 (PC1 : SCL , PC0 : SDA) , with I2C interrupt flow (MPU6050 is 8bit REGISTER)

2. base on sample code behavior

https://github.com/hepingood/mpu6050

3. enable MPU6050_test , 

with define : REGISTER_TEST

![image](https://github.com/released/M031BSP_I2C_MPU6050/blob/main/log_REGISTER_TEST.jpg)	

with define : READ_TEST

![image](https://github.com/released/M031BSP_I2C_MPU6050/blob/main/log_READ_TEST.jpg)	

with define : FIFO_TEST

![image](https://github.com/released/M031BSP_I2C_MPU6050/blob/main/log_FIFO_TEST.jpg)	

with define : READ_EXAMPLE

![image](https://github.com/released/M031BSP_I2C_MPU6050/blob/main/log_READ_EXAMPLE.jpg)	

with define : FIFO_EXAMPLE

![image](https://github.com/released/M031BSP_I2C_MPU6050/blob/main/log_FIFO_EXAMPLE.jpg)	

![image](https://github.com/released/M031BSP_I2C_MPU6050/blob/main/log_REGISTER_TEST.jpg)	

![image](https://github.com/released/M031BSP_I2C_MPU6050/blob/main/log_REGISTER_TEST.jpg)	


4. enable MPU6050_simple_read , 

![image](https://github.com/released/M031BSP_I2C_MPU6050/blob/main/log_MPU6050_simple_read.jpg)	

5. below is LA when use MPU6050_simple_read

![image](https://github.com/released/M031BSP_I2C_MPU6050/blob/main/LA_1_read_config.jpg)	

![image](https://github.com/released/M031BSP_I2C_MPU6050/blob/main/LA_2_read_acc_gyro.jpg)	

![image](https://github.com/released/M031BSP_I2C_MPU6050/blob/main/LA_3_read_temp.jpg)	

