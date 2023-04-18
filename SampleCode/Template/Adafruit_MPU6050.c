/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "NuMicro.h"

#include "Adafruit_MPU6050.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

/**
 * @brief Accelerometer range options
 *
 * Allowed values for `setAccelerometerRange`.
 */
typedef enum {
  MPU6050_RANGE_2_G,  ///< +/- 2g (default value)
  MPU6050_RANGE_4_G,  ///< +/- 4g
  MPU6050_RANGE_8_G,  ///< +/- 8g
  MPU6050_RANGE_16_G, ///< +/- 16g
} mpu6050_accel_range_t;

/**
 * @brief Gyroscope range options
 *
 * Allowed values for `setGyroRange`.
 */
typedef enum {
  MPU6050_RANGE_250_DEG,  ///< +/- 250 deg/s (default value)
  MPU6050_RANGE_500_DEG,  ///< +/- 500 deg/s
  MPU6050_RANGE_1000_DEG, ///< +/- 1000 deg/s
  MPU6050_RANGE_2000_DEG, ///< +/- 2000 deg/s
} mpu6050_gyro_range_t;

/**
 * @brief Digital low pass filter bandthwidth options
 *
 * Allowed values for `setFilterBandwidth`.
 */
typedef enum {
  MPU6050_BAND_260_HZ, ///< Docs imply this disables the filter
  MPU6050_BAND_184_HZ, ///< 184 Hz
  MPU6050_BAND_94_HZ,  ///< 94 Hz
  MPU6050_BAND_44_HZ,  ///< 44 Hz
  MPU6050_BAND_21_HZ,  ///< 21 Hz
  MPU6050_BAND_10_HZ,  ///< 10 Hz
  MPU6050_BAND_5_HZ,   ///< 5 Hz
} mpu6050_bandwidth_t;

/**
 * @brief Accelerometer high pass filter options
 *
 * Allowed values for `setHighPassFilter`.
 */
typedef enum {
  MPU6050_HIGHPASS_DISABLE,
  MPU6050_HIGHPASS_5_HZ,
  MPU6050_HIGHPASS_2_5_HZ,
  MPU6050_HIGHPASS_1_25_HZ,
  MPU6050_HIGHPASS_0_63_HZ,
  MPU6050_HIGHPASS_UNUSED,
  MPU6050_HIGHPASS_HOLD,
} mpu6050_highpass_t;

/**
 * @brief Periodic measurement options
 *
 * Allowed values for `setCycleRate`.
 */
typedef enum {
  MPU6050_CYCLE_1_25_HZ, ///< 1.25 Hz
  MPU6050_CYCLE_5_HZ,    ///< 5 Hz
  MPU6050_CYCLE_20_HZ,   ///< 20 Hz
  MPU6050_CYCLE_40_HZ,   ///< 40 Hz
} mpu6050_cycle_rate_t;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
extern void delay_ms(uint32_t ms);
extern void I2C0_Init(void);


uint8_t MPU6050_getMotionInterruptStatus(void)
{
    uint8_t tmp = 0;
    uint8_t res = 0; 

    res = i2c_reg_read(MPU6050_I2CADDR_DEFAULT,MPU6050_INT_STATUS, &tmp , 1);  
    tmp = tmp >> 6 ; 
    (void)(res);

    return tmp;
}

uint8_t MPU6050_setMotionInterrupt(uint8_t active)
{
    uint8_t tmp = 0;
    uint8_t res = 0;    

    tmp = active << 6;
    res = i2c_reg_write(MPU6050_I2CADDR_DEFAULT,MPU6050_INT_ENABLE, &tmp, 1);    

    return res;
}

uint8_t MPU6050_setInterruptPinPolarity(uint8_t active_low)
{
    uint8_t tmp = 0;
    uint8_t res = 0;    

    tmp = active_low << 7;
    res = i2c_reg_write(MPU6050_I2CADDR_DEFAULT,MPU6050_INT_PIN_CONFIG, &tmp, 1);    

    return res;
}

uint8_t MPU6050_setInterruptPinLatch(uint8_t held)
{
    uint8_t tmp = 0;
    uint8_t res = 0;    

    tmp = held << 5;
    res = i2c_reg_write(MPU6050_I2CADDR_DEFAULT,MPU6050_INT_PIN_CONFIG, &tmp, 1);    

    return res;
}

uint8_t MPU6050_setMotionDetectionDuration(uint8_t dhr)
{
    uint8_t tmp = dhr;
    uint8_t res = 0;    

    res = i2c_reg_write(MPU6050_I2CADDR_DEFAULT,MPU6050_MOT_DUR, &tmp, 1);    

    return res;
}

uint8_t MPU6050_setMotionDetectionThreshold(uint8_t thr)
{
    uint8_t tmp = thr;
    uint8_t res = 0;    

    res = i2c_reg_write(MPU6050_I2CADDR_DEFAULT,MPU6050_MOT_THR, &tmp, 1);    

    return res;
}

/*
    XA_ST_BIT           7
    YA_ST_BIT           6
    ZA_ST_BIT           5
    AFS_SEL_BIT         4
    ACCEL_HPF_BIT       2
*/
uint8_t MPU6050_setHighPassFilter(uint8_t bandwidth)
{
    uint8_t tmp = bandwidth;
    uint8_t res = 0;    

    // tmp = bandwidth << 2;
    res = i2c_reg_write(MPU6050_I2CADDR_DEFAULT,MPU6050_ACCEL_CONFIG, &tmp, 1);    

    return res;
}

uint8_t MPU6050_setFilterBandwidth(uint8_t bandwidth)
{
    uint8_t tmp = bandwidth;
    uint8_t res = 0;

    res = i2c_reg_write(MPU6050_I2CADDR_DEFAULT,MPU6050_CONFIG, &tmp, 1);

    return res;
}

uint8_t MPU6050_getFilterBandwidth(uint8_t *bandwidth)
{
    uint8_t res = 0;
    res = i2c_reg_read(MPU6050_I2CADDR_DEFAULT,MPU6050_CONFIG, bandwidth , 1);  

    return res;
}

uint8_t MPU6050_setSampleRateDivisor(uint8_t divisor)
{
    uint8_t tmp = divisor;
    uint8_t res = 0;

    res = i2c_reg_write(MPU6050_I2CADDR_DEFAULT,MPU6050_SMPRT_DIV, &tmp, 1);

    return res;
}

uint8_t MPU6050_setGyroRange(uint8_t new_range)
{
    uint8_t tmp = 0;
    uint8_t res = 0;   

    tmp = new_range << 3;
    res = i2c_reg_write(MPU6050_I2CADDR_DEFAULT,MPU6050_GYRO_CONFIG, &tmp, 1);

    return res;
}

uint8_t MPU6050_getGyroRange(uint8_t *new_range)
{

    uint8_t res = 0;
    uint8_t tmp = 0;

    res = i2c_reg_read(MPU6050_I2CADDR_DEFAULT,MPU6050_GYRO_CONFIG, &tmp, 1);
    *new_range = tmp >> 3; 

    return res;
}

uint8_t MPU6050_setAccRange(uint8_t new_range)
{
    uint8_t tmp = 0;
    uint8_t res = 0;    

    tmp = new_range << 3;
    res = i2c_reg_write(MPU6050_I2CADDR_DEFAULT,MPU6050_ACCEL_CONFIG, &tmp, 1);

    return res;
}

uint8_t MPU6050_getAccRange(uint8_t *new_range)
{
    uint8_t res = 0;
    uint8_t tmp = 0;

    res = i2c_reg_read(MPU6050_I2CADDR_DEFAULT,MPU6050_ACCEL_CONFIG, &tmp, 1);
    *new_range = tmp >> 3;

    return res;
}

uint8_t MPU6050_Adafruit_init(void)
{
    uint8_t tmp = 0;
    uint8_t res = 0;

    uint8_t data[2] = {0};
    uint16_t timeout = 0;
    uint8_t flag = 0;

    // who am I
    res = i2c_reg_read(MPU6050_I2CADDR_DEFAULT,MPU6050_WHO_AM_I, &tmp , 1);
    if (res != 0)
    {
        printf("%s : run MPU6050_WHO_AM_I test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
        return 1;
    }    
    if (tmp != MPU6050_DEVICE_ID)
    {
        printf("%s : run MPU6050_WHO_AM_I test FAIL (0x%2X)\r\n", __FUNCTION__ , tmp);
        return 1;
    }

    // reset
    data[0] = 0x00 | BIT7 ;                                                                       
    res = i2c_reg_write(MPU6050_I2CADDR_DEFAULT,MPU6050_PWR_MGMT_1, data, 1);    
    if (res != 0)
    {
        printf("%s : run MPU6050_PWR_MGMT_1 test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
        return 1;
    }

    timeout = 100;
    while(timeout != 0x00)
    {
        res = i2c_reg_read(MPU6050_I2CADDR_DEFAULT,MPU6050_PWR_MGMT_1, &tmp , 1);
        if (res != 0)
        {
            printf("%s : run MPU6050_PWR_MGMT_1(R) test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
            flag = 1;
            break;
        }     

        if (( tmp >> 7) == 0)                                                 
        {   
            printf("%s : run MPU6050_PWR_MGMT_1 init ready\r\n", __FUNCTION__ );                      
            break;                                                            
        }           

        delay_ms(10);
        timeout--;

        if (timeout == 0x00)
        {
            flag = 1;
        }
    }
    if (flag)
    {
        printf("%s : run MPU6050_PWR_MGMT_1 init test FAIL \r\n", __FUNCTION__ );  
        return 1;      
    }

    // bit 2 1 0 
    data[0] = BIT2 | BIT1 | BIT0;                                                                     
    res = i2c_reg_write(MPU6050_I2CADDR_DEFAULT,MPU6050_SIGNAL_PATH_RESET, data, 1);    
    if (res != 0)
    {
        printf("%s : run MPU6050_SIGNAL_PATH_RESET test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
        return 1;
    }

    // set sample rate divide
    // bit 0
    res = MPU6050_setSampleRateDivisor(0);  
    if (res != 0)
    {
        printf("%s : run setSampleRateDivisor test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
        return 1;
    }

    // set filter band width
    // bit 0
    res = MPU6050_setFilterBandwidth(MPU6050_BAND_260_HZ);    
    if (res != 0)
    {
        printf("%s : run setFilterBandwidth test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
        return 1;
    }

    // set gyro range
    // bit 3
    res = MPU6050_setGyroRange(MPU6050_RANGE_500_DEG);    
    if (res != 0)
    {
        printf("%s : run setGyroRange test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
        return 1;
    }

    // set acc range
    // bit 3
    res = MPU6050_setAccRange(MPU6050_RANGE_2_G);    
    if (res != 0)
    {
        printf("%s : run setAccRange test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
        return 1;
    }

    // set clock to PLL , with Gyro X reference
    // bit 0
    data[0] = 0x01;                                                                      
    res = i2c_reg_write(MPU6050_I2CADDR_DEFAULT,MPU6050_PWR_MGMT_1, data, 1);    
    if (res != 0)
    {
        printf("%s : run MPU6050_PWR_MGMT_1(W) test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
        return 1;
    }

    delay_ms(100);

    printf("%s : ready\r\n", __FUNCTION__);

    return 0;
}

uint8_t MPU6050_Adafruit_read(float* g,float* dps,float* degrees)
{
    uint8_t buffer[14];

    uint8_t tmp = 0;
    uint8_t res = 0;
    int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
    float temperature, ///< Last reading's temperature (C)
        accX,          ///< Last reading's accelerometer X axis m/s^2
        accY,          ///< Last reading's accelerometer Y axis m/s^2
        accZ,          ///< Last reading's accelerometer Z axis m/s^2
        gyroX,         ///< Last reading's gyro X axis in rad/s
        gyroY,         ///< Last reading's gyro Y axis in rad/s
        gyroZ;         ///< Last reading's gyro Z axis in rad/s

    float accel_scale = 1;
    float gyro_scale = 1;

    res = i2c_reg_read(MPU6050_I2CADDR_DEFAULT,MPU6050_ACCEL_XOUT_H, &buffer[0] , 14);
    if (res != 0)
    {
        printf("%s : run MPU6050_ACCEL_XOUT_H test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
        return 1;
    } 

    rawAccX = buffer[0] << 8 | buffer[1];
    rawAccY = buffer[2] << 8 | buffer[3];
    rawAccZ = buffer[4] << 8 | buffer[5];

    rawTemp = buffer[6] << 8 | buffer[7];

    rawGyroX = buffer[8] << 8 | buffer[9];
    rawGyroY = buffer[10] << 8 | buffer[11];
    rawGyroZ = buffer[12] << 8 | buffer[13];

    temperature = (rawTemp / 340.0) + 36.53;
    *degrees = temperature;

    res = MPU6050_getAccRange(&tmp);
    switch(tmp)
    {
        case MPU6050_RANGE_2_G: 
            accel_scale = 16384;
            break;
        case MPU6050_RANGE_4_G: 
            accel_scale = 8192;
            break;
        case MPU6050_RANGE_8_G: 
            accel_scale = 4096;
            break;
        case MPU6050_RANGE_16_G: 
            accel_scale = 2048;
            break;
    }

    // setup range dependant scaling
    accX = ((float)rawAccX) / accel_scale;
    accY = ((float)rawAccY) / accel_scale;
    accZ = ((float)rawAccZ) / accel_scale;

    g[0] = accX * SENSORS_GRAVITY_STANDARD;
    g[1] = accY * SENSORS_GRAVITY_STANDARD;
    g[2] = accZ * SENSORS_GRAVITY_STANDARD;

    res = MPU6050_getGyroRange(&tmp);
    switch(tmp)
    {
        case MPU6050_RANGE_250_DEG: 
            gyro_scale = 131;
            break;
        case MPU6050_RANGE_500_DEG: 
            gyro_scale = 65.5;
            break;
        case MPU6050_RANGE_1000_DEG: 
            gyro_scale = 32.8;
            break;
        case MPU6050_RANGE_2000_DEG: 
            gyro_scale = 16.4;
            break;
    }

    gyroX = ((float)rawGyroX) / gyro_scale;
    gyroY = ((float)rawGyroY) / gyro_scale;
    gyroZ = ((float)rawGyroZ) / gyro_scale;

    dps[0] = gyroX * SENSORS_DPS_TO_RADS;
    dps[1] = gyroY * SENSORS_DPS_TO_RADS;
    dps[2] = gyroZ * SENSORS_DPS_TO_RADS;

    return 0;
}

uint8_t MPU6050_Adafruit_basic_readings(void)
{
    uint8_t tmp = 0;
    uint8_t res = 0;

    static uint8_t init_once = 1;
    static uint8_t init_ready = 1;
    float a[3];
    float g[3];
    float temp;

    if (init_once)
    {
        init_once = 0;
        I2C0_Init();

        init_ready = MPU6050_Adafruit_init();

        // set acc range : 8G
        // bit 3    
        res = MPU6050_setAccRange(MPU6050_RANGE_8_G);    
        if (res != 0)
        {
            printf("%s : run setAccRange test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
            return 1;
        }
        res = MPU6050_getAccRange(&tmp);  
        if (res != 0)
        {
            printf("%s : run getAccRange(R) test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
            return 1;
        }
        switch(tmp)
        {
            case MPU6050_RANGE_2_G: 
                printf("MPU6050_RANGE_2_G\r\n");
                break;
            case MPU6050_RANGE_4_G: 
                printf("MPU6050_RANGE_4_G\r\n");
                break;
            case MPU6050_RANGE_8_G: 
                printf("MPU6050_RANGE_8_G\r\n");
                break;
            case MPU6050_RANGE_16_G: 
                printf("MPU6050_RANGE_16_G\r\n");
                break;
        }

        // set gyro range : 500
        // bit 3
        res = MPU6050_setGyroRange(MPU6050_RANGE_500_DEG);
        if (res != 0)
        {
            printf("%s : run setGyroRange test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
            return 1;
        }
        res = MPU6050_getGyroRange(&tmp);
        if (res != 0)
        {
            printf("%s : run getGyroRange(R) test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
            return 1;
        }
        switch(tmp)
        {
            case MPU6050_RANGE_250_DEG: 
                printf("MPU6050_RANGE_250_DEG\r\n");
                break;
            case MPU6050_RANGE_500_DEG: 
                printf("MPU6050_RANGE_500_DEG\r\n");
                break;
            case MPU6050_RANGE_1000_DEG: 
                printf("MPU6050_RANGE_1000_DEG\r\n");
                break;
            case MPU6050_RANGE_2000_DEG: 
                printf("MPU6050_RANGE_2000_DEG\r\n");
                break;
        }

        // set filter band width
        // bit 0
        res = MPU6050_setFilterBandwidth(MPU6050_BAND_21_HZ);
        if (res != 0)
        {
            printf("%s : run setFilterBandwidth test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
            return 1;
        }
        res = MPU6050_getFilterBandwidth(&tmp);
        if (res != 0)
        {
            printf("%s : run setFilterBandwidth(R) test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
            return 1;
        }
        switch(tmp)
        {
            case MPU6050_BAND_260_HZ: 
                printf("MPU6050_BAND_260_HZ\r\n");
                break;
            case MPU6050_BAND_184_HZ: 
                printf("MPU6050_BAND_184_HZ\r\n");
                break;
            case MPU6050_BAND_94_HZ: 
                printf("MPU6050_BAND_94_HZ\r\n");
                break;
            case MPU6050_BAND_44_HZ: 
                printf("MPU6050_BAND_44_HZ\r\n");
                break;
            case MPU6050_BAND_21_HZ: 
                printf("MPU6050_BAND_21_HZ\r\n");
                break;
            case MPU6050_BAND_10_HZ: 
                printf("MPU6050_BAND_10_HZ\r\n");
                break;
            case MPU6050_BAND_5_HZ: 
                printf("MPU6050_BAND_5_HZ\r\n");
                break;
        }

        delay_ms(100);

    }

    if (init_ready == 0)
    {
        MPU6050_Adafruit_read(a,g,&temp);

        printf("acc:%010.2f ,%010.2f ,%010.2f (m/s^2), ", a[0],a[1],a[2]);
        printf("gyro:%010.2f,%010.2f,%010.2f (rad/s), ", g[0],g[1],g[2]);
        printf("temp:%010.2f degC\r\n" , temp);

        delay_ms(100);
        return 0;
    }
        
    printf("NOT INIT\r\n");
    return 1;   // bad status
}

uint8_t MPU6050_Adafruit_motion_detection(void)
{
    // uint8_t tmp = 0;
    uint8_t res = 0;

    static uint8_t init_once = 1;
    static uint8_t init_ready = 1;
    float a[3];
    float g[3];
    float temp;

    if (init_once)
    {
        init_once = 0;
        I2C0_Init();

        init_ready = MPU6050_Adafruit_init();

        //setupt motion detection
        res = MPU6050_setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);    
        if (res != 0)
        {
            printf("%s : run setHighPassFilter test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
            return 1;
        }

        res = MPU6050_setMotionDetectionThreshold(1);    
        if (res != 0)
        {
            printf("%s : run setMotionDetectionThreshold test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
            return 1;
        }

        res = MPU6050_setMotionDetectionDuration(20);    
        if (res != 0)
        {
            printf("%s : run setMotionDetectionDuration test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
            return 1;
        }

        res = MPU6050_setInterruptPinLatch(1);    // 1: true
        if (res != 0)
        {
            printf("%s : run setInterruptPinLatch test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
            return 1;
        }

        res = MPU6050_setInterruptPinPolarity(1);    // 1: true
        if (res != 0)
        {
            printf("%s : run setInterruptPinPolarity test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
            return 1;
        }

        res = MPU6050_setMotionInterrupt(1);    // 1: true
        if (res != 0)
        {
            printf("%s : run setMotionInterrupt test FAIL (0x%2X)\r\n", __FUNCTION__ , res);
            return 1;
        }

        delay_ms(100);

    }

    if (init_ready == 0)
    {
        if (MPU6050_getMotionInterruptStatus())
        {
            MPU6050_Adafruit_read(a,g,&temp);

            printf("Motion,");
            printf("acc:%010.2f ,%010.2f ,%010.2f (m/s^2), ", a[0],a[1],a[2]);
            printf("gyro:%010.2f,%010.2f,%010.2f (rad/s), ", g[0],g[1],g[2]);
            // printf("temp:%010.2f degC\r\n" , temp);
            printf("\r\n");

            delay_ms(100);
            return 0;
        }
    }
        
    printf("NOT INIT\r\n");
    return 1;   // bad status
}

