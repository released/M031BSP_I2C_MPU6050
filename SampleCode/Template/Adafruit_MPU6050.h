/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "NuMicro.h"
#include <stdlib.h>

#include "i2c_driver.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/


#define MPU6050_I2CADDR_DEFAULT  	(0x68)				/**< AD0 pin set LOW , 7 bit : 0x68 , 8 bit : 0xD0*/
// #define MPU6050_I2CADDR_DEFAULT  	(0x60)				/**< AD0 pin set HIGH , 7 bit : 0x69 , 8 bit : 0xD2*/

#define MPU6050_PWR_MGMT_1			(0x6B)            	/**< power management 1 register */   
#define MPU6050_SIGNAL_PATH_RESET	(0x68)     			/**< signal path reset register */  

#define MPU6050_WHO_AM_I			(0x75)              /**< who am I register */
#define MPU6050_DEVICE_ID			(0x68)        
#define MPU6050_SMPRT_DIV			(0x19)             	/**< smprt div register */
#define MPU6050_CONFIG				(0x1A)              /**< configure register */
#define MPU6050_GYRO_CONFIG			(0x1B)           	/**< gyro configure register */
#define MPU6050_ACCEL_CONFIG		(0x1C)          	/**< accel configure register */

#define MPU6050_MOT_THR             (0x1F)              ///< Motion detection threshold bits [7:0]
#define MPU6050_MOT_DUR             (0x20)              ///< Duration counter threshold for motion int. 1 kHz rate, LSB = 1 ms
#define MPU6050_INT_PIN_CONFIG      (0x37)              ///< Interrupt pin configuration register
#define MPU6050_INT_ENABLE          (0x38)              ///< Interrupt enable configuration register
#define MPU6050_INT_STATUS          (0x3A)              ///< Interrupt status register

#define MPU6050_ACCEL_XOUT_H		(0x3B)          	/**< accel xout high register */ 


#define SENSORS_GRAVITY_EARTH (9.80665F)            /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON (1.6F)                 /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN (275.0F)                /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD (SENSORS_GRAVITY_EARTH)
#define SENSORS_DPS_TO_RADS (0.017453293F)          /**< Degrees/s to rad/s multiplier */

/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

uint8_t MPU6050_Adafruit_basic_readings(void);
uint8_t MPU6050_Adafruit_motion_detection(void);
