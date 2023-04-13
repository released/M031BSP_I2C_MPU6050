/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "NuMicro.h"

#include "driver_mpu6050_register_test.h"
#include "driver_mpu6050_read_test.h"
#include "driver_mpu6050_fifo_test.h"
#include "driver_mpu6050_dmp_read_test.h"
#include "driver_mpu6050_dmp_tap_orient_motion_test.h"
#include "driver_mpu6050_dmp_pedometer_test.h"
#include "driver_mpu6050_basic.h"
#include "driver_mpu6050_fifo.h"
#include "driver_mpu6050_dmp.h"

#include "mutex.h"
#include <stdlib.h>
/*_____ D E C L A R A T I O N S ____________________________________________*/

// #define REGISTER_TEST
// #define READ_TEST
// #define FIFO_TEST
// #define DMP_TEST
// #define MOTION_TEST
// #define PEDOMETER_TEST
#define READ_EXAMPLE
// #define FIFO_EXAMPLE
// #define DMP_EXAMPLE
// #define MOTION_EXAMPLE
// #define PEDOMETER_EXAMPLE

/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void MPU6050_test(void);

void MPU6050_simple_polling(void);
void MPU6050_Adafruit_polling(void);
