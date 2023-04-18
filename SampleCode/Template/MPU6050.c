/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "NuMicro.h"

#include "MPU6050.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

uint8_t g_buf[256];                         /**< uart buffer */
volatile uint16_t g_len;                    /**< uart buffer length */
uint8_t (*g_gpio_irq)(void) = NULL;         /**< gpio irq */
int16_t gs_accel_raw[128][3];               /**< accel raw buffer */
float gs_accel_g[128][3];                   /**< accel g buffer */
int16_t gs_gyro_raw[128][3];                /**< gyro raw buffer */
float gs_gyro_dps[128][3];                  /**< gyro dps buffer */
int32_t gs_quat[128][4];                    /**< quat buffer */
float gs_pitch[128];                        /**< pitch buffer */
float gs_roll[128];                         /**< roll buffer */
float gs_yaw[128];                          /**< yaw buffer */
volatile uint8_t gs_flag;                   /**< flag */

const mpu6050_address_t addr = MPU6050_ADDRESS_AD0_LOW;
// const mpu6050_address_t addr = MPU6050_ADDRESS_AD0_HIGH;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/


void GPCDEF_IRQHandler(void)
{
    volatile uint32_t temp;

    /* To check if PC.5 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PC, BIT12))
    {
        GPIO_CLR_INT_FLAG(PC, BIT2);
        /* run the irq in the mutex mode */
        mutex_irq(g_gpio_irq);        
    }
    else
    {
        /* Un-expected interrupt. Just clear all PC interrupts */
        temp = PC->INTSRC;
        PC->INTSRC = temp;
        printf("Un-expected interrupts.\n");
    }
}

//USE PC2 , connect to MPU6050 INT pin
uint8_t gpio_interrupt_init(void)
{

    GPIO_SetMode(PC, BIT2, GPIO_MODE_INPUT);
    GPIO_EnableInt(PC, 2, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPIO_PCPDPEPF_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    // GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    // GPIO_ENABLE_DEBOUNCE(PC, BIT2);
    
    return 0;
}

/**
 * @brief  gpio interrupt deinit
 * @return status code
 *         - 0 success
 * @note   none
 */
uint8_t gpio_interrupt_deinit(void)
{

    GPIO_DisableInt(PC, 2);
    NVIC_DisableIRQ(GPIO_PCPDPEPF_IRQn);
    
    return 0;
}



/**
 * @brief     interface receive callback
 * @param[in] type is the irq type
 * @note      none
 */
static void a_receive_callback(uint8_t type)
{
    switch (type)
    {
        case MPU6050_INTERRUPT_MOTION :
        {
            gs_flag |= 1 << 0;
            mpu6050_interface_debug_print("mpu6050: irq motion.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_FIFO_OVERFLOW :
        {
            mpu6050_interface_debug_print("mpu6050: irq fifo overflow.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_I2C_MAST :
        {
            mpu6050_interface_debug_print("mpu6050: irq i2c master.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_DMP :
        {
            mpu6050_interface_debug_print("mpu6050: irq dmp\n");
            
            break;
        }
        case MPU6050_INTERRUPT_DATA_READY :
        {
            mpu6050_interface_debug_print("mpu6050: irq data ready\n");
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: irq unknown code.\n");
            
            break;
        }
    }
}

/**
 * @brief     interface dmp tap callback
 * @param[in] count is the tap count
 * @param[in] direction is the tap direction
 * @note      none
 */
static void a_dmp_tap_callback(uint8_t count, uint8_t direction)
{
    switch (direction)
    {
        case MPU6050_DMP_TAP_X_UP :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq x up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_X_DOWN :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq x down with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Y_UP :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq y up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Y_DOWN :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq y down with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Z_UP :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq z up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Z_DOWN :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq z down with %d.\n", count);
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq unknown code.\n");
            
            break;
        }
    }
}

/**
 * @brief     interface dmp orient callback
 * @param[in] orient is the dmp orient
 * @note      none
 */
static void a_dmp_orient_callback(uint8_t orientation)
{
    switch (orientation)
    {
        case MPU6050_DMP_ORIENT_PORTRAIT :
        {
            gs_flag |= 1 << 2;
            mpu6050_interface_debug_print("mpu6050: orient irq portrait.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_LANDSCAPE :
        {
            gs_flag |= 1 << 2;
            mpu6050_interface_debug_print("mpu6050: orient irq landscape.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_REVERSE_PORTRAIT :
        {
            gs_flag |= 1 << 2;
            mpu6050_interface_debug_print("mpu6050: orient irq reverse portrait.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_REVERSE_LANDSCAPE :
        {
            gs_flag |= 1 << 2;
            mpu6050_interface_debug_print("mpu6050: orient irq reverse landscape.\n");
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq unknown code.\n");
            
            break;
        }
    }
}

void e_pedometer(void)
{
    uint8_t times = 3;
    uint8_t res = 0;

    uint32_t i;
    uint32_t cnt = 0;
    uint32_t cnt_check = 0;
    uint16_t l;

    /* gpio init */
    if (gpio_interrupt_init() != 0)
    {
        printf("%s : run e_pedometer test , gpio_interrupt_init FAIL\r\n", __FUNCTION__);
        return ;
    }
    
    /* set the callback */
    g_gpio_irq = mpu6050_dmp_irq_handler;
    
    /* dmp init */
    if (mpu6050_dmp_init(addr, mpu6050_interface_receive_callback, NULL, NULL) != 0)
    {
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();

        printf("%s : run e_pedometer test , mpu6050_dmp_init FAIL\r\n", __FUNCTION__);
        
        return ;
    }
    else
    {
        
        /* loop */
        i = 0;
        while (times != 0)
        {
            
            /* delay 200 ms */
            mpu6050_interface_delay_ms(200);
            
            /* mutex lock */
            (void)mutex_lock();
            
            /* read data */
            l = 128;
            res = mpu6050_dmp_read_all(gs_accel_raw, gs_accel_g,
                                    gs_gyro_raw, gs_gyro_dps, 
                                    gs_quat,
                                    gs_pitch, gs_roll, gs_yaw,
                                    &l
                                    );
            if (res == 0)
            {
                i++;
                if (i > 5)
                {
                    i = 0;
                    
                    /* get the pedometer step count */
                    res = mpu6050_dmp_get_pedometer_counter(&cnt);
                    if (res != 0)
                    {
                        mpu6050_interface_debug_print("mpu6050: dmp get pedometer step count failed.\n");
                        (void)mpu6050_dmp_deinit();
                        g_gpio_irq = NULL;
                        (void)gpio_interrupt_deinit();
                        (void)mutex_unlock();
                        
                        printf("%s : run e_pedometer test , mpu6050_dmp_get_pedometer_counter FAIL\r\n", __FUNCTION__);

                        return ;
                    }
                    
                    /* check the cnt */
                    if (cnt != cnt_check)
                    {
                        mpu6050_interface_debug_print("mpu6050: pedometer step count is %d.\n", cnt);
                        cnt_check = cnt;
                        times--;
                    }
                }
            }
            
            /* mutex unlock */
            (void)mutex_unlock();
        }
        
        /* dmp deinit */
        (void)mpu6050_dmp_deinit();
        
        /* gpio deinit */
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();
        
}
}

void e_motion(void)
{
    uint8_t res = 0;
    uint32_t i;
    uint16_t l;
    
    /* gpio init */
    if (gpio_interrupt_init() != 0)
    {
        printf("%s : run e_motion test , gpio_interrupt_init FAIL\r\n", __FUNCTION__);
        return ;
    }
    
    /* set the callback */
    g_gpio_irq = mpu6050_dmp_irq_handler;
    
    /* dmp init */
    if (mpu6050_dmp_init(addr, a_receive_callback, 
                        a_dmp_tap_callback, a_dmp_orient_callback) != 0)
    {
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();

        printf("%s : run e_motion test , mpu6050_dmp_init FAIL\r\n", __FUNCTION__);
        
        return ;
    }
    else
    {                
        /* flag init */
        gs_flag = 0;
        
        /* delay 200ms */
        mpu6050_interface_delay_ms(200);
        
        /* loop */
        for (i = 0; i < 1000; i++)
        {
            
            /* mutex lock */
            (void)mutex_lock();
            
            /* read data */
            l = 128;
            res = mpu6050_dmp_read_all(gs_accel_raw, gs_accel_g,
                                    gs_gyro_raw, gs_gyro_dps,
                                    gs_quat,
                                    gs_pitch, gs_roll, gs_yaw,
                                    &l
                                    );
            if (res != 0)
            {
                /* output data */
                mpu6050_interface_debug_print("mpu6050: dmp read failed.\n");
            }
            
            /* mutex unlock */
            (void)mutex_unlock();
            
            /* delay 500ms */
            mpu6050_interface_delay_ms(500);
            
            /* check the flag */
            if ((gs_flag & 0x7) == 0x7)
            {
                break;
            }
        }
        
        /* finish dmp tap orient motion */
        mpu6050_interface_debug_print("mpu6050: finish dmp tap orient motion.\n");
        
        /* dmp deinit */
        (void)mpu6050_dmp_deinit();
        
        /* gpio deinit */
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();
        
    }    
}

void e_dmp(void)
{    
    uint8_t times = 3;

    uint32_t i;
    uint16_t len;

    /* gpio init */
    if (gpio_interrupt_init() != 0)
    {
        printf("%s : run e_dmp test , gpio_interrupt_init FAIL\r\n", __FUNCTION__);
        return ;
    }
    
    /* set the callback */
    g_gpio_irq = mpu6050_dmp_irq_handler;
    
    /* dmp init */
    if (mpu6050_dmp_init(addr, mpu6050_interface_receive_callback, NULL, NULL) != 0)
    {
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();

        printf("%s : run e_dmp test , mpu6050_dmp_init FAIL\r\n", __FUNCTION__);
        
        return ;
    }
    else
    {
        
        /* delay 500 ms */
        mpu6050_interface_delay_ms(500);
        
        /* loop */
        for (i = 0; i < times; i++)
        {
            len = 128;
            
            /* mutex lock */
            (void)mutex_lock();
            
            /* read data */
            if (mpu6050_dmp_read_all(gs_accel_raw, gs_accel_g,
                                    gs_gyro_raw, gs_gyro_dps, 
                                    gs_quat,
                                    gs_pitch, gs_roll, gs_yaw,
                                    &len) != 0
                                    )
            {
                (void)mpu6050_dmp_deinit();
                g_gpio_irq = NULL;
                (void)gpio_interrupt_deinit();
                (void)mutex_unlock();

                printf("%s : run e_dmp test , mpu6050_dmp_read_all FAIL\r\n", __FUNCTION__);
                
                return ;
            }
            
            /* mutex unlock */
            (void)mutex_unlock();
            
            /* output */
            mpu6050_interface_debug_print("mpu6050: %d/%d.\n", i + 1, times);
            mpu6050_interface_debug_print("mpu6050: fifo %d.\n", len);
            mpu6050_interface_debug_print("mpu6050: pitch[0] is %0.2fdps.\n", gs_pitch[0]);
            mpu6050_interface_debug_print("mpu6050: roll[0] is %0.2fdps.\n", gs_roll[0]);
            mpu6050_interface_debug_print("mpu6050: yaw[0] is %0.2fdps.\n", gs_yaw[0]);
            mpu6050_interface_debug_print("mpu6050: acc x[0] is %0.2fg.\n", gs_accel_g[0][0]);
            mpu6050_interface_debug_print("mpu6050: acc y[0] is %0.2fg.\n", gs_accel_g[0][1]);
            mpu6050_interface_debug_print("mpu6050: acc z[0] is %0.2fg.\n", gs_accel_g[0][2]);
            mpu6050_interface_debug_print("mpu6050: gyro x[0] is %0.2fdps.\n", gs_gyro_dps[0][0]);
            mpu6050_interface_debug_print("mpu6050: gyro y[0] is %0.2fdps.\n", gs_gyro_dps[0][1]);
            mpu6050_interface_debug_print("mpu6050: gyro z[0] is %0.2fdps.\n", gs_gyro_dps[0][2]);
            
            /* delay 500 ms */
            mpu6050_interface_delay_ms(500);
        }
        
        /* dmp deinit */
        (void)mpu6050_dmp_deinit();
        
        /* gpio deinit */
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();
        
    }
}

void e_fifo(void)
{
    uint8_t times = 3;

    uint32_t i;
    uint16_t len;

    /* gpio init */
    if (gpio_interrupt_init() != 0)
    {
        printf("%s : run e_fifo test , gpio_interrupt_init FAIL\r\n", __FUNCTION__);
                
        return ;
    }
    
    /* don't need */
    g_gpio_irq = NULL;
    
    /* fifo init */
    if (mpu6050_fifo_init(addr) != 0)
    {
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();

        printf("%s : run e_fifo test , mpu6050_fifo_init FAIL\r\n", __FUNCTION__);
        
        return ;
    }
    else
    {
        
        /* delay 100 ms */
        mpu6050_interface_delay_ms(100);
        
        /* loop */
        for (i = 0; i < times; i++)
        {
            len = 128;
            
            /* mutex lock */
            (void)mutex_lock();
            
            /* read data */
            if (mpu6050_fifo_read(gs_accel_raw, gs_accel_g,
                                gs_gyro_raw, gs_gyro_dps, &len) != 0)
            {
                (void)mpu6050_fifo_deinit();
                g_gpio_irq = NULL;
                (void)gpio_interrupt_deinit();
                (void)mutex_unlock();

                printf("%s : run e_fifo test , mpu6050_fifo_read FAIL\r\n", __FUNCTION__);
                
                return ;
            }
            
            /* mutex unlock */
            (void)mutex_unlock();
            
            /* output */
            mpu6050_interface_debug_print("mpu6050: %d/%d.\n", i + 1, times);
            mpu6050_interface_debug_print("mpu6050: fifo %d.\n", len);
            mpu6050_interface_debug_print("mpu6050: acc x[0] is %0.2fg.\n", gs_accel_g[0][0]);
            mpu6050_interface_debug_print("mpu6050: acc y[0] is %0.2fg.\n", gs_accel_g[0][1]);
            mpu6050_interface_debug_print("mpu6050: acc z[0] is %0.2fg.\n", gs_accel_g[0][2]);
            mpu6050_interface_debug_print("mpu6050: gyro x[0] is %0.2fdps.\n", gs_gyro_dps[0][0]);
            mpu6050_interface_debug_print("mpu6050: gyro y[0] is %0.2fdps.\n", gs_gyro_dps[0][1]);
            mpu6050_interface_debug_print("mpu6050: gyro z[0] is %0.2fdps.\n", gs_gyro_dps[0][2]);
            
            /* delay 100 ms */
            mpu6050_interface_delay_ms(100);
        }
        
        /* fifo deinit */
        (void)mpu6050_fifo_deinit();
        
        /* gpio deinit */
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();
    }

}

void e_read(void)
{
    uint8_t times = 3;

    uint32_t i;
    float g[3];
    float dps[3];
    float degrees;

    /* basic init */
    if (mpu6050_basic_init(addr) != 0)
    {
        printf("%s : run e_read test , mpu6050_basic_init FAIL\r\n", __FUNCTION__);
        return ;
    }
    else
    {
        
        /* loop */
        for (i = 0; i < times; i++)
        {
            /* read data */
            if (mpu6050_basic_read(g, dps) != 0)
            {
                (void)mpu6050_basic_deinit();

                printf("%s : run e_read test , mpu6050_basic_read FAIL\r\n", __FUNCTION__);
                
                return ;
            }
            
            /* read the temperature */
            if (mpu6050_basic_read_temperature(&degrees) != 0)
            {
                (void)mpu6050_basic_deinit();

                printf("%s : run e_read test , mpu6050_basic_read_temperature FAIL\r\n", __FUNCTION__);
                
                return ;
            }
            
            /* output */
            mpu6050_interface_debug_print("mpu6050: %d/%d.\n", i + 1, times);
            mpu6050_interface_debug_print("mpu6050: acc x is %0.2fg.\n", g[0]);
            mpu6050_interface_debug_print("mpu6050: acc y is %0.2fg.\n", g[1]);
            mpu6050_interface_debug_print("mpu6050: acc z is %0.2fg.\n", g[2]);
            mpu6050_interface_debug_print("mpu6050: gyro x is %0.2fdps.\n", dps[0]);
            mpu6050_interface_debug_print("mpu6050: gyro y is %0.2fdps.\n", dps[1]);
            mpu6050_interface_debug_print("mpu6050: gyro z is %0.2fdps.\n", dps[2]);
            mpu6050_interface_debug_print("mpu6050: temperature %0.2fC.\n", degrees);
            
            /* delay 1000 ms */
            mpu6050_interface_delay_ms(1000);
        }
        
        /* basic deinit */
        (void)mpu6050_basic_deinit();
    }

}

void t_pedometer(void)
{
    uint8_t times = 3;

    /* gpio init */
    if (gpio_interrupt_init() != 0)
    {
        printf("%s : run pedometer test , gpio_interrupt_init FAIL\r\n", __FUNCTION__);
        return ;
    }
    
    /* set the callback */
    g_gpio_irq = mpu6050_dmp_pedometer_test_irq_handler;
    
    /* mutex lock */
    (void)mutex_lock();
    
    /* run pedometer test */
    if (mpu6050_dmp_pedometer_test(addr, times) != 0)
    {
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();
        (void)mutex_unlock();

        printf("%s : run pedometer test (FAIL)\r\n", __FUNCTION__);
        
        return ;
    }
    
    /* mutex unlock */
    (void)mutex_unlock();
    
    /* gpio deint */
    g_gpio_irq = NULL;
    (void)gpio_interrupt_deinit();
}

void t_motion(void)
{
    /* gpio init */
    if (gpio_interrupt_init() != 0)
    {
        printf("%s : run motion test , gpio_interrupt_init FAIL\r\n", __FUNCTION__);
        return ;
    }
    
    /* set the callback */
    g_gpio_irq = mpu6050_dmp_tap_orient_motion_test_irq_handler;
    
    /* run motion test */
    if (mpu6050_dmp_tap_orient_motion_test(addr) != 0)
    {
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();

        printf("%s : run motion test (FAIL)\r\n", __FUNCTION__);
        
        return ;
    }
    
    /* gpio deinit */
    g_gpio_irq = NULL;
    (void)gpio_interrupt_deinit();

}

void t_dmp(void)
{
    uint8_t times = 3;

    /* gpio init */
    if (gpio_interrupt_init() != 0)
    {
        printf("%s : run dmp test , gpio_interrupt_init FAIL\r\n", __FUNCTION__);
        return ;
    }
    
    /* set the callback */
    g_gpio_irq = mpu6050_dmp_read_test_irq_handler;
    
    /* mutex lock */
    (void)mutex_lock();
    
    /* run dmp test */
    if (mpu6050_dmp_read_test(addr, times) != 0)
    {
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();
        (void)mutex_unlock();

        printf("%s : run dmp test (FAIL)\r\n", __FUNCTION__);
        
        return ;
    }
    
    /* mutex unlock */
    (void)mutex_unlock();
    
    /* gpio deinit */
    g_gpio_irq = NULL;
    (void)gpio_interrupt_deinit();
}

void t_fifo(void)
{
    uint8_t times = 3;
    
    printf("%s : run fifo test\r\n", __FUNCTION__);

    /* gpio init */
    if (gpio_interrupt_init() != 0)
    {
        printf("%s : run fifo test , gpio_interrupt_init FAIL\r\n", __FUNCTION__);
        return ;
    }
        
    /* don't need */
    g_gpio_irq = NULL;
        
    /* mutex lock */
    (void)mutex_lock();
    
    /* run fifo test */
    if (mpu6050_fifo_test(addr, times) != 0)
    {
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();
        (void)mutex_unlock();

        printf("%s : run fifo test (FAIL)\r\n", __FUNCTION__);
        
        return ;
    }
        
    /* mutex unlock */
    (void)mutex_unlock();
        
    /* gpio deinit */
    g_gpio_irq = NULL;
    (void)gpio_interrupt_deinit();
}

void t_read(void)
{
    uint8_t times = 3;

    printf("%s : run read test\r\n", __FUNCTION__);
    if (mpu6050_read_test(addr, times) != 0)
    {
        printf("%s : run read test (FAIL)\r\n", __FUNCTION__);
    }
    else
    {
        printf("%s : run read test (OK)\r\n", __FUNCTION__);
    }          
}

void t_reg(void)
{
    printf("%s : run reg test\r\n", __FUNCTION__);
    if (mpu6050_register_test(addr) != 0)
    {
        printf("%s : run reg test (FAIL)\r\n", __FUNCTION__);
    }
    else
    {
        printf("%s : run reg test (OK)\r\n", __FUNCTION__);
    }  
}

void MPU6050_test(void)
{
    #if defined (REGISTER_TEST)
    t_reg();    
    #endif

    #if defined (READ_TEST)
    t_read();
    #endif

    #if defined (FIFO_TEST)
    t_fifo();
    #endif

    #if defined (DMP_TEST)
    t_dmp();
    #endif    

    #if defined (MOTION_TEST)
    t_motion();
    #endif    

    #if defined (PEDOMETER_TEST)
    t_pedometer();
    #endif    

    #if defined (READ_EXAMPLE)
    e_read();
    #endif    

    #if defined (FIFO_EXAMPLE)
    e_fifo();
    #endif    

    #if defined (DMP_EXAMPLE)
    e_dmp();
    #endif     

    #if defined (MOTION_EXAMPLE)
    e_motion();
    #endif        

    #if defined (PEDOMETER_EXAMPLE)
    e_pedometer();
    #endif  
}

void MPU6050_simple_polling(void)
{
    static uint8_t init_once = 1;
    static uint8_t init_ready = 1;
    float g[3];
    float dps[3];
    float degrees;

    if (init_once)
    {
        init_once = 0;
        init_ready = mpu6050_basic_init(addr);
    }

    if (init_ready == 0)
    {
        /* read data */
        if (mpu6050_basic_read(g, dps) != 0)
        {
            (void)mpu6050_basic_deinit();

            printf("%s : run e_read test , mpu6050_basic_read FAIL\r\n", __FUNCTION__);
            
            return ;
        }
    
        /* read the temperature */
        if (mpu6050_basic_read_temperature(&degrees) != 0)
        {
            (void)mpu6050_basic_deinit();

            printf("%s : run e_read test , mpu6050_basic_read_temperature FAIL\r\n", __FUNCTION__);
            
            return ;
        }

        mpu6050_interface_debug_print("acc : %0.2fg,%0.2fg,%0.2fg, ",
            g[0],g[1],g[2]);
        mpu6050_interface_debug_print("gyro : %0.2fdps,%0.2fdps,%0.2fdps, ",
            dps[0],dps[1],dps[2]);
        mpu6050_interface_debug_print("temperature = %0.2fC\r\n" , 
            degrees);

        /* delay 1000 ms */
        mpu6050_interface_delay_ms(100);
    }
}

