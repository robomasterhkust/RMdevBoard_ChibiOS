#ifndef _IST8310_H_
#define _IST8310_H_

/* To specify that this sensor is the slave of an IMU, i.e. MPU6500*/
#ifndef IMU_SLAVE
#define IMU_SLAVE
#endif

#define IST8310_PSC 0.003f
#define IST8310_SINGLE_MEASUREMENT    0
#define IST8310_SAMPLE_RATE_1_2HZ   255

#define IST8310_RESET()     (palClearPad(GPIOE,GPIOE_IST8310_RST))
#define IST8310_SET()       (palSetPad(GPIOE,GPIOE_IST8310_RST))

/*
 * NOTE: Read the schematic to find out the I2C address of IST8310
 * IST8310_ADDR_$(CAD1_HIGH)_$(CAD0_HIGH) or IST8310_ADDR_FLOATING
 */
typedef enum{
  IST8310_ADDR_FLOATING = 0x0E,
  IST8310_ADDR_0_0      = 0x0C,
  IST8310_ADDR_0_1      = 0x0D,
  IST8310_ADDR_1_0      = 0x0E,
  IST8310_ADDR_1_1      = 0x0F

} ist8310_i2c_addr_t;

typedef enum {
  IST8310_AXIS_REV_NO = 0,
  IST8310_AXIS_REV_X = 1,
  IST8310_AXIS_REV_Y = 2,
  IST8310_AXIS_REV_Z = 4,
} ist8310_axis_rev_t;

typedef enum{
  IST8310_INVALID_SAMPLE_RATE = 1
} ist8310_error_flag_t;

typedef struct{
  ist8310_i2c_addr_t addr;
  uint8_t sample_rate;
  uint8_t axis_rev;
}magConfigStruct;

typedef struct{
  bool _inited;
  uint32_t errorFlag;

  volatile float data[3];
  float _offset[3];
  bool  _axis_rev[3];

  #ifdef IMU_SLAVE
    SPIDriver* _spi;
  #else
    ist8310_i2c_addr_t addr;
  #endif
}magStruct;

magStruct* ist8310_get(void);

volatile float* ist8310_getValue(void);
uint32_t ist8310_getError(void);

uint8_t ist8310_init(const magConfigStruct* const conf);
uint8_t ist8310_update(void);

#endif
