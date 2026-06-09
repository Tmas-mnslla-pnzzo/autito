#include "soft_i2c.h"
#include "main.h"

#define SDA_HIGH() HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_SET)
#define SDA_LOW()  HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_RESET)
#define SCL_HIGH() HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_SET)
#define SCL_LOW()  HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_RESET)

static void i2c_delay(void) {
    for(volatile int i = 0; i < 10; i++);
}

static void i2c_start(void) {
    SDA_HIGH(); SCL_HIGH(); i2c_delay();
    SDA_LOW();  i2c_delay();
    SCL_LOW();  i2c_delay();
}

static void i2c_stop(void) {
    SDA_LOW();  SCL_HIGH(); i2c_delay();
    SDA_HIGH(); i2c_delay();
}

static void i2c_write_byte(uint8_t byte) {
    for (int i = 7; i >= 0; i--) {
        if (byte & (1 << i)) { SDA_HIGH(); }
        else                  { SDA_LOW();  }
        i2c_delay();
        SCL_HIGH(); i2c_delay();
        SCL_LOW();  i2c_delay();
    }
    SDA_HIGH();
    i2c_delay();
    SCL_HIGH(); i2c_delay();
    SCL_LOW();  i2c_delay();
}

uint8_t soft_i2c_write(uint8_t addr, const uint8_t *data, uint8_t len) {
    i2c_start();
    i2c_write_byte(addr << 1);
    for (uint8_t i = 0; i < len; i++) {
        i2c_write_byte(data[i]);
    }
    i2c_stop();
    return 0;
}
