#ifndef EMS_SETUP
#define EMS_SETUP

#define PIN_START_BUTTON  GPIO_NUM_34
#define PIN_PWR_LATCH     GPIO_NUM_46

#define PIN_SDA           GPIO_NUM_10
#define PIN_SCL           GPIO_NUM_11

#define I2C_BUS_PORT  0
#define SSD1306_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define SSD1306_PIN_NUM_SDA           3
#define SSD1306_PIN_NUM_SCL           4
#define SSD1306_PIN_NUM_RST           -1
#define SSD1306_I2C_HW_ADDR           0x3C

#define SSD1306_LCD_H_RES              128
#define SSD1306_LCD_V_RES              64
// Bit number used to represent command and parameter
#define SSD1306_LCD_CMD_BITS           8
#define SSD1306_LCD_PARAM_BITS         8

#endif