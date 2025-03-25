#ifndef EMS_SETUP
#define EMS_SETUP

// Device states
typedef enum {
    DEVICE_STATE_OFF,         // Device is off
    DEVICE_STATE_BOOT,        // Device is starting
    DEVICE_STATE_IDLE,        // Device idle
    DEVICE_STATE_SHUTDOWN     // Выключение устройства
} device_state_t;

// #define PIN_START_BUTTON  GPIO_NUM_34
// #define PIN_PWR_LATCH     GPIO_NUM_46
#define PIN_START_BUTTON  GPIO_NUM_35
#define PIN_PWR_LATCH     GPIO_NUM_36

#define PIN_SDA           GPIO_NUM_10
#define PIN_SCL           GPIO_NUM_11

#define I2C_BUS_PORT  I2C_NUM_0
#define I2C_FRQ    (100 * 1000)
#define SSD1306_PIN_NUM_SDA           PIN_SDA
#define SSD1306_PIN_NUM_SCL           PIN_SCL
#define SSD1306_PIN_NUM_RST           -1
#define SSD1306_I2C_HW_ADDR           0x3C

#define SSD1306_LCD_H_RES              128
#define SSD1306_LCD_V_RES              64
// Bit number used to represent command and parameter
#define SSD1306_LCD_CMD_BITS           8
#define SSD1306_LCD_PARAM_BITS         8

#define EMS_BLE_NAME "EMS_BLE"
#define MAIN_TASK_LOOP_TIME_MS         100

#define CONFIG_EXAMPLE_OTA_SECURITY_VERSION_2 1
#define CONFIG_EXAMPLE_OTA_SEC2_DEV_MODE 1
#define CONFIG_EXAMPLE_USE_PROTOCOMM 1


#endif