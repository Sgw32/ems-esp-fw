#ifndef EMS_SETUP
#define EMS_SETUP

//#define TEST_BRD

/** 
 * @file ems_setup.h
 * @brief Device Setup and Configuration
 *
 * This file contains definitions for device states, pin mappings, 
 * I2C configuration, display settings, and Bluetooth parameters.
 */

/** @brief Device states enumeration. */
typedef enum {
    DEVICE_STATE_OFF,        /**< Device is off */
    DEVICE_STATE_BOOT,       /**< Device is starting */
    DEVICE_STATE_IDLE,       /**< Device is idle */
    DEVICE_STATE_SHUTDOWN    /**< Device is shutting down */
} device_state_t;

#ifdef TEST_BRD

/** @name Pin Definitions
 *  GPIO pin assignments for various components.
 *  @{
 */
#define PIN_START_BUTTON  GPIO_NUM_35  /**< Start button GPIO pin */
#define PIN_PWR_LATCH     GPIO_NUM_36  /**< Power latch GPIO pin */
#define PIN_SDA           GPIO_NUM_10  /**< I2C SDA pin */
#define PIN_SCL           GPIO_NUM_11  /**< I2C SCL pin */
/** @} */
#else
#define PIN_START_BUTTON  GPIO_NUM_34  /**< Start button GPIO pin */
#define PIN_PWR_LATCH     GPIO_NUM_46  /**< Power latch GPIO pin */
#define PIN_SDA           GPIO_NUM_10  /**< I2C SDA pin */
#define PIN_SCL           GPIO_NUM_11  /**< I2C SCL pin */
#endif

/** @name I2C Configuration
 *  Definitions for I2C communication.
 *  @{
 */
#define I2C_BUS_PORT  I2C_NUM_0        /**< I2C hardware port */
#define I2C_FRQ       (100 * 1000)     /**< I2C frequency (100 kHz) */
/** @} */

/** @name SSD1306 Display Configuration
 *  Parameters for the SSD1306 OLED display.
 *  @{
 */
#define SSD1306_PIN_NUM_SDA   PIN_SDA   /**< SSD1306 I2C SDA pin */
#define SSD1306_PIN_NUM_SCL   PIN_SCL   /**< SSD1306 I2C SCL pin */
#define SSD1306_PIN_NUM_RST   -1        /**< SSD1306 reset pin (-1 if not used) */
#define SSD1306_I2C_HW_ADDR   0x3C      /**< SSD1306 I2C hardware address */
#define SSD1306_LCD_H_RES     128       /**< SSD1306 horizontal resolution */
#define SSD1306_LCD_V_RES     64        /**< SSD1306 vertical resolution */
#define SSD1306_LCD_CMD_BITS  8         /**< Command bit size */
#define SSD1306_LCD_PARAM_BITS 8        /**< Parameter bit size */
/** @} */

/** @brief Bluetooth device name. */
#define EMS_BLE_NAME "EMS_BLE"

/** @brief Main task loop execution interval in milliseconds. */
#define MAIN_TASK_LOOP_TIME_MS 100

/** @name OTA Configuration
 *  OTA update security settings.
 *  @{
 */
#define CONFIG_EXAMPLE_OTA_SECURITY_VERSION_2 1 /**< Enable OTA security version 2 */
#define CONFIG_EXAMPLE_OTA_SEC2_DEV_MODE 1      /**< Enable OTA security development mode */
#define CONFIG_EXAMPLE_USE_PROTOCOMM 1         /**< Enable ProtoComm for OTA communication */
/** @} */

#endif /* EMS_SETUP */
