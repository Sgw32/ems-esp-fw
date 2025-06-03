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

// Define F1/F2 GPIOs and DAC channels
#define F1_GPIO GPIO_NUM_7
#define F2_GPIO GPIO_NUM_9

// --- GPIO pin assignments (adjust according to your board layout) ---
// #define LED_B_GPIO          GPIO_NUM_2
// #define LED_R_GPIO          GPIO_NUM_4
// #define CHRG_ON_GPIO        GPIO_NUM_18
// #define CHRG_ILIM_GPIO      GPIO_NUM_19
// #define CHRG_IN_GPIO        GPIO_NUM_23
// #define BAL_L_GPIO          GPIO_NUM_25
// #define BAL_H_GPIO          GPIO_NUM_26
#define CHRG_OUT_GPIO       GPIO_NUM_35
// #define CHGR_COMPLITE_GPIO  GPIO_NUM_22


#define GPIO_PWM        8
#define GPIO_35V_ON     12 
#define GPIO_WCLK       47

/** @} */
#else
/** @name Pin Definitions
 *  GPIO pin assignments for various components.
 *  @{
 */
#define PIN_START_BUTTON  GPIO_NUM_41  /**< Start button GPIO pin */
#define PIN_PWR_LATCH     GPIO_NUM_38 /**< Power latch GPIO pin */
#define PIN_SDA           GPIO_NUM_10  /**< I2C SDA pin */
#define PIN_SCL           GPIO_NUM_11  /**< I2C SCL pin */

// Define F1/F2 GPIOs and DAC channels
#define F1_GPIO GPIO_NUM_7
#define F2_GPIO GPIO_NUM_9

// --- GPIO pin assignments (adjust according to your board layout) ---
// #define LED_B_GPIO          GPIO_NUM_2
// #define LED_R_GPIO          GPIO_NUM_4
// #define CHRG_ON_GPIO        GPIO_NUM_18
// #define CHRG_ILIM_GPIO      GPIO_NUM_19
// #define CHRG_IN_GPIO        GPIO_NUM_23
// #define BAL_L_GPIO          GPIO_NUM_25
// #define BAL_H_GPIO          GPIO_NUM_26
#define CHRG_OUT_GPIO       GPIO_NUM_35
// #define CHGR_COMPLITE_GPIO  GPIO_NUM_22

#define GPIO_PWM        8
#define GPIO_35V_ON     12 
#define GPIO_WCLK       47
/** @} */
#endif

#define GPIO_DECREASE GPIO_NUM_20
#define GPIO_INCREASE GPIO_NUM_21
//#define GPIO_DECREASE GPIO_NUM_43
//#define GPIO_INCREASE GPIO_NUM_44
#define DAC_VALUE_STEP    32          // Step size for increment/decrement
#define DAC_MIN_VALUE     0
#define DAC_MAX_VALUE     4095        // 12-bit DAC (2^12 - 1)
#define GPIO_DAC_CLR     21

/** @name EMS Pulse timer */
#define EMS_TIMER_GROUP TIMER_GROUP_0
#define EMS_TIMER       TIMER_0

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
