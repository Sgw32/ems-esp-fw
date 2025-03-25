#include "i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ems_setup.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "I2C_MASTER";

#define I2C_TIMEOUT_MS 100

static esp_err_t i2c_master_init(i2c_port_t i2c_port){
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = PIN_SDA;
    conf.scl_io_num = PIN_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FRQ;
    i2c_param_config(i2c_port, &conf);
    return i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0);
}

void i2c_init(void)
{
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_master_init(I2C_BUS_PORT);
}

void i2c_scan(void)
{
    uint8_t address;
    esp_err_t res;

    printf("Scanning I2C bus...\n");
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    printf("00:         ");
    
    for (address = 3; address < 0x78; address++) 
    {
        // Try to write to the device to check if it is available
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
        i2c_master_stop(cmd);

        res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
        if (address % 16 == 0)
            printf("\n%.2x:", address);
        if (res == 0)
            printf(" %.2x", address);
        else
            printf(" --");
        i2c_cmd_link_delete(cmd);
    }

    printf("\n\n");
}

