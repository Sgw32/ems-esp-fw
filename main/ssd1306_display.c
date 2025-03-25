#include "ssd1306_display.h"
#include <math.h>
#include "ems_setup.h"
#include <stdio.h>
#include "ssd1306.h"
#include "font8x8_basic.h"

static const char *TAG = "SSD1306_DISPLAY";
#define CUBE_SIZE 10  // 10x10x10 pixels
#define ROTATION_SPEED 0.1  // Adjust rotation speed
#define NUM_VERTICES 8
#define NUM_EDGES 12

static SSD1306_t dev;

static char h_rate_str[17] = "";

float r, x1, ya, z1, x2, y2, z2, x3, y3, z3;               //                                      //
int f[8][2];                                                // Draw box
int x = 64;                                                 // 64=128/2
int y = 32;                                                 // 32= 64/2
int c[8][3] = {                                            // Cube
        {-CUBE_SIZE,-CUBE_SIZE, CUBE_SIZE},{CUBE_SIZE,-CUBE_SIZE, CUBE_SIZE},{CUBE_SIZE,CUBE_SIZE, CUBE_SIZE},{-CUBE_SIZE,CUBE_SIZE, CUBE_SIZE},      //
        {-CUBE_SIZE,-CUBE_SIZE,-CUBE_SIZE},{CUBE_SIZE,-CUBE_SIZE,-CUBE_SIZE},{CUBE_SIZE,CUBE_SIZE,-CUBE_SIZE},{-CUBE_SIZE,CUBE_SIZE,-CUBE_SIZE} };    //

float anim_cnt = 0;
uint32_t cycles_cnt = 0;
// Animation callback to rotate the cube
static void anim_cube(float v) {
    for (int i = 0; i < NUM_EDGES; i++) 
    {
        _ssd1306_line(&dev, f[0][0],f[0][1],f[1][0],f[1][1],true);//
        _ssd1306_line(&dev, f[1][0],f[1][1],f[2][0],f[2][1],true);//
        _ssd1306_line(&dev, f[2][0],f[2][1],f[3][0],f[3][1],true);//
        _ssd1306_line(&dev, f[3][0],f[3][1],f[0][0],f[0][1],true);//
        _ssd1306_line(&dev, f[4][0],f[4][1],f[5][0],f[5][1],true);//
        _ssd1306_line(&dev, f[5][0],f[5][1],f[6][0],f[6][1],true);//
        _ssd1306_line(&dev, f[6][0],f[6][1],f[7][0],f[7][1],true);//
        _ssd1306_line(&dev, f[7][0],f[7][1],f[4][0],f[4][1],true);//
        _ssd1306_line(&dev, f[0][0],f[0][1],f[4][0],f[4][1],true);//
        _ssd1306_line(&dev, f[1][0],f[1][1],f[5][0],f[5][1],true);//
        _ssd1306_line(&dev, f[2][0],f[2][1],f[6][0],f[6][1],true);//
        _ssd1306_line(&dev, f[3][0],f[3][1],f[7][0],f[7][1],true);//
        _ssd1306_line(&dev, f[1][0],f[1][1],f[3][0],f[3][1],true);// cross
        _ssd1306_line(&dev, f[0][0],f[0][1],f[2][0],f[2][1],true);// cross
    }

    for (int i = 0; i < 8; i++) {                            //
        r  = v * 0.0174532;                                    // 1 degree
        x1 = c[i][2] * sin(r) + c[i][0] * cos(r);              // rotate Y
        ya = c[i][1];                                          //
        z1 = c[i][2] * cos(r) - c[i][0] * sin(r);              //
        x2 = x1;                                               //
        y2 = ya * cos(r) - z1 * sin(r);                        // rotate X
        z2 = ya * sin(r) + z1 * cos(r);                        //
        x3 = x2 * cos(r) - y2 * sin(r);                        // rotate Z
        y3 = x2 * sin(r) + y2 * cos(r);                        //
        z3 = z2;                                               //
        x3 = x3 + x ;                                          //
        y3 = y3 + y ;                                          //
        f[i][0] = x3;                                          // store new values
        f[i][1] = y3;                                          //
        f[i][2] = z3;                                          //
    }
    
    for (int i = 0; i < NUM_EDGES; i++) 
    {
        _ssd1306_line(&dev, f[0][0],f[0][1],f[1][0],f[1][1],false);//
        _ssd1306_line(&dev, f[1][0],f[1][1],f[2][0],f[2][1],false);//
        _ssd1306_line(&dev, f[2][0],f[2][1],f[3][0],f[3][1],false);//
        _ssd1306_line(&dev, f[3][0],f[3][1],f[0][0],f[0][1],false);//
        _ssd1306_line(&dev, f[4][0],f[4][1],f[5][0],f[5][1],false);//
        _ssd1306_line(&dev, f[5][0],f[5][1],f[6][0],f[6][1],false);//
        _ssd1306_line(&dev, f[6][0],f[6][1],f[7][0],f[7][1],false);//
        _ssd1306_line(&dev, f[7][0],f[7][1],f[4][0],f[4][1],false);//
        _ssd1306_line(&dev, f[0][0],f[0][1],f[4][0],f[4][1],false);//
        _ssd1306_line(&dev, f[1][0],f[1][1],f[5][0],f[5][1],false);//
        _ssd1306_line(&dev, f[2][0],f[2][1],f[6][0],f[6][1],false);//
        _ssd1306_line(&dev, f[3][0],f[3][1],f[7][0],f[7][1],false);//
        _ssd1306_line(&dev, f[1][0],f[1][1],f[3][0],f[3][1],false);// cross
        _ssd1306_line(&dev, f[0][0],f[0][1],f[2][0],f[2][1],false);// cross
    }
    
}

void setup_ui()
{
	ESP_LOGI(TAG, "INTERFACE is i2c");
	ESP_LOGI(TAG, "CONFIG_SDA_GPIO=%d",SSD1306_PIN_NUM_SDA);
	ESP_LOGI(TAG, "CONFIG_SCL_GPIO=%d",SSD1306_PIN_NUM_SCL);
	ssd1306_i2c_master_init(&dev, SSD1306_PIN_NUM_SDA, SSD1306_PIN_NUM_SCL, -1);

	ESP_LOGI(TAG, "Panel is 128x64");
	ssd1306_init(&dev, 128, 64);

	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
}

static inline void update_boot()
{
    if (cycles_cnt==0)
    {
        ssd1306_display_text(&dev, 1, "  ems test app  ", 17, false);
    }
    if (cycles_cnt<60000/MAIN_TASK_LOOP_TIME_MS)
    {
        anim_cnt+=2.0f*MAIN_TASK_LOOP_TIME_MS/20.0f;
        anim_cube(anim_cnt);
        if (anim_cnt>3600)
        {
            anim_cnt=0;
        }
        cycles_cnt+=1;
    }
    ssd1306_show_buffer(&dev);
}

static inline void update_idle(uint16_t heart_rate, uint8_t hrm_active)
{
    ssd1306_display_text(&dev, 1, "  ems test app  ", 17, false);
    if (!hrm_active)
    {
        ssd1306_display_text(&dev, 2, "Connect         ", 17, false);
        ssd1306_display_text(&dev, 3, "touch           ", 17, false);
        ssd1306_display_text(&dev, 4, "sensor          ", 17, false);
        ssd1306_display_text(&dev, 5, "                ", 17, false);
    }
    else
    {
        snprintf(h_rate_str, sizeof(h_rate_str), "BPM: %02u", heart_rate);
        ssd1306_display_text(&dev, 2, h_rate_str, 17, false);
        ssd1306_display_text(&dev, 3, "                ", 17, false);
        ssd1306_display_text(&dev, 4, "                ", 17, false);
        ssd1306_display_text(&dev, 5, "                ", 17, false);
    }
}

void update_ui(device_state_t state, uint16_t heart_rate, uint8_t hrm_active)
{
    switch (state) {
        case DEVICE_STATE_OFF:
            cycles_cnt = 0;
            break;
        case DEVICE_STATE_BOOT:
            update_boot();
            break;
        case DEVICE_STATE_IDLE:
            update_idle(heart_rate, hrm_active);
            break;
        case DEVICE_STATE_SHUTDOWN:
            ssd1306_clear_screen(&dev, false);
            break;
        default:
            break;
    }
}

