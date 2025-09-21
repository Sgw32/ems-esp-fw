#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_app_format.h"
#include "esp_log.h"
#include "esp_system.h"
#include "pwr_button.h"
#include "device_sm.h"
#include "ems_setup.h"
#include "ems_common_driver/user.h"
#include "esp_app_desc.h"

static const char *TAG = "EXTRA_TASKS";

#define MAIN_TASK_DELAY_MS 25
#define CMD_TASK_DELAY_MS  50

static uint32_t pingCnt = 0;
extern const struct    MuxSeries       muxSeries;
extern struct movingAver pulseCurrent[MUX_CHANNELS], pulseVoltage[MUX_CHANNELS];

struct ChannelLoad chanelLoad[MUX_CHANNELS];
struct ChannelLoad chanelLoadLast[MUX_CHANNELS];
static uint8_t isLoadLastInit = 0;

uint8_t                 data[2];
uint32_t                uid;
uint8_t                 str[256];

extern uint32_t countCommandError;
extern uint32_t countCommandOk;
extern SmEmsTypeDef smEms;

extern uint8_t setup_error;
extern MuxChannelTypeDef setup_channel_error;
extern uint8_t setup_old_value_error;
extern uint8_t setup_value_error;

extern struct EmsPercentCfg emsPercentCfg[2];

static uint32_t                cntFlashValue;

static volatile AtomosResultTypeDef    atomResultA, atomResultB;
static uint32_t                        intAtom = 0;
static uint32_t                        cntAtom = 0;
static HardWareType                    hardware;

static uint32_t                batVoltL, batVoltH;

static bool                    buttonIsPush = false;
static bool                    impulseMsgEnabled = true;
static bool                    blinkOn = false;
static bool                    isAfterStart = false;
static uint32_t                startTime = 0;

static uint32_t                costumeId = 0;
static uint32_t                costumeIds[16];
static uint8_t                 costumeIdText[24]; 

static uint32_t buildNumber = 0;

static void handle_button_actions(void)
{
    pwr_button_handle();

    if (pwr_get_pressed_rising()) {
        ESP_LOGI(TAG, "Button pressed");
        emsStart();
    }
    if (pwr_get_pressed_falling()) {
        ESP_LOGI(TAG, "Button released");
        emsPause();
    }
}

static bool set_ble_name = false;
static uint32_t new_ble_num = 0;

void main_task(void *p)
{
    TickType_t tickBlue = 0, tickConnect = 0, tickSerena = 0;
    uint16_t btnPushMs = 0;
    SmEmsTypeDef smEmsLast;

#define BUTTON_PUSH_TIME        2500
#define BUTTON_INC_MS           25

    while (1) {
#ifdef TURN_OFF_WHEN_CHARGING
        if(hardware == HARDWARE_ESP32 && chrgIsCharging() == CHRG_ISCHARGING)
        {
        hvccOff();
        pwrOff();
        esp_restart();
        }
#endif

        if (set_ble_name)
        {
            set_ble_name = false;
            uint8_t       strNum[10];
            ESP_LOGI(TAG, "setFFitNumber: %lu", new_ble_num);
            snprintf((char *)strNum, sizeof(strNum), "FFit-%lu", (unsigned long)new_ble_num);

            esp_err_t err = Esp32StoreFFitNumber(new_ble_num);
            if(err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to store FFit number: %s", esp_err_to_name(err));
            }
            else
            {
                ESP_LOGI(TAG, "FFit number: %s", strNum);
            }

            err = Esp32WriteBleName((const char *)strNum);
            if(err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to persist BLE name: %s", esp_err_to_name(err));
            }
            else
            {
                ESP_LOGI(TAG, "Esp32WriteBleName number: %s", strNum);
            }

            // #if defined(CONFIG_BT_ENABLED) && CONFIG_BT_ENABLED
            // #if defined(CONFIG_BT_NIMBLE_ENABLED) && CONFIG_BT_NIMBLE_ENABLED
            // int rc = ble_svc_gap_device_name_set((const char *)dev_strNum);
            // if(rc != 0)
            // {
            //     ESP_LOGE(TAG_CMD_PARSER, "Failed to update GAP device name (rc=%d)", rc);
            // }
            // #elif defined(CONFIG_BT_BLE_ENABLED) && CONFIG_BT_BLE_ENABLED
            // esp_err_t ble_err = esp_ble_gap_set_device_name((const char *)dev_strNum);
            // if(ble_err != ESP_OK)
            // {
            //     ESP_LOGE(TAG_CMD_PARSER, "Failed to update BLE GAP name: %s", esp_err_to_name(ble_err));
            // }
            // #endif
            // #endif
        }

        if(buttonIsPush == true || btnPushMs > 0)
        {
            if(buttonIsPush == false)
            {
                if (btnPushMs < BUTTON_PUSH_TIME && (smEmsLast == SM_EMS_START || smEmsLast == SM_EMS_PAUSE_END))
                    emsStart();
                else
                    emsPause();

                btnPushMs = 0;
            }
            else
            {
                if(btnPushMs == 0)
                {
                    smEmsLast = smEms;
                    sendCmdAndParam((const uint8_t*)"info", (const uint8_t*)"off_button_press", ACCEPTED_NO);
                    ledsSet(LEDS_BLUE);
                    emsPause();
                }
                else if(btnPushMs >= BUTTON_PUSH_TIME)
                {
                    ledsSet(LEDS_OFF);
                    if (smEmsLast == SM_EMS_START || smEmsLast == SM_EMS_PAUSE_END)
                    {
                        vTaskDelay(250);
                        ledsSet(LEDS_BLUE);
                        sendCmdAndParam((const uint8_t*)"info", (const uint8_t*)"pause_button", ACCEPTED_NO);
                    }
                    
                    else
                    {
                        sendCmdAndParam((const uint8_t*)"info", (const uint8_t*)"off_button", ACCEPTED_NO);
                        hvccOff();
                        pwrOff();
                        smPwr = SM_PWR_OFF;
                    }
                    while(buttonIsPush == true)
                        vTaskDelay(10);
                    vTaskDelay(250);
                }
                btnPushMs += BUTTON_INC_MS;
            }
        }
        else
        {
            if(blinkOn == true)
            {
                ledsSet(LEDS_BLUE);
                vTaskDelay(100);
                ledsSet(LEDS_RED);
                vTaskDelay(100);
            }
            
            else if(isAfterStart == true)
            {
                ledsSet(LEDS_RED);
                vTaskDelay(100);
                ledsSet(LEDS_OFF);
                vTaskDelay(100);
            }
            else if(smPwr == SM_PWR_ON)
            {
                // управление синим светодиодом
                if(smBle == SM_BLE_CONNECT)
                {
                if(tickConnect == 0)
                {
                    ledSet(LED_RED, LED_ON);
                    ledSet(LED_BLUE, LED_OFF);
                    tickConnect = xTaskGetTickCount();
                }
                if(xTaskGetTickCount() - tickConnect <  TIME_CONNECT_SERENA)
                {
                    if(xTaskGetTickCount() - tickSerena > TICK_CONNECT_SERENA)
                    {
                    tickSerena = xTaskGetTickCount();
                    ledTog(LED_BLUE);
                    ledTog(LED_RED);
                    }
                }
                else
                {
                    ledSet(LED_BLUE, LED_ON);
                    
                    // управление красным светодиодом
                    if(smEmsStg == SM_EMS_STG_STIMULATE)
                    ledsSet(LEDS_RED);
                    else
                    ledsSet(LEDS_BLUE);
                }
                }
                else
                {
                    tickConnect = 0;
                    if(xTaskGetTickCount() - tickBlue > TICK_CONNECT_LED_BLINK)
                    {
                        tickBlue = xTaskGetTickCount();
                        ledTog(LED_BLUE);
                        ledSet(LED_RED, LED_OFF);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(MAIN_TASK_DELAY_MS));
    }
}

void cmd_proc_task(void *p)
{
    TickType_t            pingTick = 0, failsTick = 0, costumeTick = 0, delayTick = 0, voltsTick = 0, resistenseTick = 0, resistenseSendTick = 0, resistenseSendExtendTick = 0, trainingStartTick = 0;
    TickType_t            tickOn = xTaskGetTickCount() + 60 * 1000;
    extern struct EmsChannelsStatus        emsChannelsStatus;
    uint32_t              channelError, channelErrorOld = 0;
    SmEmsStageTypeDef     smOldEmsStg = SM_EMS_STG_OFF;
    struct CmdCfgTypeDef  cmdCfg;
    CmdParserTypeDef      cmd;
    uint8_t               batProcOld = 0;
    uint32_t              parserUint = 0;
    uint32_t              oldCostumeId = 0;
    uint8_t               param[16];
    uint32_t              stimulateTime_ms = 0;
    uint32_t              relaxTime_ms = 0;
    uint8_t               electrodesReset = 0;
    buildNumber = Esp32GetBuildTimestamp();

    smBle = SM_BLE_READY;
    //иницализация уникального номера
    Esp32IdInit();  

    // инициализация считывателя номера костюмной платы
    owInit();

    while (1) {
        if(tickOn <= xTaskGetTickCount())
        {
            tickOn += 60 * 1000;
        }

        while((cmd = getCmd()) != PARSE_NO)
        {
            if(pingTick > 0)
            {
                pingTick = xTaskGetTickCount();
            }
            
            // если приходит команда не PARSE_PING продлеваем время простоя
            if(cmd != PARSE_PING && cmd != PARSE_ACCEPTED)
            {
                delayTick = xTaskGetTickCount();
            }
            
            switch(cmd)
            {
                case PARSE_CONNECT:
                    break;
                    
                case PARSE_DISCONN:
                    break;
                    
                // принимаем ping
                case PARSE_PING:
                    if(smBle != SM_BLE_CONNECT)
                        smBle = SM_BLE_CONNECT;
                    // ацепптед
                    sendAccepted((const uint8_t*)"");
                    // отправляем статус
                    if(smEms == SM_EMS_START || smEms == SM_EMS_PAUSE_END)
                    {
                        sendCmdAndParam((const uint8_t*)"emsState",   (const uint8_t*)"start",       ACCEPTED);
                    }
                    else
                    {
                        sendCmdAndParam((const uint8_t*)"emsState",   (const uint8_t*)"stop",       ACCEPTED);
                    }
                    // проверяем первый ли это пинг, если первый то отправляем дополнительную информацию
                    if(pingTick == 0)
                    {
                        // при коннекте обнуляем время простоя 
                        delayTick = xTaskGetTickCount();
                        // первый пинг
                        pingTick = xTaskGetTickCount();
                        // отрпавляем режим и тип протокола
                        sendCmdAndParam((const uint8_t*)"protocol",   (const uint8_t*)"text,0",       ACCEPTED);
                        sendVersion();
                        sendCmdAndParam((const uint8_t*)"mode",       (const uint8_t*)"normal",       ACCEPTED);
                        
                        // отрпавляем id блока и костюма
                        sendCmdAndParam((const uint8_t*)"id", Esp32Id(), ACCEPTED);
                        sprintf((char *)costumeIdText, "%lX", costumeId);
                        sendCmdAndParam((const uint8_t*)"costumeId", costumeIdText, ACCEPTED);
                        
                        // значение аккумулятора
                        sendAccProc(batProc);
                        batProcOld = batProc;
                        
                        // если не сконфигурирован блок то говорим об этом
                        if(!emsConfigureState())
                        {
                            sendCmd((const uint8_t*)"configNo", ACCEPTED);
                        }
                    }
                    pingCnt++;
                    break;
                // Установить время
                case PARSE_TIME:
                    if(getInt32Value(&parserUint, 0) == CMD_OK)
                    {
                        sendAccepted((const uint8_t*)"");
                        setTime(parserUint);
                    }
                    else
                    {
                        sendNotAccepted((const uint8_t*)"");
                    }
                    break;
                    
                // Отключаем аццептеты
                case PARSE_IMPULSE_MSG:
                    if(getCharValue(param, 8, 0) == CMD_OK)
                    {        
                        sendAccepted((const uint8_t*)"");
                        if(!strcmp((char *) param, "true"))
                        {
                            impulseMsgEnabled = true;
                        }
                        else
                        {
                            impulseMsgEnabled = false;
                        }
                    }
                    sendNotAccepted((const uint8_t*)"");
                    break;        
                    
                // Отложенный запуск
                case PARSE_START_AT:
                    if(getInt32Value(&parserUint, 0) == CMD_OK)
                    {
                    isAfterStart = true;

                    if(isSetTime() == true && emsConfigureState() &&  smEms != SM_EMS_START)
                    {
                        emsCfgGet(CFG_STIMUL_TIME_MS, &stimulateTime_ms);
                        emsCfgGet(CFG_RELAX_TIME_MS, &relaxTime_ms);
                        
                        if((getTime() > parserUint))
                        {
                        startTime = parserUint;

                        if(stimulateTime_ms > 0)
                        {
                            for(;;)
                            {
                            startTime += ((stimulateTime_ms + relaxTime_ms));
                            if(startTime > getTime() && ((getTime() - startTime) > 5000))
                                break;
                            }
                        }
                        }
                        else
                        {
                        startTime = parserUint;
                        }
                    }
                    sendAccepted((const uint8_t*)"");
                    }
                    else
                    {
                    sendNotAccepted((const uint8_t*)"");
                    }
                    break;

                // Обрабатываем команду
                case PARSE_EMS_CMD:
                    // проверяем задана ли конфигурация
                    
                    // проверяем выставлены ли проценты на 0
                    
                    // только потом запускаем
                    switch(getCmdEmsParam())
                    {
                    case CMD_EMS_START:
                    isLoadLastInit = 0;
                    if(!emsConfigureState())
                    {
                        sendNotAccepted((const uint8_t*)"");
                        sendCmd((const uint8_t*)"configNo", ACCEPTED);
                        break;
                    }
                    else
                    {
                        trainingStartTick = xTaskGetTickCount();
                        emsStart();
                        sendAccepted((const uint8_t*)"");

            #ifdef __DEBUG_MODE__
                        sendCmdAndParam("debug", "Start", ACCEPTED_NO);
            #endif

                        sprintf((char *)str, "batProc %u", batProc);
                        sendCmdAndParam((const uint8_t*)"info", str, ACCEPTED_NO);
                        
                        // отправляем версию hardware
                        switch(hardware)
                        {
                        case HARDWARE_G3:
                        sendCmdAndParam((const uint8_t*)"info", (const uint8_t*)"hw G3", ACCEPTED);
                        break;
                        
                        case HARDWARE_G4:
                        sendCmdAndParam((const uint8_t*)"info", (const uint8_t*)"hw G4", ACCEPTED);
                        break;
                        
                        default:
                        sendCmdAndParam((const uint8_t*)"info", (const uint8_t*)"hw NA", ACCEPTED);
                        }
                        
                        // билд намбер

                        const esp_app_desc_t *app_desc = esp_app_get_description();
                        sprintf((char *)str, "FW %s Build %u", app_desc->version, (uint8_t)buildNumber);
                        sendCmdAndParam((const uint8_t*)"info", str, ACCEPTED_NO);

                        // датчик света
                        //OPT3001_Convert(&opt3001);
                        //sprintf((char *)str, "Lux %d", opt3001.Lux);
                        sprintf((char *)str, "Lux %d", 0);
                        sendCmdAndParam((const uint8_t*)"info", str, ACCEPTED);
                        
                        // возожно это нужно удалить
                        impulseMsgEnabled = true;
                        sendCmdAndParam((const uint8_t*)"emsElectrodeFails", (const uint8_t*)"0,0,0,0,0,0,0,0,0,0,0,0", ACCEPTED_NO);
                        
                        // состояние настроек импульсов
                        if(electrodesReset == 1){
                        sprintf((char *)str, "electrodesReset");
                        sendCmd(str, ACCEPTED);
                        electrodesReset = 0;
                        }              
                                        
                        if(blinkOn == true)
                        {
                        blinkOn = false;
                        }

                        break;
                    }
                    
                    case CMD_EMS_STOP:
                        emsStop();
                        sprintf((char *)str, "batProc %u", batProc);
                        sendCmdAndParam((const uint8_t*)"info", str, ACCEPTED_NO);
                        isAfterStart = false;
                        sendAccepted((const uint8_t*)"");
                    break;
                    
                    case CMD_EMS_PAUSE:
                        emsPause();
                        sprintf((char *)str, "batProc %u", batProc);
                        sendCmdAndParam((const uint8_t*)"info", str, ACCEPTED_NO);
                        isAfterStart = false;
                        sendAccepted((const uint8_t*)"");

                        sprintf((char *)str, "Electrod1 %u %u %u %u %u %u %u %u %u %u %u %u", \
                                emsPercentCfg[0].channelPercent[0], \
                                emsPercentCfg[0].channelPercent[1], \
                                emsPercentCfg[0].channelPercent[2], \
                                emsPercentCfg[0].channelPercent[3], \
                                emsPercentCfg[0].channelPercent[4], \
                                emsPercentCfg[0].channelPercent[5], \
                                emsPercentCfg[0].channelPercent[6], \
                                emsPercentCfg[0].channelPercent[7], \
                                emsPercentCfg[0].channelPercent[8], \
                                emsPercentCfg[0].channelPercent[9], \
                                emsPercentCfg[0].channelPercent[10], \
                                emsPercentCfg[0].channelPercent[11]);
                        sendCmdAndParam((const uint8_t*)"info", str, ACCEPTED_NO);
                        
                        sprintf((char *)str, "Electrod2 %u %u %u %u %u %u %u %u %u %u %u %u", \
                                emsPercentCfg[1].channelPercent[0], \
                                emsPercentCfg[1].channelPercent[1], \
                                emsPercentCfg[1].channelPercent[2], \
                                emsPercentCfg[1].channelPercent[3], \
                                emsPercentCfg[1].channelPercent[4], \
                                emsPercentCfg[1].channelPercent[5], \
                                emsPercentCfg[1].channelPercent[6], \
                                emsPercentCfg[1].channelPercent[7], \
                                emsPercentCfg[1].channelPercent[8], \
                                emsPercentCfg[1].channelPercent[9], \
                                emsPercentCfg[1].channelPercent[10], \
                                emsPercentCfg[1].channelPercent[11]);
                        sendCmdAndParam((const uint8_t*)"info", str, ACCEPTED_NO);
                    break;
                    
                    case CMD_EMS_ERROR:
                        default:
                    break;
                    }
                    break;
                    
                    
                // устанавливаем ток
                    case PARSE_CURRS:
                    if(smEms == SM_EMS_START || smEms == SM_EMS_PAUSE_END)
                    {
                        if(setCurrents() == CMD_OK)
                        sendAccepted((const uint8_t*)"");
                        else
                        sendNotAccepted((const uint8_t*)"");
                        
                        if(setup_error)
                        {
                        setup_error = 0;
                        sprintf((char *)str, "Cur error %u %u %u", (uint8_t)setup_channel_error, setup_old_value_error, setup_value_error);
                        sendCmdAndParam((const uint8_t*)"info", str, ACCEPTED_NO);
                        }
                    }

                    // надо сделать чтобы при паузе можно было уменьшать силу
                    else
                    {
                        if(setCurrents() == CMD_OK)
                        sendAccepted((const uint8_t*)"");
                        else
                        sendNotAccepted((const uint8_t*)"");
                    }
                    break;
                    
                case PARSE_CURRS_X:
                    if(setCurrentsX() == CMD_OK)
                    sendCurrentsX();
                    else
                    sendNotAccepted((const uint8_t*)"");
                    
                    break;


                // устанавливаем конфигурацию
                case PARSE_CONF:
                    printf("PARSE_CONF\n");
                    // не работает проверка на допустимый диапазон!!!!! сделать срочно!!!!!
                    if(getConfig(&cmdCfg) == CMD_OK)
                    {
                    if(setConfig(&cmdCfg) == CMD_ERROR)
                    {
                        sendNotAccepted((const uint8_t*)"");
                    }
                    sendAccepted((const uint8_t*)"");
                    }
                    else
                    sendNotAccepted((const uint8_t*)"");
                    break;
                    
                // устанавливаем конфигурацию 
                case PARSE_CONF_TIME:
                    sendAccepted((const uint8_t*)"");
                    sendNotAccepted((const uint8_t*)"");
                    break;
                    
                case PARSE_BLINK:
                    if(getCharValue(param, 8, 0) == CMD_OK)
                    {
                    sendAccepted((const uint8_t*)"");

                    if(!strcmp((char *) param, "on"))
                    {
                        blinkOn = true;
                    }
                    else
                    {
                        blinkOn = false;
                    }
                    }
                    else
                    {
                    sendNotAccepted((const uint8_t*)"");
                    }
                    break;

                    
                // задаем имя модулю
                case PARSE_SET_NAME:
                    if(getInt32Value(&parserUint, SETNAME_PARAM_NUM) == CMD_OK)
                    {
                    sendAccepted((const uint8_t*)"");
                    new_ble_num = parserUint;
                    set_ble_name = true;
                    //vTaskDelay(100);
                    //setFFitNumber((uint8_t)parserUint);

                    
                    }
                    else
                    {
                    sendNotAccepted((const uint8_t*)"");
                    }
                    break;
                    
                    
                // возвращаем id блока  
                case PARSE_GET_ID:
                    sendCmdAndParam((const uint8_t*)"id", Esp32Id(), ACCEPTED_NO);
                    break;
                    
                    
                // возвращаем id костюма  
                case PARSE_GET_COSTUME_ID:
                    sprintf((char *)costumeIdText, "%lX", costumeId);
                    sendCmdAndParam((const uint8_t*)"costumeId", costumeIdText, ACCEPTED);
                    break;

                    
                // возвращаем mode  
                case PARSE_GET_MODE:
                    sendCmdAndParam((const uint8_t*)"mode", (const uint8_t*)"normal", ACCEPTED);
                    break;
                    
                    
                // возвращаем id костюма  costumeId:0(none)/null/guid
                case PARSE_COSTUME_PRESENT:
                    if(costumeId == 0)
                    {
                    sprintf((char *)costumeIdText, "none");
                    }
                    else
                    {
                    sprintf((char *)costumeIdText, "%lX", costumeId);
                    }
                    sendCmdAndParam((const uint8_t*)"costumeId", costumeIdText, ACCEPTED);
                    break;        
                    
                    
                // возвращаем заряд аккумулятора в процентах  
                case PARSE_GET_ACCUM:
                    sendAccProc(batProc);
                    break;
                    

                // возвращаем версию прошивки  
                case PARSE_GET_VERSION:
                    sendVersion();
                    break;
                    
                
                // напряжение на аккумуляторах
                case PARSE_GET_VOLTS:
                    sprintf((char *)str, "%u,%u,%u,%u", (uint8_t)maVbat1.value,
                            (uint8_t)(maVbats.value - maVbat1.value),
                            (uint8_t)maVref.value,
                            (uint8_t)maHvcc.value);
                    sendCmdAndParam((const uint8_t*)"volts", str, ACCEPTED_NO);
                    break;

                    
                // получить LUX
                case PARSE_GET_LUX:
                    if(hardware == HARDWARE_G4)
                    {
                    //OPT3001_Convert(&opt3001);
                    //sprintf((char *)str, "%u", opt3001.Lux);
                    sprintf((char *)str, "%u", 0);
                    sendCmdAndParam((const uint8_t*)"lux", str, ACCEPTED_NO);
                    }
                    break;

                case PARSE_EFFECT:
                    if(getInt32Value(&parserUint, 0) == CMD_OK)
                    {
                    EffectType effect = (EffectType)parserUint;

                    if(effect >= EFFECT_CNT)
                    {
                        sendNotAccepted((const uint8_t*)"");
                        break;
                    }

                    if(effect == EFFECT_NONE)
                    {
                        effectsStop();
                    }
                    else
                    {
                        effectsStart(effect);
                    }

                    sendAccepted((const uint8_t*)"");
                    }
                    else
                    {
                    sendNotAccepted((const uint8_t*)"");
                    }
                    break;
                    
                // выключаение
                case PARSE_SHUTDOWN:
                    pwrOff();
                    break;
                    
                // неизвестная команда
                // нужен счетчик неизвестных комманд, чтобы понимать как часто это происходит
                default:
                    sendNotAccepted((const uint8_t*)"");
                    break;
            }
        }
        // проверка на отложенный запуск
        if( isAfterStart == true )
        {
            if((isSetTime() == true) && (getTime() >= startTime) && \
                emsConfigureState()  && (smEms != SM_EMS_START))
            {
                    isAfterStart = false;
                    trainingStartTick = xTaskGetTickCount();
                    emsStart();
                    sendAccepted((const uint8_t*)"");

                    sendCmdAndParam((const uint8_t*)"debug", (const uint8_t*)"AfterStart", ACCEPTED_NO);

        #ifdef __DEBUG_MODE__
                    sendCmdAndParam("debug", "Start", ACCEPTED_NO);
        #endif

                    // отправляем версию hardware
                    switch(hardware)
                    {
                    case HARDWARE_G3:
                    sendCmdAndParam((const uint8_t*)"info", (const uint8_t*)"hw G3", ACCEPTED);
                    break;
                    
                    case HARDWARE_G4:
                    sendCmdAndParam((const uint8_t*)"info", (const uint8_t*)"hw G4", ACCEPTED);
                    break;

                    case HARDWARE_ESP32:
                    sendCmdAndParam((const uint8_t*)"info", (const uint8_t*)"hw EP", ACCEPTED);
                    break;
                    
                    default:
                    sendCmdAndParam((const uint8_t*)"info", (const uint8_t*)"hw NA", ACCEPTED);
                    }
                    
                    // возожно это нужно удалить
                    sendCmdAndParam((const uint8_t*)"emsElectrodeFails", (const uint8_t*)"0,0,0,0,0,0,0,0,0,0,0,0", ACCEPTED_NO);
                    
                    // состояние настроек импульсов
                    if(electrodesReset == 1){
                    sprintf((char *)str, "electrodesReset");
                    sendCmd(str, ACCEPTED);
                    electrodesReset = 0;
                    }        
                    
                    if(blinkOn == true)
                    {
                    blinkOn = false;
                    }
            }
        }
        
        // измеряем сопротивление электродов
        if(((xTaskGetTickCount() - resistenseTick) > RESISTANCE_PERIOD_MS) &&  smEms == SM_EMS_START)
        {
            resistenseTick = xTaskGetTickCount();
            
            for(uint8_t idx = 0; idx < MUX_CHANNELS; idx++)
            {
                maCalc(&pulseCurrent[idx]);
                maCalc(&pulseVoltage[idx]);
                
                chanelLoad[idx].mv =    (uint16_t)(pulseVoltage[idx].value * 15.311);
                chanelLoad[idx].ma =    (uint16_t)(pulseCurrent[idx].value / 12.409);
                if(chanelLoad[idx].ma < 5 || chanelLoad[idx].ohm > 999999)
                    chanelLoad[idx].ohm = 999999;
                else
                    chanelLoad[idx].ohm =   (uint16_t)((chanelLoad[idx].mv / chanelLoad[idx].ma) > 50) ? ((uint16_t)(chanelLoad[idx].mv / chanelLoad[idx].ma)) - 50 : 0;
                chanelLoad[idx].mw = chanelLoad[idx].mv * chanelLoad[idx].ma / 1000;
                

            }
        }
        // отправляем сопротивление электродов
        if(((xTaskGetTickCount() - resistenseSendTick) > RESISTANCE_SEND_PERIOD_MS) &&  smEms == SM_EMS_START && (xTaskGetTickCount() - trainingStartTick) > 30 * 1000 )
        {
            resistenseSendTick = xTaskGetTickCount();
            resistenseSendExtendTick = xTaskGetTickCount();
            sprintf((char *)str, "batProc %u", batProc );
            sendCmdAndParam((const uint8_t*)"info", str, ACCEPTED_NO);
            
            uint8_t curValue;
            
            for(uint8_t idx = 0; idx < MUX_CHANNELS_; idx++)
            {
                if(emsPercentGet((MuxChannelTypeDef)muxSeries.channel[idx], &curValue) == CFG_GET_OK)
                {
                    if(curValue >= 5)
                    {
                    if(chanelLoad[idx].ohm >= 99999)
                    {
                        sprintf((char *)str, "%u,E", idx);
                    }
                    else
                    {
                        sprintf((char *)str, "%u,%u",
                                (uint8_t)idx,
                                (uint8_t)chanelLoad[idx].ohm
                        );                
                    }
                    }
                    else {
                    sprintf((char *)str, "%u,0",
                            (uint8_t)idx
                    );
                    }
                    
                    sendCmdAndParam((const uint8_t*)"resist", str, ACCEPTED_NO);
                }
            }
            // отправляем количество верных и ошибочных комманд
            sprintf((char *)str, "Cmd errors %u", (uint8_t)countCommandError);
            sendCmdAndParam((const uint8_t*)"info", str, ACCEPTED_NO);
            sprintf((char *)str, "Cmd ok %u", (uint8_t)countCommandOk);
            sendCmdAndParam((const uint8_t*)"info", str, ACCEPTED_NO);
        }
        else if(((xTaskGetTickCount() - resistenseSendExtendTick) > 5000) &&  (smEms == SM_EMS_START))
        {
            uint32_t delta_ohm;
            resistenseSendExtendTick = xTaskGetTickCount();
            
            if(isLoadLastInit == 0)
            {
                for(uint8_t idx = 0; idx < MUX_CHANNELS; idx++)
                { 
                chanelLoadLast[idx].mv = 0;
                chanelLoadLast[idx].ma = 0;
                chanelLoadLast[idx].ohm = 0;
                chanelLoadLast[idx].mw = 0;       
                }
                isLoadLastInit = 1;
            }
            for(uint8_t idx = 0; idx < MUX_CHANNELS_; idx++)
            {
                if(chanelLoad[idx].ohm >= chanelLoadLast[idx].ohm)
                delta_ohm = chanelLoad[idx].ohm - chanelLoadLast[idx].ohm;
                else
                delta_ohm = chanelLoadLast[idx].ohm - chanelLoad[idx].ohm ;
                
                if(delta_ohm >= 50)
                {
                chanelLoadLast[idx] = chanelLoad[idx];
                if(chanelLoad[idx].ohm >= 999999)
                {
                    sprintf((char *)str, "%u,E", idx);
                }
                else
                {
                    sprintf((char *)str, "%u,%u",
                            (uint8_t)idx,
                            (uint8_t)chanelLoad[idx].ohm
                    );              
                }
                
                sendCmdAndParam((const uint8_t*)"resist", str, ACCEPTED_NO);
                }
            }
        }
        
    #ifdef __DEBUG_MODE__
        if(emsPulseMessage.pulseError != PULSE_OK)
        {
        if(smEms == SM_EMS_START && emsPulseMessage.pulseError == PULSE_OK)
        {
            sprintf((char *)str, "pulseError cnt.%u ch.%u set.%u cur.%u", emsPulseMessage.count, emsPulseMessage.channel, emsPulseMessage.setValue, emsPulseMessage.currentValue);
        }

        else if(emsPulseMessage.pulseError != PULSE_OK)
        {
            sprintf((char *)str, "MuxError cnt.%u ch.%u", emsPulseMessage.count, emsPulseMessage.channel);
        }
        emsPulseMessage.pulseError = PULSE_OK;
        sendCmdAndParam("debug", str, ACCEPTED_NO);
        }
    #endif

        // проверяем на простой блока
        if(smEms == SM_EMS_START)
        {
            delayTick = xTaskGetTickCount();
        }
        else
        {
            if((xTaskGetTickCount() - delayTick) > DELAY_OFF_MS)
            {
                // тут нужно добавить сообщеине о выключении блока
                sendCmdAndParam((const uint8_t*)"info", (const uint8_t*)"timeoutOFF", ACCEPTED_NO);
                vTaskDelay(500);
                for(;;)
                {
                pwrOff();
                }
            }
        }
        
        // отправляем напряжения
        if( ((xTaskGetTickCount() - voltsTick) > VOLTS_SEND_PERIOD_MS) &&  smEms == SM_EMS_START) 
        {
            voltsTick = xTaskGetTickCount();
            sprintf((char *)str, "%u,%u,%u,%u", (uint8_t)maVbat1.value,
                    (uint8_t)(maVbats.value - maVbat1.value),
                    (uint8_t)maVref.value,
                    (uint8_t)maHvcc.value);
            sendCmdAndParam((const uint8_t*)"volts", str, ACCEPTED_NO);
        }
        
        // проверяем наличие подключения
        if(pingTick != 0 && (xTaskGetTickCount() - pingTick) > PING_MAX_PERIOD_MS)
        {
    //      emsStop();
    //      electrodesReset = 1;

        emsPause();
        
        sendEror((uint8_t*)"ping");
        smBle = SM_BLE_DISCONNECT;
        
        pingTick = 0;
        pingCnt = 0;
        
        if(blinkOn == true)
        {
            blinkOn = false;
        }
        }
        
        if(smBle == SM_BLE_CONNECT && impulseMsgEnabled == true)
        {
        // отправляем начало стимуляции и релаксации
        if((smEmsStg == SM_EMS_STG_STIMULATE || smEmsStg == SM_EMS_STG_RELAX) && smEmsStg != smOldEmsStg)
        {
            smOldEmsStg = smEmsStg;
            if(smEmsStg == SM_EMS_STG_STIMULATE)
            uartWrite((const uint8_t*)"emsStimul\r");
            else
            uartWrite((const uint8_t*)"emsRelax\r");
        }
        }
        
        // error
        if(smEms == SM_EMS_START && ((xTaskGetTickCount() - failsTick) > FAILS_PERIOD_MS))
        {
        channelError = 0;
        sendElectrodStatus();

        for(uint8_t idx = 0; idx < 12; idx++)
        {
            if (!(uint8_t)emsChannelsStatus.channel[idx])
            {
            channelError |= (1 << idx);
            }
        }

        if(channelError != channelErrorOld)
        {
            for(uint8_t idx = 0; idx < 12; idx++)
            {
            if (!(uint8_t)emsChannelsStatus.channel[idx])
            {
                sprintf((char *)str, "resistance ch.%u ohm.%u",
                        (uint8_t)idx,
                        (uint8_t)chanelLoad[idx].ohm);
            }
            }
        }

        channelErrorOld = channelError;
        failsTick = xTaskGetTickCount();
        }
        
        // проверяем изменение значаения аккумулятора, если изменилось отправляем планшету
        if (batProcOld != batProc)
        {
        batProcOld = batProc;
        sendAccProc(batProc);
        }
        
        // считываем id костюма
        if((xTaskGetTickCount() - costumeTick) > COSTUME_ID_PERIOD_MS)
        {
            costumeTick = xTaskGetTickCount();
            costumeId = 0;
            
            if(owReset() == OW_STATE_PRESENT)
            {
                for(uint8_t owidx = 0; owidx <3; owidx++)
                {
                if(owReset() == OW_STATE_PRESENT)
                {
                    owSetPinOpenDrain();
                    owCmdReadRom();
                    vTaskDelay(25);
                    owSetPinPushPull();
                    vTaskDelay(25);
                    costumeIds[owidx] = owGetRom();
                }
                }
                
                if(costumeIds[0] == costumeIds[1] && costumeIds[1] == costumeIds[2])
                {
                costumeId = costumeIds[0];
                }
            }

            if(costumeId != oldCostumeId)
            {
                oldCostumeId = costumeId;
                sprintf((char *)costumeIdText, "%lX", costumeId);
                sendCmdAndParam((const uint8_t*)"costumeId", costumeIdText, ACCEPTED);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(CMD_TASK_DELAY_MS));
    }
}

