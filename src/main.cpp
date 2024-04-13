extern "C" {
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "sdkconfig.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "bt_app_core.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_ag_api.h"
#include "bt_app_hf.h"
#include "gpio_pcm_config.h"
#include "esp_console.h"
#include "app_hf_msg_set.h"
#include "app_hf_msg_prs.h"
}
#include <Arduino.h>
#include "i2s_adc_sampler.hpp"

// Our T-CALL V1.4 Development Board
#define SIM800H_IP5306_VERSION_20160721

// Contains pinouts and regulator setup functions
#include "utilities.h"

// Used for debugging
#define DUMP_AT_COMMANDS
#define TINY_GSM_DEBUG          SerialMon

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to the gsm module)
#define SerialAT  Serial1

#define TINY_GSM_MODEM_SIM800          // Modem is SIM800H
#define TINY_GSM_RX_BUFFER      1024   // Set RX buffer to 1Kb

#include <TinyGsmClient.h>

// StreamDebugger echoes commands from gsm module to our serial terminal (GSM-> ESP32-> Our computer)
// Used for debuggning
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// Set phone number to call, not currently used. Call the gsm module instead (to prevent cost for the vera sim card)
#define CALL_TARGET "+46705605783"
// NOTE, vera sim call number: +46732074304
#define BT_HF_AG_TAG    "HF_AG_MAIN"

// Task for our GSM module that will be added to tasks for rtos to handle
// rtos (real time operating system) allows tasks to be run at different prioritization levels on the 2 cores of the ESP32.
// freeRTOS is the simplified version of RTOS that is used in this project.
void gsm_modem_task(void *parameter)
{
    // Calling line identification presentation
    SerialAT.print("AT+CLIP=1\r\n");
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    for(;;){
        bool res = false;

        // Start waiting for someone to call...
        while(!res){
            SerialMon.print("Waiting for caller...");
            SerialMon.println();

            String data;
            while(SerialAT.available())
            {
                char x = SerialAT.read();
                if(x == '\n' || x == '\r'){
                    if(data == "RING"){ // Looking for a caller notification from the GSM module
                        res = modem.callAnswer();
                        break;
                    }
                    data = "";
                }
                else{
                    data += x;
                }
            }

            if (SerialMon.available()){
                if (SerialMon.readString() == "Stop"){ // Stop waiting for caller
                    SerialMon.println("Not waiting for caller.");
                    break;
                }
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        SerialMon.print("Answered:");
        SerialMon.println(res ? "OK" : "fail");

        // If we connected to a caller, wait to hang up
        if (res){
            bool hangup = false;
            while(hangup == false){
                if (SerialMon.available()){
                    if (SerialMon.readString() == "Stop"){
                        SerialMon.print("Ending call.");
                        hangup = true;
                    }
                }
            }
            res = modem.callHangup();
            DBG("Hang up:", res ? "OK" : "fail");
        }


        // Do nothing forevermore
        SerialMon.print("Reached end of loop");
    }
}


void gsm_modem_init()
{
    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    SerialMon.println("Initializing modem...");
    modem.restart();

    // Swap the audio channels
    SerialAT.print("AT+CHFA=1\r\n");
    vTaskDelay(2 / portTICK_PERIOD_MS);

    //Set ringer sound level
    SerialAT.print("AT+CRSL=10\r\n");
    vTaskDelay(2 / portTICK_PERIOD_MS);

    //Set loud speaker volume level
    SerialAT.print("AT+CLVL=0\r\n");
    vTaskDelay(2 / portTICK_PERIOD_MS);
}


void arduino_setup(void)
{
    // Set console baud rate
    SerialMon.begin(115200);

    vTaskDelay(10 / portTICK_PERIOD_MS);

    // Start power management
    if (setupPMU() == false) {
        SerialMon.println("Setting power error");
    }

    // Some start operations
    setupModem();

    // Set GSM module baud rate and UART pins
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Initialize ADC with I2S to write data from ADC DMA buffer to 
    // bluedroids RingBuffer (which sends it to connected bluetooth client).
    adc_i2s_init();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Create the gsm modem init task with priority 1 and stack size 2048 bytes.
    // Initializes the modem and creates the gsm_modem_task Task that handles a caller.
    gsm_modem_init();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Create the gsm modem task with priority 0 and stack size 8192 bytes after init is completed. 
    // Priority 0 is to prevent watchdog from barking, task running when nothing else is running.
    xTaskCreate(gsm_modem_task, "gsm_modem_task", 8192, NULL, 0, NULL); 
}


extern "C" {
    enum {
        BT_APP_EVT_STACK_UP = 0,
    };

    static void bt_hf_hdl_stack_evt(uint16_t event, void *p_param)
    {
        ESP_LOGD(BT_HF_TAG, "%s evt %d", __func__, event);
        switch (event)
        {
            case BT_APP_EVT_STACK_UP:
            {
                /* set up device name */
                char *dev_name = "ESP_HFP_AG";
                esp_bt_dev_set_device_name(dev_name);

                esp_bt_hf_register_callback(bt_app_hf_cb);

                // init and register for HFP_AG functions
                esp_bt_hf_init(hf_peer_addr);

                /*
                * Set default parameters for Legacy Pairing
                * Use variable pin, input pin code when pairing
                */
                esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
                esp_bt_pin_code_t pin_code;
                pin_code[0] = '0';
                pin_code[1] = '0';
                pin_code[2] = '0';
                pin_code[3] = '0';
                esp_bt_gap_set_pin(pin_type, 4, pin_code);

                /* set discoverable and connectable mode, wait to be connected */
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                break;
            }
            default:
                ESP_LOGE(BT_HF_TAG, "%s unhandled evt %d", __func__, event);
                break;
        }
    }

    void hfp_ag_init(void){
        /* Initialize NVS — it is used to store PHY calibration data */
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

        esp_err_t err;
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        if ((err = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
            ESP_LOGE(BT_HF_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }
        if ((err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
            ESP_LOGE(BT_HF_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }
        if ((err = esp_bluedroid_init()) != ESP_OK) {
            ESP_LOGE(BT_HF_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }
        if ((err = esp_bluedroid_enable()) != ESP_OK) {
            ESP_LOGE(BT_HF_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }

        /* create application task */
        bt_app_task_start_up();

        /* Bluetooth device name, connection mode and profile set up */
        bt_app_work_dispatch(bt_hf_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);

    #if CONFIG_BT_HFP_AUDIO_DATA_PATH_PCM
        /* configure the PCM interface and PINs used */
        app_gpio_pcm_io_cfg();
    #endif

        /* configure externel chip for acoustic echo cancellation */
    #if ACOUSTIC_ECHO_CANCELLATION_ENABLE
        app_gpio_aec_io_cfg();
    #endif /* ACOUSTIC_ECHO_CANCELLATION_ENABLE */
    }

    void app_main()
    {
        // initialize the bluetooth modem in hands free profile, as an audio gateway
        hfp_ag_init();

        // initialize arduino library before we start the tasks
        initArduino();

        // run code in setup() that arduino would normally do.
        arduino_setup();
    }
}




/* Notes to self:

- In bt_app_hf.c, row 314, there is code that starts the audio task event. Try looking at that to implement sending data to bluetooth.
- In bt_app_hf.c, row 111, there is code for the sine_int[] variable. Contains sound data for a constant sine function that can be 
  played to bluetooth client. Look here if you get lost and can't find the functions.

*/
