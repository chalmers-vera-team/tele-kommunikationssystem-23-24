#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <Arduino.h>
#include "hfp_ag_client.h"
#include "i2s_adc_sampler.hpp"
#include "sdkconfig.h"

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


// Task for our GSM module that will be added to tasks for rtos to handle
// rtos (real time operating system) allows tasks to be run at different prioritization levels on the 2 cores of the ESP32.
// freeRTOS is the simplified version of RTOS that is used in this project.
void gsm_modem_task(void *parameter)
{
    // Calling line identification presentation
    SerialAT.print("AT+CLIP=1\r\n");
    vTaskDelay(10000 / portTICK_PERIOD_MS);

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
    while (true) {
        modem.maintain();
    }
    vTaskDelete( NULL );
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


// setup() and loop() run in their own task with priority 1 in core 1 on ESP32 arduino
void setup() {
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

    vTaskDelay(6000 / portTICK_PERIOD_MS);

    // Create the gsm modem init task with priority 1 and stack size 2048 bytes.
    // Initializes the modem and creates the gsm_modem_task Task that handles a caller.
    gsm_modem_init();

    // Initialize ADC with I2S to write data from ADC DMA buffer to 
    // bluedroids RingBuffer (which sends it to connected bluetooth client).
    adc_i2s_init();

    // Initialize the bluedroid unit and make the audio gateway ready to start sending data.
    hfp_ag_init();

    // Create the gsm modem task with priority 0 and stack size 8192 bytes after init is completed. 
    // Priority 0 is to prevent watchdog from barking, task running when nothing else is running.
    xTaskCreate(gsm_modem_task, "gsm_modem_task", 8192, NULL, 0, NULL);
}

// Super loop is not utilized, using freeRTOS instead.
void loop(){}



/* Notes to self:

- In bt_app_hf.c, row 314, there is code that starts the audio task event. Try looking at that to implement sending data to bluetooth.
- In bt_app_hf.c, row 111, there is code for the sine_int[] variable. Contains sound data for a constant sine function that can be 
  played to bluetooth client. Look here if you get lost and can't find the functions.

*/
