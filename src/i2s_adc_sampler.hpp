#include "i2s_sampler/ADCSampler.h"

// Settings for DMA
const int SAMPLE_RATE = 16000;
const int DMA_BUF_COUNT = 4;
const int DMA_BUF_LEN = 1024;

// Samples to send over bluetooth at once.
const int SAMPLE_SIZE = 16384;

/*  @brief Function to send data to the bluetooth device, should not be used. Use adcWriterTask instead to send the data.
*/  
void send_data();

/*  @brief Task to write adc data from DMA buffer to bluetooth device, should not be used.
*/
void adc_writer_task(void *param);

/*  @brief Function to initialize the ADC with i2s. This also creates the adc_writer_task Task for freeRTOS to handle. Run this as a task to start the ADC DMA sampling to the bluetooth device.
*/
void adc_i2s_init();
