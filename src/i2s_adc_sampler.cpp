#include "i2s_adc_sampler.hpp"

ADCSampler *adc_sampler = NULL;

// i2s config for using the internal ADC
i2s_config_t adcI2SConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_LSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

void send_data(int *param)
{
  // send data off to the bluetooth
}

// Task to write samples from ADC to our server
void adc_writer_task(void *param)
{
    I2SSampler *sampler = (I2SSampler *)param;
    int16_t *samples = (int16_t *)malloc(sizeof(uint16_t) * SAMPLE_SIZE);
    if (!samples)
    {
        vTaskDelete(NULL);
    }
    while (true)
    {
        int samples_read = sampler->read(samples, SAMPLE_SIZE);

        send_data(&samples_read);
    }
    // Should never reach this but just in case...
    vTaskDelete(NULL);
}


void adc_i2s_init(){
    // input from analog signal
    // internal analog to digital converter sampling using i2s
    // create our samplers
    adc_sampler = new ADCSampler(ADC_UNIT_1, ADC1_CHANNEL_7, adcI2SConfig);

    // set up the adc sample writer task
    TaskHandle_t adc_writer_task_handle;
    adc_sampler->start();
    xTaskCreate(adc_writer_task, "ADC Writer Task", 4096, adc_sampler, 1, &adc_writer_task_handle);
}