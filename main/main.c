#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/i2s_std.h"

static const char *TAG = "TAG";

// ADS131A04 SPI Configuration
#define PIN_NUM_MISO        2
#define PIN_NUM_MOSI        7
#define PIN_NUM_SCLK        6
#define PIN_NUM_CS_ADC      17
#define PIN_NUM_CS_DAC      16
#define PIN_NUM_DRDY_ADC    23  // Data ready pin

#define SPI_CLOCK_HZ  16384000  // 16.384 MHz

// Frame size in fixed-frame mode with CRC disabled, 16-bit word length
// Frame = 2 bytes (status) + 5 channels * 2 bytes = 12 bytes total
#define FRAME_SIZE_BYTES        12
#define NUM_CHANNELS            5
#define BYTES_PER_CHANNEL       2

// ADS131A04 Commands
#define ADS131A04_CMD_NULL      0x0000
#define ADS131A04_CMD_RESET     0x0011
#define ADS131A04_CMD_STANDBY   0x0022
#define ADS131A04_CMD_WAKEUP    0x0033
#define ADS131A04_CMD_LOCK      0x0555
#define ADS131A04_CMD_UNLOCK    0x0655

#define I2S_PORT I2S_NUM_0

typedef struct {
    uint16_t status;        // Status word (first 2 bytes)
    int16_t channel_data[NUM_CHANNELS];  // 5 channels of 16-bit data
} ads131a04_frame_t;

static spi_device_handle_t spi_handle_ADC;
static spi_device_handle_t spi_handle_DAC;

/**
 * @brief Initialize SPI interface for ADS131A04
 */
esp_err_t spi_init(void) {
    esp_err_t ret;
    
    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0 //Defaults to 4092 if 0 when DMA enabled
    };
    
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return ret;
    }
    ESP_LOGI("MAIN", "spi_bus_initialize");
    
    
    // Configure SPI device ADC
    spi_device_interface_config_t devcfg_ADC = {
        .clock_speed_hz = SPI_CLOCK_HZ,
        .mode = 1,  // SPI mode 1 (CPOL=0, CPHA=1)
        .spics_io_num = PIN_NUM_CS_ADC,
        .queue_size = 1,
        .duty_cycle_pos = 128
    };
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg_ADC, &spi_handle_ADC);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return ret;
    }
    ESP_LOGI("MAIN", "spi_bus_add_device_ADC");

    // Configure SPI device ADC
    spi_device_interface_config_t devcfg_DAC = {
        .clock_speed_hz = SPI_CLOCK_HZ,
        .mode = 1,  // SPI mode 1 (CPOL=0, CPHA=1)
        .spics_io_num = PIN_NUM_CS_DAC,
        .queue_size = 1,
        .duty_cycle_pos = 128
    };

    ret = spi_bus_add_device(SPI2_HOST, &devcfg_DAC, &spi_handle_DAC);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return ret;
    }
    ESP_LOGI("MAIN", "spi_bus_add_device_DAC");


    // Configure DRDY pin as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_DRDY_ADC),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "ADS131A04 SPI initialized successfully");
    
    // Setup ADC after SPI initialization
    vTaskDelay(pdMS_TO_TICKS(100));
  
    
    return ESP_OK;
}

/**
 * @brief Send 16-bit command
 */
esp_err_t ADC_send_cmd(uint16_t cmd)
{

    uint8_t tx[FRAME_SIZE_BYTES] = {0};
    uint8_t rx[FRAME_SIZE_BYTES];
    
    tx[0] = (cmd >> 8) & 0xFF;
    tx[1] = cmd & 0xFF;
    
    spi_transaction_t t = {
        .length = FRAME_SIZE_BYTES * 8,  // bits
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    gpio_set_level(PIN_NUM_DRDY_ADC, 0);
    
    esp_err_t ret = spi_device_transmit(spi_handle_ADC, &t);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Sent cmd: 0x%04X, Response: 0x%04X", cmd, (rx[0] << 8) | rx[1]);
    }

    gpio_set_level(PIN_NUM_DRDY_ADC, 1);
    return ret;
}

/**
 * @brief Write to register (WREG command)
 */
esp_err_t ADC_write_reg(uint8_t reg_addr, uint8_t data)
{
    // WREG command format: 010a aaaa dddd dddd
    // Bits 15-13: 010 (WREG command)
    // Bits 12-8: register address (5 bits)
    // Bits 7-0: sent data (8 bits)
    uint16_t cmd = 0x4000 | ((reg_addr & 0x1F) << 7) | (data);
    
    ESP_LOGI(TAG, "Writing reg 0x%02X = 0x%04X (cmd: 0x%04X)", reg_addr, data, cmd);
    
    return ADC_send_cmd(cmd);
}


/**
 * @brief Wait for data ready signal
 */
bool ads131a04_wait_drdy(uint32_t timeout_ms) {
    uint32_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
        if (gpio_get_level(PIN_NUM_DRDY_ADC) == 0) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

/**
 * @brief Convert 16-bit two's complement to 32-bit signed integer
 */
static int32_t convert_16bit_to_int32(uint8_t *data) {
    int16_t value = (data[0] << 8) | data[1];
    return (int32_t)value;
}

/**
 * @brief Read ADC frame from ADS131A04
 * 
 * Reads one complete frame in fixed-frame size mode with CRC disabled, 16-bit words.
 * Frame structure (12 bytes):
 * - Bytes 0-1: status word
 * - Bytes 2-3: Channel 0 data (16-bit, MSB first)
 * - Bytes 4-5: Channel 1 data (16-bit, MSB first)
 * - Bytes 6-7: Channel 2 data (16-bit, MSB first)
 * - Bytes 8-9: Channel 3 data (16-bit, MSB first)
 * - Bytes 10-11: empty channel 0x00 (CRC disabled)
 */
esp_err_t ads131a04_read_frame(ads131a04_frame_t *frame) {
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    /*
    // Wait for data ready
    if (!ads131a04_wait_drdy(1000)) {
        ESP_LOGW(TAG, "Timeout waiting for DRDY, pin level: %d", gpio_get_level(PIN_NUM_DRDY_ADC));
        return ESP_ERR_TIMEOUT;
    }*/
    
    // Prepare buffers
    uint8_t tx_data[FRAME_SIZE_BYTES] = {0};
    uint8_t rx_data[FRAME_SIZE_BYTES] = {0};
    
    // Send NULL command (0x0000) to read data
    tx_data[0] = (ADS131A04_CMD_NULL >> 8) & 0xFF;
    tx_data[1] = ADS131A04_CMD_NULL & 0xFF;
    
    // SPI transaction
    spi_transaction_t trans = {
        .length = FRAME_SIZE_BYTES * 8,  // Length in bits
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };
    
    gpio_set_level(PIN_NUM_DRDY_ADC, 0);

    esp_err_t ret = spi_device_transmit(spi_handle_ADC, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmission failed");
        return ret;
    }
    
    gpio_set_level(PIN_NUM_DRDY_ADC, 1);

    // Extract status word (bytes 0-1)
    frame->status = (rx_data[0] << 8) | rx_data[1];
    
    // Extract channel data (16-bit each)
    for (int i = 0; i < 5; i++) {
        int offset = 2 + (i * BYTES_PER_CHANNEL);
        frame->channel_data[i] = convert_16bit_to_int32(&rx_data[offset]);
    }

    return ESP_OK;
}

/**
 * @brief Setup ADC registers
 */
void ADC_setup(ads131a04_frame_t *frame)
{
    ESP_LOGI(TAG, "Starting ADC setup...");

    while(frame->status != 0xFF04){
        ads131a04_read_frame(&frame);
    }

    // Unlock registers
    ADC_send_cmd(ADS131A04_CMD_UNLOCK);
    ESP_LOGI(TAG, "ADC UNLOCK sent");

    if(ads131a04_read_frame(&frame) == 0x0655){
        ESP_LOGI(TAG, "unlock status received");
    }
    
    
    // Configure D_SYS_CFG register (0x0C) - Fixed words frames
    // D_SYS_CFG:
    // Bit 7: watchdog timer enable = 0 (default)
    // Bit 6: CRC_MODE = 0 (default)
    // Bit 5-4: DNDLY = 11 (default)
    // Bit 3-2: HIZDLY = 11 (default)
    // Bit 1: FIXED = 1 (fixed six words data frames)
    // Bit 0: CRC_EN = 0 (disabled, default)
    ADC_write_reg(0x0C, 0x3E);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "ADC Fixed words enable");

    if(ads131a04_read_frame(&frame) == 0x4C3E){
        ESP_LOGI(TAG, "D_SYS_CFG register write status received");
    }


    // CLK1 register: select clock source (0x0D)
    // Bit 7: clk source = 0 (CLKIN)
    // Bit 6-4: reserved = 000
    // Bits 3-1: clkin divider ratio = 100 (default)
    // Bit 0: Reserved = 0
    //ADC_write_reg(0x0D, 0x08);
    //vTaskDelay(pdMS_TO_TICKS(10));
    //ESP_LOGI(TAG, "ICLK is SCLK");


    // Enable all channels
    // ADC_ENA register(0x0F)
    // Bit 7-4: Reserved = 0000
    // Bit 3-0: ENA = 1111 (All ADC channels powered up)
    ADC_write_reg(0x0F, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "ALL CHANNELS POWERED UP");

     if(ads131a04_read_frame(&frame) == 0x4F0F){
        ESP_LOGI(TAG, "Channel enable register write status received");
    }

     // Wake up ADC
    ADC_send_cmd(ADS131A04_CMD_WAKEUP);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "ADC WAKEUP sent");
    
    if(ads131a04_read_frame(&frame) == 0x0033){
        ESP_LOGI(TAG, "WAKEUP status received");
    }


    // Lock registers
    ADC_send_cmd(ADS131A04_CMD_LOCK);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "ADC LOCK sent");

    if(ads131a04_read_frame(&frame) == 0x0555){
        ESP_LOGI(TAG, "LOCK status received");
    }

    ESP_LOGI(TAG, "ADC setup complete");
}



/**
 * @brief Display frame data with all extracted words
 */
void ads131a04_display_frame(const ads131a04_frame_t *frame) {
    printf("\n========== ADS131A04 Frame Data ==========\n");
    

    // Show status word
    printf("Status Word:   0x%04X\n", frame->status);
    

    // Show channel data
    printf("\nChannel Data:\n");
    for (int i = 0; i < 4; i++) {
        printf("  CH%d: %6ld (0x%04lX)\n", 
               i+1, 
               (long)frame->channel_data[i],
               (unsigned long)(frame->channel_data[i] & 0xFFFF));
    }

    // Show CRC data
    printf("  CRC: %6ld (0x%04lX)\n", (long)frame->channel_data[4], (unsigned long)(frame->channel_data[4] & 0xFFFF));

    
    // Show channel data as voltage (VREF = 2.5V, Gain = 1)
    printf("\nChannel Voltages (VREF=2.5V, Gain=1):\n");
    float vref = 2.5;
    for (int i = 0; i < 4; i++) {
        float voltage = (frame->channel_data[i] * vref) / 32768.0;  // 2^15
        printf("  CH%d: %+.6f V\n", i, voltage);
    }
    
    printf("==========================================\n\n");
}

uint16_t voltage_to_dac(float voltage) {
    if (voltage < 0.0f) printf("voltage to dac error: voltage invalide\n");
    if (voltage > 5.0f) printf("voltage to dac error: voltage invalide\n");

    return (uint16_t)((voltage / 5.0f) * 65535.0f);
}

/**
 * @brief Write a 16-bit value to a given channel of DAC8563
 * @param channel 0 for A, 1 for B (depending on how you wire / interpret)
 * @param value 16-bit digital code to send
 */
esp_err_t dac8563_write(uint8_t channel, uint16_t value)
{
    // Build command byte: refer to datasheet, e.g. command for writing to DAC register
    // According to examples, for DAC8563 the command codes might be:
    //   0x18 = write to DAC A
    //   0x19 = write to DAC B :contentReference[oaicite:4]{index=4}
    uint8_t cmd = 0;
    if (channel == 0) {
        cmd = 0x18;
    } else {
        cmd = 0x19;
    }

    uint8_t txbuf[3];
    txbuf[0] = cmd;
    txbuf[1] = (value >> 8) & 0xFF;
    txbuf[2] = value & 0xFF;

    spi_transaction_t t = {
        .length = 8 * 3,  // bits
        .tx_buffer = txbuf,
        .rx_buffer = NULL,
    };

    esp_err_t ret = spi_device_polling_transmit(spi_handle_DAC, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
    }
    return ret;
}


/**
 * @brief Main application
 */
void app_main(void) {
    
    i2s_chan_handle_t tx_handle;

    // Basic TX channel setup
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, &tx_handle, NULL);

    // Standard I²S mode configuration
    i2s_std_slot_config_t slot_cfg =
        I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);

    i2s_std_clk_config_t clk_cfg = {
        .sample_rate_hz = 64000,                // 64 kHz sample rate
        .clk_src = I2S_CLK_SRC_DEFAULT,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256, // 256× → 16.384 MHz MCLK
    };

    i2s_std_config_t std_cfg = {
        .clk_cfg = clk_cfg,
        .slot_cfg = slot_cfg,
        .gpio_cfg = {
            .mclk = GPIO_NUM_0,  // <-- Output your MCLK here
            .bclk = I2S_GPIO_UNUSED,
            .ws   = I2S_GPIO_UNUSED,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_GPIO_UNUSED,
        },
    };

    // Initialize and start I²S
    i2s_channel_init_std_mode(tx_handle, &std_cfg);
    i2s_channel_enable(tx_handle);

    // Now the MCLK pin outputs 16.384 MHz continuously


//------------------------------------------------------------------//

    /*
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_1_BIT,   // 1-bit resolution for square wave
        .freq_hz          = 16384000,           // 16.384 MHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = 0,                   // GPIO0
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 1,                   // max duty for 1-bit resolution
    };
    ledc_channel_config(&ledc_channel);
    */
//------------------------------------------------------------------//

    setvbuf(stdin,  NULL, _IOLBF, 0);
    setvbuf(stdout, NULL, _IOLBF, 0);
    
    esp_err_t ret;
    ads131a04_frame_t frame;
    
    ESP_LOGI(TAG, "Starting ADS131A04 test...");
    
    // Initialize spi
    ret = spi_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Initialization failed");
        return;
    }

    ADC_setup(&frame);

    char input[32];
    float value;
    float value_A;
    float value_B;
    
    // Wait for device to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "Starting continuous read...");
    
    // Continuous read loop
    int success_count = 0;
    int fail_count = 0;
    
    while (1) {
        // Ask for output value
        printf("Enter float value (-7.5 to 7.5) for output");
        fflush(stdout);

        if (fgets(input, sizeof(input), stdin) == NULL) {
            vTaskDelay(200 / portTICK_PERIOD_MS);
            continue;
        }

        input[strcspn(input, "\r\n")] = '\0';  // strip newline

        if (sscanf(input, "%f", &value) == 1) {
            ESP_LOGI(TAG, "Parsed value = %f", value);

            value_A = (value + 7.5)/3;
            value_B = 5-value_A;

            printf("value_A = %f\n", value_A);
            printf("value_B = %f\n", value_B);

            uint16_t dac_output_a = voltage_to_dac(value_A);
            printf("DAC_A Value: 0x%04X (%u)\n", dac_output_a, dac_output_a);
            uint16_t dac_output_b = voltage_to_dac(value_B);
            printf("DAC_B Value: 0x%04X (%u)\n", dac_output_b, dac_output_b);
            
            if (dac8563_write(0, dac_output_a) == ESP_OK) {
                printf("Wrote to DAC A: 0x%04X\n", dac_output_a);
            } else {
                printf("SPI write failed\n");
            }

            if (dac8563_write(1, dac_output_b) == ESP_OK) {
                printf("Wrote to DAC B: 0x%04X\n", dac_output_b);
            } else {
                printf("SPI write failed\n");
            }
        } else {
            printf("Invalid hex input.\n");
        }

        //gpio_set_level(PIN_NUM_DRDY_ADC, 0);

        // Wait for approximately 96 clock cycles
        // This is a rough estimation and may vary slightly due to cache, interrupts, etc.
        // For more precise timing, consider using a hardware timer or RMT peripheral.

        //for (volatile int i = 0; i < 96; i++) {
            // Empty loop for delay
        

        ret = ads131a04_read_frame(&frame);
        if (ret == ESP_OK) {
            success_count++;
            ads131a04_display_frame(&frame);
        } else {
            fail_count++;
            ESP_LOGE(TAG, "Failed to read frame: %s (Success: %d, Failed: %d)", 
                     esp_err_to_name(ret), success_count, fail_count);
            
            if (fail_count > 5 && success_count == 0) {
                ESP_LOGE(TAG, "Multiple failures.");
            }
        }



        // Set GPIO high
        //gpio_set_level(PIN_NUM_DRDY_ADC, 1);

        
        /*
        ret = ads131a04_read_frame(&frame);
        if (ret == ESP_OK) {
            success_count++;
            ads131a04_display_frame(&frame);
        } else {
            fail_count++;
            ESP_LOGE(TAG, "Failed to read frame: %s (Success: %d, Failed: %d)", 
                     esp_err_to_name(ret), success_count, fail_count);
            
            if (fail_count > 5 && success_count == 0) {
                ESP_LOGE(TAG, "Multiple failures.");
            }
        }
            */
        
        //vTaskDelay(pdMS_TO_TICKS(50));  // Read every 500ms
    }
}