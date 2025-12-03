/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 * Semiconductor ASA integrated circuit in a product or a software update for
 * such product, must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 * Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 * engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <string.h>

#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_timer.h"

#define BREADBOARD 0 // 0 if using PCB, 1 if using breadboard

// PIN Definitions

// MUX Control Pins
#if !BREADBOARD
#define A0 20
#endif
#if BREADBOARD
#define A0 18
#endif

#define A1 3
#define A2 10
#define A3 5
#define EN 8

// SPI Pins
#define DAC_CS_PIN 6      // Chip select for DAC
#define ADC_CS_PIN 4      // Chip select for AD7789

#if !BREADBOARD
#define MOSI_PIN 0
#endif
#if BREADBOARD
#define MOSI_PIN 11
#endif
#define MISO_PIN 28
#define SCK_PIN 7

#define SPI_INSTANCE  0
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
static volatile bool spi_xfer_done;

// DAC Data Buffer
#if !BREADBOARD
static uint8_t m_tx_dac_use_internal_ref[] = {0b00111000, 0b00000000, 0b00000001};
static uint8_t m_tx_dac_set_500mV[] = {0b00011000, 0b00010011, 0b01100101};
#endif

#if BREADBOARD
static uint8_t m_tx_breadboard_dac[] = {0b00000000, 0b00011001, 0b10011010};;
#endif

static const uint8_t m_length_dac = 3;


// Data struct
#define MAX_READINGS_PER_CHANNEL 100

typedef struct {
    uint32_t timestamp_ms;
    uint32_t adc_val;
    bool error_flag;
} ADC_Reading;

typedef struct {
    ADC_Reading readings[MAX_READINGS_PER_CHANNEL];
    uint16_t count;
} Channel_Data;

Channel_Data ADC_DATA[16];

void print_data(void) {
    for (int i = 0; i < 15; i++) {
      for (int count = 0; count < ADC_DATA[i].count; i++) {
          NRF_LOG_INFO("%d, %lu", i, ADC_DATA[i].readings[count].adc_val);
      }
    }
}


// Timer for timestamps
static volatile uint32_t m_timestamp_ms = 0;

//uint32_t get_rtc_counter(void)
//{
//    return NRF_RTC1->COUNTER;
//}


/**
 * @brief SPI user event handler.
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void * p_context)
{
    spi_xfer_done = true;
}

//================================================================================
// AD7789 External ADC Functions
//================================================================================

/**
 * @brief Resets the AD7789 by writing at least 32 high bits.
 */
void ad7789_reset()
{
    uint8_t reset_cmd[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    
    nrf_gpio_pin_clear(ADC_CS_PIN);
    nrf_delay_us(1);
    
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, reset_cmd, sizeof(reset_cmd), NULL, 0));
    while (!spi_xfer_done) { __WFE(); }

    nrf_gpio_pin_set(ADC_CS_PIN);
    nrf_delay_ms(10);
    m_timestamp_ms += 10;
    NRF_LOG_INFO("AD7789 Reset.");
}

/**
 * @brief Read the status register
 */
uint8_t ad7789_read_status()
{
    uint8_t tx_data[2] = {0x08, 0x00}; // Read status register
    uint8_t rx_data[2] = {0};
    
    nrf_gpio_pin_clear(ADC_CS_PIN);
    nrf_delay_us(1);
    
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data, 2, rx_data, 2));
    while (!spi_xfer_done) { __WFE(); }
    
    nrf_gpio_pin_set(ADC_CS_PIN);
    nrf_delay_us(10);
    
    return rx_data[1];
}

/**
 * @brief Read mode register
 */
uint8_t ad7789_read_mode()
{
    uint8_t tx_data[2] = {0x18, 0x00}; // Read mode register
    uint8_t rx_data[2] = {0};
    
    nrf_gpio_pin_clear(ADC_CS_PIN);
    nrf_delay_us(1);
    
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data, 2, rx_data, 2));
    while (!spi_xfer_done) { __WFE(); }
    
    nrf_gpio_pin_set(ADC_CS_PIN);
    nrf_delay_us(10);
    
    return rx_data[1];
}

/**
 * @brief Wait for data ready by polling status register
 */
bool ad7789_wait_for_data(uint32_t timeout_ms)
{
    uint32_t elapsed = 0;
    uint8_t status;
    
    do {
        status = ad7789_read_status();
        
        // Check if RDY bit (bit 7) is cleared
        if ((status & 0x80) == 0) {
            // Data is ready
            if (status & 0x40) {
                NRF_LOG_WARNING("Data ready but ERR flag set! Status: 0x%02X", status);
            }
            return true;
        }
        
        nrf_delay_ms(10);
        m_timestamp_ms += 10;
        elapsed += 10;
        
        if (elapsed >= timeout_ms) {
            NRF_LOG_WARNING("Timeout waiting for data. Status: 0x%02X", status);
            return false;
        }
    } while (1);
}

/**
 * @brief Select and configure a channel
 */
void ad7789_select_channel(uint8_t channel)
{
    // Channel bits go in communications register when writing to mode register
    // 00 = AIN(+) - AIN(-)
    // 10 = AIN(-) - AIN(-) (internal short)
    // 11 = VDD monitor
    
    uint8_t comm_reg = 0x10 | (channel & 0x03); // Write to mode reg + channel select
    uint8_t mode_reg = 0x02; // Continuous conversion, bipolar
    
    nrf_gpio_pin_clear(ADC_CS_PIN);
    nrf_delay_us(1);
    
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &comm_reg, 1, NULL, 0));
    while (!spi_xfer_done) { __WFE(); }
    
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &mode_reg, 1, NULL, 0));
    while (!spi_xfer_done) { __WFE(); }
    
    nrf_gpio_pin_set(ADC_CS_PIN);
    nrf_delay_ms(150); // Wait for first conversion
    m_timestamp_ms += 150;
    
    NRF_LOG_INFO("Selected channel 0x%02X", channel);
}

/**
 * @brief Read 24-bit data from data register
 */
uint32_t ad7789_read_data()
{
    // Wait for data to be ready
    if (!ad7789_wait_for_data(200)) {
        NRF_LOG_ERROR("No data ready!");
        return 0xFFFFFFFF;
    }
    
    // Read data register in one transaction
    uint8_t tx_data[4] = {0x38, 0x00, 0x00, 0x00}; // Read data register
    uint8_t rx_data[4] = {0};
    
    nrf_gpio_pin_clear(ADC_CS_PIN);
    nrf_delay_us(1);
    
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data, 4, rx_data, 4));
    while (!spi_xfer_done) { __WFE(); }
    
    nrf_gpio_pin_set(ADC_CS_PIN);
    nrf_delay_us(10);
    
    // Combine bytes
    uint32_t adc_value = ((uint32_t)rx_data[1] << 16) | 
                         ((uint32_t)rx_data[2] << 8) | 
                         rx_data[3];
    
    return adc_value;
}

/**
 * @brief Configure for continuous conversion on analog input
 */
void ad7789_configure_analog_input()
{
    NRF_LOG_INFO("Configuring AD7789 for analog input channel...");
    ad7789_select_channel(0x00); // AIN(+) - AIN(-)
    
    uint8_t mode = ad7789_read_mode();
    NRF_LOG_INFO("Mode register: 0x%02X", mode);
    
    uint8_t status = ad7789_read_status();
    NRF_LOG_INFO("Status register: 0x%02X", status);
}

/**
 * @brief Convert ADC value to voltage
 */
double ad7789_convert_voltage(uint32_t adc_value) 
{
    // Bipolar mode: Code = 2^23 × ((VIN / VREF) + 1)
    // VIN = ((Code / 2^23) - 1) × VREF
    double normalized = (double)adc_value / 8388608.0; // 2^23
    return (normalized - 1.0) * 2.719; // VREF = 2.719V
}

//================================================================================
// DAC functions
//================================================================================

void send_DAC_msg(uint8_t *p_tx_data, uint8_t length) 
{
    nrf_gpio_pin_clear(DAC_CS_PIN);
    
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, p_tx_data, length, NULL, 0));
    while (!spi_xfer_done) { __WFE(); }

    nrf_gpio_pin_set(DAC_CS_PIN);
}


//================================================================================
// Bluetooth functions
//================================================================================




//================================================================================
// Main Application
//================================================================================

void log_init(void)
{    
    //NRF_RTC1->TASKS_START = 1;

    //app_timer_init();

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

int main(void)
{
    bsp_board_init(BSP_INIT_LEDS);
    log_init();

    NRF_LOG_INFO("Application Started.");

    // --- SPI Configuration ---
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.mosi_pin = MOSI_PIN;
    spi_config.miso_pin = MISO_PIN;
    spi_config.sck_pin  = SCK_PIN;
    spi_config.ss_pin   = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.frequency = NRF_SPI_FREQ_250K;
    spi_config.mode = NRF_DRV_SPI_MODE_2;
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
    NRF_LOG_INFO("SPI Initialized (Mode 2 for DAC).");

    // --- GPIO Configuration ---
    nrf_gpio_cfg_output(A0);
    nrf_gpio_cfg_output(A1);
    nrf_gpio_cfg_output(A2);
    nrf_gpio_cfg_output(A3);
    nrf_gpio_cfg_output(EN);
    
    nrf_gpio_cfg_output(ADC_CS_PIN);
    nrf_gpio_cfg_output(DAC_CS_PIN);
    nrf_gpio_pin_set(ADC_CS_PIN);
    nrf_gpio_pin_set(DAC_CS_PIN);
    NRF_LOG_INFO("GPIO Initialized.");

    // Set initial DAC output
    #if !BREADBOARD
    send_DAC_msg(m_tx_dac_use_internal_ref, m_length_dac);
    send_DAC_msg(m_tx_dac_set_500mV, m_length_dac);
    #endif

    #if BREADBOARD
    send_DAC_msg(m_tx_breadboard_dac, m_length_dac);
    #endif

    NRF_LOG_INFO("DAC output set to 0.5V");

    // Switch to SPI Mode 3 for AD7789
    nrf_drv_spi_uninit(&spi);
    spi_config.mode = NRF_DRV_SPI_MODE_3;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
    nrf_gpio_pin_set(ADC_CS_PIN);
    nrf_gpio_pin_set(DAC_CS_PIN);
    NRF_LOG_INFO("SPI Mode 3 Initialized (for AD7789).");

    // Initialize AD7789
    ad7789_reset();
    
    uint8_t status = ad7789_read_status();
    NRF_LOG_INFO("Initial status: 0x%02X (should be 0x8C)", status);
    
    // Configure for normal operation on analog input channel
    ad7789_configure_analog_input();
    
    NRF_LOG_INFO("\n=== Starting Main Loop ===\n");
    
    uint8_t channel = 15;

    while (1)
    {
        bsp_board_led_invert(BSP_BOARD_LED_0);
      
        // Increment MUX channel
        channel++;
        if (channel > 15) {
            channel = 0;
        }

        nrf_gpio_pin_write(A3, (channel >> 3) & 1);
        nrf_gpio_pin_write(A2, (channel >> 2) & 1);
        nrf_gpio_pin_write(A1, (channel >> 1) & 1);
        nrf_gpio_pin_write(A0, (channel >> 0) & 1);
        nrf_gpio_pin_write(EN, 1);
        
        // Delay for MUX to settle and input to stabilize
        nrf_delay_ms(250);
        m_timestamp_ms += 250;

        // Read from AD7789
        uint32_t ad7789_val = ad7789_read_data();
        uint8_t final_status = ad7789_read_status();

        // Add reading to struct
        ADC_Reading data = {m_timestamp_ms, ad7789_val, (final_status & 0x40)};

        uint16_t count = ADC_DATA[channel].count;

        if (count >= MAX_READINGS_PER_CHANNEL) {
            count = 0;
            
        }

        ADC_DATA[channel].readings[count] = data;
        ADC_DATA[channel].count++;

        NRF_LOG_INFO("%lu, %u, %lu", m_timestamp_ms, channel, ad7789_val)

        NRF_LOG_FLUSH();
    }
}