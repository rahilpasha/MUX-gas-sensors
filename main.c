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
#include <math.h>

#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// SAADC includes
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

// PIN Definitions
// MUX Control Pins
#define A0 18
#define A1 3
#define A2 10
#define A3 5
#define EN 8

#define DEBUG_MUX 0

// SPI Peripherals Chip Select (/CS) Pins
#define DAC_CS_PIN 6      // Chip select for DAC
#define ADC_CS_PIN 4      // Chip select for AD7789
#define ADC_RDY_PIN 28    // AD7789 Data Ready pin (/RDY)

// SPI bus pins
#define MOSI_PIN 0
#define MISO_PIN 28
#define SCK_PIN 7

#define SPI_INSTANCE  0
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
static volatile bool spi_xfer_done;

// DAC Data Buffer (Example: 0.5V vout)
static uint8_t m_tx_dac_use_internal_ref[] = {0b00111000, 0b00000000, 0b00000001};
static uint8_t m_tx_dac_set_500mV[] = {0b00011000, 0b00010011, 0b01100101};
static const uint8_t m_length_dac = 3;

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
 * This ensures the ADC is in a known default state.
 */
void ad7789_reset()
{
    uint8_t reset_cmd[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    
    nrf_gpio_pin_clear(ADC_CS_PIN); // Pull CS low
    nrf_delay_us(10); // Small delay
    
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, reset_cmd, sizeof(reset_cmd), NULL, 0));
    while (!spi_xfer_done) { __WFE(); }

    nrf_delay_us(10); // Small delay
    nrf_gpio_pin_set(ADC_CS_PIN);   // Pull CS high
    nrf_delay_ms(1); // Wait for reset to complete
    NRF_LOG_INFO("AD7789 Reset.");
}


/**
 * @brief Reads a single 24-bit value from the AD7789.
 *
 * @return The 24-bit ADC conversion result.
 */
uint32_t ad7789_read_single()
{
    uint8_t read_cmd[1] = {0x38};
    
    uint8_t adc_output[3] = {0};
    uint8_t dummy_tx[3]   = {0};

    nrf_gpio_pin_clear(ADC_CS_PIN); // Pull CS low
    nrf_delay_us(10); // Small delay

    // The AD7789 pulls the /RDY pin low when a conversion is complete.
    while (nrf_gpio_pin_read(ADC_RDY_PIN) == 1)
    {
        // Wait for /RDY to go low
    }
    
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, read_cmd, 1, NULL, 0));
    while (!spi_xfer_done) { __WFE(); }

    nrf_delay_us(10); // Small delay
    
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, dummy_tx, 3, adc_output, 3));
    while (!spi_xfer_done) { __WFE(); }

    nrf_gpio_pin_set(ADC_CS_PIN); // Deselect ADC

    // Combine the 3 bytes into a single 24-bit value
    uint32_t adc_value = (adc_output[0] << 16) | (adc_output[1] << 8) | (adc_output[2]);
    return adc_value;
}

/**
 * @brief Reads a single 24-bit value from the AD7789.
 *
 * @return The 24-bit ADC conversion result.
 */
double ad7789_convert_voltage(uint32_t adc_value) {

    double offset = adc_value / pow(2, 23);
    return (offset - 1) * 2.9;

}

//================================================================================
// DAC functions
//================================================================================

void send_DAC_msg(uint8_t *p_tx_data, uint8_t length) {
    nrf_gpio_pin_clear(DAC_CS_PIN); // Select DAC
    
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, p_tx_data, length, NULL, 0));
    while (!spi_xfer_done) { __WFE(); }

    nrf_gpio_pin_set(DAC_CS_PIN); // Deselect DAC
}

//================================================================================
// SAADC (Internal ADC) Functions
//================================================================================
void saadc_callback_handler(nrf_drv_saadc_evt_t const * p_event)
{
  // This callback is required by the driver but can be empty for simple blocking samples.
}

void saadc_init(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_saadc_init(NULL, saadc_callback_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
}

//================================================================================
// Main Application
//================================================================================

void log_init(void)
{
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
    spi_config.miso_pin = MISO_PIN; // ADDED: MISO pin is required for reading data
    spi_config.sck_pin  = SCK_PIN;
    spi_config.ss_pin   = NRF_DRV_SPI_PIN_NOT_USED; // We manage CS pins manually
    spi_config.frequency = NRF_SPI_FREQ_250K;
    spi_config.mode = NRF_DRV_SPI_MODE_2;
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
    NRF_LOG_INFO("SPI Initialized.");

    // --- GPIO Configuration ---

    // Disable NFC Pin protection for P0.10 -> Allow for GPIO
    

    // MUX control pins
    nrf_gpio_cfg_output(A0);
    nrf_gpio_cfg_output(A1);
    nrf_gpio_cfg_output(A2);
    nrf_gpio_cfg_output(A3);
    nrf_gpio_cfg_output(EN);
    
    // SPI Chip Select pins
    nrf_gpio_cfg_output(ADC_CS_PIN);
    nrf_gpio_cfg_output(DAC_CS_PIN);
    nrf_gpio_pin_set(ADC_CS_PIN); // Deselect ADC
    nrf_gpio_pin_set(DAC_CS_PIN); // Deselect DAC

    // ADC Ready pin
    nrf_gpio_cfg_input(ADC_RDY_PIN, NRF_GPIO_PIN_PULLUP);
    NRF_LOG_INFO("GPIO Initialized.");

    // --- Peripheral Initialization ---
    saadc_init();
    NRF_LOG_INFO("SAADC Initialized.");
    
    // Set initial DAC output
    send_DAC_msg(m_tx_dac_use_internal_ref, m_length_dac);
    send_DAC_msg(m_tx_dac_set_500mV, m_length_dac);
    NRF_LOG_INFO("DAC output set.");

    // Set SPI Mode to 3
    nrf_drv_spi_uninit(&spi);
    spi_config.mode = NRF_DRV_SPI_MODE_3;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
    NRF_LOG_INFO("SPI Mode 3 Initialized.");

    // Reset external ADC
    ad7789_reset();

    bool beta[] = {false, false, false, false};

    while (1)
    {
        bsp_board_led_invert(BSP_BOARD_LED_0);
      
        // --- MUX Logic ---
        // This logic increments a 4-bit binary number (0000 to 1111)
        #if DEBUG_MUX == 0
        int carry = 1;
        for (int i = 3; i >= 0; i--) {
            int sum = beta[i] + carry;
            beta[i] = sum % 2;
            carry = sum / 2;
        }
        #endif
        nrf_gpio_pin_write(A3, beta[0]);
        nrf_gpio_pin_write(A2, beta[1]);
        nrf_gpio_pin_write(A1, beta[2]);
        nrf_gpio_pin_write(A0, beta[3]);
        nrf_gpio_pin_write(EN, 1);

        char beta_str[5];
        for (int i = 0; i < 4; i++) {
            beta_str[i] = beta[i] ? '1' : '0';
        }
        beta_str[4] = '\0';
        //NRF_LOG_INFO("MUX set to: %s", (uint32_t)beta_str);
        
        nrf_delay_ms(500); // Delay for MUX to settle

        // --- Measurement ---
        // Read from internal SAADC
        //nrf_saadc_value_t saadc_val;
        //nrf_drv_saadc_sample_convert(0, &saadc_val);
        //NRF_LOG_INFO("Internal SAADC value: %d", saadc_val);

        // Read from external AD7789
        uint32_t ad7789_val = ad7789_read_single();
        NRF_LOG_INFO("External AD7789 value: %d (0x%X)", ad7789_val, ad7789_val);

        double adc_voltage = ad7789_convert_voltage(ad7789_val);
        //NRF_LOG_INFO("External AD7789 voltage: %lf test", adc_voltage);


        // CSV Output
        NRF_LOG_INFO("%s, %lu", (uint32_t)beta_str, ad7789_val)

        NRF_LOG_FLUSH();
        //nrf_delay_ms(10);
    }
}
