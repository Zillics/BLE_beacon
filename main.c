/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
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
/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_gpiote.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// SAADC
#include "nrfx.h"
#include "nrfx_timer.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"

// Capsense
#include "nrf_drv_csense.h"

// Services
#include "moisture_service.h"

// 
#include "circular_buffer.h"

#define SAMPLES_IN_BUFFER 1

#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

/* BLE */
#define NON_CONNECTABLE_ADV_TIMEOUT     400
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define APP_BEACON_INFO_LENGTH          0x17                               /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                               /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                         /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                         /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
  0x45, 0x56, 0x67, 0x78, \
0x89, 0x9a, 0xab, 0xbc, \
0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                         /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

// Capacitive sensing
#define WAKEUP_INTERVAL         5000 // How often to wake up and take measurements
#define APP_TIMER_TICKS_TIMEOUT APP_TIMER_TICKS(WAKEUP_INTERVAL)
#define AIN_1                   1
#define AIN_2                   2
#define AIN_7                   7
#define ELECTRODE_PIN           AIN_2
#define CAP_PAD_MASK            (1UL << AIN_2)
/** Pin on which to generate voltage for charging the capacitors used for measuring capacitance */
#define CHARGE_OUTPUT_PIN 26

#define MOISTURE_DATA_LENGTH 2
#define MOISTURE_DATA_BUFFER_LENGTH 50

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
  .adv_data =
  {
    .p_data = m_enc_advdata,
    .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
  },
  .scan_rsp_data =
  {
    .p_data = NULL,
    .len    = 0

  }
};

static circular_buffer_t m_moisture_buffer;
static uint16_t m_moisture_running_average = 1700;
static uint16_t m_moisture_reading = 0;
static uint8_t m_moisture_level[MOISTURE_DATA_LENGTH] = 
{
  0x00,
  0x00
};

// Converts a 16-bit value to an array of two 8-bit values and places it in dest
static void convert16to8(uint16_t src, uint8_t * dest)
{
  dest[0] = (uint8_t) (src);
  dest[1] = (uint8_t) (src >> 8);
}


// Handling of moisture data

uint16_t running_average(uint16_t prev_avg, uint16_t new_val, uint16_t N) {
  float avg = (float) prev_avg + (1.0 / (float)N) * ((float)new_val - prev_avg);
  return (uint16_t)avg;
}

void moisture_measurements_init(void) {
  uint16_t init_value = 1700;
  reset(&m_moisture_buffer, init_value);
  m_moisture_running_average = init_value;
}

void update_moisture(uint16_t val) {
  uint16_t N = length(&m_moisture_buffer);
  m_moisture_reading = val;
  m_moisture_running_average = running_average(m_moisture_running_average, val, N);
  push_back(&m_moisture_buffer, val);
}

bool too_dry(void) {
  return true;
  //return m_moisture_running_average < 1600;
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
  uint32_t      err_code;
  ble_advdata_t advdata;
  uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
  ble_advdata_manuf_data_t manuf_specific_data;

  // Setup service data
  ble_advdata_service_data_t moisture_service;
  moisture_service.service_uuid  = BLE_UUID_MOISTURE_SERVICE; 
  moisture_service.data.size = MOISTURE_DATA_LENGTH; 
  moisture_service.data.p_data = (uint8_t *) m_moisture_level;

  // Set manuf specific data
  manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
  manuf_specific_data.data.p_data = (uint8_t *) m_moisture_level;
  manuf_specific_data.data.size   = MOISTURE_DATA_LENGTH;//APP_BEACON_INFO_LENGTH;

  // Build and set advertising data.
  memset(&advdata, 0, sizeof(advdata));

  advdata.name_type             = BLE_ADVDATA_NO_NAME;
  advdata.flags                 = flags;
  advdata.p_manuf_specific_data = &manuf_specific_data;
  advdata.p_service_data_array  = &moisture_service;
  advdata.service_data_count    = 1;

  // Initialize advertising parameters (used when starting advertising).
  memset(&m_adv_params, 0, sizeof(m_adv_params));

  m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
  m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
  m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
  m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
  m_adv_params.duration        = NON_CONNECTABLE_ADV_TIMEOUT/10;

  err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
  APP_ERROR_CHECK(err_code);

  err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
*/
static void advertising_start(void)
{
  ret_code_t err_code;
  err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
  ret_code_t err_code;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);

}

/* BLE end */

/**@brief Function for initializing timers. */
static void timers_init(void)
{
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
*/
static void power_management_init(void)
{
  // Enable internal DCDC regulator (instead of default LDO)
  sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
  // Set sub power mode to "low power"
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);

  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);

}


// Capacitive sensing START

static void test_advertise_handler(void * p_context) {
  advertising_init();
  advertising_start();
}

void csense_uninitialize(void) {
  nrf_drv_csense_channels_disable(CAP_PAD_MASK);
  nrf_drv_csense_uninit();
}

/** Handler for all csense events. */
void csense_handler(nrf_drv_csense_evt_t * p_event_struct)
{
  switch (p_event_struct->analog_channel)
  {
    case ELECTRODE_PIN:
      update_moisture(p_event_struct->read_value);
      csense_uninitialize();
      if(too_dry()) {
        //convert16to8(m_moisture_running_average, m_moisture_level);
        convert16to8(m_moisture_reading, m_moisture_level);
        advertising_init();
        advertising_start();
      }
      break;

    default:
      break;
  }
}

void csense_initialize(void)
{
  ret_code_t err_code;

  nrf_drv_csense_config_t csense_config = { 0 };

  csense_config.output_pin = CHARGE_OUTPUT_PIN;

  err_code = nrf_drv_csense_init(&csense_config, csense_handler);
  APP_ERROR_CHECK(err_code);
  nrf_drv_csense_channels_enable(CAP_PAD_MASK);
}

static void csense_timeout_handler(void * p_context)
{
  csense_initialize();
  ret_code_t err_code;
  err_code = nrf_drv_csense_sample();
  if (err_code != NRF_SUCCESS)
  {
    return;
  }
}

void start_app_timer(void)
{
  ret_code_t err_code;

  APP_TIMER_DEF(timer_0);

  err_code = app_timer_create(&timer_0, APP_TIMER_MODE_REPEATED, csense_timeout_handler);
  APP_ERROR_CHECK(err_code);

  err_code = app_timer_start(timer_0, APP_TIMER_TICKS_TIMEOUT, NULL);
  APP_ERROR_CHECK(err_code);
}



// Capacitive sensing END

/**
 * @brief Function for application main entry.
 */
int main(void)
{
  moisture_measurements_init();
  timers_init();
  power_management_init();
  ble_stack_init();
  start_app_timer();
  while (1)
  {
    nrf_pwr_mgmt_run();
  }
}


/**
 * @}
 */
