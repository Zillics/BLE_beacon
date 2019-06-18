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
#include "nrfx_gpiote.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// SAADC
#include "nrfx_saadc.h"
#include "nrfx_ppi.h"
#include "nrf_drv_ppi.h"
#include "nrfx.h"
#include "nrfx_timer.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrfx_rtc.h"

#define SAMPLES_IN_BUFFER 1

static const nrfx_rtc_t     m_rtc = NRFX_RTC_INSTANCE(2);
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel_rtc_int;
static nrf_ppi_channel_t		 m_ppi_channel_int_saadc;
static nrf_ppi_channel_t		 m_ppi_channel_saadc_clr;
static uint32_t              m_adc_evt_counter;

#define INTERRUPT_OUT_GPIO_PIN 11 /* Set this to pin going to INT (event driven control pin) of S6A102 */
#define INTERRUPT_IN_GPIO_PIN 12 /* Set this pin to the "watcher" of VOUT2 pin of S6A102 */

#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

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

/* BLE */

#define ADV_TIMEOUT 400

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

static bool m_adv_ready_flag;
static bool m_oneshot_ready_flag;

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


static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};

#define MOISTURE_DATA_LENGTH 2
static uint8_t m_moisture_level[MOISTURE_DATA_LENGTH] = 
{
		0x00,
		0x00
};

// Converts a 16-bit value to an array of two 8-bit values and places it in dest
static void convert16to8(uint16_t src, uint8_t * dest)
{
	//NRF_LOG_INFO("CONVERTING %d .....",src);
	dest[0] = (uint8_t) (src >> 8);
	dest[1] = (uint8_t) (src);
	//NRF_LOG_INFO("...TO [%d , %d]",dest[0],dest[1]);
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

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

    uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

    m_beacon_info[index++] = MSB_16(major_value);
    m_beacon_info[index++] = LSB_16(major_value);

    m_beacon_info[index++] = MSB_16(minor_value);
    m_beacon_info[index++] = LSB_16(minor_value);
#endif

    manuf_specific_data.data.p_data = (uint8_t *) m_moisture_level;
    manuf_specific_data.data.size   = MOISTURE_DATA_LENGTH;//APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = ADV_TIMEOUT/10;

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
    //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    //APP_ERROR_CHECK(err_code);
}

static void advertising_stop(void)
{
	ret_code_t err_code;
	err_code = sd_ble_gap_adv_stop(m_adv_handle);
	APP_ERROR_CHECK(err_code);
}

static void ble_hids_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
	uint32_t err_code;
	switch (p_ble_evt->header.evt_id)
	{
			case BLE_GAP_EVT_TIMEOUT:
					m_adv_ready_flag = true;
					nrfx_gpiote_out_toggle(LED_3);
					advertising_start();
					break; // BLE_GAP_EVT_TIMEOUT
			default:
					// No implementation needed.
					break;
	}
}

NRF_SDH_BLE_OBSERVER(m_ble_observer, BLE_HIDS_BLE_OBSERVER_PRIO, ble_hids_on_ble_evt, NULL);

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

/* RTC */

static void rtc_handler(nrfx_rtc_int_type_t int_type)
{
	
}

void rtc_init( void )
{
		ret_code_t err_code;

    nrfx_rtc_config_t rtc_config = NRFX_RTC_DEFAULT_CONFIG;
		rtc_config.prescaler = 4095; // Set prescaler to 4095 -> frequency = 8 Hz
    rtc_config.interrupt_priority = NRFX_RTC_DEFAULT_CONFIG_IRQ_PRIORITY;
    rtc_config.reliable = NRFX_RTC_DEFAULT_CONFIG_RELIABLE;                  
    rtc_config.tick_latency = NRFX_RTC_US_TO_TICKS(NRFX_RTC_MAXIMUM_LATENCY_US,NRFX_RTC_DEFAULT_CONFIG_FREQUENCY);
    err_code = nrfx_rtc_init(&m_rtc, &rtc_config, rtc_handler);
    APP_ERROR_CHECK(err_code);
    //Set compare channel 0 to trigger when value reaches 8 -> at 1 Hz
    err_code = nrfx_rtc_cc_set(&m_rtc,0,40,false);
    APP_ERROR_CHECK(err_code);
		
		nrfx_rtc_enable(&m_rtc); //Power on RTC instance	
}

/* RTC end */

/* GPIOTE */ 
void interrupt_in_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
		nrfx_gpiote_out_toggle(LED_3);
}

void gpiote_pins_init()
{
		// Initialize interrupt output pin
		ret_code_t err_code;
		nrfx_gpiote_out_config_t config_gpiote_out;
		config_gpiote_out.action = NRF_GPIOTE_POLARITY_TOGGLE;
		config_gpiote_out.init_state = NRF_GPIOTE_INITIAL_VALUE_LOW;
		config_gpiote_out.task_pin = true; //...or should it be false?
		err_code = nrfx_gpiote_out_init(INTERRUPT_OUT_GPIO_PIN, &config_gpiote_out);
		nrfx_gpiote_out_task_enable(INTERRUPT_OUT_GPIO_PIN);
		APP_ERROR_CHECK(err_code);
		// Initialize interrupt input pin
		nrfx_gpiote_in_config_t config_gpiote_in ;//= NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_TOGGLE	(true)	;
		config_gpiote_in.sense = NRF_GPIOTE_POLARITY_LOTOHI;
		config_gpiote_in.pull = NRF_GPIO_PIN_PULLUP;
		config_gpiote_in.is_watcher = true; //...what exactly is this?
		config_gpiote_in.hi_accuracy = true; //...what exactly is this?
		err_code = nrfx_gpiote_in_init(INTERRUPT_IN_GPIO_PIN, &config_gpiote_in,interrupt_in_handler);
		APP_ERROR_CHECK(err_code);
		nrfx_gpiote_in_event_enable(INTERRUPT_IN_GPIO_PIN,true); // change to false if hi_accuracy is false
}

/* GPIOTE end */

/*SAADC*/


void timer_handler(nrf_timer_event_t event_type, void * p_context)
{

}


void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);
		err_code = nrfx_gpiote_init();
		APP_ERROR_CHECK(err_code);
		rtc_init(); // Setup and enable RTC timer
		gpiote_pins_init();
	
		// Get all event and task addresses
		uint32_t rtc_compare_event_addr = nrfx_rtc_event_address_get(&m_rtc, NRF_RTC_EVENT_COMPARE_0);
    uint32_t rtc_clear_task_addr = nrfx_rtc_task_address_get(&m_rtc, NRF_RTC_TASK_CLEAR);
    uint32_t saadc_sample_task_addr = nrfx_saadc_sample_task_get();
		uint32_t gpiote_task_addr = nrfx_gpiote_out_task_addr_get(INTERRUPT_OUT_GPIO_PIN);
		uint32_t int_in_event_addr = nrfx_gpiote_in_event_addr_get(INTERRUPT_IN_GPIO_PIN);
		uint32_t saadc_evt_end_addr = nrf_saadc_event_address_get(NRF_SAADC_EVENT_RESULTDONE);
	
		/* Setup ppi channel so that: RTC COMPARE -> TASK: output interrupt pin HIGH + FORK: RTC clear*/
    err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_rtc_int);
    APP_ERROR_CHECK(err_code);
		//nrfx_ppi_channel_assign(ppi_channel, event, task_to_activate_during_event);
		err_code = nrfx_ppi_channel_assign(m_ppi_channel_rtc_int, rtc_compare_event_addr, gpiote_task_addr);
		APP_ERROR_CHECK(err_code);
		// Assign a second task, i.e. a "fork" to be executed in addition to primary task
		err_code = nrfx_ppi_channel_fork_assign(m_ppi_channel_rtc_int, rtc_clear_task_addr);
		APP_ERROR_CHECK(err_code);
		
		/* Setup ppi channel so that: Rising edge on interrupt pin -> TASK: start SAADC sampling */
		err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_int_saadc);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_ppi_channel_assign(m_ppi_channel_int_saadc,int_in_event_addr,saadc_sample_task_addr);
		
		/* Setup ppi channel so that: SAADC_EVT_DONE -> TASK: clear output interrupt pin */
		err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_saadc_clr);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_ppi_channel_assign(m_ppi_channel_saadc_clr,saadc_evt_end_addr,gpiote_task_addr);
}


void saadc_sampling_event_enable(void)
{
		// Enable ppi channel for executing saadc sampling
    ret_code_t err_code = nrfx_ppi_channel_enable(m_ppi_channel_rtc_int);
    APP_ERROR_CHECK(err_code);
		// Enable ppi channel for toggling GPIO pin HIGH needed for saadc sampling
		err_code = nrfx_ppi_channel_enable(m_ppi_channel_int_saadc);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_ppi_channel_enable(m_ppi_channel_saadc_clr);
		APP_ERROR_CHECK(err_code);
}


void saadc_callback(nrfx_saadc_evt_t const * p_event)
{
    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        //NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter);

        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {			 
						convert16to8((uint16_t)p_event->data.done.p_buffer[i], m_moisture_level);
            //NRF_LOG_INFO("%d", p_event->data.done.p_buffer[i]);
        }
        m_adc_evt_counter++;
				//uint32_t gpiote_task_addr = nrfx_gpiote_set_task_addr_get(INTERRUPT_OUT_GPIO_PIN);
				//nrf_gpiote_task_force	(gpiote_task_addr,NRF_GPIOTE_INITIAL_VALUE_LOW);
				//nrf_gpiote_task_disable(gpiote_task_addr);
				//nrf_gpiote_te_default(gpiote_task_addr);
    }
		advertising_init();
		advertising_start();	
}


void saadc_init(void)
{
		ret_code_t err_code;
		
		nrfx_saadc_config_t saadc_config;
		saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;                                 
		saadc_config.oversample = NRF_SAADC_OVERSAMPLE_256X;
		saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;
		saadc_config.low_power_mode = true;

		nrf_saadc_channel_config_t channel_config; //= NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_config.gain       = NRF_SAADC_GAIN1_6;
    channel_config.reference  = NRF_SAADC_REFERENCE_INTERNAL;
    channel_config.acq_time   = NRF_SAADC_ACQTIME_40US;
    channel_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;
		channel_config.burst      = NRF_SAADC_BURST_ENABLED; // If ON: Executes all Oversample times as quickly as possible on each activation
    channel_config.pin_p      = NRF_SAADC_INPUT_AIN0;
    channel_config.pin_n      = NRF_SAADC_INPUT_DISABLED;
		
		err_code = nrfx_saadc_init(&saadc_config, saadc_callback);
		APP_ERROR_CHECK(err_code);

		err_code = nrfx_saadc_channel_init(0, &channel_config);
		APP_ERROR_CHECK(err_code);

		err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
		APP_ERROR_CHECK(err_code);

		err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER); // "Extra" buffer to write to when buffer 1 is full
		APP_ERROR_CHECK(err_code);
		
}


// SAADC END


/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
	
}

/**@brief Function for initializing LEDs. */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}


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
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
				uint32_t err_code;
				err_code = sd_app_evt_wait();
				APP_ERROR_CHECK(err_code);

    }
}

/**
 * @brief Customized error handler
 */

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
	//NRF_LOG_INFO("ERROR on line %d, pc: %d, info: %d", id, pc, info);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
		//leds_init();
		//log_init();
		timers_init();
		power_management_init();
		ble_stack_init();
		advertising_init();
    advertising_start();
		saadc_init();
    saadc_sampling_event_init();
    saadc_sampling_event_enable();
    //NRF_LOG_INFO("SAADC HAL simple example started.");

    while (1)
    {
        nrf_pwr_mgmt_run();
        //NRF_LOG_FLUSH();
    }
}


/**
 * @}
 */
