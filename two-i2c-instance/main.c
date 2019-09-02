/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
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
 * @defgroup tw_scanner main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include <stdbool.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"


#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "Adafruit_GFX.h"
#include "SSD1306.h"

#include "bsp.h"
#include "hardfault.h"
#include "sdk_macros.h"
#include "sdk_config.h"

#include "adafruit_pn532.h"
#include "nfc_t2t_parser.h"
#include "nfc_t4t_cc_file.h"
#include "nfc_t4t_hl_detection_procedures.h"
#include "nfc_ndef_msg_parser.h"

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

#define SEL_RES_CASCADE_BIT_NUM            3                                              /// Number of Cascade bit within SEL_RES byte.
#define SEL_RES_TAG_PLATFORM_MASK          0x60                                           /// Mask of Tag Platform bit group within SEL_RES byte.
#define SEL_RES_TAG_PLATFORM_BIT_OFFSET    5                                              /// Offset of the Tag Platform bit group within SEL_RES byte.

#define TAG_TYPE_2_UID_LENGTH              7                                              /// Length of the Tag's UID.
#define TAG_TYPE_2_DATA_AREA_SIZE_OFFSET   (T2T_CC_BLOCK_OFFSET + 2)                      /// Offset of the byte with Tag's Data size.
#define TAG_TYPE_2_DATA_AREA_MULTIPLICATOR 8                                              /// Multiplicator for a value stored in the Tag's Data size byte.
#define TAG_TYPE_2_FIRST_DATA_BLOCK_NUM    (T2T_FIRST_DATA_BLOCK_OFFSET / T2T_BLOCK_SIZE) /// First block number with Tag's Data.
#define TAG_TYPE_2_BLOCKS_PER_EXCHANGE     (T2T_MAX_DATA_EXCHANGE / T2T_BLOCK_SIZE)       /// Number of blocks fetched in single Tag's Read command.

#define TAG_TYPE_4_NDEF_FILE_SIZE           255                                           /// Size of the buffer for NDEF file.
#define TAG_TYPE_4_NLEN_FIELD_SIZE          2                                             /// Size of NLEN field inside NDEF file.


 /* Number of possible TWI addresses. */
 #define TWI_ADDRESSES      127

/* TWI instance. */
const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(1);
const nrf_drv_twi_t m_twi2 = NRF_DRV_TWI_INSTANCE(0);

/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = NRF_GPIO_PIN_MAP(0,7),
       .sda                = NRF_GPIO_PIN_MAP(0,8),
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi_master, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi_master);

    /*const nrf_drv_twi_config_t twi_config2 = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi2, &twi_config2, NULL, NULL);
    NRF_LOG_INFO("%d", err_code);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi2);*/
}

/**
 * @brief Possible Tag Types.
 */
typedef enum
{
    NFC_T2T = 0x00,      ///< Type 2 Tag Platform.
    NFC_T4T = 0x01,      ///< Type 4A Tag Platform.
    NFC_TT_NOT_SUPPORTED ///< Tag Type not supported.
} nfc_tag_type_t;


/**
 * @brief Function for waiting specified time after a Tag read operation.
 */
void after_read_delay(void)
{
    ret_code_t err_code;

    // Turn off the RF field.
    err_code = adafruit_pn532_field_off();
    APP_ERROR_CHECK(err_code);
    nrf_delay_ms(TAG_AFTER_READ_DELAY);
}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    ret_code_t err_code;
    nfc_a_tag_info tag_info;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("TWI scanner started.");
    NRF_LOG_FLUSH();
    twi_init();

    bsp_board_init(BSP_INIT_LEDS);
    bsp_board_led_invert(1);


    SSD1306_begin(SSD1306_SWITCHCAPVCC, 0x3C, false);
    Adafruit_GFX_init(SSD1306_LCDWIDTH, SSD1306_LCDHEIGHT, SSD1306_drawPixel);

    SSD1306_clearDisplay();
    Adafruit_GFX_drawBitmap(0, 0,  el_logo, 128, 64, 1);
    SSD1306_display();

    /*******************NFC**************************/
    NRF_LOG_INFO("NFC Adafruit tag reader example started.");

    err_code = adafruit_pn532_init(false);
    APP_ERROR_CHECK(err_code);

    uint32_t response;
    err_code = adafruit_pn532_firmware_version_get(&response);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("PN532 not found");
    }
    else
    {
        NRF_LOG_INFO("Firmware detected");
    }
    

    for (;;)
    {
        err_code = adafruit_pn532_nfc_a_target_init(&tag_info, TAG_DETECT_TIMEOUT);

        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_INFO("Failed to detect");
        }
        else
        {
            adafruit_pn532_tag_info_printout(&tag_info);

            uint8_t send[] = { 0x00, /* CLA */
                              0xA4, /* INS */
                              0x04, /* P1  */
                              0x00, /* P2  */
                              0x05, /* Length of AID  */
                              0xF0, 0x00, 0x00, 0x00, 0x01, /* AID defined on Android App */
                              0x00  /* Le  */ };

            uint8_t recv[] = {0x00, 0x00, 0x00, 0x00}, lenRecv = 4;

            err_code = adafruit_pn532_in_data_exchange(send, 11, recv, &lenRecv);

            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_INFO("Failed to send APDU");
            }
            else
            {
                NRF_LOG_INFO("Length of APDU recieved %d", lenRecv);
            }
        }

        NRF_LOG_FLUSH();
    }
}

/** @} */
