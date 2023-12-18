/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
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
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"

#include "app_timer.h"

#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_power.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "simple_ble.h"
#include "custom_board.h"

#include "nrf_drv_pwm.h"
#include "nrf_delay.h"
#include "nrf_drv_saadc.h"

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

uint16_t time1_cnt = 0;
uint16_t time2_cnt = 0;
uint8_t shut_flag = 0;
uint8_t shut_on_flag = 0;
uint8_t shut_off_flag = 0;
uint8_t BAT_chrg_flag = 0;
uint8_t BAT_pwr_level = 0;


nrf_saadc_value_t saadc_val;
nrf_saadc_value_t new_saadc_val;
uint8_t voltage_level = 0;
uint32_t sum = 0;
uint8_t num_samples = 0;
double adc_voltage = 0;

//
//uint32_t colors[10][8] = {{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
//													{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x0f0000},
//													{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x0f0000, 0x0f0000},
//													{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x0f0000, 0x0f0000, 0x0f0000},
//													{0x000000, 0x000000, 0x000000, 0x000000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000},
//													{0x000000, 0x000000, 0x000000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000},
//													{0x000000, 0x000000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000},
//													{0x000000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000},
//													{0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0xff0000},	
//													{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000f00},		
//													
//};

//uint32_t colors[11][8] = {{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
//													{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x0f0000},
//													{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x0f0000, 0x0f0000},
//													{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x0f0000, 0x0f0000, 0x0f0000},
//													{0x000000, 0x000000, 0x000000, 0x000000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000},
//													{0x000000, 0x000000, 0x000000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000},
//													{0x000000, 0x000000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000},
//													{0x000000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000},
////													{0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff},
//													{0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000, 0x0f0000},	
//													{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000f00},
//													{0x000f00, 0x000f00, 0x000f00, 0x000f00, 0x000f00, 0x000f00, 0x000f00, 0x000f00},	
//													
//};
//uint32_t colors[11][8] = {{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
//													{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x050000},
//													{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x050000, 0x050000},
//													{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x050000, 0x050000, 0x050000},
//													{0x000000, 0x000000, 0x000000, 0x000000, 0x050000, 0x050000, 0x050000, 0x050000},
//													{0x000000, 0x000000, 0x000000, 0x050000, 0x050000, 0x050000, 0x050000, 0x050000},
//													{0x000000, 0x000000, 0x050000, 0x050000, 0x050000, 0x050000, 0x050000, 0x050000},
//													{0x000000, 0x050000, 0x050000, 0x050000, 0x050000, 0x050000, 0x050000, 0x020000},
////													{0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff},
//													{0x050000, 0x050000, 0x050000, 0x050000, 0x050000, 0x050000, 0x050000, 0x050000},	
//													{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000500},
//													{0x000500, 0x000500, 0x000500, 0x000500, 0x000500, 0x000500, 0x000500, 0x000500},	
//													
//};

uint32_t colors[11][8] = {{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
													{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x010000},
													{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x010000, 0x010000},
													{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x010000, 0x010000, 0x010000},
													{0x000000, 0x000000, 0x000000, 0x000000, 0x010000, 0x010000, 0x010000, 0x010000},
													{0x000000, 0x000000, 0x000000, 0x010000, 0x010000, 0x010000, 0x010000, 0x010000},
													{0x000000, 0x000000, 0x010000, 0x010000, 0x010000, 0x010000, 0x010000, 0x010000},
													{0x000000, 0x010000, 0x010000, 0x010000, 0x010000, 0x010000, 0x010000, 0x010000},
//													{0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff},
													{0x010000, 0x010000, 0x010000, 0x010000, 0x010000, 0x010000, 0x010000, 0x010000},	
													{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000100},
													{0x000100, 0x000100, 0x000100, 0x000100, 0x000100, 0x000100, 0x000100, 0x000100},	
													
};
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
 
static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

void pwm_callback(nrf_drv_pwm_evt_type_t event_type)    // PWM callback function
{
    if (event_type == NRF_DRV_PWM_EVT_FINISHED) //from high to low before 
    {
        
        
        
    }
    
}

#define LED_NUM 8

void ws2812_begin(void)
{
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            LEDRING, // channel 0
            NRF_DRV_PWM_PIN_NOT_USED, // channel 1
            NRF_DRV_PWM_PIN_NOT_USED, // channel 2
            NRF_DRV_PWM_PIN_NOT_USED  // channel 3
        },
        .irq_priority = 0,
        .base_clock   = NRF_PWM_CLK_16MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 20,  //1.25us
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, pwm_callback));
      
}
void ws2812_over(void)
{
		nrfx_pwm_uninit(&m_pwm0);
      
}

#define NEOPIXEL_BYTES 24
#define NEOPIXEL_MAX_CHAINS 8

static nrf_pwm_values_common_t pwm_sequence_values[NEOPIXEL_MAX_CHAINS * NEOPIXEL_BYTES + 1];


uint32_t colorss[LED_NUM] = {0, 0, 0, 0, 0, 0, 0, 0};
void neopixel_write(uint32_t *colors, uint32_t chain) {

  nrf_pwm_values_common_t *ptr = pwm_sequence_values;
  for (uint32_t led = 0; led < chain; led++) {
    uint32_t color = colors[led];
    for (uint8_t i = 0; i < NEOPIXEL_BYTES; ++i) {
      uint16_t value = 0;
      if ((color & 0x800000) == 0) {
        value = 6;
      } else {
        value = 13;
      }
      *ptr++ = value | 0x8000;
      color <<= 1;
    }
  }
  *ptr++ = 0x8000;

  nrf_pwm_sequence_t const seq0 =
      {
        .values.p_common = pwm_sequence_values,
        .length = NEOPIXEL_BYTES * chain + 1,
        .repeats = 0,
        .end_delay = 0
      };

  (void)nrf_drv_pwm_simple_playback(&m_pwm0, &seq0, 1, NRF_DRV_PWM_FLAG_STOP);
}

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
void nus_data_handler(ble_nus_evt_t * p_evt)
{
	if (p_evt->type == BLE_NUS_EVT_RX_DATA)
	{
		NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
		NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
	}
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

//    err_code = bsp_btn_ble_init(NULL, &startup_event);
//    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
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
        nrf_pwr_mgmt_run();
    }
}

APP_TIMER_DEF(sample_timer);
APP_TIMER_DEF(dingshi_timer);


static void m_sample_timer_handler(void *p_context)
{
	NRF_LOG_INFO("time1_cnt=%d\r\n",time1_cnt);
	//开关机判断
	if (nrf_gpio_pin_read(MCU_KEY) == 1)
	{
		time1_cnt++;
//		NRF_LOG_INFO("time1_cnt = %d ",time1_cnt);
		if (time1_cnt ==  200)
		{
			if (nrf_gpio_pin_read(MCU_KEY) == 1)
			{
				time1_cnt = 0;
				shut_flag++;	
			}
			if(shut_flag == 1)
			{
				nrf_gpio_pin_set(MCU_EN);
				shut_on_flag = 1;
				NRF_LOG_INFO("3.7V power output.");
			}
			else if (shut_flag == 2)
			{
				nrf_gpio_pin_clear(MCU_EN);
				shut_on_flag = 0;
				NRF_LOG_INFO("3.7V power close.");
				shut_flag = 0;
			}
		}
	}
	else
	{
		time1_cnt = 0;
	}
}
static void d_sample_timer_handler(void *p_context)
{
	NRF_LOG_INFO("time2_cnt=%d\r\n",time2_cnt);
	//充放电状态判断
	time2_cnt++;
	if(time2_cnt%5 == 0)
	{
		nrfx_saadc_sample_convert(0, &saadc_val);
		sum += saadc_val;
		num_samples++;
		if(num_samples == 9)
		{
			new_saadc_val = sum / num_samples;
			sum = 0;
//			NRF_LOG_INFO("new_saadc_val = %d ",new_saadc_val);
			num_samples = 0;
		}
	}
	else if(time2_cnt%11 == 0)
	{
		if((nrf_gpio_pin_read(STBY) == 1)&&(nrf_gpio_pin_read(CHRG) == 0))
		{
			BAT_chrg_flag = 1;
		}
		else if((nrf_gpio_pin_read(STBY) == 0)&&(nrf_gpio_pin_read(CHRG) == 1))
		{
			BAT_chrg_flag = 2;
		}
		else 
		{
			BAT_chrg_flag = 0;
		}	
	}
	//电池电量监测
	else if(time2_cnt > 50)
	{
		time2_cnt = 0;
		//电池电量转化为0-4.2V范围，以对应
		adc_voltage = (float)new_saadc_val / 1024 * 1.8 / 0.4194;
		
		if (adc_voltage >= 3.0 && adc_voltage <= 3.711) 
		{
			if(shut_on_flag == 1)
			{
				BAT_pwr_level = 1;
			}	
			else
			{
				BAT_pwr_level = 0;
			}
		} 
		else if (adc_voltage > 3.711 && adc_voltage <= 3.746) 
		{
			if(shut_on_flag == 1)
			{
				BAT_pwr_level = 2;
			}	
			else
			{
				BAT_pwr_level = 0;
			}
		} 
		else if (adc_voltage > 3.746 && adc_voltage <= 3.769) 
		{
			if(shut_on_flag == 1)
			{
				BAT_pwr_level = 3;
			}	
			else
			{
				BAT_pwr_level = 3;
			}
		} 
		else if (adc_voltage > 3.769 && adc_voltage <= 3.826) 
		{
			if(shut_on_flag == 1)
			{
				BAT_pwr_level = 4;
			}	
			else
			{
				BAT_pwr_level = 0;
			}
		} 
		else if (adc_voltage > 3.826 && adc_voltage <= 3.877) 
		{
			if(shut_on_flag == 1)
			{
				BAT_pwr_level = 5;
			}	
			else
			{
				BAT_pwr_level = 0;
			}
		} 
		else if (adc_voltage > 3.877 && adc_voltage <= 3.949) 
		{
			if(shut_on_flag == 1)
			{
				BAT_pwr_level = 6;
			}	
			else
			{
				BAT_pwr_level = 0;
			}
		} 
		else if (adc_voltage > 3.949 && adc_voltage <= 4.038) 
		{
			if(shut_on_flag == 1)
			{
				BAT_pwr_level = 7;
			}	
			else
			{
				BAT_pwr_level = 0;
			}
		} 
		else if (adc_voltage > 4.038 && adc_voltage <= 4.200) 
		{
			if(shut_on_flag == 1)
			{
				BAT_pwr_level = 8;
			}	
			else
			{
				BAT_pwr_level = 0;
			}
		} 
		
	}

	
}



static void timers_create(void)
{
    ret_code_t err_code;
		err_code = app_timer_create(&sample_timer, APP_TIMER_MODE_REPEATED, m_sample_timer_handler);
		APP_ERROR_CHECK(err_code);
		err_code = app_timer_create(&dingshi_timer, APP_TIMER_MODE_REPEATED, d_sample_timer_handler);
		APP_ERROR_CHECK(err_code);

}

static void timers_start(void)
{
	ret_code_t err_code;
	err_code = app_timer_start(sample_timer, APP_TIMER_TICKS(9), NULL);  //9ms
	APP_ERROR_CHECK(err_code);
	err_code = app_timer_start(dingshi_timer, APP_TIMER_TICKS(19), NULL);  //19ms
	APP_ERROR_CHECK(err_code);
}


static void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    
}

void saadc_init(void)
{

    ret_code_t err_code;

    nrf_saadc_channel_config_t channel_volt = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(VMON);
    channel_volt.gain = NRF_SAADC_GAIN1_3; //1.8v

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_volt);
    APP_ERROR_CHECK(err_code);

}



void gpio_init(void)
{
	nrf_gpio_cfg_input(COMMUNICATION, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(STBY, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(CHRG, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(MCU_KEY, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_output(MCU_EN);
}
/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;
		int i = 0;
    // Initialize.

    log_init();

    buttons_leds_init(&erase_bonds);
		gpio_init();
    ws2812_begin();	
    power_management_init();
		simple_ble_init();   
    timers_init();
		saadc_init();
    timers_create();

		// Start execution.    
    advertising_start(); 
		timers_start();
	
//		neopixel_write(colors[2], 8);
		NRF_LOG_INFO("BLE Template Init.");

    // Enter main loop.
    for (;;)
    {
			nrf_delay_ms(5);
			if(BAT_chrg_flag == 0)
			{
				if(shut_on_flag == 1)
				{
					switch(BAT_pwr_level)
					{
						case 0:
							neopixel_write(colors[0], 8);
							break;
						case 1:
							neopixel_write(colors[9], 8);
							break;
						case 2:
							neopixel_write(colors[2], 8);
							break;
						case 3:
							neopixel_write(colors[3], 8);
							break;
						case 4:
							neopixel_write(colors[4], 8);
							break;
						case 5:
							neopixel_write(colors[5], 8);
							break;
						case 6:
							neopixel_write(colors[6], 8);
							break;
						case 7:
							neopixel_write(colors[7], 8);
							break;
						case 8:
							neopixel_write(colors[8], 8);
							break;
						default:
							break;
					}
				}
				else if(shut_on_flag == 0)
				{
					BAT_pwr_level = 0;
					neopixel_write(colors[0], 8);
				}
			}
			else if(BAT_chrg_flag == 1)
			{
				//充电中，全部显示红灯
					neopixel_write(colors[10], 8);
			}
			else if(BAT_chrg_flag == 2)
			{
				//充满电，全部显示绿灯
					neopixel_write(colors[8], 8);
			}	
			idle_state_handle();
    }
}


/**
 * @}
 */
