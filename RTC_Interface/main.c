/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
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
/**
 * @file
 * @brief File with example code presenting usage of drivers for TWIS slave and TWI in master mode
 *
 * @sa twi_master_with_twis_slave_example
 */

/**
 * @defgroup twi_master_with_twis_slave_example Example code presenting usage of drivers for TWIS slave and TWI in master mode
 *
 * This code presents the usage of two drivers:
 * - @ref nrf_twi_drv (in synchronous mode)
 * - @ref nrf_twis_drv (in asynchronous mode)
 *
 * On the slave, EEPROM memory is simulated.
 * The size of the simulated EEPROM is configurable in the config.h file.
 * Default memory value of the device is 320 bytes. It is simulated using internal RAM.
 * This RAM area is accessed only by simulated EEPROM so the rest of the application can access it
 * only using TWI commands via hardware configured pins.
 *
 * The selected memory chip uses a 7-bit constant address. Word to access is selected during
 * a write operation: first byte sent is used as the current address pointer.
 *
 * A maximum of an 8-byte page can be written in a single access.
 * The whole memory can be read in a single access.
 *
 * When the slave (simulated EEPROM) is initialized, it copies the given part of flash
 * (see EEPROM_SIM_FLASH_ADDRESS in config.h) into RAM (enabling the use of the slave for
 * bootloader use cases).
 *
 * Many variables like length of sequential writes/reads, TWI instance to use, endianness of
 * of slave bype addressing can be configured in config.h file
 *
 * Differences between real chip and simulated one:
 * 1. Simulated chip has practically 0 ms write time.
 *    This example does not poll the memory for readiness.
 * 2. During sequential read, when memory end is reached, there is no support for roll-over.
 *    It is recommended for master to assure that it does not try to read more than the page limits.
 *    If the master is not tracking this and trying to read after the page ends, then the slave will start to NACK
 *    for trying to over-read the memory. The master should end the transaction when slave starts NACKing, which could
 *    mean that the master has read the end of the page.
 * 3. It is possible to write a maximum of EEPROM_SIM_SEQ_WRITE_MAX_BYTES bytes in a single
 *    sequential write. However, in simulated EEPROM the whole address pointer is incremented.
 *    In a real device, writing would roll-over in memory page.
 *
 * On the master side, we communicate with that memory and allow write and read.
 * Simple commands via UART can be used to check the memory.
 *
 * Pins to short:
 * - @ref TWI_SCL_M - @ref EEPROM_SIM_SCL_S
 * - @ref TWI_SDA_M - @ref EEPROM_SIM_SDA_S
 *
 * Supported commands will be listed after Tab button press.
 * @{
 */

#ifdef _WIN32
#include <Windows.h>
#endif

#include "app_error.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "bsp.h"
#include <nrf.h>
//#include "nrf_drv_clock.h"
#include <config.h>
#include <ctype.h>
#include <nrf_drv_twi.h>
#include <nrf_gpio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <nrf_log.h>

const uint8_t rtc_write_addr7bit = 0xA2; // Address of RTC slave and write bit
const uint8_t rtc_read_addr7bit = 0xA3;  // Address of RTC slave and read bit

uint8_t reg_select[1]; // Pointer to first register in RTC slave to write to/read
                       // from

uint8_t reg_data[7]; // Pointer to data array to be written to/read from the RTC
                     // slave

APP_TIMER_DEF(t_identity);

/**
 * @brief App mode instance.
 *
 * App timer mode to ensure expiration of timer instance after a single run.
 *
 */
static const app_timer_mode_t mode = APP_TIMER_MODE_SINGLE_SHOT;

/**
 * @brief Timer instance creation.
 *
 * App timer initialization and creation of a timer instance 
 * to maintain the sleep routines.
 *
 */
static ret_code_t timer_create(void) {
  uint32_t ret;

  ret = app_timer_init();

  if (NRF_SUCCESS == ret) {
    return app_timer_create(&t_identity, mode, NULL);
  }
  return ret;
}

static ret_code_t timer_module(uint32_t ms_time) {

  uint32_t timeout_ticks;
  uint32_t ret;
  uint32_t i;
  //  Conversion of time in milliseconds to number of ticks
  timeout_ticks = APP_TIMER_TICKS(ms_time);

  ret = app_timer_start(t_identity, timeout_ticks, NULL);
  if (NRF_SUCCESS == ret) {
    do {
      i++;
    } while (i < timeout_ticks);
    return app_timer_stop(t_identity);
  }
  return ret;
}

/**
 * @brief Sleep routine definition.
 *
 * Sleep routine definition to maintain the configuration of the I2C/TWI pins
 * for fixed time periods.
 *
 */
static void sleep_routine(uint32_t ms_time) {

  //  Disable configuration for the SCL pin
  nrf_gpio_cfg(TWI_SCL_M, NULL, NRF_GPIO_PIN_INPUT_DISCONNECT,
      NRF_GPIO_PIN_PULLUP, NULL, NRF_GPIO_PIN_NOSENSE);

  //  Disable configuration for the SDA pin
  nrf_gpio_cfg(TWI_SDA_M, NULL, NRF_GPIO_PIN_INPUT_DISCONNECT,
      NRF_GPIO_PIN_PULLUP, NULL, NRF_GPIO_PIN_NOSENSE);

  //  Configuration maintainted for total sleep time
  //  Timer module invoked
  timer_module(ms_time);

  //  Reset SCL pin to default state after sleep routine
  nrf_gpio_cfg_default(TWI_SCL_M);

  //  Reset SDA pin to default state after sleep routine
  nrf_gpio_cfg_default(TWI_SDA_M);
}

/**
 * @brief TWI master instance.
 *
 * Instance of TWI master driver that will be used for communication with external
 * RTC module (RV-8263-C7).
 */
static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(MASTER_TWI_INST);

/**
 * @brief Initialize the master TWI.
 *
 * Function used to initialize the master TWI interface that would communicate with external
 * RTC module (RV-8263-C7).
 *
 * @return NRF_SUCCESS or the reason of failure.
 */
//  Utility Functions and Definitions (Platform) --> APP_IRQ_PRIORITY_HIGH = 2
static ret_code_t twi_master_init(void) {
  ret_code_t ret;
  const nrf_drv_twi_config_t config =
      {
          .scl = TWI_SCL_M,
          .sda = TWI_SDA_M,
          .frequency = NRF_DRV_TWI_FREQ_400K, // Maximum frequency supported by RTC
          .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
          .clear_bus_init = false};

  ret = nrf_drv_twi_init(&m_twi_master, &config, NULL, NULL);

  if (NRF_SUCCESS == ret) {
    nrf_drv_twi_enable(&m_twi_master);
  }
  return ret;
}

/**
 * @brief Write data to external RTC
 *
 * Function uses the TWI interface to write data into the external
 * RTC module (RV-8263-C7).
 *
 * @param     rtc_addr  Address of external RTC to write to.
 * @param[in] tx_data Pointer to the data to send to external RTC.
 * @param     length  Number of data bytes to send to external RTC.
 * @param     no_stop Input to ascertain whether to stop the TWI/I2C bus after the transfer of data.       
 *
 * @return NRF_SUCCESS or reason of error.
 */
static ret_code_t ext_rtc_write(uint8_t rtc_addr, uint8_t const *tx_data, uint8_t length, bool no_stop) {
  ret_code_t ret;
  /* If the user attempts to write more bytes than available, then an error message is returned */
  if (length < sizeof(&tx_data)) {
    ret = nrf_drv_twi_tx(&m_twi_master, rtc_addr, tx_data, length, no_stop);
    return ret;
  }
  return NRF_ERROR_INVALID_LENGTH;
}

/**
 * @brief Read data from external RTC
 *
 * Function uses the TWI interface to read data from the external
 * RTC module (RV-8263-C7).
 *
 * @param     rtc_addr  Address of external RTC to write to.
 * @param[in] rx_data Pointer to the external RTC register to receive data from.
 * @param     length  Number of data bytes to receive from the external RTC.
 * @param     no_stop Input to ascertain whether to stop the TWI/I2C bus after the transfer of data.       
 *
 * @return NRF_SUCCESS or reason of error.
 */
static ret_code_t ext_rtc_read(uint8_t rtc_addr, uint8_t *rx_data, uint8_t length) {
  ret_code_t ret;
  uint8_t end_reg_addr = 0x11;
  uint8_t begin_reg_addr = 0x00;
  /* If the user attempts to read more than the available register data from external RTC, then an error message is returned */
  if (length <= end_reg_addr - begin_reg_addr + 1) {
    ret = nrf_drv_twi_rx(&m_twi_master, rtc_addr, rx_data, length);
    return ret;
  }
  return NRF_ERROR_INVALID_LENGTH;
}

/**
 *  The main function to execute the transaction process with the external RTC
 */
int main(void) {

  bool no_stop = true;

  ret_code_t err_code;

  int sleep_status;

  int num_delays;

  //  Set the current time from the terminal manually by user
  struct tm t;
  printf("Enter current date and time:\n");
  printf("YYYY MM DD HH MM SS WD[enter]\n");
  scanf("%d %d %d %d %d %d %d", &t.tm_year, &t.tm_mon, &t.tm_mday, &t.tm_hour,
      &t.tm_min, &t.tm_sec, &t.tm_wday);

  //  Adjust for tm structure required values
  t.tm_year = t.tm_year - 1900;
  t.tm_mon = t.tm_mon - 1;

  //  Display the time entered manually by use
  char system_time[50] = "";
  strcpy(system_time, asctime(&t));
  printf("Epoch Time = %s", system_time);

  /* The external RTC module (RV-8263-C7) is initialized with the manually
   * entered system time when activated */

  //  First register in RTC slave to write to
  reg_select[0] = 0x04;

  //  The 8-bit data to be inserted in each time register starting from 004h
  //  The register counter is auto incremented on an 8-bit data push operation
  reg_data[0] = ((uint8_t)t.tm_sec);
  reg_data[1] = ((uint8_t)t.tm_min);
  reg_data[2] = ((uint8_t)t.tm_hour);
  reg_data[3] = ((uint8_t)t.tm_mday);
  reg_data[4] = ((uint8_t)t.tm_wday);
  reg_data[5] = ((uint8_t)t.tm_mon + 1);    // Range in 1 - 12
  reg_data[6] = ((uint8_t)t.tm_year - 100); // Range in 00 - 99

  err_code = twi_master_init();
  APP_ERROR_CHECK(err_code);

  //err_code = nrf_drv_clock_init();
  //APP_ERROR_CHECK(err_code);
  //nrf_drv_clock_lfclk_request(NULL);

  //err_code = app_timer_init();
  //APP_ERROR_CHECK(err_code);

  err_code = ext_rtc_write(rtc_write_addr7bit, reg_select, sizeof(reg_select), no_stop);
  APP_ERROR_CHECK(err_code);

  err_code = ext_rtc_write(rtc_write_addr7bit, reg_data, sizeof(reg_data), no_stop = false);
  APP_ERROR_CHECK(err_code);

  printf("The system time has been successfully initialized onto the external RTC");

  //  Transaction break for 1 minute
  timer_create();
  sleep_routine(60000);

  for (num_delays = 1; num_delays <= 10; num_delays++) {

    //  Enabling the TWI instance after delay
    nrf_drv_twi_enable(&m_twi_master);

    //  Write 8-bit data first to select the register address 0040h
    err_code = ext_rtc_write(rtc_write_addr7bit, reg_select, sizeof(reg_select), no_stop);
    APP_ERROR_CHECK(err_code);

    //  The next 8-bit data array is the real time data to be fetched from the RTC slave
    err_code = ext_rtc_read(rtc_read_addr7bit, reg_data, sizeof(reg_data));
    APP_ERROR_CHECK(err_code);

    //  Disabling the TWI instance
    nrf_drv_twi_disable(&m_twi_master);

    t.tm_sec = reg_data[0];
    t.tm_min = reg_data[1];
    t.tm_hour = reg_data[2];
    t.tm_mday = reg_data[3];
    t.tm_wday = reg_data[4];
    t.tm_mon = reg_data[5] - 1;    // Range in 00 - 11
    t.tm_year = reg_data[6] + 100; // Years since 1900

    // Display the time from RTC
    char rtc_time[50] = "";
    strcpy(rtc_time, asctime(&t));
    printf("Epoch Time = %s", rtc_time);

    sleep_routine(5000); // Delay of 5 seconds to display time from
                         // RTC for 10 iterations
  }

  //Uninitializing the TWI instance at the end of the transaction session
  nrf_drv_twi_uninit(&m_twi_master);
}

/** @} */ /* End of group nRF5 DK interfacing with extrenal RTC module ((RV-8263-C7) */