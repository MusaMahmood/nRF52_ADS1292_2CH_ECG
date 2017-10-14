/* Copyright (c) 2016 Musa Mahmood
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*#ifdef __cplusplus
extern "C" {
#endif*/

#include "ads1291-2.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "ble_eeg.h"
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
#include "spi_master_fast.h"
#endif
/**headers for µs delay:*/
#include "compiler_abstraction.h"
#include "nrf.h"
#include <stdio.h>

uint8_t ads1291_2_default_regs[] = {
    ADS1291_2_REGDEFAULT_CONFIG1,
    ADS1291_2_REGDEFAULT_CONFIG2,
    ADS1291_2_REGDEFAULT_LOFF,
    ADS1291_2_REGDEFAULT_CH1SET,
    ADS1291_2_REGDEFAULT_CH2SET,
    ADS1291_2_REGDEFAULT_RLD_SENS,
    ADS1291_2_REGDEFAULT_LOFF_SENS,
    ADS1291_2_REGDEFAULT_LOFF_STAT,
    ADS1291_2_REGDEFAULT_RESP1,
    ADS1291_2_REGDEFAULT_RESP2,
    ADS1291_2_REGDEFAULT_GPIO};
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
#else
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(0);
#endif
#define RX_DATA_LEN 12
static uint8_t rx_data[RX_DATA_LEN];
static volatile bool spi_xfer_done;

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const *p_event,
    void *p_context) {
  spi_xfer_done = true;
}
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
#else
#endif
/**@INITIALIZE SPI INSTANCE */
void ads_spi_init(void) {
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
#else
  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
  //SCLK = 1MHz is right speed because fCLK = (1/2)*SCLK, and fMOD = fCLK/4, and fMOD MUST BE 128kHz. Do the math.
  spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
  spi_config.irq_priority = APP_IRQ_PRIORITY_HIGHEST; //APP_IRQ_PRIORITY_HIGHEST;
  spi_config.mode = NRF_DRV_SPI_MODE_1;               //CPOL = 0 (Active High); CPHA = TRAILING (1)
  spi_config.miso_pin = ADS1291_2_MISO_PIN;
  spi_config.sck_pin = ADS1291_2_SCK_PIN;
  spi_config.mosi_pin = ADS1291_2_MOSI_PIN;
  spi_config.ss_pin = ADS1291_2_SS_PIN;
  spi_config.orc = 0x55;
  APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
#endif
}

void ads_spi_uninit(void) {
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1

#else
  nrf_drv_spi_uninit(&spi);
  NRF_LOG_INFO(" SPI UNinitialized \r\n");
#endif
}

void ads_spi_init_with_sample_freq(uint8_t spi_sclk) {
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  SPI_config_t spi_config = {.pin_SCK = ADS1291_2_SCK_PIN,
      .pin_MOSI = ADS1291_2_MOSI_PIN,
      .pin_MISO = ADS1291_2_MISO_PIN,
      .pin_CSN = ADS1291_2_SS_PIN,
      .frequency = SPI_FREQ_4MBPS,
      .config.fields.mode = 1,
      .config.fields.bit_order = SPI_BITORDER_MSB_LSB};

  if (spi_sclk == 0) {
    spi_config.frequency = SPI_FREQ_500KBPS;
    spi_master_init(SPI0, &spi_config);
  } else if (spi_sclk == 4 || spi_sclk == 8) {
    spi_config.frequency = SPI_FREQ_4MBPS;
    spi_set_frequency(&spi_config);
  }

#else
  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  switch (spi_sclk) {
  case 0:
    spi_config.frequency = NRF_DRV_SPI_FREQ_500K;
    break;
  case 1:
    spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
    break;
  case 2:
    spi_config.frequency = NRF_DRV_SPI_FREQ_2M;
    break;
  case 4:
    spi_config.frequency = NRF_DRV_SPI_FREQ_4M;
    break;
  case 8:
    spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
    break;
  default:
    break;
  }
  spi_config.irq_priority = APP_IRQ_PRIORITY_HIGHEST; //APP_IRQ_PRIORITY_HIGHEST;
  spi_config.mode = NRF_DRV_SPI_MODE_1;               //CPOL = 0 (Active High); CPHA = TRAILING (1)
  spi_config.miso_pin = ADS1291_2_MISO_PIN;
  spi_config.sck_pin = ADS1291_2_SCK_PIN;
  spi_config.mosi_pin = ADS1291_2_MOSI_PIN;
  spi_config.ss_pin = ADS1291_2_SS_PIN;
  spi_config.orc = 0x55;
  APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
  NRF_LOG_INFO(" SPI Initialized @ %d MHz\r\n", spi_sclk);
#endif
}

/* SYSTEM CONTROL FUNCTIONS **********************************************************************************************************************/

void ads1291_2_init_regs(void) {
  /**@TODO: REWRITE THIS FUNCTION. Not sure it works correctly.*/
  uint8_t i = 0;
  uint8_t num_registers = 11;
  uint8_t txrx_size = num_registers + 2;
  uint8_t tx_data_spi[txrx_size]; //Size = 14 bytes
  uint8_t rx_data_spi[txrx_size]; //Size = 14 bytes
  uint8_t opcode_1 = 0x41;
  for (i = 0; i < txrx_size; i++) {
    tx_data_spi[i] = 0; // Set array to zero.
    rx_data_spi[i] = 0; // Set array to zero.
  }
  // Set first byte to opcode WREG | Starting Address, which is = 0x41
  //tx_data_spi[0] = ADS1291_2_OPC_WREG | ADS1291_2_REGADDR_CONFIG1;
  tx_data_spi[0] = opcode_1;
  tx_data_spi[1] = num_registers - 1; //is the number of registers to write ? 1. (OPCODE2)
  //fill remainder of tx with commands:
  //this should be memcpy
  memcpy(&tx_data_spi[2], &ads1291_2_default_regs[0], num_registers);
  //  for (i = 0; i < num_registers; i++) {
  //    tx_data_spi[i + 2] = ads1291_2_default_regs[i];
  //  }
  spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  //  spi_master_tx(SPI0, num_registers + 2, tx_data_spi);
  spi_master_tx_rx(SPI0, num_registers + 2, tx_data_spi, rx_data_spi);
#else
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data_spi, num_registers + 2, rx_data_spi, num_registers + 2));
  nrf_delay_ms(10);
  while (!spi_xfer_done) {
    __WFE();
  }
  NRF_LOG_INFO(" Power-on reset and initialization procedure..\r\n");
#endif
}

void ads1291_2_standby(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1291_2_OPC_STANDBY;
  spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  spi_master_tx_rx(SPI0, 1, &tx_data_spi, &rx_data_spi);
#else
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));

  while (!spi_xfer_done) {
    __WFE();
  }
#endif
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" ADS1292 placed in standby mode...\r\n");
#endif
}

void ads1291_2_wake(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1291_2_OPC_WAKEUP;
  spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  spi_master_tx_rx(SPI0, 1, &tx_data_spi, &rx_data_spi);
#else
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));
  while (!spi_xfer_done) {
    __WFE();
  }
#endif
  nrf_delay_ms(10); // Allow time to wake up - 10ms
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" ADS1292 Wakeup..\r\n");
#endif
}

void ads1291_2_soft_start_conversion(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1291_2_OPC_START;
  spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  spi_master_tx_rx(SPI0, 1, &tx_data_spi, &rx_data_spi);
#else
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));
  while (!spi_xfer_done) {
    __WFE();
  }
#endif
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" Start ADC conversion..\r\n");
#endif
}

void ads1291_2_stop_rdatac(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1291_2_OPC_SDATAC;
  spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  spi_master_tx_rx(SPI0, 1, &tx_data_spi, &rx_data_spi);
#else
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));

  while (!spi_xfer_done) {
    __WFE();
  }
#endif
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" Continuous Data Output Disabled..\r\n");
#endif
}

void ads1291_2_start_rdatac(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1291_2_OPC_RDATAC;
  spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  spi_master_tx_rx(SPI0, 1, &tx_data_spi, &rx_data_spi);
#else
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));
  while (!spi_xfer_done) {
    __WFE();
  }
#endif
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" Continuous Data Output Enabled..\r\n");
#endif
}

void ads1291_2_powerdn(void) {
  nrf_gpio_pin_clear(ADS1291_2_PWDN_PIN);
  nrf_delay_ms(10);
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" ADS1292 POWERED DOWN..\r\n");
#endif
}

void ads1291_2_powerup(void) {
  nrf_gpio_pin_set(ADS1291_2_PWDN_PIN);
  nrf_delay_ms(1000); // Allow time for power-on reset
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" ADS1292 POWERED UP...\r\n");
#endif
}

/* DATA RETRIEVAL FUNCTIONS **********************************************************************************************************************/
void ads1291_2_check_id(void) {
  uint8_t device_id;
#if defined(ADS1291)
  device_id = ADS1291_DEVICE_ID;
#elif defined(ADS1292)
  device_id = ADS1292_DEVICE_ID;
#endif
  uint8_t device_id_reg_value;
  uint8_t tx_data_spi[6];
  uint8_t rx_data_spi[6];
  //  memset(rx_data_spi, 0, 7);
  tx_data_spi[0] = 0x20; // First command byte = 001r rrrr (r rrrr = register start address)
  tx_data_spi[1] = 0x01; // Intend to read 1 byte: (Bytes to read)-1 = 0
  tx_data_spi[2] = 0x00; //This will be replaced by Reg Data
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  spi_master_tx_rx(SPI0, 6, tx_data_spi, rx_data_spi);
#else
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data_spi, 2, rx_data_spi, 6));
  while (!spi_xfer_done) {
    __WFE();
  }
  nrf_delay_ms(10);
#endif

//NOTE: CHANGES FROM [2] to [3] for EASY DMA
#if SPI0_USE_EASY_DMA == 1
  device_id_reg_value = rx_data_spi[3];
#else
  device_id_reg_value = rx_data_spi[2];
#endif
  if (device_id_reg_value == device_id) {
    NRF_LOG_INFO("Check ID (match): 0x%x \r\n", device_id_reg_value);
  } else {
    NRF_LOG_INFO("Check ID (not match): 0[0x%x] 1[0x%x] 2[0x%x] \r\n", rx_data_spi[0], rx_data_spi[1], rx_data_spi[2]);
    NRF_LOG_INFO("Check ID (not match): 3[0x%x] 4[0x%x] 5[0x%x] \r\n", rx_data_spi[3], rx_data_spi[4], rx_data_spi[5]);
  }
}

void get_eeg_voltage_array_2ch(ble_eeg_t *p_eeg) {
  uint8_t rx_data[9];
//  memset(rx_data, 0, RX_DATA_LEN);
  spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  spi_master_tx_rx(SPI0, 9, rx_data, rx_data);
#else
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, rx_data, 9));
  while (!spi_xfer_done)
    __WFE();
#endif
  //  if (((rx_data[0] + rx_data[1] + rx_data[2]) == 0xC0) && ((rx_data[9] + rx_data[10] + rx_data[11]) == 0x00)) {
  p_eeg->eeg_ch1_buffer[p_eeg->eeg_ch1_count] = rx_data[3];
  p_eeg->eeg_ch1_buffer[p_eeg->eeg_ch1_count + 1] = rx_data[4];
  p_eeg->eeg_ch1_buffer[p_eeg->eeg_ch1_count + 2] = rx_data[5];
  p_eeg->eeg_ch2_buffer[p_eeg->eeg_ch1_count++] = rx_data[6];
  p_eeg->eeg_ch2_buffer[p_eeg->eeg_ch1_count++] = rx_data[7];
  p_eeg->eeg_ch2_buffer[p_eeg->eeg_ch1_count++] = rx_data[8];
  //  }
}

void get_eeg_voltage_array_2ch_low_resolution(ble_eeg_t *p_eeg) {
//  memset(rx_data, 0, RX_DATA_LEN);
  spi_xfer_done = false;
//  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, rx_data, 8));
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
uint8_t tx_data[9];
  spi_master_tx_rx(SPI0, 9, tx_data, rx_data);
#else
  nrf_drv_spi_transfer(&spi, NULL, NULL, rx_data, 9);
  while (!spi_xfer_done)
    __WFE();
#endif
  p_eeg->eeg_ch1_buffer[p_eeg->eeg_ch1_count] = rx_data[3];
  p_eeg->eeg_ch1_buffer[p_eeg->eeg_ch1_count + 1] = rx_data[4];
//  p_eeg->eeg_ch1_buffer[p_eeg->eeg_ch1_count + 2] = rx_data[5];
  p_eeg->eeg_ch2_buffer[p_eeg->eeg_ch1_count] = rx_data[6];
  p_eeg->eeg_ch2_buffer[p_eeg->eeg_ch1_count + 1] = rx_data[7];
//  p_eeg->eeg_ch2_buffer[p_eeg->eeg_ch1_count + 2] = rx_data[8];
  
}