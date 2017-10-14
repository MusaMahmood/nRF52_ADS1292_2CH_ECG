/** @file
 *
 * @brief Functions for initializing and controlling Texas Instruments ADS1291/2 analog front-end.
 */

#ifndef ADS1291_2_H__
#define ADS1291_2_H__

#include "ble_eeg.h"
#include "nrf_drv_spi.h"
#include <stdint.h>

//#ifdef __cplusplus
//extern "C" {
//#endif

#ifdef BOARD_PCA10028
#define ADS1291_2_PWDN_PIN 4  /**< ADS1291 power-down/reset pin - A3 on Arduino */
#define ADS1291_2_DRDY_PIN 15 /**< ADS1291 data ready interrupt pin - D3 on Arduino */
#elif defined(BOARD_2CH_ECG_RAT)
#define ADS1291_2_PWDN_PIN 16 /**< ADS1291 power-down/reset pin */
#define ADS1291_2_DRDY_PIN 11 /**< ADS1291 data ready interrupt pin */
#define ADS1291_2_SCK_PIN 13
#define ADS1291_2_MOSI_PIN 14
#define ADS1291_2_MISO_PIN 12
#define ADS1291_2_SS_PIN 15
#endif

/**
 *	\brief Error codes for interacting with the ADS1291_2.
 *
 */
typedef enum {
  ADS1291_2_STATUS_OK = 0,         ///< No error.
  ADS1291_2_ERROR_SPI_TIMEOUT = 1, ///< SPI timed out. Check SPI configuration and hardware connections.
                                   /* Expand with other codes if desired */
} ads1291_2_error_t;

#define ADS1291_2_NUM_REGS 12

/**
 *	\brief ADS1291_2 register addresses.
 *
 * Consult the ADS1291/2 datasheet and user's guide for more information.
 */
#define ADS1291_2_REGADDR_ID 0x00        ///< Chip ID register. Read-only.
#define ADS1291_2_REGADDR_CONFIG1 0x01   ///< Configuration register 1. Controls conversion mode and data rate.
#define ADS1291_2_REGADDR_CONFIG2 0x02   ///< Configuration register 2. Controls LOFF comparator, reference, CLK pin, and test signal.
#define ADS1291_2_REGADDR_LOFF 0x03      ///< Lead-off control register. Controls lead-off frequency, magnitude, and threshold.
#define ADS1291_2_REGADDR_CH1SET 0x04    ///< Channel 1 settings register. Controls channel 1 input mux, gain, and power-down.
#define ADS1291_2_REGADDR_CH2SET 0x05    ///< Channel 2 settings register (ADS1292x only). Controls channel 2 input mux, gain, and power-down.
#define ADS1291_2_REGADDR_RLD_SENS 0x06  ///< RLD sense selection. Controls PGA chop frequency, RLD buffer, and channels for RLD derivation.
#define ADS1291_2_REGADDR_LOFF_SENS 0x07 ///< Lead-off sense selection. Controls current direction and selects channels that will use lead-off detection.
#define ADS1291_2_REGADDR_LOFF_STAT 0x08 ///< Lead-off status register. Bit 6 controls clock divider. For bits 4:0, 0: lead on, 1: lead off.
#define ADS1291_2_REGADDR_RESP1 0x09     ///< Respiration 1 (ADS1292R only). See datasheet.
#define ADS1291_2_REGADDR_RESP2 0x0A     ///< Respiration 2. Controls offset calibration, respiration modulator freq, and RLDREF signal source.
#define ADS1291_2_REGADDR_GPIO 0x0B      ///< GPIO register. Controls state and direction of the ADS1291_2 GPIO pins.
/**
 *	\brief ADS1291/2 SPI communication opcodes.
 *	
 * Consult the ADS1291/2 datasheet and user's guide for more information.
 * For RREG and WREG opcodes, the first byte (opcode) must be ORed with the address of the register to be read/written. 
 * The command is completed with a second byte 000n nnnn, where n nnnn is (# registers to read) - 1.
 */
#define ADS1291_2_OPC_WAKEUP 0x02    ///< Wake up from standby.
#define ADS1291_2_OPC_STANDBY 0x04   ///< Enter standby.
#define ADS1291_2_OPC_RESET 0x06     ///< Reset all registers.
#define ADS1291_2_OPC_START 0x08     ///< Start data conversions.
#define ADS1291_2_OPC_STOP 0x0A      ///< Stop data conversions.
#define ADS1291_2_OPC_OFFSETCAL 0x1A ///< Calibrate channel offset. RESP2.CALIB_ON must be 1. Execute after every PGA gain change.

#define ADS1291_2_OPC_RDATAC 0x10 ///< Read data continuously (registers cannot be read or written in this mode).
#define ADS1291_2_OPC_SDATAC 0x11 ///< Stop continuous data read.
#define ADS1291_2_OPC_RDATA 0x12  ///< Read single data value.

#define ADS1291_2_OPC_RREG 0x20 ///< Read register value. System must not be in RDATAC mode.
#define ADS1291_2_OPC_WREG 0x40 ///< Write register value. System must not be in RDATAC mode.

/* ID REGISTER ********************************************************************/

/**
 *  \brief Factory-programmed device ID for ADS1291/2.
 */
#define ADS1291_DEVICE_ID 0x52
#define ADS1292_DEVICE_ID 0x53
#define ADS1292R_DEVICE_ID 0x73

/* DEFAULT REGISTER VALUES ********************************************************/

#define ADS1291_2_REGDEFAULT_CONFIG1 0x06   ///< Continuous conversion, data rate = 125->8kSPS
#define ADS1291_2_REGDEFAULT_CONFIG2 0xA3   ///< LOFF off, REFBUF on, VREF=2.42, CLK_EN=0, INT_TEST=1, TEST_FREQ @ 1Hz
#define ADS1291_2_REGDEFAULT_LOFF 0x00      ///< 95%/5% LOFF comparator threshold, DC lead-off at 6 nA
#define ADS1291_2_REGDEFAULT_CH1SET 0x60    ///< Channel on, G=12, normal electrode
#define ADS1291_2_REGDEFAULT_CH2SET 0x60    ///< Channel off, G=1, input short
#define ADS1291_2_REGDEFAULT_RLD_SENS 0x23  ///< Chop @ fmod/16, RLD buffer on, LOFF off, RLD derivation from CH1 P+N
#define ADS1291_2_REGDEFAULT_LOFF_SENS 0x00 ///< Current source @ IN+, sink @ IN-, all LOFF channels disconnected
#define ADS1291_2_REGDEFAULT_LOFF_STAT 0x00 ///< Fmod = fclk/4 (for fclk = 512 kHz)
#define ADS1291_2_REGDEFAULT_RESP1 0x02     ///< Resp measurement disabled
#define ADS1291_2_REGDEFAULT_RESP2 0x07     ///< Offset calibration disabled, RLD internally generated
#define ADS1291_2_REGDEFAULT_GPIO 0x00      ///< All GPIO set to output, logic low
/**@TYPEDEFS: */

/**************************************************************************************************************************************************
*               Prototypes                                                                                                                        *
**************************************************************************************************************************************************/
void ads_spi_init(void);
/**
 *	\brief Initialize the ADS1291/2.
 *
 * This function performs the power-on reset and initialization procedure documented on page 58 of the
 * ADS1291_2 datasheet, up to "Send SDATAC Command."
 *
 * \pre Requires spi_master.h from the nRF51 SDK.
 * \return Zero if successful, or an error code if unsuccessful.
 */
void ads1291_2_init_regs(void);

/**
 *	\brief Read a single register from the ADS1291_2.
 *
 * This function sends the RREG opcode, logical OR'd with the specified register address, and
 * writes the obtained register value to a variable. This command will have no effect if the 
 * device is in continuous read mode.
 *
 * \pre Requires spi.h from the Atmel Software Framework.
 * \param reg_addr The register address of the register to be read.
 * \param num_to_read The number of registers to read, starting at @param(reg_addr).
 * \param read_reg_val_ptr Pointer to the variable to store the read register value(s).
 * \return Zero if successful, or an error code if unsuccessful.
 */
void ads1291_2_rreg(uint8_t reg_addr, uint8_t num_to_read, uint8_t *read_reg_val_ptr);

/**
 *	\brief Write a single register on the ADS1291_2.
 *
 * This function sends the WREG opcode, logical OR'd with the specified register address, and
 * then writes the specified value to that register. This command will have no effect if the 
 * device is in continuous read mode.
 *
 * \pre Requires spi_master.h from the nRF51 SDK.
 * \param reg_addr The register address of the register to be written.
 * \param num_to_write The number of registers to write, starting at @param(reg_addr).
 * \param write_reg_val_ptr The value(s) to be written to the specified register.
 * \return Zero if successful, or an error code if unsuccessful.
 */
void ads1291_2_wreg(uint8_t reg_addr, uint8_t num_to_write, uint8_t *write_reg_val_ptr);

/**
 *	\brief Put the ADS1291_2 in standby mode.
 *
 * This function sends the STANDBY opcode to the ADS1291_2. This places the device in a low-power mode by
 * shutting down all parts of the circuit except for the reference section. Return from standby using 
 * ads1291_2_wake(). Do not send any other commands during standby mode.
 *
 * \pre Requires spi_master.h from the nRF51 SDK.
 * \return Zero if successful, or an error code if unsuccessful.
 */
void ads1291_2_standby(void);

/**
 *	\brief Wake the ADS1291_2 from standby mode.
 *
 * This function sends the WAKEUP opcode to the ADS1291_2. This returns the device to normal operation 
 * after entering standby mode using ads1291_2_standby(). The host must wait 4 ADS1291_2 clock cycles
 * (approximately 2 us at 2.048 MHz) after sending this opcode to allow the device to wake up. 
 *
 * \pre Requires spi_master.h from the nRF51 SDK.
 * \return Zero if successful, or an error code if unsuccessful.
 */
void ads1291_2_wake(void);

#ifdef ADS1291_2_START_PIN
/**
 *	\brief Start analog-to-digital conversion on the ADS1291/2 by setting the START pin.
 *
 * This function pulls the START pin high, which begins analog-to-digital conversion on the ADS1291/2.
 * If conversions are already in progress, this has no effect. Pulling the START pin low 
 * ads1291_2_hard_stop_conversion() must follow this command by at least 4 ADS1291/2 clock cycles 
 * (approximately 8 us at 512 kHz). This command should not be used if ads1291_2_soft_start_conversion() 
 * has been used but has not yet been followed by ads1291_2_soft_stop_conversion().
 *
 * \pre Requires nrf_gpio.h from the nRF51 SDK.
 */
void ads1291_2_hard_start_conversion(void);

/**
 *	\brief Stop analog-to-digital conversion on the ADS1291/2 by clearing the START pin.
 *
 * This function pulls the START pin low, which halts analog-to-digital conversion on the ADS1291/2.
 * This command must follow pulling the START pin high ads1291_2_hard_start_conversion() by at least 
 * 4 ADS1291/2 clock cycles (approximately 8 us at 512 kHz).
 *
 * \pre Requires nrf_gpio.h from the nRF51 SDK.
 */
void ads1291_2_hard_stop_conversion(void);
#endif // ADS1291_2_START_PIN

/**
 *	\brief Start analog-to-digital conversion on the ADS1291/2 using the START opcode.
 *
 * This function sends the START opcode, which begins analog-to-digital conversion on the ADS1291/2.
 * It is provided in case the START pin is not available for use in the user application.
 * If conversions are already in progress, this has no effect. The STOP command ads1291_2_soft_stop_conversion()
 * must follow this command by at least 4 ADS1291/2 clock cycles (approximately 8 us at 512 kHz). 
 * This command should not be used if ads1291_2_hard_start_conversion() has not yet been followed by 
 * ads1291_2_hard_stop_conversion().
 *
 * \pre Requires spi_master.h from the nRF51 SDK.
 * \return Zero if successful, or an error code if unsuccessful.
 */
void ads1291_2_soft_start_conversion(void);

/**
 *	\brief Stop analog-to-digital conversion on the ADS1291/2 using the STOP opcode.
 *
 * This function sends the STOP opcode, which halts analog-to-digital conversion on the ADS1291/2.
 * It is provided in case the START pin is not available for use in the user application.
 * This command must follow a START opcode ads1291_2_soft_start_conversion() by at least 4 ADS1291/2
 * clock cycles (approximately 8 us at 512 kHz). This command should not be used if 
 * ads1291_2_hard_start_conversion() has not yet been followed by ads1291_2_hard_stop_conversion().
 *
 * \pre Requires spi_master.h from the nRF51 SDK.
 * \return Zero if successful, or an error code if unsuccessful.
 */
void ads1291_2_soft_stop_conversion(void);

/**
 *	\brief Enable continuous data output.
 *
 * This function sends the RDATAC opcode, which makes conversion data immediately available
 * to the host as soon as the DRDY pin goes low. The host need only send SCLKs to retrieve
 * the data, rather than starting with a RDATA command. Registers cannot be read or written
 * (RREG or WREG) in this mode. Cancel continuous read mode using ads1291_2_stop_rdatac().
 *
 * \pre Requires spi_master.h from the nRF51 SDK.
 * \return Zero if successful, or an error code if unsuccessful.
 */
//uint32_t ads1291_2_start_rdatac(void);
void ads1291_2_start_rdatac(void);
/**
 *	\brief Disable continuous data output.
 *
 * This function sends the SDATAC opcode, which exits the continuous data mode.
 *
 * \pre Requires spi_master.h from the nRF51 SDK.
 * \return Zero if successful, or an error code if unsuccessful.
 */
void ads1291_2_stop_rdatac(void);

void ads1291_2_calibrate(void);

void ads1291_2_powerdn(void);

void ads1291_2_powerup(void);

void ads1291_2_soft_reset(void);

void ads1291_2_check_id(void);

/**@DATA RETRIEVAL FUNCTIONS****/

void set_sampling_rate(uint8_t sampling_rate);

void get_eeg_voltage_array_2ch(ble_eeg_t *p_eeg);

void get_eeg_voltage_array_2ch_low_resolution(ble_eeg_t *p_eeg);

void ads_spi_uninit(void);

void ads_spi_init_with_sample_freq(uint8_t spi_sclk);
//#ifdef __cplusplus
//}
//#endif
#endif // ADS1291_2_H__