use ::libc;
extern "C" {
    #[no_mangle]
    static mut rssiSource: rssiSource_e;
    #[no_mangle]
    fn setRssi(rssiValue: uint16_t, source: rssiSource_e);
    // Flush the RX FIFO buffer.
    // Flush the TX FIFO buffer.
    // Reset real time clock.
    // No operation. May be used to pad strobe commands to two
         // bytes for simpler software.
    //----------------------------------------------------------------------------------
// Chip Status Byte
//----------------------------------------------------------------------------------
    // Bit fields in the chip status byte
    // Chip states
    //----------------------------------------------------------------------------------
// Other register bit fields
//----------------------------------------------------------------------------------
    #[no_mangle]
    fn cc2500ReadFifo(dpbuffer: *mut uint8_t, len: uint8_t) -> uint8_t;
    #[no_mangle]
    fn cc2500ReadReg(reg: uint8_t) -> uint8_t;
    #[no_mangle]
    fn cc2500Strobe(address: uint8_t);
    #[no_mangle]
    fn cc2500WriteReg(address: uint8_t, data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn cc2500Reset() -> uint8_t;
    // declare available IO pins. Available pins are specified per target
    // unimplemented
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn IORead(io: IO_t) -> bool;
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    /*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
    #[no_mangle]
    fn delayMicroseconds(us: timeUs_t);
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn millis() -> timeMs_t;
    #[no_mangle]
    fn writeEEPROM();
    #[no_mangle]
    fn frSkyDSetRcData(rcData: *mut uint16_t, payload: *const uint8_t);
    #[no_mangle]
    fn frSkyDInit();
    #[no_mangle]
    fn frSkyDHandlePacket(packet: *mut uint8_t, protocolState_0: *mut uint8_t)
     -> rx_spi_received_e;
    #[no_mangle]
    fn frSkyXSetRcData(rcData: *mut uint16_t, payload: *const uint8_t);
    #[no_mangle]
    fn frSkyXInit();
    #[no_mangle]
    fn frSkyXHandlePacket(packet: *mut uint8_t, protocolState_0: *mut uint8_t)
     -> rx_spi_received_e;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  ******************************************************************************
  * @file    stm32f30x_gpio.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the GPIO 
  *          firmware library. 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F30x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup GPIO
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup Configuration_Mode_enumeration 
  * @{
  */
pub type C2RustUnnamed = libc::c_uint;
/* !< GPIO Analog In/Out Mode      */
/* !< GPIO Alternate function Mode */
pub const GPIO_Mode_AN: C2RustUnnamed = 3;
/* !< GPIO Output Mode */
pub const GPIO_Mode_AF: C2RustUnnamed = 2;
/* !< GPIO Input Mode */
pub const GPIO_Mode_OUT: C2RustUnnamed = 1;
pub const GPIO_Mode_IN: C2RustUnnamed = 0;
/* *
  * @}
  */
/* * @defgroup Output_type_enumeration
  * @{
  */
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GPIO_OType_OD: C2RustUnnamed_0 = 1;
pub const GPIO_OType_PP: C2RustUnnamed_0 = 0;
/* *
  * @}
  */
/* * @defgroup Configuration_Pull-Up_Pull-Down_enumeration 
  * @{
  */
pub type C2RustUnnamed_1 = libc::c_uint;
pub const GPIO_PuPd_DOWN: C2RustUnnamed_1 = 2;
pub const GPIO_PuPd_UP: C2RustUnnamed_1 = 1;
pub const GPIO_PuPd_NOPULL: C2RustUnnamed_1 = 0;
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
pub type pgn_t = uint16_t;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed_2 = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed_2 = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed_2 = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed_2 = 4095;
// function that resets a single parameter group instance
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_3,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_3 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
}
pub type pgRegistry_t = pgRegistry_s;
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
// type specifying IO pin. Currently ioRec_t pointer, but this may change
// NONE initializer for ioTag_t variables
// NONE initializer for IO_t variable
// both ioTag_t and IO_t are guarantied to be zero if pinid is NONE (no pin)
// this simplifies initialization (globals are zeroed on start) and allows
//  omitting unused fields in structure initializers.
// it is also possible to use IO_t and ioTag_t as boolean value
//   TODO - this may conflict with requirement to generate warning/error on IO_t - ioTag_t assignment
//   IO_t being pointer is only possibility I know of ..
// pin config handling
// pin config is packed into ioConfig_t to decrease memory requirements
// IOCFG_x macros are defined for common combinations for all CPUs; this
//  helps masking CPU differences
pub type ioConfig_t = uint8_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxSpiConfig_s {
    pub rx_spi_protocol: uint8_t,
    pub rx_spi_id: uint32_t,
    pub rx_spi_rf_channel_count: uint8_t,
    pub csnTag: ioTag_t,
    pub spibus: uint8_t,
}
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
pub type rxSpiConfig_t = rxSpiConfig_s;
pub type timeDelta_t = int32_t;
pub type timeMs_t = uint32_t;
pub type timeUs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxRuntimeConfig_s {
    pub channelCount: uint8_t,
    pub rxRefreshRate: uint16_t,
    pub rcReadRawFn: rcReadRawDataFnPtr,
    pub rcFrameStatusFn: rcFrameStatusFnPtr,
    pub rcProcessFrameFn: rcProcessFrameFnPtr,
    pub channelData: *mut uint16_t,
    pub frameData: *mut libc::c_void,
}
pub type rcProcessFrameFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s) -> bool>;
// RX protocol
// type of SPI RX protocol
                                            // nrf24: 0 = v202 250kbps. (Must be enabled by FEATURE_RX_NRF24 first.)
// SPI Bus
// used by receiver driver to return channel data
pub type rcFrameStatusFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut rxRuntimeConfig_s) -> uint8_t>;
pub type rcReadRawDataFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s, _: uint8_t)
               -> uint16_t>;
pub type rxRuntimeConfig_t = rxRuntimeConfig_s;
pub type rssiSource_e = libc::c_uint;
pub const RSSI_SOURCE_FRAME_ERRORS: rssiSource_e = 5;
pub const RSSI_SOURCE_MSP: rssiSource_e = 4;
pub const RSSI_SOURCE_RX_PROTOCOL: rssiSource_e = 3;
pub const RSSI_SOURCE_RX_CHANNEL: rssiSource_e = 2;
pub const RSSI_SOURCE_ADC: rssiSource_e = 1;
pub const RSSI_SOURCE_NONE: rssiSource_e = 0;
// number of RC channels as reported by current input driver
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
// Used in MSP. Append at end.
pub type rx_spi_protocol_e = libc::c_uint;
pub const RX_SPI_PROTOCOL_COUNT: rx_spi_protocol_e = 13;
pub const RX_SPI_NRF24_KN: rx_spi_protocol_e = 12;
pub const RX_SPI_A7105_FLYSKY_2A: rx_spi_protocol_e = 11;
pub const RX_SPI_A7105_FLYSKY: rx_spi_protocol_e = 10;
pub const RX_SPI_FRSKY_X: rx_spi_protocol_e = 9;
pub const RX_SPI_FRSKY_D: rx_spi_protocol_e = 8;
pub const RX_SPI_NRF24_INAV: rx_spi_protocol_e = 7;
pub const RX_SPI_NRF24_H8_3D: rx_spi_protocol_e = 6;
pub const RX_SPI_NRF24_CX10A: rx_spi_protocol_e = 5;
pub const RX_SPI_NRF24_CX10: rx_spi_protocol_e = 4;
pub const RX_SPI_NRF24_SYMA_X5C: rx_spi_protocol_e = 3;
pub const RX_SPI_NRF24_SYMA_X: rx_spi_protocol_e = 2;
pub const RX_SPI_NRF24_V202_1M: rx_spi_protocol_e = 1;
pub const RX_SPI_NRF24_V202_250K: rx_spi_protocol_e = 0;
pub type rx_spi_received_e = libc::c_uint;
pub const RX_SPI_RECEIVED_DATA: rx_spi_received_e = 2;
pub const RX_SPI_RECEIVED_BIND: rx_spi_received_e = 1;
pub const RX_SPI_RECEIVED_NONE: rx_spi_received_e = 0;
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
/*
 CC2500 SPI drivers
*/
pub type C2RustUnnamed_4 = libc::c_uint;
pub const CC2500_3F_RXFIFO: C2RustUnnamed_4 = 63;
pub const CC2500_3F_TXFIFO: C2RustUnnamed_4 = 63;
// Overflow and # of bytes in RXFIFO
// Multi byte memory locations
pub const CC2500_3E_PATABLE: C2RustUnnamed_4 = 62;
// Underflow and # of bytes in TXFIFO
pub const CC2500_3B_RXBYTES: C2RustUnnamed_4 = 59;
// Current setting from PLL cal module
pub const CC2500_3A_TXBYTES: C2RustUnnamed_4 = 58;
// Current GDOx status and packet status
pub const CC2500_39_VCO_VC_DAC: C2RustUnnamed_4 = 57;
// Low byte of WOR timer
pub const CC2500_38_PKTSTATUS: C2RustUnnamed_4 = 56;
// High byte of WOR timer
pub const CC2500_37_WORTIME0: C2RustUnnamed_4 = 55;
// Control state machine state
pub const CC2500_36_WORTIME1: C2RustUnnamed_4 = 54;
// Received signal strength indication
pub const CC2500_35_MARCSTATE: C2RustUnnamed_4 = 53;
// Demodulator estimate for link quality
pub const CC2500_34_RSSI: C2RustUnnamed_4 = 52;
// Frequency offset estimate
pub const CC2500_33_LQI: C2RustUnnamed_4 = 51;
// Current version number
pub const CC2500_32_FREQEST: C2RustUnnamed_4 = 50;
// Part number
pub const CC2500_31_VERSION: C2RustUnnamed_4 = 49;
// Various test settings
// Status registers
pub const CC2500_30_PARTNUM: C2RustUnnamed_4 = 48;
// Various test settings
pub const CC2500_2E_TEST0: C2RustUnnamed_4 = 46;
// Various test settings
pub const CC2500_2D_TEST1: C2RustUnnamed_4 = 45;
// AGC test
pub const CC2500_2C_TEST2: C2RustUnnamed_4 = 44;
// Production test
pub const CC2500_2B_AGCTEST: C2RustUnnamed_4 = 43;
// Frequency synthesizer cal control
pub const CC2500_2A_PTEST: C2RustUnnamed_4 = 42;
// RC oscillator configuration
pub const CC2500_29_FSTEST: C2RustUnnamed_4 = 41;
// RC oscillator configuration
pub const CC2500_28_RCCTRL0: C2RustUnnamed_4 = 40;
// Frequency synthesizer calibration
pub const CC2500_27_RCCTRL1: C2RustUnnamed_4 = 39;
// Frequency synthesizer calibration
pub const CC2500_26_FSCAL0: C2RustUnnamed_4 = 38;
// Frequency synthesizer calibration
pub const CC2500_25_FSCAL1: C2RustUnnamed_4 = 37;
// Frequency synthesizer calibration
pub const CC2500_24_FSCAL2: C2RustUnnamed_4 = 36;
// Front end TX configuration
pub const CC2500_23_FSCAL3: C2RustUnnamed_4 = 35;
// Front end RX configuration
pub const CC2500_22_FREND0: C2RustUnnamed_4 = 34;
// Wake On Radio control
pub const CC2500_21_FREND1: C2RustUnnamed_4 = 33;
// Low byte Event 0 timeout
pub const CC2500_20_WORCTRL: C2RustUnnamed_4 = 32;
// High byte Event 0 timeout
pub const CC2500_1F_WOREVT0: C2RustUnnamed_4 = 31;
// AGC control
pub const CC2500_1E_WOREVT1: C2RustUnnamed_4 = 30;
// AGC control
pub const CC2500_1D_AGCCTRL0: C2RustUnnamed_4 = 29;
// AGC control
pub const CC2500_1C_AGCCTRL1: C2RustUnnamed_4 = 28;
// Bit Synchronization configuration
pub const CC2500_1B_AGCCTRL2: C2RustUnnamed_4 = 27;
// Frequency Offset Compensation config
pub const CC2500_1A_BSCFG: C2RustUnnamed_4 = 26;
// Main Radio Cntrl State Machine config
pub const CC2500_19_FOCCFG: C2RustUnnamed_4 = 25;
// Main Radio Cntrl State Machine config
pub const CC2500_18_MCSM0: C2RustUnnamed_4 = 24;
// Main Radio Cntrl State Machine config
pub const CC2500_17_MCSM1: C2RustUnnamed_4 = 23;
// Modem deviation setting
pub const CC2500_16_MCSM2: C2RustUnnamed_4 = 22;
// Modem configuration
pub const CC2500_15_DEVIATN: C2RustUnnamed_4 = 21;
// Modem configuration
pub const CC2500_14_MDMCFG0: C2RustUnnamed_4 = 20;
// Modem configuration
pub const CC2500_13_MDMCFG1: C2RustUnnamed_4 = 19;
// Modem configuration
pub const CC2500_12_MDMCFG2: C2RustUnnamed_4 = 18;
// Modem configuration
pub const CC2500_11_MDMCFG3: C2RustUnnamed_4 = 17;
// Frequency control word, low byte
pub const CC2500_10_MDMCFG4: C2RustUnnamed_4 = 16;
// Frequency control word, middle byte
pub const CC2500_0F_FREQ0: C2RustUnnamed_4 = 15;
// Frequency control word, high byte
pub const CC2500_0E_FREQ1: C2RustUnnamed_4 = 14;
// Frequency synthesizer control
pub const CC2500_0D_FREQ2: C2RustUnnamed_4 = 13;
// Frequency synthesizer control
pub const CC2500_0C_FSCTRL0: C2RustUnnamed_4 = 12;
// Channel number
pub const CC2500_0B_FSCTRL1: C2RustUnnamed_4 = 11;
// Device address
pub const CC2500_0A_CHANNR: C2RustUnnamed_4 = 10;
// Packet automation control
pub const CC2500_09_ADDR: C2RustUnnamed_4 = 9;
// Packet automation control
pub const CC2500_08_PKTCTRL0: C2RustUnnamed_4 = 8;
// Packet length
pub const CC2500_07_PKTCTRL1: C2RustUnnamed_4 = 7;
// Sync word, low byte
pub const CC2500_06_PKTLEN: C2RustUnnamed_4 = 6;
// Sync word, high byte
pub const CC2500_05_SYNC0: C2RustUnnamed_4 = 5;
// RX FIFO and TX FIFO thresholds
pub const CC2500_04_SYNC1: C2RustUnnamed_4 = 4;
// GDO0 output pin configuration
pub const CC2500_03_FIFOTHR: C2RustUnnamed_4 = 3;
// GDO1 output pin configuration
pub const CC2500_02_IOCFG0: C2RustUnnamed_4 = 2;
// GDO2 output pin configuration
pub const CC2500_01_IOCFG1: C2RustUnnamed_4 = 1;
pub const CC2500_00_IOCFG2: C2RustUnnamed_4 = 0;
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
pub type resourceOwner_e = libc::c_uint;
pub const OWNER_TOTAL_COUNT: resourceOwner_e = 55;
pub const OWNER_SPI_PREINIT_OPU: resourceOwner_e = 54;
pub const OWNER_SPI_PREINIT_IPU: resourceOwner_e = 53;
pub const OWNER_USB_MSC_PIN: resourceOwner_e = 52;
pub const OWNER_PINIO: resourceOwner_e = 51;
pub const OWNER_RX_SPI: resourceOwner_e = 50;
pub const OWNER_RANGEFINDER: resourceOwner_e = 49;
pub const OWNER_TIMUP: resourceOwner_e = 48;
pub const OWNER_CAMERA_CONTROL: resourceOwner_e = 47;
pub const OWNER_ESCSERIAL: resourceOwner_e = 46;
pub const OWNER_RX_BIND_PLUG: resourceOwner_e = 45;
pub const OWNER_COMPASS_CS: resourceOwner_e = 44;
pub const OWNER_VTX: resourceOwner_e = 43;
pub const OWNER_TRANSPONDER: resourceOwner_e = 42;
pub const OWNER_LED_STRIP: resourceOwner_e = 41;
pub const OWNER_INVERTER: resourceOwner_e = 40;
pub const OWNER_RX_BIND: resourceOwner_e = 39;
pub const OWNER_OSD: resourceOwner_e = 38;
pub const OWNER_BEEPER: resourceOwner_e = 37;
pub const OWNER_USB_DETECT: resourceOwner_e = 36;
pub const OWNER_USB: resourceOwner_e = 35;
pub const OWNER_COMPASS_EXTI: resourceOwner_e = 34;
pub const OWNER_BARO_EXTI: resourceOwner_e = 33;
pub const OWNER_MPU_EXTI: resourceOwner_e = 32;
pub const OWNER_SPI_CS: resourceOwner_e = 31;
pub const OWNER_RX_SPI_CS: resourceOwner_e = 30;
pub const OWNER_OSD_CS: resourceOwner_e = 29;
pub const OWNER_MPU_CS: resourceOwner_e = 28;
pub const OWNER_BARO_CS: resourceOwner_e = 27;
pub const OWNER_FLASH_CS: resourceOwner_e = 26;
pub const OWNER_SDCARD_DETECT: resourceOwner_e = 25;
pub const OWNER_SDCARD_CS: resourceOwner_e = 24;
pub const OWNER_SDCARD: resourceOwner_e = 23;
pub const OWNER_I2C_SDA: resourceOwner_e = 22;
pub const OWNER_I2C_SCL: resourceOwner_e = 21;
pub const OWNER_SPI_MOSI: resourceOwner_e = 20;
pub const OWNER_SPI_MISO: resourceOwner_e = 19;
pub const OWNER_SPI_SCK: resourceOwner_e = 18;
pub const OWNER_SYSTEM: resourceOwner_e = 17;
pub const OWNER_SONAR_ECHO: resourceOwner_e = 16;
pub const OWNER_SONAR_TRIGGER: resourceOwner_e = 15;
pub const OWNER_TIMER: resourceOwner_e = 14;
pub const OWNER_PINDEBUG: resourceOwner_e = 13;
pub const OWNER_SERIAL_RX: resourceOwner_e = 12;
pub const OWNER_SERIAL_TX: resourceOwner_e = 11;
pub const OWNER_ADC_RSSI: resourceOwner_e = 10;
pub const OWNER_ADC_EXT: resourceOwner_e = 9;
pub const OWNER_ADC_CURR: resourceOwner_e = 8;
pub const OWNER_ADC_BATT: resourceOwner_e = 7;
pub const OWNER_ADC: resourceOwner_e = 6;
pub const OWNER_LED: resourceOwner_e = 5;
pub const OWNER_SERVO: resourceOwner_e = 4;
pub const OWNER_MOTOR: resourceOwner_e = 3;
pub const OWNER_PPMINPUT: resourceOwner_e = 2;
pub const OWNER_PWMINPUT: resourceOwner_e = 1;
pub const OWNER_FREE: resourceOwner_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxFrSkySpiConfig_s {
    pub autoBind: uint8_t,
    pub bindTxId: [uint8_t; 2],
    pub bindOffset: int8_t,
    pub bindHopData: [uint8_t; 50],
    pub rxNum: uint8_t,
    pub useExternalAdc: uint8_t,
}
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
pub type rxFrSkySpiConfig_t = rxFrSkySpiConfig_s;
pub const STATE_INIT: C2RustUnnamed_5 = 0;
pub type setRcDataFn
    =
    unsafe extern "C" fn(_: *mut uint16_t, _: *const uint8_t) -> ();
pub type handlePacketFn
    =
    unsafe extern "C" fn(_: *mut uint8_t, _: *mut uint8_t) -> uint8_t;
pub const STATE_STARTING: C2RustUnnamed_5 = 6;
pub const STATE_BIND_COMPLETE: C2RustUnnamed_5 = 5;
pub const STATE_BIND_BINDING2: C2RustUnnamed_5 = 4;
pub const STATE_BIND_BINDING1: C2RustUnnamed_5 = 3;
pub const STATE_BIND_TUNING: C2RustUnnamed_5 = 2;
pub const STATE_BIND: C2RustUnnamed_5 = 1;
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
pub type C2RustUnnamed_5 = libc::c_uint;
pub const STATE_RESUME: C2RustUnnamed_5 = 10;
pub const STATE_TELEMETRY: C2RustUnnamed_5 = 9;
pub const STATE_DATA: C2RustUnnamed_5 = 8;
pub const STATE_UPDATE: C2RustUnnamed_5 = 7;
#[inline]
unsafe extern "C" fn rxFrSkySpiConfig() -> *const rxFrSkySpiConfig_t {
    return &mut rxFrSkySpiConfig_System;
}
#[inline]
unsafe extern "C" fn rxFrSkySpiConfigMutable() -> *mut rxFrSkySpiConfig_t {
    return &mut rxFrSkySpiConfig_System;
}
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
static mut spiProtocol: rx_spi_protocol_e = RX_SPI_NRF24_V202_250K;
static mut start_time: timeMs_t = 0;
static mut protocolState: uint8_t = 0;
#[no_mangle]
pub static mut missingPackets: uint32_t = 0;
#[no_mangle]
pub static mut timeoutUs: timeDelta_t = 0;
static mut calData: [[uint8_t; 3]; 255] = [[0; 3]; 255];
static mut timeTunedMs: timeMs_t = 0;
#[no_mangle]
pub static mut listLength: uint8_t = 0;
static mut bindIdx: uint8_t = 0;
static mut bindOffset: int8_t = 0;
static mut lastBindPinStatus: bool = false;
static mut bindRequested: bool = false;
static mut handlePacket: Option<handlePacketFn> = None;
static mut setRcData: Option<setRcDataFn> = None;
#[no_mangle]
pub static mut gdoPin: IO_t = 0 as *const libc::c_void as *mut libc::c_void;
static mut bindPin: IO_t = 0 as *const libc::c_void as *mut libc::c_void;
static mut frSkyLedPin: IO_t = 0 as *const libc::c_void as *mut libc::c_void;
static mut txEnPin: IO_t = 0 as *const libc::c_void as *mut libc::c_void;
static mut rxLnaEnPin: IO_t = 0 as *const libc::c_void as *mut libc::c_void;
static mut antSelPin: IO_t = 0 as *const libc::c_void as *mut libc::c_void;
#[no_mangle]
pub static mut rssiDbm: int16_t = 0;
#[no_mangle]
pub static mut rxFrSkySpiConfig_System: rxFrSkySpiConfig_t =
    rxFrSkySpiConfig_t{autoBind: 0,
                       bindTxId: [0; 2],
                       bindOffset: 0,
                       bindHopData: [0; 50],
                       rxNum: 0,
                       useExternalAdc: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut rxFrSkySpiConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (523 as libc::c_int |
                                      (1 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<rxFrSkySpiConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &rxFrSkySpiConfig_System as
                                     *const rxFrSkySpiConfig_t as
                                     *mut rxFrSkySpiConfig_t as *mut uint8_t,
                             copy:
                                 &rxFrSkySpiConfig_Copy as
                                     *const rxFrSkySpiConfig_t as
                                     *mut rxFrSkySpiConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_3{ptr:
                                                     &pgResetTemplate_rxFrSkySpiConfig
                                                         as
                                                         *const rxFrSkySpiConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut rxFrSkySpiConfig_Copy: rxFrSkySpiConfig_t =
    rxFrSkySpiConfig_t{autoBind: 0,
                       bindTxId: [0; 2],
                       bindOffset: 0,
                       bindHopData: [0; 50],
                       rxNum: 0,
                       useExternalAdc: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_rxFrSkySpiConfig: rxFrSkySpiConfig_t =
    {
        let mut init =
            rxFrSkySpiConfig_s{autoBind: 0 as libc::c_int as uint8_t,
                               bindTxId:
                                   [0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t],
                               bindOffset: 0 as libc::c_int as int8_t,
                               bindHopData:
                                   [0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t,
                                    0 as libc::c_int as uint8_t],
                               rxNum: 0 as libc::c_int as uint8_t,
                               useExternalAdc: 0 as libc::c_int as uint8_t,};
        init
    };
#[no_mangle]
pub unsafe extern "C" fn setRssiDbm(mut value: uint8_t) {
    if value as libc::c_int >= 128 as libc::c_int {
        rssiDbm =
            ((value as uint16_t as libc::c_int * 18 as libc::c_int >>
                  5 as libc::c_int) - 82 as libc::c_int) as int16_t
    } else {
        rssiDbm =
            ((value as uint16_t as libc::c_int * 18 as libc::c_int >>
                  5 as libc::c_int) + 65 as libc::c_int) as int16_t
    }
    setRssi(((rssiDbm as libc::c_int) << 3 as libc::c_int) as uint16_t,
            RSSI_SOURCE_RX_PROTOCOL);
}
// USE_RX_FRSKY_SPI_TELEMETRY
#[no_mangle]
pub unsafe extern "C" fn TxEnable() { IOHi(txEnPin); }
#[no_mangle]
pub unsafe extern "C" fn TxDisable() { IOLo(txEnPin); }
#[no_mangle]
pub unsafe extern "C" fn LedOn() { IOHi(frSkyLedPin); }
#[no_mangle]
pub unsafe extern "C" fn LedOff() { IOLo(frSkyLedPin); }
#[no_mangle]
pub unsafe extern "C" fn frSkySpiBind() {
    bindRequested = 1 as libc::c_int != 0;
}
unsafe extern "C" fn initialise() {
    cc2500Reset();
    cc2500WriteReg(CC2500_02_IOCFG0 as libc::c_int as uint8_t,
                   0x1 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_17_MCSM1 as libc::c_int as uint8_t,
                   0xc as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_18_MCSM0 as libc::c_int as uint8_t,
                   0x18 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_07_PKTCTRL1 as libc::c_int as uint8_t,
                   0x4 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_3E_PATABLE as libc::c_int as uint8_t,
                   0xff as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_0C_FSCTRL0 as libc::c_int as uint8_t,
                   0 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_0D_FREQ2 as libc::c_int as uint8_t,
                   0x5c as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_0E_FREQ1 as libc::c_int as uint8_t,
                   0x76 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_0F_FREQ0 as libc::c_int as uint8_t,
                   0x27 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_13_MDMCFG1 as libc::c_int as uint8_t,
                   0x23 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_14_MDMCFG0 as libc::c_int as uint8_t,
                   0x7a as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_19_FOCCFG as libc::c_int as uint8_t,
                   0x16 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_1A_BSCFG as libc::c_int as uint8_t,
                   0x6c as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_1B_AGCCTRL2 as libc::c_int as uint8_t,
                   0x3 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_1C_AGCCTRL1 as libc::c_int as uint8_t,
                   0x40 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_1D_AGCCTRL0 as libc::c_int as uint8_t,
                   0x91 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_21_FREND1 as libc::c_int as uint8_t,
                   0x56 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_22_FREND0 as libc::c_int as uint8_t,
                   0x10 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_23_FSCAL3 as libc::c_int as uint8_t,
                   0xa9 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_24_FSCAL2 as libc::c_int as uint8_t,
                   0xa as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_25_FSCAL1 as libc::c_int as uint8_t,
                   0 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_26_FSCAL0 as libc::c_int as uint8_t,
                   0x11 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_29_FSTEST as libc::c_int as uint8_t,
                   0x59 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_2C_TEST2 as libc::c_int as uint8_t,
                   0x88 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_2D_TEST1 as libc::c_int as uint8_t,
                   0x31 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_2E_TEST0 as libc::c_int as uint8_t,
                   0xb as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_03_FIFOTHR as libc::c_int as uint8_t,
                   0x7 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_09_ADDR as libc::c_int as uint8_t,
                   0 as libc::c_int as uint8_t);
    match spiProtocol as libc::c_uint {
        8 => {
            cc2500WriteReg(CC2500_06_PKTLEN as libc::c_int as uint8_t,
                           0x19 as libc::c_int as uint8_t);
            cc2500WriteReg(CC2500_08_PKTCTRL0 as libc::c_int as uint8_t,
                           0x5 as libc::c_int as uint8_t);
            cc2500WriteReg(CC2500_0B_FSCTRL1 as libc::c_int as uint8_t,
                           0x8 as libc::c_int as uint8_t);
            cc2500WriteReg(CC2500_10_MDMCFG4 as libc::c_int as uint8_t,
                           0xaa as libc::c_int as uint8_t);
            cc2500WriteReg(CC2500_11_MDMCFG3 as libc::c_int as uint8_t,
                           0x39 as libc::c_int as uint8_t);
            cc2500WriteReg(CC2500_12_MDMCFG2 as libc::c_int as uint8_t,
                           0x11 as libc::c_int as uint8_t);
            cc2500WriteReg(CC2500_15_DEVIATN as libc::c_int as uint8_t,
                           0x42 as libc::c_int as uint8_t);
        }
        9 => {
            cc2500WriteReg(CC2500_06_PKTLEN as libc::c_int as uint8_t,
                           0x1e as libc::c_int as uint8_t);
            cc2500WriteReg(CC2500_08_PKTCTRL0 as libc::c_int as uint8_t,
                           0x1 as libc::c_int as uint8_t);
            cc2500WriteReg(CC2500_0B_FSCTRL1 as libc::c_int as uint8_t,
                           0xa as libc::c_int as uint8_t);
            cc2500WriteReg(CC2500_10_MDMCFG4 as libc::c_int as uint8_t,
                           0x7b as libc::c_int as uint8_t);
            cc2500WriteReg(CC2500_11_MDMCFG3 as libc::c_int as uint8_t,
                           0x61 as libc::c_int as uint8_t);
            cc2500WriteReg(CC2500_12_MDMCFG2 as libc::c_int as uint8_t,
                           0x13 as libc::c_int as uint8_t);
            cc2500WriteReg(CC2500_15_DEVIATN as libc::c_int as uint8_t,
                           0x51 as libc::c_int as uint8_t);
        }
        _ => { }
    }
    let mut c: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while c < 0xff as libc::c_int as libc::c_uint {
        //calibrate all channels
        cc2500Strobe(0x36 as libc::c_int as uint8_t); //
        cc2500WriteReg(CC2500_0A_CHANNR as libc::c_int as uint8_t,
                       c as uint8_t); // waiting flush FIFO
        cc2500Strobe(0x33 as libc::c_int as uint8_t);
        delayMicroseconds(900 as libc::c_int as timeUs_t);
        calData[c as usize][0 as libc::c_int as usize] =
            cc2500ReadReg(CC2500_23_FSCAL3 as libc::c_int as uint8_t);
        calData[c as usize][1 as libc::c_int as usize] =
            cc2500ReadReg(CC2500_24_FSCAL2 as libc::c_int as uint8_t);
        calData[c as usize][2 as libc::c_int as usize] =
            cc2500ReadReg(CC2500_25_FSCAL1 as libc::c_int as uint8_t);
        c = c.wrapping_add(1)
    };
}
#[no_mangle]
pub unsafe extern "C" fn initialiseData(mut adr: uint8_t) {
    cc2500WriteReg(CC2500_0C_FSCTRL0 as libc::c_int as uint8_t,
                   (*rxFrSkySpiConfig()).bindOffset as uint8_t);
    cc2500WriteReg(CC2500_18_MCSM0 as libc::c_int as uint8_t,
                   0x8 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_09_ADDR as libc::c_int as uint8_t,
                   if adr as libc::c_int != 0 {
                       0x3 as libc::c_int
                   } else {
                       (*rxFrSkySpiConfig()).bindTxId[0 as libc::c_int as
                                                          usize] as
                           libc::c_int
                   } as uint8_t);
    cc2500WriteReg(CC2500_07_PKTCTRL1 as libc::c_int as uint8_t,
                   0xd as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_19_FOCCFG as libc::c_int as uint8_t,
                   0x16 as libc::c_int as uint8_t);
    delay(10 as libc::c_int as timeMs_t);
}
unsafe extern "C" fn initTuneRx() {
    cc2500WriteReg(CC2500_19_FOCCFG as libc::c_int as uint8_t,
                   0x14 as libc::c_int as uint8_t);
    timeTunedMs = millis();
    bindOffset = -(126 as libc::c_int) as int8_t;
    cc2500WriteReg(CC2500_0C_FSCTRL0 as libc::c_int as uint8_t,
                   bindOffset as uint8_t);
    cc2500WriteReg(CC2500_07_PKTCTRL1 as libc::c_int as uint8_t,
                   0xc as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_18_MCSM0 as libc::c_int as uint8_t,
                   0x8 as libc::c_int as uint8_t);
    cc2500Strobe(0x36 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_23_FSCAL3 as libc::c_int as uint8_t,
                   calData[0 as libc::c_int as
                               usize][0 as libc::c_int as usize]);
    cc2500WriteReg(CC2500_24_FSCAL2 as libc::c_int as uint8_t,
                   calData[0 as libc::c_int as
                               usize][1 as libc::c_int as usize]);
    cc2500WriteReg(CC2500_25_FSCAL1 as libc::c_int as uint8_t,
                   calData[0 as libc::c_int as
                               usize][2 as libc::c_int as usize]);
    cc2500WriteReg(CC2500_0A_CHANNR as libc::c_int as uint8_t,
                   0 as libc::c_int as uint8_t);
    cc2500Strobe(0x3a as libc::c_int as uint8_t);
    cc2500Strobe(0x34 as libc::c_int as uint8_t);
}
unsafe extern "C" fn tuneRx(mut packet: *mut uint8_t) -> bool {
    if bindOffset as libc::c_int >= 126 as libc::c_int {
        bindOffset = -(126 as libc::c_int) as int8_t
    }
    if millis().wrapping_sub(timeTunedMs) > 50 as libc::c_int as libc::c_uint
       {
        timeTunedMs = millis();
        bindOffset = (bindOffset as libc::c_int + 5 as libc::c_int) as int8_t;
        cc2500WriteReg(CC2500_0C_FSCTRL0 as libc::c_int as uint8_t,
                       bindOffset as uint8_t);
    }
    if IORead(gdoPin) {
        let mut ccLen: uint8_t =
            (cc2500ReadReg((CC2500_3B_RXBYTES as libc::c_int |
                                0xc0 as libc::c_int) as uint8_t) as
                 libc::c_int & 0x7f as libc::c_int) as uint8_t;
        if ccLen != 0 {
            cc2500ReadFifo(packet, ccLen);
            if *packet.offset((ccLen as libc::c_int - 1 as libc::c_int) as
                                  isize) as libc::c_int & 0x80 as libc::c_int
                   != 0 {
                if *packet.offset(2 as libc::c_int as isize) as libc::c_int ==
                       0x1 as libc::c_int {
                    let mut Lqi: uint8_t =
                        (*packet.offset((ccLen as libc::c_int -
                                             1 as libc::c_int) as isize) as
                             libc::c_int & 0x7f as libc::c_int) as uint8_t;
                    if (Lqi as libc::c_int) < 50 as libc::c_int {
                        (*rxFrSkySpiConfigMutable()).bindOffset = bindOffset;
                        return 1 as libc::c_int != 0
                    }
                }
            }
        }
    }
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn initGetBind() {
    cc2500Strobe(0x36 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_23_FSCAL3 as libc::c_int as uint8_t,
                   calData[0 as libc::c_int as
                               usize][0 as libc::c_int as usize]);
    cc2500WriteReg(CC2500_24_FSCAL2 as libc::c_int as uint8_t,
                   calData[0 as libc::c_int as
                               usize][1 as libc::c_int as usize]);
    cc2500WriteReg(CC2500_25_FSCAL1 as libc::c_int as uint8_t,
                   calData[0 as libc::c_int as
                               usize][2 as libc::c_int as usize]);
    cc2500WriteReg(CC2500_0A_CHANNR as libc::c_int as uint8_t,
                   0 as libc::c_int as uint8_t);
    cc2500Strobe(0x3a as libc::c_int as uint8_t);
    delayMicroseconds(20 as libc::c_int as timeUs_t);
    cc2500Strobe(0x34 as libc::c_int as uint8_t);
    listLength = 0 as libc::c_int as uint8_t;
    bindIdx = 0x5 as libc::c_int as uint8_t;
}
unsafe extern "C" fn getBind1(mut packet: *mut uint8_t) -> bool {
    // len|bind |tx
    // id|03|01|idx|h0|h1|h2|h3|h4|00|00|00|00|00|00|00|00|00|00|00|00|00|00|00|CHK1|CHK2|RSSI|LQI/CRC|
    // Start by getting bind packet 0 and the txid
    if IORead(gdoPin) {
        let mut ccLen: uint8_t =
            (cc2500ReadReg((CC2500_3B_RXBYTES as libc::c_int |
                                0xc0 as libc::c_int) as uint8_t) as
                 libc::c_int & 0x7f as libc::c_int) as
                uint8_t; //CC2500 read registers chip part num
        if ccLen != 0 {
            cc2500ReadFifo(packet,
                           ccLen); //CC2500 read registers chip version
            if *packet.offset((ccLen as libc::c_int - 1 as libc::c_int) as
                                  isize) as libc::c_int & 0x80 as libc::c_int
                   != 0 {
                if *packet.offset(2 as libc::c_int as isize) as libc::c_int ==
                       0x1 as libc::c_int {
                    if *packet.offset(5 as libc::c_int as isize) as
                           libc::c_int == 0 as libc::c_int {
                        (*rxFrSkySpiConfigMutable()).bindTxId[0 as libc::c_int
                                                                  as usize] =
                            *packet.offset(3 as libc::c_int as isize);
                        (*rxFrSkySpiConfigMutable()).bindTxId[1 as libc::c_int
                                                                  as usize] =
                            *packet.offset(4 as libc::c_int as isize);
                        let mut n: uint8_t = 0 as libc::c_int as uint8_t;
                        while (n as libc::c_int) < 5 as libc::c_int {
                            (*rxFrSkySpiConfigMutable()).bindHopData[(*packet.offset(5
                                                                                         as
                                                                                         libc::c_int
                                                                                         as
                                                                                         isize)
                                                                          as
                                                                          libc::c_int
                                                                          +
                                                                          n as
                                                                              libc::c_int)
                                                                         as
                                                                         usize]
                                =
                                *packet.offset((6 as libc::c_int +
                                                    n as libc::c_int) as
                                                   isize);
                            n = n.wrapping_add(1)
                        }
                        (*rxFrSkySpiConfigMutable()).rxNum =
                            *packet.offset(12 as libc::c_int as isize);
                        return 1 as libc::c_int != 0
                    }
                }
            }
        }
    }
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn getBind2(mut packet: *mut uint8_t) -> bool {
    if bindIdx as libc::c_int <= 120 as libc::c_int {
        if IORead(gdoPin) {
            let mut ccLen: uint8_t =
                (cc2500ReadReg((CC2500_3B_RXBYTES as libc::c_int |
                                    0xc0 as libc::c_int) as uint8_t) as
                     libc::c_int & 0x7f as libc::c_int) as uint8_t;
            if ccLen != 0 {
                cc2500ReadFifo(packet, ccLen);
                if *packet.offset((ccLen as libc::c_int - 1 as libc::c_int) as
                                      isize) as libc::c_int &
                       0x80 as libc::c_int != 0 {
                    if *packet.offset(2 as libc::c_int as isize) as
                           libc::c_int == 0x1 as libc::c_int {
                        if *packet.offset(3 as libc::c_int as isize) as
                               libc::c_int ==
                               (*rxFrSkySpiConfig()).bindTxId[0 as libc::c_int
                                                                  as usize] as
                                   libc::c_int &&
                               *packet.offset(4 as libc::c_int as isize) as
                                   libc::c_int ==
                                   (*rxFrSkySpiConfig()).bindTxId[1 as
                                                                      libc::c_int
                                                                      as
                                                                      usize]
                                       as libc::c_int {
                            if *packet.offset(5 as libc::c_int as isize) as
                                   libc::c_int == bindIdx as libc::c_int {
                                let mut n: uint8_t =
                                    0 as libc::c_int as uint8_t;
                                while (n as libc::c_int) < 5 as libc::c_int {
                                    if *packet.offset((6 as libc::c_int +
                                                           n as libc::c_int)
                                                          as isize) as
                                           libc::c_int ==
                                           *packet.offset((ccLen as
                                                               libc::c_int -
                                                               3 as
                                                                   libc::c_int)
                                                              as isize) as
                                               libc::c_int ||
                                           *packet.offset((6 as libc::c_int +
                                                               n as
                                                                   libc::c_int)
                                                              as isize) as
                                               libc::c_int == 0 as libc::c_int
                                       {
                                        if bindIdx as libc::c_int >=
                                               0x2d as libc::c_int {
                                            listLength =
                                                (*packet.offset(5 as
                                                                    libc::c_int
                                                                    as isize)
                                                     as libc::c_int +
                                                     n as libc::c_int) as
                                                    uint8_t;
                                            return 1 as libc::c_int != 0
                                        }
                                    }
                                    (*rxFrSkySpiConfigMutable()).bindHopData[(*packet.offset(5
                                                                                                 as
                                                                                                 libc::c_int
                                                                                                 as
                                                                                                 isize)
                                                                                  as
                                                                                  libc::c_int
                                                                                  +
                                                                                  n
                                                                                      as
                                                                                      libc::c_int)
                                                                                 as
                                                                                 usize]
                                        =
                                        *packet.offset((6 as libc::c_int +
                                                            n as libc::c_int)
                                                           as isize);
                                    n = n.wrapping_add(1)
                                }
                                bindIdx =
                                    (bindIdx as libc::c_int +
                                         5 as libc::c_int) as uint8_t;
                                return 0 as libc::c_int != 0
                            }
                        }
                    }
                }
            }
        }
        return 0 as libc::c_int != 0
    } else { return 1 as libc::c_int != 0 };
}
#[no_mangle]
pub unsafe extern "C" fn checkBindRequested(mut reset: bool) -> bool {
    if !bindPin.is_null() {
        let mut bindPinStatus: bool = IORead(bindPin);
        if lastBindPinStatus as libc::c_int != 0 && !bindPinStatus {
            bindRequested = 1 as libc::c_int != 0
        }
        lastBindPinStatus = bindPinStatus
    }
    if !bindRequested {
        return 0 as libc::c_int != 0
    } else {
        if reset { bindRequested = 0 as libc::c_int != 0 }
        return 1 as libc::c_int != 0
    };
}
#[no_mangle]
pub unsafe extern "C" fn frSkySpiDataReceived(mut packet: *mut uint8_t)
 -> rx_spi_received_e {
    let mut ret: rx_spi_received_e = RX_SPI_RECEIVED_NONE;
    match protocolState as libc::c_int {
        0 => {
            if millis().wrapping_sub(start_time) >
                   10 as libc::c_int as libc::c_uint {
                initialise();
                protocolState = STATE_BIND as libc::c_int as uint8_t
            }
        }
        1 => {
            if checkBindRequested(1 as libc::c_int != 0) as libc::c_int != 0
                   || (*rxFrSkySpiConfig()).autoBind as libc::c_int != 0 {
                LedOn();
                initTuneRx();
                protocolState = STATE_BIND_TUNING as libc::c_int as uint8_t
            } else {
                protocolState = STATE_STARTING as libc::c_int as uint8_t
            }
        }
        2 => {
            if tuneRx(packet) {
                initGetBind();
                initialiseData(1 as libc::c_int as uint8_t);
                protocolState = STATE_BIND_BINDING1 as libc::c_int as uint8_t
            }
        }
        3 => {
            if getBind1(packet) {
                protocolState = STATE_BIND_BINDING2 as libc::c_int as uint8_t
            }
        }
        4 => {
            if getBind2(packet) {
                cc2500Strobe(0x36 as libc::c_int as uint8_t);
                protocolState = STATE_BIND_COMPLETE as libc::c_int as uint8_t
            }
        }
        5 => {
            if (*rxFrSkySpiConfig()).autoBind == 0 {
                writeEEPROM();
            } else {
                let mut ctr: uint8_t = 40 as libc::c_int as uint8_t;
                loop  {
                    let fresh0 = ctr;
                    ctr = ctr.wrapping_sub(1);
                    if !(fresh0 != 0) { break ; }
                    LedOn();
                    delay(50 as libc::c_int as timeMs_t);
                    LedOff();
                    delay(50 as libc::c_int as timeMs_t);
                }
            }
            ret = RX_SPI_RECEIVED_BIND;
            protocolState = STATE_STARTING as libc::c_int as uint8_t
        }
        _ => {
            ret =
                handlePacket.expect("non-null function pointer")(packet,
                                                                 &mut protocolState)
                    as rx_spi_received_e
        }
    }
    return ret;
}
#[no_mangle]
pub unsafe extern "C" fn frSkySpiSetRcData(mut rcData: *mut uint16_t,
                                           mut payload: *const uint8_t) {
    setRcData.expect("non-null function pointer")(rcData, payload);
}
#[no_mangle]
pub unsafe extern "C" fn nextChannel(mut skip: uint8_t) {
    static mut channr: uint8_t = 0 as libc::c_int as uint8_t;
    channr = (channr as libc::c_int + skip as libc::c_int) as uint8_t;
    while channr as libc::c_int >= listLength as libc::c_int {
        channr =
            (channr as libc::c_int - listLength as libc::c_int) as uint8_t
    }
    cc2500Strobe(0x36 as libc::c_int as uint8_t);
    cc2500WriteReg(CC2500_23_FSCAL3 as libc::c_int as uint8_t,
                   calData[(*rxFrSkySpiConfig()).bindHopData[channr as usize]
                               as usize][0 as libc::c_int as usize]);
    cc2500WriteReg(CC2500_24_FSCAL2 as libc::c_int as uint8_t,
                   calData[(*rxFrSkySpiConfig()).bindHopData[channr as usize]
                               as usize][1 as libc::c_int as usize]);
    cc2500WriteReg(CC2500_25_FSCAL1 as libc::c_int as uint8_t,
                   calData[(*rxFrSkySpiConfig()).bindHopData[channr as usize]
                               as usize][2 as libc::c_int as usize]);
    cc2500WriteReg(CC2500_0A_CHANNR as libc::c_int as uint8_t,
                   (*rxFrSkySpiConfig()).bindHopData[channr as usize]);
    if spiProtocol as libc::c_uint ==
           RX_SPI_FRSKY_D as libc::c_int as libc::c_uint {
        cc2500Strobe(0x3a as libc::c_int as uint8_t);
    };
}
#[no_mangle]
pub unsafe extern "C" fn switchAntennae() {
    static mut alternativeAntennaSelected: bool = 1 as libc::c_int != 0;
    if alternativeAntennaSelected {
        IOLo(antSelPin);
    } else { IOHi(antSelPin); }
    alternativeAntennaSelected = !alternativeAntennaSelected;
}
unsafe extern "C" fn frSkySpiDetect() -> bool {
    let chipPartNum: uint8_t =
        cc2500ReadReg((CC2500_30_PARTNUM as libc::c_int | 0xc0 as libc::c_int)
                          as uint8_t);
    let chipVersion: uint8_t =
        cc2500ReadReg((CC2500_31_VERSION as libc::c_int | 0xc0 as libc::c_int)
                          as uint8_t);
    if chipPartNum as libc::c_int == 0x80 as libc::c_int &&
           chipVersion as libc::c_int == 0x3 as libc::c_int {
        return 1 as libc::c_int != 0
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn frSkySpiInit(mut rxSpiConfig: *const rxSpiConfig_t,
                                      mut rxRuntimeConfig:
                                          *mut rxRuntimeConfig_t) -> bool {
    if !frSkySpiDetect() { return 0 as libc::c_int != 0 }
    spiProtocol = (*rxSpiConfig).rx_spi_protocol as rx_spi_protocol_e;
    match spiProtocol as libc::c_uint {
        8 => {
            (*rxRuntimeConfig).channelCount = 8 as libc::c_int as uint8_t;
            handlePacket =
                ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                         *mut uint8_t,
                                                                     _:
                                                                         *mut uint8_t)
                                                    -> rx_spi_received_e>,
                                         Option<handlePacketFn>>(Some(frSkyDHandlePacket
                                                                          as
                                                                          unsafe extern "C" fn(_:
                                                                                                   *mut uint8_t,
                                                                                               _:
                                                                                                   *mut uint8_t)
                                                                              ->
                                                                                  rx_spi_received_e));
            setRcData =
                Some(frSkyDSetRcData as
                         unsafe extern "C" fn(_: *mut uint16_t,
                                              _: *const uint8_t) -> ());
            frSkyDInit();
        }
        9 => {
            (*rxRuntimeConfig).channelCount = 16 as libc::c_int as uint8_t;
            handlePacket =
                ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                         *mut uint8_t,
                                                                     _:
                                                                         *mut uint8_t)
                                                    -> rx_spi_received_e>,
                                         Option<handlePacketFn>>(Some(frSkyXHandlePacket
                                                                          as
                                                                          unsafe extern "C" fn(_:
                                                                                                   *mut uint8_t,
                                                                                               _:
                                                                                                   *mut uint8_t)
                                                                              ->
                                                                                  rx_spi_received_e));
            setRcData =
                Some(frSkyXSetRcData as
                         unsafe extern "C" fn(_: *mut uint16_t,
                                              _: *const uint8_t) -> ());
            frSkyXInit();
        }
        _ => { }
    }
    if rssiSource as libc::c_uint ==
           RSSI_SOURCE_NONE as libc::c_int as libc::c_uint {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL
    }
    // gpio init here
    gdoPin =
        IOGetByTag(((1 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 0 as libc::c_int) as
                       ioTag_t); // always on at the moment
    IOInit(gdoPin, OWNER_RX_SPI, 0 as libc::c_int as uint8_t);
    IOConfigGPIO(gdoPin,
                 (GPIO_Mode_IN as libc::c_int |
                      (0 as libc::c_int) << 2 as libc::c_int |
                      (0 as libc::c_int) << 4 as libc::c_int |
                      (GPIO_PuPd_NOPULL as libc::c_int) << 5 as libc::c_int)
                     as ioConfig_t);
    frSkyLedPin =
        IOGetByTag(((1 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 6 as libc::c_int) as ioTag_t);
    IOInit(frSkyLedPin, OWNER_LED, 0 as libc::c_int as uint8_t);
    IOConfigGPIO(frSkyLedPin,
                 (GPIO_Mode_OUT as libc::c_int |
                      (0 as libc::c_int) << 2 as libc::c_int |
                      (GPIO_OType_PP as libc::c_int) << 4 as libc::c_int |
                      (GPIO_PuPd_NOPULL as libc::c_int) << 5 as libc::c_int)
                     as ioConfig_t);
    rxLnaEnPin =
        IOGetByTag(((1 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 11 as libc::c_int) as ioTag_t);
    IOInit(rxLnaEnPin, OWNER_RX_SPI, 0 as libc::c_int as uint8_t);
    IOConfigGPIO(rxLnaEnPin,
                 (GPIO_Mode_OUT as libc::c_int |
                      (0 as libc::c_int) << 2 as libc::c_int |
                      (GPIO_OType_PP as libc::c_int) << 4 as libc::c_int |
                      (GPIO_PuPd_NOPULL as libc::c_int) << 5 as libc::c_int)
                     as ioConfig_t);
    IOHi(rxLnaEnPin);
    txEnPin =
        IOGetByTag(((1 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 1 as libc::c_int) as ioTag_t);
    IOInit(txEnPin, OWNER_RX_SPI, 0 as libc::c_int as uint8_t);
    IOConfigGPIO(txEnPin,
                 (GPIO_Mode_OUT as libc::c_int |
                      (0 as libc::c_int) << 2 as libc::c_int |
                      (GPIO_OType_PP as libc::c_int) << 4 as libc::c_int |
                      (GPIO_PuPd_NOPULL as libc::c_int) << 5 as libc::c_int)
                     as ioConfig_t);
    antSelPin =
        IOGetByTag(((1 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 2 as libc::c_int) as ioTag_t);
    IOInit(antSelPin, OWNER_RX_SPI, 0 as libc::c_int as uint8_t);
    IOConfigGPIO(antSelPin,
                 (GPIO_Mode_OUT as libc::c_int |
                      (0 as libc::c_int) << 2 as libc::c_int |
                      (GPIO_OType_PP as libc::c_int) << 4 as libc::c_int |
                      (GPIO_PuPd_NOPULL as libc::c_int) << 5 as libc::c_int)
                     as ioConfig_t);
    // USE_RX_FRSKY_SPI_PA_LNA
    bindPin =
        IOGetByTag(((2 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 13 as libc::c_int) as ioTag_t);
    IOInit(bindPin, OWNER_RX_BIND, 0 as libc::c_int as uint8_t);
    IOConfigGPIO(bindPin,
                 (GPIO_Mode_IN as libc::c_int |
                      (0 as libc::c_int) << 2 as libc::c_int |
                      (0 as libc::c_int) << 4 as libc::c_int |
                      (GPIO_PuPd_UP as libc::c_int) << 5 as libc::c_int) as
                     ioConfig_t);
    lastBindPinStatus = IORead(bindPin);
    IOHi(antSelPin);
    TxDisable();
    // USE_RX_FRSKY_SPI_PA_LNA
    missingPackets = 0 as libc::c_int as uint32_t;
    timeoutUs = 50 as libc::c_int;
    start_time = millis();
    protocolState = STATE_INIT as libc::c_int as uint8_t;
    return 1 as libc::c_int != 0;
}
