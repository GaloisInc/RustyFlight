use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    // declare available IO pins. Available pins are specified per target
    #[no_mangle]
    fn IORead(io: IO_t) -> bool;
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    // [0]: TRX is disabled. [1]: TRX is enabled.
    #[no_mangle]
    fn A7105Init(id: uint32_t);
    #[no_mangle]
    fn A7105Config(regsTable: *const uint8_t, size: uint8_t);
    #[no_mangle]
    fn A7105ReadReg(reg: A7105Reg_t) -> uint8_t;
    #[no_mangle]
    fn A7105WriteReg(reg: A7105Reg_t, data: uint8_t);
    #[no_mangle]
    fn A7105Strobe(state: A7105State_t);
    #[no_mangle]
    fn A7105ReadFIFO(data: *mut uint8_t, num: uint8_t);
    #[no_mangle]
    fn A7105WriteFIFO(data: *mut uint8_t, num: uint8_t);
    #[no_mangle]
    fn A7105RxTxFinished(timeStamp: *mut uint32_t) -> bool;
    #[no_mangle]
    fn pgReset(reg: *const pgRegistry_t);
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn writeEEPROM();
    #[no_mangle]
    static mut rssiSource: rssiSource_e;
    #[no_mangle]
    fn setRssiDirect(newRssi: uint16_t, source: rssiSource_e);
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
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
/* * @defgroup Configuration_Pull-Up_Pull-Down_enumeration 
  * @{
  */
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GPIO_PuPd_DOWN: C2RustUnnamed_0 = 2;
pub const GPIO_PuPd_UP: C2RustUnnamed_0 = 1;
pub const GPIO_PuPd_NOPULL: C2RustUnnamed_0 = 0;
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
pub type A7105State_t = libc::c_uint;
pub const A7105_RST_RDPTR: A7105State_t = 240;
pub const A7105_RST_WRPTR: A7105State_t = 224;
pub const A7105_TX: A7105State_t = 208;
pub const A7105_RX: A7105State_t = 192;
pub const A7105_PLL: A7105State_t = 176;
pub const A7105_STANDBY: A7105State_t = 160;
pub const A7105_IDLE: A7105State_t = 144;
pub const A7105_SLEEP: A7105State_t = 128;
pub type A7105Reg_t = libc::c_uint;
pub const A7105_32_FILTER_TEST: A7105Reg_t = 50;
pub const A7105_31_RSCALE: A7105Reg_t = 49;
pub const A7105_30_IFAT: A7105Reg_t = 48;
pub const A7105_2F_VCO_TEST_II: A7105Reg_t = 47;
pub const A7105_2E_VCO_TEST_I: A7105Reg_t = 46;
pub const A7105_2D_PLL_TEST: A7105Reg_t = 45;
pub const A7105_2C_XTAL_TEST: A7105Reg_t = 44;
pub const A7105_2B_CPC: A7105Reg_t = 43;
pub const A7105_2A_RX_DEM_TEST_II: A7105Reg_t = 42;
pub const A7105_29_RX_DEM_TEST_I: A7105Reg_t = 41;
pub const A7105_28_TX_TEST: A7105Reg_t = 40;
pub const A7105_27_BATTERY_DET: A7105Reg_t = 39;
pub const A7105_26_VCO_SBCAL_II: A7105Reg_t = 38;
pub const A7105_25_VCO_SBCAL_I: A7105Reg_t = 37;
pub const A7105_24_VCO_CURCAL: A7105Reg_t = 36;
pub const A7105_23_IF_CALIB_II: A7105Reg_t = 35;
pub const A7105_22_IF_CALIB_I: A7105Reg_t = 34;
pub const A7105_21_CODE_III: A7105Reg_t = 33;
pub const A7105_20_CODE_II: A7105Reg_t = 32;
pub const A7105_1F_CODE_I: A7105Reg_t = 31;
pub const A7105_1E_ADC: A7105Reg_t = 30;
pub const A7105_1D_RSSI_THOLD: A7105Reg_t = 29;
pub const A7105_1C_RX_GAIN_IV: A7105Reg_t = 28;
pub const A7105_1B_RX_GAIN_III: A7105Reg_t = 27;
pub const A7105_1A_RX_GAIN_II: A7105Reg_t = 26;
pub const A7105_19_RX_GAIN_I: A7105Reg_t = 25;
pub const A7105_18_RX: A7105Reg_t = 24;
pub const A7105_17_DELAY_II: A7105Reg_t = 23;
pub const A7105_16_DELAY_I: A7105Reg_t = 22;
pub const A7105_15_TX_II: A7105Reg_t = 21;
pub const A7105_14_TX_I: A7105Reg_t = 20;
pub const A7105_13_PLL_V: A7105Reg_t = 19;
pub const A7105_12_PLL_IV: A7105Reg_t = 18;
pub const A7105_11_PLL_III: A7105Reg_t = 17;
pub const A7105_10_PLL_II: A7105Reg_t = 16;
pub const A7105_0F_CHANNEL: A7105Reg_t = 15;
pub const A7105_0F_PLL_I: A7105Reg_t = 15;
pub const A7105_0E_DATA_RATE: A7105Reg_t = 14;
pub const A7105_0D_CLOCK: A7105Reg_t = 13;
pub const A7105_0C_GPIO2_PIN_II: A7105Reg_t = 12;
pub const A7105_0B_GPIO1_PIN_I: A7105Reg_t = 11;
pub const A7105_0A_CK0_PIN: A7105Reg_t = 10;
pub const A7105_09_RC_OSC_III: A7105Reg_t = 9;
pub const A7105_08_RC_OSC_II: A7105Reg_t = 8;
pub const A7105_07_RC_OSC_I: A7105Reg_t = 7;
pub const A7105_06_ID_DATA: A7105Reg_t = 6;
pub const A7105_05_FIFO_DATA: A7105Reg_t = 5;
pub const A7105_04_FIFOII: A7105Reg_t = 4;
pub const A7105_03_FIFOI: A7105Reg_t = 3;
pub const A7105_02_CALC: A7105Reg_t = 2;
pub const A7105_01_MODE_CONTROL: A7105Reg_t = 1;
pub const A7105_00_MODE: A7105Reg_t = 0;
pub type pgn_t = uint16_t;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed_1 = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed_1 = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed_1 = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed_1 = 4095;
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
    pub reset: C2RustUnnamed_2,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_2 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
}
pub type pgRegistry_t = pgRegistry_s;
// microsecond time
pub type timeUs_t = uint32_t;
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
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct flySky2ASens_t {
    pub type_0: uint8_t,
    pub number: uint8_t,
    pub valueL: uint8_t,
    pub valueH: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct flySky2ATelemetryPkt_t {
    pub type_0: uint8_t,
    pub txId: uint32_t,
    pub rxId: uint32_t,
    pub sens: [flySky2ASens_t; 7],
}
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct flySky2ABindPkt_t {
    pub type_0: uint8_t,
    pub txId: uint32_t,
    pub rxId: uint32_t,
    pub state: uint8_t,
    pub reserved1: uint8_t,
    pub rfChannelMap: [uint8_t; 16],
    pub reserved2: [uint8_t; 10],
}
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct flySky2ARcDataPkt_t {
    pub type_0: uint8_t,
    pub txId: uint32_t,
    pub rxId: uint32_t,
    pub data: [uint8_t; 28],
}
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct flySkyRcDataPkt_t {
    pub type_0: uint8_t,
    pub txId: uint32_t,
    pub data: [uint8_t; 16],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timings_t {
    pub packet: uint32_t,
    pub firstPacket: uint32_t,
    pub syncPacket: uint32_t,
    pub telemetry: uint32_t,
}
pub type C2RustUnnamed_3 = libc::c_uint;
pub const SENSOR_ERR_RATE: C2RustUnnamed_3 = 254;
pub const SENSOR_RSSI: C2RustUnnamed_3 = 252;
pub const SENSOR_EXT_V: C2RustUnnamed_3 = 3;
pub const SENSOR_MOT_RPM: C2RustUnnamed_3 = 2;
pub const SENSOR_TEMP: C2RustUnnamed_3 = 1;
pub const SENSOR_INT_V: C2RustUnnamed_3 = 0;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const FLYSKY_PACKET_BIND: C2RustUnnamed_4 = 170;
pub const FLYSKY_PACKET_RC_DATA: C2RustUnnamed_4 = 85;
pub const FLYSKY_2A_PACKET_TELEMETRY: C2RustUnnamed_4 = 170;
pub const FLYSKY_2A_PACKET_SETTINGS: C2RustUnnamed_4 = 170;
pub const FLYSKY_2A_PACKET_FS_SETTINGS: C2RustUnnamed_4 = 86;
pub const FLYSKY_2A_PACKET_BIND2: C2RustUnnamed_4 = 188;
pub const FLYSKY_2A_PACKET_BIND1: C2RustUnnamed_4 = 187;
pub const FLYSKY_2A_PACKET_RC_DATA: C2RustUnnamed_4 = 88;
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
pub type rssiSource_e = libc::c_uint;
pub const RSSI_SOURCE_FRAME_ERRORS: rssiSource_e = 5;
pub const RSSI_SOURCE_MSP: rssiSource_e = 4;
pub const RSSI_SOURCE_RX_PROTOCOL: rssiSource_e = 3;
pub const RSSI_SOURCE_RX_CHANNEL: rssiSource_e = 2;
pub const RSSI_SOURCE_ADC: rssiSource_e = 1;
pub const RSSI_SOURCE_NONE: rssiSource_e = 0;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct flySkyConfig_s {
    pub txId: uint32_t,
    pub rfChannelMap: [uint8_t; 16],
    pub protocol: rx_spi_protocol_e,
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
pub type flySkyConfig_t = flySkyConfig_s;
#[inline]
unsafe extern "C" fn flySkyConfig() -> *const flySkyConfig_t {
    return &mut flySkyConfig_System;
}
#[inline]
unsafe extern "C" fn flySkyConfigMutable() -> *mut flySkyConfig_t {
    return &mut flySkyConfig_System;
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
#[no_mangle]
pub static mut flySkyConfig_System: flySkyConfig_t =
    flySkyConfig_t{txId: 0,
                   rfChannelMap: [0; 16],
                   protocol: RX_SPI_NRF24_V202_250K,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut flySkyConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (525 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<flySkyConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &flySkyConfig_System as *const flySkyConfig_t
                                     as *mut flySkyConfig_t as *mut uint8_t,
                             copy:
                                 &flySkyConfig_Copy as *const flySkyConfig_t
                                     as *mut flySkyConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_2{ptr:
                                                     &pgResetTemplate_flySkyConfig
                                                         as
                                                         *const flySkyConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut flySkyConfig_Copy: flySkyConfig_t =
    flySkyConfig_t{txId: 0,
                   rfChannelMap: [0; 16],
                   protocol: RX_SPI_NRF24_V202_250K,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_flySkyConfig: flySkyConfig_t =
    {
        let mut init =
            flySkyConfig_s{txId: 0 as libc::c_int as uint32_t,
                           rfChannelMap:
                               [0 as libc::c_int as uint8_t, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0],
                           protocol: RX_SPI_NRF24_V202_250K,};
        init
    };
static mut flySkyRegs: [uint8_t; 50] =
    [0xff as libc::c_int as uint8_t, 0x42 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xff as libc::c_int as uint8_t,
     0xff as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     0x19 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x50 as libc::c_int as uint8_t,
     0x9e as libc::c_int as uint8_t, 0x4b as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x16 as libc::c_int as uint8_t, 0x2b as libc::c_int as uint8_t,
     0x12 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x62 as libc::c_int as uint8_t, 0x80 as libc::c_int as uint8_t,
     0x80 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0xa as libc::c_int as uint8_t, 0x32 as libc::c_int as uint8_t,
     0xc3 as libc::c_int as uint8_t, 0xf as libc::c_int as uint8_t,
     0x13 as libc::c_int as uint8_t, 0xc3 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xff as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x3b as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x17 as libc::c_int as uint8_t, 0x47 as libc::c_int as uint8_t,
     0x80 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x45 as libc::c_int as uint8_t,
     0x18 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0xf as libc::c_int as uint8_t];
static mut flySky2ARegs: [uint8_t; 50] =
    [0xff as libc::c_int as uint8_t, 0x62 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x25 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xff as libc::c_int as uint8_t,
     0xff as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     0x19 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x50 as libc::c_int as uint8_t,
     0x9e as libc::c_int as uint8_t, 0x4b as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x16 as libc::c_int as uint8_t, 0x2b as libc::c_int as uint8_t,
     0x12 as libc::c_int as uint8_t, 0x4f as libc::c_int as uint8_t,
     0x62 as libc::c_int as uint8_t, 0x80 as libc::c_int as uint8_t,
     0xff as libc::c_int as uint8_t, 0xff as libc::c_int as uint8_t,
     0x2a as libc::c_int as uint8_t, 0x32 as libc::c_int as uint8_t,
     0xc3 as libc::c_int as uint8_t, 0x1f as libc::c_int as uint8_t,
     0x1e as libc::c_int as uint8_t, 0xff as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xff as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x3b as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x17 as libc::c_int as uint8_t, 0x47 as libc::c_int as uint8_t,
     0x80 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x45 as libc::c_int as uint8_t,
     0x18 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0xf as libc::c_int as uint8_t];
static mut flySky2ABindChannels: [uint8_t; 2] =
    [0xd as libc::c_int as uint8_t, 0x8c as libc::c_int as uint8_t];
static mut flySkyRfChannels: [[uint8_t; 16]; 16] =
    [[0xa as libc::c_int as uint8_t, 0x5a as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t, 0x64 as libc::c_int as uint8_t,
      0x1e as libc::c_int as uint8_t, 0x6e as libc::c_int as uint8_t,
      0x28 as libc::c_int as uint8_t, 0x78 as libc::c_int as uint8_t,
      0x32 as libc::c_int as uint8_t, 0x82 as libc::c_int as uint8_t,
      0x3c as libc::c_int as uint8_t, 0x8c as libc::c_int as uint8_t,
      0x46 as libc::c_int as uint8_t, 0x96 as libc::c_int as uint8_t,
      0x50 as libc::c_int as uint8_t, 0xa0 as libc::c_int as uint8_t],
     [0xa0 as libc::c_int as uint8_t, 0x50 as libc::c_int as uint8_t,
      0x96 as libc::c_int as uint8_t, 0x46 as libc::c_int as uint8_t,
      0x8c as libc::c_int as uint8_t, 0x3c as libc::c_int as uint8_t,
      0x82 as libc::c_int as uint8_t, 0x32 as libc::c_int as uint8_t,
      0x78 as libc::c_int as uint8_t, 0x28 as libc::c_int as uint8_t,
      0x6e as libc::c_int as uint8_t, 0x1e as libc::c_int as uint8_t,
      0x64 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x5a as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t],
     [0xa as libc::c_int as uint8_t, 0x5a as libc::c_int as uint8_t,
      0x50 as libc::c_int as uint8_t, 0xa0 as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t, 0x64 as libc::c_int as uint8_t,
      0x46 as libc::c_int as uint8_t, 0x96 as libc::c_int as uint8_t,
      0x1e as libc::c_int as uint8_t, 0x6e as libc::c_int as uint8_t,
      0x3c as libc::c_int as uint8_t, 0x8c as libc::c_int as uint8_t,
      0x28 as libc::c_int as uint8_t, 0x78 as libc::c_int as uint8_t,
      0x32 as libc::c_int as uint8_t, 0x82 as libc::c_int as uint8_t],
     [0x82 as libc::c_int as uint8_t, 0x32 as libc::c_int as uint8_t,
      0x78 as libc::c_int as uint8_t, 0x28 as libc::c_int as uint8_t,
      0x8c as libc::c_int as uint8_t, 0x3c as libc::c_int as uint8_t,
      0x6e as libc::c_int as uint8_t, 0x1e as libc::c_int as uint8_t,
      0x96 as libc::c_int as uint8_t, 0x46 as libc::c_int as uint8_t,
      0x64 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0xa0 as libc::c_int as uint8_t, 0x50 as libc::c_int as uint8_t,
      0x5a as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t],
     [0x28 as libc::c_int as uint8_t, 0x78 as libc::c_int as uint8_t,
      0xa as libc::c_int as uint8_t, 0x5a as libc::c_int as uint8_t,
      0x50 as libc::c_int as uint8_t, 0xa0 as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t, 0x64 as libc::c_int as uint8_t,
      0x1e as libc::c_int as uint8_t, 0x6e as libc::c_int as uint8_t,
      0x3c as libc::c_int as uint8_t, 0x8c as libc::c_int as uint8_t,
      0x32 as libc::c_int as uint8_t, 0x82 as libc::c_int as uint8_t,
      0x46 as libc::c_int as uint8_t, 0x96 as libc::c_int as uint8_t],
     [0x96 as libc::c_int as uint8_t, 0x46 as libc::c_int as uint8_t,
      0x82 as libc::c_int as uint8_t, 0x32 as libc::c_int as uint8_t,
      0x8c as libc::c_int as uint8_t, 0x3c as libc::c_int as uint8_t,
      0x6e as libc::c_int as uint8_t, 0x1e as libc::c_int as uint8_t,
      0x64 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0xa0 as libc::c_int as uint8_t, 0x50 as libc::c_int as uint8_t,
      0x5a as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
      0x78 as libc::c_int as uint8_t, 0x28 as libc::c_int as uint8_t],
     [0x50 as libc::c_int as uint8_t, 0xa0 as libc::c_int as uint8_t,
      0x28 as libc::c_int as uint8_t, 0x78 as libc::c_int as uint8_t,
      0xa as libc::c_int as uint8_t, 0x5a as libc::c_int as uint8_t,
      0x1e as libc::c_int as uint8_t, 0x6e as libc::c_int as uint8_t,
      0x3c as libc::c_int as uint8_t, 0x8c as libc::c_int as uint8_t,
      0x32 as libc::c_int as uint8_t, 0x82 as libc::c_int as uint8_t,
      0x46 as libc::c_int as uint8_t, 0x96 as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t, 0x64 as libc::c_int as uint8_t],
     [0x64 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x96 as libc::c_int as uint8_t, 0x46 as libc::c_int as uint8_t,
      0x82 as libc::c_int as uint8_t, 0x32 as libc::c_int as uint8_t,
      0x8c as libc::c_int as uint8_t, 0x3c as libc::c_int as uint8_t,
      0x6e as libc::c_int as uint8_t, 0x1e as libc::c_int as uint8_t,
      0x5a as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
      0x78 as libc::c_int as uint8_t, 0x28 as libc::c_int as uint8_t,
      0xa0 as libc::c_int as uint8_t, 0x50 as libc::c_int as uint8_t],
     [0x50 as libc::c_int as uint8_t, 0xa0 as libc::c_int as uint8_t,
      0x46 as libc::c_int as uint8_t, 0x96 as libc::c_int as uint8_t,
      0x3c as libc::c_int as uint8_t, 0x8c as libc::c_int as uint8_t,
      0x28 as libc::c_int as uint8_t, 0x78 as libc::c_int as uint8_t,
      0xa as libc::c_int as uint8_t, 0x5a as libc::c_int as uint8_t,
      0x32 as libc::c_int as uint8_t, 0x82 as libc::c_int as uint8_t,
      0x1e as libc::c_int as uint8_t, 0x6e as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t, 0x64 as libc::c_int as uint8_t],
     [0x64 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x6e as libc::c_int as uint8_t, 0x1e as libc::c_int as uint8_t,
      0x82 as libc::c_int as uint8_t, 0x32 as libc::c_int as uint8_t,
      0x5a as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
      0x78 as libc::c_int as uint8_t, 0x28 as libc::c_int as uint8_t,
      0x8c as libc::c_int as uint8_t, 0x3c as libc::c_int as uint8_t,
      0x96 as libc::c_int as uint8_t, 0x46 as libc::c_int as uint8_t,
      0xa0 as libc::c_int as uint8_t, 0x50 as libc::c_int as uint8_t],
     [0x46 as libc::c_int as uint8_t, 0x96 as libc::c_int as uint8_t,
      0x3c as libc::c_int as uint8_t, 0x8c as libc::c_int as uint8_t,
      0x50 as libc::c_int as uint8_t, 0xa0 as libc::c_int as uint8_t,
      0x28 as libc::c_int as uint8_t, 0x78 as libc::c_int as uint8_t,
      0xa as libc::c_int as uint8_t, 0x5a as libc::c_int as uint8_t,
      0x1e as libc::c_int as uint8_t, 0x6e as libc::c_int as uint8_t,
      0x32 as libc::c_int as uint8_t, 0x82 as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t, 0x64 as libc::c_int as uint8_t],
     [0x64 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x82 as libc::c_int as uint8_t, 0x32 as libc::c_int as uint8_t,
      0x6e as libc::c_int as uint8_t, 0x1e as libc::c_int as uint8_t,
      0x5a as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
      0x78 as libc::c_int as uint8_t, 0x28 as libc::c_int as uint8_t,
      0xa0 as libc::c_int as uint8_t, 0x50 as libc::c_int as uint8_t,
      0x8c as libc::c_int as uint8_t, 0x3c as libc::c_int as uint8_t,
      0x96 as libc::c_int as uint8_t, 0x46 as libc::c_int as uint8_t],
     [0x46 as libc::c_int as uint8_t, 0x96 as libc::c_int as uint8_t,
      0xa as libc::c_int as uint8_t, 0x5a as libc::c_int as uint8_t,
      0x3c as libc::c_int as uint8_t, 0x8c as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t, 0x64 as libc::c_int as uint8_t,
      0x50 as libc::c_int as uint8_t, 0xa0 as libc::c_int as uint8_t,
      0x28 as libc::c_int as uint8_t, 0x78 as libc::c_int as uint8_t,
      0x1e as libc::c_int as uint8_t, 0x6e as libc::c_int as uint8_t,
      0x32 as libc::c_int as uint8_t, 0x82 as libc::c_int as uint8_t],
     [0x82 as libc::c_int as uint8_t, 0x32 as libc::c_int as uint8_t,
      0x6e as libc::c_int as uint8_t, 0x1e as libc::c_int as uint8_t,
      0x78 as libc::c_int as uint8_t, 0x28 as libc::c_int as uint8_t,
      0xa0 as libc::c_int as uint8_t, 0x50 as libc::c_int as uint8_t,
      0x64 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x8c as libc::c_int as uint8_t, 0x3c as libc::c_int as uint8_t,
      0x5a as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
      0x96 as libc::c_int as uint8_t, 0x46 as libc::c_int as uint8_t],
     [0x46 as libc::c_int as uint8_t, 0x96 as libc::c_int as uint8_t,
      0xa as libc::c_int as uint8_t, 0x5a as libc::c_int as uint8_t,
      0x50 as libc::c_int as uint8_t, 0xa0 as libc::c_int as uint8_t,
      0x3c as libc::c_int as uint8_t, 0x8c as libc::c_int as uint8_t,
      0x28 as libc::c_int as uint8_t, 0x78 as libc::c_int as uint8_t,
      0x1e as libc::c_int as uint8_t, 0x6e as libc::c_int as uint8_t,
      0x32 as libc::c_int as uint8_t, 0x82 as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t, 0x64 as libc::c_int as uint8_t],
     [0x64 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x82 as libc::c_int as uint8_t, 0x32 as libc::c_int as uint8_t,
      0x6e as libc::c_int as uint8_t, 0x1e as libc::c_int as uint8_t,
      0x78 as libc::c_int as uint8_t, 0x28 as libc::c_int as uint8_t,
      0x8c as libc::c_int as uint8_t, 0x3c as libc::c_int as uint8_t,
      0xa0 as libc::c_int as uint8_t, 0x50 as libc::c_int as uint8_t,
      0x5a as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
      0x96 as libc::c_int as uint8_t, 0x46 as libc::c_int as uint8_t]];
#[no_mangle]
pub static mut flySkyTimings: timings_t =
    {
        let mut init =
            timings_t{packet: 1500 as libc::c_int as uint32_t,
                      firstPacket: 1900 as libc::c_int as uint32_t,
                      syncPacket: 2250 as libc::c_int as uint32_t,
                      telemetry: 0xffffffff as libc::c_uint,};
        init
    };
#[no_mangle]
pub static mut flySky2ATimings: timings_t =
    {
        let mut init =
            timings_t{packet: 3850 as libc::c_int as uint32_t,
                      firstPacket: 4850 as libc::c_int as uint32_t,
                      syncPacket: 5775 as libc::c_int as uint32_t,
                      telemetry: 57000 as libc::c_int as uint32_t,};
        init
    };
static mut protocol: rx_spi_protocol_e = RX_SPI_A7105_FLYSKY_2A;
static mut timings: *const timings_t =
    unsafe { &flySky2ATimings as *const timings_t };
static mut timeout: uint32_t = 0 as libc::c_int as uint32_t;
static mut timeLastPacket: uint32_t = 0 as libc::c_int as uint32_t;
static mut timeLastBind: uint32_t = 0 as libc::c_int as uint32_t;
static mut timeTxRequest: uint32_t = 0 as libc::c_int as uint32_t;
static mut countTimeout: uint32_t = 0 as libc::c_int as uint32_t;
static mut countPacket: uint32_t = 0 as libc::c_int as uint32_t;
static mut txId: uint32_t = 0 as libc::c_int as uint32_t;
static mut rxId: uint32_t = 0 as libc::c_int as uint32_t;
static mut bound: bool = 0 as libc::c_int != 0;
static mut sendTelemetry: bool = 0 as libc::c_int != 0;
static mut waitTx: bool = 0 as libc::c_int != 0;
static mut errorRate: uint16_t = 0 as libc::c_int as uint16_t;
static mut rssi_dBm: uint16_t = 0 as libc::c_int as uint16_t;
static mut rfChannelMap: [uint8_t; 16] =
    [0 as libc::c_int as uint8_t, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0];
/* USE_RX_FLYSKY_SPI_LED */
unsafe extern "C" fn getNextChannel(mut step: uint8_t) -> uint8_t {
    static mut channel: uint8_t = 0 as libc::c_int as uint8_t;
    channel =
        (channel as libc::c_int + step as libc::c_int & 0xf as libc::c_int) as
            uint8_t;
    return rfChannelMap[channel as usize];
}
unsafe extern "C" fn flySkyCalculateRfChannels() {
    let mut channelRow: uint32_t = txId & 0xf as libc::c_int as libc::c_uint;
    let mut channelOffset: uint32_t =
        ((txId & 0xf0 as libc::c_int as libc::c_uint) >>
             4 as libc::c_int).wrapping_add(1 as libc::c_int as libc::c_uint);
    if channelOffset > 9 as libc::c_int as libc::c_uint {
        channelOffset = 9 as libc::c_int as uint32_t
        // from sloped soarer findings, bug in flysky protocol
    } // range about [95...52], -dBm
    let mut i: uint32_t =
        0 as libc::c_int as uint32_t; // convert to [0...1023]
    while i < 16 as libc::c_int as libc::c_uint {
        rfChannelMap[i as usize] =
            (flySkyRfChannels[channelRow as usize][i as usize] as
                 libc::c_uint).wrapping_sub(channelOffset) as
                uint8_t; // first time write full packet to buffer a7105
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn resetTimeout(timeStamp: uint32_t) {
    timeLastPacket = timeStamp;
    timeout = (*timings).firstPacket;
    countTimeout = 0 as libc::c_int as uint32_t;
    countPacket = countPacket.wrapping_add(1);
}
unsafe extern "C" fn checkTimeout() {
    static mut timeMeasuareErrRate: uint32_t = 0 as libc::c_int as uint32_t;
    static mut timeLastTelemetry: uint32_t = 0 as libc::c_int as uint32_t;
    let mut time: uint32_t = micros();
    if time.wrapping_sub(timeMeasuareErrRate) >
           (101 as libc::c_int as
                libc::c_uint).wrapping_mul((*timings).packet) {
        errorRate =
            if countPacket >= 100 as libc::c_int as libc::c_uint {
                0 as libc::c_int as libc::c_uint
            } else {
                (100 as libc::c_int as libc::c_uint).wrapping_sub(countPacket)
            } as uint16_t;
        countPacket = 0 as libc::c_int as uint32_t;
        timeMeasuareErrRate = time
    }
    if time.wrapping_sub(timeLastTelemetry) > (*timings).telemetry {
        timeLastTelemetry = time;
        sendTelemetry = 1 as libc::c_int != 0
    }
    if time.wrapping_sub(timeLastPacket) > timeout {
        let mut stepOver: uint32_t =
            time.wrapping_sub(timeLastPacket).wrapping_div((*timings).packet);
        timeLastPacket =
            if stepOver > 1 as libc::c_int as libc::c_uint {
                time
            } else { timeLastPacket.wrapping_add(timeout) };
        A7105Strobe(A7105_STANDBY);
        A7105WriteReg(A7105_0F_CHANNEL,
                      getNextChannel(stepOver.wrapping_rem(16 as libc::c_int
                                                               as
                                                               libc::c_uint)
                                         as uint8_t));
        A7105Strobe(A7105_RX);
        if countTimeout > 31 as libc::c_int as libc::c_uint {
            timeout = (*timings).syncPacket;
            setRssiDirect(0 as libc::c_int as uint16_t,
                          RSSI_SOURCE_RX_PROTOCOL);
        } else {
            timeout = (*timings).packet;
            countTimeout = countTimeout.wrapping_add(1)
        }
    };
}
unsafe extern "C" fn checkRSSI() {
    static mut buf: [uint8_t; 16] =
        [0 as libc::c_int as uint8_t, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0];
    static mut sum: int16_t = 0 as libc::c_int as int16_t;
    static mut currentIndex: uint16_t = 0 as libc::c_int as uint16_t;
    let mut adcValue: uint8_t = A7105ReadReg(A7105_1D_RSSI_THOLD);
    sum = (sum as libc::c_int + adcValue as libc::c_int) as int16_t;
    sum =
        (sum as libc::c_int - buf[currentIndex as usize] as libc::c_int) as
            int16_t;
    buf[currentIndex as usize] = adcValue;
    currentIndex =
        ((currentIndex as libc::c_int + 1 as libc::c_int) % 16 as libc::c_int)
            as uint16_t;
    rssi_dBm =
        (50 as libc::c_int +
             sum as libc::c_int / (3 as libc::c_int * 16 as libc::c_int)) as
            uint16_t;
    let mut tmp: int16_t =
        (2280 as libc::c_int - 24 as libc::c_int * rssi_dBm as libc::c_int) as
            int16_t;
    setRssiDirect(tmp as uint16_t, RSSI_SOURCE_RX_PROTOCOL);
}
unsafe extern "C" fn isValidPacket(mut packet: *const uint8_t) -> bool {
    let mut rcPacket: *const flySky2ARcDataPkt_t =
        packet as *const flySky2ARcDataPkt_t;
    return (*rcPacket).rxId == rxId && (*rcPacket).txId == txId;
}
unsafe extern "C" fn buildAndWriteTelemetry(mut packet: *mut uint8_t) {
    if !packet.is_null() {
        static mut bytesToWrite: uint8_t = 37 as libc::c_int as uint8_t;
        let mut telemertyPacket: *mut flySky2ATelemetryPkt_t =
            packet as *mut flySky2ATelemetryPkt_t;
        let mut voltage: uint16_t =
            (10 as libc::c_int * getBatteryVoltage() as libc::c_int) as
                uint16_t;
        (*telemertyPacket).type_0 =
            FLYSKY_2A_PACKET_TELEMETRY as libc::c_int as uint8_t;
        (*telemertyPacket).sens[0 as libc::c_int as usize].type_0 =
            SENSOR_INT_V as libc::c_int as uint8_t;
        (*telemertyPacket).sens[0 as libc::c_int as usize].number =
            0 as libc::c_int as uint8_t;
        (*telemertyPacket).sens[0 as libc::c_int as usize].valueL =
            (voltage as libc::c_int & 0xff as libc::c_int) as uint8_t;
        (*telemertyPacket).sens[0 as libc::c_int as usize].valueH =
            (voltage as libc::c_int >> 8 as libc::c_int & 0xff as libc::c_int)
                as uint8_t;
        (*telemertyPacket).sens[1 as libc::c_int as usize].type_0 =
            SENSOR_RSSI as libc::c_int as uint8_t;
        (*telemertyPacket).sens[1 as libc::c_int as usize].number =
            0 as libc::c_int as uint8_t;
        (*telemertyPacket).sens[1 as libc::c_int as usize].valueL =
            (rssi_dBm as libc::c_int & 0xff as libc::c_int) as uint8_t;
        (*telemertyPacket).sens[1 as libc::c_int as usize].valueH =
            (rssi_dBm as libc::c_int >> 8 as libc::c_int &
                 0xff as libc::c_int) as uint8_t;
        (*telemertyPacket).sens[2 as libc::c_int as usize].type_0 =
            SENSOR_ERR_RATE as libc::c_int as uint8_t;
        (*telemertyPacket).sens[2 as libc::c_int as usize].number =
            0 as libc::c_int as uint8_t;
        (*telemertyPacket).sens[2 as libc::c_int as usize].valueL =
            (errorRate as libc::c_int & 0xff as libc::c_int) as uint8_t;
        (*telemertyPacket).sens[2 as libc::c_int as usize].valueH =
            (errorRate as libc::c_int >> 8 as libc::c_int &
                 0xff as libc::c_int) as uint8_t;
        memset(&mut *(*telemertyPacket).sens.as_mut_ptr().offset(3 as
                                                                     libc::c_int
                                                                     as isize)
                   as *mut flySky2ASens_t as *mut libc::c_void,
               0xff as libc::c_int,
               (4 as libc::c_int as
                    libc::c_ulong).wrapping_mul(::core::mem::size_of::<flySky2ASens_t>()
                                                    as libc::c_ulong));
        A7105WriteFIFO(packet, bytesToWrite);
        bytesToWrite =
            (9 as libc::c_int as
                 libc::c_ulong).wrapping_add((3 as libc::c_int as
                                                  libc::c_ulong).wrapping_mul(::core::mem::size_of::<flySky2ASens_t>()
                                                                                  as
                                                                                  libc::c_ulong))
                as uint8_t
    };
}
unsafe extern "C" fn flySky2AReadAndProcess(mut payload: *mut uint8_t,
                                            timeStamp: uint32_t)
 -> rx_spi_received_e {
    let mut result: rx_spi_received_e = RX_SPI_RECEIVED_NONE;
    let mut packet: [uint8_t; 37] = [0; 37];
    let mut bytesToRead: uint8_t =
        if bound as libc::c_int != 0 {
            (9 as libc::c_int) + 2 as libc::c_int * 10 as libc::c_int
        } else { (11 as libc::c_int) + 16 as libc::c_int } as uint8_t;
    A7105ReadFIFO(packet.as_mut_ptr(), bytesToRead);
    match packet[0 as libc::c_int as usize] as libc::c_int {
        88 | 86 | 170 => {
            // failsafe settings
            // receiver settings
            if isValidPacket(packet.as_mut_ptr()) {
                checkRSSI();
                resetTimeout(timeStamp);
                let mut rcPacket: *const flySky2ARcDataPkt_t =
                    packet.as_mut_ptr() as *const flySky2ARcDataPkt_t;
                if (*rcPacket).type_0 as libc::c_int ==
                       FLYSKY_2A_PACKET_RC_DATA as libc::c_int {
                    if !payload.is_null() {
                        memcpy(payload as *mut libc::c_void,
                               (*rcPacket).data.as_ptr() as
                                   *const libc::c_void,
                               (2 as libc::c_int * 10 as libc::c_int) as
                                   libc::c_ulong);
                    }
                    if sendTelemetry {
                        buildAndWriteTelemetry(packet.as_mut_ptr());
                        sendTelemetry = 0 as libc::c_int != 0;
                        timeTxRequest = timeStamp;
                        waitTx = 1 as libc::c_int != 0
                    }
                    result = RX_SPI_RECEIVED_DATA
                }
                if !waitTx {
                    A7105WriteReg(A7105_0F_CHANNEL,
                                  getNextChannel(1 as libc::c_int as
                                                     uint8_t));
                }
            }
        }
        187 | 188 => {
            if !bound {
                resetTimeout(timeStamp);
                let mut bindPacket: *mut flySky2ABindPkt_t =
                    packet.as_mut_ptr() as *mut flySky2ABindPkt_t;
                if (*bindPacket).rfChannelMap[0 as libc::c_int as usize] as
                       libc::c_int != 0xff as libc::c_int {
                    memcpy(rfChannelMap.as_mut_ptr() as *mut libc::c_void,
                           (*bindPacket).rfChannelMap.as_mut_ptr() as
                               *const libc::c_void,
                           16 as libc::c_int as libc::c_ulong);
                    // get TX channels
                } // erase channelMap and 10 bytes after it
                txId = (*bindPacket).txId;
                (*bindPacket).rxId = rxId;
                memset((*bindPacket).rfChannelMap.as_mut_ptr() as
                           *mut libc::c_void, 0xff as libc::c_int,
                       26 as libc::c_int as libc::c_ulong);
                timeLastBind = timeStamp;
                timeTxRequest = timeLastBind;
                waitTx = 1 as libc::c_int != 0;
                A7105WriteFIFO(packet.as_mut_ptr(),
                               37 as libc::c_int as uint8_t);
            }
        }
        _ => { }
    }
    if !waitTx { A7105Strobe(A7105_RX); }
    return result;
}
unsafe extern "C" fn flySkyReadAndProcess(mut payload: *mut uint8_t,
                                          timeStamp: uint32_t)
 -> rx_spi_received_e {
    let mut result: rx_spi_received_e = RX_SPI_RECEIVED_NONE;
    let mut packet: [uint8_t; 21] = [0; 21];
    let mut bytesToRead: uint8_t =
        if bound as libc::c_int != 0 {
            (5 as libc::c_int) + 2 as libc::c_int * 8 as libc::c_int
        } else { 5 as libc::c_int } as uint8_t;
    A7105ReadFIFO(packet.as_mut_ptr(), bytesToRead);
    let mut rcPacket: *const flySkyRcDataPkt_t =
        packet.as_mut_ptr() as *const flySkyRcDataPkt_t;
    if bound as libc::c_int != 0 &&
           (*rcPacket).type_0 as libc::c_int ==
               FLYSKY_PACKET_RC_DATA as libc::c_int &&
           (*rcPacket).txId == txId {
        checkRSSI();
        resetTimeout(timeStamp);
        if !payload.is_null() {
            memcpy(payload as *mut libc::c_void,
                   (*rcPacket).data.as_ptr() as *const libc::c_void,
                   (2 as libc::c_int * 8 as libc::c_int) as libc::c_ulong);
        }
        A7105WriteReg(A7105_0F_CHANNEL,
                      getNextChannel(1 as libc::c_int as uint8_t));
        result = RX_SPI_RECEIVED_DATA
    }
    if !bound &&
           (*rcPacket).type_0 as libc::c_int ==
               FLYSKY_PACKET_BIND as libc::c_int {
        resetTimeout(timeStamp);
        txId = (*rcPacket).txId;
        flySkyCalculateRfChannels();
        A7105WriteReg(A7105_0F_CHANNEL,
                      getNextChannel(0 as libc::c_int as uint8_t));
        timeLastBind = timeStamp
    }
    A7105Strobe(A7105_RX);
    return result;
}
#[no_mangle]
pub unsafe extern "C" fn flySkyInit(mut rxSpiConfig: *const rxSpiConfig_t,
                                    mut rxRuntimeConfig:
                                        *mut rxRuntimeConfig_s) -> bool {
    protocol = (*rxSpiConfig).rx_spi_protocol as rx_spi_protocol_e;
    if protocol as libc::c_uint != (*flySkyConfig()).protocol as libc::c_uint
       {
        extern "C" {
            #[link_name = "flySkyConfig_Registry"]
            static flySkyConfig_Registry_0: pgRegistry_t;
        }
        pgReset(&flySkyConfig_Registry);
    }
    let mut bindPin: IO_t =
        IOGetByTag(((0 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 1 as libc::c_int) as ioTag_t);
    IOInit(bindPin, OWNER_RX_BIND, 0 as libc::c_int as uint8_t);
    IOConfigGPIO(bindPin,
                 (GPIO_Mode_IN as libc::c_int |
                      (0 as libc::c_int) << 2 as libc::c_int |
                      (0 as libc::c_int) << 4 as libc::c_int |
                      (GPIO_PuPd_UP as libc::c_int) << 5 as libc::c_int) as
                     ioConfig_t);
    /* USE_RX_FLYSKY_SPI_LED */
    let mut startRxChannel: uint8_t = 0; // load TXID
    if protocol as libc::c_uint ==
           RX_SPI_A7105_FLYSKY_2A as libc::c_int as libc::c_uint {
        (*rxRuntimeConfig).channelCount =
            10 as libc::c_int as uint8_t; // load channel map
        timings = &flySky2ATimings; // start listening
        rxId =
            *(0x1ffff7ac as libc::c_int as *mut uint32_t) ^
                *(0x1ffff7b0 as libc::c_int as *mut uint32_t) ^
                *(0x1ffff7b4 as libc::c_int as *mut uint32_t);
        startRxChannel = flySky2ABindChannels[0 as libc::c_int as usize];
        A7105Init(0x5475c52a as libc::c_int as uint32_t);
        A7105Config(flySky2ARegs.as_ptr(),
                    ::core::mem::size_of::<[uint8_t; 50]>() as libc::c_ulong
                        as uint8_t);
    } else {
        (*rxRuntimeConfig).channelCount = 8 as libc::c_int as uint8_t;
        timings = &flySkyTimings;
        startRxChannel = 0 as libc::c_int as uint8_t;
        A7105Init(0x5475c52a as libc::c_int as uint32_t);
        A7105Config(flySkyRegs.as_ptr(),
                    ::core::mem::size_of::<[uint8_t; 50]>() as libc::c_ulong
                        as uint8_t);
    }
    if !IORead(bindPin) ||
           (*flySkyConfig()).txId == 0 as libc::c_int as libc::c_uint {
        bound = 0 as libc::c_int != 0
    } else {
        bound = 1 as libc::c_int != 0;
        txId = (*flySkyConfig()).txId;
        memcpy(rfChannelMap.as_mut_ptr() as *mut libc::c_void,
               (*flySkyConfig()).rfChannelMap.as_ptr() as *const libc::c_void,
               16 as libc::c_int as libc::c_ulong);
        startRxChannel = getNextChannel(0 as libc::c_int as uint8_t)
    }
    if rssiSource as libc::c_uint ==
           RSSI_SOURCE_NONE as libc::c_int as libc::c_uint {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL
    }
    A7105WriteReg(A7105_0F_CHANNEL, startRxChannel);
    A7105Strobe(A7105_RX);
    resetTimeout(micros());
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn flySkySetRcDataFromPayload(mut rcData: *mut uint16_t,
                                                    mut payload:
                                                        *const uint8_t) {
    if !rcData.is_null() && !payload.is_null() {
        let mut channelCount: uint32_t = 0;
        channelCount =
            if protocol as libc::c_uint ==
                   RX_SPI_A7105_FLYSKY_2A as libc::c_int as libc::c_uint {
                10 as libc::c_int
            } else { 8 as libc::c_int } as uint32_t;
        let mut i: uint8_t = 0 as libc::c_int as uint8_t;
        while (i as libc::c_uint) < channelCount {
            *rcData.offset(i as isize) =
                ((*payload.offset((2 as libc::c_int * i as libc::c_int +
                                       1 as libc::c_int) as isize) as
                      libc::c_int) << 8 as libc::c_int |
                     *payload.offset((2 as libc::c_int * i as libc::c_int +
                                          0 as libc::c_int) as isize) as
                         libc::c_int) as uint16_t;
            i = i.wrapping_add(1)
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn flySkyDataReceived(mut payload: *mut uint8_t)
 -> rx_spi_received_e {
    /* USE_RX_FLYSKY_SPI_LED */
    let mut result: rx_spi_received_e = RX_SPI_RECEIVED_NONE;
    let mut timeStamp: uint32_t = 0;
    if A7105RxTxFinished(&mut timeStamp) {
        let mut modeReg: uint8_t = A7105ReadReg(A7105_00_MODE);
        if modeReg as libc::c_int & 0x2 as libc::c_int != 0 as libc::c_int &&
               modeReg as libc::c_int & 0x1 as libc::c_int == 0 as libc::c_int
           {
            // TX complete
            if bound {
                A7105WriteReg(A7105_0F_CHANNEL,
                              getNextChannel(1 as libc::c_int as uint8_t));
            }
            A7105Strobe(A7105_RX);
        } else if modeReg as libc::c_int &
                      (0x20 as libc::c_int | 0x1 as libc::c_int) ==
                      0 as libc::c_int {
            // RX complete, CRC pass
            if protocol as libc::c_uint ==
                   RX_SPI_A7105_FLYSKY_2A as libc::c_int as libc::c_uint {
                result = flySky2AReadAndProcess(payload, timeStamp)
            } else { result = flySkyReadAndProcess(payload, timeStamp) }
        } else { A7105Strobe(A7105_RX); }
    }
    if waitTx as libc::c_int != 0 &&
           micros().wrapping_sub(timeTxRequest) >
               500 as libc::c_int as libc::c_uint {
        A7105Strobe(A7105_TX);
        waitTx = 0 as libc::c_int != 0
    }
    if bound {
        checkTimeout();
        /* USE_RX_FLYSKY_SPI_LED */
    } else if micros().wrapping_sub(timeLastBind) >
                  200000 as libc::c_int as libc::c_uint &&
                  rfChannelMap[0 as libc::c_int as usize] as libc::c_int !=
                      0 as libc::c_int &&
                  txId != 0 as libc::c_int as libc::c_uint {
        result = RX_SPI_RECEIVED_BIND; // store TXID
        bound = 1 as libc::c_int != 0; // store channel map
        (*flySkyConfigMutable()).txId = txId;
        memcpy((*flySkyConfigMutable()).rfChannelMap.as_mut_ptr() as
                   *mut libc::c_void,
               rfChannelMap.as_mut_ptr() as *const libc::c_void,
               16 as libc::c_int as libc::c_ulong);
        (*flySkyConfigMutable()).protocol = protocol;
        writeEEPROM();
    }
    return result;
}
/* USE_RX_FLYSKY_SPI_LED */
/* USE_RX_FLYSKY */
