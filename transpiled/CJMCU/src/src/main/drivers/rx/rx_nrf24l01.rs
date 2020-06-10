use ::libc;
extern "C" {
    #[no_mangle]
    fn spiDeviceByInstance(instance: *mut SPI_TypeDef) -> SPIDevice;
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    static mut ioRecs: [ioRec_t; 0];
    #[no_mangle]
    fn rxSpiTransferByte(data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn rxSpiWriteByte(data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn rxSpiWriteCommand(command: uint8_t, data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn rxSpiWriteCommandMulti(command: uint8_t, data: *const uint8_t,
                              length: uint8_t) -> uint8_t;
    #[no_mangle]
    fn rxSpiReadCommand(command: uint8_t, commandData: uint8_t) -> uint8_t;
    #[no_mangle]
    fn rxSpiReadCommandMulti(command: uint8_t, commandData: uint8_t,
                             retData: *mut uint8_t, length: uint8_t)
     -> uint8_t;
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
    fn micros() -> timeUs_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* * 
  * @brief General Purpose I/O
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub CRL: uint32_t,
    pub CRH: uint32_t,
    pub IDR: uint32_t,
    pub ODR: uint32_t,
    pub BSRR: uint32_t,
    pub BRR: uint32_t,
    pub LCKR: uint32_t,
}
/* * 
  * @brief Serial Peripheral Interface
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED1: uint16_t,
    pub SR: uint16_t,
    pub RESERVED2: uint16_t,
    pub DR: uint16_t,
    pub RESERVED3: uint16_t,
    pub CRCPR: uint16_t,
    pub RESERVED4: uint16_t,
    pub RXCRCR: uint16_t,
    pub RESERVED5: uint16_t,
    pub TXCRCR: uint16_t,
    pub RESERVED6: uint16_t,
    pub I2SCFGR: uint16_t,
    pub RESERVED7: uint16_t,
    pub I2SPR: uint16_t,
    pub RESERVED8: uint16_t,
}
/* *
  ******************************************************************************
  * @file    stm32f10x_gpio.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the GPIO 
  *          firmware library.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup GPIO
  * @{
  */
/* * @defgroup GPIO_Exported_Types
  * @{
  */
/* * 
  * @brief  Output Maximum frequency selection  
  */
pub type C2RustUnnamed = libc::c_uint;
pub const GPIO_Speed_50MHz: C2RustUnnamed = 3;
pub const GPIO_Speed_2MHz: C2RustUnnamed = 2;
pub const GPIO_Speed_10MHz: C2RustUnnamed = 1;
/* * 
  * @brief  Configuration Mode enumeration  
  */
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GPIO_Mode_AF_PP: C2RustUnnamed_0 = 24;
pub const GPIO_Mode_AF_OD: C2RustUnnamed_0 = 28;
pub const GPIO_Mode_Out_PP: C2RustUnnamed_0 = 16;
pub const GPIO_Mode_Out_OD: C2RustUnnamed_0 = 20;
pub const GPIO_Mode_IPU: C2RustUnnamed_0 = 72;
pub const GPIO_Mode_IPD: C2RustUnnamed_0 = 40;
pub const GPIO_Mode_IN_FLOATING: C2RustUnnamed_0 = 4;
pub const GPIO_Mode_AIN: C2RustUnnamed_0 = 0;
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
pub type SPIDevice = libc::c_int;
pub const SPIDEV_4: SPIDevice = 3;
pub const SPIDEV_3: SPIDevice = 2;
pub const SPIDEV_2: SPIDevice = 1;
pub const SPIDEV_1: SPIDevice = 0;
pub const SPIINVALID: SPIDevice = -1;
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
pub struct ioRec_s {
    pub gpio: *mut GPIO_TypeDef,
    pub pin: uint16_t,
    pub owner: resourceOwner_e,
    pub index: uint8_t,
}
pub type ioRec_t = ioRec_s;
// microsecond time
pub type timeUs_t = uint32_t;
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
// This file is copied with modifications from project Deviation,
// see http://deviationtx.com, file iface_nrf24l01.h
// bit value
// Register map of nRF24L01
pub type C2RustUnnamed_1 = libc::c_uint;
// DYNamic PayloaD
pub const NRF24L01_1D_FEATURE: C2RustUnnamed_1 = 29;
pub const NRF24L01_1C_DYNPD: C2RustUnnamed_1 = 28;
pub const NRF24L01_17_FIFO_STATUS: C2RustUnnamed_1 = 23;
pub const NRF24L01_16_RX_PW_P5: C2RustUnnamed_1 = 22;
pub const NRF24L01_15_RX_PW_P4: C2RustUnnamed_1 = 21;
pub const NRF24L01_14_RX_PW_P3: C2RustUnnamed_1 = 20;
pub const NRF24L01_13_RX_PW_P2: C2RustUnnamed_1 = 19;
// Payload Width
pub const NRF24L01_12_RX_PW_P1: C2RustUnnamed_1 = 18;
pub const NRF24L01_11_RX_PW_P0: C2RustUnnamed_1 = 17;
pub const NRF24L01_10_TX_ADDR: C2RustUnnamed_1 = 16;
pub const NRF24L01_0F_RX_ADDR_P5: C2RustUnnamed_1 = 15;
pub const NRF24L01_0E_RX_ADDR_P4: C2RustUnnamed_1 = 14;
pub const NRF24L01_0D_RX_ADDR_P3: C2RustUnnamed_1 = 13;
pub const NRF24L01_0C_RX_ADDR_P2: C2RustUnnamed_1 = 12;
pub const NRF24L01_0B_RX_ADDR_P1: C2RustUnnamed_1 = 11;
//Received Power Detector in the nRF23L01+, called CD (Carrier Detect) in the nRF24L01
pub const NRF24L01_0A_RX_ADDR_P0: C2RustUnnamed_1 = 10;
pub const NRF24L01_09_RPD: C2RustUnnamed_1 = 9;
pub const NRF24L01_08_OBSERVE_TX: C2RustUnnamed_1 = 8;
pub const NRF24L01_07_STATUS: C2RustUnnamed_1 = 7;
// RF CHannel
pub const NRF24L01_06_RF_SETUP: C2RustUnnamed_1 = 6;
// automatic RETRansmission
pub const NRF24L01_05_RF_CH: C2RustUnnamed_1 = 5;
// Address Width
pub const NRF24L01_04_SETUP_RETR: C2RustUnnamed_1 = 4;
pub const NRF24L01_03_SETUP_AW: C2RustUnnamed_1 = 3;
// Auto Acknowledge
pub const NRF24L01_02_EN_RXADDR: C2RustUnnamed_1 = 2;
pub const NRF24L01_01_EN_AA: C2RustUnnamed_1 = 1;
pub const NRF24L01_00_CONFIG: C2RustUnnamed_1 = 0;
// Bit position mnemonics
pub type C2RustUnnamed_2 = libc::c_uint;
pub const NRF24L01_1D_FEATURE_EN_DYN_ACK: C2RustUnnamed_2 = 0;
pub const NRF24L01_1D_FEATURE_EN_ACK_PAY: C2RustUnnamed_2 = 1;
pub const NRF24L01_1D_FEATURE_EN_DPL: C2RustUnnamed_2 = 2;
pub const NRF24L01_1C_DYNPD_DPL_P0: C2RustUnnamed_2 = 0;
pub const NRF24L01_1C_DYNPD_DPL_P1: C2RustUnnamed_2 = 1;
pub const NRF24L01_1C_DYNPD_DPL_P2: C2RustUnnamed_2 = 2;
pub const NRF24L01_1C_DYNPD_DPL_P3: C2RustUnnamed_2 = 3;
pub const NRF24L01_1C_DYNPD_DPL_P4: C2RustUnnamed_2 = 4;
pub const NRF24L01_1C_DYNPD_DPL_P5: C2RustUnnamed_2 = 5;
pub const NRF24L01_17_FIFO_STATUS_RX_EMPTY: C2RustUnnamed_2 = 0;
pub const NRF24L01_17_FIFO_STATUS_RX_FULL: C2RustUnnamed_2 = 1;
pub const NRF24L01_17_FIFO_STATUS_TX_EMPTY: C2RustUnnamed_2 = 4;
pub const NRF24L01_17_FIFO_STATUS_TX_FULL: C2RustUnnamed_2 = 5;
pub const NRF24L01_07_STATUS_MAX_RT: C2RustUnnamed_2 = 4;
pub const NRF24L01_07_STATUS_TX_DS: C2RustUnnamed_2 = 5;
pub const NRF24L01_07_STATUS_RX_DR: C2RustUnnamed_2 = 6;
pub const NRF24L01_06_RF_SETUP_RF_PWR_LOW: C2RustUnnamed_2 = 1;
pub const NRF24L01_06_RF_SETUP_RF_PWR_HIGH: C2RustUnnamed_2 = 2;
pub const NRF24L01_06_RF_SETUP_RF_DR_HIGH: C2RustUnnamed_2 = 3;
pub const NRF24L01_06_RF_SETUP_RF_DR_LOW: C2RustUnnamed_2 = 5;
pub const NRF24L01_02_EN_RXADDR_ERX_P0: C2RustUnnamed_2 = 0;
pub const NRF24L01_02_EN_RXADDR_ERX_P1: C2RustUnnamed_2 = 1;
pub const NRF24L01_02_EN_RXADDR_ERX_P2: C2RustUnnamed_2 = 2;
pub const NRF24L01_02_EN_RXADDR_ERX_P3: C2RustUnnamed_2 = 3;
pub const NRF24L01_02_EN_RXADDR_ERX_P4: C2RustUnnamed_2 = 4;
pub const NRF24L01_02_EN_RXADDR_ERX_P5: C2RustUnnamed_2 = 5;
pub const NRF24L01_01_EN_AA_ENAA_P0: C2RustUnnamed_2 = 0;
pub const NRF24L01_01_EN_AA_ENAA_P1: C2RustUnnamed_2 = 1;
pub const NRF24L01_01_EN_AA_ENAA_P2: C2RustUnnamed_2 = 2;
pub const NRF24L01_01_EN_AA_ENAA_P3: C2RustUnnamed_2 = 3;
pub const NRF24L01_01_EN_AA_ENAA_P4: C2RustUnnamed_2 = 4;
pub const NRF24L01_01_EN_AA_ENAA_P5: C2RustUnnamed_2 = 5;
pub const NRF24L01_00_CONFIG_PRIM_RX: C2RustUnnamed_2 = 0;
pub const NRF24L01_00_CONFIG_PWR_UP: C2RustUnnamed_2 = 1;
pub const NRF24L01_00_CONFIG_CRCO: C2RustUnnamed_2 = 2;
pub const NRF24L01_00_CONFIG_EN_CRC: C2RustUnnamed_2 = 3;
pub const NRF24L01_00_CONFIG_MASK_MAX_RT: C2RustUnnamed_2 = 4;
pub const NRF24L01_00_CONFIG_MASK_TX_DS: C2RustUnnamed_2 = 5;
pub const NRF24L01_00_CONFIG_MASK_RX_DR: C2RustUnnamed_2 = 6;
// Pre-shifted and combined bits
pub type C2RustUnnamed_3 = libc::c_uint;
pub const NRF24L01_1C_DYNPD_ALL_PIPES: C2RustUnnamed_3 = 63;
pub const NRF24L01_06_RF_SETUP_RF_PWR_0dbm: C2RustUnnamed_3 = 6;
pub const NRF24L01_06_RF_SETUP_RF_PWR_n6dbm: C2RustUnnamed_3 = 4;
pub const NRF24L01_06_RF_SETUP_RF_PWR_n12dbm: C2RustUnnamed_3 = 2;
pub const NRF24L01_06_RF_SETUP_RF_PWR_n18dbm: C2RustUnnamed_3 = 1;
pub const NRF24L01_06_RF_SETUP_RF_DR_250Kbps: C2RustUnnamed_3 = 32;
pub const NRF24L01_06_RF_SETUP_RF_DR_1Mbps: C2RustUnnamed_3 = 0;
pub const NRF24L01_06_RF_SETUP_RF_DR_2Mbps: C2RustUnnamed_3 = 8;
pub const NRF24L01_04_SETUP_RETR_ARC_15: C2RustUnnamed_3 = 15;
pub const NRF24L01_04_SETUP_RETR_ARC_14: C2RustUnnamed_3 = 14;
pub const NRF24L01_04_SETUP_RETR_ARC_13: C2RustUnnamed_3 = 13;
pub const NRF24L01_04_SETUP_RETR_ARC_12: C2RustUnnamed_3 = 12;
pub const NRF24L01_04_SETUP_RETR_ARC_11: C2RustUnnamed_3 = 11;
pub const NRF24L01_04_SETUP_RETR_ARC_10: C2RustUnnamed_3 = 10;
pub const NRF24L01_04_SETUP_RETR_ARC_9: C2RustUnnamed_3 = 9;
pub const NRF24L01_04_SETUP_RETR_ARC_8: C2RustUnnamed_3 = 8;
pub const NRF24L01_04_SETUP_RETR_ARC_7: C2RustUnnamed_3 = 7;
pub const NRF24L01_04_SETUP_RETR_ARC_6: C2RustUnnamed_3 = 6;
pub const NRF24L01_04_SETUP_RETR_ARC_5: C2RustUnnamed_3 = 5;
pub const NRF24L01_04_SETUP_RETR_ARC_4: C2RustUnnamed_3 = 4;
pub const NRF24L01_04_SETUP_RETR_ARC_3: C2RustUnnamed_3 = 3;
pub const NRF24L01_04_SETUP_RETR_ARC_2: C2RustUnnamed_3 = 2;
pub const NRF24L01_04_SETUP_RETR_ARC_1: C2RustUnnamed_3 = 1;
pub const NRF24L01_04_SETUP_RETR_ARC_0: C2RustUnnamed_3 = 0;
pub const NRF24L01_04_SETUP_RETR_ARD_4000us: C2RustUnnamed_3 = 240;
pub const NRF24L01_04_SETUP_RETR_ARD_3750us: C2RustUnnamed_3 = 224;
pub const NRF24L01_04_SETUP_RETR_ARD_3500us: C2RustUnnamed_3 = 208;
pub const NRF24L01_04_SETUP_RETR_ARD_3250us: C2RustUnnamed_3 = 192;
pub const NRF24L01_04_SETUP_RETR_ARD_3000us: C2RustUnnamed_3 = 176;
pub const NRF24L01_04_SETUP_RETR_ARD_2750us: C2RustUnnamed_3 = 160;
pub const NRF24L01_04_SETUP_RETR_ARD_2500us: C2RustUnnamed_3 = 144;
pub const NRF24L01_04_SETUP_RETR_ARD_2250us: C2RustUnnamed_3 = 128;
pub const NRF24L01_04_SETUP_RETR_ARD_2000us: C2RustUnnamed_3 = 112;
pub const NRF24L01_04_SETUP_RETR_ARD_1750us: C2RustUnnamed_3 = 96;
pub const NRF24L01_04_SETUP_RETR_ARD_1500us: C2RustUnnamed_3 = 80;
pub const NRF24L01_04_SETUP_RETR_ARD_1250us: C2RustUnnamed_3 = 64;
pub const NRF24L01_04_SETUP_RETR_ARD_1000us: C2RustUnnamed_3 = 48;
pub const NRF24L01_04_SETUP_RETR_ARD_750us: C2RustUnnamed_3 = 32;
pub const NRF24L01_04_SETUP_RETR_ARD_500us: C2RustUnnamed_3 = 16;
pub const NRF24L01_04_SETUP_RETR_ARD_250us: C2RustUnnamed_3 = 0;
pub const NRF24L01_03_SETUP_AW_5BYTES: C2RustUnnamed_3 = 3;
pub const NRF24L01_03_SETUP_AW_4BYTES: C2RustUnnamed_3 = 2;
pub const NRF24L01_03_SETUP_AW_3BYTES: C2RustUnnamed_3 = 1;
pub const NRF24L01_02_EN_RXADDR_ERX_ALL_PIPES: C2RustUnnamed_3 = 63;
pub const NRF24L01_01_EN_AA_ALL_PIPES: C2RustUnnamed_3 = 63;
unsafe extern "C" fn NRF24L01_InitGpio() {
    // CE as OUTPUT
    let rxSPIDevice: SPIDevice =
        spiDeviceByInstance((0x40000000 as libc::c_int as
                                 uint32_t).wrapping_add(0x10000 as libc::c_int
                                                            as
                                                            libc::c_uint).wrapping_add(0x3000
                                                                                           as
                                                                                           libc::c_int
                                                                                           as
                                                                                           libc::c_uint)
                                as *mut SPI_TypeDef);
    IOInit(ioRecs.as_mut_ptr().offset((((0xffff as libc::c_int &
                                             ((1 as libc::c_int) <<
                                                  4 as libc::c_int) -
                                                 1 as libc::c_int) -
                                            ((0xffff as libc::c_int &
                                                  ((1 as libc::c_int) <<
                                                       4 as libc::c_int) -
                                                      1 as libc::c_int) >>
                                                 1 as libc::c_int &
                                                 0x77777777 as libc::c_int) -
                                            ((0xffff as libc::c_int &
                                                  ((1 as libc::c_int) <<
                                                       4 as libc::c_int) -
                                                      1 as libc::c_int) >>
                                                 2 as libc::c_int &
                                                 0x33333333 as libc::c_int) -
                                            ((0xffff as libc::c_int &
                                                  ((1 as libc::c_int) <<
                                                       4 as libc::c_int) -
                                                      1 as libc::c_int) >>
                                                 3 as libc::c_int &
                                                 0x11111111 as libc::c_int) +
                                            ((0xffff as libc::c_int &
                                                  ((1 as libc::c_int) <<
                                                       4 as libc::c_int) -
                                                      1 as libc::c_int) -
                                                 ((0xffff as libc::c_int &
                                                       ((1 as libc::c_int) <<
                                                            4 as libc::c_int)
                                                           - 1 as libc::c_int)
                                                      >> 1 as libc::c_int &
                                                      0x77777777 as
                                                          libc::c_int) -
                                                 ((0xffff as libc::c_int &
                                                       ((1 as libc::c_int) <<
                                                            4 as libc::c_int)
                                                           - 1 as libc::c_int)
                                                      >> 2 as libc::c_int &
                                                      0x33333333 as
                                                          libc::c_int) -
                                                 ((0xffff as libc::c_int &
                                                       ((1 as libc::c_int) <<
                                                            4 as libc::c_int)
                                                           - 1 as libc::c_int)
                                                      >> 3 as libc::c_int &
                                                      0x11111111 as
                                                          libc::c_int) >>
                                                 4 as libc::c_int) &
                                            0xf0f0f0f as libc::c_int) %
                                           255 as libc::c_int +
                                           0 as libc::c_int) as isize) as
               IO_t, OWNER_RX_SPI_CS,
           (rxSPIDevice as libc::c_int + 1 as libc::c_int) as uint8_t);
    IOConfigGPIO(ioRecs.as_mut_ptr().offset((((0xffff as libc::c_int &
                                                   ((1 as libc::c_int) <<
                                                        4 as libc::c_int) -
                                                       1 as libc::c_int) -
                                                  ((0xffff as libc::c_int &
                                                        ((1 as libc::c_int) <<
                                                             4 as libc::c_int)
                                                            -
                                                            1 as libc::c_int)
                                                       >> 1 as libc::c_int &
                                                       0x77777777 as
                                                           libc::c_int) -
                                                  ((0xffff as libc::c_int &
                                                        ((1 as libc::c_int) <<
                                                             4 as libc::c_int)
                                                            -
                                                            1 as libc::c_int)
                                                       >> 2 as libc::c_int &
                                                       0x33333333 as
                                                           libc::c_int) -
                                                  ((0xffff as libc::c_int &
                                                        ((1 as libc::c_int) <<
                                                             4 as libc::c_int)
                                                            -
                                                            1 as libc::c_int)
                                                       >> 3 as libc::c_int &
                                                       0x11111111 as
                                                           libc::c_int) +
                                                  ((0xffff as libc::c_int &
                                                        ((1 as libc::c_int) <<
                                                             4 as libc::c_int)
                                                            -
                                                            1 as libc::c_int)
                                                       -
                                                       ((0xffff as libc::c_int
                                                             &
                                                             ((1 as
                                                                   libc::c_int)
                                                                  <<
                                                                  4 as
                                                                      libc::c_int)
                                                                 -
                                                                 1 as
                                                                     libc::c_int)
                                                            >>
                                                            1 as libc::c_int &
                                                            0x77777777 as
                                                                libc::c_int) -
                                                       ((0xffff as libc::c_int
                                                             &
                                                             ((1 as
                                                                   libc::c_int)
                                                                  <<
                                                                  4 as
                                                                      libc::c_int)
                                                                 -
                                                                 1 as
                                                                     libc::c_int)
                                                            >>
                                                            2 as libc::c_int &
                                                            0x33333333 as
                                                                libc::c_int) -
                                                       ((0xffff as libc::c_int
                                                             &
                                                             ((1 as
                                                                   libc::c_int)
                                                                  <<
                                                                  4 as
                                                                      libc::c_int)
                                                                 -
                                                                 1 as
                                                                     libc::c_int)
                                                            >>
                                                            3 as libc::c_int &
                                                            0x11111111 as
                                                                libc::c_int)
                                                       >> 4 as libc::c_int) &
                                                  0xf0f0f0f as libc::c_int) %
                                                 255 as libc::c_int +
                                                 0 as libc::c_int) as isize)
                     as IO_t,
                 (GPIO_Mode_Out_PP as libc::c_int |
                      GPIO_Speed_50MHz as libc::c_int) as ioConfig_t);
    IOLo(ioRecs.as_mut_ptr().offset((((0xffff as libc::c_int &
                                           ((1 as libc::c_int) <<
                                                4 as libc::c_int) -
                                               1 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               1 as libc::c_int &
                                               0x77777777 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               2 as libc::c_int &
                                               0x33333333 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               3 as libc::c_int &
                                               0x11111111 as libc::c_int) +
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    1 as libc::c_int &
                                                    0x77777777 as libc::c_int)
                                               -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    2 as libc::c_int &
                                                    0x33333333 as libc::c_int)
                                               -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    3 as libc::c_int &
                                                    0x11111111 as libc::c_int)
                                               >> 4 as libc::c_int) &
                                          0xf0f0f0f as libc::c_int) %
                                         255 as libc::c_int +
                                         0 as libc::c_int) as isize) as IO_t);
}
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_WriteReg(mut reg: uint8_t,
                                           mut data: uint8_t) -> uint8_t {
    return rxSpiWriteCommand((0x20 as libc::c_int |
                                  0x1f as libc::c_int & reg as libc::c_int) as
                                 uint8_t, data);
}
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_WriteRegisterMulti(mut reg: uint8_t,
                                                     mut data: *const uint8_t,
                                                     mut length: uint8_t)
 -> uint8_t {
    return rxSpiWriteCommandMulti((0x20 as libc::c_int |
                                       0x1f as libc::c_int &
                                           reg as libc::c_int) as uint8_t,
                                  data, length);
}
/*
 * Transfer the payload to the nRF24L01 TX FIFO
 * Packets in the TX FIFO are transmitted when the
 * nRF24L01 next enters TX mode
 */
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_WritePayload(mut data: *const uint8_t,
                                               mut length: uint8_t)
 -> uint8_t {
    return rxSpiWriteCommandMulti(0xa0 as libc::c_int as uint8_t, data,
                                  length);
}
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_WriteAckPayload(mut data: *const uint8_t,
                                                  mut length: uint8_t,
                                                  mut pipe: uint8_t)
 -> uint8_t {
    return rxSpiWriteCommandMulti((0xa8 as libc::c_int |
                                       pipe as libc::c_int &
                                           0x7 as libc::c_int) as uint8_t,
                                  data, length);
}
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_ReadReg(mut reg: uint8_t) -> uint8_t {
    return rxSpiReadCommand((0 as libc::c_int |
                                 0x1f as libc::c_int & reg as libc::c_int) as
                                uint8_t, 0xff as libc::c_int as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_ReadRegisterMulti(mut reg: uint8_t,
                                                    mut data: *mut uint8_t,
                                                    mut length: uint8_t)
 -> uint8_t {
    return rxSpiReadCommandMulti((0 as libc::c_int |
                                      0x1f as libc::c_int &
                                          reg as libc::c_int) as uint8_t,
                                 0xff as libc::c_int as uint8_t, data,
                                 length);
}
/*
 * Read a packet from the nRF24L01 RX FIFO.
 */
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_ReadPayload(mut data: *mut uint8_t,
                                              mut length: uint8_t)
 -> uint8_t {
    return rxSpiReadCommandMulti(0x61 as libc::c_int as uint8_t,
                                 0xff as libc::c_int as uint8_t, data,
                                 length);
}
/*
 * Empty the transmit FIFO buffer.
 */
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_FlushTx() {
    rxSpiWriteByte(0xe1 as libc::c_int as uint8_t);
}
/*
 * Empty the receive FIFO buffer.
 */
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_FlushRx() {
    rxSpiWriteByte(0xe2 as libc::c_int as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_Activate(mut code: uint8_t) -> uint8_t {
    return rxSpiWriteCommand(0x50 as libc::c_int as uint8_t, code);
}
// standby configuration, used to simplify switching between RX, TX, and Standby modes
static mut standbyConfig: uint8_t = 0;
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_Initialize(mut baseConfig: uint8_t) {
    standbyConfig =
        ((1 as libc::c_int) << NRF24L01_00_CONFIG_PWR_UP as libc::c_int |
             baseConfig as libc::c_int) as uint8_t;
    NRF24L01_InitGpio();
    // nRF24L01+ needs 100 milliseconds settling time from PowerOnReset to PowerDown mode
    static mut settlingTimeUs: uint32_t = 100000 as libc::c_int as uint32_t;
    let currentTimeUs: uint32_t = micros();
    if currentTimeUs < settlingTimeUs {
        delayMicroseconds(settlingTimeUs.wrapping_sub(currentTimeUs));
    }
    // now in PowerDown mode
    NRF24L01_WriteReg(NRF24L01_00_CONFIG as libc::c_int as uint8_t,
                      standbyConfig); // set PWR_UP to enter Standby mode
    // nRF24L01+ needs 4500 microseconds from PowerDown mode to Standby mode, for crystal oscillator startup
    delayMicroseconds(4500 as libc::c_int as timeUs_t);
    // now in Standby mode
}
/*
 * Common setup of registers
 */
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_SetupBasic() {
    NRF24L01_WriteReg(NRF24L01_01_EN_AA as libc::c_int as uint8_t,
                      0 as libc::c_int as uint8_t); // No auto acknowledgment
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR as libc::c_int as uint8_t,
                      ((1 as libc::c_int) <<
                           NRF24L01_02_EN_RXADDR_ERX_P0 as libc::c_int) as
                          uint8_t); // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW as libc::c_int as uint8_t,
                      NRF24L01_03_SETUP_AW_5BYTES as libc::c_int as uint8_t);
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD as libc::c_int as uint8_t,
                      0 as libc::c_int as uint8_t);
    // Disable dynamic payload length on all pipes
}
/*
 * Enter standby mode
 */
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_SetStandbyMode() {
    // set CE low and clear the PRIM_RX bit to enter standby mode
    IOLo(ioRecs.as_mut_ptr().offset((((0xffff as libc::c_int &
                                           ((1 as libc::c_int) <<
                                                4 as libc::c_int) -
                                               1 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               1 as libc::c_int &
                                               0x77777777 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               2 as libc::c_int &
                                               0x33333333 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               3 as libc::c_int &
                                               0x11111111 as libc::c_int) +
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    1 as libc::c_int &
                                                    0x77777777 as libc::c_int)
                                               -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    2 as libc::c_int &
                                                    0x33333333 as libc::c_int)
                                               -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    3 as libc::c_int &
                                                    0x11111111 as libc::c_int)
                                               >> 4 as libc::c_int) &
                                          0xf0f0f0f as libc::c_int) %
                                         255 as libc::c_int +
                                         0 as libc::c_int) as isize) as IO_t);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG as libc::c_int as uint8_t,
                      standbyConfig);
}
/*
 * Enter receive mode
 */
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_SetRxMode() {
    IOLo(ioRecs.as_mut_ptr().offset((((0xffff as libc::c_int &
                                           ((1 as libc::c_int) <<
                                                4 as libc::c_int) -
                                               1 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               1 as libc::c_int &
                                               0x77777777 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               2 as libc::c_int &
                                               0x33333333 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               3 as libc::c_int &
                                               0x11111111 as libc::c_int) +
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    1 as libc::c_int &
                                                    0x77777777 as libc::c_int)
                                               -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    2 as libc::c_int &
                                                    0x33333333 as libc::c_int)
                                               -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    3 as libc::c_int &
                                                    0x11111111 as libc::c_int)
                                               >> 4 as libc::c_int) &
                                          0xf0f0f0f as libc::c_int) %
                                         255 as libc::c_int +
                                         0 as libc::c_int) as isize) as IO_t);
    // drop into standby mode
    // set the PRIM_RX bit
    NRF24L01_WriteReg(NRF24L01_00_CONFIG as libc::c_int as uint8_t,
                      (standbyConfig as libc::c_int |
                           (1 as libc::c_int) <<
                               NRF24L01_00_CONFIG_PRIM_RX as libc::c_int) as
                          uint8_t);
    NRF24L01_ClearAllInterrupts();
    // finally set CE high to start enter RX mode
    IOHi(ioRecs.as_mut_ptr().offset((((0xffff as libc::c_int &
                                           ((1 as libc::c_int) <<
                                                4 as libc::c_int) -
                                               1 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               1 as libc::c_int &
                                               0x77777777 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               2 as libc::c_int &
                                               0x33333333 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               3 as libc::c_int &
                                               0x11111111 as libc::c_int) +
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    1 as libc::c_int &
                                                    0x77777777 as libc::c_int)
                                               -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    2 as libc::c_int &
                                                    0x33333333 as libc::c_int)
                                               -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    3 as libc::c_int &
                                                    0x11111111 as libc::c_int)
                                               >> 4 as libc::c_int) &
                                          0xf0f0f0f as libc::c_int) %
                                         255 as libc::c_int +
                                         0 as libc::c_int) as isize) as IO_t);
    // nRF24L01+ will now transition from Standby mode to RX mode after 130 microseconds settling time
}
/*
 * Enter transmit mode. Anything in the transmit FIFO will be transmitted.
 */
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_SetTxMode() {
    // Ensure in standby mode, since can only enter TX mode from standby mode
    NRF24L01_SetStandbyMode();
    NRF24L01_ClearAllInterrupts();
    // pulse CE for 10 microseconds to enter TX mode
    IOHi(ioRecs.as_mut_ptr().offset((((0xffff as libc::c_int &
                                           ((1 as libc::c_int) <<
                                                4 as libc::c_int) -
                                               1 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               1 as libc::c_int &
                                               0x77777777 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               2 as libc::c_int &
                                               0x33333333 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               3 as libc::c_int &
                                               0x11111111 as libc::c_int) +
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    1 as libc::c_int &
                                                    0x77777777 as libc::c_int)
                                               -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    2 as libc::c_int &
                                                    0x33333333 as libc::c_int)
                                               -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    3 as libc::c_int &
                                                    0x11111111 as libc::c_int)
                                               >> 4 as libc::c_int) &
                                          0xf0f0f0f as libc::c_int) %
                                         255 as libc::c_int +
                                         0 as libc::c_int) as isize) as IO_t);
    delayMicroseconds(10 as libc::c_int as timeUs_t);
    IOLo(ioRecs.as_mut_ptr().offset((((0xffff as libc::c_int &
                                           ((1 as libc::c_int) <<
                                                4 as libc::c_int) -
                                               1 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               1 as libc::c_int &
                                               0x77777777 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               2 as libc::c_int &
                                               0x33333333 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               3 as libc::c_int &
                                               0x11111111 as libc::c_int) +
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     4 as libc::c_int) -
                                                    1 as libc::c_int) -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    1 as libc::c_int &
                                                    0x77777777 as libc::c_int)
                                               -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    2 as libc::c_int &
                                                    0x33333333 as libc::c_int)
                                               -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          4 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    3 as libc::c_int &
                                                    0x11111111 as libc::c_int)
                                               >> 4 as libc::c_int) &
                                          0xf0f0f0f as libc::c_int) %
                                         255 as libc::c_int +
                                         0 as libc::c_int) as isize) as IO_t);
    // nRF24L01+ will now transition from Standby mode to TX mode after 130 microseconds settling time.
    // Transmission will then begin and continue until TX FIFO is empty.
}
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_ClearAllInterrupts() {
    // Writing to the STATUS register clears the specified interrupt bits
    NRF24L01_WriteReg(NRF24L01_07_STATUS as libc::c_int as uint8_t,
                      ((1 as libc::c_int) <<
                           NRF24L01_07_STATUS_RX_DR as libc::c_int |
                           (1 as libc::c_int) <<
                               NRF24L01_07_STATUS_TX_DS as libc::c_int |
                           (1 as libc::c_int) <<
                               NRF24L01_07_STATUS_MAX_RT as libc::c_int) as
                          uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_SetChannel(mut channel: uint8_t) {
    NRF24L01_WriteReg(NRF24L01_05_RF_CH as libc::c_int as uint8_t, channel);
}
// Pipes
// Utility functions
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_ReadPayloadIfAvailable(mut data:
                                                             *mut uint8_t,
                                                         mut length: uint8_t)
 -> bool {
    if NRF24L01_ReadReg(NRF24L01_17_FIFO_STATUS as libc::c_int as uint8_t) as
           libc::c_int &
           (1 as libc::c_int) <<
               NRF24L01_17_FIFO_STATUS_RX_EMPTY as libc::c_int != 0 {
        return 0 as libc::c_int != 0
    }
    NRF24L01_ReadPayload(data, length);
    return 1 as libc::c_int != 0;
}
/*
 * Fast read of payload, for use in interrupt service routine
 */
#[no_mangle]
pub unsafe extern "C" fn NRF24L01_ReadPayloadIfAvailableFast(mut data:
                                                                 *mut uint8_t,
                                                             mut length:
                                                                 uint8_t)
 -> bool {
    // number of bits transferred = 8 * (3 + length)
    // for 16 byte payload, that is 8*19 = 152
    // at 50MHz clock rate that is approximately 3 microseconds
    let mut ret: bool = 0 as libc::c_int != 0;
    IOLo(ioRecs.as_mut_ptr().offset((((0xffff as libc::c_int &
                                           ((1 as libc::c_int) <<
                                                11 as libc::c_int) -
                                               1 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     11 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               1 as libc::c_int &
                                               0x77777777 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     11 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               2 as libc::c_int &
                                               0x33333333 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     11 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               3 as libc::c_int &
                                               0x11111111 as libc::c_int) +
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     11 as libc::c_int) -
                                                    1 as libc::c_int) -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          11 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    1 as libc::c_int &
                                                    0x77777777 as libc::c_int)
                                               -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          11 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    2 as libc::c_int &
                                                    0x33333333 as libc::c_int)
                                               -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          11 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    3 as libc::c_int &
                                                    0x11111111 as libc::c_int)
                                               >> 4 as libc::c_int) &
                                          0xf0f0f0f as libc::c_int) %
                                         255 as libc::c_int +
                                         0 as libc::c_int) as isize) as IO_t);
    rxSpiTransferByte((0 as libc::c_int |
                           0x1f as libc::c_int &
                               NRF24L01_07_STATUS as libc::c_int) as uint8_t);
    let status: uint8_t = rxSpiTransferByte(0xff as libc::c_int as uint8_t);
    if status as libc::c_int &
           (1 as libc::c_int) << NRF24L01_07_STATUS_RX_DR as libc::c_int ==
           0 as libc::c_int {
        ret = 1 as libc::c_int != 0;
        // clear RX_DR flag
        rxSpiTransferByte((0x20 as libc::c_int |
                               0x1f as libc::c_int &
                                   NRF24L01_07_STATUS as libc::c_int) as
                              uint8_t);
        rxSpiTransferByte(((1 as libc::c_int) <<
                               NRF24L01_07_STATUS_RX_DR as libc::c_int) as
                              uint8_t);
        rxSpiTransferByte(0x61 as libc::c_int as uint8_t);
        let mut i: uint8_t = 0 as libc::c_int as uint8_t;
        while (i as libc::c_int) < length as libc::c_int {
            *data.offset(i as isize) =
                rxSpiTransferByte(0xff as libc::c_int as uint8_t);
            i = i.wrapping_add(1)
        }
    }
    IOHi(ioRecs.as_mut_ptr().offset((((0xffff as libc::c_int &
                                           ((1 as libc::c_int) <<
                                                11 as libc::c_int) -
                                               1 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     11 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               1 as libc::c_int &
                                               0x77777777 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     11 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               2 as libc::c_int &
                                               0x33333333 as libc::c_int) -
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     11 as libc::c_int) -
                                                    1 as libc::c_int) >>
                                               3 as libc::c_int &
                                               0x11111111 as libc::c_int) +
                                          ((0xffff as libc::c_int &
                                                ((1 as libc::c_int) <<
                                                     11 as libc::c_int) -
                                                    1 as libc::c_int) -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          11 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    1 as libc::c_int &
                                                    0x77777777 as libc::c_int)
                                               -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          11 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    2 as libc::c_int &
                                                    0x33333333 as libc::c_int)
                                               -
                                               ((0xffff as libc::c_int &
                                                     ((1 as libc::c_int) <<
                                                          11 as libc::c_int) -
                                                         1 as libc::c_int) >>
                                                    3 as libc::c_int &
                                                    0x11111111 as libc::c_int)
                                               >> 4 as libc::c_int) &
                                          0xf0f0f0f as libc::c_int) %
                                         255 as libc::c_int +
                                         0 as libc::c_int) as isize) as IO_t);
    return ret;
}
// USE_RX_NRF24
// UNIT_TEST
