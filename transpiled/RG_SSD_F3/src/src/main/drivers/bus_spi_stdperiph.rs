use ::libc;
extern "C" {
    #[no_mangle]
    fn ffs(__i: libc::c_int) -> libc::c_int;
    /* Exported constants --------------------------------------------------------*/
    /* * @defgroup SPI_Exported_Constants
  * @{
  */
    /* * @defgroup SPI_data_direction 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_mode 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_data_size
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_CRC_length
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_Clock_Polarity 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_Clock_Phase 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_Slave_Select_management 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_BaudRate_Prescaler 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_MSB_LSB_transmission 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2S_Mode 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2S_Standard 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2S_Data_Format 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2S_MCLK_Output 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2S_Audio_Frequency 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2S_Clock_Polarity 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_FIFO_reception_threshold 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_I2S_DMA_transfer_requests 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_last_DMA_transfers
  * @{
  */
    /* *
  * @}
  */
/* * @defgroup SPI_NSS_internal_software_management 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_CRC_Transmit_Receive 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_direction_transmit_receive 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_I2S_interrupts_definition 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_transmission_fifo_status_level 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_reception_fifo_status_level 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_I2S_flags_definition 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup SPI_CRC_polynomial 
  * @{
  */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
    /* Function used to set the SPI configuration to the default reset state*******/
    #[no_mangle]
    fn SPI_I2S_DeInit(SPIx: *mut SPI_TypeDef);
    /* Initialization and Configuration functions *********************************/
    #[no_mangle]
    fn SPI_Init(SPIx: *mut SPI_TypeDef, SPI_InitStruct: *mut SPI_InitTypeDef);
    #[no_mangle]
    fn SPI_Cmd(SPIx: *mut SPI_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn SPI_RxFIFOThresholdConfig(SPIx: *mut SPI_TypeDef,
                                 SPI_RxFIFOThreshold: uint16_t);
    /* Data transfers functions ***************************************************/
    #[no_mangle]
    fn SPI_SendData8(SPIx: *mut SPI_TypeDef, Data: uint8_t);
    #[no_mangle]
    fn SPI_ReceiveData8(SPIx: *mut SPI_TypeDef) -> uint8_t;
    #[no_mangle]
    fn SPI_GetTransmissionFIFOStatus(SPIx: *mut SPI_TypeDef) -> uint16_t;
    #[no_mangle]
    fn SPI_I2S_GetFlagStatus(SPIx: *mut SPI_TypeDef, SPI_I2S_FLAG: uint16_t)
     -> FlagStatus;
    #[no_mangle]
    fn spiTimeoutUserCallback(instance: *mut SPI_TypeDef) -> uint32_t;
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    fn RCC_ClockCmd(periphTag: rccPeriphTag_t, NewState: FunctionalState);
    #[no_mangle]
    fn RCC_ResetCmd(periphTag: rccPeriphTag_t, NewState: FunctionalState);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* !< Read Only */
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
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
/* * @defgroup Output_Maximum_frequency_enumeration 
  * @{
  */
pub type C2RustUnnamed_1 = libc::c_uint;
/* !< High Speed     */
/* !< Meduim Speed   */
pub const GPIO_Speed_Level_3: C2RustUnnamed_1 = 3;
/* !< Fast Speed     */
pub const GPIO_Speed_Level_2: C2RustUnnamed_1 = 2;
pub const GPIO_Speed_Level_1: C2RustUnnamed_1 = 1;
/* *
  * @}
  */
/* * @defgroup Configuration_Pull-Up_Pull-Down_enumeration 
  * @{
  */
pub type C2RustUnnamed_2 = libc::c_uint;
pub const GPIO_PuPd_DOWN: C2RustUnnamed_2 = 2;
pub const GPIO_PuPd_UP: C2RustUnnamed_2 = 1;
pub const GPIO_PuPd_NOPULL: C2RustUnnamed_2 = 0;
/* *
  ******************************************************************************
  * @file    stm32f30x_spi.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the SPI 
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
/* * @addtogroup SPI
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  SPI Init structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_InitTypeDef {
    pub SPI_Direction: uint16_t,
    pub SPI_Mode: uint16_t,
    pub SPI_DataSize: uint16_t,
    pub SPI_CPOL: uint16_t,
    pub SPI_CPHA: uint16_t,
    pub SPI_NSS: uint16_t,
    pub SPI_BaudRatePrescaler: uint16_t,
    pub SPI_FirstBit: uint16_t,
    pub SPI_CRCPolynomial: uint16_t,
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
pub type rccPeriphTag_t = uint8_t;
pub type SPIDevice = libc::c_int;
pub const SPIDEV_4: SPIDevice = 3;
pub const SPIDEV_3: SPIDevice = 2;
pub const SPIDEV_2: SPIDevice = 1;
pub const SPIDEV_1: SPIDevice = 0;
pub const SPIINVALID: SPIDevice = -1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPIDevice_s {
    pub dev: *mut SPI_TypeDef,
    pub sck: ioTag_t,
    pub miso: ioTag_t,
    pub mosi: ioTag_t,
    pub af: uint8_t,
    pub rcc: rccPeriphTag_t,
    pub errorCount: uint16_t,
    pub leadingEdge: bool,
}
pub type spiDevice_t = SPIDevice_s;
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
#[no_mangle]
pub static mut spiDevice: [spiDevice_t; 3] =
    [spiDevice_t{dev: 0 as *const SPI_TypeDef as *mut SPI_TypeDef,
                 sck: 0,
                 miso: 0,
                 mosi: 0,
                 af: 0,
                 rcc: 0,
                 errorCount: 0,
                 leadingEdge: false,}; 3];
#[no_mangle]
pub unsafe extern "C" fn spiInitDevice(mut device: SPIDevice) {
    let mut spi: *mut spiDevice_t =
        &mut *spiDevice.as_mut_ptr().offset(device as isize) as
            *mut spiDevice_t;
    if (*spi).dev ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x3000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut SPI_TypeDef {
        (*spi).leadingEdge = 1 as libc::c_int != 0
    }
    // Enable SPI clock
    RCC_ClockCmd((*spi).rcc, ENABLE);
    RCC_ResetCmd((*spi).rcc, ENABLE);
    IOInit(IOGetByTag((*spi).sck), OWNER_SPI_SCK,
           (device as libc::c_int + 1 as libc::c_int) as uint8_t);
    IOInit(IOGetByTag((*spi).miso), OWNER_SPI_MISO,
           (device as libc::c_int + 1 as libc::c_int) as uint8_t);
    IOInit(IOGetByTag((*spi).mosi), OWNER_SPI_MOSI,
           (device as libc::c_int + 1 as libc::c_int) as uint8_t);
    IOConfigGPIOAF(IOGetByTag((*spi).sck),
                   (GPIO_Mode_AF as libc::c_int |
                        (GPIO_Speed_Level_3 as libc::c_int) <<
                            2 as libc::c_int |
                        (GPIO_OType_PP as libc::c_int) << 4 as libc::c_int |
                        (GPIO_PuPd_NOPULL as libc::c_int) << 5 as libc::c_int)
                       as ioConfig_t, (*spi).af);
    IOConfigGPIOAF(IOGetByTag((*spi).miso),
                   (GPIO_Mode_AF as libc::c_int |
                        (GPIO_Speed_Level_3 as libc::c_int) <<
                            2 as libc::c_int |
                        (GPIO_OType_PP as libc::c_int) << 4 as libc::c_int |
                        (GPIO_PuPd_NOPULL as libc::c_int) << 5 as libc::c_int)
                       as ioConfig_t, (*spi).af);
    IOConfigGPIOAF(IOGetByTag((*spi).mosi),
                   (GPIO_Mode_AF as libc::c_int |
                        (GPIO_Speed_Level_3 as libc::c_int) <<
                            2 as libc::c_int |
                        (GPIO_OType_PP as libc::c_int) << 4 as libc::c_int |
                        (GPIO_PuPd_NOPULL as libc::c_int) << 5 as libc::c_int)
                       as ioConfig_t, (*spi).af);
    // Init SPI hardware
    SPI_I2S_DeInit((*spi).dev);
    let mut spiInit: SPI_InitTypeDef =
        SPI_InitTypeDef{SPI_Direction: 0,
                        SPI_Mode: 0,
                        SPI_DataSize: 0,
                        SPI_CPOL: 0,
                        SPI_CPHA: 0,
                        SPI_NSS: 0,
                        SPI_BaudRatePrescaler: 0,
                        SPI_FirstBit: 0,
                        SPI_CRCPolynomial: 0,};
    spiInit.SPI_Mode = 0x104 as libc::c_int as uint16_t;
    spiInit.SPI_Direction = 0 as libc::c_int as uint16_t;
    spiInit.SPI_DataSize = 0x700 as libc::c_int as uint16_t;
    spiInit.SPI_NSS = 0x200 as libc::c_int as uint16_t;
    spiInit.SPI_FirstBit = 0 as libc::c_int as uint16_t;
    spiInit.SPI_CRCPolynomial = 7 as libc::c_int as uint16_t;
    spiInit.SPI_BaudRatePrescaler = 0x10 as libc::c_int as uint16_t;
    if (*spi).leadingEdge {
        spiInit.SPI_CPOL = 0 as libc::c_int as uint16_t;
        spiInit.SPI_CPHA = 0 as libc::c_int as uint16_t
    } else {
        spiInit.SPI_CPOL = 0x2 as libc::c_int as uint16_t;
        spiInit.SPI_CPHA = 0x1 as libc::c_int as uint16_t
    }
    // Configure for 8-bit reads.
    SPI_RxFIFOThresholdConfig((*spi).dev, 0x1000 as libc::c_int as uint16_t);
    SPI_Init((*spi).dev, &mut spiInit);
    SPI_Cmd((*spi).dev, ENABLE);
}
// return uint8_t value or -1 when failure
#[no_mangle]
pub unsafe extern "C" fn spiTransferByte(mut instance: *mut SPI_TypeDef,
                                         mut txByte: uint8_t) -> uint8_t {
    let mut spiTimeout: uint16_t = 1000 as libc::c_int as uint16_t;
    while SPI_I2S_GetFlagStatus(instance, 0x2 as libc::c_int as uint16_t) as
              libc::c_uint == RESET as libc::c_int as libc::c_uint {
        let fresh0 = spiTimeout;
        spiTimeout = spiTimeout.wrapping_sub(1);
        if fresh0 as libc::c_int == 0 as libc::c_int {
            return spiTimeoutUserCallback(instance) as uint8_t
        }
    }
    SPI_SendData8(instance, txByte);
    spiTimeout = 1000 as libc::c_int as uint16_t;
    while SPI_I2S_GetFlagStatus(instance, 0x1 as libc::c_int as uint16_t) as
              libc::c_uint == RESET as libc::c_int as libc::c_uint {
        let fresh1 = spiTimeout;
        spiTimeout = spiTimeout.wrapping_sub(1);
        if fresh1 as libc::c_int == 0 as libc::c_int {
            return spiTimeoutUserCallback(instance) as uint8_t
        }
    }
    return SPI_ReceiveData8(instance);
}
/* *
 * Return true if the bus is currently in the middle of a transmission.
 */
#[no_mangle]
pub unsafe extern "C" fn spiIsBusBusy(mut instance: *mut SPI_TypeDef)
 -> bool {
    return SPI_GetTransmissionFIFOStatus(instance) as libc::c_int !=
               0 as libc::c_int as uint16_t as libc::c_int ||
               SPI_I2S_GetFlagStatus(instance,
                                     0x80 as libc::c_int as uint16_t) as
                   libc::c_uint == SET as libc::c_int as libc::c_uint;
}
#[no_mangle]
pub unsafe extern "C" fn spiTransfer(mut instance: *mut SPI_TypeDef,
                                     mut txData: *const uint8_t,
                                     mut rxData: *mut uint8_t,
                                     mut len: libc::c_int) -> bool {
    let mut spiTimeout: uint16_t = 1000 as libc::c_int as uint16_t;
    let mut b: uint8_t = 0;
    loop  {
        let fresh2 = len;
        len = len - 1;
        if !(fresh2 != 0) { break ; }
        b =
            if !txData.is_null() {
                let fresh3 = txData;
                txData = txData.offset(1);
                *fresh3 as libc::c_int
            } else { 0xff as libc::c_int } as uint8_t;
        while SPI_I2S_GetFlagStatus(instance, 0x2 as libc::c_int as uint16_t)
                  as libc::c_uint == RESET as libc::c_int as libc::c_uint {
            let fresh4 = spiTimeout;
            spiTimeout = spiTimeout.wrapping_sub(1);
            if fresh4 as libc::c_int == 0 as libc::c_int {
                return spiTimeoutUserCallback(instance) != 0
            }
        }
        SPI_SendData8(instance, b);
        spiTimeout = 1000 as libc::c_int as uint16_t;
        while SPI_I2S_GetFlagStatus(instance, 0x1 as libc::c_int as uint16_t)
                  as libc::c_uint == RESET as libc::c_int as libc::c_uint {
            let fresh5 = spiTimeout;
            spiTimeout = spiTimeout.wrapping_sub(1);
            if fresh5 as libc::c_int == 0 as libc::c_int {
                return spiTimeoutUserCallback(instance) != 0
            }
        }
        b = SPI_ReceiveData8(instance);
        if !rxData.is_null() {
            let fresh6 = rxData;
            rxData = rxData.offset(1);
            *fresh6 = b
        }
    }
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn spiSetDivisor(mut instance: *mut SPI_TypeDef,
                                       mut divisor: uint16_t) {
    SPI_Cmd(instance, DISABLE);
    let tempRegister: uint16_t =
        ((*instance).CR1 as libc::c_int &
             !((1 as libc::c_int) << 5 as libc::c_int |
                   (1 as libc::c_int) << 4 as libc::c_int |
                   (1 as libc::c_int) << 3 as libc::c_int)) as uint16_t;
    ::core::ptr::write_volatile(&mut (*instance).CR1 as *mut uint16_t,
                                (tempRegister as libc::c_int |
                                     (if divisor as libc::c_int != 0 {
                                          ((ffs(divisor as libc::c_int |
                                                    0x100 as libc::c_int) -
                                                2 as libc::c_int)) <<
                                              3 as libc::c_int
                                      } else { 0 as libc::c_int })) as
                                    uint16_t);
    SPI_Cmd(instance, ENABLE);
}
