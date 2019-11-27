use core;
use libc;
extern "C" {
    #[no_mangle]
    fn I2C_Init(I2Cx: *mut I2C_TypeDef, I2C_InitStruct: *mut I2C_InitTypeDef);
    #[no_mangle]
    fn I2C_Cmd(I2Cx: *mut I2C_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn I2C_StretchClockCmd(I2Cx: *mut I2C_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn I2C_TransferHandling(I2Cx: *mut I2C_TypeDef, Address: uint16_t,
                            Number_Bytes: uint8_t, ReloadEndMode: uint32_t,
                            StartStopMode: uint32_t);
    /* Data transfers management functions ****************************************/
    #[no_mangle]
    fn I2C_SendData(I2Cx: *mut I2C_TypeDef, Data: uint8_t);
    #[no_mangle]
    fn I2C_ReceiveData(I2Cx: *mut I2C_TypeDef) -> uint8_t;
    /* Interrupts and flags management functions **********************************/
    #[no_mangle]
    fn I2C_GetFlagStatus(I2Cx: *mut I2C_TypeDef, I2C_FLAG: uint32_t)
     -> FlagStatus;
    #[no_mangle]
    fn I2C_ClearFlag(I2Cx: *mut I2C_TypeDef, I2C_FLAG: uint32_t);
    #[no_mangle]
    fn RCC_I2CCLKConfig(RCC_I2CCLK: uint32_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    fn RCC_ClockCmd(periphTag: rccPeriphTag_t, NewState: FunctionalState);
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
  * @brief Inter-integrated Circuit Interface
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct I2C_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub OAR1: uint32_t,
    pub OAR2: uint32_t,
    pub TIMINGR: uint32_t,
    pub TIMEOUTR: uint32_t,
    pub ISR: uint32_t,
    pub ICR: uint32_t,
    pub PECR: uint32_t,
    pub RXDR: uint32_t,
    pub TXDR: uint32_t,
    /* !< I2C Transmit data register,        Address offset: 0x28 */
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
  * @file    stm32f30x_i2c.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the I2C firmware
  *          library.
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
/* * @addtogroup I2C
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* *
  * @brief  I2C Init structure definition
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct I2C_InitTypeDef {
    pub I2C_Timing: uint32_t,
    pub I2C_AnalogFilter: uint32_t,
    pub I2C_DigitalFilter: uint32_t,
    pub I2C_Mode: uint32_t,
    pub I2C_OwnAddress1: uint32_t,
    pub I2C_Ack: uint32_t,
    pub I2C_AcknowledgedAddress: uint32_t,
    /* !< Specifies if 7-bit or 10-bit address is acknowledged.
                                         This parameter can be a value of @ref I2C_acknowledged_address */
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
pub type IO_t = *mut libc::c_void;
// packet tag to specify IO pin
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
pub type rcc_reg = libc::c_uint;
pub const RCC_AHB1: rcc_reg = 4;
pub const RCC_APB1: rcc_reg = 3;
pub const RCC_APB2: rcc_reg = 2;
// make sure that default value (0) does not enable anything
pub const RCC_AHB: rcc_reg = 1;
pub const RCC_EMPTY: rcc_reg = 0;
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
pub type i2cDevice_t = i2cDevice_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct i2cDevice_s {
    pub hardware: *const i2cHardware_t,
    pub reg: *mut I2C_TypeDef,
    pub scl: IO_t,
    pub sda: IO_t,
    pub overClock: bool,
    pub pullUp: bool,
}
pub type i2cHardware_t = i2cHardware_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct i2cHardware_s {
    pub device: I2CDevice,
    pub reg: *mut I2C_TypeDef,
    pub sclPins: [i2cPinDef_t; 4],
    pub sdaPins: [i2cPinDef_t; 4],
    pub rcc: rccPeriphTag_t,
}
pub type i2cPinDef_t = i2cPinDef_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct i2cPinDef_s {
    pub ioTag: ioTag_t,
}
// MCU/Driver dependent member follows
// 400 Khz, 72Mhz Clock, Analog Filter Delay ON, Rise 100, Fall 10.
static mut i2cTimeout: uint32_t = 0;
static mut i2cErrorCount: uint16_t = 0i32 as uint16_t;
// Initialized in run_static_initializers
#[no_mangle]
pub static mut i2cHardware: [i2cHardware_t; 2] =
    [i2cHardware_t{device: I2CDEV_1,
                   reg: 0 as *mut I2C_TypeDef,
                   sclPins: [i2cPinDef_t{ioTag: 0,}; 4],
                   sdaPins: [i2cPinDef_t{ioTag: 0,}; 4],
                   rcc: 0,}; 2];
#[no_mangle]
pub static mut i2cDevice: [i2cDevice_t; 2] =
    [i2cDevice_t{hardware: 0 as *const i2cHardware_t,
                 reg: 0 as *const I2C_TypeDef as *mut I2C_TypeDef,
                 scl: 0 as *const libc::c_void as *mut libc::c_void,
                 sda: 0 as *const libc::c_void as *mut libc::c_void,
                 overClock: false,
                 pullUp: false,}; 2];
// /////////////////////////////////////////////////////////////////////////////
// I2C TimeoutUserCallback
// /////////////////////////////////////////////////////////////////////////////
#[no_mangle]
pub unsafe extern "C" fn i2cTimeoutUserCallback() -> uint32_t {
    ::core::ptr::write_volatile(&mut i2cErrorCount as *mut uint16_t,
                                ::core::ptr::read_volatile::<uint16_t>(&i2cErrorCount
                                                                           as
                                                                           *const uint16_t).wrapping_add(1));
    return 0i32 as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn i2cInit(mut device: I2CDevice) {
    if device as libc::c_int == I2CINVALID as libc::c_int ||
           device as libc::c_int > 2i32 {
        return
    }
    let mut pDev: *mut i2cDevice_t =
        &mut *i2cDevice.as_mut_ptr().offset(device as isize) as
            *mut i2cDevice_t;
    let mut hw: *const i2cHardware_t = (*pDev).hardware;
    if hw.is_null() { return }
    let mut I2Cx: *mut I2C_TypeDef = (*pDev).reg;
    let mut scl: IO_t = (*pDev).scl;
    let mut sda: IO_t = (*pDev).sda;
    RCC_ClockCmd((*hw).rcc, ENABLE);
    RCC_I2CCLKConfig(if I2Cx ==
                            (0x40000000i32 as
                                 uint32_t).wrapping_add(0x5800i32 as
                                                            libc::c_uint) as
                                *mut I2C_TypeDef {
                         0x10000020i32 as uint32_t
                     } else { 0x10i32 as uint32_t });
    IOInit(scl, OWNER_I2C_SCL, (device as libc::c_int + 1i32) as uint8_t);
    IOConfigGPIOAF(scl,
                   if (*pDev).pullUp as libc::c_int != 0 {
                       (GPIO_Mode_AF as libc::c_int |
                            (GPIO_Speed_Level_3 as libc::c_int) << 2i32 |
                            (GPIO_OType_OD as libc::c_int) << 4i32) |
                           (GPIO_PuPd_UP as libc::c_int) << 5i32
                   } else {
                       (GPIO_Mode_AF as libc::c_int |
                            (GPIO_Speed_Level_3 as libc::c_int) << 2i32 |
                            (GPIO_OType_OD as libc::c_int) << 4i32) |
                           (GPIO_PuPd_NOPULL as libc::c_int) << 5i32
                   } as ioConfig_t, 0x4i32 as uint8_t);
    IOInit(sda, OWNER_I2C_SDA, (device as libc::c_int + 1i32) as uint8_t);
    IOConfigGPIOAF(sda,
                   if (*pDev).pullUp as libc::c_int != 0 {
                       (GPIO_Mode_AF as libc::c_int |
                            (GPIO_Speed_Level_3 as libc::c_int) << 2i32 |
                            (GPIO_OType_OD as libc::c_int) << 4i32) |
                           (GPIO_PuPd_UP as libc::c_int) << 5i32
                   } else {
                       (GPIO_Mode_AF as libc::c_int |
                            (GPIO_Speed_Level_3 as libc::c_int) << 2i32 |
                            (GPIO_OType_OD as libc::c_int) << 4i32) |
                           (GPIO_PuPd_NOPULL as libc::c_int) << 5i32
                   } as ioConfig_t, 0x4i32 as uint8_t);
    let mut i2cInit_0: I2C_InitTypeDef =
        {
            let mut init =
                I2C_InitTypeDef{I2C_Timing:
                                    if (*pDev).overClock as libc::c_int != 0 {
                                        0x500e30i32
                                    } else { 0xe0257ai32 } as uint32_t,
                                I2C_AnalogFilter: 0i32 as uint32_t,
                                I2C_DigitalFilter: 0i32 as uint32_t,
                                I2C_Mode: 0i32 as uint32_t,
                                I2C_OwnAddress1: 0i32 as uint32_t,
                                I2C_Ack: 0i32 as uint32_t,
                                I2C_AcknowledgedAddress: 0i32 as uint32_t,};
            init
        };
    I2C_Init(I2Cx, &mut i2cInit_0);
    I2C_StretchClockCmd(I2Cx, ENABLE);
    I2C_Cmd(I2Cx, ENABLE);
}
#[no_mangle]
pub unsafe extern "C" fn i2cGetErrorCounter() -> uint16_t {
    return i2cErrorCount;
}
#[no_mangle]
pub unsafe extern "C" fn i2cWrite(mut device: I2CDevice, mut addr_: uint8_t,
                                  mut reg: uint8_t, mut data: uint8_t)
 -> bool {
    if device as libc::c_int == I2CINVALID as libc::c_int ||
           device as libc::c_int > 2i32 {
        return 0i32 != 0
    }
    let mut I2Cx: *mut I2C_TypeDef = i2cDevice[device as usize].reg;
    if I2Cx.is_null() { return 0i32 != 0 }
    addr_ = ((addr_ as libc::c_int) << 1i32) as uint8_t;
    /* Test on BUSY Flag */
    i2cTimeout = (10i32 as libc::c_uint).wrapping_mul(0x1000i32 as uint32_t);
    while I2C_GetFlagStatus(I2Cx, 0x8000i32 as uint32_t) as libc::c_uint !=
              RESET as libc::c_int as libc::c_uint {
        let fresh0 = i2cTimeout;
        i2cTimeout = i2cTimeout.wrapping_sub(1);
        if fresh0 == 0i32 as libc::c_uint {
            return i2cTimeoutUserCallback() != 0
        }
    }
    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_ as uint16_t, 1i32 as uint8_t,
                         0x1000000i32 as uint32_t, 0x2000i32 as uint32_t);
    /* Wait until TXIS flag is set */
    i2cTimeout = (10i32 as libc::c_uint).wrapping_mul(0x1000i32 as uint32_t);
    while I2C_GetFlagStatus(I2Cx, 0x2i32 as uint32_t) as libc::c_uint ==
              RESET as libc::c_int as libc::c_uint {
        let fresh1 = i2cTimeout;
        i2cTimeout = i2cTimeout.wrapping_sub(1);
        if fresh1 == 0i32 as libc::c_uint {
            return i2cTimeoutUserCallback() != 0
        }
    }
    /* Send Register address */
    I2C_SendData(I2Cx, reg);
    /* Wait until TCR flag is set */
    i2cTimeout = (10i32 as libc::c_uint).wrapping_mul(0x1000i32 as uint32_t);
    while I2C_GetFlagStatus(I2Cx, 0x80i32 as uint32_t) as libc::c_uint ==
              RESET as libc::c_int as libc::c_uint {
        let fresh2 = i2cTimeout;
        i2cTimeout = i2cTimeout.wrapping_sub(1);
        if fresh2 == 0i32 as libc::c_uint {
            return i2cTimeoutUserCallback() != 0
        }
    }
    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_ as uint16_t, 1i32 as uint8_t,
                         0x2000000i32 as uint32_t, 0i32 as uint32_t);
    /* Wait until TXIS flag is set */
    i2cTimeout = (10i32 as libc::c_uint).wrapping_mul(0x1000i32 as uint32_t);
    while I2C_GetFlagStatus(I2Cx, 0x2i32 as uint32_t) as libc::c_uint ==
              RESET as libc::c_int as libc::c_uint {
        let fresh3 = i2cTimeout;
        i2cTimeout = i2cTimeout.wrapping_sub(1);
        if fresh3 == 0i32 as libc::c_uint {
            return i2cTimeoutUserCallback() != 0
        }
    }
    /* Write data to TXDR */
    I2C_SendData(I2Cx, data);
    /* Wait until STOPF flag is set */
    i2cTimeout = (10i32 as libc::c_uint).wrapping_mul(0x1000i32 as uint32_t);
    while I2C_GetFlagStatus(I2Cx, 0x20i32 as uint32_t) as libc::c_uint ==
              RESET as libc::c_int as libc::c_uint {
        let fresh4 = i2cTimeout;
        i2cTimeout = i2cTimeout.wrapping_sub(1);
        if fresh4 == 0i32 as libc::c_uint {
            return i2cTimeoutUserCallback() != 0
        }
    }
    /* Clear STOPF flag */
    I2C_ClearFlag(I2Cx, 0x20i32 as uint32_t);
    return 1i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn i2cRead(mut device: I2CDevice, mut addr_: uint8_t,
                                 mut reg: uint8_t, mut len: uint8_t,
                                 mut buf: *mut uint8_t) -> bool {
    if device as libc::c_int == I2CINVALID as libc::c_int ||
           device as libc::c_int > 2i32 {
        return 0i32 != 0
    }
    let mut I2Cx: *mut I2C_TypeDef = i2cDevice[device as usize].reg;
    if I2Cx.is_null() { return 0i32 != 0 }
    addr_ = ((addr_ as libc::c_int) << 1i32) as uint8_t;
    /* Test on BUSY Flag */
    i2cTimeout = (10i32 as libc::c_uint).wrapping_mul(0x1000i32 as uint32_t);
    while I2C_GetFlagStatus(I2Cx, 0x8000i32 as uint32_t) as libc::c_uint !=
              RESET as libc::c_int as libc::c_uint {
        let fresh5 = i2cTimeout;
        i2cTimeout = i2cTimeout.wrapping_sub(1);
        if fresh5 == 0i32 as libc::c_uint {
            return i2cTimeoutUserCallback() != 0
        }
    }
    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_ as uint16_t, 1i32 as uint8_t,
                         0i32 as uint32_t, 0x2000i32 as uint32_t);
    /* Wait until TXIS flag is set */
    i2cTimeout = (10i32 as libc::c_uint).wrapping_mul(0x1000i32 as uint32_t);
    while I2C_GetFlagStatus(I2Cx, 0x2i32 as uint32_t) as libc::c_uint ==
              RESET as libc::c_int as libc::c_uint {
        let fresh6 = i2cTimeout;
        i2cTimeout = i2cTimeout.wrapping_sub(1);
        if fresh6 == 0i32 as libc::c_uint {
            return i2cTimeoutUserCallback() != 0
        }
    }
    /* Send Register address */
    I2C_SendData(I2Cx, reg);
    /* Wait until TC flag is set */
    i2cTimeout = (10i32 as libc::c_uint).wrapping_mul(0x1000i32 as uint32_t);
    while I2C_GetFlagStatus(I2Cx, 0x40i32 as uint32_t) as libc::c_uint ==
              RESET as libc::c_int as libc::c_uint {
        let fresh7 = i2cTimeout;
        i2cTimeout = i2cTimeout.wrapping_sub(1);
        if fresh7 == 0i32 as libc::c_uint {
            return i2cTimeoutUserCallback() != 0
        }
    }
    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_ as uint16_t, len,
                         0x2000000i32 as uint32_t,
                         0x2000i32 as uint32_t | 0x400i32 as uint32_t);
    /* Wait until all data are received */
    while len != 0 {
        /* Wait until RXNE flag is set */
        i2cTimeout =
            (10i32 as libc::c_uint).wrapping_mul(0x1000i32 as uint32_t);
        while I2C_GetFlagStatus(I2Cx, 0x4i32 as uint32_t) as libc::c_uint ==
                  RESET as libc::c_int as libc::c_uint {
            let fresh8 = i2cTimeout;
            i2cTimeout = i2cTimeout.wrapping_sub(1);
            if fresh8 == 0i32 as libc::c_uint {
                return i2cTimeoutUserCallback() != 0
            }
        }
        /* Read data from RXDR */
        *buf = I2C_ReceiveData(I2Cx);
        /* Point to the next location where the byte read will be saved */
        buf = buf.offset(1);
        /* Decrement the read bytes counter */
        len = len.wrapping_sub(1)
    }
    /* Wait until STOPF flag is set */
    i2cTimeout = (10i32 as libc::c_uint).wrapping_mul(0x1000i32 as uint32_t);
    while I2C_GetFlagStatus(I2Cx, 0x20i32 as uint32_t) as libc::c_uint ==
              RESET as libc::c_int as libc::c_uint {
        let fresh9 = i2cTimeout;
        i2cTimeout = i2cTimeout.wrapping_sub(1);
        if fresh9 == 0i32 as libc::c_uint {
            return i2cTimeoutUserCallback() != 0
        }
    }
    /* Clear STOPF flag */
    I2C_ClearFlag(I2Cx, 0x20i32 as uint32_t);
    /* If all operations OK */
    return 1i32 != 0;
}
unsafe extern "C" fn run_static_initializers() {
    i2cHardware =
        [{
             let mut init =
                 i2cHardware_s{device: I2CDEV_1,
                               reg:
                                   (0x40000000i32 as
                                        uint32_t).wrapping_add(0x5400i32 as
                                                                   libc::c_uint)
                                       as *mut I2C_TypeDef,
                               sclPins:
                                   [{
                                        let mut init =
                                            i2cPinDef_s{ioTag:
                                                            (0i32 + 1i32 <<
                                                                 4i32 | 15i32)
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            i2cPinDef_s{ioTag:
                                                            (1i32 + 1i32 <<
                                                                 4i32 | 6i32)
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            i2cPinDef_s{ioTag:
                                                            (1i32 + 1i32 <<
                                                                 4i32 | 8i32)
                                                                as ioTag_t,};
                                        init
                                    }, i2cPinDef_t{ioTag: 0,}],
                               sdaPins:
                                   [{
                                        let mut init =
                                            i2cPinDef_s{ioTag:
                                                            (0i32 + 1i32 <<
                                                                 4i32 | 14i32)
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            i2cPinDef_s{ioTag:
                                                            (1i32 + 1i32 <<
                                                                 4i32 | 7i32)
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            i2cPinDef_s{ioTag:
                                                            (1i32 + 1i32 <<
                                                                 4i32 | 9i32)
                                                                as ioTag_t,};
                                        init
                                    }, i2cPinDef_t{ioTag: 0,}],
                               rcc:
                                   (((RCC_APB1 as libc::c_int) << 5i32) as
                                        libc::c_long |
                                        (16i32 *
                                             (0x200000i32 as uint32_t as
                                                  libc::c_long > 65535i64) as
                                                 libc::c_int) as libc::c_long
                                            +
                                            ((8i32 *
                                                  (0x200000i32 as uint32_t as
                                                       libc::c_long * 1i64 >>
                                                       16i32 *
                                                           (0x200000i32 as
                                                                uint32_t as
                                                                libc::c_long >
                                                                65535i64) as
                                                               libc::c_int >
                                                       255i32 as libc::c_long)
                                                      as libc::c_int) as
                                                 libc::c_long +
                                                 (8i32 as libc::c_long -
                                                      90i32 as libc::c_long /
                                                          ((0x200000i32 as
                                                                uint32_t as
                                                                libc::c_long *
                                                                1i64 >>
                                                                16i32 *
                                                                    (0x200000i32
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535i64)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8i32 *
                                                                    (0x200000i32
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1i64
                                                                         >>
                                                                         16i32
                                                                             *
                                                                             (0x200000i32
                                                                                  as
                                                                                  uint32_t
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535i64)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255i32
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               4i32 as
                                                                   libc::c_long
                                                               +
                                                               14i32 as
                                                                   libc::c_long
                                                               |
                                                               1i32 as
                                                                   libc::c_long)
                                                      -
                                                      2i32 as libc::c_long /
                                                          ((0x200000i32 as
                                                                uint32_t as
                                                                libc::c_long *
                                                                1i64 >>
                                                                16i32 *
                                                                    (0x200000i32
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535i64)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8i32 *
                                                                    (0x200000i32
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1i64
                                                                         >>
                                                                         16i32
                                                                             *
                                                                             (0x200000i32
                                                                                  as
                                                                                  uint32_t
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535i64)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255i32
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               2i32 as
                                                                   libc::c_long
                                                               +
                                                               1i32 as
                                                                   libc::c_long))))
                                       as rccPeriphTag_t,};
             init
         },
         i2cHardware_t{device: I2CDEV_1,
                       reg: 0 as *mut I2C_TypeDef,
                       sclPins: [i2cPinDef_t{ioTag: 0,}; 4],
                       sdaPins: [i2cPinDef_t{ioTag: 0,}; 4],
                       rcc: 0,}]
}
#[used]
#[cfg_attr ( target_os = "linux", link_section = ".init_array" )]
#[cfg_attr ( target_os = "windows", link_section = ".CRT$XIB" )]
#[cfg_attr ( target_os = "macos", link_section = "__DATA,__mod_init_func" )]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
