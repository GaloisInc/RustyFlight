use core;
use libc;
extern "C" {
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
    // preprocessor is used to convert pinid to requested C data value
// compile-time error is generated if requested pin is not available (not set in TARGET_IO_PORTx)
// ioTag_t and IO_t is supported, but ioTag_t is preferred
    // expand pinid to to ioTag_t
    // TODO
    // declare available IO pins. Available pins are specified per target
    // unimplemented
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    static mut spiPreinitIPUConfig_SystemArray: [spiCs_t; 11];
    #[no_mangle]
    static mut spiPreinitOPUConfig_SystemArray: [spiCs_t; 2];
}
pub type __uint8_t = libc::c_uchar;
pub type uint8_t = __uint8_t;
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
// Place holder for CS pins for pre-initialization
pub type spiCs_t = spiCs_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct spiCs_s {
    pub csnTag: ioTag_t,
}
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
#[inline]
unsafe extern "C" fn spiPreinitIPUConfig(mut _index: libc::c_int)
 -> *const spiCs_t {
    return &mut *spiPreinitIPUConfig_SystemArray.as_mut_ptr().offset(_index as
                                                                         isize)
               as *mut spiCs_t;
}
#[inline]
unsafe extern "C" fn spiPreinitOPUConfig(mut _index: libc::c_int)
 -> *const spiCs_t {
    return &mut *spiPreinitOPUConfig_SystemArray.as_mut_ptr().offset(_index as
                                                                         isize)
               as *mut spiCs_t;
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
// Bring a pin for possible CS line to pull-up state in preparation for
// sequential initialization by relevant drivers.
// There are two versions locally:
// spiPreInitCsIPU set the pin to input with pullup (IOCFG_IPU) for safety.
// spiPreInitCsOPU actually drive the pin for digital hi.
//
// The later is required for SPI slave devices on some targets, interfaced through level shifters, such as Kakute F4.
//
// Two ioTag_t array PGs, spiPreinitIPUConfig and spiPreinitOPUConfig are used to
// determine pin to be IPU or OPU.
// The IPU array is initialized with a hard coded initialization array,
// while the OPU array is initialized from target dependent config.c.
// With generic targets, both arrays are setup with resource commands.
unsafe extern "C" fn spiPreInitCsIPU(mut iotag: ioTag_t,
                                     mut index: libc::c_int) {
    let mut io: IO_t = IOGetByTag(iotag);
    if !io.is_null() {
        IOInit(io, OWNER_SPI_PREINIT_IPU, index as uint8_t);
        IOConfigGPIO(io,
                     (GPIO_Mode_IN as libc::c_int | 0i32 << 2i32 |
                          0i32 << 4i32 |
                          (GPIO_PuPd_UP as libc::c_int) << 5i32) as
                         ioConfig_t);
        IOHi(io);
    };
}
unsafe extern "C" fn spiPreInitCsOPU(mut iotag: ioTag_t,
                                     mut index: libc::c_int) {
    let mut io: IO_t = IOGetByTag(iotag);
    if !io.is_null() {
        IOInit(io, OWNER_SPI_PREINIT_OPU, index as uint8_t);
        IOConfigGPIO(io,
                     (GPIO_Mode_OUT as libc::c_int | 0i32 << 2i32 |
                          (GPIO_OType_PP as libc::c_int) << 4i32 |
                          (GPIO_PuPd_NOPULL as libc::c_int) << 5i32) as
                         ioConfig_t);
        IOHi(io);
    };
}
#[no_mangle]
pub unsafe extern "C" fn spiPreInit() {
    let mut i: libc::c_int = 0i32;
    while i < 11i32 {
        if (*spiPreinitIPUConfig(i)).csnTag != 0 {
            spiPreInitCsIPU((*spiPreinitIPUConfig(i)).csnTag, i);
        }
        i += 1
    }
    let mut i_0: libc::c_int = 0i32;
    while i_0 < 2i32 {
        if (*spiPreinitOPUConfig(i_0)).csnTag != 0 {
            spiPreInitCsOPU((*spiPreinitOPUConfig(i_0)).csnTag, i_0);
        }
        i_0 += 1
    };
}
// Back to pre-init state
#[no_mangle]
pub unsafe extern "C" fn spiPreinitCsByIO(mut io: IO_t) {
    let mut i: libc::c_int = 0i32;
    while i < 11i32 {
        if IOGetByTag((*spiPreinitIPUConfig(i)).csnTag) == io {
            spiPreInitCsIPU((*spiPreinitIPUConfig(i)).csnTag, i);
            return
        }
        i += 1
    }
    let mut i_0: libc::c_int = 0i32;
    while i_0 < 2i32 {
        if IOGetByTag((*spiPreinitOPUConfig(i_0)).csnTag) == io {
            spiPreInitCsOPU((*spiPreinitOPUConfig(i_0)).csnTag, i_0);
            return
        }
        i_0 += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn spiPreinitCsByTag(mut iotag: ioTag_t) {
    spiPreinitCsByIO(IOGetByTag(iotag));
}
