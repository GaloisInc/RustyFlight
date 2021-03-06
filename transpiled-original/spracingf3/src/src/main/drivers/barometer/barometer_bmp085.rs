use core;
use libc;
extern "C" {
    #[no_mangle]
    fn busWriteRegister(bus: *const busDevice_t, reg: uint8_t, data: uint8_t)
     -> bool;
    #[no_mangle]
    fn busReadRegisterBuffer(bus: *const busDevice_t, reg: uint8_t,
                             data: *mut uint8_t, length: uint8_t) -> bool;
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn delay(ms: timeMs_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  * @brief Serial Peripheral Interface
  */
#[derive ( Copy, Clone )]
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
    /* !< Reserved, 0x22                                                            */
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
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed_2,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed_2 {
    pub spi: deviceSpi_s,
    pub i2c: deviceI2C_s,
    pub mpuSlave: deviceMpuSlave_s,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct deviceMpuSlave_s {
    pub master: *const busDevice_s,
    pub address: uint8_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct deviceI2C_s {
    pub device: I2CDevice,
    pub address: uint8_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct deviceSpi_s {
    pub instance: *mut SPI_TypeDef,
    pub csnPin: IO_t,
}
pub type busDevice_t = busDevice_s;
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct baroDev_s {
    pub busdev: busDevice_t,
    pub ut_delay: uint16_t,
    pub up_delay: uint16_t,
    pub start_ut: baroOpFuncPtr,
    pub get_ut: baroOpFuncPtr,
    pub start_up: baroOpFuncPtr,
    pub get_up: baroOpFuncPtr,
    pub calculate: baroCalculateFuncPtr,
}
pub type baroCalculateFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut int32_t, _: *mut int32_t) -> ()>;
pub type baroOpFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut baroDev_s) -> ()>;
pub type baroDev_t = baroDev_s;
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
// millisecond time
pub type timeMs_t = uint32_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct bmp085Config_s {
    pub xclrIO: ioTag_t,
    pub eocIO: ioTag_t,
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
pub type bmp085Config_t = bmp085Config_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct bmp085_t {
    pub cal_param: bmp085_smd500_calibration_param_t,
    pub mode: uint8_t,
    pub chip_id: uint8_t,
    pub ml_version: uint8_t,
    pub al_version: uint8_t,
    pub dev_addr: uint8_t,
    pub param_b5: int32_t,
    pub oversampling_setting: int16_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct bmp085_smd500_calibration_param_t {
    pub ac1: int16_t,
    pub ac2: int16_t,
    pub ac3: int16_t,
    pub ac4: uint16_t,
    pub ac5: uint16_t,
    pub ac6: uint16_t,
    pub b1: int16_t,
    pub b2: int16_t,
    pub mb: int16_t,
    pub mc: int16_t,
    pub md: int16_t,
}
//calibration parameter
static mut bmp085: bmp085_t =
    bmp085_t{cal_param:
                 bmp085_smd500_calibration_param_t{ac1: 0,
                                                   ac2: 0,
                                                   ac3: 0,
                                                   ac4: 0,
                                                   ac5: 0,
                                                   ac6: 0,
                                                   b1: 0,
                                                   b2: 0,
                                                   mb: 0,
                                                   mc: 0,
                                                   md: 0,},
             mode: 0,
             chip_id: 0,
             ml_version: 0,
             al_version: 0,
             dev_addr: 0,
             param_b5: 0,
             oversampling_setting: 0,};
// 6000+21000=27000 1.5ms margin according to the spec (25.5ms P conversion time with OSS=3)
static mut bmp085InitDone: bool = 0i32 != 0;
static mut bmp085_ut: uint16_t = 0;
// static result of temperature measurement
static mut bmp085_up: uint32_t = 0;
static mut xclrIO: IO_t = 0 as *const libc::c_void as *mut libc::c_void;
#[no_mangle]
pub unsafe extern "C" fn bmp085InitXclrIO(mut config: *const bmp085Config_t) {
    if xclrIO.is_null() && !config.is_null() &&
           (*config).xclrIO as libc::c_int != 0 {
        xclrIO = IOGetByTag((*config).xclrIO);
        IOInit(xclrIO, OWNER_BARO_CS, 0i32 as uint8_t);
        IOConfigGPIO(xclrIO,
                     (GPIO_Mode_OUT as libc::c_int | 0i32 << 2i32 |
                          (GPIO_OType_PP as libc::c_int) << 4i32 |
                          (GPIO_PuPd_NOPULL as libc::c_int) << 5i32) as
                         ioConfig_t);
    };
}
#[no_mangle]
pub unsafe extern "C" fn bmp085Disable(mut config: *const bmp085Config_t) {
    bmp085InitXclrIO(config);
}
#[no_mangle]
pub unsafe extern "C" fn bmp085Detect(mut config: *const bmp085Config_t,
                                      mut baro: *mut baroDev_t) -> bool {
    let mut data: uint8_t = 0;
    let mut ack: bool = false;
    let mut defaultAddressApplied: bool = 0i32 != 0;
    if bmp085InitDone { return 1i32 != 0 }
    bmp085InitXclrIO(config);
    // enable baro
    delay(20i32 as
              timeMs_t); // datasheet says 10ms, we'll be careful and do 20.
    let mut busdev: *mut busDevice_t = &mut (*baro).busdev;
    if (*busdev).bustype as libc::c_uint ==
           BUSTYPE_I2C as libc::c_int as libc::c_uint &&
           (*busdev).busdev_u.i2c.address as libc::c_int == 0i32 {
        // Default address for BMP085
        (*busdev).busdev_u.i2c.address =
            0x77i32 as uint8_t; /* read Chip Id */
        defaultAddressApplied = 1i32 != 0
    }
    ack =
        busReadRegisterBuffer(busdev, 0xd0i32 as uint8_t, &mut data,
                              1i32 as uint8_t);
    if ack {
        bmp085.chip_id = ((data as libc::c_int & 0xffi32) >> 0i32) as uint8_t;
        bmp085.oversampling_setting = 3i32 as int16_t;
        if bmp085.chip_id as libc::c_int == 0x55i32 {
            /* get bitslice */
            busReadRegisterBuffer(busdev, 0xd1i32 as uint8_t, &mut data,
                                  1i32 as uint8_t); /* read Version reg */
            bmp085.ml_version =
                ((data as libc::c_int & 0xfi32) >> 0i32) as
                    uint8_t; /* get ML Version */
            bmp085.al_version =
                ((data as libc::c_int & 0xf0i32) >> 4i32) as
                    uint8_t; /* get AL Version */
            bmp085_get_cal_param(busdev); /* readout bmp085 calibparam structure */
            (*baro).ut_delay =
                6000i32 as
                    uint16_t; // temperature in 0.01 C (make same as MS5611)
            (*baro).up_delay = 27000i32 as uint16_t;
            (*baro).start_ut =
                Some(bmp085_start_ut as
                         unsafe extern "C" fn(_: *mut baroDev_t) -> ());
            (*baro).get_ut =
                Some(bmp085_get_ut as
                         unsafe extern "C" fn(_: *mut baroDev_t) -> ());
            (*baro).start_up =
                Some(bmp085_start_up as
                         unsafe extern "C" fn(_: *mut baroDev_t) -> ());
            (*baro).get_up =
                Some(bmp085_get_up as
                         unsafe extern "C" fn(_: *mut baroDev_t) -> ());
            (*baro).calculate =
                Some(bmp085_calculate as
                         unsafe extern "C" fn(_: *mut int32_t,
                                              _: *mut int32_t) -> ());
            bmp085InitDone = 1i32 != 0;
            return 1i32 != 0
        }
    }
    if defaultAddressApplied {
        (*busdev).busdev_u.i2c.address = 0i32 as uint8_t
    }
    return 0i32 != 0;
}
unsafe extern "C" fn bmp085_get_temperature(mut ut: uint32_t) -> int32_t {
    let mut temperature: int32_t = 0;
    let mut x1: int32_t = 0;
    let mut x2: int32_t = 0;
    x1 =
        (ut as int32_t - bmp085.cal_param.ac6 as int32_t) *
            bmp085.cal_param.ac5 as int32_t >> 15i32;
    x2 =
        ((bmp085.cal_param.mc as int32_t) << 11i32) /
            (x1 + bmp085.cal_param.md as libc::c_int);
    bmp085.param_b5 = x1 + x2;
    temperature = bmp085.param_b5 * 10i32 + 8i32 >> 4i32;
    return temperature;
}
unsafe extern "C" fn bmp085_get_pressure(mut up: uint32_t) -> int32_t {
    let mut pressure: int32_t = 0;
    let mut x1: int32_t = 0;
    let mut x2: int32_t = 0;
    let mut x3: int32_t = 0;
    let mut b3: int32_t = 0;
    let mut b6: int32_t = 0;
    let mut b4: uint32_t = 0;
    let mut b7: uint32_t = 0;
    b6 = bmp085.param_b5 - 4000i32;
    // *****calculate B3************
    x1 = b6 * b6 >> 12i32;
    x1 *= bmp085.cal_param.b2 as libc::c_int;
    x1 >>= 11i32;
    x2 = bmp085.cal_param.ac2 as libc::c_int * b6;
    x2 >>= 11i32;
    x3 = x1 + x2;
    b3 =
        (bmp085.cal_param.ac1 as int32_t * 4i32 + x3 <<
             bmp085.oversampling_setting as libc::c_int) + 2i32 >> 2i32;
    // *****calculate B4************
    x1 = bmp085.cal_param.ac3 as libc::c_int * b6 >> 13i32; // pressure in Pa
    x2 = bmp085.cal_param.b1 as libc::c_int * (b6 * b6 >> 12i32) >> 16i32;
    x3 = x1 + x2 + 2i32 >> 2i32;
    b4 =
        (bmp085.cal_param.ac4 as
             libc::c_uint).wrapping_mul((x3 + 32768i32) as uint32_t) >> 15i32;
    b7 =
        up.wrapping_sub(b3 as
                            libc::c_uint).wrapping_mul((50000i32 >>
                                                            bmp085.oversampling_setting
                                                                as
                                                                libc::c_int)
                                                           as libc::c_uint);
    if b7 < 0x80000000u32 {
        pressure = (b7 << 1i32).wrapping_div(b4) as int32_t
    } else { pressure = (b7.wrapping_div(b4) << 1i32) as int32_t }
    x1 = pressure >> 8i32;
    x1 *= x1;
    x1 = x1 * 3038i32 >> 16i32;
    x2 = pressure * -7357i32 >> 16i32;
    pressure += x1 + x2 + 3791i32 >> 4i32;
    return pressure;
}
unsafe extern "C" fn bmp085_start_ut(mut baro: *mut baroDev_t) {
    busWriteRegister(&mut (*baro).busdev, 0xf4i32 as uint8_t,
                     0x2ei32 as uint8_t);
}
unsafe extern "C" fn bmp085_get_ut(mut baro: *mut baroDev_t) {
    let mut data: [uint8_t; 2] = [0; 2];
    busReadRegisterBuffer(&mut (*baro).busdev, 0xf6i32 as uint8_t,
                          data.as_mut_ptr(), 2i32 as uint8_t);
    bmp085_ut =
        ((data[0] as libc::c_int) << 8i32 | data[1] as libc::c_int) as
            uint16_t;
}
unsafe extern "C" fn bmp085_start_up(mut baro: *mut baroDev_t) {
    let mut ctrl_reg_data: uint8_t = 0;
    ctrl_reg_data =
        (0x34i32 + ((bmp085.oversampling_setting as libc::c_int) << 6i32)) as
            uint8_t;
    busWriteRegister(&mut (*baro).busdev, 0xf4i32 as uint8_t, ctrl_reg_data);
}
/* * read out up for pressure conversion
 depending on the oversampling ratio setting up can be 16 to 19 bit
 \return up parameter that represents the uncompensated pressure value
 */
unsafe extern "C" fn bmp085_get_up(mut baro: *mut baroDev_t) {
    let mut data: [uint8_t; 3] = [0; 3];
    busReadRegisterBuffer(&mut (*baro).busdev, 0xf6i32 as uint8_t,
                          data.as_mut_ptr(), 3i32 as uint8_t);
    bmp085_up =
        ((data[0] as uint32_t) << 16i32 | (data[1] as uint32_t) << 8i32 |
             data[2] as uint32_t) >>
            8i32 - bmp085.oversampling_setting as libc::c_int;
}
unsafe extern "C" fn bmp085_calculate(mut pressure: *mut int32_t,
                                      mut temperature: *mut int32_t) {
    let mut temp: int32_t = 0;
    let mut press: int32_t = 0;
    temp = bmp085_get_temperature(bmp085_ut as uint32_t);
    press = bmp085_get_pressure(bmp085_up);
    if !pressure.is_null() { *pressure = press }
    if !temperature.is_null() { *temperature = temp };
}
// static result of pressure measurement
unsafe extern "C" fn bmp085_get_cal_param(mut busdev: *mut busDevice_t) {
    let mut data: [uint8_t; 22] = [0; 22];
    busReadRegisterBuffer(busdev, 0xaai32 as uint8_t, data.as_mut_ptr(),
                          22i32 as uint8_t);
    /*parameters AC1-AC6*/
    bmp085.cal_param.ac1 =
        ((data[0] as libc::c_int) << 8i32 | data[1] as libc::c_int) as
            int16_t;
    bmp085.cal_param.ac2 =
        ((data[2] as libc::c_int) << 8i32 | data[3] as libc::c_int) as
            int16_t;
    bmp085.cal_param.ac3 =
        ((data[4] as libc::c_int) << 8i32 | data[5] as libc::c_int) as
            int16_t;
    bmp085.cal_param.ac4 =
        ((data[6] as libc::c_int) << 8i32 | data[7] as libc::c_int) as
            uint16_t;
    bmp085.cal_param.ac5 =
        ((data[8] as libc::c_int) << 8i32 | data[9] as libc::c_int) as
            uint16_t;
    bmp085.cal_param.ac6 =
        ((data[10] as libc::c_int) << 8i32 | data[11] as libc::c_int) as
            uint16_t;
    /*parameters B1,B2*/
    bmp085.cal_param.b1 =
        ((data[12] as libc::c_int) << 8i32 | data[13] as libc::c_int) as
            int16_t;
    bmp085.cal_param.b2 =
        ((data[14] as libc::c_int) << 8i32 | data[15] as libc::c_int) as
            int16_t;
    /*parameters MB,MC,MD*/
    bmp085.cal_param.mb =
        ((data[16] as libc::c_int) << 8i32 | data[17] as libc::c_int) as
            int16_t;
    bmp085.cal_param.mc =
        ((data[18] as libc::c_int) << 8i32 | data[19] as libc::c_int) as
            int16_t;
    bmp085.cal_param.md =
        ((data[20] as libc::c_int) << 8i32 | data[21] as libc::c_int) as
            int16_t;
}
/* BARO */
