use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
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
    static mut debug: [int16_t; 4];
    #[no_mangle]
    static mut debugMode: uint8_t;
    #[no_mangle]
    fn spiSetDivisor(instance: *mut SPI_TypeDef, divisor: uint16_t);
    #[no_mangle]
    fn spiTransferByte(instance: *mut SPI_TypeDef, data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn spiTransfer(instance: *mut SPI_TypeDef, txData: *const uint8_t,
                   rxData: *mut uint8_t, len: libc::c_int) -> bool;
    #[no_mangle]
    fn spiInstanceByDevice(device: SPIDevice) -> *mut SPI_TypeDef;
    #[no_mangle]
    fn spiBusSetInstance(bus: *mut busDevice_t, instance: *mut SPI_TypeDef);
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOIsFreeOrPreinit(io: IO_t) -> bool;
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
    #[no_mangle]
    fn ledToggle(led: libc::c_int);
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn millis() -> timeMs_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
pub type C2RustUnnamed_3 = libc::c_uint;
pub const DEBUG_COUNT: C2RustUnnamed_3 = 44;
pub const DEBUG_ANTI_GRAVITY: C2RustUnnamed_3 = 43;
pub const DEBUG_RC_SMOOTHING_RATE: C2RustUnnamed_3 = 42;
pub const DEBUG_RX_SIGNAL_LOSS: C2RustUnnamed_3 = 41;
pub const DEBUG_RC_SMOOTHING: C2RustUnnamed_3 = 40;
pub const DEBUG_ACRO_TRAINER: C2RustUnnamed_3 = 39;
pub const DEBUG_ITERM_RELAX: C2RustUnnamed_3 = 38;
pub const DEBUG_RTH: C2RustUnnamed_3 = 37;
pub const DEBUG_SMARTAUDIO: C2RustUnnamed_3 = 36;
pub const DEBUG_USB: C2RustUnnamed_3 = 35;
pub const DEBUG_CURRENT: C2RustUnnamed_3 = 34;
pub const DEBUG_SDIO: C2RustUnnamed_3 = 33;
pub const DEBUG_RUNAWAY_TAKEOFF: C2RustUnnamed_3 = 32;
pub const DEBUG_CORE_TEMP: C2RustUnnamed_3 = 31;
pub const DEBUG_LIDAR_TF: C2RustUnnamed_3 = 30;
pub const DEBUG_RANGEFINDER_QUALITY: C2RustUnnamed_3 = 29;
pub const DEBUG_RANGEFINDER: C2RustUnnamed_3 = 28;
pub const DEBUG_FPORT: C2RustUnnamed_3 = 27;
pub const DEBUG_SBUS: C2RustUnnamed_3 = 26;
pub const DEBUG_MAX7456_SPICLOCK: C2RustUnnamed_3 = 25;
pub const DEBUG_MAX7456_SIGNAL: C2RustUnnamed_3 = 24;
pub const DEBUG_DUAL_GYRO_DIFF: C2RustUnnamed_3 = 23;
pub const DEBUG_DUAL_GYRO_COMBINE: C2RustUnnamed_3 = 22;
pub const DEBUG_DUAL_GYRO_RAW: C2RustUnnamed_3 = 21;
pub const DEBUG_DUAL_GYRO: C2RustUnnamed_3 = 20;
pub const DEBUG_GYRO_RAW: C2RustUnnamed_3 = 19;
pub const DEBUG_RX_FRSKY_SPI: C2RustUnnamed_3 = 18;
pub const DEBUG_FFT_FREQ: C2RustUnnamed_3 = 17;
pub const DEBUG_FFT_TIME: C2RustUnnamed_3 = 16;
pub const DEBUG_FFT: C2RustUnnamed_3 = 15;
pub const DEBUG_ALTITUDE: C2RustUnnamed_3 = 14;
pub const DEBUG_ESC_SENSOR_TMP: C2RustUnnamed_3 = 13;
pub const DEBUG_ESC_SENSOR_RPM: C2RustUnnamed_3 = 12;
pub const DEBUG_STACK: C2RustUnnamed_3 = 11;
pub const DEBUG_SCHEDULER: C2RustUnnamed_3 = 10;
pub const DEBUG_ESC_SENSOR: C2RustUnnamed_3 = 9;
pub const DEBUG_ANGLERATE: C2RustUnnamed_3 = 8;
pub const DEBUG_RC_INTERPOLATION: C2RustUnnamed_3 = 7;
pub const DEBUG_GYRO_SCALED: C2RustUnnamed_3 = 6;
pub const DEBUG_PIDLOOP: C2RustUnnamed_3 = 5;
pub const DEBUG_ACCELEROMETER: C2RustUnnamed_3 = 4;
pub const DEBUG_GYRO_FILTERED: C2RustUnnamed_3 = 3;
pub const DEBUG_BATTERY: C2RustUnnamed_3 = 2;
pub const DEBUG_CYCLETIME: C2RustUnnamed_3 = 1;
pub const DEBUG_NONE: C2RustUnnamed_3 = 0;
pub type ioTag_t = uint8_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct max7456Config_s {
    pub clockConfig: uint8_t,
    pub csTag: ioTag_t,
    pub spiDevice: uint8_t,
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
pub type max7456Config_t = max7456Config_s;
// SPI clock based on device type and overclock state (MAX7456_CLOCK_CONFIG_xxxx)
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
// Video Character Display parameters
pub type VIDEO_SYSTEMS = libc::c_uint;
pub const VIDEO_SYSTEM_NTSC: VIDEO_SYSTEMS = 2;
pub const VIDEO_SYSTEM_PAL: VIDEO_SYSTEMS = 1;
pub const VIDEO_SYSTEM_AUTO: VIDEO_SYSTEMS = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vcdProfile_s {
    pub video_system: uint8_t,
    pub h_offset: int8_t,
    pub v_offset: int8_t,
}
pub type vcdProfile_t = vcdProfile_s;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed_4,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_4 {
    pub spi: deviceSpi_s,
    pub i2c: deviceI2C_s,
    pub mpuSlave: deviceMpuSlave_s,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceMpuSlave_s {
    pub master: *const busDevice_s,
    pub address: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceI2C_s {
    pub device: I2CDevice,
    pub address: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceSpi_s {
    pub instance: *mut SPI_TypeDef,
    pub csnPin: IO_t,
}
pub type busDevice_t = busDevice_s;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const SPI_CLOCK_ULTRAFAST: C2RustUnnamed_5 = 2;
pub const SPI_CLOCK_FAST: C2RustUnnamed_5 = 2;
pub const SPI_CLOCK_STANDARD: C2RustUnnamed_5 = 4;
pub const SPI_CLOCK_SLOW: C2RustUnnamed_5 = 128;
pub const SPI_CLOCK_INITIALIZATON: C2RustUnnamed_5 = 256;
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
pub type timeMs_t = uint32_t;
// XXX Should be related to VIDEO_BUFFER_CHARS_*?
// On shared SPI buss we want to change clock for OSD chip and restore for other devices.
#[no_mangle]
pub static mut max7456BusDevice: busDevice_t =
    busDevice_t{bustype: BUSTYPE_NONE,
                busdev_u:
                    C2RustUnnamed_4{spi:
                                        deviceSpi_s{instance:
                                                        0 as
                                                            *const SPI_TypeDef
                                                            as
                                                            *mut SPI_TypeDef,
                                                    csnPin:
                                                        0 as
                                                            *const libc::c_void
                                                            as
                                                            *mut libc::c_void,},},};
#[no_mangle]
pub static mut busdev: *mut busDevice_t =
    unsafe { &max7456BusDevice as *const busDevice_t as *mut busDevice_t };
static mut max7456SpiClock: uint16_t =
    (SPI_CLOCK_STANDARD as libc::c_int * 2 as libc::c_int) as uint16_t;
#[no_mangle]
pub static mut maxScreenSize: uint16_t = 480 as libc::c_int as uint16_t;
// We write everything in screenBuffer and then compare
// screenBuffer with shadowBuffer to upgrade only changed chars.
// This solution is faster then redrawing entire screen.
static mut screenBuffer: [uint8_t; 520] = [0; 520];
// For faster writes we use memcpy so we need some space to don't overwrite buffer
static mut shadowBuffer: [uint8_t; 480] = [0; 480];
static mut spiBuff: [uint8_t; 600] = [0; 600];
static mut videoSignalCfg: uint8_t = 0;
static mut videoSignalReg: uint8_t = 0x8 as libc::c_int as uint8_t;
// OSD_ENABLE required to trigger first ReInit
static mut displayMemoryModeReg: uint8_t = 0 as libc::c_int as uint8_t;
static mut hosRegValue: uint8_t = 0;
// HOS (Horizontal offset register) value
static mut vosRegValue: uint8_t = 0;
// VOS (Vertical offset register) value
static mut max7456Lock: bool = 0 as libc::c_int != 0;
static mut fontIsLoading: bool = 0 as libc::c_int != 0;
static mut max7456DeviceType: uint8_t = 0;
unsafe extern "C" fn max7456Send(mut add: uint8_t, mut data: uint8_t)
 -> uint8_t {
    spiTransferByte((*busdev).busdev_u.spi.instance, add);
    return spiTransferByte((*busdev).busdev_u.spi.instance, data);
}
#[no_mangle]
pub unsafe extern "C" fn max7456GetRowsCount() -> uint8_t {
    return if videoSignalReg as libc::c_int & 0x40 as libc::c_int != 0 {
               16 as libc::c_int
           } else { 13 as libc::c_int } as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn max7456ReInit() {
    let mut srdata: uint8_t = 0 as libc::c_int as uint8_t;
    static mut firstInit: bool = 1 as libc::c_int != 0;
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    IOLo((*busdev).busdev_u.spi.csnPin);
    match videoSignalCfg as libc::c_int {
        1 => {
            videoSignalReg =
                (0x40 as libc::c_int | 0x8 as libc::c_int) as uint8_t
        }
        2 => {
            videoSignalReg =
                (0 as libc::c_int | 0x8 as libc::c_int) as uint8_t
        }
        0 => {
            srdata =
                max7456Send(0xa0 as libc::c_int as uint8_t,
                            0 as libc::c_int as uint8_t);
            if srdata as libc::c_int & 0x4 as libc::c_int == 0 &&
                   srdata as libc::c_int & 0x2 as libc::c_int != 0 {
                videoSignalReg =
                    (0 as libc::c_int | 0x8 as libc::c_int) as uint8_t
            } else if srdata as libc::c_int & 0x4 as libc::c_int == 0 &&
                          srdata as libc::c_int & 0x1 as libc::c_int != 0 {
                videoSignalReg =
                    (0x40 as libc::c_int | 0x8 as libc::c_int) as uint8_t
            } else {
                // No valid input signal, fallback to default (XXX NTSC for now)
                videoSignalReg =
                    (0 as libc::c_int | 0x8 as libc::c_int) as uint8_t
            }
        }
        _ => { }
    } // NTSC
    if videoSignalReg as libc::c_int & 0x40 as libc::c_int != 0 {
        //PAL
        maxScreenSize = 480 as libc::c_int as uint16_t
    } else { maxScreenSize = 390 as libc::c_int as uint16_t }
    // Set all rows to same charactor black/white level
    max7456Brightness(0 as libc::c_int as uint8_t,
                      2 as libc::c_int as uint8_t);
    // Re-enable MAX7456 (last function call disables it)
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    IOLo((*busdev).busdev_u.spi.csnPin);
    // Make sure the Max7456 is enabled
    max7456Send(0 as libc::c_int as uint8_t, videoSignalReg);
    max7456Send(0x2 as libc::c_int as uint8_t, hosRegValue);
    max7456Send(0x3 as libc::c_int as uint8_t, vosRegValue);
    max7456Send(0x4 as libc::c_int as uint8_t,
                (displayMemoryModeReg as libc::c_int | 0x4 as libc::c_int) as
                    uint8_t);
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
    // Clear shadow to force redraw all screen in non-dma mode.
    memset(shadowBuffer.as_mut_ptr() as *mut libc::c_void, 0 as libc::c_int,
           maxScreenSize as libc::c_ulong);
    if firstInit {
        max7456DrawScreenSlow();
        firstInit = 0 as libc::c_int != 0
    };
}
// Here we init only CS and try to init MAX for first time.
// Also detect device type (MAX v.s. AT)
#[no_mangle]
pub unsafe extern "C" fn max7456Init(mut max7456Config:
                                         *const max7456Config_t,
                                     mut pVcdProfile: *const vcdProfile_t,
                                     mut cpuOverclock: bool) -> bool {
    max7456HardwareReset();
    if (*max7456Config).csTag == 0 { return 0 as libc::c_int != 0 }
    (*busdev).busdev_u.spi.csnPin = IOGetByTag((*max7456Config).csTag);
    if !IOIsFreeOrPreinit((*busdev).busdev_u.spi.csnPin) {
        return 0 as libc::c_int != 0
    }
    IOInit((*busdev).busdev_u.spi.csnPin, OWNER_OSD_CS,
           0 as libc::c_int as uint8_t);
    IOConfigGPIO((*busdev).busdev_u.spi.csnPin,
                 (GPIO_Mode_OUT as libc::c_int |
                      (GPIO_Speed_Level_3 as libc::c_int) << 2 as libc::c_int
                      | (GPIO_OType_PP as libc::c_int) << 4 as libc::c_int |
                      (GPIO_PuPd_NOPULL as libc::c_int) << 5 as libc::c_int)
                     as ioConfig_t);
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiBusSetInstance(busdev,
                      spiInstanceByDevice(((*max7456Config).spiDevice as
                                               libc::c_int - 1 as libc::c_int)
                                              as SPIDevice));
    // Detect device type by writing and reading CA[8] bit at CMAL[6].
    // Do this at half the speed for safety.
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  (SPI_CLOCK_STANDARD as libc::c_int * 2 as libc::c_int *
                       2 as libc::c_int) as uint16_t); // CA[8] bit
    max7456Send(0xa as libc::c_int as uint8_t,
                ((1 as libc::c_int) << 6 as libc::c_int) as uint8_t);
    if max7456Send((0xa as libc::c_int | 0x80 as libc::c_int) as uint8_t,
                   0xff as libc::c_int as uint8_t) as libc::c_int &
           (1 as libc::c_int) << 6 as libc::c_int != 0 {
        max7456DeviceType = 1 as libc::c_int as uint8_t
    } else { max7456DeviceType = 0 as libc::c_int as uint8_t }
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    // force soft reset on Max7456
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    IOLo((*busdev).busdev_u.spi.csnPin);
    max7456Send(0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t);
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
    // Setup values to write to registers
    videoSignalCfg = (*pVcdProfile).video_system;
    hosRegValue =
        (32 as libc::c_int - (*pVcdProfile).h_offset as libc::c_int) as
            uint8_t;
    vosRegValue =
        (16 as libc::c_int - (*pVcdProfile).v_offset as libc::c_int) as
            uint8_t;
    // Real init will be made later when driver detect idle.
    return 1 as libc::c_int != 0;
}
/* *
 * Sets inversion of black and white pixels.
 */
#[no_mangle]
pub unsafe extern "C" fn max7456Invert(mut invert: bool) {
    if invert {
        displayMemoryModeReg =
            (displayMemoryModeReg as libc::c_int | 0x8 as libc::c_int) as
                uint8_t
    } else {
        displayMemoryModeReg =
            (displayMemoryModeReg as libc::c_int & !(0x8 as libc::c_int)) as
                uint8_t
    }
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    IOLo((*busdev).busdev_u.spi.csnPin);
    max7456Send(0x4 as libc::c_int as uint8_t, displayMemoryModeReg);
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
}
/* *
 * Sets the brighness of black and white pixels.
 *
 * @param black Black brightness (0-3, 0 is darkest)
 * @param white White brightness (0-3, 0 is darkest)
 */
#[no_mangle]
pub unsafe extern "C" fn max7456Brightness(mut black: uint8_t,
                                           mut white: uint8_t) {
    let reg: uint8_t =
        ((black as libc::c_int) << 2 as libc::c_int |
             3 as libc::c_int - white as libc::c_int) as uint8_t;
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    IOLo((*busdev).busdev_u.spi.csnPin);
    let mut i: libc::c_int = 0x10 as libc::c_int;
    while i <= 0x1f as libc::c_int { max7456Send(i as uint8_t, reg); i += 1 }
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
}
//just fill with spaces with some tricks
#[no_mangle]
pub unsafe extern "C" fn max7456ClearScreen() {
    memset(screenBuffer.as_mut_ptr() as *mut libc::c_void,
           0x20 as libc::c_int, 480 as libc::c_int as libc::c_ulong);
}
#[no_mangle]
pub unsafe extern "C" fn max7456GetScreenBuffer() -> *mut uint8_t {
    return screenBuffer.as_mut_ptr();
}
#[no_mangle]
pub unsafe extern "C" fn max7456WriteChar(mut x: uint8_t, mut y: uint8_t,
                                          mut c: uint8_t) {
    screenBuffer[(y as libc::c_int * 30 as libc::c_int + x as libc::c_int) as
                     usize] = c;
}
#[no_mangle]
pub unsafe extern "C" fn max7456Write(mut x: uint8_t, mut y: uint8_t,
                                      mut buff: *const libc::c_char) {
    let mut i: libc::c_int = 0 as libc::c_int;
    while *buff.offset(i as isize) != 0 {
        if x as libc::c_int + i < 30 as libc::c_int {
            // Do not write over screen
            screenBuffer[(y as libc::c_int * 30 as libc::c_int +
                              x as libc::c_int + i) as usize] =
                *buff.offset(i as isize) as uint8_t
        }
        i += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn max7456DmaInProgress() -> bool {
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn max7456BuffersSynced() -> bool {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < maxScreenSize as libc::c_int {
        if screenBuffer[i as usize] as libc::c_int !=
               shadowBuffer[i as usize] as libc::c_int {
            return 0 as libc::c_int != 0
        }
        i += 1
    }
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn max7456ReInitIfRequired() {
    static mut lastSigCheckMs: uint32_t = 0 as libc::c_int as uint32_t;
    static mut videoDetectTimeMs: uint32_t = 0 as libc::c_int as uint32_t;
    static mut reInitCount: uint16_t = 0 as libc::c_int as uint16_t;
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    IOLo((*busdev).busdev_u.spi.csnPin);
    let stallCheck: uint8_t =
        max7456Send((0 as libc::c_int | 0x80 as libc::c_int) as uint8_t,
                    0 as libc::c_int as uint8_t);
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
    let nowMs: timeMs_t = millis();
    if stallCheck as libc::c_int != videoSignalReg as libc::c_int {
        max7456ReInit();
    } else if videoSignalCfg as libc::c_int ==
                  VIDEO_SYSTEM_AUTO as libc::c_int &&
                  nowMs.wrapping_sub(lastSigCheckMs) >
                      1000 as libc::c_int as libc::c_uint {
        // Adjust output format based on the current input format.
        spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
        IOLo((*busdev).busdev_u.spi.csnPin);
        let videoSense: uint8_t =
            max7456Send(0xa0 as libc::c_int as uint8_t,
                        0 as libc::c_int as uint8_t);
        IOHi((*busdev).busdev_u.spi.csnPin);
        spiSetDivisor((*busdev).busdev_u.spi.instance,
                      SPI_CLOCK_FAST as libc::c_int as uint16_t);
        if debugMode as libc::c_int == DEBUG_MAX7456_SIGNAL as libc::c_int {
            debug[0 as libc::c_int as usize] =
                (videoSignalReg as libc::c_int & 0x40 as libc::c_int) as
                    int16_t
        }
        if debugMode as libc::c_int == DEBUG_MAX7456_SIGNAL as libc::c_int {
            debug[1 as libc::c_int as usize] =
                (videoSense as libc::c_int & 0x7 as libc::c_int) as int16_t
        }
        if debugMode as libc::c_int == DEBUG_MAX7456_SIGNAL as libc::c_int {
            debug[3 as libc::c_int as usize] =
                max7456GetRowsCount() as int16_t
        }
        if videoSense as libc::c_int & 0x4 as libc::c_int != 0 {
            videoDetectTimeMs = 0 as libc::c_int as uint32_t
        } else if videoSense as libc::c_int & 0x4 as libc::c_int == 0 &&
                      videoSense as libc::c_int & 0x1 as libc::c_int != 0 &&
                      videoSignalReg as libc::c_int & 0x40 as libc::c_int ==
                          0 as libc::c_int ||
                      videoSense as libc::c_int & 0x4 as libc::c_int == 0 &&
                          videoSense as libc::c_int & 0x1 as libc::c_int == 0
                          &&
                          videoSignalReg as libc::c_int & 0x40 as libc::c_int
                              == 0x40 as libc::c_int {
            if videoDetectTimeMs != 0 {
                if millis().wrapping_sub(videoDetectTimeMs) >
                       100 as libc::c_int as libc::c_uint {
                    max7456ReInit();
                    if debugMode as libc::c_int ==
                           DEBUG_MAX7456_SIGNAL as libc::c_int {
                        reInitCount = reInitCount.wrapping_add(1);
                        debug[2 as libc::c_int as usize] =
                            reInitCount as int16_t
                    }
                }
            } else {
                // Wait for signal to stabilize
                videoDetectTimeMs = millis()
            }
        }
        lastSigCheckMs = nowMs
    };
    //------------   end of (re)init-------------------------------------
}
#[no_mangle]
pub unsafe extern "C" fn max7456DrawScreen() {
    static mut pos: uint16_t = 0 as libc::c_int as uint16_t;
    if !max7456Lock && !fontIsLoading {
        // (Re)Initialize MAX7456 at startup or stall is detected.
        max7456Lock = 1 as libc::c_int != 0;
        max7456ReInitIfRequired();
        let mut buff_len: libc::c_int = 0 as libc::c_int;
        let mut k: libc::c_int = 0 as libc::c_int;
        while k < 100 as libc::c_int {
            if screenBuffer[pos as usize] as libc::c_int !=
                   shadowBuffer[pos as usize] as libc::c_int {
                let fresh0 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh0 as usize] = 0x5 as libc::c_int as uint8_t;
                let fresh1 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh1 as usize] =
                    (pos as libc::c_int >> 8 as libc::c_int) as uint8_t;
                let fresh2 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh2 as usize] = 0x6 as libc::c_int as uint8_t;
                let fresh3 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh3 as usize] =
                    (pos as libc::c_int & 0xff as libc::c_int) as uint8_t;
                let fresh4 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh4 as usize] = 0x7 as libc::c_int as uint8_t;
                let fresh5 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh5 as usize] = screenBuffer[pos as usize];
                shadowBuffer[pos as usize] = screenBuffer[pos as usize]
            }
            pos = pos.wrapping_add(1);
            if pos as libc::c_int >= maxScreenSize as libc::c_int {
                pos = 0 as libc::c_int as uint16_t;
                break ;
            } else { k += 1 }
        }
        if buff_len != 0 {
            spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
            IOLo((*busdev).busdev_u.spi.csnPin);
            spiTransfer((*busdev).busdev_u.spi.instance, spiBuff.as_mut_ptr(),
                        0 as *mut uint8_t, buff_len);
            IOHi((*busdev).busdev_u.spi.csnPin);
            spiSetDivisor((*busdev).busdev_u.spi.instance,
                          SPI_CLOCK_FAST as libc::c_int as uint16_t);
            // MAX7456_DMA_CHANNEL_TX
        }
        max7456Lock = 0 as libc::c_int != 0
    };
}
unsafe extern "C" fn max7456DrawScreenSlow() {
    let mut escapeCharFound: bool = false;
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    IOLo((*busdev).busdev_u.spi.csnPin);
    // Enable auto-increment mode and update every character in the screenBuffer.
    // The "escape" character 0xFF must be skipped as it causes the MAX7456 to exit auto-increment mode.
    max7456Send(0x5 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t);
    max7456Send(0x6 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t);
    max7456Send(0x4 as libc::c_int as uint8_t,
                (displayMemoryModeReg as libc::c_int | 1 as libc::c_int) as
                    uint8_t);
    let mut xx: libc::c_int = 0 as libc::c_int;
    while xx < maxScreenSize as libc::c_int {
        if screenBuffer[xx as usize] as libc::c_int == 0xff as libc::c_int {
            escapeCharFound = 1 as libc::c_int != 0;
            max7456Send(0x7 as libc::c_int as uint8_t, ' ' as i32 as uint8_t);
            // replace the 0xFF character with a blank in the first pass to avoid terminating auto-increment
        } else {
            max7456Send(0x7 as libc::c_int as uint8_t,
                        screenBuffer[xx as usize]);
        }
        shadowBuffer[xx as usize] = screenBuffer[xx as usize];
        xx += 1
    }
    max7456Send(0x7 as libc::c_int as uint8_t,
                0xff as libc::c_int as uint8_t);
    max7456Send(0x4 as libc::c_int as uint8_t, displayMemoryModeReg);
    // If we found any of the "escape" character 0xFF, then make a second pass
    // to update them with direct addressing
    if escapeCharFound {
        let mut xx_0: libc::c_int = 0 as libc::c_int;
        while xx_0 < maxScreenSize as libc::c_int {
            if screenBuffer[xx_0 as usize] as libc::c_int ==
                   0xff as libc::c_int {
                max7456Send(0x5 as libc::c_int as uint8_t,
                            (xx_0 >> 8 as libc::c_int) as uint8_t);
                max7456Send(0x6 as libc::c_int as uint8_t,
                            (xx_0 & 0xff as libc::c_int) as uint8_t);
                max7456Send(0x7 as libc::c_int as uint8_t,
                            0xff as libc::c_int as uint8_t);
            }
            xx_0 += 1
        }
    }
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
}
// should not be used when armed
#[no_mangle]
pub unsafe extern "C" fn max7456RefreshAll() {
    if !max7456Lock {
        max7456Lock = 1 as libc::c_int != 0;
        max7456ReInitIfRequired();
        max7456DrawScreenSlow();
        max7456Lock = 0 as libc::c_int != 0
    };
}
#[no_mangle]
pub unsafe extern "C" fn max7456WriteNvm(mut char_address: uint8_t,
                                         mut font_data: *const uint8_t) {
    while max7456Lock { }
    max7456Lock = 1 as libc::c_int != 0;
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    IOLo((*busdev).busdev_u.spi.csnPin);
    // disable display
    fontIsLoading = 1 as libc::c_int != 0; // set start address high
    max7456Send(0 as libc::c_int as uint8_t,
                0 as libc::c_int as uint8_t); //set start address low
    max7456Send(0x9 as libc::c_int as uint8_t, char_address);
    let mut x: libc::c_int = 0 as libc::c_int;
    while x < 54 as libc::c_int {
        max7456Send(0xa as libc::c_int as uint8_t, x as uint8_t);
        max7456Send(0xb as libc::c_int as uint8_t,
                    *font_data.offset(x as isize));
        ledToggle(0 as libc::c_int);
        x += 1
    }
    // Transfer 54 bytes from shadow ram to NVM
    max7456Send(0x8 as libc::c_int as uint8_t,
                0xa0 as libc::c_int as uint8_t);
    // Wait until bit 5 in the status register returns to 0 (12ms)
    while max7456Send(0xa0 as libc::c_int as uint8_t,
                      0 as libc::c_int as uint8_t) as libc::c_int &
              0x20 as libc::c_int != 0 as libc::c_int {
    }
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
    max7456Lock = 0 as libc::c_int != 0;
}
static mut max7456ResetPin: IO_t = 0 as *const libc::c_void as IO_t;
#[no_mangle]
pub unsafe extern "C" fn max7456HardwareReset() {
    max7456ResetPin =
        IOGetByTag(((2 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 14 as libc::c_int) as ioTag_t);
    IOInit(max7456ResetPin, OWNER_OSD, 0 as libc::c_int as uint8_t);
    IOConfigGPIO(max7456ResetPin,
                 (GPIO_Mode_OUT as libc::c_int |
                      (GPIO_Speed_Level_2 as libc::c_int) << 2 as libc::c_int
                      | (GPIO_OType_PP as libc::c_int) << 4 as libc::c_int |
                      (GPIO_PuPd_DOWN as libc::c_int) << 5 as libc::c_int) as
                     ioConfig_t);
    // RESET
    IOLo(max7456ResetPin);
    delay(100 as libc::c_int as timeMs_t);
    IOHi(max7456ResetPin);
}
// USE_MAX7456
