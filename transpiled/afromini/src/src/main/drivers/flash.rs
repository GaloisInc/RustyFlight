use core;
use libc;
extern "C" {
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
    fn m25p16_detect(fdevice: *mut flashDevice_t, chipID: uint32_t) -> bool;
    // Macros to convert between CLI bus number and SPIDevice.
    // Size of SPI CS pre-initialization tag arrays
    #[no_mangle]
    fn spiPreinitCsByTag(iotag: ioTag_t);
    #[no_mangle]
    fn spiSetDivisor(instance: *mut SPI_TypeDef, divisor: uint16_t);
    #[no_mangle]
    fn spiInstanceByDevice(device: SPIDevice) -> *mut SPI_TypeDef;
    #[no_mangle]
    fn spiBusTransfer(bus: *const busDevice_t, txData: *const uint8_t,
                      rxData: *mut uint8_t, length: libc::c_int) -> bool;
    #[no_mangle]
    fn spiBusSetInstance(bus: *mut busDevice_t, instance: *mut SPI_TypeDef);
    #[no_mangle]
    fn delay(ms: timeMs_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
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
pub type ioTag_t = uint8_t;
pub type IO_t = *mut libc::c_void;
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct flashConfig_s {
    pub csTag: ioTag_t,
    pub spiDevice: uint8_t,
}
pub type flashConfig_t = flashConfig_s;
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
pub type flashType_e = libc::c_uint;
pub const FLASH_TYPE_NAND: flashType_e = 1;
pub const FLASH_TYPE_NOR: flashType_e = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct flashGeometry_s {
    pub sectors: uint16_t,
    pub pageSize: uint16_t,
    pub sectorSize: uint32_t,
    pub totalSize: uint32_t,
    pub pagesPerSector: uint16_t,
    pub flashType: flashType_e,
}
pub type flashGeometry_t = flashGeometry_s;
pub type flashDevice_t = flashDevice_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct flashDevice_s {
    pub busdev: *mut busDevice_t,
    pub vTable: *const flashVTable_s,
    pub geometry: flashGeometry_t,
    pub currentWriteAddress: uint32_t,
    pub isLargeFlash: bool,
    pub couldBeBusy: bool,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct flashVTable_s {
    pub isReady: Option<unsafe extern "C" fn(_: *mut flashDevice_t) -> bool>,
    pub waitForReady: Option<unsafe extern "C" fn(_: *mut flashDevice_t,
                                                  _: uint32_t) -> bool>,
    pub eraseSector: Option<unsafe extern "C" fn(_: *mut flashDevice_t,
                                                 _: uint32_t) -> ()>,
    pub eraseCompletely: Option<unsafe extern "C" fn(_: *mut flashDevice_t)
                                    -> ()>,
    pub pageProgramBegin: Option<unsafe extern "C" fn(_: *mut flashDevice_t,
                                                      _: uint32_t) -> ()>,
    pub pageProgramContinue: Option<unsafe extern "C" fn(_:
                                                             *mut flashDevice_t,
                                                         _: *const uint8_t,
                                                         _: libc::c_int)
                                        -> ()>,
    pub pageProgramFinish: Option<unsafe extern "C" fn(_: *mut flashDevice_t)
                                      -> ()>,
    pub pageProgram: Option<unsafe extern "C" fn(_: *mut flashDevice_t,
                                                 _: uint32_t,
                                                 _: *const uint8_t,
                                                 _: libc::c_int) -> ()>,
    pub flush: Option<unsafe extern "C" fn(_: *mut flashDevice_t) -> ()>,
    pub readBytes: Option<unsafe extern "C" fn(_: *mut flashDevice_t,
                                               _: uint32_t, _: *mut uint8_t,
                                               _: libc::c_int)
                              -> libc::c_int>,
    pub getGeometry: Option<unsafe extern "C" fn(_: *mut flashDevice_t)
                                -> *const flashGeometry_t>,
}
pub type busDevice_t = busDevice_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed_3,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed_3 {
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
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct deviceSpi_s {
    pub instance: *mut SPI_TypeDef,
    pub csnPin: IO_t,
}
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
// millisecond time
pub type timeMs_t = uint32_t;
pub const SPI_CLOCK_STANDARD: C2RustUnnamed_4 = 4;
pub type SPIDevice = libc::c_int;
pub const SPIDEV_4: SPIDevice = 3;
pub const SPIDEV_3: SPIDevice = 2;
pub const SPIDEV_2: SPIDevice = 1;
pub const SPIDEV_1: SPIDevice = 0;
pub const SPIINVALID: SPIDevice = -1;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const SPI_CLOCK_ULTRAFAST: C2RustUnnamed_4 = 2;
pub const SPI_CLOCK_FAST: C2RustUnnamed_4 = 2;
pub const SPI_CLOCK_SLOW: C2RustUnnamed_4 = 128;
pub const SPI_CLOCK_INITIALIZATON: C2RustUnnamed_4 = 256;
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
static mut busInstance: busDevice_t =
    busDevice_t{bustype: BUSTYPE_NONE,
                busdev_u:
                    C2RustUnnamed_3{spi:
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
static mut busdev: *mut busDevice_t =
    0 as *const busDevice_t as *mut busDevice_t;
static mut flashDevice: flashDevice_t =
    flashDevice_t{busdev: 0 as *const busDevice_t as *mut busDevice_t,
                  vTable: 0 as *const flashVTable_s,
                  geometry:
                      flashGeometry_t{sectors: 0,
                                      pageSize: 0,
                                      sectorSize: 0,
                                      totalSize: 0,
                                      pagesPerSector: 0,
                                      flashType: FLASH_TYPE_NOR,},
                  currentWriteAddress: 0,
                  isLargeFlash: false,
                  couldBeBusy: false,};
// Read chip identification and send it to device detect
#[no_mangle]
pub unsafe extern "C" fn flashInit(mut flashConfig: *const flashConfig_t)
 -> bool {
    busdev = &mut busInstance;
    if (*flashConfig).csTag != 0 {
        (*busdev).busdev_u.spi.csnPin = IOGetByTag((*flashConfig).csTag)
    } else { return 0i32 != 0 }
    if !IOIsFreeOrPreinit((*busdev).busdev_u.spi.csnPin) { return 0i32 != 0 }
    (*busdev).bustype = BUSTYPE_SPI;
    spiBusSetInstance(busdev,
                      spiInstanceByDevice(((*flashConfig).spiDevice as
                                               libc::c_int - 1i32) as
                                              SPIDevice));
    IOInit((*busdev).busdev_u.spi.csnPin, OWNER_FLASH_CS, 0i32 as uint8_t);
    IOConfigGPIO((*busdev).busdev_u.spi.csnPin,
                 (GPIO_Mode_OUT as libc::c_int |
                      (GPIO_Speed_Level_3 as libc::c_int) << 2i32 |
                      (GPIO_OType_PP as libc::c_int) << 4i32 |
                      (GPIO_PuPd_NOPULL as libc::c_int) << 5i32) as
                     ioConfig_t);
    IOHi((*busdev).busdev_u.spi.csnPin);
    //Maximum speed for standard READ command is 20mHz, other commands tolerate 25mHz
    //spiSetDivisor(busdev->busdev_u.spi.instance, SPI_CLOCK_FAST);
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  (SPI_CLOCK_STANDARD as libc::c_int * 2i32) as
                      uint16_t); // short delay required after initialisation of SPI device instance.
    flashDevice.busdev = busdev;
    let out: [uint8_t; 4] =
        [0x9fi32 as uint8_t, 0i32 as uint8_t, 0i32 as uint8_t,
         0i32 as uint8_t];
    delay(50i32 as timeMs_t);
    /* Just in case transfer fails and writes nothing, so we don't try to verify the ID against random garbage
     * from the stack:
     */
    let mut in_0: [uint8_t; 4] = [0; 4];
    in_0[1] = 0i32 as uint8_t;
    // Clearing the CS bit terminates the command early so we don't have to read the chip UID:
    spiBusTransfer(busdev, out.as_ptr(), in_0.as_mut_ptr(),
                   ::core::mem::size_of::<[uint8_t; 4]>() as libc::c_ulong as
                       libc::c_int);
    // Manufacturer, memory type, and capacity
    let mut chipID: uint32_t =
        ((in_0[1] as libc::c_int) << 16i32 | (in_0[2] as libc::c_int) << 8i32
             | in_0[3] as libc::c_int) as uint32_t;
    if m25p16_detect(&mut flashDevice, chipID) { return 1i32 != 0 }
    spiPreinitCsByTag((*flashConfig).csTag);
    return 0i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn flashIsReady() -> bool {
    return (*flashDevice.vTable).isReady.expect("non-null function pointer")(&mut flashDevice);
}
#[no_mangle]
pub unsafe extern "C" fn flashWaitForReady(mut timeoutMillis: uint32_t)
 -> bool {
    return (*flashDevice.vTable).waitForReady.expect("non-null function pointer")(&mut flashDevice,
                                                                                  timeoutMillis);
}
#[no_mangle]
pub unsafe extern "C" fn flashEraseSector(mut address: uint32_t) {
    (*flashDevice.vTable).eraseSector.expect("non-null function pointer")(&mut flashDevice,
                                                                          address);
}
#[no_mangle]
pub unsafe extern "C" fn flashEraseCompletely() {
    (*flashDevice.vTable).eraseCompletely.expect("non-null function pointer")(&mut flashDevice);
}
#[no_mangle]
pub unsafe extern "C" fn flashPageProgramBegin(mut address: uint32_t) {
    (*flashDevice.vTable).pageProgramBegin.expect("non-null function pointer")(&mut flashDevice,
                                                                               address);
}
#[no_mangle]
pub unsafe extern "C" fn flashPageProgramContinue(mut data: *const uint8_t,
                                                  mut length: libc::c_int) {
    (*flashDevice.vTable).pageProgramContinue.expect("non-null function pointer")(&mut flashDevice,
                                                                                  data,
                                                                                  length);
}
#[no_mangle]
pub unsafe extern "C" fn flashPageProgramFinish() {
    (*flashDevice.vTable).pageProgramFinish.expect("non-null function pointer")(&mut flashDevice);
}
#[no_mangle]
pub unsafe extern "C" fn flashPageProgram(mut address: uint32_t,
                                          mut data: *const uint8_t,
                                          mut length: libc::c_int) {
    (*flashDevice.vTable).pageProgram.expect("non-null function pointer")(&mut flashDevice,
                                                                          address,
                                                                          data,
                                                                          length);
}
#[no_mangle]
pub unsafe extern "C" fn flashReadBytes(mut address: uint32_t,
                                        mut buffer: *mut uint8_t,
                                        mut length: libc::c_int)
 -> libc::c_int {
    return (*flashDevice.vTable).readBytes.expect("non-null function pointer")(&mut flashDevice,
                                                                               address,
                                                                               buffer,
                                                                               length);
}
#[no_mangle]
pub unsafe extern "C" fn flashFlush() {
    (*flashDevice.vTable).flush.expect("non-null function pointer")(&mut flashDevice);
}
static mut noFlashGeometry: flashGeometry_t =
    {
        let mut init =
            flashGeometry_s{sectors: 0,
                            pageSize: 0,
                            sectorSize: 0,
                            totalSize: 0i32 as uint32_t,
                            pagesPerSector: 0,
                            flashType: FLASH_TYPE_NOR,};
        init
    };
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
// Maximum page size of all supported SPI flash devices.
// Used to detect flashfs allocation size being too small.
// Count of the number of erasable blocks on the device
// In bytes
// This is just pagesPerSector * pageSize
// This is just sectorSize * sectors
#[no_mangle]
pub unsafe extern "C" fn flashGetGeometry() -> *const flashGeometry_t {
    if !flashDevice.vTable.is_null() &&
           (*flashDevice.vTable).getGeometry.is_some() {
        return (*flashDevice.vTable).getGeometry.expect("non-null function pointer")(&mut flashDevice)
    }
    return &noFlashGeometry;
}
// USE_FLASH
