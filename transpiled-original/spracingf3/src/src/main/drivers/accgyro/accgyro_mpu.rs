use core;
use libc;
extern "C" {
    #[no_mangle]
    fn busReadRegisterBuffer(bus: *const busDevice_t, reg: uint8_t,
                             data: *mut uint8_t, length: uint8_t) -> bool;
    #[no_mangle]
    fn spiBusTransfer(bus: *const busDevice_t, txData: *const uint8_t,
                      rxData: *mut uint8_t, length: libc::c_int) -> bool;
    #[no_mangle]
    fn EXTIHandlerInit(cb: *mut extiCallbackRec_t,
                       fn_0:
                           Option<unsafe extern "C" fn(_:
                                                           *mut extiCallbackRec_t)
                                      -> ()>);
    #[no_mangle]
    fn EXTIConfig(io: IO_t, cb: *mut extiCallbackRec_t,
                  irqPriority: libc::c_int, trigger: EXTITrigger_TypeDef);
    #[no_mangle]
    fn EXTIEnable(io: IO_t, enable: bool);
    // declare available IO pins. Available pins are specified per target
    #[no_mangle]
    fn IORead(io: IO_t) -> bool;
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn failureMode(mode: failureMode_e);
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
  * @brief  EXTI Trigger enumeration  
  */
pub type EXTITrigger_TypeDef = libc::c_uint;
pub const EXTI_Trigger_Rising_Falling: EXTITrigger_TypeDef = 16;
pub const EXTI_Trigger_Falling: EXTITrigger_TypeDef = 12;
pub const EXTI_Trigger_Rising: EXTITrigger_TypeDef = 8;
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
    pub busdev_u: C2RustUnnamed_1,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed_1 {
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct extiCallbackRec_s {
    pub fn_0: Option<unsafe extern "C" fn(_: *mut extiCallbackRec_t) -> ()>,
}
pub type extiHandlerCallback
    =
    unsafe extern "C" fn(_: *mut extiCallbackRec_t) -> ();
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
pub type extiCallbackRec_t = extiCallbackRec_s;
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
pub type sensor_align_e = libc::c_uint;
pub const CW270_DEG_FLIP: sensor_align_e = 8;
pub const CW180_DEG_FLIP: sensor_align_e = 7;
pub const CW90_DEG_FLIP: sensor_align_e = 6;
pub const CW0_DEG_FLIP: sensor_align_e = 5;
pub const CW270_DEG: sensor_align_e = 4;
pub const CW180_DEG: sensor_align_e = 3;
pub const CW90_DEG: sensor_align_e = 2;
pub const CW0_DEG: sensor_align_e = 1;
pub const ALIGN_DEFAULT: sensor_align_e = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct accDev_s {
    pub initFn: sensorAccInitFuncPtr,
    pub readFn: sensorAccReadFuncPtr,
    pub bus: busDevice_t,
    pub acc_1G: uint16_t,
    pub ADCRaw: [int16_t; 3],
    pub mpuDetectionResult: mpuDetectionResult_t,
    pub accAlign: sensor_align_e,
    pub dataReady: bool,
    pub acc_high_fsr: bool,
    pub revisionCode: libc::c_char,
    pub filler: [uint8_t; 2],
}
pub type mpuDetectionResult_t = mpuDetectionResult_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct mpuDetectionResult_s {
    pub sensor: mpuSensor_e,
    pub resolution: mpu6050Resolution_e,
}
pub type mpu6050Resolution_e = libc::c_uint;
pub const MPU_FULL_RESOLUTION: mpu6050Resolution_e = 1;
pub const MPU_HALF_RESOLUTION: mpu6050Resolution_e = 0;
pub type mpuSensor_e = libc::c_uint;
pub const BMI_160_SPI: mpuSensor_e = 12;
pub const ICM_20689_SPI: mpuSensor_e = 11;
pub const ICM_20649_SPI: mpuSensor_e = 10;
pub const ICM_20608_SPI: mpuSensor_e = 9;
pub const ICM_20602_SPI: mpuSensor_e = 8;
pub const ICM_20601_SPI: mpuSensor_e = 7;
pub const MPU_9250_SPI: mpuSensor_e = 6;
pub const MPU_65xx_SPI: mpuSensor_e = 5;
pub const MPU_65xx_I2C: mpuSensor_e = 4;
pub const MPU_60x0_SPI: mpuSensor_e = 3;
pub const MPU_60x0: mpuSensor_e = 2;
pub const MPU_3050: mpuSensor_e = 1;
pub const MPU_NONE: mpuSensor_e = 0;
pub type sensorAccReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> bool>;
pub type sensorAccInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> ()>;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct gyroDev_s {
    pub initFn: sensorGyroInitFuncPtr,
    pub readFn: sensorGyroReadFuncPtr,
    pub temperatureFn: sensorGyroReadDataFuncPtr,
    pub exti: extiCallbackRec_t,
    pub bus: busDevice_t,
    pub scale: libc::c_float,
    pub gyroZero: [libc::c_float; 3],
    pub gyroADC: [libc::c_float; 3],
    pub gyroADCf: [libc::c_float; 3],
    pub gyroADCRawPrevious: [int32_t; 3],
    pub gyroADCRaw: [int16_t; 3],
    pub temperature: int16_t,
    pub mpuConfiguration: mpuConfiguration_t,
    pub mpuDetectionResult: mpuDetectionResult_t,
    pub gyroAlign: sensor_align_e,
    pub gyroRateKHz: gyroRateKHz_e,
    pub dataReady: bool,
    pub gyro_high_fsr: bool,
    pub hardware_lpf: uint8_t,
    pub hardware_32khz_lpf: uint8_t,
    pub mpuDividerDrops: uint8_t,
    pub mpuIntExtiTag: ioTag_t,
    pub gyroHasOverflowProtection: uint8_t,
    pub gyroHardware: gyroSensor_e,
}
pub type gyroSensor_e = libc::c_uint;
pub const GYRO_FAKE: gyroSensor_e = 15;
pub const GYRO_BMI160: gyroSensor_e = 14;
pub const GYRO_ICM20689: gyroSensor_e = 13;
pub const GYRO_ICM20649: gyroSensor_e = 12;
pub const GYRO_ICM20608G: gyroSensor_e = 11;
pub const GYRO_ICM20602: gyroSensor_e = 10;
pub const GYRO_ICM20601: gyroSensor_e = 9;
pub const GYRO_MPU9250: gyroSensor_e = 8;
pub const GYRO_MPU6500: gyroSensor_e = 7;
pub const GYRO_MPU6000: gyroSensor_e = 6;
pub const GYRO_L3GD20: gyroSensor_e = 5;
pub const GYRO_MPU3050: gyroSensor_e = 4;
pub const GYRO_L3G4200D: gyroSensor_e = 3;
pub const GYRO_MPU6050: gyroSensor_e = 2;
pub const GYRO_DEFAULT: gyroSensor_e = 1;
pub const GYRO_NONE: gyroSensor_e = 0;
pub type gyroRateKHz_e = libc::c_uint;
pub const GYRO_RATE_32_kHz: gyroRateKHz_e = 5;
pub const GYRO_RATE_9_kHz: gyroRateKHz_e = 4;
pub const GYRO_RATE_8_kHz: gyroRateKHz_e = 3;
pub const GYRO_RATE_3200_Hz: gyroRateKHz_e = 2;
pub const GYRO_RATE_1100_Hz: gyroRateKHz_e = 1;
pub const GYRO_RATE_1_kHz: gyroRateKHz_e = 0;
pub type mpuConfiguration_t = mpuConfiguration_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct mpuConfiguration_s {
    pub resetFn: mpuResetFnPtr,
}
pub type mpuResetFnPtr = Option<unsafe extern "C" fn() -> ()>;
pub type sensorGyroReadDataFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut gyroDev_s, _: *mut int16_t) -> bool>;
pub type sensorGyroReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut gyroDev_s) -> bool>;
pub type sensorGyroInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut gyroDev_s) -> ()>;
pub type failureMode_e = libc::c_uint;
pub const FAILURE_GYRO_INIT_FAILED: failureMode_e = 6;
pub const FAILURE_FLASH_WRITE_FAILED: failureMode_e = 5;
pub const FAILURE_INVALID_EEPROM_CONTENTS: failureMode_e = 4;
pub const FAILURE_ACC_INCOMPATIBLE: failureMode_e = 3;
pub const FAILURE_ACC_INIT: failureMode_e = 2;
pub const FAILURE_MISSING_ACC: failureMode_e = 1;
pub const FAILURE_DEVELOPER: failureMode_e = 0;
// millisecond time
pub type timeMs_t = uint32_t;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const Z: C2RustUnnamed_2 = 2;
pub const Y: C2RustUnnamed_2 = 1;
pub const X: C2RustUnnamed_2 = 0;
pub type fchoice_b = libc::c_uint;
pub const FCB_3600_32: fchoice_b = 2;
pub const FCB_8800_32: fchoice_b = 1;
pub const FCB_DISABLED: fchoice_b = 0;
pub type gyroDev_t = gyroDev_s;
pub type accDev_t = accDev_s;
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
pub static mut mpuResetFn: mpuResetFnPtr = None;
unsafe extern "C" fn mpu6050FindRevision(mut gyro: *mut gyroDev_t) {
    // There is a map of revision contained in the android source tree which is quite comprehensive and may help to understand this code
    // See https://android.googlesource.com/kernel/msm.git/+/eaf36994a3992b8f918c18e4f7411e8b2320a35f/drivers/misc/mpu6050/mldl_cfg.c
    // determine product ID and revision
    let mut readBuffer: [uint8_t; 6] = [0; 6];
    let mut ack: bool =
        busReadRegisterBuffer(&mut (*gyro).bus, 0x6i32 as uint8_t,
                              readBuffer.as_mut_ptr(), 6i32 as uint8_t);
    let mut revision: uint8_t =
        ((readBuffer[5] as libc::c_int & 0x1i32) << 2i32 |
             (readBuffer[3] as libc::c_int & 0x1i32) << 1i32 |
             readBuffer[1] as libc::c_int & 0x1i32) as uint8_t;
    if ack as libc::c_int != 0 && revision as libc::c_int != 0 {
        // Congrats, these parts are better
        if revision as libc::c_int == 1i32 {
            (*gyro).mpuDetectionResult.resolution = MPU_HALF_RESOLUTION
        } else if revision as libc::c_int == 2i32 {
            (*gyro).mpuDetectionResult.resolution = MPU_FULL_RESOLUTION
        } else if revision as libc::c_int == 3i32 ||
                      revision as libc::c_int == 7i32 {
            (*gyro).mpuDetectionResult.resolution = MPU_FULL_RESOLUTION
        } else { failureMode(FAILURE_ACC_INCOMPATIBLE); }
    } else {
        let mut productId: uint8_t = 0;
        ack =
            busReadRegisterBuffer(&mut (*gyro).bus, 0xci32 as uint8_t,
                                  &mut productId, 1i32 as uint8_t);
        revision = (productId as libc::c_int & 0xfi32) as uint8_t;
        if !ack || revision as libc::c_int == 0i32 {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        } else if revision as libc::c_int == 4i32 {
            (*gyro).mpuDetectionResult.resolution = MPU_HALF_RESOLUTION
        } else { (*gyro).mpuDetectionResult.resolution = MPU_FULL_RESOLUTION }
    };
}
/*
 * Gyro interrupt service routine
 */
unsafe extern "C" fn mpuIntExtiHandler(mut cb: *mut extiCallbackRec_t) {
    let mut gyro: *mut gyroDev_t =
        ({
             let mut __mptr: *const extiCallbackRec_t =
                 cb; // TODO - maybe pullup / pulldown ?
             (__mptr as *mut libc::c_char).offset(-24) as *mut gyroDev_t
         });
    (*gyro).dataReady = 1i32 != 0;
}
unsafe extern "C" fn mpuIntExtiInit(mut gyro: *mut gyroDev_t) {
    if (*gyro).mpuIntExtiTag as libc::c_int == 0i32 { return }
    let mpuIntIO: IO_t = IOGetByTag((*gyro).mpuIntExtiTag);
    let mut status: uint8_t = IORead(mpuIntIO) as uint8_t;
    if status != 0 { return }
    IOInit(mpuIntIO, OWNER_MPU_EXTI, 0i32 as uint8_t);
    IOConfigGPIO(mpuIntIO,
                 (GPIO_Mode_IN as libc::c_int | 0i32 << 2i32 | 0i32 << 4i32 |
                      (GPIO_PuPd_NOPULL as libc::c_int) << 5i32) as
                     ioConfig_t);
    EXTIHandlerInit(&mut (*gyro).exti,
                    Some(mpuIntExtiHandler as
                             unsafe extern "C" fn(_: *mut extiCallbackRec_t)
                                 -> ()));
    EXTIConfig(mpuIntIO, &mut (*gyro).exti,
               (0xfi32 <<
                    (4i32 as
                         libc::c_uint).wrapping_sub((7i32 as
                                                         libc::c_uint).wrapping_sub(0x500i32
                                                                                        as
                                                                                        uint32_t
                                                                                        >>
                                                                                        8i32))
                    |
                    0xfi32 &
                        0xfi32 >>
                            (7i32 as
                                 libc::c_uint).wrapping_sub(0x500i32 as
                                                                uint32_t >>
                                                                8i32)) << 4i32
                   & 0xf0i32, EXTI_Trigger_Rising);
    EXTIEnable(mpuIntIO, 1i32 != 0);
}
// MPU_INT_EXTI
#[no_mangle]
pub unsafe extern "C" fn mpuAccRead(mut acc: *mut accDev_t) -> bool {
    let mut data: [uint8_t; 6] = [0; 6];
    let ack: bool =
        busReadRegisterBuffer(&mut (*acc).bus, 0x3bi32 as uint8_t,
                              data.as_mut_ptr(), 6i32 as uint8_t);
    if !ack { return 0i32 != 0 }
    (*acc).ADCRaw[X as libc::c_int as usize] =
        ((data[0] as libc::c_int) << 8i32 | data[1] as libc::c_int) as
            int16_t;
    (*acc).ADCRaw[Y as libc::c_int as usize] =
        ((data[2] as libc::c_int) << 8i32 | data[3] as libc::c_int) as
            int16_t;
    (*acc).ADCRaw[Z as libc::c_int as usize] =
        ((data[4] as libc::c_int) << 8i32 | data[5] as libc::c_int) as
            int16_t;
    return 1i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn mpuGyroRead(mut gyro: *mut gyroDev_t) -> bool {
    let mut data: [uint8_t; 6] = [0; 6];
    let ack: bool =
        busReadRegisterBuffer(&mut (*gyro).bus, 0x43i32 as uint8_t,
                              data.as_mut_ptr(), 6i32 as uint8_t);
    if !ack { return 0i32 != 0 }
    (*gyro).gyroADCRaw[X as libc::c_int as usize] =
        ((data[0] as libc::c_int) << 8i32 | data[1] as libc::c_int) as
            int16_t;
    (*gyro).gyroADCRaw[Y as libc::c_int as usize] =
        ((data[2] as libc::c_int) << 8i32 | data[3] as libc::c_int) as
            int16_t;
    (*gyro).gyroADCRaw[Z as libc::c_int as usize] =
        ((data[4] as libc::c_int) << 8i32 | data[5] as libc::c_int) as
            int16_t;
    return 1i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn mpuGyroReadSPI(mut gyro: *mut gyroDev_t) -> bool {
    static mut dataToSend: [uint8_t; 7] =
        [(0x43i32 | 0x80i32) as uint8_t, 0xffi32 as uint8_t,
         0xffi32 as uint8_t, 0xffi32 as uint8_t, 0xffi32 as uint8_t,
         0xffi32 as uint8_t, 0xffi32 as uint8_t];
    let mut data: [uint8_t; 7] = [0; 7];
    let ack: bool =
        spiBusTransfer(&mut (*gyro).bus, dataToSend.as_ptr(),
                       data.as_mut_ptr(), 7i32);
    if !ack { return 0i32 != 0 }
    (*gyro).gyroADCRaw[X as libc::c_int as usize] =
        ((data[1] as libc::c_int) << 8i32 | data[2] as libc::c_int) as
            int16_t;
    (*gyro).gyroADCRaw[Y as libc::c_int as usize] =
        ((data[3] as libc::c_int) << 8i32 | data[4] as libc::c_int) as
            int16_t;
    (*gyro).gyroADCRaw[Z as libc::c_int as usize] =
        ((data[5] as libc::c_int) << 8i32 | data[6] as libc::c_int) as
            int16_t;
    return 1i32 != 0;
}
unsafe extern "C" fn detectSPISensorsAndUpdateDetectionResult(mut gyro:
                                                                  *mut gyroDev_t)
 -> bool {
    // since there are FCs which have gyro on I2C but other devices on SPI
    let mut sensor: uint8_t = MPU_NONE as libc::c_int as uint8_t;
    // note, when USE_DUAL_GYRO is enabled the gyro->bus must already be initialised.
    return 0i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn mpuDetect(mut gyro: *mut gyroDev_t) {
    // MPU datasheet specifies 30ms.
    delay(35i32 as timeMs_t);
    if (*gyro).bus.bustype as libc::c_uint ==
           BUSTYPE_NONE as libc::c_int as libc::c_uint {
        // if no bustype is selected try I2C first.
        (*gyro).bus.bustype = BUSTYPE_I2C
    }
    if (*gyro).bus.bustype as libc::c_uint ==
           BUSTYPE_I2C as libc::c_int as libc::c_uint {
        (*gyro).bus.busdev_u.i2c.device = I2CDEV_1;
        (*gyro).bus.busdev_u.i2c.address = 0x68i32 as uint8_t;
        let mut sig: uint8_t = 0i32 as uint8_t;
        let mut ack: bool =
            busReadRegisterBuffer(&mut (*gyro).bus, 0x75i32 as uint8_t,
                                  &mut sig, 1i32 as uint8_t);
        if ack {
            // If an MPU3050 is connected sig will contain 0.
            let mut inquiryResult: uint8_t = 0;
            ack =
                busReadRegisterBuffer(&mut (*gyro).bus, 0i32 as uint8_t,
                                      &mut inquiryResult, 1i32 as uint8_t);
            inquiryResult =
                (inquiryResult as libc::c_int & 0x7ei32) as uint8_t;
            if ack as libc::c_int != 0 &&
                   inquiryResult as libc::c_int == 0x68i32 {
                (*gyro).mpuDetectionResult.sensor = MPU_3050;
                return
            }
            sig = (sig as libc::c_int & 0x7ei32) as uint8_t;
            if sig as libc::c_int == 0x68i32 {
                (*gyro).mpuDetectionResult.sensor = MPU_60x0;
                mpu6050FindRevision(gyro);
            } else if sig as libc::c_int == 0x70i32 {
                (*gyro).mpuDetectionResult.sensor = MPU_65xx_I2C
            }
            return
        }
    }
    (*gyro).bus.bustype = BUSTYPE_SPI;
    detectSPISensorsAndUpdateDetectionResult(gyro);
}
#[no_mangle]
pub unsafe extern "C" fn mpuGyroInit(mut gyro: *mut gyroDev_t) {
    mpuIntExtiInit(gyro);
}
#[no_mangle]
pub unsafe extern "C" fn mpuGyroDLPF(mut gyro: *mut gyroDev_t) -> uint8_t {
    let mut ret: uint8_t = 0;
    if (*gyro).gyroRateKHz as libc::c_uint >
           GYRO_RATE_8_kHz as libc::c_int as libc::c_uint {
        ret = 0i32 as uint8_t
        // If gyro is in 32KHz mode then the DLPF bits aren't used - set to 0
    } else {
        match (*gyro).hardware_lpf as libc::c_int {
            0 => { ret = 0i32 as uint8_t }
            1 => { ret = 7i32 as uint8_t }
            2 => { ret = 1i32 as uint8_t }
            _ => { ret = 0i32 as uint8_t }
        }
    }
    return ret;
}
#[no_mangle]
pub unsafe extern "C" fn mpuGyroFCHOICE(mut gyro: *mut gyroDev_t) -> uint8_t {
    if (*gyro).gyroRateKHz as libc::c_uint >
           GYRO_RATE_8_kHz as libc::c_int as libc::c_uint {
        if (*gyro).hardware_32khz_lpf as libc::c_int == 1i32 {
            return FCB_8800_32 as libc::c_int as uint8_t
        } else { return FCB_3600_32 as libc::c_int as uint8_t }
    } else {
        return FCB_DISABLED as libc::c_int as uint8_t
        // Not in 32KHz mode, set FCHOICE to select 8KHz sampling
    };
}
#[no_mangle]
pub unsafe extern "C" fn mpuGyroReadRegister(mut bus: *const busDevice_t,
                                             mut reg: uint8_t) -> uint8_t {
    let mut data: uint8_t = 0;
    let ack: bool =
        busReadRegisterBuffer(bus, reg, &mut data, 1i32 as uint8_t);
    if ack { return data } else { return 0i32 as uint8_t };
}
