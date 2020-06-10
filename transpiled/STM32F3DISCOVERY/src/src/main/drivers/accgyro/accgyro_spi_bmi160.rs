use ::libc;
extern "C" {
    #[no_mangle]
    fn spiSetDivisor(instance: *mut SPI_TypeDef, divisor: uint16_t);
    #[no_mangle]
    fn spiTransfer(instance: *mut SPI_TypeDef, txData: *const uint8_t,
                   rxData: *mut uint8_t, len: libc::c_int) -> bool;
    #[no_mangle]
    fn spiBusWriteRegister(bus: *const busDevice_t, reg: uint8_t,
                           data: uint8_t) -> bool;
    #[no_mangle]
    fn spiBusReadRegister(bus: *const busDevice_t, reg: uint8_t) -> uint8_t;
    #[no_mangle]
    fn EXTIHandlerInit(cb: *mut extiCallbackRec_t,
                       fn_0: Option<extiHandlerCallback>);
    #[no_mangle]
    fn EXTIConfig(io: IO_t, cb: *mut extiCallbackRec_t,
                  irqPriority: libc::c_int, trigger: EXTITrigger_TypeDef);
    #[no_mangle]
    fn EXTIEnable(io: IO_t, enable: bool);
    #[no_mangle]
    fn IOLo(io: IO_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn IOHi(io: IO_t);
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
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
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
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed_3,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_3 {
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct extiCallbackRec_s {
    pub fn_0: Option<extiHandlerCallback>,
}
pub type extiHandlerCallback
    =
    unsafe extern "C" fn(_: *mut extiCallbackRec_t) -> ();
// Slave I2C on SPI master
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
// initialize function
// read 3 axis data function
// read temperature if available
// scalefactor
// gyro data after calibration and alignment
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
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
// millisecond time
pub type timeMs_t = uint32_t;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const Z: C2RustUnnamed_4 = 2;
pub const Y: C2RustUnnamed_4 = 1;
pub const X: C2RustUnnamed_4 = 0;
pub type gyroDev_t = gyroDev_s;
pub type accDev_t = accDev_s;
pub type bmi160_odr = libc::c_uint;
pub const BMI160_ODR_3200_Hz: bmi160_odr = 13;
pub const BMI160_ODR_1600_Hz: bmi160_odr = 12;
pub const BMI160_ODR_800_Hz: bmi160_odr = 11;
pub type bmi160_acc_range = libc::c_uint;
pub const BMI160_RANGE_16G: bmi160_acc_range = 12;
pub const BMI160_RANGE_8G: bmi160_acc_range = 8;
pub const BMI160_RANGE_4G: bmi160_acc_range = 5;
pub const BMI160_RANGE_2G: bmi160_acc_range = 3;
pub type bmi160_gyro_range = libc::c_uint;
pub const BMI160_RANGE_2000DPS: bmi160_gyro_range = 0;
pub const BMI160_RANGE_1000DPS: bmi160_gyro_range = 1;
pub const BMI160_RANGE_500DPS: bmi160_gyro_range = 2;
pub const BMI160_RANGE_250DPS: bmi160_gyro_range = 3;
pub const BMI160_RANGE_125DPS: bmi160_gyro_range = 4;
pub const IDX_ACCEL_ZOUT_L: C2RustUnnamed_5 = 5;
pub const IDX_ACCEL_ZOUT_H: C2RustUnnamed_5 = 6;
pub const IDX_ACCEL_YOUT_L: C2RustUnnamed_5 = 3;
pub const IDX_ACCEL_YOUT_H: C2RustUnnamed_5 = 4;
pub const IDX_ACCEL_XOUT_L: C2RustUnnamed_5 = 1;
pub const IDX_ACCEL_XOUT_H: C2RustUnnamed_5 = 2;
pub const BUFFER_SIZE: C2RustUnnamed_5 = 7;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const IDX_REG: C2RustUnnamed_5 = 0;
pub const IDX_GYRO_ZOUT_L: C2RustUnnamed_6 = 5;
pub const IDX_GYRO_ZOUT_H: C2RustUnnamed_6 = 6;
pub const IDX_GYRO_YOUT_L: C2RustUnnamed_6 = 3;
pub const IDX_GYRO_YOUT_H: C2RustUnnamed_6 = 4;
pub const IDX_GYRO_XOUT_L: C2RustUnnamed_6 = 1;
pub const IDX_GYRO_XOUT_H: C2RustUnnamed_6 = 2;
pub const BUFFER_SIZE_0: C2RustUnnamed_6 = 7;
pub type C2RustUnnamed_6 = libc::c_uint;
pub const IDX_REG_0: C2RustUnnamed_6 = 0;
// /* Global Variables */
static mut BMI160InitDone: bool = 0 as libc::c_int != 0;
static mut BMI160Detected: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub unsafe extern "C" fn bmi160Detect(mut bus: *const busDevice_t)
 -> uint8_t {
    if BMI160Detected { return BMI_160_SPI as libc::c_int as uint8_t }
    IOInit((*bus).busdev_u.spi.csnPin, OWNER_MPU_CS,
           0 as libc::c_int as uint8_t);
    IOConfigGPIO((*bus).busdev_u.spi.csnPin,
                 (GPIO_Mode_OUT as libc::c_int |
                      (GPIO_Speed_Level_3 as libc::c_int) << 2 as libc::c_int
                      | (GPIO_OType_PP as libc::c_int) << 4 as libc::c_int |
                      (GPIO_PuPd_NOPULL as libc::c_int) << 5 as libc::c_int)
                     as ioConfig_t);
    IOHi((*bus).busdev_u.spi.csnPin);
    spiSetDivisor((*bus).busdev_u.spi.instance,
                  16 as libc::c_int as uint16_t);
    /* Read this address to activate SPI (see p. 84) */
    spiBusReadRegister(bus,
                       0x7f as libc::c_int as
                           uint8_t); // Give SPI some time to start up
    delay(100 as libc::c_int as timeMs_t);
    /* Check the chip ID */
    if spiBusReadRegister(bus, 0 as libc::c_int as uint8_t) as libc::c_int !=
           0xd1 as libc::c_int {
        return MPU_NONE as libc::c_int as uint8_t
    }
    ::core::ptr::write_volatile(&mut BMI160Detected as *mut bool,
                                1 as libc::c_int != 0);
    return BMI_160_SPI as libc::c_int as uint8_t;
}
/* *
 * @brief Initialize the BMI160 6-axis sensor.
 * @return 0 for success, -1 for failure to allocate, -10 for failure to get irq
 */
unsafe extern "C" fn BMI160_Init(mut bus: *const busDevice_t) {
    if BMI160InitDone as libc::c_int != 0 || !BMI160Detected { return }
    /* Configure the BMI160 Sensor */
    if BMI160_Config(bus) != 0 as libc::c_int { return }
    let mut do_foc: bool = 0 as libc::c_int != 0;
    /* Perform fast offset compensation if requested */
    if do_foc { BMI160_do_foc(bus); }
    ::core::ptr::write_volatile(&mut BMI160InitDone as *mut bool,
                                1 as libc::c_int != 0);
}
// ! Private functions
/* *
 * @brief Configure the sensor
 */
unsafe extern "C" fn BMI160_Config(mut bus: *const busDevice_t) -> int32_t {
    // Set normal power mode for gyro and accelerometer
    spiBusWriteRegister(bus, 0x7e as libc::c_int as uint8_t,
                        0x15 as libc::c_int as
                            uint8_t); // can take up to 80ms
    delay(100 as libc::c_int as timeMs_t); // can take up to 3.8ms
    spiBusWriteRegister(bus, 0x7e as libc::c_int as uint8_t,
                        0x11 as libc::c_int as uint8_t);
    delay(5 as libc::c_int as timeMs_t);
    // Verify that normal power mode was entered
    let mut pmu_status: uint8_t =
        spiBusReadRegister(bus, 0x3 as libc::c_int as uint8_t);
    if pmu_status as libc::c_int & 0x3c as libc::c_int != 0x14 as libc::c_int
       {
        return -(3 as libc::c_int)
    }
    // Set odr and ranges
    // Set acc_us = 0 acc_bwp = 0b010 so only the first filter stage is used
    spiBusWriteRegister(bus, 0x40 as libc::c_int as uint8_t,
                        (0x20 as libc::c_int |
                             BMI160_ODR_800_Hz as libc::c_int) as uint8_t);
    delay(1 as libc::c_int as timeMs_t);
    // Set gyr_bwp = 0b010 so only the first filter stage is used
    spiBusWriteRegister(bus, 0x42 as libc::c_int as uint8_t,
                        (0x20 as libc::c_int |
                             BMI160_ODR_3200_Hz as libc::c_int) as uint8_t);
    delay(1 as libc::c_int as timeMs_t);
    spiBusWriteRegister(bus, 0x41 as libc::c_int as uint8_t,
                        BMI160_RANGE_8G as libc::c_int as uint8_t);
    delay(1 as libc::c_int as timeMs_t);
    spiBusWriteRegister(bus, 0x43 as libc::c_int as uint8_t,
                        BMI160_RANGE_2000DPS as libc::c_int as uint8_t);
    delay(1 as libc::c_int as timeMs_t);
    // Enable offset compensation
    let mut val: uint8_t =
        spiBusReadRegister(bus, 0x77 as libc::c_int as uint8_t);
    spiBusWriteRegister(bus, 0x77 as libc::c_int as uint8_t,
                        (val as libc::c_int | 0xc0 as libc::c_int) as
                            uint8_t);
    // Enable data ready interrupt
    spiBusWriteRegister(bus, 0x51 as libc::c_int as uint8_t,
                        0x10 as libc::c_int as uint8_t);
    delay(1 as libc::c_int as timeMs_t);
    // Enable INT1 pin
    spiBusWriteRegister(bus, 0x53 as libc::c_int as uint8_t,
                        0xa as libc::c_int as uint8_t);
    delay(1 as libc::c_int as timeMs_t);
    // Map data ready interrupt to INT1 pin
    spiBusWriteRegister(bus, 0x56 as libc::c_int as uint8_t,
                        0x80 as libc::c_int as uint8_t);
    delay(1 as libc::c_int as timeMs_t);
    return 0 as libc::c_int;
}
unsafe extern "C" fn BMI160_do_foc(mut bus: *const busDevice_t) -> int32_t {
    // assume sensor is mounted on top
    let mut val: uint8_t = 0x7d as libc::c_int as uint8_t;
    spiBusWriteRegister(bus, 0x69 as libc::c_int as uint8_t, val);
    // Start FOC
    spiBusWriteRegister(bus, 0x7e as libc::c_int as uint8_t,
                        0x3 as libc::c_int as uint8_t);
    // Wait for FOC to complete
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 50 as libc::c_int {
        val = spiBusReadRegister(bus, 0x1b as libc::c_int as uint8_t);
        if val as libc::c_int & 0x8 as libc::c_int != 0 { break ; }
        delay(10 as libc::c_int as timeMs_t);
        i += 1
    }
    if val as libc::c_int & 0x8 as libc::c_int == 0 {
        return -(3 as libc::c_int)
    }
    // Program NVM
    val = spiBusReadRegister(bus, 0x6a as libc::c_int as uint8_t);
    spiBusWriteRegister(bus, 0x6a as libc::c_int as uint8_t,
                        (val as libc::c_int | 0x2 as libc::c_int) as uint8_t);
    spiBusWriteRegister(bus, 0x7e as libc::c_int as uint8_t,
                        0xa0 as libc::c_int as uint8_t);
    // Wait for NVM programming to complete
    let mut i_0: libc::c_int =
        0 as libc::c_int; // TODO - maybe pullup / pulldown ?
    while i_0 < 50 as libc::c_int {
        val =
            spiBusReadRegister(bus,
                               0x1b as libc::c_int as
                                   uint8_t); // receive response
        if val as libc::c_int & 0x10 as libc::c_int != 0 {
            break ; // receive response
        }
        delay(10 as libc::c_int as timeMs_t);
        i_0 += 1
    }
    if val as libc::c_int & 0x10 as libc::c_int == 0 {
        return -(6 as libc::c_int)
    }
    return 0 as libc::c_int;
}
#[no_mangle]
pub static mut bmi160IntCallbackRec: extiCallbackRec_t =
    extiCallbackRec_t{fn_0: None,};
#[no_mangle]
pub unsafe extern "C" fn bmi160ExtiHandler(mut cb: *mut extiCallbackRec_t) {
    let mut gyro: *mut gyroDev_t =
        ({
             let mut __mptr: *const extiCallbackRec_t = cb;
             (__mptr as
                  *mut libc::c_char).offset(-(24 as libc::c_ulong as isize))
                 as *mut gyroDev_t
         });
    (*gyro).dataReady = 1 as libc::c_int != 0;
}
unsafe extern "C" fn bmi160IntExtiInit(mut gyro: *mut gyroDev_t) {
    static mut bmi160ExtiInitDone: bool = 0 as libc::c_int != 0;
    if bmi160ExtiInitDone { return }
    let mut mpuIntIO: IO_t =
        IOGetByTag(((2 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 13 as libc::c_int) as ioTag_t);
    IOInit(mpuIntIO, OWNER_MPU_EXTI, 0 as libc::c_int as uint8_t);
    IOConfigGPIO(mpuIntIO,
                 (GPIO_Mode_IN as libc::c_int |
                      (0 as libc::c_int) << 2 as libc::c_int |
                      (0 as libc::c_int) << 4 as libc::c_int |
                      (GPIO_PuPd_NOPULL as libc::c_int) << 5 as libc::c_int)
                     as ioConfig_t);
    EXTIHandlerInit(&mut (*gyro).exti,
                    Some(bmi160ExtiHandler as
                             unsafe extern "C" fn(_: *mut extiCallbackRec_t)
                                 -> ()));
    EXTIConfig(mpuIntIO, &mut (*gyro).exti,
               ((0xf as libc::c_int) <<
                    (4 as libc::c_int as
                         libc::c_uint).wrapping_sub((7 as libc::c_int as
                                                         libc::c_uint).wrapping_sub(0x500
                                                                                        as
                                                                                        libc::c_int
                                                                                        as
                                                                                        uint32_t
                                                                                        >>
                                                                                        8
                                                                                            as
                                                                                            libc::c_int))
                    |
                    0xf as libc::c_int &
                        0xf as libc::c_int >>
                            (7 as libc::c_int as
                                 libc::c_uint).wrapping_sub(0x500 as
                                                                libc::c_int as
                                                                uint32_t >>
                                                                8 as
                                                                    libc::c_int))
                   << 4 as libc::c_int & 0xf0 as libc::c_int,
               EXTI_Trigger_Rising);
    EXTIEnable(mpuIntIO, 1 as libc::c_int != 0);
    bmi160ExtiInitDone = 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn bmi160AccRead(mut acc: *mut accDev_t) -> bool {
    let mut bmi160_rx_buf: [uint8_t; 7] = [0; 7];
    static mut bmi160_tx_buf: [uint8_t; 7] =
        [(0x12 as libc::c_int | 0x80 as libc::c_int) as uint8_t,
         0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
    IOLo((*acc).bus.busdev_u.spi.csnPin);
    spiTransfer((*acc).bus.busdev_u.spi.instance, bmi160_tx_buf.as_ptr(),
                bmi160_rx_buf.as_mut_ptr(), BUFFER_SIZE as libc::c_int);
    IOHi((*acc).bus.busdev_u.spi.csnPin);
    (*acc).ADCRaw[X as libc::c_int as usize] =
        ((bmi160_rx_buf[IDX_ACCEL_XOUT_H as libc::c_int as usize] as
              libc::c_int) << 8 as libc::c_int |
             bmi160_rx_buf[IDX_ACCEL_XOUT_L as libc::c_int as usize] as
                 libc::c_int) as int16_t;
    (*acc).ADCRaw[Y as libc::c_int as usize] =
        ((bmi160_rx_buf[IDX_ACCEL_YOUT_H as libc::c_int as usize] as
              libc::c_int) << 8 as libc::c_int |
             bmi160_rx_buf[IDX_ACCEL_YOUT_L as libc::c_int as usize] as
                 libc::c_int) as int16_t;
    (*acc).ADCRaw[Z as libc::c_int as usize] =
        ((bmi160_rx_buf[IDX_ACCEL_ZOUT_H as libc::c_int as usize] as
              libc::c_int) << 8 as libc::c_int |
             bmi160_rx_buf[IDX_ACCEL_ZOUT_L as libc::c_int as usize] as
                 libc::c_int) as int16_t;
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn bmi160GyroRead(mut gyro: *mut gyroDev_t) -> bool {
    let mut bmi160_rx_buf: [uint8_t; 7] = [0; 7];
    static mut bmi160_tx_buf: [uint8_t; 7] =
        [(0xc as libc::c_int | 0x80 as libc::c_int) as uint8_t,
         0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
    IOLo((*gyro).bus.busdev_u.spi.csnPin);
    spiTransfer((*gyro).bus.busdev_u.spi.instance, bmi160_tx_buf.as_ptr(),
                bmi160_rx_buf.as_mut_ptr(), BUFFER_SIZE_0 as libc::c_int);
    IOHi((*gyro).bus.busdev_u.spi.csnPin);
    (*gyro).gyroADCRaw[X as libc::c_int as usize] =
        ((bmi160_rx_buf[IDX_GYRO_XOUT_H as libc::c_int as usize] as
              libc::c_int) << 8 as libc::c_int |
             bmi160_rx_buf[IDX_GYRO_XOUT_L as libc::c_int as usize] as
                 libc::c_int) as int16_t;
    (*gyro).gyroADCRaw[Y as libc::c_int as usize] =
        ((bmi160_rx_buf[IDX_GYRO_YOUT_H as libc::c_int as usize] as
              libc::c_int) << 8 as libc::c_int |
             bmi160_rx_buf[IDX_GYRO_YOUT_L as libc::c_int as usize] as
                 libc::c_int) as int16_t;
    (*gyro).gyroADCRaw[Z as libc::c_int as usize] =
        ((bmi160_rx_buf[IDX_GYRO_ZOUT_H as libc::c_int as usize] as
              libc::c_int) << 8 as libc::c_int |
             bmi160_rx_buf[IDX_GYRO_ZOUT_L as libc::c_int as usize] as
                 libc::c_int) as int16_t;
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn bmi160SpiGyroInit(mut gyro: *mut gyroDev_t) {
    BMI160_Init(&mut (*gyro).bus);
    bmi160IntExtiInit(gyro);
}
#[no_mangle]
pub unsafe extern "C" fn bmi160SpiAccInit(mut acc: *mut accDev_t) {
    BMI160_Init(&mut (*acc).bus);
    (*acc).acc_1G = (512 as libc::c_int * 8 as libc::c_int) as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn bmi160SpiAccDetect(mut acc: *mut accDev_t) -> bool {
    if bmi160Detect(&mut (*acc).bus) as libc::c_int == MPU_NONE as libc::c_int
       {
        return 0 as libc::c_int != 0
    }
    (*acc).initFn =
        Some(bmi160SpiAccInit as
                 unsafe extern "C" fn(_: *mut accDev_t) -> ());
    (*acc).readFn =
        Some(bmi160AccRead as unsafe extern "C" fn(_: *mut accDev_t) -> bool);
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn bmi160SpiGyroDetect(mut gyro: *mut gyroDev_t)
 -> bool {
    if bmi160Detect(&mut (*gyro).bus) as libc::c_int ==
           MPU_NONE as libc::c_int {
        return 0 as libc::c_int != 0
    }
    (*gyro).initFn =
        Some(bmi160SpiGyroInit as
                 unsafe extern "C" fn(_: *mut gyroDev_t) -> ());
    (*gyro).readFn =
        Some(bmi160GyroRead as
                 unsafe extern "C" fn(_: *mut gyroDev_t) -> bool);
    (*gyro).scale = 1.0f32 / 16.4f32;
    return 1 as libc::c_int != 0;
}
// USE_ACCGYRO_BMI160
