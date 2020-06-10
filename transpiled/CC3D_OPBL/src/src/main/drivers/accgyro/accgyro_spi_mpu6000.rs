use ::libc;
extern "C" {
    #[no_mangle]
    fn mpuGyroInit(gyro: *mut gyroDev_s);
    #[no_mangle]
    fn mpuGyroRead(gyro: *mut gyroDev_s) -> bool;
    #[no_mangle]
    fn mpuGyroReadSPI(gyro: *mut gyroDev_s) -> bool;
    #[no_mangle]
    fn mpuGyroDLPF(gyro: *mut gyroDev_s) -> uint8_t;
    #[no_mangle]
    fn mpuAccRead(acc: *mut accDev_s) -> bool;
    #[no_mangle]
    fn spiSetDivisor(instance: *mut SPI_TypeDef, divisor: uint16_t);
    #[no_mangle]
    fn spiBusWriteRegister(bus: *const busDevice_t, reg: uint8_t,
                           data: uint8_t) -> bool;
    #[no_mangle]
    fn spiBusReadRegister(bus: *const busDevice_t, reg: uint8_t) -> uint8_t;
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
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
    fn delayMicroseconds(us: timeUs_t);
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn failureMode(mode: failureMode_e);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct extiCallbackRec_s {
    pub fn_0: Option<extiHandlerCallback>,
}
pub type extiHandlerCallback
    =
    unsafe extern "C" fn(_: *mut extiCallbackRec_t) -> ();
pub type extiCallbackRec_t = extiCallbackRec_s;
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
    pub busdev_u: C2RustUnnamed_1,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_1 {
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
pub type sensor_align_e = libc::c_uint;
pub const CW270_DEG_FLIP: sensor_align_e = 8;
pub const CW180_DEG_FLIP: sensor_align_e = 7;
pub const CW90_DEG_FLIP: sensor_align_e = 6;
pub const CW0_DEG_FLIP: sensor_align_e = 5;
pub const CW270_DEG: sensor_align_e = 4;
pub const CW180_DEG: sensor_align_e = 3;
pub const CW90_DEG: sensor_align_e = 2;
// driver-provided alignment
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
// RF = Register Flag
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
pub type gyro_fsr_e = libc::c_uint;
pub const NUM_GYRO_FSR: gyro_fsr_e = 4;
pub const INV_FSR_2000DPS: gyro_fsr_e = 3;
pub const INV_FSR_1000DPS: gyro_fsr_e = 2;
pub const INV_FSR_500DPS: gyro_fsr_e = 1;
pub const INV_FSR_250DPS: gyro_fsr_e = 0;
pub type accel_fsr_e = libc::c_uint;
pub const NUM_ACCEL_FSR: accel_fsr_e = 4;
pub const INV_FSR_16G: accel_fsr_e = 3;
pub const INV_FSR_8G: accel_fsr_e = 2;
pub const INV_FSR_4G: accel_fsr_e = 1;
pub const INV_FSR_2G: accel_fsr_e = 0;
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
pub type gyroDev_t = gyroDev_s;
pub type accDev_t = accDev_s;
pub const SPI_CLOCK_INITIALIZATON: C2RustUnnamed_2 = 256;
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
pub type failureMode_e = libc::c_uint;
pub const FAILURE_GYRO_INIT_FAILED: failureMode_e = 6;
pub const FAILURE_FLASH_WRITE_FAILED: failureMode_e = 5;
pub const FAILURE_INVALID_EEPROM_CONTENTS: failureMode_e = 4;
pub const FAILURE_ACC_INCOMPATIBLE: failureMode_e = 3;
pub const FAILURE_ACC_INIT: failureMode_e = 2;
pub const FAILURE_MISSING_ACC: failureMode_e = 1;
pub const FAILURE_DEVELOPER: failureMode_e = 0;
pub const SPI_CLOCK_FAST: C2RustUnnamed_2 = 2;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const SPI_CLOCK_ULTRAFAST: C2RustUnnamed_2 = 2;
pub const SPI_CLOCK_STANDARD: C2RustUnnamed_2 = 4;
pub const SPI_CLOCK_SLOW: C2RustUnnamed_2 = 128;
static mut mpuSpi6000InitDone: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub unsafe extern "C" fn mpu6000SpiGyroInit(mut gyro: *mut gyroDev_t) {
    mpuGyroInit(gyro);
    mpu6000AccAndGyroInit(gyro);
    spiSetDivisor((*gyro).bus.busdev_u.spi.instance,
                  SPI_CLOCK_INITIALIZATON as libc::c_int as uint16_t);
    // Accel and Gyro DLPF Setting
    spiBusWriteRegister(&mut (*gyro).bus, 0x1a as libc::c_int as uint8_t,
                        mpuGyroDLPF(gyro)); // 18 MHz SPI clock
    delayMicroseconds(1 as libc::c_int as timeUs_t);
    spiSetDivisor((*gyro).bus.busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
    mpuGyroRead(gyro);
    if (*gyro).gyroADCRaw[1 as libc::c_int as usize] as int8_t as libc::c_int
           == -(1 as libc::c_int) &&
           (*gyro).gyroADCRaw[0 as libc::c_int as usize] as int8_t as
               libc::c_int == -(1 as libc::c_int) {
        failureMode(FAILURE_GYRO_INIT_FAILED);
    };
}
#[no_mangle]
pub unsafe extern "C" fn mpu6000SpiAccInit(mut acc: *mut accDev_t) {
    (*acc).acc_1G = (512 as libc::c_int * 4 as libc::c_int) as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn mpu6000SpiDetect(mut bus: *const busDevice_t)
 -> uint8_t {
    IOInit((*bus).busdev_u.spi.csnPin, OWNER_MPU_CS,
           0 as libc::c_int as uint8_t);
    IOConfigGPIO((*bus).busdev_u.spi.csnPin,
                 (GPIO_Mode_Out_PP as libc::c_int |
                      GPIO_Speed_50MHz as libc::c_int) as ioConfig_t);
    IOHi((*bus).busdev_u.spi.csnPin);
    spiSetDivisor((*bus).busdev_u.spi.instance,
                  SPI_CLOCK_INITIALIZATON as libc::c_int as uint16_t);
    spiBusWriteRegister(bus, 0x6b as libc::c_int as uint8_t,
                        0x80 as libc::c_int as uint8_t);
    let mut attemptsRemaining: uint8_t = 5 as libc::c_int as uint8_t;
    loop  {
        delay(150 as libc::c_int as timeMs_t);
        let whoAmI: uint8_t =
            spiBusReadRegister(bus, 0x75 as libc::c_int as uint8_t);
        if whoAmI as libc::c_int == 0x68 as libc::c_int { break ; }
        if attemptsRemaining == 0 {
            return MPU_NONE as libc::c_int as uint8_t
        }
        let fresh0 = attemptsRemaining;
        attemptsRemaining = attemptsRemaining.wrapping_sub(1);
        if !(fresh0 != 0) { break ; }
    }
    let productID: uint8_t =
        spiBusReadRegister(bus, 0xc as libc::c_int as uint8_t);
    /* look for a product ID we recognise */
    // verify product revision
    match productID as libc::c_int {
        20 | 21 | 84 | 85 | 22 | 23 | 24 | 86 | 87 | 88 | 89 | 90 => {
            return MPU_60x0_SPI as libc::c_int as uint8_t
        }
        _ => { }
    }
    return MPU_NONE as libc::c_int as uint8_t;
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
/*
 * Authors:
 * Dominic Clifton - Cleanflight implementation
 * John Ihlein - Initial FF32 code
*/
unsafe extern "C" fn mpu6000AccAndGyroInit(mut gyro: *mut gyroDev_t) {
    if mpuSpi6000InitDone { return }
    spiSetDivisor((*gyro).bus.busdev_u.spi.instance,
                  SPI_CLOCK_INITIALIZATON as libc::c_int as uint16_t);
    // Device Reset
    spiBusWriteRegister(&mut (*gyro).bus, 0x6b as libc::c_int as uint8_t,
                        0x80 as libc::c_int as uint8_t);
    delay(150 as libc::c_int as timeMs_t);
    spiBusWriteRegister(&mut (*gyro).bus, 0x68 as libc::c_int as uint8_t,
                        (3 as libc::c_int | 2 as libc::c_int |
                             1 as libc::c_int) as uint8_t);
    delay(150 as libc::c_int as timeMs_t);
    // Clock Source PPL with Z axis gyro reference
    spiBusWriteRegister(&mut (*gyro).bus, 0x6b as libc::c_int as uint8_t,
                        0x3 as libc::c_int as uint8_t);
    delayMicroseconds(15 as libc::c_int as timeUs_t);
    // Disable Primary I2C Interface
    spiBusWriteRegister(&mut (*gyro).bus, 0x6a as libc::c_int as uint8_t,
                        0x10 as libc::c_int as uint8_t);
    delayMicroseconds(15 as libc::c_int as timeUs_t);
    spiBusWriteRegister(&mut (*gyro).bus, 0x6c as libc::c_int as uint8_t,
                        0 as libc::c_int as uint8_t);
    delayMicroseconds(15 as libc::c_int as timeUs_t);
    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    spiBusWriteRegister(&mut (*gyro).bus, 0x19 as libc::c_int as uint8_t,
                        (*gyro).mpuDividerDrops);
    delayMicroseconds(15 as libc::c_int as timeUs_t);
    // Gyro +/- 1000 DPS Full Scale
    spiBusWriteRegister(&mut (*gyro).bus, 0x1b as libc::c_int as uint8_t,
                        ((INV_FSR_2000DPS as libc::c_int) << 3 as libc::c_int)
                            as uint8_t);
    delayMicroseconds(15 as libc::c_int as timeUs_t);
    // Accel +/- 16 G Full Scale
    spiBusWriteRegister(&mut (*gyro).bus, 0x1c as libc::c_int as uint8_t,
                        ((INV_FSR_16G as libc::c_int) << 3 as libc::c_int) as
                            uint8_t); // INT_ANYRD_2CLEAR
    delayMicroseconds(15 as libc::c_int as timeUs_t);
    spiBusWriteRegister(&mut (*gyro).bus, 0x37 as libc::c_int as uint8_t,
                        ((0 as libc::c_int) << 7 as libc::c_int |
                             (0 as libc::c_int) << 6 as libc::c_int |
                             (0 as libc::c_int) << 5 as libc::c_int |
                             (1 as libc::c_int) << 4 as libc::c_int |
                             (0 as libc::c_int) << 3 as libc::c_int |
                             (0 as libc::c_int) << 2 as libc::c_int |
                             (0 as libc::c_int) << 1 as libc::c_int |
                             (0 as libc::c_int) << 0 as libc::c_int) as
                            uint8_t);
    delayMicroseconds(15 as libc::c_int as timeUs_t);
    spiBusWriteRegister(&mut (*gyro).bus, 0x38 as libc::c_int as uint8_t,
                        ((1 as libc::c_int) << 0 as libc::c_int) as uint8_t);
    delayMicroseconds(15 as libc::c_int as timeUs_t);
    spiSetDivisor((*gyro).bus.busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
    delayMicroseconds(1 as libc::c_int as timeUs_t);
    mpuSpi6000InitDone = 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn mpu6000SpiAccDetect(mut acc: *mut accDev_t) -> bool {
    if (*acc).mpuDetectionResult.sensor as libc::c_uint !=
           MPU_60x0_SPI as libc::c_int as libc::c_uint {
        return 0 as libc::c_int != 0
    }
    (*acc).initFn =
        Some(mpu6000SpiAccInit as
                 unsafe extern "C" fn(_: *mut accDev_t) -> ());
    (*acc).readFn =
        Some(mpuAccRead as unsafe extern "C" fn(_: *mut accDev_s) -> bool);
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn mpu6000SpiGyroDetect(mut gyro: *mut gyroDev_t)
 -> bool {
    if (*gyro).mpuDetectionResult.sensor as libc::c_uint !=
           MPU_60x0_SPI as libc::c_int as libc::c_uint {
        return 0 as libc::c_int != 0
    }
    (*gyro).initFn =
        Some(mpu6000SpiGyroInit as
                 unsafe extern "C" fn(_: *mut gyroDev_t) -> ());
    (*gyro).readFn =
        Some(mpuGyroReadSPI as
                 unsafe extern "C" fn(_: *mut gyroDev_s) -> bool);
    // 16.4 dps/lsb scalefactor
    (*gyro).scale = 1.0f32 / 16.4f32;
    return 1 as libc::c_int != 0;
}
