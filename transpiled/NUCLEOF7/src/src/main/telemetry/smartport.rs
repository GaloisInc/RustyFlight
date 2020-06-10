use ::libc;
extern "C" {
    #[no_mangle]
    fn abs(_: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    fn millis() -> timeMs_t;
    #[no_mangle]
    static mut currentPidProfile: *mut pidProfile_s;
    #[no_mangle]
    static mut currentControlRateProfile: *mut controlRateConfig_t;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn isArmingDisabled() -> bool;
    #[no_mangle]
    static mut flightModeFlags: uint16_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    fn getEstimatedAltitude() -> int32_t;
    #[no_mangle]
    fn getEstimatedVario() -> int16_t;
    #[no_mangle]
    static mut attitude: attitudeEulerAngles_t;
    #[no_mangle]
    fn getMotorCount() -> uint8_t;
    #[no_mangle]
    static mut GPS_distanceToHome: uint16_t;
    #[no_mangle]
    static mut gpsSol: gpsSolutionData_t;
    #[no_mangle]
    fn serialWrite(instance: *mut serialPort_t, ch: uint8_t);
    #[no_mangle]
    fn serialRxBytesWaiting(instance: *const serialPort_t) -> uint32_t;
    #[no_mangle]
    fn serialRead(instance: *mut serialPort_t) -> uint8_t;
    #[no_mangle]
    fn findSerialPortConfig(function: serialPortFunction_e)
     -> *mut serialPortConfig_t;
    #[no_mangle]
    fn determinePortSharing(portConfig_0: *const serialPortConfig_t,
                            function: serialPortFunction_e) -> portSharing_e;
    //
// runtime
//
    #[no_mangle]
    fn openSerialPort(identifier: serialPortIdentifier_e,
                      function: serialPortFunction_e,
                      rxCallback: serialReceiveCallbackPtr,
                      rxCallbackData: *mut libc::c_void, baudrate: uint32_t,
                      mode: portMode_e, options: portOptions_e)
     -> *mut serialPort_t;
    #[no_mangle]
    fn closeSerialPort(serialPort: *mut serialPort_t);
    #[no_mangle]
    fn isBatteryVoltageConfigured() -> bool;
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
    #[no_mangle]
    fn getBatteryCellCount() -> uint8_t;
    #[no_mangle]
    fn isAmperageConfigured() -> bool;
    #[no_mangle]
    fn getAmperage() -> int32_t;
    #[no_mangle]
    fn getMAhDrawn() -> int32_t;
    #[no_mangle]
    static mut acc: acc_t;
    #[no_mangle]
    fn getEscSensorData(motorNumber: uint8_t) -> *mut escSensorData_t;
    #[no_mangle]
    fn calcEscRpm(erpm: libc::c_int) -> libc::c_int;
    #[no_mangle]
    static mut telemetryConfig_System: telemetryConfig_t;
    #[no_mangle]
    fn telemetryDetermineEnabledState(portSharing: portSharing_e) -> bool;
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
    fn sendMspReply(payloadSize: uint8_t, responseFn: mspResponseFnPtr)
     -> bool;
    #[no_mangle]
    fn handleMspFrame(frameStart: *mut uint8_t, frameLength: libc::c_int)
     -> bool;
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
/* *
  * @brief DMA Controller
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_Stream_TypeDef {
    pub CR: uint32_t,
    pub NDTR: uint32_t,
    pub PAR: uint32_t,
    pub M0AR: uint32_t,
    pub M1AR: uint32_t,
    pub FCR: uint32_t,
}
/* *
  * @brief Serial Peripheral Interface
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SR: uint32_t,
    pub DR: uint32_t,
    pub CRCPR: uint32_t,
    pub RXCRCR: uint32_t,
    pub TXCRCR: uint32_t,
    pub I2SCFGR: uint32_t,
    pub I2SPR: uint32_t,
}
/* * 
  * @brief  HAL Lock structures definition  
  */
pub type HAL_LockTypeDef = libc::c_uint;
pub const HAL_LOCKED: HAL_LockTypeDef = 1;
pub const HAL_UNLOCKED: HAL_LockTypeDef = 0;
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_dma.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of DMA HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_HAL_Driver
  * @{
  */
/* * @addtogroup DMA
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup DMA_Exported_Types DMA Exported Types
  * @brief    DMA Exported Types 
  * @{
  */
/* * 
  * @brief  DMA Configuration Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_InitTypeDef {
    pub Channel: uint32_t,
    pub Direction: uint32_t,
    pub PeriphInc: uint32_t,
    pub MemInc: uint32_t,
    pub PeriphDataAlignment: uint32_t,
    pub MemDataAlignment: uint32_t,
    pub Mode: uint32_t,
    pub Priority: uint32_t,
    pub FIFOMode: uint32_t,
    pub FIFOThreshold: uint32_t,
    pub MemBurst: uint32_t,
    pub PeriphBurst: uint32_t,
}
/* * 
  * @brief  HAL DMA State structures definition
  */
pub type HAL_DMA_StateTypeDef = libc::c_uint;
/* !< DMA Abort state                     */
/* !< DMA error state                     */
pub const HAL_DMA_STATE_ABORT: HAL_DMA_StateTypeDef = 5;
/* !< DMA timeout state                   */
pub const HAL_DMA_STATE_ERROR: HAL_DMA_StateTypeDef = 4;
/* !< DMA process is ongoing              */
pub const HAL_DMA_STATE_TIMEOUT: HAL_DMA_StateTypeDef = 3;
/* !< DMA initialized and ready for use   */
pub const HAL_DMA_STATE_BUSY: HAL_DMA_StateTypeDef = 2;
/* !< DMA not yet initialized or disabled */
pub const HAL_DMA_STATE_READY: HAL_DMA_StateTypeDef = 1;
pub const HAL_DMA_STATE_RESET: HAL_DMA_StateTypeDef = 0;
/* * 
  * @brief  DMA handle Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __DMA_HandleTypeDef {
    pub Instance: *mut DMA_Stream_TypeDef,
    pub Init: DMA_InitTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub State: HAL_DMA_StateTypeDef,
    pub Parent: *mut libc::c_void,
    pub XferCpltCallback: Option<unsafe extern "C" fn(_:
                                                          *mut __DMA_HandleTypeDef)
                                     -> ()>,
    pub XferHalfCpltCallback: Option<unsafe extern "C" fn(_:
                                                              *mut __DMA_HandleTypeDef)
                                         -> ()>,
    pub XferM1CpltCallback: Option<unsafe extern "C" fn(_:
                                                            *mut __DMA_HandleTypeDef)
                                       -> ()>,
    pub XferM1HalfCpltCallback: Option<unsafe extern "C" fn(_:
                                                                *mut __DMA_HandleTypeDef)
                                           -> ()>,
    pub XferErrorCallback: Option<unsafe extern "C" fn(_:
                                                           *mut __DMA_HandleTypeDef)
                                      -> ()>,
    pub XferAbortCallback: Option<unsafe extern "C" fn(_:
                                                           *mut __DMA_HandleTypeDef)
                                      -> ()>,
    pub ErrorCode: uint32_t,
    pub StreamBaseAddress: uint32_t,
    pub StreamIndex: uint32_t,
}
pub type DMA_HandleTypeDef = __DMA_HandleTypeDef;
/* !< DMA Stream Index                       */
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_spi.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of SPI HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_HAL_Driver
  * @{
  */
/* * @addtogroup SPI
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup SPI_Exported_Types SPI Exported Types
  * @{
  */
/* *
  * @brief  SPI Configuration Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_InitTypeDef {
    pub Mode: uint32_t,
    pub Direction: uint32_t,
    pub DataSize: uint32_t,
    pub CLKPolarity: uint32_t,
    pub CLKPhase: uint32_t,
    pub NSS: uint32_t,
    pub BaudRatePrescaler: uint32_t,
    pub FirstBit: uint32_t,
    pub TIMode: uint32_t,
    pub CRCCalculation: uint32_t,
    pub CRCPolynomial: uint32_t,
    pub CRCLength: uint32_t,
    pub NSSPMode: uint32_t,
}
/* *
  * @brief  HAL SPI State structure definition
  */
pub type HAL_SPI_StateTypeDef = libc::c_uint;
/* !< SPI abort is ongoing                               */
/* !< SPI error state                                    */
pub const HAL_SPI_STATE_ABORT: HAL_SPI_StateTypeDef = 7;
/* !< Data Transmission and Reception process is ongoing */
pub const HAL_SPI_STATE_ERROR: HAL_SPI_StateTypeDef = 6;
/* !< Data Reception process is ongoing                  */
pub const HAL_SPI_STATE_BUSY_TX_RX: HAL_SPI_StateTypeDef = 5;
/* !< Data Transmission process is ongoing               */
pub const HAL_SPI_STATE_BUSY_RX: HAL_SPI_StateTypeDef = 4;
/* !< an internal process is ongoing                     */
pub const HAL_SPI_STATE_BUSY_TX: HAL_SPI_StateTypeDef = 3;
/* !< Peripheral Initialized and ready for use           */
pub const HAL_SPI_STATE_BUSY: HAL_SPI_StateTypeDef = 2;
/* !< Peripheral not Initialized                         */
pub const HAL_SPI_STATE_READY: HAL_SPI_StateTypeDef = 1;
pub const HAL_SPI_STATE_RESET: HAL_SPI_StateTypeDef = 0;
/* *
  * @brief  SPI handle Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __SPI_HandleTypeDef {
    pub Instance: *mut SPI_TypeDef,
    pub Init: SPI_InitTypeDef,
    pub pTxBuffPtr: *mut uint8_t,
    pub TxXferSize: uint16_t,
    pub TxXferCount: uint16_t,
    pub pRxBuffPtr: *mut uint8_t,
    pub RxXferSize: uint16_t,
    pub RxXferCount: uint16_t,
    pub CRCSize: uint32_t,
    pub RxISR: Option<unsafe extern "C" fn(_: *mut __SPI_HandleTypeDef)
                          -> ()>,
    pub TxISR: Option<unsafe extern "C" fn(_: *mut __SPI_HandleTypeDef)
                          -> ()>,
    pub hdmatx: *mut DMA_HandleTypeDef,
    pub hdmarx: *mut DMA_HandleTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub State: HAL_SPI_StateTypeDef,
    pub ErrorCode: uint32_t,
}
pub type SPI_HandleTypeDef = __SPI_HandleTypeDef;
pub type C2RustUnnamed = libc::c_uint;
pub const Z: C2RustUnnamed = 2;
pub const Y: C2RustUnnamed = 1;
pub const X: C2RustUnnamed = 0;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const FD_YAW: C2RustUnnamed_0 = 2;
pub const FD_PITCH: C2RustUnnamed_0 = 1;
pub const FD_ROLL: C2RustUnnamed_0 = 0;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_1 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_1 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_1 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_1 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_1 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_1 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_1 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_1 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_1 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_1 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_1 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_1 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_1 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_1 = 8192;
pub const FEATURE_3D: C2RustUnnamed_1 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_1 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_1 = 512;
pub const FEATURE_GPS: C2RustUnnamed_1 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_1 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_1 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_1 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_1 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_1 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_1 = 1;
/* !< SPI Error code                           */
// packet tag to specify IO pin
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
    pub busdev_u: C2RustUnnamed_2,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_2 {
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
    pub handle: *mut SPI_HandleTypeDef,
    pub csnPin: IO_t,
}
pub type busDevice_t = busDevice_s;
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
// millisecond time
pub type timeMs_t = uint32_t;
pub type accDev_t = accDev_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidProfile_s {
    pub yaw_lowpass_hz: uint16_t,
    pub dterm_lowpass_hz: uint16_t,
    pub dterm_notch_hz: uint16_t,
    pub dterm_notch_cutoff: uint16_t,
    pub pid: [pidf_t; 5],
    pub dterm_filter_type: uint8_t,
    pub itermWindupPointPercent: uint8_t,
    pub pidSumLimit: uint16_t,
    pub pidSumLimitYaw: uint16_t,
    pub pidAtMinThrottle: uint8_t,
    pub levelAngleLimit: uint8_t,
    pub horizon_tilt_effect: uint8_t,
    pub horizon_tilt_expert_mode: uint8_t,
    pub antiGravityMode: uint8_t,
    pub itermThrottleThreshold: uint16_t,
    pub itermAcceleratorGain: uint16_t,
    pub yawRateAccelLimit: uint16_t,
    pub rateAccelLimit: uint16_t,
    pub crash_dthreshold: uint16_t,
    pub crash_gthreshold: uint16_t,
    pub crash_setpoint_threshold: uint16_t,
    pub crash_time: uint16_t,
    pub crash_delay: uint16_t,
    pub crash_recovery_angle: uint8_t,
    pub crash_recovery_rate: uint8_t,
    pub vbatPidCompensation: uint8_t,
    pub feedForwardTransition: uint8_t,
    pub crash_limit_yaw: uint16_t,
    pub itermLimit: uint16_t,
    pub dterm_lowpass2_hz: uint16_t,
    pub crash_recovery: uint8_t,
    pub throttle_boost: uint8_t,
    pub throttle_boost_cutoff: uint8_t,
    pub iterm_rotation: uint8_t,
    pub smart_feedforward: uint8_t,
    pub iterm_relax_type: uint8_t,
    pub iterm_relax_cutoff: uint8_t,
    pub iterm_relax: uint8_t,
    pub acro_trainer_angle_limit: uint8_t,
    pub acro_trainer_debug_axis: uint8_t,
    pub acro_trainer_gain: uint8_t,
    pub acro_trainer_lookahead_ms: uint16_t,
    pub abs_control_gain: uint8_t,
    pub abs_control_limit: uint8_t,
    pub abs_control_error_limit: uint8_t,
}
pub type pidf_t = pidf_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidf_s {
    pub P: uint8_t,
    pub I: uint8_t,
    pub D: uint8_t,
    pub F: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct controlRateConfig_s {
    pub thrMid8: uint8_t,
    pub thrExpo8: uint8_t,
    pub rates_type: uint8_t,
    pub rcRates: [uint8_t; 3],
    pub rcExpo: [uint8_t; 3],
    pub rates: [uint8_t; 3],
    pub dynThrPID: uint8_t,
    pub tpa_breakpoint: uint16_t,
    pub throttle_limit_type: uint8_t,
    pub throttle_limit_percent: uint8_t,
}
pub type controlRateConfig_t = controlRateConfig_s;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_3 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_3 = 2;
pub const ARMED: C2RustUnnamed_3 = 1;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const GPS_RESCUE_MODE: C2RustUnnamed_4 = 2048;
pub const FAILSAFE_MODE: C2RustUnnamed_4 = 1024;
pub const PASSTHRU_MODE: C2RustUnnamed_4 = 256;
pub const HEADFREE_MODE: C2RustUnnamed_4 = 64;
pub const GPS_HOLD_MODE: C2RustUnnamed_4 = 32;
pub const GPS_HOME_MODE: C2RustUnnamed_4 = 16;
pub const BARO_MODE: C2RustUnnamed_4 = 8;
pub const MAG_MODE: C2RustUnnamed_4 = 4;
pub const HORIZON_MODE: C2RustUnnamed_4 = 2;
pub const ANGLE_MODE: C2RustUnnamed_4 = 1;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const FIXED_WING: C2RustUnnamed_5 = 16;
pub const SMALL_ANGLE: C2RustUnnamed_5 = 8;
pub const CALIBRATE_MAG: C2RustUnnamed_5 = 4;
pub const GPS_FIX: C2RustUnnamed_5 = 2;
pub const GPS_FIX_HOME: C2RustUnnamed_5 = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub union attitudeEulerAngles_t {
    pub raw: [int16_t; 3],
    pub values: C2RustUnnamed_6,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_6 {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
pub type C2RustUnnamed_7 = libc::c_uint;
pub const PID_ITEM_COUNT: C2RustUnnamed_7 = 5;
pub const PID_MAG: C2RustUnnamed_7 = 4;
pub const PID_LEVEL: C2RustUnnamed_7 = 3;
pub const PID_YAW: C2RustUnnamed_7 = 2;
pub const PID_PITCH: C2RustUnnamed_7 = 1;
pub const PID_ROLL: C2RustUnnamed_7 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPort_s {
    pub vTable: *const serialPortVTable,
    pub mode: portMode_e,
    pub options: portOptions_e,
    pub baudRate: uint32_t,
    pub rxBufferSize: uint32_t,
    pub txBufferSize: uint32_t,
    pub rxBuffer: *mut uint8_t,
    pub txBuffer: *mut uint8_t,
    pub rxBufferHead: uint32_t,
    pub rxBufferTail: uint32_t,
    pub txBufferHead: uint32_t,
    pub txBufferTail: uint32_t,
    pub rxCallback: serialReceiveCallbackPtr,
    pub rxCallbackData: *mut libc::c_void,
    pub identifier: uint8_t,
}
// Breakpoint where TPA is activated
// Sets the throttle limiting type - off, scale or clip
// Sets the maximum pilot commanded throttle limit
// Define known line control states which may be passed up by underlying serial driver callback
pub type serialReceiveCallbackPtr
    =
    Option<unsafe extern "C" fn(_: uint16_t, _: *mut libc::c_void) -> ()>;
pub type portOptions_e = libc::c_uint;
pub const SERIAL_BIDIR_NOPULL: portOptions_e = 32;
pub const SERIAL_BIDIR_PP: portOptions_e = 16;
pub const SERIAL_BIDIR_OD: portOptions_e = 0;
pub const SERIAL_BIDIR: portOptions_e = 8;
pub const SERIAL_UNIDIR: portOptions_e = 0;
pub const SERIAL_PARITY_EVEN: portOptions_e = 4;
pub const SERIAL_PARITY_NO: portOptions_e = 0;
pub const SERIAL_STOPBITS_2: portOptions_e = 2;
pub const SERIAL_STOPBITS_1: portOptions_e = 0;
pub const SERIAL_INVERTED: portOptions_e = 1;
pub const SERIAL_NOT_INVERTED: portOptions_e = 0;
pub type portMode_e = libc::c_uint;
pub const MODE_RXTX: portMode_e = 3;
pub const MODE_TX: portMode_e = 2;
pub const MODE_RX: portMode_e = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPortVTable {
    pub serialWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                 _: uint8_t) -> ()>,
    pub serialTotalRxWaiting: Option<unsafe extern "C" fn(_:
                                                              *const serialPort_t)
                                         -> uint32_t>,
    pub serialTotalTxFree: Option<unsafe extern "C" fn(_: *const serialPort_t)
                                      -> uint32_t>,
    pub serialRead: Option<unsafe extern "C" fn(_: *mut serialPort_t)
                               -> uint8_t>,
    pub serialSetBaudRate: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                       _: uint32_t) -> ()>,
    pub isSerialTransmitBufferEmpty: Option<unsafe extern "C" fn(_:
                                                                     *const serialPort_t)
                                                -> bool>,
    pub setMode: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                             _: portMode_e) -> ()>,
    pub setCtrlLineStateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                        _:
                                                            Option<unsafe extern "C" fn(_:
                                                                                            *mut libc::c_void,
                                                                                        _:
                                                                                            uint16_t)
                                                                       -> ()>,
                                                        _: *mut libc::c_void)
                                       -> ()>,
    pub setBaudRateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                   _:
                                                       Option<unsafe extern "C" fn(_:
                                                                                       *mut serialPort_t,
                                                                                   _:
                                                                                       uint32_t)
                                                                  -> ()>,
                                                   _: *mut serialPort_t)
                                  -> ()>,
    pub writeBuf: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                              _: *const libc::c_void,
                                              _: libc::c_int) -> ()>,
    pub beginWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
    pub endWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
}
// used by serial drivers to return frames to app
pub type serialPort_t = serialPort_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsLocation_s {
    pub lat: int32_t,
    pub lon: int32_t,
    pub alt: int32_t,
}
/* LLH Location in NEU axis system */
pub type gpsLocation_t = gpsLocation_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsSolutionData_s {
    pub llh: gpsLocation_t,
    pub groundSpeed: uint16_t,
    pub groundCourse: uint16_t,
    pub hdop: uint16_t,
    pub numSat: uint8_t,
}
pub type gpsSolutionData_t = gpsSolutionData_s;
pub type portSharing_e = libc::c_uint;
pub const PORTSHARING_SHARED: portSharing_e = 2;
pub const PORTSHARING_NOT_SHARED: portSharing_e = 1;
pub const PORTSHARING_UNUSED: portSharing_e = 0;
pub type serialPortFunction_e = libc::c_uint;
pub const FUNCTION_LIDAR_TF: serialPortFunction_e = 32768;
pub const FUNCTION_RCDEVICE: serialPortFunction_e = 16384;
pub const FUNCTION_VTX_TRAMP: serialPortFunction_e = 8192;
pub const FUNCTION_TELEMETRY_IBUS: serialPortFunction_e = 4096;
pub const FUNCTION_VTX_SMARTAUDIO: serialPortFunction_e = 2048;
pub const FUNCTION_ESC_SENSOR: serialPortFunction_e = 1024;
pub const FUNCTION_TELEMETRY_MAVLINK: serialPortFunction_e = 512;
pub const FUNCTION_BLACKBOX: serialPortFunction_e = 128;
pub const FUNCTION_RX_SERIAL: serialPortFunction_e = 64;
pub const FUNCTION_TELEMETRY_SMARTPORT: serialPortFunction_e = 32;
pub const FUNCTION_TELEMETRY_LTM: serialPortFunction_e = 16;
pub const FUNCTION_TELEMETRY_HOTT: serialPortFunction_e = 8;
pub const FUNCTION_TELEMETRY_FRSKY_HUB: serialPortFunction_e = 4;
pub const FUNCTION_GPS: serialPortFunction_e = 2;
pub const FUNCTION_MSP: serialPortFunction_e = 1;
pub const FUNCTION_NONE: serialPortFunction_e = 0;
pub type serialPortIdentifier_e = libc::c_int;
pub const SERIAL_PORT_SOFTSERIAL2: serialPortIdentifier_e = 31;
pub const SERIAL_PORT_SOFTSERIAL1: serialPortIdentifier_e = 30;
pub const SERIAL_PORT_USB_VCP: serialPortIdentifier_e = 20;
pub const SERIAL_PORT_USART8: serialPortIdentifier_e = 7;
pub const SERIAL_PORT_USART7: serialPortIdentifier_e = 6;
pub const SERIAL_PORT_USART6: serialPortIdentifier_e = 5;
pub const SERIAL_PORT_UART5: serialPortIdentifier_e = 4;
pub const SERIAL_PORT_UART4: serialPortIdentifier_e = 3;
pub const SERIAL_PORT_USART3: serialPortIdentifier_e = 2;
pub const SERIAL_PORT_USART2: serialPortIdentifier_e = 1;
pub const SERIAL_PORT_USART1: serialPortIdentifier_e = 0;
pub const SERIAL_PORT_NONE: serialPortIdentifier_e = -1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPortConfig_s {
    pub functionMask: uint16_t,
    pub identifier: serialPortIdentifier_e,
    pub msp_baudrateIndex: uint8_t,
    pub gps_baudrateIndex: uint8_t,
    pub blackbox_baudrateIndex: uint8_t,
    pub telemetry_baudrateIndex: uint8_t,
}
pub type serialPortConfig_t = serialPortConfig_s;
pub type C2RustUnnamed_8 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_8 = 64;
pub const SENSOR_GPS: C2RustUnnamed_8 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_8 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_8 = 16;
pub const SENSOR_MAG: C2RustUnnamed_8 = 8;
pub const SENSOR_BARO: C2RustUnnamed_8 = 4;
pub const SENSOR_ACC: C2RustUnnamed_8 = 2;
pub const SENSOR_GYRO: C2RustUnnamed_8 = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct acc_s {
    pub dev: accDev_t,
    pub accSamplingInterval: uint32_t,
    pub accADC: [libc::c_float; 3],
    pub isAccelUpdatedAtLeastOnce: bool,
}
pub type acc_t = acc_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct escSensorData_t {
    pub dataAge: uint8_t,
    pub temperature: int8_t,
    pub voltage: int16_t,
    pub current: int32_t,
    pub consumption: int32_t,
    pub rpm: int16_t,
}
pub type frskyGpsCoordFormat_e = libc::c_uint;
pub const FRSKY_FORMAT_NMEA: frskyGpsCoordFormat_e = 1;
pub const FRSKY_FORMAT_DMS: frskyGpsCoordFormat_e = 0;
pub type frskyUnit_e = libc::c_uint;
pub const FRSKY_UNIT_IMPERIALS: frskyUnit_e = 1;
pub const FRSKY_UNIT_METRICS: frskyUnit_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct telemetryConfig_s {
    pub gpsNoFixLatitude: int16_t,
    pub gpsNoFixLongitude: int16_t,
    pub telemetry_inverted: uint8_t,
    pub halfDuplex: uint8_t,
    pub frsky_coordinate_format: frskyGpsCoordFormat_e,
    pub frsky_unit: frskyUnit_e,
    pub frsky_vfas_precision: uint8_t,
    pub hottAlarmSoundInterval: uint8_t,
    pub pidValuesAsTelemetry: uint8_t,
    pub report_cell_voltage: uint8_t,
    pub flysky_sensors: [uint8_t; 15],
    pub smartport_use_extra_sensors: uint8_t,
}
pub type telemetryConfig_t = telemetryConfig_s;
// latitude * 1e+7
// longitude * 1e+7
// altitude in 0.01m
// speed in 0.1m/s
// degrees * 10
// generic HDOP value (*100)
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
 * smartport.h
 *
 *  Created on: 25 October 2014
 *      Author: Frank26080115
 */
pub type C2RustUnnamed_9 = libc::c_uint;
pub const FSSP_SENSOR_ID4: C2RustUnnamed_9 = 103;
pub const FSSP_SENSOR_ID3: C2RustUnnamed_9 = 52;
pub const FSSP_SENSOR_ID2: C2RustUnnamed_9 = 13;
// there are 32 ID's polled by smartport master
    // remaining 3 bits are crc (according to comments in openTx code)
// MSP server frame
// ID of sensor. Must be something that is polled by FrSky RX
pub const FSSP_SENSOR_ID1: C2RustUnnamed_9 = 27;
// MSP client frame
pub const FSSP_MSPS_FRAME: C2RustUnnamed_9 = 50;
// MSP client frame
pub const FSSP_MSPC_FRAME_FPORT: C2RustUnnamed_9 = 49;
pub const FSSP_MSPC_FRAME_SMARTPORT: C2RustUnnamed_9 = 48;
pub const FSSP_DATA_FRAME: C2RustUnnamed_9 = 16;
pub const FSSP_DLE_XOR: C2RustUnnamed_9 = 32;
pub const FSSP_DLE: C2RustUnnamed_9 = 125;
pub const FSSP_START_STOP: C2RustUnnamed_9 = 126;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct smartPortPayload_s {
    pub frameId: uint8_t,
    pub valueId: uint16_t,
    pub data: uint32_t,
}
pub type smartPortPayload_t = smartPortPayload_s;
pub type smartPortWriteFrameFn
    =
    unsafe extern "C" fn(_: *const smartPortPayload_t) -> ();
pub type smartPortCheckQueueEmptyFn = unsafe extern "C" fn() -> bool;
pub const TELEMETRY_STATE_INITIALIZED_SERIAL: C2RustUnnamed_11 = 1;
pub const TELEMETRY_STATE_UNINITIALIZED: C2RustUnnamed_11 = 0;
pub type frSkyTableInfo_t = frSkyTableInfo_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct frSkyTableInfo_s {
    pub table: *mut uint16_t,
    pub size: uint8_t,
    pub index: uint8_t,
}
pub const FSSP_DATAID_TEMP: C2RustUnnamed_10 = 2928;
pub const FSSP_DATAID_VFAS: C2RustUnnamed_10 = 528;
pub const FSSP_DATAID_RPM: C2RustUnnamed_10 = 1280;
pub const FSSP_DATAID_CURRENT: C2RustUnnamed_10 = 512;
pub const FSSP_DATAID_GPS_ALT: C2RustUnnamed_10 = 2080;
pub const FSSP_DATAID_HOME_DIST: C2RustUnnamed_10 = 1056;
pub const FSSP_DATAID_LATLONG: C2RustUnnamed_10 = 2048;
pub const FSSP_DATAID_SPEED: C2RustUnnamed_10 = 2096;
pub const FSSP_DATAID_VARIO: C2RustUnnamed_10 = 272;
pub const FSSP_DATAID_ALTITUDE: C2RustUnnamed_10 = 256;
pub const FSSP_DATAID_ACCZ: C2RustUnnamed_10 = 1824;
pub const FSSP_DATAID_ACCY: C2RustUnnamed_10 = 1808;
pub const FSSP_DATAID_ACCX: C2RustUnnamed_10 = 1792;
pub const FSSP_DATAID_HEADING: C2RustUnnamed_10 = 2112;
pub const FSSP_DATAID_FUEL: C2RustUnnamed_10 = 1536;
pub const FSSP_DATAID_A4: C2RustUnnamed_10 = 2320;
pub const FSSP_DATAID_T2: C2RustUnnamed_10 = 1040;
pub const FSSP_DATAID_T1: C2RustUnnamed_10 = 1024;
pub const TELEMETRY_STATE_INITIALIZED_EXTERNAL: C2RustUnnamed_11 = 2;
pub const FSSP_DATAID_TEMP1: C2RustUnnamed_10 = 2929;
pub const FSSP_DATAID_TEMP8: C2RustUnnamed_10 = 2936;
pub const FSSP_DATAID_TEMP7: C2RustUnnamed_10 = 2935;
pub const FSSP_DATAID_TEMP6: C2RustUnnamed_10 = 2934;
pub const FSSP_DATAID_TEMP5: C2RustUnnamed_10 = 2933;
pub const FSSP_DATAID_TEMP4: C2RustUnnamed_10 = 2932;
pub const FSSP_DATAID_TEMP3: C2RustUnnamed_10 = 2931;
pub const FSSP_DATAID_TEMP2: C2RustUnnamed_10 = 2930;
pub const FSSP_DATAID_RPM1: C2RustUnnamed_10 = 1281;
pub const FSSP_DATAID_RPM8: C2RustUnnamed_10 = 1288;
pub const FSSP_DATAID_RPM7: C2RustUnnamed_10 = 1287;
pub const FSSP_DATAID_RPM6: C2RustUnnamed_10 = 1286;
pub const FSSP_DATAID_RPM5: C2RustUnnamed_10 = 1285;
pub const FSSP_DATAID_RPM4: C2RustUnnamed_10 = 1284;
pub const FSSP_DATAID_RPM3: C2RustUnnamed_10 = 1283;
pub const FSSP_DATAID_RPM2: C2RustUnnamed_10 = 1282;
pub const FSSP_DATAID_CURRENT1: C2RustUnnamed_10 = 513;
pub const FSSP_DATAID_CURRENT8: C2RustUnnamed_10 = 520;
pub const FSSP_DATAID_CURRENT7: C2RustUnnamed_10 = 519;
pub const FSSP_DATAID_CURRENT6: C2RustUnnamed_10 = 518;
pub const FSSP_DATAID_CURRENT5: C2RustUnnamed_10 = 517;
pub const FSSP_DATAID_CURRENT4: C2RustUnnamed_10 = 516;
pub const FSSP_DATAID_CURRENT3: C2RustUnnamed_10 = 515;
pub const FSSP_DATAID_CURRENT2: C2RustUnnamed_10 = 514;
pub const FSSP_DATAID_VFAS1: C2RustUnnamed_10 = 529;
pub const FSSP_DATAID_VFAS8: C2RustUnnamed_10 = 536;
pub const FSSP_DATAID_VFAS7: C2RustUnnamed_10 = 535;
pub const FSSP_DATAID_VFAS6: C2RustUnnamed_10 = 534;
pub const FSSP_DATAID_VFAS5: C2RustUnnamed_10 = 533;
pub const FSSP_DATAID_VFAS4: C2RustUnnamed_10 = 532;
pub const FSSP_DATAID_VFAS3: C2RustUnnamed_10 = 531;
pub const FSSP_DATAID_VFAS2: C2RustUnnamed_10 = 530;
pub type mspResponseFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut uint8_t) -> ()>;
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
 * SmartPort Telemetry implementation by frank26080115
 * see https://github.com/frank26080115/cleanflight/wiki/Using-Smart-Port
 */
// these data identifiers are obtained from https://github.com/opentx/opentx/blob/master/radio/src/telemetry/frsky_hub.h
pub type C2RustUnnamed_10 = libc::c_uint;
pub const FSSP_DATAID_A3: C2RustUnnamed_10 = 2304;
pub const FSSP_DATAID_ASPD: C2RustUnnamed_10 = 2560;
pub const FSSP_DATAID_CELLS_LAST: C2RustUnnamed_10 = 783;
pub const FSSP_DATAID_CELLS: C2RustUnnamed_10 = 768;
pub const FSSP_DATAID_CAP_USED: C2RustUnnamed_10 = 1536;
pub const FSSP_DATAID_ADC2: C2RustUnnamed_10 = 61699;
pub const FSSP_DATAID_ADC1: C2RustUnnamed_10 = 61698;
pub type C2RustUnnamed_11 = libc::c_uint;
#[inline]
unsafe extern "C" fn telemetryConfig() -> *const telemetryConfig_t {
    return &mut telemetryConfig_System;
}
// if adding more sensors then increase this value
static mut frSkyDataIdTable: [uint16_t; 17] = [0; 17];
static mut frSkyEscDataIdTable: [uint16_t; 4] =
    [FSSP_DATAID_CURRENT as libc::c_int as uint16_t,
     FSSP_DATAID_RPM as libc::c_int as uint16_t,
     FSSP_DATAID_VFAS as libc::c_int as uint16_t,
     FSSP_DATAID_TEMP as libc::c_int as uint16_t];
static mut frSkyDataIdTableInfo: frSkyTableInfo_t =
    unsafe {
        {
            let mut init =
                frSkyTableInfo_s{table: frSkyDataIdTable.as_ptr() as *mut _,
                                 size: 0 as libc::c_int as uint8_t,
                                 index: 0 as libc::c_int as uint8_t,};
            init
        }
    };
// Initialized in run_static_initializers
static mut frSkyEscDataIdTableInfo: frSkyTableInfo_t =
    frSkyTableInfo_t{table: 0 as *const uint16_t as *mut uint16_t,
                     size: 0,
                     index: 0,};
// max allowed time to find a value to send
static mut smartPortSerialPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
// The 'SmartPort'(tm) Port.
static mut portConfig: *mut serialPortConfig_t =
    0 as *const serialPortConfig_t as *mut serialPortConfig_t;
static mut smartPortPortSharing: portSharing_e = PORTSHARING_UNUSED;
static mut telemetryState: uint8_t =
    TELEMETRY_STATE_UNINITIALIZED as libc::c_int as uint8_t;
static mut smartPortWriteFrame: Option<smartPortWriteFrameFn> = None;
static mut smartPortMspReplyPending: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub unsafe extern "C" fn smartPortDataReceive(mut c: uint16_t,
                                              mut clearToSend: *mut bool,
                                              mut checkQueueEmpty:
                                                  Option<smartPortCheckQueueEmptyFn>,
                                              mut useChecksum: bool)
 -> *mut smartPortPayload_t {
    static mut rxBuffer: [uint8_t; 7] = [0; 7];
    static mut smartPortRxBytes: uint8_t = 0 as libc::c_int as uint8_t;
    static mut skipUntilStart: bool = 1 as libc::c_int != 0;
    static mut awaitingSensorId: bool = 0 as libc::c_int != 0;
    static mut byteStuffing: bool = 0 as libc::c_int != 0;
    static mut checksum: uint16_t = 0 as libc::c_int as uint16_t;
    if c as libc::c_int == FSSP_START_STOP as libc::c_int {
        *clearToSend = 0 as libc::c_int != 0;
        smartPortRxBytes = 0 as libc::c_int as uint8_t;
        awaitingSensorId = 1 as libc::c_int != 0;
        skipUntilStart = 0 as libc::c_int != 0;
        return 0 as *mut smartPortPayload_t
    } else { if skipUntilStart { return 0 as *mut smartPortPayload_t } }
    if awaitingSensorId {
        awaitingSensorId = 0 as libc::c_int != 0;
        if c as libc::c_int == FSSP_SENSOR_ID1 as libc::c_int &&
               checkQueueEmpty.expect("non-null function pointer")() as
                   libc::c_int != 0 {
            // our slot is starting, no need to decode more
            *clearToSend = 1 as libc::c_int != 0;
            skipUntilStart = 1 as libc::c_int != 0
        } else if c as libc::c_int == FSSP_SENSOR_ID2 as libc::c_int {
            checksum = 0 as libc::c_int as uint16_t
        } else { skipUntilStart = 1 as libc::c_int != 0 }
    } else {
        if c as libc::c_int == FSSP_DLE as libc::c_int {
            byteStuffing = 1 as libc::c_int != 0;
            return 0 as *mut smartPortPayload_t
        } else {
            if byteStuffing {
                c =
                    (c as libc::c_int ^ FSSP_DLE_XOR as libc::c_int) as
                        uint16_t;
                byteStuffing = 0 as libc::c_int != 0
            }
        }
        if (smartPortRxBytes as libc::c_ulong) <
               ::core::mem::size_of::<smartPortPayload_t>() as libc::c_ulong {
            let fresh0 = smartPortRxBytes;
            smartPortRxBytes = smartPortRxBytes.wrapping_add(1);
            rxBuffer[fresh0 as usize] = c as uint8_t;
            checksum =
                (checksum as libc::c_int + c as libc::c_int) as uint16_t;
            if !useChecksum &&
                   smartPortRxBytes as libc::c_ulong ==
                       ::core::mem::size_of::<smartPortPayload_t>() as
                           libc::c_ulong {
                skipUntilStart = 1 as libc::c_int != 0;
                return &mut rxBuffer as *mut [uint8_t; 7] as
                           *mut smartPortPayload_t
            }
        } else {
            skipUntilStart = 1 as libc::c_int != 0;
            checksum =
                (checksum as libc::c_int + c as libc::c_int) as uint16_t;
            checksum =
                ((checksum as libc::c_int & 0xff as libc::c_int) +
                     (checksum as libc::c_int >> 8 as libc::c_int)) as
                    uint16_t;
            if checksum as libc::c_int == 0xff as libc::c_int {
                return &mut rxBuffer as *mut [uint8_t; 7] as
                           *mut smartPortPayload_t
            }
        }
    }
    return 0 as *mut smartPortPayload_t;
}
#[no_mangle]
pub unsafe extern "C" fn smartPortSendByte(mut c: uint8_t,
                                           mut checksum: *mut uint16_t,
                                           mut port: *mut serialPort_t) {
    // smart port escape sequence
    if c as libc::c_int == FSSP_DLE as libc::c_int ||
           c as libc::c_int == FSSP_START_STOP as libc::c_int {
        serialWrite(port,
                    FSSP_DLE as libc::c_int as
                        uint8_t); // twice (one for lat, one for long)
        serialWrite(port,
                    (c as libc::c_int ^ FSSP_DLE_XOR as libc::c_int) as
                        uint8_t);
    } else { serialWrite(port, c); }
    if !checksum.is_null() {
        *checksum = (*checksum as libc::c_int + c as libc::c_int) as uint16_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn smartPortPayloadContainsMSP(mut payload:
                                                         *const smartPortPayload_t)
 -> bool {
    return (*payload).frameId as libc::c_int ==
               FSSP_MSPC_FRAME_SMARTPORT as libc::c_int ||
               (*payload).frameId as libc::c_int ==
                   FSSP_MSPC_FRAME_FPORT as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn smartPortWriteFrameSerial(mut payload:
                                                       *const smartPortPayload_t,
                                                   mut port:
                                                       *mut serialPort_t,
                                                   mut checksum: uint16_t) {
    let mut data: *mut uint8_t = payload as *mut uint8_t;
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while (i as libc::c_ulong) <
              ::core::mem::size_of::<smartPortPayload_t>() as libc::c_ulong {
        let fresh1 = data;
        data = data.offset(1);
        smartPortSendByte(*fresh1, &mut checksum, port);
        i = i.wrapping_add(1)
    }
    checksum =
        (0xff as libc::c_int -
             ((checksum as libc::c_int & 0xff as libc::c_int) +
                  (checksum as libc::c_int >> 8 as libc::c_int))) as uint16_t;
    smartPortSendByte(checksum as uint8_t, 0 as *mut uint16_t, port);
}
unsafe extern "C" fn smartPortWriteFrameInternal(mut payload:
                                                     *const smartPortPayload_t) {
    smartPortWriteFrameSerial(payload, smartPortSerialPort,
                              0 as libc::c_int as uint16_t);
}
unsafe extern "C" fn smartPortSendPackage(mut id: uint16_t,
                                          mut val: uint32_t) {
    let mut payload: smartPortPayload_t =
        smartPortPayload_t{frameId: 0, valueId: 0, data: 0,};
    payload.frameId = FSSP_DATA_FRAME as libc::c_int as uint8_t;
    payload.valueId = id;
    payload.data = val;
    smartPortWriteFrame.expect("non-null function pointer")(&mut payload);
}
unsafe extern "C" fn reportExtendedEscSensors() -> bool {
    return feature(FEATURE_ESC_SENSOR as libc::c_int as uint32_t) as
               libc::c_int != 0 &&
               (*telemetryConfig()).smartport_use_extra_sensors as libc::c_int
                   != 0;
}
unsafe extern "C" fn initSmartPortSensors() {
    frSkyDataIdTableInfo.index = 0 as libc::c_int as uint8_t;
    let fresh2 = frSkyDataIdTableInfo.index;
    frSkyDataIdTableInfo.index = frSkyDataIdTableInfo.index.wrapping_add(1);
    *frSkyDataIdTableInfo.table.offset(fresh2 as isize) =
        FSSP_DATAID_T1 as libc::c_int as uint16_t;
    let fresh3 = frSkyDataIdTableInfo.index;
    frSkyDataIdTableInfo.index = frSkyDataIdTableInfo.index.wrapping_add(1);
    *frSkyDataIdTableInfo.table.offset(fresh3 as isize) =
        FSSP_DATAID_T2 as libc::c_int as uint16_t;
    if isBatteryVoltageConfigured() {
        if !reportExtendedEscSensors() {
            let fresh4 = frSkyDataIdTableInfo.index;
            frSkyDataIdTableInfo.index =
                frSkyDataIdTableInfo.index.wrapping_add(1);
            *frSkyDataIdTableInfo.table.offset(fresh4 as isize) =
                FSSP_DATAID_VFAS as libc::c_int as uint16_t
        }
        let fresh5 = frSkyDataIdTableInfo.index;
        frSkyDataIdTableInfo.index =
            frSkyDataIdTableInfo.index.wrapping_add(1);
        *frSkyDataIdTableInfo.table.offset(fresh5 as isize) =
            FSSP_DATAID_A4 as libc::c_int as uint16_t
    }
    if isAmperageConfigured() {
        if !reportExtendedEscSensors() {
            let fresh6 = frSkyDataIdTableInfo.index;
            frSkyDataIdTableInfo.index =
                frSkyDataIdTableInfo.index.wrapping_add(1);
            *frSkyDataIdTableInfo.table.offset(fresh6 as isize) =
                FSSP_DATAID_CURRENT as libc::c_int as uint16_t
        }
        let fresh7 = frSkyDataIdTableInfo.index;
        frSkyDataIdTableInfo.index =
            frSkyDataIdTableInfo.index.wrapping_add(1);
        *frSkyDataIdTableInfo.table.offset(fresh7 as isize) =
            FSSP_DATAID_FUEL as libc::c_int as uint16_t
    }
    if sensors(SENSOR_ACC as libc::c_int as uint32_t) {
        let fresh8 = frSkyDataIdTableInfo.index;
        frSkyDataIdTableInfo.index =
            frSkyDataIdTableInfo.index.wrapping_add(1);
        *frSkyDataIdTableInfo.table.offset(fresh8 as isize) =
            FSSP_DATAID_HEADING as libc::c_int as uint16_t;
        let fresh9 = frSkyDataIdTableInfo.index;
        frSkyDataIdTableInfo.index =
            frSkyDataIdTableInfo.index.wrapping_add(1);
        *frSkyDataIdTableInfo.table.offset(fresh9 as isize) =
            FSSP_DATAID_ACCX as libc::c_int as uint16_t;
        let fresh10 = frSkyDataIdTableInfo.index;
        frSkyDataIdTableInfo.index =
            frSkyDataIdTableInfo.index.wrapping_add(1);
        *frSkyDataIdTableInfo.table.offset(fresh10 as isize) =
            FSSP_DATAID_ACCY as libc::c_int as uint16_t;
        let fresh11 = frSkyDataIdTableInfo.index;
        frSkyDataIdTableInfo.index =
            frSkyDataIdTableInfo.index.wrapping_add(1);
        *frSkyDataIdTableInfo.table.offset(fresh11 as isize) =
            FSSP_DATAID_ACCZ as libc::c_int as uint16_t
    }
    if sensors(SENSOR_BARO as libc::c_int as uint32_t) {
        let fresh12 = frSkyDataIdTableInfo.index;
        frSkyDataIdTableInfo.index =
            frSkyDataIdTableInfo.index.wrapping_add(1);
        *frSkyDataIdTableInfo.table.offset(fresh12 as isize) =
            FSSP_DATAID_ALTITUDE as libc::c_int as uint16_t;
        let fresh13 = frSkyDataIdTableInfo.index;
        frSkyDataIdTableInfo.index =
            frSkyDataIdTableInfo.index.wrapping_add(1);
        *frSkyDataIdTableInfo.table.offset(fresh13 as isize) =
            FSSP_DATAID_VARIO as libc::c_int as uint16_t
    }
    if feature(FEATURE_GPS as libc::c_int as uint32_t) {
        let fresh14 = frSkyDataIdTableInfo.index;
        frSkyDataIdTableInfo.index =
            frSkyDataIdTableInfo.index.wrapping_add(1);
        *frSkyDataIdTableInfo.table.offset(fresh14 as isize) =
            FSSP_DATAID_SPEED as libc::c_int as uint16_t;
        let fresh15 = frSkyDataIdTableInfo.index;
        frSkyDataIdTableInfo.index =
            frSkyDataIdTableInfo.index.wrapping_add(1);
        *frSkyDataIdTableInfo.table.offset(fresh15 as isize) =
            FSSP_DATAID_LATLONG as libc::c_int as uint16_t;
        let fresh16 = frSkyDataIdTableInfo.index;
        frSkyDataIdTableInfo.index =
            frSkyDataIdTableInfo.index.wrapping_add(1);
        *frSkyDataIdTableInfo.table.offset(fresh16 as isize) =
            FSSP_DATAID_LATLONG as libc::c_int as uint16_t;
        let fresh17 = frSkyDataIdTableInfo.index;
        frSkyDataIdTableInfo.index =
            frSkyDataIdTableInfo.index.wrapping_add(1);
        *frSkyDataIdTableInfo.table.offset(fresh17 as isize) =
            FSSP_DATAID_HOME_DIST as libc::c_int as uint16_t;
        let fresh18 = frSkyDataIdTableInfo.index;
        frSkyDataIdTableInfo.index =
            frSkyDataIdTableInfo.index.wrapping_add(1);
        *frSkyDataIdTableInfo.table.offset(fresh18 as isize) =
            FSSP_DATAID_GPS_ALT as libc::c_int as uint16_t
    }
    frSkyDataIdTableInfo.size = frSkyDataIdTableInfo.index;
    frSkyDataIdTableInfo.index = 0 as libc::c_int as uint8_t;
    if reportExtendedEscSensors() {
        frSkyEscDataIdTableInfo.size =
            (::core::mem::size_of::<[uint16_t; 4]>() as
                 libc::c_ulong).wrapping_div(::core::mem::size_of::<uint16_t>()
                                                 as libc::c_ulong) as uint8_t
    } else { frSkyEscDataIdTableInfo.size = 0 as libc::c_int as uint8_t };
}
#[no_mangle]
pub unsafe extern "C" fn initSmartPortTelemetry() -> bool {
    if telemetryState as libc::c_int ==
           TELEMETRY_STATE_UNINITIALIZED as libc::c_int {
        portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_SMARTPORT);
        if !portConfig.is_null() {
            smartPortPortSharing =
                determinePortSharing(portConfig,
                                     FUNCTION_TELEMETRY_SMARTPORT);
            smartPortWriteFrame =
                Some(smartPortWriteFrameInternal as
                         unsafe extern "C" fn(_: *const smartPortPayload_t)
                             -> ());
            initSmartPortSensors();
            telemetryState =
                TELEMETRY_STATE_INITIALIZED_SERIAL as libc::c_int as uint8_t
        }
        return 1 as libc::c_int != 0
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn initSmartPortTelemetryExternal(mut smartPortWriteFrameExternal:
                                                            Option<smartPortWriteFrameFn>)
 -> bool {
    if telemetryState as libc::c_int ==
           TELEMETRY_STATE_UNINITIALIZED as libc::c_int {
        smartPortWriteFrame = smartPortWriteFrameExternal;
        initSmartPortSensors();
        telemetryState =
            TELEMETRY_STATE_INITIALIZED_EXTERNAL as libc::c_int as uint8_t;
        return 1 as libc::c_int != 0
    }
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn freeSmartPortTelemetryPort() {
    closeSerialPort(smartPortSerialPort);
    smartPortSerialPort = 0 as *mut serialPort_t;
}
unsafe extern "C" fn configureSmartPortTelemetryPort() {
    if !portConfig.is_null() {
        let mut portOptions: portOptions_e =
            ((if (*telemetryConfig()).halfDuplex as libc::c_int != 0 {
                  SERIAL_BIDIR as libc::c_int
              } else { SERIAL_UNIDIR as libc::c_int }) |
                 (if (*telemetryConfig()).telemetry_inverted as libc::c_int !=
                         0 {
                      SERIAL_NOT_INVERTED as libc::c_int
                  } else { SERIAL_INVERTED as libc::c_int })) as
                portOptions_e;
        smartPortSerialPort =
            openSerialPort((*portConfig).identifier,
                           FUNCTION_TELEMETRY_SMARTPORT, None,
                           0 as *mut libc::c_void,
                           57600 as libc::c_int as uint32_t, MODE_RXTX,
                           portOptions)
    };
}
#[no_mangle]
pub unsafe extern "C" fn checkSmartPortTelemetryState() {
    if telemetryState as libc::c_int ==
           TELEMETRY_STATE_INITIALIZED_SERIAL as libc::c_int {
        let mut enableSerialTelemetry: bool =
            telemetryDetermineEnabledState(smartPortPortSharing);
        if enableSerialTelemetry as libc::c_int != 0 &&
               smartPortSerialPort.is_null() {
            configureSmartPortTelemetryPort();
        } else if !enableSerialTelemetry && !smartPortSerialPort.is_null() {
            freeSmartPortTelemetryPort();
        }
    };
}
unsafe extern "C" fn smartPortSendMspResponse(mut data: *mut uint8_t) {
    let mut payload: smartPortPayload_t =
        smartPortPayload_t{frameId: 0, valueId: 0, data: 0,};
    payload.frameId = FSSP_MSPS_FRAME as libc::c_int as uint8_t;
    memcpy(&mut payload.valueId as *mut uint16_t as *mut libc::c_void,
           data as *const libc::c_void,
           (::core::mem::size_of::<smartPortPayload_t>() as
                libc::c_ulong).wrapping_sub(::core::mem::size_of::<uint8_t>()
                                                as libc::c_ulong));
    smartPortWriteFrame.expect("non-null function pointer")(&mut payload);
}
#[no_mangle]
pub unsafe extern "C" fn processSmartPortTelemetry(mut payload:
                                                       *mut smartPortPayload_t,
                                                   mut clearToSend: *mut bool,
                                                   mut requestTimeout:
                                                       *const uint32_t) {
    static mut smartPortIdCycleCnt: uint8_t = 0 as libc::c_int as uint8_t;
    static mut t1Cnt: uint8_t = 0 as libc::c_int as uint8_t;
    static mut t2Cnt: uint8_t = 0 as libc::c_int as uint8_t;
    static mut smartPortIdOffset: uint8_t = 0 as libc::c_int as uint8_t;
    if !payload.is_null() &&
           smartPortPayloadContainsMSP(payload) as libc::c_int != 0 {
        // Do not check the physical ID here again
        // unless we start receiving other sensors' packets
        // Pass only the payload: skip frameId
        let mut frameStart: *mut uint8_t =
            &mut (*payload).valueId as *mut uint16_t as *mut uint8_t;
        smartPortMspReplyPending =
            handleMspFrame(frameStart,
                           (::core::mem::size_of::<smartPortPayload_t>() as
                                libc::c_ulong).wrapping_sub(::core::mem::size_of::<uint8_t>()
                                                                as
                                                                libc::c_ulong)
                               as libc::c_int)
    }
    let mut doRun: bool = 1 as libc::c_int != 0;
    while doRun as libc::c_int != 0 && *clearToSend as libc::c_int != 0 {
        // Ensure we won't get stuck in the loop if there happens to be nothing available to send in a timely manner - dump the slot if we loop in there for too long.
        if !requestTimeout.is_null() {
            if millis() >= *requestTimeout {
                ::core::ptr::write_volatile(clearToSend,
                                            0 as libc::c_int != 0);
                return
            }
        } else { doRun = 0 as libc::c_int != 0 }
        if smartPortMspReplyPending {
            smartPortMspReplyPending =
                sendMspReply((::core::mem::size_of::<smartPortPayload_t>() as
                                  libc::c_ulong).wrapping_sub(::core::mem::size_of::<uint8_t>()
                                                                  as
                                                                  libc::c_ulong)
                                 as uint8_t,
                             Some(smartPortSendMspResponse as
                                      unsafe extern "C" fn(_: *mut uint8_t)
                                          -> ()));
            ::core::ptr::write_volatile(clearToSend, 0 as libc::c_int != 0);
            return
        }
        // we can send back any data we want, our tables keep track of the order and frequency of each data type we send
        let mut tableInfo: *mut frSkyTableInfo_t = &mut frSkyDataIdTableInfo;
        if smartPortIdCycleCnt as libc::c_int >= 7 as libc::c_int {
            // send ESC sensors
            tableInfo = &mut frSkyEscDataIdTableInfo;
            if (*tableInfo).index as libc::c_int ==
                   (*tableInfo).size as libc::c_int {
                // end of ESC table, return to other sensors
                (*tableInfo).index = 0 as libc::c_int as uint8_t;
                smartPortIdCycleCnt = 0 as libc::c_int as uint8_t;
                smartPortIdOffset = smartPortIdOffset.wrapping_add(1);
                if smartPortIdOffset as libc::c_int ==
                       getMotorCount() as libc::c_int + 1 as libc::c_int {
                    // each motor and ESC_SENSOR_COMBINED
                    smartPortIdOffset = 0 as libc::c_int as uint8_t
                }
            }
        }
        if (smartPortIdCycleCnt as libc::c_int) < 7 as libc::c_int {
            // send other sensors
            tableInfo = &mut frSkyDataIdTableInfo;
            if (*tableInfo).index as libc::c_int ==
                   (*tableInfo).size as libc::c_int {
                // end of table reached, loop back
                (*tableInfo).index = 0 as libc::c_int as uint8_t
            }
        }
        let mut id: uint16_t =
            *(*tableInfo).table.offset((*tableInfo).index as isize);
        if smartPortIdCycleCnt as libc::c_int >= 7 as libc::c_int {
            id =
                (id as libc::c_int + smartPortIdOffset as libc::c_int) as
                    uint16_t
        }
        smartPortIdCycleCnt = smartPortIdCycleCnt.wrapping_add(1);
        (*tableInfo).index = (*tableInfo).index.wrapping_add(1);
        let mut tmpi: int32_t = 0;
        let mut tmp2: uint32_t = 0 as libc::c_int as uint32_t;
        let mut vfasVoltage: uint16_t = 0;
        let mut cellCount: uint8_t = 0;
        let mut escData: *mut escSensorData_t = 0 as *mut escSensorData_t;
        match id as libc::c_int {
            528 => {
                vfasVoltage = getBatteryVoltage();
                if (*telemetryConfig()).report_cell_voltage != 0 {
                    cellCount = getBatteryCellCount();
                    vfasVoltage =
                        if cellCount as libc::c_int != 0 {
                            (getBatteryVoltage() as libc::c_int) /
                                cellCount as libc::c_int
                        } else { 0 as libc::c_int } as uint16_t
                }
                // if nothing is sent, hasRequest isn't cleared, we already incremented the counter, just loop back to the start
                smartPortSendPackage(id,
                                     (vfasVoltage as libc::c_int *
                                          10 as libc::c_int) as
                                         uint32_t); // given in 0.1V, convert to volts
                ::core::ptr::write_volatile(clearToSend,
                                            0 as libc::c_int != 0)
            }
            529 | 530 | 531 | 532 | 533 | 534 | 535 | 536 => {
                escData =
                    getEscSensorData((id as libc::c_int -
                                          FSSP_DATAID_VFAS1 as libc::c_int) as
                                         uint8_t); // given in 10mA steps, unknown requested unit
                if !escData.is_null() {
                    smartPortSendPackage(id,
                                         (*escData).voltage as
                                             uint32_t); // unknown given unit, requested 100 = 1 meter
                    ::core::ptr::write_volatile(clearToSend,
                                                0 as libc::c_int != 0)
                }
            }
            512 => {
                smartPortSendPackage(id,
                                     (getAmperage() / 10 as libc::c_int) as
                                         uint32_t); // given in mAh, unknown requested unit
                ::core::ptr::write_volatile(clearToSend,
                                            0 as libc::c_int != 0)
            }
            513 | 514 | 515 | 516 | 517 | 518 | 519 | 520 => {
                escData =
                    getEscSensorData((id as libc::c_int -
                                          FSSP_DATAID_CURRENT1 as libc::c_int)
                                         as
                                         uint8_t); // unknown given unit but requested in 100 = 1m/s
                if !escData.is_null() {
                    smartPortSendPackage(id,
                                         (*escData).current as
                                             uint32_t); // given in 10*deg, requested in 10000 = 100 deg
                    ::core::ptr::write_volatile(clearToSend,
                                                0 as libc::c_int != 0)
                }
            }
            1280 => {
                escData =
                    getEscSensorData(255 as libc::c_int as
                                         uint8_t); // Multiply by 100 to show as x.xx g on Taranis
                if !escData.is_null() {
                    smartPortSendPackage(id,
                                         calcEscRpm((*escData).rpm as
                                                        libc::c_int) as
                                             uint32_t);
                    ::core::ptr::write_volatile(clearToSend,
                                                0 as libc::c_int != 0)
                }
            }
            1281 | 1282 | 1283 | 1284 | 1285 | 1286 | 1287 | 1288 => {
                escData =
                    getEscSensorData((id as libc::c_int -
                                          FSSP_DATAID_RPM1 as libc::c_int) as
                                         uint8_t);
                if !escData.is_null() {
                    smartPortSendPackage(id,
                                         calcEscRpm((*escData).rpm as
                                                        libc::c_int) as
                                             uint32_t);
                    ::core::ptr::write_volatile(clearToSend,
                                                0 as libc::c_int != 0)
                }
            }
            2928 => {
                escData = getEscSensorData(255 as libc::c_int as uint8_t);
                if !escData.is_null() {
                    smartPortSendPackage(id,
                                         (*escData).temperature as uint32_t);
                    ::core::ptr::write_volatile(clearToSend,
                                                0 as libc::c_int != 0)
                }
            }
            2929 | 2930 | 2931 | 2932 | 2933 | 2934 | 2935 | 2936 => {
                escData =
                    getEscSensorData((id as libc::c_int -
                                          FSSP_DATAID_TEMP1 as libc::c_int) as
                                         uint8_t);
                if !escData.is_null() {
                    smartPortSendPackage(id,
                                         (*escData).temperature as uint32_t);
                    ::core::ptr::write_volatile(clearToSend,
                                                0 as libc::c_int != 0)
                }
            }
            256 => {
                smartPortSendPackage(id, getEstimatedAltitude() as uint32_t);
                ::core::ptr::write_volatile(clearToSend,
                                            0 as libc::c_int != 0)
            }
            1536 => {
                smartPortSendPackage(id, getMAhDrawn() as uint32_t);
                ::core::ptr::write_volatile(clearToSend,
                                            0 as libc::c_int != 0)
            }
            272 => {
                smartPortSendPackage(id, getEstimatedVario() as uint32_t);
                ::core::ptr::write_volatile(clearToSend,
                                            0 as libc::c_int != 0)
            }
            2112 => {
                smartPortSendPackage(id,
                                     (attitude.values.yaw as libc::c_int *
                                          10 as libc::c_int) as uint32_t);
                ::core::ptr::write_volatile(clearToSend,
                                            0 as libc::c_int != 0)
            }
            1792 => {
                smartPortSendPackage(id,
                                     lrintf(100 as libc::c_int as
                                                libc::c_float *
                                                acc.accADC[X as libc::c_int as
                                                               usize] /
                                                acc.dev.acc_1G as libc::c_int
                                                    as libc::c_float) as
                                         uint32_t);
                ::core::ptr::write_volatile(clearToSend,
                                            0 as libc::c_int != 0)
            }
            1808 => {
                smartPortSendPackage(id,
                                     lrintf(100 as libc::c_int as
                                                libc::c_float *
                                                acc.accADC[Y as libc::c_int as
                                                               usize] /
                                                acc.dev.acc_1G as libc::c_int
                                                    as libc::c_float) as
                                         uint32_t);
                ::core::ptr::write_volatile(clearToSend,
                                            0 as libc::c_int != 0)
            }
            1824 => {
                smartPortSendPackage(id,
                                     lrintf(100 as libc::c_int as
                                                libc::c_float *
                                                acc.accADC[Z as libc::c_int as
                                                               usize] /
                                                acc.dev.acc_1G as libc::c_int
                                                    as libc::c_float) as
                                         uint32_t);
                ::core::ptr::write_volatile(clearToSend,
                                            0 as libc::c_int != 0)
            }
            1024 => {
                // we send all the flags as decimal digits for easy reading
                // the t1Cnt simply allows the telemetry view to show at least some changes
                t1Cnt =
                    t1Cnt.wrapping_add(1); // start off with at least one digit so the most significant 0 won't be cut off
                if t1Cnt as libc::c_int == 4 as libc::c_int {
                    t1Cnt = 1 as libc::c_int as uint8_t
                }
                tmpi = t1Cnt as libc::c_int * 10000 as libc::c_int;
                // the Taranis seems to be able to fit 5 digits on the screen
                // the Taranis seems to consider this number a signed 16 bit integer
                if !isArmingDisabled() {
                    tmpi += 1 as libc::c_int
                } else { tmpi += 2 as libc::c_int }
                if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
                    tmpi += 4 as libc::c_int
                }
                if flightModeFlags as libc::c_int & ANGLE_MODE as libc::c_int
                       != 0 {
                    tmpi += 10 as libc::c_int
                }
                if flightModeFlags as libc::c_int &
                       HORIZON_MODE as libc::c_int != 0 {
                    tmpi += 20 as libc::c_int
                }
                if flightModeFlags as libc::c_int &
                       PASSTHRU_MODE as libc::c_int != 0 {
                    tmpi += 40 as libc::c_int
                }
                if flightModeFlags as libc::c_int & MAG_MODE as libc::c_int !=
                       0 {
                    tmpi += 100 as libc::c_int
                }
                if flightModeFlags as libc::c_int & BARO_MODE as libc::c_int
                       != 0 {
                    tmpi += 200 as libc::c_int
                }
                if flightModeFlags as libc::c_int &
                       GPS_HOLD_MODE as libc::c_int != 0 {
                    tmpi += 1000 as libc::c_int
                }
                if flightModeFlags as libc::c_int &
                       GPS_HOME_MODE as libc::c_int != 0 {
                    tmpi += 2000 as libc::c_int
                }
                if flightModeFlags as libc::c_int &
                       HEADFREE_MODE as libc::c_int != 0 {
                    tmpi += 4000 as libc::c_int
                }
                smartPortSendPackage(id, tmpi as uint32_t);
                ::core::ptr::write_volatile(clearToSend,
                                            0 as libc::c_int != 0)
            }
            1040 => {
                if sensors(SENSOR_GPS as libc::c_int as uint32_t) {
                    // provide GPS lock status
                    smartPortSendPackage(id,
                                         ((if stateFlags as libc::c_int &
                                                  GPS_FIX as libc::c_int != 0
                                              {
                                               1000 as libc::c_int
                                           } else { 0 as libc::c_int }) +
                                              (if stateFlags as libc::c_int &
                                                      GPS_FIX_HOME as
                                                          libc::c_int != 0 {
                                                   2000 as libc::c_int
                                               } else { 0 as libc::c_int }) +
                                              gpsSol.numSat as libc::c_int) as
                                             uint32_t);
                    ::core::ptr::write_volatile(clearToSend,
                                                0 as libc::c_int != 0)
                } else if feature(FEATURE_GPS as libc::c_int as uint32_t) {
                    smartPortSendPackage(id, 0 as libc::c_int as uint32_t);
                    ::core::ptr::write_volatile(clearToSend,
                                                0 as libc::c_int != 0)
                } else if (*telemetryConfig()).pidValuesAsTelemetry != 0 {
                    match t2Cnt as libc::c_int {
                        0 => {
                            tmp2 =
                                (*currentPidProfile).pid[PID_ROLL as
                                                             libc::c_int as
                                                             usize].P as
                                    uint32_t;
                            tmp2 =
                                (tmp2 as
                                     libc::c_uint).wrapping_add((((*currentPidProfile).pid[PID_PITCH
                                                                                               as
                                                                                               libc::c_int
                                                                                               as
                                                                                               usize].P
                                                                      as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    as
                                                                    libc::c_uint)
                                    as uint32_t as uint32_t;
                            tmp2 =
                                (tmp2 as
                                     libc::c_uint).wrapping_add((((*currentPidProfile).pid[PID_YAW
                                                                                               as
                                                                                               libc::c_int
                                                                                               as
                                                                                               usize].P
                                                                      as
                                                                      libc::c_int)
                                                                     <<
                                                                     16 as
                                                                         libc::c_int)
                                                                    as
                                                                    libc::c_uint)
                                    as uint32_t as uint32_t
                        }
                        1 => {
                            tmp2 =
                                (*currentPidProfile).pid[PID_ROLL as
                                                             libc::c_int as
                                                             usize].I as
                                    uint32_t;
                            tmp2 =
                                (tmp2 as
                                     libc::c_uint).wrapping_add((((*currentPidProfile).pid[PID_PITCH
                                                                                               as
                                                                                               libc::c_int
                                                                                               as
                                                                                               usize].I
                                                                      as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    as
                                                                    libc::c_uint)
                                    as uint32_t as uint32_t;
                            tmp2 =
                                (tmp2 as
                                     libc::c_uint).wrapping_add((((*currentPidProfile).pid[PID_YAW
                                                                                               as
                                                                                               libc::c_int
                                                                                               as
                                                                                               usize].I
                                                                      as
                                                                      libc::c_int)
                                                                     <<
                                                                     16 as
                                                                         libc::c_int)
                                                                    as
                                                                    libc::c_uint)
                                    as uint32_t as uint32_t
                        }
                        2 => {
                            tmp2 =
                                (*currentPidProfile).pid[PID_ROLL as
                                                             libc::c_int as
                                                             usize].D as
                                    uint32_t;
                            tmp2 =
                                (tmp2 as
                                     libc::c_uint).wrapping_add((((*currentPidProfile).pid[PID_PITCH
                                                                                               as
                                                                                               libc::c_int
                                                                                               as
                                                                                               usize].D
                                                                      as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    as
                                                                    libc::c_uint)
                                    as uint32_t as uint32_t;
                            tmp2 =
                                (tmp2 as
                                     libc::c_uint).wrapping_add((((*currentPidProfile).pid[PID_YAW
                                                                                               as
                                                                                               libc::c_int
                                                                                               as
                                                                                               usize].D
                                                                      as
                                                                      libc::c_int)
                                                                     <<
                                                                     16 as
                                                                         libc::c_int)
                                                                    as
                                                                    libc::c_uint)
                                    as uint32_t as uint32_t
                        }
                        3 => {
                            tmp2 =
                                (*currentControlRateProfile).rates[FD_ROLL as
                                                                       libc::c_int
                                                                       as
                                                                       usize]
                                    as uint32_t;
                            tmp2 =
                                (tmp2 as
                                     libc::c_uint).wrapping_add((((*currentControlRateProfile).rates[FD_PITCH
                                                                                                         as
                                                                                                         libc::c_int
                                                                                                         as
                                                                                                         usize]
                                                                      as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    as
                                                                    libc::c_uint)
                                    as uint32_t as uint32_t;
                            tmp2 =
                                (tmp2 as
                                     libc::c_uint).wrapping_add((((*currentControlRateProfile).rates[FD_YAW
                                                                                                         as
                                                                                                         libc::c_int
                                                                                                         as
                                                                                                         usize]
                                                                      as
                                                                      libc::c_int)
                                                                     <<
                                                                     16 as
                                                                         libc::c_int)
                                                                    as
                                                                    libc::c_uint)
                                    as uint32_t as uint32_t
                        }
                        _ => { }
                    }
                    tmp2 =
                        (tmp2 as
                             libc::c_uint).wrapping_add(((t2Cnt as
                                                              libc::c_int) <<
                                                             24 as
                                                                 libc::c_int)
                                                            as libc::c_uint)
                            as uint32_t as uint32_t;
                    t2Cnt = t2Cnt.wrapping_add(1);
                    if t2Cnt as libc::c_int == 4 as libc::c_int {
                        t2Cnt = 0 as libc::c_int as uint8_t
                    }
                    smartPortSendPackage(id, tmp2);
                    ::core::ptr::write_volatile(clearToSend,
                                                0 as libc::c_int != 0)
                }
            }
            2096 => {
                if stateFlags as libc::c_int & GPS_FIX as libc::c_int != 0 {
                    //convert to knots: 1cm/s = 0.0194384449 knots
                    //Speed should be sent in knots/1000 (GPS speed is in cm/s)
                    let mut tmpui: uint32_t =
                        (gpsSol.groundSpeed as libc::c_int *
                             1944 as libc::c_int / 100 as libc::c_int) as
                            uint32_t;
                    smartPortSendPackage(id, tmpui);
                    ::core::ptr::write_volatile(clearToSend,
                                                0 as libc::c_int != 0)
                }
            }
            2048 => {
                if stateFlags as libc::c_int & GPS_FIX as libc::c_int != 0 {
                    let mut tmpui_0: uint32_t = 0 as libc::c_int as uint32_t;
                    // the same ID is sent twice, one for longitude, one for latitude
                    // the MSB of the sent uint32_t helps FrSky keep track
                    // the even/odd bit of our counter helps us keep track
                    if (*tableInfo).index as libc::c_int & 1 as libc::c_int !=
                           0 {
                        tmpui_0 =
                            abs(gpsSol.llh.lon) as
                                uint32_t; // now we have unsigned value and one bit to spare
                        tmpui_0 =
                            tmpui_0.wrapping_add(tmpui_0.wrapping_div(2 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)).wrapping_div(25
                                                                                                          as
                                                                                                          libc::c_int
                                                                                                          as
                                                                                                          libc::c_uint)
                                |
                                0x80000000 as
                                    libc::c_uint; // 6/100 = 1.5/25, division by power of 2 is fast
                        if gpsSol.llh.lon < 0 as libc::c_int {
                            tmpui_0 |=
                                0x40000000 as libc::c_int as libc::c_uint
                        }
                    } else {
                        tmpui_0 =
                            abs(gpsSol.llh.lat) as
                                uint32_t; // now we have unsigned value and one bit to spare
                        tmpui_0 =
                            tmpui_0.wrapping_add(tmpui_0.wrapping_div(2 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)).wrapping_div(25
                                                                                                          as
                                                                                                          libc::c_int
                                                                                                          as
                                                                                                          libc::c_uint); // 6/100 = 1.5/25, division by power of 2 is fast
                        if gpsSol.llh.lat < 0 as libc::c_int {
                            tmpui_0 |=
                                0x40000000 as libc::c_int as libc::c_uint
                        }
                    } // given in 0.1m , requested in 10 = 1m (should be in mm, probably a bug in opentx, tested on 2.0.1.7)
                    smartPortSendPackage(id,
                                         tmpui_0); // given in 0.1V, convert to volts
                    ::core::ptr::write_volatile(clearToSend,
                                                0 as libc::c_int != 0)
                }
            }
            1056 => {
                if stateFlags as libc::c_int & GPS_FIX as libc::c_int != 0 {
                    smartPortSendPackage(id, GPS_distanceToHome as uint32_t);
                    ::core::ptr::write_volatile(clearToSend,
                                                0 as libc::c_int != 0)
                }
            }
            2080 => {
                if stateFlags as libc::c_int & GPS_FIX as libc::c_int != 0 {
                    smartPortSendPackage(id,
                                         (gpsSol.llh.alt * 100 as libc::c_int)
                                             as uint32_t);
                    ::core::ptr::write_volatile(clearToSend,
                                                0 as libc::c_int != 0)
                }
            }
            2320 => {
                cellCount = getBatteryCellCount();
                vfasVoltage =
                    if cellCount as libc::c_int != 0 {
                        (getBatteryVoltage() as libc::c_int *
                             10 as libc::c_int) / cellCount as libc::c_int
                    } else { 0 as libc::c_int } as uint16_t;
                smartPortSendPackage(id, vfasVoltage as uint32_t);
                ::core::ptr::write_volatile(clearToSend,
                                            0 as libc::c_int != 0)
            }
            _ => { }
        }
    };
}
unsafe extern "C" fn serialCheckQueueEmpty() -> bool {
    return serialRxBytesWaiting(smartPortSerialPort) ==
               0 as libc::c_int as libc::c_uint;
}
#[no_mangle]
pub unsafe extern "C" fn handleSmartPortTelemetry() {
    let requestTimeout: uint32_t =
        millis().wrapping_add(1 as libc::c_int as libc::c_uint);
    if telemetryState as libc::c_int ==
           TELEMETRY_STATE_INITIALIZED_SERIAL as libc::c_int &&
           !smartPortSerialPort.is_null() {
        let mut payload: *mut smartPortPayload_t =
            0 as *mut smartPortPayload_t;
        let mut clearToSend: bool = 0 as libc::c_int != 0;
        while serialRxBytesWaiting(smartPortSerialPort) >
                  0 as libc::c_int as libc::c_uint && payload.is_null() {
            let mut c: uint8_t = serialRead(smartPortSerialPort);
            payload =
                smartPortDataReceive(c as uint16_t, &mut clearToSend,
                                     Some(serialCheckQueueEmpty as
                                              unsafe extern "C" fn() -> bool),
                                     1 as libc::c_int != 0)
        }
        processSmartPortTelemetry(payload,
                                  &mut clearToSend as *mut bool as *mut bool,
                                  &requestTimeout);
    };
}
unsafe extern "C" fn run_static_initializers() {
    frSkyEscDataIdTableInfo =
        {
            let mut init =
                frSkyTableInfo_s{table: frSkyEscDataIdTable.as_mut_ptr(),
                                 size:
                                     (::core::mem::size_of::<[uint16_t; 4]>()
                                          as
                                          libc::c_ulong).wrapping_div(::core::mem::size_of::<uint16_t>()
                                                                          as
                                                                          libc::c_ulong)
                                         as uint8_t,
                                 index: 0 as libc::c_int as uint8_t,};
            init
        }
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];