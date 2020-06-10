use ::libc;
extern "C" {
    #[no_mangle]
    fn serialWrite(instance: *mut serialPort_t, ch: uint8_t);
    #[no_mangle]
    fn millis() -> timeMs_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut acc: acc_t;
    #[no_mangle]
    fn getMAhDrawn() -> int32_t;
    #[no_mangle]
    fn getAmperage() -> int32_t;
    #[no_mangle]
    fn isAmperageConfigured() -> bool;
    #[no_mangle]
    fn getBatteryCellCount() -> uint8_t;
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
    #[no_mangle]
    fn isBatteryVoltageConfigured() -> bool;
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    fn calculateBatteryPercentageRemaining() -> uint8_t;
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
    static mut gpsSol: gpsSolutionData_t;
    #[no_mangle]
    static mut attitude: attitudeEulerAngles_t;
    #[no_mangle]
    fn getEstimatedAltitude() -> int32_t;
    #[no_mangle]
    fn getEstimatedVario() -> int16_t;
    #[no_mangle]
    static mut telemetryConfig_System: telemetryConfig_t;
    #[no_mangle]
    static mut telemetrySharedPort: *mut serialPort_t;
    #[no_mangle]
    fn telemetryCheckRxPortShared(portConfig_0: *const serialPortConfig_t)
     -> bool;
    #[no_mangle]
    fn telemetryDetermineEnabledState(portSharing: portSharing_e) -> bool;
    #[no_mangle]
    fn getEscSensorData(motorNumber: uint8_t) -> *mut escSensorData_t;
    #[no_mangle]
    fn calcEscRpm(erpm: libc::c_int) -> libc::c_int;
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
pub struct DMA_Stream_TypeDef {
    pub CR: uint32_t,
    pub NDTR: uint32_t,
    pub PAR: uint32_t,
    pub M0AR: uint32_t,
    pub M1AR: uint32_t,
    pub FCR: uint32_t,
}
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
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
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
// cached here for efficiency
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
//#define DEBUG_MPU_DATA_READY_INTERRUPT
// MPU6050
// MPU3050, 6000 and 6050
// RA = Register Address
//[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
//[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
//[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
//[7:0] X_FINE_GAIN
//[7:0] Y_FINE_GAIN
//[7:0] Z_FINE_GAIN
//[15:0] XA_OFFS
//[15:0] YA_OFFS
//[15:0] ZA_OFFS
// Product ID Register
//[15:0] XG_OFFS_USR
//[15:0] YG_OFFS_USR
//[15:0] ZG_OFFS_USR
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
pub type timeDelta_t = int32_t;
pub type timeMs_t = uint32_t;
pub type timeUs_t = uint32_t;
pub type accDev_t = accDev_s;
pub type portMode_e = libc::c_uint;
pub const MODE_RXTX: portMode_e = 3;
pub const MODE_TX: portMode_e = 2;
pub const MODE_RX: portMode_e = 1;
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
pub type serialReceiveCallbackPtr
    =
    Option<unsafe extern "C" fn(_: uint16_t, _: *mut libc::c_void) -> ()>;
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
pub type serialPort_t = serialPort_s;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const FIXED_WING: C2RustUnnamed_0 = 16;
pub const SMALL_ANGLE: C2RustUnnamed_0 = 8;
pub const CALIBRATE_MAG: C2RustUnnamed_0 = 4;
pub const GPS_FIX: C2RustUnnamed_0 = 2;
pub const GPS_FIX_HOME: C2RustUnnamed_0 = 1;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_1 = 64;
pub const SENSOR_GPS: C2RustUnnamed_1 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_1 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_1 = 16;
pub const SENSOR_MAG: C2RustUnnamed_1 = 8;
pub const SENSOR_BARO: C2RustUnnamed_1 = 4;
pub const SENSOR_ACC: C2RustUnnamed_1 = 2;
pub const SENSOR_GYRO: C2RustUnnamed_1 = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct acc_s {
    pub dev: accDev_t,
    pub accSamplingInterval: uint32_t,
    pub accADC: [libc::c_float; 3],
    pub isAccelUpdatedAtLeastOnce: bool,
}
pub type acc_t = acc_s;
pub type currentMeterSource_e = libc::c_uint;
pub const CURRENT_METER_COUNT: currentMeterSource_e = 5;
pub const CURRENT_METER_MSP: currentMeterSource_e = 4;
pub const CURRENT_METER_ESC: currentMeterSource_e = 3;
pub const CURRENT_METER_VIRTUAL: currentMeterSource_e = 2;
pub const CURRENT_METER_ADC: currentMeterSource_e = 1;
pub const CURRENT_METER_NONE: currentMeterSource_e = 0;
pub type voltageMeterSource_e = libc::c_uint;
pub const VOLTAGE_METER_COUNT: voltageMeterSource_e = 3;
pub const VOLTAGE_METER_ESC: voltageMeterSource_e = 2;
pub const VOLTAGE_METER_ADC: voltageMeterSource_e = 1;
pub const VOLTAGE_METER_NONE: voltageMeterSource_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct batteryConfig_s {
    pub vbatmaxcellvoltage: uint8_t,
    pub vbatmincellvoltage: uint8_t,
    pub vbatwarningcellvoltage: uint8_t,
    pub vbatnotpresentcellvoltage: uint8_t,
    pub lvcPercentage: uint8_t,
    pub voltageMeterSource: voltageMeterSource_e,
    pub currentMeterSource: currentMeterSource_e,
    pub batteryCapacity: uint16_t,
    pub useVBatAlerts: bool,
    pub useConsumptionAlerts: bool,
    pub consumptionWarningPercentage: uint8_t,
    pub vbathysteresis: uint8_t,
    pub vbatfullcellvoltage: uint8_t,
}
pub type batteryConfig_t = batteryConfig_s;
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
//
// configuration
//
pub type serialPortConfig_t = serialPortConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsCoordinateDDDMMmmmm_s {
    pub dddmm: int16_t,
    pub mmmm: int16_t,
}
pub type gpsCoordinateDDDMMmmmm_t = gpsCoordinateDDDMMmmmm_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsLocation_s {
    pub lat: int32_t,
    pub lon: int32_t,
    pub alt: int32_t,
}
// not used for all telemetry systems, e.g. HoTT only works at 19200.
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
#[derive(Copy, Clone)]
#[repr(C)]
pub union attitudeEulerAngles_t {
    pub raw: [int16_t; 3],
    pub values: C2RustUnnamed_2,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_2 {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
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
pub type C2RustUnnamed_3 = libc::c_uint;
pub const FRSKY_VFAS_PRECISION_HIGH: C2RustUnnamed_3 = 1;
pub const FRSKY_VFAS_PRECISION_LOW: C2RustUnnamed_3 = 0;
pub type frSkyHubWriteByteFn = unsafe extern "C" fn(_: libc::c_char) -> ();
pub const TELEMETRY_STATE_INITIALIZED_SERIAL: C2RustUnnamed_4 = 1;
pub const TELEMETRY_STATE_UNINITIALIZED: C2RustUnnamed_4 = 0;
pub const TELEMETRY_STATE_INITIALIZED_EXTERNAL: C2RustUnnamed_4 = 2;
// latitude * 1e+7
// longitude * 1e+7
// altitude in 0.01m
// speed in 0.1m/s
// degrees * 10
// generic HDOP value (*100)
// should set 12 blades in Taranis
pub type C2RustUnnamed_4 = libc::c_uint;
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
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
// time difference, 32 bits always sufficient
// millisecond time
// microsecond time
#[inline]
unsafe extern "C" fn cmpTimeUs(mut a: timeUs_t, mut b: timeUs_t)
 -> timeDelta_t {
    return a.wrapping_sub(b) as timeDelta_t;
}
#[inline]
unsafe extern "C" fn batteryConfig() -> *const batteryConfig_t {
    return &mut batteryConfig_System;
}
#[inline]
unsafe extern "C" fn telemetryConfig() -> *const telemetryConfig_t {
    return &mut telemetryConfig_System;
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
 * Initial FrSky Hub Telemetry implementation by silpstream @ rcgroups.
 * Addition protocol work by airmamaf @ github.
 */
static mut frSkyHubPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
static mut portConfig: *mut serialPortConfig_t =
    0 as *const serialPortConfig_t as *mut serialPortConfig_t;
static mut frSkyHubPortSharing: portSharing_e = PORTSHARING_UNUSED;
static mut frSkyHubWriteByte: Option<frSkyHubWriteByteFn> = None;
static mut telemetryState: uint8_t =
    TELEMETRY_STATE_UNINITIALIZED as libc::c_int as uint8_t;
unsafe extern "C" fn serializeFrSkyHub(mut data: uint8_t) {
    // take care of byte stuffing
    if data as libc::c_int == 0x5e as libc::c_int {
        frSkyHubWriteByte.expect("non-null function pointer")(0x5d as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_char);
        frSkyHubWriteByte.expect("non-null function pointer")(0x3e as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_char);
    } else if data as libc::c_int == 0x5d as libc::c_int {
        frSkyHubWriteByte.expect("non-null function pointer")(0x5d as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_char);
        frSkyHubWriteByte.expect("non-null function pointer")(0x3d as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_char);
    } else {
        frSkyHubWriteByte.expect("non-null function pointer")(data as
                                                                  libc::c_char);
    };
}
unsafe extern "C" fn frSkyHubWriteFrame(id: uint8_t, data: int16_t) {
    frSkyHubWriteByte.expect("non-null function pointer")(0x5e as libc::c_int
                                                              as
                                                              libc::c_char);
    frSkyHubWriteByte.expect("non-null function pointer")(id as libc::c_char);
    serializeFrSkyHub(data as uint8_t);
    serializeFrSkyHub((data as libc::c_int >> 8 as libc::c_int) as uint8_t);
}
unsafe extern "C" fn sendTelemetryTail() {
    frSkyHubWriteByte.expect("non-null function pointer")(0x5e as libc::c_int
                                                              as
                                                              libc::c_char);
}
unsafe extern "C" fn frSkyHubWriteByteInternal(data: libc::c_char) {
    serialWrite(frSkyHubPort, data as uint8_t);
}
unsafe extern "C" fn sendAccel() {
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i < 3 as libc::c_int as libc::c_uint {
        frSkyHubWriteFrame((0x24 as libc::c_int as
                                libc::c_uint).wrapping_add(i) as uint8_t,
                           ((acc.accADC[i as usize] /
                                 acc.dev.acc_1G as libc::c_int as
                                     libc::c_float) as int16_t as libc::c_int
                                * 1000 as libc::c_int) as int16_t);
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn sendThrottleOrBatterySizeAsRpm() {
    let mut data: int16_t = 0 as libc::c_int as int16_t;
    let mut escData: *mut escSensorData_t =
        getEscSensorData(255 as libc::c_int as uint8_t);
    if !escData.is_null() {
        data =
            if ((*escData).dataAge as libc::c_int) < 255 as libc::c_int {
                (calcEscRpm((*escData).rpm as libc::c_int)) /
                    10 as libc::c_int
            } else { 0 as libc::c_int } as int16_t
    }
    frSkyHubWriteFrame(0x3 as libc::c_int as uint8_t, data);
}
unsafe extern "C" fn sendTemperature1() {
    let mut data: int16_t = 0 as libc::c_int as int16_t;
    let mut escData: *mut escSensorData_t =
        getEscSensorData(255 as libc::c_int as uint8_t);
    if !escData.is_null() {
        data =
            if ((*escData).dataAge as libc::c_int) < 255 as libc::c_int {
                (*escData).temperature as libc::c_int
            } else { 0 as libc::c_int } as int16_t
    }
    frSkyHubWriteFrame(0x2 as libc::c_int as uint8_t, data);
}
unsafe extern "C" fn sendTime() {
    let mut seconds: uint32_t =
        millis().wrapping_div(1000 as libc::c_int as libc::c_uint);
    let mut minutes: uint8_t =
        seconds.wrapping_div(60 as libc::c_int as
                                 libc::c_uint).wrapping_rem(60 as libc::c_int
                                                                as
                                                                libc::c_uint)
            as uint8_t;
    // if we fly for more than an hour, something's wrong anyway
    frSkyHubWriteFrame(0x17 as libc::c_int as uint8_t,
                       ((minutes as libc::c_int) << 8 as libc::c_int) as
                           int16_t);
    frSkyHubWriteFrame(0x18 as libc::c_int as uint8_t,
                       seconds.wrapping_rem(60 as libc::c_int as libc::c_uint)
                           as int16_t);
}
// Frsky pdf: dddmm.mmmm
// .mmmm is returned in decimal fraction of minutes.
unsafe extern "C" fn GPStoDDDMM_MMMM(mut mwiigps: int32_t,
                                     mut result:
                                         *mut gpsCoordinateDDDMMmmmm_t) {
    let mut absgps: int32_t = 0; // absgps = Minutes left * 10^7
    let mut deg: int32_t = 0; // minutes left
    let mut min: int32_t = 0;
    absgps =
        ({
             let mut _x: int32_t = mwiigps;
             if _x > 0 as libc::c_int { _x } else { -_x }
         });
    deg = (absgps as libc::c_long / 10000000 as libc::c_long) as int32_t;
    absgps =
        ((absgps as libc::c_long -
              deg as libc::c_long * 10000000 as libc::c_long) *
             60 as libc::c_int as libc::c_long) as int32_t;
    min = (absgps as libc::c_long / 10000000 as libc::c_long) as int32_t;
    if (*telemetryConfig()).frsky_coordinate_format as libc::c_uint ==
           FRSKY_FORMAT_DMS as libc::c_int as libc::c_uint {
        (*result).dddmm = (deg * 100 as libc::c_int + min) as int16_t
    } else { (*result).dddmm = (deg * 60 as libc::c_int + min) as int16_t }
    (*result).mmmm =
        ((absgps as libc::c_long -
              min as libc::c_long * 10000000 as libc::c_long) /
             1000 as libc::c_int as libc::c_long) as int16_t;
}
unsafe extern "C" fn sendLatLong(mut coord: *mut int32_t) {
    let mut coordinate: gpsCoordinateDDDMMmmmm_t =
        gpsCoordinateDDDMMmmmm_t{dddmm: 0, mmmm: 0,};
    GPStoDDDMM_MMMM(*coord.offset(0 as libc::c_int as isize),
                    &mut coordinate);
    frSkyHubWriteFrame(0x13 as libc::c_int as uint8_t, coordinate.dddmm);
    frSkyHubWriteFrame(0x1b as libc::c_int as uint8_t, coordinate.mmmm);
    frSkyHubWriteFrame(0x23 as libc::c_int as uint8_t,
                       if *coord.offset(0 as libc::c_int as isize) <
                              0 as libc::c_int {
                           'S' as i32
                       } else { 'N' as i32 } as int16_t);
    GPStoDDDMM_MMMM(*coord.offset(1 as libc::c_int as isize),
                    &mut coordinate);
    frSkyHubWriteFrame(0x12 as libc::c_int as uint8_t, coordinate.dddmm);
    frSkyHubWriteFrame(0x1a as libc::c_int as uint8_t, coordinate.mmmm);
    frSkyHubWriteFrame(0x22 as libc::c_int as uint8_t,
                       if *coord.offset(1 as libc::c_int as isize) <
                              0 as libc::c_int {
                           'W' as i32
                       } else { 'E' as i32 } as int16_t);
}
unsafe extern "C" fn sendGpsAltitude() {
    let mut altitude: uint16_t = gpsSol.llh.alt as uint16_t;
    // Send real GPS altitude only if it's reliable (there's a GPS fix)
    if stateFlags as libc::c_int & GPS_FIX as libc::c_int == 0 {
        altitude = 0 as libc::c_int as uint16_t
    }
    frSkyHubWriteFrame(0x1 as libc::c_int as uint8_t, altitude as int16_t);
    frSkyHubWriteFrame(0x9 as libc::c_int as uint8_t,
                       0 as libc::c_int as int16_t);
}
unsafe extern "C" fn sendSatalliteSignalQualityAsTemperature2(mut cycleNum:
                                                                  uint8_t) {
    let mut satellite: uint16_t = gpsSol.numSat as uint16_t;
    if gpsSol.hdop as libc::c_int > 300 as libc::c_int &&
           (cycleNum as libc::c_int % 16 as libc::c_int) < 8 as libc::c_int {
        // Every 1s
        satellite =
            constrain(gpsSol.hdop as libc::c_int, 0 as libc::c_int,
                      9999 as libc::c_int) as uint16_t
    }
    let mut data: int16_t = 0;
    if (*telemetryConfig()).frsky_unit as libc::c_uint ==
           FRSKY_UNIT_METRICS as libc::c_int as libc::c_uint {
        data = satellite as int16_t
    } else {
        let mut tmp: libc::c_float =
            (satellite as libc::c_int - 32 as libc::c_int) as libc::c_float /
                1.8f32;
        // Round the value
        tmp +=
            if tmp < 0 as libc::c_int as libc::c_float {
                -0.5f32
            } else { 0.5f32 };
        data = tmp as int16_t
    }
    frSkyHubWriteFrame(0x5 as libc::c_int as uint8_t, data);
}
unsafe extern "C" fn sendSpeed() {
    if stateFlags as libc::c_int & GPS_FIX as libc::c_int == 0 { return }
    // Speed should be sent in knots (GPS speed is in cm/s)
    // convert to knots: 1cm/s = 0.0194384449 knots
    frSkyHubWriteFrame(0x11 as libc::c_int as uint8_t,
                       (gpsSol.groundSpeed as libc::c_int *
                            1944 as libc::c_int / 100000 as libc::c_int) as
                           int16_t);
    frSkyHubWriteFrame(0x19 as libc::c_int as uint8_t,
                       (gpsSol.groundSpeed as libc::c_int *
                            1944 as libc::c_int / 100 as libc::c_int %
                            100 as libc::c_int) as int16_t);
}
unsafe extern "C" fn sendFakeLatLong() {
    // Heading is only displayed on OpenTX if non-zero lat/long is also sent
    let mut coord: [int32_t; 2] = [0 as libc::c_int, 0 as libc::c_int];
    coord[0 as libc::c_int as usize] =
        (0.01f32 *
             (*telemetryConfig()).gpsNoFixLatitude as libc::c_int as
                 libc::c_float * 10000000 as libc::c_long as libc::c_float) as
            int32_t;
    coord[1 as libc::c_int as usize] =
        (0.01f32 *
             (*telemetryConfig()).gpsNoFixLongitude as libc::c_int as
                 libc::c_float * 10000000 as libc::c_long as libc::c_float) as
            int32_t;
    sendLatLong(coord.as_mut_ptr());
}
unsafe extern "C" fn sendGPSLatLong() {
    static mut gpsFixOccured: uint8_t = 0 as libc::c_int as uint8_t;
    let mut coord: [int32_t; 2] = [0 as libc::c_int, 0 as libc::c_int];
    if stateFlags as libc::c_int & GPS_FIX as libc::c_int != 0 ||
           gpsFixOccured as libc::c_int == 1 as libc::c_int {
        // If we have ever had a fix, send the last known lat/long
        gpsFixOccured = 1 as libc::c_int as uint8_t;
        coord[0 as libc::c_int as usize] = gpsSol.llh.lat;
        coord[1 as libc::c_int as usize] = gpsSol.llh.lon;
        sendLatLong(coord.as_mut_ptr());
    } else {
        // otherwise send fake lat/long in order to display compass value
        sendFakeLatLong();
    };
}
/*
 * Send vertical speed for opentx. ID_VERT_SPEED
 * Unit is cm/s
 */
unsafe extern "C" fn sendVario() {
    frSkyHubWriteFrame(0x30 as libc::c_int as uint8_t, getEstimatedVario());
}
/*
 * Send voltage via ID_VOLT
 *
 * NOTE: This sends voltage divided by batteryCellCount. To get the real
 * battery voltage, you need to multiply the value by batteryCellCount.
 */
unsafe extern "C" fn sendVoltageCells() {
    static mut currentCell: uint16_t = 0;
    let mut cellVoltage: uint32_t = 0 as libc::c_int as uint32_t;
    let cellCount: uint8_t = getBatteryCellCount();
    if cellCount != 0 {
        currentCell =
            (currentCell as libc::c_int % cellCount as libc::c_int) as
                uint16_t;
        /*
        * Format for Voltage Data for single cells is like this:
        *
        *  llll llll cccc hhhh
        *  l: Low voltage bits
        *  h: High voltage bits
        *  c: Cell number (starting at 0)
        *
        * The actual value sent for cell voltage has resolution of 0.002 volts
        * Since vbat has resolution of 0.1 volts it has to be multiplied by 50
        */
        cellVoltage =
            (getBatteryVoltage() as
                 uint32_t).wrapping_mul(100 as libc::c_int as
                                            libc::c_uint).wrapping_add(cellCount
                                                                           as
                                                                           libc::c_uint).wrapping_div((cellCount
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           *
                                                                                                           2
                                                                                                               as
                                                                                                               libc::c_int)
                                                                                                          as
                                                                                                          libc::c_uint)
    } else { currentCell = 0 as libc::c_int as uint16_t }
    // Cell number is at bit 9-12
    let mut data: uint16_t =
        ((currentCell as libc::c_int) << 4 as libc::c_int) as uint16_t;
    // Lower voltage bits are at bit 0-8
    data =
        (data as libc::c_uint |
             (cellVoltage & 0xff as libc::c_int as libc::c_uint) <<
                 8 as libc::c_int) as uint16_t;
    // Higher voltage bits are at bits 13-15
    data =
        (data as libc::c_uint |
             (cellVoltage & 0xf00 as libc::c_int as libc::c_uint) >>
                 8 as libc::c_int) as uint16_t;
    frSkyHubWriteFrame(0x6 as libc::c_int as uint8_t, data as int16_t);
    currentCell = currentCell.wrapping_add(1);
}
/*
 * Send voltage with ID_VOLTAGE_AMP
 */
unsafe extern "C" fn sendVoltageAmp() {
    let mut voltage: uint16_t = getBatteryVoltage();
    let cellCount: uint8_t = getBatteryCellCount();
    if (*telemetryConfig()).frsky_vfas_precision as libc::c_int ==
           FRSKY_VFAS_PRECISION_HIGH as libc::c_int {
        // Use new ID 0x39 to send voltage directly in 0.1 volts resolution
        if (*telemetryConfig()).report_cell_voltage as libc::c_int != 0 &&
               cellCount as libc::c_int != 0 {
            voltage =
                (voltage as libc::c_int / cellCount as libc::c_int) as
                    uint16_t
        }
        frSkyHubWriteFrame(0x39 as libc::c_int as uint8_t,
                           voltage as int16_t);
    } else {
        // send in 0.2 volts resolution
        voltage =
            (voltage as libc::c_int *
                 (110 as libc::c_int / 21 as libc::c_int)) as uint16_t;
        if (*telemetryConfig()).report_cell_voltage as libc::c_int != 0 &&
               cellCount as libc::c_int != 0 {
            voltage =
                (voltage as libc::c_int / cellCount as libc::c_int) as
                    uint16_t
        }
        frSkyHubWriteFrame(0x3a as libc::c_int as uint8_t,
                           (voltage as libc::c_int / 100 as libc::c_int) as
                               int16_t);
        frSkyHubWriteFrame(0x3b as libc::c_int as uint8_t,
                           ((voltage as libc::c_int % 100 as libc::c_int +
                                 5 as libc::c_int) / 10 as libc::c_int) as
                               int16_t);
    };
}
unsafe extern "C" fn sendAmperage() {
    frSkyHubWriteFrame(0x28 as libc::c_int as uint8_t,
                       (getAmperage() / 10 as libc::c_int) as uint16_t as
                           int16_t);
}
unsafe extern "C" fn sendFuelLevel() {
    let mut data: int16_t = 0;
    if (*batteryConfig()).batteryCapacity as libc::c_int > 0 as libc::c_int {
        data = calculateBatteryPercentageRemaining() as uint16_t as int16_t
    } else {
        data =
            constrain(getMAhDrawn(), 0 as libc::c_int, 0xffff as libc::c_int)
                as uint16_t as int16_t
    }
    frSkyHubWriteFrame(0x4 as libc::c_int as uint8_t, data);
}
unsafe extern "C" fn sendFakeLatLongThatAllowsHeadingDisplay() {
    // Heading is only displayed on OpenTX if non-zero lat/long is also sent
    let mut coord: [int32_t; 2] =
        [(1 as libc::c_int as libc::c_long * 10000000 as libc::c_long) as
             int32_t,
         (1 as libc::c_int as libc::c_long * 10000000 as libc::c_long) as
             int32_t];
    sendLatLong(coord.as_mut_ptr());
}
unsafe extern "C" fn sendHeading() {
    frSkyHubWriteFrame(0x14 as libc::c_int as uint8_t,
                       (attitude.values.yaw as libc::c_int /
                            10 as libc::c_int) as int16_t);
    frSkyHubWriteFrame(0x1c as libc::c_int as uint8_t,
                       0 as libc::c_int as int16_t);
}
#[no_mangle]
pub unsafe extern "C" fn initFrSkyHubTelemetry() -> bool {
    if telemetryState as libc::c_int ==
           TELEMETRY_STATE_UNINITIALIZED as libc::c_int {
        portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_FRSKY_HUB);
        if !portConfig.is_null() {
            frSkyHubPortSharing =
                determinePortSharing(portConfig,
                                     FUNCTION_TELEMETRY_FRSKY_HUB);
            frSkyHubWriteByte =
                Some(frSkyHubWriteByteInternal as
                         unsafe extern "C" fn(_: libc::c_char) -> ());
            telemetryState =
                TELEMETRY_STATE_INITIALIZED_SERIAL as libc::c_int as uint8_t
        }
        return 1 as libc::c_int != 0
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn initFrSkyHubTelemetryExternal(mut frSkyHubWriteByteExternal:
                                                           Option<frSkyHubWriteByteFn>)
 -> bool {
    if telemetryState as libc::c_int ==
           TELEMETRY_STATE_UNINITIALIZED as libc::c_int {
        frSkyHubWriteByte = frSkyHubWriteByteExternal;
        telemetryState =
            TELEMETRY_STATE_INITIALIZED_EXTERNAL as libc::c_int as uint8_t;
        return 1 as libc::c_int != 0
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn freeFrSkyHubTelemetryPort() {
    closeSerialPort(frSkyHubPort);
    frSkyHubPort = 0 as *mut serialPort_t;
}
unsafe extern "C" fn configureFrSkyHubTelemetryPort() {
    if !portConfig.is_null() {
        frSkyHubPort =
            openSerialPort((*portConfig).identifier,
                           FUNCTION_TELEMETRY_FRSKY_HUB, None,
                           0 as *mut libc::c_void,
                           9600 as libc::c_int as uint32_t, MODE_TX,
                           if (*telemetryConfig()).telemetry_inverted as
                                  libc::c_int != 0 {
                               SERIAL_NOT_INVERTED as libc::c_int
                           } else { SERIAL_INVERTED as libc::c_int } as
                               portOptions_e)
    };
}
#[no_mangle]
pub unsafe extern "C" fn checkFrSkyHubTelemetryState() {
    if telemetryState as libc::c_int ==
           TELEMETRY_STATE_INITIALIZED_SERIAL as libc::c_int {
        if telemetryCheckRxPortShared(portConfig) {
            if frSkyHubPort.is_null() && !telemetrySharedPort.is_null() {
                frSkyHubPort = telemetrySharedPort
            }
        } else {
            let mut enableSerialTelemetry: bool =
                telemetryDetermineEnabledState(frSkyHubPortSharing);
            if enableSerialTelemetry as libc::c_int != 0 &&
                   frSkyHubPort.is_null() {
                configureFrSkyHubTelemetryPort();
            } else if !enableSerialTelemetry && !frSkyHubPort.is_null() {
                freeFrSkyHubTelemetryPort();
            }
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn processFrSkyHubTelemetry(mut currentTimeUs:
                                                      timeUs_t) {
    static mut frSkyHubLastCycleTime: uint32_t = 0 as libc::c_int as uint32_t;
    static mut cycleNum: uint8_t = 0 as libc::c_int as uint8_t;
    if cmpTimeUs(currentTimeUs, frSkyHubLastCycleTime) < 125000 as libc::c_int
       {
        return
    }
    frSkyHubLastCycleTime = currentTimeUs;
    cycleNum = cycleNum.wrapping_add(1);
    if sensors(SENSOR_ACC as libc::c_int as uint32_t) {
        // Sent every 125ms
        sendAccel();
    }
    if sensors((SENSOR_BARO as libc::c_int |
                    SENSOR_RANGEFINDER as libc::c_int) as uint32_t) {
        // Sent every 125ms
        sendVario();
        // Sent every 500ms
        if cycleNum as libc::c_int % 4 as libc::c_int == 0 as libc::c_int {
            let mut altitude: int16_t =
                ({
                     let mut _x: int32_t = getEstimatedAltitude();
                     if _x > 0 as libc::c_int { _x } else { -_x }
                 }) as int16_t;
            /* Allow 5s to boot correctly othervise send zero to prevent OpenTX
             * sensor lost notifications after warm boot. */
            if frSkyHubLastCycleTime < 5000000 as libc::c_int as libc::c_uint
               {
                altitude = 0 as libc::c_int as int16_t
            }
            frSkyHubWriteFrame(0x10 as libc::c_int as uint8_t,
                               (altitude as libc::c_int / 100 as libc::c_int)
                                   as int16_t);
            frSkyHubWriteFrame(0x21 as libc::c_int as uint8_t,
                               (altitude as libc::c_int % 100 as libc::c_int)
                                   as int16_t);
        }
    }
    if sensors(SENSOR_MAG as libc::c_int as uint32_t) {
        // Sent every 500ms
        if cycleNum as libc::c_int % 4 as libc::c_int == 0 as libc::c_int {
            sendHeading();
        }
    }
    // Sent every 1s
    if cycleNum as libc::c_int % 8 as libc::c_int == 0 as libc::c_int {
        sendTemperature1();
        sendThrottleOrBatterySizeAsRpm();
        if isBatteryVoltageConfigured() {
            sendVoltageCells();
            sendVoltageAmp();
            if isAmperageConfigured() { sendAmperage(); sendFuelLevel(); }
        }
        if sensors(SENSOR_GPS as libc::c_int as uint32_t) {
            sendSpeed();
            sendGpsAltitude();
            sendSatalliteSignalQualityAsTemperature2(cycleNum);
            sendGPSLatLong();
        } else if sensors(SENSOR_MAG as libc::c_int as uint32_t) {
            sendFakeLatLongThatAllowsHeadingDisplay();
        }
    }
    // Sent every 5s
    if cycleNum as libc::c_int == 40 as libc::c_int {
        cycleNum = 0 as libc::c_int as uint8_t;
        sendTime();
    }
    sendTelemetryTail();
}
#[no_mangle]
pub unsafe extern "C" fn handleFrSkyHubTelemetry(mut currentTimeUs:
                                                     timeUs_t) {
    if telemetryState as libc::c_int ==
           TELEMETRY_STATE_INITIALIZED_SERIAL as libc::c_int &&
           !frSkyHubPort.is_null() {
        processFrSkyHubTelemetry(currentTimeUs);
    };
}
