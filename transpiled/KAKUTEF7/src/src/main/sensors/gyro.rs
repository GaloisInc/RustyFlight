use ::libc;
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
    pub type filter_s;
    #[no_mangle]
    fn fabsf(_: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
    #[no_mangle]
    fn abs(_: libc::c_int) -> libc::c_int;
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
    fn devClear(dev: *mut stdev_t);
    #[no_mangle]
    fn devPush(dev: *mut stdev_t, x: libc::c_float);
    #[no_mangle]
    fn devStandardDeviation(dev: *mut stdev_t) -> libc::c_float;
    #[no_mangle]
    fn nullFilterApply(filter: *mut filter_t, input: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn biquadFilterInitLPF(filter: *mut biquadFilter_t,
                           filterFreq: libc::c_float, refreshRate: uint32_t);
    #[no_mangle]
    fn biquadFilterInit(filter: *mut biquadFilter_t,
                        filterFreq: libc::c_float, refreshRate: uint32_t,
                        Q: libc::c_float, filterType: biquadFilterType_e);
    #[no_mangle]
    fn biquadFilterApplyDF1(filter: *mut biquadFilter_t, input: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn biquadFilterApply(filter: *mut biquadFilter_t, input: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn filterGetNotchQ(centerFreq: libc::c_float, cutoffFreq: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn pt1FilterGain(f_cut: uint16_t, dT: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn pt1FilterInit(filter: *mut pt1Filter_t, k: libc::c_float);
    #[no_mangle]
    fn pt1FilterApply(filter: *mut pt1Filter_t, input: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut mpuResetFn: mpuResetFnPtr;
    #[no_mangle]
    fn mpuDetect(gyro_0: *mut gyroDev_s);
    #[no_mangle]
    fn mpuGyroReadRegister(bus: *const busDevice_t, reg: uint8_t) -> uint8_t;
    #[no_mangle]
    fn icm20689SpiGyroDetect(gyro_0: *mut gyroDev_t) -> bool;
    #[no_mangle]
    fn gyroSetSampleRate(gyro_0: *mut gyroDev_t, lpf: uint8_t,
                         gyroSyncDenominator: uint8_t, gyro_use_32khz: bool)
     -> uint32_t;
    #[no_mangle]
    fn getArmingDisableFlags() -> armingDisableFlags_e;
    #[no_mangle]
    fn sensorsSet(mask: uint32_t);
    #[no_mangle]
    fn beeper(mode: beeperMode_e);
    #[no_mangle]
    fn schedulerResetTaskStatistics(taskId: cfTaskId_e);
    #[no_mangle]
    fn alignSensors(dest: *mut libc::c_float, rotation: uint8_t);
    #[no_mangle]
    fn gyroDataAnalyseStateInit(gyroAnalyse: *mut gyroAnalyseState_t,
                                targetLooptime: uint32_t);
    #[no_mangle]
    fn gyroDataAnalysePush(gyroAnalyse: *mut gyroAnalyseState_t,
                           axis: libc::c_int, sample: libc::c_float);
    #[no_mangle]
    fn gyroDataAnalyse(gyroAnalyse: *mut gyroAnalyseState_t,
                       notchFilterDyn: *mut biquadFilter_t);
    #[no_mangle]
    static mut detectedSensors: [uint8_t; 5];
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
pub const DEBUG_COUNT: C2RustUnnamed = 44;
pub const DEBUG_ANTI_GRAVITY: C2RustUnnamed = 43;
pub const DEBUG_RC_SMOOTHING_RATE: C2RustUnnamed = 42;
pub const DEBUG_RX_SIGNAL_LOSS: C2RustUnnamed = 41;
pub const DEBUG_RC_SMOOTHING: C2RustUnnamed = 40;
pub const DEBUG_ACRO_TRAINER: C2RustUnnamed = 39;
pub const DEBUG_ITERM_RELAX: C2RustUnnamed = 38;
pub const DEBUG_RTH: C2RustUnnamed = 37;
pub const DEBUG_SMARTAUDIO: C2RustUnnamed = 36;
pub const DEBUG_USB: C2RustUnnamed = 35;
pub const DEBUG_CURRENT: C2RustUnnamed = 34;
pub const DEBUG_SDIO: C2RustUnnamed = 33;
pub const DEBUG_RUNAWAY_TAKEOFF: C2RustUnnamed = 32;
pub const DEBUG_CORE_TEMP: C2RustUnnamed = 31;
pub const DEBUG_LIDAR_TF: C2RustUnnamed = 30;
pub const DEBUG_RANGEFINDER_QUALITY: C2RustUnnamed = 29;
pub const DEBUG_RANGEFINDER: C2RustUnnamed = 28;
pub const DEBUG_FPORT: C2RustUnnamed = 27;
pub const DEBUG_SBUS: C2RustUnnamed = 26;
pub const DEBUG_MAX7456_SPICLOCK: C2RustUnnamed = 25;
pub const DEBUG_MAX7456_SIGNAL: C2RustUnnamed = 24;
pub const DEBUG_DUAL_GYRO_DIFF: C2RustUnnamed = 23;
pub const DEBUG_DUAL_GYRO_COMBINE: C2RustUnnamed = 22;
pub const DEBUG_DUAL_GYRO_RAW: C2RustUnnamed = 21;
pub const DEBUG_DUAL_GYRO: C2RustUnnamed = 20;
pub const DEBUG_GYRO_RAW: C2RustUnnamed = 19;
pub const DEBUG_RX_FRSKY_SPI: C2RustUnnamed = 18;
pub const DEBUG_FFT_FREQ: C2RustUnnamed = 17;
pub const DEBUG_FFT_TIME: C2RustUnnamed = 16;
pub const DEBUG_FFT: C2RustUnnamed = 15;
pub const DEBUG_ALTITUDE: C2RustUnnamed = 14;
pub const DEBUG_ESC_SENSOR_TMP: C2RustUnnamed = 13;
pub const DEBUG_ESC_SENSOR_RPM: C2RustUnnamed = 12;
pub const DEBUG_STACK: C2RustUnnamed = 11;
pub const DEBUG_SCHEDULER: C2RustUnnamed = 10;
pub const DEBUG_ESC_SENSOR: C2RustUnnamed = 9;
pub const DEBUG_ANGLERATE: C2RustUnnamed = 8;
pub const DEBUG_RC_INTERPOLATION: C2RustUnnamed = 7;
pub const DEBUG_GYRO_SCALED: C2RustUnnamed = 6;
pub const DEBUG_PIDLOOP: C2RustUnnamed = 5;
pub const DEBUG_ACCELEROMETER: C2RustUnnamed = 4;
pub const DEBUG_GYRO_FILTERED: C2RustUnnamed = 3;
pub const DEBUG_BATTERY: C2RustUnnamed = 2;
pub const DEBUG_CYCLETIME: C2RustUnnamed = 1;
pub const DEBUG_NONE: C2RustUnnamed = 0;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const Z: C2RustUnnamed_0 = 2;
pub const Y: C2RustUnnamed_0 = 1;
pub const X: C2RustUnnamed_0 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct stdev_s {
    pub m_oldM: libc::c_float,
    pub m_newM: libc::c_float,
    pub m_oldS: libc::c_float,
    pub m_newS: libc::c_float,
    pub m_n: libc::c_int,
}
pub type stdev_t = stdev_s;
pub type filter_t = filter_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pt1Filter_s {
    pub state: libc::c_float,
    pub k: libc::c_float,
}
pub type pt1Filter_t = pt1Filter_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct biquadFilter_s {
    pub b0: libc::c_float,
    pub b1: libc::c_float,
    pub b2: libc::c_float,
    pub a1: libc::c_float,
    pub a2: libc::c_float,
    pub x1: libc::c_float,
    pub x2: libc::c_float,
    pub y1: libc::c_float,
    pub y2: libc::c_float,
}
/* !< SPI Error code                           */
/* this holds the data required to update samples thru a filter */
pub type biquadFilter_t = biquadFilter_s;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const FILTER_BIQUAD: C2RustUnnamed_1 = 1;
pub const FILTER_PT1: C2RustUnnamed_1 = 0;
pub type biquadFilterType_e = libc::c_uint;
pub const FILTER_BPF: biquadFilterType_e = 2;
pub const FILTER_NOTCH: biquadFilterType_e = 1;
pub const FILTER_LPF: biquadFilterType_e = 0;
pub type filterApplyFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut filter_t, _: libc::c_float)
               -> libc::c_float>;
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
pub type pgn_t = uint16_t;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed_2 = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed_2 = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed_2 = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed_2 = 4095;
// function that resets a single parameter group instance
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_3,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_3 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
}
pub type pgRegistry_t = pgRegistry_s;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_4 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_4 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_4 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_4 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_4 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_4 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_4 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_4 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_4 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_4 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_4 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_4 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_4 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_4 = 8192;
pub const FEATURE_3D: C2RustUnnamed_4 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_4 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_4 = 512;
pub const FEATURE_GPS: C2RustUnnamed_4 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_4 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_4 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_4 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_4 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_4 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_4 = 1;
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
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
    pub busdev_u: C2RustUnnamed_5,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_5 {
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
pub type gyroOverflow_e = libc::c_uint;
pub const GYRO_OVERFLOW_Z: gyroOverflow_e = 4;
pub const GYRO_OVERFLOW_Y: gyroOverflow_e = 2;
pub const GYRO_OVERFLOW_X: gyroOverflow_e = 1;
pub const GYRO_OVERFLOW_NONE: gyroOverflow_e = 0;
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
pub type timeDelta_t = int32_t;
// microsecond time
pub type timeUs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gyro_s {
    pub targetLooptime: uint32_t,
    pub gyroADCf: [libc::c_float; 3],
}
pub type gyro_t = gyro_s;
pub type C2RustUnnamed_6 = libc::c_uint;
pub const GYRO_OVERFLOW_CHECK_ALL_AXES: C2RustUnnamed_6 = 2;
pub const GYRO_OVERFLOW_CHECK_YAW: C2RustUnnamed_6 = 1;
pub const GYRO_OVERFLOW_CHECK_NONE: C2RustUnnamed_6 = 0;
pub type C2RustUnnamed_7 = libc::c_uint;
pub const FILTER_LOWPASS2: C2RustUnnamed_7 = 1;
pub const FILTER_LOWPASS: C2RustUnnamed_7 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gyroConfig_s {
    pub gyro_align: uint8_t,
    pub gyroMovementCalibrationThreshold: uint8_t,
    pub gyro_sync_denom: uint8_t,
    pub gyro_hardware_lpf: uint8_t,
    pub gyro_32khz_hardware_lpf: uint8_t,
    pub gyro_high_fsr: uint8_t,
    pub gyro_use_32khz: uint8_t,
    pub gyro_to_use: uint8_t,
    pub gyro_lowpass_hz: uint16_t,
    pub gyro_lowpass2_hz: uint16_t,
    pub gyro_soft_notch_hz_1: uint16_t,
    pub gyro_soft_notch_cutoff_1: uint16_t,
    pub gyro_soft_notch_hz_2: uint16_t,
    pub gyro_soft_notch_cutoff_2: uint16_t,
    pub gyro_offset_yaw: int16_t,
    pub checkOverflow: uint8_t,
    pub gyro_lowpass_type: uint8_t,
    pub gyro_lowpass2_type: uint8_t,
    pub yaw_spin_recovery: uint8_t,
    pub yaw_spin_threshold: int16_t,
    pub gyroCalibrationDuration: uint16_t,
    pub dyn_notch_quality: uint8_t,
    pub dyn_notch_width_percent: uint8_t,
}
pub type gyroConfig_t = gyroConfig_s;
pub type gyroDev_t = gyroDev_s;
pub type gyroSensor_t = gyroSensor_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gyroSensor_s {
    pub gyroDev: gyroDev_t,
    pub calibration: gyroCalibration_t,
    pub lowpassFilterApplyFn: filterApplyFnPtr,
    pub lowpassFilter: [gyroLowpassFilter_t; 3],
    pub lowpass2FilterApplyFn: filterApplyFnPtr,
    pub lowpass2Filter: [gyroLowpassFilter_t; 3],
    pub notchFilter1ApplyFn: filterApplyFnPtr,
    pub notchFilter1: [biquadFilter_t; 3],
    pub notchFilter2ApplyFn: filterApplyFnPtr,
    pub notchFilter2: [biquadFilter_t; 3],
    pub notchFilterDynApplyFn: filterApplyFnPtr,
    pub notchFilterDyn: [biquadFilter_t; 3],
    pub overflowTimeUs: timeUs_t,
    pub overflowDetected: bool,
    pub yawSpinTimeUs: timeUs_t,
    pub yawSpinDetected: bool,
    pub gyroAnalyseState: gyroAnalyseState_t,
}
pub type gyroAnalyseState_t = gyroAnalyseState_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gyroAnalyseState_s {
    pub sampleCount: uint8_t,
    pub maxSampleCount: uint8_t,
    pub maxSampleCountRcp: libc::c_float,
    pub oversampledGyroAccumulator: [libc::c_float; 3],
    pub gyroBandpassFilter: [biquadFilter_t; 3],
    pub circularBufferIdx: uint8_t,
    pub downsampledGyroData: [[libc::c_float; 32]; 3],
    pub updateTicks: uint8_t,
    pub updateStep: uint8_t,
    pub updateAxis: uint8_t,
    pub fftInstance: arm_rfft_fast_instance_f32,
    pub fftData: [libc::c_float; 32],
    pub rfftData: [libc::c_float; 32],
    pub detectedFrequencyFilter: [biquadFilter_t; 3],
    pub centerFreq: [uint16_t; 3],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct arm_rfft_fast_instance_f32 {
    pub Sint: arm_cfft_instance_f32,
    pub fftLenRFFT: uint16_t,
    pub pTwiddleRFFT: *mut float32_t,
}
// lowpass gyro soft filter
// lowpass2 gyro soft filter
// notch filters
// overflow and recovery
// USE_YAW_SPIN_RECOVERY
/* *
   * @brief 32-bit floating-point type definition.
   */
pub type float32_t = libc::c_float;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct arm_cfft_instance_f32 {
    pub fftLen: uint16_t,
    pub pTwiddle: *const float32_t,
    pub pBitRevTable: *const uint16_t,
    pub bitRevLength: uint16_t,
}
pub type gyroLowpassFilter_t = gyroLowpassFilter_u;
#[derive(Copy, Clone)]
#[repr(C)]
pub union gyroLowpassFilter_u {
    pub pt1FilterState: pt1Filter_t,
    pub biquadFilterState: biquadFilter_t,
}
pub type gyroCalibration_t = gyroCalibration_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gyroCalibration_s {
    pub sum: [libc::c_float; 3],
    pub var: [stdev_t; 3],
    pub cyclesRemaining: int32_t,
}
pub const SENSOR_GYRO: C2RustUnnamed_9 = 1;
pub const SENSOR_INDEX_GYRO: C2RustUnnamed_8 = 0;
pub type beeperMode_e = libc::c_uint;
pub const BEEPER_ALL: beeperMode_e = 24;
pub const BEEPER_RC_SMOOTHING_INIT_FAIL: beeperMode_e = 23;
pub const BEEPER_CAM_CONNECTION_CLOSE: beeperMode_e = 22;
pub const BEEPER_CAM_CONNECTION_OPEN: beeperMode_e = 21;
pub const BEEPER_CRASH_FLIP_MODE: beeperMode_e = 20;
pub const BEEPER_BLACKBOX_ERASE: beeperMode_e = 19;
pub const BEEPER_USB: beeperMode_e = 18;
pub const BEEPER_SYSTEM_INIT: beeperMode_e = 17;
pub const BEEPER_ARMED: beeperMode_e = 16;
pub const BEEPER_DISARM_REPEAT: beeperMode_e = 15;
pub const BEEPER_MULTI_BEEPS: beeperMode_e = 14;
pub const BEEPER_READY_BEEP: beeperMode_e = 13;
pub const BEEPER_ACC_CALIBRATION_FAIL: beeperMode_e = 12;
pub const BEEPER_ACC_CALIBRATION: beeperMode_e = 11;
pub const BEEPER_RX_SET: beeperMode_e = 10;
pub const BEEPER_GPS_STATUS: beeperMode_e = 9;
pub const BEEPER_BAT_LOW: beeperMode_e = 8;
pub const BEEPER_BAT_CRIT_LOW: beeperMode_e = 7;
pub const BEEPER_ARMING_GPS_FIX: beeperMode_e = 6;
pub const BEEPER_ARMING: beeperMode_e = 5;
pub const BEEPER_DISARMING: beeperMode_e = 4;
pub const BEEPER_RX_LOST_LANDING: beeperMode_e = 3;
pub const BEEPER_RX_LOST: beeperMode_e = 2;
pub const BEEPER_GYRO_CALIBRATED: beeperMode_e = 1;
pub const BEEPER_SILENCE: beeperMode_e = 0;
pub const ARMING_DISABLED_CALIBRATING: armingDisableFlags_e = 2048;
pub type armingDisableFlags_e = libc::c_uint;
pub const ARMING_DISABLED_ARM_SWITCH: armingDisableFlags_e = 524288;
pub const ARMING_DISABLED_GPS: armingDisableFlags_e = 262144;
pub const ARMING_DISABLED_PARALYZE: armingDisableFlags_e = 131072;
pub const ARMING_DISABLED_MSP: armingDisableFlags_e = 65536;
pub const ARMING_DISABLED_BST: armingDisableFlags_e = 32768;
pub const ARMING_DISABLED_OSD_MENU: armingDisableFlags_e = 16384;
pub const ARMING_DISABLED_CMS_MENU: armingDisableFlags_e = 8192;
pub const ARMING_DISABLED_CLI: armingDisableFlags_e = 4096;
pub const ARMING_DISABLED_LOAD: armingDisableFlags_e = 1024;
pub const ARMING_DISABLED_NOPREARM: armingDisableFlags_e = 512;
pub const ARMING_DISABLED_BOOT_GRACE_TIME: armingDisableFlags_e = 256;
pub const ARMING_DISABLED_ANGLE: armingDisableFlags_e = 128;
pub const ARMING_DISABLED_THROTTLE: armingDisableFlags_e = 64;
pub const ARMING_DISABLED_RUNAWAY_TAKEOFF: armingDisableFlags_e = 32;
pub const ARMING_DISABLED_BOXFAILSAFE: armingDisableFlags_e = 16;
pub const ARMING_DISABLED_BAD_RX_RECOVERY: armingDisableFlags_e = 8;
pub const ARMING_DISABLED_RX_FAILSAFE: armingDisableFlags_e = 4;
pub const ARMING_DISABLED_FAILSAFE: armingDisableFlags_e = 2;
pub const ARMING_DISABLED_NO_GYRO: armingDisableFlags_e = 1;
pub type cfTaskId_e = libc::c_uint;
pub const TASK_SELF: cfTaskId_e = 28;
pub const TASK_NONE: cfTaskId_e = 27;
pub const TASK_COUNT: cfTaskId_e = 27;
pub const TASK_PINIOBOX: cfTaskId_e = 26;
pub const TASK_ADC_INTERNAL: cfTaskId_e = 25;
pub const TASK_RCDEVICE: cfTaskId_e = 24;
pub const TASK_CAMCTRL: cfTaskId_e = 23;
pub const TASK_VTXCTRL: cfTaskId_e = 22;
pub const TASK_CMS: cfTaskId_e = 21;
pub const TASK_ESC_SENSOR: cfTaskId_e = 20;
pub const TASK_OSD: cfTaskId_e = 19;
pub const TASK_LEDSTRIP: cfTaskId_e = 18;
pub const TASK_TELEMETRY: cfTaskId_e = 17;
pub const TASK_DASHBOARD: cfTaskId_e = 16;
pub const TASK_ALTITUDE: cfTaskId_e = 15;
pub const TASK_BARO: cfTaskId_e = 14;
pub const TASK_COMPASS: cfTaskId_e = 13;
pub const TASK_GPS: cfTaskId_e = 12;
pub const TASK_BEEPER: cfTaskId_e = 11;
pub const TASK_BATTERY_ALERTS: cfTaskId_e = 10;
pub const TASK_BATTERY_CURRENT: cfTaskId_e = 9;
pub const TASK_BATTERY_VOLTAGE: cfTaskId_e = 8;
pub const TASK_DISPATCH: cfTaskId_e = 7;
pub const TASK_SERIAL: cfTaskId_e = 6;
pub const TASK_RX: cfTaskId_e = 5;
pub const TASK_ATTITUDE: cfTaskId_e = 4;
pub const TASK_ACCEL: cfTaskId_e = 3;
pub const TASK_GYROPID: cfTaskId_e = 2;
pub const TASK_MAIN: cfTaskId_e = 1;
pub const TASK_SYSTEM: cfTaskId_e = 0;
pub type C2RustUnnamed_8 = libc::c_uint;
pub const SENSOR_INDEX_COUNT: C2RustUnnamed_8 = 5;
pub const SENSOR_INDEX_RANGEFINDER: C2RustUnnamed_8 = 4;
pub const SENSOR_INDEX_MAG: C2RustUnnamed_8 = 3;
pub const SENSOR_INDEX_BARO: C2RustUnnamed_8 = 2;
pub const SENSOR_INDEX_ACC: C2RustUnnamed_8 = 1;
pub type C2RustUnnamed_9 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_9 = 64;
pub const SENSOR_GPS: C2RustUnnamed_9 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_9 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_9 = 16;
pub const SENSOR_MAG: C2RustUnnamed_9 = 8;
pub const SENSOR_BARO: C2RustUnnamed_9 = 4;
pub const SENSOR_ACC: C2RustUnnamed_9 = 2;
#[inline]
unsafe extern "C" fn cmpTimeUs(mut a: timeUs_t, mut b: timeUs_t)
 -> timeDelta_t {
    return a.wrapping_sub(b) as timeDelta_t;
}
#[inline]
unsafe extern "C" fn gyroConfig() -> *const gyroConfig_t {
    return &mut gyroConfig_System;
}
#[inline]
unsafe extern "C" fn gyroConfigMutable() -> *mut gyroConfig_t {
    return &mut gyroConfig_System;
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
#[no_mangle]
#[link_section = ".fastram_bss"]
pub static mut gyro: gyro_t = gyro_t{targetLooptime: 0, gyroADCf: [0.; 3],};
#[link_section = ".fastram_bss"]
static mut gyroDebugMode: uint8_t = 0;
static mut gyroToUse: uint8_t = 0 as libc::c_int as uint8_t;
#[link_section = ".fastram_bss"]
static mut overflowAxisMask: uint8_t = 0;
#[link_section = ".fastram_bss"]
static mut accumulatedMeasurements: [libc::c_float; 3] = [0.; 3];
#[link_section = ".fastram_bss"]
static mut gyroPrevious: [libc::c_float; 3] = [0.; 3];
#[link_section = ".fastram_bss"]
static mut accumulatedMeasurementTimeUs: timeUs_t = 0;
#[link_section = ".fastram_bss"]
static mut accumulationLastTimeSampledUs: timeUs_t = 0;
static mut gyroHasOverflowProtection: bool = 1 as libc::c_int != 0;
#[no_mangle]
pub static mut firstArmingCalibrationWasStarted: bool = 0 as libc::c_int != 0;
#[link_section = ".fastram_bss"]
static mut gyroSensor1: gyroSensor_t =
    gyroSensor_t{gyroDev:
                     gyroDev_t{initFn: None,
                               readFn: None,
                               temperatureFn: None,
                               exti: extiCallbackRec_t{fn_0: None,},
                               bus:
                                   busDevice_t{bustype: BUSTYPE_NONE,
                                               busdev_u:
                                                   C2RustUnnamed_5{spi:
                                                                       deviceSpi_s{instance:
                                                                                       0
                                                                                           as
                                                                                           *const SPI_TypeDef
                                                                                           as
                                                                                           *mut SPI_TypeDef,
                                                                                   handle:
                                                                                       0
                                                                                           as
                                                                                           *const SPI_HandleTypeDef
                                                                                           as
                                                                                           *mut SPI_HandleTypeDef,
                                                                                   csnPin:
                                                                                       0
                                                                                           as
                                                                                           *const libc::c_void
                                                                                           as
                                                                                           *mut libc::c_void,},},},
                               scale: 0.,
                               gyroZero: [0.; 3],
                               gyroADC: [0.; 3],
                               gyroADCf: [0.; 3],
                               gyroADCRawPrevious: [0; 3],
                               gyroADCRaw: [0; 3],
                               temperature: 0,
                               mpuConfiguration:
                                   mpuConfiguration_t{resetFn: None,},
                               mpuDetectionResult:
                                   mpuDetectionResult_t{sensor: MPU_NONE,
                                                        resolution:
                                                            MPU_HALF_RESOLUTION,},
                               gyroAlign: ALIGN_DEFAULT,
                               gyroRateKHz: GYRO_RATE_1_kHz,
                               dataReady: false,
                               gyro_high_fsr: false,
                               hardware_lpf: 0,
                               hardware_32khz_lpf: 0,
                               mpuDividerDrops: 0,
                               mpuIntExtiTag: 0,
                               gyroHasOverflowProtection: 0,
                               gyroHardware: GYRO_NONE,},
                 calibration:
                     gyroCalibration_t{sum: [0.; 3],
                                       var:
                                           [stdev_t{m_oldM: 0.,
                                                    m_newM: 0.,
                                                    m_oldS: 0.,
                                                    m_newS: 0.,
                                                    m_n: 0,}; 3],
                                       cyclesRemaining: 0,},
                 lowpassFilterApplyFn: None,
                 lowpassFilter:
                     [gyroLowpassFilter_u{pt1FilterState:
                                              pt1Filter_t{state: 0.,
                                                          k: 0.,},}; 3],
                 lowpass2FilterApplyFn: None,
                 lowpass2Filter:
                     [gyroLowpassFilter_u{pt1FilterState:
                                              pt1Filter_t{state: 0.,
                                                          k: 0.,},}; 3],
                 notchFilter1ApplyFn: None,
                 notchFilter1:
                     [biquadFilter_t{b0: 0.,
                                     b1: 0.,
                                     b2: 0.,
                                     a1: 0.,
                                     a2: 0.,
                                     x1: 0.,
                                     x2: 0.,
                                     y1: 0.,
                                     y2: 0.,}; 3],
                 notchFilter2ApplyFn: None,
                 notchFilter2:
                     [biquadFilter_t{b0: 0.,
                                     b1: 0.,
                                     b2: 0.,
                                     a1: 0.,
                                     a2: 0.,
                                     x1: 0.,
                                     x2: 0.,
                                     y1: 0.,
                                     y2: 0.,}; 3],
                 notchFilterDynApplyFn: None,
                 notchFilterDyn:
                     [biquadFilter_t{b0: 0.,
                                     b1: 0.,
                                     b2: 0.,
                                     a1: 0.,
                                     a2: 0.,
                                     x1: 0.,
                                     x2: 0.,
                                     y1: 0.,
                                     y2: 0.,}; 3],
                 overflowTimeUs: 0,
                 overflowDetected: false,
                 yawSpinTimeUs: 0,
                 yawSpinDetected: false,
                 gyroAnalyseState:
                     gyroAnalyseState_t{sampleCount: 0,
                                        maxSampleCount: 0,
                                        maxSampleCountRcp: 0.,
                                        oversampledGyroAccumulator: [0.; 3],
                                        gyroBandpassFilter:
                                            [biquadFilter_t{b0: 0.,
                                                            b1: 0.,
                                                            b2: 0.,
                                                            a1: 0.,
                                                            a2: 0.,
                                                            x1: 0.,
                                                            x2: 0.,
                                                            y1: 0.,
                                                            y2: 0.,}; 3],
                                        circularBufferIdx: 0,
                                        downsampledGyroData: [[0.; 32]; 3],
                                        updateTicks: 0,
                                        updateStep: 0,
                                        updateAxis: 0,
                                        fftInstance:
                                            arm_rfft_fast_instance_f32{Sint:
                                                                           arm_cfft_instance_f32{fftLen:
                                                                                                     0,
                                                                                                 pTwiddle:
                                                                                                     0
                                                                                                         as
                                                                                                         *const float32_t,
                                                                                                 pBitRevTable:
                                                                                                     0
                                                                                                         as
                                                                                                         *const uint16_t,
                                                                                                 bitRevLength:
                                                                                                     0,},
                                                                       fftLenRFFT:
                                                                           0,
                                                                       pTwiddleRFFT:
                                                                           0
                                                                               as
                                                                               *const float32_t
                                                                               as
                                                                               *mut float32_t,},
                                        fftData: [0.; 32],
                                        rfftData: [0.; 32],
                                        detectedFrequencyFilter:
                                            [biquadFilter_t{b0: 0.,
                                                            b1: 0.,
                                                            b2: 0.,
                                                            a1: 0.,
                                                            a2: 0.,
                                                            x1: 0.,
                                                            x2: 0.,
                                                            y1: 0.,
                                                            y2: 0.,}; 3],
                                        centerFreq: [0; 3],},};
// 92.5% full scale (1850dps for 2000dps gyro)
#[no_mangle]
pub static mut gyroConfig_System: gyroConfig_t =
    gyroConfig_t{gyro_align: 0,
                 gyroMovementCalibrationThreshold: 0,
                 gyro_sync_denom: 0,
                 gyro_hardware_lpf: 0,
                 gyro_32khz_hardware_lpf: 0,
                 gyro_high_fsr: 0,
                 gyro_use_32khz: 0,
                 gyro_to_use: 0,
                 gyro_lowpass_hz: 0,
                 gyro_lowpass2_hz: 0,
                 gyro_soft_notch_hz_1: 0,
                 gyro_soft_notch_cutoff_1: 0,
                 gyro_soft_notch_hz_2: 0,
                 gyro_soft_notch_cutoff_2: 0,
                 gyro_offset_yaw: 0,
                 checkOverflow: 0,
                 gyro_lowpass_type: 0,
                 gyro_lowpass2_type: 0,
                 yaw_spin_recovery: 0,
                 yaw_spin_threshold: 0,
                 gyroCalibrationDuration: 0,
                 dyn_notch_quality: 0,
                 dyn_notch_width_percent: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut gyroConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (10 as libc::c_int |
                                      (4 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<gyroConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &gyroConfig_System as *const gyroConfig_t as
                                     *mut gyroConfig_t as *mut uint8_t,
                             copy:
                                 &gyroConfig_Copy as *const gyroConfig_t as
                                     *mut gyroConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_3{ptr:
                                                     &pgResetTemplate_gyroConfig
                                                         as
                                                         *const gyroConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut gyroConfig_Copy: gyroConfig_t =
    gyroConfig_t{gyro_align: 0,
                 gyroMovementCalibrationThreshold: 0,
                 gyro_sync_denom: 0,
                 gyro_hardware_lpf: 0,
                 gyro_32khz_hardware_lpf: 0,
                 gyro_high_fsr: 0,
                 gyro_use_32khz: 0,
                 gyro_to_use: 0,
                 gyro_lowpass_hz: 0,
                 gyro_lowpass2_hz: 0,
                 gyro_soft_notch_hz_1: 0,
                 gyro_soft_notch_cutoff_1: 0,
                 gyro_soft_notch_hz_2: 0,
                 gyro_soft_notch_cutoff_2: 0,
                 gyro_offset_yaw: 0,
                 checkOverflow: 0,
                 gyro_lowpass_type: 0,
                 gyro_lowpass2_type: 0,
                 yaw_spin_recovery: 0,
                 yaw_spin_threshold: 0,
                 gyroCalibrationDuration: 0,
                 dyn_notch_quality: 0,
                 dyn_notch_width_percent: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_gyroConfig: gyroConfig_t =
    {
        let mut init =
            gyroConfig_s{gyro_align: ALIGN_DEFAULT as libc::c_int as uint8_t,
                         gyroMovementCalibrationThreshold:
                             48 as libc::c_int as uint8_t,
                         gyro_sync_denom: 1 as libc::c_int as uint8_t,
                         gyro_hardware_lpf: 0 as libc::c_int as uint8_t,
                         gyro_32khz_hardware_lpf: 0 as libc::c_int as uint8_t,
                         gyro_high_fsr: 0 as libc::c_int as uint8_t,
                         gyro_use_32khz: 0 as libc::c_int as uint8_t,
                         gyro_to_use: 0 as libc::c_int as uint8_t,
                         gyro_lowpass_hz: 100 as libc::c_int as uint16_t,
                         gyro_lowpass2_hz: 300 as libc::c_int as uint16_t,
                         gyro_soft_notch_hz_1: 0 as libc::c_int as uint16_t,
                         gyro_soft_notch_cutoff_1:
                             0 as libc::c_int as uint16_t,
                         gyro_soft_notch_hz_2: 0 as libc::c_int as uint16_t,
                         gyro_soft_notch_cutoff_2:
                             0 as libc::c_int as uint16_t,
                         gyro_offset_yaw: 0 as libc::c_int as int16_t,
                         checkOverflow:
                             GYRO_OVERFLOW_CHECK_ALL_AXES as libc::c_int as
                                 uint8_t,
                         gyro_lowpass_type:
                             FILTER_PT1 as libc::c_int as uint8_t,
                         gyro_lowpass2_type:
                             FILTER_PT1 as libc::c_int as uint8_t,
                         yaw_spin_recovery: 1 as libc::c_int as uint8_t,
                         yaw_spin_threshold: 1950 as libc::c_int as int16_t,
                         gyroCalibrationDuration:
                             125 as libc::c_int as uint16_t,
                         dyn_notch_quality: 70 as libc::c_int as uint8_t,
                         dyn_notch_width_percent:
                             50 as libc::c_int as uint8_t,};
        init
    };
#[no_mangle]
pub unsafe extern "C" fn gyroSensorBus() -> *const busDevice_t {
    return &mut gyroSensor1.gyroDev.bus;
}
#[no_mangle]
pub unsafe extern "C" fn gyroSensorBusByDevice(mut whichSensor: uint8_t)
 -> *const busDevice_t {
    return &mut gyroSensor1.gyroDev.bus;
}
// USE_GYRO_REGISTER_DUMP
#[no_mangle]
pub unsafe extern "C" fn gyroMpuConfiguration() -> *const mpuConfiguration_s {
    return &mut gyroSensor1.gyroDev.mpuConfiguration; // must be set after mpuDetect
}
#[no_mangle]
pub unsafe extern "C" fn gyroMpuDetectionResult()
 -> *const mpuDetectionResult_s {
    return &mut gyroSensor1.gyroDev.mpuDetectionResult;
}
unsafe extern "C" fn gyroDetect(mut dev: *mut gyroDev_t) -> gyroSensor_e {
    let mut gyroHardware: gyroSensor_e = GYRO_DEFAULT;
    let mut current_block_6: u64;
    match gyroHardware as libc::c_uint {
        1 | 13 => {
            if icm20689SpiGyroDetect(dev) {
                gyroHardware = GYRO_ICM20689;
                (*dev).gyroAlign = CW270_DEG;
                current_block_6 = 2968425633554183086;
            } else { current_block_6 = 3114499540105030204; }
        }
        _ => { current_block_6 = 3114499540105030204; }
    }
    match current_block_6 {
        3114499540105030204 => { gyroHardware = GYRO_NONE }
        _ => { }
    }
    if gyroHardware as libc::c_uint !=
           GYRO_NONE as libc::c_int as libc::c_uint {
        detectedSensors[SENSOR_INDEX_GYRO as libc::c_int as usize] =
            gyroHardware as uint8_t;
        sensorsSet(SENSOR_GYRO as libc::c_int as uint32_t);
    }
    return gyroHardware;
}
unsafe extern "C" fn gyroInitSensor(mut gyroSensor: *mut gyroSensor_t)
 -> bool {
    (*gyroSensor).gyroDev.gyro_high_fsr = (*gyroConfig()).gyro_high_fsr != 0;
    mpuDetect(&mut (*gyroSensor).gyroDev);
    mpuResetFn = (*gyroSensor).gyroDev.mpuConfiguration.resetFn;
    let gyroHardware: gyroSensor_e = gyroDetect(&mut (*gyroSensor).gyroDev);
    (*gyroSensor).gyroDev.gyroHardware = gyroHardware;
    if gyroHardware as libc::c_uint ==
           GYRO_NONE as libc::c_int as libc::c_uint {
        return 0 as libc::c_int != 0
    }
    match gyroHardware as libc::c_uint {
        7 | 8 | 9 | 10 | 11 | 13 => { }
        _ => {
            // gyro does not support 32kHz
            (*gyroConfigMutable()).gyro_use_32khz =
                0 as libc::c_int as uint8_t
        }
    }
    // Must set gyro targetLooptime before gyroDev.init and initialisation of filters
    gyro.targetLooptime =
        gyroSetSampleRate(&mut (*gyroSensor).gyroDev,
                          (*gyroConfig()).gyro_hardware_lpf,
                          (*gyroConfig()).gyro_sync_denom,
                          (*gyroConfig()).gyro_use_32khz != 0);
    (*gyroSensor).gyroDev.hardware_lpf = (*gyroConfig()).gyro_hardware_lpf;
    (*gyroSensor).gyroDev.hardware_32khz_lpf =
        (*gyroConfig()).gyro_32khz_hardware_lpf;
    (*gyroSensor).gyroDev.initFn.expect("non-null function pointer")(&mut (*gyroSensor).gyroDev);
    if (*gyroConfig()).gyro_align as libc::c_int !=
           ALIGN_DEFAULT as libc::c_int {
        (*gyroSensor).gyroDev.gyroAlign =
            (*gyroConfig()).gyro_align as sensor_align_e
    }
    // As new gyros are supported, be sure to add them below based on whether they are subject to the overflow/inversion bug
    // Any gyro not explicitly defined will default to not having built-in overflow protection as a safe alternative.
    match gyroHardware as libc::c_uint {
        0 | 1 | 15 | 2 | 3 | 4 | 5 | 14 | 6 | 7 | 8 => {
            // Won't ever actually get here, but included to account for all gyro types
            (*gyroSensor).gyroDev.gyroHasOverflowProtection =
                1 as libc::c_int as uint8_t
        }
        9 | 10 | 11 | 12 | 13 => {
            // we don't actually know if this is affected, but as there are currently no flight controllers using it we err on the side of caution
            (*gyroSensor).gyroDev.gyroHasOverflowProtection =
                0 as libc::c_int as uint8_t
        }
        _ => {
            (*gyroSensor).gyroDev.gyroHasOverflowProtection =
                0 as libc::c_int as uint8_t
        }
    } // default catch for newly added gyros until proven to be unaffected
    gyroInitSensorFilters(gyroSensor);
    gyroDataAnalyseStateInit(&mut (*gyroSensor).gyroAnalyseState,
                             gyro.targetLooptime);
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn gyroInit() -> bool {
    if (*gyroConfig()).checkOverflow as libc::c_int ==
           GYRO_OVERFLOW_CHECK_YAW as libc::c_int {
        overflowAxisMask = GYRO_OVERFLOW_Z as libc::c_int as uint8_t
    } else if (*gyroConfig()).checkOverflow as libc::c_int ==
                  GYRO_OVERFLOW_CHECK_ALL_AXES as libc::c_int {
        overflowAxisMask =
            (GYRO_OVERFLOW_X as libc::c_int | GYRO_OVERFLOW_Y as libc::c_int |
                 GYRO_OVERFLOW_Z as libc::c_int) as uint8_t
    } else { overflowAxisMask = 0 as libc::c_int as uint8_t }
    match debugMode as libc::c_int {
        15 | 17 | 19 | 6 | 3 => { gyroDebugMode = debugMode }
        _ => {
            // debugMode is not gyro-related
            gyroDebugMode = DEBUG_NONE as libc::c_int as uint8_t
        }
    }
    firstArmingCalibrationWasStarted = 0 as libc::c_int != 0;
    let mut ret: bool = 0 as libc::c_int != 0;
    gyroToUse = (*gyroConfig()).gyro_to_use;
    gyroSensor1.gyroDev.gyroAlign = ALIGN_DEFAULT;
    gyroSensor1.gyroDev.mpuIntExtiTag =
        ((4 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
             1 as libc::c_int) as ioTag_t;
    // GYRO_1_EXTI_PIN
    // USE_DUAL_GYRO
    ret = gyroInitSensor(&mut gyroSensor1);
    gyroHasOverflowProtection =
        gyroHasOverflowProtection as libc::c_int != 0 &&
            gyroSensor1.gyroDev.gyroHasOverflowProtection as libc::c_int != 0;
    // USE_DUAL_GYRO
    // USE_DUAL_GYRO
    // USE_DUAL_GYRO
    return ret;
}
unsafe extern "C" fn gyroInitLowpassFilterLpf(mut gyroSensor:
                                                  *mut gyroSensor_t,
                                              mut slot: libc::c_int,
                                              mut type_0: libc::c_int,
                                              mut lpfHz: uint16_t) {
    let mut lowpassFilterApplyFn: *mut filterApplyFnPtr =
        0 as *mut filterApplyFnPtr;
    let mut lowpassFilter: *mut gyroLowpassFilter_t =
        0 as *mut gyroLowpassFilter_t;
    match slot {
        0 => {
            lowpassFilterApplyFn = &mut (*gyroSensor).lowpassFilterApplyFn;
            lowpassFilter = (*gyroSensor).lowpassFilter.as_mut_ptr()
        }
        1 => {
            lowpassFilterApplyFn = &mut (*gyroSensor).lowpass2FilterApplyFn;
            lowpassFilter = (*gyroSensor).lowpass2Filter.as_mut_ptr()
        }
        _ => { return }
    }
    // Establish some common constants
    let gyroFrequencyNyquist: uint32_t =
        ((1000000 as libc::c_int / 2 as libc::c_int) as
             libc::c_uint).wrapping_div(gyro.targetLooptime);
    let gyroDt: libc::c_float =
        gyro.targetLooptime as libc::c_float * 1e-6f32;
    // Gain could be calculated a little later as it is specific to the pt1/bqrcf2/fkf branches
    let gain: libc::c_float = pt1FilterGain(lpfHz, gyroDt);
    // Dereference the pointer to null before checking valid cutoff and filter
    // type. It will be overridden for positive cases.
    *lowpassFilterApplyFn =
        Some(nullFilterApply as
                 unsafe extern "C" fn(_: *mut filter_t, _: libc::c_float)
                     -> libc::c_float);
    // If lowpass cutoff has been specified and is less than the Nyquist frequency
    if lpfHz as libc::c_int != 0 &&
           lpfHz as libc::c_uint <= gyroFrequencyNyquist {
        match type_0 {
            0 => {
                *lowpassFilterApplyFn =
                    ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                             *mut pt1Filter_t,
                                                                         _:
                                                                             libc::c_float)
                                                        -> libc::c_float>,
                                             filterApplyFnPtr>(Some(pt1FilterApply
                                                                        as
                                                                        unsafe extern "C" fn(_:
                                                                                                 *mut pt1Filter_t,
                                                                                             _:
                                                                                                 libc::c_float)
                                                                            ->
                                                                                libc::c_float)); // must be this function, not DF2
                let mut axis: libc::c_int =
                    0 as libc::c_int; //just any init value
                while axis < 3 as libc::c_int {
                    pt1FilterInit(&mut (*lowpassFilter.offset(axis as
                                                                  isize)).pt1FilterState,
                                  gain);
                    axis += 1
                }
            }
            1 => {
                *lowpassFilterApplyFn =
                    ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                             *mut biquadFilter_t,
                                                                         _:
                                                                             libc::c_float)
                                                        -> libc::c_float>,
                                             filterApplyFnPtr>(Some(biquadFilterApply
                                                                        as
                                                                        unsafe extern "C" fn(_:
                                                                                                 *mut biquadFilter_t,
                                                                                             _:
                                                                                                 libc::c_float)
                                                                            ->
                                                                                libc::c_float));
                let mut axis_0: libc::c_int = 0 as libc::c_int;
                while axis_0 < 3 as libc::c_int {
                    biquadFilterInitLPF(&mut (*lowpassFilter.offset(axis_0 as
                                                                        isize)).biquadFilterState,
                                        lpfHz as libc::c_float,
                                        gyro.targetLooptime);
                    axis_0 += 1
                }
            }
            _ => { }
        }
    };
}
unsafe extern "C" fn calculateNyquistAdjustedNotchHz(mut notchHz: uint16_t,
                                                     mut notchCutoffHz:
                                                         uint16_t)
 -> uint16_t {
    let gyroFrequencyNyquist: uint32_t =
        ((1000000 as libc::c_int / 2 as libc::c_int) as
             libc::c_uint).wrapping_div(gyro.targetLooptime);
    if notchHz as libc::c_uint > gyroFrequencyNyquist {
        if (notchCutoffHz as libc::c_uint) < gyroFrequencyNyquist {
            notchHz = gyroFrequencyNyquist as uint16_t
        } else { notchHz = 0 as libc::c_int as uint16_t }
    }
    return notchHz;
}
#[no_mangle]
pub unsafe extern "C" fn gyroInitSlewLimiter(mut gyroSensor:
                                                 *mut gyroSensor_t) {
    let mut axis: libc::c_int = 0 as libc::c_int;
    while axis < 3 as libc::c_int {
        (*gyroSensor).gyroDev.gyroADCRawPrevious[axis as usize] =
            0 as libc::c_int;
        axis += 1
    };
}
unsafe extern "C" fn gyroInitFilterNotch1(mut gyroSensor: *mut gyroSensor_t,
                                          mut notchHz: uint16_t,
                                          mut notchCutoffHz: uint16_t) {
    (*gyroSensor).notchFilter1ApplyFn =
        Some(nullFilterApply as
                 unsafe extern "C" fn(_: *mut filter_t, _: libc::c_float)
                     -> libc::c_float);
    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);
    if notchHz as libc::c_int != 0 as libc::c_int &&
           notchCutoffHz as libc::c_int != 0 as libc::c_int {
        (*gyroSensor).notchFilter1ApplyFn =
            ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                     *mut biquadFilter_t,
                                                                 _:
                                                                     libc::c_float)
                                                -> libc::c_float>,
                                     filterApplyFnPtr>(Some(biquadFilterApply
                                                                as
                                                                unsafe extern "C" fn(_:
                                                                                         *mut biquadFilter_t,
                                                                                     _:
                                                                                         libc::c_float)
                                                                    ->
                                                                        libc::c_float));
        let notchQ: libc::c_float =
            filterGetNotchQ(notchHz as libc::c_float,
                            notchCutoffHz as libc::c_float);
        let mut axis: libc::c_int = 0 as libc::c_int;
        while axis < 3 as libc::c_int {
            biquadFilterInit(&mut *(*gyroSensor).notchFilter1.as_mut_ptr().offset(axis
                                                                                      as
                                                                                      isize),
                             notchHz as libc::c_float, gyro.targetLooptime,
                             notchQ, FILTER_NOTCH);
            axis += 1
        }
    };
}
unsafe extern "C" fn gyroInitFilterNotch2(mut gyroSensor: *mut gyroSensor_t,
                                          mut notchHz: uint16_t,
                                          mut notchCutoffHz: uint16_t) {
    (*gyroSensor).notchFilter2ApplyFn =
        Some(nullFilterApply as
                 unsafe extern "C" fn(_: *mut filter_t, _: libc::c_float)
                     -> libc::c_float);
    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);
    if notchHz as libc::c_int != 0 as libc::c_int &&
           notchCutoffHz as libc::c_int != 0 as libc::c_int {
        (*gyroSensor).notchFilter2ApplyFn =
            ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                     *mut biquadFilter_t,
                                                                 _:
                                                                     libc::c_float)
                                                -> libc::c_float>,
                                     filterApplyFnPtr>(Some(biquadFilterApply
                                                                as
                                                                unsafe extern "C" fn(_:
                                                                                         *mut biquadFilter_t,
                                                                                     _:
                                                                                         libc::c_float)
                                                                    ->
                                                                        libc::c_float));
        let notchQ: libc::c_float =
            filterGetNotchQ(notchHz as libc::c_float,
                            notchCutoffHz as libc::c_float);
        let mut axis: libc::c_int = 0 as libc::c_int;
        while axis < 3 as libc::c_int {
            biquadFilterInit(&mut *(*gyroSensor).notchFilter2.as_mut_ptr().offset(axis
                                                                                      as
                                                                                      isize),
                             notchHz as libc::c_float, gyro.targetLooptime,
                             notchQ, FILTER_NOTCH);
            axis += 1
        }
    };
}
unsafe extern "C" fn isDynamicFilterActive() -> bool {
    return feature(FEATURE_DYNAMIC_FILTER as libc::c_int as uint32_t);
}
unsafe extern "C" fn gyroInitFilterDynamicNotch(mut gyroSensor:
                                                    *mut gyroSensor_t) {
    (*gyroSensor).notchFilterDynApplyFn =
        Some(nullFilterApply as
                 unsafe extern "C" fn(_: *mut filter_t, _: libc::c_float)
                     -> libc::c_float);
    if isDynamicFilterActive() {
        (*gyroSensor).notchFilterDynApplyFn =
            ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                     *mut biquadFilter_t,
                                                                 _:
                                                                     libc::c_float)
                                                -> libc::c_float>,
                                     filterApplyFnPtr>(Some(biquadFilterApplyDF1
                                                                as
                                                                unsafe extern "C" fn(_:
                                                                                         *mut biquadFilter_t,
                                                                                     _:
                                                                                         libc::c_float)
                                                                    ->
                                                                        libc::c_float));
        let notchQ: libc::c_float =
            filterGetNotchQ(400 as libc::c_int as libc::c_float,
                            390 as libc::c_int as libc::c_float);
        let mut axis: libc::c_int = 0 as libc::c_int;
        while axis < 3 as libc::c_int {
            biquadFilterInit(&mut *(*gyroSensor).notchFilterDyn.as_mut_ptr().offset(axis
                                                                                        as
                                                                                        isize),
                             400 as libc::c_int as libc::c_float,
                             gyro.targetLooptime, notchQ, FILTER_NOTCH);
            axis += 1
        }
    };
}
unsafe extern "C" fn gyroInitSensorFilters(mut gyroSensor:
                                               *mut gyroSensor_t) {
    gyroInitSlewLimiter(gyroSensor);
    gyroInitLowpassFilterLpf(gyroSensor, FILTER_LOWPASS as libc::c_int,
                             (*gyroConfig()).gyro_lowpass_type as libc::c_int,
                             (*gyroConfig()).gyro_lowpass_hz);
    gyroInitLowpassFilterLpf(gyroSensor, FILTER_LOWPASS2 as libc::c_int,
                             (*gyroConfig()).gyro_lowpass2_type as
                                 libc::c_int,
                             (*gyroConfig()).gyro_lowpass2_hz);
    gyroInitFilterNotch1(gyroSensor, (*gyroConfig()).gyro_soft_notch_hz_1,
                         (*gyroConfig()).gyro_soft_notch_cutoff_1);
    gyroInitFilterNotch2(gyroSensor, (*gyroConfig()).gyro_soft_notch_hz_2,
                         (*gyroConfig()).gyro_soft_notch_cutoff_2);
    gyroInitFilterDynamicNotch(gyroSensor);
}
#[no_mangle]
pub unsafe extern "C" fn gyroInitFilters() {
    gyroInitSensorFilters(&mut gyroSensor1);
}
#[no_mangle]
pub unsafe extern "C" fn isGyroSensorCalibrationComplete(mut gyroSensor:
                                                             *const gyroSensor_t)
 -> bool {
    return (*gyroSensor).calibration.cyclesRemaining == 0 as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn isGyroCalibrationComplete() -> bool {
    return isGyroSensorCalibrationComplete(&mut gyroSensor1);
}
unsafe extern "C" fn isOnFinalGyroCalibrationCycle(mut gyroCalibration:
                                                       *const gyroCalibration_t)
 -> bool {
    return (*gyroCalibration).cyclesRemaining == 1 as libc::c_int;
}
unsafe extern "C" fn gyroCalculateCalibratingCycles() -> int32_t {
    return (((*gyroConfig()).gyroCalibrationDuration as libc::c_int *
                 10000 as libc::c_int) as
                libc::c_uint).wrapping_div(gyro.targetLooptime) as int32_t;
}
unsafe extern "C" fn isOnFirstGyroCalibrationCycle(mut gyroCalibration:
                                                       *const gyroCalibration_t)
 -> bool {
    return (*gyroCalibration).cyclesRemaining ==
               gyroCalculateCalibratingCycles();
}
unsafe extern "C" fn gyroSetCalibrationCycles(mut gyroSensor:
                                                  *mut gyroSensor_t) {
    (*gyroSensor).calibration.cyclesRemaining =
        gyroCalculateCalibratingCycles();
}
#[no_mangle]
pub unsafe extern "C" fn gyroStartCalibration(mut isFirstArmingCalibration:
                                                  bool) {
    if !(isFirstArmingCalibration as libc::c_int != 0 &&
             firstArmingCalibrationWasStarted as libc::c_int != 0) {
        gyroSetCalibrationCycles(&mut gyroSensor1);
        if isFirstArmingCalibration {
            firstArmingCalibrationWasStarted = 1 as libc::c_int != 0
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn isFirstArmingGyroCalibrationRunning() -> bool {
    return firstArmingCalibrationWasStarted as libc::c_int != 0 &&
               !isGyroCalibrationComplete();
}
unsafe extern "C" fn performGyroCalibration(mut gyroSensor: *mut gyroSensor_t,
                                            mut gyroMovementCalibrationThreshold:
                                                uint8_t) {
    let mut axis: libc::c_int = 0 as libc::c_int;
    while axis < 3 as libc::c_int {
        // Reset g[axis] at start of calibration
        if isOnFirstGyroCalibrationCycle(&mut (*gyroSensor).calibration) {
            (*gyroSensor).calibration.sum[axis as usize] = 0.0f32;
            devClear(&mut *(*gyroSensor).calibration.var.as_mut_ptr().offset(axis
                                                                                 as
                                                                                 isize));
            // gyroZero is set to zero until calibration complete
            (*gyroSensor).gyroDev.gyroZero[axis as usize] = 0.0f32
        }
        // Sum up CALIBRATING_GYRO_TIME_US readings
        (*gyroSensor).calibration.sum[axis as usize] +=
            (*gyroSensor).gyroDev.gyroADCRaw[axis as usize] as libc::c_int as
                libc::c_float;
        devPush(&mut *(*gyroSensor).calibration.var.as_mut_ptr().offset(axis
                                                                            as
                                                                            isize),
                (*gyroSensor).gyroDev.gyroADCRaw[axis as usize] as
                    libc::c_float);
        if isOnFinalGyroCalibrationCycle(&mut (*gyroSensor).calibration) {
            let stddev: libc::c_float =
                devStandardDeviation(&mut *(*gyroSensor).calibration.var.as_mut_ptr().offset(axis
                                                                                                 as
                                                                                                 isize));
            // DEBUG_GYRO_CALIBRATION records the standard deviation of roll
            // into the spare field - debug[3], in DEBUG_GYRO_RAW
            if axis == X as libc::c_int {
                if debugMode as libc::c_int == DEBUG_GYRO_RAW as libc::c_int {
                    debug[3 as libc::c_int as usize] =
                        lrintf(stddev) as int16_t
                }
            }
            // check deviation and startover in case the model was moved
            if gyroMovementCalibrationThreshold as libc::c_int != 0 &&
                   stddev >
                       gyroMovementCalibrationThreshold as libc::c_int as
                           libc::c_float {
                gyroSetCalibrationCycles(gyroSensor);
                return
            }
            // please take care with exotic boardalignment !!
            (*gyroSensor).gyroDev.gyroZero[axis as usize] =
                (*gyroSensor).calibration.sum[axis as usize] /
                    gyroCalculateCalibratingCycles() as
                        libc::c_float; // so calibration cycles do not pollute tasks statistics
            if axis == Z as libc::c_int {
                (*gyroSensor).gyroDev.gyroZero[axis as usize] -=
                    (*gyroConfig()).gyro_offset_yaw as libc::c_float /
                        100 as libc::c_int as libc::c_float
            }
        }
        axis += 1
    }
    if isOnFinalGyroCalibrationCycle(&mut (*gyroSensor).calibration) {
        schedulerResetTaskStatistics(TASK_SELF);
        if !firstArmingCalibrationWasStarted ||
               getArmingDisableFlags() as libc::c_uint &
                   !(ARMING_DISABLED_CALIBRATING as libc::c_int) as
                       libc::c_uint == 0 as libc::c_int as libc::c_uint {
            beeper(BEEPER_GYRO_CALIBRATED);
        }
    }
    (*gyroSensor).calibration.cyclesRemaining -= 1;
}
#[no_mangle]
pub unsafe extern "C" fn gyroSlewLimiter(mut gyroSensor: *mut gyroSensor_t,
                                         mut axis: libc::c_int) -> int32_t {
    let mut ret: int32_t =
        (*gyroSensor).gyroDev.gyroADCRaw[axis as usize] as int32_t;
    if (*gyroConfig()).checkOverflow as libc::c_int != 0 ||
           gyroHasOverflowProtection as libc::c_int != 0 {
        // don't use the slew limiter if overflow checking is on or gyro is not subject to overflow bug
        return ret
    }
    if abs(ret - (*gyroSensor).gyroDev.gyroADCRawPrevious[axis as usize]) >
           (1 as libc::c_int) << 14 as libc::c_int {
        // there has been a large change in value, so assume overflow has occurred and return the previous value
        ret = (*gyroSensor).gyroDev.gyroADCRawPrevious[axis as usize]
    } else { (*gyroSensor).gyroDev.gyroADCRawPrevious[axis as usize] = ret }
    return ret;
}
#[inline(never)]
unsafe extern "C" fn handleOverflow(mut gyroSensor: *mut gyroSensor_t,
                                    mut currentTimeUs: timeUs_t) {
    let gyroOverflowResetRate: libc::c_float =
        30340 as libc::c_int as libc::c_float * (*gyroSensor).gyroDev.scale;
    if (abs(gyro.gyroADCf[X as libc::c_int as usize] as libc::c_int) as
            libc::c_float) < gyroOverflowResetRate &&
           (abs(gyro.gyroADCf[Y as libc::c_int as usize] as libc::c_int) as
                libc::c_float) < gyroOverflowResetRate &&
           (abs(gyro.gyroADCf[Z as libc::c_int as usize] as libc::c_int) as
                libc::c_float) < gyroOverflowResetRate {
        // if we have 50ms of consecutive OK gyro vales, then assume yaw readings are OK again and reset overflowDetected
        // reset requires good OK values on all axes
        if cmpTimeUs(currentTimeUs, (*gyroSensor).overflowTimeUs) >
               50000 as libc::c_int {
            (*gyroSensor).overflowDetected = 0 as libc::c_int != 0
        }
    } else {
        // not a consecutive OK value, so reset the overflow time
        (*gyroSensor).overflowTimeUs = currentTimeUs
    };
}
unsafe extern "C" fn checkForOverflow(mut gyroSensor: *mut gyroSensor_t,
                                      mut currentTimeUs: timeUs_t) {
    // check for overflow to handle Yaw Spin To The Moon (YSTTM)
    // ICM gyros are specified to +/- 2000 deg/sec, in a crash they can go out of spec.
    // This can cause an overflow and sign reversal in the output.
    // Overflow and sign reversal seems to result in a gyro value of +1996 or -1996.
    if (*gyroSensor).overflowDetected {
        handleOverflow(gyroSensor, currentTimeUs);
    } else {
        // check for overflow in the axes set in overflowAxisMask
        let mut overflowCheck: gyroOverflow_e = GYRO_OVERFLOW_NONE;
        let gyroOverflowTriggerRate: libc::c_float =
            31980 as libc::c_int as libc::c_float *
                (*gyroSensor).gyroDev.scale;
        if abs(gyro.gyroADCf[X as libc::c_int as usize] as libc::c_int) as
               libc::c_float > gyroOverflowTriggerRate {
            overflowCheck =
                ::core::mem::transmute::<libc::c_uint,
                                         gyroOverflow_e>(overflowCheck as
                                                             libc::c_uint |
                                                             GYRO_OVERFLOW_X
                                                                 as
                                                                 libc::c_int
                                                                 as
                                                                 libc::c_uint)
        }
        if abs(gyro.gyroADCf[Y as libc::c_int as usize] as libc::c_int) as
               libc::c_float > gyroOverflowTriggerRate {
            overflowCheck =
                ::core::mem::transmute::<libc::c_uint,
                                         gyroOverflow_e>(overflowCheck as
                                                             libc::c_uint |
                                                             GYRO_OVERFLOW_Y
                                                                 as
                                                                 libc::c_int
                                                                 as
                                                                 libc::c_uint)
        }
        if abs(gyro.gyroADCf[Z as libc::c_int as usize] as libc::c_int) as
               libc::c_float > gyroOverflowTriggerRate {
            overflowCheck =
                ::core::mem::transmute::<libc::c_uint,
                                         gyroOverflow_e>(overflowCheck as
                                                             libc::c_uint |
                                                             GYRO_OVERFLOW_Z
                                                                 as
                                                                 libc::c_int
                                                                 as
                                                                 libc::c_uint)
        }
        if overflowCheck as libc::c_uint & overflowAxisMask as libc::c_uint !=
               0 {
            (*gyroSensor).overflowDetected = 1 as libc::c_int != 0;
            (*gyroSensor).overflowTimeUs = currentTimeUs;
            (*gyroSensor).yawSpinDetected = 0 as libc::c_int != 0
            // USE_YAW_SPIN_RECOVERY
        }
    };
}
// USE_GYRO_OVERFLOW_CHECK
#[inline(never)]
unsafe extern "C" fn handleYawSpin(mut gyroSensor: *mut gyroSensor_t,
                                   mut currentTimeUs: timeUs_t) {
    let yawSpinResetRate: libc::c_float =
        (*gyroConfig()).yaw_spin_threshold as libc::c_int as libc::c_float -
            100.0f32;
    if (abs(gyro.gyroADCf[Z as libc::c_int as usize] as libc::c_int) as
            libc::c_float) < yawSpinResetRate {
        // testing whether 20ms of consecutive OK gyro yaw values is enough
        if cmpTimeUs(currentTimeUs, (*gyroSensor).yawSpinTimeUs) >
               20000 as libc::c_int {
            (*gyroSensor).yawSpinDetected = 0 as libc::c_int != 0
        }
    } else {
        // reset the yaw spin time
        (*gyroSensor).yawSpinTimeUs = currentTimeUs
    };
}
unsafe extern "C" fn checkForYawSpin(mut gyroSensor: *mut gyroSensor_t,
                                     mut currentTimeUs: timeUs_t) {
    // if not in overflow mode, handle yaw spins above threshold
    if (*gyroSensor).overflowDetected {
        (*gyroSensor).yawSpinDetected = 0 as libc::c_int != 0;
        return
    }
    // USE_GYRO_OVERFLOW_CHECK
    if (*gyroSensor).yawSpinDetected {
        handleYawSpin(gyroSensor, currentTimeUs);
    } else if abs(gyro.gyroADCf[Z as libc::c_int as usize] as libc::c_int) >
                  (*gyroConfig()).yaw_spin_threshold as libc::c_int {
        (*gyroSensor).yawSpinDetected = 1 as libc::c_int != 0;
        (*gyroSensor).yawSpinTimeUs = currentTimeUs
    };
}
unsafe extern "C" fn filterGyroDebug(mut gyroSensor: *mut gyroSensor_t,
                                     mut sampleDeltaUs: timeDelta_t) {
    let mut axis: libc::c_int = 0 as libc::c_int;
    while axis < 3 as libc::c_int {
        if debugMode as libc::c_int == DEBUG_GYRO_RAW as libc::c_int {
            debug[axis as usize] =
                (*gyroSensor).gyroDev.gyroADCRaw[axis as usize]
        }
        // check for spin on yaw axis only
        // SIMULATOR_BUILD
        // scale gyro output to degrees per second
        // scale gyro output to degrees per second
        let mut gyroADCf: libc::c_float =
            (*gyroSensor).gyroDev.gyroADC[axis as usize] *
                (*gyroSensor).gyroDev.scale;
        // DEBUG_GYRO_SCALED records the unfiltered, scaled gyro output
        // DEBUG_GYRO_SCALED records the unfiltered, scaled gyro output
        if debugMode as libc::c_int == DEBUG_GYRO_SCALED as libc::c_int {
            debug[axis as usize] = lrintf(gyroADCf) as int16_t
        }
        if isDynamicFilterActive() {
            if axis == X as libc::c_int {
                if debugMode as libc::c_int == DEBUG_FFT as libc::c_int {
                    debug[0 as libc::c_int as usize] =
                        lrintf(gyroADCf) as int16_t
                }
                // store raw data
                // store raw data
                if debugMode as libc::c_int == DEBUG_FFT_FREQ as libc::c_int {
                    debug[3 as libc::c_int as usize] =
                        lrintf(gyroADCf) as int16_t
                }
            }
        }
        // store raw data
        // store raw data
        // apply static notch filters and software lowpass filters
        // apply static notch filters and software lowpass filters
        gyroADCf =
            (*gyroSensor).notchFilter1ApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).notchFilter1.as_mut_ptr().offset(axis
                                                                                                                                           as
                                                                                                                                           isize)
                                                                                      as
                                                                                      *mut biquadFilter_t
                                                                                      as
                                                                                      *mut filter_t,
                                                                                  gyroADCf);
        gyroADCf =
            (*gyroSensor).notchFilter2ApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).notchFilter2.as_mut_ptr().offset(axis
                                                                                                                                           as
                                                                                                                                           isize)
                                                                                      as
                                                                                      *mut biquadFilter_t
                                                                                      as
                                                                                      *mut filter_t,
                                                                                  gyroADCf);
        gyroADCf =
            (*gyroSensor).lowpassFilterApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).lowpassFilter.as_mut_ptr().offset(axis
                                                                                                                                             as
                                                                                                                                             isize)
                                                                                       as
                                                                                       *mut gyroLowpassFilter_t
                                                                                       as
                                                                                       *mut filter_t,
                                                                                   gyroADCf);
        gyroADCf =
            (*gyroSensor).lowpass2FilterApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).lowpass2Filter.as_mut_ptr().offset(axis
                                                                                                                                               as
                                                                                                                                               isize)
                                                                                        as
                                                                                        *mut gyroLowpassFilter_t
                                                                                        as
                                                                                        *mut filter_t,
                                                                                    gyroADCf);
        if isDynamicFilterActive() {
            gyroDataAnalysePush(&mut (*gyroSensor).gyroAnalyseState, axis,
                                gyroADCf);
            gyroADCf =
                (*gyroSensor).notchFilterDynApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).notchFilterDyn.as_mut_ptr().offset(axis
                                                                                                                                                   as
                                                                                                                                                   isize)
                                                                                            as
                                                                                            *mut biquadFilter_t
                                                                                            as
                                                                                            *mut filter_t,
                                                                                        gyroADCf);
            if axis == X as libc::c_int {
                if debugMode as libc::c_int == DEBUG_FFT as libc::c_int {
                    debug[1 as libc::c_int as usize] =
                        lrintf(gyroADCf) as int16_t
                }
                // store data after dynamic notch
                // store data after dynamic notch
            }
        }
        // DEBUG_GYRO_FILTERED records the scaled, filtered, after all software filtering has been applied.
        // DEBUG_GYRO_FILTERED records the scaled, filtered, after all software filtering has been applied.
        if debugMode as libc::c_int == DEBUG_GYRO_FILTERED as libc::c_int {
            debug[axis as usize] = lrintf(gyroADCf) as int16_t
        }
        (*gyroSensor).gyroDev.gyroADCf[axis as usize] = gyroADCf;
        if !(*gyroSensor).overflowDetected {
            // integrate using trapezium rule to avoid bias
            // integrate using trapezium rule to avoid bias
            accumulatedMeasurements[axis as usize] +=
                0.5f32 * (gyroPrevious[axis as usize] + gyroADCf) *
                    sampleDeltaUs as libc::c_float;
            gyroPrevious[axis as usize] = gyroADCf
        }
        axis += 1
    };
}
unsafe extern "C" fn filterGyro(mut gyroSensor: *mut gyroSensor_t,
                                mut sampleDeltaUs: timeDelta_t) {
    let mut axis: libc::c_int = 0 as libc::c_int;
    while axis < 3 as libc::c_int {
        let mut gyroADCf: libc::c_float =
            (*gyroSensor).gyroDev.gyroADC[axis as usize] *
                (*gyroSensor).gyroDev.scale;
        if isDynamicFilterActive() { (axis) == X as libc::c_int; }
        gyroADCf =
            (*gyroSensor).notchFilter1ApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).notchFilter1.as_mut_ptr().offset(axis
                                                                                                                                           as
                                                                                                                                           isize)
                                                                                      as
                                                                                      *mut biquadFilter_t
                                                                                      as
                                                                                      *mut filter_t,
                                                                                  gyroADCf);
        gyroADCf =
            (*gyroSensor).notchFilter2ApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).notchFilter2.as_mut_ptr().offset(axis
                                                                                                                                           as
                                                                                                                                           isize)
                                                                                      as
                                                                                      *mut biquadFilter_t
                                                                                      as
                                                                                      *mut filter_t,
                                                                                  gyroADCf);
        gyroADCf =
            (*gyroSensor).lowpassFilterApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).lowpassFilter.as_mut_ptr().offset(axis
                                                                                                                                             as
                                                                                                                                             isize)
                                                                                       as
                                                                                       *mut gyroLowpassFilter_t
                                                                                       as
                                                                                       *mut filter_t,
                                                                                   gyroADCf);
        gyroADCf =
            (*gyroSensor).lowpass2FilterApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).lowpass2Filter.as_mut_ptr().offset(axis
                                                                                                                                               as
                                                                                                                                               isize)
                                                                                        as
                                                                                        *mut gyroLowpassFilter_t
                                                                                        as
                                                                                        *mut filter_t,
                                                                                    gyroADCf);
        if isDynamicFilterActive() {
            gyroDataAnalysePush(&mut (*gyroSensor).gyroAnalyseState, axis,
                                gyroADCf);
            gyroADCf =
                (*gyroSensor).notchFilterDynApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).notchFilterDyn.as_mut_ptr().offset(axis
                                                                                                                                                   as
                                                                                                                                                   isize)
                                                                                            as
                                                                                            *mut biquadFilter_t
                                                                                            as
                                                                                            *mut filter_t,
                                                                                        gyroADCf);
            (axis) == X as libc::c_int;
        }
        (*gyroSensor).gyroDev.gyroADCf[axis as usize] = gyroADCf;
        if !(*gyroSensor).overflowDetected {
            accumulatedMeasurements[axis as usize] +=
                0.5f32 * (gyroPrevious[axis as usize] + gyroADCf) *
                    sampleDeltaUs as libc::c_float;
            gyroPrevious[axis as usize] = gyroADCf
        }
        axis += 1
    };
}
// USE_YAW_SPIN_RECOVERY
#[inline(never)]
unsafe extern "C" fn gyroUpdateSensor(mut gyroSensor: *mut gyroSensor_t,
                                      mut currentTimeUs: timeUs_t) {
    if !(*gyroSensor).gyroDev.readFn.expect("non-null function pointer")(&mut (*gyroSensor).gyroDev)
       {
        return
    }
    (*gyroSensor).gyroDev.dataReady = 0 as libc::c_int != 0;
    if isGyroSensorCalibrationComplete(gyroSensor) {
        // move 16-bit gyro data into 32-bit variables to avoid overflows in calculations
        (*gyroSensor).gyroDev.gyroADC[X as libc::c_int as usize] =
            gyroSlewLimiter(gyroSensor, X as libc::c_int) as libc::c_float -
                (*gyroSensor).gyroDev.gyroZero[X as libc::c_int as usize];
        (*gyroSensor).gyroDev.gyroADC[Y as libc::c_int as usize] =
            gyroSlewLimiter(gyroSensor, Y as libc::c_int) as libc::c_float -
                (*gyroSensor).gyroDev.gyroZero[Y as libc::c_int as usize];
        (*gyroSensor).gyroDev.gyroADC[Z as libc::c_int as usize] =
            gyroSlewLimiter(gyroSensor, Z as libc::c_int) as libc::c_float -
                (*gyroSensor).gyroDev.gyroZero[Z as libc::c_int as usize];
        alignSensors((*gyroSensor).gyroDev.gyroADC.as_mut_ptr(),
                     (*gyroSensor).gyroDev.gyroAlign as uint8_t);
    } else {
        performGyroCalibration(gyroSensor,
                               (*gyroConfig()).gyroMovementCalibrationThreshold);
        // still calibrating, so no need to further process gyro data
        return
    }
    let sampleDeltaUs: timeDelta_t =
        currentTimeUs.wrapping_sub(accumulationLastTimeSampledUs) as
            timeDelta_t;
    accumulationLastTimeSampledUs = currentTimeUs;
    accumulatedMeasurementTimeUs =
        (accumulatedMeasurementTimeUs as
             libc::c_uint).wrapping_add(sampleDeltaUs as libc::c_uint) as
            timeUs_t as timeUs_t;
    if (*gyroConfig()).checkOverflow as libc::c_int != 0 &&
           !gyroHasOverflowProtection {
        checkForOverflow(gyroSensor, currentTimeUs);
    }
    if (*gyroConfig()).yaw_spin_recovery != 0 {
        checkForYawSpin(gyroSensor, currentTimeUs);
    }
    if gyroDebugMode as libc::c_int == DEBUG_NONE as libc::c_int {
        filterGyro(gyroSensor, sampleDeltaUs);
    } else { filterGyroDebug(gyroSensor, sampleDeltaUs); }
    if isDynamicFilterActive() {
        gyroDataAnalyse(&mut (*gyroSensor).gyroAnalyseState,
                        (*gyroSensor).notchFilterDyn.as_mut_ptr());
    };
}
#[no_mangle]
pub unsafe extern "C" fn gyroUpdate(mut currentTimeUs: timeUs_t) {
    gyroUpdateSensor(&mut gyroSensor1, currentTimeUs);
    gyro.gyroADCf[X as libc::c_int as usize] =
        gyroSensor1.gyroDev.gyroADCf[X as libc::c_int as usize];
    gyro.gyroADCf[Y as libc::c_int as usize] =
        gyroSensor1.gyroDev.gyroADCf[Y as libc::c_int as usize];
    gyro.gyroADCf[Z as libc::c_int as usize] =
        gyroSensor1.gyroDev.gyroADCf[Z as libc::c_int as usize];
}
#[no_mangle]
pub unsafe extern "C" fn gyroGetAccumulationAverage(mut accumulationAverage:
                                                        *mut libc::c_float)
 -> bool {
    if accumulatedMeasurementTimeUs > 0 as libc::c_int as libc::c_uint {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        let mut axis: libc::c_int = 0 as libc::c_int;
        while axis < 3 as libc::c_int {
            *accumulationAverage.offset(axis as isize) =
                accumulatedMeasurements[axis as usize] /
                    accumulatedMeasurementTimeUs as libc::c_float;
            accumulatedMeasurements[axis as usize] = 0.0f32;
            axis += 1
        }
        accumulatedMeasurementTimeUs = 0 as libc::c_int as timeUs_t;
        return 1 as libc::c_int != 0
    } else {
        let mut axis_0: libc::c_int = 0 as libc::c_int;
        while axis_0 < 3 as libc::c_int {
            *accumulationAverage.offset(axis_0 as isize) = 0.0f32;
            axis_0 += 1
        }
        return 0 as libc::c_int != 0
    };
}
#[no_mangle]
pub unsafe extern "C" fn gyroReadTemperature() {
    if gyroSensor1.gyroDev.temperatureFn.is_some() {
        gyroSensor1.gyroDev.temperatureFn.expect("non-null function pointer")(&mut gyroSensor1.gyroDev,
                                                                              &mut gyroSensor1.gyroDev.temperature);
    };
}
#[no_mangle]
pub unsafe extern "C" fn gyroGetTemperature() -> int16_t {
    return gyroSensor1.gyroDev.temperature;
}
#[no_mangle]
pub unsafe extern "C" fn gyroRateDps(mut axis: libc::c_int) -> int16_t {
    return lrintf(gyro.gyroADCf[axis as usize] / gyroSensor1.gyroDev.scale) as
               int16_t;
}
#[no_mangle]
pub unsafe extern "C" fn gyroOverflowDetected() -> bool {
    return gyroSensor1.overflowDetected;
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
// gyro alignment
// people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
// Gyro sample divider
// gyro DLPF setting
// gyro 32khz DLPF setting
// Lowpass primary/secondary
// Gyro calibration duration in 1/100 second
// bandpass quality factor, 100 for steep sided bandpass
#[no_mangle]
pub unsafe extern "C" fn gyroYawSpinDetected() -> bool {
    return gyroSensor1.yawSpinDetected;
}
// USE_YAW_SPIN_RECOVERY
#[no_mangle]
pub unsafe extern "C" fn gyroAbsRateDps(mut axis: libc::c_int) -> uint16_t {
    return fabsf(gyro.gyroADCf[axis as usize]) as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn gyroReadRegister(mut whichSensor: uint8_t,
                                          mut reg: uint8_t) -> uint8_t {
    return mpuGyroReadRegister(gyroSensorBusByDevice(whichSensor), reg);
}
// USE_GYRO_REGISTER_DUMP
