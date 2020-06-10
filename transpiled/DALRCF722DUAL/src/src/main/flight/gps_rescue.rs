use ::libc;
extern "C" {
    #[no_mangle]
    fn cos(_: libc::c_double) -> libc::c_double;
    #[no_mangle]
    fn sqrt(_: libc::c_double) -> libc::c_double;
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
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
    fn scaleRangef(x: libc::c_float, srcFrom: libc::c_float,
                   srcTo: libc::c_float, destFrom: libc::c_float,
                   destTo: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    static mut GPS_distanceToHome: uint16_t;
    // distance to home point in meters
    #[no_mangle]
    static mut GPS_directionToHome: int16_t;
    #[no_mangle]
    static mut gpsSol: gpsSolutionData_t;
    #[no_mangle]
    fn GPS_reset_home_position();
    #[no_mangle]
    fn disarm();
    // (Super) rates are constrained to [0, 100] for Betaflight rates, so values higher than 100 won't make a difference. Range extended for RaceFlight rates.
    #[no_mangle]
    static mut rcCommand: [libc::c_float; 4];
    #[no_mangle]
    static mut rcControlsConfig_System: rcControlsConfig_t;
    #[no_mangle]
    fn isModeActivationConditionPresent(modeId: boxId_e) -> bool;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn setArmingDisabled(flag: armingDisableFlags_e);
    #[no_mangle]
    static mut flightModeFlags: uint16_t;
    #[no_mangle]
    static mut failsafeConfig_System: failsafeConfig_t;
    #[no_mangle]
    fn failsafeIsActive() -> bool;
    #[no_mangle]
    static mut attitude: attitudeEulerAngles_t;
    #[no_mangle]
    fn getCosTiltAngle() -> libc::c_float;
    #[no_mangle]
    fn crashRecoveryModeActive() -> bool;
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
    fn isAltitudeOffset() -> bool;
    #[no_mangle]
    fn getEstimatedAltitude() -> int32_t;
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    static mut acc: acc_t;
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
pub type C2RustUnnamed_1 = libc::c_uint;
pub const AI_PITCH: C2RustUnnamed_1 = 1;
pub const AI_ROLL: C2RustUnnamed_1 = 0;
/* !< SPI Error code                           */
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
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
// microsecond time
pub type timeUs_t = uint32_t;
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
pub type rc_alias = libc::c_uint;
pub const AUX8: rc_alias = 11;
pub const AUX7: rc_alias = 10;
pub const AUX6: rc_alias = 9;
pub const AUX5: rc_alias = 8;
pub const AUX4: rc_alias = 7;
pub const AUX3: rc_alias = 6;
pub const AUX2: rc_alias = 5;
pub const AUX1: rc_alias = 4;
pub const THROTTLE: rc_alias = 3;
pub const YAW: rc_alias = 2;
pub const PITCH: rc_alias = 1;
pub const ROLL: rc_alias = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rcControlsConfig_s {
    pub deadband: uint8_t,
    pub yaw_deadband: uint8_t,
    pub alt_hold_deadband: uint8_t,
    pub alt_hold_fast_change: uint8_t,
    pub yaw_control_reversed: bool,
}
pub type rcControlsConfig_t = rcControlsConfig_s;
pub type boxId_e = libc::c_uint;
pub const CHECKBOX_ITEM_COUNT: boxId_e = 41;
pub const BOXACROTRAINER: boxId_e = 40;
pub const BOXPIDAUDIO: boxId_e = 39;
pub const BOXUSER4: boxId_e = 38;
pub const BOXUSER3: boxId_e = 37;
pub const BOXUSER2: boxId_e = 36;
pub const BOXUSER1: boxId_e = 35;
pub const BOXPARALYZE: boxId_e = 34;
pub const BOXVTXPITMODE: boxId_e = 33;
pub const BOXBEEPGPSCOUNT: boxId_e = 32;
pub const BOXPREARM: boxId_e = 31;
pub const BOXFLIPOVERAFTERCRASH: boxId_e = 30;
pub const BOXCAMERA3: boxId_e = 29;
pub const BOXCAMERA2: boxId_e = 28;
pub const BOXCAMERA1: boxId_e = 27;
pub const BOXBLACKBOXERASE: boxId_e = 26;
pub const BOXFPVANGLEMIX: boxId_e = 25;
pub const BOX3D: boxId_e = 24;
pub const BOXAIRMODE: boxId_e = 23;
pub const BOXBLACKBOX: boxId_e = 22;
pub const BOXSERVO3: boxId_e = 21;
pub const BOXSERVO2: boxId_e = 20;
pub const BOXSERVO1: boxId_e = 19;
pub const BOXTELEMETRY: boxId_e = 18;
pub const BOXOSD: boxId_e = 17;
pub const BOXCALIB: boxId_e = 16;
pub const BOXLEDLOW: boxId_e = 15;
pub const BOXBEEPERON: boxId_e = 14;
pub const BOXCAMSTAB: boxId_e = 13;
pub const BOXHEADADJ: boxId_e = 12;
pub const BOXANTIGRAVITY: boxId_e = 11;
pub const BOXID_FLIGHTMODE_LAST: boxId_e = 10;
pub const BOXGPSRESCUE: boxId_e = 10;
pub const BOXFAILSAFE: boxId_e = 9;
pub const BOXPASSTHRU: boxId_e = 8;
pub const BOXHEADFREE: boxId_e = 7;
pub const BOXGPSHOLD: boxId_e = 6;
pub const BOXGPSHOME: boxId_e = 5;
pub const BOXBARO: boxId_e = 4;
pub const BOXMAG: boxId_e = 3;
pub const BOXHORIZON: boxId_e = 2;
pub const BOXANGLE: boxId_e = 1;
pub const BOXARM: boxId_e = 0;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_4 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_4 = 2;
pub const ARMED: C2RustUnnamed_4 = 1;
pub type armingDisableFlags_e = libc::c_uint;
pub const ARMING_DISABLED_ARM_SWITCH: armingDisableFlags_e = 524288;
pub const ARMING_DISABLED_GPS: armingDisableFlags_e = 262144;
pub const ARMING_DISABLED_PARALYZE: armingDisableFlags_e = 131072;
pub const ARMING_DISABLED_MSP: armingDisableFlags_e = 65536;
pub const ARMING_DISABLED_BST: armingDisableFlags_e = 32768;
pub const ARMING_DISABLED_OSD_MENU: armingDisableFlags_e = 16384;
pub const ARMING_DISABLED_CMS_MENU: armingDisableFlags_e = 8192;
pub const ARMING_DISABLED_CLI: armingDisableFlags_e = 4096;
pub const ARMING_DISABLED_CALIBRATING: armingDisableFlags_e = 2048;
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
pub type C2RustUnnamed_5 = libc::c_uint;
pub const GPS_RESCUE_MODE: C2RustUnnamed_5 = 2048;
pub const FAILSAFE_MODE: C2RustUnnamed_5 = 1024;
pub const PASSTHRU_MODE: C2RustUnnamed_5 = 256;
pub const HEADFREE_MODE: C2RustUnnamed_5 = 64;
pub const GPS_HOLD_MODE: C2RustUnnamed_5 = 32;
pub const GPS_HOME_MODE: C2RustUnnamed_5 = 16;
pub const BARO_MODE: C2RustUnnamed_5 = 8;
pub const MAG_MODE: C2RustUnnamed_5 = 4;
pub const HORIZON_MODE: C2RustUnnamed_5 = 2;
pub const ANGLE_MODE: C2RustUnnamed_5 = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct failsafeConfig_s {
    pub failsafe_throttle: uint16_t,
    pub failsafe_throttle_low_delay: uint16_t,
    pub failsafe_delay: uint8_t,
    pub failsafe_off_delay: uint8_t,
    pub failsafe_switch_mode: uint8_t,
    pub failsafe_procedure: uint8_t,
}
// latitude * 1e+7
// longitude * 1e+7
// altitude in 0.01m
// speed in 0.1m/s
// degrees * 10
// generic HDOP value (*100)
// introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
// introduce a deadband around the stick center for yaw axis. Must be greater than zero.
// defines the neutral zone of throttle stick during altitude hold, default setting is +/-40
// when disabled, turn off the althold when throttle stick is out of deadband defined with alt_hold_deadband; when enabled, altitude changes slowly proportional to stick movement
// invert control direction of yaw
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
// millis
// millis
pub type failsafeConfig_t = failsafeConfig_s;
pub type C2RustUnnamed_6 = libc::c_uint;
pub const FAILSAFE_PROCEDURE_GPS_RESCUE: C2RustUnnamed_6 = 2;
pub const FAILSAFE_PROCEDURE_DROP_IT: C2RustUnnamed_6 = 1;
pub const FAILSAFE_PROCEDURE_AUTO_LANDING: C2RustUnnamed_6 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub union attitudeEulerAngles_t {
    pub raw: [int16_t; 3],
    pub values: C2RustUnnamed_7,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_7 {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
// Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.
// Time throttle stick must have been below 'min_check' to "JustDisarm" instead of "full failsafe procedure".
// Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
// Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example (200)
// failsafe switch action is 0: stage1 (identical to rc link loss), 1: disarms instantly, 2: stage2
// selected full failsafe procedure is 0: auto-landing, 1: Drop it
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
// used by receiver driver to return channel data
// number of RC channels as reported by current input driver
// !!TODO remove this extern, only needed once for channelCount
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxConfig_s {
    pub rcmap: [uint8_t; 8],
    pub serialrx_provider: uint8_t,
    pub serialrx_inverted: uint8_t,
    pub halfDuplex: uint8_t,
    pub spektrum_bind_pin_override_ioTag: ioTag_t,
    pub spektrum_bind_plug_ioTag: ioTag_t,
    pub spektrum_sat_bind: uint8_t,
    pub spektrum_sat_bind_autoreset: uint8_t,
    pub rssi_channel: uint8_t,
    pub rssi_scale: uint8_t,
    pub rssi_invert: uint8_t,
    pub midrc: uint16_t,
    pub mincheck: uint16_t,
    pub maxcheck: uint16_t,
    pub rcInterpolation: uint8_t,
    pub rcInterpolationChannels: uint8_t,
    pub rcInterpolationInterval: uint8_t,
    pub fpvCamAngleDegrees: uint8_t,
    pub airModeActivateThreshold: uint8_t,
    pub rx_min_usec: uint16_t,
    pub rx_max_usec: uint16_t,
    pub max_aux_channel: uint8_t,
    pub rssi_src_frame_errors: uint8_t,
    pub rssi_offset: int8_t,
    pub rc_smoothing_type: uint8_t,
    pub rc_smoothing_input_cutoff: uint8_t,
    pub rc_smoothing_derivative_cutoff: uint8_t,
    pub rc_smoothing_debug_axis: uint8_t,
    pub rc_smoothing_input_type: uint8_t,
    pub rc_smoothing_derivative_type: uint8_t,
}
pub type rxConfig_t = rxConfig_s;
// Derivative filter type (0 = OFF, 1 = PT1, 2 = BIQUAD)
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
    pub busdev_u: C2RustUnnamed_8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_8 {
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
pub type accDev_t = accDev_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct acc_s {
    pub dev: accDev_t,
    pub accSamplingInterval: uint32_t,
    pub accADC: [libc::c_float; 3],
    pub isAccelUpdatedAtLeastOnce: bool,
}
pub type acc_t = acc_s;
pub type gpsRescueSanity_e = libc::c_uint;
pub const RESCUE_SANITY_FS_ONLY: gpsRescueSanity_e = 2;
pub const RESCUE_SANITY_ON: gpsRescueSanity_e = 1;
pub const RESCUE_SANITY_OFF: gpsRescueSanity_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsRescue_s {
    pub angle: uint16_t,
    pub initialAltitude: uint16_t,
    pub descentDistance: uint16_t,
    pub rescueGroundspeed: uint16_t,
    pub throttleP: uint16_t,
    pub throttleI: uint16_t,
    pub throttleD: uint16_t,
    pub yawP: uint16_t,
    pub throttleMin: uint16_t,
    pub throttleMax: uint16_t,
    pub throttleHover: uint16_t,
    pub velP: uint16_t,
    pub velI: uint16_t,
    pub velD: uint16_t,
    pub minSats: uint8_t,
    pub sanityChecks: gpsRescueSanity_e,
}
pub type gpsRescueConfig_t = gpsRescue_s;
pub type rescuePhase_e = libc::c_uint;
pub const RESCUE_COMPLETE: rescuePhase_e = 7;
pub const RESCUE_ABORT: rescuePhase_e = 6;
pub const RESCUE_LANDING: rescuePhase_e = 5;
pub const RESCUE_LANDING_APPROACH: rescuePhase_e = 4;
pub const RESCUE_CROSSTRACK: rescuePhase_e = 3;
pub const RESCUE_ATTAIN_ALT: rescuePhase_e = 2;
pub const RESCUE_INITIALIZE: rescuePhase_e = 1;
pub const RESCUE_IDLE: rescuePhase_e = 0;
pub type rescueFailureState_e = libc::c_uint;
pub const RESCUE_TOO_CLOSE: rescueFailureState_e = 3;
pub const RESCUE_CRASH_DETECTED: rescueFailureState_e = 2;
pub const RESCUE_FLYAWAY: rescueFailureState_e = 1;
pub const RESCUE_HEALTHY: rescueFailureState_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rescueIntent_s {
    pub targetAltitude: int32_t,
    pub targetGroundspeed: int32_t,
    pub minAngleDeg: uint8_t,
    pub maxAngleDeg: uint8_t,
    pub crosstrack: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rescueSensorData_s {
    pub maxAltitude: int32_t,
    pub currentAltitude: int32_t,
    pub distanceToHome: uint16_t,
    pub maxDistanceToHome: uint16_t,
    pub directionToHome: int16_t,
    pub groundSpeed: uint16_t,
    pub numSat: uint8_t,
    pub zVelocity: libc::c_float,
    pub zVelocityAvg: libc::c_float,
    pub accMagnitude: libc::c_float,
    pub accMagnitudeAvg: libc::c_float,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rescueState_s {
    pub phase: rescuePhase_e,
    pub failure: rescueFailureState_e,
    pub sensor: rescueSensorData_s,
    pub intent: rescueIntent_s,
    pub isFailsafe: bool,
}
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn constrainf(mut amt: libc::c_float,
                                mut low: libc::c_float,
                                mut high: libc::c_float) -> libc::c_float {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn rcControlsConfig() -> *const rcControlsConfig_t {
    return &mut rcControlsConfig_System;
}
#[inline]
unsafe extern "C" fn failsafeConfig() -> *const failsafeConfig_t {
    return &mut failsafeConfig_System;
}
#[inline]
unsafe extern "C" fn rxConfig() -> *const rxConfig_t {
    return &mut rxConfig_System;
}
#[inline]
unsafe extern "C" fn gpsRescueConfig() -> *const gpsRescueConfig_t {
    return &mut gpsRescueConfig_System;
}
// initialize function
// read 3 axis data function
// a revision code for the sensor, if known
//degrees
//meters
//meters
// centimeters per second
// Scale the commanded yaw rate when the error is less then this angle
#[no_mangle]
pub static mut gpsRescueConfig_System: gpsRescueConfig_t =
    gpsRescueConfig_t{angle: 0,
                      initialAltitude: 0,
                      descentDistance: 0,
                      rescueGroundspeed: 0,
                      throttleP: 0,
                      throttleI: 0,
                      throttleD: 0,
                      yawP: 0,
                      throttleMin: 0,
                      throttleMax: 0,
                      throttleHover: 0,
                      velP: 0,
                      velI: 0,
                      velD: 0,
                      minSats: 0,
                      sanityChecks: RESCUE_SANITY_OFF,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut gpsRescueConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (55 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<gpsRescueConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &gpsRescueConfig_System as
                                     *const gpsRescueConfig_t as
                                     *mut gpsRescueConfig_t as *mut uint8_t,
                             copy:
                                 &gpsRescueConfig_Copy as
                                     *const gpsRescueConfig_t as
                                     *mut gpsRescueConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_3{ptr:
                                                     &pgResetTemplate_gpsRescueConfig
                                                         as
                                                         *const gpsRescueConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut gpsRescueConfig_Copy: gpsRescueConfig_t =
    gpsRescueConfig_t{angle: 0,
                      initialAltitude: 0,
                      descentDistance: 0,
                      rescueGroundspeed: 0,
                      throttleP: 0,
                      throttleI: 0,
                      throttleD: 0,
                      yawP: 0,
                      throttleMin: 0,
                      throttleMax: 0,
                      throttleHover: 0,
                      velP: 0,
                      velI: 0,
                      velD: 0,
                      minSats: 0,
                      sanityChecks: RESCUE_SANITY_OFF,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_gpsRescueConfig: gpsRescueConfig_t =
    {
        let mut init =
            gpsRescue_s{angle: 32 as libc::c_int as uint16_t,
                        initialAltitude: 50 as libc::c_int as uint16_t,
                        descentDistance: 200 as libc::c_int as uint16_t,
                        rescueGroundspeed: 2000 as libc::c_int as uint16_t,
                        throttleP: 150 as libc::c_int as uint16_t,
                        throttleI: 20 as libc::c_int as uint16_t,
                        throttleD: 50 as libc::c_int as uint16_t,
                        yawP: 40 as libc::c_int as uint16_t,
                        throttleMin: 1200 as libc::c_int as uint16_t,
                        throttleMax: 1600 as libc::c_int as uint16_t,
                        throttleHover: 1280 as libc::c_int as uint16_t,
                        velP: 80 as libc::c_int as uint16_t,
                        velI: 20 as libc::c_int as uint16_t,
                        velD: 15 as libc::c_int as uint16_t,
                        minSats: 8 as libc::c_int as uint8_t,
                        sanityChecks: RESCUE_SANITY_ON,};
        init
    };
static mut rescueThrottle: uint16_t = 0;
static mut rescueYaw: libc::c_float = 0.;
#[no_mangle]
pub static mut gpsRescueAngle: [int32_t; 2] =
    [0 as libc::c_int, 0 as libc::c_int];
#[no_mangle]
pub static mut hoverThrottle: uint16_t = 0 as libc::c_int as uint16_t;
#[no_mangle]
pub static mut averageThrottle: libc::c_float = 0.0f64 as libc::c_float;
#[no_mangle]
pub static mut altitudeError: libc::c_float = 0.0f64 as libc::c_float;
#[no_mangle]
pub static mut throttleSamples: uint32_t = 0 as libc::c_int as uint32_t;
static mut newGPSData: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub static mut rescueState: rescueState_s =
    rescueState_s{phase: RESCUE_IDLE,
                  failure: RESCUE_HEALTHY,
                  sensor:
                      rescueSensorData_s{maxAltitude: 0,
                                         currentAltitude: 0,
                                         distanceToHome: 0,
                                         maxDistanceToHome: 0,
                                         directionToHome: 0,
                                         groundSpeed: 0,
                                         numSat: 0,
                                         zVelocity: 0.,
                                         zVelocityAvg: 0.,
                                         accMagnitude: 0.,
                                         accMagnitudeAvg: 0.,},
                  intent:
                      rescueIntent_s{targetAltitude: 0,
                                     targetGroundspeed: 0,
                                     minAngleDeg: 0,
                                     maxAngleDeg: 0,
                                     crosstrack: false,},
                  isFailsafe: false,};
/*
 If we have new GPS data, update home heading
 if possible and applicable.
*/
#[no_mangle]
pub unsafe extern "C" fn rescueNewGpsData() {
    if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
        GPS_reset_home_position();
    }
    newGPSData = 1 as libc::c_int != 0;
}
//NOTE: ANGLES ARE IN CENTIDEGREES
/*
    Determine what phase we are in, determine if all criteria are met to move to the next phase
*/
#[no_mangle]
pub unsafe extern "C" fn updateGPSRescueState() {
    if flightModeFlags as libc::c_int & GPS_RESCUE_MODE as libc::c_int == 0 {
        rescueStop();
    } else if flightModeFlags as libc::c_int & GPS_RESCUE_MODE as libc::c_int
                  != 0 &&
                  rescueState.phase as libc::c_uint ==
                      RESCUE_IDLE as libc::c_int as libc::c_uint {
        rescueStart();
    }
    rescueState.isFailsafe = failsafeIsActive();
    sensorUpdate();
    let mut newAlt: int32_t = 0;
    let mut newSpeed: int32_t = 0;
    let mut current_block_61: u64;
    match rescueState.phase as libc::c_uint {
        0 => { idleTasks(); current_block_61 = 12930649117290160518; }
        1 => {
            if hoverThrottle as libc::c_int == 0 as libc::c_int {
                //no actual throttle data yet, let's use the default.
                hoverThrottle = (*gpsRescueConfig()).throttleHover
            }
            // Minimum distance detection (100m).  Disarm regardless of sanity check configuration.  Rescue too close is never a good idea.
            if (rescueState.sensor.distanceToHome as libc::c_int) <
                   100 as libc::c_int {
                // Never allow rescue mode to engage as a failsafe within 100 meters or when disarmed.
                if rescueState.isFailsafe as libc::c_int != 0 ||
                       armingFlags as libc::c_int & ARMED as libc::c_int == 0
                   {
                    rescueState.failure = RESCUE_TOO_CLOSE;
                    setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
                    disarm();
                } else {
                    // Leave it up to the sanity check setting
                    rescueState.failure = RESCUE_TOO_CLOSE
                }
            }
            rescueState.phase = RESCUE_ATTAIN_ALT;
            current_block_61 = 12144408659737254161;
        }
        2 => { current_block_61 = 12144408659737254161; }
        3 => {
            if (rescueState.sensor.distanceToHome as libc::c_int) <
                   (*gpsRescueConfig()).descentDistance as libc::c_int {
                rescueState.phase = RESCUE_LANDING_APPROACH
            }
            // We can assume at this point that we are at or above our RTH height, so we need to try and point to home and tilt while maintaining alt
        // Is our altitude way off?  We should probably kick back to phase RESCUE_ATTAIN_ALT
            rescueState.intent.targetGroundspeed =
                (*gpsRescueConfig()).rescueGroundspeed as int32_t;
            rescueState.intent.targetAltitude =
                ({
                     let mut _a: libc::c_int =
                         (*gpsRescueConfig()).initialAltitude as libc::c_int *
                             100 as libc::c_int;
                     let mut _b: libc::c_int =
                         rescueState.sensor.maxAltitude + 1500 as libc::c_int;
                     if _a > _b { _a } else { _b }
                 });
            rescueState.intent.crosstrack = 1 as libc::c_int != 0;
            rescueState.intent.minAngleDeg = 15 as libc::c_int as uint8_t;
            rescueState.intent.maxAngleDeg =
                (*gpsRescueConfig()).angle as uint8_t;
            current_block_61 = 12930649117290160518;
        }
        4 => {
            // We are getting close to home in the XY plane, get Z where it needs to be to move to landing phase
            if (rescueState.sensor.distanceToHome as libc::c_int) <
                   10 as libc::c_int &&
                   rescueState.sensor.currentAltitude <= 1000 as libc::c_int {
                rescueState.phase = RESCUE_LANDING
            }
            // Only allow new altitude and new speed to be equal or lower than the current values (to prevent parabolic movement on overshoot)
            newAlt =
                (*gpsRescueConfig()).initialAltitude as libc::c_int *
                    100 as libc::c_int *
                    rescueState.sensor.distanceToHome as libc::c_int /
                    (*gpsRescueConfig()).descentDistance as libc::c_int;
            newSpeed =
                (*gpsRescueConfig()).rescueGroundspeed as libc::c_int *
                    rescueState.sensor.distanceToHome as libc::c_int /
                    (*gpsRescueConfig()).descentDistance as libc::c_int;
            rescueState.intent.targetAltitude =
                constrain(newAlt, 100 as libc::c_int,
                          rescueState.intent.targetAltitude);
            rescueState.intent.targetGroundspeed =
                constrain(newSpeed, 100 as libc::c_int,
                          rescueState.intent.targetGroundspeed);
            rescueState.intent.crosstrack = 1 as libc::c_int != 0;
            rescueState.intent.minAngleDeg = 10 as libc::c_int as uint8_t;
            rescueState.intent.maxAngleDeg = 20 as libc::c_int as uint8_t;
            current_block_61 = 12930649117290160518;
        }
        5 => {
            // We have reached the XYZ envelope to be considered at "home".  We need to land gently and check our accelerometer for abnormal data.
        // At this point, do not let the target altitude go up anymore, so if we overshoot, we dont' move in a parabolic trajectory
            // If we are over 120% of average magnitude, just disarm since we're pretty much home
            if rescueState.sensor.accMagnitude as libc::c_double >
                   rescueState.sensor.accMagnitudeAvg as libc::c_double *
                       1.5f64 {
                setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
                disarm();
                rescueState.phase = RESCUE_COMPLETE
            }
            rescueState.intent.targetGroundspeed = 0 as libc::c_int;
            rescueState.intent.targetAltitude = 0 as libc::c_int;
            rescueState.intent.crosstrack = 1 as libc::c_int != 0;
            rescueState.intent.minAngleDeg = 0 as libc::c_int as uint8_t;
            rescueState.intent.maxAngleDeg = 15 as libc::c_int as uint8_t;
            current_block_61 = 12930649117290160518;
        }
        7 => { rescueStop(); current_block_61 = 12930649117290160518; }
        6 => {
            setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            disarm();
            rescueStop();
            current_block_61 = 12930649117290160518;
        }
        _ => { current_block_61 = 12930649117290160518; }
    }
    match current_block_61 {
        12144408659737254161 => {
            // Get to a safe altitude at a low velocity ASAP
            if ({
                    let mut _x: libc::c_int =
                        rescueState.intent.targetAltitude -
                            rescueState.sensor.currentAltitude;
                    (if _x > 0 as libc::c_int { _x } else { -_x })
                }) < 1000 as libc::c_int {
                rescueState.phase = RESCUE_CROSSTRACK
            }
            rescueState.intent.targetGroundspeed = 500 as libc::c_int;
            rescueState.intent.targetAltitude =
                ({
                     let mut _a: libc::c_int =
                         (*gpsRescueConfig()).initialAltitude as libc::c_int *
                             100 as libc::c_int;
                     let mut _b: libc::c_int =
                         rescueState.sensor.maxAltitude + 1500 as libc::c_int;
                     if _a > _b { _a } else { _b }
                 });
            rescueState.intent.crosstrack = 1 as libc::c_int != 0;
            rescueState.intent.minAngleDeg = 10 as libc::c_int as uint8_t;
            rescueState.intent.maxAngleDeg = 15 as libc::c_int as uint8_t
        }
        _ => { }
    }
    performSanityChecks();
    if rescueState.phase as libc::c_uint !=
           RESCUE_IDLE as libc::c_int as libc::c_uint {
        rescueAttainPosition();
    }
    newGPSData = 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn sensorUpdate() {
    rescueState.sensor.currentAltitude = getEstimatedAltitude();
    // Calculate altitude velocity
    static mut previousTimeUs: uint32_t = 0;
    static mut previousAltitude: int32_t = 0;
    let currentTimeUs: uint32_t = micros();
    let dTime: libc::c_float =
        currentTimeUs.wrapping_sub(previousTimeUs) as libc::c_float;
    if newGPSData {
        // Calculate velocity at lowest common denominator
        rescueState.sensor.distanceToHome = GPS_distanceToHome;
        rescueState.sensor.directionToHome = GPS_directionToHome;
        rescueState.sensor.numSat = gpsSol.numSat;
        rescueState.sensor.groundSpeed = gpsSol.groundSpeed;
        rescueState.sensor.zVelocity =
            (rescueState.sensor.currentAltitude - previousAltitude) as
                libc::c_float * 1000000.0f32 / dTime;
        rescueState.sensor.zVelocityAvg =
            0.8f32 * rescueState.sensor.zVelocityAvg +
                rescueState.sensor.zVelocity * 0.2f32;
        rescueState.sensor.accMagnitude =
            sqrt((acc.accADC[Z as libc::c_int as usize] *
                      acc.accADC[Z as libc::c_int as usize] +
                      acc.accADC[X as libc::c_int as usize] *
                          acc.accADC[X as libc::c_int as usize] +
                      acc.accADC[Y as libc::c_int as usize] *
                          acc.accADC[Y as libc::c_int as usize] /
                          (acc.dev.acc_1G as libc::c_int *
                               acc.dev.acc_1G as libc::c_int) as
                              libc::c_float) as libc::c_double) as
                libc::c_float;
        rescueState.sensor.accMagnitudeAvg =
            rescueState.sensor.accMagnitudeAvg * 0.8f32 +
                rescueState.sensor.accMagnitude * 0.2f32;
        previousAltitude = rescueState.sensor.currentAltitude;
        previousTimeUs = currentTimeUs
    };
}
#[no_mangle]
pub unsafe extern "C" fn performSanityChecks() {
    if rescueState.phase as libc::c_uint ==
           RESCUE_IDLE as libc::c_int as libc::c_uint {
        rescueState.failure = RESCUE_HEALTHY;
        return
    }
    // Do not abort until each of these items is fully tested
    if rescueState.failure as libc::c_uint !=
           RESCUE_HEALTHY as libc::c_int as libc::c_uint {
        if (*gpsRescueConfig()).sanityChecks as libc::c_uint ==
               RESCUE_SANITY_ON as libc::c_int as libc::c_uint ||
               (*gpsRescueConfig()).sanityChecks as libc::c_uint ==
                   RESCUE_SANITY_FS_ONLY as libc::c_int as libc::c_uint &&
                   rescueState.isFailsafe as libc::c_int == 1 as libc::c_int {
            rescueState.phase = RESCUE_ABORT
        }
    }
    // Check if crash recovery mode is active, disarm if so.
    if crashRecoveryModeActive() {
        rescueState.failure = RESCUE_CRASH_DETECTED
    }
    //  Things that should run at a low refresh rate (such as flyaway detection, etc)
    //  This runs at ~1hz
    static mut previousTimeUs: uint32_t = 0;
    let currentTimeUs: uint32_t = micros();
    let dTime: uint32_t = currentTimeUs.wrapping_sub(previousTimeUs);
    if dTime < 1000000 as libc::c_int as libc::c_uint {
        //1hz
        return
    }
    previousTimeUs = currentTimeUs;
    // Stalled movement detection
    static mut gsI: int8_t = 0 as libc::c_int as int8_t;
    gsI =
        constrain(if gsI as libc::c_int +
                         ((rescueState.sensor.groundSpeed as libc::c_int) <
                              150 as libc::c_int) as libc::c_int != 0 {
                      1 as libc::c_int
                  } else { -(1 as libc::c_int) }, -(10 as libc::c_int),
                  10 as libc::c_int) as int8_t;
    if gsI as libc::c_int == 10 as libc::c_int {
        rescueState.failure = RESCUE_CRASH_DETECTED
    }
    // Minimum sat detection
    static mut msI: int8_t = 0 as libc::c_int as int8_t;
    msI =
        constrain(if msI as libc::c_int +
                         ((rescueState.sensor.numSat as libc::c_int) <
                              (*gpsRescueConfig()).minSats as libc::c_int) as
                             libc::c_int != 0 {
                      1 as libc::c_int
                  } else { -(1 as libc::c_int) }, -(5 as libc::c_int),
                  5 as libc::c_int) as int8_t;
    if msI as libc::c_int == 5 as libc::c_int {
        rescueState.failure = RESCUE_FLYAWAY
    };
}
#[no_mangle]
pub unsafe extern "C" fn rescueStart() {
    rescueState.phase = RESCUE_INITIALIZE;
}
#[no_mangle]
pub unsafe extern "C" fn rescueStop() { rescueState.phase = RESCUE_IDLE; }
// Things that need to run regardless of GPS rescue mode being enabled or not
#[no_mangle]
pub unsafe extern "C" fn idleTasks() {
    // Do not calculate any of the idle task values when we are not flying
    if armingFlags as libc::c_int & ARMED as libc::c_int == 0 { return }
    // Don't update any rescue flight statistics if we haven't applied a proper altitude offset yet
    if !isAltitudeOffset() { return }
    gpsRescueAngle[AI_PITCH as libc::c_int as usize] = 0 as libc::c_int;
    gpsRescueAngle[AI_ROLL as libc::c_int as usize] = 0 as libc::c_int;
    // Store the max altitude we see not during RTH so we know our fly-back minimum alt
    rescueState.sensor.maxAltitude =
        ({
             let mut _a: int32_t = rescueState.sensor.currentAltitude;
             let mut _b: int32_t = rescueState.sensor.maxAltitude;
             if _a > _b { _a } else { _b }
         });
    // Store the max distance to home during normal flight so we know if a flyaway is happening
    rescueState.sensor.maxDistanceToHome =
        ({
             let mut _a: uint16_t = rescueState.sensor.distanceToHome;
             let mut _b: uint16_t = rescueState.sensor.maxDistanceToHome;
             if _a as libc::c_int > _b as libc::c_int {
                 _a as libc::c_int
             } else { _b as libc::c_int }
         }) as uint16_t;
    rescueThrottle = rcCommand[THROTTLE as libc::c_int as usize] as uint16_t;
    //to do: have a default value for hoverThrottle
    // FIXME: GPS Rescue throttle handling should take into account min_check as the
    // active throttle is from min_check through PWM_RANGE_MAX. Currently adjusting for this
    // in gpsRescueGetThrottle() but it would be better handled here.
    let mut ct: libc::c_float = getCosTiltAngle();
    if ct as libc::c_double > 0.5f64 && (ct as libc::c_double) < 0.96f64 &&
           (throttleSamples as libc::c_double) < 1E6f64 &&
           rescueThrottle as libc::c_int > 1070 as libc::c_int {
        //5 to 45 degrees tilt
        //TO DO: only sample when acceleration is low
        let mut adjustedThrottle: uint16_t =
            (1000 as libc::c_int as libc::c_float +
                 (rescueThrottle as libc::c_int - 1000 as libc::c_int) as
                     libc::c_float * ct) as uint16_t;
        if throttleSamples == 0 as libc::c_int as libc::c_uint {
            averageThrottle = adjustedThrottle as libc::c_float
        } else {
            averageThrottle +=
                (adjustedThrottle as libc::c_int as libc::c_float -
                     averageThrottle) /
                    throttleSamples.wrapping_add(1 as libc::c_int as
                                                     libc::c_uint) as
                        libc::c_float
        }
        hoverThrottle = lrintf(averageThrottle) as uint16_t;
        throttleSamples = throttleSamples.wrapping_add(1)
    };
}
#[no_mangle]
pub unsafe extern "C" fn rescueAttainPosition() {
    // Point to home if that is in our intent
    if rescueState.intent.crosstrack {
        setBearing(rescueState.sensor.directionToHome);
    }
    if !newGPSData { return }
    /* *
        Speed controller
    */
    static mut previousSpeedError: libc::c_float =
        0 as libc::c_int as libc::c_float;
    static mut speedIntegral: int16_t = 0 as libc::c_int as int16_t;
    let speedError: int16_t =
        ((rescueState.intent.targetGroundspeed -
              rescueState.sensor.groundSpeed as libc::c_int) /
             100 as libc::c_int) as int16_t;
    let speedDerivative: int16_t =
        (speedError as libc::c_int as libc::c_float - previousSpeedError) as
            int16_t;
    speedIntegral =
        constrain(speedIntegral as libc::c_int + speedError as libc::c_int,
                  -(100 as libc::c_int), 100 as libc::c_int) as int16_t;
    previousSpeedError = speedError as libc::c_float;
    let mut angleAdjustment: int16_t =
        ((*gpsRescueConfig()).velP as libc::c_int * speedError as libc::c_int
             +
             (*gpsRescueConfig()).velI as libc::c_int *
                 speedIntegral as libc::c_int / 100 as libc::c_int +
             (*gpsRescueConfig()).velD as libc::c_int *
                 speedDerivative as libc::c_int) as int16_t;
    gpsRescueAngle[AI_PITCH as libc::c_int as usize] =
        constrain(gpsRescueAngle[AI_PITCH as libc::c_int as usize] +
                      ({
                           let mut _a: int16_t = angleAdjustment;
                           let mut _b: libc::c_int = 80 as libc::c_int;
                           (if (_a as libc::c_int) < _b {
                                _a as libc::c_int
                            } else { _b })
                       }),
                  rescueState.intent.minAngleDeg as libc::c_int *
                      100 as libc::c_int,
                  rescueState.intent.maxAngleDeg as libc::c_int *
                      100 as libc::c_int);
    let mut ct: libc::c_float =
        cos(((gpsRescueAngle[AI_PITCH as libc::c_int as usize] /
                  10 as libc::c_int) as libc::c_float / 10.0f32 *
                 0.0174532925f32) as libc::c_double) as libc::c_float;
    /* *
        Altitude controller
    */
    static mut previousAltitudeError: libc::c_float =
        0 as libc::c_int as libc::c_float; // Error in meters
    static mut altitudeIntegral: int16_t = 0 as libc::c_int as int16_t;
    let altitudeError_0: int16_t =
        ((rescueState.intent.targetAltitude -
              rescueState.sensor.currentAltitude) / 100 as libc::c_int) as
            int16_t;
    let altitudeDerivative: int16_t =
        (altitudeError_0 as libc::c_int as libc::c_float -
             previousAltitudeError) as int16_t;
    // Only allow integral windup within +-15m absolute altitude error
    if ({
            let _x: int16_t = altitudeError_0;
            (if _x as libc::c_int > 0 as libc::c_int {
                 _x as libc::c_int
             } else { -(_x as libc::c_int) })
        }) < 25 as libc::c_int {
        altitudeIntegral =
            constrain(altitudeIntegral as libc::c_int +
                          altitudeError_0 as libc::c_int,
                      -(250 as libc::c_int), 250 as libc::c_int) as int16_t
    } else { altitudeIntegral = 0 as libc::c_int as int16_t }
    previousAltitudeError = altitudeError_0 as libc::c_float;
    let mut altitudeAdjustment: int16_t =
        (((*gpsRescueConfig()).throttleP as libc::c_int *
              altitudeError_0 as libc::c_int +
              (*gpsRescueConfig()).throttleI as libc::c_int *
                  altitudeIntegral as libc::c_int / 10 as libc::c_int *
                  (*gpsRescueConfig()).throttleD as libc::c_int *
                  altitudeDerivative as libc::c_int) as libc::c_float / ct /
             20 as libc::c_int as libc::c_float) as int16_t;
    let mut hoverAdjustment: int16_t =
        ((hoverThrottle as libc::c_int - 1000 as libc::c_int) as libc::c_float
             / ct) as int16_t;
    rescueThrottle =
        constrain(1000 as libc::c_int + altitudeAdjustment as libc::c_int +
                      hoverAdjustment as libc::c_int,
                  (*gpsRescueConfig()).throttleMin as libc::c_int,
                  (*gpsRescueConfig()).throttleMax as libc::c_int) as
            uint16_t;
    if debugMode as libc::c_int == DEBUG_RTH as libc::c_int {
        debug[0 as libc::c_int as usize] = rescueThrottle as int16_t
    }
    if debugMode as libc::c_int == DEBUG_RTH as libc::c_int {
        debug[1 as libc::c_int as usize] =
            gpsRescueAngle[AI_PITCH as libc::c_int as usize] as int16_t
    }
    if debugMode as libc::c_int == DEBUG_RTH as libc::c_int {
        debug[2 as libc::c_int as usize] = altitudeAdjustment
    }
    if debugMode as libc::c_int == DEBUG_RTH as libc::c_int {
        debug[3 as libc::c_int as usize] = rescueState.failure as int16_t
    };
}
// Very similar to maghold function on betaflight/cleanflight
#[no_mangle]
pub unsafe extern "C" fn setBearing(mut desiredHeading: int16_t) {
    let mut errorAngle: libc::c_float =
        attitude.values.yaw as libc::c_int as libc::c_float / 10.0f32 -
            desiredHeading as libc::c_int as libc::c_float;
    // Determine the most efficient direction to rotate
    if errorAngle <= -(180 as libc::c_int) as libc::c_float {
        errorAngle += 360 as libc::c_int as libc::c_float
    } else if errorAngle > 180 as libc::c_int as libc::c_float {
        errorAngle -= 360 as libc::c_int as libc::c_float
    }
    errorAngle *=
        -if (*rcControlsConfig()).yaw_control_reversed as libc::c_int != 0 {
             -(1 as libc::c_int)
         } else { 1 as libc::c_int } as libc::c_float;
    // Calculate a desired yaw rate based on a maximum limit beyond
    // an error window and then scale the requested rate down inside
    // the window as error approaches 0.
    rescueYaw =
        -constrainf(errorAngle / 45 as libc::c_int as libc::c_float *
                        360 as libc::c_int as libc::c_float,
                    -(360 as libc::c_int) as libc::c_float,
                    360 as libc::c_int as libc::c_float);
}
#[no_mangle]
pub unsafe extern "C" fn gpsRescueGetYawRate() -> libc::c_float {
    return rescueYaw;
}
#[no_mangle]
pub unsafe extern "C" fn gpsRescueGetThrottle() -> libc::c_float {
    // Calculated a desired commanded throttle scaled from 0.0 to 1.0 for use in the mixer.
    // We need to compensate for min_check since the throttle value set by gps rescue
    // is based on the raw rcCommand value commanded by the pilot.
    let mut commandedThrottle: libc::c_float =
        scaleRangef(rescueThrottle as libc::c_float,
                    ({
                         let _a: uint16_t = (*rxConfig()).mincheck;
                         let mut _b: libc::c_int = 1000 as libc::c_int;
                         if _a as libc::c_int > _b {
                             _a as libc::c_int
                         } else { _b }
                     }) as libc::c_float,
                    2000 as libc::c_int as libc::c_float, 0.0f32, 1.0f32);
    commandedThrottle = constrainf(commandedThrottle, 0.0f32, 1.0f32);
    return commandedThrottle;
}
#[no_mangle]
pub unsafe extern "C" fn gpsRescueIsConfigured() -> bool {
    return (*failsafeConfig()).failsafe_procedure as libc::c_int ==
               FAILSAFE_PROCEDURE_GPS_RESCUE as libc::c_int ||
               isModeActivationConditionPresent(BOXGPSRESCUE) as libc::c_int
                   != 0;
}
