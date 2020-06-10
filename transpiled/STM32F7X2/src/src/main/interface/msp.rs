use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
    #[no_mangle]
    static buildTime: *const libc::c_char;
    #[no_mangle]
    static buildDate: *const libc::c_char;
    #[no_mangle]
    static shortGitRevision: *const libc::c_char;
    #[no_mangle]
    static targetName: *const libc::c_char;
    #[no_mangle]
    fn rtcGetDateTime(dt: *mut dateTime_t) -> bool;
    #[no_mangle]
    fn rtcSetDateTime(dt: *mut dateTime_t) -> bool;
    #[no_mangle]
    static mut blackboxConfig_System: blackboxConfig_t;
    #[no_mangle]
    fn blackboxCalculatePDenom(rateNum: libc::c_int, rateDenom: libc::c_int)
     -> libc::c_int;
    #[no_mangle]
    fn blackboxGetRateDenom() -> uint8_t;
    #[no_mangle]
    fn blackboxMayEditConfig() -> bool;
    #[no_mangle]
    static mut debug: [int16_t; 4];
    #[no_mangle]
    fn sbufWriteU8(dst: *mut sbuf_t, val: uint8_t);
    #[no_mangle]
    fn sbufWriteU16(dst: *mut sbuf_t, val: uint16_t);
    #[no_mangle]
    fn sbufWriteU32(dst: *mut sbuf_t, val: uint32_t);
    #[no_mangle]
    fn sbufWriteData(dst: *mut sbuf_t, data: *const libc::c_void,
                     len: libc::c_int);
    #[no_mangle]
    fn sbufWriteString(dst: *mut sbuf_t, string: *const libc::c_char);
    #[no_mangle]
    fn sbufReadU8(src: *mut sbuf_t) -> uint8_t;
    #[no_mangle]
    fn sbufReadU16(src: *mut sbuf_t) -> uint16_t;
    #[no_mangle]
    fn sbufReadU32(src: *mut sbuf_t) -> uint32_t;
    #[no_mangle]
    fn sbufReadData(dst: *mut sbuf_t, data: *mut libc::c_void,
                    len: libc::c_int);
    #[no_mangle]
    fn sbufBytesRemaining(buf: *mut sbuf_t) -> libc::c_int;
    #[no_mangle]
    fn sbufAdvance(buf: *mut sbuf_t, size: libc::c_int);
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    fn featureSet(mask: uint32_t);
    #[no_mangle]
    fn featureClearAll();
    #[no_mangle]
    fn featureMask() -> uint32_t;
    #[no_mangle]
    static mut gyroConfig_System: gyroConfig_t;
    #[no_mangle]
    fn gyroInitFilters();
    #[no_mangle]
    fn i2cGetErrorCounter() -> uint16_t;
    #[no_mangle]
    fn gyroRateDps(axis: libc::c_int) -> int16_t;
    #[no_mangle]
    fn cameraControlKeyPress(key: cameraControlKey_e,
                             holdDurationMs: uint32_t);
    #[no_mangle]
    fn pwmGetMotors() -> *mut pwmOutputPort_t;
    #[no_mangle]
    fn sdcard_isInserted() -> bool;
    #[no_mangle]
    fn sdcard_isInitialized() -> bool;
    #[no_mangle]
    fn sdcard_isFunctional() -> bool;
    #[no_mangle]
    fn sdcard_getMetadata() -> *const sdcardMetadata_t;
    // serialPort API
    #[no_mangle]
    fn escEnablePassthrough(escPassthroughPort: *mut serialPort_t,
                            output: uint16_t, mode: uint8_t);
    // bootloader/IAP
    #[no_mangle]
    fn systemReset();
    #[no_mangle]
    fn systemResetToBootloader();
    #[no_mangle]
    fn systemResetToMsc();
    #[no_mangle]
    fn vtxCommonDevice() -> *mut vtxDevice_t;
    #[no_mangle]
    fn vtxCommonGetDeviceType(vtxDevice: *const vtxDevice_t) -> vtxDevType_e;
    #[no_mangle]
    fn vtxCommonSetPitMode(vtxDevice: *mut vtxDevice_t, onoff: uint8_t);
    #[no_mangle]
    fn vtxCommonGetPitMode(vtxDevice: *const vtxDevice_t,
                           pOnOff: *mut uint8_t) -> bool;
    #[no_mangle]
    fn getBoardName() -> *mut libc::c_char;
    #[no_mangle]
    fn getManufacturerId() -> *mut libc::c_char;
    #[no_mangle]
    fn boardInformationIsSet() -> bool;
    #[no_mangle]
    fn setBoardName(newBoardName: *mut libc::c_char) -> bool;
    #[no_mangle]
    fn setManufacturerId(newManufacturerId: *mut libc::c_char) -> bool;
    #[no_mangle]
    fn persistBoardInformation() -> bool;
    #[no_mangle]
    fn getSignature() -> *mut uint8_t;
    #[no_mangle]
    fn signatureIsSet() -> bool;
    #[no_mangle]
    fn setSignature(newSignature: *mut uint8_t) -> bool;
    #[no_mangle]
    fn persistSignature() -> bool;
    #[no_mangle]
    static mut pilotConfig_System: pilotConfig_t;
    #[no_mangle]
    static mut systemConfig_System: systemConfig_t;
    #[no_mangle]
    static mut currentPidProfile: *mut pidProfile_s;
    #[no_mangle]
    fn resetEEPROM();
    #[no_mangle]
    fn readEEPROM() -> bool;
    #[no_mangle]
    fn writeEEPROM();
    #[no_mangle]
    fn validateAndFixGyroConfig();
    #[no_mangle]
    fn getCurrentPidProfileIndex() -> uint8_t;
    #[no_mangle]
    fn changePidProfile(pidProfileIndex: uint8_t);
    #[no_mangle]
    fn resetPidProfile(profile: *mut pidProfile_s);
    #[no_mangle]
    fn getCurrentControlRateProfileIndex() -> uint8_t;
    #[no_mangle]
    fn changeControlRateProfile(profileIndex: uint8_t);
    #[no_mangle]
    static mut currentControlRateProfile: *mut controlRateConfig_t;
    #[no_mangle]
    fn copyControlRateProfile(dstControlRateProfileIndex: uint8_t,
                              srcControlRateProfileIndex: uint8_t);
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
    static mut magHold: int16_t;
    #[no_mangle]
    fn disarm();
    #[no_mangle]
    fn runawayTakeoffTemporaryDisable(disableFlag: uint8_t);
    #[no_mangle]
    fn initRcProcessing();
    #[no_mangle]
    static mut modeActivationConditions_SystemArray:
           [modeActivationCondition_t; 20];
    #[no_mangle]
    static mut adjustmentRanges_SystemArray: [adjustmentRange_t; 15];
    #[no_mangle]
    static mut rcControlsConfig_System: rcControlsConfig_t;
    #[no_mangle]
    static mut flight3DConfig_System: flight3DConfig_t;
    #[no_mangle]
    static mut armingConfig_System: armingConfig_t;
    #[no_mangle]
    fn useRcControlsConfig(pidProfileToUse: *mut pidProfile_s);
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn setArmingDisabled(flag: armingDisableFlags_e);
    #[no_mangle]
    fn unsetArmingDisabled(flag: armingDisableFlags_e);
    #[no_mangle]
    fn getArmingDisableFlags() -> armingDisableFlags_e;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    fn getEstimatedAltitude() -> int32_t;
    #[no_mangle]
    fn getEstimatedVario() -> int16_t;
    #[no_mangle]
    static mut failsafeConfig_System: failsafeConfig_t;
    #[no_mangle]
    static mut attitude: attitudeEulerAngles_t;
    #[no_mangle]
    static mut imuConfig_System: imuConfig_t;
    #[no_mangle]
    static mut mixerConfig_System: mixerConfig_t;
    #[no_mangle]
    static mut motorConfig_System: motorConfig_t;
    #[no_mangle]
    static mut motor: [libc::c_float; 8];
    #[no_mangle]
    static mut motor_disarmed: [libc::c_float; 8];
    #[no_mangle]
    fn getMotorCount() -> uint8_t;
    #[no_mangle]
    fn stopPwmAllMotors();
    #[no_mangle]
    fn convertExternalToMotor(externalValue: uint16_t) -> libc::c_float;
    #[no_mangle]
    fn convertMotorToExternal(motorValue: libc::c_float) -> uint16_t;
    #[no_mangle]
    static mut pidConfig_System: pidConfig_t;
    #[no_mangle]
    static pidNames: [libc::c_char; 0];
    #[no_mangle]
    fn pidInitFilters(pidProfile: *const pidProfile_t);
    #[no_mangle]
    fn pidInitConfig(pidProfile: *const pidProfile_t);
    #[no_mangle]
    fn pidCopyProfile(dstPidProfileIndex: uint8_t,
                      srcPidProfileIndex: uint8_t);
    #[no_mangle]
    static mut customServoMixers_SystemArray: [servoMixer_t; 16];
    #[no_mangle]
    static mut servoParams_SystemArray: [servoParam_t; 8];
    #[no_mangle]
    static mut servo: [int16_t; 8];
    #[no_mangle]
    fn loadCustomServoMixer();
    #[no_mangle]
    fn findBoxByBoxId(boxId: boxId_e) -> *const box_t;
    #[no_mangle]
    fn findBoxByPermanentId(permanentId: uint8_t) -> *const box_t;
    #[no_mangle]
    fn packFlightModeFlags(mspFlightModeFlags: *mut boxBitmask_s)
     -> libc::c_int;
    #[no_mangle]
    fn serializeBoxNameFn(dst: *mut sbuf_s, box_0: *const box_t);
    #[no_mangle]
    fn serializeBoxPermanentIdFn(dst: *mut sbuf_s, box_0: *const box_t);
    #[no_mangle]
    fn serializeBoxReply(dst: *mut sbuf_s, page: libc::c_int,
                         serializeBox: Option<serializeBoxFn>);
    #[no_mangle]
    fn initActiveBoxIds();
    #[no_mangle]
    fn afatfs_getContiguousFreeSpace() -> uint32_t;
    #[no_mangle]
    fn afatfs_getLastError() -> afatfsError_e;
    #[no_mangle]
    fn afatfs_getFilesystemState() -> afatfsFilesystemState_e;
    #[no_mangle]
    static mut gpsConfig_System: gpsConfig_t;
    #[no_mangle]
    static mut GPS_distanceToHome: uint16_t;
    // distance to home point in meters
    #[no_mangle]
    static mut GPS_directionToHome: int16_t;
    #[no_mangle]
    static mut gpsSol: gpsSolutionData_t;
    #[no_mangle]
    static mut GPS_update: uint8_t;
    #[no_mangle]
    static mut GPS_numCh: uint8_t;
    // Number of channels
    #[no_mangle]
    static mut GPS_svinfo_chn: [uint8_t; 16];
    // Channel number
    #[no_mangle]
    static mut GPS_svinfo_svid: [uint8_t; 16];
    // Satellite ID
    #[no_mangle]
    static mut GPS_svinfo_quality: [uint8_t; 16];
    // Bitfield Qualtity
    #[no_mangle]
    static mut GPS_svinfo_cno: [uint8_t; 16];
    #[no_mangle]
    static mut ledStripConfig_System: ledStripConfig_t;
    #[no_mangle]
    fn reevaluateLedConfig();
    #[no_mangle]
    fn setModeColor(modeIndex: ledModeIndex_e, modeColorIndex: libc::c_int,
                    colorIndex: libc::c_int) -> bool;
    #[no_mangle]
    static mut osdConfig_System: osdConfig_t;
    #[no_mangle]
    fn osdStatSetState(statIndex: uint8_t, enabled: bool);
    #[no_mangle]
    fn osdStatGetState(statIndex: uint8_t) -> bool;
    #[no_mangle]
    static mut serialConfig_System: serialConfig_t;
    #[no_mangle]
    fn serialIsPortAvailable(identifier: serialPortIdentifier_e) -> bool;
    #[no_mangle]
    fn serialFindPortConfiguration(identifier: serialPortIdentifier_e)
     -> *mut serialPortConfig_t;
    #[no_mangle]
    fn esc4wayInit() -> uint8_t;
    #[no_mangle]
    fn esc4wayProcess(mspPort: *mut serialPort_s);
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
    fn mscCheckFilesystemReady() -> bool;
    #[no_mangle]
    static mut vtxSettingsConfig_System: vtxSettingsConfig_t;
    #[no_mangle]
    fn vtx58_Bandchan2Freq(band: uint8_t, channel: uint8_t) -> uint16_t;
    #[no_mangle]
    static mut beeperConfig_System: beeperConfig_t;
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    static mut usbDevConfig_System: usbDev_t;
    #[no_mangle]
    static mut rcData: [int16_t; 18];
    #[no_mangle]
    static mut rxFailsafeChannelConfigs_SystemArray:
           [rxFailsafeChannelConfig_t; 18];
    #[no_mangle]
    static mut rssiSource: rssiSource_e;
    #[no_mangle]
    static mut rxRuntimeConfig: rxRuntimeConfig_t;
    #[no_mangle]
    fn setRssiMsp(newMspRssi: uint8_t);
    #[no_mangle]
    fn getRssi() -> uint16_t;
    #[no_mangle]
    fn rxMspFrameReceive(frame: *mut uint16_t, channelCount: libc::c_int);
    #[no_mangle]
    static mut averageSystemLoadPercent: uint16_t;
    #[no_mangle]
    fn getTaskDeltaTime(taskId: cfTaskId_e) -> timeDelta_t;
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    fn getBatteryState() -> batteryState_e;
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
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
    // current read by current sensor in centiampere (1/100th A)
    // current read by current sensor in centiampere (1/100th A) (unfiltered)
    // milliampere hours drawn from the battery since start
    // WARNING - do not mix usage of CURRENT_SENSOR_* and CURRENT_METER_*, they are separate concerns.
    // milliampere hours drawn from the battery since start
    //
// Sensors
//
    //
// ADC
//
    // current read by current sensor in centiampere (1/100th A)
    // current read by current sensor in centiampere (1/100th A) (unfiltered)
    // scale the current sensor output voltage to milliamps. Value in mV/10A
    // offset of the current sensor in mA
    //
// Virtual
//
    // current read by current sensor in centiampere (1/100th A)
    // scale the current sensor output voltage to milliamps. Value in 1/10th mV/A
    // offset of the current sensor in millivolt steps
    #[no_mangle]
    static mut currentSensorVirtualConfig_System:
           currentSensorVirtualConfig_t;
    //
// Main API
//
    //
// API for reading/configuring current meters by id.
//
    #[no_mangle]
    static voltageMeterADCtoIDMap: [uint8_t; 1];
    #[no_mangle]
    static mut voltageSensorADCConfig_SystemArray:
           [voltageSensorADCConfig_t; 1];
    #[no_mangle]
    static mut currentSensorADCConfig_System: currentSensorADCConfig_t;
    #[no_mangle]
    static voltageMeterIds: [uint8_t; 0];
    #[no_mangle]
    fn voltageMeterRead(id: voltageMeterId_e,
                        voltageMeter: *mut voltageMeter_t);
    #[no_mangle]
    static supportedVoltageMeterCount: uint8_t;
    #[no_mangle]
    fn getAmperage() -> int32_t;
    #[no_mangle]
    fn getMAhDrawn() -> int32_t;
    #[no_mangle]
    fn getBatteryCellCount() -> uint8_t;
    //
// API for reading current meters by id.
//
    #[no_mangle]
    static supportedCurrentMeterCount: uint8_t;
    #[no_mangle]
    static currentMeterIds: [uint8_t; 0];
    #[no_mangle]
    fn currentMeterRead(id: currentMeterId_e,
                        currentMeter: *mut currentMeter_t);
    #[no_mangle]
    static mut acc: acc_t;
    #[no_mangle]
    static mut accelerometerConfig_System: accelerometerConfig_t;
    #[no_mangle]
    fn accSetCalibrationCycles(calibrationCyclesRequired: uint16_t);
    #[no_mangle]
    static mut barometerConfig_System: barometerConfig_t;
    #[no_mangle]
    static mut boardAlignment_System: boardAlignment_t;
    #[no_mangle]
    fn getEscSensorData(motorNumber: uint8_t) -> *mut escSensorData_t;
    #[no_mangle]
    static mut mag: mag_t;
    #[no_mangle]
    static mut compassConfig_System: compassConfig_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SMCR: uint32_t,
    pub DIER: uint32_t,
    pub SR: uint32_t,
    pub EGR: uint32_t,
    pub CCMR1: uint32_t,
    pub CCMR2: uint32_t,
    pub CCER: uint32_t,
    pub CNT: uint32_t,
    pub PSC: uint32_t,
    pub ARR: uint32_t,
    pub RCR: uint32_t,
    pub CCR1: uint32_t,
    pub CCR2: uint32_t,
    pub CCR3: uint32_t,
    pub CCR4: uint32_t,
    pub BDTR: uint32_t,
    pub DCR: uint32_t,
    pub DMAR: uint32_t,
    pub OR: uint32_t,
    pub CCMR3: uint32_t,
    pub CCR5: uint32_t,
    pub CCR6: uint32_t,
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
// time difference, 32 bits always sufficient
pub type timeDelta_t = int32_t;
// microsecond time
pub type timeUs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _dateTime_s {
    pub year: uint16_t,
    pub month: uint8_t,
    pub day: uint8_t,
    pub hours: uint8_t,
    pub minutes: uint8_t,
    pub seconds: uint8_t,
    pub millis: uint16_t,
}
pub type dateTime_t = _dateTime_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct blackboxConfig_s {
    pub p_ratio: uint16_t,
    pub device: uint8_t,
    pub record_acc: uint8_t,
    pub mode: uint8_t,
}
pub type blackboxConfig_t = blackboxConfig_s;
pub type C2RustUnnamed = libc::c_uint;
pub const FD_YAW: C2RustUnnamed = 2;
pub const FD_PITCH: C2RustUnnamed = 1;
pub const FD_ROLL: C2RustUnnamed = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct hsvColor_s {
    pub h: uint16_t,
    pub s: uint8_t,
    pub v: uint8_t,
}
pub type hsvColor_t = hsvColor_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sbuf_s {
    pub ptr: *mut uint8_t,
    pub end: *mut uint8_t,
}
// full year
// 1-12
// 1-31
// 0-23
// 0-59
// 0-59
// 0-999
// 0 - 359
// 0 - 255
// 0 - 255
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
// simple buffer-based serializer/deserializer without implicit size check
pub type sbuf_t = sbuf_s;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_0 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_0 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_0 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_0 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_0 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_0 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_0 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_0 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_0 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_0 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_0 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_0 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_0 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_0 = 8192;
pub const FEATURE_3D: C2RustUnnamed_0 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_0 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_0 = 512;
pub const FEATURE_GPS: C2RustUnnamed_0 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_0 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_0 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_0 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_0 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_0 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_0 = 1;
pub type ioTag_t = uint8_t;
// data pointer must be first (sbuf_t* is equivalent to uint8_t **)
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
pub type accDev_t = accDev_s;
pub type cameraControlKey_e = libc::c_uint;
pub const CAMERA_CONTROL_KEYS_COUNT: cameraControlKey_e = 5;
pub const CAMERA_CONTROL_KEY_DOWN: cameraControlKey_e = 4;
pub const CAMERA_CONTROL_KEY_RIGHT: cameraControlKey_e = 3;
pub const CAMERA_CONTROL_KEY_UP: cameraControlKey_e = 2;
pub const CAMERA_CONTROL_KEY_LEFT: cameraControlKey_e = 1;
pub const CAMERA_CONTROL_KEY_ENTER: cameraControlKey_e = 0;
// gyro alignment
// people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
// Gyro sample divider
// gyro DLPF setting
// gyro 32khz DLPF setting
// Lowpass primary/secondary
// Gyro calibration duration in 1/100 second
// bandpass quality factor, 100 for steep sided bandpass
// 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)
pub type timCCR_t = uint32_t;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const PWM_TYPE_MAX: C2RustUnnamed_2 = 10;
pub const PWM_TYPE_PROSHOT1000: C2RustUnnamed_2 = 9;
pub const PWM_TYPE_DSHOT1200: C2RustUnnamed_2 = 8;
pub const PWM_TYPE_DSHOT600: C2RustUnnamed_2 = 7;
pub const PWM_TYPE_DSHOT300: C2RustUnnamed_2 = 6;
pub const PWM_TYPE_DSHOT150: C2RustUnnamed_2 = 5;
pub const PWM_TYPE_BRUSHED: C2RustUnnamed_2 = 4;
pub const PWM_TYPE_MULTISHOT: C2RustUnnamed_2 = 3;
pub const PWM_TYPE_ONESHOT42: C2RustUnnamed_2 = 2;
pub const PWM_TYPE_ONESHOT125: C2RustUnnamed_2 = 1;
pub const PWM_TYPE_STANDARD: C2RustUnnamed_2 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerChannel_t {
    pub ccr: *mut timCCR_t,
    pub tim: *mut TIM_TypeDef,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pwmOutputPort_t {
    pub channel: timerChannel_t,
    pub pulseScale: libc::c_float,
    pub pulseOffset: libc::c_float,
    pub forceOverflow: bool,
    pub enabled: bool,
    pub io: IO_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorDevConfig_s {
    pub motorPwmRate: uint16_t,
    pub motorPwmProtocol: uint8_t,
    pub motorPwmInversion: uint8_t,
    pub useUnsyncedPwm: uint8_t,
    pub useBurstDshot: uint8_t,
    pub ioTags: [ioTag_t; 8],
}
pub type motorDevConfig_t = motorDevConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sdcardMetadata_s {
    pub numBlocks: uint32_t,
    pub oemID: uint16_t,
    pub manufacturerID: uint8_t,
    pub productName: [libc::c_char; 5],
    pub productSerial: uint32_t,
    pub productRevisionMajor: uint8_t,
    pub productRevisionMinor: uint8_t,
    pub productionYear: uint16_t,
    pub productionMonth: uint8_t,
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
pub type sdcardMetadata_t = sdcardMetadata_s;
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
/* Card capacity in 512-byte blocks*/
// Define known line control states which may be passed up by underlying serial driver callback
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
// used by serial drivers to return frames to app
pub type serialPort_t = serialPort_s;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const PROTOCOL_COUNT: C2RustUnnamed_3 = 5;
pub const PROTOCOL_CASTLE: C2RustUnnamed_3 = 4;
pub const PROTOCOL_KISSALL: C2RustUnnamed_3 = 3;
pub const PROTOCOL_KISS: C2RustUnnamed_3 = 2;
pub const PROTOCOL_BLHELI: C2RustUnnamed_3 = 1;
pub const PROTOCOL_SIMONK: C2RustUnnamed_3 = 0;
pub type vtxDevType_e = libc::c_uint;
pub const VTXDEV_UNKNOWN: vtxDevType_e = 255;
pub const VTXDEV_TRAMP: vtxDevType_e = 4;
pub const VTXDEV_SMARTAUDIO: vtxDevType_e = 3;
pub const VTXDEV_RTC6705: vtxDevType_e = 1;
pub const VTXDEV_UNSUPPORTED: vtxDevType_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vtxDeviceCapability_s {
    pub bandCount: uint8_t,
    pub channelCount: uint8_t,
    pub powerCount: uint8_t,
    pub filler: uint8_t,
}
// VTX magic numbers
// RTC6705 RF Power index "---", 25 or 200 mW
// SmartAudio "---", 25, 200, 500, 800 mW
// Tramp "---", 25, 100, 200, 400, 600 mW
pub type vtxDeviceCapability_t = vtxDeviceCapability_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vtxVTable_s {
    pub process: Option<unsafe extern "C" fn(_: *mut vtxDevice_t, _: timeUs_t)
                            -> ()>,
    pub getDeviceType: Option<unsafe extern "C" fn(_: *const vtxDevice_t)
                                  -> vtxDevType_e>,
    pub isReady: Option<unsafe extern "C" fn(_: *const vtxDevice_t) -> bool>,
    pub setBandAndChannel: Option<unsafe extern "C" fn(_: *mut vtxDevice_t,
                                                       _: uint8_t, _: uint8_t)
                                      -> ()>,
    pub setPowerByIndex: Option<unsafe extern "C" fn(_: *mut vtxDevice_t,
                                                     _: uint8_t) -> ()>,
    pub setPitMode: Option<unsafe extern "C" fn(_: *mut vtxDevice_t,
                                                _: uint8_t) -> ()>,
    pub setFrequency: Option<unsafe extern "C" fn(_: *mut vtxDevice_t,
                                                  _: uint16_t) -> ()>,
    pub getBandAndChannel: Option<unsafe extern "C" fn(_: *const vtxDevice_t,
                                                       _: *mut uint8_t,
                                                       _: *mut uint8_t)
                                      -> bool>,
    pub getPowerIndex: Option<unsafe extern "C" fn(_: *const vtxDevice_t,
                                                   _: *mut uint8_t) -> bool>,
    pub getPitMode: Option<unsafe extern "C" fn(_: *const vtxDevice_t,
                                                _: *mut uint8_t) -> bool>,
    pub getFrequency: Option<unsafe extern "C" fn(_: *const vtxDevice_t,
                                                  _: *mut uint16_t) -> bool>,
}
pub type vtxDevice_t = vtxDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vtxDevice_s {
    pub vTable: *const vtxVTable_s,
    pub capability: vtxDeviceCapability_t,
    pub frequencyTable: *mut uint16_t,
    pub bandNames: *mut *mut libc::c_char,
    pub channelNames: *mut *mut libc::c_char,
    pub powerNames: *mut *mut libc::c_char,
    pub frequency: uint16_t,
    pub band: uint8_t,
    pub channel: uint8_t,
    pub powerIndex: uint8_t,
    pub pitMode: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pilotConfig_s {
    pub name: [libc::c_char; 17],
}
// Array of [bandCount][channelCount]
// char *bandNames[bandCount]
// char *channelNames[channelCount]
// char *powerNames[powerCount]
// Band = 1, 1-based
// CH1 = 1, 1-based
// Lowest/Off = 0
// 0 = non-PIT, 1 = PIT
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
pub type pilotConfig_t = pilotConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct systemConfig_s {
    pub pidProfileIndex: uint8_t,
    pub activeRateProfile: uint8_t,
    pub debug_mode: uint8_t,
    pub task_statistics: uint8_t,
    pub rateProfile6PosSwitch: uint8_t,
    pub cpu_overclock: uint8_t,
    pub powerOnArmingGraceTime: uint8_t,
    pub boardIdentifier: [libc::c_char; 6],
}
pub type systemConfig_t = systemConfig_s;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub union rollAndPitchTrims_u {
    pub raw: [int16_t; 2],
    pub values: rollAndPitchTrims_t_def,
}
pub type rollAndPitchTrims_t_def = rollAndPitchTrims_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rollAndPitchTrims_s {
    pub roll: int16_t,
    pub pitch: int16_t,
}
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
pub type modeLogic_e = libc::c_uint;
pub const MODELOGIC_AND: modeLogic_e = 1;
pub const MODELOGIC_OR: modeLogic_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct boxBitmask_s {
    pub bits: [uint32_t; 2],
}
// in seconds
// Breakpoint where TPA is activated
// Sets the throttle limiting type - off, scale or clip
// Sets the maximum pilot commanded throttle limit
// type to hold enough bits for CHECKBOX_ITEM_COUNT. Struct used for value-like behavior
pub type boxBitmask_t = boxBitmask_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct channelRange_s {
    pub startStep: uint8_t,
    pub endStep: uint8_t,
}
// steps are 25 apart
// a value of 0 corresponds to a channel value of 900 or less
// a value of 48 corresponds to a channel value of 2100 or more
// 48 steps between 900 and 2100
pub type channelRange_t = channelRange_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct modeActivationCondition_s {
    pub modeId: boxId_e,
    pub auxChannelIndex: uint8_t,
    pub range: channelRange_t,
    pub modeLogic: modeLogic_e,
    pub linkedTo: boxId_e,
}
pub type modeActivationCondition_t = modeActivationCondition_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adjustmentRange_s {
    pub auxChannelIndex: uint8_t,
    pub range: channelRange_t,
    pub adjustmentFunction: uint8_t,
    pub auxSwitchChannelIndex: uint8_t,
    pub adjustmentIndex: uint8_t,
    pub adjustmentCenter: uint16_t,
    pub adjustmentScale: uint16_t,
}
pub type adjustmentRange_t = adjustmentRange_s;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct flight3DConfig_s {
    pub deadband3d_low: uint16_t,
    pub deadband3d_high: uint16_t,
    pub neutral3d: uint16_t,
    pub deadband3d_throttle: uint16_t,
    pub limit3d_low: uint16_t,
    pub limit3d_high: uint16_t,
    pub switched_mode3d: uint8_t,
}
pub type flight3DConfig_t = flight3DConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct armingConfig_s {
    pub gyro_cal_on_first_arm: uint8_t,
    pub auto_disarm_delay: uint8_t,
}
pub type armingConfig_t = armingConfig_s;
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
pub const FIXED_WING: C2RustUnnamed_5 = 16;
pub const SMALL_ANGLE: C2RustUnnamed_5 = 8;
pub const CALIBRATE_MAG: C2RustUnnamed_5 = 4;
pub const GPS_FIX: C2RustUnnamed_5 = 2;
pub const GPS_FIX_HOME: C2RustUnnamed_5 = 1;
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
// introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
// introduce a deadband around the stick center for yaw axis. Must be greater than zero.
// defines the neutral zone of throttle stick during altitude hold, default setting is +/-40
// when disabled, turn off the althold when throttle stick is out of deadband defined with alt_hold_deadband; when enabled, altitude changes slowly proportional to stick movement
// invert control direction of yaw
// min 3d value
// max 3d value
// center 3d value
// default throttle deadband from MIDRC
// pwm output value for max negative thrust
// pwm output value for max positive thrust
// enable '3D Switched Mode'
// allow disarm/arm on throttle down + roll left/right
// allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct accDeadband_s {
    pub xy: uint8_t,
    pub z: uint8_t,
}
pub type accDeadband_t = accDeadband_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct imuConfig_s {
    pub dcm_kp: uint16_t,
    pub dcm_ki: uint16_t,
    pub small_angle: uint8_t,
    pub acc_unarmedcal: uint8_t,
    pub accDeadband: accDeadband_t,
}
pub type imuConfig_t = imuConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mixerConfig_s {
    pub mixerMode: uint8_t,
    pub yaw_motors_reversed: bool,
    pub crashflip_motor_percent: uint8_t,
}
pub type mixerConfig_t = mixerConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorConfig_s {
    pub dev: motorDevConfig_t,
    pub digitalIdleOffsetValue: uint16_t,
    pub minthrottle: uint16_t,
    pub maxthrottle: uint16_t,
    pub mincommand: uint16_t,
    pub motorPoleCount: uint8_t,
}
pub type motorConfig_t = motorConfig_s;
// Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.
// Time throttle stick must have been below 'min_check' to "JustDisarm" instead of "full failsafe procedure".
// Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
// Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example (200)
// failsafe switch action is 0: stage1 (identical to rc link loss), 1: disarms instantly, 2: stage2
// selected full failsafe procedure is 0: auto-landing, 1: Drop it
// set the acc deadband for xy-Axis
// set the acc deadband for z-Axis, this ignores small accelerations
// DCM filter proportional gain ( x 10000)
// DCM filter integral gain ( x 10000)
// turn automatic acc compensation on/off
// Idle value for DShot protocol, full motor output = 10000
// Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
// This is the maximum value for the ESCs at full power this value can be increased up to 2000
// This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
// Magnetic poles in the motors for calculating actual RPM from eRPM provided by ESC telemetry
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
pub type C2RustUnnamed_7 = libc::c_uint;
pub const PID_ITEM_COUNT: C2RustUnnamed_7 = 5;
pub const PID_MAG: C2RustUnnamed_7 = 4;
pub const PID_LEVEL: C2RustUnnamed_7 = 3;
pub const PID_YAW: C2RustUnnamed_7 = 2;
pub const PID_PITCH: C2RustUnnamed_7 = 1;
pub const PID_ROLL: C2RustUnnamed_7 = 0;
pub type pidProfile_t = pidProfile_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidConfig_s {
    pub pid_process_denom: uint8_t,
    pub runaway_takeoff_prevention: uint8_t,
    pub runaway_takeoff_deactivate_delay: uint16_t,
    pub runaway_takeoff_deactivate_throttle: uint8_t,
}
pub type pidConfig_t = pidConfig_s;
// Additional yaw filter when yaw axis too noisy
// Delta Filter in hz
// Biquad dterm notch hz
// Biquad dterm notch low cutoff
// Filter selection for dterm
// Experimental ITerm windup threshold, percent motor saturation
// Disable/Enable pids on zero throttle. Normally even without airmode P and D would be active.
// Max angle in degrees in level mode
// inclination factor for Horizon mode
// OFF or ON
// Betaflight PID controller parameters
// type of anti gravity method
// max allowed throttle delta before iterm accelerated in ms
// Iterm Accelerator Gain when itermThrottlethreshold is hit
// yaw accel limiter for deg/sec/ms
// accel limiter roll/pitch deg/sec/ms
// dterm crash value
// gyro crash value
// setpoint must be below this value to detect crash, so flips and rolls are not interpreted as crashes
// ms
// ms
// degrees
// degree/second
// Scale PIDsum to battery voltage
// Feed forward weight transition
// limits yaw errorRate, so crashes don't cause huge throttle increase
// Extra PT1 Filter on D in hz
// off, on, on and beeps when it is in crash recovery mode
// how much should throttle be boosted during transient changes 0-100, 100 adds 10x hpf filtered throttle
// Which cutoff frequency to use for throttle boost. higher cutoffs keep the boost on for shorter. Specified in hz.
// rotates iterm to translate world errors to local coordinate system
// takes only the larger of P and the D weight feed forward term if they have the same sign.
// Specifies type of relax algorithm
// This cutoff frequency specifies a low pass filter which predicts average response of the quad to setpoint
// Enable iterm suppression during stick input
// Acro trainer roll/pitch angle limit in degrees
// The axis for which record debugging values are captured 0=roll, 1=pitch
// The strength of the limiting. Raising may reduce overshoot but also lead to oscillation around the angle limit
// The lookahead window in milliseconds used to reduce overshoot
// How strongly should the absolute accumulated error be corrected for
// Limit to the correction
// Limit to the accumulated error
// Processing denominator for PID controller vs gyro sampling rate
// off, on - enables pidsum runaway disarm logic
// delay in ms for "in-flight" conditions before deactivation (successful flight)
// minimum throttle percent required during deactivation phase
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
// These must be consecutive, see 'reversedSources'
pub type C2RustUnnamed_8 = libc::c_uint;
pub const INPUT_SOURCE_COUNT: C2RustUnnamed_8 = 14;
pub const INPUT_GIMBAL_ROLL: C2RustUnnamed_8 = 13;
pub const INPUT_GIMBAL_PITCH: C2RustUnnamed_8 = 12;
pub const INPUT_RC_AUX4: C2RustUnnamed_8 = 11;
pub const INPUT_RC_AUX3: C2RustUnnamed_8 = 10;
pub const INPUT_RC_AUX2: C2RustUnnamed_8 = 9;
pub const INPUT_RC_AUX1: C2RustUnnamed_8 = 8;
pub const INPUT_RC_THROTTLE: C2RustUnnamed_8 = 7;
pub const INPUT_RC_YAW: C2RustUnnamed_8 = 6;
pub const INPUT_RC_PITCH: C2RustUnnamed_8 = 5;
pub const INPUT_RC_ROLL: C2RustUnnamed_8 = 4;
pub const INPUT_STABILIZED_THROTTLE: C2RustUnnamed_8 = 3;
pub const INPUT_STABILIZED_YAW: C2RustUnnamed_8 = 2;
pub const INPUT_STABILIZED_PITCH: C2RustUnnamed_8 = 1;
pub const INPUT_STABILIZED_ROLL: C2RustUnnamed_8 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct servoMixer_s {
    pub targetChannel: uint8_t,
    pub inputSource: uint8_t,
    pub rate: int8_t,
    pub speed: uint8_t,
    pub min: int8_t,
    pub max: int8_t,
    pub box_0: uint8_t,
}
pub type servoMixer_t = servoMixer_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct servoParam_s {
    pub reversedSources: uint32_t,
    pub min: int16_t,
    pub max: int16_t,
    pub middle: int16_t,
    pub rate: int8_t,
    pub forwardFromChannel: int8_t,
}
pub type servoParam_t = servoParam_s;
pub type mspResult_e = libc::c_int;
pub const MSP_RESULT_CMD_UNKNOWN: mspResult_e = -2;
pub const MSP_RESULT_NO_REPLY: mspResult_e = 0;
pub const MSP_RESULT_ERROR: mspResult_e = -1;
pub const MSP_RESULT_ACK: mspResult_e = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mspPacket_s {
    pub buf: sbuf_t,
    pub cmd: int16_t,
    pub flags: uint8_t,
    pub result: int16_t,
    pub direction: uint8_t,
}
pub type mspPacket_t = mspPacket_s;
pub type mspPostProcessFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut serialPort_s) -> ()>;
pub type ledModeIndex_e = libc::c_uint;
pub const LED_AUX_CHANNEL: ledModeIndex_e = 7;
pub const LED_SPECIAL: ledModeIndex_e = 6;
pub const LED_MODE_BARO: ledModeIndex_e = 5;
pub const LED_MODE_MAG: ledModeIndex_e = 4;
pub const LED_MODE_ANGLE: ledModeIndex_e = 3;
pub const LED_MODE_HORIZON: ledModeIndex_e = 2;
pub const LED_MODE_HEADFREE: ledModeIndex_e = 1;
pub const LED_MODE_ORIENTATION: ledModeIndex_e = 0;
pub type ledConfig_t = uint32_t;
pub type ledStripConfig_t = ledStripConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ledStripConfig_s {
    pub ledConfigs: [ledConfig_t; 32],
    pub colors: [hsvColor_t; 16],
    pub modeColors: [modeColorIndexes_t; 6],
    pub specialColors: specialColorIndexes_t,
    pub ledstrip_visual_beeper: uint8_t,
    pub ledstrip_aux_channel: uint8_t,
    pub ioTag: ioTag_t,
    pub ledstrip_grb_rgb: ledStripFormatRGB_e,
}
pub type ledStripFormatRGB_e = libc::c_uint;
pub const LED_RGB: ledStripFormatRGB_e = 1;
pub const LED_GRB: ledStripFormatRGB_e = 0;
pub type specialColorIndexes_t = specialColorIndexes_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct specialColorIndexes_s {
    pub color: [uint8_t; 11],
}
pub type modeColorIndexes_t = modeColorIndexes_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct modeColorIndexes_s {
    pub color: [uint8_t; 6],
}
// servo that receives the output of the rule
// input channel for this rule
// range [-125;+125] ; can be used to adjust a rate 0-125% and a direction
// reduces the speed of the rule, 0=unlimited speed
// lower bound of rule range [0;100]% of servo max-min
// lower bound of rule range [0;100]% of servo max-min
// active rule if box is enabled, range [0;3], 0=no box, 1=BOXSERVO1, 2=BOXSERVO2, 3=BOXSERVO3
// the direction of servo movement for each input source of the servo mixer, bit set=inverted
// servo min
// servo max
// servo middle
// range [-125;+125] ; can be used to adjust a rate 0-125% and a direction
// RX channel index, 0 based.  See CHANNEL_FORWARDING_DISABLED
// suppress LEDLOW mode if beeper is on
//
// configuration
//
pub type serialPortConfig_t = serialPortConfig_s;
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
// not used for all telemetry systems, e.g. HoTT only works at 19200.
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
pub type rxConfig_t = rxConfig_s;
pub type rxFailsafeChannelConfig_t = rxFailsafeChannelConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxFailsafeChannelConfig_s {
    pub mode: uint8_t,
    pub step: uint8_t,
}
pub type usbDev_t = usbDev_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct usbDev_s {
    pub type_0: uint8_t,
    pub mscButtonPin: ioTag_t,
    pub mscButtonUsePullup: uint8_t,
}
// mapping of radio channels to internal RPYTA+ order
// type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
// invert the serial RX protocol compared to it's default setting
// allow rx to operate in half duplex mode on F4, ignored for F1 and F3.
// number of bind pulses for Spektrum satellite receivers
// whenever we will reset (exit) binding mode after hard reboot
// Some radios have not a neutral point centered on 1500. can be changed here
// minimum rc end
// maximum rc end
// Camera angle to be scaled into rc commands
// Throttle setpoint percent where airmode gets activated
// true to use frame drop flags in the rx protocol
// offset applied to the RSSI value before it is returned
// Determines the smoothing algorithm to use: INTERPOLATION or FILTER
// Filter cutoff frequency for the input filter (0 = auto)
// Filter cutoff frequency for the setpoint weight derivative filter (0 = auto)
// Axis to log as debug values when debug_mode = RC_SMOOTHING
// Input filter type (0 = PT1, 1 = BIQUAD)
// Derivative filter type (0 = OFF, 1 = PT1, 2 = BIQUAD)
// See rxFailsafeChannelMode_e
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
pub type boardAlignment_t = boardAlignment_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct boardAlignment_s {
    pub rollDegrees: int32_t,
    pub pitchDegrees: int32_t,
    pub yawDegrees: int32_t,
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
pub type beeperConfig_t = beeperConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct beeperConfig_s {
    pub beeper_off_flags: uint32_t,
    pub dshotBeaconTone: uint8_t,
    pub dshotBeaconOffFlags: uint32_t,
}
pub type gpsSolutionData_t = gpsSolutionData_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsSolutionData_s {
    pub llh: gpsLocation_t,
    pub groundSpeed: uint16_t,
    pub groundCourse: uint16_t,
    pub hdop: uint16_t,
    pub numSat: uint8_t,
}
// speed in 0.1m/s
// degrees * 10
// generic HDOP value (*100)
/* LLH Location in NEU axis system */
pub type gpsLocation_t = gpsLocation_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsLocation_s {
    pub lat: int32_t,
    pub lon: int32_t,
    pub alt: int32_t,
}
// latitude * 1e+7
// longitude * 1e+7
// altitude in 0.01m
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
pub type vtxSettingsConfig_t = vtxSettingsConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vtxSettingsConfig_s {
    pub band: uint8_t,
    pub channel: uint8_t,
    pub power: uint8_t,
    pub freq: uint16_t,
    pub pitModeFreq: uint16_t,
    pub lowPowerDisarm: uint8_t,
}
pub type compassConfig_t = compassConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct compassConfig_s {
    pub mag_declination: int16_t,
    pub mag_align: sensor_align_e,
    pub mag_hardware: uint8_t,
    pub mag_bustype: uint8_t,
    pub mag_i2c_device: uint8_t,
    pub mag_i2c_address: uint8_t,
    pub mag_spi_device: uint8_t,
    pub mag_spi_csn: ioTag_t,
    pub interruptTag: ioTag_t,
    pub magZero: flightDynamicsTrims_t,
}
pub type flightDynamicsTrims_t = flightDynamicsTrims_u;
#[derive(Copy, Clone)]
#[repr(C)]
pub union flightDynamicsTrims_u {
    pub raw: [int16_t; 3],
    pub values: flightDynamicsTrims_def_t,
}
pub type flightDynamicsTrims_def_t = int16_flightDynamicsTrims_s;
// 1=A, 2=B, 3=E, 4=F(Airwaves/Fatshark), 5=Raceband
// 1-8
// 0 = lowest
// sets freq in MHz if band=0
// sets out-of-range pitmode frequency
// min power while disarmed
// Get your magnetic decliniation from here : http://magnetic-declination.com/
                                            // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
// mag alignment
// Which mag hardware to use on boards with more than one device
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct int16_flightDynamicsTrims_s {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
pub type barometerConfig_t = barometerConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct barometerConfig_s {
    pub baro_bustype: uint8_t,
    pub baro_spi_device: uint8_t,
    pub baro_spi_csn: ioTag_t,
    pub baro_i2c_device: uint8_t,
    pub baro_i2c_address: uint8_t,
    pub baro_hardware: uint8_t,
    pub baro_sample_count: uint8_t,
    pub baro_noise_lpf: uint16_t,
    pub baro_cf_vel: uint16_t,
    pub baro_cf_alt: uint16_t,
}
pub type accelerometerConfig_t = accelerometerConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct accelerometerConfig_s {
    pub acc_lpf_hz: uint16_t,
    pub acc_align: sensor_align_e,
    pub acc_hardware: uint8_t,
    pub acc_high_fsr: bool,
    pub accZero: flightDynamicsTrims_t,
    pub accelerometerTrims: rollAndPitchTrims_t,
}
pub type rollAndPitchTrims_t = rollAndPitchTrims_u;
pub type gpsAutoBaud_e = libc::c_uint;
pub const GPS_AUTOBAUD_ON: gpsAutoBaud_e = 1;
pub const GPS_AUTOBAUD_OFF: gpsAutoBaud_e = 0;
pub type gpsConfig_t = gpsConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsConfig_s {
    pub provider: gpsProvider_e,
    pub sbasMode: sbasMode_e,
    pub autoConfig: gpsAutoConfig_e,
    pub autoBaud: gpsAutoBaud_e,
    pub gps_ublox_use_galileo: uint8_t,
}
pub type gpsAutoConfig_e = libc::c_uint;
pub const GPS_AUTOCONFIG_ON: gpsAutoConfig_e = 1;
pub const GPS_AUTOCONFIG_OFF: gpsAutoConfig_e = 0;
pub type sbasMode_e = libc::c_uint;
pub const SBAS_GAGAN: sbasMode_e = 4;
pub const SBAS_MSAS: sbasMode_e = 3;
pub const SBAS_WAAS: sbasMode_e = 2;
pub const SBAS_EGNOS: sbasMode_e = 1;
pub const SBAS_AUTO: sbasMode_e = 0;
pub type gpsProvider_e = libc::c_uint;
pub const GPS_UBLOX: gpsProvider_e = 1;
pub const GPS_NMEA: gpsProvider_e = 0;
// Also used as XCLR (positive logic) for BMP085
// Barometer hardware to use
// size of baro filter array
// additional LPF to reduce baro noise
// apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)
// apply CF to use ACC for height estimation
// cutoff frequency for the low pass filter used on the acc z-axis for althold in Hz
// acc alignment
// Which acc hardware to use on boards with more than one device
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
pub type box_t = box_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct box_s {
    pub boxId: uint8_t,
    pub boxName: *const libc::c_char,
    pub permanentId: uint8_t,
}
pub type osdConfig_t = osdConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct osdConfig_s {
    pub item_pos: [uint16_t; 43],
    pub cap_alarm: uint16_t,
    pub alt_alarm: uint16_t,
    pub rssi_alarm: uint8_t,
    pub units: osd_unit_e,
    pub timers: [uint16_t; 2],
    pub enabledWarnings: uint16_t,
    pub ahMaxPitch: uint8_t,
    pub ahMaxRoll: uint8_t,
    pub enabled_stats: uint32_t,
    pub esc_temp_alarm: int8_t,
    pub esc_rpm_alarm: int16_t,
    pub esc_current_alarm: int16_t,
    pub core_temp_alarm: uint8_t,
}
pub type osd_unit_e = libc::c_uint;
pub const OSD_UNIT_METRIC: osd_unit_e = 1;
pub const OSD_UNIT_IMPERIAL: osd_unit_e = 0;
pub const OSD_ITEM_COUNT: C2RustUnnamed_9 = 43;
pub const OSD_STAT_COUNT: C2RustUnnamed_10 = 14;
pub const OSD_TIMER_COUNT: C2RustUnnamed_11 = 2;
pub type currentMeterSource_e = libc::c_uint;
pub const CURRENT_METER_COUNT: currentMeterSource_e = 5;
pub const CURRENT_METER_MSP: currentMeterSource_e = 4;
pub const CURRENT_METER_ESC: currentMeterSource_e = 3;
pub const CURRENT_METER_VIRTUAL: currentMeterSource_e = 2;
pub const CURRENT_METER_ADC: currentMeterSource_e = 1;
pub const CURRENT_METER_NONE: currentMeterSource_e = 0;
// see boxId_e
// GUI-readable box name
// permanent ID used to identify BOX. This ID is unique for one function, DO NOT REUSE IT
// Alarms
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
pub type batteryConfig_t = batteryConfig_s;
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
pub type voltageMeterSource_e = libc::c_uint;
pub const VOLTAGE_METER_COUNT: voltageMeterSource_e = 3;
pub const VOLTAGE_METER_ESC: voltageMeterSource_e = 2;
pub const VOLTAGE_METER_ADC: voltageMeterSource_e = 1;
pub const VOLTAGE_METER_NONE: voltageMeterSource_e = 0;
pub type currentSensorVirtualConfig_t = currentSensorVirtualConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct currentSensorVirtualConfig_s {
    pub scale: int16_t,
    pub offset: uint16_t,
}
pub const CURRENT_METER_ID_VIRTUAL_1: currentMeterId_e = 80;
pub type currentSensorADCConfig_t = currentSensorADCConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct currentSensorADCConfig_s {
    pub scale: int16_t,
    pub offset: int16_t,
}
pub const CURRENT_METER_ID_BATTERY_1: currentMeterId_e = 10;
pub type voltageSensorADCConfig_t = voltageSensorADCConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct voltageSensorADCConfig_s {
    pub vbatscale: uint8_t,
    pub vbatresdivval: uint8_t,
    pub vbatresdivmultiplier: uint8_t,
}
pub const MSP_REBOOT_MSC: C2RustUnnamed_16 = 2;
pub const MSP_REBOOT_BOOTLOADER: C2RustUnnamed_16 = 1;
pub const MSP_REBOOT_FIRMWARE: C2RustUnnamed_16 = 0;
pub const MSP_REBOOT_COUNT: C2RustUnnamed_16 = 3;
pub type serializeBoxFn
    =
    unsafe extern "C" fn(_: *mut sbuf_s, _: *const box_t) -> ();
pub type rssiSource_e = libc::c_uint;
pub const RSSI_SOURCE_FRAME_ERRORS: rssiSource_e = 5;
pub const RSSI_SOURCE_MSP: rssiSource_e = 4;
pub const RSSI_SOURCE_RX_PROTOCOL: rssiSource_e = 3;
pub const RSSI_SOURCE_RX_CHANNEL: rssiSource_e = 2;
pub const RSSI_SOURCE_ADC: rssiSource_e = 1;
pub const RSSI_SOURCE_NONE: rssiSource_e = 0;
pub type afatfsError_e = libc::c_uint;
pub const AFATFS_ERROR_BAD_FILESYSTEM_HEADER: afatfsError_e = 3;
pub const AFATFS_ERROR_BAD_MBR: afatfsError_e = 2;
pub const AFATFS_ERROR_GENERIC: afatfsError_e = 1;
pub const AFATFS_ERROR_NONE: afatfsError_e = 0;
pub const MSP_SDCARD_STATE_FATAL: C2RustUnnamed_17 = 1;
pub const AFATFS_FILESYSTEM_STATE_UNKNOWN: afatfsFilesystemState_e = 0;
pub const AFATFS_FILESYSTEM_STATE_FATAL: afatfsFilesystemState_e = 1;
pub const MSP_SDCARD_STATE_CARD_INIT: C2RustUnnamed_17 = 2;
pub const MSP_SDCARD_STATE_FS_INIT: C2RustUnnamed_17 = 3;
pub const AFATFS_FILESYSTEM_STATE_INITIALIZATION: afatfsFilesystemState_e = 2;
pub const MSP_SDCARD_STATE_READY: C2RustUnnamed_17 = 4;
pub const AFATFS_FILESYSTEM_STATE_READY: afatfsFilesystemState_e = 3;
pub type afatfsFilesystemState_e = libc::c_uint;
pub const MSP_SDCARD_STATE_NOT_PRESENT: C2RustUnnamed_17 = 0;
pub const MSP_SDCARD_FLAG_SUPPORTTED: C2RustUnnamed_18 = 1;
pub type serialConfig_t = serialConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialConfig_s {
    pub portConfigs: [serialPortConfig_t; 9],
    pub serial_update_rate_hz: uint16_t,
    pub reboot_character: uint8_t,
}
pub type rxRuntimeConfig_t = rxRuntimeConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxRuntimeConfig_s {
    pub channelCount: uint8_t,
    pub rxRefreshRate: uint16_t,
    pub rcReadRawFn: rcReadRawDataFnPtr,
    pub rcFrameStatusFn: rcFrameStatusFnPtr,
    pub rcProcessFrameFn: rcProcessFrameFnPtr,
    pub channelData: *mut uint16_t,
    pub frameData: *mut libc::c_void,
}
pub type rcProcessFrameFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s) -> bool>;
// voltage
// maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
// minimum voltage per cell, this triggers battery critical alarm, in 0.1V units, default is 33 (3.3V)
// warning voltage per cell, this triggers battery warning alarm, in 0.1V units, default is 35 (3.5V)
// Between vbatmaxcellvoltage and 2*this is considered to be USB powered. Below this it is notpresent
// Percentage of throttle when lvc is triggered
// source of battery voltage meter used, either ADC or ESC
// current
// source of battery current meter used, either ADC, Virtual or ESC
// mAh
// warnings / alerts
// Issue alerts based on VBat readings
// Issue alerts based on total power consumption
// Percentage of remaining capacity that should trigger a battery warning
// hysteresis for alarm, default 1 = 0.1V
// Cell voltage at which the battery is deemed to be "full" 0.1V units, default is 41 (4.1V)
// which byte is used to reboot. Default 'R', could be changed carefully to something else.
// number of RC channels as reported by current input driver
// used by receiver driver to return channel data
pub type rcFrameStatusFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut rxRuntimeConfig_s) -> uint8_t>;
pub type rcReadRawDataFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s, _: uint8_t)
               -> uint16_t>;
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
pub type mag_t = mag_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mag_s {
    pub magADC: [libc::c_float; 3],
    pub magneticDeclination: libc::c_float,
}
pub type acc_t = acc_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct acc_s {
    pub dev: accDev_t,
    pub accSamplingInterval: uint32_t,
    pub accADC: [libc::c_float; 3],
    pub isAccelUpdatedAtLeastOnce: bool,
}
pub const SENSOR_GYRO: C2RustUnnamed_15 = 1;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_15 = 16;
pub const SENSOR_GPS: C2RustUnnamed_15 = 32;
pub const SENSOR_MAG: C2RustUnnamed_15 = 8;
pub const SENSOR_BARO: C2RustUnnamed_15 = 4;
pub const SENSOR_ACC: C2RustUnnamed_15 = 2;
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
pub const CURRENT_SENSOR_VIRTUAL: C2RustUnnamed_12 = 0;
pub const CURRENT_SENSOR_ADC: C2RustUnnamed_12 = 1;
pub const VOLTAGE_SENSOR_ADC_VBAT: C2RustUnnamed_14 = 0;
pub const VOLTAGE_SENSOR_TYPE_ADC_RESISTOR_DIVIDER: C2RustUnnamed_13 = 0;
pub type currentMeter_t = currentMeter_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct currentMeter_s {
    pub amperage: int32_t,
    pub amperageLatest: int32_t,
    pub mAhDrawn: int32_t,
}
pub type currentMeterId_e = libc::c_uint;
pub const CURRENT_METER_ID_MSP_2: currentMeterId_e = 91;
pub const CURRENT_METER_ID_MSP_1: currentMeterId_e = 90;
pub const CURRENT_METER_ID_VIRTUAL_2: currentMeterId_e = 81;
pub const CURRENT_METER_ID_ESC_MOTOR_20: currentMeterId_e = 79;
pub const CURRENT_METER_ID_ESC_MOTOR_12: currentMeterId_e = 71;
pub const CURRENT_METER_ID_ESC_MOTOR_11: currentMeterId_e = 70;
pub const CURRENT_METER_ID_ESC_MOTOR_10: currentMeterId_e = 69;
pub const CURRENT_METER_ID_ESC_MOTOR_9: currentMeterId_e = 68;
pub const CURRENT_METER_ID_ESC_MOTOR_8: currentMeterId_e = 67;
pub const CURRENT_METER_ID_ESC_MOTOR_7: currentMeterId_e = 66;
pub const CURRENT_METER_ID_ESC_MOTOR_6: currentMeterId_e = 65;
pub const CURRENT_METER_ID_ESC_MOTOR_5: currentMeterId_e = 64;
pub const CURRENT_METER_ID_ESC_MOTOR_4: currentMeterId_e = 63;
pub const CURRENT_METER_ID_ESC_MOTOR_3: currentMeterId_e = 62;
pub const CURRENT_METER_ID_ESC_MOTOR_2: currentMeterId_e = 61;
pub const CURRENT_METER_ID_ESC_MOTOR_1: currentMeterId_e = 60;
pub const CURRENT_METER_ID_ESC_COMBINED_10: currentMeterId_e = 59;
pub const CURRENT_METER_ID_ESC_COMBINED_1: currentMeterId_e = 50;
pub const CURRENT_METER_ID_12V_10: currentMeterId_e = 49;
pub const CURRENT_METER_ID_12V_2: currentMeterId_e = 41;
pub const CURRENT_METER_ID_12V_1: currentMeterId_e = 40;
pub const CURRENT_METER_ID_9V_10: currentMeterId_e = 39;
pub const CURRENT_METER_ID_9V_2: currentMeterId_e = 31;
pub const CURRENT_METER_ID_9V_1: currentMeterId_e = 30;
pub const CURRENT_METER_ID_5V_10: currentMeterId_e = 29;
pub const CURRENT_METER_ID_5V_2: currentMeterId_e = 21;
pub const CURRENT_METER_ID_5V_1: currentMeterId_e = 20;
pub const CURRENT_METER_ID_BATTERY_10: currentMeterId_e = 19;
pub const CURRENT_METER_ID_BATTERY_2: currentMeterId_e = 11;
pub const CURRENT_METER_ID_NONE: currentMeterId_e = 0;
pub type voltageMeter_t = voltageMeter_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct voltageMeter_s {
    pub filtered: uint16_t,
    pub unfiltered: uint16_t,
    pub lowVoltageCutoff: bool,
}
pub type voltageMeterId_e = libc::c_uint;
pub const VOLTAGE_METER_ID_CELL_40: voltageMeterId_e = 119;
pub const VOLTAGE_METER_ID_CELL_2: voltageMeterId_e = 81;
pub const VOLTAGE_METER_ID_CELL_1: voltageMeterId_e = 80;
pub const VOLTAGE_METER_ID_ESC_MOTOR_20: voltageMeterId_e = 79;
pub const VOLTAGE_METER_ID_ESC_MOTOR_12: voltageMeterId_e = 71;
pub const VOLTAGE_METER_ID_ESC_MOTOR_11: voltageMeterId_e = 70;
pub const VOLTAGE_METER_ID_ESC_MOTOR_10: voltageMeterId_e = 69;
pub const VOLTAGE_METER_ID_ESC_MOTOR_9: voltageMeterId_e = 68;
pub const VOLTAGE_METER_ID_ESC_MOTOR_8: voltageMeterId_e = 67;
pub const VOLTAGE_METER_ID_ESC_MOTOR_7: voltageMeterId_e = 66;
pub const VOLTAGE_METER_ID_ESC_MOTOR_6: voltageMeterId_e = 65;
pub const VOLTAGE_METER_ID_ESC_MOTOR_5: voltageMeterId_e = 64;
pub const VOLTAGE_METER_ID_ESC_MOTOR_4: voltageMeterId_e = 63;
pub const VOLTAGE_METER_ID_ESC_MOTOR_3: voltageMeterId_e = 62;
pub const VOLTAGE_METER_ID_ESC_MOTOR_2: voltageMeterId_e = 61;
pub const VOLTAGE_METER_ID_ESC_MOTOR_1: voltageMeterId_e = 60;
pub const VOLTAGE_METER_ID_ESC_COMBINED_10: voltageMeterId_e = 59;
pub const VOLTAGE_METER_ID_ESC_COMBINED_1: voltageMeterId_e = 50;
pub const VOLTAGE_METER_ID_12V_10: voltageMeterId_e = 49;
pub const VOLTAGE_METER_ID_12V_2: voltageMeterId_e = 41;
pub const VOLTAGE_METER_ID_12V_1: voltageMeterId_e = 40;
pub const VOLTAGE_METER_ID_9V_10: voltageMeterId_e = 39;
pub const VOLTAGE_METER_ID_9V_2: voltageMeterId_e = 31;
pub const VOLTAGE_METER_ID_9V_1: voltageMeterId_e = 30;
pub const VOLTAGE_METER_ID_5V_10: voltageMeterId_e = 29;
pub const VOLTAGE_METER_ID_5V_2: voltageMeterId_e = 21;
pub const VOLTAGE_METER_ID_5V_1: voltageMeterId_e = 20;
pub const VOLTAGE_METER_ID_BATTERY_10: voltageMeterId_e = 19;
pub const VOLTAGE_METER_ID_BATTERY_2: voltageMeterId_e = 11;
pub const VOLTAGE_METER_ID_BATTERY_1: voltageMeterId_e = 10;
pub const VOLTAGE_METER_ID_NONE: voltageMeterId_e = 0;
pub type batteryState_e = libc::c_uint;
pub const BATTERY_INIT: batteryState_e = 4;
pub const BATTERY_NOT_PRESENT: batteryState_e = 3;
pub const BATTERY_CRITICAL: batteryState_e = 2;
pub const BATTERY_WARNING: batteryState_e = 1;
pub const BATTERY_OK: batteryState_e = 0;
pub type C2RustUnnamed_9 = libc::c_uint;
pub const OSD_G_FORCE: C2RustUnnamed_9 = 42;
pub const OSD_ANTI_GRAVITY: C2RustUnnamed_9 = 41;
pub const OSD_CORE_TEMPERATURE: C2RustUnnamed_9 = 40;
pub const OSD_ADJUSTMENT_RANGE: C2RustUnnamed_9 = 39;
pub const OSD_RTC_DATETIME: C2RustUnnamed_9 = 38;
pub const OSD_REMAINING_TIME_ESTIMATE: C2RustUnnamed_9 = 37;
pub const OSD_ESC_RPM: C2RustUnnamed_9 = 36;
pub const OSD_ESC_TMP: C2RustUnnamed_9 = 35;
pub const OSD_COMPASS_BAR: C2RustUnnamed_9 = 34;
pub const OSD_NUMERICAL_VARIO: C2RustUnnamed_9 = 33;
pub const OSD_NUMERICAL_HEADING: C2RustUnnamed_9 = 32;
pub const OSD_HOME_DIST: C2RustUnnamed_9 = 31;
pub const OSD_HOME_DIR: C2RustUnnamed_9 = 30;
pub const OSD_DISARMED: C2RustUnnamed_9 = 29;
pub const OSD_MAIN_BATT_USAGE: C2RustUnnamed_9 = 28;
pub const OSD_ROLL_ANGLE: C2RustUnnamed_9 = 27;
pub const OSD_PITCH_ANGLE: C2RustUnnamed_9 = 26;
pub const OSD_DEBUG: C2RustUnnamed_9 = 25;
pub const OSD_GPS_LAT: C2RustUnnamed_9 = 24;
pub const OSD_GPS_LON: C2RustUnnamed_9 = 23;
pub const OSD_AVG_CELL_VOLTAGE: C2RustUnnamed_9 = 22;
pub const OSD_WARNINGS: C2RustUnnamed_9 = 21;
pub const OSD_PIDRATE_PROFILE: C2RustUnnamed_9 = 20;
pub const OSD_POWER: C2RustUnnamed_9 = 19;
pub const OSD_YAW_PIDS: C2RustUnnamed_9 = 18;
pub const OSD_PITCH_PIDS: C2RustUnnamed_9 = 17;
pub const OSD_ROLL_PIDS: C2RustUnnamed_9 = 16;
pub const OSD_ALTITUDE: C2RustUnnamed_9 = 15;
pub const OSD_GPS_SATS: C2RustUnnamed_9 = 14;
pub const OSD_GPS_SPEED: C2RustUnnamed_9 = 13;
pub const OSD_MAH_DRAWN: C2RustUnnamed_9 = 12;
pub const OSD_CURRENT_DRAW: C2RustUnnamed_9 = 11;
pub const OSD_VTX_CHANNEL: C2RustUnnamed_9 = 10;
pub const OSD_THROTTLE_POS: C2RustUnnamed_9 = 9;
pub const OSD_CRAFT_NAME: C2RustUnnamed_9 = 8;
pub const OSD_FLYMODE: C2RustUnnamed_9 = 7;
pub const OSD_ITEM_TIMER_2: C2RustUnnamed_9 = 6;
pub const OSD_ITEM_TIMER_1: C2RustUnnamed_9 = 5;
pub const OSD_HORIZON_SIDEBARS: C2RustUnnamed_9 = 4;
pub const OSD_ARTIFICIAL_HORIZON: C2RustUnnamed_9 = 3;
pub const OSD_CROSSHAIRS: C2RustUnnamed_9 = 2;
pub const OSD_MAIN_BATT_VOLTAGE: C2RustUnnamed_9 = 1;
pub const OSD_RSSI_VALUE: C2RustUnnamed_9 = 0;
pub type C2RustUnnamed_10 = libc::c_uint;
pub const OSD_STAT_BLACKBOX_NUMBER: C2RustUnnamed_10 = 13;
pub const OSD_STAT_BLACKBOX: C2RustUnnamed_10 = 12;
pub const OSD_STAT_MAX_ALTITUDE: C2RustUnnamed_10 = 11;
pub const OSD_STAT_USED_MAH: C2RustUnnamed_10 = 10;
pub const OSD_STAT_MAX_CURRENT: C2RustUnnamed_10 = 9;
pub const OSD_STAT_MIN_RSSI: C2RustUnnamed_10 = 8;
pub const OSD_STAT_BATTERY: C2RustUnnamed_10 = 7;
pub const OSD_STAT_END_BATTERY: C2RustUnnamed_10 = 6;
pub const OSD_STAT_MIN_BATTERY: C2RustUnnamed_10 = 5;
pub const OSD_STAT_MAX_DISTANCE: C2RustUnnamed_10 = 4;
pub const OSD_STAT_MAX_SPEED: C2RustUnnamed_10 = 3;
pub const OSD_STAT_TIMER_2: C2RustUnnamed_10 = 2;
pub const OSD_STAT_TIMER_1: C2RustUnnamed_10 = 1;
pub const OSD_STAT_RTC_DATE_TIME: C2RustUnnamed_10 = 0;
pub type C2RustUnnamed_11 = libc::c_uint;
pub const OSD_TIMER_2: C2RustUnnamed_11 = 1;
pub const OSD_TIMER_1: C2RustUnnamed_11 = 0;
pub type C2RustUnnamed_12 = libc::c_uint;
pub const CURRENT_SENSOR_MSP: C2RustUnnamed_12 = 3;
pub const CURRENT_SENSOR_ESC: C2RustUnnamed_12 = 2;
pub type C2RustUnnamed_13 = libc::c_uint;
pub const VOLTAGE_SENSOR_TYPE_ESC: C2RustUnnamed_13 = 1;
//
// adc sensors
//
// VBAT - some boards have external, 12V, 9V and 5V meters.
pub type C2RustUnnamed_14 = libc::c_uint;
pub const VOLTAGE_SENSOR_ADC_5V: C2RustUnnamed_14 = 3;
pub const VOLTAGE_SENSOR_ADC_9V: C2RustUnnamed_14 = 2;
pub const VOLTAGE_SENSOR_ADC_12V: C2RustUnnamed_14 = 1;
pub type C2RustUnnamed_15 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_15 = 64;
pub const SENSOR_SONAR: C2RustUnnamed_15 = 16;
// 4 UPPER CASE alpha numeric characters that identify the flight controller.
pub type C2RustUnnamed_16 = libc::c_uint;
pub type C2RustUnnamed_17 = libc::c_uint;
pub type C2RustUnnamed_18 = libc::c_uint;
#[inline]
unsafe extern "C" fn blackboxConfig() -> *const blackboxConfig_t {
    return &mut blackboxConfig_System;
}
#[inline]
unsafe extern "C" fn blackboxConfigMutable() -> *mut blackboxConfig_t {
    return &mut blackboxConfig_System;
}
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn gyroConfigMutable() -> *mut gyroConfig_t {
    return &mut gyroConfig_System;
}
#[inline]
unsafe extern "C" fn gyroConfig() -> *const gyroConfig_t {
    return &mut gyroConfig_System;
}
#[inline]
unsafe extern "C" fn pilotConfig() -> *const pilotConfig_t {
    return &mut pilotConfig_System;
}
#[inline]
unsafe extern "C" fn pilotConfigMutable() -> *mut pilotConfig_t {
    return &mut pilotConfig_System;
}
#[inline]
unsafe extern "C" fn systemConfig() -> *const systemConfig_t {
    return &mut systemConfig_System;
}
#[inline]
unsafe extern "C" fn modeActivationConditionsMutable(mut _index: libc::c_int)
 -> *mut modeActivationCondition_t {
    return &mut *modeActivationConditions_SystemArray.as_mut_ptr().offset(_index
                                                                              as
                                                                              isize)
               as *mut modeActivationCondition_t;
}
#[inline]
unsafe extern "C" fn modeActivationConditions(mut _index: libc::c_int)
 -> *const modeActivationCondition_t {
    return &mut *modeActivationConditions_SystemArray.as_mut_ptr().offset(_index
                                                                              as
                                                                              isize)
               as *mut modeActivationCondition_t;
}
#[inline]
unsafe extern "C" fn adjustmentRangesMutable(mut _index: libc::c_int)
 -> *mut adjustmentRange_t {
    return &mut *adjustmentRanges_SystemArray.as_mut_ptr().offset(_index as
                                                                      isize)
               as *mut adjustmentRange_t;
}
#[inline]
unsafe extern "C" fn adjustmentRanges(mut _index: libc::c_int)
 -> *const adjustmentRange_t {
    return &mut *adjustmentRanges_SystemArray.as_mut_ptr().offset(_index as
                                                                      isize)
               as *mut adjustmentRange_t;
}
#[inline]
unsafe extern "C" fn rcControlsConfig() -> *const rcControlsConfig_t {
    return &mut rcControlsConfig_System;
}
#[inline]
unsafe extern "C" fn rcControlsConfigMutable() -> *mut rcControlsConfig_t {
    return &mut rcControlsConfig_System;
}
#[inline]
unsafe extern "C" fn flight3DConfig() -> *const flight3DConfig_t {
    return &mut flight3DConfig_System;
}
#[inline]
unsafe extern "C" fn flight3DConfigMutable() -> *mut flight3DConfig_t {
    return &mut flight3DConfig_System;
}
#[inline]
unsafe extern "C" fn armingConfig() -> *const armingConfig_t {
    return &mut armingConfig_System;
}
#[inline]
unsafe extern "C" fn armingConfigMutable() -> *mut armingConfig_t {
    return &mut armingConfig_System;
}
#[inline]
unsafe extern "C" fn failsafeConfig() -> *const failsafeConfig_t {
    return &mut failsafeConfig_System;
}
#[inline]
unsafe extern "C" fn failsafeConfigMutable() -> *mut failsafeConfig_t {
    return &mut failsafeConfig_System;
}
#[inline]
unsafe extern "C" fn imuConfigMutable() -> *mut imuConfig_t {
    return &mut imuConfig_System;
}
#[inline]
unsafe extern "C" fn imuConfig() -> *const imuConfig_t {
    return &mut imuConfig_System;
}
#[inline]
unsafe extern "C" fn mixerConfigMutable() -> *mut mixerConfig_t {
    return &mut mixerConfig_System;
}
#[inline]
unsafe extern "C" fn mixerConfig() -> *const mixerConfig_t {
    return &mut mixerConfig_System;
}
#[inline]
unsafe extern "C" fn motorConfig() -> *const motorConfig_t {
    return &mut motorConfig_System;
}
#[inline]
unsafe extern "C" fn motorConfigMutable() -> *mut motorConfig_t {
    return &mut motorConfig_System;
}
#[inline]
unsafe extern "C" fn pidConfigMutable() -> *mut pidConfig_t {
    return &mut pidConfig_System;
}
#[inline]
unsafe extern "C" fn pidConfig() -> *const pidConfig_t {
    return &mut pidConfig_System;
}
#[no_mangle]
pub static mut inputSource_e: C2RustUnnamed_8 = INPUT_STABILIZED_ROLL;
#[inline]
unsafe extern "C" fn customServoMixers(mut _index: libc::c_int)
 -> *const servoMixer_t {
    return &mut *customServoMixers_SystemArray.as_mut_ptr().offset(_index as
                                                                       isize)
               as *mut servoMixer_t;
}
#[inline]
unsafe extern "C" fn customServoMixersMutable(mut _index: libc::c_int)
 -> *mut servoMixer_t {
    return &mut *customServoMixers_SystemArray.as_mut_ptr().offset(_index as
                                                                       isize)
               as *mut servoMixer_t;
}
#[inline]
unsafe extern "C" fn servoParams(mut _index: libc::c_int)
 -> *const servoParam_t {
    return &mut *servoParams_SystemArray.as_mut_ptr().offset(_index as isize)
               as *mut servoParam_t;
}
#[inline]
unsafe extern "C" fn servoParamsMutable(mut _index: libc::c_int)
 -> *mut servoParam_t {
    return &mut *servoParams_SystemArray.as_mut_ptr().offset(_index as isize)
               as *mut servoParam_t;
}
#[inline]
unsafe extern "C" fn gpsConfig() -> *const gpsConfig_t {
    return &mut gpsConfig_System;
}
#[inline]
unsafe extern "C" fn gpsConfigMutable() -> *mut gpsConfig_t {
    return &mut gpsConfig_System;
}
#[inline]
unsafe extern "C" fn ledStripConfig() -> *const ledStripConfig_t {
    return &mut ledStripConfig_System;
}
#[inline]
unsafe extern "C" fn ledStripConfigMutable() -> *mut ledStripConfig_t {
    return &mut ledStripConfig_System;
}
#[no_mangle]
pub static mut colors: *mut hsvColor_t =
    0 as *const hsvColor_t as *mut hsvColor_t;
#[no_mangle]
pub static mut modeColors: *const modeColorIndexes_t =
    0 as *const modeColorIndexes_t;
#[no_mangle]
pub static mut specialColors: specialColorIndexes_t =
    specialColorIndexes_t{color: [0; 11],};
#[inline]
unsafe extern "C" fn osdConfigMutable() -> *mut osdConfig_t {
    return &mut osdConfig_System;
}
#[inline]
unsafe extern "C" fn osdConfig() -> *const osdConfig_t {
    return &mut osdConfig_System;
}
#[inline]
unsafe extern "C" fn serialConfig() -> *const serialConfig_t {
    return &mut serialConfig_System;
}
#[inline]
unsafe extern "C" fn vtxSettingsConfigMutable() -> *mut vtxSettingsConfig_t {
    return &mut vtxSettingsConfig_System;
}
#[inline]
unsafe extern "C" fn vtxSettingsConfig() -> *const vtxSettingsConfig_t {
    return &mut vtxSettingsConfig_System;
}
#[inline]
unsafe extern "C" fn beeperConfig() -> *const beeperConfig_t {
    return &mut beeperConfig_System;
}
#[inline]
unsafe extern "C" fn beeperConfigMutable() -> *mut beeperConfig_t {
    return &mut beeperConfig_System;
}
#[inline]
unsafe extern "C" fn rxConfig() -> *const rxConfig_t {
    return &mut rxConfig_System;
}
#[inline]
unsafe extern "C" fn rxConfigMutable() -> *mut rxConfig_t {
    return &mut rxConfig_System;
}
#[inline]
unsafe extern "C" fn usbDevConfig() -> *const usbDev_t {
    return &mut usbDevConfig_System;
}
#[inline]
unsafe extern "C" fn usbDevConfigMutable() -> *mut usbDev_t {
    return &mut usbDevConfig_System;
}
#[inline]
unsafe extern "C" fn rxFailsafeChannelConfigsMutable(mut _index: libc::c_int)
 -> *mut rxFailsafeChannelConfig_t {
    return &mut *rxFailsafeChannelConfigs_SystemArray.as_mut_ptr().offset(_index
                                                                              as
                                                                              isize)
               as *mut rxFailsafeChannelConfig_t;
}
#[inline]
unsafe extern "C" fn rxFailsafeChannelConfigs(mut _index: libc::c_int)
 -> *const rxFailsafeChannelConfig_t {
    return &mut *rxFailsafeChannelConfigs_SystemArray.as_mut_ptr().offset(_index
                                                                              as
                                                                              isize)
               as *mut rxFailsafeChannelConfig_t;
}
#[inline]
unsafe extern "C" fn batteryConfig() -> *const batteryConfig_t {
    return &mut batteryConfig_System;
}
#[inline]
unsafe extern "C" fn batteryConfigMutable() -> *mut batteryConfig_t {
    return &mut batteryConfig_System;
}
#[inline]
unsafe extern "C" fn voltageSensorADCConfigMutable(mut _index: libc::c_int)
 -> *mut voltageSensorADCConfig_t {
    return &mut *voltageSensorADCConfig_SystemArray.as_mut_ptr().offset(_index
                                                                            as
                                                                            isize)
               as *mut voltageSensorADCConfig_t;
}
#[inline]
unsafe extern "C" fn currentSensorADCConfigMutable()
 -> *mut currentSensorADCConfig_t {
    return &mut currentSensorADCConfig_System;
}
#[inline]
unsafe extern "C" fn currentSensorVirtualConfig()
 -> *const currentSensorVirtualConfig_t {
    return &mut currentSensorVirtualConfig_System;
}
#[inline]
unsafe extern "C" fn currentSensorADCConfig()
 -> *const currentSensorADCConfig_t {
    return &mut currentSensorADCConfig_System;
}
#[inline]
unsafe extern "C" fn voltageSensorADCConfig(mut _index: libc::c_int)
 -> *const voltageSensorADCConfig_t {
    return &mut *voltageSensorADCConfig_SystemArray.as_mut_ptr().offset(_index
                                                                            as
                                                                            isize)
               as *mut voltageSensorADCConfig_t;
}
#[inline]
unsafe extern "C" fn currentSensorVirtualConfigMutable()
 -> *mut currentSensorVirtualConfig_t {
    return &mut currentSensorVirtualConfig_System;
}
#[inline]
unsafe extern "C" fn accelerometerConfig() -> *const accelerometerConfig_t {
    return &mut accelerometerConfig_System;
}
#[inline]
unsafe extern "C" fn accelerometerConfigMutable()
 -> *mut accelerometerConfig_t {
    return &mut accelerometerConfig_System;
}
#[inline]
unsafe extern "C" fn barometerConfigMutable() -> *mut barometerConfig_t {
    return &mut barometerConfig_System;
}
#[inline]
unsafe extern "C" fn barometerConfig() -> *const barometerConfig_t {
    return &mut barometerConfig_System;
}
#[inline]
unsafe extern "C" fn boardAlignmentMutable() -> *mut boardAlignment_t {
    return &mut boardAlignment_System;
}
#[inline]
unsafe extern "C" fn boardAlignment() -> *const boardAlignment_t {
    return &mut boardAlignment_System;
}
#[inline]
unsafe extern "C" fn compassConfigMutable() -> *mut compassConfig_t {
    return &mut compassConfig_System;
}
#[inline]
unsafe extern "C" fn compassConfig() -> *const compassConfig_t {
    return &mut compassConfig_System;
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
static mut flightControllerIdentifier: *const libc::c_char =
    b"CLFL\x00" as *const u8 as *const libc::c_char;
static mut rebootMode: uint8_t = 0;
#[no_mangle]
pub static mut escMode: uint8_t = 0;
#[no_mangle]
pub static mut escPortIndex: uint8_t = 0;
unsafe extern "C" fn mspEscPassthroughFn(mut serialPort: *mut serialPort_t) {
    escEnablePassthrough(serialPort, escPortIndex as uint16_t, escMode);
}
unsafe extern "C" fn mspFc4waySerialCommand(mut dst: *mut sbuf_t,
                                            mut src: *mut sbuf_t,
                                            mut mspPostProcessFn:
                                                *mut mspPostProcessFnPtr) {
    let dataSize: libc::c_uint = sbufBytesRemaining(src) as libc::c_uint;
    if dataSize == 0 as libc::c_int as libc::c_uint {
        // Legacy format
        escMode = 0xff as libc::c_int as uint8_t
    } else { escMode = sbufReadU8(src); escPortIndex = sbufReadU8(src) }
    let mut current_block_16: u64;
    match escMode as libc::c_int {
        255 => {
            // get channel number
        // switch all motor lines HI
        // reply with the count of ESC found
            sbufWriteU8(dst, esc4wayInit());
            if !mspPostProcessFn.is_null() {
                *mspPostProcessFn =
                    Some(esc4wayProcess as
                             unsafe extern "C" fn(_: *mut serialPort_s) -> ())
            }
            current_block_16 = 18317007320854588510;
        }
        0 | 1 | 2 | 3 | 4 => {
            if (escPortIndex as libc::c_int) < getMotorCount() as libc::c_int
                   ||
                   escMode as libc::c_int == PROTOCOL_KISS as libc::c_int &&
                       escPortIndex as libc::c_int == 255 as libc::c_int {
                sbufWriteU8(dst, 1 as libc::c_int as uint8_t);
                if !mspPostProcessFn.is_null() {
                    *mspPostProcessFn =
                        Some(mspEscPassthroughFn as
                                 unsafe extern "C" fn(_: *mut serialPort_t)
                                     -> ())
                }
                current_block_16 = 18317007320854588510;
            } else { current_block_16 = 6498864853541362327; }
        }
        _ => { current_block_16 = 6498864853541362327; }
    }
    match current_block_16 {
        6498864853541362327 => {
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
        }
        _ => { }
    };
}
//USE_SERIAL_4WAY_BLHELI_INTERFACE
unsafe extern "C" fn mspRebootFn(mut serialPort: *mut serialPort_t) {
    stopPwmAllMotors();
    match rebootMode as libc::c_int {
        0 => { systemReset(); }
        1 => { systemResetToBootloader(); }
        2 => { systemResetToMsc(); }
        _ => { }
    }
    loop 
         // control should never return here.
         {
    };
}
unsafe extern "C" fn serializeSDCardSummaryReply(mut dst: *mut sbuf_t) {
    let mut flags: uint8_t =
        MSP_SDCARD_FLAG_SUPPORTTED as libc::c_int as uint8_t;
    let mut state: uint8_t = 0 as libc::c_int as uint8_t;
    sbufWriteU8(dst, flags);
    // Merge the card and filesystem states together
    if !sdcard_isInserted() {
        state = MSP_SDCARD_STATE_NOT_PRESENT as libc::c_int as uint8_t
    } else if !sdcard_isFunctional() {
        state = MSP_SDCARD_STATE_FATAL as libc::c_int as uint8_t
    } else {
        match afatfs_getFilesystemState() as libc::c_uint {
            3 => { state = MSP_SDCARD_STATE_READY as libc::c_int as uint8_t }
            2 => {
                if sdcard_isInitialized() {
                    state = MSP_SDCARD_STATE_FS_INIT as libc::c_int as uint8_t
                } else {
                    state =
                        MSP_SDCARD_STATE_CARD_INIT as libc::c_int as uint8_t
                }
            }
            1 | 0 | _ => {
                state = MSP_SDCARD_STATE_FATAL as libc::c_int as uint8_t
            }
        }
    }
    sbufWriteU8(dst, state);
    sbufWriteU8(dst, afatfs_getLastError() as uint8_t);
    // Write free space and total space in kilobytes
    sbufWriteU32(dst,
                 afatfs_getContiguousFreeSpace().wrapping_div(1024 as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_uint));
    sbufWriteU32(dst,
                 (*sdcard_getMetadata()).numBlocks.wrapping_div(2 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint));
    // Block size is half a kilobyte
}
unsafe extern "C" fn serializeDataflashSummaryReply(mut dst: *mut sbuf_t) {
    // FlashFS is not configured or valid device is not detected
    sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
    sbufWriteU32(dst, 0 as libc::c_int as uint32_t);
    sbufWriteU32(dst, 0 as libc::c_int as uint32_t);
    sbufWriteU32(dst, 0 as libc::c_int as uint32_t);
}
// USE_FLASHFS
// USE_OSD_SLAVE
/*
 * Returns true if the command was processd, false otherwise.
 * May set mspPostProcessFunc to a function to be called once the command has been processed
 */
unsafe extern "C" fn mspCommonProcessOutCommand(mut cmdMSP: uint8_t,
                                                mut dst: *mut sbuf_t,
                                                mut mspPostProcessFn:
                                                    *mut mspPostProcessFnPtr)
 -> bool {
    match cmdMSP as libc::c_int {
        1 => {
            sbufWriteU8(dst,
                        0 as libc::c_int as
                            uint8_t); // No other build targets currently have hardware revision detection.
            sbufWriteU8(dst, 1 as libc::c_int as uint8_t); // 0 == FC
            sbufWriteU8(dst, 40 as libc::c_int as uint8_t);
        }
        2 => {
            sbufWriteData(dst,
                          flightControllerIdentifier as *const libc::c_void,
                          4 as libc::c_int);
        }
        3 => {
            sbufWriteU8(dst, 2 as libc::c_int as uint8_t);
            sbufWriteU8(dst, 5 as libc::c_int as uint8_t);
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
        }
        4 => {
            sbufWriteData(dst,
                          (*systemConfig()).boardIdentifier.as_ptr() as
                              *const libc::c_void, 4 as libc::c_int);
            sbufWriteU16(dst, 0 as libc::c_int as uint16_t);
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
            // Board communication capabilities (uint8)
        // Bit 0: 1 iff the board has VCP
        // Bit 1: 1 iff the board supports software serial
            let mut commCapabilities: uint8_t = 0 as libc::c_int as uint8_t;
            commCapabilities =
                (commCapabilities as libc::c_int |
                     (1 as libc::c_int) << 0 as libc::c_int) as uint8_t;
            commCapabilities =
                (commCapabilities as libc::c_int |
                     (1 as libc::c_int) << 1 as libc::c_int) as uint8_t;
            sbufWriteU8(dst, commCapabilities);
            // Target name with explicit length
            sbufWriteU8(dst, strlen(targetName) as uint8_t);
            sbufWriteData(dst, targetName as *const libc::c_void,
                          strlen(targetName) as libc::c_int);
            // Board name with explicit length
            let mut value: *mut libc::c_char = getBoardName();
            sbufWriteU8(dst, strlen(value) as uint8_t);
            sbufWriteString(dst, value);
            // Manufacturer id with explicit length
            value = getManufacturerId();
            sbufWriteU8(dst, strlen(value) as uint8_t);
            sbufWriteString(dst, value);
            // Signature
            sbufWriteData(dst, getSignature() as *const libc::c_void,
                          32 as
                              libc::c_int); // milliamp hours drawn from battery
        }
        5 => {
            sbufWriteData(dst, buildDate as *const libc::c_void,
                          11 as
                              libc::c_int); // send current in 0.01 A steps, range is -320A to 320A
            sbufWriteData(dst, buildTime as *const libc::c_void,
                          8 as libc::c_int);
            sbufWriteData(dst, shortGitRevision as *const libc::c_void,
                          7 as libc::c_int);
        }
        110 => {
            sbufWriteU8(dst,
                        constrain(getBatteryVoltage() as libc::c_int,
                                  0 as libc::c_int, 255 as libc::c_int) as
                            uint8_t);
            sbufWriteU16(dst,
                         constrain(getMAhDrawn(), 0 as libc::c_int,
                                   0xffff as libc::c_int) as uint16_t);
            sbufWriteU16(dst, getRssi());
            sbufWriteU16(dst,
                         constrain(getAmperage(), -(0x8000 as libc::c_int),
                                   0x7fff as libc::c_int) as int16_t as
                             uint16_t);
        }
        254 => {
            let mut i: libc::c_int = 0 as libc::c_int;
            while i < 4 as libc::c_int {
                sbufWriteU16(dst, debug[i as usize] as uint16_t);
                i += 1
                // 4 variables are here for general monitoring purpose
            }
        }
        160 => {
            sbufWriteU32(dst, *(0x1ff07a10 as libc::c_int as *mut uint32_t));
            sbufWriteU32(dst, *(0x1ff07a14 as libc::c_int as *mut uint32_t));
            sbufWriteU32(dst, *(0x1ff07a18 as libc::c_int as *mut uint32_t));
        }
        36 => { sbufWriteU32(dst, featureMask()); }
        184 => {
            sbufWriteU32(dst, (*beeperConfig()).beeper_off_flags);
            sbufWriteU8(dst, (*beeperConfig()).dshotBeaconTone);
            sbufWriteU32(dst, (*beeperConfig()).dshotBeaconOffFlags);
        }
        130 => {
            // battery characteristics
            sbufWriteU8(dst,
                        constrain(getBatteryCellCount() as libc::c_int,
                                  0 as libc::c_int, 255 as libc::c_int) as
                            uint8_t); // 0 indicates battery not detected.
            sbufWriteU16(dst, (*batteryConfig()).batteryCapacity); // in mAh
            // battery state
            sbufWriteU8(dst,
                        constrain(getBatteryVoltage() as libc::c_int,
                                  0 as libc::c_int, 255 as libc::c_int) as
                            uint8_t); // in 0.1V steps
            sbufWriteU16(dst,
                         constrain(getMAhDrawn(), 0 as libc::c_int,
                                   0xffff as libc::c_int) as
                             uint16_t); // milliamp hours drawn from battery
            sbufWriteU16(dst,
                         constrain(getAmperage(), -(0x8000 as libc::c_int),
                                   0x7fff as libc::c_int) as int16_t as
                             uint16_t); // send current in 0.01 A steps, range is -320A to 320A
            // battery alerts
            sbufWriteU8(dst, getBatteryState() as uint8_t);
        }
        128 => {
            // write out id and voltage meter values, once for each meter we support
            let mut count: uint8_t = supportedVoltageMeterCount;
            count =
                (count as libc::c_int -
                     (12 as libc::c_int - getMotorCount() as libc::c_int)) as
                    uint8_t;
            let mut i_0: libc::c_int = 0 as libc::c_int;
            while i_0 < count as libc::c_int {
                let mut meter: voltageMeter_t =
                    voltageMeter_t{filtered: 0,
                                   unfiltered: 0,
                                   lowVoltageCutoff: false,};
                let mut id: uint8_t =
                    *voltageMeterIds.as_ptr().offset(i_0 as isize);
                voltageMeterRead(id as voltageMeterId_e, &mut meter);
                sbufWriteU8(dst, id);
                sbufWriteU8(dst,
                            constrain(meter.filtered as libc::c_int,
                                      0 as libc::c_int, 255 as libc::c_int) as
                                uint8_t);
                i_0 += 1
            }
        }
        129 => {
            // write out id and current meter values, once for each meter we support
            let mut count_0: uint8_t = supportedCurrentMeterCount;
            count_0 =
                (count_0 as libc::c_int -
                     (12 as libc::c_int - getMotorCount() as libc::c_int)) as
                    uint8_t;
            let mut i_1: libc::c_int = 0 as libc::c_int;
            while i_1 < count_0 as libc::c_int {
                let mut meter_0: currentMeter_t =
                    currentMeter_t{amperage: 0,
                                   amperageLatest: 0,
                                   mAhDrawn: 0,};
                let mut id_0: uint8_t =
                    *currentMeterIds.as_ptr().offset(i_1 as isize);
                currentMeterRead(id_0 as currentMeterId_e, &mut meter_0);
                sbufWriteU8(dst, id_0);
                // send amperage in 0.001 A steps (mA). Negative range is truncated to zero
                sbufWriteU16(dst,
                             constrain(meter_0.mAhDrawn, 0 as libc::c_int,
                                       0xffff as libc::c_int) as
                                 uint16_t); // milliamp hours drawn from battery
                sbufWriteU16(dst,
                             constrain(meter_0.amperage * 10 as libc::c_int,
                                       0 as libc::c_int,
                                       0xffff as libc::c_int) as uint16_t);
                i_1 += 1
            }
        }
        56 => {
            // by using a sensor type and a sub-frame length it's possible to configure any type of voltage meter,
        // e.g. an i2c/spi/can sensor or any sensor not built directly into the FC such as ESC/RX/SPort/SBus that has
        // different configuration requirements.
            sbufWriteU8(dst,
                        1 as libc::c_int as
                            uint8_t); // voltage meters in payload
            let mut i_2: libc::c_int =
                VOLTAGE_SENSOR_ADC_VBAT as
                    libc::c_int; // length of id, type, vbatscale, vbatresdivval, vbatresdivmultipler, in bytes
            while i_2 < 1 as libc::c_int {
                let adcSensorSubframeLength: uint8_t =
                    (1 as libc::c_int + 1 as libc::c_int + 1 as libc::c_int +
                         1 as libc::c_int + 1 as libc::c_int) as
                        uint8_t; // ADC sensor sub-frame length
                sbufWriteU8(dst, adcSensorSubframeLength); // id of the sensor
                sbufWriteU8(dst,
                            voltageMeterADCtoIDMap[i_2 as
                                                       usize]); // indicate the type of sensor that the next part of the payload is for
                sbufWriteU8(dst,
                            VOLTAGE_SENSOR_TYPE_ADC_RESISTOR_DIVIDER as
                                libc::c_int as uint8_t);
                sbufWriteU8(dst, (*voltageSensorADCConfig(i_2)).vbatscale);
                sbufWriteU8(dst,
                            (*voltageSensorADCConfig(i_2)).vbatresdivval);
                sbufWriteU8(dst,
                            (*voltageSensorADCConfig(i_2)).vbatresdivmultiplier);
                i_2 += 1
            }
        }
        40 => {
            // the ADC and VIRTUAL sensors have the same configuration requirements, however this API reflects
        // that this situation may change and allows us to support configuration of any current sensor with
        // specialist configuration requirements.
            let mut currentMeterCount: libc::c_int =
                1 as
                    libc::c_int; // length of id, type, scale, offset, in bytes
            currentMeterCount += 1; // the id of the meter
            sbufWriteU8(dst,
                        currentMeterCount as
                            uint8_t); // indicate the type of sensor that the next part of the payload is for
            let adcSensorSubframeLength_0: uint8_t =
                (1 as libc::c_int + 1 as libc::c_int + 2 as libc::c_int +
                     2 as libc::c_int) as
                    uint8_t; // length of id, type, scale, offset, in bytes
            sbufWriteU8(dst,
                        adcSensorSubframeLength_0); // the id of the meter
            sbufWriteU8(dst,
                        CURRENT_METER_ID_BATTERY_1 as libc::c_int as
                            uint8_t); // indicate the type of sensor that the next part of the payload is for
            sbufWriteU8(dst,
                        CURRENT_SENSOR_ADC as libc::c_int as
                            uint8_t); // no providers
            sbufWriteU16(dst, (*currentSensorADCConfig()).scale as uint16_t);
            sbufWriteU16(dst, (*currentSensorADCConfig()).offset as uint16_t);
            let virtualSensorSubframeLength: int8_t =
                (1 as libc::c_int + 1 as libc::c_int + 2 as libc::c_int +
                     2 as libc::c_int) as int8_t;
            sbufWriteU8(dst, virtualSensorSubframeLength as uint8_t);
            sbufWriteU8(dst,
                        CURRENT_METER_ID_VIRTUAL_1 as libc::c_int as uint8_t);
            sbufWriteU8(dst,
                        CURRENT_SENSOR_VIRTUAL as libc::c_int as uint8_t);
            sbufWriteU16(dst,
                         (*currentSensorVirtualConfig()).scale as uint16_t);
            sbufWriteU16(dst, (*currentSensorVirtualConfig()).offset);
        }
        32 => {
            sbufWriteU8(dst, (*batteryConfig()).vbatmincellvoltage);
            sbufWriteU8(dst, (*batteryConfig()).vbatmaxcellvoltage);
            sbufWriteU8(dst, (*batteryConfig()).vbatwarningcellvoltage);
            sbufWriteU16(dst, (*batteryConfig()).batteryCapacity);
            sbufWriteU8(dst,
                        (*batteryConfig()).voltageMeterSource as uint8_t);
            sbufWriteU8(dst,
                        (*batteryConfig()).currentMeterSource as uint8_t);
        }
        82 => { sbufWriteU8(dst, 0 as libc::c_int as uint8_t); }
        84 => {
            let mut osdFlags: uint8_t = 0 as libc::c_int as uint8_t;
            osdFlags =
                (osdFlags as libc::c_int |
                     (1 as libc::c_int) << 0 as libc::c_int) as uint8_t;
            sbufWriteU8(dst, osdFlags);
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
            // OSD specific, not applicable to OSD slaves.
            // Configuration
            sbufWriteU8(dst, (*osdConfig()).units as uint8_t);
            // Alarms
            sbufWriteU8(dst, (*osdConfig()).rssi_alarm);
            sbufWriteU16(dst, (*osdConfig()).cap_alarm);
            // Reuse old timer alarm (U16) as OSD_ITEM_COUNT
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
            sbufWriteU8(dst, OSD_ITEM_COUNT as libc::c_int as uint8_t);
            sbufWriteU16(dst, (*osdConfig()).alt_alarm);
            // Element position and visibility
            let mut i_3: libc::c_int = 0 as libc::c_int;
            while i_3 < OSD_ITEM_COUNT as libc::c_int {
                sbufWriteU16(dst, (*osdConfig()).item_pos[i_3 as usize]);
                i_3 += 1
            }
            // Post flight statistics
            sbufWriteU8(dst, OSD_STAT_COUNT as libc::c_int as uint8_t);
            let mut i_4: libc::c_int = 0 as libc::c_int;
            while i_4 < OSD_STAT_COUNT as libc::c_int {
                sbufWriteU8(dst, osdStatGetState(i_4 as uint8_t) as uint8_t);
                i_4 += 1
            }
            // Timers
            sbufWriteU8(dst, OSD_TIMER_COUNT as libc::c_int as uint8_t);
            let mut i_5: libc::c_int = 0 as libc::c_int;
            while i_5 < OSD_TIMER_COUNT as libc::c_int {
                sbufWriteU16(dst, (*osdConfig()).timers[i_5 as usize]);
                i_5 += 1
            }
            // Enabled warnings
            sbufWriteU16(dst,
                         (*osdConfig()).enabledWarnings); // unconditional part of flags, first 32 bits
        }
        _ => { return 0 as libc::c_int != 0 }
    } // MSP_STATUS
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn mspProcessOutCommand(mut cmdMSP: uint8_t,
                                          mut dst: *mut sbuf_t) -> bool {
    let mut unsupportedCommand: bool = 0 as libc::c_int != 0;
    let mut rtcDateTimeIsSet: uint8_t = 0;
    let mut dt: dateTime_t =
        dateTime_t{year: 0,
                   month: 0,
                   day: 0,
                   hours: 0,
                   minutes: 0,
                   seconds: 0,
                   millis: 0,};
    match cmdMSP as libc::c_int {
        150 | 101 => {
            let mut flightModeFlags: boxBitmask_t =
                boxBitmask_t{bits: [0; 2],};
            let flagBits: libc::c_int =
                packFlightModeFlags(&mut flightModeFlags);
            sbufWriteU16(dst, getTaskDeltaTime(TASK_GYROPID) as uint16_t);
            sbufWriteU16(dst, i2cGetErrorCounter());
            sbufWriteU16(dst,
                         (sensors(SENSOR_ACC as libc::c_int as uint32_t) as
                              libc::c_int |
                              (sensors(SENSOR_BARO as libc::c_int as uint32_t)
                                   as libc::c_int) << 1 as libc::c_int |
                              (sensors(SENSOR_MAG as libc::c_int as uint32_t)
                                   as libc::c_int) << 2 as libc::c_int |
                              (sensors(SENSOR_GPS as libc::c_int as uint32_t)
                                   as libc::c_int) << 3 as libc::c_int |
                              (sensors(SENSOR_RANGEFINDER as libc::c_int as
                                           uint32_t) as libc::c_int) <<
                                  4 as libc::c_int |
                              (sensors(SENSOR_GYRO as libc::c_int as uint32_t)
                                   as libc::c_int) << 5 as libc::c_int) as
                             uint16_t);
            sbufWriteData(dst,
                          &mut flightModeFlags as *mut boxBitmask_t as
                              *const libc::c_void, 4 as libc::c_int);
            sbufWriteU8(dst, getCurrentPidProfileIndex());
            sbufWriteU16(dst,
                         constrain(averageSystemLoadPercent as libc::c_int,
                                   0 as libc::c_int, 100 as libc::c_int) as
                             uint16_t);
            if cmdMSP as libc::c_int == 150 as libc::c_int {
                sbufWriteU8(dst, 3 as libc::c_int as uint8_t);
                sbufWriteU8(dst, getCurrentControlRateProfileIndex());
            } else {
                sbufWriteU16(dst, 0 as libc::c_int as uint16_t);
                // gyro cycle time
            }
            // write flightModeFlags header. Lowest 4 bits contain number of bytes that follow
            // header is emited even when all bits fit into 32 bits to allow future extension
            let mut byteCount: libc::c_int =
                (flagBits - 32 as libc::c_int + 7 as libc::c_int) /
                    8 as libc::c_int; // 32 already stored, round up
            byteCount =
                constrain(byteCount, 0 as libc::c_int,
                          15 as libc::c_int); // limit to 16 bytes (128 bits)
            sbufWriteU8(dst, byteCount as uint8_t);
            sbufWriteData(dst,
                          (&mut flightModeFlags as *mut boxBitmask_t as
                               *mut uint8_t).offset(4 as libc::c_int as isize)
                              as *const libc::c_void, byteCount);
            // Write arming disable flags
            // 1 byte, flag count
            sbufWriteU8(dst, 20 as libc::c_int as uint8_t);
            // 4 bytes, flags
            let armingDisableFlags: uint32_t =
                getArmingDisableFlags() as uint32_t;
            sbufWriteU32(dst, armingDisableFlags);
        }
        102 => {
            // Hack scale due to choice of units for sensor data in multiwii
            let mut scale: uint8_t = 0;
            if acc.dev.acc_1G as libc::c_int >
                   512 as libc::c_int * 4 as libc::c_int {
                scale = 8 as libc::c_int as uint8_t
            } else if acc.dev.acc_1G as libc::c_int >
                          512 as libc::c_int * 2 as libc::c_int {
                scale = 4 as libc::c_int as uint8_t
            } else if acc.dev.acc_1G as libc::c_int >= 512 as libc::c_int {
                scale = 2 as libc::c_int as uint8_t
            } else { scale = 1 as libc::c_int as uint8_t }
            let mut i: libc::c_int = 0 as libc::c_int;
            while i < 3 as libc::c_int {
                sbufWriteU16(dst,
                             lrintf(acc.accADC[i as usize] /
                                        scale as libc::c_int as libc::c_float)
                                 as uint16_t);
                i += 1
            }
            let mut i_0: libc::c_int = 0 as libc::c_int;
            while i_0 < 3 as libc::c_int {
                sbufWriteU16(dst, gyroRateDps(i_0) as uint16_t);
                i_0 += 1
            }
            let mut i_1: libc::c_int = 0 as libc::c_int;
            while i_1 < 3 as libc::c_int {
                sbufWriteU16(dst,
                             lrintf(mag.magADC[i_1 as usize]) as uint16_t);
                i_1 += 1
            }
        }
        10 => {
            let nameLen: libc::c_int =
                strlen((*pilotConfig()).name.as_ptr()) as libc::c_int;
            let mut i_2: libc::c_int = 0 as libc::c_int;
            while i_2 < nameLen {
                sbufWriteU8(dst,
                            (*pilotConfig()).name[i_2 as usize] as uint8_t);
                i_2 += 1
            }
        }
        103 => {
            sbufWriteData(dst,
                          &mut servo as *mut [int16_t; 8] as
                              *const libc::c_void,
                          8 as libc::c_int * 2 as libc::c_int);
        }
        120 => {
            let mut i_3: libc::c_int = 0 as libc::c_int;
            while i_3 < 8 as libc::c_int {
                sbufWriteU16(dst, (*servoParams(i_3)).min as uint16_t);
                sbufWriteU16(dst, (*servoParams(i_3)).max as uint16_t);
                sbufWriteU16(dst, (*servoParams(i_3)).middle as uint16_t);
                sbufWriteU8(dst, (*servoParams(i_3)).rate as uint8_t);
                sbufWriteU8(dst,
                            (*servoParams(i_3)).forwardFromChannel as
                                uint8_t);
                sbufWriteU32(dst, (*servoParams(i_3)).reversedSources);
                i_3 += 1
            }
        }
        241 => {
            let mut i_4: libc::c_int = 0 as libc::c_int;
            while i_4 < 2 as libc::c_int * 8 as libc::c_int {
                sbufWriteU8(dst, (*customServoMixers(i_4)).targetChannel);
                sbufWriteU8(dst, (*customServoMixers(i_4)).inputSource);
                sbufWriteU8(dst, (*customServoMixers(i_4)).rate as uint8_t);
                sbufWriteU8(dst, (*customServoMixers(i_4)).speed);
                sbufWriteU8(dst, (*customServoMixers(i_4)).min as uint8_t);
                sbufWriteU8(dst, (*customServoMixers(i_4)).max as uint8_t);
                sbufWriteU8(dst, (*customServoMixers(i_4)).box_0);
                i_4 += 1
            }
        }
        104 => {
            let mut i_5: libc::c_uint = 0 as libc::c_int as libc::c_uint;
            while i_5 < 8 as libc::c_int as libc::c_uint {
                if i_5 >= 8 as libc::c_int as libc::c_uint ||
                       !(*pwmGetMotors().offset(i_5 as isize)).enabled {
                    sbufWriteU16(dst, 0 as libc::c_int as uint16_t);
                } else {
                    sbufWriteU16(dst,
                                 convertMotorToExternal(motor[i_5 as usize]));
                }
                i_5 = i_5.wrapping_add(1)
            }
        }
        105 => {
            let mut i_6: libc::c_int = 0 as libc::c_int;
            while i_6 < rxRuntimeConfig.channelCount as libc::c_int {
                sbufWriteU16(dst, rcData[i_6 as usize] as uint16_t);
                i_6 += 1
            }
        }
        108 => {
            sbufWriteU16(dst, attitude.values.roll as uint16_t);
            sbufWriteU16(dst, attitude.values.pitch as uint16_t);
            sbufWriteU16(dst,
                         (attitude.values.yaw as libc::c_int /
                              10 as libc::c_int) as uint16_t);
        }
        109 => {
            sbufWriteU32(dst, getEstimatedAltitude() as uint32_t);
            sbufWriteU16(dst, getEstimatedVario() as uint16_t);
        }
        58 => { sbufWriteU32(dst, 0 as libc::c_int as uint32_t); }
        38 => {
            sbufWriteU16(dst, (*boardAlignment()).rollDegrees as uint16_t);
            sbufWriteU16(dst, (*boardAlignment()).pitchDegrees as uint16_t);
            sbufWriteU16(dst, (*boardAlignment()).yawDegrees as uint16_t);
        }
        61 => {
            sbufWriteU8(dst, (*armingConfig()).auto_disarm_delay);
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
            sbufWriteU8(dst, (*imuConfig()).small_angle);
        }
        111 => {
            sbufWriteU8(dst,
                        (*currentControlRateProfile).rcRates[FD_ROLL as
                                                                 libc::c_int
                                                                 as usize]);
            sbufWriteU8(dst,
                        (*currentControlRateProfile).rcExpo[FD_ROLL as
                                                                libc::c_int as
                                                                usize]);
            let mut i_7: libc::c_int = 0 as libc::c_int;
            while i_7 < 3 as libc::c_int {
                sbufWriteU8(dst,
                            (*currentControlRateProfile).rates[i_7 as usize]);
                i_7 += 1
                // R,P,Y see flight_dynamics_index_t
            } // alt changed from 1m to 0.01m per lsb since MSP API 1.39 by RTH. To maintain backwards compatibility compensate to 1m per lsb in MSP again.
            sbufWriteU8(dst,
                        (*currentControlRateProfile).dynThrPID); //Blackbox supported
            sbufWriteU8(dst,
                        (*currentControlRateProfile).thrMid8); // Rate numerator, not used anymore
            sbufWriteU8(dst,
                        (*currentControlRateProfile).thrExpo8); // was pidProfile.yaw_p_limit
            sbufWriteU16(dst,
                         (*currentControlRateProfile).tpa_breakpoint); // reserved
            sbufWriteU8(dst,
                        (*currentControlRateProfile).rcExpo[FD_YAW as
                                                                libc::c_int as
                                                                usize]); // was low byte of currentPidProfile->dtermSetpointWeight
            sbufWriteU8(dst,
                        (*currentControlRateProfile).rcRates[FD_YAW as
                                                                 libc::c_int
                                                                 as
                                                                 usize]); // reserved
            sbufWriteU8(dst,
                        (*currentControlRateProfile).rcRates[FD_PITCH as
                                                                 libc::c_int
                                                                 as
                                                                 usize]); // reserved
            sbufWriteU8(dst,
                        (*currentControlRateProfile).rcExpo[FD_PITCH as
                                                                libc::c_int as
                                                                usize]); // reserved
        }
        112 => {
            let mut i_8: libc::c_int =
                0 as libc::c_int; // was pidProfile.levelSensitivity
            while i_8 < PID_ITEM_COUNT as libc::c_int {
                sbufWriteU8(dst,
                            (*currentPidProfile).pid[i_8 as
                                                         usize].P); // was currentPidProfile->dtermSetpointWeight
                sbufWriteU8(dst, (*currentPidProfile).pid[i_8 as usize].I);
                sbufWriteU8(dst, (*currentPidProfile).pid[i_8 as usize].D);
                i_8 += 1
            }
        }
        117 => {
            let mut c: *const libc::c_char = pidNames.as_ptr();
            while *c != 0 { sbufWriteU8(dst, *c as uint8_t); c = c.offset(1) }
        }
        59 => { sbufWriteU8(dst, 1 as libc::c_int as uint8_t); }
        34 => {
            let mut i_9: libc::c_int = 0 as libc::c_int;
            while i_9 < 20 as libc::c_int {
                let mut mac: *const modeActivationCondition_t =
                    modeActivationConditions(i_9);
                let mut box_0: *const box_t = findBoxByBoxId((*mac).modeId);
                sbufWriteU8(dst, (*box_0).permanentId);
                sbufWriteU8(dst, (*mac).auxChannelIndex);
                sbufWriteU8(dst, (*mac).range.startStep);
                sbufWriteU8(dst, (*mac).range.endStep);
                i_9 += 1
            }
        }
        52 => {
            let mut i_10: libc::c_int = 0 as libc::c_int;
            while i_10 < 15 as libc::c_int {
                let mut adjRange: *const adjustmentRange_t =
                    adjustmentRanges(i_10);
                sbufWriteU8(dst, (*adjRange).adjustmentIndex);
                sbufWriteU8(dst, (*adjRange).auxChannelIndex);
                sbufWriteU8(dst, (*adjRange).range.startStep);
                sbufWriteU8(dst, (*adjRange).range.endStep);
                sbufWriteU8(dst, (*adjRange).adjustmentFunction);
                sbufWriteU8(dst, (*adjRange).auxSwitchChannelIndex);
                i_10 += 1
            }
        }
        131 => {
            sbufWriteU16(dst, (*motorConfig()).minthrottle);
            sbufWriteU16(dst, (*motorConfig()).maxthrottle);
            sbufWriteU16(dst, (*motorConfig()).mincommand);
        }
        133 => {
            sbufWriteU16(dst,
                         ((*compassConfig()).mag_declination as libc::c_int /
                              10 as libc::c_int) as uint16_t);
        }
        134 => {
            if feature(FEATURE_ESC_SENSOR as libc::c_int as uint32_t) {
                sbufWriteU8(dst, getMotorCount());
                let mut i_11: libc::c_int = 0 as libc::c_int;
                while i_11 < getMotorCount() as libc::c_int {
                    let mut escData: *const escSensorData_t =
                        getEscSensorData(i_11 as uint8_t);
                    sbufWriteU8(dst, (*escData).temperature as uint8_t);
                    sbufWriteU16(dst, (*escData).rpm as uint16_t);
                    i_11 += 1
                }
            } else { unsupportedCommand = 1 as libc::c_int != 0 }
        }
        132 => {
            sbufWriteU8(dst, (*gpsConfig()).provider as uint8_t);
            sbufWriteU8(dst, (*gpsConfig()).sbasMode as uint8_t);
            sbufWriteU8(dst, (*gpsConfig()).autoConfig as uint8_t);
            sbufWriteU8(dst, (*gpsConfig()).autoBaud as uint8_t);
        }
        106 => {
            sbufWriteU8(dst,
                        (stateFlags as libc::c_int & GPS_FIX as libc::c_int)
                            as uint8_t);
            sbufWriteU8(dst, gpsSol.numSat);
            sbufWriteU32(dst, gpsSol.llh.lat as uint32_t);
            sbufWriteU32(dst, gpsSol.llh.lon as uint32_t);
            sbufWriteU16(dst,
                         constrain(gpsSol.llh.alt / 100 as libc::c_int,
                                   0 as libc::c_int, 65535 as libc::c_int) as
                             uint16_t);
            sbufWriteU16(dst, gpsSol.groundSpeed);
            sbufWriteU16(dst, gpsSol.groundCourse);
        }
        107 => {
            sbufWriteU16(dst, GPS_distanceToHome);
            sbufWriteU16(dst, GPS_directionToHome as uint16_t);
            sbufWriteU8(dst,
                        (GPS_update as libc::c_int & 1 as libc::c_int) as
                            uint8_t);
        }
        164 => {
            sbufWriteU8(dst, GPS_numCh);
            let mut i_12: libc::c_int = 0 as libc::c_int;
            while i_12 < GPS_numCh as libc::c_int {
                sbufWriteU8(dst, GPS_svinfo_chn[i_12 as usize]);
                sbufWriteU8(dst, GPS_svinfo_svid[i_12 as usize]);
                sbufWriteU8(dst, GPS_svinfo_quality[i_12 as usize]);
                sbufWriteU8(dst, GPS_svinfo_cno[i_12 as usize]);
                i_12 += 1
            }
        }
        240 => {
            sbufWriteU16(dst,
                         (*accelerometerConfig()).accelerometerTrims.values.pitch
                             as uint16_t);
            sbufWriteU16(dst,
                         (*accelerometerConfig()).accelerometerTrims.values.roll
                             as uint16_t);
        }
        42 => {
            sbufWriteU8(dst, (*mixerConfig()).mixerMode);
            sbufWriteU8(dst, (*mixerConfig()).yaw_motors_reversed as uint8_t);
        }
        44 => {
            sbufWriteU8(dst, (*rxConfig()).serialrx_provider);
            sbufWriteU16(dst, (*rxConfig()).maxcheck);
            sbufWriteU16(dst, (*rxConfig()).midrc);
            sbufWriteU16(dst, (*rxConfig()).mincheck);
            sbufWriteU8(dst, (*rxConfig()).spektrum_sat_bind);
            sbufWriteU16(dst, (*rxConfig()).rx_min_usec);
            sbufWriteU16(dst, (*rxConfig()).rx_max_usec);
            sbufWriteU8(dst, (*rxConfig()).rcInterpolation);
            sbufWriteU8(dst, (*rxConfig()).rcInterpolationInterval);
            sbufWriteU16(dst,
                         ((*rxConfig()).airModeActivateThreshold as
                              libc::c_int * 10 as libc::c_int +
                              1000 as libc::c_int) as uint16_t);
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
            sbufWriteU32(dst, 0 as libc::c_int as uint32_t);
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
            sbufWriteU8(dst, (*rxConfig()).fpvCamAngleDegrees);
            sbufWriteU8(dst, (*rxConfig()).rcInterpolationChannels);
            sbufWriteU8(dst, (*rxConfig()).rc_smoothing_type);
            sbufWriteU8(dst, (*rxConfig()).rc_smoothing_input_cutoff);
            sbufWriteU8(dst, (*rxConfig()).rc_smoothing_derivative_cutoff);
            sbufWriteU8(dst, (*rxConfig()).rc_smoothing_input_type);
            sbufWriteU8(dst, (*rxConfig()).rc_smoothing_derivative_type);
            sbufWriteU8(dst, (*usbDevConfig()).type_0);
        }
        75 => {
            sbufWriteU8(dst, (*failsafeConfig()).failsafe_delay);
            sbufWriteU8(dst, (*failsafeConfig()).failsafe_off_delay);
            sbufWriteU16(dst, (*failsafeConfig()).failsafe_throttle);
            sbufWriteU8(dst, (*failsafeConfig()).failsafe_switch_mode);
            sbufWriteU16(dst,
                         (*failsafeConfig()).failsafe_throttle_low_delay);
            sbufWriteU8(dst, (*failsafeConfig()).failsafe_procedure);
        }
        77 => {
            let mut i_13: libc::c_int = 0 as libc::c_int;
            while i_13 < rxRuntimeConfig.channelCount as libc::c_int {
                sbufWriteU8(dst, (*rxFailsafeChannelConfigs(i_13)).mode);
                sbufWriteU16(dst,
                             (750 as libc::c_int +
                                  25 as libc::c_int *
                                      (*rxFailsafeChannelConfigs(i_13)).step
                                          as libc::c_int) as uint16_t);
                i_13 += 1
            }
        }
        50 => { sbufWriteU8(dst, (*rxConfig()).rssi_channel); }
        64 => {
            sbufWriteData(dst,
                          (*rxConfig()).rcmap.as_ptr() as *const libc::c_void,
                          8 as libc::c_int);
        }
        54 => {
            let mut i_14: libc::c_int = 0 as libc::c_int;
            while i_14 < 9 as libc::c_int {
                if serialIsPortAvailable((*serialConfig()).portConfigs[i_14 as
                                                                           usize].identifier)
                   {
                    sbufWriteU8(dst,
                                (*serialConfig()).portConfigs[i_14 as
                                                                  usize].identifier
                                    as uint8_t);
                    sbufWriteU16(dst,
                                 (*serialConfig()).portConfigs[i_14 as
                                                                   usize].functionMask);
                    sbufWriteU8(dst,
                                (*serialConfig()).portConfigs[i_14 as
                                                                  usize].msp_baudrateIndex);
                    sbufWriteU8(dst,
                                (*serialConfig()).portConfigs[i_14 as
                                                                  usize].gps_baudrateIndex);
                    sbufWriteU8(dst,
                                (*serialConfig()).portConfigs[i_14 as
                                                                  usize].telemetry_baudrateIndex);
                    sbufWriteU8(dst,
                                (*serialConfig()).portConfigs[i_14 as
                                                                  usize].blackbox_baudrateIndex);
                }
                i_14 += 1
            }
        }
        46 => {
            let mut i_15: libc::c_int = 0 as libc::c_int;
            while i_15 < 16 as libc::c_int {
                let mut color: *const hsvColor_t =
                    &*(*(ledStripConfig as
                             unsafe extern "C" fn()
                                 ->
                                     *const ledStripConfig_t)()).colors.as_ptr().offset(i_15
                                                                                            as
                                                                                            isize)
                        as *const hsvColor_t;
                sbufWriteU16(dst, (*color).h);
                sbufWriteU8(dst, (*color).s);
                sbufWriteU8(dst, (*color).v);
                i_15 += 1
            }
        }
        48 => {
            let mut i_16: libc::c_int = 0 as libc::c_int;
            while i_16 < 32 as libc::c_int {
                let mut ledConfig: *const ledConfig_t =
                    &*(*(ledStripConfig as
                             unsafe extern "C" fn()
                                 ->
                                     *const ledStripConfig_t)()).ledConfigs.as_ptr().offset(i_16
                                                                                                as
                                                                                                isize)
                        as *const ledConfig_t;
                sbufWriteU32(dst, *ledConfig);
                i_16 += 1
            }
        }
        127 => {
            let mut i_17: libc::c_int = 0 as libc::c_int;
            while i_17 < 6 as libc::c_int {
                let mut j: libc::c_int = 0 as libc::c_int;
                while j < 6 as libc::c_int {
                    sbufWriteU8(dst, i_17 as uint8_t);
                    sbufWriteU8(dst, j as uint8_t);
                    sbufWriteU8(dst,
                                (*ledStripConfig()).modeColors[i_17 as
                                                                   usize].color[j
                                                                                    as
                                                                                    usize]);
                    j += 1
                }
                i_17 += 1
            }
            let mut j_0: libc::c_int = 0 as libc::c_int;
            while j_0 < 11 as libc::c_int {
                sbufWriteU8(dst, 6 as libc::c_int as uint8_t);
                sbufWriteU8(dst, j_0 as uint8_t);
                sbufWriteU8(dst,
                            (*ledStripConfig()).specialColors.color[j_0 as
                                                                        usize]);
                j_0 += 1
            }
            sbufWriteU8(dst, LED_AUX_CHANNEL as libc::c_int as uint8_t);
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
            sbufWriteU8(dst, (*ledStripConfig()).ledstrip_aux_channel);
        }
        70 => { serializeDataflashSummaryReply(dst); }
        80 => {
            sbufWriteU8(dst, 1 as libc::c_int as uint8_t);
            sbufWriteU8(dst, (*blackboxConfig()).device);
            sbufWriteU8(dst, 1 as libc::c_int as uint8_t);
            sbufWriteU8(dst, blackboxGetRateDenom());
            sbufWriteU16(dst, (*blackboxConfig()).p_ratio);
        }
        79 => { serializeSDCardSummaryReply(dst); }
        124 => {
            sbufWriteU16(dst, (*flight3DConfig()).deadband3d_low);
            sbufWriteU16(dst, (*flight3DConfig()).deadband3d_high);
            sbufWriteU16(dst, (*flight3DConfig()).neutral3d);
        }
        125 => {
            sbufWriteU8(dst, (*rcControlsConfig()).deadband);
            sbufWriteU8(dst, (*rcControlsConfig()).yaw_deadband);
            sbufWriteU8(dst, (*rcControlsConfig()).alt_hold_deadband);
            sbufWriteU16(dst, (*flight3DConfig()).deadband3d_throttle);
        }
        126 => {
            sbufWriteU8(dst, (*gyroConfig()).gyro_align);
            sbufWriteU8(dst, (*accelerometerConfig()).acc_align as uint8_t);
            sbufWriteU8(dst, (*compassConfig()).mag_align as uint8_t);
        }
        90 => {
            sbufWriteU8(dst, (*gyroConfig()).gyro_sync_denom);
            sbufWriteU8(dst, (*pidConfig()).pid_process_denom);
            sbufWriteU8(dst, (*motorConfig()).dev.useUnsyncedPwm);
            sbufWriteU8(dst, (*motorConfig()).dev.motorPwmProtocol);
            sbufWriteU16(dst, (*motorConfig()).dev.motorPwmRate);
            sbufWriteU16(dst, (*motorConfig()).digitalIdleOffsetValue);
            sbufWriteU8(dst, (*gyroConfig()).gyro_use_32khz);
            sbufWriteU8(dst, (*motorConfig()).dev.motorPwmInversion);
            sbufWriteU8(dst, (*gyroConfig()).gyro_to_use);
            sbufWriteU8(dst, (*gyroConfig()).gyro_high_fsr);
            sbufWriteU8(dst,
                        (*gyroConfig()).gyroMovementCalibrationThreshold);
            sbufWriteU16(dst, (*gyroConfig()).gyroCalibrationDuration);
            sbufWriteU16(dst, (*gyroConfig()).gyro_offset_yaw as uint16_t);
            sbufWriteU8(dst, (*gyroConfig()).checkOverflow);
        }
        92 => {
            sbufWriteU8(dst, (*gyroConfig()).gyro_lowpass_hz as uint8_t);
            sbufWriteU16(dst, (*currentPidProfile).dterm_lowpass_hz);
            sbufWriteU16(dst, (*currentPidProfile).yaw_lowpass_hz);
            sbufWriteU16(dst, (*gyroConfig()).gyro_soft_notch_hz_1);
            sbufWriteU16(dst, (*gyroConfig()).gyro_soft_notch_cutoff_1);
            sbufWriteU16(dst, (*currentPidProfile).dterm_notch_hz);
            sbufWriteU16(dst, (*currentPidProfile).dterm_notch_cutoff);
            sbufWriteU16(dst, (*gyroConfig()).gyro_soft_notch_hz_2);
            sbufWriteU16(dst, (*gyroConfig()).gyro_soft_notch_cutoff_2);
            sbufWriteU8(dst, (*currentPidProfile).dterm_filter_type);
            sbufWriteU8(dst, (*gyroConfig()).gyro_hardware_lpf);
            sbufWriteU8(dst, (*gyroConfig()).gyro_32khz_hardware_lpf);
            sbufWriteU16(dst, (*gyroConfig()).gyro_lowpass_hz);
            sbufWriteU16(dst, (*gyroConfig()).gyro_lowpass2_hz);
            sbufWriteU8(dst, (*gyroConfig()).gyro_lowpass_type);
            sbufWriteU8(dst, (*gyroConfig()).gyro_lowpass2_type);
            sbufWriteU16(dst, (*currentPidProfile).dterm_lowpass2_hz);
        }
        94 => {
            sbufWriteU16(dst, 0 as libc::c_int as uint16_t);
            sbufWriteU16(dst, 0 as libc::c_int as uint16_t);
            sbufWriteU16(dst, 0 as libc::c_int as uint16_t);
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
            sbufWriteU8(dst, (*currentPidProfile).vbatPidCompensation);
            sbufWriteU8(dst, (*currentPidProfile).feedForwardTransition);
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
            sbufWriteU16(dst, (*currentPidProfile).rateAccelLimit);
            sbufWriteU16(dst, (*currentPidProfile).yawRateAccelLimit);
            sbufWriteU8(dst, (*currentPidProfile).levelAngleLimit);
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
            sbufWriteU16(dst, (*currentPidProfile).itermThrottleThreshold);
            sbufWriteU16(dst, (*currentPidProfile).itermAcceleratorGain);
            sbufWriteU16(dst, 0 as libc::c_int as uint16_t);
            sbufWriteU8(dst, (*currentPidProfile).iterm_rotation);
            sbufWriteU8(dst, (*currentPidProfile).smart_feedforward);
            sbufWriteU8(dst, (*currentPidProfile).iterm_relax);
            sbufWriteU8(dst, (*currentPidProfile).iterm_relax_type);
            sbufWriteU8(dst, (*currentPidProfile).abs_control_gain);
            sbufWriteU8(dst, (*currentPidProfile).throttle_boost);
            sbufWriteU8(dst, (*currentPidProfile).acro_trainer_angle_limit);
            sbufWriteU16(dst,
                         (*currentPidProfile).pid[PID_ROLL as libc::c_int as
                                                      usize].F);
            sbufWriteU16(dst,
                         (*currentPidProfile).pid[PID_PITCH as libc::c_int as
                                                      usize].F);
            sbufWriteU16(dst,
                         (*currentPidProfile).pid[PID_YAW as libc::c_int as
                                                      usize].F);
            sbufWriteU8(dst, (*currentPidProfile).antiGravityMode);
        }
        96 => {
            sbufWriteU8(dst, (*accelerometerConfig()).acc_hardware);
            sbufWriteU8(dst, (*barometerConfig()).baro_hardware);
            sbufWriteU8(dst, (*compassConfig()).mag_hardware);
        }
        88 => {
            let mut vtxDevice: *const vtxDevice_t = vtxCommonDevice();
            if !vtxDevice.is_null() {
                let mut pitmode: uint8_t = 0 as libc::c_int as uint8_t;
                vtxCommonGetPitMode(vtxDevice, &mut pitmode);
                sbufWriteU8(dst,
                            vtxCommonGetDeviceType(vtxDevice) as uint8_t);
                sbufWriteU8(dst, (*vtxSettingsConfig()).band);
                sbufWriteU8(dst, (*vtxSettingsConfig()).channel);
                sbufWriteU8(dst, (*vtxSettingsConfig()).power);
                sbufWriteU8(dst, pitmode);
                sbufWriteU16(dst, (*vtxSettingsConfig()).freq);
                // future extensions here...
            } else {
                sbufWriteU8(dst, VTXDEV_UNKNOWN as libc::c_int as uint8_t);
                // no VTX detected
            }
        }
        187 => {
            sbufWriteU8(dst, rssiSource as uint8_t);
            rtcDateTimeIsSet = 0 as libc::c_int as uint8_t;
            dt =
                dateTime_t{year: 0,
                           month: 0,
                           day: 0,
                           hours: 0,
                           minutes: 0,
                           seconds: 0,
                           millis: 0,};
            if rtcGetDateTime(&mut dt) {
                rtcDateTimeIsSet = 1 as libc::c_int as uint8_t
            }
            sbufWriteU8(dst, rtcDateTimeIsSet);
        }
        247 => {
            let mut dt_0: dateTime_t =
                dateTime_t{year: 0,
                           month: 0,
                           day: 0,
                           hours: 0,
                           minutes: 0,
                           seconds: 0,
                           millis: 0,};
            if rtcGetDateTime(&mut dt_0) {
                sbufWriteU16(dst, dt_0.year);
                sbufWriteU8(dst, dt_0.month);
                sbufWriteU8(dst, dt_0.day);
                sbufWriteU8(dst, dt_0.hours);
                sbufWriteU8(dst, dt_0.minutes);
                sbufWriteU8(dst, dt_0.seconds);
                sbufWriteU16(dst, dt_0.millis);
            }
        }
        _ => { unsupportedCommand = 1 as libc::c_int != 0 }
    }
    return !unsupportedCommand;
}
// USE_OSD_SLAVE
unsafe extern "C" fn mspFcProcessOutCommandWithArg(mut cmdMSP: uint8_t,
                                                   mut src: *mut sbuf_t,
                                                   mut dst: *mut sbuf_t,
                                                   mut mspPostProcessFn:
                                                       *mut mspPostProcessFnPtr)
 -> mspResult_e {
    match cmdMSP as libc::c_int {
        116 => {
            let page: libc::c_int =
                if sbufBytesRemaining(src) != 0 {
                    sbufReadU8(src) as libc::c_int
                } else {
                    0 as libc::c_int
                }; // 0 = pid profile, 1 = control rate profile
            serializeBoxReply(dst, page,
                              Some(serializeBoxNameFn as
                                       unsafe extern "C" fn(_: *mut sbuf_s,
                                                            _: *const box_t)
                                           -> ())); // reserved
        }
        119 => {
            let page_0: libc::c_int =
                if sbufBytesRemaining(src) != 0 {
                    sbufReadU8(src) as libc::c_int
                } else { 0 as libc::c_int };
            serializeBoxReply(dst, page_0,
                              Some(serializeBoxPermanentIdFn as
                                       unsafe extern "C" fn(_: *mut sbuf_s,
                                                            _: *const box_t)
                                           -> ()));
        }
        68 => {
            if sbufBytesRemaining(src) != 0 {
                rebootMode = sbufReadU8(src);
                if rebootMode as libc::c_int >=
                       MSP_REBOOT_COUNT as libc::c_int {
                    return MSP_RESULT_ERROR
                }
            } else {
                rebootMode = MSP_REBOOT_FIRMWARE as libc::c_int as uint8_t
            }
            sbufWriteU8(dst, rebootMode);
            if rebootMode as libc::c_int == MSP_REBOOT_MSC as libc::c_int {
                if mscCheckFilesystemReady() {
                    sbufWriteU8(dst, 1 as libc::c_int as uint8_t);
                } else {
                    sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
                    return MSP_RESULT_ACK
                }
            }
            if !mspPostProcessFn.is_null() {
                *mspPostProcessFn =
                    Some(mspRebootFn as
                             unsafe extern "C" fn(_: *mut serialPort_t) -> ())
            }
        }
        _ => { return MSP_RESULT_CMD_UNKNOWN }
    }
    return MSP_RESULT_ACK;
}
unsafe extern "C" fn mspProcessInCommand(mut cmdMSP: uint8_t,
                                         mut src: *mut sbuf_t)
 -> mspResult_e {
    let mut i: uint32_t = 0;
    let mut value: uint8_t = 0;
    let dataSize: libc::c_uint = sbufBytesRemaining(src) as libc::c_uint;
    let mut dstProfileIndex: uint8_t = 0;
    let mut srcProfileIndex: uint8_t = 0;
    match cmdMSP as libc::c_int {
        210 => {
            value = sbufReadU8(src);
            if value as libc::c_int & (1 as libc::c_int) << 7 as libc::c_int
                   == 0 as libc::c_int {
                if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
                    if value as libc::c_int >= 3 as libc::c_int {
                        value = 0 as libc::c_int as uint8_t
                    }
                    changePidProfile(value);
                }
            } else {
                value =
                    (value as libc::c_int &
                         !((1 as libc::c_int) << 7 as libc::c_int)) as
                        uint8_t;
                if value as libc::c_int >= 6 as libc::c_int {
                    value = 0 as libc::c_int as uint8_t
                }
                changeControlRateProfile(value);
            }
        }
        183 => {
            value = sbufReadU8(src);
            dstProfileIndex = sbufReadU8(src);
            srcProfileIndex = sbufReadU8(src);
            if value as libc::c_int == 0 as libc::c_int {
                pidCopyProfile(dstProfileIndex, srcProfileIndex);
            } else if value as libc::c_int == 1 as libc::c_int {
                copyControlRateProfile(dstProfileIndex, srcProfileIndex);
            }
        }
        211 => { magHold = sbufReadU16(src) as int16_t }
        200 => {
            let mut channelCount: uint8_t =
                (dataSize as
                     libc::c_ulong).wrapping_div(::core::mem::size_of::<uint16_t>()
                                                     as libc::c_ulong) as
                    uint8_t;
            if channelCount as libc::c_int > 18 as libc::c_int {
                return MSP_RESULT_ERROR
            } else {
                let mut frame: [uint16_t; 18] = [0; 18];
                let mut i_0: libc::c_int = 0 as libc::c_int;
                while i_0 < channelCount as libc::c_int {
                    frame[i_0 as usize] = sbufReadU16(src);
                    i_0 += 1
                }
                rxMspFrameReceive(frame.as_mut_ptr(),
                                  channelCount as libc::c_int);
            }
        }
        239 => {
            (*accelerometerConfigMutable()).accelerometerTrims.values.pitch =
                sbufReadU16(src) as int16_t;
            (*accelerometerConfigMutable()).accelerometerTrims.values.roll =
                sbufReadU16(src) as int16_t
        }
        62 => {
            (*armingConfigMutable()).auto_disarm_delay = sbufReadU8(src);
            sbufReadU8(src);
            if sbufBytesRemaining(src) != 0 {
                (*imuConfigMutable()).small_angle = sbufReadU8(src)
            }
        }
        60 => { }
        202 => {
            let mut i_1: libc::c_int = 0 as libc::c_int;
            while i_1 < PID_ITEM_COUNT as libc::c_int {
                (*currentPidProfile).pid[i_1 as usize].P = sbufReadU8(src);
                (*currentPidProfile).pid[i_1 as usize].I = sbufReadU8(src);
                (*currentPidProfile).pid[i_1 as usize].D = sbufReadU8(src);
                i_1 += 1
            }
            pidInitConfig(currentPidProfile);
        }
        35 => {
            i = sbufReadU8(src) as uint32_t;
            if i < 20 as libc::c_int as libc::c_uint {
                let mut mac: *mut modeActivationCondition_t =
                    modeActivationConditionsMutable(i as libc::c_int);
                i = sbufReadU8(src) as uint32_t;
                let mut box_0: *const box_t =
                    findBoxByPermanentId(i as uint8_t);
                if !box_0.is_null() {
                    (*mac).modeId = (*box_0).boxId as boxId_e;
                    (*mac).auxChannelIndex = sbufReadU8(src);
                    (*mac).range.startStep = sbufReadU8(src);
                    (*mac).range.endStep = sbufReadU8(src);
                    useRcControlsConfig(currentPidProfile);
                } else { return MSP_RESULT_ERROR }
            } else { return MSP_RESULT_ERROR }
        }
        53 => {
            i = sbufReadU8(src) as uint32_t;
            if i < 15 as libc::c_int as libc::c_uint {
                let mut adjRange: *mut adjustmentRange_t =
                    adjustmentRangesMutable(i as libc::c_int);
                i = sbufReadU8(src) as uint32_t;
                if i < 4 as libc::c_int as libc::c_uint {
                    (*adjRange).adjustmentIndex = i as uint8_t;
                    (*adjRange).auxChannelIndex = sbufReadU8(src);
                    (*adjRange).range.startStep = sbufReadU8(src);
                    (*adjRange).range.endStep = sbufReadU8(src);
                    (*adjRange).adjustmentFunction = sbufReadU8(src);
                    (*adjRange).auxSwitchChannelIndex = sbufReadU8(src)
                } else { return MSP_RESULT_ERROR }
            } else { return MSP_RESULT_ERROR }
        }
        204 => {
            if sbufBytesRemaining(src) >= 10 as libc::c_int {
                value = sbufReadU8(src);
                if (*currentControlRateProfile).rcRates[FD_PITCH as
                                                            libc::c_int as
                                                            usize] as
                       libc::c_int ==
                       (*currentControlRateProfile).rcRates[FD_ROLL as
                                                                libc::c_int as
                                                                usize] as
                           libc::c_int {
                    (*currentControlRateProfile).rcRates[FD_PITCH as
                                                             libc::c_int as
                                                             usize] = value
                }
                (*currentControlRateProfile).rcRates[FD_ROLL as libc::c_int as
                                                         usize] = value;
                value = sbufReadU8(src);
                if (*currentControlRateProfile).rcExpo[FD_PITCH as libc::c_int
                                                           as usize] as
                       libc::c_int ==
                       (*currentControlRateProfile).rcExpo[FD_ROLL as
                                                               libc::c_int as
                                                               usize] as
                           libc::c_int {
                    (*currentControlRateProfile).rcExpo[FD_PITCH as
                                                            libc::c_int as
                                                            usize] = value
                }
                (*currentControlRateProfile).rcExpo[FD_ROLL as libc::c_int as
                                                        usize] = value;
                let mut i_2: libc::c_int = 0 as libc::c_int;
                while i_2 < 3 as libc::c_int {
                    (*currentControlRateProfile).rates[i_2 as usize] =
                        sbufReadU8(src);
                    i_2 += 1
                }
                value = sbufReadU8(src);
                (*currentControlRateProfile).dynThrPID =
                    ({
                         let mut _a: uint8_t = value;
                         let mut _b: libc::c_int = 100 as libc::c_int;
                         if (_a as libc::c_int) < _b {
                             _a as libc::c_int
                         } else { _b }
                     }) as uint8_t;
                (*currentControlRateProfile).thrMid8 = sbufReadU8(src);
                (*currentControlRateProfile).thrExpo8 = sbufReadU8(src);
                (*currentControlRateProfile).tpa_breakpoint =
                    sbufReadU16(src);
                if sbufBytesRemaining(src) >= 1 as libc::c_int {
                    (*currentControlRateProfile).rcExpo[FD_YAW as libc::c_int
                                                            as usize] =
                        sbufReadU8(src)
                }
                if sbufBytesRemaining(src) >= 1 as libc::c_int {
                    (*currentControlRateProfile).rcRates[FD_YAW as libc::c_int
                                                             as usize] =
                        sbufReadU8(src)
                }
                if sbufBytesRemaining(src) >= 1 as libc::c_int {
                    (*currentControlRateProfile).rcRates[FD_PITCH as
                                                             libc::c_int as
                                                             usize] =
                        sbufReadU8(src)
                }
                if sbufBytesRemaining(src) >= 1 as libc::c_int {
                    (*currentControlRateProfile).rcExpo[FD_PITCH as
                                                            libc::c_int as
                                                            usize] =
                        sbufReadU8(src)
                }
                initRcProcessing();
            } else { return MSP_RESULT_ERROR }
        }
        222 => {
            (*motorConfigMutable()).minthrottle = sbufReadU16(src);
            (*motorConfigMutable()).maxthrottle = sbufReadU16(src);
            (*motorConfigMutable()).mincommand = sbufReadU16(src)
        }
        223 => {
            (*gpsConfigMutable()).provider = sbufReadU8(src) as gpsProvider_e;
            (*gpsConfigMutable()).sbasMode = sbufReadU8(src) as sbasMode_e;
            (*gpsConfigMutable()).autoConfig =
                sbufReadU8(src) as gpsAutoConfig_e;
            (*gpsConfigMutable()).autoBaud = sbufReadU8(src) as gpsAutoBaud_e
        }
        224 => {
            (*compassConfigMutable()).mag_declination =
                (sbufReadU16(src) as libc::c_int * 10 as libc::c_int) as
                    int16_t
        }
        214 => {
            let mut i_3: libc::c_int = 0 as libc::c_int;
            while i_3 < getMotorCount() as libc::c_int {
                motor_disarmed[i_3 as usize] =
                    convertExternalToMotor(sbufReadU16(src));
                i_3 += 1
            }
        }
        212 => {
            if dataSize !=
                   (1 as libc::c_int + 12 as libc::c_int) as libc::c_uint {
                return MSP_RESULT_ERROR
            }
            i = sbufReadU8(src) as uint32_t;
            if i >= 8 as libc::c_int as libc::c_uint {
                return MSP_RESULT_ERROR
            } else {
                (*servoParamsMutable(i as libc::c_int)).min =
                    sbufReadU16(src) as int16_t;
                (*servoParamsMutable(i as libc::c_int)).max =
                    sbufReadU16(src) as int16_t;
                (*servoParamsMutable(i as libc::c_int)).middle =
                    sbufReadU16(src) as int16_t;
                (*servoParamsMutable(i as libc::c_int)).rate =
                    sbufReadU8(src) as int8_t;
                (*servoParamsMutable(i as libc::c_int)).forwardFromChannel =
                    sbufReadU8(src) as int8_t;
                (*servoParamsMutable(i as libc::c_int)).reversedSources =
                    sbufReadU32(src)
            }
        }
        242 => {
            i = sbufReadU8(src) as uint32_t;
            if i >= (2 as libc::c_int * 8 as libc::c_int) as libc::c_uint {
                return MSP_RESULT_ERROR
            } else {
                (*customServoMixersMutable(i as libc::c_int)).targetChannel =
                    sbufReadU8(src);
                (*customServoMixersMutable(i as libc::c_int)).inputSource =
                    sbufReadU8(src);
                (*customServoMixersMutable(i as libc::c_int)).rate =
                    sbufReadU8(src) as int8_t;
                (*customServoMixersMutable(i as libc::c_int)).speed =
                    sbufReadU8(src);
                (*customServoMixersMutable(i as libc::c_int)).min =
                    sbufReadU8(src) as int8_t;
                (*customServoMixersMutable(i as libc::c_int)).max =
                    sbufReadU8(src) as int8_t;
                (*customServoMixersMutable(i as libc::c_int)).box_0 =
                    sbufReadU8(src);
                loadCustomServoMixer();
            }
        }
        217 => {
            (*flight3DConfigMutable()).deadband3d_low = sbufReadU16(src);
            (*flight3DConfigMutable()).deadband3d_high = sbufReadU16(src);
            (*flight3DConfigMutable()).neutral3d = sbufReadU16(src)
        }
        218 => {
            (*rcControlsConfigMutable()).deadband = sbufReadU8(src);
            (*rcControlsConfigMutable()).yaw_deadband = sbufReadU8(src);
            (*rcControlsConfigMutable()).alt_hold_deadband = sbufReadU8(src);
            (*flight3DConfigMutable()).deadband3d_throttle = sbufReadU16(src)
        }
        219 => { resetPidProfile(currentPidProfile); }
        220 => {
            (*gyroConfigMutable()).gyro_align = sbufReadU8(src);
            (*accelerometerConfigMutable()).acc_align =
                sbufReadU8(src) as sensor_align_e;
            (*compassConfigMutable()).mag_align =
                sbufReadU8(src) as sensor_align_e
        }
        91 => {
            (*gyroConfigMutable()).gyro_sync_denom = sbufReadU8(src);
            (*pidConfigMutable()).pid_process_denom = sbufReadU8(src);
            (*motorConfigMutable()).dev.useUnsyncedPwm = sbufReadU8(src);
            (*motorConfigMutable()).dev.motorPwmProtocol =
                constrain(sbufReadU8(src) as libc::c_int, 0 as libc::c_int,
                          PWM_TYPE_MAX as libc::c_int - 1 as libc::c_int) as
                    uint8_t;
            (*motorConfigMutable()).dev.motorPwmRate = sbufReadU16(src);
            if sbufBytesRemaining(src) >= 2 as libc::c_int {
                (*motorConfigMutable()).digitalIdleOffsetValue =
                    sbufReadU16(src)
            }
            if sbufBytesRemaining(src) != 0 {
                (*gyroConfigMutable()).gyro_use_32khz = sbufReadU8(src)
            }
            if sbufBytesRemaining(src) != 0 {
                (*motorConfigMutable()).dev.motorPwmInversion =
                    sbufReadU8(src)
            }
            if sbufBytesRemaining(src) >= 8 as libc::c_int {
                (*gyroConfigMutable()).gyro_to_use = sbufReadU8(src);
                (*gyroConfigMutable()).gyro_high_fsr = sbufReadU8(src);
                (*gyroConfigMutable()).gyroMovementCalibrationThreshold =
                    sbufReadU8(src);
                (*gyroConfigMutable()).gyroCalibrationDuration =
                    sbufReadU16(src);
                (*gyroConfigMutable()).gyro_offset_yaw =
                    sbufReadU16(src) as int16_t;
                (*gyroConfigMutable()).checkOverflow = sbufReadU8(src)
            }
            validateAndFixGyroConfig();
        }
        93 => {
            (*gyroConfigMutable()).gyro_lowpass_hz =
                sbufReadU8(src) as uint16_t;
            (*currentPidProfile).dterm_lowpass_hz = sbufReadU16(src);
            (*currentPidProfile).yaw_lowpass_hz = sbufReadU16(src);
            if sbufBytesRemaining(src) >= 8 as libc::c_int {
                (*gyroConfigMutable()).gyro_soft_notch_hz_1 =
                    sbufReadU16(src);
                (*gyroConfigMutable()).gyro_soft_notch_cutoff_1 =
                    sbufReadU16(src);
                (*currentPidProfile).dterm_notch_hz = sbufReadU16(src);
                (*currentPidProfile).dterm_notch_cutoff = sbufReadU16(src)
            }
            if sbufBytesRemaining(src) >= 4 as libc::c_int {
                (*gyroConfigMutable()).gyro_soft_notch_hz_2 =
                    sbufReadU16(src);
                (*gyroConfigMutable()).gyro_soft_notch_cutoff_2 =
                    sbufReadU16(src)
            }
            if sbufBytesRemaining(src) >= 1 as libc::c_int {
                (*currentPidProfile).dterm_filter_type = sbufReadU8(src)
            }
            if sbufBytesRemaining(src) >= 10 as libc::c_int {
                (*gyroConfigMutable()).gyro_hardware_lpf = sbufReadU8(src);
                (*gyroConfigMutable()).gyro_32khz_hardware_lpf =
                    sbufReadU8(src);
                (*gyroConfigMutable()).gyro_lowpass_hz = sbufReadU16(src);
                (*gyroConfigMutable()).gyro_lowpass2_hz = sbufReadU16(src);
                (*gyroConfigMutable()).gyro_lowpass_type = sbufReadU8(src);
                (*gyroConfigMutable()).gyro_lowpass2_type = sbufReadU8(src);
                (*currentPidProfile).dterm_lowpass2_hz = sbufReadU16(src)
            }
            // reinitialize the gyro filters with the new values
            validateAndFixGyroConfig();
            gyroInitFilters();
            // reinitialize the PID filters with the new values
            pidInitFilters(currentPidProfile); // was pidProfile.yaw_p_limit
        }
        95 => {
            sbufReadU16(src); // reserved
            sbufReadU16(src); // was low byte of currentPidProfile->dtermSetpointWeight
            sbufReadU16(src); // reserved
            sbufReadU8(src); // reserved
            (*currentPidProfile).vbatPidCompensation =
                sbufReadU8(src); // reserved
            (*currentPidProfile).feedForwardTransition = sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
            (*currentPidProfile).rateAccelLimit = sbufReadU16(src);
            (*currentPidProfile).yawRateAccelLimit = sbufReadU16(src);
            if sbufBytesRemaining(src) >= 2 as libc::c_int {
                (*currentPidProfile).levelAngleLimit = sbufReadU8(src);
                sbufReadU8(src);
                // was pidProfile.levelSensitivity
            }
            if sbufBytesRemaining(src) >= 4 as libc::c_int {
                (*currentPidProfile).itermThrottleThreshold =
                    sbufReadU16(src);
                (*currentPidProfile).itermAcceleratorGain = sbufReadU16(src)
            }
            if sbufBytesRemaining(src) >= 2 as libc::c_int {
                sbufReadU16(src);
                // was currentPidProfile->dtermSetpointWeight
            }
            if sbufBytesRemaining(src) >= 14 as libc::c_int {
                // Added in MSP API 1.40
                (*currentPidProfile).iterm_rotation = sbufReadU8(src);
                (*currentPidProfile).smart_feedforward = sbufReadU8(src);
                (*currentPidProfile).iterm_relax = sbufReadU8(src);
                (*currentPidProfile).iterm_relax_type = sbufReadU8(src);
                (*currentPidProfile).abs_control_gain = sbufReadU8(src);
                (*currentPidProfile).throttle_boost = sbufReadU8(src);
                (*currentPidProfile).acro_trainer_angle_limit =
                    sbufReadU8(src);
                // PID controller feedforward terms
                (*currentPidProfile).pid[PID_ROLL as libc::c_int as usize].F =
                    sbufReadU16(src);
                (*currentPidProfile).pid[PID_PITCH as libc::c_int as usize].F
                    = sbufReadU16(src);
                (*currentPidProfile).pid[PID_YAW as libc::c_int as usize].F =
                    sbufReadU16(src);
                (*currentPidProfile).antiGravityMode = sbufReadU8(src)
            }
            pidInitConfig(currentPidProfile);
        }
        97 => {
            (*accelerometerConfigMutable()).acc_hardware = sbufReadU8(src);
            (*barometerConfigMutable()).baro_hardware = sbufReadU8(src);
            (*compassConfigMutable()).mag_hardware = sbufReadU8(src)
        }
        208 => {
            if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
                resetEEPROM();
                readEEPROM();
            }
        }
        205 => {
            if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
                accSetCalibrationCycles(400 as libc::c_int as uint16_t);
            }
        }
        206 => {
            if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
                stateFlags =
                    (stateFlags as libc::c_int | CALIBRATE_MAG as libc::c_int)
                        as uint8_t
            }
        }
        250 => {
            if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
                return MSP_RESULT_ERROR
            }
            writeEEPROM();
            readEEPROM();
        }
        81 => {
            // Don't allow config to be updated while Blackbox is logging
            if blackboxMayEditConfig() {
                (*blackboxConfigMutable()).device =
                    sbufReadU8(src); // was rate_num
                let rateNum: libc::c_int =
                    sbufReadU8(src) as libc::c_int; // was rate_denom
                let rateDenom: libc::c_int = sbufReadU8(src) as libc::c_int;
                if sbufBytesRemaining(src) >= 2 as libc::c_int {
                    // p_ratio specified, so use it directly
                    (*blackboxConfigMutable()).p_ratio = sbufReadU16(src)
                } else {
                    // p_ratio not specified in MSP, so calculate it from old rateNum and rateDenom
                    (*blackboxConfigMutable()).p_ratio =
                        blackboxCalculatePDenom(rateNum, rateDenom) as
                            uint16_t
                }
            }
        }
        89 => {
            let mut vtxDevice: *mut vtxDevice_t =
                vtxCommonDevice(); //value is frequency in MHz
            if !vtxDevice.is_null() {
                if vtxCommonGetDeviceType(vtxDevice) as libc::c_uint !=
                       VTXDEV_UNKNOWN as libc::c_int as libc::c_uint {
                    let mut newFrequency: uint16_t = sbufReadU16(src);
                    if newFrequency as libc::c_int <=
                           (((7 as libc::c_int) << 3 as libc::c_int) +
                                7 as libc::c_int) as uint16_t as libc::c_int {
                        let newBand: uint8_t =
                            (newFrequency as libc::c_int / 8 as libc::c_int +
                                 1 as libc::c_int) as uint8_t;
                        let newChannel: uint8_t =
                            (newFrequency as libc::c_int % 8 as libc::c_int +
                                 1 as libc::c_int) as uint8_t;
                        (*vtxSettingsConfigMutable()).band = newBand;
                        (*vtxSettingsConfigMutable()).channel = newChannel;
                        (*vtxSettingsConfigMutable()).freq =
                            vtx58_Bandchan2Freq(newBand, newChannel)
                        //value is band and channel
                    } else {
                        (*vtxSettingsConfigMutable()).band =
                            0 as libc::c_int as uint8_t;
                        (*vtxSettingsConfigMutable()).freq = newFrequency
                    }
                    if sbufBytesRemaining(src) > 1 as libc::c_int {
                        (*vtxSettingsConfigMutable()).power = sbufReadU8(src);
                        // Delegate pitmode to vtx directly
                        let newPitmode: uint8_t =
                            sbufReadU8(src); // alt changed from 1m to 0.01m per lsb since MSP API 1.39 by RTH. Received MSP altitudes in 1m per lsb have to upscaled.
                        let mut currentPitmode: uint8_t =
                            0 as libc::c_int as
                                uint8_t; // New data signalisation to GPS functions // FIXME Magic Numbers
                        vtxCommonGetPitMode(vtxDevice, &mut currentPitmode);
                        if currentPitmode as libc::c_int !=
                               newPitmode as libc::c_int {
                            vtxCommonSetPitMode(vtxDevice, newPitmode);
                        }
                    }
                }
            }
        }
        98 => {
            if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
                return MSP_RESULT_ERROR
            }
            let key: uint8_t = sbufReadU8(src);
            cameraControlKeyPress(key as cameraControlKey_e,
                                  0 as libc::c_int as uint32_t);
        }
        99 => {
            let command: uint8_t = sbufReadU8(src);
            let mut disableRunawayTakeoff: uint8_t =
                0 as libc::c_int as uint8_t;
            if sbufBytesRemaining(src) != 0 {
                disableRunawayTakeoff = sbufReadU8(src)
            }
            if command != 0 {
                setArmingDisabled(ARMING_DISABLED_MSP);
                if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
                    disarm();
                }
                runawayTakeoffTemporaryDisable(0 as libc::c_int as uint8_t);
            } else {
                unsetArmingDisabled(ARMING_DISABLED_MSP);
                runawayTakeoffTemporaryDisable(disableRunawayTakeoff);
            }
        }
        201 => {
            if sbufReadU8(src) != 0 {
                stateFlags =
                    (stateFlags as libc::c_int | GPS_FIX as libc::c_int) as
                        uint8_t
            } else {
                stateFlags =
                    (stateFlags as libc::c_int & !(GPS_FIX as libc::c_int)) as
                        uint8_t
            }
            gpsSol.numSat = sbufReadU8(src);
            gpsSol.llh.lat = sbufReadU32(src) as int32_t;
            gpsSol.llh.lon = sbufReadU32(src) as int32_t;
            gpsSol.llh.alt =
                sbufReadU16(src) as libc::c_int * 100 as libc::c_int;
            gpsSol.groundSpeed = sbufReadU16(src);
            GPS_update =
                (GPS_update as libc::c_int | 2 as libc::c_int) as uint8_t
        }
        37 => {
            // USE_GPS
            featureClearAll(); // features bitmap
            featureSet(sbufReadU32(src));
        }
        185 => {
            (*beeperConfigMutable()).beeper_off_flags = sbufReadU32(src);
            if sbufBytesRemaining(src) >= 1 as libc::c_int {
                (*beeperConfigMutable()).dshotBeaconTone = sbufReadU8(src)
            }
            if sbufBytesRemaining(src) >= 4 as libc::c_int {
                (*beeperConfigMutable()).dshotBeaconOffFlags =
                    sbufReadU32(src)
            }
        }
        39 => {
            (*boardAlignmentMutable()).rollDegrees =
                sbufReadU16(src) as int32_t;
            (*boardAlignmentMutable()).pitchDegrees =
                sbufReadU16(src) as int32_t;
            (*boardAlignmentMutable()).yawDegrees =
                sbufReadU16(src) as int32_t
        }
        43 => {
            (*mixerConfigMutable()).mixerMode = sbufReadU8(src);
            if sbufBytesRemaining(src) >= 1 as libc::c_int {
                (*mixerConfigMutable()).yaw_motors_reversed =
                    sbufReadU8(src) != 0
            }
        }
        45 => {
            (*rxConfigMutable()).serialrx_provider = sbufReadU8(src);
            (*rxConfigMutable()).maxcheck = sbufReadU16(src);
            (*rxConfigMutable()).midrc = sbufReadU16(src);
            (*rxConfigMutable()).mincheck = sbufReadU16(src);
            (*rxConfigMutable()).spektrum_sat_bind = sbufReadU8(src);
            if sbufBytesRemaining(src) >= 4 as libc::c_int {
                (*rxConfigMutable()).rx_min_usec = sbufReadU16(src);
                (*rxConfigMutable()).rx_max_usec = sbufReadU16(src)
            }
            if sbufBytesRemaining(src) >= 4 as libc::c_int {
                (*rxConfigMutable()).rcInterpolation = sbufReadU8(src);
                (*rxConfigMutable()).rcInterpolationInterval =
                    sbufReadU8(src);
                (*rxConfigMutable()).airModeActivateThreshold =
                    ((sbufReadU16(src) as libc::c_int - 1000 as libc::c_int) /
                         10 as libc::c_int) as uint8_t
            }
            if sbufBytesRemaining(src) >= 6 as libc::c_int {
                sbufReadU8(src);
                sbufReadU32(src);
                sbufReadU8(src);
            }
            if sbufBytesRemaining(src) >= 1 as libc::c_int {
                (*rxConfigMutable()).fpvCamAngleDegrees = sbufReadU8(src)
            }
            if sbufBytesRemaining(src) >= 6 as libc::c_int {
                // Added in MSP API 1.40
                (*rxConfigMutable()).rcInterpolationChannels =
                    sbufReadU8(src);
                (*rxConfigMutable()).rc_smoothing_type = sbufReadU8(src);
                (*rxConfigMutable()).rc_smoothing_input_cutoff =
                    sbufReadU8(src);
                (*rxConfigMutable()).rc_smoothing_derivative_cutoff =
                    sbufReadU8(src);
                (*rxConfigMutable()).rc_smoothing_input_type =
                    sbufReadU8(src);
                (*rxConfigMutable()).rc_smoothing_derivative_type =
                    sbufReadU8(src);
                (*usbDevConfigMutable()).type_0 = sbufReadU8(src)
            }
        }
        76 => {
            (*failsafeConfigMutable()).failsafe_delay = sbufReadU8(src);
            (*failsafeConfigMutable()).failsafe_off_delay = sbufReadU8(src);
            (*failsafeConfigMutable()).failsafe_throttle = sbufReadU16(src);
            (*failsafeConfigMutable()).failsafe_switch_mode = sbufReadU8(src);
            (*failsafeConfigMutable()).failsafe_throttle_low_delay =
                sbufReadU16(src);
            (*failsafeConfigMutable()).failsafe_procedure = sbufReadU8(src)
        }
        78 => {
            i = sbufReadU8(src) as uint32_t;
            if i < 18 as libc::c_int as libc::c_uint {
                (*rxFailsafeChannelConfigsMutable(i as libc::c_int)).mode =
                    sbufReadU8(src);
                (*rxFailsafeChannelConfigsMutable(i as libc::c_int)).step =
                    ((constrain(sbufReadU16(src) as libc::c_int,
                                750 as libc::c_int, 2250 as libc::c_int) -
                          750 as libc::c_int) / 25 as libc::c_int) as uint8_t
            } else { return MSP_RESULT_ERROR }
        }
        51 => { (*rxConfigMutable()).rssi_channel = sbufReadU8(src) }
        65 => {
            let mut i_4: libc::c_int = 0 as libc::c_int;
            while i_4 < 8 as libc::c_int {
                (*rxConfigMutable()).rcmap[i_4 as usize] = sbufReadU8(src);
                i_4 += 1
            }
        }
        55 => {
            let mut portConfigSize: uint8_t =
                (::core::mem::size_of::<uint8_t>() as
                     libc::c_ulong).wrapping_add(::core::mem::size_of::<uint16_t>()
                                                     as
                                                     libc::c_ulong).wrapping_add((::core::mem::size_of::<uint8_t>()
                                                                                      as
                                                                                      libc::c_ulong).wrapping_mul(4
                                                                                                                      as
                                                                                                                      libc::c_int
                                                                                                                      as
                                                                                                                      libc::c_ulong))
                    as uint8_t;
            if dataSize.wrapping_rem(portConfigSize as libc::c_uint) !=
                   0 as libc::c_int as libc::c_uint {
                return MSP_RESULT_ERROR
            }
            let mut remainingPortsInPacket: uint8_t =
                dataSize.wrapping_div(portConfigSize as libc::c_uint) as
                    uint8_t;
            loop  {
                let fresh0 = remainingPortsInPacket;
                remainingPortsInPacket =
                    remainingPortsInPacket.wrapping_sub(1);
                if !(fresh0 != 0) { break ; }
                let mut identifier: uint8_t = sbufReadU8(src);
                let mut portConfig: *mut serialPortConfig_t =
                    serialFindPortConfiguration(identifier as
                                                    serialPortIdentifier_e);
                if portConfig.is_null() { return MSP_RESULT_ERROR }
                (*portConfig).identifier =
                    identifier as serialPortIdentifier_e;
                (*portConfig).functionMask = sbufReadU16(src);
                (*portConfig).msp_baudrateIndex = sbufReadU8(src);
                (*portConfig).gps_baudrateIndex = sbufReadU8(src);
                (*portConfig).telemetry_baudrateIndex = sbufReadU8(src);
                (*portConfig).blackbox_baudrateIndex = sbufReadU8(src)
            }
        }
        47 => {
            let mut i_5: libc::c_int = 0 as libc::c_int;
            while i_5 < 16 as libc::c_int {
                let mut color: *mut hsvColor_t =
                    &mut *(*(ledStripConfigMutable as
                                 unsafe extern "C" fn()
                                     ->
                                         *mut ledStripConfig_t)()).colors.as_mut_ptr().offset(i_5
                                                                                                  as
                                                                                                  isize)
                        as *mut hsvColor_t;
                (*color).h = sbufReadU16(src);
                (*color).s = sbufReadU8(src);
                (*color).v = sbufReadU8(src);
                i_5 += 1
            }
        }
        49 => {
            i = sbufReadU8(src) as uint32_t;
            if i >= 32 as libc::c_int as libc::c_uint ||
                   dataSize !=
                       (1 as libc::c_int + 4 as libc::c_int) as libc::c_uint {
                return MSP_RESULT_ERROR
            }
            let mut ledConfig: *mut ledConfig_t =
                &mut *(*(ledStripConfigMutable as
                             unsafe extern "C" fn()
                                 ->
                                     *mut ledStripConfig_t)()).ledConfigs.as_mut_ptr().offset(i
                                                                                                  as
                                                                                                  isize)
                    as *mut ledConfig_t;
            *ledConfig = sbufReadU32(src);
            reevaluateLedConfig();
        }
        221 => {
            let mut modeIdx: ledModeIndex_e =
                sbufReadU8(src) as ledModeIndex_e;
            let mut funIdx: libc::c_int = sbufReadU8(src) as libc::c_int;
            let mut color_0: libc::c_int = sbufReadU8(src) as libc::c_int;
            if !setModeColor(modeIdx, funIdx, color_0) {
                return MSP_RESULT_ERROR
            }
        }
        11 => {
            memset((*pilotConfigMutable()).name.as_mut_ptr() as
                       *mut libc::c_void, 0 as libc::c_int,
                   (::core::mem::size_of::<[libc::c_char; 17]>() as
                        libc::c_ulong).wrapping_div(::core::mem::size_of::<libc::c_char>()
                                                        as libc::c_ulong));
            let mut i_6: libc::c_uint = 0 as libc::c_int as libc::c_uint;
            while i_6 <
                      ({
                           let mut _a: libc::c_uint = 16 as libc::c_uint;
                           let _b: libc::c_uint = dataSize;
                           (if _a < _b { _a } else { _b })
                       }) {
                (*pilotConfigMutable()).name[i_6 as usize] =
                    sbufReadU8(src) as libc::c_char;
                i_6 = i_6.wrapping_add(1)
            }
        }
        246 => {
            let mut dt: dateTime_t =
                dateTime_t{year: 0,
                           month: 0,
                           day: 0,
                           hours: 0,
                           minutes: 0,
                           seconds: 0,
                           millis: 0,};
            dt.year = sbufReadU16(src);
            dt.month = sbufReadU8(src);
            dt.day = sbufReadU8(src);
            dt.hours = sbufReadU8(src);
            dt.minutes = sbufReadU8(src);
            dt.seconds = sbufReadU8(src);
            dt.millis = 0 as libc::c_int as uint16_t;
            rtcSetDateTime(&mut dt);
        }
        186 => { setRssiMsp(sbufReadU8(src)); }
        248 => {
            if !boardInformationIsSet() {
                let mut length: uint8_t = sbufReadU8(src);
                let mut boardName: [libc::c_char; 21] = [0; 21];
                sbufReadData(src, boardName.as_mut_ptr() as *mut libc::c_void,
                             ({
                                  let mut _a: uint8_t = length;
                                  let mut _b: libc::c_int = 20 as libc::c_int;
                                  if (_a as libc::c_int) < _b {
                                      _a as libc::c_int
                                  } else { _b }
                              }));
                if length as libc::c_int > 20 as libc::c_int {
                    sbufAdvance(src,
                                length as libc::c_int - 20 as libc::c_int);
                }
                boardName[length as usize] = '\u{0}' as i32 as libc::c_char;
                length = sbufReadU8(src);
                let mut manufacturerId: [libc::c_char; 5] = [0; 5];
                sbufReadData(src,
                             manufacturerId.as_mut_ptr() as *mut libc::c_void,
                             ({
                                  let mut _a: uint8_t = length;
                                  let mut _b: libc::c_int = 4 as libc::c_int;
                                  if (_a as libc::c_int) < _b {
                                      _a as libc::c_int
                                  } else { _b }
                              }));
                if length as libc::c_int > 4 as libc::c_int {
                    sbufAdvance(src,
                                length as libc::c_int - 4 as libc::c_int);
                }
                manufacturerId[length as usize] =
                    '\u{0}' as i32 as libc::c_char;
                setBoardName(boardName.as_mut_ptr());
                setManufacturerId(manufacturerId.as_mut_ptr());
                persistBoardInformation();
            } else { return MSP_RESULT_ERROR }
        }
        249 => {
            if !signatureIsSet() {
                let mut signature: [uint8_t; 32] = [0; 32];
                sbufReadData(src, signature.as_mut_ptr() as *mut libc::c_void,
                             32 as libc::c_int);
                setSignature(signature.as_mut_ptr());
                persistSignature();
            } else { return MSP_RESULT_ERROR }
        }
        _ => {
            // USE_BOARD_INFO
            // we do not know how to handle the (valid) message, indicate error MSP $M!
            return MSP_RESULT_ERROR
        }
    }
    return MSP_RESULT_ACK;
}
// USE_OSD_SLAVE
unsafe extern "C" fn mspCommonProcessInCommand(mut cmdMSP: uint8_t,
                                               mut src: *mut sbuf_t,
                                               mut mspPostProcessFn:
                                                   *mut mspPostProcessFnPtr)
 -> mspResult_e {
    let dataSize: libc::c_uint = sbufBytesRemaining(src) as libc::c_uint;
    // maybe unused due to compiler options
    match cmdMSP as libc::c_int {
        57 => {
            let mut id: int8_t = sbufReadU8(src) as int8_t;
            //
        // find and configure an ADC voltage sensor
        //
            let mut voltageSensorADCIndex: int8_t = 0;
            voltageSensorADCIndex = 0 as libc::c_int as int8_t;
            while (voltageSensorADCIndex as libc::c_int) < 1 as libc::c_int {
                if id as libc::c_int ==
                       voltageMeterADCtoIDMap[voltageSensorADCIndex as usize]
                           as libc::c_int {
                    break ;
                }
                voltageSensorADCIndex += 1
            }
            if (voltageSensorADCIndex as libc::c_int) < 1 as libc::c_int {
                (*voltageSensorADCConfigMutable(voltageSensorADCIndex as
                                                    libc::c_int)).vbatscale =
                    sbufReadU8(src);
                (*voltageSensorADCConfigMutable(voltageSensorADCIndex as
                                                    libc::c_int)).vbatresdivval
                    = sbufReadU8(src);
                (*voltageSensorADCConfigMutable(voltageSensorADCIndex as
                                                    libc::c_int)).vbatresdivmultiplier
                    = sbufReadU8(src)
            } else {
                // if we had any other types of voltage sensor to configure, this is where we'd do it.
                sbufReadU8(src); // vbatlevel_warn1 in MWC2.3 GUI
                sbufReadU8(src); // vbatlevel_warn2 in MWC2.3 GUI
                sbufReadU8(src); // vbatlevel when buzzer starts to alert
            }
        }
        41 => {
            let mut id_0: libc::c_int = sbufReadU8(src) as libc::c_int;
            match id_0 {
                10 => {
                    (*currentSensorADCConfigMutable()).scale =
                        sbufReadU16(src) as int16_t;
                    (*currentSensorADCConfigMutable()).offset =
                        sbufReadU16(src) as int16_t
                }
                80 => {
                    (*currentSensorVirtualConfigMutable()).scale =
                        sbufReadU16(src) as int16_t;
                    (*currentSensorVirtualConfigMutable()).offset =
                        sbufReadU16(src)
                }
                _ => { sbufReadU16(src); sbufReadU16(src); }
            }
        }
        33 => {
            (*batteryConfigMutable()).vbatmincellvoltage = sbufReadU8(src);
            (*batteryConfigMutable()).vbatmaxcellvoltage = sbufReadU8(src);
            (*batteryConfigMutable()).vbatwarningcellvoltage =
                sbufReadU8(src);
            (*batteryConfigMutable()).batteryCapacity = sbufReadU16(src);
            (*batteryConfigMutable()).voltageMeterSource =
                sbufReadU8(src) as voltageMeterSource_e;
            (*batteryConfigMutable()).currentMeterSource =
                sbufReadU8(src) as currentMeterSource_e
        }
        85 => {
            let addr: uint8_t = sbufReadU8(src);
            if addr as int8_t as libc::c_int == -(1 as libc::c_int) {
                /* Set general OSD settings */
                sbufReadU8(src); // Skip video system
                (*osdConfigMutable()).units = sbufReadU8(src) as osd_unit_e;
                // Alarms
                (*osdConfigMutable()).rssi_alarm =
                    sbufReadU8(src); // Skip unused (previously fly timer)
                (*osdConfigMutable()).cap_alarm = sbufReadU16(src);
                sbufReadU16(src);
                (*osdConfigMutable()).alt_alarm = sbufReadU16(src);
                if sbufBytesRemaining(src) >= 2 as libc::c_int {
                    /* Enabled warnings */
                    (*osdConfigMutable()).enabledWarnings = sbufReadU16(src)
                }
            } else if addr as int8_t as libc::c_int == -(2 as libc::c_int) {
                // Timers
                let mut index: uint8_t = sbufReadU8(src);
                if index as libc::c_int > OSD_TIMER_COUNT as libc::c_int {
                    return MSP_RESULT_ERROR
                }
                (*osdConfigMutable()).timers[index as usize] =
                    sbufReadU16(src);
                return MSP_RESULT_ERROR
            } else {
                let value: uint16_t = sbufReadU16(src);
                /* Get screen index, 0 is post flight statistics, 1 and above are in flight OSD screens */
                let screen: uint8_t =
                    if sbufBytesRemaining(src) >= 1 as libc::c_int {
                        sbufReadU8(src) as libc::c_int
                    } else { 1 as libc::c_int } as uint8_t;
                if screen as libc::c_int == 0 as libc::c_int &&
                       (addr as libc::c_int) < OSD_STAT_COUNT as libc::c_int {
                    /* Set statistic item enable */
                    osdStatSetState(addr,
                                    value as libc::c_int != 0 as libc::c_int);
                } else if (addr as libc::c_int) <
                              OSD_ITEM_COUNT as libc::c_int {
                    /* Set element positions */
                    (*osdConfigMutable()).item_pos[addr as usize] = value
                } else { return MSP_RESULT_ERROR }
            }
        }
        87 => { return MSP_RESULT_ERROR }
        _ => {
            // OSD || USE_OSD_SLAVE
            return mspProcessInCommand(cmdMSP, src)
        }
    }
    return MSP_RESULT_ACK;
}
/*
 * Returns MSP_RESULT_ACK, MSP_RESULT_ERROR or MSP_RESULT_NO_REPLY
 */
#[no_mangle]
pub unsafe extern "C" fn mspFcProcessCommand(mut cmd: *mut mspPacket_t,
                                             mut reply: *mut mspPacket_t,
                                             mut mspPostProcessFn:
                                                 *mut mspPostProcessFnPtr)
 -> mspResult_e {
    let mut ret: libc::c_int = MSP_RESULT_ACK as libc::c_int;
    let mut dst: *mut sbuf_t = &mut (*reply).buf;
    let mut src: *mut sbuf_t = &mut (*cmd).buf;
    let cmdMSP: uint8_t = (*cmd).cmd as uint8_t;
    // initialize reply by default
    (*reply).cmd = (*cmd).cmd;
    if mspCommonProcessOutCommand(cmdMSP, dst, mspPostProcessFn) {
        ret = MSP_RESULT_ACK as libc::c_int
    } else if mspProcessOutCommand(cmdMSP, dst) {
        ret = MSP_RESULT_ACK as libc::c_int
    } else {
        ret =
            mspFcProcessOutCommandWithArg(cmdMSP, src, dst, mspPostProcessFn)
                as libc::c_int;
        if !(ret != MSP_RESULT_CMD_UNKNOWN as libc::c_int) {
            if cmdMSP as libc::c_int == 245 as libc::c_int {
                mspFc4waySerialCommand(dst, src, mspPostProcessFn);
                ret = MSP_RESULT_ACK as libc::c_int
            } else {
                ret =
                    mspCommonProcessInCommand(cmdMSP, src, mspPostProcessFn)
                        as libc::c_int
            }
        }
    }
    (*reply).result = ret as int16_t;
    return ret as mspResult_e;
}
#[no_mangle]
pub unsafe extern "C" fn mspFcProcessReply(mut reply: *mut mspPacket_t) {
    let mut src: *mut sbuf_t = &mut (*reply).buf;
    // potentially unused depending on compile options.
    match (*reply).cmd as libc::c_int {
        110 => {
            let mut batteryVoltage: uint8_t = sbufReadU8(src);
            let mut mAhDrawn: uint16_t = sbufReadU16(src);
            let mut rssi: uint16_t = sbufReadU16(src);
            let mut amperage: uint16_t = sbufReadU16(src);
        }
        _ => { }
    };
}
#[no_mangle]
pub unsafe extern "C" fn mspInit() { initActiveBoxIds(); }
