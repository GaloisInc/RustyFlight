use core;
use libc;
extern "C" {
    #[no_mangle]
    fn atoi(__nptr: *const libc::c_char) -> libc::c_int;
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memcmp(_: *const libc::c_void, _: *const libc::c_void,
              _: libc::c_ulong) -> libc::c_int;
    #[no_mangle]
    fn strncpy(_: *mut libc::c_char, _: *const libc::c_char, _: libc::c_ulong)
     -> *mut libc::c_char;
    #[no_mangle]
    fn strcmp(_: *const libc::c_char, _: *const libc::c_char) -> libc::c_int;
    #[no_mangle]
    fn strncmp(_: *const libc::c_char, _: *const libc::c_char,
               _: libc::c_ulong) -> libc::c_int;
    #[no_mangle]
    fn strchr(_: *const libc::c_char, _: libc::c_int) -> *mut libc::c_char;
    #[no_mangle]
    fn strstr(_: *const libc::c_char, _: *const libc::c_char)
     -> *mut libc::c_char;
    #[no_mangle]
    fn strcasecmp(_: *const libc::c_char, _: *const libc::c_char)
     -> libc::c_int;
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
    #[no_mangle]
    fn ffs(__i: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn strtok_r(__s: *mut libc::c_char, __delim: *const libc::c_char,
                __save_ptr: *mut *mut libc::c_char) -> *mut libc::c_char;
    #[no_mangle]
    fn strcasestr(__haystack: *const libc::c_char,
                  __needle: *const libc::c_char) -> *mut libc::c_char;
    #[no_mangle]
    fn strncasecmp(_: *const libc::c_char, _: *const libc::c_char,
                   _: libc::c_ulong) -> libc::c_int;
    #[no_mangle]
    fn isspace(_: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn toupper(_: libc::c_int) -> libc::c_int;
    /* *
  ******************************************************************************
  * @file    system_stm32f30x.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    28-March-2014
  * @brief   CMSIS Cortex-M4 Device System Source File for STM32F30x devices.
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
    /* * @addtogroup CMSIS
  * @{
  */
    /* * @addtogroup stm32f30x_system
  * @{
  */
    /* *
  * @brief Define to prevent recursive inclusion
  */
    /* Exported types ------------------------------------------------------------*/
    #[no_mangle]
    static mut SystemCoreClock: uint32_t;
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
    // increment when a major release is made (big new feature, etc)
    // increment when a minor release is made (small new feature, change etc)
    // increment when a bug is fixed
    #[no_mangle]
    static targetName: *const libc::c_char;
    // lower case hexadecimal digits.
    #[no_mangle]
    static shortGitRevision: *const libc::c_char;
    #[no_mangle]
    static buildDate: *const libc::c_char;
    // "MMM DD YYYY" MMM = Jan/Feb/...
    #[no_mangle]
    static buildTime: *const libc::c_char;
    #[no_mangle]
    static mut __config_start: uint8_t;
    // configured via linker script when building binaries.
    #[no_mangle]
    static mut __config_end: uint8_t;
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
    // parameter group registry flags
    // documentary
    // function that resets a single parameter group instance
    /* base */
    /* size */
    // The parameter group number, the top 4 bits are reserved for version
    // Size of the group in RAM, the top 4 bits are reserved for flags
    // Address of the group in RAM.
    // Address of the copy in RAM.
    // The pointer to update after loading the record into ram.
    // Pointer to init template
    // Pointer to pgResetFunc
    #[no_mangle]
    static __pg_registry_end: [pgRegistry_t; 0];
    // Helper to iterate over the PG register.  Cheaper than a visitor style callback.
    // Reset configuration to default (by name)
    /* */
    // Declare system config
    /* */
    // Declare system config array
    /* */
    // Register system config
    /* Force external linkage for g++. Catch multi registration */
    /* */
    /* */
    /* */
    /* */
    // Register system config array
    /* */
    /* */
    /* */
    // Emit reset defaults for config.
// Config must be registered with PG_REGISTER_<xxx>_WITH_RESET_TEMPLATE macro
    /* */
    #[no_mangle]
    fn pgFind(pgn: pgn_t) -> *const pgRegistry_t;
    #[no_mangle]
    static __pg_registry_start: [pgRegistry_t; 0];
    #[no_mangle]
    fn tfp_format(putp: *mut libc::c_void,
                  putf:
                      Option<unsafe extern "C" fn(_: *mut libc::c_void,
                                                  _: libc::c_char) -> ()>,
                  fmt: *const libc::c_char, va: ::core::ffi::VaList)
     -> libc::c_int;
    #[no_mangle]
    fn setPrintfSerialPort(serialPort: *mut serialPort_s);
    #[no_mangle]
    fn ftoa(x: libc::c_float, floatString: *mut libc::c_char)
     -> *mut libc::c_char;
    #[no_mangle]
    fn fastA2F(p: *const libc::c_char) -> libc::c_float;
    #[no_mangle]
    fn itoa(i: libc::c_int, a: *mut libc::c_char, r: libc::c_int)
     -> *mut libc::c_char;
    #[no_mangle]
    fn getEEPROMConfigSize() -> uint16_t;
    #[no_mangle]
    static mut featureConfig_System: featureConfig_t;
    #[no_mangle]
    static mut featureConfig_Copy: featureConfig_t;
    #[no_mangle]
    fn featureSet(mask: uint32_t);
    #[no_mangle]
    fn featureClear(mask: uint32_t);
    #[no_mangle]
    fn featureMask() -> uint32_t;
    #[no_mangle]
    fn i2cGetErrorCounter() -> uint16_t;
    #[no_mangle]
    fn gyroReadRegister(whichSensor: uint8_t, reg: uint8_t) -> uint8_t;
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn millis() -> timeMs_t;
    // Initialise a block of memory as a buffered writer.
//
// b should be sizeof(bufWriter_t) + the number of bytes to buffer.
// total_size should be the total size of b.
//
    #[no_mangle]
    fn bufWriterInit(b: *mut uint8_t, total_size: libc::c_int,
                     writer: bufWrite_t, p: *mut libc::c_void)
     -> *mut bufWriter_t;
    #[no_mangle]
    fn bufWriterAppend(b: *mut bufWriter_t, ch: uint8_t);
    #[no_mangle]
    fn bufWriterFlush(b: *mut bufWriter_t);
    #[no_mangle]
    fn dmaGetOwner(identifier: dmaIdentifier_e) -> resourceOwner_e;
    #[no_mangle]
    fn dmaGetResourceIndex(identifier: dmaIdentifier_e) -> uint8_t;
    #[no_mangle]
    static ownerNames: [*const libc::c_char; 55];
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
    // preprocessor is used to convert pinid to requested C data value
// compile-time error is generated if requested pin is not available (not set in TARGET_IO_PORTx)
// ioTag_t and IO_t is supported, but ioTag_t is preferred
    // expand pinid to to ioTag_t
    // TODO
    // declare available IO pins. Available pins are specified per target
    // unimplemented
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn serialRxBytesWaiting(instance: *const serialPort_t) -> uint32_t;
    #[no_mangle]
    fn serialRead(instance: *mut serialPort_t) -> uint8_t;
    #[no_mangle]
    fn serialSetBaudRate(instance: *mut serialPort_t, baudRate: uint32_t);
    #[no_mangle]
    fn serialSetMode(instance: *mut serialPort_t, mode: portMode_e);
    #[no_mangle]
    fn serialSetCtrlLineStateCb(instance: *mut serialPort_t,
                                cb:
                                    Option<unsafe extern "C" fn(_:
                                                                    *mut libc::c_void,
                                                                _: uint16_t)
                                               -> ()>,
                                context: *mut libc::c_void);
    #[no_mangle]
    fn serialSetBaudRateCb(instance: *mut serialPort_t,
                           cb:
                               Option<unsafe extern "C" fn(_:
                                                               *mut serialPort_t,
                                                           _: uint32_t)
                                          -> ()>, context: *mut serialPort_t);
    // A shim that adapts the bufWriter API to the serialWriteBuf() API.
    #[no_mangle]
    fn serialWriteBufShim(instance: *mut libc::c_void, data: *const uint8_t,
                          count: libc::c_int);
    #[no_mangle]
    static mut ioRecs: [ioRec_t; 0];
    #[no_mangle]
    fn IO_GPIOPortIdx(io: IO_t) -> libc::c_int;
    #[no_mangle]
    fn IO_GPIOPinIdx(io: IO_t) -> libc::c_int;
    #[no_mangle]
    fn IO_Rec(io: IO_t) -> *mut ioRec_t;
    #[no_mangle]
    fn getBatteryStateString() -> *const libc::c_char;
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
    #[no_mangle]
    fn getBatteryCellCount() -> uint8_t;
    // serialPort API
    #[no_mangle]
    fn escEnablePassthrough(escPassthroughPort: *mut serialPort_t,
                            output: uint16_t, mode: uint8_t);
    #[no_mangle]
    fn stackTotalSize() -> uint32_t;
    #[no_mangle]
    fn stackHighMem() -> uint32_t;
    // bootloader/IAP
    #[no_mangle]
    fn systemReset();
    #[no_mangle]
    fn systemResetToBootloader();
    #[no_mangle]
    static mut pilotConfig_System: pilotConfig_t;
    #[no_mangle]
    static mut pilotConfig_Copy: pilotConfig_t;
    #[no_mangle]
    static mut systemConfig_System: systemConfig_t;
    #[no_mangle]
    static mut systemConfig_Copy: systemConfig_t;
    #[no_mangle]
    fn writeEEPROM();
    #[no_mangle]
    fn getCurrentPidProfileIndex() -> uint8_t;
    #[no_mangle]
    fn changePidProfile(pidProfileIndex: uint8_t);
    #[no_mangle]
    fn getCurrentControlRateProfileIndex() -> uint8_t;
    #[no_mangle]
    fn changeControlRateProfile(profileIndex: uint8_t);
    #[no_mangle]
    fn resetConfigs();
    #[no_mangle]
    static mut currentRxRefreshRate: uint16_t;
    #[no_mangle]
    fn rcSmoothingGetValue(whichValue: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn rcSmoothingAutoCalculate() -> bool;
    #[no_mangle]
    static mut adjustmentRanges_SystemArray: [adjustmentRange_t; 15];
    #[no_mangle]
    static mut adjustmentRanges_CopyArray: [adjustmentRange_t; 15];
    #[no_mangle]
    static mut modeActivationConditions_SystemArray:
           [modeActivationCondition_t; 20];
    #[no_mangle]
    static mut modeActivationConditions_CopyArray:
           [modeActivationCondition_t; 20];
    #[no_mangle]
    static mut armingDisableFlagNames: [*const libc::c_char; 20];
    #[no_mangle]
    fn setArmingDisabled(flag: armingDisableFlags_e);
    #[no_mangle]
    fn getArmingDisableFlags() -> armingDisableFlags_e;
    #[no_mangle]
    static mut customMotorMixer_SystemArray: [motorMixer_t; 8];
    #[no_mangle]
    static mut customMotorMixer_CopyArray: [motorMixer_t; 8];
    #[no_mangle]
    static mut mixerConfig_Copy: mixerConfig_t;
    #[no_mangle]
    fn convertExternalToMotor(externalValue: uint16_t) -> libc::c_float;
    #[no_mangle]
    fn stopPwmAllMotors();
    #[no_mangle]
    fn mixerResetDisarmedMotors();
    #[no_mangle]
    fn mixerLoadMix(index: libc::c_int, customMixers: *mut motorMixer_t);
    #[no_mangle]
    static mut mixerConfig_System: mixerConfig_t;
    #[no_mangle]
    static mut motorConfig_System: motorConfig_t;
    #[no_mangle]
    static mut motor_disarmed: [libc::c_float; 8];
    #[no_mangle]
    fn getMotorCount() -> uint8_t;
    #[no_mangle]
    fn pwmWriteDshotCommand(index: uint8_t, motorCount: uint8_t,
                            command: uint8_t, blocking: bool);
    #[no_mangle]
    fn pwmDisableMotors();
    #[no_mangle]
    fn pwmEnableMotors();
    #[no_mangle]
    static mut pidConfig_System: pidConfig_t;
    #[no_mangle]
    static mut customServoMixers_SystemArray: [servoMixer_t; 16];
    #[no_mangle]
    static mut customServoMixers_CopyArray: [servoMixer_t; 16];
    #[no_mangle]
    static mut servoParams_CopyArray: [servoParam_t; 8];
    #[no_mangle]
    static mut servoParams_SystemArray: [servoParam_t; 8];
    #[no_mangle]
    fn servoMixerLoadMix(index: libc::c_int);
    #[no_mangle]
    fn findBoxByBoxId(boxId: boxId_e) -> *const box_t;
    #[no_mangle]
    fn findBoxByPermanentId(permanentId: uint8_t) -> *const box_t;
    #[no_mangle]
    static lookupTables: [lookupTableEntry_t; 0];
    #[no_mangle]
    static valueTableEntryCount: uint16_t;
    #[no_mangle]
    static valueTable: [clivalue_t; 0];
    #[no_mangle]
    fn beeper(mode: beeperMode_e);
    #[no_mangle]
    fn beeperModeMaskForTableIndex(idx: libc::c_int) -> uint32_t;
    #[no_mangle]
    fn beeperNameForTableIndex(idx: libc::c_int) -> *const libc::c_char;
    #[no_mangle]
    fn beeperTableEntryCount() -> libc::c_int;
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
    // Automatically trigger a flush when this much data is in the buffer
    #[no_mangle]
    fn flashfsEraseCompletely();
    #[no_mangle]
    fn flashfsGetOffset() -> uint32_t;
    #[no_mangle]
    fn flashfsGetGeometry() -> *const flashGeometry_s;
    #[no_mangle]
    fn flashfsIsSupported() -> bool;
    #[no_mangle]
    fn flashfsIsReady() -> bool;
    #[no_mangle]
    static mut ledStripConfig_Copy: ledStripConfig_t;
    #[no_mangle]
    static mut ledStripConfig_System: ledStripConfig_t;
    #[no_mangle]
    fn parseColor(index: libc::c_int, colorConfig: *const libc::c_char)
     -> bool;
    #[no_mangle]
    fn parseLedStripConfig(ledIndex: libc::c_int, config: *const libc::c_char)
     -> bool;
    #[no_mangle]
    fn generateLedConfig(ledConfig: *mut ledConfig_t,
                         ledConfigBuffer: *mut libc::c_char,
                         bufferSize: size_t);
    #[no_mangle]
    fn setModeColor(modeIndex: ledModeIndex_e, modeColorIndex: libc::c_int,
                    colorIndex: libc::c_int) -> bool;
    #[no_mangle]
    static baudRates: [uint32_t; 0];
    #[no_mangle]
    static mut serialConfig_Copy: serialConfig_t;
    #[no_mangle]
    static mut serialConfig_System: serialConfig_t;
    #[no_mangle]
    fn serialIsPortAvailable(identifier: serialPortIdentifier_e) -> bool;
    #[no_mangle]
    fn serialFindPortConfiguration(identifier: serialPortIdentifier_e)
     -> *mut serialPortConfig_t;
    // !!TODO remove need for this
    #[no_mangle]
    fn findSerialPortUsageByIdentifier(identifier: serialPortIdentifier_e)
     -> *mut serialPortUsage_t;
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
    fn waitForSerialPortToFinishTransmitting(serialPort: *mut serialPort_t);
    #[no_mangle]
    fn lookupBaudRateIndex(baudRate: uint32_t) -> baudRate_e;
    //
// msp/cli/bootloader
//
    #[no_mangle]
    fn serialPassthrough(left: *mut serialPort_t, right: *mut serialPort_t,
                         leftC:
                             Option<unsafe extern "C" fn(_: uint8_t) -> ()>,
                         rightC:
                             Option<unsafe extern "C" fn(_: uint8_t) -> ()>);
    #[no_mangle]
    static mut vtxConfig_Copy: vtxConfig_t;
    #[no_mangle]
    static mut vtxConfig_System: vtxConfig_t;
    #[no_mangle]
    static mut beeperConfig_Copy: beeperConfig_t;
    #[no_mangle]
    static mut beeperConfig_System: beeperConfig_t;
    #[no_mangle]
    fn pinioSet(index: libc::c_int, on: bool);
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    static mut rxConfig_Copy: rxConfig_t;
    #[no_mangle]
    static rcChannelLetters: [libc::c_char; 0];
    #[no_mangle]
    static mut rxFailsafeChannelConfigs_CopyArray:
           [rxFailsafeChannelConfig_t; 18];
    #[no_mangle]
    static mut rxFailsafeChannelConfigs_SystemArray:
           [rxFailsafeChannelConfig_t; 18];
    #[no_mangle]
    static mut rxChannelRangeConfigs_SystemArray: [rxChannelRangeConfig_t; 4];
    #[no_mangle]
    static mut rxChannelRangeConfigs_CopyArray: [rxChannelRangeConfig_t; 4];
    #[no_mangle]
    fn parseRcChannels(input: *const libc::c_char,
                       rxConfig_0: *mut rxConfig_s);
    #[no_mangle]
    fn resetAllRxChannelRangeConfigurations(rxChannelRangeConfig:
                                                *mut rxChannelRangeConfig_t);
    #[no_mangle]
    static mut averageSystemLoadPercent: uint16_t;
    #[no_mangle]
    fn getCheckFuncInfo(checkFuncInfo: *mut cfCheckFuncInfo_t);
    #[no_mangle]
    fn getTaskInfo(taskId: cfTaskId_e, taskInfo: *mut cfTaskInfo_t);
    #[no_mangle]
    fn getTaskDeltaTime(taskId: cfTaskId_e) -> timeDelta_t;
    #[no_mangle]
    fn schedulerSetCalulateTaskStatistics(calculateTaskStatistics: bool);
    #[no_mangle]
    fn schedulerResetTaskMaxExecutionTime(taskId: cfTaskId_e);
}
pub type __builtin_va_list = [__va_list_tag; 1];
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct __va_list_tag {
    pub gp_offset: libc::c_uint,
    pub fp_offset: libc::c_uint,
    pub overflow_arg_area: *mut libc::c_void,
    pub reg_save_area: *mut libc::c_void,
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
pub type intptr_t = libc::c_long;
pub type size_t = libc::c_ulong;
pub type va_list = __builtin_va_list;
/* * 
  * @brief General Purpose I/O
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub MODER: uint32_t,
    pub OTYPER: uint16_t,
    pub RESERVED0: uint16_t,
    pub OSPEEDR: uint32_t,
    pub PUPDR: uint32_t,
    pub IDR: uint16_t,
    pub RESERVED1: uint16_t,
    pub ODR: uint16_t,
    pub RESERVED2: uint16_t,
    pub BSRR: uint32_t,
    pub LCKR: uint32_t,
    pub AFR: [uint32_t; 2],
    pub BRR: uint16_t,
    pub RESERVED3: uint16_t,
    /* !< Reserved,                                                                 0x2A */
}
pub type pgn_t = uint16_t;
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<unsafe extern "C" fn(_: *mut libc::c_void,
                                          _: libc::c_int) -> ()>,
}
pub type pgRegistry_t = pgRegistry_s;
pub type timeDelta_t = int32_t;
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
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
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
pub struct displayPortVTable_s {
    pub grab: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                         -> libc::c_int>,
    pub release: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                            -> libc::c_int>,
    pub clearScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                                -> libc::c_int>,
    pub drawScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                               -> libc::c_int>,
    pub screenSize: Option<unsafe extern "C" fn(_: *const displayPort_t)
                               -> libc::c_int>,
    pub writeString: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                                 _: uint8_t, _: uint8_t,
                                                 _: *const libc::c_char)
                                -> libc::c_int>,
    pub writeChar: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                               _: uint8_t, _: uint8_t,
                                               _: uint8_t) -> libc::c_int>,
    pub isTransferInProgress: Option<unsafe extern "C" fn(_:
                                                              *const displayPort_t)
                                         -> bool>,
    pub heartbeat: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                              -> libc::c_int>,
    pub resync: Option<unsafe extern "C" fn(_: *mut displayPort_t) -> ()>,
    pub isSynced: Option<unsafe extern "C" fn(_: *const displayPort_t)
                             -> bool>,
    pub txBytesFree: Option<unsafe extern "C" fn(_: *const displayPort_t)
                                -> uint32_t>,
}
pub type displayPort_t = displayPort_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct displayPort_s {
    pub vTable: *const displayPortVTable_s,
    pub device: *mut libc::c_void,
    pub rows: uint8_t,
    pub cols: uint8_t,
    pub posX: uint8_t,
    pub posY: uint8_t,
    pub cleared: bool,
    pub cursorRow: int8_t,
    pub grabCount: int8_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct hsvColor_s {
    pub h: uint16_t,
    pub s: uint8_t,
    pub v: uint8_t,
}
pub type hsvColor_t = hsvColor_s;
#[derive ( Copy, Clone )]
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
#[derive ( Copy, Clone )]
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct featureConfig_s {
    pub enabledFeatures: uint32_t,
}
pub type featureConfig_t = featureConfig_s;
pub type ioTag_t = uint8_t;
pub type IO_t = *mut libc::c_void;
// 0 - 359
// 0 - 255
// 0 - 255
// Macros to convert between CLI bus number and I2CDevice.
// I2C device address range in 7-bit address mode
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct i2cConfig_s {
    pub ioTagScl: ioTag_t,
    pub ioTagSda: ioTag_t,
    pub overClock: bool,
    pub pullUp: bool,
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
// Called to flush the buffer.
pub type bufWrite_t
    =
    Option<unsafe extern "C" fn(_: *mut libc::c_void, _: *mut libc::c_void,
                                _: libc::c_int) -> ()>;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct bufWriter_s {
    pub writer: bufWrite_t,
    pub arg: *mut libc::c_void,
    pub capacity: uint8_t,
    pub at: uint8_t,
    pub data: [uint8_t; 0],
}
pub type bufWriter_t = bufWriter_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct spiPinConfig_s {
    pub ioTagSck: ioTag_t,
    pub ioTagMiso: ioTag_t,
    pub ioTagMosi: ioTag_t,
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
pub type dmaIdentifier_e = libc::c_uint;
pub const DMA_LAST_HANDLER: dmaIdentifier_e = 12;
pub const DMA2_CH5_HANDLER: dmaIdentifier_e = 12;
pub const DMA2_CH4_HANDLER: dmaIdentifier_e = 11;
pub const DMA2_CH3_HANDLER: dmaIdentifier_e = 10;
pub const DMA2_CH2_HANDLER: dmaIdentifier_e = 9;
pub const DMA2_CH1_HANDLER: dmaIdentifier_e = 8;
pub const DMA1_CH7_HANDLER: dmaIdentifier_e = 7;
pub const DMA1_CH6_HANDLER: dmaIdentifier_e = 6;
pub const DMA1_CH5_HANDLER: dmaIdentifier_e = 5;
pub const DMA1_CH4_HANDLER: dmaIdentifier_e = 4;
pub const DMA1_CH3_HANDLER: dmaIdentifier_e = 3;
pub const DMA1_CH2_HANDLER: dmaIdentifier_e = 2;
pub const DMA1_CH1_HANDLER: dmaIdentifier_e = 1;
pub const DMA_NONE: dmaIdentifier_e = 0;
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct ioRec_s {
    pub gpio: *mut GPIO_TypeDef,
    pub pin: uint16_t,
    pub owner: resourceOwner_e,
    pub index: uint8_t,
}
pub type ioRec_t = ioRec_s;
#[derive ( Copy, Clone )]
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
pub type C2RustUnnamed_1 = libc::c_uint;
pub const PROTOCOL_COUNT: C2RustUnnamed_1 = 5;
pub const PROTOCOL_CASTLE: C2RustUnnamed_1 = 4;
pub const PROTOCOL_KISSALL: C2RustUnnamed_1 = 3;
pub const PROTOCOL_KISS: C2RustUnnamed_1 = 2;
pub const PROTOCOL_BLHELI: C2RustUnnamed_1 = 1;
pub const PROTOCOL_SIMONK: C2RustUnnamed_1 = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pilotConfig_s {
    pub name: [libc::c_char; 17],
}
// Count of the number of erasable blocks on the device
// In bytes
// This is just pagesPerSector * pageSize
// This is just sectorSize * sectors
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
#[derive ( Copy, Clone )]
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
#[derive ( Copy, Clone )]
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pidf_s {
    pub P: uint8_t,
    pub I: uint8_t,
    pub D: uint8_t,
    pub F: uint16_t,
}
#[derive ( Copy, Clone )]
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct channelRange_s {
    pub startStep: uint8_t,
    pub endStep: uint8_t,
}
pub type channelRange_t = channelRange_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct modeActivationCondition_s {
    pub modeId: boxId_e,
    pub auxChannelIndex: uint8_t,
    pub range: channelRange_t,
    pub modeLogic: modeLogic_e,
    pub linkedTo: boxId_e,
}
// in seconds
// Breakpoint where TPA is activated
// Sets the throttle limiting type - off, scale or clip
// Sets the maximum pilot commanded throttle limit
// type to hold enough bits for CHECKBOX_ITEM_COUNT. Struct used for value-like behavior
// steps are 25 apart
// a value of 0 corresponds to a channel value of 900 or less
// a value of 48 corresponds to a channel value of 2100 or more
// 48 steps between 900 and 2100
pub type modeActivationCondition_t = modeActivationCondition_s;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const ADJUSTMENT_FUNCTION_COUNT: C2RustUnnamed_2 = 33;
pub const ADJUSTMENT_YAW_F: C2RustUnnamed_2 = 32;
pub const ADJUSTMENT_ROLL_F: C2RustUnnamed_2 = 31;
pub const ADJUSTMENT_PITCH_F: C2RustUnnamed_2 = 30;
pub const ADJUSTMENT_PID_AUDIO: C2RustUnnamed_2 = 29;
pub const ADJUSTMENT_PITCH_RC_EXPO: C2RustUnnamed_2 = 28;
pub const ADJUSTMENT_ROLL_RC_EXPO: C2RustUnnamed_2 = 27;
pub const ADJUSTMENT_PITCH_RC_RATE: C2RustUnnamed_2 = 26;
pub const ADJUSTMENT_ROLL_RC_RATE: C2RustUnnamed_2 = 25;
pub const ADJUSTMENT_HORIZON_STRENGTH: C2RustUnnamed_2 = 24;
pub const ADJUSTMENT_FEEDFORWARD_TRANSITION: C2RustUnnamed_2 = 23;
pub const ADJUSTMENT_PITCH_ROLL_F: C2RustUnnamed_2 = 22;
pub const ADJUSTMENT_RC_RATE_YAW: C2RustUnnamed_2 = 21;
pub const ADJUSTMENT_ROLL_D: C2RustUnnamed_2 = 20;
pub const ADJUSTMENT_ROLL_I: C2RustUnnamed_2 = 19;
pub const ADJUSTMENT_ROLL_P: C2RustUnnamed_2 = 18;
pub const ADJUSTMENT_PITCH_D: C2RustUnnamed_2 = 17;
pub const ADJUSTMENT_PITCH_I: C2RustUnnamed_2 = 16;
pub const ADJUSTMENT_PITCH_P: C2RustUnnamed_2 = 15;
pub const ADJUSTMENT_ROLL_RATE: C2RustUnnamed_2 = 14;
pub const ADJUSTMENT_PITCH_RATE: C2RustUnnamed_2 = 13;
pub const ADJUSTMENT_RATE_PROFILE: C2RustUnnamed_2 = 12;
pub const ADJUSTMENT_YAW_D: C2RustUnnamed_2 = 11;
pub const ADJUSTMENT_YAW_I: C2RustUnnamed_2 = 10;
pub const ADJUSTMENT_YAW_P: C2RustUnnamed_2 = 9;
pub const ADJUSTMENT_PITCH_ROLL_D: C2RustUnnamed_2 = 8;
pub const ADJUSTMENT_PITCH_ROLL_I: C2RustUnnamed_2 = 7;
pub const ADJUSTMENT_PITCH_ROLL_P: C2RustUnnamed_2 = 6;
pub const ADJUSTMENT_YAW_RATE: C2RustUnnamed_2 = 5;
pub const ADJUSTMENT_PITCH_ROLL_RATE: C2RustUnnamed_2 = 4;
pub const ADJUSTMENT_THROTTLE_EXPO: C2RustUnnamed_2 = 3;
pub const ADJUSTMENT_RC_EXPO: C2RustUnnamed_2 = 2;
pub const ADJUSTMENT_RC_RATE: C2RustUnnamed_2 = 1;
pub const ADJUSTMENT_NONE: C2RustUnnamed_2 = 0;
#[derive ( Copy, Clone )]
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
pub type C2RustUnnamed_3 = libc::c_uint;
pub const RC_SMOOTHING_TYPE_FILTER: C2RustUnnamed_3 = 1;
pub const RC_SMOOTHING_TYPE_INTERPOLATION: C2RustUnnamed_3 = 0;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const RC_SMOOTHING_DERIVATIVE_BIQUAD: C2RustUnnamed_4 = 2;
pub const RC_SMOOTHING_DERIVATIVE_PT1: C2RustUnnamed_4 = 1;
pub const RC_SMOOTHING_DERIVATIVE_OFF: C2RustUnnamed_4 = 0;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const RC_SMOOTHING_VALUE_AVERAGE_FRAME: C2RustUnnamed_5 = 2;
pub const RC_SMOOTHING_VALUE_DERIVATIVE_ACTIVE: C2RustUnnamed_5 = 1;
pub const RC_SMOOTHING_VALUE_INPUT_ACTIVE: C2RustUnnamed_5 = 0;
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
pub type C2RustUnnamed_6 = libc::c_uint;
pub const DSHOT_CMD_MAX: C2RustUnnamed_6 = 47;
pub const DSHOT_CMD_SILENT_MODE_ON_OFF: C2RustUnnamed_6 = 31;
pub const DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF: C2RustUnnamed_6 = 30;
pub const DSHOT_CMD_LED3_OFF: C2RustUnnamed_6 = 29;
pub const DSHOT_CMD_LED2_OFF: C2RustUnnamed_6 = 28;
pub const DSHOT_CMD_LED1_OFF: C2RustUnnamed_6 = 27;
pub const DSHOT_CMD_LED0_OFF: C2RustUnnamed_6 = 26;
pub const DSHOT_CMD_LED3_ON: C2RustUnnamed_6 = 25;
pub const DSHOT_CMD_LED2_ON: C2RustUnnamed_6 = 24;
pub const DSHOT_CMD_LED1_ON: C2RustUnnamed_6 = 23;
pub const DSHOT_CMD_LED0_ON: C2RustUnnamed_6 = 22;
pub const DSHOT_CMD_SPIN_DIRECTION_REVERSED: C2RustUnnamed_6 = 21;
pub const DSHOT_CMD_SPIN_DIRECTION_NORMAL: C2RustUnnamed_6 = 20;
pub const DSHOT_CMD_SAVE_SETTINGS: C2RustUnnamed_6 = 12;
pub const DSHOT_CMD_SETTINGS_REQUEST: C2RustUnnamed_6 = 11;
pub const DSHOT_CMD_3D_MODE_ON: C2RustUnnamed_6 = 10;
pub const DSHOT_CMD_3D_MODE_OFF: C2RustUnnamed_6 = 9;
pub const DSHOT_CMD_SPIN_DIRECTION_2: C2RustUnnamed_6 = 8;
pub const DSHOT_CMD_SPIN_DIRECTION_1: C2RustUnnamed_6 = 7;
pub const DSHOT_CMD_ESC_INFO: C2RustUnnamed_6 = 6;
pub const DSHOT_CMD_BEACON5: C2RustUnnamed_6 = 5;
pub const DSHOT_CMD_BEACON4: C2RustUnnamed_6 = 4;
pub const DSHOT_CMD_BEACON3: C2RustUnnamed_6 = 3;
pub const DSHOT_CMD_BEACON2: C2RustUnnamed_6 = 2;
pub const DSHOT_CMD_BEACON1: C2RustUnnamed_6 = 1;
pub const DSHOT_CMD_MOTOR_STOP: C2RustUnnamed_6 = 0;
pub type C2RustUnnamed_7 = libc::c_uint;
pub const PWM_TYPE_MAX: C2RustUnnamed_7 = 10;
pub const PWM_TYPE_PROSHOT1000: C2RustUnnamed_7 = 9;
pub const PWM_TYPE_DSHOT1200: C2RustUnnamed_7 = 8;
pub const PWM_TYPE_DSHOT600: C2RustUnnamed_7 = 7;
pub const PWM_TYPE_DSHOT300: C2RustUnnamed_7 = 6;
pub const PWM_TYPE_DSHOT150: C2RustUnnamed_7 = 5;
pub const PWM_TYPE_BRUSHED: C2RustUnnamed_7 = 4;
pub const PWM_TYPE_MULTISHOT: C2RustUnnamed_7 = 3;
pub const PWM_TYPE_ONESHOT42: C2RustUnnamed_7 = 2;
pub const PWM_TYPE_ONESHOT125: C2RustUnnamed_7 = 1;
pub const PWM_TYPE_STANDARD: C2RustUnnamed_7 = 0;
#[derive ( Copy, Clone )]
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct motorMixer_s {
    pub throttle: libc::c_float,
    pub roll: libc::c_float,
    pub pitch: libc::c_float,
    pub yaw: libc::c_float,
}
// when aux channel is in range...
// ..then apply the adjustment function to the auxSwitchChannel ...
// ... via slot
// Custom mixer data per motor
pub type motorMixer_t = motorMixer_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct mixerConfig_s {
    pub mixerMode: uint8_t,
    pub yaw_motors_reversed: bool,
    pub crashflip_motor_percent: uint8_t,
}
pub type mixerConfig_t = mixerConfig_s;
#[derive ( Copy, Clone )]
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
pub type pidProfile_t = pidProfile_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pidConfig_s {
    pub pid_process_denom: uint8_t,
    pub runaway_takeoff_prevention: uint8_t,
    pub runaway_takeoff_deactivate_delay: uint16_t,
    pub runaway_takeoff_deactivate_throttle: uint8_t,
}
pub type pidConfig_t = pidConfig_s;
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
#[derive ( Copy, Clone )]
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
#[derive ( Copy, Clone )]
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
pub struct clivalue_s {
    pub name: *const libc::c_char,
    pub type_0: uint8_t,
    pub config: cliValueConfig_t,
    pub pgn: pgn_t,
    pub offset: uint16_t,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union cliValueConfig_t {
    pub lookup: cliLookupTableConfig_t,
    pub minmax: cliMinMaxConfig_t,
    pub array: cliArrayLengthConfig_t,
    pub bitpos: uint8_t,
}
pub type cliArrayLengthConfig_t = cliArrayLengthConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct cliArrayLengthConfig_s {
    pub length: uint8_t,
}
pub type cliMinMaxConfig_t = cliMinMaxConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct cliMinMaxConfig_s {
    pub min: int16_t,
    pub max: int16_t,
}
pub type cliLookupTableConfig_t = cliLookupTableConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct cliLookupTableConfig_s {
    pub tableIndex: lookupTableIndex_e,
}
pub type lookupTableIndex_e = libc::c_uint;
pub const LOOKUP_TABLE_COUNT: lookupTableIndex_e = 37;
pub const TABLE_RC_SMOOTHING_DERIVATIVE_TYPE: lookupTableIndex_e = 36;
pub const TABLE_RC_SMOOTHING_INPUT_TYPE: lookupTableIndex_e = 35;
pub const TABLE_RC_SMOOTHING_DEBUG: lookupTableIndex_e = 34;
pub const TABLE_RC_SMOOTHING_TYPE: lookupTableIndex_e = 33;
pub const TABLE_ACRO_TRAINER_DEBUG: lookupTableIndex_e = 32;
pub const TABLE_ITERM_RELAX_TYPE: lookupTableIndex_e = 31;
pub const TABLE_ITERM_RELAX: lookupTableIndex_e = 30;
pub const TABLE_THROTTLE_LIMIT_TYPE: lookupTableIndex_e = 29;
pub const TABLE_RGB_GRB: lookupTableIndex_e = 28;
pub const TABLE_RATES_TYPE: lookupTableIndex_e = 27;
pub const TABLE_GYRO_OVERFLOW_CHECK: lookupTableIndex_e = 26;
pub const TABLE_RANGEFINDER_HARDWARE: lookupTableIndex_e = 25;
pub const TABLE_BUS_TYPE: lookupTableIndex_e = 24;
pub const TABLE_CAMERA_CONTROL_MODE: lookupTableIndex_e = 23;
pub const TABLE_CRASH_RECOVERY: lookupTableIndex_e = 22;
pub const TABLE_FAILSAFE_SWITCH_MODE: lookupTableIndex_e = 21;
pub const TABLE_FAILSAFE: lookupTableIndex_e = 20;
pub const TABLE_ANTI_GRAVITY_MODE: lookupTableIndex_e = 19;
pub const TABLE_DTERM_LOWPASS_TYPE: lookupTableIndex_e = 18;
pub const TABLE_LOWPASS_TYPE: lookupTableIndex_e = 17;
pub const TABLE_RC_INTERPOLATION_CHANNELS: lookupTableIndex_e = 16;
pub const TABLE_RC_INTERPOLATION: lookupTableIndex_e = 15;
pub const TABLE_MOTOR_PWM_PROTOCOL: lookupTableIndex_e = 14;
pub const TABLE_DEBUG: lookupTableIndex_e = 13;
pub const TABLE_MAG_HARDWARE: lookupTableIndex_e = 12;
pub const TABLE_BARO_HARDWARE: lookupTableIndex_e = 11;
pub const TABLE_ACC_HARDWARE: lookupTableIndex_e = 10;
pub const TABLE_GYRO_HARDWARE_LPF: lookupTableIndex_e = 9;
pub const TABLE_SERIAL_RX: lookupTableIndex_e = 8;
pub const TABLE_GIMBAL_MODE: lookupTableIndex_e = 7;
pub const TABLE_VOLTAGE_METER: lookupTableIndex_e = 6;
pub const TABLE_CURRENT_METER: lookupTableIndex_e = 5;
pub const TABLE_BLACKBOX_MODE: lookupTableIndex_e = 4;
pub const TABLE_BLACKBOX_DEVICE: lookupTableIndex_e = 3;
pub const TABLE_ALIGNMENT: lookupTableIndex_e = 2;
pub const TABLE_UNIT: lookupTableIndex_e = 1;
pub const TABLE_OFF_ON: lookupTableIndex_e = 0;
pub type clivalue_t = clivalue_s;
pub const PROFILE_RATE_VALUE: C2RustUnnamed_14 = 16;
pub const PROFILE_VALUE: C2RustUnnamed_14 = 8;
pub const MASTER_VALUE: C2RustUnnamed_14 = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct serialConfig_s {
    pub portConfigs: [serialPortConfig_t; 5],
    pub serial_update_rate_hz: uint16_t,
    pub reboot_character: uint8_t,
}
pub type serialPortConfig_t = serialPortConfig_s;
#[derive ( Copy, Clone )]
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
pub type serialConfig_t = serialConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct clicmd_t {
    pub name: *const libc::c_char,
    pub func: Option<unsafe extern "C" fn(_: *mut libc::c_char) -> ()>,
}
pub type vtxChannelActivationCondition_t = vtxChannelActivationCondition_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct vtxChannelActivationCondition_s {
    pub auxChannelIndex: uint8_t,
    pub band: uint8_t,
    pub channel: uint8_t,
    pub range: channelRange_t,
}
pub type vtxConfig_t = vtxConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct vtxConfig_s {
    pub vtxChannelActivationConditions: [vtxChannelActivationCondition_t; 10],
    pub halfDuplex: uint8_t,
}
pub const DO_DIFF: C2RustUnnamed_15 = 16;
pub type ledConfig_t = uint32_t;
pub type ledStripConfig_t = ledStripConfig_s;
#[derive ( Copy, Clone )]
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct specialColorIndexes_s {
    pub color: [uint8_t; 11],
}
pub type modeColorIndexes_t = modeColorIndexes_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct modeColorIndexes_s {
    pub color: [uint8_t; 6],
}
pub type rxChannelRangeConfig_t = rxChannelRangeConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct rxChannelRangeConfig_s {
    pub min: uint16_t,
    pub max: uint16_t,
}
pub type serialConsumer = unsafe extern "C" fn(_: uint8_t) -> ();
pub const DUMP_MASTER: C2RustUnnamed_15 = 1;
pub const SHOW_DEFAULTS: C2RustUnnamed_15 = 32;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct cfCheckFuncInfo_t {
    pub maxExecutionTime: timeUs_t,
    pub totalExecutionTime: timeUs_t,
    pub averageExecutionTime: timeUs_t,
}
pub type cfTaskId_e = libc::c_uint;
pub const TASK_SELF: cfTaskId_e = 25;
pub const TASK_NONE: cfTaskId_e = 24;
pub const TASK_COUNT: cfTaskId_e = 24;
pub const TASK_PINIOBOX: cfTaskId_e = 23;
pub const TASK_RCDEVICE: cfTaskId_e = 22;
pub const TASK_CAMCTRL: cfTaskId_e = 21;
pub const TASK_VTXCTRL: cfTaskId_e = 20;
pub const TASK_ESC_SENSOR: cfTaskId_e = 19;
pub const TASK_OSD: cfTaskId_e = 18;
pub const TASK_LEDSTRIP: cfTaskId_e = 17;
pub const TASK_TELEMETRY: cfTaskId_e = 16;
pub const TASK_ALTITUDE: cfTaskId_e = 15;
pub const TASK_RANGEFINDER: cfTaskId_e = 14;
pub const TASK_BARO: cfTaskId_e = 13;
pub const TASK_COMPASS: cfTaskId_e = 12;
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct cfTaskInfo_t {
    pub taskName: *const libc::c_char,
    pub subTaskName: *const libc::c_char,
    pub isEnabled: bool,
    pub staticPriority: uint8_t,
    pub desiredPeriod: timeDelta_t,
    pub latestDeltaTime: timeDelta_t,
    pub maxExecutionTime: timeUs_t,
    pub totalExecutionTime: timeUs_t,
    pub averageExecutionTime: timeUs_t,
}
pub const BOX: C2RustUnnamed_9 = 7;
pub const RULE: C2RustUnnamed_9 = 0;
pub const MAX: C2RustUnnamed_9 = 6;
pub const MIN: C2RustUnnamed_9 = 5;
pub const SPEED: C2RustUnnamed_9 = 4;
pub const RATE: C2RustUnnamed_9 = 3;
pub const INPUT: C2RustUnnamed_9 = 2;
pub const TARGET: C2RustUnnamed_9 = 1;
pub const ARGS_COUNT: C2RustUnnamed_9 = 8;
pub type C2RustUnnamed_9 = libc::c_uint;
pub const INPUT_0: C2RustUnnamed_10 = 1;
pub const SERVO: C2RustUnnamed_10 = 0;
pub const ARGS_COUNT_0: C2RustUnnamed_10 = 3;
pub type C2RustUnnamed_10 = libc::c_uint;
pub const REVERSE: C2RustUnnamed_10 = 2;
pub const MODE_BITSET: C2RustUnnamed_14 = 96;
pub type lookupTableEntry_t = lookupTableEntry_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct lookupTableEntry_s {
    pub values: *const *const libc::c_char,
    pub valueCount: uint8_t,
}
pub const MODE_LOOKUP: C2RustUnnamed_14 = 32;
pub const MODE_DIRECT: C2RustUnnamed_14 = 0;
pub const VAR_UINT32: C2RustUnnamed_14 = 4;
pub const VAR_INT16: C2RustUnnamed_14 = 3;
pub const VAR_UINT16: C2RustUnnamed_14 = 2;
pub const VAR_INT8: C2RustUnnamed_14 = 1;
pub const VAR_UINT8: C2RustUnnamed_14 = 0;
pub const MODE_ARRAY: C2RustUnnamed_14 = 64;
pub const FORWARD: C2RustUnnamed_11 = 5;
pub const RATE_0: C2RustUnnamed_11 = 4;
pub const MIDDLE: C2RustUnnamed_11 = 3;
pub const MAX_0: C2RustUnnamed_11 = 2;
pub const MIN_0: C2RustUnnamed_11 = 1;
pub const SERVO_ARGUMENT_COUNT: C2RustUnnamed_12 = 6;
pub const INDEX: C2RustUnnamed_11 = 0;
pub type C2RustUnnamed_11 = libc::c_uint;
pub type C2RustUnnamed_12 = libc::c_uint;
pub type serialPortUsage_t = serialPortUsage_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct serialPortUsage_s {
    pub serialPort: *mut serialPort_t,
    pub function: serialPortFunction_e,
    pub identifier: serialPortIdentifier_e,
}
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
pub type baudRate_e = libc::c_uint;
pub const BAUD_2470000: baudRate_e = 15;
pub const BAUD_2000000: baudRate_e = 14;
pub const BAUD_1500000: baudRate_e = 13;
pub const BAUD_1000000: baudRate_e = 12;
pub const BAUD_921600: baudRate_e = 11;
pub const BAUD_500000: baudRate_e = 10;
pub const BAUD_460800: baudRate_e = 9;
pub const BAUD_400000: baudRate_e = 8;
pub const BAUD_250000: baudRate_e = 7;
pub const BAUD_230400: baudRate_e = 6;
pub const BAUD_115200: baudRate_e = 5;
pub const BAUD_57600: baudRate_e = 4;
pub const BAUD_38400: baudRate_e = 3;
pub const BAUD_19200: baudRate_e = 2;
pub const BAUD_9600: baudRate_e = 1;
pub const BAUD_AUTO: baudRate_e = 0;
pub type rxFailsafeChannelConfig_t = rxFailsafeChannelConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct rxFailsafeChannelConfig_s {
    pub mode: uint8_t,
    pub step: uint8_t,
}
pub const RX_FAILSAFE_MODE_SET: rxFailsafeChannelMode_e = 2;
pub type rxFailsafeChannelMode_e = libc::c_uint;
pub const RX_FAILSAFE_MODE_INVALID: rxFailsafeChannelMode_e = 3;
pub const RX_FAILSAFE_MODE_HOLD: rxFailsafeChannelMode_e = 1;
pub const RX_FAILSAFE_MODE_AUTO: rxFailsafeChannelMode_e = 0;
pub type rxFailsafeChannelType_e = libc::c_uint;
pub const RX_FAILSAFE_TYPE_AUX: rxFailsafeChannelType_e = 1;
pub const RX_FAILSAFE_TYPE_FLIGHT: rxFailsafeChannelType_e = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct cliResourceValue_t {
    pub owner: uint8_t,
    pub pgn: pgn_t,
    pub stride: uint8_t,
    pub offset: uint8_t,
    pub maxIndex: uint8_t,
}
pub type spiPinConfig_t = spiPinConfig_s;
pub type i2cConfig_t = i2cConfig_s;
pub const HIDE_UNUSED: C2RustUnnamed_15 = 64;
pub type rxConfig_t = rxConfig_s;
pub const COLOR: C2RustUnnamed_13 = 2;
pub const FUNCTION: C2RustUnnamed_13 = 1;
pub const MODE: C2RustUnnamed_13 = 0;
pub type ledModeIndex_e = libc::c_uint;
pub const LED_AUX_CHANNEL: ledModeIndex_e = 7;
pub const LED_SPECIAL: ledModeIndex_e = 6;
pub const LED_MODE_BARO: ledModeIndex_e = 5;
pub const LED_MODE_MAG: ledModeIndex_e = 4;
pub const LED_MODE_ANGLE: ledModeIndex_e = 3;
pub const LED_MODE_HORIZON: ledModeIndex_e = 2;
pub const LED_MODE_HEADFREE: ledModeIndex_e = 1;
pub const LED_MODE_ORIENTATION: ledModeIndex_e = 0;
pub const ARGS_COUNT_1: C2RustUnnamed_13 = 3;
pub type C2RustUnnamed_13 = libc::c_uint;
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
pub const DUMP_RATES: C2RustUnnamed_15 = 4;
pub const DUMP_PROFILE: C2RustUnnamed_15 = 2;
pub const DUMP_ALL: C2RustUnnamed_15 = 8;
// which byte is used to reboot. Default 'R', could be changed carefully to something else.
// suppress LEDLOW mode if beeper is on
// See rxFailsafeChannelMode_e
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
pub type box_t = box_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct box_s {
    pub boxId: uint8_t,
    pub boxName: *const libc::c_char,
    pub permanentId: uint8_t,
}
// see boxId_e
// GUI-readable box name
// permanent ID used to identify BOX. This ID is unique for one function, DO NOT REUSE IT
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct beeperConfig_s {
    pub beeper_off_flags: uint32_t,
    pub dshotBeaconTone: uint8_t,
    pub dshotBeaconOffFlags: uint32_t,
}
pub type C2RustUnnamed_14 = libc::c_uint;
pub type C2RustUnnamed_15 = libc::c_uint;
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
// FIXME remove this for targets that don't need a CLI.  Perhaps use a no-op macro when USE_CLI is not enabled
// signal that we're in cli mode
#[no_mangle]
pub static mut cliMode: uint8_t = 0i32 as uint8_t;
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn featureConfig() -> *const featureConfig_t {
    return &mut featureConfig_System;
}
#[inline]
unsafe extern "C" fn pilotConfigMutable() -> *mut pilotConfig_t {
    return &mut pilotConfig_System;
}
#[inline]
unsafe extern "C" fn pilotConfig() -> *const pilotConfig_t {
    return &mut pilotConfig_System;
}
#[inline]
unsafe extern "C" fn systemConfig() -> *const systemConfig_t {
    return &mut systemConfig_System;
}
#[inline]
unsafe extern "C" fn adjustmentRanges(mut _index: libc::c_int)
 -> *const adjustmentRange_t {
    return &mut *adjustmentRanges_SystemArray.as_mut_ptr().offset(_index as
                                                                      isize)
               as *mut adjustmentRange_t;
}
#[inline]
unsafe extern "C" fn adjustmentRangesMutable(mut _index: libc::c_int)
 -> *mut adjustmentRange_t {
    return &mut *adjustmentRanges_SystemArray.as_mut_ptr().offset(_index as
                                                                      isize)
               as *mut adjustmentRange_t;
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
unsafe extern "C" fn modeActivationConditionsMutable(mut _index: libc::c_int)
 -> *mut modeActivationCondition_t {
    return &mut *modeActivationConditions_SystemArray.as_mut_ptr().offset(_index
                                                                              as
                                                                              isize)
               as *mut modeActivationCondition_t;
}
#[inline]
unsafe extern "C" fn customMotorMixer(mut _index: libc::c_int)
 -> *const motorMixer_t {
    return &mut *customMotorMixer_SystemArray.as_mut_ptr().offset(_index as
                                                                      isize)
               as *mut motorMixer_t;
}
#[inline]
unsafe extern "C" fn customMotorMixerMutable(mut _index: libc::c_int)
 -> *mut motorMixer_t {
    return &mut *customMotorMixer_SystemArray.as_mut_ptr().offset(_index as
                                                                      isize)
               as *mut motorMixer_t;
}
#[inline]
unsafe extern "C" fn mixerConfig() -> *const mixerConfig_t {
    return &mut mixerConfig_System;
}
#[inline]
unsafe extern "C" fn mixerConfigMutable() -> *mut mixerConfig_t {
    return &mut mixerConfig_System;
}
#[inline]
unsafe extern "C" fn motorConfig() -> *const motorConfig_t {
    return &mut motorConfig_System;
}
#[inline]
unsafe extern "C" fn pidConfig() -> *const pidConfig_t {
    return &mut pidConfig_System;
}
#[no_mangle]
pub static mut inputSource_e: C2RustUnnamed_8 = INPUT_STABILIZED_ROLL;
#[inline]
unsafe extern "C" fn customServoMixers_array() -> *mut [servoMixer_t; 16] {
    return &mut customServoMixers_SystemArray;
}
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
unsafe extern "C" fn ledStripConfig() -> *const ledStripConfig_t {
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
unsafe extern "C" fn serialConfig() -> *const serialConfig_t {
    return &mut serialConfig_System;
}
#[inline]
unsafe extern "C" fn vtxConfigMutable() -> *mut vtxConfig_t {
    return &mut vtxConfig_System;
}
#[inline]
unsafe extern "C" fn vtxConfig() -> *const vtxConfig_t {
    return &mut vtxConfig_System;
}
#[inline]
unsafe extern "C" fn beeperConfigMutable() -> *mut beeperConfig_t {
    return &mut beeperConfig_System;
}
#[inline]
unsafe extern "C" fn beeperConfig() -> *const beeperConfig_t {
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
unsafe extern "C" fn rxChannelRangeConfigsMutable(mut _index: libc::c_int)
 -> *mut rxChannelRangeConfig_t {
    return &mut *rxChannelRangeConfigs_SystemArray.as_mut_ptr().offset(_index
                                                                           as
                                                                           isize)
               as *mut rxChannelRangeConfig_t;
}
#[inline]
unsafe extern "C" fn rxChannelRangeConfigs(mut _index: libc::c_int)
 -> *const rxChannelRangeConfig_t {
    return &mut *rxChannelRangeConfigs_SystemArray.as_mut_ptr().offset(_index
                                                                           as
                                                                           isize)
               as *mut rxChannelRangeConfig_t;
}
static mut cliPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
// Space required to set array parameters
static mut cliWriter: *mut bufWriter_t =
    0 as *const bufWriter_t as *mut bufWriter_t;
static mut cliWriteBuffer: [uint8_t; 88] = [0; 88];
static mut cliBuffer: [libc::c_char; 256] = [0; 256];
static mut bufferIndex: uint32_t = 0i32 as uint32_t;
static mut configIsInCopy: bool = 0i32 != 0;
static mut pidProfileIndexToUse: int8_t = -1i32 as int8_t;
static mut rateProfileIndexToUse: int8_t = -1i32 as int8_t;
// USE_BOARD_INFO
static mut emptyName: *const libc::c_char =
    b"-\x00" as *const u8 as *const libc::c_char;
static mut emptyString: *const libc::c_char =
    b"\x00" as *const u8 as *const libc::c_char;
// sync this with mixerMode_e
static mut mixerNames: [*const libc::c_char; 27] =
    [b"TRI\x00" as *const u8 as *const libc::c_char,
     b"QUADP\x00" as *const u8 as *const libc::c_char,
     b"QUADX\x00" as *const u8 as *const libc::c_char,
     b"BI\x00" as *const u8 as *const libc::c_char,
     b"GIMBAL\x00" as *const u8 as *const libc::c_char,
     b"Y6\x00" as *const u8 as *const libc::c_char,
     b"HEX6\x00" as *const u8 as *const libc::c_char,
     b"FLYING_WING\x00" as *const u8 as *const libc::c_char,
     b"Y4\x00" as *const u8 as *const libc::c_char,
     b"HEX6X\x00" as *const u8 as *const libc::c_char,
     b"OCTOX8\x00" as *const u8 as *const libc::c_char,
     b"OCTOFLATP\x00" as *const u8 as *const libc::c_char,
     b"OCTOFLATX\x00" as *const u8 as *const libc::c_char,
     b"AIRPLANE\x00" as *const u8 as *const libc::c_char,
     b"HELI_120_CCPM\x00" as *const u8 as *const libc::c_char,
     b"HELI_90_DEG\x00" as *const u8 as *const libc::c_char,
     b"VTAIL4\x00" as *const u8 as *const libc::c_char,
     b"HEX6H\x00" as *const u8 as *const libc::c_char,
     b"RX_TO_SERVO\x00" as *const u8 as *const libc::c_char,
     b"DUALCOPTER\x00" as *const u8 as *const libc::c_char,
     b"SINGLECOPTER\x00" as *const u8 as *const libc::c_char,
     b"ATAIL4\x00" as *const u8 as *const libc::c_char,
     b"CUSTOM\x00" as *const u8 as *const libc::c_char,
     b"CUSTOMAIRPLANE\x00" as *const u8 as *const libc::c_char,
     b"CUSTOMTRI\x00" as *const u8 as *const libc::c_char,
     b"QUADX1234\x00" as *const u8 as *const libc::c_char,
     0 as *const libc::c_char];
// sync this with features_e
static mut featureNames: [*const libc::c_char; 31] =
    [b"RX_PPM\x00" as *const u8 as *const libc::c_char,
     b"\x00" as *const u8 as *const libc::c_char,
     b"INFLIGHT_ACC_CAL\x00" as *const u8 as *const libc::c_char,
     b"RX_SERIAL\x00" as *const u8 as *const libc::c_char,
     b"MOTOR_STOP\x00" as *const u8 as *const libc::c_char,
     b"SERVO_TILT\x00" as *const u8 as *const libc::c_char,
     b"SOFTSERIAL\x00" as *const u8 as *const libc::c_char,
     b"GPS\x00" as *const u8 as *const libc::c_char,
     b"\x00" as *const u8 as *const libc::c_char,
     b"RANGEFINDER\x00" as *const u8 as *const libc::c_char,
     b"TELEMETRY\x00" as *const u8 as *const libc::c_char,
     b"\x00" as *const u8 as *const libc::c_char,
     b"3D\x00" as *const u8 as *const libc::c_char,
     b"RX_PARALLEL_PWM\x00" as *const u8 as *const libc::c_char,
     b"RX_MSP\x00" as *const u8 as *const libc::c_char,
     b"RSSI_ADC\x00" as *const u8 as *const libc::c_char,
     b"LED_STRIP\x00" as *const u8 as *const libc::c_char,
     b"DISPLAY\x00" as *const u8 as *const libc::c_char,
     b"OSD\x00" as *const u8 as *const libc::c_char,
     b"\x00" as *const u8 as *const libc::c_char,
     b"CHANNEL_FORWARDING\x00" as *const u8 as *const libc::c_char,
     b"TRANSPONDER\x00" as *const u8 as *const libc::c_char,
     b"AIRMODE\x00" as *const u8 as *const libc::c_char,
     b"\x00" as *const u8 as *const libc::c_char,
     b"\x00" as *const u8 as *const libc::c_char,
     b"RX_SPI\x00" as *const u8 as *const libc::c_char,
     b"SOFTSPI\x00" as *const u8 as *const libc::c_char,
     b"ESC_SENSOR\x00" as *const u8 as *const libc::c_char,
     b"ANTI_GRAVITY\x00" as *const u8 as *const libc::c_char,
     b"DYNAMIC_FILTER\x00" as *const u8 as *const libc::c_char,
     0 as *const libc::c_char];
// sync this with rxFailsafeChannelMode_e
static mut rxFailsafeModeCharacters: [libc::c_char; 4] = [97, 104, 115, 0];
static mut rxFailsafeModesTable: [[rxFailsafeChannelMode_e; 3]; 2] =
    [[RX_FAILSAFE_MODE_AUTO, RX_FAILSAFE_MODE_HOLD, RX_FAILSAFE_MODE_INVALID],
     [RX_FAILSAFE_MODE_INVALID, RX_FAILSAFE_MODE_HOLD, RX_FAILSAFE_MODE_SET]];
// USE_SENSOR_NAMES
unsafe extern "C" fn backupPgConfig(mut pg: *const pgRegistry_t) {
    memcpy((*pg).copy as *mut libc::c_void,
           (*pg).address as *const libc::c_void, (*pg).size as libc::c_ulong);
}
unsafe extern "C" fn restorePgConfig(mut pg: *const pgRegistry_t) {
    memcpy((*pg).address as *mut libc::c_void,
           (*pg).copy as *const libc::c_void, (*pg).size as libc::c_ulong);
}
unsafe extern "C" fn backupConfigs() {
    // make copies of configs to do differencing
    let mut pg: *const pgRegistry_t = __pg_registry_start.as_ptr();
    while pg < __pg_registry_end.as_ptr() {
        backupPgConfig(pg);
        pg = pg.offset(1)
    }
    configIsInCopy = 1i32 != 0;
}
unsafe extern "C" fn restoreConfigs() {
    let mut pg: *const pgRegistry_t = __pg_registry_start.as_ptr();
    while pg < __pg_registry_end.as_ptr() {
        restorePgConfig(pg);
        pg = pg.offset(1)
    }
    configIsInCopy = 0i32 != 0;
}
unsafe extern "C" fn backupAndResetConfigs() {
    backupConfigs();
    // reset all configs to defaults to do differencing
    resetConfigs();
}
unsafe extern "C" fn cliPrint(mut str: *const libc::c_char) {
    while *str != 0 {
        let fresh0 = str;
        str = str.offset(1);
        bufWriterAppend(cliWriter, *fresh0 as uint8_t);
    }
    bufWriterFlush(cliWriter);
}
unsafe extern "C" fn cliPrintLinefeed() {
    cliPrint(b"\r\n\x00" as *const u8 as *const libc::c_char);
}
unsafe extern "C" fn cliPrintLine(mut str: *const libc::c_char) {
    cliPrint(str);
    cliPrintLinefeed();
}
unsafe extern "C" fn cliPutp(mut p: *mut libc::c_void, mut ch: libc::c_char) {
    bufWriterAppend(p as *mut bufWriter_t, ch as uint8_t);
}
unsafe extern "C" fn cliPrintfva(mut format: *const libc::c_char,
                                 mut va: ::core::ffi::VaList) {
    tfp_format(cliWriter as *mut libc::c_void,
               Some(cliPutp as
                        unsafe extern "C" fn(_: *mut libc::c_void,
                                             _: libc::c_char) -> ()), format,
               va.as_va_list());
    bufWriterFlush(cliWriter);
}
unsafe extern "C" fn cliDumpPrintLinef(mut dumpMask: uint8_t,
                                       mut equalsDefault: bool,
                                       mut format: *const libc::c_char,
                                       mut args: ...) -> bool {
    if !(dumpMask as libc::c_int & DO_DIFF as libc::c_int != 0 &&
             equalsDefault as libc::c_int != 0) {
        let mut va: ::core::ffi::VaListImpl;
        va = args.clone();
        cliPrintfva(format, va.as_va_list());
        cliPrintLinefeed();
        return 1i32 != 0
    } else { return 0i32 != 0 };
}
unsafe extern "C" fn cliWrite(mut ch: uint8_t) {
    bufWriterAppend(cliWriter, ch);
}
unsafe extern "C" fn cliDefaultPrintLinef(mut dumpMask: uint8_t,
                                          mut equalsDefault: bool,
                                          mut format: *const libc::c_char,
                                          mut args: ...) -> bool {
    if dumpMask as libc::c_int & SHOW_DEFAULTS as libc::c_int != 0 &&
           !equalsDefault {
        cliWrite('#' as i32 as uint8_t);
        let mut va: ::core::ffi::VaListImpl;
        va = args.clone();
        cliPrintfva(format, va.as_va_list());
        cliPrintLinefeed();
        return 1i32 != 0
    } else { return 0i32 != 0 };
}
unsafe extern "C" fn cliPrintf(mut format: *const libc::c_char,
                               mut args: ...) {
    let mut va: ::core::ffi::VaListImpl;
    va = args.clone();
    cliPrintfva(format, va.as_va_list());
}
unsafe extern "C" fn cliPrintLinef(mut format: *const libc::c_char,
                                   mut args: ...) {
    let mut va: ::core::ffi::VaListImpl;
    va = args.clone();
    cliPrintfva(format, va.as_va_list());
    cliPrintLinefeed();
}
unsafe extern "C" fn cliPrintErrorLinef(mut format: *const libc::c_char,
                                        mut args: ...) {
    cliPrint(b"###ERROR### \x00" as *const u8 as *const libc::c_char);
    let mut va: ::core::ffi::VaListImpl;
    va = args.clone();
    cliPrintfva(format, va.as_va_list());
    cliPrintLinefeed();
}
unsafe extern "C" fn printValuePointer(mut var: *const clivalue_t,
                                       mut valuePointer: *const libc::c_void,
                                       mut full: bool) {
    if (*var).type_0 as libc::c_int & 0x60i32 == MODE_ARRAY as libc::c_int {
        let mut i: libc::c_int = 0i32;
        while i < (*var).config.array.length as libc::c_int {
            match (*var).type_0 as libc::c_int & 0x7i32 {
                1 => {
                    // int8_t array
                    cliPrintf(b"%d\x00" as *const u8 as *const libc::c_char,
                              *(valuePointer as
                                    *mut int8_t).offset(i as isize) as
                                  libc::c_int);
                }
                2 => {
                    // uin16_t array
                    cliPrintf(b"%d\x00" as *const u8 as *const libc::c_char,
                              *(valuePointer as
                                    *mut uint16_t).offset(i as isize) as
                                  libc::c_int);
                }
                3 => {
                    // int16_t array
                    cliPrintf(b"%d\x00" as *const u8 as *const libc::c_char,
                              *(valuePointer as
                                    *mut int16_t).offset(i as isize) as
                                  libc::c_int);
                }
                0 | _ => {
                    // uint8_t array
                    cliPrintf(b"%d\x00" as *const u8 as *const libc::c_char,
                              *(valuePointer as
                                    *mut uint8_t).offset(i as isize) as
                                  libc::c_int);
                }
            }
            if i < (*var).config.array.length as libc::c_int - 1i32 {
                cliPrint(b",\x00" as *const u8 as *const libc::c_char);
            }
            i += 1
        }
    } else {
        let mut value: libc::c_int = 0i32;
        match (*var).type_0 as libc::c_int & 0x7i32 {
            0 => { value = *(valuePointer as *mut uint8_t) as libc::c_int }
            1 => { value = *(valuePointer as *mut int8_t) as libc::c_int }
            2 | 3 => {
                value = *(valuePointer as *mut int16_t) as libc::c_int
            }
            4 => { value = *(valuePointer as *mut uint32_t) as libc::c_int }
            _ => { }
        }
        match (*var).type_0 as libc::c_int & 0x60i32 {
            0 => {
                cliPrintf(b"%d\x00" as *const u8 as *const libc::c_char,
                          value);
                if full {
                    cliPrintf(b" %d %d\x00" as *const u8 as
                                  *const libc::c_char,
                              (*var).config.minmax.min as libc::c_int,
                              (*var).config.minmax.max as libc::c_int);
                }
            }
            32 => {
                cliPrint(*(*lookupTables.as_ptr().offset((*var).config.lookup.tableIndex
                                                             as
                                                             isize)).values.offset(value
                                                                                       as
                                                                                       isize));
            }
            96 => {
                if value & 1i32 << (*var).config.bitpos as libc::c_int != 0 {
                    cliPrintf(b"ON\x00" as *const u8 as *const libc::c_char);
                } else {
                    cliPrintf(b"OFF\x00" as *const u8 as *const libc::c_char);
                }
            }
            _ => { }
        }
    };
}
unsafe extern "C" fn valuePtrEqualsDefault(mut var: *const clivalue_t,
                                           mut ptr: *const libc::c_void,
                                           mut ptrDefault:
                                               *const libc::c_void) -> bool {
    let mut result: bool = 1i32 != 0;
    let mut elementCount: libc::c_int = 1i32;
    let mut mask: uint32_t = 0xffffffffu32;
    if (*var).type_0 as libc::c_int & 0x60i32 == MODE_ARRAY as libc::c_int {
        elementCount = (*var).config.array.length as libc::c_int
    }
    if (*var).type_0 as libc::c_int & 0x60i32 == MODE_BITSET as libc::c_int {
        mask = (1i32 << (*var).config.bitpos as libc::c_int) as uint32_t
    }
    let mut i: libc::c_int = 0i32;
    while i < elementCount {
        match (*var).type_0 as libc::c_int & 0x7i32 {
            0 => {
                result =
                    result as libc::c_int != 0 &&
                        *(ptr as *mut uint8_t).offset(i as isize) as
                            libc::c_uint & mask ==
                            *(ptrDefault as *mut uint8_t).offset(i as isize)
                                as libc::c_uint & mask
            }
            1 => {
                result =
                    result as libc::c_int != 0 &&
                        *(ptr as *mut int8_t).offset(i as isize) as
                            libc::c_int ==
                            *(ptrDefault as *mut int8_t).offset(i as isize) as
                                libc::c_int
            }
            2 => {
                result =
                    result as libc::c_int != 0 &&
                        *(ptr as *mut int16_t).offset(i as isize) as
                            libc::c_uint & mask ==
                            *(ptrDefault as *mut int16_t).offset(i as isize)
                                as libc::c_uint & mask
            }
            3 => {
                result =
                    result as libc::c_int != 0 &&
                        *(ptr as *mut int16_t).offset(i as isize) as
                            libc::c_int ==
                            *(ptrDefault as *mut int16_t).offset(i as isize)
                                as libc::c_int
            }
            4 => {
                result =
                    result as libc::c_int != 0 &&
                        *(ptr as *mut uint32_t).offset(i as isize) & mask ==
                            *(ptrDefault as *mut uint32_t).offset(i as isize)
                                & mask
            }
            _ => { }
        }
        i += 1
    }
    return result;
}
unsafe extern "C" fn getPidProfileIndexToUse() -> uint8_t {
    return if pidProfileIndexToUse as libc::c_int == -1i32 {
               getCurrentPidProfileIndex() as libc::c_int
           } else { pidProfileIndexToUse as libc::c_int } as uint8_t;
}
unsafe extern "C" fn getRateProfileIndexToUse() -> uint8_t {
    return if rateProfileIndexToUse as libc::c_int == -1i32 {
               getCurrentControlRateProfileIndex() as libc::c_int
           } else { rateProfileIndexToUse as libc::c_int } as uint8_t;
}
unsafe extern "C" fn getValueOffset(mut value: *const clivalue_t)
 -> uint16_t {
    match (*value).type_0 as libc::c_int & 0x18i32 {
        0 => { return (*value).offset }
        8 => {
            return ((*value).offset as
                        libc::c_ulong).wrapping_add((::core::mem::size_of::<pidProfile_t>()
                                                         as
                                                         libc::c_ulong).wrapping_mul(getPidProfileIndexToUse()
                                                                                         as
                                                                                         libc::c_ulong))
                       as uint16_t
        }
        16 => {
            return ((*value).offset as
                        libc::c_ulong).wrapping_add((::core::mem::size_of::<controlRateConfig_t>()
                                                         as
                                                         libc::c_ulong).wrapping_mul(getRateProfileIndexToUse()
                                                                                         as
                                                                                         libc::c_ulong))
                       as uint16_t
        }
        _ => { }
    }
    return 0i32 as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn cliGetValuePointer(mut value: *const clivalue_t)
 -> *mut libc::c_void {
    let mut rec: *const pgRegistry_t = pgFind((*value).pgn);
    if configIsInCopy {
        return (*rec).copy.offset(getValueOffset(value) as libc::c_int as
                                      isize) as *mut libc::c_void
    } else {
        return (*rec).address.offset(getValueOffset(value) as libc::c_int as
                                         isize) as *mut libc::c_void
    };
}
#[no_mangle]
pub unsafe extern "C" fn cliGetDefaultPointer(mut value: *const clivalue_t)
 -> *const libc::c_void {
    let mut rec: *const pgRegistry_t = pgFind((*value).pgn);
    return (*rec).address.offset(getValueOffset(value) as libc::c_int as
                                     isize) as *const libc::c_void;
}
unsafe extern "C" fn dumpPgValue(mut value: *const clivalue_t,
                                 mut dumpMask: uint8_t) {
    let mut pg: *const pgRegistry_t = pgFind((*value).pgn);
    let mut format: *const libc::c_char =
        b"set %s = \x00" as *const u8 as *const libc::c_char;
    let mut defaultFormat: *const libc::c_char =
        b"#set %s = \x00" as *const u8 as *const libc::c_char;
    let valueOffset: libc::c_int = getValueOffset(value) as libc::c_int;
    let equalsDefault: bool =
        valuePtrEqualsDefault(value,
                              (*pg).copy.offset(valueOffset as isize) as
                                  *const libc::c_void,
                              (*pg).address.offset(valueOffset as isize) as
                                  *const libc::c_void);
    if dumpMask as libc::c_int & DO_DIFF as libc::c_int == 0i32 ||
           !equalsDefault {
        if dumpMask as libc::c_int & SHOW_DEFAULTS as libc::c_int != 0 &&
               !equalsDefault {
            cliPrintf(defaultFormat, (*value).name);
            printValuePointer(value,
                              (*pg).address.offset(valueOffset as isize) as
                                  *const libc::c_void, 0i32 != 0);
            cliPrintLinefeed();
        }
        cliPrintf(format, (*value).name);
        printValuePointer(value,
                          (*pg).copy.offset(valueOffset as isize) as
                              *const libc::c_void, 0i32 != 0);
        cliPrintLinefeed();
    };
}
unsafe extern "C" fn dumpAllValues(mut valueSection: uint16_t,
                                   mut dumpMask: uint8_t) {
    let mut i: uint32_t = 0i32 as uint32_t;
    while i < valueTableEntryCount as libc::c_uint {
        let mut value: *const clivalue_t =
            &*valueTable.as_ptr().offset(i as isize) as *const clivalue_t;
        bufWriterFlush(cliWriter);
        if (*value).type_0 as libc::c_int & 0x18i32 ==
               valueSection as libc::c_int {
            dumpPgValue(value, dumpMask);
        }
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn cliPrintVar(mut var: *const clivalue_t, mut full: bool) {
    let mut ptr: *const libc::c_void = cliGetValuePointer(var);
    printValuePointer(var, ptr, full);
}
unsafe extern "C" fn cliPrintVarRange(mut var: *const clivalue_t) {
    match (*var).type_0 as libc::c_int & 0x60i32 {
        0 => {
            cliPrintLinef(b"Allowed range: %d - %d\x00" as *const u8 as
                              *const libc::c_char,
                          (*var).config.minmax.min as libc::c_int,
                          (*var).config.minmax.max as libc::c_int);
        }
        32 => {
            let mut tableEntry: *const lookupTableEntry_t =
                &*lookupTables.as_ptr().offset((*var).config.lookup.tableIndex
                                                   as isize) as
                    *const lookupTableEntry_t;
            cliPrint(b"Allowed values: \x00" as *const u8 as
                         *const libc::c_char);
            let mut firstEntry: bool = 1i32 != 0;
            let mut i: libc::c_uint = 0i32 as libc::c_uint;
            while i < (*tableEntry).valueCount as libc::c_uint {
                if !(*(*tableEntry).values.offset(i as isize)).is_null() {
                    if !firstEntry {
                        cliPrint(b", \x00" as *const u8 as
                                     *const libc::c_char);
                    }
                    cliPrintf(b"%s\x00" as *const u8 as *const libc::c_char,
                              *(*tableEntry).values.offset(i as isize));
                    firstEntry = 0i32 != 0
                }
                i = i.wrapping_add(1)
            }
            cliPrintLinefeed();
        }
        64 => {
            cliPrintLinef(b"Array length: %d\x00" as *const u8 as
                              *const libc::c_char,
                          (*var).config.array.length as libc::c_int);
        }
        96 => {
            cliPrintLinef(b"Allowed values: OFF, ON\x00" as *const u8 as
                              *const libc::c_char);
        }
        _ => { }
    };
}
unsafe extern "C" fn cliSetVar(mut var: *const clivalue_t, value: int16_t) {
    let mut ptr: *mut libc::c_void = cliGetValuePointer(var);
    let mut workValue: uint32_t = 0;
    let mut mask: uint32_t = 0;
    if (*var).type_0 as libc::c_int & 0x60i32 == MODE_BITSET as libc::c_int {
        match (*var).type_0 as libc::c_int & 0x7i32 {
            0 => {
                mask =
                    (1i32 << (*var).config.bitpos as libc::c_int & 0xffi32) as
                        uint32_t;
                if value != 0 {
                    workValue = *(ptr as *mut uint8_t) as libc::c_uint | mask
                } else {
                    workValue = *(ptr as *mut uint8_t) as libc::c_uint & !mask
                }
                *(ptr as *mut uint8_t) = workValue as uint8_t
            }
            2 => {
                mask =
                    (1i32 << (*var).config.bitpos as libc::c_int & 0xffffi32)
                        as uint32_t;
                if value != 0 {
                    workValue = *(ptr as *mut uint16_t) as libc::c_uint | mask
                } else {
                    workValue =
                        *(ptr as *mut uint16_t) as libc::c_uint & !mask
                }
                *(ptr as *mut uint16_t) = workValue as uint16_t
            }
            4 => {
                mask =
                    (1i32 << (*var).config.bitpos as libc::c_int) as uint32_t;
                if value != 0 {
                    workValue = *(ptr as *mut uint32_t) | mask
                } else { workValue = *(ptr as *mut uint32_t) & !mask }
                *(ptr as *mut uint32_t) = workValue
            }
            _ => { }
        }
    } else {
        match (*var).type_0 as libc::c_int & 0x7i32 {
            0 => { *(ptr as *mut uint8_t) = value as uint8_t }
            1 => { *(ptr as *mut int8_t) = value as int8_t }
            2 | 3 => { *(ptr as *mut int16_t) = value }
            _ => { }
        }
    };
}
unsafe extern "C" fn cliPrompt() {
    cliPrint(b"\r\n# \x00" as *const u8 as *const libc::c_char);
}
unsafe extern "C" fn cliShowParseError() {
    cliPrintErrorLinef(b"Parse error\x00" as *const u8 as
                           *const libc::c_char);
}
unsafe extern "C" fn cliShowArgumentRangeError(mut name: *mut libc::c_char,
                                               mut min: libc::c_int,
                                               mut max: libc::c_int) {
    cliPrintErrorLinef(b"%s not between %d and %d\x00" as *const u8 as
                           *const libc::c_char, name, min, max);
}
unsafe extern "C" fn nextArg(mut currentArg: *const libc::c_char)
 -> *const libc::c_char {
    let mut ptr: *const libc::c_char = strchr(currentArg, ' ' as i32);
    while !ptr.is_null() && *ptr as libc::c_int == ' ' as i32 {
        ptr = ptr.offset(1)
    }
    return ptr;
}
unsafe extern "C" fn processChannelRangeArgs(mut ptr: *const libc::c_char,
                                             mut range: *mut channelRange_t,
                                             mut validArgumentCount:
                                                 *mut uint8_t)
 -> *const libc::c_char {
    let mut argIndex: uint32_t = 0i32 as uint32_t;
    while argIndex < 2i32 as libc::c_uint {
        ptr = nextArg(ptr);
        if !ptr.is_null() {
            let mut val: libc::c_int = atoi(ptr);
            val = (constrain(val, 900i32, 2100i32) - 900i32) / 25i32;
            if val >= 0i32 && val <= (2100i32 - 900i32) / 25i32 {
                if argIndex == 0i32 as libc::c_uint {
                    (*range).startStep = val as uint8_t
                } else { (*range).endStep = val as uint8_t }
                *validArgumentCount = (*validArgumentCount).wrapping_add(1)
            }
        }
        argIndex = argIndex.wrapping_add(1)
    }
    return ptr;
}
// Check if a string's length is zero
unsafe extern "C" fn isEmpty(mut string: *const libc::c_char) -> bool {
    return if string.is_null() || *string as libc::c_int == '\u{0}' as i32 {
               1i32
           } else { 0i32 } != 0;
}
unsafe extern "C" fn printRxFailsafe(mut dumpMask: uint8_t,
                                     mut rxFailsafeChannelConfigs_0:
                                         *const rxFailsafeChannelConfig_t,
                                     mut defaultRxFailsafeChannelConfigs:
                                         *const rxFailsafeChannelConfig_t) {
    // print out rxConfig failsafe settings
    let mut channel: uint32_t = 0i32 as uint32_t;
    while channel < 18i32 as libc::c_uint {
        let mut channelFailsafeConfig: *const rxFailsafeChannelConfig_t =
            &*rxFailsafeChannelConfigs_0.offset(channel as isize) as
                *const rxFailsafeChannelConfig_t;
        let mut defaultChannelFailsafeConfig:
                *const rxFailsafeChannelConfig_t =
            &*defaultRxFailsafeChannelConfigs.offset(channel as isize) as
                *const rxFailsafeChannelConfig_t;
        let equalsDefault: bool =
            memcmp(channelFailsafeConfig as *const libc::c_void,
                   defaultChannelFailsafeConfig as *const libc::c_void,
                   ::core::mem::size_of::<rxFailsafeChannelConfig_t>() as
                       libc::c_ulong) == 0;
        let requireValue: bool =
            (*channelFailsafeConfig).mode as libc::c_int ==
                RX_FAILSAFE_MODE_SET as libc::c_int;
        if requireValue {
            let mut format: *const libc::c_char =
                b"rxfail %u %c %d\x00" as *const u8 as *const libc::c_char;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, channel,
                                 rxFailsafeModeCharacters[(*defaultChannelFailsafeConfig).mode
                                                              as usize] as
                                     libc::c_int,
                                 750i32 +
                                     25i32 *
                                         (*defaultChannelFailsafeConfig).step
                                             as libc::c_int);
            cliDumpPrintLinef(dumpMask, equalsDefault, format, channel,
                              rxFailsafeModeCharacters[(*channelFailsafeConfig).mode
                                                           as usize] as
                                  libc::c_int,
                              750i32 +
                                  25i32 *
                                      (*channelFailsafeConfig).step as
                                          libc::c_int);
        } else {
            let mut format_0: *const libc::c_char =
                b"rxfail %u %c\x00" as *const u8 as *const libc::c_char;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format_0, channel,
                                 rxFailsafeModeCharacters[(*defaultChannelFailsafeConfig).mode
                                                              as usize] as
                                     libc::c_int);
            cliDumpPrintLinef(dumpMask, equalsDefault, format_0, channel,
                              rxFailsafeModeCharacters[(*channelFailsafeConfig).mode
                                                           as usize] as
                                  libc::c_int);
        }
        channel = channel.wrapping_add(1)
    };
}
unsafe extern "C" fn cliRxFailsafe(mut cmdline: *mut libc::c_char) {
    let mut channel: uint8_t = 0;
    let mut buf: [libc::c_char; 3] = [0; 3];
    if isEmpty(cmdline) {
        // print out rxConfig failsafe settings
        channel = 0i32 as uint8_t;
        while (channel as libc::c_int) < 18i32 {
            cliRxFailsafe(itoa(channel as libc::c_int, buf.as_mut_ptr(),
                               10i32));
            channel = channel.wrapping_add(1)
        }
    } else {
        let mut ptr: *const libc::c_char = cmdline;
        let fresh1 = ptr;
        ptr = ptr.offset(1);
        channel = atoi(fresh1) as uint8_t;
        if (channel as libc::c_int) < 18i32 {
            let mut channelFailsafeConfig: *mut rxFailsafeChannelConfig_t =
                rxFailsafeChannelConfigsMutable(channel as libc::c_int);
            let type_0: rxFailsafeChannelType_e =
                if (channel as libc::c_int) < 4i32 {
                    RX_FAILSAFE_TYPE_FLIGHT as libc::c_int
                } else { RX_FAILSAFE_TYPE_AUX as libc::c_int } as
                    rxFailsafeChannelType_e;
            let mut mode: rxFailsafeChannelMode_e =
                (*channelFailsafeConfig).mode as rxFailsafeChannelMode_e;
            let mut requireValue: bool =
                (*channelFailsafeConfig).mode as libc::c_int ==
                    RX_FAILSAFE_MODE_SET as libc::c_int;
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                let mut p: *const libc::c_char =
                    strchr(rxFailsafeModeCharacters.as_ptr(),
                           *ptr as libc::c_int);
                if !p.is_null() {
                    let requestedMode: uint8_t =
                        p.wrapping_offset_from(rxFailsafeModeCharacters.as_ptr())
                            as libc::c_long as uint8_t;
                    mode =
                        rxFailsafeModesTable[type_0 as
                                                 usize][requestedMode as
                                                            usize]
                } else { mode = RX_FAILSAFE_MODE_INVALID }
                if mode as libc::c_uint ==
                       RX_FAILSAFE_MODE_INVALID as libc::c_int as libc::c_uint
                   {
                    cliShowParseError();
                    return
                }
                requireValue =
                    mode as libc::c_uint ==
                        RX_FAILSAFE_MODE_SET as libc::c_int as libc::c_uint;
                ptr = nextArg(ptr);
                if !ptr.is_null() {
                    if !requireValue { cliShowParseError(); return }
                    let mut value: uint16_t = atoi(ptr) as uint16_t;
                    value =
                        ((constrain(value as libc::c_int, 750i32, 2250i32) -
                              750i32) / 25i32) as uint16_t;
                    if value as libc::c_int > (2250i32 - 750i32) / 25i32 {
                        cliPrintLine(b"Value out of range\x00" as *const u8 as
                                         *const libc::c_char);
                        return
                    }
                    (*channelFailsafeConfig).step = value as uint8_t
                } else if requireValue { cliShowParseError(); return }
                (*channelFailsafeConfig).mode = mode as uint8_t
            }
            let mut modeCharacter: libc::c_char =
                rxFailsafeModeCharacters[(*channelFailsafeConfig).mode as
                                             usize];
            // double use of cliPrintf below
            // 1. acknowledge interpretation on command,
            // 2. query current setting on single item,
            if requireValue {
                cliPrintLinef(b"rxfail %u %c %d\x00" as *const u8 as
                                  *const libc::c_char, channel as libc::c_int,
                              modeCharacter as libc::c_int,
                              750i32 +
                                  25i32 *
                                      (*channelFailsafeConfig).step as
                                          libc::c_int);
            } else {
                cliPrintLinef(b"rxfail %u %c\x00" as *const u8 as
                                  *const libc::c_char, channel as libc::c_int,
                              modeCharacter as libc::c_int);
            }
        } else {
            cliShowArgumentRangeError(b"channel\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char, 0i32,
                                      18i32 - 1i32);
        }
    };
}
unsafe extern "C" fn printAux(mut dumpMask: uint8_t,
                              mut modeActivationConditions_0:
                                  *const modeActivationCondition_t,
                              mut defaultModeActivationConditions:
                                  *const modeActivationCondition_t) {
    let mut format: *const libc::c_char =
        b"aux %u %u %u %u %u %u %u\x00" as *const u8 as *const libc::c_char;
    // print out aux channel settings
    let mut i: uint32_t = 0i32 as uint32_t;
    while i < 20i32 as libc::c_uint {
        let mut mac: *const modeActivationCondition_t =
            &*modeActivationConditions_0.offset(i as isize) as
                *const modeActivationCondition_t;
        let mut equalsDefault: bool = 0i32 != 0;
        if !defaultModeActivationConditions.is_null() {
            let mut macDefault: *const modeActivationCondition_t =
                &*defaultModeActivationConditions.offset(i as isize) as
                    *const modeActivationCondition_t;
            equalsDefault =
                memcmp(mac as *const libc::c_void,
                       macDefault as *const libc::c_void,
                       ::core::mem::size_of::<modeActivationCondition_t>() as
                           libc::c_ulong) == 0;
            let mut box_0: *const box_t =
                findBoxByBoxId((*macDefault).modeId);
            let mut linkedTo: *const box_t =
                findBoxByBoxId((*macDefault).linkedTo);
            if !box_0.is_null() {
                cliDefaultPrintLinef(dumpMask, equalsDefault, format, i,
                                     (*box_0).permanentId as libc::c_int,
                                     (*macDefault).auxChannelIndex as
                                         libc::c_int,
                                     900i32 +
                                         25i32 *
                                             (*macDefault).range.startStep as
                                                 libc::c_int,
                                     900i32 +
                                         25i32 *
                                             (*macDefault).range.endStep as
                                                 libc::c_int,
                                     (*macDefault).modeLogic as libc::c_uint,
                                     if !linkedTo.is_null() {
                                         (*linkedTo).permanentId as
                                             libc::c_int
                                     } else { 0i32 });
            }
        }
        let mut box_1: *const box_t = findBoxByBoxId((*mac).modeId);
        let mut linkedTo_0: *const box_t = findBoxByBoxId((*mac).linkedTo);
        if !box_1.is_null() {
            cliDumpPrintLinef(dumpMask, equalsDefault, format, i,
                              (*box_1).permanentId as libc::c_int,
                              (*mac).auxChannelIndex as libc::c_int,
                              900i32 +
                                  25i32 *
                                      (*mac).range.startStep as libc::c_int,
                              900i32 +
                                  25i32 * (*mac).range.endStep as libc::c_int,
                              (*mac).modeLogic as libc::c_uint,
                              if !linkedTo_0.is_null() {
                                  (*linkedTo_0).permanentId as libc::c_int
                              } else { 0i32 });
        }
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn cliAux(mut cmdline: *mut libc::c_char) {
    let mut i: libc::c_int = 0;
    let mut val: libc::c_int = 0i32;
    let mut ptr: *const libc::c_char = 0 as *const libc::c_char;
    if isEmpty(cmdline) {
        printAux(DUMP_MASTER as libc::c_int as uint8_t,
                 modeActivationConditions(0i32),
                 0 as *const modeActivationCondition_t);
    } else {
        ptr = cmdline;
        let fresh2 = ptr;
        ptr = ptr.offset(1);
        i = atoi(fresh2);
        if i < 20i32 {
            let mut mac: *mut modeActivationCondition_t =
                modeActivationConditionsMutable(i);
            let mut validArgumentCount: uint8_t = 0i32 as uint8_t;
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                val = atoi(ptr);
                let mut box_0: *const box_t =
                    findBoxByPermanentId(val as uint8_t);
                if !box_0.is_null() {
                    (*mac).modeId = (*box_0).boxId as boxId_e;
                    validArgumentCount = validArgumentCount.wrapping_add(1)
                }
            }
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                val = atoi(ptr);
                if val >= 0i32 && val < 18i32 - 4i32 {
                    (*mac).auxChannelIndex = val as uint8_t;
                    validArgumentCount = validArgumentCount.wrapping_add(1)
                }
            }
            ptr =
                processChannelRangeArgs(ptr, &mut (*mac).range,
                                        &mut validArgumentCount);
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                val = atoi(ptr);
                if val == MODELOGIC_OR as libc::c_int ||
                       val == MODELOGIC_AND as libc::c_int {
                    (*mac).modeLogic = val as modeLogic_e;
                    validArgumentCount = validArgumentCount.wrapping_add(1)
                }
            }
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                val = atoi(ptr);
                let mut box_1: *const box_t =
                    findBoxByPermanentId(val as uint8_t);
                if !box_1.is_null() {
                    (*mac).linkedTo = (*box_1).boxId as boxId_e;
                    validArgumentCount = validArgumentCount.wrapping_add(1)
                }
            }
            if validArgumentCount as libc::c_int == 4i32 {
                // for backwards compatibility
                (*mac).modeLogic = MODELOGIC_OR
            } else if validArgumentCount as libc::c_int == 5i32 {
                // for backwards compatibility
                (*mac).linkedTo = BOXARM
            } else if validArgumentCount as libc::c_int != 6i32 {
                memset(mac as *mut libc::c_void, 0i32,
                       ::core::mem::size_of::<modeActivationCondition_t>() as
                           libc::c_ulong);
            }
            cliPrintLinef(b"aux %u %u %u %u %u %u %u\x00" as *const u8 as
                              *const libc::c_char, i,
                          (*mac).modeId as libc::c_uint,
                          (*mac).auxChannelIndex as libc::c_int,
                          900i32 +
                              25i32 * (*mac).range.startStep as libc::c_int,
                          900i32 +
                              25i32 * (*mac).range.endStep as libc::c_int,
                          (*mac).modeLogic as libc::c_uint,
                          (*mac).linkedTo as libc::c_uint);
        } else {
            cliShowArgumentRangeError(b"index\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char, 0i32,
                                      20i32 - 1i32);
        }
    };
}
unsafe extern "C" fn printSerial(mut dumpMask: uint8_t,
                                 mut serialConfig_0: *const serialConfig_t,
                                 mut serialConfigDefault:
                                     *const serialConfig_t) {
    let mut format: *const libc::c_char =
        b"serial %d %d %ld %ld %ld %ld\x00" as *const u8 as
            *const libc::c_char;
    let mut i: uint32_t = 0i32 as uint32_t;
    while i < 5i32 as libc::c_uint {
        if serialIsPortAvailable((*serialConfig_0).portConfigs[i as
                                                                   usize].identifier)
           {
            let mut equalsDefault: bool = 0i32 != 0;
            if !serialConfigDefault.is_null() {
                equalsDefault =
                    memcmp(&*(*serialConfig_0).portConfigs.as_ptr().offset(i
                                                                               as
                                                                               isize)
                               as *const serialPortConfig_t as
                               *const libc::c_void,
                           &*(*serialConfigDefault).portConfigs.as_ptr().offset(i
                                                                                    as
                                                                                    isize)
                               as *const serialPortConfig_t as
                               *const libc::c_void,
                           ::core::mem::size_of::<serialPortConfig_t>() as
                               libc::c_ulong) == 0;
                cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                                     (*serialConfigDefault).portConfigs[i as
                                                                            usize].identifier
                                         as libc::c_int,
                                     (*serialConfigDefault).portConfigs[i as
                                                                            usize].functionMask
                                         as libc::c_int,
                                     *baudRates.as_ptr().offset((*serialConfigDefault).portConfigs[i
                                                                                                       as
                                                                                                       usize].msp_baudrateIndex
                                                                    as isize),
                                     *baudRates.as_ptr().offset((*serialConfigDefault).portConfigs[i
                                                                                                       as
                                                                                                       usize].gps_baudrateIndex
                                                                    as isize),
                                     *baudRates.as_ptr().offset((*serialConfigDefault).portConfigs[i
                                                                                                       as
                                                                                                       usize].telemetry_baudrateIndex
                                                                    as isize),
                                     *baudRates.as_ptr().offset((*serialConfigDefault).portConfigs[i
                                                                                                       as
                                                                                                       usize].blackbox_baudrateIndex
                                                                    as
                                                                    isize));
            }
            cliDumpPrintLinef(dumpMask, equalsDefault, format,
                              (*serialConfig_0).portConfigs[i as
                                                                usize].identifier
                                  as libc::c_int,
                              (*serialConfig_0).portConfigs[i as
                                                                usize].functionMask
                                  as libc::c_int,
                              *baudRates.as_ptr().offset((*serialConfig_0).portConfigs[i
                                                                                           as
                                                                                           usize].msp_baudrateIndex
                                                             as isize),
                              *baudRates.as_ptr().offset((*serialConfig_0).portConfigs[i
                                                                                           as
                                                                                           usize].gps_baudrateIndex
                                                             as isize),
                              *baudRates.as_ptr().offset((*serialConfig_0).portConfigs[i
                                                                                           as
                                                                                           usize].telemetry_baudrateIndex
                                                             as isize),
                              *baudRates.as_ptr().offset((*serialConfig_0).portConfigs[i
                                                                                           as
                                                                                           usize].blackbox_baudrateIndex
                                                             as isize));
        }
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn cliSerial(mut cmdline: *mut libc::c_char) {
    let mut format: *const libc::c_char =
        b"serial %d %d %ld %ld %ld %ld\x00" as *const u8 as
            *const libc::c_char;
    if isEmpty(cmdline) {
        printSerial(DUMP_MASTER as libc::c_int as uint8_t, serialConfig(),
                    0 as *const serialConfig_t);
        return
    }
    let mut portConfig: serialPortConfig_t =
        serialPortConfig_t{functionMask: 0,
                           identifier: SERIAL_PORT_USART1,
                           msp_baudrateIndex: 0,
                           gps_baudrateIndex: 0,
                           blackbox_baudrateIndex: 0,
                           telemetry_baudrateIndex: 0,};
    memset(&mut portConfig as *mut serialPortConfig_t as *mut libc::c_void,
           0i32,
           ::core::mem::size_of::<serialPortConfig_t>() as libc::c_ulong);
    let mut currentConfig: *mut serialPortConfig_t =
        0 as *mut serialPortConfig_t;
    let mut validArgumentCount: uint8_t = 0i32 as uint8_t;
    let mut ptr: *const libc::c_char = cmdline;
    let fresh3 = ptr;
    ptr = ptr.offset(1);
    let mut val: libc::c_int = atoi(fresh3);
    currentConfig =
        serialFindPortConfiguration(val as serialPortIdentifier_e);
    if !currentConfig.is_null() {
        portConfig.identifier = val as serialPortIdentifier_e;
        validArgumentCount = validArgumentCount.wrapping_add(1)
    }
    ptr = nextArg(ptr);
    if !ptr.is_null() {
        val = atoi(ptr);
        portConfig.functionMask = (val & 0xffffi32) as uint16_t;
        validArgumentCount = validArgumentCount.wrapping_add(1)
    }
    let mut current_block_22: u64;
    let mut i: libc::c_int = 0i32;
    while i < 4i32 {
        ptr = nextArg(ptr);
        if ptr.is_null() { break ; }
        val = atoi(ptr);
        let mut baudRateIndex: uint8_t =
            lookupBaudRateIndex(val as uint32_t) as uint8_t;
        if *baudRates.as_ptr().offset(baudRateIndex as isize) !=
               val as uint32_t {
            break ;
        }
        match i {
            0 => {
                if (baudRateIndex as libc::c_int) < BAUD_9600 as libc::c_int
                       ||
                       baudRateIndex as libc::c_int >
                           BAUD_1000000 as libc::c_int {
                    current_block_22 = 6009453772311597924;
                } else {
                    portConfig.msp_baudrateIndex = baudRateIndex;
                    current_block_22 = 9007357115414505193;
                }
            }
            1 => {
                if (baudRateIndex as libc::c_int) < BAUD_9600 as libc::c_int
                       ||
                       baudRateIndex as libc::c_int >
                           BAUD_115200 as libc::c_int {
                    current_block_22 = 6009453772311597924;
                } else {
                    portConfig.gps_baudrateIndex = baudRateIndex;
                    current_block_22 = 9007357115414505193;
                }
            }
            2 => {
                if baudRateIndex as libc::c_int != BAUD_AUTO as libc::c_int &&
                       baudRateIndex as libc::c_int >
                           BAUD_115200 as libc::c_int {
                    current_block_22 = 6009453772311597924;
                } else {
                    portConfig.telemetry_baudrateIndex = baudRateIndex;
                    current_block_22 = 9007357115414505193;
                }
            }
            3 => {
                if (baudRateIndex as libc::c_int) < BAUD_19200 as libc::c_int
                       ||
                       baudRateIndex as libc::c_int >
                           BAUD_2470000 as libc::c_int {
                    current_block_22 = 6009453772311597924;
                } else {
                    portConfig.blackbox_baudrateIndex = baudRateIndex;
                    current_block_22 = 9007357115414505193;
                }
            }
            _ => { current_block_22 = 9007357115414505193; }
        }
        match current_block_22 {
            9007357115414505193 => {
                validArgumentCount = validArgumentCount.wrapping_add(1)
            }
            _ => { }
        }
        i += 1
    }
    if (validArgumentCount as libc::c_int) < 6i32 {
        cliShowParseError();
        return
    }
    memcpy(currentConfig as *mut libc::c_void,
           &mut portConfig as *mut serialPortConfig_t as *const libc::c_void,
           ::core::mem::size_of::<serialPortConfig_t>() as libc::c_ulong);
    cliDumpPrintLinef(0i32 as uint8_t, 0i32 != 0, format,
                      portConfig.identifier as libc::c_int,
                      portConfig.functionMask as libc::c_int,
                      *baudRates.as_ptr().offset(portConfig.msp_baudrateIndex
                                                     as isize),
                      *baudRates.as_ptr().offset(portConfig.gps_baudrateIndex
                                                     as isize),
                      *baudRates.as_ptr().offset(portConfig.telemetry_baudrateIndex
                                                     as isize),
                      *baudRates.as_ptr().offset(portConfig.blackbox_baudrateIndex
                                                     as isize));
}
unsafe extern "C" fn cbCtrlLine(mut context: *mut libc::c_void,
                                mut ctrl: uint16_t) {
    let mut pinioDtr: libc::c_int = context as libc::c_long as libc::c_int;
    pinioSet(pinioDtr, ctrl as libc::c_int & 1i32 << 0i32 == 0);
}
/* USE_PINIO */
unsafe extern "C" fn cliSerialPassthrough(mut cmdline: *mut libc::c_char) {
    if isEmpty(cmdline) { cliShowParseError(); return }
    let mut id: libc::c_int = -1i32;
    let mut baud: uint32_t = 0i32 as uint32_t;
    let mut enableBaudCb: bool = 0i32 != 0;
    let mut pinioDtr: libc::c_int = 0i32;
    /* USE_PINIO */
    let mut mode: libc::c_uint = 0i32 as libc::c_uint;
    let mut saveptr: *mut libc::c_char = 0 as *mut libc::c_char;
    let mut tok: *mut libc::c_char =
        strtok_r(cmdline, b" \x00" as *const u8 as *const libc::c_char,
                 &mut saveptr);
    let mut index: libc::c_int = 0i32;
    while !tok.is_null() {
        match index {
            0 => {
                id = atoi(tok)
                /* USE_PINIO */
            }
            1 => { baud = atoi(tok) as uint32_t }
            2 => {
                if !strstr(tok,
                           b"rx\x00" as *const u8 as
                               *const libc::c_char).is_null() ||
                       !strstr(tok,
                               b"RX\x00" as *const u8 as
                                   *const libc::c_char).is_null() {
                    mode |= MODE_RX as libc::c_int as libc::c_uint
                }
                if !strstr(tok,
                           b"tx\x00" as *const u8 as
                               *const libc::c_char).is_null() ||
                       !strstr(tok,
                               b"TX\x00" as *const u8 as
                                   *const libc::c_char).is_null() {
                    mode |= MODE_TX as libc::c_int as libc::c_uint
                }
            }
            3 => { pinioDtr = atoi(tok) }
            _ => { }
        }
        index += 1;
        tok =
            strtok_r(0 as *mut libc::c_char,
                     b" \x00" as *const u8 as *const libc::c_char,
                     &mut saveptr)
    }
    if baud == 0i32 as libc::c_uint { enableBaudCb = 1i32 != 0 }
    cliPrintf(b"Port %d \x00" as *const u8 as *const libc::c_char, id);
    let mut passThroughPort: *mut serialPort_t = 0 as *mut serialPort_t;
    let mut passThroughPortUsage: *mut serialPortUsage_t =
        findSerialPortUsageByIdentifier(id as serialPortIdentifier_e);
    if passThroughPortUsage.is_null() ||
           (*passThroughPortUsage).serialPort.is_null() {
        if enableBaudCb {
            // Set default baud
            baud = 57600i32 as uint32_t
        }
        if mode == 0 { mode = MODE_RXTX as libc::c_int as libc::c_uint }
        passThroughPort =
            openSerialPort(id as serialPortIdentifier_e, FUNCTION_NONE, None,
                           0 as *mut libc::c_void, baud, mode as portMode_e,
                           SERIAL_NOT_INVERTED);
        if passThroughPort.is_null() {
            cliPrintLine(b"could not be opened.\x00" as *const u8 as
                             *const libc::c_char);
            return
        }
        if enableBaudCb {
            cliPrintf(b"opened, default baud = %d.\r\n\x00" as *const u8 as
                          *const libc::c_char, baud);
        } else {
            cliPrintf(b"opened, baud = %d.\r\n\x00" as *const u8 as
                          *const libc::c_char, baud);
        }
    } else {
        passThroughPort = (*passThroughPortUsage).serialPort;
        // If the user supplied a mode, override the port's mode, otherwise
        // leave the mode unchanged. serialPassthrough() handles one-way ports.
        // Set the baud rate if specified
        if baud != 0 {
            cliPrintf(b"already open, setting baud = %d.\n\r\x00" as *const u8
                          as *const libc::c_char, baud);
            serialSetBaudRate(passThroughPort, baud);
        } else {
            cliPrintf(b"already open, baud = %d.\n\r\x00" as *const u8 as
                          *const libc::c_char, (*passThroughPort).baudRate);
        }
        if mode != 0 && (*passThroughPort).mode as libc::c_uint != mode {
            cliPrintf(b"Mode changed from %d to %d.\r\n\x00" as *const u8 as
                          *const libc::c_char,
                      (*passThroughPort).mode as libc::c_uint, mode);
            serialSetMode(passThroughPort, mode as portMode_e);
        }
        // If this port has a rx callback associated we need to remove it now.
        // Otherwise no data will be pushed in the serial port buffer!
        if (*passThroughPort).rxCallback.is_some() {
            (*passThroughPort).rxCallback = None
        }
    }
    // If no baud rate is specified allow to be set via USB
    if enableBaudCb {
        cliPrintLine(b"Baud rate change over USB enabled.\x00" as *const u8 as
                         *const libc::c_char);
        // Register the right side baud rate setting routine with the left side which allows setting of the UART
        // baud rate over USB without setting it using the serialpassthrough command
        serialSetBaudRateCb(cliPort,
                            Some(serialSetBaudRate as
                                     unsafe extern "C" fn(_:
                                                              *mut serialPort_t,
                                                          _: uint32_t) -> ()),
                            passThroughPort);
    }
    cliPrintLine(b"Forwarding, power cycle to exit.\x00" as *const u8 as
                     *const libc::c_char);
    // Register control line state callback
    if pinioDtr != 0 {
        serialSetCtrlLineStateCb(cliPort,
                                 Some(cbCtrlLine as
                                          unsafe extern "C" fn(_:
                                                                   *mut libc::c_void,
                                                               _: uint16_t)
                                              -> ()),
                                 (pinioDtr - 1i32) as intptr_t as
                                     *mut libc::c_void);
    }
    /* USE_PINIO */
    serialPassthrough(cliPort, passThroughPort, None, None);
}
unsafe extern "C" fn printAdjustmentRange(mut dumpMask: uint8_t,
                                          mut adjustmentRanges_0:
                                              *const adjustmentRange_t,
                                          mut defaultAdjustmentRanges:
                                              *const adjustmentRange_t) {
    let mut format: *const libc::c_char =
        b"adjrange %u %u %u %u %u %u %u %u %u\x00" as *const u8 as
            *const libc::c_char;
    // print out adjustment ranges channel settings
    let mut i: uint32_t = 0i32 as uint32_t;
    while i < 15i32 as libc::c_uint {
        let mut ar: *const adjustmentRange_t =
            &*adjustmentRanges_0.offset(i as isize) as
                *const adjustmentRange_t;
        let mut equalsDefault: bool = 0i32 != 0;
        if !defaultAdjustmentRanges.is_null() {
            let mut arDefault: *const adjustmentRange_t =
                &*defaultAdjustmentRanges.offset(i as isize) as
                    *const adjustmentRange_t;
            equalsDefault =
                memcmp(ar as *const libc::c_void,
                       arDefault as *const libc::c_void,
                       ::core::mem::size_of::<adjustmentRange_t>() as
                           libc::c_ulong) == 0;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, i,
                                 (*arDefault).adjustmentIndex as libc::c_int,
                                 (*arDefault).auxChannelIndex as libc::c_int,
                                 900i32 +
                                     25i32 *
                                         (*arDefault).range.startStep as
                                             libc::c_int,
                                 900i32 +
                                     25i32 *
                                         (*arDefault).range.endStep as
                                             libc::c_int,
                                 (*arDefault).adjustmentFunction as
                                     libc::c_int,
                                 (*arDefault).auxSwitchChannelIndex as
                                     libc::c_int,
                                 (*arDefault).adjustmentCenter as libc::c_int,
                                 (*arDefault).adjustmentScale as libc::c_int);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, i,
                          (*ar).adjustmentIndex as libc::c_int,
                          (*ar).auxChannelIndex as libc::c_int,
                          900i32 +
                              25i32 * (*ar).range.startStep as libc::c_int,
                          900i32 + 25i32 * (*ar).range.endStep as libc::c_int,
                          (*ar).adjustmentFunction as libc::c_int,
                          (*ar).auxSwitchChannelIndex as libc::c_int,
                          (*ar).adjustmentCenter as libc::c_int,
                          (*ar).adjustmentScale as libc::c_int);
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn cliAdjustmentRange(mut cmdline: *mut libc::c_char) {
    let mut format: *const libc::c_char =
        b"adjrange %u %u %u %u %u %u %u %u %u\x00" as *const u8 as
            *const libc::c_char;
    let mut i: libc::c_int = 0;
    let mut val: libc::c_int = 0i32;
    let mut ptr: *const libc::c_char = 0 as *const libc::c_char;
    if isEmpty(cmdline) {
        printAdjustmentRange(DUMP_MASTER as libc::c_int as uint8_t,
                             adjustmentRanges(0i32),
                             0 as *const adjustmentRange_t);
    } else {
        ptr = cmdline;
        let fresh4 = ptr;
        ptr = ptr.offset(1);
        i = atoi(fresh4);
        if i < 15i32 {
            let mut ar: *mut adjustmentRange_t = adjustmentRangesMutable(i);
            let mut validArgumentCount: uint8_t = 0i32 as uint8_t;
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                val = atoi(ptr);
                if val >= 0i32 && val < 4i32 {
                    (*ar).adjustmentIndex = val as uint8_t;
                    validArgumentCount = validArgumentCount.wrapping_add(1)
                }
            }
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                val = atoi(ptr);
                if val >= 0i32 && val < 18i32 - 4i32 {
                    (*ar).auxChannelIndex = val as uint8_t;
                    validArgumentCount = validArgumentCount.wrapping_add(1)
                }
            }
            ptr =
                processChannelRangeArgs(ptr, &mut (*ar).range,
                                        &mut validArgumentCount);
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                val = atoi(ptr);
                if val >= 0i32 &&
                       val < ADJUSTMENT_FUNCTION_COUNT as libc::c_int {
                    (*ar).adjustmentFunction = val as uint8_t;
                    validArgumentCount = validArgumentCount.wrapping_add(1)
                }
            }
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                val = atoi(ptr);
                if val >= 0i32 && val < 18i32 - 4i32 {
                    (*ar).auxSwitchChannelIndex = val as uint8_t;
                    validArgumentCount = validArgumentCount.wrapping_add(1)
                }
            }
            if validArgumentCount as libc::c_int != 6i32 {
                memset(ar as *mut libc::c_void, 0i32,
                       ::core::mem::size_of::<adjustmentRange_t>() as
                           libc::c_ulong);
                cliShowParseError();
                return
            }
            // Optional arguments
            (*ar).adjustmentCenter = 0i32 as uint16_t;
            (*ar).adjustmentScale = 0i32 as uint16_t;
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                val = atoi(ptr);
                (*ar).adjustmentCenter = val as uint16_t;
                validArgumentCount = validArgumentCount.wrapping_add(1)
            }
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                val = atoi(ptr);
                (*ar).adjustmentScale = val as uint16_t;
                validArgumentCount = validArgumentCount.wrapping_add(1)
            }
            cliDumpPrintLinef(0i32 as uint8_t, 0i32 != 0, format, i,
                              (*ar).adjustmentIndex as libc::c_int,
                              (*ar).auxChannelIndex as libc::c_int,
                              900i32 +
                                  25i32 *
                                      (*ar).range.startStep as libc::c_int,
                              900i32 +
                                  25i32 * (*ar).range.endStep as libc::c_int,
                              (*ar).adjustmentFunction as libc::c_int,
                              (*ar).auxSwitchChannelIndex as libc::c_int,
                              (*ar).adjustmentCenter as libc::c_int,
                              (*ar).adjustmentScale as libc::c_int);
        } else {
            cliShowArgumentRangeError(b"index\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char, 0i32,
                                      15i32 - 1i32);
        }
    };
}
unsafe extern "C" fn printMotorMix(mut dumpMask: uint8_t,
                                   mut customMotorMixer_0:
                                       *const motorMixer_t,
                                   mut defaultCustomMotorMixer:
                                       *const motorMixer_t) {
    let mut format: *const libc::c_char =
        b"mmix %d %s %s %s %s\x00" as *const u8 as *const libc::c_char;
    let mut buf0: [libc::c_char; 11] = [0; 11];
    let mut buf1: [libc::c_char; 11] = [0; 11];
    let mut buf2: [libc::c_char; 11] = [0; 11];
    let mut buf3: [libc::c_char; 11] = [0; 11];
    let mut i: uint32_t = 0i32 as uint32_t;
    while i < 8i32 as libc::c_uint {
        if (*customMotorMixer_0.offset(i as isize)).throttle == 0.0f32 {
            break ;
        }
        let thr: libc::c_float =
            (*customMotorMixer_0.offset(i as isize)).throttle;
        let roll: libc::c_float =
            (*customMotorMixer_0.offset(i as isize)).roll;
        let pitch: libc::c_float =
            (*customMotorMixer_0.offset(i as isize)).pitch;
        let yaw: libc::c_float = (*customMotorMixer_0.offset(i as isize)).yaw;
        let mut equalsDefault: bool = 0i32 != 0;
        if !defaultCustomMotorMixer.is_null() {
            let thrDefault: libc::c_float =
                (*defaultCustomMotorMixer.offset(i as isize)).throttle;
            let rollDefault: libc::c_float =
                (*defaultCustomMotorMixer.offset(i as isize)).roll;
            let pitchDefault: libc::c_float =
                (*defaultCustomMotorMixer.offset(i as isize)).pitch;
            let yawDefault: libc::c_float =
                (*defaultCustomMotorMixer.offset(i as isize)).yaw;
            let equalsDefault_0: bool =
                thr == thrDefault && roll == rollDefault &&
                    pitch == pitchDefault && yaw == yawDefault;
            cliDefaultPrintLinef(dumpMask, equalsDefault_0, format, i,
                                 ftoa(thrDefault, buf0.as_mut_ptr()),
                                 ftoa(rollDefault, buf1.as_mut_ptr()),
                                 ftoa(pitchDefault, buf2.as_mut_ptr()),
                                 ftoa(yawDefault, buf3.as_mut_ptr()));
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, i,
                          ftoa(thr, buf0.as_mut_ptr()),
                          ftoa(roll, buf1.as_mut_ptr()),
                          ftoa(pitch, buf2.as_mut_ptr()),
                          ftoa(yaw, buf3.as_mut_ptr()));
        i = i.wrapping_add(1)
    };
}
// USE_QUAD_MIXER_ONLY
unsafe extern "C" fn cliMotorMix(mut cmdline: *mut libc::c_char) {
    let mut check: libc::c_int = 0i32;
    let mut len: uint8_t = 0;
    let mut ptr: *const libc::c_char = 0 as *const libc::c_char;
    if isEmpty(cmdline) {
        printMotorMix(DUMP_MASTER as libc::c_int as uint8_t,
                      customMotorMixer(0i32), 0 as *const motorMixer_t);
    } else if strncasecmp(cmdline,
                          b"reset\x00" as *const u8 as *const libc::c_char,
                          5i32 as libc::c_ulong) == 0i32 {
        // erase custom mixer
        let mut i: uint32_t = 0i32 as uint32_t; // get motor number
        while i < 8i32 as libc::c_uint {
            (*customMotorMixerMutable(i as libc::c_int)).throttle = 0.0f32;
            i = i.wrapping_add(1)
        }
    } else if strncasecmp(cmdline,
                          b"load\x00" as *const u8 as *const libc::c_char,
                          4i32 as libc::c_ulong) == 0i32 {
        ptr = nextArg(cmdline);
        if !ptr.is_null() {
            len = strlen(ptr) as uint8_t;
            let mut i_0: uint32_t = 0i32 as uint32_t;
            loop  {
                if mixerNames[i_0 as usize].is_null() {
                    cliPrintErrorLinef(b"Invalid name\x00" as *const u8 as
                                           *const libc::c_char);
                    break ;
                } else if strncasecmp(ptr, mixerNames[i_0 as usize],
                                      len as libc::c_ulong) == 0i32 {
                    mixerLoadMix(i_0 as libc::c_int,
                                 customMotorMixerMutable(0i32));
                    cliPrintLinef(b"Loaded %s\x00" as *const u8 as
                                      *const libc::c_char,
                                  mixerNames[i_0 as usize]);
                    cliMotorMix(b"\x00" as *const u8 as *const libc::c_char as
                                    *mut libc::c_char);
                    break ;
                } else { i_0 = i_0.wrapping_add(1) }
            }
        }
    } else {
        ptr = cmdline;
        let mut i_1: uint32_t = atoi(ptr) as uint32_t;
        if i_1 < 8i32 as libc::c_uint {
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                (*customMotorMixerMutable(i_1 as libc::c_int)).throttle =
                    fastA2F(ptr);
                check += 1
            }
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                (*customMotorMixerMutable(i_1 as libc::c_int)).roll =
                    fastA2F(ptr);
                check += 1
            }
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                (*customMotorMixerMutable(i_1 as libc::c_int)).pitch =
                    fastA2F(ptr);
                check += 1
            }
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                (*customMotorMixerMutable(i_1 as libc::c_int)).yaw =
                    fastA2F(ptr);
                check += 1
            }
            if check != 4i32 {
                cliShowParseError();
            } else {
                printMotorMix(DUMP_MASTER as libc::c_int as uint8_t,
                              customMotorMixer(0i32),
                              0 as *const motorMixer_t);
            }
        } else {
            cliShowArgumentRangeError(b"index\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char, 0i32,
                                      8i32 - 1i32);
        }
    };
}
unsafe extern "C" fn printRxRange(mut dumpMask: uint8_t,
                                  mut channelRangeConfigs:
                                      *const rxChannelRangeConfig_t,
                                  mut defaultChannelRangeConfigs:
                                      *const rxChannelRangeConfig_t) {
    let mut format: *const libc::c_char =
        b"rxrange %u %u %u\x00" as *const u8 as *const libc::c_char;
    let mut i: uint32_t = 0i32 as uint32_t;
    while i < 4i32 as libc::c_uint {
        let mut equalsDefault: bool = 0i32 != 0;
        if !defaultChannelRangeConfigs.is_null() {
            equalsDefault =
                memcmp(&*channelRangeConfigs.offset(i as isize) as
                           *const rxChannelRangeConfig_t as
                           *const libc::c_void,
                       &*defaultChannelRangeConfigs.offset(i as isize) as
                           *const rxChannelRangeConfig_t as
                           *const libc::c_void,
                       ::core::mem::size_of::<rxChannelRangeConfig_t>() as
                           libc::c_ulong) == 0;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, i,
                                 (*defaultChannelRangeConfigs.offset(i as
                                                                         isize)).min
                                     as libc::c_int,
                                 (*defaultChannelRangeConfigs.offset(i as
                                                                         isize)).max
                                     as libc::c_int);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, i,
                          (*channelRangeConfigs.offset(i as isize)).min as
                              libc::c_int,
                          (*channelRangeConfigs.offset(i as isize)).max as
                              libc::c_int);
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn cliRxRange(mut cmdline: *mut libc::c_char) {
    let mut format: *const libc::c_char =
        b"rxrange %u %u %u\x00" as *const u8 as *const libc::c_char;
    let mut i: libc::c_int = 0;
    let mut validArgumentCount: libc::c_int = 0i32;
    let mut ptr: *const libc::c_char = 0 as *const libc::c_char;
    if isEmpty(cmdline) {
        printRxRange(DUMP_MASTER as libc::c_int as uint8_t,
                     rxChannelRangeConfigs(0i32),
                     0 as *const rxChannelRangeConfig_t);
    } else if strcasecmp(cmdline,
                         b"reset\x00" as *const u8 as *const libc::c_char) ==
                  0i32 {
        resetAllRxChannelRangeConfigurations(rxChannelRangeConfigsMutable(0i32));
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if i >= 0i32 && i < 4i32 {
            let mut rangeMin: libc::c_int = 750i32;
            let mut rangeMax: libc::c_int = 2250i32;
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                rangeMin = atoi(ptr);
                validArgumentCount += 1
            }
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                rangeMax = atoi(ptr);
                validArgumentCount += 1
            }
            if validArgumentCount != 2i32 {
                cliShowParseError();
            } else if rangeMin < 750i32 || rangeMin > 2250i32 ||
                          rangeMax < 750i32 || rangeMax > 2250i32 {
                cliShowParseError();
            } else {
                let mut channelRangeConfig: *mut rxChannelRangeConfig_t =
                    rxChannelRangeConfigsMutable(i);
                (*channelRangeConfig).min = rangeMin as uint16_t;
                (*channelRangeConfig).max = rangeMax as uint16_t;
                cliDumpPrintLinef(0i32 as uint8_t, 0i32 != 0, format, i,
                                  (*channelRangeConfig).min as libc::c_int,
                                  (*channelRangeConfig).max as libc::c_int);
            }
        } else {
            cliShowArgumentRangeError(b"channel\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char, 0i32,
                                      4i32 - 1i32);
        }
    };
}
unsafe extern "C" fn printLed(mut dumpMask: uint8_t,
                              mut ledConfigs: *const ledConfig_t,
                              mut defaultLedConfigs: *const ledConfig_t) {
    let mut format: *const libc::c_char =
        b"led %u %s\x00" as *const u8 as *const libc::c_char;
    let mut ledConfigBuffer: [libc::c_char; 20] = [0; 20];
    let mut ledConfigDefaultBuffer: [libc::c_char; 20] = [0; 20];
    let mut i: uint32_t = 0i32 as uint32_t;
    while i < 32i32 as libc::c_uint {
        let mut ledConfig: ledConfig_t = *ledConfigs.offset(i as isize);
        generateLedConfig(&mut ledConfig, ledConfigBuffer.as_mut_ptr(),
                          ::core::mem::size_of::<[libc::c_char; 20]>() as
                              libc::c_ulong);
        let mut equalsDefault: bool = 0i32 != 0;
        if !defaultLedConfigs.is_null() {
            let mut ledConfigDefault: ledConfig_t =
                *defaultLedConfigs.offset(i as isize);
            equalsDefault = ledConfig == ledConfigDefault;
            generateLedConfig(&mut ledConfigDefault,
                              ledConfigDefaultBuffer.as_mut_ptr(),
                              ::core::mem::size_of::<[libc::c_char; 20]>() as
                                  libc::c_ulong);
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, i,
                                 ledConfigDefaultBuffer.as_mut_ptr());
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, i,
                          ledConfigBuffer.as_mut_ptr());
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn cliLed(mut cmdline: *mut libc::c_char) {
    let mut format: *const libc::c_char =
        b"led %u %s\x00" as *const u8 as *const libc::c_char;
    let mut ledConfigBuffer: [libc::c_char; 20] = [0; 20];
    let mut i: libc::c_int = 0;
    let mut ptr: *const libc::c_char = 0 as *const libc::c_char;
    if isEmpty(cmdline) {
        printLed(DUMP_MASTER as libc::c_int as uint8_t,
                 (*ledStripConfig()).ledConfigs.as_ptr(),
                 0 as *const ledConfig_t);
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if i < 32i32 {
            ptr = nextArg(cmdline);
            if parseLedStripConfig(i, ptr) {
                generateLedConfig(&*(*(ledStripConfig as
                                           unsafe extern "C" fn()
                                               ->
                                                   *const ledStripConfig_t)()).ledConfigs.as_ptr().offset(i
                                                                                                              as
                                                                                                              isize)
                                      as *const ledConfig_t as
                                      *mut ledConfig_t,
                                  ledConfigBuffer.as_mut_ptr(),
                                  ::core::mem::size_of::<[libc::c_char; 20]>()
                                      as libc::c_ulong);
                cliDumpPrintLinef(0i32 as uint8_t, 0i32 != 0, format, i,
                                  ledConfigBuffer.as_mut_ptr());
            } else { cliShowParseError(); }
        } else {
            cliShowArgumentRangeError(b"index\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char, 0i32,
                                      32i32 - 1i32);
        }
    };
}
unsafe extern "C" fn printColor(mut dumpMask: uint8_t,
                                mut colors_0: *const hsvColor_t,
                                mut defaultColors: *const hsvColor_t) {
    let mut format: *const libc::c_char =
        b"color %u %d,%u,%u\x00" as *const u8 as *const libc::c_char;
    let mut i: uint32_t = 0i32 as uint32_t;
    while i < 16i32 as libc::c_uint {
        let mut color: *const hsvColor_t =
            &*colors_0.offset(i as isize) as *const hsvColor_t;
        let mut equalsDefault: bool = 0i32 != 0;
        if !defaultColors.is_null() {
            let mut colorDefault: *const hsvColor_t =
                &*defaultColors.offset(i as isize) as *const hsvColor_t;
            equalsDefault =
                memcmp(color as *const libc::c_void,
                       colorDefault as *const libc::c_void,
                       ::core::mem::size_of::<hsvColor_t>() as libc::c_ulong)
                    == 0;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, i,
                                 (*colorDefault).h as libc::c_int,
                                 (*colorDefault).s as libc::c_int,
                                 (*colorDefault).v as libc::c_int);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, i,
                          (*color).h as libc::c_int,
                          (*color).s as libc::c_int,
                          (*color).v as libc::c_int);
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn cliColor(mut cmdline: *mut libc::c_char) {
    let mut format: *const libc::c_char =
        b"color %u %d,%u,%u\x00" as *const u8 as *const libc::c_char;
    if isEmpty(cmdline) {
        printColor(DUMP_MASTER as libc::c_int as uint8_t,
                   (*ledStripConfig()).colors.as_ptr(),
                   0 as *const hsvColor_t);
    } else {
        let mut ptr: *const libc::c_char = cmdline;
        let i: libc::c_int = atoi(ptr);
        if i < 16i32 {
            ptr = nextArg(cmdline);
            if parseColor(i, ptr) {
                let mut color: *const hsvColor_t =
                    &*(*(ledStripConfig as
                             unsafe extern "C" fn()
                                 ->
                                     *const ledStripConfig_t)()).colors.as_ptr().offset(i
                                                                                            as
                                                                                            isize)
                        as *const hsvColor_t;
                cliDumpPrintLinef(0i32 as uint8_t, 0i32 != 0, format, i,
                                  (*color).h as libc::c_int,
                                  (*color).s as libc::c_int,
                                  (*color).v as libc::c_int);
            } else { cliShowParseError(); }
        } else {
            cliShowArgumentRangeError(b"index\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char, 0i32,
                                      16i32 - 1i32);
        }
    };
}
unsafe extern "C" fn printModeColor(mut dumpMask: uint8_t,
                                    mut ledStripConfig_0:
                                        *const ledStripConfig_t,
                                    mut defaultLedStripConfig:
                                        *const ledStripConfig_t) {
    let mut format: *const libc::c_char =
        b"mode_color %u %u %u\x00" as *const u8 as *const libc::c_char;
    let mut i: uint32_t = 0i32 as uint32_t;
    while i < 6i32 as libc::c_uint {
        let mut j: uint32_t = 0i32 as uint32_t;
        while j < 6i32 as libc::c_uint {
            let mut colorIndex: libc::c_int =
                (*ledStripConfig_0).modeColors[i as usize].color[j as usize]
                    as libc::c_int;
            let mut equalsDefault: bool = 0i32 != 0;
            if !defaultLedStripConfig.is_null() {
                let mut colorIndexDefault: libc::c_int =
                    (*defaultLedStripConfig).modeColors[i as
                                                            usize].color[j as
                                                                             usize]
                        as libc::c_int;
                equalsDefault = colorIndex == colorIndexDefault;
                cliDefaultPrintLinef(dumpMask, equalsDefault, format, i, j,
                                     colorIndexDefault);
            }
            cliDumpPrintLinef(dumpMask, equalsDefault, format, i, j,
                              colorIndex);
            j = j.wrapping_add(1)
        }
        i = i.wrapping_add(1)
    }
    let mut j_0: uint32_t = 0i32 as uint32_t;
    while j_0 < 11i32 as libc::c_uint {
        let colorIndex_0: libc::c_int =
            (*ledStripConfig_0).specialColors.color[j_0 as usize] as
                libc::c_int;
        let mut equalsDefault_0: bool = 0i32 != 0;
        if !defaultLedStripConfig.is_null() {
            let colorIndexDefault_0: libc::c_int =
                (*defaultLedStripConfig).specialColors.color[j_0 as usize] as
                    libc::c_int;
            equalsDefault_0 = colorIndex_0 == colorIndexDefault_0;
            cliDefaultPrintLinef(dumpMask, equalsDefault_0, format,
                                 LED_SPECIAL as libc::c_int, j_0,
                                 colorIndexDefault_0);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault_0, format,
                          LED_SPECIAL as libc::c_int, j_0, colorIndex_0);
        j_0 = j_0.wrapping_add(1)
    }
    let ledStripAuxChannel: libc::c_int =
        (*ledStripConfig_0).ledstrip_aux_channel as libc::c_int;
    let mut equalsDefault_1: bool = 0i32 != 0;
    if !defaultLedStripConfig.is_null() {
        let ledStripAuxChannelDefault: libc::c_int =
            (*defaultLedStripConfig).ledstrip_aux_channel as libc::c_int;
        equalsDefault_1 = ledStripAuxChannel == ledStripAuxChannelDefault;
        cliDefaultPrintLinef(dumpMask, equalsDefault_1, format,
                             LED_AUX_CHANNEL as libc::c_int, 0i32,
                             ledStripAuxChannelDefault);
    }
    cliDumpPrintLinef(dumpMask, equalsDefault_1, format,
                      LED_AUX_CHANNEL as libc::c_int, 0i32,
                      ledStripAuxChannel);
}
unsafe extern "C" fn cliModeColor(mut cmdline: *mut libc::c_char) {
    if isEmpty(cmdline) {
        printModeColor(DUMP_MASTER as libc::c_int as uint8_t,
                       ledStripConfig(), 0 as *const ledStripConfig_t);
    } else {
        let mut args: [libc::c_int; 3] = [0; 3];
        let mut argNo: libc::c_int = 0i32;
        let mut saveptr: *mut libc::c_char = 0 as *mut libc::c_char;
        let mut ptr: *const libc::c_char =
            strtok_r(cmdline, b" \x00" as *const u8 as *const libc::c_char,
                     &mut saveptr);
        while !ptr.is_null() && argNo < ARGS_COUNT_1 as libc::c_int {
            let fresh5 = argNo;
            argNo = argNo + 1;
            args[fresh5 as usize] = atoi(ptr);
            ptr =
                strtok_r(0 as *mut libc::c_char,
                         b" \x00" as *const u8 as *const libc::c_char,
                         &mut saveptr)
        }
        if !ptr.is_null() || argNo != ARGS_COUNT_1 as libc::c_int {
            cliShowParseError();
            return
        }
        let mut modeIdx: libc::c_int = args[MODE as libc::c_int as usize];
        let mut funIdx: libc::c_int = args[FUNCTION as libc::c_int as usize];
        let mut color: libc::c_int = args[COLOR as libc::c_int as usize];
        if !setModeColor(modeIdx as ledModeIndex_e, funIdx, color) {
            cliShowParseError();
            return
        }
        // values are validated
        cliPrintLinef(b"mode_color %u %u %u\x00" as *const u8 as
                          *const libc::c_char, modeIdx, funIdx, color);
    };
}
unsafe extern "C" fn printServo(mut dumpMask: uint8_t,
                                mut servoParams_0: *const servoParam_t,
                                mut defaultServoParams: *const servoParam_t) {
    // print out servo settings
    let mut format: *const libc::c_char =
        b"servo %u %d %d %d %d %d\x00" as *const u8 as *const libc::c_char;
    let mut i: uint32_t = 0i32 as uint32_t;
    while i < 8i32 as libc::c_uint {
        let mut servoConf: *const servoParam_t =
            &*servoParams_0.offset(i as isize) as *const servoParam_t;
        let mut equalsDefault: bool = 0i32 != 0;
        if !defaultServoParams.is_null() {
            let mut defaultServoConf: *const servoParam_t =
                &*defaultServoParams.offset(i as isize) as
                    *const servoParam_t;
            equalsDefault =
                memcmp(servoConf as *const libc::c_void,
                       defaultServoConf as *const libc::c_void,
                       ::core::mem::size_of::<servoParam_t>() as
                           libc::c_ulong) == 0;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, i,
                                 (*defaultServoConf).min as libc::c_int,
                                 (*defaultServoConf).max as libc::c_int,
                                 (*defaultServoConf).middle as libc::c_int,
                                 (*defaultServoConf).rate as libc::c_int,
                                 (*defaultServoConf).forwardFromChannel as
                                     libc::c_int);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, i,
                          (*servoConf).min as libc::c_int,
                          (*servoConf).max as libc::c_int,
                          (*servoConf).middle as libc::c_int,
                          (*servoConf).rate as libc::c_int,
                          (*servoConf).forwardFromChannel as libc::c_int);
        i = i.wrapping_add(1)
    }
    // print servo directions
    let mut i_0: uint32_t = 0i32 as uint32_t;
    while i_0 < 8i32 as libc::c_uint {
        let mut format_0: *const libc::c_char =
            b"smix reverse %d %d r\x00" as *const u8 as *const libc::c_char;
        let mut servoConf_0: *const servoParam_t =
            &*servoParams_0.offset(i_0 as isize) as *const servoParam_t;
        let mut servoConfDefault: *const servoParam_t =
            &*defaultServoParams.offset(i_0 as isize) as *const servoParam_t;
        if !defaultServoParams.is_null() {
            let mut equalsDefault_0: bool =
                (*servoConf_0).reversedSources ==
                    (*servoConfDefault).reversedSources;
            let mut channel: uint32_t = 0i32 as uint32_t;
            while channel < INPUT_SOURCE_COUNT as libc::c_int as libc::c_uint
                  {
                equalsDefault_0 =
                    !((*servoConf_0).reversedSources ^
                          (*servoConfDefault).reversedSources) &
                        (1i32 << channel) as libc::c_uint != 0;
                if (*servoConfDefault).reversedSources &
                       (1i32 << channel) as libc::c_uint != 0 {
                    cliDefaultPrintLinef(dumpMask, equalsDefault_0, format_0,
                                         i_0, channel);
                }
                if (*servoConf_0).reversedSources &
                       (1i32 << channel) as libc::c_uint != 0 {
                    cliDumpPrintLinef(dumpMask, equalsDefault_0, format_0,
                                      i_0, channel);
                }
                channel = channel.wrapping_add(1)
            }
        } else {
            let mut channel_0: uint32_t = 0i32 as uint32_t;
            while channel_0 <
                      INPUT_SOURCE_COUNT as libc::c_int as libc::c_uint {
                if (*servoConf_0).reversedSources &
                       (1i32 << channel_0) as libc::c_uint != 0 {
                    cliDumpPrintLinef(dumpMask, 1i32 != 0, format_0, i_0,
                                      channel_0);
                }
                channel_0 = channel_0.wrapping_add(1)
            }
        }
        i_0 = i_0.wrapping_add(1)
    };
}
unsafe extern "C" fn cliServo(mut cmdline: *mut libc::c_char) {
    let mut format: *const libc::c_char =
        b"servo %u %d %d %d %d %d\x00" as *const u8 as *const libc::c_char;
    let mut arguments: [int16_t; 6] = [0; 6];
    let mut servo: *mut servoParam_t = 0 as *mut servoParam_t;
    let mut i: libc::c_int = 0;
    let mut ptr: *mut libc::c_char = 0 as *mut libc::c_char;
    if isEmpty(cmdline) {
        printServo(DUMP_MASTER as libc::c_int as uint8_t, servoParams(0i32),
                   0 as *const servoParam_t);
    } else {
        let mut validArgumentCount: libc::c_int = 0i32;
        ptr = cmdline;
        // Command line is integers (possibly negative) separated by spaces, no other characters allowed.
        // If command line doesn't fit the format, don't modify the config
        while *ptr != 0 {
            if *ptr as libc::c_int == '-' as i32 ||
                   *ptr as libc::c_int >= '0' as i32 &&
                       *ptr as libc::c_int <= '9' as i32 {
                if validArgumentCount >= SERVO_ARGUMENT_COUNT as libc::c_int {
                    cliShowParseError();
                    return
                }
                let fresh6 = validArgumentCount;
                validArgumentCount = validArgumentCount + 1;
                arguments[fresh6 as usize] = atoi(ptr) as int16_t;
                loop  {
                    ptr = ptr.offset(1);
                    if !(*ptr as libc::c_int >= '0' as i32 &&
                             *ptr as libc::c_int <= '9' as i32) {
                        break ;
                    }
                }
            } else if *ptr as libc::c_int == ' ' as i32 {
                ptr = ptr.offset(1)
            } else { cliShowParseError(); return }
        }
        i = arguments[INDEX as libc::c_int as usize] as libc::c_int;
        // Check we got the right number of args and the servo index is correct (don't validate the other values)
        if validArgumentCount != SERVO_ARGUMENT_COUNT as libc::c_int ||
               i < 0i32 || i >= 8i32 {
            cliShowParseError();
            return
        }
        servo = servoParamsMutable(i);
        if (arguments[MIN_0 as libc::c_int as usize] as libc::c_int) < 750i32
               ||
               arguments[MIN_0 as libc::c_int as usize] as libc::c_int >
                   2250i32 ||
               (arguments[MAX_0 as libc::c_int as usize] as libc::c_int) <
                   750i32 ||
               arguments[MAX_0 as libc::c_int as usize] as libc::c_int >
                   2250i32 ||
               (arguments[MIDDLE as libc::c_int as usize] as libc::c_int) <
                   arguments[MIN_0 as libc::c_int as usize] as libc::c_int ||
               arguments[MIDDLE as libc::c_int as usize] as libc::c_int >
                   arguments[MAX_0 as libc::c_int as usize] as libc::c_int ||
               arguments[MIN_0 as libc::c_int as usize] as libc::c_int >
                   arguments[MAX_0 as libc::c_int as usize] as libc::c_int ||
               (arguments[MAX_0 as libc::c_int as usize] as libc::c_int) <
                   arguments[MIN_0 as libc::c_int as usize] as libc::c_int ||
               (arguments[RATE_0 as libc::c_int as usize] as libc::c_int) <
                   -100i32 ||
               arguments[RATE_0 as libc::c_int as usize] as libc::c_int >
                   100i32 ||
               arguments[FORWARD as libc::c_int as usize] as libc::c_int >=
                   18i32 {
            cliShowParseError();
            return
        }
        (*servo).min = arguments[MIN_0 as libc::c_int as usize];
        (*servo).max = arguments[MAX_0 as libc::c_int as usize];
        (*servo).middle = arguments[MIDDLE as libc::c_int as usize];
        (*servo).rate = arguments[RATE_0 as libc::c_int as usize] as int8_t;
        (*servo).forwardFromChannel =
            arguments[FORWARD as libc::c_int as usize] as int8_t;
        cliDumpPrintLinef(0i32 as uint8_t, 0i32 != 0, format, i,
                          (*servo).min as libc::c_int,
                          (*servo).max as libc::c_int,
                          (*servo).middle as libc::c_int,
                          (*servo).rate as libc::c_int,
                          (*servo).forwardFromChannel as libc::c_int);
    };
}
unsafe extern "C" fn printServoMix(mut dumpMask: uint8_t,
                                   mut customServoMixers_0:
                                       *const servoMixer_t,
                                   mut defaultCustomServoMixers:
                                       *const servoMixer_t) {
    let mut format: *const libc::c_char =
        b"smix %d %d %d %d %d %d %d %d\x00" as *const u8 as
            *const libc::c_char;
    let mut i: uint32_t = 0i32 as uint32_t;
    while i < (2i32 * 8i32) as libc::c_uint {
        let customServoMixer: servoMixer_t =
            *customServoMixers_0.offset(i as isize);
        if customServoMixer.rate as libc::c_int == 0i32 { break ; }
        let mut equalsDefault: bool = 0i32 != 0;
        if !defaultCustomServoMixers.is_null() {
            let mut customServoMixerDefault: servoMixer_t =
                *defaultCustomServoMixers.offset(i as isize);
            equalsDefault =
                memcmp(&customServoMixer as *const servoMixer_t as
                           *const libc::c_void,
                       &mut customServoMixerDefault as *mut servoMixer_t as
                           *const libc::c_void,
                       ::core::mem::size_of::<servoMixer_t>() as
                           libc::c_ulong) == 0;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, i,
                                 customServoMixerDefault.targetChannel as
                                     libc::c_int,
                                 customServoMixerDefault.inputSource as
                                     libc::c_int,
                                 customServoMixerDefault.rate as libc::c_int,
                                 customServoMixerDefault.speed as libc::c_int,
                                 customServoMixerDefault.min as libc::c_int,
                                 customServoMixerDefault.max as libc::c_int,
                                 customServoMixerDefault.box_0 as
                                     libc::c_int);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, i,
                          customServoMixer.targetChannel as libc::c_int,
                          customServoMixer.inputSource as libc::c_int,
                          customServoMixer.rate as libc::c_int,
                          customServoMixer.speed as libc::c_int,
                          customServoMixer.min as libc::c_int,
                          customServoMixer.max as libc::c_int,
                          customServoMixer.box_0 as libc::c_int);
        i = i.wrapping_add(1)
    }
    cliPrintLinefeed();
}
unsafe extern "C" fn cliServoMix(mut cmdline: *mut libc::c_char) {
    let mut args: [libc::c_int; 8] = [0; 8];
    let mut check: libc::c_int = 0i32;
    let mut len: libc::c_int = strlen(cmdline) as libc::c_int;
    if len == 0i32 {
        printServoMix(DUMP_MASTER as libc::c_int as uint8_t,
                      customServoMixers(0i32), 0 as *const servoMixer_t);
    } else if strncasecmp(cmdline,
                          b"reset\x00" as *const u8 as *const libc::c_char,
                          5i32 as libc::c_ulong) == 0i32 {
        // erase custom mixer
        memset(customServoMixers_array() as *mut libc::c_void, 0i32,
               ::core::mem::size_of::<[servoMixer_t; 16]>() as libc::c_ulong);
        let mut i: uint32_t = 0i32 as uint32_t;
        while i < 8i32 as libc::c_uint {
            (*servoParamsMutable(i as libc::c_int)).reversedSources =
                0i32 as uint32_t;
            i = i.wrapping_add(1)
        }
    } else if strncasecmp(cmdline,
                          b"load\x00" as *const u8 as *const libc::c_char,
                          4i32 as libc::c_ulong) == 0i32 {
        let mut ptr: *const libc::c_char = nextArg(cmdline);
        if !ptr.is_null() {
            len = strlen(ptr) as libc::c_int;
            let mut i_0: uint32_t = 0i32 as uint32_t;
            loop  {
                if mixerNames[i_0 as usize].is_null() {
                    cliPrintErrorLinef(b"Invalid name\x00" as *const u8 as
                                           *const libc::c_char);
                    break ;
                } else if strncasecmp(ptr, mixerNames[i_0 as usize],
                                      len as libc::c_ulong) == 0i32 {
                    servoMixerLoadMix(i_0 as libc::c_int);
                    cliPrintLinef(b"Loaded %s\x00" as *const u8 as
                                      *const libc::c_char,
                                  mixerNames[i_0 as usize]);
                    cliServoMix(b"\x00" as *const u8 as *const libc::c_char as
                                    *mut libc::c_char);
                    break ;
                } else { i_0 = i_0.wrapping_add(1) }
            }
        }
    } else if strncasecmp(cmdline,
                          b"reverse\x00" as *const u8 as *const libc::c_char,
                          7i32 as libc::c_ulong) == 0i32 {
        let mut ptr_0: *mut libc::c_char = strchr(cmdline, ' ' as i32);
        if ptr_0.is_null() {
            cliPrintf(b"s\x00" as *const u8 as *const libc::c_char);
            let mut inputSource: uint32_t = 0i32 as uint32_t;
            while inputSource <
                      INPUT_SOURCE_COUNT as libc::c_int as libc::c_uint {
                cliPrintf(b"\ti%d\x00" as *const u8 as *const libc::c_char,
                          inputSource);
                inputSource = inputSource.wrapping_add(1)
            }
            cliPrintLinefeed();
            let mut servoIndex: uint32_t = 0i32 as uint32_t;
            while servoIndex < 8i32 as libc::c_uint {
                cliPrintf(b"%d\x00" as *const u8 as *const libc::c_char,
                          servoIndex);
                let mut inputSource_0: uint32_t = 0i32 as uint32_t;
                while inputSource_0 <
                          INPUT_SOURCE_COUNT as libc::c_int as libc::c_uint {
                    cliPrintf(b"\t%s  \x00" as *const u8 as
                                  *const libc::c_char,
                              if (*servoParams(servoIndex as
                                                   libc::c_int)).reversedSources
                                     & (1i32 << inputSource_0) as libc::c_uint
                                     != 0 {
                                  b"r\x00" as *const u8 as *const libc::c_char
                              } else {
                                  b"n\x00" as *const u8 as *const libc::c_char
                              });
                    inputSource_0 = inputSource_0.wrapping_add(1)
                }
                cliPrintLinefeed();
                servoIndex = servoIndex.wrapping_add(1)
            }
            return
        }
        let mut saveptr: *mut libc::c_char = 0 as *mut libc::c_char;
        ptr_0 =
            strtok_r(ptr_0, b" \x00" as *const u8 as *const libc::c_char,
                     &mut saveptr);
        while !ptr_0.is_null() && check < ARGS_COUNT_0 as libc::c_int - 1i32 {
            let fresh7 = check;
            check = check + 1;
            args[fresh7 as usize] = atoi(ptr_0);
            ptr_0 =
                strtok_r(0 as *mut libc::c_char,
                         b" \x00" as *const u8 as *const libc::c_char,
                         &mut saveptr)
        }
        if ptr_0.is_null() || check != ARGS_COUNT_0 as libc::c_int - 1i32 {
            cliShowParseError();
            return
        }
        if args[SERVO as libc::c_int as usize] >= 0i32 &&
               args[SERVO as libc::c_int as usize] < 8i32 &&
               args[INPUT_0 as libc::c_int as usize] >= 0i32 &&
               args[INPUT_0 as libc::c_int as usize] <
                   INPUT_SOURCE_COUNT as libc::c_int &&
               (*ptr_0 as libc::c_int == 'r' as i32 ||
                    *ptr_0 as libc::c_int == 'n' as i32) {
            if *ptr_0 as libc::c_int == 'r' as i32 {
                let ref mut fresh8 =
                    (*servoParamsMutable(args[SERVO as libc::c_int as
                                                  usize])).reversedSources;
                *fresh8 |=
                    (1i32 << args[INPUT_0 as libc::c_int as usize]) as
                        libc::c_uint
            } else {
                let ref mut fresh9 =
                    (*servoParamsMutable(args[SERVO as libc::c_int as
                                                  usize])).reversedSources;
                *fresh9 &=
                    !(1i32 << args[INPUT_0 as libc::c_int as usize]) as
                        libc::c_uint
            }
        } else { cliShowParseError(); return }
        cliServoMix(b"reverse\x00" as *const u8 as *const libc::c_char as
                        *mut libc::c_char);
    } else {
        let mut saveptr_0: *mut libc::c_char = 0 as *mut libc::c_char;
        let mut ptr_1: *mut libc::c_char =
            strtok_r(cmdline, b" \x00" as *const u8 as *const libc::c_char,
                     &mut saveptr_0);
        while !ptr_1.is_null() && check < ARGS_COUNT as libc::c_int {
            let fresh10 = check;
            check = check + 1;
            args[fresh10 as usize] = atoi(ptr_1);
            ptr_1 =
                strtok_r(0 as *mut libc::c_char,
                         b" \x00" as *const u8 as *const libc::c_char,
                         &mut saveptr_0)
        }
        if !ptr_1.is_null() || check != ARGS_COUNT as libc::c_int {
            cliShowParseError();
            return
        }
        let mut i_1: int32_t = args[RULE as libc::c_int as usize];
        if i_1 >= 0i32 && i_1 < 2i32 * 8i32 &&
               args[TARGET as libc::c_int as usize] >= 0i32 &&
               args[TARGET as libc::c_int as usize] < 8i32 &&
               args[INPUT as libc::c_int as usize] >= 0i32 &&
               args[INPUT as libc::c_int as usize] <
                   INPUT_SOURCE_COUNT as libc::c_int &&
               args[RATE as libc::c_int as usize] >= -100i32 &&
               args[RATE as libc::c_int as usize] <= 100i32 &&
               args[SPEED as libc::c_int as usize] >= 0i32 &&
               args[SPEED as libc::c_int as usize] <= 255i32 &&
               args[MIN as libc::c_int as usize] >= 0i32 &&
               args[MIN as libc::c_int as usize] <= 100i32 &&
               args[MAX as libc::c_int as usize] >= 0i32 &&
               args[MAX as libc::c_int as usize] <= 100i32 &&
               args[MIN as libc::c_int as usize] <
                   args[MAX as libc::c_int as usize] &&
               args[BOX as libc::c_int as usize] >= 0i32 &&
               args[BOX as libc::c_int as usize] <= 3i32 {
            (*customServoMixersMutable(i_1)).targetChannel =
                args[TARGET as libc::c_int as usize] as uint8_t;
            (*customServoMixersMutable(i_1)).inputSource =
                args[INPUT as libc::c_int as usize] as uint8_t;
            (*customServoMixersMutable(i_1)).rate =
                args[RATE as libc::c_int as usize] as int8_t;
            (*customServoMixersMutable(i_1)).speed =
                args[SPEED as libc::c_int as usize] as uint8_t;
            (*customServoMixersMutable(i_1)).min =
                args[MIN as libc::c_int as usize] as int8_t;
            (*customServoMixersMutable(i_1)).max =
                args[MAX as libc::c_int as usize] as int8_t;
            (*customServoMixersMutable(i_1)).box_0 =
                args[BOX as libc::c_int as usize] as uint8_t;
            cliServoMix(b"\x00" as *const u8 as *const libc::c_char as
                            *mut libc::c_char);
        } else { cliShowParseError(); }
    };
}
unsafe extern "C" fn cliFlashInfo(mut cmdline: *mut libc::c_char) {
    let mut layout: *const flashGeometry_t = flashfsGetGeometry();
    cliPrintLinef(b"Flash sectors=%u, sectorSize=%u, pagesPerSector=%u, pageSize=%u, totalSize=%u, usedSize=%u\x00"
                      as *const u8 as *const libc::c_char,
                  (*layout).sectors as libc::c_int, (*layout).sectorSize,
                  (*layout).pagesPerSector as libc::c_int,
                  (*layout).pageSize as libc::c_int, (*layout).totalSize,
                  flashfsGetOffset());
}
unsafe extern "C" fn cliFlashErase(mut cmdline: *mut libc::c_char) {
    if !flashfsIsSupported() { return }
    cliPrintLine(b"Erasing,\x00" as *const u8 as *const libc::c_char);
    bufWriterFlush(cliWriter);
    flashfsEraseCompletely();
    while !flashfsIsReady() { delay(100i32 as timeMs_t); }
    beeper(BEEPER_BLACKBOX_ERASE);
    cliPrintLinefeed();
    cliPrintLine(b"Done.\x00" as *const u8 as *const libc::c_char);
}
unsafe extern "C" fn printVtx(mut dumpMask: uint8_t,
                              mut vtxConfig_0: *const vtxConfig_t,
                              mut vtxConfigDefault: *const vtxConfig_t) {
    // print out vtx channel settings
    let mut format: *const libc::c_char =
        b"vtx %u %u %u %u %u %u\x00" as *const u8 as *const libc::c_char;
    let mut equalsDefault: bool = 0i32 != 0;
    let mut i: uint32_t = 0i32 as uint32_t;
    while i < 10i32 as libc::c_uint {
        let mut cac: *const vtxChannelActivationCondition_t =
            &*(*vtxConfig_0).vtxChannelActivationConditions.as_ptr().offset(i
                                                                                as
                                                                                isize)
                as *const vtxChannelActivationCondition_t;
        if !vtxConfigDefault.is_null() {
            let mut cacDefault: *const vtxChannelActivationCondition_t =
                &*(*vtxConfigDefault).vtxChannelActivationConditions.as_ptr().offset(i
                                                                                         as
                                                                                         isize)
                    as *const vtxChannelActivationCondition_t;
            equalsDefault =
                memcmp(cac as *const libc::c_void,
                       cacDefault as *const libc::c_void,
                       ::core::mem::size_of::<vtxChannelActivationCondition_t>()
                           as libc::c_ulong) == 0;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, i,
                                 (*cacDefault).auxChannelIndex as libc::c_int,
                                 (*cacDefault).band as libc::c_int,
                                 (*cacDefault).channel as libc::c_int,
                                 900i32 +
                                     25i32 *
                                         (*cacDefault).range.startStep as
                                             libc::c_int,
                                 900i32 +
                                     25i32 *
                                         (*cacDefault).range.endStep as
                                             libc::c_int);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, i,
                          (*cac).auxChannelIndex as libc::c_int,
                          (*cac).band as libc::c_int,
                          (*cac).channel as libc::c_int,
                          900i32 +
                              25i32 * (*cac).range.startStep as libc::c_int,
                          900i32 +
                              25i32 * (*cac).range.endStep as libc::c_int);
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn cliVtx(mut cmdline: *mut libc::c_char) {
    let mut format: *const libc::c_char =
        b"vtx %u %u %u %u %u %u\x00" as *const u8 as *const libc::c_char;
    let mut i: libc::c_int = 0;
    let mut val: libc::c_int = 0i32;
    let mut ptr: *const libc::c_char = 0 as *const libc::c_char;
    if isEmpty(cmdline) {
        printVtx(DUMP_MASTER as libc::c_int as uint8_t, vtxConfig(),
                 0 as *const vtxConfig_t);
    } else {
        ptr = cmdline;
        let fresh11 = ptr;
        ptr = ptr.offset(1);
        i = atoi(fresh11);
        if i < 10i32 {
            let mut cac: *mut vtxChannelActivationCondition_t =
                &mut *(*(vtxConfigMutable as
                             unsafe extern "C" fn()
                                 ->
                                     *mut vtxConfig_t)()).vtxChannelActivationConditions.as_mut_ptr().offset(i
                                                                                                                 as
                                                                                                                 isize)
                    as *mut vtxChannelActivationCondition_t;
            let mut validArgumentCount: uint8_t = 0i32 as uint8_t;
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                val = atoi(ptr);
                if val >= 0i32 && val < 18i32 - 4i32 {
                    (*cac).auxChannelIndex = val as uint8_t;
                    validArgumentCount = validArgumentCount.wrapping_add(1)
                }
            }
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                val = atoi(ptr);
                // FIXME Use VTX API to get min/max
                if val >= 1i32 && val <= 5i32 {
                    (*cac).band = val as uint8_t;
                    validArgumentCount = validArgumentCount.wrapping_add(1)
                }
            }
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                val = atoi(ptr);
                // FIXME Use VTX API to get min/max
                if val >= 1i32 && val <= 8i32 {
                    (*cac).channel = val as uint8_t;
                    validArgumentCount = validArgumentCount.wrapping_add(1)
                }
            }
            ptr =
                processChannelRangeArgs(ptr, &mut (*cac).range,
                                        &mut validArgumentCount);
            if validArgumentCount as libc::c_int != 5i32 {
                memset(cac as *mut libc::c_void, 0i32,
                       ::core::mem::size_of::<vtxChannelActivationCondition_t>()
                           as libc::c_ulong);
                cliShowParseError();
            } else {
                cliDumpPrintLinef(0i32 as uint8_t, 0i32 != 0, format, i,
                                  (*cac).auxChannelIndex as libc::c_int,
                                  (*cac).band as libc::c_int,
                                  (*cac).channel as libc::c_int,
                                  900i32 +
                                      25i32 *
                                          (*cac).range.startStep as
                                              libc::c_int,
                                  900i32 +
                                      25i32 *
                                          (*cac).range.endStep as
                                              libc::c_int);
            }
        } else {
            cliShowArgumentRangeError(b"index\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char, 0i32,
                                      10i32 - 1i32);
        }
    };
}
// VTX_CONTROL
unsafe extern "C" fn printName(mut dumpMask: uint8_t,
                               mut pilotConfig_0: *const pilotConfig_t) {
    let equalsDefault: bool =
        strlen((*pilotConfig_0).name.as_ptr()) == 0i32 as libc::c_ulong;
    cliDumpPrintLinef(dumpMask, equalsDefault,
                      b"name %s\x00" as *const u8 as *const libc::c_char,
                      if equalsDefault as libc::c_int != 0 {
                          emptyName
                      } else { (*pilotConfig_0).name.as_ptr() });
}
unsafe extern "C" fn cliName(mut cmdline: *mut libc::c_char) {
    let len: libc::c_uint = strlen(cmdline) as libc::c_uint;
    if len > 0i32 as libc::c_uint {
        memset((*pilotConfigMutable()).name.as_mut_ptr() as *mut libc::c_void,
               0i32,
               (::core::mem::size_of::<[libc::c_char; 17]>() as
                    libc::c_ulong).wrapping_div(::core::mem::size_of::<libc::c_char>()
                                                    as libc::c_ulong));
        if strncmp(cmdline, emptyName, len as libc::c_ulong) != 0 {
            strncpy((*pilotConfigMutable()).name.as_mut_ptr(), cmdline,
                    ({
                         let _a: libc::c_uint = len;
                         let mut _b: libc::c_uint = 16u32;
                         if _a < _b { _a } else { _b }
                     }) as libc::c_ulong);
        }
    }
    printName(DUMP_MASTER as libc::c_int as uint8_t, pilotConfig());
}
// USE_BOARD_INFO
unsafe extern "C" fn cliMcuId(mut cmdline: *mut libc::c_char) {
    cliPrintLinef(b"mcu_id %08x%08x%08x\x00" as *const u8 as
                      *const libc::c_char, *(0x1ffff7aci32 as *mut uint32_t),
                  *(0x1ffff7b0i32 as *mut uint32_t),
                  *(0x1ffff7b4i32 as *mut uint32_t));
}
unsafe extern "C" fn printFeature(mut dumpMask: uint8_t,
                                  mut featureConfig_0: *const featureConfig_t,
                                  mut featureConfigDefault:
                                      *const featureConfig_t) {
    let mask: uint32_t = (*featureConfig_0).enabledFeatures;
    let defaultMask: uint32_t = (*featureConfigDefault).enabledFeatures;
    let mut i: uint32_t = 0i32 as uint32_t;
    while !featureNames[i as usize].is_null() {
        // disabled features first
        if strcmp(featureNames[i as usize], emptyString) != 0i32 {
            //Skip unused
            let mut format: *const libc::c_char =
                b"feature -%s\x00" as *const u8 as *const libc::c_char;
            cliDefaultPrintLinef(dumpMask,
                                 (defaultMask | !mask) &
                                     (1i32 << i) as libc::c_uint != 0, format,
                                 featureNames[i as usize]);
            cliDumpPrintLinef(dumpMask,
                              (!defaultMask | mask) &
                                  (1i32 << i) as libc::c_uint != 0, format,
                              featureNames[i as usize]);
        }
        i = i.wrapping_add(1)
    }
    let mut i_0: uint32_t = 0i32 as uint32_t;
    while !featureNames[i_0 as usize].is_null() {
        // enabled features
        if strcmp(featureNames[i_0 as usize], emptyString) != 0i32 {
            //Skip unused
            let mut format_0: *const libc::c_char =
                b"feature %s\x00" as *const u8 as *const libc::c_char;
            if defaultMask & (1i32 << i_0) as libc::c_uint != 0 {
                cliDefaultPrintLinef(dumpMask,
                                     (!defaultMask | mask) &
                                         (1i32 << i_0) as libc::c_uint != 0,
                                     format_0, featureNames[i_0 as usize]);
            }
            if mask & (1i32 << i_0) as libc::c_uint != 0 {
                cliDumpPrintLinef(dumpMask,
                                  (defaultMask | !mask) &
                                      (1i32 << i_0) as libc::c_uint != 0,
                                  format_0, featureNames[i_0 as usize]);
            }
        }
        i_0 = i_0.wrapping_add(1)
    };
}
unsafe extern "C" fn cliFeature(mut cmdline: *mut libc::c_char) {
    let mut len: uint32_t = strlen(cmdline) as uint32_t;
    let mut mask: uint32_t = featureMask();
    if len == 0i32 as libc::c_uint {
        cliPrint(b"Enabled: \x00" as *const u8 as *const libc::c_char);
        let mut i: uint32_t = 0i32 as uint32_t;
        while !featureNames[i as usize].is_null() {
            if mask & (1i32 << i) as libc::c_uint != 0 {
                cliPrintf(b"%s \x00" as *const u8 as *const libc::c_char,
                          featureNames[i as usize]);
            }
            i = i.wrapping_add(1)
        }
        cliPrintLinefeed();
    } else if strncasecmp(cmdline,
                          b"list\x00" as *const u8 as *const libc::c_char,
                          len as libc::c_ulong) == 0i32 {
        cliPrint(b"Available:\x00" as *const u8 as *const libc::c_char);
        let mut i_0: uint32_t = 0i32 as uint32_t;
        while !featureNames[i_0 as usize].is_null() {
            if strcmp(featureNames[i_0 as usize], emptyString) != 0i32 {
                //Skip unused
                cliPrintf(b" %s\x00" as *const u8 as *const libc::c_char,
                          featureNames[i_0 as usize]);
            }
            i_0 = i_0.wrapping_add(1)
        }
        cliPrintLinefeed();
        return
    } else {
        let mut remove: bool = 0i32 != 0;
        if *cmdline.offset(0) as libc::c_int == '-' as i32 {
            // remove feature
            remove = 1i32 != 0; // skip over -
            cmdline = cmdline.offset(1); // this is for beeper OFF condition
            len = len.wrapping_sub(1)
        }
        let mut i_1: uint32_t = 0i32 as uint32_t;
        loop  {
            if featureNames[i_1 as usize].is_null() {
                cliPrintErrorLinef(b"Invalid name\x00" as *const u8 as
                                       *const libc::c_char);
                break ;
            } else if strncasecmp(cmdline, featureNames[i_1 as usize],
                                  len as libc::c_ulong) == 0i32 {
                mask = (1i32 << i_1) as uint32_t;
                if mask & FEATURE_GPS as libc::c_int as libc::c_uint != 0 {
                    cliPrintLine(b"unavailable\x00" as *const u8 as
                                     *const libc::c_char);
                    break ;
                } else {
                    if remove {
                        featureClear(mask);
                        cliPrint(b"Disabled\x00" as *const u8 as
                                     *const libc::c_char);
                    } else {
                        featureSet(mask);
                        cliPrint(b"Enabled\x00" as *const u8 as
                                     *const libc::c_char);
                    }
                    cliPrintLinef(b" %s\x00" as *const u8 as
                                      *const libc::c_char,
                                  featureNames[i_1 as usize]);
                    break ;
                }
            } else { i_1 = i_1.wrapping_add(1) }
        }
    };
}
unsafe extern "C" fn printBeeper(mut dumpMask: uint8_t, offFlags: uint32_t,
                                 offFlagsDefault: uint32_t,
                                 mut name: *const libc::c_char) {
    let beeperCount: uint8_t = beeperTableEntryCount() as uint8_t;
    let mut i: int32_t = 0i32;
    while i < beeperCount as libc::c_int - 1i32 {
        let mut formatOff: *const libc::c_char =
            b"%s -%s\x00" as *const u8 as *const libc::c_char;
        let mut formatOn: *const libc::c_char =
            b"%s %s\x00" as *const u8 as *const libc::c_char;
        let beeperModeMask: uint32_t = beeperModeMaskForTableIndex(i);
        cliDefaultPrintLinef(dumpMask,
                             !(offFlags ^ offFlagsDefault) & beeperModeMask !=
                                 0,
                             if offFlags & beeperModeMask != 0 {
                                 formatOn
                             } else { formatOff }, name,
                             beeperNameForTableIndex(i));
        cliDumpPrintLinef(dumpMask,
                          !(offFlags ^ offFlagsDefault) & beeperModeMask != 0,
                          if offFlags & beeperModeMask != 0 {
                              formatOff
                          } else { formatOn }, name,
                          beeperNameForTableIndex(i));
        i += 1
    };
}
unsafe extern "C" fn processBeeperCommand(mut cmdline: *mut libc::c_char,
                                          mut offFlags: *mut uint32_t,
                                          allowedFlags: uint32_t) {
    let mut len: uint32_t = strlen(cmdline) as uint32_t;
    let mut beeperCount: uint8_t = beeperTableEntryCount() as uint8_t;
    if len == 0i32 as libc::c_uint {
        cliPrintf(b"Disabled:\x00" as *const u8 as *const libc::c_char);
        let mut i: int32_t = 0i32;
        loop  {
            if i == beeperCount as libc::c_int - 1i32 {
                if *offFlags == 0i32 as libc::c_uint {
                    cliPrint(b"  none\x00" as *const u8 as
                                 *const libc::c_char);
                }
                break ;
            } else {
                if beeperModeMaskForTableIndex(i) & *offFlags != 0 {
                    cliPrintf(b"  %s\x00" as *const u8 as *const libc::c_char,
                              beeperNameForTableIndex(i));
                }
                i += 1
            }
        }
        cliPrintLinefeed();
    } else if strncasecmp(cmdline,
                          b"list\x00" as *const u8 as *const libc::c_char,
                          len as libc::c_ulong) == 0i32 {
        cliPrint(b"Available:\x00" as *const u8 as *const libc::c_char);
        let mut i_0: uint32_t = 0i32 as uint32_t;
        while i_0 < beeperCount as libc::c_uint {
            if beeperModeMaskForTableIndex(i_0 as libc::c_int) & allowedFlags
                   != 0 {
                cliPrintf(b" %s\x00" as *const u8 as *const libc::c_char,
                          beeperNameForTableIndex(i_0 as libc::c_int));
            }
            i_0 = i_0.wrapping_add(1)
        }
        cliPrintLinefeed();
    } else {
        let mut remove: bool = 0i32 != 0;
        if *cmdline.offset(0) as libc::c_int == '-' as i32 {
            remove = 1i32 != 0;
            cmdline = cmdline.offset(1);
            len = len.wrapping_sub(1)
        }
        let mut i_1: uint32_t = 0i32 as uint32_t;
        loop  {
            if i_1 == beeperCount as libc::c_uint {
                cliPrintErrorLinef(b"Invalid name\x00" as *const u8 as
                                       *const libc::c_char);
                break ;
            } else if strncasecmp(cmdline,
                                  beeperNameForTableIndex(i_1 as libc::c_int),
                                  len as libc::c_ulong) == 0i32 &&
                          beeperModeMaskForTableIndex(i_1 as libc::c_int) &
                              (allowedFlags |
                                   (1i32 << BEEPER_ALL as libc::c_int - 1i32)
                                       as libc::c_uint) != 0 {
                if remove {
                    // beeper off
                    if i_1 ==
                           (BEEPER_ALL as libc::c_int - 1i32) as libc::c_uint
                       {
                        *offFlags = allowedFlags
                    } else {
                        *offFlags |=
                            beeperModeMaskForTableIndex(i_1 as libc::c_int)
                    }
                    cliPrint(b"Disabled\x00" as *const u8 as
                                 *const libc::c_char);
                } else {
                    // beeper on
                    if i_1 ==
                           (BEEPER_ALL as libc::c_int - 1i32) as libc::c_uint
                       {
                        *offFlags = 0i32 as uint32_t
                    } else {
                        *offFlags &=
                            !beeperModeMaskForTableIndex(i_1 as libc::c_int)
                    }
                    cliPrint(b"Enabled\x00" as *const u8 as
                                 *const libc::c_char);
                }
                cliPrintLinef(b" %s\x00" as *const u8 as *const libc::c_char,
                              beeperNameForTableIndex(i_1 as libc::c_int));
                break ;
            } else { i_1 = i_1.wrapping_add(1) }
        }
    };
}
unsafe extern "C" fn cliBeacon(mut cmdline: *mut libc::c_char) {
    processBeeperCommand(cmdline,
                         &mut (*(beeperConfigMutable as
                                     unsafe extern "C" fn()
                                         ->
                                             *mut beeperConfig_t)()).dshotBeaconOffFlags,
                         (1i32 << BEEPER_RX_LOST as libc::c_int - 1i32 |
                              1i32 << BEEPER_RX_SET as libc::c_int - 1i32) as
                             uint32_t);
}
unsafe extern "C" fn cliBeeper(mut cmdline: *mut libc::c_char) {
    processBeeperCommand(cmdline,
                         &mut (*(beeperConfigMutable as
                                     unsafe extern "C" fn()
                                         ->
                                             *mut beeperConfig_t)()).beeper_off_flags,
                         (1i32 << BEEPER_GYRO_CALIBRATED as libc::c_int - 1i32
                              | 1i32 << BEEPER_RX_LOST as libc::c_int - 1i32 |
                              1i32 <<
                                  BEEPER_RX_LOST_LANDING as libc::c_int - 1i32
                              | 1i32 << BEEPER_DISARMING as libc::c_int - 1i32
                              | 1i32 << BEEPER_ARMING as libc::c_int - 1i32 |
                              1i32 <<
                                  BEEPER_ARMING_GPS_FIX as libc::c_int - 1i32
                              |
                              1i32 <<
                                  BEEPER_BAT_CRIT_LOW as libc::c_int - 1i32 |
                              1i32 << BEEPER_BAT_LOW as libc::c_int - 1i32 |
                              1i32 << BEEPER_GPS_STATUS as libc::c_int - 1i32
                              | 1i32 << BEEPER_RX_SET as libc::c_int - 1i32 |
                              1i32 <<
                                  BEEPER_ACC_CALIBRATION as libc::c_int - 1i32
                              |
                              1i32 <<
                                  BEEPER_ACC_CALIBRATION_FAIL as libc::c_int -
                                      1i32 |
                              1i32 << BEEPER_READY_BEEP as libc::c_int - 1i32
                              |
                              1i32 << BEEPER_MULTI_BEEPS as libc::c_int - 1i32
                              |
                              1i32 <<
                                  BEEPER_DISARM_REPEAT as libc::c_int - 1i32 |
                              1i32 << BEEPER_ARMED as libc::c_int - 1i32 |
                              1i32 << BEEPER_SYSTEM_INIT as libc::c_int - 1i32
                              | 1i32 << BEEPER_USB as libc::c_int - 1i32 |
                              1i32 <<
                                  BEEPER_BLACKBOX_ERASE as libc::c_int - 1i32
                              |
                              1i32 <<
                                  BEEPER_CRASH_FLIP_MODE as libc::c_int - 1i32
                              |
                              1i32 <<
                                  BEEPER_CAM_CONNECTION_OPEN as libc::c_int -
                                      1i32 |
                              1i32 <<
                                  BEEPER_CAM_CONNECTION_CLOSE as libc::c_int -
                                      1i32 |
                              1i32 <<
                                  BEEPER_RC_SMOOTHING_INIT_FAIL as libc::c_int
                                      - 1i32) as uint32_t);
}
unsafe extern "C" fn printMap(mut dumpMask: uint8_t,
                              mut rxConfig_0: *const rxConfig_t,
                              mut defaultRxConfig: *const rxConfig_t) {
    let mut equalsDefault: bool = 1i32 != 0;
    let mut buf: [libc::c_char; 9] = [0; 9];
    let mut bufDefault: [libc::c_char; 9] = [0; 9];
    let mut i: uint32_t = 0;
    i = 0i32 as uint32_t;
    while i < 8i32 as libc::c_uint {
        buf[(*rxConfig_0).rcmap[i as usize] as usize] =
            *rcChannelLetters.as_ptr().offset(i as isize);
        if !defaultRxConfig.is_null() {
            bufDefault[(*defaultRxConfig).rcmap[i as usize] as usize] =
                *rcChannelLetters.as_ptr().offset(i as isize);
            equalsDefault =
                equalsDefault as libc::c_int != 0 &&
                    (*rxConfig_0).rcmap[i as usize] as libc::c_int ==
                        (*defaultRxConfig).rcmap[i as usize] as libc::c_int
        }
        i = i.wrapping_add(1)
    }
    buf[i as usize] = '\u{0}' as i32 as libc::c_char;
    let mut formatMap: *const libc::c_char =
        b"map %s\x00" as *const u8 as *const libc::c_char;
    cliDefaultPrintLinef(dumpMask, equalsDefault, formatMap,
                         bufDefault.as_mut_ptr());
    cliDumpPrintLinef(dumpMask, equalsDefault, formatMap, buf.as_mut_ptr());
}
unsafe extern "C" fn cliMap(mut cmdline: *mut libc::c_char) {
    let mut i: uint32_t = 0;
    let mut buf: [libc::c_char; 9] = [0; 9];
    let mut len: uint32_t = strlen(cmdline) as uint32_t;
    if len == 8i32 as libc::c_uint {
        i = 0i32 as uint32_t;
        while i < 8i32 as libc::c_uint {
            buf[i as usize] =
                toupper(*cmdline.offset(i as isize) as libc::c_uchar as
                            libc::c_int) as libc::c_char;
            i = i.wrapping_add(1)
        }
        buf[i as usize] = '\u{0}' as i32 as libc::c_char;
        i = 0i32 as uint32_t;
        while i < 8i32 as libc::c_uint {
            buf[i as usize] =
                toupper(*cmdline.offset(i as isize) as libc::c_uchar as
                            libc::c_int) as libc::c_char;
            if !strchr(rcChannelLetters.as_ptr(),
                       buf[i as usize] as libc::c_int).is_null() &&
                   strchr(buf.as_mut_ptr().offset(i as isize).offset(1),
                          buf[i as usize] as libc::c_int).is_null() {
                i = i.wrapping_add(1)
            } else { cliShowParseError(); return }
        }
        parseRcChannels(buf.as_mut_ptr(), rxConfigMutable());
    } else if len > 0i32 as libc::c_uint { cliShowParseError(); return }
    i = 0i32 as uint32_t;
    while i < 8i32 as libc::c_uint {
        buf[(*rxConfig()).rcmap[i as usize] as usize] =
            *rcChannelLetters.as_ptr().offset(i as isize);
        i = i.wrapping_add(1)
    }
    buf[i as usize] = '\u{0}' as i32 as libc::c_char;
    cliPrintLinef(b"map %s\x00" as *const u8 as *const libc::c_char,
                  buf.as_mut_ptr());
}
unsafe extern "C" fn skipSpace(mut buffer: *mut libc::c_char)
 -> *mut libc::c_char {
    while *buffer as libc::c_int == ' ' as i32 { buffer = buffer.offset(1) }
    return buffer;
}
unsafe extern "C" fn checkCommand(mut cmdLine: *mut libc::c_char,
                                  mut command: *const libc::c_char)
 -> *mut libc::c_char {
    if strncasecmp(cmdLine, command, strlen(command)) == 0 &&
           (isspace(*cmdLine.offset(strlen(command) as isize) as libc::c_uint
                        as libc::c_int) != 0 ||
                *cmdLine.offset(strlen(command) as isize) as libc::c_int ==
                    0i32) {
        return skipSpace(cmdLine.offset(strlen(command) as isize).offset(1))
    } else { return 0 as *mut libc::c_char };
}
unsafe extern "C" fn cliRebootEx(mut bootLoader: bool) {
    cliPrint(b"\r\nRebooting\x00" as *const u8 as *const libc::c_char);
    bufWriterFlush(cliWriter);
    waitForSerialPortToFinishTransmitting(cliPort);
    stopPwmAllMotors();
    if bootLoader { systemResetToBootloader(); return }
    systemReset();
}
unsafe extern "C" fn cliReboot() { cliRebootEx(0i32 != 0); }
unsafe extern "C" fn cliBootloader(mut cmdLine: *mut libc::c_char) {
    cliRebootEx(1i32 != 0);
}
unsafe extern "C" fn cliExit(mut cmdline: *mut libc::c_char) {
    bufWriterFlush(cliWriter);
    *cliBuffer.as_mut_ptr() = '\u{0}' as i32 as libc::c_char;
    bufferIndex = 0i32 as uint32_t;
    cliMode = 0i32 as uint8_t;
    // incase a motor was left running during motortest, clear it here
    mixerResetDisarmedMotors();
    cliReboot();
    cliWriter = 0 as *mut bufWriter_t;
}
unsafe extern "C" fn cliPrintGyroRegisters(mut whichSensor: uint8_t) {
    cliPrintLinef(b"# WHO_AM_I    0x%X\x00" as *const u8 as
                      *const libc::c_char,
                  gyroReadRegister(whichSensor, 0x75i32 as uint8_t) as
                      libc::c_int);
    cliPrintLinef(b"# CONFIG      0x%X\x00" as *const u8 as
                      *const libc::c_char,
                  gyroReadRegister(whichSensor, 0x1ai32 as uint8_t) as
                      libc::c_int);
    cliPrintLinef(b"# GYRO_CONFIG 0x%X\x00" as *const u8 as
                      *const libc::c_char,
                  gyroReadRegister(whichSensor, 0x1bi32 as uint8_t) as
                      libc::c_int);
}
unsafe extern "C" fn cliDumpGyroRegisters(mut cmdline: *mut libc::c_char) {
    cliPrintGyroRegisters(0i32 as uint8_t);
}
unsafe extern "C" fn parseOutputIndex(mut pch: *mut libc::c_char,
                                      mut allowAllEscs: bool) -> libc::c_int {
    let mut outputIndex: libc::c_int = atoi(pch);
    if outputIndex >= 0i32 && outputIndex < getMotorCount() as libc::c_int {
        cliPrintLinef(b"Using output %d.\x00" as *const u8 as
                          *const libc::c_char, outputIndex);
    } else if allowAllEscs as libc::c_int != 0 && outputIndex == 255i32 {
        cliPrintLinef(b"Using all outputs.\x00" as *const u8 as
                          *const libc::c_char);
    } else {
        cliPrintErrorLinef(b"Invalid output number. Range: 0  %d.\x00" as
                               *const u8 as *const libc::c_char,
                           getMotorCount() as libc::c_int - 1i32);
        return -1i32
    }
    return outputIndex;
}
// USE_ESC_SENSOR && USE_ESC_SENSOR_INFO
unsafe extern "C" fn cliDshotProg(mut cmdline: *mut libc::c_char) {
    if isEmpty(cmdline) as libc::c_int != 0 ||
           ((*motorConfig()).dev.motorPwmProtocol as libc::c_int) <
               PWM_TYPE_DSHOT150 as libc::c_int {
        cliShowParseError();
        return
    }
    let mut saveptr: *mut libc::c_char = 0 as *mut libc::c_char;
    let mut pch: *mut libc::c_char =
        strtok_r(cmdline, b" \x00" as *const u8 as *const libc::c_char,
                 &mut saveptr);
    let mut pos: libc::c_int = 0i32;
    let mut escIndex: libc::c_int = 0i32;
    let mut firstCommand: bool = 1i32 != 0;
    while !pch.is_null() {
        match pos {
            0 => {
                escIndex = parseOutputIndex(pch, 1i32 != 0);
                if escIndex == -1i32 { return }
            }
            _ => {
                let mut command: libc::c_int = atoi(pch);
                if command >= 0i32 && command < 48i32 {
                    if firstCommand {
                        pwmDisableMotors();
                        if command == DSHOT_CMD_ESC_INFO as libc::c_int {
                            delay(5i32 as timeMs_t);
                            // Wait for potential ESC telemetry transmission to finish
                        } else { delay(1i32 as timeMs_t); }
                        firstCommand = 0i32 != 0
                    }
                    if command != DSHOT_CMD_ESC_INFO as libc::c_int {
                        pwmWriteDshotCommand(escIndex as uint8_t,
                                             getMotorCount(),
                                             command as uint8_t, 1i32 != 0);
                    } else {
                        cliPrintLine(b"Not supported.\x00" as *const u8 as
                                         *const libc::c_char);
                    }
                    cliPrintLinef(b"Command Sent: %d\x00" as *const u8 as
                                      *const libc::c_char, command);
                } else {
                    cliPrintErrorLinef(b"Invalid command. Range: 1 - %d.\x00"
                                           as *const u8 as
                                           *const libc::c_char, 48i32 - 1i32);
                }
            }
        }
        pos += 1;
        pch =
            strtok_r(0 as *mut libc::c_char,
                     b" \x00" as *const u8 as *const libc::c_char,
                     &mut saveptr)
    }
    pwmEnableMotors();
}
// USE_DSHOT
unsafe extern "C" fn cliEscPassthrough(mut cmdline: *mut libc::c_char) {
    if isEmpty(cmdline) { cliShowParseError(); return }
    let mut saveptr: *mut libc::c_char = 0 as *mut libc::c_char;
    let mut pch: *mut libc::c_char =
        strtok_r(cmdline, b" \x00" as *const u8 as *const libc::c_char,
                 &mut saveptr);
    let mut pos: libc::c_int = 0i32;
    let mut mode: uint8_t = 0i32 as uint8_t;
    let mut escIndex: libc::c_int = 0i32;
    while !pch.is_null() {
        match pos {
            0 => {
                if strncasecmp(pch,
                               b"sk\x00" as *const u8 as *const libc::c_char,
                               strlen(pch)) == 0i32 {
                    mode = PROTOCOL_SIMONK as libc::c_int as uint8_t
                } else if strncasecmp(pch,
                                      b"bl\x00" as *const u8 as
                                          *const libc::c_char, strlen(pch)) ==
                              0i32 {
                    mode = PROTOCOL_BLHELI as libc::c_int as uint8_t
                } else if strncasecmp(pch,
                                      b"ki\x00" as *const u8 as
                                          *const libc::c_char, strlen(pch)) ==
                              0i32 {
                    mode = PROTOCOL_KISS as libc::c_int as uint8_t
                } else if strncasecmp(pch,
                                      b"cc\x00" as *const u8 as
                                          *const libc::c_char, strlen(pch)) ==
                              0i32 {
                    mode = PROTOCOL_KISSALL as libc::c_int as uint8_t
                } else { cliShowParseError(); return }
            }
            1 => {
                escIndex =
                    parseOutputIndex(pch,
                                     mode as libc::c_int ==
                                         PROTOCOL_KISS as libc::c_int);
                if escIndex == -1i32 { return }
            }
            _ => { cliShowParseError(); return }
        }
        pos += 1;
        pch =
            strtok_r(0 as *mut libc::c_char,
                     b" \x00" as *const u8 as *const libc::c_char,
                     &mut saveptr)
    }
    escEnablePassthrough(cliPort, escIndex as uint16_t, mode);
}
unsafe extern "C" fn cliMixer(mut cmdline: *mut libc::c_char) {
    let mut len: libc::c_int = 0;
    len = strlen(cmdline) as libc::c_int;
    if len == 0i32 {
        cliPrintLinef(b"Mixer: %s\x00" as *const u8 as *const libc::c_char,
                      mixerNames[((*mixerConfig()).mixerMode as libc::c_int -
                                      1i32) as usize]);
        return
    } else {
        if strncasecmp(cmdline,
                       b"list\x00" as *const u8 as *const libc::c_char,
                       len as libc::c_ulong) == 0i32 {
            cliPrint(b"Available:\x00" as *const u8 as *const libc::c_char);
            let mut i: uint32_t = 0i32 as uint32_t;
            while !mixerNames[i as usize].is_null() {
                cliPrintf(b" %s\x00" as *const u8 as *const libc::c_char,
                          mixerNames[i as usize]);
                i = i.wrapping_add(1)
            }
            cliPrintLinefeed();
            return
        }
    }
    let mut i_0: uint32_t = 0i32 as uint32_t;
    loop  {
        if mixerNames[i_0 as usize].is_null() {
            cliPrintErrorLinef(b"Invalid name\x00" as *const u8 as
                                   *const libc::c_char);
            return
        }
        if strncasecmp(cmdline, mixerNames[i_0 as usize],
                       len as libc::c_ulong) == 0i32 {
            (*mixerConfigMutable()).mixerMode =
                i_0.wrapping_add(1i32 as libc::c_uint) as uint8_t;
            break ;
        } else { i_0 = i_0.wrapping_add(1) }
    }
    cliMixer(b"\x00" as *const u8 as *const libc::c_char as
                 *mut libc::c_char);
}
unsafe extern "C" fn cliMotor(mut cmdline: *mut libc::c_char) {
    if isEmpty(cmdline) { cliShowParseError(); return }
    let mut motorIndex: libc::c_int = 0i32;
    let mut motorValue: libc::c_int = 0i32;
    let mut saveptr: *mut libc::c_char = 0 as *mut libc::c_char;
    let mut pch: *mut libc::c_char =
        strtok_r(cmdline, b" \x00" as *const u8 as *const libc::c_char,
                 &mut saveptr);
    let mut index: libc::c_int = 0i32;
    while !pch.is_null() {
        match index {
            0 => {
                motorIndex = parseOutputIndex(pch, 1i32 != 0);
                if motorIndex == -1i32 { return }
            }
            1 => { motorValue = atoi(pch) }
            _ => { }
        }
        index += 1;
        pch =
            strtok_r(0 as *mut libc::c_char,
                     b" \x00" as *const u8 as *const libc::c_char,
                     &mut saveptr)
    }
    if index == 2i32 {
        if motorValue < 1000i32 || motorValue > 2000i32 {
            cliShowArgumentRangeError(b"value\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char, 1000i32,
                                      2000i32);
        } else {
            let mut motorOutputValue: uint32_t =
                convertExternalToMotor(motorValue as uint16_t) as uint32_t;
            if motorIndex != 255i32 {
                motor_disarmed[motorIndex as usize] =
                    motorOutputValue as libc::c_float;
                cliPrintLinef(b"motor %d: %d\x00" as *const u8 as
                                  *const libc::c_char, motorIndex,
                              motorOutputValue);
            } else {
                let mut i: libc::c_int = 0i32;
                while i < getMotorCount() as libc::c_int {
                    motor_disarmed[i as usize] =
                        motorOutputValue as libc::c_float;
                    i += 1
                }
                cliPrintLinef(b"all motors: %d\x00" as *const u8 as
                                  *const libc::c_char, motorOutputValue);
            }
        }
    } else { cliShowParseError(); };
}
unsafe extern "C" fn cliProfile(mut cmdline: *mut libc::c_char) {
    if isEmpty(cmdline) {
        cliPrintLinef(b"profile %d\x00" as *const u8 as *const libc::c_char,
                      getPidProfileIndexToUse() as libc::c_int);
        return
    } else {
        let i: libc::c_int = atoi(cmdline);
        if i >= 0i32 && i < 3i32 {
            changePidProfile(i as uint8_t);
            cliProfile(b"\x00" as *const u8 as *const libc::c_char as
                           *mut libc::c_char);
        }
    };
}
unsafe extern "C" fn cliRateProfile(mut cmdline: *mut libc::c_char) {
    if isEmpty(cmdline) {
        cliPrintLinef(b"rateprofile %d\x00" as *const u8 as
                          *const libc::c_char,
                      getRateProfileIndexToUse() as libc::c_int);
        return
    } else {
        let i: libc::c_int = atoi(cmdline);
        if i >= 0i32 && i < 6i32 {
            changeControlRateProfile(i as uint8_t);
            cliRateProfile(b"\x00" as *const u8 as *const libc::c_char as
                               *mut libc::c_char);
        }
    };
}
unsafe extern "C" fn cliDumpPidProfile(mut pidProfileIndex: uint8_t,
                                       mut dumpMask: uint8_t) {
    if pidProfileIndex as libc::c_int >= 3i32 {
        // Faulty values
        return
    }
    pidProfileIndexToUse = pidProfileIndex as int8_t;
    cliProfile(b"\x00" as *const u8 as *const libc::c_char as
                   *mut libc::c_char);
    cliPrintLinefeed();
    dumpAllValues(PROFILE_VALUE as libc::c_int as uint16_t, dumpMask);
    pidProfileIndexToUse = -1i32 as int8_t;
}
unsafe extern "C" fn cliDumpRateProfile(mut rateProfileIndex: uint8_t,
                                        mut dumpMask: uint8_t) {
    if rateProfileIndex as libc::c_int >= 6i32 {
        // Faulty values
        return
    }
    rateProfileIndexToUse = rateProfileIndex as int8_t;
    cliRateProfile(b"\x00" as *const u8 as *const libc::c_char as
                       *mut libc::c_char);
    cliPrintLinefeed();
    dumpAllValues(PROFILE_RATE_VALUE as libc::c_int as uint16_t, dumpMask);
    rateProfileIndexToUse = -1i32 as int8_t;
}
unsafe extern "C" fn cliSave(mut cmdline: *mut libc::c_char) {
    // USE_BOARD_INFO
    writeEEPROM(); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
    cliReboot();
}
unsafe extern "C" fn cliDefaults(mut cmdline: *mut libc::c_char) {
    let mut saveConfigs: bool = false;
    if isEmpty(cmdline) {
        saveConfigs = 1i32 != 0
    } else if strncasecmp(cmdline,
                          b"nosave\x00" as *const u8 as *const libc::c_char,
                          6i32 as libc::c_ulong) == 0i32 {
        saveConfigs = 0i32 != 0
    } else { return }
    resetConfigs();
    if saveConfigs { cliSave(0 as *mut libc::c_char); };
}
#[no_mangle]
pub unsafe extern "C" fn cliPrintVarDefault(mut value: *const clivalue_t) {
    let mut pg: *const pgRegistry_t = pgFind((*value).pgn);
    if !pg.is_null() {
        let mut defaultFormat: *const libc::c_char =
            b"Default value: \x00" as *const u8 as *const libc::c_char;
        let valueOffset: libc::c_int = getValueOffset(value) as libc::c_int;
        let equalsDefault: bool =
            valuePtrEqualsDefault(value,
                                  (*pg).copy.offset(valueOffset as isize) as
                                      *const libc::c_void,
                                  (*pg).address.offset(valueOffset as isize)
                                      as *const libc::c_void);
        if !equalsDefault {
            cliPrintf(defaultFormat, (*value).name);
            printValuePointer(value,
                              (*pg).address.offset(valueOffset as isize) as
                                  *const libc::c_void, 0i32 != 0);
            cliPrintLinefeed();
        }
    };
}
unsafe extern "C" fn cliGet(mut cmdline: *mut libc::c_char) {
    let mut val: *const clivalue_t = 0 as *const clivalue_t;
    let mut matchedCommands: libc::c_int = 0i32;
    pidProfileIndexToUse = getCurrentPidProfileIndex() as int8_t;
    rateProfileIndexToUse = getCurrentControlRateProfileIndex() as int8_t;
    backupAndResetConfigs();
    let mut i: uint32_t = 0i32 as uint32_t;
    while i < valueTableEntryCount as libc::c_uint {
        if !strcasestr((*valueTable.as_ptr().offset(i as isize)).name,
                       cmdline).is_null() {
            val =
                &*valueTable.as_ptr().offset(i as isize) as *const clivalue_t;
            if matchedCommands > 0i32 { cliPrintLinefeed(); }
            cliPrintf(b"%s = \x00" as *const u8 as *const libc::c_char,
                      (*valueTable.as_ptr().offset(i as isize)).name);
            cliPrintVar(val, 0i32 != 0);
            cliPrintLinefeed();
            match (*val).type_0 as libc::c_int & 0x18i32 {
                8 => {
                    cliProfile(b"\x00" as *const u8 as *const libc::c_char as
                                   *mut libc::c_char);
                }
                16 => {
                    cliRateProfile(b"\x00" as *const u8 as *const libc::c_char
                                       as *mut libc::c_char);
                }
                _ => { }
            }
            cliPrintVarRange(val);
            cliPrintVarDefault(val);
            matchedCommands += 1
        }
        i = i.wrapping_add(1)
    }
    restoreConfigs();
    pidProfileIndexToUse = -1i32 as int8_t;
    rateProfileIndexToUse = -1i32 as int8_t;
    if matchedCommands != 0 { return }
    cliPrintErrorLinef(b"Invalid name\x00" as *const u8 as
                           *const libc::c_char);
}
unsafe extern "C" fn getWordLength(mut bufBegin: *mut libc::c_char,
                                   mut bufEnd: *mut libc::c_char) -> uint8_t {
    while *bufEnd.offset(-1) as libc::c_int == ' ' as i32 {
        bufEnd = bufEnd.offset(-1)
    }
    return bufEnd.wrapping_offset_from(bufBegin) as libc::c_long as uint8_t;
}
unsafe extern "C" fn cliSet(mut cmdline: *mut libc::c_char) {
    let len: uint32_t = strlen(cmdline) as uint32_t;
    let mut eqptr: *mut libc::c_char = 0 as *mut libc::c_char;
    if len == 0i32 as libc::c_uint ||
           len == 1i32 as libc::c_uint &&
               *cmdline.offset(0) as libc::c_int == '*' as i32 {
        cliPrintLine(b"Current settings: \x00" as *const u8 as
                         *const libc::c_char);
        let mut i: uint32_t = 0i32 as uint32_t;
        while i < valueTableEntryCount as libc::c_uint {
            let mut val: *const clivalue_t =
                &*valueTable.as_ptr().offset(i as isize) as *const clivalue_t;
            cliPrintf(b"%s = \x00" as *const u8 as *const libc::c_char,
                      (*valueTable.as_ptr().offset(i as isize)).name);
            cliPrintVar(val, len != 0);
            cliPrintLinefeed();
            i = i.wrapping_add(1)
        }
    } else {
        eqptr = strstr(cmdline, b"=\x00" as *const u8 as *const libc::c_char);
        if !eqptr.is_null() {
            // has equals
            let mut variableNameLength: uint8_t =
                getWordLength(cmdline, eqptr);
            // skip the '=' and any ' ' characters
            eqptr = eqptr.offset(1);
            eqptr = skipSpace(eqptr);
            let mut i_0: uint32_t = 0i32 as uint32_t;
            while i_0 < valueTableEntryCount as libc::c_uint {
                let mut val_0: *const clivalue_t =
                    &*valueTable.as_ptr().offset(i_0 as isize) as
                        *const clivalue_t;
                // ensure exact match when setting to prevent setting variables with shorter names
                if strncasecmp(cmdline, (*val_0).name, strlen((*val_0).name))
                       == 0i32 &&
                       variableNameLength as libc::c_ulong ==
                           strlen((*val_0).name) {
                    let mut valueChanged: bool = 0i32 != 0;
                    let mut value: int16_t = 0i32 as int16_t;
                    match (*val_0).type_0 as libc::c_int & 0x60i32 {
                        0 => {
                            let mut value_0: int16_t = atoi(eqptr) as int16_t;
                            if value_0 as libc::c_int >=
                                   (*val_0).config.minmax.min as libc::c_int
                                   &&
                                   value_0 as libc::c_int <=
                                       (*val_0).config.minmax.max as
                                           libc::c_int {
                                cliSetVar(val_0, value_0);
                                valueChanged = 1i32 != 0
                            }
                        }
                        32 | 96 => {
                            let mut tableIndex: libc::c_int = 0;
                            if (*val_0).type_0 as libc::c_int & 0x60i32 ==
                                   MODE_BITSET as libc::c_int {
                                tableIndex = TABLE_OFF_ON as libc::c_int
                            } else {
                                tableIndex =
                                    (*val_0).config.lookup.tableIndex as
                                        libc::c_int
                            }
                            let mut tableEntry: *const lookupTableEntry_t =
                                &*lookupTables.as_ptr().offset(tableIndex as
                                                                   isize) as
                                    *const lookupTableEntry_t;
                            let mut matched: bool = 0i32 != 0;
                            let mut tableValueIndex: uint32_t =
                                0i32 as uint32_t;
                            while tableValueIndex <
                                      (*tableEntry).valueCount as libc::c_uint
                                      && !matched {
                                matched =
                                    !(*(*tableEntry).values.offset(tableValueIndex
                                                                       as
                                                                       isize)).is_null()
                                        &&
                                        strcasecmp(*(*tableEntry).values.offset(tableValueIndex
                                                                                    as
                                                                                    isize),
                                                   eqptr) == 0i32;
                                if matched {
                                    value = tableValueIndex as int16_t;
                                    cliSetVar(val_0, value);
                                    valueChanged = 1i32 != 0
                                }
                                tableValueIndex =
                                    tableValueIndex.wrapping_add(1)
                            }
                        }
                        64 => {
                            let arrayLength: uint8_t =
                                (*val_0).config.array.length;
                            let mut valPtr: *mut libc::c_char = eqptr;
                            let mut i_1: libc::c_int = 0i32;
                            while i_1 < arrayLength as libc::c_int &&
                                      !valPtr.is_null() {
                                // skip spaces
                                valPtr = skipSpace(valPtr);
                                // process substring starting at valPtr
                            // note: no need to copy substrings for atoi()
                            //       it stops at the first character that cannot be converted...
                                match (*val_0).type_0 as libc::c_int & 0x7i32
                                    {
                                    1 => {
                                        // fetch data pointer
                                        let mut data_0: *mut int8_t =
                                            (cliGetValuePointer(val_0) as
                                                 *mut int8_t).offset(i_1 as
                                                                         isize);
                                        // store value
                                        *data_0 =
                                            atoi(valPtr as
                                                     *const libc::c_char) as
                                                int8_t
                                    }
                                    2 => {
                                        // fetch data pointer
                                        let mut data_1: *mut uint16_t =
                                            (cliGetValuePointer(val_0) as
                                                 *mut uint16_t).offset(i_1 as
                                                                           isize);
                                        // store value
                                        *data_1 =
                                            atoi(valPtr as
                                                     *const libc::c_char) as
                                                uint16_t
                                    }
                                    3 => {
                                        // fetch data pointer
                                        let mut data_2: *mut int16_t =
                                            (cliGetValuePointer(val_0) as
                                                 *mut int16_t).offset(i_1 as
                                                                          isize);
                                        // store value
                                        *data_2 =
                                            atoi(valPtr as
                                                     *const libc::c_char) as
                                                int16_t
                                    }
                                    0 | _ => {
                                        // fetch data pointer
                                        let mut data: *mut uint8_t =
                                            (cliGetValuePointer(val_0) as
                                                 *mut uint8_t).offset(i_1 as
                                                                          isize);
                                        // store value
                                        *data =
                                            atoi(valPtr as
                                                     *const libc::c_char) as
                                                uint8_t
                                    }
                                }
                                // find next comma (or end of string)
                                valPtr = strchr(valPtr, ',' as i32).offset(1);
                                i_1 += 1
                            }
                            // mark as changed
                            valueChanged = 1i32 != 0
                        }
                        _ => { }
                    }
                    if valueChanged {
                        cliPrintf(b"%s set to \x00" as *const u8 as
                                      *const libc::c_char, (*val_0).name);
                        cliPrintVar(val_0, 0i32 != 0);
                    } else {
                        cliPrintErrorLinef(b"Invalid value\x00" as *const u8
                                               as *const libc::c_char);
                        cliPrintVarRange(val_0);
                    }
                    return
                }
                i_0 = i_0.wrapping_add(1)
            }
            cliPrintErrorLinef(b"Invalid name\x00" as *const u8 as
                                   *const libc::c_char);
        } else {
            // no equals, check for matching variables.
            cliGet(cmdline);
        }
    };
}
unsafe extern "C" fn cliStatus(mut cmdline: *mut libc::c_char) {
    cliPrintLinef(b"System Uptime: %d seconds\x00" as *const u8 as
                      *const libc::c_char,
                  millis().wrapping_div(1000i32 as libc::c_uint));
    cliPrintLinef(b"Voltage: %d * 0.1V (%dS battery - %s)\x00" as *const u8 as
                      *const libc::c_char, getBatteryVoltage() as libc::c_int,
                  getBatteryCellCount() as libc::c_int,
                  getBatteryStateString());
    cliPrintf(b"CPU Clock=%dMHz\x00" as *const u8 as *const libc::c_char,
              SystemCoreClock.wrapping_div(1000000i32 as libc::c_uint));
    /* USE_SENSOR_NAMES */
    cliPrintLinefeed();
    let i2cErrorCounter: uint16_t = i2cGetErrorCounter();
    cliPrintLinef(b"Stack size: %d, Stack address: 0x%x\x00" as *const u8 as
                      *const libc::c_char, stackTotalSize(), stackHighMem());
    cliPrintLinef(b"I2C Errors: %d, config size: %d, max available config: %d\x00"
                      as *const u8 as *const libc::c_char,
                  i2cErrorCounter as libc::c_int,
                  getEEPROMConfigSize() as libc::c_int,
                  (&mut __config_end as
                       *mut uint8_t).wrapping_offset_from(&mut __config_start
                                                              as *mut uint8_t)
                      as libc::c_long);
    let gyroRate: libc::c_int =
        if getTaskDeltaTime(TASK_GYROPID) == 0i32 {
            0i32
        } else {
            (1000000.0f32 / getTaskDeltaTime(TASK_GYROPID) as libc::c_float)
                as libc::c_int
        };
    let rxRate: libc::c_int =
        if currentRxRefreshRate as libc::c_int == 0i32 {
            0i32
        } else {
            (1000000.0f32 / currentRxRefreshRate as libc::c_float) as
                libc::c_int
        };
    let systemRate: libc::c_int =
        if getTaskDeltaTime(TASK_SYSTEM) == 0i32 {
            0i32
        } else {
            (1000000.0f32 / getTaskDeltaTime(TASK_SYSTEM) as libc::c_float) as
                libc::c_int
        };
    cliPrintLinef(b"CPU:%d%%, cycle time: %d, GYRO rate: %d, RX rate: %d, System rate: %d\x00"
                      as *const u8 as *const libc::c_char,
                  constrain(averageSystemLoadPercent as libc::c_int, 0i32,
                            100i32), getTaskDeltaTime(TASK_GYROPID), gyroRate,
                  rxRate, systemRate);
    cliPrint(b"Arming disable flags:\x00" as *const u8 as
                 *const libc::c_char);
    let mut flags: armingDisableFlags_e = getArmingDisableFlags();
    while flags as u64 != 0 {
        let bitpos: libc::c_int = ffs(flags as libc::c_int) - 1i32;
        flags =
            ::core::mem::transmute::<libc::c_uint,
                                     armingDisableFlags_e>(flags as
                                                               libc::c_uint &
                                                               !(1i32 <<
                                                                     bitpos)
                                                                   as
                                                                   libc::c_uint);
        cliPrintf(b" %s\x00" as *const u8 as *const libc::c_char,
                  armingDisableFlagNames[bitpos as usize]);
    }
    cliPrintLinefeed();
}
unsafe extern "C" fn cliTasks(mut cmdline: *mut libc::c_char) {
    let mut maxLoadSum: libc::c_int = 0i32;
    let mut averageLoadSum: libc::c_int = 0i32;
    let mut taskId: cfTaskId_e = TASK_SYSTEM;
    while (taskId as libc::c_uint) < TASK_COUNT as libc::c_int as libc::c_uint
          {
        let mut taskInfo: cfTaskInfo_t =
            cfTaskInfo_t{taskName: 0 as *const libc::c_char,
                         subTaskName: 0 as *const libc::c_char,
                         isEnabled: false,
                         staticPriority: 0,
                         desiredPeriod: 0,
                         latestDeltaTime: 0,
                         maxExecutionTime: 0,
                         totalExecutionTime: 0,
                         averageExecutionTime: 0,};
        getTaskInfo(taskId, &mut taskInfo);
        if taskInfo.isEnabled {
            let mut taskFrequency: libc::c_int = 0;
            let mut subTaskFrequency: libc::c_int = 0i32;
            if taskId as libc::c_uint ==
                   TASK_GYROPID as libc::c_int as libc::c_uint {
                subTaskFrequency =
                    if taskInfo.latestDeltaTime == 0i32 {
                        0i32
                    } else {
                        (1000000.0f32 /
                             taskInfo.latestDeltaTime as libc::c_float) as
                            libc::c_int
                    };
                taskFrequency =
                    subTaskFrequency /
                        (*pidConfig()).pid_process_denom as libc::c_int;
                if (*pidConfig()).pid_process_denom as libc::c_int > 1i32 {
                    cliPrintf(b"%02d - (%15s) \x00" as *const u8 as
                                  *const libc::c_char, taskId as libc::c_uint,
                              taskInfo.taskName);
                } else {
                    taskFrequency = subTaskFrequency;
                    cliPrintf(b"%02d - (%11s/%3s) \x00" as *const u8 as
                                  *const libc::c_char, taskId as libc::c_uint,
                              taskInfo.subTaskName, taskInfo.taskName);
                }
            } else {
                taskFrequency =
                    if taskInfo.latestDeltaTime == 0i32 {
                        0i32
                    } else {
                        (1000000.0f32 /
                             taskInfo.latestDeltaTime as libc::c_float) as
                            libc::c_int
                    };
                cliPrintf(b"%02d - (%15s) \x00" as *const u8 as
                              *const libc::c_char, taskId as libc::c_uint,
                          taskInfo.taskName);
            }
            let maxLoad: libc::c_int =
                if taskInfo.maxExecutionTime == 0i32 as libc::c_uint {
                    0i32 as libc::c_uint
                } else {
                    taskInfo.maxExecutionTime.wrapping_mul(taskFrequency as
                                                               libc::c_uint).wrapping_add(5000i32
                                                                                              as
                                                                                              libc::c_uint).wrapping_div(1000i32
                                                                                                                             as
                                                                                                                             libc::c_uint)
                } as libc::c_int;
            let averageLoad: libc::c_int =
                if taskInfo.averageExecutionTime == 0i32 as libc::c_uint {
                    0i32 as libc::c_uint
                } else {
                    taskInfo.averageExecutionTime.wrapping_mul(taskFrequency
                                                                   as
                                                                   libc::c_uint).wrapping_add(5000i32
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_div(1000i32
                                                                                                                                 as
                                                                                                                                 libc::c_uint)
                } as libc::c_int;
            if taskId as libc::c_uint !=
                   TASK_SERIAL as libc::c_int as libc::c_uint {
                maxLoadSum += maxLoad;
                averageLoadSum += averageLoad
            }
            if (*systemConfig()).task_statistics != 0 {
                cliPrintLinef(b"%6d %7d %7d %4d.%1d%% %4d.%1d%% %9d\x00" as
                                  *const u8 as *const libc::c_char,
                              taskFrequency, taskInfo.maxExecutionTime,
                              taskInfo.averageExecutionTime, maxLoad / 10i32,
                              maxLoad % 10i32, averageLoad / 10i32,
                              averageLoad % 10i32,
                              taskInfo.totalExecutionTime.wrapping_div(1000i32
                                                                           as
                                                                           libc::c_uint));
            } else {
                cliPrintLinef(b"%6d\x00" as *const u8 as *const libc::c_char,
                              taskFrequency);
            }
            if taskId as libc::c_uint ==
                   TASK_GYROPID as libc::c_int as libc::c_uint &&
                   (*pidConfig()).pid_process_denom as libc::c_int > 1i32 {
                cliPrintLinef(b"   - (%15s) %6d\x00" as *const u8 as
                                  *const libc::c_char, taskInfo.subTaskName,
                              subTaskFrequency);
            }
            schedulerResetTaskMaxExecutionTime(taskId);
        }
        taskId += 1
    }
    if (*systemConfig()).task_statistics != 0 {
        let mut checkFuncInfo: cfCheckFuncInfo_t =
            cfCheckFuncInfo_t{maxExecutionTime: 0,
                              totalExecutionTime: 0,
                              averageExecutionTime: 0,};
        getCheckFuncInfo(&mut checkFuncInfo);
        cliPrintLinef(b"RX Check Function %19d %7d %25d\x00" as *const u8 as
                          *const libc::c_char, checkFuncInfo.maxExecutionTime,
                      checkFuncInfo.averageExecutionTime,
                      checkFuncInfo.totalExecutionTime.wrapping_div(1000i32 as
                                                                        libc::c_uint));
        cliPrintLinef(b"Total (excluding SERIAL) %25d.%1d%% %4d.%1d%%\x00" as
                          *const u8 as *const libc::c_char,
                      maxLoadSum / 10i32, maxLoadSum % 10i32,
                      averageLoadSum / 10i32, averageLoadSum % 10i32);
    };
}
unsafe extern "C" fn cliVersion(mut cmdline: *mut libc::c_char) {
    cliPrintLinef(b"# %s / %s (%s) %s %s / %s (%s) MSP API: %s\x00" as
                      *const u8 as *const libc::c_char,
                  b"Cleanflight\x00" as *const u8 as *const libc::c_char,
                  targetName, (*systemConfig()).boardIdentifier.as_ptr(),
                  b"2.5.0\x00" as *const u8 as *const libc::c_char, buildDate,
                  buildTime, shortGitRevision,
                  b"1.40\x00" as *const u8 as *const libc::c_char);
}
unsafe extern "C" fn cliRcSmoothing(mut cmdline: *mut libc::c_char) {
    cliPrint(b"# RC Smoothing Type: \x00" as *const u8 as
                 *const libc::c_char);
    if (*rxConfig()).rc_smoothing_type as libc::c_int ==
           RC_SMOOTHING_TYPE_FILTER as libc::c_int {
        cliPrintLine(b"FILTER\x00" as *const u8 as *const libc::c_char);
        let mut avgRxFrameMs: uint16_t =
            rcSmoothingGetValue(RC_SMOOTHING_VALUE_AVERAGE_FRAME as
                                    libc::c_int) as uint16_t;
        if rcSmoothingAutoCalculate() {
            cliPrint(b"# Detected RX frame rate: \x00" as *const u8 as
                         *const libc::c_char);
            if avgRxFrameMs as libc::c_int == 0i32 {
                cliPrintLine(b"NO SIGNAL\x00" as *const u8 as
                                 *const libc::c_char);
            } else {
                cliPrintLinef(b"%d.%dms\x00" as *const u8 as
                                  *const libc::c_char,
                              avgRxFrameMs as libc::c_int / 1000i32,
                              avgRxFrameMs as libc::c_int % 1000i32);
            }
        }
        cliPrint(b"# Input filter type: \x00" as *const u8 as
                     *const libc::c_char);
        cliPrintLinef(*(*lookupTables.as_ptr().offset(TABLE_RC_SMOOTHING_INPUT_TYPE
                                                          as libc::c_int as
                                                          isize)).values.offset((*rxConfig()).rc_smoothing_input_type
                                                                                    as
                                                                                    isize));
        cliPrintf(b"# Active input cutoff: %dhz \x00" as *const u8 as
                      *const libc::c_char,
                  rcSmoothingGetValue(RC_SMOOTHING_VALUE_INPUT_ACTIVE as
                                          libc::c_int));
        if (*rxConfig()).rc_smoothing_input_cutoff as libc::c_int == 0i32 {
            cliPrintLine(b"(auto)\x00" as *const u8 as *const libc::c_char);
        } else {
            cliPrintLine(b"(manual)\x00" as *const u8 as *const libc::c_char);
        }
        cliPrint(b"# Derivative filter type: \x00" as *const u8 as
                     *const libc::c_char);
        cliPrintLinef(*(*lookupTables.as_ptr().offset(TABLE_RC_SMOOTHING_DERIVATIVE_TYPE
                                                          as libc::c_int as
                                                          isize)).values.offset((*rxConfig()).rc_smoothing_derivative_type
                                                                                    as
                                                                                    isize));
        cliPrintf(b"# Active derivative cutoff: %dhz (\x00" as *const u8 as
                      *const libc::c_char,
                  rcSmoothingGetValue(RC_SMOOTHING_VALUE_DERIVATIVE_ACTIVE as
                                          libc::c_int));
        if (*rxConfig()).rc_smoothing_derivative_type as libc::c_int ==
               RC_SMOOTHING_DERIVATIVE_OFF as libc::c_int {
            cliPrintLine(b"off)\x00" as *const u8 as *const libc::c_char);
        } else if (*rxConfig()).rc_smoothing_derivative_cutoff as libc::c_int
                      == 0i32 {
            cliPrintLine(b"auto)\x00" as *const u8 as *const libc::c_char);
        } else {
            cliPrintLine(b"manual)\x00" as *const u8 as *const libc::c_char);
        }
    } else {
        cliPrintLine(b"INTERPOLATION\x00" as *const u8 as
                         *const libc::c_char);
    };
}
// Handy macros for keeping the table tidy.
// DEFS : Single entry
// DEFA : Array of uint8_t (stride = 1)
// DEFW : Wider stride case; array of structs.
#[no_mangle]
pub static mut resourceTable: [cliResourceValue_t; 31] =
    [{
         let mut init =
             cliResourceValue_t{owner: OWNER_BEEPER as libc::c_int as uint8_t,
                                pgn: 503i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 0u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner: OWNER_MOTOR as libc::c_int as uint8_t,
                                pgn: 6i32 as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 6u64 as uint8_t,
                                maxIndex: 8i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner: OWNER_SERVO as libc::c_int as uint8_t,
                                pgn: 52i32 as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 4u64 as uint8_t,
                                maxIndex: 8i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_PPMINPUT as libc::c_int as uint8_t,
                                pgn: 507i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 0u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_PWMINPUT as libc::c_int as uint8_t,
                                pgn: 508i32 as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 0u64 as uint8_t,
                                maxIndex: 8i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_SONAR_TRIGGER as libc::c_int as
                                        uint8_t,
                                pgn: 516i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 0u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_SONAR_ECHO as libc::c_int as
                                        uint8_t,
                                pgn: 516i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 1u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_LED_STRIP as libc::c_int as uint8_t,
                                pgn: 27i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 241u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_SERIAL_TX as libc::c_int as uint8_t,
                                pgn: 509i32 as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 0u64 as uint8_t,
                                maxIndex: (10i32 + 2i32) as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_SERIAL_RX as libc::c_int as uint8_t,
                                pgn: 509i32 as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 12u64 as uint8_t,
                                maxIndex: (10i32 + 2i32) as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_I2C_SCL as libc::c_int as uint8_t,
                                pgn: 518i32 as pgn_t,
                                stride:
                                    ::core::mem::size_of::<i2cConfig_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 0u64 as uint8_t,
                                maxIndex: 2i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_I2C_SDA as libc::c_int as uint8_t,
                                pgn: 518i32 as pgn_t,
                                stride:
                                    ::core::mem::size_of::<i2cConfig_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 1u64 as uint8_t,
                                maxIndex: 2i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner: OWNER_LED as libc::c_int as uint8_t,
                                pgn: 505i32 as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 0u64 as uint8_t,
                                maxIndex: 3i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_RX_BIND as libc::c_int as uint8_t,
                                pgn: 24i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 11u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_RX_BIND_PLUG as libc::c_int as
                                        uint8_t,
                                pgn: 24i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 12u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_SPI_SCK as libc::c_int as uint8_t,
                                pgn: 520i32 as pgn_t,
                                stride:
                                    ::core::mem::size_of::<spiPinConfig_t>()
                                        as libc::c_ulong as uint8_t,
                                offset: 0u64 as uint8_t,
                                maxIndex: 3i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_SPI_MISO as libc::c_int as uint8_t,
                                pgn: 520i32 as pgn_t,
                                stride:
                                    ::core::mem::size_of::<spiPinConfig_t>()
                                        as libc::c_ulong as uint8_t,
                                offset: 1u64 as uint8_t,
                                maxIndex: 3i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_SPI_MOSI as libc::c_int as uint8_t,
                                pgn: 520i32 as pgn_t,
                                stride:
                                    ::core::mem::size_of::<spiPinConfig_t>()
                                        as libc::c_ulong as uint8_t,
                                offset: 2u64 as uint8_t,
                                maxIndex: 3i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_ESCSERIAL as libc::c_int as uint8_t,
                                pgn: 521i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 0u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_CAMERA_CONTROL as libc::c_int as
                                        uint8_t,
                                pgn: 522i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 10u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_ADC_BATT as libc::c_int as uint8_t,
                                pgn: 510i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 1u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_ADC_RSSI as libc::c_int as uint8_t,
                                pgn: 510i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 3u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_ADC_CURR as libc::c_int as uint8_t,
                                pgn: 510i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 5u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_ADC_EXT as libc::c_int as uint8_t,
                                pgn: 510i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 7u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_BARO_CS as libc::c_int as uint8_t,
                                pgn: 38i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 2u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_COMPASS_CS as libc::c_int as
                                        uint8_t,
                                pgn: 40i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 13u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_COMPASS_EXTI as libc::c_int as
                                        uint8_t,
                                pgn: 40i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 14u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner: OWNER_PINIO as libc::c_int as uint8_t,
                                pgn: 529i32 as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 0u64 as uint8_t,
                                maxIndex: 4i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_FLASH_CS as libc::c_int as uint8_t,
                                pgn: 506i32 as pgn_t,
                                stride: 0i32 as uint8_t,
                                offset: 0u64 as uint8_t,
                                maxIndex: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_SPI_PREINIT_IPU as libc::c_int as
                                        uint8_t,
                                pgn: 535i32 as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 0u64 as uint8_t,
                                maxIndex: 11i32 as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_SPI_PREINIT_OPU as libc::c_int as
                                        uint8_t,
                                pgn: 536i32 as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 0u64 as uint8_t,
                                maxIndex: 2i32 as uint8_t,};
         init
     }];
unsafe extern "C" fn getIoTag(value: cliResourceValue_t, mut index: uint8_t)
 -> *mut ioTag_t {
    let mut rec: *const pgRegistry_t = pgFind(value.pgn);
    return (*rec).address.offset((value.stride as libc::c_int *
                                      index as libc::c_int) as
                                     isize).offset(value.offset as libc::c_int
                                                       as isize) as
               *mut ioTag_t;
}
unsafe extern "C" fn printResource(mut dumpMask: uint8_t) {
    let mut i: libc::c_uint = 0i32 as libc::c_uint;
    while (i as libc::c_ulong) <
              (::core::mem::size_of::<[cliResourceValue_t; 31]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<cliResourceValue_t>()
                                                   as libc::c_ulong) {
        let mut owner: *const libc::c_char =
            ownerNames[resourceTable[i as usize].owner as usize];
        let mut pg: *const pgRegistry_t =
            pgFind(resourceTable[i as usize].pgn);
        let mut currentConfig: *const libc::c_void = 0 as *const libc::c_void;
        let mut defaultConfig: *const libc::c_void = 0 as *const libc::c_void;
        if configIsInCopy {
            currentConfig = (*pg).copy as *const libc::c_void;
            defaultConfig = (*pg).address as *const libc::c_void
        } else {
            currentConfig = (*pg).address as *const libc::c_void;
            defaultConfig = 0 as *const libc::c_void
        }
        let mut index: libc::c_int = 0i32;
        while index <
                  (if resourceTable[i as usize].maxIndex as libc::c_int ==
                          0i32 {
                       1i32
                   } else {
                       resourceTable[i as usize].maxIndex as libc::c_int
                   }) {
            let ioTag: ioTag_t =
                *(currentConfig as
                      *const uint8_t).offset((resourceTable[i as usize].stride
                                                  as libc::c_int * index) as
                                                 isize).offset(resourceTable[i
                                                                                 as
                                                                                 usize].offset
                                                                   as
                                                                   libc::c_int
                                                                   as isize);
            let ioTagDefault: ioTag_t =
                *(defaultConfig as
                      *const uint8_t).offset((resourceTable[i as usize].stride
                                                  as libc::c_int * index) as
                                                 isize).offset(resourceTable[i
                                                                                 as
                                                                                 usize].offset
                                                                   as
                                                                   libc::c_int
                                                                   as isize);
            let mut equalsDefault: bool =
                ioTag as libc::c_int == ioTagDefault as libc::c_int;
            let mut format: *const libc::c_char =
                b"resource %s %d %c%02d\x00" as *const u8 as
                    *const libc::c_char;
            let mut formatUnassigned: *const libc::c_char =
                b"resource %s %d NONE\x00" as *const u8 as
                    *const libc::c_char;
            if ioTagDefault == 0 {
                cliDefaultPrintLinef(dumpMask, equalsDefault,
                                     formatUnassigned, owner, index + 1i32);
            } else {
                cliDefaultPrintLinef(dumpMask, equalsDefault, format, owner,
                                     index + 1i32,
                                     (ioTagDefault as libc::c_int >> 4i32) -
                                         1i32 + 'A' as i32,
                                     ioTagDefault as libc::c_int & 0xfi32);
            }
            if ioTag == 0 {
                if dumpMask as libc::c_int & HIDE_UNUSED as libc::c_int == 0 {
                    cliDumpPrintLinef(dumpMask, equalsDefault,
                                      formatUnassigned, owner, index + 1i32);
                }
            } else {
                cliDumpPrintLinef(dumpMask, equalsDefault, format, owner,
                                  index + 1i32,
                                  (ioTag as libc::c_int >> 4i32) - 1i32 +
                                      'A' as i32,
                                  ioTag as libc::c_int & 0xfi32);
            }
            index += 1
        }
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn printResourceOwner(mut owner: uint8_t,
                                        mut index: uint8_t) {
    cliPrintf(b"%s\x00" as *const u8 as *const libc::c_char,
              ownerNames[resourceTable[owner as usize].owner as usize]);
    if resourceTable[owner as usize].maxIndex as libc::c_int > 0i32 {
        cliPrintf(b" %d\x00" as *const u8 as *const libc::c_char,
                  index as libc::c_int + 1i32);
    };
}
unsafe extern "C" fn resourceCheck(mut resourceIndex: uint8_t,
                                   mut index: uint8_t, mut newTag: ioTag_t) {
    if newTag == 0 { return }
    let mut format: *const libc::c_char =
        b"\r\nNOTE: %c%02d already assigned to \x00" as *const u8 as
            *const libc::c_char;
    let mut r: libc::c_int = 0i32;
    while r <
              (::core::mem::size_of::<[cliResourceValue_t; 31]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<cliResourceValue_t>()
                                                   as libc::c_ulong) as
                  libc::c_int {
        let mut current_block_12: u64;
        let mut i: libc::c_int = 0i32;
        while i <
                  (if resourceTable[r as usize].maxIndex as libc::c_int ==
                          0i32 {
                       1i32
                   } else {
                       resourceTable[r as usize].maxIndex as libc::c_int
                   }) {
            let mut tag: *mut ioTag_t =
                getIoTag(resourceTable[r as usize], i as uint8_t);
            if *tag as libc::c_int == newTag as libc::c_int {
                let mut cleared: bool = 0i32 != 0;
                if r == resourceIndex as libc::c_int {
                    if i == index as libc::c_int {
                        current_block_12 = 15427931788582360902;
                    } else {
                        *tag = 0i32 as ioTag_t;
                        cleared = 1i32 != 0;
                        current_block_12 = 12800627514080957624;
                    }
                } else { current_block_12 = 12800627514080957624; }
                match current_block_12 {
                    15427931788582360902 => { }
                    _ => {
                        cliPrintf(format,
                                  (newTag as libc::c_int >> 4i32) - 1i32 +
                                      'A' as i32,
                                  newTag as libc::c_int & 0xfi32);
                        printResourceOwner(r as uint8_t, i as uint8_t);
                        if cleared {
                            cliPrintf(b". \x00" as *const u8 as
                                          *const libc::c_char);
                            printResourceOwner(r as uint8_t, i as uint8_t);
                            cliPrintf(b" disabled\x00" as *const u8 as
                                          *const libc::c_char);
                        }
                        cliPrintLine(b".\x00" as *const u8 as
                                         *const libc::c_char);
                    }
                }
            }
            i += 1
        }
        r += 1
    };
}
unsafe extern "C" fn strToPin(mut pch: *mut libc::c_char,
                              mut tag: *mut ioTag_t) -> bool {
    if strcasecmp(pch, b"NONE\x00" as *const u8 as *const libc::c_char) ==
           0i32 {
        *tag = 0i32 as ioTag_t;
        return 1i32 != 0
    } else {
        let mut pin: libc::c_uint = 0i32 as libc::c_uint;
        let mut port: libc::c_uint =
            if *pch as libc::c_int >= 'a' as i32 {
                (*pch as libc::c_int) - 'a' as i32
            } else { (*pch as libc::c_int) - 'A' as i32 } as libc::c_uint;
        if port < 8i32 as libc::c_uint {
            pch = pch.offset(1);
            pin = atoi(pch) as libc::c_uint;
            if pin < 16i32 as libc::c_uint {
                *tag =
                    (port.wrapping_add(1i32 as libc::c_uint) << 4i32 | pin) as
                        ioTag_t;
                return 1i32 != 0
            }
        }
    }
    return 0i32 != 0;
}
unsafe extern "C" fn cliResource(mut cmdline: *mut libc::c_char) {
    let mut len: libc::c_int = strlen(cmdline) as libc::c_int;
    if len == 0i32 {
        printResource((DUMP_MASTER as libc::c_int |
                           HIDE_UNUSED as libc::c_int) as uint8_t);
        return
    } else {
        if strncasecmp(cmdline,
                       b"list\x00" as *const u8 as *const libc::c_char,
                       len as libc::c_ulong) == 0i32 {
            cliPrintLine(b"IO\x00" as *const u8 as *const libc::c_char);
            let mut i: libc::c_int = 0i32;
            while i <
                      (0xffffi32 - (0xffffi32 >> 1i32 & 0x77777777i32) -
                           (0xffffi32 >> 2i32 & 0x33333333i32) -
                           (0xffffi32 >> 3i32 & 0x11111111i32) +
                           (0xffffi32 - (0xffffi32 >> 1i32 & 0x77777777i32) -
                                (0xffffi32 >> 2i32 & 0x33333333i32) -
                                (0xffffi32 >> 3i32 & 0x11111111i32) >> 4i32) &
                           0xf0f0f0fi32) % 255i32 +
                          (0xffffi32 - (0xffffi32 >> 1i32 & 0x77777777i32) -
                               (0xffffi32 >> 2i32 & 0x33333333i32) -
                               (0xffffi32 >> 3i32 & 0x11111111i32) +
                               (0xffffi32 -
                                    (0xffffi32 >> 1i32 & 0x77777777i32) -
                                    (0xffffi32 >> 2i32 & 0x33333333i32) -
                                    (0xffffi32 >> 3i32 & 0x11111111i32) >>
                                    4i32) & 0xf0f0f0fi32) % 255i32 +
                          ((1i32 << 13i32 | 1i32 << 14i32 | 1i32 << 15i32) -
                               ((1i32 << 13i32 | 1i32 << 14i32 |
                                     1i32 << 15i32) >> 1i32 & 0x77777777i32) -
                               ((1i32 << 13i32 | 1i32 << 14i32 |
                                     1i32 << 15i32) >> 2i32 & 0x33333333i32) -
                               ((1i32 << 13i32 | 1i32 << 14i32 |
                                     1i32 << 15i32) >> 3i32 & 0x11111111i32) +
                               ((1i32 << 13i32 | 1i32 << 14i32 |
                                     1i32 << 15i32) -
                                    ((1i32 << 13i32 | 1i32 << 14i32 |
                                          1i32 << 15i32) >> 1i32 &
                                         0x77777777i32) -
                                    ((1i32 << 13i32 | 1i32 << 14i32 |
                                          1i32 << 15i32) >> 2i32 &
                                         0x33333333i32) -
                                    ((1i32 << 13i32 | 1i32 << 14i32 |
                                          1i32 << 15i32) >> 3i32 &
                                         0x11111111i32) >> 4i32) &
                               0xf0f0f0fi32) % 255i32 + 0i32 + 0i32 +
                          ((1i32 << 0i32 | 1i32 << 1i32 | 1i32 << 3i32 |
                                1i32 << 4i32) -
                               ((1i32 << 0i32 | 1i32 << 1i32 | 1i32 << 3i32 |
                                     1i32 << 4i32) >> 1i32 & 0x77777777i32) -
                               ((1i32 << 0i32 | 1i32 << 1i32 | 1i32 << 3i32 |
                                     1i32 << 4i32) >> 2i32 & 0x33333333i32) -
                               ((1i32 << 0i32 | 1i32 << 1i32 | 1i32 << 3i32 |
                                     1i32 << 4i32) >> 3i32 & 0x11111111i32) +
                               ((1i32 << 0i32 | 1i32 << 1i32 | 1i32 << 3i32 |
                                     1i32 << 4i32) -
                                    ((1i32 << 0i32 | 1i32 << 1i32 |
                                          1i32 << 3i32 | 1i32 << 4i32) >> 1i32
                                         & 0x77777777i32) -
                                    ((1i32 << 0i32 | 1i32 << 1i32 |
                                          1i32 << 3i32 | 1i32 << 4i32) >> 2i32
                                         & 0x33333333i32) -
                                    ((1i32 << 0i32 | 1i32 << 1i32 |
                                          1i32 << 3i32 | 1i32 << 4i32) >> 3i32
                                         & 0x11111111i32) >> 4i32) &
                               0xf0f0f0fi32) % 255i32 + 0i32 {
                let mut owner: *const libc::c_char = 0 as *const libc::c_char;
                owner =
                    ownerNames[(*ioRecs.as_mut_ptr().offset(i as isize)).owner
                                   as usize];
                cliPrintf(b"%c%02d: %s\x00" as *const u8 as
                              *const libc::c_char,
                          IO_GPIOPortIdx(ioRecs.as_mut_ptr().offset(i as
                                                                        isize)
                                             as IO_t) + 'A' as i32,
                          IO_GPIOPinIdx(ioRecs.as_mut_ptr().offset(i as isize)
                                            as IO_t), owner);
                if (*ioRecs.as_mut_ptr().offset(i as isize)).index as
                       libc::c_int > 0i32 {
                    cliPrintf(b" %d\x00" as *const u8 as *const libc::c_char,
                              (*ioRecs.as_mut_ptr().offset(i as isize)).index
                                  as libc::c_int);
                }
                cliPrintLinefeed();
                i += 1
            }
            return
        }
    }
    let mut resourceIndex: uint8_t = 0i32 as uint8_t;
    let mut index: libc::c_int = 0i32;
    let mut pch: *mut libc::c_char = 0 as *mut libc::c_char;
    let mut saveptr: *mut libc::c_char = 0 as *mut libc::c_char;
    pch =
        strtok_r(cmdline, b" \x00" as *const u8 as *const libc::c_char,
                 &mut saveptr);
    resourceIndex = 0i32 as uint8_t;
    loop  {
        if resourceIndex as libc::c_ulong >=
               (::core::mem::size_of::<[cliResourceValue_t; 31]>() as
                    libc::c_ulong).wrapping_div(::core::mem::size_of::<cliResourceValue_t>()
                                                    as libc::c_ulong) {
            cliPrintErrorLinef(b"Invalid\x00" as *const u8 as
                                   *const libc::c_char);
            return
        }
        if strncasecmp(pch,
                       ownerNames[resourceTable[resourceIndex as usize].owner
                                      as usize], len as libc::c_ulong) == 0i32
           {
            break ;
        }
        resourceIndex = resourceIndex.wrapping_add(1)
    }
    pch =
        strtok_r(0 as *mut libc::c_char,
                 b" \x00" as *const u8 as *const libc::c_char, &mut saveptr);
    index = atoi(pch);
    if resourceTable[resourceIndex as usize].maxIndex as libc::c_int > 0i32 ||
           index > 0i32 {
        if index <= 0i32 ||
               index >
                   (if resourceTable[resourceIndex as usize].maxIndex as
                           libc::c_int == 0i32 {
                        1i32
                    } else {
                        resourceTable[resourceIndex as usize].maxIndex as
                            libc::c_int
                    }) {
            cliShowArgumentRangeError(b"index\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char, 1i32,
                                      if resourceTable[resourceIndex as
                                                           usize].maxIndex as
                                             libc::c_int == 0i32 {
                                          1i32
                                      } else {
                                          resourceTable[resourceIndex as
                                                            usize].maxIndex as
                                              libc::c_int
                                      });
            return
        }
        index -= 1i32;
        pch =
            strtok_r(0 as *mut libc::c_char,
                     b" \x00" as *const u8 as *const libc::c_char,
                     &mut saveptr)
    }
    let mut tag: *mut ioTag_t =
        getIoTag(resourceTable[resourceIndex as usize], index as uint8_t);
    if strlen(pch) > 0i32 as libc::c_ulong {
        if strToPin(pch, tag) {
            if *tag as libc::c_int == 0i32 {
                cliPrintLine(b"Freed\x00" as *const u8 as
                                 *const libc::c_char);
                return
            } else {
                let mut rec: *mut ioRec_t = IO_Rec(IOGetByTag(*tag));
                if !rec.is_null() {
                    resourceCheck(resourceIndex, index as uint8_t, *tag);
                    cliPrintLinef(b" %c%02d set\x00" as *const u8 as
                                      *const libc::c_char,
                                  IO_GPIOPortIdx(rec as IO_t) + 'A' as i32,
                                  IO_GPIOPinIdx(rec as IO_t));
                } else { cliShowParseError(); }
                return
            }
        }
    }
    cliShowParseError();
}
unsafe extern "C" fn printDma() {
    cliPrintLinefeed();
    cliPrintLine(b"DMA:\x00" as *const u8 as *const libc::c_char);
    let mut i: libc::c_int = 1i32;
    while i <= DMA_LAST_HANDLER as libc::c_int {
        let mut owner: *const libc::c_char = 0 as *const libc::c_char;
        owner = ownerNames[dmaGetOwner(i as dmaIdentifier_e) as usize];
        cliPrintf(b"DMA%d Channel %d:\x00" as *const u8 as
                      *const libc::c_char, (i - 1i32) / 7i32 + 1i32,
                  (i - 1i32) % 7i32 + 1i32);
        let mut resourceIndex: uint8_t =
            dmaGetResourceIndex(i as dmaIdentifier_e);
        if resourceIndex as libc::c_int > 0i32 {
            cliPrintLinef(b" %s %d\x00" as *const u8 as *const libc::c_char,
                          owner, resourceIndex as libc::c_int);
        } else {
            cliPrintLinef(b" %s\x00" as *const u8 as *const libc::c_char,
                          owner);
        }
        i += 1
    };
}
unsafe extern "C" fn cliDma(mut cmdLine: *mut libc::c_char) { printDma(); }
/* USE_RESOURCE_MGMT */
unsafe extern "C" fn printConfig(mut cmdline: *mut libc::c_char,
                                 mut doDiff: bool) {
    let mut dumpMask: uint8_t = DUMP_MASTER as libc::c_int as uint8_t;
    let mut options: *mut libc::c_char = 0 as *mut libc::c_char;
    options =
        checkCommand(cmdline,
                     b"master\x00" as *const u8 as *const libc::c_char);
    if !options.is_null() {
        dumpMask = DUMP_MASTER as libc::c_int as uint8_t
        // only
    } else {
        options =
            checkCommand(cmdline,
                         b"profile\x00" as *const u8 as *const libc::c_char);
        if !options.is_null() {
            dumpMask = DUMP_PROFILE as libc::c_int as uint8_t
            // only
        } else {
            options =
                checkCommand(cmdline,
                             b"rates\x00" as *const u8 as
                                 *const libc::c_char);
            if !options.is_null() {
                dumpMask = DUMP_RATES as libc::c_int as uint8_t
                // only
            } else {
                options =
                    checkCommand(cmdline,
                                 b"all\x00" as *const u8 as
                                     *const libc::c_char);
                if !options.is_null() {
                    dumpMask = DUMP_ALL as libc::c_int as uint8_t
                    // all profiles and rates
                } else { options = cmdline }
            }
        }
    }
    if doDiff {
        dumpMask =
            (dumpMask as libc::c_int | DO_DIFF as libc::c_int) as uint8_t
    }
    backupAndResetConfigs();
    if !checkCommand(options,
                     b"defaults\x00" as *const u8 as
                         *const libc::c_char).is_null() {
        dumpMask =
            (dumpMask as libc::c_int | SHOW_DEFAULTS as libc::c_int) as
                uint8_t
        // add default values as comments for changed values
    }
    if dumpMask as libc::c_int & DUMP_MASTER as libc::c_int != 0 ||
           dumpMask as libc::c_int & DUMP_ALL as libc::c_int != 0 {
        cliVersion(0 as *mut libc::c_char);
        cliPrintLinefeed();
        if dumpMask as libc::c_int & DUMP_ALL as libc::c_int != 0 {
            cliMcuId(0 as *mut libc::c_char);
        }
        if dumpMask as libc::c_int &
               (DUMP_ALL as libc::c_int | DO_DIFF as libc::c_int) ==
               DUMP_ALL as libc::c_int | DO_DIFF as libc::c_int {
            cliPrint(b"defaults nosave\x00" as *const u8 as
                         *const libc::c_char);
            cliPrintLinefeed();
        }
        printName(dumpMask, &mut pilotConfig_Copy);
        printResource(dumpMask);
        let equalsDefault: bool =
            mixerConfig_Copy.mixerMode as libc::c_int ==
                (*mixerConfig()).mixerMode as libc::c_int;
        let mut formatMixer: *const libc::c_char =
            b"mixer %s\x00" as *const u8 as *const libc::c_char;
        cliDefaultPrintLinef(dumpMask, equalsDefault, formatMixer,
                             mixerNames[((*mixerConfig()).mixerMode as
                                             libc::c_int - 1i32) as usize]);
        cliDumpPrintLinef(dumpMask, equalsDefault, formatMixer,
                          mixerNames[(mixerConfig_Copy.mixerMode as
                                          libc::c_int - 1i32) as usize]);
        cliDumpPrintLinef(dumpMask,
                          (*customMotorMixer(0i32)).throttle == 0.0f32,
                          b"\r\nmmix reset\r\n\x00" as *const u8 as
                              *const libc::c_char);
        printMotorMix(dumpMask, customMotorMixer_CopyArray.as_mut_ptr(),
                      customMotorMixer(0i32));
        printServo(dumpMask, servoParams_CopyArray.as_mut_ptr(),
                   servoParams(0i32));
        // print custom servo mixer if exists
        cliDumpPrintLinef(dumpMask,
                          (*customServoMixers(0i32)).rate as libc::c_int ==
                              0i32,
                          b"smix reset\r\n\x00" as *const u8 as
                              *const libc::c_char);
        printServoMix(dumpMask, customServoMixers_CopyArray.as_mut_ptr(),
                      customServoMixers(0i32));
        printFeature(dumpMask, &mut featureConfig_Copy, featureConfig());
        printBeeper(dumpMask, beeperConfig_Copy.beeper_off_flags,
                    (*beeperConfig()).beeper_off_flags,
                    b"beeper\x00" as *const u8 as *const libc::c_char);
        printBeeper(dumpMask, beeperConfig_Copy.dshotBeaconOffFlags,
                    (*beeperConfig()).dshotBeaconOffFlags,
                    b"beacon\x00" as *const u8 as *const libc::c_char);
        // USE_BEEPER
        printMap(dumpMask, &mut rxConfig_Copy, rxConfig());
        printSerial(dumpMask, &mut serialConfig_Copy, serialConfig());
        printLed(dumpMask, ledStripConfig_Copy.ledConfigs.as_mut_ptr(),
                 (*ledStripConfig()).ledConfigs.as_ptr());
        printColor(dumpMask, ledStripConfig_Copy.colors.as_mut_ptr(),
                   (*ledStripConfig()).colors.as_ptr());
        printModeColor(dumpMask, &mut ledStripConfig_Copy, ledStripConfig());
        printAux(dumpMask, modeActivationConditions_CopyArray.as_mut_ptr(),
                 modeActivationConditions(0i32));
        printAdjustmentRange(dumpMask,
                             adjustmentRanges_CopyArray.as_mut_ptr(),
                             adjustmentRanges(0i32));
        printRxRange(dumpMask, rxChannelRangeConfigs_CopyArray.as_mut_ptr(),
                     rxChannelRangeConfigs(0i32));
        printVtx(dumpMask, &mut vtxConfig_Copy, vtxConfig());
        printRxFailsafe(dumpMask,
                        rxFailsafeChannelConfigs_CopyArray.as_mut_ptr(),
                        rxFailsafeChannelConfigs(0i32));
        dumpAllValues(MASTER_VALUE as libc::c_int as uint16_t, dumpMask);
        if dumpMask as libc::c_int & DUMP_ALL as libc::c_int != 0 {
            let mut pidProfileIndex: uint32_t = 0i32 as uint32_t;
            while pidProfileIndex < 3i32 as libc::c_uint {
                cliDumpPidProfile(pidProfileIndex as uint8_t, dumpMask);
                pidProfileIndex = pidProfileIndex.wrapping_add(1)
            }
            pidProfileIndexToUse =
                systemConfig_Copy.pidProfileIndex as int8_t;
            cliProfile(b"\x00" as *const u8 as *const libc::c_char as
                           *mut libc::c_char);
            pidProfileIndexToUse = -1i32 as int8_t;
            let mut rateIndex: uint32_t = 0i32 as uint32_t;
            while rateIndex < 6i32 as libc::c_uint {
                cliDumpRateProfile(rateIndex as uint8_t, dumpMask);
                rateIndex = rateIndex.wrapping_add(1)
            }
            rateProfileIndexToUse =
                systemConfig_Copy.activeRateProfile as int8_t;
            cliRateProfile(b"\x00" as *const u8 as *const libc::c_char as
                               *mut libc::c_char);
            rateProfileIndexToUse = -1i32 as int8_t;
            cliPrint(b"save\x00" as *const u8 as *const libc::c_char);
        } else {
            cliDumpPidProfile(systemConfig_Copy.pidProfileIndex, dumpMask);
            cliDumpRateProfile(systemConfig_Copy.activeRateProfile, dumpMask);
        }
    }
    if dumpMask as libc::c_int & DUMP_PROFILE as libc::c_int != 0 {
        cliDumpPidProfile(systemConfig_Copy.pidProfileIndex, dumpMask);
    }
    if dumpMask as libc::c_int & DUMP_RATES as libc::c_int != 0 {
        cliDumpRateProfile(systemConfig_Copy.activeRateProfile, dumpMask);
    }
    // restore configs from copies
    restoreConfigs();
}
unsafe extern "C" fn cliDump(mut cmdline: *mut libc::c_char) {
    printConfig(cmdline, 0i32 != 0);
}
unsafe extern "C" fn cliDiff(mut cmdline: *mut libc::c_char) {
    printConfig(cmdline, 1i32 != 0);
}
// should be sorted a..z for bsearch()
#[no_mangle]
pub static mut cmdTable: [clicmd_t; 43] =
    unsafe {
        [{
             let mut init =
                 clicmd_t{name:
                              b"adjrange\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliAdjustmentRange as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"aux\x00" as *const u8 as *const libc::c_char,
                          func:
                              Some(cliAux as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"beacon\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliBeacon as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"beeper\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliBeeper as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name: b"bl\x00" as *const u8 as *const libc::c_char,
                          func:
                              Some(cliBootloader as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"color\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliColor as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"defaults\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliDefaults as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"diff\x00" as *const u8 as *const libc::c_char,
                          func:
                              Some(cliDiff as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"dma\x00" as *const u8 as *const libc::c_char,
                          func:
                              Some(cliDma as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"dshotprog\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliDshotProg as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"dump\x00" as *const u8 as *const libc::c_char,
                          func:
                              Some(cliDump as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"escprog\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliEscPassthrough as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"exit\x00" as *const u8 as *const libc::c_char,
                          func:
                              Some(cliExit as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"feature\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliFeature as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"flash_erase\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliFlashErase as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"flash_info\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliFlashInfo as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"get\x00" as *const u8 as *const libc::c_char,
                          func:
                              Some(cliGet as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"gyroregisters\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliDumpGyroRegisters as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"help\x00" as *const u8 as *const libc::c_char,
                          func:
                              Some(cliHelp as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"led\x00" as *const u8 as *const libc::c_char,
                          func:
                              Some(cliLed as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"map\x00" as *const u8 as *const libc::c_char,
                          func:
                              Some(cliMap as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"mcu_id\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliMcuId as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"mixer\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliMixer as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"mmix\x00" as *const u8 as *const libc::c_char,
                          func:
                              Some(cliMotorMix as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"mode_color\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliModeColor as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"motor\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliMotor as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"name\x00" as *const u8 as *const libc::c_char,
                          func:
                              Some(cliName as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"profile\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliProfile as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"rateprofile\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliRateProfile as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"rc_smoothing_info\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliRcSmoothing as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"resource\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliResource as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"rxfail\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliRxFailsafe as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"rxrange\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliRxRange as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"save\x00" as *const u8 as *const libc::c_char,
                          func:
                              Some(cliSave as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"serial\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliSerial as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"serialpassthrough\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliSerialPassthrough as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"servo\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliServo as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"set\x00" as *const u8 as *const libc::c_char,
                          func:
                              Some(cliSet as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"smix\x00" as *const u8 as *const libc::c_char,
                          func:
                              Some(cliServoMix as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"status\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliStatus as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"tasks\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliTasks as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"version\x00" as *const u8 as
                                  *const libc::c_char,
                          func:
                              Some(cliVersion as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         },
         {
             let mut init =
                 clicmd_t{name:
                              b"vtx\x00" as *const u8 as *const libc::c_char,
                          func:
                              Some(cliVtx as
                                       unsafe extern "C" fn(_:
                                                                *mut libc::c_char)
                                           -> ()),};
             init
         }]
    };
unsafe extern "C" fn cliHelp(mut cmdline: *mut libc::c_char) {
    let mut i: uint32_t = 0i32 as uint32_t;
    while (i as libc::c_ulong) <
              (::core::mem::size_of::<[clicmd_t; 43]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<clicmd_t>()
                                                   as libc::c_ulong) {
        cliPrint(cmdTable[i as usize].name);
        cliPrintLinefeed();
        i = i.wrapping_add(1)
    };
}
#[no_mangle]
pub unsafe extern "C" fn cliProcess() {
    if cliWriter.is_null() { return }
    // Be a little bit tricky.  Flush the last inputs buffer, if any.
    bufWriterFlush(cliWriter);
    while serialRxBytesWaiting(cliPort) != 0 {
        let mut c: uint8_t = serialRead(cliPort);
        if c as libc::c_int == '\t' as i32 || c as libc::c_int == '?' as i32 {
            // do tab completion
            let mut cmd: *const clicmd_t = 0 as *const clicmd_t;
            let mut pstart: *const clicmd_t = 0 as *const clicmd_t;
            let mut pend: *const clicmd_t = 0 as *const clicmd_t;
            let mut i: uint32_t = bufferIndex;
            cmd = cmdTable.as_ptr();
            while cmd <
                      cmdTable.as_ptr().offset((::core::mem::size_of::<[clicmd_t; 43]>()
                                                    as
                                                    libc::c_ulong).wrapping_div(::core::mem::size_of::<clicmd_t>()
                                                                                    as
                                                                                    libc::c_ulong)
                                                   as isize) {
                if !(bufferIndex != 0 &&
                         strncasecmp(cliBuffer.as_mut_ptr(), (*cmd).name,
                                     bufferIndex as libc::c_ulong) != 0i32) {
                    if pstart.is_null() { pstart = cmd }
                    pend = cmd
                }
                cmd = cmd.offset(1)
            }
            if !pstart.is_null() {
                /* Buffer matches one or more commands */
                while !(*(*pstart).name.offset(bufferIndex as isize) as
                            libc::c_int !=
                            *(*pend).name.offset(bufferIndex as isize) as
                                libc::c_int) {
                    if *(*pstart).name.offset(bufferIndex as isize) == 0 &&
                           (bufferIndex as libc::c_ulong) <
                               (::core::mem::size_of::<[libc::c_char; 256]>()
                                    as
                                    libc::c_ulong).wrapping_sub(2i32 as
                                                                    libc::c_ulong)
                       {
                        /* Unambiguous -- append a space */
                        let fresh12 = bufferIndex;
                        bufferIndex = bufferIndex.wrapping_add(1);
                        cliBuffer[fresh12 as usize] =
                            ' ' as i32 as libc::c_char;
                        cliBuffer[bufferIndex as usize] =
                            '\u{0}' as i32 as libc::c_char;
                        break ;
                    } else {
                        cliBuffer[bufferIndex as usize] =
                            *(*pstart).name.offset(bufferIndex as isize);
                        bufferIndex = bufferIndex.wrapping_add(1)
                    }
                }
            }
            if bufferIndex == 0 || pstart != pend {
                /* Print list of ambiguous matches */
                cliPrint(b"\r\x1b[K\x00" as *const u8 as *const libc::c_char);
                cmd = pstart;
                while cmd <= pend {
                    cliPrint((*cmd).name);
                    cliWrite('\t' as i32 as uint8_t);
                    cmd = cmd.offset(1)
                }
                cliPrompt();
                i = 0i32 as uint32_t
                /* Redraw prompt */
            }
            while i < bufferIndex {
                cliWrite(cliBuffer[i as usize] as uint8_t);
                i = i.wrapping_add(1)
            }
        } else if bufferIndex == 0 && c as libc::c_int == 4i32 {
            // CTRL-D
            cliExit(cliBuffer.as_mut_ptr());
            return
        } else if c as libc::c_int == 12i32 {
            // NewPage / CTRL-L
            // clear screen
            cliPrint(b"\x1b[2J\x1b[1;1H\x00" as *const u8 as
                         *const libc::c_char);
            cliPrompt();
        } else if bufferIndex != 0 &&
                      (c as libc::c_int == '\n' as i32 ||
                           c as libc::c_int == '\r' as i32) {
            // enter pressed
            cliPrintLinefeed();
            // Strip comment starting with # from line
            let mut p: *mut libc::c_char = cliBuffer.as_mut_ptr();
            p = strchr(p, '#' as i32);
            if !p.is_null() {
                bufferIndex =
                    p.wrapping_offset_from(cliBuffer.as_mut_ptr()) as
                        libc::c_long as uint32_t
            }
            // Strip trailing whitespace
            while bufferIndex > 0i32 as libc::c_uint &&
                      cliBuffer[bufferIndex.wrapping_sub(1i32 as libc::c_uint)
                                    as usize] as libc::c_int == ' ' as i32 {
                bufferIndex = bufferIndex.wrapping_sub(1)
            }
            // Process non-empty lines
            if bufferIndex > 0i32 as libc::c_uint {
                cliBuffer[bufferIndex as usize] =
                    0i32 as libc::c_char; // null terminate
                let mut cmd_0: *const clicmd_t = 0 as *const clicmd_t;
                let mut options: *mut libc::c_char = 0 as *mut libc::c_char;
                cmd_0 = cmdTable.as_ptr();
                while cmd_0 <
                          cmdTable.as_ptr().offset((::core::mem::size_of::<[clicmd_t; 43]>()
                                                        as
                                                        libc::c_ulong).wrapping_div(::core::mem::size_of::<clicmd_t>()
                                                                                        as
                                                                                        libc::c_ulong)
                                                       as isize) {
                    options =
                        checkCommand(cliBuffer.as_mut_ptr(), (*cmd_0).name);
                    if !options.is_null() { break ; }
                    cmd_0 = cmd_0.offset(1)
                }
                if cmd_0 <
                       cmdTable.as_ptr().offset((::core::mem::size_of::<[clicmd_t; 43]>()
                                                     as
                                                     libc::c_ulong).wrapping_div(::core::mem::size_of::<clicmd_t>()
                                                                                     as
                                                                                     libc::c_ulong)
                                                    as isize) {
                    (*cmd_0).func.expect("non-null function pointer")(options);
                } else {
                    cliPrint(b"Unknown command, try \'help\'\x00" as *const u8
                                 as *const libc::c_char);
                }
                bufferIndex = 0i32 as uint32_t
            }
            memset(cliBuffer.as_mut_ptr() as *mut libc::c_void, 0i32,
                   ::core::mem::size_of::<[libc::c_char; 256]>() as
                       libc::c_ulong);
            // 'exit' will reset this flag, so we don't need to print prompt again
            if cliMode == 0 { return }
            cliPrompt();
        } else if c as libc::c_int == 127i32 {
            // backspace
            if bufferIndex != 0 {
                bufferIndex =
                    bufferIndex.wrapping_sub(1); // Ignore leading spaces
                cliBuffer[bufferIndex as usize] = 0i32 as libc::c_char;
                cliPrint(b"\x08 \x08\x00" as *const u8 as
                             *const libc::c_char);
            }
        } else {
            if !((bufferIndex as libc::c_ulong) <
                     ::core::mem::size_of::<[libc::c_char; 256]>() as
                         libc::c_ulong && c as libc::c_int >= 32i32 &&
                     c as libc::c_int <= 126i32) {
                continue ;
            }
            if bufferIndex == 0 && c as libc::c_int == ' ' as i32 {
                continue ;
            }
            let fresh13 = bufferIndex;
            bufferIndex = bufferIndex.wrapping_add(1);
            cliBuffer[fresh13 as usize] = c as libc::c_char;
            cliWrite(c);
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn cliEnter(mut serialPort: *mut serialPort_t) {
    cliMode = 1i32 as uint8_t;
    cliPort = serialPort;
    setPrintfSerialPort(cliPort);
    cliWriter =
        bufWriterInit(cliWriteBuffer.as_mut_ptr(),
                      ::core::mem::size_of::<[uint8_t; 88]>() as libc::c_ulong
                          as libc::c_int,
                      ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                               *mut libc::c_void,
                                                                           _:
                                                                               *const uint8_t,
                                                                           _:
                                                                               libc::c_int)
                                                          -> ()>,
                                               bufWrite_t>(Some(serialWriteBufShim
                                                                    as
                                                                    unsafe extern "C" fn(_:
                                                                                             *mut libc::c_void,
                                                                                         _:
                                                                                             *const uint8_t,
                                                                                         _:
                                                                                             libc::c_int)
                                                                        ->
                                                                            ())),
                      serialPort as *mut libc::c_void);
    schedulerSetCalulateTaskStatistics((*systemConfig()).task_statistics !=
                                           0);
    cliPrintLine(b"\r\nCLI\x00" as *const u8 as *const libc::c_char);
    cliPrompt();
    setArmingDisabled(ARMING_DISABLED_CLI);
}
#[no_mangle]
pub unsafe extern "C" fn cliInit(mut serialConfig_0: *const serialConfig_t) {
}
// USE_CLI
