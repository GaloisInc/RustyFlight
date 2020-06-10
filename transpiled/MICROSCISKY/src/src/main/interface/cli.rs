use ::libc;
extern "C" {
    #[no_mangle]
    fn atoi(__nptr: *const libc::c_char) -> libc::c_int;
    #[no_mangle]
    fn ffs(__i: libc::c_int) -> libc::c_int;
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
    fn strtok_r(__s: *mut libc::c_char, __delim: *const libc::c_char,
                __save_ptr: *mut *mut libc::c_char) -> *mut libc::c_char;
    #[no_mangle]
    fn strcasestr(__haystack: *const libc::c_char,
                  __needle: *const libc::c_char) -> *mut libc::c_char;
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
    #[no_mangle]
    fn strncpy(_: *mut libc::c_char, _: *const libc::c_char, _: libc::c_ulong)
     -> *mut libc::c_char;
    #[no_mangle]
    fn memcmp(_: *const libc::c_void, _: *const libc::c_void,
              _: libc::c_ulong) -> libc::c_int;
    #[no_mangle]
    fn strcasecmp(_: *const libc::c_char, _: *const libc::c_char)
     -> libc::c_int;
    #[no_mangle]
    fn strncasecmp(_: *const libc::c_char, _: *const libc::c_char,
                   _: libc::c_ulong) -> libc::c_int;
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn isspace(_: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn toupper(_: libc::c_int) -> libc::c_int;
    /* *
  ******************************************************************************
  * @file    system_stm32f10x.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Header File.
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
    /* * @addtogroup CMSIS
  * @{
  */
    /* * @addtogroup stm32f10x_system
  * @{
  */
    /* *
  * @brief Define to prevent recursive inclusion
  */
    /* * @addtogroup STM32F10x_System_Includes
  * @{
  */
    /* *
  * @}
  */
    /* * @addtogroup STM32F10x_System_Exported_types
  * @{
  */
    #[no_mangle]
    static mut SystemCoreClock: uint32_t;
    // increment when a major release is made (big new feature, etc)
    // increment when a minor release is made (small new feature, change etc)
    // increment when a bug is fixed
    // lower case hexadecimal digits.
    // "MMM DD YYYY" MMM = Jan/Feb/...
    #[no_mangle]
    static buildTime: *const libc::c_char;
    #[no_mangle]
    static buildDate: *const libc::c_char;
    #[no_mangle]
    static shortGitRevision: *const libc::c_char;
    #[no_mangle]
    static targetName: *const libc::c_char;
    #[no_mangle]
    static mut __config_start: uint8_t;
    // configured via linker script when building binaries.
    #[no_mangle]
    static mut __config_end: uint8_t;
    #[no_mangle]
    static __pg_registry_start: [pgRegistry_t; 0];
    #[no_mangle]
    static __pg_registry_end: [pgRegistry_t; 0];
    #[no_mangle]
    fn pgFind(pgn: pgn_t) -> *const pgRegistry_t;
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
    static mut pilotConfig_Copy: pilotConfig_t;
    #[no_mangle]
    static mut pilotConfig_System: pilotConfig_t;
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
    static mut motor_disarmed: [libc::c_float; 4];
    #[no_mangle]
    fn getMotorCount() -> uint8_t;
    #[no_mangle]
    fn mixerResetDisarmedMotors();
    #[no_mangle]
    fn stopPwmAllMotors();
    #[no_mangle]
    fn convertExternalToMotor(externalValue: uint16_t) -> libc::c_float;
    #[no_mangle]
    static mut pidConfig_System: pidConfig_t;
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
    fn beeperModeMaskForTableIndex(idx: libc::c_int) -> uint32_t;
    #[no_mangle]
    fn beeperNameForTableIndex(idx: libc::c_int) -> *const libc::c_char;
    #[no_mangle]
    fn beeperTableEntryCount() -> libc::c_int;
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
                         leftC: Option<serialConsumer>,
                         rightC: Option<serialConsumer>);
    #[no_mangle]
    static mut beeperConfig_Copy: beeperConfig_t;
    #[no_mangle]
    static mut beeperConfig_System: beeperConfig_t;
    #[no_mangle]
    static mut rxConfig_Copy: rxConfig_t;
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    static rcChannelLetters: [libc::c_char; 0];
    #[no_mangle]
    static mut rxFailsafeChannelConfigs_CopyArray:
           [rxFailsafeChannelConfig_t; 18];
    #[no_mangle]
    static mut rxFailsafeChannelConfigs_SystemArray:
           [rxFailsafeChannelConfig_t; 18];
    #[no_mangle]
    static mut rxChannelRangeConfigs_CopyArray: [rxChannelRangeConfig_t; 4];
    #[no_mangle]
    static mut rxChannelRangeConfigs_SystemArray: [rxChannelRangeConfig_t; 4];
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
#[derive(Copy, Clone)]
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
pub type size_t = libc::c_ulong;
pub type va_list = __builtin_va_list;
/* * 
  * @brief General Purpose I/O
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub CRL: uint32_t,
    pub CRH: uint32_t,
    pub IDR: uint32_t,
    pub ODR: uint32_t,
    pub BSRR: uint32_t,
    pub BRR: uint32_t,
    pub LCKR: uint32_t,
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
pub type pgn_t = uint16_t;
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
    pub reset: C2RustUnnamed,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
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
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
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
// 0 - 359
// 0 - 255
// 0 - 255
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct featureConfig_s {
    pub enabledFeatures: uint32_t,
}
pub type featureConfig_t = featureConfig_s;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct bufWriter_s {
    pub writer: bufWrite_t,
    pub arg: *mut libc::c_void,
    pub capacity: uint8_t,
    pub at: uint8_t,
    pub data: [uint8_t; 0],
}
pub type bufWriter_t = bufWriter_s;
#[derive(Copy, Clone)]
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
pub const DMA_LAST_HANDLER: dmaIdentifier_e = 7;
pub const DMA1_CH7_HANDLER: dmaIdentifier_e = 7;
pub const DMA1_CH6_HANDLER: dmaIdentifier_e = 6;
pub const DMA1_CH5_HANDLER: dmaIdentifier_e = 5;
pub const DMA1_CH4_HANDLER: dmaIdentifier_e = 4;
pub const DMA1_CH3_HANDLER: dmaIdentifier_e = 3;
pub const DMA1_CH2_HANDLER: dmaIdentifier_e = 2;
pub const DMA1_CH1_HANDLER: dmaIdentifier_e = 1;
pub const DMA_NONE: dmaIdentifier_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ioRec_s {
    pub gpio: *mut GPIO_TypeDef,
    pub pin: uint16_t,
    pub owner: resourceOwner_e,
    pub index: uint8_t,
}
pub type ioRec_t = ioRec_s;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pilotConfig_s {
    pub name: [libc::c_char; 17],
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
pub struct channelRange_s {
    pub startStep: uint8_t,
    pub endStep: uint8_t,
}
// in seconds
// Breakpoint where TPA is activated
// Sets the throttle limiting type - off, scale or clip
// Sets the maximum pilot commanded throttle limit
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
pub type C2RustUnnamed_1 = libc::c_uint;
pub const ADJUSTMENT_FUNCTION_COUNT: C2RustUnnamed_1 = 33;
pub const ADJUSTMENT_YAW_F: C2RustUnnamed_1 = 32;
pub const ADJUSTMENT_ROLL_F: C2RustUnnamed_1 = 31;
pub const ADJUSTMENT_PITCH_F: C2RustUnnamed_1 = 30;
pub const ADJUSTMENT_PID_AUDIO: C2RustUnnamed_1 = 29;
pub const ADJUSTMENT_PITCH_RC_EXPO: C2RustUnnamed_1 = 28;
pub const ADJUSTMENT_ROLL_RC_EXPO: C2RustUnnamed_1 = 27;
pub const ADJUSTMENT_PITCH_RC_RATE: C2RustUnnamed_1 = 26;
pub const ADJUSTMENT_ROLL_RC_RATE: C2RustUnnamed_1 = 25;
pub const ADJUSTMENT_HORIZON_STRENGTH: C2RustUnnamed_1 = 24;
pub const ADJUSTMENT_FEEDFORWARD_TRANSITION: C2RustUnnamed_1 = 23;
pub const ADJUSTMENT_PITCH_ROLL_F: C2RustUnnamed_1 = 22;
pub const ADJUSTMENT_RC_RATE_YAW: C2RustUnnamed_1 = 21;
pub const ADJUSTMENT_ROLL_D: C2RustUnnamed_1 = 20;
pub const ADJUSTMENT_ROLL_I: C2RustUnnamed_1 = 19;
pub const ADJUSTMENT_ROLL_P: C2RustUnnamed_1 = 18;
pub const ADJUSTMENT_PITCH_D: C2RustUnnamed_1 = 17;
pub const ADJUSTMENT_PITCH_I: C2RustUnnamed_1 = 16;
pub const ADJUSTMENT_PITCH_P: C2RustUnnamed_1 = 15;
pub const ADJUSTMENT_ROLL_RATE: C2RustUnnamed_1 = 14;
pub const ADJUSTMENT_PITCH_RATE: C2RustUnnamed_1 = 13;
pub const ADJUSTMENT_RATE_PROFILE: C2RustUnnamed_1 = 12;
pub const ADJUSTMENT_YAW_D: C2RustUnnamed_1 = 11;
pub const ADJUSTMENT_YAW_I: C2RustUnnamed_1 = 10;
pub const ADJUSTMENT_YAW_P: C2RustUnnamed_1 = 9;
pub const ADJUSTMENT_PITCH_ROLL_D: C2RustUnnamed_1 = 8;
pub const ADJUSTMENT_PITCH_ROLL_I: C2RustUnnamed_1 = 7;
pub const ADJUSTMENT_PITCH_ROLL_P: C2RustUnnamed_1 = 6;
pub const ADJUSTMENT_YAW_RATE: C2RustUnnamed_1 = 5;
pub const ADJUSTMENT_PITCH_ROLL_RATE: C2RustUnnamed_1 = 4;
pub const ADJUSTMENT_THROTTLE_EXPO: C2RustUnnamed_1 = 3;
pub const ADJUSTMENT_RC_EXPO: C2RustUnnamed_1 = 2;
pub const ADJUSTMENT_RC_RATE: C2RustUnnamed_1 = 1;
pub const ADJUSTMENT_NONE: C2RustUnnamed_1 = 0;
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
pub type C2RustUnnamed_2 = libc::c_uint;
pub const INPUT_SOURCE_COUNT: C2RustUnnamed_2 = 14;
pub const INPUT_GIMBAL_ROLL: C2RustUnnamed_2 = 13;
pub const INPUT_GIMBAL_PITCH: C2RustUnnamed_2 = 12;
pub const INPUT_RC_AUX4: C2RustUnnamed_2 = 11;
pub const INPUT_RC_AUX3: C2RustUnnamed_2 = 10;
pub const INPUT_RC_AUX2: C2RustUnnamed_2 = 9;
pub const INPUT_RC_AUX1: C2RustUnnamed_2 = 8;
pub const INPUT_RC_THROTTLE: C2RustUnnamed_2 = 7;
pub const INPUT_RC_YAW: C2RustUnnamed_2 = 6;
pub const INPUT_RC_PITCH: C2RustUnnamed_2 = 5;
pub const INPUT_RC_ROLL: C2RustUnnamed_2 = 4;
pub const INPUT_STABILIZED_THROTTLE: C2RustUnnamed_2 = 3;
pub const INPUT_STABILIZED_YAW: C2RustUnnamed_2 = 2;
pub const INPUT_STABILIZED_PITCH: C2RustUnnamed_2 = 1;
pub const INPUT_STABILIZED_ROLL: C2RustUnnamed_2 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct clivalue_s {
    pub name: *const libc::c_char,
    pub type_0: uint8_t,
    pub config: cliValueConfig_t,
    pub pgn: pgn_t,
    pub offset: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union cliValueConfig_t {
    pub lookup: cliLookupTableConfig_t,
    pub minmax: cliMinMaxConfig_t,
    pub array: cliArrayLengthConfig_t,
    pub bitpos: uint8_t,
}
pub type cliArrayLengthConfig_t = cliArrayLengthConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct cliArrayLengthConfig_s {
    pub length: uint8_t,
}
pub type cliMinMaxConfig_t = cliMinMaxConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct cliMinMaxConfig_s {
    pub min: int16_t,
    pub max: int16_t,
}
pub type cliLookupTableConfig_t = cliLookupTableConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct cliLookupTableConfig_s {
    pub tableIndex: lookupTableIndex_e,
}
pub type lookupTableIndex_e = libc::c_uint;
pub const LOOKUP_TABLE_COUNT: lookupTableIndex_e = 27;
pub const TABLE_ACRO_TRAINER_DEBUG: lookupTableIndex_e = 26;
pub const TABLE_THROTTLE_LIMIT_TYPE: lookupTableIndex_e = 25;
pub const TABLE_RGB_GRB: lookupTableIndex_e = 24;
pub const TABLE_RATES_TYPE: lookupTableIndex_e = 23;
pub const TABLE_BUS_TYPE: lookupTableIndex_e = 22;
pub const TABLE_CRASH_RECOVERY: lookupTableIndex_e = 21;
pub const TABLE_FAILSAFE_SWITCH_MODE: lookupTableIndex_e = 20;
pub const TABLE_FAILSAFE: lookupTableIndex_e = 19;
pub const TABLE_ANTI_GRAVITY_MODE: lookupTableIndex_e = 18;
pub const TABLE_DTERM_LOWPASS_TYPE: lookupTableIndex_e = 17;
pub const TABLE_LOWPASS_TYPE: lookupTableIndex_e = 16;
pub const TABLE_RC_INTERPOLATION_CHANNELS: lookupTableIndex_e = 15;
pub const TABLE_RC_INTERPOLATION: lookupTableIndex_e = 14;
pub const TABLE_MOTOR_PWM_PROTOCOL: lookupTableIndex_e = 13;
pub const TABLE_DEBUG: lookupTableIndex_e = 12;
pub const TABLE_MAG_HARDWARE: lookupTableIndex_e = 11;
pub const TABLE_BARO_HARDWARE: lookupTableIndex_e = 10;
pub const TABLE_ACC_HARDWARE: lookupTableIndex_e = 9;
pub const TABLE_GYRO_HARDWARE_LPF: lookupTableIndex_e = 8;
pub const TABLE_SERIAL_RX: lookupTableIndex_e = 7;
pub const TABLE_VOLTAGE_METER: lookupTableIndex_e = 6;
pub const TABLE_CURRENT_METER: lookupTableIndex_e = 5;
pub const TABLE_BLACKBOX_MODE: lookupTableIndex_e = 4;
pub const TABLE_BLACKBOX_DEVICE: lookupTableIndex_e = 3;
pub const TABLE_ALIGNMENT: lookupTableIndex_e = 2;
pub const TABLE_UNIT: lookupTableIndex_e = 1;
pub const TABLE_OFF_ON: lookupTableIndex_e = 0;
pub type clivalue_t = clivalue_s;
pub const PROFILE_RATE_VALUE: C2RustUnnamed_4 = 16;
pub const PROFILE_VALUE: C2RustUnnamed_4 = 8;
pub const MASTER_VALUE: C2RustUnnamed_4 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialConfig_s {
    pub portConfigs: [serialPortConfig_t; 2],
    pub serial_update_rate_hz: uint16_t,
    pub reboot_character: uint8_t,
}
// see cliValueFlag_e
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
pub type serialConfig_t = serialConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct clicmd_t {
    pub name: *const libc::c_char,
    pub func: Option<unsafe extern "C" fn(_: *mut libc::c_char) -> ()>,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct cfCheckFuncInfo_t {
    pub maxExecutionTime: timeUs_t,
    pub totalExecutionTime: timeUs_t,
    pub averageExecutionTime: timeUs_t,
}
pub type cfTaskId_e = libc::c_uint;
pub const TASK_SELF: cfTaskId_e = 18;
pub const TASK_NONE: cfTaskId_e = 17;
pub const TASK_COUNT: cfTaskId_e = 17;
pub const TASK_LEDSTRIP: cfTaskId_e = 16;
pub const TASK_TELEMETRY: cfTaskId_e = 15;
pub const TASK_ALTITUDE: cfTaskId_e = 14;
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
#[derive(Copy, Clone)]
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
pub const MODE_BITSET: C2RustUnnamed_4 = 96;
pub type lookupTableEntry_t = lookupTableEntry_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct lookupTableEntry_s {
    pub values: *const *const libc::c_char,
    pub valueCount: uint8_t,
}
pub const MODE_LOOKUP: C2RustUnnamed_4 = 32;
pub const MODE_DIRECT: C2RustUnnamed_4 = 0;
pub const VAR_UINT32: C2RustUnnamed_4 = 4;
pub const VAR_INT16: C2RustUnnamed_4 = 3;
pub const VAR_UINT16: C2RustUnnamed_4 = 2;
pub const VAR_INT8: C2RustUnnamed_4 = 1;
pub const VAR_UINT8: C2RustUnnamed_4 = 0;
pub const MODE_ARRAY: C2RustUnnamed_4 = 64;
pub type rxChannelRangeConfig_t = rxChannelRangeConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxChannelRangeConfig_s {
    pub min: uint16_t,
    pub max: uint16_t,
}
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
pub type serialConsumer = unsafe extern "C" fn(_: uint8_t) -> ();
// not used for all telemetry systems, e.g. HoTT only works at 19200.
// which byte is used to reboot. Default 'R', could be changed carefully to something else.
// suppress LEDLOW mode if beeper is on
//
// runtime
//
pub type serialPortUsage_t = serialPortUsage_s;
#[derive(Copy, Clone)]
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
pub const DO_DIFF: C2RustUnnamed_6 = 16;
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
pub const DUMP_MASTER: C2RustUnnamed_6 = 1;
pub const SHOW_DEFAULTS: C2RustUnnamed_6 = 32;
pub type rxFailsafeChannelConfig_t = rxFailsafeChannelConfig_s;
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct cliResourceValue_t {
    pub owner: uint8_t,
    pub pgn: pgn_t,
    pub stride: uint8_t,
    pub offset: uint8_t,
    pub maxIndex: uint8_t,
}
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
pub type spiPinConfig_t = spiPinConfig_s;
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
pub type i2cConfig_t = i2cConfig_s;
pub const HIDE_UNUSED: C2RustUnnamed_6 = 64;
pub const COLOR: C2RustUnnamed_3 = 2;
pub const FUNCTION: C2RustUnnamed_3 = 1;
pub const MODE: C2RustUnnamed_3 = 0;
pub type ledModeIndex_e = libc::c_uint;
pub const LED_AUX_CHANNEL: ledModeIndex_e = 7;
pub const LED_SPECIAL: ledModeIndex_e = 6;
pub const LED_MODE_BARO: ledModeIndex_e = 5;
pub const LED_MODE_MAG: ledModeIndex_e = 4;
pub const LED_MODE_ANGLE: ledModeIndex_e = 3;
pub const LED_MODE_HORIZON: ledModeIndex_e = 2;
pub const LED_MODE_HEADFREE: ledModeIndex_e = 1;
pub const LED_MODE_ORIENTATION: ledModeIndex_e = 0;
pub const ARGS_COUNT: C2RustUnnamed_3 = 3;
pub type C2RustUnnamed_3 = libc::c_uint;
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
pub const DUMP_RATES: C2RustUnnamed_6 = 4;
pub const DUMP_PROFILE: C2RustUnnamed_6 = 2;
pub const DUMP_ALL: C2RustUnnamed_6 = 8;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct beeperConfig_s {
    pub beeper_off_flags: uint32_t,
    pub dshotBeaconTone: uint8_t,
    pub dshotBeaconOffFlags: uint32_t,
}
pub const BEEPER_RC_SMOOTHING_INIT_FAIL: C2RustUnnamed_5 = 23;
pub const BEEPER_CAM_CONNECTION_CLOSE: C2RustUnnamed_5 = 22;
pub const BEEPER_CAM_CONNECTION_OPEN: C2RustUnnamed_5 = 21;
pub const BEEPER_CRASH_FLIP_MODE: C2RustUnnamed_5 = 20;
pub const BEEPER_BLACKBOX_ERASE: C2RustUnnamed_5 = 19;
pub const BEEPER_USB: C2RustUnnamed_5 = 18;
pub const BEEPER_SYSTEM_INIT: C2RustUnnamed_5 = 17;
pub const BEEPER_ARMED: C2RustUnnamed_5 = 16;
pub const BEEPER_DISARM_REPEAT: C2RustUnnamed_5 = 15;
pub const BEEPER_MULTI_BEEPS: C2RustUnnamed_5 = 14;
pub const BEEPER_READY_BEEP: C2RustUnnamed_5 = 13;
pub const BEEPER_ACC_CALIBRATION_FAIL: C2RustUnnamed_5 = 12;
pub const BEEPER_ACC_CALIBRATION: C2RustUnnamed_5 = 11;
pub const BEEPER_RX_SET: C2RustUnnamed_5 = 10;
pub const BEEPER_GPS_STATUS: C2RustUnnamed_5 = 9;
pub const BEEPER_BAT_LOW: C2RustUnnamed_5 = 8;
pub const BEEPER_BAT_CRIT_LOW: C2RustUnnamed_5 = 7;
pub const BEEPER_ARMING_GPS_FIX: C2RustUnnamed_5 = 6;
pub const BEEPER_ARMING: C2RustUnnamed_5 = 5;
pub const BEEPER_DISARMING: C2RustUnnamed_5 = 4;
pub const BEEPER_RX_LOST_LANDING: C2RustUnnamed_5 = 3;
pub const BEEPER_RX_LOST: C2RustUnnamed_5 = 2;
pub const BEEPER_GYRO_CALIBRATED: C2RustUnnamed_5 = 1;
pub const BEEPER_ALL: C2RustUnnamed_5 = 24;
pub type C2RustUnnamed_4 = libc::c_uint;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const BEEPER_SILENCE: C2RustUnnamed_5 = 0;
pub type C2RustUnnamed_6 = libc::c_uint;
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
pub static mut cliMode: uint8_t = 0 as libc::c_int as uint8_t;
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
unsafe extern "C" fn pidConfig() -> *const pidConfig_t {
    return &mut pidConfig_System;
}
#[no_mangle]
pub static mut inputSource_e: C2RustUnnamed_2 = INPUT_STABILIZED_ROLL;
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
unsafe extern "C" fn beeperConfig() -> *const beeperConfig_t {
    return &mut beeperConfig_System;
}
#[inline]
unsafe extern "C" fn beeperConfigMutable() -> *mut beeperConfig_t {
    return &mut beeperConfig_System;
}
#[inline]
unsafe extern "C" fn rxConfigMutable() -> *mut rxConfig_t {
    return &mut rxConfig_System;
}
#[inline]
unsafe extern "C" fn rxConfig() -> *const rxConfig_t {
    return &mut rxConfig_System;
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
unsafe extern "C" fn rxFailsafeChannelConfigsMutable(mut _index: libc::c_int)
 -> *mut rxFailsafeChannelConfig_t {
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
static mut cliWriter: *mut bufWriter_t =
    0 as *const bufWriter_t as *mut bufWriter_t;
static mut cliWriteBuffer: [uint8_t; 88] = [0; 88];
static mut cliBuffer: [libc::c_char; 128] = [0; 128];
static mut bufferIndex: uint32_t = 0 as libc::c_int as uint32_t;
static mut configIsInCopy: bool = 0 as libc::c_int != 0;
static mut pidProfileIndexToUse: int8_t = -(1 as libc::c_int) as int8_t;
static mut rateProfileIndexToUse: int8_t = -(1 as libc::c_int) as int8_t;
// USE_BOARD_INFO
static mut emptyName: *const libc::c_char =
    b"-\x00" as *const u8 as *const libc::c_char;
static mut emptyString: *const libc::c_char =
    b"\x00" as *const u8 as *const libc::c_char;
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
    configIsInCopy = 1 as libc::c_int != 0;
}
unsafe extern "C" fn restoreConfigs() {
    let mut pg: *const pgRegistry_t = __pg_registry_start.as_ptr();
    while pg < __pg_registry_end.as_ptr() {
        restorePgConfig(pg);
        pg = pg.offset(1)
    }
    configIsInCopy = 0 as libc::c_int != 0;
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
        return 1 as libc::c_int != 0
    } else { return 0 as libc::c_int != 0 };
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
        return 1 as libc::c_int != 0
    } else { return 0 as libc::c_int != 0 };
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
    if (*var).type_0 as libc::c_int & 0x60 as libc::c_int ==
           MODE_ARRAY as libc::c_int {
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < (*var).config.array.length as libc::c_int {
            match (*var).type_0 as libc::c_int & 0x7 as libc::c_int {
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
            if i <
                   (*var).config.array.length as libc::c_int -
                       1 as libc::c_int {
                cliPrint(b",\x00" as *const u8 as *const libc::c_char);
            }
            i += 1
        }
    } else {
        let mut value: libc::c_int = 0 as libc::c_int;
        match (*var).type_0 as libc::c_int & 0x7 as libc::c_int {
            0 => { value = *(valuePointer as *mut uint8_t) as libc::c_int }
            1 => { value = *(valuePointer as *mut int8_t) as libc::c_int }
            2 | 3 => {
                value = *(valuePointer as *mut int16_t) as libc::c_int
            }
            4 => { value = *(valuePointer as *mut uint32_t) as libc::c_int }
            _ => { }
        }
        match (*var).type_0 as libc::c_int & 0x60 as libc::c_int {
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
                if value &
                       (1 as libc::c_int) <<
                           (*var).config.bitpos as libc::c_int != 0 {
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
    let mut result: bool = 1 as libc::c_int != 0;
    let mut elementCount: libc::c_int = 1 as libc::c_int;
    let mut mask: uint32_t = 0xffffffff as libc::c_uint;
    if (*var).type_0 as libc::c_int & 0x60 as libc::c_int ==
           MODE_ARRAY as libc::c_int {
        elementCount = (*var).config.array.length as libc::c_int
    }
    if (*var).type_0 as libc::c_int & 0x60 as libc::c_int ==
           MODE_BITSET as libc::c_int {
        mask =
            ((1 as libc::c_int) << (*var).config.bitpos as libc::c_int) as
                uint32_t
    }
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < elementCount {
        match (*var).type_0 as libc::c_int & 0x7 as libc::c_int {
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
    return if pidProfileIndexToUse as libc::c_int == -(1 as libc::c_int) {
               getCurrentPidProfileIndex() as libc::c_int
           } else { pidProfileIndexToUse as libc::c_int } as uint8_t;
}
unsafe extern "C" fn getRateProfileIndexToUse() -> uint8_t {
    return if rateProfileIndexToUse as libc::c_int == -(1 as libc::c_int) {
               getCurrentControlRateProfileIndex() as libc::c_int
           } else { rateProfileIndexToUse as libc::c_int } as uint8_t;
}
unsafe extern "C" fn getValueOffset(mut value: *const clivalue_t)
 -> uint16_t {
    match (*value).type_0 as libc::c_int & 0x18 as libc::c_int {
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
    return 0 as libc::c_int as uint16_t;
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
    if dumpMask as libc::c_int & DO_DIFF as libc::c_int == 0 as libc::c_int ||
           !equalsDefault {
        if dumpMask as libc::c_int & SHOW_DEFAULTS as libc::c_int != 0 &&
               !equalsDefault {
            cliPrintf(defaultFormat, (*value).name);
            printValuePointer(value,
                              (*pg).address.offset(valueOffset as isize) as
                                  *const libc::c_void, 0 as libc::c_int != 0);
            cliPrintLinefeed();
        }
        cliPrintf(format, (*value).name);
        printValuePointer(value,
                          (*pg).copy.offset(valueOffset as isize) as
                              *const libc::c_void, 0 as libc::c_int != 0);
        cliPrintLinefeed();
    };
}
unsafe extern "C" fn dumpAllValues(mut valueSection: uint16_t,
                                   mut dumpMask: uint8_t) {
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    while i < valueTableEntryCount as libc::c_uint {
        let mut value: *const clivalue_t =
            &*valueTable.as_ptr().offset(i as isize) as *const clivalue_t;
        bufWriterFlush(cliWriter);
        if (*value).type_0 as libc::c_int & 0x18 as libc::c_int ==
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
    match (*var).type_0 as libc::c_int & 0x60 as libc::c_int {
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
            let mut firstEntry: bool = 1 as libc::c_int != 0;
            let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
            while i < (*tableEntry).valueCount as libc::c_uint {
                if !(*(*tableEntry).values.offset(i as isize)).is_null() {
                    if !firstEntry {
                        cliPrint(b", \x00" as *const u8 as
                                     *const libc::c_char);
                    }
                    cliPrintf(b"%s\x00" as *const u8 as *const libc::c_char,
                              *(*tableEntry).values.offset(i as isize));
                    firstEntry = 0 as libc::c_int != 0
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
    if (*var).type_0 as libc::c_int & 0x60 as libc::c_int ==
           MODE_BITSET as libc::c_int {
        match (*var).type_0 as libc::c_int & 0x7 as libc::c_int {
            0 => {
                mask =
                    ((1 as libc::c_int) << (*var).config.bitpos as libc::c_int
                         & 0xff as libc::c_int) as uint32_t;
                if value != 0 {
                    workValue = *(ptr as *mut uint8_t) as libc::c_uint | mask
                } else {
                    workValue = *(ptr as *mut uint8_t) as libc::c_uint & !mask
                }
                *(ptr as *mut uint8_t) = workValue as uint8_t
            }
            2 => {
                mask =
                    ((1 as libc::c_int) << (*var).config.bitpos as libc::c_int
                         & 0xffff as libc::c_int) as uint32_t;
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
                    ((1 as libc::c_int) <<
                         (*var).config.bitpos as libc::c_int) as uint32_t;
                if value != 0 {
                    workValue = *(ptr as *mut uint32_t) | mask
                } else { workValue = *(ptr as *mut uint32_t) & !mask }
                *(ptr as *mut uint32_t) = workValue
            }
            _ => { }
        }
    } else {
        match (*var).type_0 as libc::c_int & 0x7 as libc::c_int {
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
    let mut argIndex: uint32_t = 0 as libc::c_int as uint32_t;
    while argIndex < 2 as libc::c_int as libc::c_uint {
        ptr = nextArg(ptr);
        if !ptr.is_null() {
            let mut val: libc::c_int = atoi(ptr);
            val =
                (constrain(val, 900 as libc::c_int, 2100 as libc::c_int) -
                     900 as libc::c_int) / 25 as libc::c_int;
            if val >= 0 as libc::c_int &&
                   val <=
                       (2100 as libc::c_int - 900 as libc::c_int) /
                           25 as libc::c_int {
                if argIndex == 0 as libc::c_int as libc::c_uint {
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
               1 as libc::c_int
           } else { 0 as libc::c_int } != 0;
}
unsafe extern "C" fn printRxFailsafe(mut dumpMask: uint8_t,
                                     mut rxFailsafeChannelConfigs_0:
                                         *const rxFailsafeChannelConfig_t,
                                     mut defaultRxFailsafeChannelConfigs:
                                         *const rxFailsafeChannelConfig_t) {
    // print out rxConfig failsafe settings
    let mut channel: uint32_t = 0 as libc::c_int as uint32_t;
    while channel < 18 as libc::c_int as libc::c_uint {
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
                                 750 as libc::c_int +
                                     25 as libc::c_int *
                                         (*defaultChannelFailsafeConfig).step
                                             as libc::c_int);
            cliDumpPrintLinef(dumpMask, equalsDefault, format, channel,
                              rxFailsafeModeCharacters[(*channelFailsafeConfig).mode
                                                           as usize] as
                                  libc::c_int,
                              750 as libc::c_int +
                                  25 as libc::c_int *
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
        channel = 0 as libc::c_int as uint8_t;
        while (channel as libc::c_int) < 18 as libc::c_int {
            cliRxFailsafe(itoa(channel as libc::c_int, buf.as_mut_ptr(),
                               10 as libc::c_int));
            channel = channel.wrapping_add(1)
        }
    } else {
        let mut ptr: *const libc::c_char = cmdline;
        let fresh1 = ptr;
        ptr = ptr.offset(1);
        channel = atoi(fresh1) as uint8_t;
        if (channel as libc::c_int) < 18 as libc::c_int {
            let mut channelFailsafeConfig: *mut rxFailsafeChannelConfig_t =
                rxFailsafeChannelConfigsMutable(channel as libc::c_int);
            let type_0: rxFailsafeChannelType_e =
                if (channel as libc::c_int) < 4 as libc::c_int {
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
                        ((constrain(value as libc::c_int, 750 as libc::c_int,
                                    2250 as libc::c_int) - 750 as libc::c_int)
                             / 25 as libc::c_int) as uint16_t;
                    if value as libc::c_int >
                           (2250 as libc::c_int - 750 as libc::c_int) /
                               25 as libc::c_int {
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
                              750 as libc::c_int +
                                  25 as libc::c_int *
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
                                          *mut libc::c_char, 0 as libc::c_int,
                                      18 as libc::c_int - 1 as libc::c_int);
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
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    while i < 20 as libc::c_int as libc::c_uint {
        let mut mac: *const modeActivationCondition_t =
            &*modeActivationConditions_0.offset(i as isize) as
                *const modeActivationCondition_t;
        let mut equalsDefault: bool = 0 as libc::c_int != 0;
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
                                     900 as libc::c_int +
                                         25 as libc::c_int *
                                             (*macDefault).range.startStep as
                                                 libc::c_int,
                                     900 as libc::c_int +
                                         25 as libc::c_int *
                                             (*macDefault).range.endStep as
                                                 libc::c_int,
                                     (*macDefault).modeLogic as libc::c_uint,
                                     if !linkedTo.is_null() {
                                         (*linkedTo).permanentId as
                                             libc::c_int
                                     } else { 0 as libc::c_int });
            }
        }
        let mut box_1: *const box_t = findBoxByBoxId((*mac).modeId);
        let mut linkedTo_0: *const box_t = findBoxByBoxId((*mac).linkedTo);
        if !box_1.is_null() {
            cliDumpPrintLinef(dumpMask, equalsDefault, format, i,
                              (*box_1).permanentId as libc::c_int,
                              (*mac).auxChannelIndex as libc::c_int,
                              900 as libc::c_int +
                                  25 as libc::c_int *
                                      (*mac).range.startStep as libc::c_int,
                              900 as libc::c_int +
                                  25 as libc::c_int *
                                      (*mac).range.endStep as libc::c_int,
                              (*mac).modeLogic as libc::c_uint,
                              if !linkedTo_0.is_null() {
                                  (*linkedTo_0).permanentId as libc::c_int
                              } else { 0 as libc::c_int });
        }
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn cliAux(mut cmdline: *mut libc::c_char) {
    let mut i: libc::c_int = 0;
    let mut val: libc::c_int = 0 as libc::c_int;
    let mut ptr: *const libc::c_char = 0 as *const libc::c_char;
    if isEmpty(cmdline) {
        printAux(DUMP_MASTER as libc::c_int as uint8_t,
                 modeActivationConditions(0 as libc::c_int),
                 0 as *const modeActivationCondition_t);
    } else {
        ptr = cmdline;
        let fresh2 = ptr;
        ptr = ptr.offset(1);
        i = atoi(fresh2);
        if i < 20 as libc::c_int {
            let mut mac: *mut modeActivationCondition_t =
                modeActivationConditionsMutable(i);
            let mut validArgumentCount: uint8_t = 0 as libc::c_int as uint8_t;
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
                if val >= 0 as libc::c_int &&
                       val < 18 as libc::c_int - 4 as libc::c_int {
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
            if validArgumentCount as libc::c_int == 4 as libc::c_int {
                // for backwards compatibility
                (*mac).modeLogic = MODELOGIC_OR
            } else if validArgumentCount as libc::c_int == 5 as libc::c_int {
                // for backwards compatibility
                (*mac).linkedTo = BOXARM
            } else if validArgumentCount as libc::c_int != 6 as libc::c_int {
                memset(mac as *mut libc::c_void, 0 as libc::c_int,
                       ::core::mem::size_of::<modeActivationCondition_t>() as
                           libc::c_ulong);
            }
            cliPrintLinef(b"aux %u %u %u %u %u %u %u\x00" as *const u8 as
                              *const libc::c_char, i,
                          (*mac).modeId as libc::c_uint,
                          (*mac).auxChannelIndex as libc::c_int,
                          900 as libc::c_int +
                              25 as libc::c_int *
                                  (*mac).range.startStep as libc::c_int,
                          900 as libc::c_int +
                              25 as libc::c_int *
                                  (*mac).range.endStep as libc::c_int,
                          (*mac).modeLogic as libc::c_uint,
                          (*mac).linkedTo as libc::c_uint);
        } else {
            cliShowArgumentRangeError(b"index\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char, 0 as libc::c_int,
                                      20 as libc::c_int - 1 as libc::c_int);
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
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    while i < 2 as libc::c_int as libc::c_uint {
        if serialIsPortAvailable((*serialConfig_0).portConfigs[i as
                                                                   usize].identifier)
           {
            let mut equalsDefault: bool = 0 as libc::c_int != 0;
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
           0 as libc::c_int,
           ::core::mem::size_of::<serialPortConfig_t>() as libc::c_ulong);
    let mut currentConfig: *mut serialPortConfig_t =
        0 as *mut serialPortConfig_t;
    let mut validArgumentCount: uint8_t = 0 as libc::c_int as uint8_t;
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
        portConfig.functionMask = (val & 0xffff as libc::c_int) as uint16_t;
        validArgumentCount = validArgumentCount.wrapping_add(1)
    }
    let mut current_block_22: u64;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 4 as libc::c_int {
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
    if (validArgumentCount as libc::c_int) < 6 as libc::c_int {
        cliShowParseError();
        return
    }
    memcpy(currentConfig as *mut libc::c_void,
           &mut portConfig as *mut serialPortConfig_t as *const libc::c_void,
           ::core::mem::size_of::<serialPortConfig_t>() as libc::c_ulong);
    cliDumpPrintLinef(0 as libc::c_int as uint8_t, 0 as libc::c_int != 0,
                      format, portConfig.identifier as libc::c_int,
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
/* USE_PINIO */
unsafe extern "C" fn cliSerialPassthrough(mut cmdline: *mut libc::c_char) {
    if isEmpty(cmdline) { cliShowParseError(); return }
    let mut id: libc::c_int = -(1 as libc::c_int);
    let mut baud: uint32_t = 0 as libc::c_int as uint32_t;
    let mut enableBaudCb: bool = 0 as libc::c_int != 0;
    /* USE_PINIO */
    let mut mode: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    let mut saveptr: *mut libc::c_char = 0 as *mut libc::c_char;
    let mut tok: *mut libc::c_char =
        strtok_r(cmdline, b" \x00" as *const u8 as *const libc::c_char,
                 &mut saveptr);
    let mut index: libc::c_int = 0 as libc::c_int;
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
            _ => { }
        }
        index += 1;
        tok =
            strtok_r(0 as *mut libc::c_char,
                     b" \x00" as *const u8 as *const libc::c_char,
                     &mut saveptr)
    }
    if baud == 0 as libc::c_int as libc::c_uint {
        enableBaudCb = 1 as libc::c_int != 0
    }
    cliPrintf(b"Port %d \x00" as *const u8 as *const libc::c_char, id);
    let mut passThroughPort: *mut serialPort_t = 0 as *mut serialPort_t;
    let mut passThroughPortUsage: *mut serialPortUsage_t =
        findSerialPortUsageByIdentifier(id as serialPortIdentifier_e);
    if passThroughPortUsage.is_null() ||
           (*passThroughPortUsage).serialPort.is_null() {
        if enableBaudCb {
            // Set default baud
            baud = 57600 as libc::c_int as uint32_t
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
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    while i < 15 as libc::c_int as libc::c_uint {
        let mut ar: *const adjustmentRange_t =
            &*adjustmentRanges_0.offset(i as isize) as
                *const adjustmentRange_t;
        let mut equalsDefault: bool = 0 as libc::c_int != 0;
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
                                 900 as libc::c_int +
                                     25 as libc::c_int *
                                         (*arDefault).range.startStep as
                                             libc::c_int,
                                 900 as libc::c_int +
                                     25 as libc::c_int *
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
                          900 as libc::c_int +
                              25 as libc::c_int *
                                  (*ar).range.startStep as libc::c_int,
                          900 as libc::c_int +
                              25 as libc::c_int *
                                  (*ar).range.endStep as libc::c_int,
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
    let mut val: libc::c_int = 0 as libc::c_int;
    let mut ptr: *const libc::c_char = 0 as *const libc::c_char;
    if isEmpty(cmdline) {
        printAdjustmentRange(DUMP_MASTER as libc::c_int as uint8_t,
                             adjustmentRanges(0 as libc::c_int),
                             0 as *const adjustmentRange_t);
    } else {
        ptr = cmdline;
        let fresh4 = ptr;
        ptr = ptr.offset(1);
        i = atoi(fresh4);
        if i < 15 as libc::c_int {
            let mut ar: *mut adjustmentRange_t = adjustmentRangesMutable(i);
            let mut validArgumentCount: uint8_t = 0 as libc::c_int as uint8_t;
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                val = atoi(ptr);
                if val >= 0 as libc::c_int && val < 4 as libc::c_int {
                    (*ar).adjustmentIndex = val as uint8_t;
                    validArgumentCount = validArgumentCount.wrapping_add(1)
                }
            }
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                val = atoi(ptr);
                if val >= 0 as libc::c_int &&
                       val < 18 as libc::c_int - 4 as libc::c_int {
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
                if val >= 0 as libc::c_int &&
                       val < ADJUSTMENT_FUNCTION_COUNT as libc::c_int {
                    (*ar).adjustmentFunction = val as uint8_t;
                    validArgumentCount = validArgumentCount.wrapping_add(1)
                }
            }
            ptr = nextArg(ptr);
            if !ptr.is_null() {
                val = atoi(ptr);
                if val >= 0 as libc::c_int &&
                       val < 18 as libc::c_int - 4 as libc::c_int {
                    (*ar).auxSwitchChannelIndex = val as uint8_t;
                    validArgumentCount = validArgumentCount.wrapping_add(1)
                }
            }
            if validArgumentCount as libc::c_int != 6 as libc::c_int {
                memset(ar as *mut libc::c_void, 0 as libc::c_int,
                       ::core::mem::size_of::<adjustmentRange_t>() as
                           libc::c_ulong);
                cliShowParseError();
                return
            }
            // Optional arguments
            (*ar).adjustmentCenter = 0 as libc::c_int as uint16_t;
            (*ar).adjustmentScale = 0 as libc::c_int as uint16_t;
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
            cliDumpPrintLinef(0 as libc::c_int as uint8_t,
                              0 as libc::c_int != 0, format, i,
                              (*ar).adjustmentIndex as libc::c_int,
                              (*ar).auxChannelIndex as libc::c_int,
                              900 as libc::c_int +
                                  25 as libc::c_int *
                                      (*ar).range.startStep as libc::c_int,
                              900 as libc::c_int +
                                  25 as libc::c_int *
                                      (*ar).range.endStep as libc::c_int,
                              (*ar).adjustmentFunction as libc::c_int,
                              (*ar).auxSwitchChannelIndex as libc::c_int,
                              (*ar).adjustmentCenter as libc::c_int,
                              (*ar).adjustmentScale as libc::c_int);
        } else {
            cliShowArgumentRangeError(b"index\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char, 0 as libc::c_int,
                                      15 as libc::c_int - 1 as libc::c_int);
        }
    };
}
// USE_QUAD_MIXER_ONLY
unsafe extern "C" fn cliMotorMix(mut cmdline: *mut libc::c_char) { }
unsafe extern "C" fn printRxRange(mut dumpMask: uint8_t,
                                  mut channelRangeConfigs:
                                      *const rxChannelRangeConfig_t,
                                  mut defaultChannelRangeConfigs:
                                      *const rxChannelRangeConfig_t) {
    let mut format: *const libc::c_char =
        b"rxrange %u %u %u\x00" as *const u8 as *const libc::c_char;
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    while i < 4 as libc::c_int as libc::c_uint {
        let mut equalsDefault: bool = 0 as libc::c_int != 0;
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
    let mut validArgumentCount: libc::c_int = 0 as libc::c_int;
    let mut ptr: *const libc::c_char = 0 as *const libc::c_char;
    if isEmpty(cmdline) {
        printRxRange(DUMP_MASTER as libc::c_int as uint8_t,
                     rxChannelRangeConfigs(0 as libc::c_int),
                     0 as *const rxChannelRangeConfig_t);
    } else if strcasecmp(cmdline,
                         b"reset\x00" as *const u8 as *const libc::c_char) ==
                  0 as libc::c_int {
        resetAllRxChannelRangeConfigurations(rxChannelRangeConfigsMutable(0 as
                                                                              libc::c_int));
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if i >= 0 as libc::c_int && i < 4 as libc::c_int {
            let mut rangeMin: libc::c_int = 750 as libc::c_int;
            let mut rangeMax: libc::c_int = 2250 as libc::c_int;
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
            if validArgumentCount != 2 as libc::c_int {
                cliShowParseError();
            } else if rangeMin < 750 as libc::c_int ||
                          rangeMin > 2250 as libc::c_int ||
                          rangeMax < 750 as libc::c_int ||
                          rangeMax > 2250 as libc::c_int {
                cliShowParseError();
            } else {
                let mut channelRangeConfig: *mut rxChannelRangeConfig_t =
                    rxChannelRangeConfigsMutable(i);
                (*channelRangeConfig).min = rangeMin as uint16_t;
                (*channelRangeConfig).max = rangeMax as uint16_t;
                cliDumpPrintLinef(0 as libc::c_int as uint8_t,
                                  0 as libc::c_int != 0, format, i,
                                  (*channelRangeConfig).min as libc::c_int,
                                  (*channelRangeConfig).max as libc::c_int);
            }
        } else {
            cliShowArgumentRangeError(b"channel\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char, 0 as libc::c_int,
                                      4 as libc::c_int - 1 as libc::c_int);
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
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    while i < 32 as libc::c_int as libc::c_uint {
        let mut ledConfig: ledConfig_t = *ledConfigs.offset(i as isize);
        generateLedConfig(&mut ledConfig, ledConfigBuffer.as_mut_ptr(),
                          ::core::mem::size_of::<[libc::c_char; 20]>() as
                              libc::c_ulong);
        let mut equalsDefault: bool = 0 as libc::c_int != 0;
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
        if i < 32 as libc::c_int {
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
                cliDumpPrintLinef(0 as libc::c_int as uint8_t,
                                  0 as libc::c_int != 0, format, i,
                                  ledConfigBuffer.as_mut_ptr());
            } else { cliShowParseError(); }
        } else {
            cliShowArgumentRangeError(b"index\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char, 0 as libc::c_int,
                                      32 as libc::c_int - 1 as libc::c_int);
        }
    };
}
unsafe extern "C" fn printColor(mut dumpMask: uint8_t,
                                mut colors_0: *const hsvColor_t,
                                mut defaultColors: *const hsvColor_t) {
    let mut format: *const libc::c_char =
        b"color %u %d,%u,%u\x00" as *const u8 as *const libc::c_char;
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    while i < 16 as libc::c_int as libc::c_uint {
        let mut color: *const hsvColor_t =
            &*colors_0.offset(i as isize) as *const hsvColor_t;
        let mut equalsDefault: bool = 0 as libc::c_int != 0;
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
        if i < 16 as libc::c_int {
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
                cliDumpPrintLinef(0 as libc::c_int as uint8_t,
                                  0 as libc::c_int != 0, format, i,
                                  (*color).h as libc::c_int,
                                  (*color).s as libc::c_int,
                                  (*color).v as libc::c_int);
            } else { cliShowParseError(); }
        } else {
            cliShowArgumentRangeError(b"index\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char, 0 as libc::c_int,
                                      16 as libc::c_int - 1 as libc::c_int);
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
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    while i < 6 as libc::c_int as libc::c_uint {
        let mut j: uint32_t = 0 as libc::c_int as uint32_t;
        while j < 6 as libc::c_int as libc::c_uint {
            let mut colorIndex: libc::c_int =
                (*ledStripConfig_0).modeColors[i as usize].color[j as usize]
                    as libc::c_int;
            let mut equalsDefault: bool = 0 as libc::c_int != 0;
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
    let mut j_0: uint32_t = 0 as libc::c_int as uint32_t;
    while j_0 < 11 as libc::c_int as libc::c_uint {
        let colorIndex_0: libc::c_int =
            (*ledStripConfig_0).specialColors.color[j_0 as usize] as
                libc::c_int;
        let mut equalsDefault_0: bool = 0 as libc::c_int != 0;
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
    let mut equalsDefault_1: bool = 0 as libc::c_int != 0;
    if !defaultLedStripConfig.is_null() {
        let ledStripAuxChannelDefault: libc::c_int =
            (*defaultLedStripConfig).ledstrip_aux_channel as libc::c_int;
        equalsDefault_1 = ledStripAuxChannel == ledStripAuxChannelDefault;
        cliDefaultPrintLinef(dumpMask, equalsDefault_1, format,
                             LED_AUX_CHANNEL as libc::c_int, 0 as libc::c_int,
                             ledStripAuxChannelDefault);
    }
    cliDumpPrintLinef(dumpMask, equalsDefault_1, format,
                      LED_AUX_CHANNEL as libc::c_int, 0 as libc::c_int,
                      ledStripAuxChannel);
}
unsafe extern "C" fn cliModeColor(mut cmdline: *mut libc::c_char) {
    if isEmpty(cmdline) {
        printModeColor(DUMP_MASTER as libc::c_int as uint8_t,
                       ledStripConfig(), 0 as *const ledStripConfig_t);
    } else {
        let mut args: [libc::c_int; 3] = [0; 3];
        let mut argNo: libc::c_int = 0 as libc::c_int;
        let mut saveptr: *mut libc::c_char = 0 as *mut libc::c_char;
        let mut ptr: *const libc::c_char =
            strtok_r(cmdline, b" \x00" as *const u8 as *const libc::c_char,
                     &mut saveptr);
        while !ptr.is_null() && argNo < ARGS_COUNT as libc::c_int {
            let fresh5 = argNo;
            argNo = argNo + 1;
            args[fresh5 as usize] = atoi(ptr);
            ptr =
                strtok_r(0 as *mut libc::c_char,
                         b" \x00" as *const u8 as *const libc::c_char,
                         &mut saveptr)
        }
        if !ptr.is_null() || argNo != ARGS_COUNT as libc::c_int {
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
// VTX_CONTROL
unsafe extern "C" fn printName(mut dumpMask: uint8_t,
                               mut pilotConfig_0: *const pilotConfig_t) {
    let equalsDefault: bool =
        strlen((*pilotConfig_0).name.as_ptr()) ==
            0 as libc::c_int as libc::c_ulong;
    cliDumpPrintLinef(dumpMask, equalsDefault,
                      b"name %s\x00" as *const u8 as *const libc::c_char,
                      if equalsDefault as libc::c_int != 0 {
                          emptyName
                      } else { (*pilotConfig_0).name.as_ptr() });
}
unsafe extern "C" fn cliName(mut cmdline: *mut libc::c_char) {
    let len: libc::c_uint = strlen(cmdline) as libc::c_uint;
    if len > 0 as libc::c_int as libc::c_uint {
        memset((*pilotConfigMutable()).name.as_mut_ptr() as *mut libc::c_void,
               0 as libc::c_int,
               (::core::mem::size_of::<[libc::c_char; 17]>() as
                    libc::c_ulong).wrapping_div(::core::mem::size_of::<libc::c_char>()
                                                    as libc::c_ulong));
        if strncmp(cmdline, emptyName, len as libc::c_ulong) != 0 {
            strncpy((*pilotConfigMutable()).name.as_mut_ptr(), cmdline,
                    ({
                         let _a: libc::c_uint = len;
                         let mut _b: libc::c_uint = 16 as libc::c_uint;
                         if _a < _b { _a } else { _b }
                     }) as libc::c_ulong);
        }
    }
    printName(DUMP_MASTER as libc::c_int as uint8_t, pilotConfig());
}
// USE_BOARD_INFO
unsafe extern "C" fn cliMcuId(mut cmdline: *mut libc::c_char) {
    cliPrintLinef(b"mcu_id %08x%08x%08x\x00" as *const u8 as
                      *const libc::c_char,
                  *(0x1ffff7e8 as libc::c_int as *mut uint32_t),
                  *(0x1ffff7ec as libc::c_int as *mut uint32_t),
                  *(0x1ffff7f0 as libc::c_int as *mut uint32_t));
}
unsafe extern "C" fn printFeature(mut dumpMask: uint8_t,
                                  mut featureConfig_0: *const featureConfig_t,
                                  mut featureConfigDefault:
                                      *const featureConfig_t) {
    let mask: uint32_t = (*featureConfig_0).enabledFeatures;
    let defaultMask: uint32_t = (*featureConfigDefault).enabledFeatures;
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    while !featureNames[i as usize].is_null() {
        // disabled features first
        if strcmp(featureNames[i as usize], emptyString) != 0 as libc::c_int {
            //Skip unused
            let mut format: *const libc::c_char =
                b"feature -%s\x00" as *const u8 as *const libc::c_char;
            cliDefaultPrintLinef(dumpMask,
                                 (defaultMask | !mask) &
                                     ((1 as libc::c_int) << i) as libc::c_uint
                                     != 0, format, featureNames[i as usize]);
            cliDumpPrintLinef(dumpMask,
                              (!defaultMask | mask) &
                                  ((1 as libc::c_int) << i) as libc::c_uint !=
                                  0, format, featureNames[i as usize]);
        }
        i = i.wrapping_add(1)
    }
    let mut i_0: uint32_t = 0 as libc::c_int as uint32_t;
    while !featureNames[i_0 as usize].is_null() {
        // enabled features
        if strcmp(featureNames[i_0 as usize], emptyString) != 0 as libc::c_int
           {
            //Skip unused
            let mut format_0: *const libc::c_char =
                b"feature %s\x00" as *const u8 as *const libc::c_char;
            if defaultMask & ((1 as libc::c_int) << i_0) as libc::c_uint != 0
               {
                cliDefaultPrintLinef(dumpMask,
                                     (!defaultMask | mask) &
                                         ((1 as libc::c_int) << i_0) as
                                             libc::c_uint != 0, format_0,
                                     featureNames[i_0 as usize]);
            }
            if mask & ((1 as libc::c_int) << i_0) as libc::c_uint != 0 {
                cliDumpPrintLinef(dumpMask,
                                  (defaultMask | !mask) &
                                      ((1 as libc::c_int) << i_0) as
                                          libc::c_uint != 0, format_0,
                                  featureNames[i_0 as usize]);
            }
        }
        i_0 = i_0.wrapping_add(1)
    };
}
unsafe extern "C" fn cliFeature(mut cmdline: *mut libc::c_char) {
    let mut len: uint32_t = strlen(cmdline) as uint32_t;
    let mut mask: uint32_t = featureMask();
    if len == 0 as libc::c_int as libc::c_uint {
        cliPrint(b"Enabled: \x00" as *const u8 as *const libc::c_char);
        let mut i: uint32_t = 0 as libc::c_int as uint32_t;
        while !featureNames[i as usize].is_null() {
            if mask & ((1 as libc::c_int) << i) as libc::c_uint != 0 {
                cliPrintf(b"%s \x00" as *const u8 as *const libc::c_char,
                          featureNames[i as usize]);
            }
            i = i.wrapping_add(1)
        }
        cliPrintLinefeed();
    } else if strncasecmp(cmdline,
                          b"list\x00" as *const u8 as *const libc::c_char,
                          len as libc::c_ulong) == 0 as libc::c_int {
        cliPrint(b"Available:\x00" as *const u8 as *const libc::c_char);
        let mut i_0: uint32_t = 0 as libc::c_int as uint32_t;
        while !featureNames[i_0 as usize].is_null() {
            if strcmp(featureNames[i_0 as usize], emptyString) !=
                   0 as libc::c_int {
                //Skip unused
                cliPrintf(b" %s\x00" as *const u8 as *const libc::c_char,
                          featureNames[i_0 as usize]);
            }
            i_0 = i_0.wrapping_add(1)
        }
        cliPrintLinefeed();
        return
    } else {
        let mut remove: bool = 0 as libc::c_int != 0;
        if *cmdline.offset(0 as libc::c_int as isize) as libc::c_int ==
               '-' as i32 {
            // remove feature
            remove = 1 as libc::c_int != 0; // skip over -
            cmdline = cmdline.offset(1); // this is for beeper OFF condition
            len = len.wrapping_sub(1)
        }
        let mut i_1: uint32_t = 0 as libc::c_int as uint32_t;
        loop  {
            if featureNames[i_1 as usize].is_null() {
                cliPrintErrorLinef(b"Invalid name\x00" as *const u8 as
                                       *const libc::c_char);
                break ;
            } else if strncasecmp(cmdline, featureNames[i_1 as usize],
                                  len as libc::c_ulong) == 0 as libc::c_int {
                mask = ((1 as libc::c_int) << i_1) as uint32_t;
                if mask & FEATURE_GPS as libc::c_int as libc::c_uint != 0 {
                    cliPrintLine(b"unavailable\x00" as *const u8 as
                                     *const libc::c_char);
                    break ;
                } else if mask &
                              FEATURE_RANGEFINDER as libc::c_int as
                                  libc::c_uint != 0 {
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
    let mut i: int32_t = 0 as libc::c_int;
    while i < beeperCount as libc::c_int - 1 as libc::c_int {
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
    if len == 0 as libc::c_int as libc::c_uint {
        cliPrintf(b"Disabled:\x00" as *const u8 as *const libc::c_char);
        let mut i: int32_t = 0 as libc::c_int;
        loop  {
            if i == beeperCount as libc::c_int - 1 as libc::c_int {
                if *offFlags == 0 as libc::c_int as libc::c_uint {
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
                          len as libc::c_ulong) == 0 as libc::c_int {
        cliPrint(b"Available:\x00" as *const u8 as *const libc::c_char);
        let mut i_0: uint32_t = 0 as libc::c_int as uint32_t;
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
        let mut remove: bool = 0 as libc::c_int != 0;
        if *cmdline.offset(0 as libc::c_int as isize) as libc::c_int ==
               '-' as i32 {
            remove = 1 as libc::c_int != 0;
            cmdline = cmdline.offset(1);
            len = len.wrapping_sub(1)
        }
        let mut i_1: uint32_t = 0 as libc::c_int as uint32_t;
        loop  {
            if i_1 == beeperCount as libc::c_uint {
                cliPrintErrorLinef(b"Invalid name\x00" as *const u8 as
                                       *const libc::c_char);
                break ;
            } else if strncasecmp(cmdline,
                                  beeperNameForTableIndex(i_1 as libc::c_int),
                                  len as libc::c_ulong) == 0 as libc::c_int &&
                          beeperModeMaskForTableIndex(i_1 as libc::c_int) &
                              (allowedFlags |
                                   ((1 as libc::c_int) <<
                                        BEEPER_ALL as libc::c_int -
                                            1 as libc::c_int) as libc::c_uint)
                              != 0 {
                if remove {
                    // beeper off
                    if i_1 ==
                           (BEEPER_ALL as libc::c_int - 1 as libc::c_int) as
                               libc::c_uint {
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
                           (BEEPER_ALL as libc::c_int - 1 as libc::c_int) as
                               libc::c_uint {
                        *offFlags = 0 as libc::c_int as uint32_t
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
unsafe extern "C" fn cliBeeper(mut cmdline: *mut libc::c_char) {
    processBeeperCommand(cmdline,
                         &mut (*(beeperConfigMutable as
                                     unsafe extern "C" fn()
                                         ->
                                             *mut beeperConfig_t)()).beeper_off_flags,
                         ((1 as libc::c_int) <<
                              BEEPER_GYRO_CALIBRATED as libc::c_int -
                                  1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_RX_LOST as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_RX_LOST_LANDING as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_DISARMING as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_ARMING as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_ARMING_GPS_FIX as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_BAT_CRIT_LOW as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_BAT_LOW as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_GPS_STATUS as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_RX_SET as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_ACC_CALIBRATION as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_ACC_CALIBRATION_FAIL as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_READY_BEEP as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_MULTI_BEEPS as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_DISARM_REPEAT as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_ARMED as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_SYSTEM_INIT as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_USB as libc::c_int - 1 as libc::c_int
                              |
                              (1 as libc::c_int) <<
                                  BEEPER_BLACKBOX_ERASE as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_CRASH_FLIP_MODE as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_CAM_CONNECTION_OPEN as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_CAM_CONNECTION_CLOSE as libc::c_int -
                                      1 as libc::c_int |
                              (1 as libc::c_int) <<
                                  BEEPER_RC_SMOOTHING_INIT_FAIL as libc::c_int
                                      - 1 as libc::c_int) as uint32_t);
}
unsafe extern "C" fn printMap(mut dumpMask: uint8_t,
                              mut rxConfig_0: *const rxConfig_t,
                              mut defaultRxConfig: *const rxConfig_t) {
    let mut equalsDefault: bool = 1 as libc::c_int != 0;
    let mut buf: [libc::c_char; 9] = [0; 9];
    let mut bufDefault: [libc::c_char; 9] = [0; 9];
    let mut i: uint32_t = 0;
    i = 0 as libc::c_int as uint32_t;
    while i < 8 as libc::c_int as libc::c_uint {
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
    if len == 8 as libc::c_int as libc::c_uint {
        i = 0 as libc::c_int as uint32_t;
        while i < 8 as libc::c_int as libc::c_uint {
            buf[i as usize] =
                toupper(*cmdline.offset(i as isize) as libc::c_uchar as
                            libc::c_int) as libc::c_char;
            i = i.wrapping_add(1)
        }
        buf[i as usize] = '\u{0}' as i32 as libc::c_char;
        i = 0 as libc::c_int as uint32_t;
        while i < 8 as libc::c_int as libc::c_uint {
            buf[i as usize] =
                toupper(*cmdline.offset(i as isize) as libc::c_uchar as
                            libc::c_int) as libc::c_char;
            if !strchr(rcChannelLetters.as_ptr(),
                       buf[i as usize] as libc::c_int).is_null() &&
                   strchr(buf.as_mut_ptr().offset(i as
                                                      isize).offset(1 as
                                                                        libc::c_int
                                                                        as
                                                                        isize),
                          buf[i as usize] as libc::c_int).is_null() {
                i = i.wrapping_add(1)
            } else { cliShowParseError(); return }
        }
        parseRcChannels(buf.as_mut_ptr(), rxConfigMutable());
    } else if len > 0 as libc::c_int as libc::c_uint {
        cliShowParseError();
        return
    }
    i = 0 as libc::c_int as uint32_t;
    while i < 8 as libc::c_int as libc::c_uint {
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
                    0 as libc::c_int) {
        return skipSpace(cmdLine.offset(strlen(command) as
                                            isize).offset(1 as libc::c_int as
                                                              isize))
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
unsafe extern "C" fn cliReboot() { cliRebootEx(0 as libc::c_int != 0); }
unsafe extern "C" fn cliBootloader(mut cmdLine: *mut libc::c_char) {
    cliRebootEx(1 as libc::c_int != 0);
}
unsafe extern "C" fn cliExit(mut cmdline: *mut libc::c_char) {
    bufWriterFlush(cliWriter);
    *cliBuffer.as_mut_ptr() = '\u{0}' as i32 as libc::c_char;
    bufferIndex = 0 as libc::c_int as uint32_t;
    cliMode = 0 as libc::c_int as uint8_t;
    // incase a motor was left running during motortest, clear it here
    mixerResetDisarmedMotors();
    cliReboot();
    cliWriter = 0 as *mut bufWriter_t;
}
unsafe extern "C" fn cliPrintGyroRegisters(mut whichSensor: uint8_t) {
    cliPrintLinef(b"# WHO_AM_I    0x%X\x00" as *const u8 as
                      *const libc::c_char,
                  gyroReadRegister(whichSensor,
                                   0x75 as libc::c_int as uint8_t) as
                      libc::c_int);
    cliPrintLinef(b"# CONFIG      0x%X\x00" as *const u8 as
                      *const libc::c_char,
                  gyroReadRegister(whichSensor,
                                   0x1a as libc::c_int as uint8_t) as
                      libc::c_int);
    cliPrintLinef(b"# GYRO_CONFIG 0x%X\x00" as *const u8 as
                      *const libc::c_char,
                  gyroReadRegister(whichSensor,
                                   0x1b as libc::c_int as uint8_t) as
                      libc::c_int);
}
unsafe extern "C" fn cliDumpGyroRegisters(mut cmdline: *mut libc::c_char) {
    cliPrintGyroRegisters(0 as libc::c_int as uint8_t);
}
unsafe extern "C" fn parseOutputIndex(mut pch: *mut libc::c_char,
                                      mut allowAllEscs: bool) -> libc::c_int {
    let mut outputIndex: libc::c_int = atoi(pch);
    if outputIndex >= 0 as libc::c_int &&
           outputIndex < getMotorCount() as libc::c_int {
        cliPrintLinef(b"Using output %d.\x00" as *const u8 as
                          *const libc::c_char, outputIndex);
    } else if allowAllEscs as libc::c_int != 0 &&
                  outputIndex == 255 as libc::c_int {
        cliPrintLinef(b"Using all outputs.\x00" as *const u8 as
                          *const libc::c_char);
    } else {
        cliPrintErrorLinef(b"Invalid output number. Range: 0  %d.\x00" as
                               *const u8 as *const libc::c_char,
                           getMotorCount() as libc::c_int - 1 as libc::c_int);
        return -(1 as libc::c_int)
    }
    return outputIndex;
}
// USE_DSHOT
unsafe extern "C" fn cliMotor(mut cmdline: *mut libc::c_char) {
    if isEmpty(cmdline) { cliShowParseError(); return }
    let mut motorIndex: libc::c_int = 0 as libc::c_int;
    let mut motorValue: libc::c_int = 0 as libc::c_int;
    let mut saveptr: *mut libc::c_char = 0 as *mut libc::c_char;
    let mut pch: *mut libc::c_char =
        strtok_r(cmdline, b" \x00" as *const u8 as *const libc::c_char,
                 &mut saveptr);
    let mut index: libc::c_int = 0 as libc::c_int;
    while !pch.is_null() {
        match index {
            0 => {
                motorIndex = parseOutputIndex(pch, 1 as libc::c_int != 0);
                if motorIndex == -(1 as libc::c_int) { return }
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
    if index == 2 as libc::c_int {
        if motorValue < 1000 as libc::c_int ||
               motorValue > 2000 as libc::c_int {
            cliShowArgumentRangeError(b"value\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char,
                                      1000 as libc::c_int,
                                      2000 as libc::c_int);
        } else {
            let mut motorOutputValue: uint32_t =
                convertExternalToMotor(motorValue as uint16_t) as uint32_t;
            if motorIndex != 255 as libc::c_int {
                motor_disarmed[motorIndex as usize] =
                    motorOutputValue as libc::c_float;
                cliPrintLinef(b"motor %d: %d\x00" as *const u8 as
                                  *const libc::c_char, motorIndex,
                              motorOutputValue);
            } else {
                let mut i: libc::c_int = 0 as libc::c_int;
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
        if i >= 0 as libc::c_int && i < 3 as libc::c_int {
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
        if i >= 0 as libc::c_int && i < 6 as libc::c_int {
            changeControlRateProfile(i as uint8_t);
            cliRateProfile(b"\x00" as *const u8 as *const libc::c_char as
                               *mut libc::c_char);
        }
    };
}
unsafe extern "C" fn cliDumpPidProfile(mut pidProfileIndex: uint8_t,
                                       mut dumpMask: uint8_t) {
    if pidProfileIndex as libc::c_int >= 3 as libc::c_int {
        // Faulty values
        return
    }
    pidProfileIndexToUse = pidProfileIndex as int8_t;
    cliProfile(b"\x00" as *const u8 as *const libc::c_char as
                   *mut libc::c_char);
    cliPrintLinefeed();
    dumpAllValues(PROFILE_VALUE as libc::c_int as uint16_t, dumpMask);
    pidProfileIndexToUse = -(1 as libc::c_int) as int8_t;
}
unsafe extern "C" fn cliDumpRateProfile(mut rateProfileIndex: uint8_t,
                                        mut dumpMask: uint8_t) {
    if rateProfileIndex as libc::c_int >= 6 as libc::c_int {
        // Faulty values
        return
    }
    rateProfileIndexToUse = rateProfileIndex as int8_t;
    cliRateProfile(b"\x00" as *const u8 as *const libc::c_char as
                       *mut libc::c_char);
    cliPrintLinefeed();
    dumpAllValues(PROFILE_RATE_VALUE as libc::c_int as uint16_t, dumpMask);
    rateProfileIndexToUse = -(1 as libc::c_int) as int8_t;
}
unsafe extern "C" fn cliSave(mut cmdline: *mut libc::c_char) {
    // USE_BOARD_INFO
    writeEEPROM(); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
    cliReboot();
}
unsafe extern "C" fn cliDefaults(mut cmdline: *mut libc::c_char) {
    let mut saveConfigs: bool = false;
    if isEmpty(cmdline) {
        saveConfigs = 1 as libc::c_int != 0
    } else if strncasecmp(cmdline,
                          b"nosave\x00" as *const u8 as *const libc::c_char,
                          6 as libc::c_int as libc::c_ulong) ==
                  0 as libc::c_int {
        saveConfigs = 0 as libc::c_int != 0
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
                                  *const libc::c_void, 0 as libc::c_int != 0);
            cliPrintLinefeed();
        }
    };
}
unsafe extern "C" fn cliGet(mut cmdline: *mut libc::c_char) {
    let mut val: *const clivalue_t = 0 as *const clivalue_t;
    let mut matchedCommands: libc::c_int = 0 as libc::c_int;
    pidProfileIndexToUse = getCurrentPidProfileIndex() as int8_t;
    rateProfileIndexToUse = getCurrentControlRateProfileIndex() as int8_t;
    backupAndResetConfigs();
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    while i < valueTableEntryCount as libc::c_uint {
        if !strcasestr((*valueTable.as_ptr().offset(i as isize)).name,
                       cmdline).is_null() {
            val =
                &*valueTable.as_ptr().offset(i as isize) as *const clivalue_t;
            if matchedCommands > 0 as libc::c_int { cliPrintLinefeed(); }
            cliPrintf(b"%s = \x00" as *const u8 as *const libc::c_char,
                      (*valueTable.as_ptr().offset(i as isize)).name);
            cliPrintVar(val, 0 as libc::c_int != 0);
            cliPrintLinefeed();
            match (*val).type_0 as libc::c_int & 0x18 as libc::c_int {
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
    pidProfileIndexToUse = -(1 as libc::c_int) as int8_t;
    rateProfileIndexToUse = -(1 as libc::c_int) as int8_t;
    if matchedCommands != 0 { return }
    cliPrintErrorLinef(b"Invalid name\x00" as *const u8 as
                           *const libc::c_char);
}
unsafe extern "C" fn getWordLength(mut bufBegin: *mut libc::c_char,
                                   mut bufEnd: *mut libc::c_char) -> uint8_t {
    while *bufEnd.offset(-(1 as libc::c_int as isize)) as libc::c_int ==
              ' ' as i32 {
        bufEnd = bufEnd.offset(-1)
    }
    return bufEnd.wrapping_offset_from(bufBegin) as libc::c_long as uint8_t;
}
unsafe extern "C" fn cliSet(mut cmdline: *mut libc::c_char) {
    let len: uint32_t = strlen(cmdline) as uint32_t;
    let mut eqptr: *mut libc::c_char = 0 as *mut libc::c_char;
    if len == 0 as libc::c_int as libc::c_uint ||
           len == 1 as libc::c_int as libc::c_uint &&
               *cmdline.offset(0 as libc::c_int as isize) as libc::c_int ==
                   '*' as i32 {
        cliPrintLine(b"Current settings: \x00" as *const u8 as
                         *const libc::c_char);
        let mut i: uint32_t = 0 as libc::c_int as uint32_t;
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
            let mut i_0: uint32_t = 0 as libc::c_int as uint32_t;
            while i_0 < valueTableEntryCount as libc::c_uint {
                let mut val_0: *const clivalue_t =
                    &*valueTable.as_ptr().offset(i_0 as isize) as
                        *const clivalue_t;
                // ensure exact match when setting to prevent setting variables with shorter names
                if strncasecmp(cmdline, (*val_0).name, strlen((*val_0).name))
                       == 0 as libc::c_int &&
                       variableNameLength as libc::c_ulong ==
                           strlen((*val_0).name) {
                    let mut valueChanged: bool = 0 as libc::c_int != 0;
                    let mut value: int16_t = 0 as libc::c_int as int16_t;
                    match (*val_0).type_0 as libc::c_int & 0x60 as libc::c_int
                        {
                        0 => {
                            let mut value_0: int16_t = atoi(eqptr) as int16_t;
                            if value_0 as libc::c_int >=
                                   (*val_0).config.minmax.min as libc::c_int
                                   &&
                                   value_0 as libc::c_int <=
                                       (*val_0).config.minmax.max as
                                           libc::c_int {
                                cliSetVar(val_0, value_0);
                                valueChanged = 1 as libc::c_int != 0
                            }
                        }
                        32 | 96 => {
                            let mut tableIndex: libc::c_int = 0;
                            if (*val_0).type_0 as libc::c_int &
                                   0x60 as libc::c_int ==
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
                            let mut matched: bool = 0 as libc::c_int != 0;
                            let mut tableValueIndex: uint32_t =
                                0 as libc::c_int as uint32_t;
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
                                                   eqptr) == 0 as libc::c_int;
                                if matched {
                                    value = tableValueIndex as int16_t;
                                    cliSetVar(val_0, value);
                                    valueChanged = 1 as libc::c_int != 0
                                }
                                tableValueIndex =
                                    tableValueIndex.wrapping_add(1)
                            }
                        }
                        64 => {
                            let arrayLength: uint8_t =
                                (*val_0).config.array.length;
                            let mut valPtr: *mut libc::c_char = eqptr;
                            let mut i_1: libc::c_int = 0 as libc::c_int;
                            while i_1 < arrayLength as libc::c_int &&
                                      !valPtr.is_null() {
                                // skip spaces
                                valPtr = skipSpace(valPtr);
                                // process substring starting at valPtr
                            // note: no need to copy substrings for atoi()
                            //       it stops at the first character that cannot be converted...
                                match (*val_0).type_0 as libc::c_int &
                                          0x7 as libc::c_int {
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
                                valPtr =
                                    strchr(valPtr,
                                           ',' as
                                               i32).offset(1 as libc::c_int as
                                                               isize);
                                i_1 += 1
                            }
                            // mark as changed
                            valueChanged = 1 as libc::c_int != 0
                        }
                        _ => { }
                    }
                    if valueChanged {
                        cliPrintf(b"%s set to \x00" as *const u8 as
                                      *const libc::c_char, (*val_0).name);
                        cliPrintVar(val_0, 0 as libc::c_int != 0);
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
                  millis().wrapping_div(1000 as libc::c_int as libc::c_uint));
    cliPrintLinef(b"Voltage: %d * 0.1V (%dS battery - %s)\x00" as *const u8 as
                      *const libc::c_char, getBatteryVoltage() as libc::c_int,
                  getBatteryCellCount() as libc::c_int,
                  getBatteryStateString());
    cliPrintf(b"CPU Clock=%dMHz\x00" as *const u8 as *const libc::c_char,
              SystemCoreClock.wrapping_div(1000000 as libc::c_int as
                                               libc::c_uint));
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
        if getTaskDeltaTime(TASK_GYROPID) == 0 as libc::c_int {
            0 as libc::c_int
        } else {
            (1000000.0f32 / getTaskDeltaTime(TASK_GYROPID) as libc::c_float)
                as libc::c_int
        };
    let rxRate: libc::c_int =
        if currentRxRefreshRate as libc::c_int == 0 as libc::c_int {
            0 as libc::c_int
        } else {
            (1000000.0f32 / currentRxRefreshRate as libc::c_float) as
                libc::c_int
        };
    let systemRate: libc::c_int =
        if getTaskDeltaTime(TASK_SYSTEM) == 0 as libc::c_int {
            0 as libc::c_int
        } else {
            (1000000.0f32 / getTaskDeltaTime(TASK_SYSTEM) as libc::c_float) as
                libc::c_int
        };
    cliPrintLinef(b"CPU:%d%%, cycle time: %d, GYRO rate: %d, RX rate: %d, System rate: %d\x00"
                      as *const u8 as *const libc::c_char,
                  constrain(averageSystemLoadPercent as libc::c_int,
                            0 as libc::c_int, 100 as libc::c_int),
                  getTaskDeltaTime(TASK_GYROPID), gyroRate, rxRate,
                  systemRate);
    cliPrint(b"Arming disable flags:\x00" as *const u8 as
                 *const libc::c_char);
    let mut flags: armingDisableFlags_e = getArmingDisableFlags();
    while flags as u64 != 0 {
        let bitpos: libc::c_int =
            ffs(flags as libc::c_int) - 1 as libc::c_int;
        flags =
            ::core::mem::transmute::<libc::c_uint,
                                     armingDisableFlags_e>(flags as
                                                               libc::c_uint &
                                                               !((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     bitpos)
                                                                   as
                                                                   libc::c_uint);
        cliPrintf(b" %s\x00" as *const u8 as *const libc::c_char,
                  armingDisableFlagNames[bitpos as usize]);
    }
    cliPrintLinefeed();
}
unsafe extern "C" fn cliTasks(mut cmdline: *mut libc::c_char) {
    let mut maxLoadSum: libc::c_int = 0 as libc::c_int;
    let mut averageLoadSum: libc::c_int = 0 as libc::c_int;
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
            let mut subTaskFrequency: libc::c_int = 0 as libc::c_int;
            if taskId as libc::c_uint ==
                   TASK_GYROPID as libc::c_int as libc::c_uint {
                subTaskFrequency =
                    if taskInfo.latestDeltaTime == 0 as libc::c_int {
                        0 as libc::c_int
                    } else {
                        (1000000.0f32 /
                             taskInfo.latestDeltaTime as libc::c_float) as
                            libc::c_int
                    };
                taskFrequency =
                    subTaskFrequency /
                        (*pidConfig()).pid_process_denom as libc::c_int;
                if (*pidConfig()).pid_process_denom as libc::c_int >
                       1 as libc::c_int {
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
                    if taskInfo.latestDeltaTime == 0 as libc::c_int {
                        0 as libc::c_int
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
                if taskInfo.maxExecutionTime ==
                       0 as libc::c_int as libc::c_uint {
                    0 as libc::c_int as libc::c_uint
                } else {
                    taskInfo.maxExecutionTime.wrapping_mul(taskFrequency as
                                                               libc::c_uint).wrapping_add(5000
                                                                                              as
                                                                                              libc::c_int
                                                                                              as
                                                                                              libc::c_uint).wrapping_div(1000
                                                                                                                             as
                                                                                                                             libc::c_int
                                                                                                                             as
                                                                                                                             libc::c_uint)
                } as libc::c_int;
            let averageLoad: libc::c_int =
                if taskInfo.averageExecutionTime ==
                       0 as libc::c_int as libc::c_uint {
                    0 as libc::c_int as libc::c_uint
                } else {
                    taskInfo.averageExecutionTime.wrapping_mul(taskFrequency
                                                                   as
                                                                   libc::c_uint).wrapping_add(5000
                                                                                                  as
                                                                                                  libc::c_int
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_div(1000
                                                                                                                                 as
                                                                                                                                 libc::c_int
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
                              taskInfo.averageExecutionTime,
                              maxLoad / 10 as libc::c_int,
                              maxLoad % 10 as libc::c_int,
                              averageLoad / 10 as libc::c_int,
                              averageLoad % 10 as libc::c_int,
                              taskInfo.totalExecutionTime.wrapping_div(1000 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_uint));
            } else {
                cliPrintLinef(b"%6d\x00" as *const u8 as *const libc::c_char,
                              taskFrequency);
            }
            if taskId as libc::c_uint ==
                   TASK_GYROPID as libc::c_int as libc::c_uint &&
                   (*pidConfig()).pid_process_denom as libc::c_int >
                       1 as libc::c_int {
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
                      checkFuncInfo.totalExecutionTime.wrapping_div(1000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint));
        cliPrintLinef(b"Total (excluding SERIAL) %25d.%1d%% %4d.%1d%%\x00" as
                          *const u8 as *const libc::c_char,
                      maxLoadSum / 10 as libc::c_int,
                      maxLoadSum % 10 as libc::c_int,
                      averageLoadSum / 10 as libc::c_int,
                      averageLoadSum % 10 as libc::c_int);
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
// Handy macros for keeping the table tidy.
// DEFS : Single entry
// DEFA : Array of uint8_t (stride = 1)
// DEFW : Wider stride case; array of structs.
#[no_mangle]
pub static mut resourceTable: [cliResourceValue_t; 19] =
    [{
         let mut init =
             cliResourceValue_t{owner: OWNER_BEEPER as libc::c_int as uint8_t,
                                pgn: 503 as libc::c_int as pgn_t,
                                stride: 0 as libc::c_int as uint8_t,
                                offset: 0 as libc::c_ulong as uint8_t,
                                maxIndex: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner: OWNER_MOTOR as libc::c_int as uint8_t,
                                pgn: 6 as libc::c_int as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 6 as libc::c_ulong as uint8_t,
                                maxIndex: 4 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_PPMINPUT as libc::c_int as uint8_t,
                                pgn: 507 as libc::c_int as pgn_t,
                                stride: 0 as libc::c_int as uint8_t,
                                offset: 0 as libc::c_ulong as uint8_t,
                                maxIndex: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_PWMINPUT as libc::c_int as uint8_t,
                                pgn: 508 as libc::c_int as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 0 as libc::c_ulong as uint8_t,
                                maxIndex: 8 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_LED_STRIP as libc::c_int as uint8_t,
                                pgn: 27 as libc::c_int as pgn_t,
                                stride: 0 as libc::c_int as uint8_t,
                                offset: 241 as libc::c_ulong as uint8_t,
                                maxIndex: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_SERIAL_TX as libc::c_int as uint8_t,
                                pgn: 509 as libc::c_int as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 0 as libc::c_ulong as uint8_t,
                                maxIndex: 10 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_SERIAL_RX as libc::c_int as uint8_t,
                                pgn: 509 as libc::c_int as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 10 as libc::c_ulong as uint8_t,
                                maxIndex: 10 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_INVERTER as libc::c_int as uint8_t,
                                pgn: 509 as libc::c_int as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 20 as libc::c_ulong as uint8_t,
                                maxIndex: 10 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_I2C_SCL as libc::c_int as uint8_t,
                                pgn: 518 as libc::c_int as pgn_t,
                                stride:
                                    ::core::mem::size_of::<i2cConfig_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 0 as libc::c_ulong as uint8_t,
                                maxIndex: 2 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_I2C_SDA as libc::c_int as uint8_t,
                                pgn: 518 as libc::c_int as pgn_t,
                                stride:
                                    ::core::mem::size_of::<i2cConfig_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 1 as libc::c_ulong as uint8_t,
                                maxIndex: 2 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner: OWNER_LED as libc::c_int as uint8_t,
                                pgn: 505 as libc::c_int as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 0 as libc::c_ulong as uint8_t,
                                maxIndex: 3 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_SPI_SCK as libc::c_int as uint8_t,
                                pgn: 520 as libc::c_int as pgn_t,
                                stride:
                                    ::core::mem::size_of::<spiPinConfig_t>()
                                        as libc::c_ulong as uint8_t,
                                offset: 0 as libc::c_ulong as uint8_t,
                                maxIndex: 2 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_SPI_MISO as libc::c_int as uint8_t,
                                pgn: 520 as libc::c_int as pgn_t,
                                stride:
                                    ::core::mem::size_of::<spiPinConfig_t>()
                                        as libc::c_ulong as uint8_t,
                                offset: 1 as libc::c_ulong as uint8_t,
                                maxIndex: 2 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_SPI_MOSI as libc::c_int as uint8_t,
                                pgn: 520 as libc::c_int as pgn_t,
                                stride:
                                    ::core::mem::size_of::<spiPinConfig_t>()
                                        as libc::c_ulong as uint8_t,
                                offset: 2 as libc::c_ulong as uint8_t,
                                maxIndex: 2 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_BARO_CS as libc::c_int as uint8_t,
                                pgn: 38 as libc::c_int as pgn_t,
                                stride: 0 as libc::c_int as uint8_t,
                                offset: 2 as libc::c_ulong as uint8_t,
                                maxIndex: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_COMPASS_CS as libc::c_int as
                                        uint8_t,
                                pgn: 40 as libc::c_int as pgn_t,
                                stride: 0 as libc::c_int as uint8_t,
                                offset: 13 as libc::c_ulong as uint8_t,
                                maxIndex: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_COMPASS_EXTI as libc::c_int as
                                        uint8_t,
                                pgn: 40 as libc::c_int as pgn_t,
                                stride: 0 as libc::c_int as uint8_t,
                                offset: 14 as libc::c_ulong as uint8_t,
                                maxIndex: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_SPI_PREINIT_IPU as libc::c_int as
                                        uint8_t,
                                pgn: 535 as libc::c_int as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 0 as libc::c_ulong as uint8_t,
                                maxIndex: 11 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             cliResourceValue_t{owner:
                                    OWNER_SPI_PREINIT_OPU as libc::c_int as
                                        uint8_t,
                                pgn: 536 as libc::c_int as pgn_t,
                                stride:
                                    ::core::mem::size_of::<ioTag_t>() as
                                        libc::c_ulong as uint8_t,
                                offset: 0 as libc::c_ulong as uint8_t,
                                maxIndex: 2 as libc::c_int as uint8_t,};
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
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while (i as libc::c_ulong) <
              (::core::mem::size_of::<[cliResourceValue_t; 19]>() as
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
        let mut index: libc::c_int = 0 as libc::c_int;
        while index <
                  (if resourceTable[i as usize].maxIndex as libc::c_int ==
                          0 as libc::c_int {
                       1 as libc::c_int
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
                                     formatUnassigned, owner,
                                     index + 1 as libc::c_int);
            } else {
                cliDefaultPrintLinef(dumpMask, equalsDefault, format, owner,
                                     index + 1 as libc::c_int,
                                     (ioTagDefault as libc::c_int >>
                                          4 as libc::c_int) - 1 as libc::c_int
                                         + 'A' as i32,
                                     ioTagDefault as libc::c_int &
                                         0xf as libc::c_int);
            }
            if ioTag == 0 {
                if dumpMask as libc::c_int & HIDE_UNUSED as libc::c_int == 0 {
                    cliDumpPrintLinef(dumpMask, equalsDefault,
                                      formatUnassigned, owner,
                                      index + 1 as libc::c_int);
                }
            } else {
                cliDumpPrintLinef(dumpMask, equalsDefault, format, owner,
                                  index + 1 as libc::c_int,
                                  (ioTag as libc::c_int >> 4 as libc::c_int) -
                                      1 as libc::c_int + 'A' as i32,
                                  ioTag as libc::c_int & 0xf as libc::c_int);
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
    if resourceTable[owner as usize].maxIndex as libc::c_int >
           0 as libc::c_int {
        cliPrintf(b" %d\x00" as *const u8 as *const libc::c_char,
                  index as libc::c_int + 1 as libc::c_int);
    };
}
unsafe extern "C" fn resourceCheck(mut resourceIndex: uint8_t,
                                   mut index: uint8_t, mut newTag: ioTag_t) {
    if newTag == 0 { return }
    let mut format: *const libc::c_char =
        b"\r\nNOTE: %c%02d already assigned to \x00" as *const u8 as
            *const libc::c_char;
    let mut r: libc::c_int = 0 as libc::c_int;
    while r <
              (::core::mem::size_of::<[cliResourceValue_t; 19]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<cliResourceValue_t>()
                                                   as libc::c_ulong) as
                  libc::c_int {
        let mut current_block_12: u64;
        let mut i: libc::c_int = 0 as libc::c_int;
        while i <
                  (if resourceTable[r as usize].maxIndex as libc::c_int ==
                          0 as libc::c_int {
                       1 as libc::c_int
                   } else {
                       resourceTable[r as usize].maxIndex as libc::c_int
                   }) {
            let mut tag: *mut ioTag_t =
                getIoTag(resourceTable[r as usize], i as uint8_t);
            if *tag as libc::c_int == newTag as libc::c_int {
                let mut cleared: bool = 0 as libc::c_int != 0;
                if r == resourceIndex as libc::c_int {
                    if i == index as libc::c_int {
                        current_block_12 = 15427931788582360902;
                    } else {
                        *tag = 0 as libc::c_int as ioTag_t;
                        cleared = 1 as libc::c_int != 0;
                        current_block_12 = 12800627514080957624;
                    }
                } else { current_block_12 = 12800627514080957624; }
                match current_block_12 {
                    15427931788582360902 => { }
                    _ => {
                        cliPrintf(format,
                                  (newTag as libc::c_int >> 4 as libc::c_int)
                                      - 1 as libc::c_int + 'A' as i32,
                                  newTag as libc::c_int & 0xf as libc::c_int);
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
           0 as libc::c_int {
        *tag = 0 as libc::c_int as ioTag_t;
        return 1 as libc::c_int != 0
    } else {
        let mut pin: libc::c_uint = 0 as libc::c_int as libc::c_uint;
        let mut port: libc::c_uint =
            if *pch as libc::c_int >= 'a' as i32 {
                (*pch as libc::c_int) - 'a' as i32
            } else { (*pch as libc::c_int) - 'A' as i32 } as libc::c_uint;
        if port < 8 as libc::c_int as libc::c_uint {
            pch = pch.offset(1);
            pin = atoi(pch) as libc::c_uint;
            if pin < 16 as libc::c_int as libc::c_uint {
                *tag =
                    (port.wrapping_add(1 as libc::c_int as libc::c_uint) <<
                         4 as libc::c_int | pin) as ioTag_t;
                return 1 as libc::c_int != 0
            }
        }
    }
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn cliResource(mut cmdline: *mut libc::c_char) {
    let mut len: libc::c_int = strlen(cmdline) as libc::c_int;
    if len == 0 as libc::c_int {
        printResource((DUMP_MASTER as libc::c_int |
                           HIDE_UNUSED as libc::c_int) as uint8_t);
        return
    } else {
        if strncasecmp(cmdline,
                       b"list\x00" as *const u8 as *const libc::c_char,
                       len as libc::c_ulong) == 0 as libc::c_int {
            cliPrintLine(b"IO\x00" as *const u8 as *const libc::c_char);
            let mut i: libc::c_int = 0 as libc::c_int;
            while i <
                      (0xffff as libc::c_int -
                           (0xffff as libc::c_int >> 1 as libc::c_int &
                                0x77777777 as libc::c_int) -
                           (0xffff as libc::c_int >> 2 as libc::c_int &
                                0x33333333 as libc::c_int) -
                           (0xffff as libc::c_int >> 3 as libc::c_int &
                                0x11111111 as libc::c_int) +
                           (0xffff as libc::c_int -
                                (0xffff as libc::c_int >> 1 as libc::c_int &
                                     0x77777777 as libc::c_int) -
                                (0xffff as libc::c_int >> 2 as libc::c_int &
                                     0x33333333 as libc::c_int) -
                                (0xffff as libc::c_int >> 3 as libc::c_int &
                                     0x11111111 as libc::c_int) >>
                                4 as libc::c_int) & 0xf0f0f0f as libc::c_int)
                          % 255 as libc::c_int +
                          (0xffff as libc::c_int -
                               (0xffff as libc::c_int >> 1 as libc::c_int &
                                    0x77777777 as libc::c_int) -
                               (0xffff as libc::c_int >> 2 as libc::c_int &
                                    0x33333333 as libc::c_int) -
                               (0xffff as libc::c_int >> 3 as libc::c_int &
                                    0x11111111 as libc::c_int) +
                               (0xffff as libc::c_int -
                                    (0xffff as libc::c_int >> 1 as libc::c_int
                                         & 0x77777777 as libc::c_int) -
                                    (0xffff as libc::c_int >> 2 as libc::c_int
                                         & 0x33333333 as libc::c_int) -
                                    (0xffff as libc::c_int >> 3 as libc::c_int
                                         & 0x11111111 as libc::c_int) >>
                                    4 as libc::c_int) &
                               0xf0f0f0f as libc::c_int) % 255 as libc::c_int
                          +
                          (((1 as libc::c_int) << 13 as libc::c_int |
                                (1 as libc::c_int) << 14 as libc::c_int |
                                (1 as libc::c_int) << 15 as libc::c_int) -
                               (((1 as libc::c_int) << 13 as libc::c_int |
                                     (1 as libc::c_int) << 14 as libc::c_int |
                                     (1 as libc::c_int) << 15 as libc::c_int)
                                    >> 1 as libc::c_int &
                                    0x77777777 as libc::c_int) -
                               (((1 as libc::c_int) << 13 as libc::c_int |
                                     (1 as libc::c_int) << 14 as libc::c_int |
                                     (1 as libc::c_int) << 15 as libc::c_int)
                                    >> 2 as libc::c_int &
                                    0x33333333 as libc::c_int) -
                               (((1 as libc::c_int) << 13 as libc::c_int |
                                     (1 as libc::c_int) << 14 as libc::c_int |
                                     (1 as libc::c_int) << 15 as libc::c_int)
                                    >> 3 as libc::c_int &
                                    0x11111111 as libc::c_int) +
                               (((1 as libc::c_int) << 13 as libc::c_int |
                                     (1 as libc::c_int) << 14 as libc::c_int |
                                     (1 as libc::c_int) << 15 as libc::c_int)
                                    -
                                    (((1 as libc::c_int) << 13 as libc::c_int
                                          |
                                          (1 as libc::c_int) <<
                                              14 as libc::c_int |
                                          (1 as libc::c_int) <<
                                              15 as libc::c_int) >>
                                         1 as libc::c_int &
                                         0x77777777 as libc::c_int) -
                                    (((1 as libc::c_int) << 13 as libc::c_int
                                          |
                                          (1 as libc::c_int) <<
                                              14 as libc::c_int |
                                          (1 as libc::c_int) <<
                                              15 as libc::c_int) >>
                                         2 as libc::c_int &
                                         0x33333333 as libc::c_int) -
                                    (((1 as libc::c_int) << 13 as libc::c_int
                                          |
                                          (1 as libc::c_int) <<
                                              14 as libc::c_int |
                                          (1 as libc::c_int) <<
                                              15 as libc::c_int) >>
                                         3 as libc::c_int &
                                         0x11111111 as libc::c_int) >>
                                    4 as libc::c_int) &
                               0xf0f0f0f as libc::c_int) % 255 as libc::c_int
                          + 0 as libc::c_int + 0 as libc::c_int +
                          0 as libc::c_int + 0 as libc::c_int {
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
                       libc::c_int > 0 as libc::c_int {
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
    let mut resourceIndex: uint8_t = 0 as libc::c_int as uint8_t;
    let mut index: libc::c_int = 0 as libc::c_int;
    let mut pch: *mut libc::c_char = 0 as *mut libc::c_char;
    let mut saveptr: *mut libc::c_char = 0 as *mut libc::c_char;
    pch =
        strtok_r(cmdline, b" \x00" as *const u8 as *const libc::c_char,
                 &mut saveptr);
    resourceIndex = 0 as libc::c_int as uint8_t;
    loop  {
        if resourceIndex as libc::c_ulong >=
               (::core::mem::size_of::<[cliResourceValue_t; 19]>() as
                    libc::c_ulong).wrapping_div(::core::mem::size_of::<cliResourceValue_t>()
                                                    as libc::c_ulong) {
            cliPrintErrorLinef(b"Invalid\x00" as *const u8 as
                                   *const libc::c_char);
            return
        }
        if strncasecmp(pch,
                       ownerNames[resourceTable[resourceIndex as usize].owner
                                      as usize], len as libc::c_ulong) ==
               0 as libc::c_int {
            break ;
        }
        resourceIndex = resourceIndex.wrapping_add(1)
    }
    pch =
        strtok_r(0 as *mut libc::c_char,
                 b" \x00" as *const u8 as *const libc::c_char, &mut saveptr);
    index = atoi(pch);
    if resourceTable[resourceIndex as usize].maxIndex as libc::c_int >
           0 as libc::c_int || index > 0 as libc::c_int {
        if index <= 0 as libc::c_int ||
               index >
                   (if resourceTable[resourceIndex as usize].maxIndex as
                           libc::c_int == 0 as libc::c_int {
                        1 as libc::c_int
                    } else {
                        resourceTable[resourceIndex as usize].maxIndex as
                            libc::c_int
                    }) {
            cliShowArgumentRangeError(b"index\x00" as *const u8 as
                                          *const libc::c_char as
                                          *mut libc::c_char, 1 as libc::c_int,
                                      if resourceTable[resourceIndex as
                                                           usize].maxIndex as
                                             libc::c_int == 0 as libc::c_int {
                                          1 as libc::c_int
                                      } else {
                                          resourceTable[resourceIndex as
                                                            usize].maxIndex as
                                              libc::c_int
                                      });
            return
        }
        index -= 1 as libc::c_int;
        pch =
            strtok_r(0 as *mut libc::c_char,
                     b" \x00" as *const u8 as *const libc::c_char,
                     &mut saveptr)
    }
    let mut tag: *mut ioTag_t =
        getIoTag(resourceTable[resourceIndex as usize], index as uint8_t);
    if strlen(pch) > 0 as libc::c_int as libc::c_ulong {
        if strToPin(pch, tag) {
            if *tag as libc::c_int == 0 as libc::c_int {
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
    let mut i: libc::c_int = 1 as libc::c_int;
    while i <= DMA_LAST_HANDLER as libc::c_int {
        let mut owner: *const libc::c_char = 0 as *const libc::c_char;
        owner = ownerNames[dmaGetOwner(i as dmaIdentifier_e) as usize];
        cliPrintf(b"DMA%d Channel %d:\x00" as *const u8 as
                      *const libc::c_char,
                  (i - 1 as libc::c_int) / 7 as libc::c_int +
                      1 as libc::c_int,
                  (i - 1 as libc::c_int) % 7 as libc::c_int +
                      1 as libc::c_int);
        let mut resourceIndex: uint8_t =
            dmaGetResourceIndex(i as dmaIdentifier_e);
        if resourceIndex as libc::c_int > 0 as libc::c_int {
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
        printFeature(dumpMask, &mut featureConfig_Copy, featureConfig());
        printBeeper(dumpMask, beeperConfig_Copy.beeper_off_flags,
                    (*beeperConfig()).beeper_off_flags,
                    b"beeper\x00" as *const u8 as *const libc::c_char);
        // USE_BEEPER
        printMap(dumpMask, &mut rxConfig_Copy, rxConfig());
        printSerial(dumpMask, &mut serialConfig_Copy, serialConfig());
        printLed(dumpMask, ledStripConfig_Copy.ledConfigs.as_mut_ptr(),
                 (*ledStripConfig()).ledConfigs.as_ptr());
        printColor(dumpMask, ledStripConfig_Copy.colors.as_mut_ptr(),
                   (*ledStripConfig()).colors.as_ptr());
        printModeColor(dumpMask, &mut ledStripConfig_Copy, ledStripConfig());
        printAux(dumpMask, modeActivationConditions_CopyArray.as_mut_ptr(),
                 modeActivationConditions(0 as libc::c_int));
        printAdjustmentRange(dumpMask,
                             adjustmentRanges_CopyArray.as_mut_ptr(),
                             adjustmentRanges(0 as libc::c_int));
        printRxRange(dumpMask, rxChannelRangeConfigs_CopyArray.as_mut_ptr(),
                     rxChannelRangeConfigs(0 as libc::c_int));
        printRxFailsafe(dumpMask,
                        rxFailsafeChannelConfigs_CopyArray.as_mut_ptr(),
                        rxFailsafeChannelConfigs(0 as libc::c_int));
        dumpAllValues(MASTER_VALUE as libc::c_int as uint16_t, dumpMask);
        if dumpMask as libc::c_int & DUMP_ALL as libc::c_int != 0 {
            let mut pidProfileIndex: uint32_t = 0 as libc::c_int as uint32_t;
            while pidProfileIndex < 3 as libc::c_int as libc::c_uint {
                cliDumpPidProfile(pidProfileIndex as uint8_t, dumpMask);
                pidProfileIndex = pidProfileIndex.wrapping_add(1)
            }
            pidProfileIndexToUse =
                systemConfig_Copy.pidProfileIndex as int8_t;
            cliProfile(b"\x00" as *const u8 as *const libc::c_char as
                           *mut libc::c_char);
            pidProfileIndexToUse = -(1 as libc::c_int) as int8_t;
            let mut rateIndex: uint32_t = 0 as libc::c_int as uint32_t;
            while rateIndex < 6 as libc::c_int as libc::c_uint {
                cliDumpRateProfile(rateIndex as uint8_t, dumpMask);
                rateIndex = rateIndex.wrapping_add(1)
            }
            rateProfileIndexToUse =
                systemConfig_Copy.activeRateProfile as int8_t;
            cliRateProfile(b"\x00" as *const u8 as *const libc::c_char as
                               *mut libc::c_char);
            rateProfileIndexToUse = -(1 as libc::c_int) as int8_t;
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
    printConfig(cmdline, 0 as libc::c_int != 0);
}
unsafe extern "C" fn cliDiff(mut cmdline: *mut libc::c_char) {
    printConfig(cmdline, 1 as libc::c_int != 0);
}
// should be sorted a..z for bsearch()
#[no_mangle]
pub static mut cmdTable: [clicmd_t; 33] =
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
         }]
    };
unsafe extern "C" fn cliHelp(mut cmdline: *mut libc::c_char) {
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    while (i as libc::c_ulong) <
              (::core::mem::size_of::<[clicmd_t; 33]>() as
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
                      cmdTable.as_ptr().offset((::core::mem::size_of::<[clicmd_t; 33]>()
                                                    as
                                                    libc::c_ulong).wrapping_div(::core::mem::size_of::<clicmd_t>()
                                                                                    as
                                                                                    libc::c_ulong)
                                                   as isize) {
                if !(bufferIndex != 0 &&
                         strncasecmp(cliBuffer.as_mut_ptr(), (*cmd).name,
                                     bufferIndex as libc::c_ulong) !=
                             0 as libc::c_int) {
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
                               (::core::mem::size_of::<[libc::c_char; 128]>()
                                    as
                                    libc::c_ulong).wrapping_sub(2 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_ulong)
                       {
                        /* Unambiguous -- append a space */
                        let fresh6 = bufferIndex;
                        bufferIndex = bufferIndex.wrapping_add(1);
                        cliBuffer[fresh6 as usize] =
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
                i = 0 as libc::c_int as uint32_t
                /* Redraw prompt */
            }
            while i < bufferIndex {
                cliWrite(cliBuffer[i as usize] as uint8_t);
                i = i.wrapping_add(1)
            }
        } else if bufferIndex == 0 && c as libc::c_int == 4 as libc::c_int {
            // CTRL-D
            cliExit(cliBuffer.as_mut_ptr());
            return
        } else if c as libc::c_int == 12 as libc::c_int {
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
            while bufferIndex > 0 as libc::c_int as libc::c_uint &&
                      cliBuffer[bufferIndex.wrapping_sub(1 as libc::c_int as
                                                             libc::c_uint) as
                                    usize] as libc::c_int == ' ' as i32 {
                bufferIndex = bufferIndex.wrapping_sub(1)
            }
            // Process non-empty lines
            if bufferIndex > 0 as libc::c_int as libc::c_uint {
                cliBuffer[bufferIndex as usize] =
                    0 as libc::c_int as libc::c_char; // null terminate
                let mut cmd_0: *const clicmd_t = 0 as *const clicmd_t;
                let mut options: *mut libc::c_char = 0 as *mut libc::c_char;
                cmd_0 = cmdTable.as_ptr();
                while cmd_0 <
                          cmdTable.as_ptr().offset((::core::mem::size_of::<[clicmd_t; 33]>()
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
                       cmdTable.as_ptr().offset((::core::mem::size_of::<[clicmd_t; 33]>()
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
                bufferIndex = 0 as libc::c_int as uint32_t
            }
            memset(cliBuffer.as_mut_ptr() as *mut libc::c_void,
                   0 as libc::c_int,
                   ::core::mem::size_of::<[libc::c_char; 128]>() as
                       libc::c_ulong);
            // 'exit' will reset this flag, so we don't need to print prompt again
            if cliMode == 0 { return }
            cliPrompt();
        } else if c as libc::c_int == 127 as libc::c_int {
            // backspace
            if bufferIndex != 0 {
                bufferIndex =
                    bufferIndex.wrapping_sub(1); // Ignore leading spaces
                cliBuffer[bufferIndex as usize] =
                    0 as libc::c_int as libc::c_char;
                cliPrint(b"\x08 \x08\x00" as *const u8 as
                             *const libc::c_char);
            }
        } else {
            if !((bufferIndex as libc::c_ulong) <
                     ::core::mem::size_of::<[libc::c_char; 128]>() as
                         libc::c_ulong &&
                     c as libc::c_int >= 32 as libc::c_int &&
                     c as libc::c_int <= 126 as libc::c_int) {
                continue ;
            }
            if bufferIndex == 0 && c as libc::c_int == ' ' as i32 {
                continue ;
            }
            let fresh7 = bufferIndex;
            bufferIndex = bufferIndex.wrapping_add(1);
            cliBuffer[fresh7 as usize] = c as libc::c_char;
            cliWrite(c);
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn cliEnter(mut serialPort: *mut serialPort_t) {
    cliMode = 1 as libc::c_int as uint8_t;
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