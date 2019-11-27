use core;
use libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
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
    fn trampCmsUpdateStatusString();
    #[no_mangle]
    fn vtxCommonSetDevice(vtxDevice: *mut vtxDevice_t);
    #[no_mangle]
    fn serialRxBytesWaiting(instance: *const serialPort_t) -> uint32_t;
    #[no_mangle]
    fn serialWriteBuf(instance: *mut serialPort_t, data: *const uint8_t,
                      count: libc::c_int);
    #[no_mangle]
    fn serialRead(instance: *mut serialPort_t) -> uint8_t;
    #[no_mangle]
    fn findSerialPortConfig(function: serialPortFunction_e)
     -> *mut serialPortConfig_t;
    #[no_mangle]
    fn openSerialPort(identifier: serialPortIdentifier_e,
                      function: serialPortFunction_e,
                      rxCallback: serialReceiveCallbackPtr,
                      rxCallbackData: *mut libc::c_void, baudrate: uint32_t,
                      mode: portMode_e, options: portOptions_e)
     -> *mut serialPort_t;
    #[no_mangle]
    static mut vtxConfig_System: vtxConfig_t;
    #[no_mangle]
    static vtx58BandNames: [*const libc::c_char; 0];
    #[no_mangle]
    static vtx58ChannelNames: [*const libc::c_char; 0];
    #[no_mangle]
    fn vtx58_Freq2Bandchan(freq: uint16_t, pBand: *mut uint8_t,
                           pChannel: *mut uint8_t) -> bool;
    #[no_mangle]
    fn vtx58_Bandchan2Freq(band: uint8_t, channel: uint8_t) -> uint16_t;
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
// microsecond time
pub type timeUs_t = uint32_t;
pub type vtxDevType_e = libc::c_uint;
pub const VTXDEV_UNKNOWN: vtxDevType_e = 255;
pub const VTXDEV_TRAMP: vtxDevType_e = 4;
pub const VTXDEV_SMARTAUDIO: vtxDevType_e = 3;
pub const VTXDEV_RTC6705: vtxDevType_e = 1;
pub const VTXDEV_UNSUPPORTED: vtxDevType_e = 0;
#[derive ( Copy, Clone )]
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
#[derive ( Copy, Clone )]
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
#[derive ( Copy, Clone )]
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
pub type vtxVTable_t = vtxVTable_s;
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
// Define known line control states which may be passed up by underlying serial driver callback
pub type serialReceiveCallbackPtr
    =
    Option<unsafe extern "C" fn(_: uint16_t, _: *mut libc::c_void) -> ()>;
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
// used by serial drivers to return frames to app
pub type serialPort_t = serialPort_s;
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
// 1
// 2
// 4
// 8
// 16
// 32
// 64
// 128
// 512
// 1024
// 2048
// 4096
// 8192
// 16384
// 32768
// serial port identifiers are now fixed, these values are used by MSP commands.
//
// runtime
//
//
// configuration
//
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct serialPortConfig_s {
    pub functionMask: uint16_t,
    pub identifier: serialPortIdentifier_e,
    pub msp_baudrateIndex: uint8_t,
    pub gps_baudrateIndex: uint8_t,
    pub blackbox_baudrateIndex: uint8_t,
    pub telemetry_baudrateIndex: uint8_t,
    // not used for all telemetry systems, e.g. HoTT only works at 19200.
}
pub type serialPortConfig_t = serialPortConfig_s;
pub const TRAMP_STATUS_OFFLINE: trampStatus_e = 0;
pub type trampStatus_e = libc::c_int;
pub const TRAMP_STATUS_CHECK_FREQ_PW: trampStatus_e = 3;
pub const TRAMP_STATUS_SET_FREQ_PW: trampStatus_e = 2;
pub const TRAMP_STATUS_ONLINE: trampStatus_e = 1;
pub const TRAMP_STATUS_BAD_DEVICE: trampStatus_e = -1;
pub type trampReceiveState_e = libc::c_uint;
pub const S_DATA: trampReceiveState_e = 2;
pub const S_WAIT_CODE: trampReceiveState_e = 1;
pub const S_WAIT_LEN: trampReceiveState_e = 0;
pub type vtxConfig_t = vtxConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct vtxConfig_s {
    pub vtxChannelActivationConditions: [vtxChannelActivationCondition_t; 10],
    pub halfDuplex: uint8_t,
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
// steps are 25 apart
// a value of 0 corresponds to a channel value of 900 or less
// a value of 48 corresponds to a channel value of 2100 or more
// 48 steps between 900 and 2100
pub type channelRange_t = channelRange_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct channelRange_s {
    pub startStep: uint8_t,
    pub endStep: uint8_t,
}
#[inline]
unsafe extern "C" fn cmp32(mut a: uint32_t, mut b: uint32_t) -> int32_t {
    return a.wrapping_sub(b) as int32_t;
}
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[inline]
unsafe extern "C" fn vtxConfig() -> *const vtxConfig_t {
    return &mut vtxConfig_System;
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
/* Created by jflyper */
#[no_mangle]
pub static mut trampPowerTable: [uint16_t; 5] =
    [25i32 as uint16_t, 100i32 as uint16_t, 200i32 as uint16_t,
     400i32 as uint16_t, 600i32 as uint16_t];
#[no_mangle]
pub static mut trampPowerNames: [*const libc::c_char; 6] =
    [b"---\x00" as *const u8 as *const libc::c_char,
     b"25 \x00" as *const u8 as *const libc::c_char,
     b"100\x00" as *const u8 as *const libc::c_char,
     b"200\x00" as *const u8 as *const libc::c_char,
     b"400\x00" as *const u8 as *const libc::c_char,
     b"600\x00" as *const u8 as *const libc::c_char];
// forward
static mut vtxTramp: vtxDevice_t =
    unsafe {
        {
            let mut init =
                vtxDevice_s{vTable: &trampVTable as *const vtxVTable_t,
                            capability:
                                {
                                    let mut init =
                                        vtxDeviceCapability_s{bandCount:
                                                                  (5i32 - 1i32
                                                                       + 1i32)
                                                                      as
                                                                      uint8_t,
                                                              channelCount:
                                                                  (8i32 - 1i32
                                                                       + 1i32)
                                                                      as
                                                                      uint8_t,
                                                              powerCount:
                                                                  ::core::mem::size_of::<[uint16_t; 5]>()
                                                                      as
                                                                      libc::c_ulong
                                                                      as
                                                                      uint8_t,
                                                              filler: 0,};
                                    init
                                },
                            frequencyTable:
                                0 as *const uint16_t as *mut uint16_t,
                            bandNames:
                                vtx58BandNames.as_ptr() as
                                    *mut *mut libc::c_char,
                            channelNames:
                                vtx58ChannelNames.as_ptr() as
                                    *mut *mut libc::c_char,
                            powerNames:
                                trampPowerNames.as_ptr() as
                                    *mut *mut libc::c_char,
                            frequency: 0,
                            band: 0,
                            channel: 0,
                            powerIndex: 0,
                            pitMode: 0,};
            init
        }
    };
static mut trampSerialPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
static mut trampReqBuffer: [uint8_t; 16] = [0; 16];
static mut trampRespBuffer: [uint8_t; 16] = [0; 16];
#[no_mangle]
pub static mut trampStatus: trampStatus_e = TRAMP_STATUS_OFFLINE;
#[no_mangle]
pub static mut trampRFFreqMin: uint32_t = 0;
#[no_mangle]
pub static mut trampRFFreqMax: uint32_t = 0;
#[no_mangle]
pub static mut trampRFPowerMax: uint32_t = 0;
#[no_mangle]
pub static mut trampSetByFreqFlag: bool = 0i32 != 0;
//false = set via band/channel
#[no_mangle]
pub static mut trampCurFreq: uint32_t = 0i32 as uint32_t;
#[no_mangle]
pub static mut trampBand: uint8_t = 0i32 as uint8_t;
#[no_mangle]
pub static mut trampChannel: uint8_t = 0i32 as uint8_t;
#[no_mangle]
pub static mut trampPower: uint16_t = 0i32 as uint16_t;
// Actual transmitting power
#[no_mangle]
pub static mut trampConfiguredPower: uint16_t = 0i32 as uint16_t;
// Configured transmitting power
#[no_mangle]
pub static mut trampTemperature: int16_t = 0i32 as int16_t;
#[no_mangle]
pub static mut trampPitMode: uint8_t = 0i32 as uint8_t;
#[no_mangle]
pub static mut trampConfFreq: uint32_t = 0i32 as uint32_t;
#[no_mangle]
pub static mut trampFreqRetries: uint8_t = 0i32 as uint8_t;
#[no_mangle]
pub static mut trampConfPower: uint16_t = 0i32 as uint16_t;
#[no_mangle]
pub static mut trampPowerRetries: uint8_t = 0i32 as uint8_t;
unsafe extern "C" fn trampWriteBuf(mut buf: *mut uint8_t) {
    serialWriteBuf(trampSerialPort, buf, 16i32); //set freq via MHz value
}
unsafe extern "C" fn trampChecksum(mut trampBuf: *mut uint8_t) -> uint8_t {
    let mut cksum: uint8_t = 0i32 as uint8_t; //set freq via band/channel
    let mut i: libc::c_int = 1i32;
    while i < 14i32 {
        cksum =
            (cksum as libc::c_int +
                 *trampBuf.offset(i as isize) as libc::c_int) as uint8_t;
        i += 1
    }
    return cksum;
}
#[no_mangle]
pub unsafe extern "C" fn trampCmdU16(mut cmd: uint8_t, mut param: uint16_t) {
    if trampSerialPort.is_null() { return }
    memset(trampReqBuffer.as_mut_ptr() as *mut libc::c_void, 0i32,
           (::core::mem::size_of::<[uint8_t; 16]>() as
                libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                as libc::c_ulong));
    trampReqBuffer[0] = 15i32 as uint8_t;
    trampReqBuffer[1] = cmd;
    trampReqBuffer[2] = (param as libc::c_int & 0xffi32) as uint8_t;
    trampReqBuffer[3] = (param as libc::c_int >> 8i32 & 0xffi32) as uint8_t;
    trampReqBuffer[14] = trampChecksum(trampReqBuffer.as_mut_ptr());
    trampWriteBuf(trampReqBuffer.as_mut_ptr());
}
unsafe extern "C" fn trampValidateFreq(mut freq: uint16_t) -> bool {
    return freq as libc::c_int >= 5000i32 && freq as libc::c_int <= 5999i32;
}
unsafe extern "C" fn trampDevSetFreq(mut freq: uint16_t) {
    trampConfFreq = freq as uint32_t;
    if trampConfFreq != trampCurFreq { trampFreqRetries = 2i32 as uint8_t };
}
#[no_mangle]
pub unsafe extern "C" fn trampSetFreq(mut freq: uint16_t) {
    trampSetByFreqFlag = 1i32 != 0;
    trampDevSetFreq(freq);
}
#[no_mangle]
pub unsafe extern "C" fn trampSendFreq(mut freq: uint16_t) {
    trampCmdU16('F' as i32 as uint8_t, freq);
}
unsafe extern "C" fn trampValidateBandAndChannel(mut band: uint8_t,
                                                 mut channel: uint8_t)
 -> bool {
    return band as libc::c_int >= 1i32 && band as libc::c_int <= 5i32 &&
               channel as libc::c_int >= 1i32 &&
               channel as libc::c_int <= 8i32;
}
unsafe extern "C" fn trampDevSetBandAndChannel(mut band: uint8_t,
                                               mut channel: uint8_t) {
    trampDevSetFreq(vtx58_Bandchan2Freq(band, channel));
}
#[no_mangle]
pub unsafe extern "C" fn trampSetBandAndChannel(mut band: uint8_t,
                                                mut channel: uint8_t) {
    trampSetByFreqFlag = 0i32 != 0;
    trampDevSetBandAndChannel(band, channel);
}
#[no_mangle]
pub unsafe extern "C" fn trampSetRFPower(mut level: uint16_t) {
    trampConfPower = level;
    if trampConfPower as libc::c_int != trampPower as libc::c_int {
        trampPowerRetries = 2i32 as uint8_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn trampSendRFPower(mut level: uint16_t) {
    trampCmdU16('P' as i32 as uint8_t, level);
}
// return false if error
#[no_mangle]
pub unsafe extern "C" fn trampCommitChanges() -> bool {
    if trampStatus as libc::c_int != TRAMP_STATUS_ONLINE as libc::c_int {
        return 0i32 != 0
    }
    trampStatus = TRAMP_STATUS_SET_FREQ_PW;
    return 1i32 != 0;
}
// return false if index out of range
unsafe extern "C" fn trampDevSetPowerByIndex(mut index: uint8_t) -> bool {
    if index as libc::c_int > 0i32 &&
           index as libc::c_ulong <=
               ::core::mem::size_of::<[uint16_t; 5]>() as libc::c_ulong {
        trampSetRFPower(trampPowerTable[(index as libc::c_int - 1i32) as
                                            usize]);
        trampCommitChanges();
        return 1i32 != 0
    }
    return 0i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn trampSetPitMode(mut onoff: uint8_t) {
    trampCmdU16('I' as i32 as uint8_t,
                if onoff as libc::c_int != 0 { 0i32 } else { 1i32 } as
                    uint16_t);
}
// returns completed response code
#[no_mangle]
pub unsafe extern "C" fn trampHandleResponse() -> libc::c_char {
    let respCode: uint8_t = trampRespBuffer[1];
    match respCode as libc::c_int {
        114 => {
            let min_freq: uint16_t =
                (trampRespBuffer[2] as libc::c_int |
                     (trampRespBuffer[3] as libc::c_int) << 8i32) as uint16_t;
            if min_freq as libc::c_int != 0i32 {
                trampRFFreqMin = min_freq as uint32_t;
                trampRFFreqMax =
                    (trampRespBuffer[4] as libc::c_int |
                         (trampRespBuffer[5] as libc::c_int) << 8i32) as
                        uint32_t;
                trampRFPowerMax =
                    (trampRespBuffer[6] as libc::c_int |
                         (trampRespBuffer[7] as libc::c_int) << 8i32) as
                        uint32_t;
                return 'r' as i32 as libc::c_char
            }
            // throw bytes echoed from tx to rx in bidirectional mode away
        }
        118 => {
            let freq: uint16_t =
                (trampRespBuffer[2] as libc::c_int |
                     (trampRespBuffer[3] as libc::c_int) << 8i32) as uint16_t;
            if freq as libc::c_int != 0i32 {
                trampCurFreq = freq as uint32_t;
                trampConfiguredPower =
                    (trampRespBuffer[4] as libc::c_int |
                         (trampRespBuffer[5] as libc::c_int) << 8i32) as
                        uint16_t;
                trampPitMode = trampRespBuffer[7];
                trampPower =
                    (trampRespBuffer[8] as libc::c_int |
                         (trampRespBuffer[9] as libc::c_int) << 8i32) as
                        uint16_t;
                // throw bytes echoed from tx to rx in bidirectional mode away
                // if no band/chan match then make sure set-by-freq mode is flagged
                if !vtx58_Freq2Bandchan(trampCurFreq as uint16_t,
                                        &mut trampBand, &mut trampChannel) {
                    trampSetByFreqFlag = 1i32 != 0
                }
                if trampConfFreq == 0i32 as libc::c_uint {
                    trampConfFreq = trampCurFreq
                }
                if trampConfPower as libc::c_int == 0i32 {
                    trampConfPower = trampPower
                }
                return 'v' as i32 as libc::c_char
            }
        }
        115 => {
            let temp: uint16_t =
                (trampRespBuffer[6] as libc::c_int |
                     (trampRespBuffer[7] as libc::c_int) << 8i32) as int16_t
                    as uint16_t;
            if temp as libc::c_int != 0i32 {
                trampTemperature = temp as int16_t;
                return 's' as i32 as libc::c_char
            }
        }
        _ => { }
    }
    return 0i32 as libc::c_char;
}
static mut trampReceiveState: trampReceiveState_e = S_WAIT_LEN;
static mut trampReceivePos: libc::c_int = 0i32;
unsafe extern "C" fn trampResetReceiver() {
    trampReceiveState = S_WAIT_LEN;
    trampReceivePos = 0i32;
}
unsafe extern "C" fn trampIsValidResponseCode(mut code: uint8_t) -> bool {
    if code as libc::c_int == 'r' as i32 || code as libc::c_int == 'v' as i32
           || code as libc::c_int == 's' as i32 {
        return 1i32 != 0
    } else { return 0i32 != 0 };
}
// returns completed response code or 0
unsafe extern "C" fn trampReceive(mut currentTimeUs: uint32_t)
 -> libc::c_char {
    if trampSerialPort.is_null() { return 0i32 as libc::c_char }
    while serialRxBytesWaiting(trampSerialPort) != 0 {
        let c: uint8_t = serialRead(trampSerialPort);
        let fresh0 = trampReceivePos;
        trampReceivePos = trampReceivePos + 1;
        trampRespBuffer[fresh0 as usize] = c;
        match trampReceiveState as libc::c_uint {
            0 => {
                if c as libc::c_int == 0xfi32 {
                    trampReceiveState = S_WAIT_CODE
                } else { trampReceivePos = 0i32 }
            }
            1 => {
                if trampIsValidResponseCode(c) {
                    trampReceiveState = S_DATA
                } else { trampResetReceiver(); }
            }
            2 => {
                if trampReceivePos == 16i32 {
                    let mut cksum: uint8_t =
                        trampChecksum(trampRespBuffer.as_mut_ptr());
                    trampResetReceiver();
                    if trampRespBuffer[14] as libc::c_int ==
                           cksum as libc::c_int &&
                           trampRespBuffer[15] as libc::c_int == 0i32 {
                        return trampHandleResponse()
                    }
                }
            }
            _ => { trampResetReceiver(); }
        }
    }
    return 0i32 as libc::c_char;
}
#[no_mangle]
pub unsafe extern "C" fn trampQuery(mut cmd: uint8_t) {
    trampResetReceiver();
    trampCmdU16(cmd, 0i32 as uint16_t);
}
#[no_mangle]
pub unsafe extern "C" fn trampQueryR() { trampQuery('r' as i32 as uint8_t); }
#[no_mangle]
pub unsafe extern "C" fn trampQueryV() { trampQuery('v' as i32 as uint8_t); }
#[no_mangle]
pub unsafe extern "C" fn trampQueryS() { trampQuery('s' as i32 as uint8_t); }
unsafe extern "C" fn vtxTrampProcess(mut vtxDevice: *mut vtxDevice_t,
                                     mut currentTimeUs: timeUs_t) {
    static mut lastQueryTimeUs: timeUs_t = 0i32 as timeUs_t;
    static mut initSettingsDoneFlag: bool = 0i32 != 0;
    if trampStatus as libc::c_int == TRAMP_STATUS_BAD_DEVICE as libc::c_int {
        return
    }
    let replyCode: libc::c_char = trampReceive(currentTimeUs);
    match replyCode as libc::c_int {
        114 => {
            if trampStatus as libc::c_int <=
                   TRAMP_STATUS_OFFLINE as libc::c_int {
                trampStatus = TRAMP_STATUS_ONLINE;
                // once device is ready enter vtx settings
                if !initSettingsDoneFlag {
                    initSettingsDoneFlag = 1i32 != 0
                    // if vtx_band!=0 then enter 'vtx_band/chan' values (and power)
                }
            }
        }
        118 => {
            if trampStatus as libc::c_int ==
                   TRAMP_STATUS_CHECK_FREQ_PW as libc::c_int {
                trampStatus = TRAMP_STATUS_SET_FREQ_PW
            }
        }
        _ => { }
    }
    match trampStatus as libc::c_int {
        0 | 1 => {
            if cmp32(currentTimeUs, lastQueryTimeUs) > 1000i32 * 1000i32 {
                // 1s
                if trampStatus as libc::c_int ==
                       TRAMP_STATUS_OFFLINE as libc::c_int {
                    trampQueryR();
                } else {
                    static mut cnt: libc::c_uint = 0i32 as libc::c_uint;
                    let fresh1 = cnt;
                    cnt = cnt.wrapping_add(1);
                    if fresh1 & 1i32 as libc::c_uint == 0i32 as libc::c_uint {
                        trampQueryV();
                    } else { trampQueryS(); }
                }
                lastQueryTimeUs = currentTimeUs
            }
        }
        2 => {
            let mut done: bool = 1i32 != 0;
            if trampConfFreq != 0 && trampFreqRetries as libc::c_int != 0 &&
                   trampConfFreq != trampCurFreq {
                trampSendFreq(trampConfFreq as uint16_t);
                trampFreqRetries = trampFreqRetries.wrapping_sub(1);
                done = 0i32 != 0
            } else if trampConfPower as libc::c_int != 0 &&
                          trampPowerRetries as libc::c_int != 0 &&
                          trampConfPower as libc::c_int !=
                              trampConfiguredPower as libc::c_int {
                trampSendRFPower(trampConfPower);
                trampPowerRetries = trampPowerRetries.wrapping_sub(1);
                done = 0i32 != 0
            }
            if !done {
                trampStatus = TRAMP_STATUS_CHECK_FREQ_PW;
                // delay next status query by 300ms
                lastQueryTimeUs =
                    currentTimeUs.wrapping_add((300i32 * 1000i32) as
                                                   libc::c_uint)
            } else {
                // everything has been done, let's return to original state
                trampStatus = TRAMP_STATUS_ONLINE;
                // reset configuration value in case it failed (no more retries)
                trampConfFreq = trampCurFreq;
                trampConfPower = trampPower;
                trampPowerRetries = 0i32 as uint8_t;
                trampFreqRetries = trampPowerRetries
            }
        }
        3 => {
            if cmp32(currentTimeUs, lastQueryTimeUs) > 200i32 * 1000i32 {
                trampQueryV();
                lastQueryTimeUs = currentTimeUs
            }
        }
        _ => { }
    }
    trampCmsUpdateStatusString();
}
// Interface to common VTX API
unsafe extern "C" fn vtxTrampGetDeviceType(mut vtxDevice: *const vtxDevice_t)
 -> vtxDevType_e {
    return VTXDEV_TRAMP;
}
unsafe extern "C" fn vtxTrampIsReady(mut vtxDevice: *const vtxDevice_t)
 -> bool {
    return !vtxDevice.is_null() &&
               trampStatus as libc::c_int >
                   TRAMP_STATUS_OFFLINE as libc::c_int;
}
unsafe extern "C" fn vtxTrampSetBandAndChannel(mut vtxDevice:
                                                   *mut vtxDevice_t,
                                               mut band: uint8_t,
                                               mut channel: uint8_t) {
    if trampValidateBandAndChannel(band, channel) {
        trampSetBandAndChannel(band, channel);
        trampCommitChanges();
    };
}
unsafe extern "C" fn vtxTrampSetPowerByIndex(mut vtxDevice: *mut vtxDevice_t,
                                             mut index: uint8_t) {
    trampDevSetPowerByIndex(index);
}
unsafe extern "C" fn vtxTrampSetPitMode(mut vtxDevice: *mut vtxDevice_t,
                                        mut onoff: uint8_t) {
    trampSetPitMode(onoff);
}
unsafe extern "C" fn vtxTrampSetFreq(mut vtxDevice: *mut vtxDevice_t,
                                     mut freq: uint16_t) {
    if trampValidateFreq(freq) { trampSetFreq(freq); trampCommitChanges(); };
}
unsafe extern "C" fn vtxTrampGetBandAndChannel(mut vtxDevice:
                                                   *const vtxDevice_t,
                                               mut pBand: *mut uint8_t,
                                               mut pChannel: *mut uint8_t)
 -> bool {
    if !vtxTrampIsReady(vtxDevice) { return 0i32 != 0 }
    // if in user-freq mode then report band as zero
    *pBand =
        if trampSetByFreqFlag as libc::c_int != 0 {
            0i32
        } else { trampBand as libc::c_int } as uint8_t;
    *pChannel = trampChannel;
    return 1i32 != 0;
}
unsafe extern "C" fn vtxTrampGetPowerIndex(mut vtxDevice: *const vtxDevice_t,
                                           mut pIndex: *mut uint8_t) -> bool {
    if !vtxTrampIsReady(vtxDevice) { return 0i32 != 0 }
    if trampConfiguredPower as libc::c_int > 0i32 {
        let mut i: uint8_t = 0i32 as uint8_t;
        while (i as libc::c_ulong) <
                  ::core::mem::size_of::<[uint16_t; 5]>() as libc::c_ulong {
            if trampConfiguredPower as libc::c_int <=
                   trampPowerTable[i as usize] as libc::c_int {
                *pIndex = (i as libc::c_int + 1i32) as uint8_t;
                break ;
            } else { i = i.wrapping_add(1) }
        }
    }
    return 1i32 != 0;
}
unsafe extern "C" fn vtxTrampGetPitMode(mut vtxDevice: *const vtxDevice_t,
                                        mut pOnOff: *mut uint8_t) -> bool {
    if !vtxTrampIsReady(vtxDevice) { return 0i32 != 0 }
    *pOnOff = trampPitMode;
    return 1i32 != 0;
}
unsafe extern "C" fn vtxTrampGetFreq(mut vtxDevice: *const vtxDevice_t,
                                     mut pFreq: *mut uint16_t) -> bool {
    if !vtxTrampIsReady(vtxDevice) { return 0i32 != 0 }
    *pFreq = trampCurFreq as uint16_t;
    return 1i32 != 0;
}
static mut trampVTable: vtxVTable_t =
    unsafe {
        {
            let mut init =
                vtxVTable_s{process:
                                Some(vtxTrampProcess as
                                         unsafe extern "C" fn(_:
                                                                  *mut vtxDevice_t,
                                                              _: timeUs_t)
                                             -> ()),
                            getDeviceType:
                                Some(vtxTrampGetDeviceType as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t)
                                             -> vtxDevType_e),
                            isReady:
                                Some(vtxTrampIsReady as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t)
                                             -> bool),
                            setBandAndChannel:
                                Some(vtxTrampSetBandAndChannel as
                                         unsafe extern "C" fn(_:
                                                                  *mut vtxDevice_t,
                                                              _: uint8_t,
                                                              _: uint8_t)
                                             -> ()),
                            setPowerByIndex:
                                Some(vtxTrampSetPowerByIndex as
                                         unsafe extern "C" fn(_:
                                                                  *mut vtxDevice_t,
                                                              _: uint8_t)
                                             -> ()),
                            setPitMode:
                                Some(vtxTrampSetPitMode as
                                         unsafe extern "C" fn(_:
                                                                  *mut vtxDevice_t,
                                                              _: uint8_t)
                                             -> ()),
                            setFrequency:
                                Some(vtxTrampSetFreq as
                                         unsafe extern "C" fn(_:
                                                                  *mut vtxDevice_t,
                                                              _: uint16_t)
                                             -> ()),
                            getBandAndChannel:
                                Some(vtxTrampGetBandAndChannel as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t,
                                                              _: *mut uint8_t,
                                                              _: *mut uint8_t)
                                             -> bool),
                            getPowerIndex:
                                Some(vtxTrampGetPowerIndex as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t,
                                                              _: *mut uint8_t)
                                             -> bool),
                            getPitMode:
                                Some(vtxTrampGetPitMode as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t,
                                                              _: *mut uint8_t)
                                             -> bool),
                            getFrequency:
                                Some(vtxTrampGetFreq as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t,
                                                              _:
                                                                  *mut uint16_t)
                                             -> bool),};
            init
        }
    };
//max freq in MHz
// Actual transmitting power
// Configured transmitting power
#[no_mangle]
pub unsafe extern "C" fn vtxTrampInit() -> bool {
    let mut portConfig: *mut serialPortConfig_t =
        findSerialPortConfig(FUNCTION_VTX_TRAMP);
    if !portConfig.is_null() {
        let mut portOptions: portOptions_e = SERIAL_NOT_INVERTED;
        portOptions =
            (portOptions as libc::c_uint |
                 (if (*vtxConfig()).halfDuplex as libc::c_int != 0 {
                      SERIAL_BIDIR as libc::c_int
                  } else { SERIAL_UNIDIR as libc::c_int }) as libc::c_uint) as
                portOptions_e;
        trampSerialPort =
            openSerialPort((*portConfig).identifier, FUNCTION_VTX_TRAMP, None,
                           0 as *mut libc::c_void, 9600i32 as uint32_t,
                           MODE_RXTX, portOptions)
    }
    if trampSerialPort.is_null() { return 0i32 != 0 }
    vtxCommonSetDevice(&mut vtxTramp);
    return 1i32 != 0;
}
// VTX_TRAMP
