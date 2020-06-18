use core;
use libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memcmp(_: *const libc::c_void, _: *const libc::c_void,
              _: libc::c_ulong) -> libc::c_int;
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
    fn millis() -> timeMs_t;
    #[no_mangle]
    fn vtxCommonSetDevice(vtxDevice: *mut vtxDevice_t);
    #[no_mangle]
    fn serialWrite(instance: *mut serialPort_t, ch: uint8_t);
    #[no_mangle]
    fn serialRxBytesWaiting(instance: *const serialPort_t) -> uint32_t;
    #[no_mangle]
    fn serialRead(instance: *mut serialPort_t) -> uint8_t;
    #[no_mangle]
    fn findSerialPortConfig(function: serialPortFunction_e)
     -> *mut serialPortConfig_t;
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
    static mut vtxConfig_System: vtxConfig_t;
    #[no_mangle]
    static vtx58BandNames: [*const libc::c_char; 0];
    #[no_mangle]
    static vtx58ChannelNames: [*const libc::c_char; 0];
    #[no_mangle]
    fn vtx58_Bandchan2Freq(band: uint8_t, channel: uint8_t) -> uint16_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
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
//
// configuration
//
pub type serialPortConfig_t = serialPortConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct channelRange_s {
    pub startStep: uint8_t,
    pub endStep: uint8_t,
}
// not used for all telemetry systems, e.g. HoTT only works at 19200.
// steps are 25 apart
// a value of 0 corresponds to a channel value of 900 or less
// a value of 48 corresponds to a channel value of 2100 or more
// 48 steps between 900 and 2100
pub type channelRange_t = channelRange_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct vtxChannelActivationCondition_s {
    pub auxChannelIndex: uint8_t,
    pub band: uint8_t,
    pub channel: uint8_t,
    pub range: channelRange_t,
}
pub type vtxChannelActivationCondition_t = vtxChannelActivationCondition_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct vtxConfig_s {
    pub vtxChannelActivationConditions: [vtxChannelActivationCondition_t; 10],
    pub halfDuplex: uint8_t,
}
pub type vtxConfig_t = vtxConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct smartAudioDevice_s {
    pub version: int8_t,
    pub channel: int8_t,
    pub power: int8_t,
    pub mode: int8_t,
    pub freq: uint16_t,
    pub orfreq: uint16_t,
}
// For generic API use, but here for now
pub type smartAudioDevice_t = smartAudioDevice_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct saPowerTable_s {
    pub rfpower: libc::c_int,
    pub valueV1: int16_t,
    pub valueV2: int16_t,
}
pub type saPowerTable_t = saPowerTable_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct smartAudioStat_s {
    pub pktsent: uint16_t,
    pub pktrcvd: uint16_t,
    pub badpre: uint16_t,
    pub badlen: uint16_t,
    pub crc: uint16_t,
    pub ooopresp: uint16_t,
    pub badcode: uint16_t,
}
pub type smartAudioStat_t = smartAudioStat_s;
pub const SA_CMD_SET_CHAN: C2RustUnnamed_0 = 3;
// Command queue management
pub type saCmdQueue_t = saCmdQueue_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct saCmdQueue_s {
    pub buf: *mut uint8_t,
    pub len: libc::c_int,
}
pub const SA_CMD_SET_MODE: C2RustUnnamed_0 = 5;
pub const SA_CMD_SET_POWER: C2RustUnnamed_0 = 2;
pub const SA_CMD_SET_FREQ: C2RustUnnamed_0 = 4;
pub const SA_CMD_NONE: C2RustUnnamed_0 = 0;
pub const SA_CMD_GET_SETTINGS: C2RustUnnamed_0 = 1;
pub type saFramerState_e = libc::c_uint;
// Waiting for CRC
// Receiving data
pub const S_WAITCRC: saFramerState_e = 5;
// Waiting for length
pub const S_DATA: saFramerState_e = 4;
// Waiting for response code
pub const S_WAITLEN: saFramerState_e = 3;
// Waiting for preamble 2 (0x55)
pub const S_WAITRESP: saFramerState_e = 2;
// Waiting for preamble 1 (0xAA)
pub const S_WAITPRE2: saFramerState_e = 1;
pub const S_WAITPRE1: saFramerState_e = 0;
pub const SA_CMD_GET_SETTINGS_V2: C2RustUnnamed_0 = 9;
// SmartAudio command and response codes
pub type C2RustUnnamed_0 = libc::c_uint;
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[inline]
unsafe extern "C" fn vtxConfig() -> *const vtxConfig_t {
    return &mut vtxConfig_System;
}
// Response only
// Time window after command polling for state change
//#define USE_SMARTAUDIO_DPRINTF
//#define DPRINTF_SERIAL_PORT SERIAL_PORT_USART1
// USE_SMARTAUDIO_DPRINTF
static mut smartAudioSerialPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
#[no_mangle]
pub static mut saPowerNames: [*const libc::c_char; 5] =
    [b"---\x00" as *const u8 as *const libc::c_char,
     b"25 \x00" as *const u8 as *const libc::c_char,
     b"200\x00" as *const u8 as *const libc::c_char,
     b"500\x00" as *const u8 as *const libc::c_char,
     b"800\x00" as *const u8 as *const libc::c_char];
// Forward
static mut vtxSmartAudio: vtxDevice_t =
    unsafe {
        {
            let mut init =
                vtxDevice_s{vTable: &saVTable as *const vtxVTable_t,
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
                                                                  4i32 as
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
                                saPowerNames.as_ptr() as
                                    *mut *mut libc::c_char,
                            frequency: 0,
                            band: 0,
                            channel: 0,
                            powerIndex: 0,
                            pitMode: 0,};
            init
        }
    };
#[no_mangle]
pub static mut smartAudioCommand_e: C2RustUnnamed_0 = SA_CMD_NONE;
// This is not a good design; can't distinguish command from response this way.
// convert between 'saDevice.channel' and band/channel values
// Statistical counters, for user side trouble shooting.
#[no_mangle]
pub static mut saStat: smartAudioStat_t =
    {
        let mut init =
            smartAudioStat_s{pktsent: 0i32 as uint16_t,
                             pktrcvd: 0i32 as uint16_t,
                             badpre: 0i32 as uint16_t,
                             badlen: 0i32 as uint16_t,
                             crc: 0i32 as uint16_t,
                             ooopresp: 0i32 as uint16_t,
                             badcode: 0i32 as uint16_t,};
        init
    };
#[no_mangle]
pub static mut saPowerTable: [saPowerTable_t; 4] =
    [{
         let mut init =
             saPowerTable_s{rfpower: 25i32,
                            valueV1: 7i32 as int16_t,
                            valueV2: 0i32 as int16_t,};
         init
     },
     {
         let mut init =
             saPowerTable_s{rfpower: 200i32,
                            valueV1: 16i32 as int16_t,
                            valueV2: 1i32 as int16_t,};
         init
     },
     {
         let mut init =
             saPowerTable_s{rfpower: 500i32,
                            valueV1: 25i32 as int16_t,
                            valueV2: 2i32 as int16_t,};
         init
     },
     {
         let mut init =
             saPowerTable_s{rfpower: 800i32,
                            valueV1: 40i32 as int16_t,
                            valueV2: 3i32 as int16_t,};
         init
     }];
// Last received device ('hard') states
#[no_mangle]
pub static mut saDevice: smartAudioDevice_t =
    {
        let mut init =
            smartAudioDevice_s{version: 0i32 as int8_t,
                               channel: -1i32 as int8_t,
                               power: -1i32 as int8_t,
                               mode: 0i32 as int8_t,
                               freq: 0i32 as uint16_t,
                               orfreq: 0i32 as uint16_t,};
        init
    };
static mut saDevicePrev: smartAudioDevice_t =
    {
        let mut init =
            smartAudioDevice_s{version: 0i32 as int8_t,
                               channel: 0,
                               power: 0,
                               mode: 0,
                               freq: 0,
                               orfreq: 0,};
        init
    };
// XXX Possible compliance problem here. Need LOCK/UNLOCK menu?
static mut saLockMode: uint8_t = 8i32 as uint8_t;
// saCms variable?
// XXX Should be configurable by user?
#[no_mangle]
pub static mut saDeferred: bool = 1i32 != 0;
static mut sa_rbuf: [uint8_t; 15] = [0; 15];
unsafe extern "C" fn CRC8(mut data: *const uint8_t, len: int8_t) -> uint8_t {
    let mut crc: uint8_t =
        0i32 as uint8_t; /* start with 0 so first byte can be 'xored' in */
    let mut currByte: uint8_t = 0; /* XOR-in the next input byte */
    let mut i: libc::c_int = 0i32;
    while i < len as libc::c_int {
        currByte = *data.offset(i as isize);
        crc = (crc as libc::c_int ^ currByte as libc::c_int) as uint8_t;
        let mut i_0: libc::c_int = 0i32;
        while i_0 < 8i32 {
            if crc as libc::c_int & 0x80i32 != 0i32 {
                crc = ((crc as libc::c_int) << 1i32 ^ 0xd5i32) as uint8_t
            } else { crc = ((crc as libc::c_int) << 1i32) as uint8_t }
            i_0 += 1
        }
        i += 1
    }
    return crc;
}
#[no_mangle]
pub unsafe extern "C" fn saDacToPowerIndex(mut dac: libc::c_int)
 -> libc::c_int {
    let mut idx: libc::c_int = 3i32;
    while idx >= 0i32 {
        if saPowerTable[idx as usize].valueV1 as libc::c_int <= dac {
            return idx
        }
        idx -= 1
    }
    return 0i32;
}
#[no_mangle]
pub static mut sa_smartbaud: uint16_t = 4800i32 as uint16_t;
static mut sa_adjdir: libc::c_int = 1i32;
// -1=going down, 1=going up
static mut sa_baudstep: libc::c_int = 50i32;
unsafe extern "C" fn saAutobaud() {
    if (saStat.pktsent as libc::c_int) < 10i32 {
        // Not enough samples collected
        return
    }
    if saStat.pktrcvd as libc::c_int * 100i32 / saStat.pktsent as libc::c_int
           >= 70i32 {
        // This is okay
        saStat.pktsent = 0i32 as uint16_t; // Should be more moderate?
        saStat.pktrcvd = 0i32 as uint16_t;
        return
    }
    if sa_adjdir == 1i32 && sa_smartbaud as libc::c_int == 4950i32 {
        sa_adjdir = -1i32
    } else if sa_adjdir == -1i32 && sa_smartbaud as libc::c_int == 4800i32 {
        sa_adjdir = 1i32
    }
    sa_smartbaud =
        (sa_smartbaud as libc::c_int + sa_baudstep * sa_adjdir) as uint16_t;
    (*(*smartAudioSerialPort).vTable).serialSetBaudRate.expect("non-null function pointer")(smartAudioSerialPort,
                                                                                            sa_smartbaud
                                                                                                as
                                                                                                uint32_t);
    saStat.pktsent = 0i32 as uint16_t;
    saStat.pktrcvd = 0i32 as uint16_t;
}
// Transport level variables
static mut sa_lastTransmissionMs: timeUs_t = 0i32 as timeUs_t;
static mut sa_outstanding: uint8_t = SA_CMD_NONE as libc::c_int as uint8_t;
// Outstanding command
static mut sa_osbuf: [uint8_t; 32] = [0; 32];
// Outstanding comamnd frame for retransmission
static mut sa_oslen: libc::c_int = 0;
// And associate length
unsafe extern "C" fn saProcessResponse(mut buf: *mut uint8_t,
                                       mut len: libc::c_int) {
    let mut resp: uint8_t = *buf.offset(0);
    if resp as libc::c_int == sa_outstanding as libc::c_int {
        sa_outstanding = SA_CMD_NONE as libc::c_int as uint8_t
    } else if resp as libc::c_int == SA_CMD_GET_SETTINGS_V2 as libc::c_int &&
                  sa_outstanding as libc::c_int ==
                      SA_CMD_GET_SETTINGS as libc::c_int {
        sa_outstanding = SA_CMD_NONE as libc::c_int as uint8_t
    } else { saStat.ooopresp = saStat.ooopresp.wrapping_add(1) }
    let mut freq: uint16_t = 0;
    let mut current_block_47: u64;
    match resp as libc::c_int {
        9 => {
            // Version 2 Get Settings
            current_block_47 = 17189961260979847415;
        }
        1 => { current_block_47 = 17189961260979847415; }
        2 => {
            // Set Power
            current_block_47 = 13321564401369230990;
        }
        3 => {
            // Set Channel
            current_block_47 = 13321564401369230990;
        }
        4 => {
            // Set Frequency
            if len < 5i32 {
                current_block_47 = 13321564401369230990;
            } else {
                freq =
                    ((*buf.offset(2) as libc::c_int) << 8i32 |
                         *buf.offset(3) as libc::c_int) as uint16_t;
                if freq as libc::c_int & 1i32 << 14i32 != 0 {
                    saDevice.orfreq =
                        (freq as libc::c_int &
                             !(1i32 << 14i32 | 1i32 << 15i32)) as uint16_t
                } else if freq as libc::c_int & 1i32 << 15i32 != 0 {
                    saDevice.orfreq =
                        (freq as libc::c_int &
                             !(1i32 << 14i32 | 1i32 << 15i32)) as uint16_t
                } else { saDevice.freq = freq }
                current_block_47 = 13321564401369230990;
            }
        }
        5 => { current_block_47 = 13321564401369230990; }
        _ => { saStat.badcode = saStat.badcode.wrapping_add(1); return }
    }
    match current_block_47 {
        17189961260979847415 =>
        // Version 1 Get Settings
        {
            if !(len < 7i32) {
                saDevice.version =
                    if *buf.offset(0) as libc::c_int ==
                           SA_CMD_GET_SETTINGS as libc::c_int {
                        1i32
                    } else { 2i32 } as int8_t;
                saDevice.channel = *buf.offset(2) as int8_t;
                saDevice.power = *buf.offset(3) as int8_t;
                saDevice.mode = *buf.offset(4) as int8_t;
                saDevice.freq =
                    ((*buf.offset(5) as libc::c_int) << 8i32 |
                         *buf.offset(6) as libc::c_int) as uint16_t;
                if debugMode as libc::c_int == DEBUG_SMARTAUDIO as libc::c_int
                   {
                    debug[0] =
                        (saDevice.version as libc::c_int * 100i32 +
                             saDevice.mode as libc::c_int) as int16_t
                }
                if debugMode as libc::c_int == DEBUG_SMARTAUDIO as libc::c_int
                   {
                    debug[1] = saDevice.channel as int16_t
                }
                if debugMode as libc::c_int == DEBUG_SMARTAUDIO as libc::c_int
                   {
                    debug[2] = saDevice.freq as int16_t
                }
                if debugMode as libc::c_int == DEBUG_SMARTAUDIO as libc::c_int
                   {
                    debug[3] = saDevice.power as int16_t
                }
            }
        }
        _ => { }
    }
    (memcmp(&mut saDevice as *mut smartAudioDevice_t as *const libc::c_void,
            &mut saDevicePrev as *mut smartAudioDevice_t as
                *const libc::c_void,
            ::core::mem::size_of::<smartAudioDevice_t>() as libc::c_ulong)) !=
        0;
    saDevicePrev = saDevice;
    // Todo: Update states in saVtxDevice?
}
//
// Datalink
//
unsafe extern "C" fn saReceiveFramer(mut c: uint8_t) {
    static mut state: saFramerState_e = S_WAITPRE1;
    static mut len: libc::c_int = 0;
    static mut dlen: libc::c_int = 0;
    match state as libc::c_uint {
        0 => {
            if c as libc::c_int == 0xaai32 {
                state = S_WAITPRE2
            } else {
                state = S_WAITPRE1
                // Don't need this (no change)
            }
        }
        1 => {
            if c as libc::c_int == 0x55i32 {
                state = S_WAITRESP
            } else {
                saStat.badpre = saStat.badpre.wrapping_add(1);
                state = S_WAITPRE1
            }
        }
        2 => { sa_rbuf[0] = c; state = S_WAITLEN }
        3 => {
            sa_rbuf[1] = c;
            len = c as libc::c_int;
            if len > 11i32 - 2i32 {
                saStat.badlen = saStat.badlen.wrapping_add(1);
                state = S_WAITPRE1
            } else if len == 0i32 {
                state = S_WAITCRC
            } else { dlen = 0i32; state = S_DATA }
        }
        4 => {
            // XXX Should check buffer overflow (-> saerr_overflow)
            sa_rbuf[(2i32 + dlen) as usize] = c;
            dlen += 1;
            if dlen == len { state = S_WAITCRC }
        }
        5 => {
            if CRC8(sa_rbuf.as_mut_ptr(), (2i32 + len) as int8_t) as
                   libc::c_int == c as libc::c_int {
                // Got a response
                saProcessResponse(sa_rbuf.as_mut_ptr(),
                                  len + 2i32); // Generate 1st start bit
                saStat.pktrcvd = saStat.pktrcvd.wrapping_add(1)
            } else if !(sa_rbuf[0] as libc::c_int & 1i32 != 0) {
                saStat.crc = saStat.crc.wrapping_add(1)
            }
            state = S_WAITPRE1
        }
        _ => { }
    };
}
unsafe extern "C" fn saSendFrame(mut buf: *mut uint8_t,
                                 mut len: libc::c_int) {
    match (*smartAudioSerialPort).identifier as libc::c_int {
        30 | 31 => { }
        _ => { serialWrite(smartAudioSerialPort, 0i32 as uint8_t); }
    }
    let mut i: libc::c_int = 0i32;
    while i < len {
        serialWrite(smartAudioSerialPort, *buf.offset(i as isize));
        i += 1
    }
    sa_lastTransmissionMs = millis();
    saStat.pktsent = saStat.pktsent.wrapping_add(1);
}
/*
 * Retransmission and command queuing
 *
 *   The transport level support includes retransmission on response timeout
 * and command queueing.
 *
 * Resend buffer:
 *   The smartaudio returns response for valid command frames in no less
 * than 60msec, which we can't wait. So there's a need for a resend buffer.
 *
 * Command queueing:
 *   The driver autonomously sends GetSettings command for auto-bauding,
 * asynchronous to user initiated commands; commands issued while another
 * command is outstanding must be queued for later processing.
 *   The queueing also handles the case in which multiple commands are
 * required to implement a user level command.
 */
// Retransmission
unsafe extern "C" fn saResendCmd() {
    saSendFrame(sa_osbuf.as_mut_ptr(), sa_oslen);
}
unsafe extern "C" fn saSendCmd(mut buf: *mut uint8_t, mut len: libc::c_int) {
    let mut i: libc::c_int = 0i32;
    while i < len { sa_osbuf[i as usize] = *buf.offset(i as isize); i += 1 }
    sa_oslen = len;
    sa_outstanding = (*buf.offset(2) as libc::c_int >> 1i32) as uint8_t;
    saSendFrame(sa_osbuf.as_mut_ptr(), sa_oslen);
}
// 1 heartbeat (GetSettings) + 2 commands + 1 slack
static mut sa_queue: [saCmdQueue_t; 6] =
    [saCmdQueue_t{buf: 0 as *const uint8_t as *mut uint8_t, len: 0,}; 6];
static mut sa_qhead: uint8_t = 0i32 as uint8_t;
static mut sa_qtail: uint8_t = 0i32 as uint8_t;
unsafe extern "C" fn saQueueEmpty() -> bool {
    return sa_qhead as libc::c_int == sa_qtail as libc::c_int;
}
unsafe extern "C" fn saQueueFull() -> bool {
    return (sa_qhead as libc::c_int + 1i32) % 6i32 == sa_qtail as libc::c_int;
}
unsafe extern "C" fn saQueueCmd(mut buf: *mut uint8_t, mut len: libc::c_int) {
    if saQueueFull() { return }
    sa_queue[sa_qhead as usize].buf = buf;
    sa_queue[sa_qhead as usize].len = len;
    sa_qhead = ((sa_qhead as libc::c_int + 1i32) % 6i32) as uint8_t;
}
unsafe extern "C" fn saSendQueue() {
    if saQueueEmpty() { return }
    saSendCmd(sa_queue[sa_qtail as usize].buf,
              sa_queue[sa_qtail as usize].len);
    sa_qtail = ((sa_qtail as libc::c_int + 1i32) % 6i32) as uint8_t;
}
// Individual commands
unsafe extern "C" fn saGetSettings() {
    static mut bufGetSettings: [uint8_t; 5] =
        [0xaai32 as uint8_t, 0x55i32 as uint8_t,
         ((SA_CMD_GET_SETTINGS as libc::c_int) << 1i32 | 1i32) as uint8_t,
         0i32 as uint8_t, 0x9fi32 as uint8_t];
    saQueueCmd(bufGetSettings.as_mut_ptr(), 5i32);
}
unsafe extern "C" fn saValidateFreq(mut freq: uint16_t) -> bool {
    return freq as libc::c_int >= 5000i32 && freq as libc::c_int <= 5999i32;
}
unsafe extern "C" fn saDoDevSetFreq(mut freq: uint16_t) {
    static mut buf: [uint8_t; 7] =
        [0xaai32 as uint8_t, 0x55i32 as uint8_t,
         ((SA_CMD_SET_FREQ as libc::c_int) << 1i32 | 1i32) as uint8_t,
         2i32 as uint8_t, 0, 0, 0];
    static mut switchBuf: [uint8_t; 7] = [0; 7];
    if !(freq as libc::c_int & 1i32 << 14i32 != 0) {
        (freq as libc::c_int & 1i32 << 15i32) != 0;
    }
    buf[4] = (freq as libc::c_int >> 8i32 & 0xffi32) as uint8_t;
    buf[5] = (freq as libc::c_int & 0xffi32) as uint8_t;
    buf[6] = CRC8(buf.as_mut_ptr(), 6i32 as int8_t);
    // Need to work around apparent SmartAudio bug when going from 'channel'
    // to 'user-freq' mode, where the set-freq command will fail if the freq
    // value is unchanged from the previous 'user-freq' mode
    if saDevice.mode as libc::c_int & 1i32 == 0i32 &&
           freq as libc::c_int == saDevice.freq as libc::c_int {
        memcpy(&mut switchBuf as *mut [uint8_t; 7] as *mut libc::c_void,
               &mut buf as *mut [uint8_t; 7] as *const libc::c_void,
               ::core::mem::size_of::<[uint8_t; 7]>() as libc::c_ulong);
        let switchFreq: uint16_t =
            (freq as libc::c_int +
                 (if freq as libc::c_int == 5999i32 { -1i32 } else { 1i32 }))
                as uint16_t;
        switchBuf[4] = (switchFreq as libc::c_int >> 8i32) as uint8_t;
        switchBuf[5] = (switchFreq as libc::c_int & 0xffi32) as uint8_t;
        switchBuf[6] = CRC8(switchBuf.as_mut_ptr(), 6i32 as int8_t);
        saQueueCmd(switchBuf.as_mut_ptr(), 7i32);
    }
    saQueueCmd(buf.as_mut_ptr(), 7i32);
}
#[no_mangle]
pub unsafe extern "C" fn saSetFreq(mut freq: uint16_t) {
    saDoDevSetFreq(freq);
}
#[no_mangle]
pub unsafe extern "C" fn saSetPitFreq(mut freq: uint16_t) {
    saDoDevSetFreq((freq as libc::c_int | 1i32 << 15i32) as uint16_t);
}
unsafe extern "C" fn saValidateBandAndChannel(mut band: uint8_t,
                                              mut channel: uint8_t) -> bool {
    return band as libc::c_int >= 1i32 && band as libc::c_int <= 5i32 &&
               channel as libc::c_int >= 1i32 &&
               channel as libc::c_int <= 8i32;
}
unsafe extern "C" fn saDevSetBandAndChannel(mut band: uint8_t,
                                            mut channel: uint8_t) {
    static mut buf: [uint8_t; 6] =
        [0xaai32 as uint8_t, 0x55i32 as uint8_t,
         ((SA_CMD_SET_CHAN as libc::c_int) << 1i32 | 1i32) as uint8_t,
         1i32 as uint8_t, 0, 0];
    buf[4] =
        (band as libc::c_int * 8i32 as uint8_t as libc::c_int +
             channel as libc::c_int) as uint8_t;
    buf[5] = CRC8(buf.as_mut_ptr(), 5i32 as int8_t);
    saQueueCmd(buf.as_mut_ptr(), 6i32);
}
#[no_mangle]
pub unsafe extern "C" fn saSetBandAndChannel(mut band: uint8_t,
                                             mut channel: uint8_t) {
    saDevSetBandAndChannel(band, channel);
}
#[no_mangle]
pub unsafe extern "C" fn saSetMode(mut mode: libc::c_int) {
    static mut buf: [uint8_t; 6] =
        [0xaai32 as uint8_t, 0x55i32 as uint8_t,
         ((SA_CMD_SET_MODE as libc::c_int) << 1i32 | 1i32) as uint8_t,
         1i32 as uint8_t, 0, 0];
    buf[4] = (mode & 0x3fi32 | saLockMode as libc::c_int) as uint8_t;
    buf[5] = CRC8(buf.as_mut_ptr(), 5i32 as int8_t);
    saQueueCmd(buf.as_mut_ptr(), 6i32);
}
unsafe extern "C" fn saDevSetPowerByIndex(mut index: uint8_t) {
    static mut buf: [uint8_t; 6] =
        [0xaai32 as uint8_t, 0x55i32 as uint8_t,
         ((SA_CMD_SET_POWER as libc::c_int) << 1i32 | 1i32) as uint8_t,
         1i32 as uint8_t, 0, 0];
    if saDevice.version as libc::c_int == 0i32 {
        // Unknown or yet unknown version.
        return
    }
    if index as libc::c_int >= 4i32 { return }
    buf[4] =
        if saDevice.version as libc::c_int == 1i32 {
            saPowerTable[index as usize].valueV1 as libc::c_int
        } else { saPowerTable[index as usize].valueV2 as libc::c_int } as
            uint8_t;
    buf[5] = CRC8(buf.as_mut_ptr(), 5i32 as int8_t);
    saQueueCmd(buf.as_mut_ptr(), 6i32);
}
#[no_mangle]
pub unsafe extern "C" fn saSetPowerByIndex(mut index: uint8_t) {
    saDevSetPowerByIndex(index);
}
#[no_mangle]
pub unsafe extern "C" fn vtxSmartAudioInit() -> bool {
    let mut portConfig: *mut serialPortConfig_t =
        findSerialPortConfig(FUNCTION_VTX_SMARTAUDIO);
    if !portConfig.is_null() {
        let mut portOptions: portOptions_e =
            (SERIAL_STOPBITS_2 as libc::c_int |
                 SERIAL_BIDIR_NOPULL as libc::c_int) as portOptions_e;
        portOptions =
            (portOptions as libc::c_uint |
                 (if (*vtxConfig()).halfDuplex as libc::c_int != 0 {
                      (SERIAL_BIDIR as libc::c_int) |
                          SERIAL_BIDIR_PP as libc::c_int
                  } else { SERIAL_UNIDIR as libc::c_int }) as libc::c_uint) as
                portOptions_e;
        smartAudioSerialPort =
            openSerialPort((*portConfig).identifier, FUNCTION_VTX_SMARTAUDIO,
                           None, 0 as *mut libc::c_void, 4800i32 as uint32_t,
                           MODE_RXTX, portOptions)
    }
    if smartAudioSerialPort.is_null() { return 0i32 != 0 }
    vtxCommonSetDevice(&mut vtxSmartAudio);
    return 1i32 != 0;
}
unsafe extern "C" fn vtxSAProcess(mut vtxDevice: *mut vtxDevice_t,
                                  mut currentTimeUs: timeUs_t) {
    static mut initPhase: libc::c_char = 0i32 as libc::c_char;
    if smartAudioSerialPort.is_null() { return }
    while serialRxBytesWaiting(smartAudioSerialPort) > 0i32 as libc::c_uint {
        let mut c: uint8_t = serialRead(smartAudioSerialPort);
        saReceiveFramer(c as uint16_t as uint8_t);
    }
    // Re-evaluate baudrate after each frame reception
    saAutobaud();
    match initPhase as libc::c_int {
        0 => {
            saGetSettings();
            //saSendQueue();
            initPhase = 1i32 as libc::c_char
        }
        1 => {
            // Don't send SA_FREQ_GETPIT to V1 device; it act as plain SA_CMD_SET_FREQ,
        // and put the device into user frequency mode with uninitialized freq.
            if saDevice.version != 0 {
                if saDevice.version as libc::c_int == 2i32 {
                    saDoDevSetFreq((1i32 << 14i32) as uint16_t);
                    initPhase = 2i32 as libc::c_char
                } else { initPhase = 3i32 as libc::c_char }
            }
        }
        2 => { if saDevice.orfreq != 0 { initPhase = 3i32 as libc::c_char } }
        3 | _ => { }
    }
    // Command queue control
    let mut nowMs: timeMs_t =
        millis(); // Don't substitute with "currentTimeUs / 1000"; sa_lastTransmissionMs is based on millis().
    static mut lastCommandSentMs: timeMs_t =
        0i32 as timeMs_t; // Last non-GET_SETTINGS sent
    if sa_outstanding as libc::c_int != SA_CMD_NONE as libc::c_int &&
           nowMs.wrapping_sub(sa_lastTransmissionMs) > 120i32 as libc::c_uint
       {
        // Last command timed out
        // dprintf(("process: resending 0x%x\r\n", sa_outstanding));
        // XXX Todo: Resend termination and possible offline transition
        saResendCmd();
        lastCommandSentMs = nowMs
    } else if !saQueueEmpty() {
        // Command pending. Send it.
        // dprintf(("process: sending queue\r\n"));
        saSendQueue();
        lastCommandSentMs = nowMs
    } else if nowMs.wrapping_sub(lastCommandSentMs) < 1000i32 as libc::c_uint
                  &&
                  nowMs.wrapping_sub(sa_lastTransmissionMs) >=
                      150i32 as libc::c_uint {
        //dprintf(("process: sending status change polling\r\n"));
        saGetSettings();
        saSendQueue();
    };
}
// Interface to common VTX API
#[no_mangle]
pub unsafe extern "C" fn vtxSAGetDeviceType(mut vtxDevice: *const vtxDevice_t)
 -> vtxDevType_e {
    return VTXDEV_SMARTAUDIO;
}
unsafe extern "C" fn vtxSAIsReady(mut vtxDevice: *const vtxDevice_t) -> bool {
    return !vtxDevice.is_null() && !(saDevice.version as libc::c_int == 0i32);
}
unsafe extern "C" fn vtxSASetBandAndChannel(mut vtxDevice: *mut vtxDevice_t,
                                            mut band: uint8_t,
                                            mut channel: uint8_t) {
    if saValidateBandAndChannel(band, channel) {
        saSetBandAndChannel((band as libc::c_int - 1i32) as uint8_t,
                            (channel as libc::c_int - 1i32) as uint8_t);
    };
}
unsafe extern "C" fn vtxSASetPowerByIndex(mut vtxDevice: *mut vtxDevice_t,
                                          mut index: uint8_t) {
    if index as libc::c_int == 0i32 {
        // SmartAudio doesn't support power off.
        return
    }
    saSetPowerByIndex((index as libc::c_int - 1i32) as uint8_t);
}
unsafe extern "C" fn vtxSASetPitMode(mut vtxDevice: *mut vtxDevice_t,
                                     mut onoff: uint8_t) {
    if !(vtxSAIsReady(vtxDevice) as libc::c_int != 0 &&
             saDevice.version as libc::c_int == 2i32) {
        return
    }
    if onoff != 0 {
        // SmartAudio can not turn pit mode on by software.
        return
    } //need to be in FREE mode to set freq
    let mut newmode: uint8_t = 4i32 as uint8_t;
    if saDevice.mode as libc::c_int & 4i32 != 0 {
        newmode = (newmode as libc::c_int | 1i32) as uint8_t
    }
    if saDevice.mode as libc::c_int & 8i32 != 0 {
        newmode = (newmode as libc::c_int | 2i32) as uint8_t
    }
    saSetMode(newmode as libc::c_int);
}
unsafe extern "C" fn vtxSASetFreq(mut vtxDevice: *mut vtxDevice_t,
                                  mut freq: uint16_t) {
    if saValidateFreq(freq) { saSetMode(0i32); saSetFreq(freq); };
}
unsafe extern "C" fn vtxSAGetBandAndChannel(mut vtxDevice: *const vtxDevice_t,
                                            mut pBand: *mut uint8_t,
                                            mut pChannel: *mut uint8_t)
 -> bool {
    if !vtxSAIsReady(vtxDevice) { return 0i32 != 0 }
    // if in user-freq mode then report band as zero
    *pBand =
        if saDevice.mode as libc::c_int & 1i32 != 0 {
            0i32
        } else {
            (saDevice.channel as libc::c_int / 8i32 as uint8_t as libc::c_int)
                + 1i32
        } as uint8_t;
    *pChannel =
        (saDevice.channel as libc::c_int % 8i32 as uint8_t as libc::c_int +
             1i32) as uint8_t;
    return 1i32 != 0;
}
unsafe extern "C" fn vtxSAGetPowerIndex(mut vtxDevice: *const vtxDevice_t,
                                        mut pIndex: *mut uint8_t) -> bool {
    if !vtxSAIsReady(vtxDevice) { return 0i32 != 0 }
    *pIndex =
        ((if saDevice.version as libc::c_int == 1i32 {
              saDacToPowerIndex(saDevice.power as libc::c_int)
          } else { saDevice.power as libc::c_int }) + 1i32) as uint8_t;
    return 1i32 != 0;
}
unsafe extern "C" fn vtxSAGetPitMode(mut vtxDevice: *const vtxDevice_t,
                                     mut pOnOff: *mut uint8_t) -> bool {
    if !(vtxSAIsReady(vtxDevice) as libc::c_int != 0 &&
             saDevice.version as libc::c_int == 2i32) {
        return 0i32 != 0
    }
    *pOnOff =
        if saDevice.mode as libc::c_int & 2i32 != 0 { 1i32 } else { 0i32 } as
            uint8_t;
    return 1i32 != 0;
}
unsafe extern "C" fn vtxSAGetFreq(mut vtxDevice: *const vtxDevice_t,
                                  mut pFreq: *mut uint16_t) -> bool {
    if !vtxSAIsReady(vtxDevice) { return 0i32 != 0 }
    // if not in user-freq mode then convert band/chan to frequency
    *pFreq =
        if saDevice.mode as libc::c_int & 1i32 != 0 {
            saDevice.freq as libc::c_int
        } else {
            vtx58_Bandchan2Freq((saDevice.channel as libc::c_int /
                                     8i32 as uint8_t as libc::c_int + 1i32) as
                                    uint8_t,
                                (saDevice.channel as libc::c_int %
                                     8i32 as uint8_t as libc::c_int + 1i32) as
                                    uint8_t) as libc::c_int
        } as uint16_t;
    return 1i32 != 0;
}
static mut saVTable: vtxVTable_t =
    unsafe {
        {
            let mut init =
                vtxVTable_s{process:
                                Some(vtxSAProcess as
                                         unsafe extern "C" fn(_:
                                                                  *mut vtxDevice_t,
                                                              _: timeUs_t)
                                             -> ()),
                            getDeviceType:
                                Some(vtxSAGetDeviceType as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t)
                                             -> vtxDevType_e),
                            isReady:
                                Some(vtxSAIsReady as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t)
                                             -> bool),
                            setBandAndChannel:
                                Some(vtxSASetBandAndChannel as
                                         unsafe extern "C" fn(_:
                                                                  *mut vtxDevice_t,
                                                              _: uint8_t,
                                                              _: uint8_t)
                                             -> ()),
                            setPowerByIndex:
                                Some(vtxSASetPowerByIndex as
                                         unsafe extern "C" fn(_:
                                                                  *mut vtxDevice_t,
                                                              _: uint8_t)
                                             -> ()),
                            setPitMode:
                                Some(vtxSASetPitMode as
                                         unsafe extern "C" fn(_:
                                                                  *mut vtxDevice_t,
                                                              _: uint8_t)
                                             -> ()),
                            setFrequency:
                                Some(vtxSASetFreq as
                                         unsafe extern "C" fn(_:
                                                                  *mut vtxDevice_t,
                                                              _: uint16_t)
                                             -> ()),
                            getBandAndChannel:
                                Some(vtxSAGetBandAndChannel as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t,
                                                              _: *mut uint8_t,
                                                              _: *mut uint8_t)
                                             -> bool),
                            getPowerIndex:
                                Some(vtxSAGetPowerIndex as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t,
                                                              _: *mut uint8_t)
                                             -> bool),
                            getPitMode:
                                Some(vtxSAGetPitMode as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t,
                                                              _: *mut uint8_t)
                                             -> bool),
                            getFrequency:
                                Some(vtxSAGetFreq as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t,
                                                              _:
                                                                  *mut uint16_t)
                                             -> bool),};
            init
        }
    };
// VTX_SMARTAUDIO
// VTX_COMMON
