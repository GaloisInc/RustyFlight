use ::libc;
use ::c2rust_bitfields;
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
    #[no_mangle]
    static mut debug: [int16_t; 4];
    #[no_mangle]
    static mut debugMode: uint8_t;
    #[no_mangle]
    fn micros() -> timeUs_t;
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
    static mut telemetrySharedPort: *mut serialPort_t;
    #[no_mangle]
    fn telemetryCheckRxPortShared(portConfig: *const serialPortConfig_t)
     -> bool;
    #[no_mangle]
    static mut rssiSource: rssiSource_e;
    #[no_mangle]
    fn sbusChannelsDecode(rxRuntimeConfig: *mut rxRuntimeConfig_t,
                          channels: *const sbusChannels_t) -> uint8_t;
    #[no_mangle]
    fn sbusChannelsInit(rxConfig: *const rxConfig_t,
                        rxRuntimeConfig: *mut rxRuntimeConfig_t);
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
// microsecond time
pub type timeUs_t = uint32_t;
pub type ioTag_t = uint8_t;
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
pub type C2RustUnnamed_0 = libc::c_uint;
pub const RX_FRAME_DROPPED: C2RustUnnamed_0 = 8;
pub const RX_FRAME_PROCESSING_REQUIRED: C2RustUnnamed_0 = 4;
pub const RX_FRAME_FAILSAFE: C2RustUnnamed_0 = 2;
pub const RX_FRAME_COMPLETE: C2RustUnnamed_0 = 1;
pub const RX_FRAME_PENDING: C2RustUnnamed_0 = 0;
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
// used by receiver driver to return channel data
pub type rcFrameStatusFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut rxRuntimeConfig_s) -> uint8_t>;
pub type rcReadRawDataFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s, _: uint8_t)
               -> uint16_t>;
pub type rxRuntimeConfig_t = rxRuntimeConfig_s;
pub type rssiSource_e = libc::c_uint;
pub const RSSI_SOURCE_FRAME_ERRORS: rssiSource_e = 5;
pub const RSSI_SOURCE_MSP: rssiSource_e = 4;
pub const RSSI_SOURCE_RX_PROTOCOL: rssiSource_e = 3;
pub const RSSI_SOURCE_RX_CHANNEL: rssiSource_e = 2;
pub const RSSI_SOURCE_ADC: rssiSource_e = 1;
pub const RSSI_SOURCE_NONE: rssiSource_e = 0;
pub type sbusFrameData_t = sbusFrameData_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sbusFrameData_s {
    pub frame: sbusFrame_t,
    pub startAtUs: uint32_t,
    pub stateFlags: uint16_t,
    pub position: uint8_t,
    pub done: bool,
}
pub type sbusFrame_t = sbusFrame_u;
#[derive(Copy, Clone)]
#[repr(C)]
pub union sbusFrame_u {
    pub bytes: [uint8_t; 25],
    pub frame: sbusFrame_s,
}
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct sbusFrame_s {
    pub syncByte: uint8_t,
    pub channels: sbusChannels_t,
    pub endByte: uint8_t,
}
pub type sbusChannels_t = sbusChannels_s;
#[derive(Copy, Clone, BitfieldStruct)]
#[repr(C, packed)]
pub struct sbusChannels_s {
    #[bitfield(name = "chan0", ty = "libc::c_uint", bits = "0..=10")]
    #[bitfield(name = "chan1", ty = "libc::c_uint", bits = "11..=21")]
    #[bitfield(name = "chan2", ty = "libc::c_uint", bits = "22..=32")]
    #[bitfield(name = "chan3", ty = "libc::c_uint", bits = "33..=43")]
    #[bitfield(name = "chan4", ty = "libc::c_uint", bits = "44..=54")]
    #[bitfield(name = "chan5", ty = "libc::c_uint", bits = "55..=65")]
    #[bitfield(name = "chan6", ty = "libc::c_uint", bits = "66..=76")]
    #[bitfield(name = "chan7", ty = "libc::c_uint", bits = "77..=87")]
    #[bitfield(name = "chan8", ty = "libc::c_uint", bits = "88..=98")]
    #[bitfield(name = "chan9", ty = "libc::c_uint", bits = "99..=109")]
    #[bitfield(name = "chan10", ty = "libc::c_uint", bits = "110..=120")]
    #[bitfield(name = "chan11", ty = "libc::c_uint", bits = "121..=131")]
    #[bitfield(name = "chan12", ty = "libc::c_uint", bits = "132..=142")]
    #[bitfield(name = "chan13", ty = "libc::c_uint", bits = "143..=153")]
    #[bitfield(name = "chan14", ty = "libc::c_uint", bits = "154..=164")]
    #[bitfield(name = "chan15", ty = "libc::c_uint", bits = "165..=175")]
    pub chan0_chan1_chan2_chan3_chan4_chan5_chan6_chan7_chan8_chan9_chan10_chan11_chan12_chan13_chan14_chan15: [u8; 22],
    pub flags: uint8_t,
}
pub const DEBUG_SBUS_FRAME_TIME: C2RustUnnamed_1 = 2;
pub const DEBUG_SBUS_STATE_FLAGS: C2RustUnnamed_1 = 1;
pub const DEBUG_SBUS_FRAME_FLAGS: C2RustUnnamed_1 = 0;
pub type C2RustUnnamed_1 = libc::c_uint;
// number of RC channels as reported by current input driver
// 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
// Receive ISR callback
unsafe extern "C" fn sbusDataReceive(mut c: uint16_t,
                                     mut data: *mut libc::c_void) {
    let mut sbusFrameData: *mut sbusFrameData_t =
        data as *mut sbusFrameData_t;
    let nowUs: uint32_t = micros();
    let sbusFrameTime: int32_t =
        nowUs.wrapping_sub((*sbusFrameData).startAtUs) as int32_t;
    if sbusFrameTime as libc::c_long >
           (3000 as libc::c_int + 500 as libc::c_int) as libc::c_long {
        (*sbusFrameData).position = 0 as libc::c_int as uint8_t
    }
    if (*sbusFrameData).position as libc::c_int == 0 as libc::c_int {
        if c as libc::c_int != 0xf as libc::c_int { return }
        (*sbusFrameData).startAtUs = nowUs
    }
    if ((*sbusFrameData).position as libc::c_ulong) <
           (::core::mem::size_of::<sbusChannels_t>() as
                libc::c_ulong).wrapping_add(2 as libc::c_int as libc::c_ulong)
       {
        let fresh0 = (*sbusFrameData).position;
        (*sbusFrameData).position = (*sbusFrameData).position.wrapping_add(1);
        (*sbusFrameData).frame.bytes[fresh0 as usize] = c as uint8_t;
        if ((*sbusFrameData).position as libc::c_ulong) <
               (::core::mem::size_of::<sbusChannels_t>() as
                    libc::c_ulong).wrapping_add(2 as libc::c_int as
                                                    libc::c_ulong) {
            (*sbusFrameData).done = 0 as libc::c_int != 0
        } else {
            (*sbusFrameData).done = 1 as libc::c_int != 0;
            if debugMode as libc::c_int == DEBUG_SBUS as libc::c_int {
                debug[DEBUG_SBUS_FRAME_TIME as libc::c_int as usize] =
                    sbusFrameTime as int16_t
            }
        }
    };
}
unsafe extern "C" fn sbusFrameStatus(mut rxRuntimeConfig:
                                         *mut rxRuntimeConfig_t) -> uint8_t {
    let mut sbusFrameData: *mut sbusFrameData_t =
        (*rxRuntimeConfig).frameData as *mut sbusFrameData_t;
    if !(*sbusFrameData).done {
        return RX_FRAME_PENDING as libc::c_int as uint8_t
    }
    (*sbusFrameData).done = 0 as libc::c_int != 0;
    if debugMode as libc::c_int == DEBUG_SBUS as libc::c_int {
        debug[DEBUG_SBUS_FRAME_FLAGS as libc::c_int as usize] =
            (*sbusFrameData).frame.frame.channels.flags as int16_t
    }
    if (*sbusFrameData).frame.frame.channels.flags as libc::c_int &
           (1 as libc::c_int) << 2 as libc::c_int != 0 {
        (*sbusFrameData).stateFlags =
            ((*sbusFrameData).stateFlags as libc::c_int |
                 (1 as libc::c_int) << 1 as libc::c_int) as uint16_t;
        if debugMode as libc::c_int == DEBUG_SBUS as libc::c_int {
            debug[DEBUG_SBUS_STATE_FLAGS as libc::c_int as usize] =
                (*sbusFrameData).stateFlags as int16_t
        }
    }
    if (*sbusFrameData).frame.frame.channels.flags as libc::c_int &
           (1 as libc::c_int) << 3 as libc::c_int != 0 {
        (*sbusFrameData).stateFlags =
            ((*sbusFrameData).stateFlags as libc::c_int |
                 (1 as libc::c_int) << 0 as libc::c_int) as uint16_t;
        if debugMode as libc::c_int == DEBUG_SBUS as libc::c_int {
            debug[DEBUG_SBUS_STATE_FLAGS as libc::c_int as usize] =
                (*sbusFrameData).stateFlags as int16_t
        }
    }
    if debugMode as libc::c_int == DEBUG_SBUS as libc::c_int {
        debug[DEBUG_SBUS_STATE_FLAGS as libc::c_int as usize] =
            (*sbusFrameData).stateFlags as int16_t
    }
    return sbusChannelsDecode(rxRuntimeConfig,
                              &mut (*sbusFrameData).frame.frame.channels);
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
pub unsafe extern "C" fn sbusInit(mut rxConfig: *const rxConfig_t,
                                  mut rxRuntimeConfig: *mut rxRuntimeConfig_t)
 -> bool {
    static mut sbusChannelData: [uint16_t; 18] = [0; 18];
    static mut sbusFrameData: sbusFrameData_t =
        sbusFrameData_t{frame: sbusFrame_u{bytes: [0; 25],},
                        startAtUs: 0,
                        stateFlags: 0,
                        position: 0,
                        done: false,};
    (*rxRuntimeConfig).channelData = sbusChannelData.as_mut_ptr();
    (*rxRuntimeConfig).frameData =
        &mut sbusFrameData as *mut sbusFrameData_t as *mut libc::c_void;
    sbusChannelsInit(rxConfig, rxRuntimeConfig);
    (*rxRuntimeConfig).channelCount = 18 as libc::c_int as uint8_t;
    (*rxRuntimeConfig).rxRefreshRate = 11000 as libc::c_int as uint16_t;
    (*rxRuntimeConfig).rcFrameStatusFn =
        Some(sbusFrameStatus as
                 unsafe extern "C" fn(_: *mut rxRuntimeConfig_t) -> uint8_t);
    let mut portConfig: *const serialPortConfig_t =
        findSerialPortConfig(FUNCTION_RX_SERIAL);
    if portConfig.is_null() { return 0 as libc::c_int != 0 }
    let mut portShared: bool = telemetryCheckRxPortShared(portConfig);
    let mut sBusPort: *mut serialPort_t =
        openSerialPort((*portConfig).identifier, FUNCTION_RX_SERIAL,
                       Some(sbusDataReceive as
                                unsafe extern "C" fn(_: uint16_t,
                                                     _: *mut libc::c_void)
                                    -> ()),
                       &mut sbusFrameData as *mut sbusFrameData_t as
                           *mut libc::c_void,
                       100000 as libc::c_int as uint32_t,
                       if portShared as libc::c_int != 0 {
                           MODE_RXTX as libc::c_int
                       } else { MODE_RX as libc::c_int } as portMode_e,
                       (SERIAL_STOPBITS_2 as libc::c_int |
                            SERIAL_PARITY_EVEN as libc::c_int |
                            (if (*rxConfig).serialrx_inverted as libc::c_int
                                    != 0 {
                                 0 as libc::c_int
                             } else { SERIAL_INVERTED as libc::c_int }) |
                            (if (*rxConfig).halfDuplex as libc::c_int != 0 {
                                 SERIAL_BIDIR as libc::c_int
                             } else { 0 as libc::c_int })) as portOptions_e);
    if (*rxConfig).rssi_src_frame_errors != 0 {
        rssiSource = RSSI_SOURCE_FRAME_ERRORS
    }
    if portShared { telemetrySharedPort = sBusPort }
    return !sBusPort.is_null();
}
