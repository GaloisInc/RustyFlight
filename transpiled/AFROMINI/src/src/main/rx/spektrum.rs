use ::libc;
extern "C" {
    #[no_mangle]
    fn micros() -> timeUs_t;
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
    fn dispatchEnable();
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type ioTag_t = uint8_t;
// microsecond time
pub type timeUs_t = uint32_t;
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
pub type C2RustUnnamed = libc::c_uint;
pub const RX_FRAME_DROPPED: C2RustUnnamed = 8;
pub const RX_FRAME_PROCESSING_REQUIRED: C2RustUnnamed = 4;
pub const RX_FRAME_FAILSAFE: C2RustUnnamed = 2;
pub const RX_FRAME_COMPLETE: C2RustUnnamed = 1;
pub const RX_FRAME_PENDING: C2RustUnnamed = 0;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const SERIALRX_FPORT: C2RustUnnamed_0 = 12;
pub const SERIALRX_TARGET_CUSTOM: C2RustUnnamed_0 = 11;
pub const SERIALRX_SRXL: C2RustUnnamed_0 = 10;
pub const SERIALRX_CRSF: C2RustUnnamed_0 = 9;
pub const SERIALRX_JETIEXBUS: C2RustUnnamed_0 = 8;
pub const SERIALRX_IBUS: C2RustUnnamed_0 = 7;
pub const SERIALRX_XBUS_MODE_B_RJ01: C2RustUnnamed_0 = 6;
pub const SERIALRX_XBUS_MODE_B: C2RustUnnamed_0 = 5;
pub const SERIALRX_SUMH: C2RustUnnamed_0 = 4;
pub const SERIALRX_SUMD: C2RustUnnamed_0 = 3;
pub const SERIALRX_SBUS: C2RustUnnamed_0 = 2;
pub const SERIALRX_SPEKTRUM2048: C2RustUnnamed_0 = 1;
pub const SERIALRX_SPEKTRUM1024: C2RustUnnamed_0 = 0;
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
// number of RC channels as reported by current input driver
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
// driver for spektrum satellite receiver / sbus
#[no_mangle]
pub static mut srxlEnabled: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub static mut resolution: int32_t = 0;
#[no_mangle]
pub static mut rssi_channel: uint8_t = 0;
static mut spek_chan_shift: uint8_t = 0;
static mut spek_chan_mask: uint8_t = 0;
static mut rcFrameComplete: bool = 0 as libc::c_int != 0;
static mut spekHiRes: bool = 0 as libc::c_int != 0;
static mut spekFrame: [uint8_t; 16] = [0; 16];
static mut rxRuntimeConfigPtr: *mut rxRuntimeConfig_t =
    0 as *const rxRuntimeConfig_t as *mut rxRuntimeConfig_t;
static mut serialPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
// Receive ISR callback
unsafe extern "C" fn spektrumDataReceive(mut c: uint16_t,
                                         mut data: *mut libc::c_void) {
    let mut spekTime: uint32_t = 0;
    let mut spekTimeInterval: uint32_t = 0;
    static mut spekTimeLast: uint32_t = 0 as libc::c_int as uint32_t;
    static mut spekFramePosition: uint8_t = 0 as libc::c_int as uint8_t;
    spekTime = micros();
    spekTimeInterval = spekTime.wrapping_sub(spekTimeLast);
    spekTimeLast = spekTime;
    if spekTimeInterval > 5000 as libc::c_int as libc::c_uint {
        spekFramePosition = 0 as libc::c_int as uint8_t
    }
    if (spekFramePosition as libc::c_int) < 16 as libc::c_int {
        let fresh0 = spekFramePosition;
        spekFramePosition = spekFramePosition.wrapping_add(1);
        ::core::ptr::write_volatile(&mut spekFrame[fresh0 as usize] as
                                        *mut uint8_t, c as uint8_t);
        if (spekFramePosition as libc::c_int) < 16 as libc::c_int {
            rcFrameComplete = 0 as libc::c_int != 0
        } else { rcFrameComplete = 1 as libc::c_int != 0 }
    };
}
#[no_mangle]
pub static mut spekChannelData: [uint32_t; 12] = [0; 12];
unsafe extern "C" fn spektrumFrameStatus(mut rxRuntimeConfig:
                                             *mut rxRuntimeConfig_t)
 -> uint8_t {
    if !rcFrameComplete { return RX_FRAME_PENDING as libc::c_int as uint8_t }
    rcFrameComplete = 0 as libc::c_int != 0;
    // Get the VTX control bytes in a frame
    let mut vtxControl: uint32_t =
        ((spekFrame[(16 as libc::c_int - 4 as libc::c_int) as usize] as
              libc::c_int) << 24 as libc::c_int |
             (spekFrame[(16 as libc::c_int - 3 as libc::c_int) as usize] as
                  libc::c_int) << 16 as libc::c_int |
             (spekFrame[(16 as libc::c_int - 2 as libc::c_int) as usize] as
                  libc::c_int) << 8 as libc::c_int |
             (spekFrame[(16 as libc::c_int - 1 as libc::c_int) as usize] as
                  libc::c_int) << 0 as libc::c_int) as uint32_t;
    let mut spektrumRcDataSize: int8_t = 0;
    // Handle VTX control frame.
    if vtxControl & 0xf000f000 as libc::c_uint == 0xe000e000 as libc::c_uint
           &&
           spekFrame[2 as libc::c_int as usize] as libc::c_int &
               0x80 as libc::c_int == 0 as libc::c_int {
        spektrumRcDataSize = (16 as libc::c_int - 4 as libc::c_int) as int8_t
    } else { spektrumRcDataSize = 16 as libc::c_int as int8_t }
    // Get the RC control channel inputs
    let mut b: libc::c_int = 3 as libc::c_int; // 1024 mode
    while b < spektrumRcDataSize as libc::c_int {
        let spekChannel: uint8_t =
            (0xf as libc::c_int &
                 spekFrame[(b - 1 as libc::c_int) as usize] as libc::c_int >>
                     spek_chan_shift as libc::c_int) as uint8_t; // 2048 mode
        if (spekChannel as libc::c_int) <
               (*rxRuntimeConfigPtr).channelCount as libc::c_int &&
               (spekChannel as libc::c_int) < 12 as libc::c_int {
            if rssi_channel as libc::c_int == 0 as libc::c_int ||
                   spekChannel as libc::c_int != rssi_channel as libc::c_int {
                spekChannelData[spekChannel as usize] =
                    (((spekFrame[(b - 1 as libc::c_int) as usize] as
                           libc::c_int & spek_chan_mask as libc::c_int) as
                          uint32_t) <<
                         8 as
                             libc::c_int).wrapping_add(spekFrame[b as usize]
                                                           as libc::c_uint)
            }
        }
        b += 2 as libc::c_int
    }
    return RX_FRAME_COMPLETE as libc::c_int as uint8_t;
}
unsafe extern "C" fn spektrumReadRawRC(mut rxRuntimeConfig:
                                           *const rxRuntimeConfig_t,
                                       mut chan: uint8_t) -> uint16_t {
    let mut data: uint16_t = 0;
    if chan as libc::c_int >= (*rxRuntimeConfig).channelCount as libc::c_int {
        return 0 as libc::c_int as uint16_t
    }
    if spekHiRes {
        data =
            (988 as libc::c_int as
                 libc::c_uint).wrapping_add(spekChannelData[chan as usize] >>
                                                1 as libc::c_int) as uint16_t
    } else {
        data =
            (988 as libc::c_int as
                 libc::c_uint).wrapping_add(spekChannelData[chan as usize]) as
                uint16_t
    }
    return data;
}
// USE_SPEKTRUM_BIND
#[no_mangle]
pub unsafe extern "C" fn spektrumInit(mut rxConfig: *const rxConfig_t,
                                      mut rxRuntimeConfig:
                                          *mut rxRuntimeConfig_t) -> bool {
    rxRuntimeConfigPtr = rxRuntimeConfig;
    let mut portConfig: *const serialPortConfig_t =
        findSerialPortConfig(FUNCTION_RX_SERIAL);
    if portConfig.is_null() { return 0 as libc::c_int != 0 }
    srxlEnabled = 0 as libc::c_int != 0;
    let mut portShared: bool = 0 as libc::c_int != 0;
    match (*rxConfig).serialrx_provider as libc::c_int {
        10 | 1 => {
            // 11 bit frames
            spek_chan_shift = 3 as libc::c_int as uint8_t;
            spek_chan_mask = 0x7 as libc::c_int as uint8_t;
            spekHiRes = 1 as libc::c_int != 0;
            resolution = 2048 as libc::c_int;
            (*rxRuntimeConfig).channelCount = 12 as libc::c_int as uint8_t;
            (*rxRuntimeConfig).rxRefreshRate =
                11000 as libc::c_int as uint16_t
        }
        0 => {
            // 10 bit frames
            spek_chan_shift =
                2 as libc::c_int as
                    uint8_t; // -1 because rxConfig->rssi_channel is 1-based and rssi_channel is 0-based.
            spek_chan_mask = 0x3 as libc::c_int as uint8_t;
            spekHiRes = 0 as libc::c_int != 0;
            resolution = 1024 as libc::c_int;
            (*rxRuntimeConfig).channelCount = 7 as libc::c_int as uint8_t;
            (*rxRuntimeConfig).rxRefreshRate =
                22000 as libc::c_int as uint16_t
        }
        _ => { }
    }
    (*rxRuntimeConfig).rcReadRawFn =
        Some(spektrumReadRawRC as
                 unsafe extern "C" fn(_: *const rxRuntimeConfig_t, _: uint8_t)
                     -> uint16_t);
    (*rxRuntimeConfig).rcFrameStatusFn =
        Some(spektrumFrameStatus as
                 unsafe extern "C" fn(_: *mut rxRuntimeConfig_t) -> uint8_t);
    serialPort =
        openSerialPort((*portConfig).identifier, FUNCTION_RX_SERIAL,
                       Some(spektrumDataReceive as
                                unsafe extern "C" fn(_: uint16_t,
                                                     _: *mut libc::c_void)
                                    -> ()), 0 as *mut libc::c_void,
                       115200 as libc::c_int as uint32_t,
                       if portShared as libc::c_int != 0 ||
                              srxlEnabled as libc::c_int != 0 {
                           MODE_RXTX as libc::c_int
                       } else { MODE_RX as libc::c_int } as portMode_e,
                       ((if (*rxConfig).serialrx_inverted as libc::c_int != 0
                            {
                             SERIAL_INVERTED as libc::c_int
                         } else { 0 as libc::c_int }) |
                            (if srxlEnabled as libc::c_int != 0 ||
                                    (*rxConfig).halfDuplex as libc::c_int != 0
                                {
                                 SERIAL_BIDIR as libc::c_int
                             } else { 0 as libc::c_int })) as portOptions_e);
    rssi_channel =
        ((*rxConfig).rssi_channel as libc::c_int - 1 as libc::c_int) as
            uint8_t;
    if rssi_channel as libc::c_int >=
           (*rxRuntimeConfig).channelCount as libc::c_int {
        rssi_channel = 0 as libc::c_int as uint8_t
    }
    if !serialPort.is_null() && srxlEnabled as libc::c_int != 0 {
        dispatchEnable();
    }
    return !serialPort.is_null();
}
#[no_mangle]
pub unsafe extern "C" fn srxlRxIsActive() -> bool {
    return !serialPort.is_null();
}
// SERIAL_RX
