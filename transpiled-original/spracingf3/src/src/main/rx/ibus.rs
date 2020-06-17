use core;
use libc;
extern "C" {
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn findSerialPortConfig(function: serialPortFunction_e)
     -> *mut serialPortConfig_t;
    #[no_mangle]
    fn isSerialPortShared(portConfig: *const serialPortConfig_t,
                          functionMask: uint16_t,
                          sharedWithFunction: serialPortFunction_e) -> bool;
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
    //defined(TELEMETRY) && defined(TELEMETRY_IBUS)
    #[no_mangle]
    fn isChecksumOkIa6b(ibusPacket: *const uint8_t, length: uint8_t) -> bool;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type ioTag_t = uint8_t;
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
pub type timeUs_t = uint32_t;
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
pub type C2RustUnnamed = libc::c_uint;
pub const RX_FRAME_DROPPED: C2RustUnnamed = 8;
pub const RX_FRAME_PROCESSING_REQUIRED: C2RustUnnamed = 4;
pub const RX_FRAME_FAILSAFE: C2RustUnnamed = 2;
pub const RX_FRAME_COMPLETE: C2RustUnnamed = 1;
pub const RX_FRAME_PENDING: C2RustUnnamed = 0;
#[derive ( Copy, Clone )]
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
pub type rcFrameStatusFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut rxRuntimeConfig_s) -> uint8_t>;
pub type rcReadRawDataFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s, _: uint8_t)
               -> uint16_t>;
pub type rxRuntimeConfig_t = rxRuntimeConfig_s;
static mut ibusModel: uint8_t = 0;
static mut ibusSyncByte: uint8_t = 0;
static mut ibusFrameSize: uint8_t = 0;
static mut ibusChannelOffset: uint8_t = 0;
static mut rxBytesToIgnore: uint8_t = 0;
static mut ibusChecksum: uint16_t = 0;
static mut ibusFrameDone: bool = 0i32 != 0;
static mut ibusChannelData: [uint32_t; 14] = [0; 14];
static mut ibus: [uint8_t; 32] =
    [0i32 as uint8_t, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
unsafe extern "C" fn isValidIa6bIbusPacketLength(mut length: uint8_t)
 -> bool {
    return length as libc::c_int == 4i32 || length as libc::c_int == 32i32;
}
// not used for all telemetry systems, e.g. HoTT only works at 19200.
// Receive ISR callback
unsafe extern "C" fn ibusDataReceive(mut c: uint16_t,
                                     mut data: *mut libc::c_void) {
    let mut ibusTime: uint32_t = 0;
    static mut ibusTimeLast: uint32_t = 0;
    static mut ibusFramePosition: uint8_t = 0;
    ibusTime = micros();
    if ibusTime.wrapping_sub(ibusTimeLast) > 500i32 as libc::c_uint {
        ibusFramePosition = 0i32 as uint8_t;
        rxBytesToIgnore = 0i32 as uint8_t
    } else if rxBytesToIgnore != 0 {
        rxBytesToIgnore = rxBytesToIgnore.wrapping_sub(1);
        return
    }
    ibusTimeLast = ibusTime;
    if ibusFramePosition as libc::c_int == 0i32 {
        if isValidIa6bIbusPacketLength(c as uint8_t) {
            ibusModel = 0i32 as uint8_t;
            ibusSyncByte = c as uint8_t;
            ibusFrameSize = c as uint8_t;
            ibusChannelOffset = 2i32 as uint8_t;
            ibusChecksum = 0xffffi32 as uint16_t
        } else if ibusSyncByte as libc::c_int == 0i32 &&
                      c as libc::c_int == 0x55i32 {
            ibusModel = 1i32 as uint8_t;
            ibusSyncByte = 0x55i32 as uint8_t;
            ibusFrameSize = 31i32 as uint8_t;
            ibusChecksum = 0i32 as uint16_t;
            ibusChannelOffset = 1i32 as uint8_t
        } else if ibusSyncByte as libc::c_int != c as libc::c_int { return }
    }
    ibus[ibusFramePosition as usize] = c as uint8_t;
    if ibusFramePosition as libc::c_int == ibusFrameSize as libc::c_int - 1i32
       {
        ibusFrameDone = 1i32 != 0
    } else { ibusFramePosition = ibusFramePosition.wrapping_add(1) };
}
unsafe extern "C" fn isChecksumOkIa6() -> bool {
    let mut offset: uint8_t = 0;
    let mut i: uint8_t = 0;
    let mut chksum: uint16_t = 0;
    let mut rxsum: uint16_t = 0;
    chksum = ibusChecksum;
    rxsum =
        (ibus[(ibusFrameSize as libc::c_int - 2i32) as usize] as libc::c_int +
             ((ibus[(ibusFrameSize as libc::c_int - 1i32) as usize] as
                   libc::c_int) << 8i32)) as uint16_t;
    i = 0i32 as uint8_t;
    offset = ibusChannelOffset;
    while (i as libc::c_int) < 14i32 {
        chksum =
            (chksum as libc::c_int +
                 (ibus[offset as usize] as libc::c_int +
                      ((ibus[(offset as libc::c_int + 1i32) as usize] as
                            libc::c_int) << 8i32))) as uint16_t;
        i = i.wrapping_add(1);
        offset = (offset as libc::c_int + 2i32) as uint8_t
    }
    return chksum as libc::c_int == rxsum as libc::c_int;
}
unsafe extern "C" fn checksumIsOk() -> bool {
    if ibusModel as libc::c_int == 1i32 {
        return isChecksumOkIa6()
    } else { return isChecksumOkIa6b(ibus.as_mut_ptr(), ibusFrameSize) };
}
unsafe extern "C" fn updateChannelData() {
    let mut i: uint8_t = 0;
    let mut offset: uint8_t = 0;
    i = 0i32 as uint8_t;
    offset = ibusChannelOffset;
    while (i as libc::c_int) < 14i32 {
        ibusChannelData[i as usize] =
            (ibus[offset as usize] as libc::c_int +
                 ((ibus[(offset as libc::c_int + 1i32) as usize] as
                       libc::c_int) << 8i32)) as uint32_t;
        i = i.wrapping_add(1);
        offset = (offset as libc::c_int + 2i32) as uint8_t
    };
}
unsafe extern "C" fn ibusFrameStatus(mut rxRuntimeConfig:
                                         *mut rxRuntimeConfig_t) -> uint8_t {
    let mut frameStatus: uint8_t = RX_FRAME_PENDING as libc::c_int as uint8_t;
    if !ibusFrameDone { return frameStatus }
    ibusFrameDone = 0i32 != 0;
    if checksumIsOk() {
        if ibusModel as libc::c_int == 1i32 ||
               ibusSyncByte as libc::c_int == 0x20i32 {
            updateChannelData();
            frameStatus = RX_FRAME_COMPLETE as libc::c_int as uint8_t
        }
    }
    return frameStatus;
}
unsafe extern "C" fn ibusReadRawRC(mut rxRuntimeConfig:
                                       *const rxRuntimeConfig_t,
                                   mut chan: uint8_t) -> uint16_t {
    return ibusChannelData[chan as usize] as uint16_t;
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
pub unsafe extern "C" fn ibusInit(mut rxConfig: *const rxConfig_t,
                                  mut rxRuntimeConfig: *mut rxRuntimeConfig_t)
 -> bool {
    ibusSyncByte = 0i32 as uint8_t; // TODO - Verify speed
    (*rxRuntimeConfig).channelCount = 14i32 as uint8_t;
    (*rxRuntimeConfig).rxRefreshRate = 20000i32 as uint16_t;
    (*rxRuntimeConfig).rcReadRawFn =
        Some(ibusReadRawRC as
                 unsafe extern "C" fn(_: *const rxRuntimeConfig_t, _: uint8_t)
                     -> uint16_t);
    (*rxRuntimeConfig).rcFrameStatusFn =
        Some(ibusFrameStatus as
                 unsafe extern "C" fn(_: *mut rxRuntimeConfig_t) -> uint8_t);
    let mut portConfig: *const serialPortConfig_t =
        findSerialPortConfig(FUNCTION_RX_SERIAL);
    if portConfig.is_null() { return 0i32 != 0 }
    // bool portShared = telemetryCheckRxPortShared(portConfig);
    let mut portShared: bool =
        isSerialPortShared(portConfig,
                           FUNCTION_RX_SERIAL as libc::c_int as uint16_t,
                           FUNCTION_TELEMETRY_IBUS);
    rxBytesToIgnore = 0i32 as uint8_t;
    let mut ibusPort: *mut serialPort_t =
        openSerialPort((*portConfig).identifier, FUNCTION_RX_SERIAL,
                       Some(ibusDataReceive as
                                unsafe extern "C" fn(_: uint16_t,
                                                     _: *mut libc::c_void)
                                    -> ()), 0 as *mut libc::c_void,
                       115200i32 as uint32_t,
                       if portShared as libc::c_int != 0 {
                           MODE_RXTX as libc::c_int
                       } else { MODE_RX as libc::c_int } as portMode_e,
                       ((if (*rxConfig).serialrx_inverted as libc::c_int != 0
                            {
                             SERIAL_INVERTED as libc::c_int
                         } else { 0i32 }) |
                            (if (*rxConfig).halfDuplex as libc::c_int != 0 ||
                                    portShared as libc::c_int != 0 {
                                 SERIAL_BIDIR as libc::c_int
                             } else { 0i32 })) as portOptions_e);
    return !ibusPort.is_null();
}
//SERIAL_RX
