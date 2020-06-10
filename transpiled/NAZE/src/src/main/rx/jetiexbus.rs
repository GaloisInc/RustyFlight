use ::libc;
extern "C" {
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn serialSetMode(instance: *mut serialPort_t, mode: portMode_e);
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
// not used for all telemetry systems, e.g. HoTT only works at 19200.
// used by receiver driver to return channel data
pub type rcFrameStatusFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut rxRuntimeConfig_s) -> uint8_t>;
pub type rcReadRawDataFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s, _: uint8_t)
               -> uint16_t>;
pub type rxRuntimeConfig_t = rxRuntimeConfig_s;
pub type exBusHeader_e = libc::c_uint;
pub const EXBUS_HEADER_DATA: exBusHeader_e = 6;
pub const EXBUS_HEADER_SUBLEN: exBusHeader_e = 5;
pub const EXBUS_HEADER_DATA_ID: exBusHeader_e = 4;
pub const EXBUS_HEADER_PACKET_ID: exBusHeader_e = 3;
pub const EXBUS_HEADER_MSG_LEN: exBusHeader_e = 2;
pub const EXBUS_HEADER_REQ: exBusHeader_e = 1;
pub const EXBUS_HEADER_SYNC: exBusHeader_e = 0;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const EXBUS_STATE_PROCESSED: C2RustUnnamed_0 = 3;
pub const EXBUS_STATE_RECEIVED: C2RustUnnamed_0 = 2;
pub const EXBUS_STATE_IN_PROGRESS: C2RustUnnamed_0 = 1;
pub const EXBUS_STATE_ZERO: C2RustUnnamed_0 = 0;
// number of RC channels as reported by current input driver
// Frame contains Channel Data, but with a request for data
// Frame is a request Frame
#[no_mangle]
pub static mut jetiExBusPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
#[no_mangle]
pub static mut jetiTimeStampRequest: uint32_t = 0 as libc::c_int as uint32_t;
static mut jetiExBusFramePosition: uint8_t = 0;
static mut jetiExBusFrameLength: uint8_t = 0;
static mut jetiExBusFrameState: uint8_t =
    EXBUS_STATE_ZERO as libc::c_int as uint8_t;
#[no_mangle]
pub static mut jetiExBusRequestState: uint8_t =
    EXBUS_STATE_ZERO as libc::c_int as uint8_t;
// Use max values for ram areas
static mut jetiExBusChannelFrame: [uint8_t; 40] = [0; 40];
#[no_mangle]
pub static mut jetiExBusRequestFrame: [uint8_t; 9] = [0; 9];
static mut jetiExBusChannelData: [uint16_t; 16] = [0; 16];
// Jeti Ex Bus CRC calculations for a frame
#[no_mangle]
pub unsafe extern "C" fn jetiExBusCalcCRC16(mut pt: *mut uint8_t,
                                            mut msgLen: uint8_t) -> uint16_t {
    let mut crc16_data: uint16_t = 0 as libc::c_int as uint16_t;
    let mut data: uint8_t = 0 as libc::c_int as uint8_t;
    let mut mlen: uint8_t = 0 as libc::c_int as uint8_t;
    while (mlen as libc::c_int) < msgLen as libc::c_int {
        data =
            (*pt.offset(mlen as isize) as libc::c_int ^
                 crc16_data as uint8_t as libc::c_int &
                     0xff as libc::c_int as uint8_t as libc::c_int) as
                uint8_t;
        data =
            (data as libc::c_int ^ (data as libc::c_int) << 4 as libc::c_int)
                as uint8_t;
        crc16_data =
            (((data as uint16_t as libc::c_int) << 8 as libc::c_int |
                  (crc16_data as libc::c_int & 0xff00 as libc::c_int) >>
                      8 as libc::c_int) ^
                 (data as libc::c_int >> 4 as libc::c_int) as uint8_t as
                     libc::c_int ^
                 (data as uint16_t as libc::c_int) << 3 as libc::c_int) as
                uint16_t;
        mlen = mlen.wrapping_add(1)
    }
    return crc16_data;
}
#[no_mangle]
pub unsafe extern "C" fn jetiExBusDecodeChannelFrame(mut exBusFrame:
                                                         *mut uint8_t) {
    let mut value: uint16_t = 0;
    let mut frameAddr: uint8_t = 0;
    // Decode header
    match (*exBusFrame.offset(EXBUS_HEADER_SYNC as libc::c_int as isize) as
               uint16_t as libc::c_int) << 8 as libc::c_int |
              *exBusFrame.offset(EXBUS_HEADER_REQ as libc::c_int as isize) as
                  uint16_t as libc::c_int {
        15873 | 15875 => {
            // not yet specified
            let mut i: uint8_t = 0 as libc::c_int as uint8_t;
            while (i as libc::c_int) < 16 as libc::c_int {
                frameAddr =
                    (6 as libc::c_int + i as libc::c_int * 2 as libc::c_int)
                        as uint8_t;
                value =
                    ((*exBusFrame.offset((frameAddr as libc::c_int +
                                              1 as libc::c_int) as isize) as
                          uint16_t as libc::c_int) << 8 as libc::c_int) as
                        uint16_t;
                value =
                    (value as libc::c_int +
                         *exBusFrame.offset(frameAddr as isize) as uint16_t as
                             libc::c_int) as uint16_t;
                // Convert to internal format
                jetiExBusChannelData[i as usize] =
                    (value as libc::c_int >> 3 as libc::c_int) as uint16_t;
                i = i.wrapping_add(1)
            }
        }
        _ => { }
    };
}
#[no_mangle]
pub unsafe extern "C" fn jetiExBusFrameReset() {
    jetiExBusFramePosition = 0 as libc::c_int as uint8_t;
    jetiExBusFrameLength =
        (6 as libc::c_int + 16 as libc::c_int * 2 as libc::c_int +
             2 as libc::c_int) as uint8_t;
}
/*
  supported:
  0x3E 0x01 LEN Packet_ID 0x31 SUB_LEN Data_array CRC16      // Channel Data with telemetry request (2nd byte 0x01)
  0x3E 0x03 LEN Packet_ID 0x31 SUB_LEN Data_array CRC16      // Channel Data forbids answering (2nd byte 0x03)
  0x3D 0x01 0x08 Packet_ID 0x3A 0x00 CRC16                   // Telemetry Request EX telemetry (5th byte 0x3A)

  other messages - not supported:
  0x3D 0x01 0x09 Packet_ID 0x3B 0x01 0xF0 CRC16              // Jetibox request (5th byte 0x3B)
  ...
*/
// Receive ISR callback
unsafe extern "C" fn jetiExBusDataReceive(mut c: uint16_t,
                                          mut data: *mut libc::c_void) {
    let mut now: uint32_t = 0;
    static mut jetiExBusTimeLast: uint32_t = 0 as libc::c_int as uint32_t;
    static mut jetiExBusTimeInterval: int32_t = 0;
    static mut jetiExBusFrame: *mut uint8_t =
        0 as *const uint8_t as *mut uint8_t;
    // Check if we shall reset frame position due to time
    now = micros();
    jetiExBusTimeInterval = now.wrapping_sub(jetiExBusTimeLast) as int32_t;
    jetiExBusTimeLast = now;
    if jetiExBusTimeInterval > 1000 as libc::c_int {
        jetiExBusFrameReset();
        jetiExBusFrameState = EXBUS_STATE_ZERO as libc::c_int as uint8_t;
        jetiExBusRequestState = EXBUS_STATE_ZERO as libc::c_int as uint8_t
    }
    // Check if we shall start a frame?
    if jetiExBusFramePosition as libc::c_int == 0 as libc::c_int {
        match c as libc::c_int {
            62 => {
                jetiExBusFrameState =
                    EXBUS_STATE_IN_PROGRESS as libc::c_int as uint8_t;
                jetiExBusFrame = jetiExBusChannelFrame.as_mut_ptr()
            }
            61 => {
                jetiExBusRequestState =
                    EXBUS_STATE_IN_PROGRESS as libc::c_int as uint8_t;
                jetiExBusFrame = jetiExBusRequestFrame.as_mut_ptr()
            }
            _ => { return }
        }
    }
    // Store in frame copy
    *jetiExBusFrame.offset(jetiExBusFramePosition as isize) = c as uint8_t;
    jetiExBusFramePosition = jetiExBusFramePosition.wrapping_add(1);
    // Check the header for the message length
    if jetiExBusFramePosition as libc::c_int == 6 as libc::c_int {
        if jetiExBusFrameState as libc::c_int ==
               EXBUS_STATE_IN_PROGRESS as libc::c_int &&
               *jetiExBusFrame.offset(EXBUS_HEADER_MSG_LEN as libc::c_int as
                                          isize) as libc::c_int <=
                   6 as libc::c_int + 16 as libc::c_int * 2 as libc::c_int +
                       2 as libc::c_int {
            jetiExBusFrameLength =
                *jetiExBusFrame.offset(EXBUS_HEADER_MSG_LEN as libc::c_int as
                                           isize); // not a valid frame
            return
        }
        if jetiExBusRequestState as libc::c_int ==
               EXBUS_STATE_IN_PROGRESS as libc::c_int &&
               *jetiExBusFrame.offset(EXBUS_HEADER_MSG_LEN as libc::c_int as
                                          isize) as libc::c_int <=
                   9 as libc::c_int {
            jetiExBusFrameLength =
                *jetiExBusFrame.offset(EXBUS_HEADER_MSG_LEN as libc::c_int as
                                           isize);
            return
        }
        jetiExBusFrameReset();
        jetiExBusFrameState = EXBUS_STATE_ZERO as libc::c_int as uint8_t;
        jetiExBusRequestState = EXBUS_STATE_ZERO as libc::c_int as uint8_t;
        return
    }
    // Done?
    if jetiExBusFrameLength as libc::c_int ==
           jetiExBusFramePosition as libc::c_int {
        if jetiExBusFrameState as libc::c_int ==
               EXBUS_STATE_IN_PROGRESS as libc::c_int {
            jetiExBusFrameState =
                EXBUS_STATE_RECEIVED as libc::c_int as uint8_t
        }
        if jetiExBusRequestState as libc::c_int ==
               EXBUS_STATE_IN_PROGRESS as libc::c_int {
            jetiExBusRequestState =
                EXBUS_STATE_RECEIVED as libc::c_int as uint8_t;
            jetiTimeStampRequest = micros()
        }
        jetiExBusFrameReset();
    };
}
// Check if it is time to read a frame from the data...
unsafe extern "C" fn jetiExBusFrameStatus(mut rxRuntimeConfig:
                                              *mut rxRuntimeConfig_t)
 -> uint8_t {
    if jetiExBusFrameState as libc::c_int !=
           EXBUS_STATE_RECEIVED as libc::c_int {
        return RX_FRAME_PENDING as libc::c_int as uint8_t
    }
    if jetiExBusCalcCRC16(jetiExBusChannelFrame.as_mut_ptr(),
                          jetiExBusChannelFrame[EXBUS_HEADER_MSG_LEN as
                                                    libc::c_int as usize]) as
           libc::c_int == 0 as libc::c_int {
        jetiExBusDecodeChannelFrame(jetiExBusChannelFrame.as_mut_ptr());
        jetiExBusFrameState = EXBUS_STATE_ZERO as libc::c_int as uint8_t;
        return RX_FRAME_COMPLETE as libc::c_int as uint8_t
    } else {
        jetiExBusFrameState = EXBUS_STATE_ZERO as libc::c_int as uint8_t;
        return RX_FRAME_PENDING as libc::c_int as uint8_t
    };
}
unsafe extern "C" fn jetiExBusReadRawRC(mut rxRuntimeConfig:
                                            *const rxRuntimeConfig_t,
                                        mut chan: uint8_t) -> uint16_t {
    if chan as libc::c_int >= (*rxRuntimeConfig).channelCount as libc::c_int {
        return 0 as libc::c_int as uint16_t
    }
    return jetiExBusChannelData[chan as usize];
}
#[no_mangle]
pub unsafe extern "C" fn jetiExBusInit(mut rxConfig: *const rxConfig_t,
                                       mut rxRuntimeConfig:
                                           *mut rxRuntimeConfig_t) -> bool {
    (*rxRuntimeConfig).channelCount = 16 as libc::c_int as uint8_t;
    (*rxRuntimeConfig).rxRefreshRate = 5500 as libc::c_int as uint16_t;
    (*rxRuntimeConfig).rcReadRawFn =
        Some(jetiExBusReadRawRC as
                 unsafe extern "C" fn(_: *const rxRuntimeConfig_t, _: uint8_t)
                     -> uint16_t);
    (*rxRuntimeConfig).rcFrameStatusFn =
        Some(jetiExBusFrameStatus as
                 unsafe extern "C" fn(_: *mut rxRuntimeConfig_t) -> uint8_t);
    jetiExBusFrameReset();
    let mut portConfig: *const serialPortConfig_t =
        findSerialPortConfig(FUNCTION_RX_SERIAL);
    if portConfig.is_null() { return 0 as libc::c_int != 0 }
    jetiExBusPort =
        openSerialPort((*portConfig).identifier, FUNCTION_RX_SERIAL,
                       Some(jetiExBusDataReceive as
                                unsafe extern "C" fn(_: uint16_t,
                                                     _: *mut libc::c_void)
                                    -> ()), 0 as *mut libc::c_void,
                       125000 as libc::c_int as uint32_t, MODE_RXTX,
                       (SERIAL_STOPBITS_1 as libc::c_int |
                            SERIAL_PARITY_NO as libc::c_int |
                            (if (*rxConfig).serialrx_inverted as libc::c_int
                                    != 0 {
                                 SERIAL_INVERTED as libc::c_int
                             } else { 0 as libc::c_int }) |
                            (if (*rxConfig).halfDuplex as libc::c_int != 0 {
                                 SERIAL_BIDIR as libc::c_int
                             } else { 0 as libc::c_int })) as portOptions_e);
    serialSetMode(jetiExBusPort, MODE_RX);
    return !jetiExBusPort.is_null();
}
// SERIAL_RX
