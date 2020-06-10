use ::libc;
extern "C" {
    #[no_mangle]
    fn crc16_ccitt_update(crc: uint16_t, data: *const libc::c_void,
                          length: uint32_t) -> uint16_t;
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
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
//16 channels transfare, but only 12 channels use for
// Pulse length convertion from [0...4095] to µs:
//      800µs  -> 0x000
//      1500µs -> 0x800
//      2200µs -> 0xFFF
// Total range is: 2200 - 800 = 1400 <==> 4095
// Use formula: 800 + value * 1400 / 4096 (i.e. a shift by 12)
static mut xBusFrameReceived: bool = 0 as libc::c_int != 0;
static mut xBusDataIncoming: bool = 0 as libc::c_int != 0;
static mut xBusFramePosition: uint8_t = 0;
static mut xBusFrameLength: uint8_t = 0;
static mut xBusChannelCount: uint8_t = 0;
static mut xBusProvider: uint8_t = 0;
// Use max values for ram areas
static mut xBusFrame: [uint8_t; 35] = [0; 35];
//size 35 for 16 channels in xbus_Mode_B
static mut xBusChannelData: [uint16_t; 12] = [0; 12];
// Full RJ01 message CRC calculations
unsafe extern "C" fn xBusRj01CRC8(mut inData: uint8_t, mut seed: uint8_t)
 -> uint8_t {
    let mut bitsLeft: uint8_t = 8 as libc::c_int as uint8_t;
    while bitsLeft as libc::c_int > 0 as libc::c_int {
        let temp: uint8_t =
            ((seed as libc::c_int ^ inData as libc::c_int) &
                 0x1 as libc::c_int) as uint8_t;
        if temp as libc::c_int == 0 as libc::c_int {
            seed = (seed as libc::c_int >> 1 as libc::c_int) as uint8_t
        } else {
            seed = (seed as libc::c_int ^ 0x18 as libc::c_int) as uint8_t;
            seed = (seed as libc::c_int >> 1 as libc::c_int) as uint8_t;
            seed = (seed as libc::c_int | 0x80 as libc::c_int) as uint8_t
        }
        inData = (inData as libc::c_int >> 1 as libc::c_int) as uint8_t;
        bitsLeft = bitsLeft.wrapping_sub(1)
    }
    return seed;
}
unsafe extern "C" fn xBusUnpackModeBFrame(mut offsetBytes: uint8_t) {
    // Calculate the CRC of the incoming frame
    // Calculate on all bytes except the final two CRC bytes
    let inCrc: uint16_t =
        crc16_ccitt_update(0 as libc::c_int as uint16_t,
                           &mut *xBusFrame.as_mut_ptr().offset(offsetBytes as
                                                                   isize) as
                               *mut uint8_t as *mut uint8_t as
                               *const libc::c_void,
                           (xBusFrameLength as libc::c_int - 2 as libc::c_int)
                               as uint32_t);
    // Get the received CRC
    let crc: uint16_t =
        (((xBusFrame[(offsetBytes as libc::c_int +
                          xBusFrameLength as libc::c_int - 2 as libc::c_int)
                         as usize] as uint16_t as libc::c_int) <<
              8 as libc::c_int) +
             xBusFrame[(offsetBytes as libc::c_int +
                            xBusFrameLength as libc::c_int - 1 as libc::c_int)
                           as usize] as uint16_t as libc::c_int) as uint16_t;
    if crc as libc::c_int == inCrc as libc::c_int {
        // Unpack the data, we have a valid frame, only 12 channel unpack also when receive 16 channel
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < xBusChannelCount as libc::c_int {
            let frameAddr: uint8_t =
                (offsetBytes as libc::c_int + 1 as libc::c_int +
                     i * 2 as libc::c_int) as uint8_t;
            let mut value: uint16_t =
                ((xBusFrame[frameAddr as usize] as uint16_t as libc::c_int) <<
                     8 as libc::c_int) as uint16_t;
            value =
                (value as libc::c_int +
                     xBusFrame[(frameAddr as libc::c_int + 1 as libc::c_int)
                                   as usize] as uint16_t as libc::c_int) as
                    uint16_t;
            // Convert to internal format
            xBusChannelData[i as usize] =
                (800 as libc::c_int +
                     (value as libc::c_int * 1400 as libc::c_int >>
                          12 as libc::c_int)) as uint16_t;
            i += 1
        }
        xBusFrameReceived = 1 as libc::c_int != 0
    };
}
unsafe extern "C" fn xBusUnpackRJ01Frame() {
    // Calculate the CRC of the incoming frame
    let mut outerCrc: uint8_t = 0 as libc::c_int as uint8_t;
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    // When using the Align RJ01 receiver with
    // a MODE B setting in the radio (XG14 tested)
    // the MODE_B -frame is packed within some
    // at the moment unknown bytes before and after:
    // 0xA1 LEN __ 0xA1 12*(High + Low) CRC1 CRC2 + __ __ CRC_OUTER
    // Compared to a standard MODE B frame that only
    // contains the "middle" package.
    // Hence, at the moment, the unknown header and footer
    // of the RJ01 MODEB packages are discarded.
    // However, the LAST byte (CRC_OUTER) is infact an 8-bit
    // CRC for the whole package, using the Dallas-One-Wire CRC
    // method.
    // So, we check both these values as well as the provided length
    // of the outer/full message (LEN)
    //
    // Check we have correct length of message
    //
    if xBusFrame[1 as libc::c_int as usize] as libc::c_int !=
           30 as libc::c_int {
        // Unknown package as length is not ok
        return
    }
    //
    // CRC calculation & check for full message
    //
    i = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) <
              xBusFrameLength as libc::c_int - 1 as libc::c_int {
        outerCrc = xBusRj01CRC8(outerCrc, xBusFrame[i as usize]);
        i = i.wrapping_add(1)
    }
    if outerCrc as libc::c_int !=
           xBusFrame[(xBusFrameLength as libc::c_int - 1 as libc::c_int) as
                         usize] as libc::c_int {
        // CRC does not match, skip this frame
        return
    }
    // Now unpack the "embedded MODE B frame"
    xBusUnpackModeBFrame(3 as libc::c_int as uint8_t);
}
// Receive ISR callback
unsafe extern "C" fn xBusDataReceive(mut c: uint16_t,
                                     mut data: *mut libc::c_void) {
    let mut now: uint32_t = 0;
    static mut xBusTimeLast: uint32_t = 0;
    static mut xBusTimeInterval: uint32_t = 0;
    // Check if we shall reset frame position due to time
    now = micros();
    xBusTimeInterval = now.wrapping_sub(xBusTimeLast);
    xBusTimeLast = now;
    if xBusTimeInterval > 8000 as libc::c_int as libc::c_uint {
        xBusFramePosition = 0 as libc::c_int as uint8_t;
        xBusDataIncoming = 0 as libc::c_int != 0
    }
    // Check if we shall start a frame?
    if xBusFramePosition as libc::c_int == 0 as libc::c_int {
        if c as libc::c_int == 0xa1 as libc::c_int {
            xBusDataIncoming = 1 as libc::c_int != 0;
            xBusFrameLength = 27 as libc::c_int as uint8_t
            //decrease framesize (when receiver change, otherwise board must reboot)
        } else if c as libc::c_int == 0xa2 as libc::c_int {
            //16channel packet
            xBusDataIncoming = 1 as libc::c_int != 0;
            xBusFrameLength = 35 as libc::c_int as uint8_t
            //increase framesize
        }
    }
    // Only do this if we are receiving to a frame
    if xBusDataIncoming as libc::c_int == 1 as libc::c_int {
        // Store in frame copy
        ::core::ptr::write_volatile(&mut xBusFrame[xBusFramePosition as usize]
                                        as *mut uint8_t, c as uint8_t);
        xBusFramePosition = xBusFramePosition.wrapping_add(1)
    }
    // Done?
    if xBusFramePosition as libc::c_int == xBusFrameLength as libc::c_int {
        let mut current_block_25: u64;
        match xBusProvider as libc::c_int {
            5 => {
                xBusUnpackModeBFrame(0 as libc::c_int as uint8_t);
                current_block_25 = 3662251116992205846;
            }
            6 => { current_block_25 = 3662251116992205846; }
            _ => { current_block_25 = 14763689060501151050; }
        }
        match current_block_25 {
            3662251116992205846 =>
            // !!TODO - check this fall through is correct
            {
                xBusUnpackRJ01Frame();
            }
            _ => { }
        }
        xBusDataIncoming = 0 as libc::c_int != 0;
        xBusFramePosition = 0 as libc::c_int as uint8_t
    };
}
// Indicate time to read a frame from the data...
unsafe extern "C" fn xBusFrameStatus(mut rxRuntimeConfig:
                                         *mut rxRuntimeConfig_t) -> uint8_t {
    if !xBusFrameReceived {
        return RX_FRAME_PENDING as libc::c_int as uint8_t
    }
    xBusFrameReceived = 0 as libc::c_int != 0;
    return RX_FRAME_COMPLETE as libc::c_int as uint8_t;
}
unsafe extern "C" fn xBusReadRawRC(mut rxRuntimeConfig:
                                       *const rxRuntimeConfig_t,
                                   mut chan: uint8_t) -> uint16_t {
    let mut data: uint16_t = 0;
    // Deliver the data wanted
    if chan as libc::c_int >= (*rxRuntimeConfig).channelCount as libc::c_int {
        return 0 as libc::c_int as uint16_t
    }
    data = xBusChannelData[chan as usize];
    return data;
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
pub unsafe extern "C" fn xBusInit(mut rxConfig: *const rxConfig_t,
                                  mut rxRuntimeConfig: *mut rxRuntimeConfig_t)
 -> bool {
    let mut baudRate: uint32_t = 0;
    match (*rxConfig).serialrx_provider as libc::c_int {
        5 => {
            (*rxRuntimeConfig).channelCount = 12 as libc::c_int as uint8_t;
            xBusFrameReceived = 0 as libc::c_int != 0;
            xBusDataIncoming = 0 as libc::c_int != 0;
            xBusFramePosition = 0 as libc::c_int as uint8_t;
            baudRate = 115200 as libc::c_int as uint32_t;
            xBusFrameLength = 27 as libc::c_int as uint8_t;
            xBusChannelCount = 12 as libc::c_int as uint8_t;
            xBusProvider = SERIALRX_XBUS_MODE_B as libc::c_int as uint8_t
        }
        6 => {
            (*rxRuntimeConfig).channelCount = 12 as libc::c_int as uint8_t;
            xBusFrameReceived = 0 as libc::c_int != 0;
            xBusDataIncoming = 0 as libc::c_int != 0;
            xBusFramePosition = 0 as libc::c_int as uint8_t;
            baudRate = 250000 as libc::c_int as uint32_t;
            xBusFrameLength = 33 as libc::c_int as uint8_t;
            xBusChannelCount = 12 as libc::c_int as uint8_t;
            xBusProvider = SERIALRX_XBUS_MODE_B_RJ01 as libc::c_int as uint8_t
        }
        _ => { return 0 as libc::c_int != 0 }
    }
    (*rxRuntimeConfig).rxRefreshRate = 11000 as libc::c_int as uint16_t;
    (*rxRuntimeConfig).rcReadRawFn =
        Some(xBusReadRawRC as
                 unsafe extern "C" fn(_: *const rxRuntimeConfig_t, _: uint8_t)
                     -> uint16_t);
    (*rxRuntimeConfig).rcFrameStatusFn =
        Some(xBusFrameStatus as
                 unsafe extern "C" fn(_: *mut rxRuntimeConfig_t) -> uint8_t);
    let mut portConfig: *const serialPortConfig_t =
        findSerialPortConfig(FUNCTION_RX_SERIAL);
    if portConfig.is_null() { return 0 as libc::c_int != 0 }
    let mut portShared: bool = telemetryCheckRxPortShared(portConfig);
    let mut xBusPort: *mut serialPort_t =
        openSerialPort((*portConfig).identifier, FUNCTION_RX_SERIAL,
                       Some(xBusDataReceive as
                                unsafe extern "C" fn(_: uint16_t,
                                                     _: *mut libc::c_void)
                                    -> ()), 0 as *mut libc::c_void, baudRate,
                       if portShared as libc::c_int != 0 {
                           MODE_RXTX as libc::c_int
                       } else { MODE_RX as libc::c_int } as portMode_e,
                       ((if (*rxConfig).serialrx_inverted as libc::c_int != 0
                            {
                             SERIAL_INVERTED as libc::c_int
                         } else { 0 as libc::c_int }) |
                            (if (*rxConfig).halfDuplex as libc::c_int != 0 {
                                 SERIAL_BIDIR as libc::c_int
                             } else { 0 as libc::c_int })) as portOptions_e);
    if portShared { telemetrySharedPort = xBusPort }
    return !xBusPort.is_null();
}
