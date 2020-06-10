use ::libc;
use ::c2rust_bitfields;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn crc8_dvb_s2(crc: uint8_t, a: libc::c_uchar) -> uint8_t;
    #[no_mangle]
    fn serialWriteBuf(instance: *mut serialPort_t, data: *const uint8_t,
                      count: libc::c_int);
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
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
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
pub type C2RustUnnamed_0 = libc::c_uint;
pub const CRSF_FRAMETYPE_DISPLAYPORT_CMD: C2RustUnnamed_0 = 125;
pub const CRSF_FRAMETYPE_MSP_WRITE: C2RustUnnamed_0 = 124;
pub const CRSF_FRAMETYPE_MSP_RESP: C2RustUnnamed_0 = 123;
pub const CRSF_FRAMETYPE_MSP_REQ: C2RustUnnamed_0 = 122;
pub const CRSF_FRAMETYPE_COMMAND: C2RustUnnamed_0 = 50;
pub const CRSF_FRAMETYPE_PARAMETER_WRITE: C2RustUnnamed_0 = 45;
pub const CRSF_FRAMETYPE_PARAMETER_READ: C2RustUnnamed_0 = 44;
pub const CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY: C2RustUnnamed_0 = 43;
pub const CRSF_FRAMETYPE_DEVICE_INFO: C2RustUnnamed_0 = 41;
pub const CRSF_FRAMETYPE_DEVICE_PING: C2RustUnnamed_0 = 40;
pub const CRSF_FRAMETYPE_FLIGHT_MODE: C2RustUnnamed_0 = 33;
pub const CRSF_FRAMETYPE_ATTITUDE: C2RustUnnamed_0 = 30;
pub const CRSF_FRAMETYPE_RC_CHANNELS_PACKED: C2RustUnnamed_0 = 22;
pub const CRSF_FRAMETYPE_LINK_STATISTICS: C2RustUnnamed_0 = 20;
pub const CRSF_FRAMETYPE_BATTERY_SENSOR: C2RustUnnamed_0 = 8;
pub const CRSF_FRAMETYPE_GPS: C2RustUnnamed_0 = 2;
pub type C2RustUnnamed_1 = libc::c_uint;
// number of RC channels as reported by current input driver
// 11 bits per channel * 16 channels = 22 bytes.
pub const CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE: C2RustUnnamed_1 = 6;
pub const CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE: C2RustUnnamed_1 = 22;
pub const CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE: C2RustUnnamed_1 = 10;
pub const CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE: C2RustUnnamed_1 = 8;
pub const CRSF_FRAME_GPS_PAYLOAD_SIZE: C2RustUnnamed_1 = 15;
pub type C2RustUnnamed_2 = libc::c_uint;
// combined length of all fields except payload
// length of Extended Dest/Origin, TYPE and CRC fields combined
pub const CRSF_FRAME_LENGTH_NON_PAYLOAD: C2RustUnnamed_2 = 4;
// length of TYPE and CRC fields combined
pub const CRSF_FRAME_LENGTH_EXT_TYPE_CRC: C2RustUnnamed_2 = 4;
// length of CRC field
pub const CRSF_FRAME_LENGTH_TYPE_CRC: C2RustUnnamed_2 = 2;
// length of TYPE field
pub const CRSF_FRAME_LENGTH_CRC: C2RustUnnamed_2 = 1;
// length of FRAMELENGTH field
pub const CRSF_FRAME_LENGTH_TYPE: C2RustUnnamed_2 = 1;
// length of ADDRESS field
pub const CRSF_FRAME_LENGTH_FRAMELENGTH: C2RustUnnamed_2 = 1;
pub const CRSF_FRAME_LENGTH_ADDRESS: C2RustUnnamed_2 = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct crsfFrameDef_s {
    pub deviceAddress: uint8_t,
    pub frameLength: uint8_t,
    pub type_0: uint8_t,
    pub payload: [uint8_t; 59],
}
pub type crsfFrameDef_t = crsfFrameDef_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub union crsfFrame_u {
    pub bytes: [uint8_t; 64],
    pub frame: crsfFrameDef_t,
}
pub type crsfFrame_t = crsfFrame_u;
pub type crsfPayloadRcChannelsPacked_t = crsfPayloadRcChannelsPacked_s;
// +1 for CRC at end of payload
/*
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 *
 */
#[derive(Copy, Clone, BitfieldStruct)]
#[repr(C, packed)]
pub struct crsfPayloadRcChannelsPacked_s {
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
}
// At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz
static mut crsfFrameDone: bool = 0 as libc::c_int != 0;
static mut crsfFrame: crsfFrame_t = crsfFrame_u{bytes: [0; 64],};
static mut crsfChannelData: [uint32_t; 16] = [0; 16];
static mut serialPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
static mut crsfFrameStartAtUs: uint32_t = 0 as libc::c_int as uint32_t;
static mut telemetryBuf: [uint8_t; 64] = [0; 64];
static mut telemetryBufLen: uint8_t = 0 as libc::c_int as uint8_t;
unsafe extern "C" fn crsfFrameCRC() -> uint8_t {
    // CRC includes type and payload
    let mut crc: uint8_t =
        crc8_dvb_s2(0 as libc::c_int as uint8_t, crsfFrame.frame.type_0);
    let mut ii: libc::c_int = 0 as libc::c_int;
    while ii <
              crsfFrame.frame.frameLength as libc::c_int -
                  CRSF_FRAME_LENGTH_TYPE_CRC as libc::c_int {
        crc = crc8_dvb_s2(crc, crsfFrame.frame.payload[ii as usize]);
        ii += 1
    }
    return crc;
}
// Receive ISR callback, called back from serial port
unsafe extern "C" fn crsfDataReceive(mut c: uint16_t,
                                     mut data: *mut libc::c_void) {
    static mut crsfFramePosition: uint8_t = 0 as libc::c_int as uint8_t;
    let currentTimeUs: uint32_t = micros();
    if currentTimeUs >
           crsfFrameStartAtUs.wrapping_add(1100 as libc::c_int as
                                               libc::c_uint) {
        // We've received a character after max time needed to complete a frame,
        // so this must be the start of a new frame.
        crsfFramePosition = 0 as libc::c_int as uint8_t
    }
    if crsfFramePosition as libc::c_int == 0 as libc::c_int {
        crsfFrameStartAtUs = currentTimeUs
    }
    // assume frame is 5 bytes long until we have received the frame length
    // full frame length includes the length of the address and framelength fields
    let fullFrameLength: libc::c_int =
        if (crsfFramePosition as libc::c_int) < 3 as libc::c_int {
            5 as libc::c_int
        } else {
            (crsfFrame.frame.frameLength as libc::c_int +
                 CRSF_FRAME_LENGTH_ADDRESS as libc::c_int) +
                CRSF_FRAME_LENGTH_FRAMELENGTH as libc::c_int
        };
    if (crsfFramePosition as libc::c_int) < fullFrameLength {
        let fresh0 = crsfFramePosition;
        crsfFramePosition = crsfFramePosition.wrapping_add(1);
        crsfFrame.bytes[fresh0 as usize] = c as uint8_t;
        crsfFrameDone =
            if (crsfFramePosition as libc::c_int) < fullFrameLength {
                0 as libc::c_int
            } else { 1 as libc::c_int } != 0;
        if crsfFrameDone {
            crsfFramePosition = 0 as libc::c_int as uint8_t;
            if crsfFrame.frame.type_0 as libc::c_int !=
                   CRSF_FRAMETYPE_RC_CHANNELS_PACKED as libc::c_int {
                let crc: uint8_t = crsfFrameCRC();
                if crc as libc::c_int ==
                       crsfFrame.bytes[(fullFrameLength - 1 as libc::c_int) as
                                           usize] as libc::c_int {
                    match crsfFrame.frame.type_0 as libc::c_int { _ => { } }
                }
            }
        }
    };
}
unsafe extern "C" fn crsfFrameStatus(mut rxRuntimeConfig:
                                         *mut rxRuntimeConfig_t) -> uint8_t {
    if crsfFrameDone {
        crsfFrameDone = 0 as libc::c_int != 0;
        if crsfFrame.frame.type_0 as libc::c_int ==
               CRSF_FRAMETYPE_RC_CHANNELS_PACKED as libc::c_int {
            // CRC includes type and payload of each frame
            let crc: uint8_t = crsfFrameCRC();
            if crc as libc::c_int !=
                   crsfFrame.frame.payload[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE
                                               as libc::c_int as usize] as
                       libc::c_int {
                return RX_FRAME_PENDING as libc::c_int as uint8_t
            }
            // unpack the RC channels
            let rcChannels: *const crsfPayloadRcChannelsPacked_t =
                &mut crsfFrame.frame.payload as *mut [uint8_t; 59] as
                    *mut crsfPayloadRcChannelsPacked_t;
            crsfChannelData[0 as libc::c_int as usize] =
                (*rcChannels).chan0();
            crsfChannelData[1 as libc::c_int as usize] =
                (*rcChannels).chan1();
            crsfChannelData[2 as libc::c_int as usize] =
                (*rcChannels).chan2();
            crsfChannelData[3 as libc::c_int as usize] =
                (*rcChannels).chan3();
            crsfChannelData[4 as libc::c_int as usize] =
                (*rcChannels).chan4();
            crsfChannelData[5 as libc::c_int as usize] =
                (*rcChannels).chan5();
            crsfChannelData[6 as libc::c_int as usize] =
                (*rcChannels).chan6();
            crsfChannelData[7 as libc::c_int as usize] =
                (*rcChannels).chan7();
            crsfChannelData[8 as libc::c_int as usize] =
                (*rcChannels).chan8();
            crsfChannelData[9 as libc::c_int as usize] =
                (*rcChannels).chan9();
            crsfChannelData[10 as libc::c_int as usize] =
                (*rcChannels).chan10();
            crsfChannelData[11 as libc::c_int as usize] =
                (*rcChannels).chan11();
            crsfChannelData[12 as libc::c_int as usize] =
                (*rcChannels).chan12();
            crsfChannelData[13 as libc::c_int as usize] =
                (*rcChannels).chan13();
            crsfChannelData[14 as libc::c_int as usize] =
                (*rcChannels).chan14();
            crsfChannelData[15 as libc::c_int as usize] =
                (*rcChannels).chan15();
            return RX_FRAME_COMPLETE as libc::c_int as uint8_t
        }
    }
    return RX_FRAME_PENDING as libc::c_int as uint8_t;
}
unsafe extern "C" fn crsfReadRawRC(mut rxRuntimeConfig:
                                       *const rxRuntimeConfig_t,
                                   mut chan: uint8_t) -> uint16_t {
    /* conversion from RC value to PWM
     *       RC     PWM
     * min  172 ->  988us
     * mid  992 -> 1500us
     * max 1811 -> 2012us
     * scale factor = (2012-988) / (1811-172) = 0.62477120195241
     * offset = 988 - 172 * 0.62477120195241 = 880.53935326418548
     */
    return (0.62477120195241f32 *
                crsfChannelData[chan as usize] as libc::c_float +
                881 as libc::c_int as libc::c_float) as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn crsfRxWriteTelemetryData(mut data:
                                                      *const libc::c_void,
                                                  mut len: libc::c_int) {
    len =
        ({
             let mut _a: libc::c_int = len;
             let mut _b: libc::c_int =
                 ::core::mem::size_of::<[uint8_t; 64]>() as libc::c_ulong as
                     libc::c_int;
             if _a < _b { _a } else { _b }
         });
    memcpy(telemetryBuf.as_mut_ptr() as *mut libc::c_void, data,
           len as libc::c_ulong);
    telemetryBufLen = len as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn crsfRxSendTelemetryData() {
    // if there is telemetry data to write
    if telemetryBufLen as libc::c_int > 0 as libc::c_int {
        serialWriteBuf(serialPort, telemetryBuf.as_mut_ptr(),
                       telemetryBufLen as libc::c_int);
        telemetryBufLen = 0 as libc::c_int as uint8_t
        // reset telemetry buffer
    }; // !!TODO this needs checking
}
#[no_mangle]
pub unsafe extern "C" fn crsfRxInit(mut rxConfig: *const rxConfig_t,
                                    mut rxRuntimeConfig:
                                        *mut rxRuntimeConfig_t) -> bool {
    let mut ii: libc::c_int = 0 as libc::c_int;
    while ii < 16 as libc::c_int {
        crsfChannelData[ii as usize] =
            (16 as libc::c_int * (*rxConfig).midrc as libc::c_int /
                 10 as libc::c_int - 1408 as libc::c_int) as uint32_t;
        ii += 1
    }
    (*rxRuntimeConfig).channelCount = 16 as libc::c_int as uint8_t;
    (*rxRuntimeConfig).rxRefreshRate = 6667 as libc::c_int as uint16_t;
    (*rxRuntimeConfig).rcReadRawFn =
        Some(crsfReadRawRC as
                 unsafe extern "C" fn(_: *const rxRuntimeConfig_t, _: uint8_t)
                     -> uint16_t);
    (*rxRuntimeConfig).rcFrameStatusFn =
        Some(crsfFrameStatus as
                 unsafe extern "C" fn(_: *mut rxRuntimeConfig_t) -> uint8_t);
    let mut portConfig: *const serialPortConfig_t =
        findSerialPortConfig(FUNCTION_RX_SERIAL);
    if portConfig.is_null() { return 0 as libc::c_int != 0 }
    serialPort =
        openSerialPort((*portConfig).identifier, FUNCTION_RX_SERIAL,
                       Some(crsfDataReceive as
                                unsafe extern "C" fn(_: uint16_t,
                                                     _: *mut libc::c_void)
                                    -> ()), 0 as *mut libc::c_void,
                       420000 as libc::c_int as uint32_t, MODE_RXTX,
                       (SERIAL_STOPBITS_1 as libc::c_int |
                            SERIAL_PARITY_NO as libc::c_int |
                            (if (*rxConfig).serialrx_inverted as libc::c_int
                                    != 0 {
                                 SERIAL_INVERTED as libc::c_int
                             } else { 0 as libc::c_int })) as portOptions_e);
    return !serialPort.is_null();
}
#[no_mangle]
pub unsafe extern "C" fn crsfRxIsActive() -> bool {
    return !serialPort.is_null();
}
