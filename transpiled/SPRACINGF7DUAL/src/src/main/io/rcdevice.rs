use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn crc8_dvb_s2(crc: uint8_t, a: libc::c_uchar) -> uint8_t;
    #[no_mangle]
    fn crc8_dvb_s2_sbuf_append(dst: *mut sbuf_s, start: *mut uint8_t);
    #[no_mangle]
    fn sbufWriteU8(dst: *mut sbuf_t, val: uint8_t);
    #[no_mangle]
    fn sbufWriteData(dst: *mut sbuf_t, data: *const libc::c_void,
                     len: libc::c_int);
    #[no_mangle]
    fn sbufBytesRemaining(buf: *mut sbuf_t) -> libc::c_int;
    #[no_mangle]
    fn sbufPtr(buf: *mut sbuf_t) -> *mut uint8_t;
    #[no_mangle]
    fn sbufSwitchToReader(buf: *mut sbuf_t, base: *mut uint8_t);
    #[no_mangle]
    fn millis() -> timeMs_t;
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
    static mut rcdeviceConfig_System: rcdeviceConfig_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sbuf_s {
    pub ptr: *mut uint8_t,
    pub end: *mut uint8_t,
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
// simple buffer-based serializer/deserializer without implicit size check
pub type sbuf_t = sbuf_s;
pub type timeMs_t = uint32_t;
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
// data pointer must be first (sbuf_t* is equivalent to uint8_t **)
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
pub type serialPortConfig_t = serialPortConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rcdeviceConfig_s {
    pub initDeviceAttempts: uint8_t,
    pub initDeviceAttemptInterval: timeMs_t,
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
pub type rcdeviceConfig_t = rcdeviceConfig_s;
pub type C2RustUnnamed = libc::c_uint;
pub const RCDEVICE_PROTOCOL_5KEY_SIMULATION_DOWN: C2RustUnnamed = 5;
pub const RCDEVICE_PROTOCOL_5KEY_SIMULATION_UP: C2RustUnnamed = 4;
pub const RCDEVICE_PROTOCOL_5KEY_SIMULATION_RIGHT: C2RustUnnamed = 3;
pub const RCDEVICE_PROTOCOL_5KEY_SIMULATION_LEFT: C2RustUnnamed = 2;
pub const RCDEVICE_PROTOCOL_5KEY_SIMULATION_SET: C2RustUnnamed = 1;
pub const RCDEVICE_PROTOCOL_5KEY_SIMULATION_NONE: C2RustUnnamed = 0;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const RCDEVICE_PROTOCOL_5KEY_CONNECTION_CLOSE: C2RustUnnamed_0 = 2;
pub const RCDEVICE_PROTOCOL_5KEY_CONNECTION_OPEN: C2RustUnnamed_0 = 1;
pub type rcdevice_protocol_version_e = libc::c_uint;
pub const RCDEVICE_PROTOCOL_UNKNOWN: rcdevice_protocol_version_e = 2;
pub const RCDEVICE_PROTOCOL_VERSION_1_0: rcdevice_protocol_version_e = 1;
pub const RCDEVICE_PROTOCOL_RCSPLIT_VERSION: rcdevice_protocol_version_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct runcamDeviceInfo_s {
    pub protocolVersion: rcdevice_protocol_version_e,
    pub features: uint16_t,
}
// end of Runcam Device definition
pub type runcamDeviceInfo_t = runcamDeviceInfo_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct runcamDevice_s {
    pub serialPort: *mut serialPort_t,
    pub buffer: [uint8_t; 64],
    pub info: runcamDeviceInfo_t,
    pub isReady: bool,
}
pub type runcamDevice_t = runcamDevice_s;
pub type rcdeviceResponseStatus_e = libc::c_uint;
pub const RCDEVICE_RESP_TIMEOUT: rcdeviceResponseStatus_e = 2;
pub const RCDEVICE_RESP_INCORRECT_CRC: rcdeviceResponseStatus_e = 1;
pub const RCDEVICE_RESP_SUCCESS: rcdeviceResponseStatus_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rcdeviceResponseParseContext_s {
    pub command: uint8_t,
    pub expectedRespLen: uint8_t,
    pub recvRespLen: uint8_t,
    pub recvBuf: *mut uint8_t,
    pub timeout: timeMs_t,
    pub timeoutTimestamp: timeMs_t,
    pub parserFunc: rcdeviceRespParseFunc,
    pub device: *mut runcamDevice_t,
    pub paramData: [uint8_t; 62],
    pub paramDataLen: uint8_t,
    pub protocolVer: uint8_t,
    pub maxRetryTimes: libc::c_int,
    pub userInfo: *mut libc::c_void,
    pub result: rcdeviceResponseStatus_e,
}
pub type rcdeviceRespParseFunc
    =
    Option<unsafe extern "C" fn(_: *mut rcdeviceResponseParseContext_t)
               -> ()>;
pub type rcdeviceResponseParseContext_t = rcdeviceResponseParseContext_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rcdeviceWaitingResponseQueue {
    pub headPos: uint8_t,
    pub tailPos: uint8_t,
    pub itemCount: uint8_t,
    pub buffer: [rcdeviceResponseParseContext_t; 5],
    pub parseFunc: rcdeviceRespParseFunc,
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
pub type runcamDeviceExpectedResponseLength_t
    =
    runcamDeviceExpectedResponseLength_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct runcamDeviceExpectedResponseLength_s {
    pub command: uint8_t,
    pub reponseLength: uint8_t,
}
#[inline]
unsafe extern "C" fn rcdeviceConfig() -> *const rcdeviceConfig_t {
    return &mut rcdeviceConfig_System;
}
static mut expectedResponsesLength: [runcamDeviceExpectedResponseLength_t; 4]
       =
    [{
         let mut init =
             runcamDeviceExpectedResponseLength_s{command:
                                                      0 as libc::c_int as
                                                          uint8_t,
                                                  reponseLength:
                                                      5 as libc::c_int as
                                                          uint8_t,};
         init
     },
     {
         let mut init =
             runcamDeviceExpectedResponseLength_s{command:
                                                      0x2 as libc::c_int as
                                                          uint8_t,
                                                  reponseLength:
                                                      2 as libc::c_int as
                                                          uint8_t,};
         init
     },
     {
         let mut init =
             runcamDeviceExpectedResponseLength_s{command:
                                                      0x3 as libc::c_int as
                                                          uint8_t,
                                                  reponseLength:
                                                      2 as libc::c_int as
                                                          uint8_t,};
         init
     },
     {
         let mut init =
             runcamDeviceExpectedResponseLength_s{command:
                                                      0x4 as libc::c_int as
                                                          uint8_t,
                                                  reponseLength:
                                                      3 as libc::c_int as
                                                          uint8_t,};
         init
     }];
#[no_mangle]
pub static mut watingResponseQueue: rcdeviceWaitingResponseQueue =
    rcdeviceWaitingResponseQueue{headPos: 0,
                                 tailPos: 0,
                                 itemCount: 0,
                                 buffer:
                                     [rcdeviceResponseParseContext_t{command:
                                                                         0,
                                                                     expectedRespLen:
                                                                         0,
                                                                     recvRespLen:
                                                                         0,
                                                                     recvBuf:
                                                                         0 as
                                                                             *const uint8_t
                                                                             as
                                                                             *mut uint8_t,
                                                                     timeout:
                                                                         0,
                                                                     timeoutTimestamp:
                                                                         0,
                                                                     parserFunc:
                                                                         None,
                                                                     device:
                                                                         0 as
                                                                             *const runcamDevice_t
                                                                             as
                                                                             *mut runcamDevice_t,
                                                                     paramData:
                                                                         [0;
                                                                             62],
                                                                     paramDataLen:
                                                                         0,
                                                                     protocolVer:
                                                                         0,
                                                                     maxRetryTimes:
                                                                         0,
                                                                     userInfo:
                                                                         0 as
                                                                             *const libc::c_void
                                                                             as
                                                                             *mut libc::c_void,
                                                                     result:
                                                                         RCDEVICE_RESP_SUCCESS,};
                                         5],
                                 parseFunc: None,};
static mut recvBuf: [uint8_t; 64] = [0; 64];
// all the response contexts using same recv buffer
unsafe extern "C" fn runcamDeviceGetRespLen(mut command: uint8_t) -> uint8_t {
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while (i as libc::c_ulong) <
              (::core::mem::size_of::<[runcamDeviceExpectedResponseLength_t; 4]>()
                   as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<runcamDeviceExpectedResponseLength_t>()
                                                   as libc::c_ulong) {
        if expectedResponsesLength[i as usize].command as libc::c_int ==
               command as libc::c_int {
            return expectedResponsesLength[i as usize].reponseLength
        }
        i = i.wrapping_add(1)
    }
    return 0 as libc::c_int as uint8_t;
}
unsafe extern "C" fn rcdeviceRespCtxQueuePushRespCtx(mut queue:
                                                         *mut rcdeviceWaitingResponseQueue,
                                                     mut respCtx:
                                                         *mut rcdeviceResponseParseContext_t)
 -> bool {
    if queue.is_null() ||
           (*queue).itemCount as libc::c_int + 1 as libc::c_int >
               5 as libc::c_int {
        return 0 as libc::c_int != 0
    }
    (*queue).buffer[(*queue).tailPos as usize] = *respCtx;
    let mut newTailPos: libc::c_int =
        (*queue).tailPos as libc::c_int + 1 as libc::c_int;
    if newTailPos >= 5 as libc::c_int { newTailPos = 0 as libc::c_int }
    (*queue).itemCount =
        ((*queue).itemCount as libc::c_int + 1 as libc::c_int) as uint8_t;
    (*queue).tailPos = newTailPos as uint8_t;
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn rcdeviceRespCtxQueuePeekFront(mut queue:
                                                       *mut rcdeviceWaitingResponseQueue)
 -> *mut rcdeviceResponseParseContext_t {
    if queue.is_null() ||
           (*queue).itemCount as libc::c_int == 0 as libc::c_int {
        return 0 as *mut rcdeviceResponseParseContext_t
    }
    let mut ctx: *mut rcdeviceResponseParseContext_t =
        &mut *(*queue).buffer.as_mut_ptr().offset((*queue).headPos as isize)
            as *mut rcdeviceResponseParseContext_t;
    return ctx;
}
unsafe extern "C" fn rcdeviceRespCtxQueueShift(mut queue:
                                                   *mut rcdeviceWaitingResponseQueue)
 -> *mut rcdeviceResponseParseContext_t {
    if queue.is_null() ||
           (*queue).itemCount as libc::c_int == 0 as libc::c_int {
        return 0 as *mut rcdeviceResponseParseContext_t
    }
    let mut ctx: *mut rcdeviceResponseParseContext_t =
        &mut *(*queue).buffer.as_mut_ptr().offset((*queue).headPos as isize)
            as *mut rcdeviceResponseParseContext_t;
    let mut newHeadPos: libc::c_int =
        (*queue).headPos as libc::c_int + 1 as libc::c_int;
    if newHeadPos >= 5 as libc::c_int { newHeadPos = 0 as libc::c_int }
    (*queue).itemCount =
        ((*queue).itemCount as libc::c_int - 1 as libc::c_int) as uint8_t;
    (*queue).headPos = newHeadPos as uint8_t;
    return ctx;
}
// every time send packet to device, and want to get something from device,
// it'd better call the method to clear the rx buffer before the packet send,
// else may be the useless data in rx buffer will cause the response decoding
// failed.
unsafe extern "C" fn runcamDeviceFlushRxBuffer(mut device:
                                                   *mut runcamDevice_t) {
    while serialRxBytesWaiting((*device).serialPort) >
              0 as libc::c_int as libc::c_uint {
        serialRead((*device).serialPort);
    };
}
// a common way to send packet to device
unsafe extern "C" fn runcamDeviceSendPacket(mut device: *mut runcamDevice_t,
                                            mut command: uint8_t,
                                            mut paramData: *mut uint8_t,
                                            mut paramDataLen: libc::c_int) {
    // is this device open?
    if (*device).serialPort.is_null() { return }
    let mut buf: sbuf_t =
        sbuf_t{ptr: 0 as *mut uint8_t, end: 0 as *mut uint8_t,};
    // prepare pointer
    buf.ptr = (*device).buffer.as_mut_ptr();
    buf.end =
        &mut *(*device).buffer.as_mut_ptr().offset((::core::mem::size_of::<[uint8_t; 64]>()
                                                        as
                                                        libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                                                        as
                                                                                        libc::c_ulong)
                                                       as isize) as
            *mut uint8_t;
    sbufWriteU8(&mut buf, 0xcc as libc::c_int as uint8_t);
    sbufWriteU8(&mut buf, command);
    if !paramData.is_null() {
        sbufWriteData(&mut buf, paramData as *const libc::c_void,
                      paramDataLen);
    }
    // add crc over (all) data
    crc8_dvb_s2_sbuf_append(&mut buf, (*device).buffer.as_mut_ptr());
    // switch to reader
    sbufSwitchToReader(&mut buf, (*device).buffer.as_mut_ptr());
    // send data if possible
    serialWriteBuf((*device).serialPort, sbufPtr(&mut buf),
                   sbufBytesRemaining(&mut buf));
}
// a common way to send a packet to device, and get response from the device.
unsafe extern "C" fn runcamDeviceSendRequestAndWaitingResp(mut device:
                                                               *mut runcamDevice_t,
                                                           mut commandID:
                                                               uint8_t,
                                                           mut paramData:
                                                               *mut uint8_t,
                                                           mut paramDataLen:
                                                               uint8_t,
                                                           mut tiemout:
                                                               timeMs_t,
                                                           mut maxRetryTimes:
                                                               libc::c_int,
                                                           mut userInfo:
                                                               *mut libc::c_void,
                                                           mut parseFunc:
                                                               rcdeviceRespParseFunc) {
    runcamDeviceFlushRxBuffer(device);
    let mut responseCtx: rcdeviceResponseParseContext_t =
        rcdeviceResponseParseContext_t{command: 0,
                                       expectedRespLen: 0,
                                       recvRespLen: 0,
                                       recvBuf:
                                           0 as *const uint8_t as
                                               *mut uint8_t,
                                       timeout: 0,
                                       timeoutTimestamp: 0,
                                       parserFunc: None,
                                       device:
                                           0 as *const runcamDevice_t as
                                               *mut runcamDevice_t,
                                       paramData: [0; 62],
                                       paramDataLen: 0,
                                       protocolVer: 0,
                                       maxRetryTimes: 0,
                                       userInfo:
                                           0 as *const libc::c_void as
                                               *mut libc::c_void,
                                       result: RCDEVICE_RESP_SUCCESS,};
    memset(&mut responseCtx as *mut rcdeviceResponseParseContext_t as
               *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<rcdeviceResponseParseContext_t>() as
               libc::c_ulong);
    responseCtx.recvBuf = recvBuf.as_mut_ptr();
    responseCtx.command = commandID;
    responseCtx.maxRetryTimes = maxRetryTimes;
    responseCtx.expectedRespLen = runcamDeviceGetRespLen(commandID);
    responseCtx.timeout = tiemout;
    responseCtx.timeoutTimestamp = millis().wrapping_add(tiemout);
    responseCtx.parserFunc = parseFunc;
    responseCtx.device = device;
    responseCtx.protocolVer =
        RCDEVICE_PROTOCOL_VERSION_1_0 as libc::c_int as uint8_t;
    memcpy(responseCtx.paramData.as_mut_ptr() as *mut libc::c_void,
           paramData as *const libc::c_void, paramDataLen as libc::c_ulong);
    responseCtx.paramDataLen = paramDataLen;
    responseCtx.userInfo = userInfo;
    rcdeviceRespCtxQueuePushRespCtx(&mut watingResponseQueue,
                                    &mut responseCtx);
    // send packet
    runcamDeviceSendPacket(device, commandID, paramData,
                           paramDataLen as libc::c_int);
}
unsafe extern "C" fn runcamDeviceParseV2DeviceInfo(mut ctx:
                                                       *mut rcdeviceResponseParseContext_t) {
    if (*ctx).result as libc::c_uint !=
           RCDEVICE_RESP_SUCCESS as libc::c_int as libc::c_uint {
        (*(*ctx).device).isReady = 0 as libc::c_int != 0;
        return
    }
    let mut device: *mut runcamDevice_t = (*ctx).device;
    (*device).info.protocolVersion =
        *(*ctx).recvBuf.offset(1 as libc::c_int as isize) as
            rcdevice_protocol_version_e;
    let mut featureLowBits: uint8_t =
        *(*ctx).recvBuf.offset(2 as libc::c_int as isize);
    let mut featureHighBits: uint8_t =
        *(*ctx).recvBuf.offset(3 as libc::c_int as isize);
    (*device).info.features =
        ((featureHighBits as libc::c_int) << 8 as libc::c_int |
             featureLowBits as libc::c_int) as uint16_t;
    (*device).isReady = 1 as libc::c_int != 0;
}
// get the device info(firmware version, protocol version and features, see the
// definition of runcamDeviceInfo_t to know more)
unsafe extern "C" fn runcamDeviceGetDeviceInfo(mut device:
                                                   *mut runcamDevice_t) {
    runcamDeviceSendRequestAndWaitingResp(device, 0 as libc::c_int as uint8_t,
                                          0 as *mut uint8_t,
                                          0 as libc::c_int as uint8_t,
                                          (*rcdeviceConfig()).initDeviceAttemptInterval,
                                          (*rcdeviceConfig()).initDeviceAttempts
                                              as libc::c_int,
                                          0 as *mut libc::c_void,
                                          Some(runcamDeviceParseV2DeviceInfo
                                                   as
                                                   unsafe extern "C" fn(_:
                                                                            *mut rcdeviceResponseParseContext_t)
                                                       -> ()));
}
// init the runcam device, it'll search the UART port with FUNCTION_RCDEVICE id
// this function will delay 400ms in the first loop to wait the device prepared,
// as we know, there are has some camera need about 200~400ms to initialization,
// and then we can send/receive from it.
#[no_mangle]
pub unsafe extern "C" fn runcamDeviceInit(mut device: *mut runcamDevice_t) {
    (*device).isReady = 0 as libc::c_int != 0;
    let mut portID: serialPortFunction_e = FUNCTION_RCDEVICE;
    let mut portConfig: *mut serialPortConfig_t =
        findSerialPortConfig(portID);
    if !portConfig.is_null() {
        (*device).serialPort =
            openSerialPort((*portConfig).identifier, portID, None,
                           0 as *mut libc::c_void,
                           115200 as libc::c_int as uint32_t, MODE_RXTX,
                           SERIAL_NOT_INVERTED);
        if !(*device).serialPort.is_null() {
            // send RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO to device to retrive
            // device info, e.g protocol version, supported features
            runcamDeviceGetDeviceInfo(device);
        }
    };
}
// camera button simulation
#[no_mangle]
pub unsafe extern "C" fn runcamDeviceSimulateCameraButton(mut device:
                                                              *mut runcamDevice_t,
                                                          mut operation:
                                                              uint8_t)
 -> bool {
    if (*device).info.protocolVersion as libc::c_uint ==
           RCDEVICE_PROTOCOL_VERSION_1_0 as libc::c_int as libc::c_uint {
        runcamDeviceSendPacket(device, 0x1 as libc::c_int as uint8_t,
                               &mut operation,
                               ::core::mem::size_of::<uint8_t>() as
                                   libc::c_ulong as libc::c_int);
    } else { return 0 as libc::c_int != 0 }
    return 1 as libc::c_int != 0;
}
// 5 key osd cable simulation
// every time start to control the OSD menu of camera, must call this method to
// camera
#[no_mangle]
pub unsafe extern "C" fn runcamDeviceOpen5KeyOSDCableConnection(mut device:
                                                                    *mut runcamDevice_t,
                                                                mut parseFunc:
                                                                    rcdeviceRespParseFunc) {
    let mut operation: uint8_t =
        RCDEVICE_PROTOCOL_5KEY_CONNECTION_OPEN as libc::c_int as uint8_t;
    runcamDeviceSendRequestAndWaitingResp(device,
                                          0x4 as libc::c_int as uint8_t,
                                          &mut operation,
                                          ::core::mem::size_of::<uint8_t>() as
                                              libc::c_ulong as uint8_t,
                                          200 as libc::c_int as timeMs_t,
                                          0 as libc::c_int,
                                          0 as *mut libc::c_void, parseFunc);
}
// when the control was stop, must call this method to the camera to disconnect
// with camera.
#[no_mangle]
pub unsafe extern "C" fn runcamDeviceClose5KeyOSDCableConnection(mut device:
                                                                     *mut runcamDevice_t,
                                                                 mut parseFunc:
                                                                     rcdeviceRespParseFunc) {
    let mut operation: uint8_t =
        RCDEVICE_PROTOCOL_5KEY_CONNECTION_CLOSE as libc::c_int as uint8_t;
    runcamDeviceSendRequestAndWaitingResp(device,
                                          0x4 as libc::c_int as uint8_t,
                                          &mut operation,
                                          ::core::mem::size_of::<uint8_t>() as
                                              libc::c_ulong as uint8_t,
                                          200 as libc::c_int as timeMs_t,
                                          0 as libc::c_int,
                                          0 as *mut libc::c_void, parseFunc);
}
// simulate button press event of 5 key osd cable with special button
#[no_mangle]
pub unsafe extern "C" fn runcamDeviceSimulate5KeyOSDCableButtonPress(mut device:
                                                                         *mut runcamDevice_t,
                                                                     mut operation:
                                                                         uint8_t,
                                                                     mut parseFunc:
                                                                         rcdeviceRespParseFunc) {
    if operation as libc::c_int ==
           RCDEVICE_PROTOCOL_5KEY_SIMULATION_NONE as libc::c_int {
        return
    }
    runcamDeviceSendRequestAndWaitingResp(device,
                                          0x2 as libc::c_int as uint8_t,
                                          &mut operation,
                                          ::core::mem::size_of::<uint8_t>() as
                                              libc::c_ulong as uint8_t,
                                          200 as libc::c_int as timeMs_t,
                                          0 as libc::c_int,
                                          0 as *mut libc::c_void, parseFunc);
}
// simulate button release event of 5 key osd cable
#[no_mangle]
pub unsafe extern "C" fn runcamDeviceSimulate5KeyOSDCableButtonRelease(mut device:
                                                                           *mut runcamDevice_t,
                                                                       mut parseFunc:
                                                                           rcdeviceRespParseFunc) {
    runcamDeviceSendRequestAndWaitingResp(device,
                                          0x3 as libc::c_int as uint8_t,
                                          0 as *mut uint8_t,
                                          0 as libc::c_int as uint8_t,
                                          200 as libc::c_int as timeMs_t,
                                          0 as libc::c_int,
                                          0 as *mut libc::c_void, parseFunc);
}
unsafe extern "C" fn getWaitingResponse(mut currentTimeMs: timeMs_t)
 -> *mut rcdeviceResponseParseContext_t {
    let mut respCtx: *mut rcdeviceResponseParseContext_t =
        rcdeviceRespCtxQueuePeekFront(&mut watingResponseQueue);
    while !respCtx.is_null() &&
              (*respCtx).timeoutTimestamp != 0 as libc::c_int as libc::c_uint
              && currentTimeMs > (*respCtx).timeoutTimestamp {
        if (*respCtx).maxRetryTimes > 0 as libc::c_int {
            runcamDeviceSendPacket((*respCtx).device, (*respCtx).command,
                                   (*respCtx).paramData.as_mut_ptr(),
                                   (*respCtx).paramDataLen as libc::c_int);
            (*respCtx).timeoutTimestamp =
                currentTimeMs.wrapping_add((*respCtx).timeout);
            (*respCtx).maxRetryTimes -= 1 as libc::c_int;
            respCtx = 0 as *mut rcdeviceResponseParseContext_t;
            break ;
        } else {
            (*respCtx).result = RCDEVICE_RESP_TIMEOUT;
            if (*respCtx).parserFunc.is_some() {
                (*respCtx).parserFunc.expect("non-null function pointer")(respCtx);
            }
            // dequeue and get next waiting response context
            rcdeviceRespCtxQueueShift(&mut watingResponseQueue);
            respCtx = rcdeviceRespCtxQueuePeekFront(&mut watingResponseQueue)
        }
    }
    return respCtx;
}
#[no_mangle]
pub unsafe extern "C" fn rcdeviceReceive(mut currentTimeUs: timeUs_t) {
    let mut respCtx: *mut rcdeviceResponseParseContext_t =
        0 as *mut rcdeviceResponseParseContext_t;
    loop  {
        respCtx = getWaitingResponse(millis());
        if !(!respCtx.is_null() &&
                 serialRxBytesWaiting((*(*respCtx).device).serialPort) != 0) {
            break ;
        }
        let c: uint8_t = serialRead((*(*respCtx).device).serialPort);
        *(*respCtx).recvBuf.offset((*respCtx).recvRespLen as isize) = c;
        (*respCtx).recvRespLen =
            ((*respCtx).recvRespLen as libc::c_int + 1 as libc::c_int) as
                uint8_t;
        // if data received done, trigger callback to parse response data, and update rcdevice state
        if (*respCtx).recvRespLen as libc::c_int ==
               (*respCtx).expectedRespLen as libc::c_int {
            // verify the crc value
            if (*respCtx).protocolVer as libc::c_int ==
                   RCDEVICE_PROTOCOL_VERSION_1_0 as libc::c_int {
                let mut crc: uint8_t = 0 as libc::c_int as uint8_t;
                let mut i: libc::c_int = 0 as libc::c_int;
                while i < (*respCtx).recvRespLen as libc::c_int {
                    crc =
                        crc8_dvb_s2(crc,
                                    *(*respCtx).recvBuf.offset(i as isize));
                    i += 1
                }
                (*respCtx).result =
                    if crc as libc::c_int == 0 as libc::c_int {
                        RCDEVICE_RESP_SUCCESS as libc::c_int
                    } else { RCDEVICE_RESP_INCORRECT_CRC as libc::c_int } as
                        rcdeviceResponseStatus_e
            }
            if (*respCtx).parserFunc.is_some() {
                (*respCtx).parserFunc.expect("non-null function pointer")(respCtx);
            }
            // dequeue current response context
            rcdeviceRespCtxQueueShift(&mut watingResponseQueue);
        }
    };
}
