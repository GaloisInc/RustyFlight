use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn sbufBytesRemaining(buf: *mut sbuf_t) -> libc::c_int;
    #[no_mangle]
    fn sbufPtr(buf: *mut sbuf_t) -> *mut uint8_t;
    #[no_mangle]
    fn sbufSwitchToReader(buf: *mut sbuf_t, base: *mut uint8_t);
    #[no_mangle]
    fn crc8_dvb_s2(crc: uint8_t, a: libc::c_uchar) -> uint8_t;
    #[no_mangle]
    fn crc8_dvb_s2_update(crc: uint8_t, data: *const libc::c_void,
                          length: uint32_t) -> uint8_t;
    #[no_mangle]
    fn systemResetToBootloader();
    #[no_mangle]
    fn serialRxBytesWaiting(instance: *const serialPort_t) -> uint32_t;
    #[no_mangle]
    fn serialTxBytesFree(instance: *const serialPort_t) -> uint32_t;
    #[no_mangle]
    fn serialWriteBuf(instance: *mut serialPort_t, data: *const uint8_t,
                      count: libc::c_int);
    #[no_mangle]
    fn serialRead(instance: *mut serialPort_t) -> uint8_t;
    #[no_mangle]
    fn isSerialTransmitBufferEmpty(instance: *const serialPort_t) -> bool;
    #[no_mangle]
    fn serialBeginWrite(instance: *mut serialPort_t);
    #[no_mangle]
    fn serialEndWrite(instance: *mut serialPort_t);
    #[no_mangle]
    static baudRates: [uint32_t; 0];
    #[no_mangle]
    static mut serialConfig_System: serialConfig_t;
    #[no_mangle]
    fn findSerialPortConfig(function: serialPortFunction_e)
     -> *mut serialPortConfig_t;
    #[no_mangle]
    fn findNextSerialPortConfig(function: serialPortFunction_e)
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
    #[no_mangle]
    fn closeSerialPort(serialPort: *mut serialPort_t);
    #[no_mangle]
    fn waitForSerialPortToFinishTransmitting(serialPort: *mut serialPort_t);
    #[no_mangle]
    fn millis() -> timeMs_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type uint_fast16_t = libc::c_ulong;
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
pub type mspVersion_e = libc::c_uint;
pub const MSP_VERSION_COUNT: mspVersion_e = 3;
pub const MSP_V2_NATIVE: mspVersion_e = 2;
pub const MSP_V2_OVER_V1: mspVersion_e = 1;
pub const MSP_V1: mspVersion_e = 0;
pub type mspResult_e = libc::c_int;
pub const MSP_RESULT_CMD_UNKNOWN: mspResult_e = -2;
pub const MSP_RESULT_NO_REPLY: mspResult_e = 0;
pub const MSP_RESULT_ERROR: mspResult_e = -1;
pub const MSP_RESULT_ACK: mspResult_e = 1;
pub type mspDirection_e = libc::c_uint;
pub const MSP_DIRECTION_REQUEST: mspDirection_e = 1;
pub const MSP_DIRECTION_REPLY: mspDirection_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mspPacket_s {
    pub buf: sbuf_t,
    pub cmd: int16_t,
    pub flags: uint8_t,
    pub result: int16_t,
    pub direction: uint8_t,
}
pub type mspPacket_t = mspPacket_s;
// data pointer must be first (sbuf_t* is equivalent to uint8_t **)
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
/*
     * Note on SERIAL_BIDIR_PP
     * With SERIAL_BIDIR_PP, the very first start bit of back-to-back bytes
     * is lost and the first data byte will be lost by a framing error.
     * To ensure the first start bit to be sent, prepend a zero byte (0x00)
     * to actual data bytes.
     */
// disable pulls in BIDIR RX mode
// Define known line control states which may be passed up by underlying serial driver callback
// used by serial drivers to return frames to app
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
pub type serialPort_t = serialPort_s;
pub type mspPostProcessFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut serialPort_s) -> ()>;
// msp post process function, used for gracefully handling reboots, etc.
pub type mspProcessCommandFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut mspPacket_t, _: *mut mspPacket_t,
                                _: *mut mspPostProcessFnPtr) -> mspResult_e>;
pub type mspProcessReplyFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut mspPacket_t) -> ()>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialConfig_s {
    pub portConfigs: [serialPortConfig_t; 2],
    pub serial_update_rate_hz: uint16_t,
    pub reboot_character: uint8_t,
}
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
pub type serialConfig_t = serialConfig_s;
// millisecond time
pub type timeMs_t = uint32_t;
pub type mspState_e = libc::c_uint;
pub const MSP_COMMAND_RECEIVED: mspState_e = 13;
pub const MSP_CHECKSUM_V2_NATIVE: mspState_e = 12;
pub const MSP_PAYLOAD_V2_NATIVE: mspState_e = 11;
pub const MSP_HEADER_V2_NATIVE: mspState_e = 10;
pub const MSP_CHECKSUM_V2_OVER_V1: mspState_e = 9;
pub const MSP_PAYLOAD_V2_OVER_V1: mspState_e = 8;
pub const MSP_HEADER_V2_OVER_V1: mspState_e = 7;
pub const MSP_CHECKSUM_V1: mspState_e = 6;
pub const MSP_PAYLOAD_V1: mspState_e = 5;
pub const MSP_HEADER_V1: mspState_e = 4;
pub const MSP_HEADER_X: mspState_e = 3;
pub const MSP_HEADER_M: mspState_e = 2;
pub const MSP_HEADER_START: mspState_e = 1;
pub const MSP_IDLE: mspState_e = 0;
pub type mspPacketType_e = libc::c_uint;
pub const MSP_PACKET_REPLY: mspPacketType_e = 1;
pub const MSP_PACKET_COMMAND: mspPacketType_e = 0;
pub type mspEvaluateNonMspData_e = libc::c_uint;
pub const MSP_SKIP_NON_MSP_DATA: mspEvaluateNonMspData_e = 1;
pub const MSP_EVALUATE_NON_MSP_DATA: mspEvaluateNonMspData_e = 0;
pub type mspPendingSystemRequest_e = libc::c_uint;
pub const MSP_PENDING_CLI: mspPendingSystemRequest_e = 2;
pub const MSP_PENDING_BOOTLOADER: mspPendingSystemRequest_e = 1;
pub const MSP_PENDING_NONE: mspPendingSystemRequest_e = 0;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct mspHeaderV1_t {
    pub size: uint8_t,
    pub cmd: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct mspHeaderJUMBO_t {
    pub size: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct mspHeaderV2_t {
    pub flags: uint8_t,
    pub cmd: uint16_t,
    pub size: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mspPort_s {
    pub port: *mut serialPort_s,
    pub lastActivityMs: timeMs_t,
    pub pendingRequest: mspPendingSystemRequest_e,
    pub c_state: mspState_e,
    pub packetType: mspPacketType_e,
    pub inBuf: [uint8_t; 192],
    pub cmdMSP: uint16_t,
    pub cmdFlags: uint8_t,
    pub mspVersion: mspVersion_e,
    pub offset: uint_fast16_t,
    pub dataSize: uint_fast16_t,
    pub checksum1: uint8_t,
    pub checksum2: uint8_t,
    pub sharedWithTelemetry: bool,
}
pub type mspPort_t = mspPort_s;
#[inline]
unsafe extern "C" fn serialConfig() -> *const serialConfig_t {
    return &mut serialConfig_System;
}
// null when port unused.
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
static mut mspPorts: [mspPort_t; 3] =
    [mspPort_t{port: 0 as *const serialPort_s as *mut serialPort_s,
               lastActivityMs: 0,
               pendingRequest: MSP_PENDING_NONE,
               c_state: MSP_IDLE,
               packetType: MSP_PACKET_COMMAND,
               inBuf: [0; 192],
               cmdMSP: 0,
               cmdFlags: 0,
               mspVersion: MSP_V1,
               offset: 0,
               dataSize: 0,
               checksum1: 0,
               checksum2: 0,
               sharedWithTelemetry: false,}; 3];
unsafe extern "C" fn resetMspPort(mut mspPortToReset: *mut mspPort_t,
                                  mut serialPort: *mut serialPort_t,
                                  mut sharedWithTelemetry: bool) {
    memset(mspPortToReset as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<mspPort_t>() as libc::c_ulong);
    (*mspPortToReset).port = serialPort;
    (*mspPortToReset).sharedWithTelemetry = sharedWithTelemetry;
}
#[no_mangle]
pub unsafe extern "C" fn mspSerialAllocatePorts() {
    let mut portIndex: uint8_t = 0 as libc::c_int as uint8_t;
    let mut portConfig: *mut serialPortConfig_t =
        findSerialPortConfig(FUNCTION_MSP);
    while !portConfig.is_null() &&
              (portIndex as libc::c_int) < 3 as libc::c_int {
        let mut mspPort: *mut mspPort_t =
            &mut *mspPorts.as_mut_ptr().offset(portIndex as isize) as
                *mut mspPort_t;
        if !(*mspPort).port.is_null() {
            portIndex = portIndex.wrapping_add(1)
        } else {
            let mut serialPort: *mut serialPort_t =
                openSerialPort((*portConfig).identifier, FUNCTION_MSP, None,
                               0 as *mut libc::c_void,
                               *baudRates.as_ptr().offset((*portConfig).msp_baudrateIndex
                                                              as isize),
                               MODE_RXTX, SERIAL_NOT_INVERTED);
            if !serialPort.is_null() {
                let mut sharedWithTelemetry: bool =
                    isSerialPortShared(portConfig,
                                       FUNCTION_MSP as libc::c_int as
                                           uint16_t,
                                       (FUNCTION_TELEMETRY_FRSKY_HUB as
                                            libc::c_int |
                                            FUNCTION_TELEMETRY_LTM as
                                                libc::c_int |
                                            FUNCTION_TELEMETRY_MAVLINK as
                                                libc::c_int |
                                            FUNCTION_TELEMETRY_HOTT as
                                                libc::c_int |
                                            FUNCTION_TELEMETRY_SMARTPORT as
                                                libc::c_int) as
                                           serialPortFunction_e);
                resetMspPort(mspPort, serialPort, sharedWithTelemetry);
                portIndex = portIndex.wrapping_add(1)
            }
            portConfig = findNextSerialPortConfig(FUNCTION_MSP)
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn mspSerialReleasePortIfAllocated(mut serialPort:
                                                             *mut serialPort_t) {
    let mut portIndex: uint8_t = 0 as libc::c_int as uint8_t;
    while (portIndex as libc::c_int) < 3 as libc::c_int {
        let mut candidateMspPort: *mut mspPort_t =
            &mut *mspPorts.as_mut_ptr().offset(portIndex as isize) as
                *mut mspPort_t;
        if (*candidateMspPort).port == serialPort {
            closeSerialPort(serialPort);
            memset(candidateMspPort as *mut libc::c_void, 0 as libc::c_int,
                   ::core::mem::size_of::<mspPort_t>() as libc::c_ulong);
        }
        portIndex = portIndex.wrapping_add(1)
    };
}
unsafe extern "C" fn mspSerialProcessReceivedData(mut mspPort: *mut mspPort_t,
                                                  mut c: uint8_t) -> bool {
    match (*mspPort).c_state as libc::c_uint {
        1 => {
            // Waiting for 'M' (MSPv1 / MSPv2_over_v1) or 'X' (MSPv2 native)
            (*mspPort).offset = 0 as libc::c_int as uint_fast16_t;
            (*mspPort).checksum1 = 0 as libc::c_int as uint8_t;
            (*mspPort).checksum2 = 0 as libc::c_int as uint8_t;
            match c as libc::c_int {
                77 => {
                    (*mspPort).c_state = MSP_HEADER_M;
                    (*mspPort).mspVersion = MSP_V1
                }
                88 => {
                    (*mspPort).c_state = MSP_HEADER_X;
                    (*mspPort).mspVersion = MSP_V2_NATIVE
                }
                _ => { (*mspPort).c_state = MSP_IDLE }
            }
        }
        2 => {
            // Waiting for '<' / '>'
            (*mspPort).c_state = MSP_HEADER_V1;
            match c as libc::c_int {
                60 => { (*mspPort).packetType = MSP_PACKET_COMMAND }
                62 => { (*mspPort).packetType = MSP_PACKET_REPLY }
                _ => { (*mspPort).c_state = MSP_IDLE }
            }
        }
        3 => {
            (*mspPort).c_state = MSP_HEADER_V2_NATIVE;
            match c as libc::c_int {
                60 => { (*mspPort).packetType = MSP_PACKET_COMMAND }
                62 => { (*mspPort).packetType = MSP_PACKET_REPLY }
                _ => { (*mspPort).c_state = MSP_IDLE }
            }
        }
        4 => {
            // Now receive v1 header (size/cmd), this is already checksummable
            let fresh0 = (*mspPort).offset;
            (*mspPort).offset = (*mspPort).offset.wrapping_add(1);
            (*mspPort).inBuf[fresh0 as usize] = c;
            (*mspPort).checksum1 =
                ((*mspPort).checksum1 as libc::c_int ^ c as libc::c_int) as
                    uint8_t;
            if (*mspPort).offset ==
                   ::core::mem::size_of::<mspHeaderV1_t>() as libc::c_ulong {
                let mut hdr: *mut mspHeaderV1_t =
                    &mut *(*mspPort).inBuf.as_mut_ptr().offset(0 as
                                                                   libc::c_int
                                                                   as isize)
                        as *mut uint8_t as *mut mspHeaderV1_t;
                // Check incoming buffer size limit
                if (*hdr).size as libc::c_int > 192 as libc::c_int {
                    (*mspPort).c_state = MSP_IDLE
                } else if (*hdr).cmd as libc::c_int == 255 as libc::c_int {
                    // MSPv1 payload must be big enough to hold V2 header + extra checksum
                    if (*hdr).size as libc::c_ulong >=
                           (::core::mem::size_of::<mspHeaderV2_t>() as
                                libc::c_ulong).wrapping_add(1 as libc::c_int
                                                                as
                                                                libc::c_ulong)
                       {
                        (*mspPort).mspVersion = MSP_V2_OVER_V1;
                        (*mspPort).c_state = MSP_HEADER_V2_OVER_V1
                    } else { (*mspPort).c_state = MSP_IDLE }
                } else {
                    (*mspPort).dataSize = (*hdr).size as uint_fast16_t;
                    (*mspPort).cmdMSP = (*hdr).cmd as uint16_t;
                    (*mspPort).cmdFlags = 0 as libc::c_int as uint8_t;
                    // If no payload - jump to checksum byte
                    (*mspPort).offset =
                        0 as libc::c_int as uint_fast16_t; // re-use buffer
                    (*mspPort).c_state =
                        if (*mspPort).dataSize >
                               0 as libc::c_int as libc::c_ulong {
                            MSP_PAYLOAD_V1 as libc::c_int
                        } else { MSP_CHECKSUM_V1 as libc::c_int } as
                            mspState_e
                }
            }
        }
        5 => {
            let fresh1 = (*mspPort).offset;
            (*mspPort).offset = (*mspPort).offset.wrapping_add(1);
            (*mspPort).inBuf[fresh1 as usize] = c;
            (*mspPort).checksum1 =
                ((*mspPort).checksum1 as libc::c_int ^ c as libc::c_int) as
                    uint8_t;
            if (*mspPort).offset == (*mspPort).dataSize {
                (*mspPort).c_state = MSP_CHECKSUM_V1
            }
        }
        6 => {
            if (*mspPort).checksum1 as libc::c_int == c as libc::c_int {
                (*mspPort).c_state = MSP_COMMAND_RECEIVED
            } else { (*mspPort).c_state = MSP_IDLE }
        }
        7 => {
            // V2 header is part of V1 payload - we need to calculate both checksums now
            let fresh2 = (*mspPort).offset; // re-use buffer
            (*mspPort).offset = (*mspPort).offset.wrapping_add(1);
            (*mspPort).inBuf[fresh2 as usize] = c;
            (*mspPort).checksum1 =
                ((*mspPort).checksum1 as libc::c_int ^ c as libc::c_int) as
                    uint8_t;
            (*mspPort).checksum2 = crc8_dvb_s2((*mspPort).checksum2, c);
            if (*mspPort).offset ==
                   (::core::mem::size_of::<mspHeaderV2_t>() as
                        libc::c_ulong).wrapping_add(::core::mem::size_of::<mspHeaderV1_t>()
                                                        as libc::c_ulong) {
                let mut hdrv2: *mut mspHeaderV2_t =
                    &mut *(*mspPort).inBuf.as_mut_ptr().offset(::core::mem::size_of::<mspHeaderV1_t>()
                                                                   as
                                                                   libc::c_ulong
                                                                   as isize)
                        as *mut uint8_t as *mut mspHeaderV2_t;
                (*mspPort).dataSize = (*hdrv2).size as uint_fast16_t;
                (*mspPort).cmdMSP = (*hdrv2).cmd;
                (*mspPort).cmdFlags = (*hdrv2).flags;
                (*mspPort).offset = 0 as libc::c_int as uint_fast16_t;
                (*mspPort).c_state =
                    if (*mspPort).dataSize > 0 as libc::c_int as libc::c_ulong
                       {
                        MSP_PAYLOAD_V2_OVER_V1 as libc::c_int
                    } else { MSP_CHECKSUM_V2_OVER_V1 as libc::c_int } as
                        mspState_e
            }
        }
        8 => {
            (*mspPort).checksum2 = crc8_dvb_s2((*mspPort).checksum2, c);
            (*mspPort).checksum1 =
                ((*mspPort).checksum1 as libc::c_int ^ c as libc::c_int) as
                    uint8_t;
            let fresh3 = (*mspPort).offset;
            (*mspPort).offset = (*mspPort).offset.wrapping_add(1);
            (*mspPort).inBuf[fresh3 as usize] = c;
            if (*mspPort).offset == (*mspPort).dataSize {
                (*mspPort).c_state = MSP_CHECKSUM_V2_OVER_V1
            }
        }
        9 => {
            (*mspPort).checksum1 =
                ((*mspPort).checksum1 as libc::c_int ^ c as libc::c_int) as
                    uint8_t;
            if (*mspPort).checksum2 as libc::c_int == c as libc::c_int {
                (*mspPort).c_state = MSP_CHECKSUM_V1
                // Checksum 2 correct - verify v1 checksum
            } else { (*mspPort).c_state = MSP_IDLE }
        }
        10 => {
            let fresh4 = (*mspPort).offset; // re-use buffer
            (*mspPort).offset = (*mspPort).offset.wrapping_add(1);
            (*mspPort).inBuf[fresh4 as usize] = c;
            (*mspPort).checksum2 = crc8_dvb_s2((*mspPort).checksum2, c);
            if (*mspPort).offset ==
                   ::core::mem::size_of::<mspHeaderV2_t>() as libc::c_ulong {
                let mut hdrv2_0: *mut mspHeaderV2_t =
                    &mut *(*mspPort).inBuf.as_mut_ptr().offset(0 as
                                                                   libc::c_int
                                                                   as isize)
                        as *mut uint8_t as *mut mspHeaderV2_t;
                (*mspPort).dataSize = (*hdrv2_0).size as uint_fast16_t;
                (*mspPort).cmdMSP = (*hdrv2_0).cmd;
                (*mspPort).cmdFlags = (*hdrv2_0).flags;
                (*mspPort).offset = 0 as libc::c_int as uint_fast16_t;
                (*mspPort).c_state =
                    if (*mspPort).dataSize > 0 as libc::c_int as libc::c_ulong
                       {
                        MSP_PAYLOAD_V2_NATIVE as libc::c_int
                    } else { MSP_CHECKSUM_V2_NATIVE as libc::c_int } as
                        mspState_e
            }
        }
        11 => {
            (*mspPort).checksum2 = crc8_dvb_s2((*mspPort).checksum2, c);
            let fresh5 = (*mspPort).offset;
            (*mspPort).offset = (*mspPort).offset.wrapping_add(1);
            (*mspPort).inBuf[fresh5 as usize] = c;
            if (*mspPort).offset == (*mspPort).dataSize {
                (*mspPort).c_state = MSP_CHECKSUM_V2_NATIVE
            }
        }
        12 => {
            if (*mspPort).checksum2 as libc::c_int == c as libc::c_int {
                (*mspPort).c_state = MSP_COMMAND_RECEIVED
            } else { (*mspPort).c_state = MSP_IDLE }
        }
        0 | _ => {
            // Waiting for '$' character
            if c as libc::c_int == '$' as i32 {
                (*mspPort).c_state = MSP_HEADER_START
            } else { return 0 as libc::c_int != 0 }
        }
    }
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn mspSerialChecksumBuf(mut checksum: uint8_t,
                                          mut data: *const uint8_t,
                                          mut len: libc::c_int) -> uint8_t {
    loop  {
        let fresh6 = len;
        len = len - 1;
        if !(fresh6 > 0 as libc::c_int) { break ; }
        let fresh7 = data;
        data = data.offset(1);
        checksum =
            (checksum as libc::c_int ^ *fresh7 as libc::c_int) as uint8_t
    }
    return checksum;
}
unsafe extern "C" fn mspSerialSendFrame(mut msp: *mut mspPort_t,
                                        mut hdr: *const uint8_t,
                                        mut hdrLen: libc::c_int,
                                        mut data: *const uint8_t,
                                        mut dataLen: libc::c_int,
                                        mut crc: *const uint8_t,
                                        mut crcLen: libc::c_int)
 -> libc::c_int {
    // We are allowed to send out the response if
    //  a) TX buffer is completely empty (we are talking to well-behaving party that follows request-response scheduling;
    //     this allows us to transmit jumbo frames bigger than TX buffer (serialWriteBuf will block, but for jumbo frames we don't care)
    //  b) Response fits into TX buffer
    let totalFrameLength: libc::c_int = hdrLen + dataLen + crcLen;
    if !isSerialTransmitBufferEmpty((*msp).port) &&
           (serialTxBytesFree((*msp).port) as libc::c_int) < totalFrameLength
       {
        return 0 as libc::c_int
    }
    // Transmit frame
    serialBeginWrite((*msp).port);
    serialWriteBuf((*msp).port, hdr, hdrLen);
    serialWriteBuf((*msp).port, data, dataLen);
    serialWriteBuf((*msp).port, crc, crcLen);
    serialEndWrite((*msp).port);
    return totalFrameLength;
}
unsafe extern "C" fn mspSerialEncode(mut msp: *mut mspPort_t,
                                     mut packet: *mut mspPacket_t,
                                     mut mspVersion: mspVersion_e)
 -> libc::c_int {
    static mut mspMagic: [uint8_t; 3] =
        ['M' as i32 as uint8_t, 'M' as i32 as uint8_t, 'X' as i32 as uint8_t];
    let dataLen: libc::c_int = sbufBytesRemaining(&mut (*packet).buf);
    let mut hdrBuf: [uint8_t; 16] =
        ['$' as i32 as uint8_t, mspMagic[mspVersion as usize],
         if (*packet).result as libc::c_int == MSP_RESULT_ERROR as libc::c_int
            {
             '!' as i32
         } else { '>' as i32 } as uint8_t, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0];
    let mut crcBuf: [uint8_t; 2] = [0; 2];
    let mut checksum: uint8_t = 0;
    let mut hdrLen: libc::c_int = 3 as libc::c_int;
    let mut crcLen: libc::c_int = 0 as libc::c_int;
    if mspVersion as libc::c_uint == MSP_V1 as libc::c_int as libc::c_uint {
        let mut hdrV1: *mut mspHeaderV1_t =
            &mut *hdrBuf.as_mut_ptr().offset(hdrLen as isize) as *mut uint8_t
                as *mut mspHeaderV1_t;
        hdrLen =
            (hdrLen as
                 libc::c_ulong).wrapping_add(::core::mem::size_of::<mspHeaderV1_t>()
                                                 as libc::c_ulong) as
                libc::c_int as libc::c_int;
        (*hdrV1).cmd = (*packet).cmd as uint8_t;
        // Add JUMBO-frame header if necessary
        if dataLen >= 255 as libc::c_int {
            let mut hdrJUMBO: *mut mspHeaderJUMBO_t =
                &mut *hdrBuf.as_mut_ptr().offset(hdrLen as isize) as
                    *mut uint8_t as *mut mspHeaderJUMBO_t;
            hdrLen =
                (hdrLen as
                     libc::c_ulong).wrapping_add(::core::mem::size_of::<mspHeaderJUMBO_t>()
                                                     as libc::c_ulong) as
                    libc::c_int as libc::c_int;
            (*hdrV1).size = 255 as libc::c_int as uint8_t;
            (*hdrJUMBO).size = dataLen as uint16_t
        } else { (*hdrV1).size = dataLen as uint8_t }
        // Pre-calculate CRC
        checksum =
            mspSerialChecksumBuf(0 as libc::c_int as uint8_t,
                                 hdrBuf.as_mut_ptr().offset(3 as libc::c_int
                                                                as isize),
                                 hdrLen -
                                     3 as
                                         libc::c_int); // MSPv2 header + data payload + MSPv2 checksum
        checksum =
            mspSerialChecksumBuf(checksum, sbufPtr(&mut (*packet).buf),
                                 dataLen);
        let fresh8 = crcLen;
        crcLen = crcLen + 1;
        crcBuf[fresh8 as usize] = checksum
    } else if mspVersion as libc::c_uint ==
                  MSP_V2_OVER_V1 as libc::c_int as libc::c_uint {
        let mut hdrV1_0: *mut mspHeaderV1_t =
            &mut *hdrBuf.as_mut_ptr().offset(hdrLen as isize) as *mut uint8_t
                as *mut mspHeaderV1_t;
        hdrLen =
            (hdrLen as
                 libc::c_ulong).wrapping_add(::core::mem::size_of::<mspHeaderV1_t>()
                                                 as libc::c_ulong) as
                libc::c_int as libc::c_int;
        let mut hdrV2: *mut mspHeaderV2_t =
            &mut *hdrBuf.as_mut_ptr().offset(hdrLen as isize) as *mut uint8_t
                as *mut mspHeaderV2_t;
        hdrLen =
            (hdrLen as
                 libc::c_ulong).wrapping_add(::core::mem::size_of::<mspHeaderV2_t>()
                                                 as libc::c_ulong) as
                libc::c_int as libc::c_int;
        let v1PayloadSize: libc::c_int =
            (::core::mem::size_of::<mspHeaderV2_t>() as
                 libc::c_ulong).wrapping_add(dataLen as
                                                 libc::c_ulong).wrapping_add(1
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_ulong)
                as libc::c_int;
        (*hdrV1_0).cmd = 255 as libc::c_int as uint8_t;
        // Add JUMBO-frame header if necessary
        if v1PayloadSize >= 255 as libc::c_int {
            let mut hdrJUMBO_0: *mut mspHeaderJUMBO_t =
                &mut *hdrBuf.as_mut_ptr().offset(hdrLen as isize) as
                    *mut uint8_t as *mut mspHeaderJUMBO_t;
            hdrLen =
                (hdrLen as
                     libc::c_ulong).wrapping_add(::core::mem::size_of::<mspHeaderJUMBO_t>()
                                                     as libc::c_ulong) as
                    libc::c_int as libc::c_int;
            (*hdrV1_0).size = 255 as libc::c_int as uint8_t;
            (*hdrJUMBO_0).size = v1PayloadSize as uint16_t
        } else { (*hdrV1_0).size = v1PayloadSize as uint8_t }
        // Fill V2 header
        (*hdrV2).flags = (*packet).flags;
        (*hdrV2).cmd = (*packet).cmd as uint16_t;
        (*hdrV2).size = dataLen as uint16_t;
        // V2 CRC: only V2 header + data payload
        checksum =
            crc8_dvb_s2_update(0 as libc::c_int as uint8_t,
                               hdrV2 as *mut uint8_t as *const libc::c_void,
                               ::core::mem::size_of::<mspHeaderV2_t>() as
                                   libc::c_ulong as uint32_t);
        checksum =
            crc8_dvb_s2_update(checksum,
                               sbufPtr(&mut (*packet).buf) as
                                   *const libc::c_void, dataLen as uint32_t);
        let fresh9 = crcLen;
        crcLen = crcLen + 1;
        crcBuf[fresh9 as usize] = checksum;
        // V1 CRC: All headers + data payload + V2 CRC byte
        checksum =
            mspSerialChecksumBuf(0 as libc::c_int as uint8_t,
                                 hdrBuf.as_mut_ptr().offset(3 as libc::c_int
                                                                as isize),
                                 hdrLen - 3 as libc::c_int);
        checksum =
            mspSerialChecksumBuf(checksum, sbufPtr(&mut (*packet).buf),
                                 dataLen);
        checksum =
            mspSerialChecksumBuf(checksum, crcBuf.as_mut_ptr(), crcLen);
        let fresh10 = crcLen;
        crcLen = crcLen + 1;
        crcBuf[fresh10 as usize] = checksum
    } else if mspVersion as libc::c_uint ==
                  MSP_V2_NATIVE as libc::c_int as libc::c_uint {
        let mut hdrV2_0: *mut mspHeaderV2_t =
            &mut *hdrBuf.as_mut_ptr().offset(hdrLen as isize) as *mut uint8_t
                as *mut mspHeaderV2_t;
        hdrLen =
            (hdrLen as
                 libc::c_ulong).wrapping_add(::core::mem::size_of::<mspHeaderV2_t>()
                                                 as libc::c_ulong) as
                libc::c_int as libc::c_int;
        (*hdrV2_0).flags = (*packet).flags;
        (*hdrV2_0).cmd = (*packet).cmd as uint16_t;
        (*hdrV2_0).size = dataLen as uint16_t;
        checksum =
            crc8_dvb_s2_update(0 as libc::c_int as uint8_t,
                               hdrV2_0 as *mut uint8_t as *const libc::c_void,
                               ::core::mem::size_of::<mspHeaderV2_t>() as
                                   libc::c_ulong as uint32_t);
        checksum =
            crc8_dvb_s2_update(checksum,
                               sbufPtr(&mut (*packet).buf) as
                                   *const libc::c_void, dataLen as uint32_t);
        let fresh11 = crcLen;
        crcLen = crcLen + 1;
        crcBuf[fresh11 as usize] = checksum
    } else {
        // Shouldn't get here
        return 0 as libc::c_int
    }
    // Send the frame
    return mspSerialSendFrame(msp, hdrBuf.as_mut_ptr(), hdrLen,
                              sbufPtr(&mut (*packet).buf), dataLen,
                              crcBuf.as_mut_ptr(),
                              crcLen); // change streambuf direction
}
unsafe extern "C" fn mspSerialProcessReceivedCommand(mut msp: *mut mspPort_t,
                                                     mut mspProcessCommandFn:
                                                         mspProcessCommandFnPtr)
 -> mspPostProcessFnPtr {
    static mut outBuf: [uint8_t; 256] = [0; 256];
    let mut reply: mspPacket_t =
        {
            let mut init =
                mspPacket_s{buf:
                                {
                                    let mut init =
                                        sbuf_s{ptr: outBuf.as_mut_ptr(),
                                               end:
                                                   &mut *outBuf.as_mut_ptr().offset((::core::mem::size_of::<[uint8_t; 256]>()
                                                                                         as
                                                                                         libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                                                                                         as
                                                                                                                         libc::c_ulong)
                                                                                        as
                                                                                        isize)
                                                       as *mut uint8_t,};
                                    init
                                },
                            cmd: -(1 as libc::c_int) as int16_t,
                            flags: 0 as libc::c_int as uint8_t,
                            result: 0 as libc::c_int as int16_t,
                            direction:
                                MSP_DIRECTION_REPLY as libc::c_int as
                                    uint8_t,};
            init
        };
    let mut outBufHead: *mut uint8_t = reply.buf.ptr;
    let mut command: mspPacket_t =
        {
            let mut init =
                mspPacket_s{buf:
                                {
                                    let mut init =
                                        sbuf_s{ptr: (*msp).inBuf.as_mut_ptr(),
                                               end:
                                                   (*msp).inBuf.as_mut_ptr().offset((*msp).dataSize
                                                                                        as
                                                                                        isize),};
                                    init
                                },
                            cmd: (*msp).cmdMSP as int16_t,
                            flags: (*msp).cmdFlags,
                            result: 0 as libc::c_int as int16_t,
                            direction:
                                MSP_DIRECTION_REQUEST as libc::c_int as
                                    uint8_t,};
            init
        };
    let mut mspPostProcessFn: mspPostProcessFnPtr = None;
    let status: mspResult_e =
        mspProcessCommandFn.expect("non-null function pointer")(&mut command,
                                                                &mut reply,
                                                                &mut mspPostProcessFn);
    if status as libc::c_int != MSP_RESULT_NO_REPLY as libc::c_int {
        sbufSwitchToReader(&mut reply.buf, outBufHead);
        mspSerialEncode(msp, &mut reply, (*msp).mspVersion);
    }
    return mspPostProcessFn;
}
unsafe extern "C" fn mspEvaluateNonMspData(mut mspPort: *mut mspPort_t,
                                           mut receivedChar: uint8_t) {
    if receivedChar as libc::c_int ==
           (*serialConfig()).reboot_character as libc::c_int {
        (*mspPort).pendingRequest = MSP_PENDING_BOOTLOADER;
        return
    };
}
unsafe extern "C" fn mspProcessPendingRequest(mut mspPort: *mut mspPort_t) {
    // If no request is pending or 100ms guard time has not elapsed - do nothing
    if (*mspPort).pendingRequest as libc::c_uint ==
           MSP_PENDING_NONE as libc::c_int as libc::c_uint ||
           millis().wrapping_sub((*mspPort).lastActivityMs) <
               100 as libc::c_int as libc::c_uint {
        return
    }
    match (*mspPort).pendingRequest as libc::c_uint {
        1 => { systemResetToBootloader(); }
        _ => { }
    };
}
unsafe extern "C" fn mspSerialProcessReceivedReply(mut msp: *mut mspPort_t,
                                                   mut mspProcessReplyFn:
                                                       mspProcessReplyFnPtr) {
    let mut reply: mspPacket_t =
        {
            let mut init =
                mspPacket_s{buf:
                                {
                                    let mut init =
                                        sbuf_s{ptr: (*msp).inBuf.as_mut_ptr(),
                                               end:
                                                   (*msp).inBuf.as_mut_ptr().offset((*msp).dataSize
                                                                                        as
                                                                                        isize),};
                                    init
                                },
                            cmd: (*msp).cmdMSP as int16_t,
                            flags: 0,
                            result: 0 as libc::c_int as int16_t,
                            direction: 0,};
            init
        };
    mspProcessReplyFn.expect("non-null function pointer")(&mut reply);
    (*msp).c_state = MSP_IDLE;
}
/*
 * Process MSP commands from serial ports configured as MSP ports.
 *
 * Called periodically by the scheduler.
 */
#[no_mangle]
pub unsafe extern "C" fn mspSerialProcess(mut evaluateNonMspData:
                                              mspEvaluateNonMspData_e,
                                          mut mspProcessCommandFn:
                                              mspProcessCommandFnPtr,
                                          mut mspProcessReplyFn:
                                              mspProcessReplyFnPtr) {
    let mut portIndex: uint8_t = 0 as libc::c_int as uint8_t;
    while (portIndex as libc::c_int) < 3 as libc::c_int {
        let mspPort: *mut mspPort_t =
            &mut *mspPorts.as_mut_ptr().offset(portIndex as isize) as
                *mut mspPort_t;
        if !(*mspPort).port.is_null() {
            let mut mspPostProcessFn: mspPostProcessFnPtr = None;
            if serialRxBytesWaiting((*mspPort).port) != 0 {
                // There are bytes incoming - abort pending request
                (*mspPort).lastActivityMs = millis();
                (*mspPort).pendingRequest = MSP_PENDING_NONE;
                while serialRxBytesWaiting((*mspPort).port) != 0 {
                    let c: uint8_t = serialRead((*mspPort).port);
                    let consumed: bool =
                        mspSerialProcessReceivedData(mspPort, c);
                    if !consumed &&
                           evaluateNonMspData as libc::c_uint ==
                               MSP_EVALUATE_NON_MSP_DATA as libc::c_int as
                                   libc::c_uint {
                        mspEvaluateNonMspData(mspPort, c);
                    }
                    if !((*mspPort).c_state as libc::c_uint ==
                             MSP_COMMAND_RECEIVED as libc::c_int as
                                 libc::c_uint) {
                        continue ;
                    }
                    if (*mspPort).packetType as libc::c_uint ==
                           MSP_PACKET_COMMAND as libc::c_int as libc::c_uint {
                        mspPostProcessFn =
                            mspSerialProcessReceivedCommand(mspPort,
                                                            mspProcessCommandFn)
                    } else if (*mspPort).packetType as libc::c_uint ==
                                  MSP_PACKET_REPLY as libc::c_int as
                                      libc::c_uint {
                        mspSerialProcessReceivedReply(mspPort,
                                                      mspProcessReplyFn);
                    }
                    (*mspPort).c_state = MSP_IDLE;
                    break ;
                    // process one command at a time so as not to block.
                }
                if mspPostProcessFn.is_some() {
                    waitForSerialPortToFinishTransmitting((*mspPort).port);
                    mspPostProcessFn.expect("non-null function pointer")((*mspPort).port);
                }
            } else { mspProcessPendingRequest(mspPort); }
        }
        portIndex = portIndex.wrapping_add(1)
    };
}
#[no_mangle]
pub unsafe extern "C" fn mspSerialWaiting() -> bool {
    let mut portIndex: uint8_t = 0 as libc::c_int as uint8_t;
    while (portIndex as libc::c_int) < 3 as libc::c_int {
        let mspPort: *mut mspPort_t =
            &mut *mspPorts.as_mut_ptr().offset(portIndex as isize) as
                *mut mspPort_t;
        if !(*mspPort).port.is_null() {
            if serialRxBytesWaiting((*mspPort).port) != 0 {
                return 1 as libc::c_int != 0
            }
        }
        portIndex = portIndex.wrapping_add(1)
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn mspSerialInit() {
    memset(mspPorts.as_mut_ptr() as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<[mspPort_t; 3]>() as libc::c_ulong);
    mspSerialAllocatePorts();
}
#[no_mangle]
pub unsafe extern "C" fn mspSerialPush(mut cmd: uint8_t,
                                       mut data: *mut uint8_t,
                                       mut datalen: libc::c_int,
                                       mut direction: mspDirection_e)
 -> libc::c_int {
    let mut ret: libc::c_int = 0 as libc::c_int;
    let mut portIndex: libc::c_int = 0 as libc::c_int;
    while portIndex < 3 as libc::c_int {
        let mspPort: *mut mspPort_t =
            &mut *mspPorts.as_mut_ptr().offset(portIndex as isize) as
                *mut mspPort_t;
        if !(*mspPort).port.is_null() {
            // XXX Kludge!!! Avoid zombie VCP port (avoid VCP entirely for now)
            if !((*(*mspPort).port).identifier as libc::c_int ==
                     SERIAL_PORT_USB_VCP as libc::c_int) {
                let mut push: mspPacket_t =
                    {
                        let mut init =
                            mspPacket_s{buf:
                                            {
                                                let mut init =
                                                    sbuf_s{ptr: data,
                                                           end:
                                                               data.offset(datalen
                                                                               as
                                                                               isize),};
                                                init
                                            },
                                        cmd: cmd as int16_t,
                                        flags: 0,
                                        result: 0 as libc::c_int as int16_t,
                                        direction: direction as uint8_t,};
                        init
                    };
                ret = mspSerialEncode(mspPort, &mut push, MSP_V1)
            }
        }
        portIndex += 1
    }
    return ret;
    // return the number of bytes written
}
#[no_mangle]
pub unsafe extern "C" fn mspSerialTxBytesFree() -> uint32_t {
    let mut ret: uint32_t = 4294967295 as libc::c_uint;
    let mut portIndex: libc::c_int = 0 as libc::c_int;
    while portIndex < 3 as libc::c_int {
        let mspPort: *mut mspPort_t =
            &mut *mspPorts.as_mut_ptr().offset(portIndex as isize) as
                *mut mspPort_t;
        if !(*mspPort).port.is_null() {
            // XXX Kludge!!! Avoid zombie VCP port (avoid VCP entirely for now)
            if !((*(*mspPort).port).identifier as libc::c_int ==
                     SERIAL_PORT_USB_VCP as libc::c_int) {
                let bytesFree: uint32_t = serialTxBytesFree((*mspPort).port);
                if bytesFree < ret { ret = bytesFree }
            }
        }
        portIndex += 1
    }
    return ret;
}
