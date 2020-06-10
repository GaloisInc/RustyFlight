use ::libc;
extern "C" {
    #[no_mangle]
    static mut blackboxConfig_System: blackboxConfig_t;
    #[no_mangle]
    static mut targetPidLooptime: uint32_t;
    #[no_mangle]
    static baudRates: [uint32_t; 0];
    #[no_mangle]
    fn findSharedSerialPort(functionMask: uint16_t,
                            sharedWithFunction: serialPortFunction_e)
     -> *mut serialPort_t;
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
    fn determinePortSharing(portConfig: *const serialPortConfig_t,
                            function: serialPortFunction_e) -> portSharing_e;
    #[no_mangle]
    fn closeSerialPort(serialPort: *mut serialPort_t);
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
    // Specified baud rate may not be allowed by an implementation, use serialGetBaudRate to determine actual baud rate in use.
    // Optional functions used to buffer large writes.
    #[no_mangle]
    fn isSerialTransmitBufferEmpty(instance: *const serialPort_t) -> bool;
    #[no_mangle]
    fn serialWrite(instance: *mut serialPort_t, ch: uint8_t);
    #[no_mangle]
    fn serialTxBytesFree(instance: *const serialPort_t) -> uint32_t;
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
    // Each MSP port requires state and a receive buffer, revisit this default if someone needs more than 3 MSP ports.
    // null when port unused.
    #[no_mangle]
    fn mspSerialAllocatePorts();
    #[no_mangle]
    fn mspSerialReleasePortIfAllocated(serialPort: *mut serialPort_s);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type BlackboxDevice = libc::c_uint;
pub const BLACKBOX_DEVICE_SERIAL: BlackboxDevice = 3;
pub const BLACKBOX_DEVICE_NONE: BlackboxDevice = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct blackboxConfig_s {
    pub p_ratio: uint16_t,
    pub device: uint8_t,
    pub record_acc: uint8_t,
    pub mode: uint8_t,
}
pub type blackboxConfig_t = blackboxConfig_s;
pub type blackboxBufferReserveStatus_e = libc::c_uint;
pub const BLACKBOX_RESERVE_PERMANENT_FAILURE: blackboxBufferReserveStatus_e =
    2;
pub const BLACKBOX_RESERVE_TEMPORARY_FAILURE: blackboxBufferReserveStatus_e =
    1;
pub const BLACKBOX_RESERVE_SUCCESS: blackboxBufferReserveStatus_e = 0;
pub type serialPort_t = serialPort_s;
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
// I-frame interval / P-frame interval
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
pub const BAUD_2470000: baudRate_e = 15;
pub const BAUD_2000000: baudRate_e = 14;
pub const BAUD_1500000: baudRate_e = 13;
pub const BAUD_1000000: baudRate_e = 12;
pub type baudRate_e = libc::c_uint;
pub const BAUD_921600: baudRate_e = 11;
pub const BAUD_500000: baudRate_e = 10;
pub const BAUD_460800: baudRate_e = 9;
pub const BAUD_400000: baudRate_e = 8;
pub const BAUD_250000: baudRate_e = 7;
pub const BAUD_230400: baudRate_e = 6;
pub const BAUD_115200: baudRate_e = 5;
pub const BAUD_57600: baudRate_e = 4;
pub const BAUD_38400: baudRate_e = 3;
pub const BAUD_19200: baudRate_e = 2;
pub const BAUD_9600: baudRate_e = 1;
pub const BAUD_AUTO: baudRate_e = 0;
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
//
// configuration
//
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
pub type portSharing_e = libc::c_uint;
pub const PORTSHARING_SHARED: portSharing_e = 2;
pub const PORTSHARING_NOT_SHARED: portSharing_e = 1;
pub const PORTSHARING_UNUSED: portSharing_e = 0;
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
// SITL (software in the loop) simulator
// use simulatior's attitude directly
// disable this if wants to test AHRS algorithm
//#define SIMULATOR_ACC_SYNC
//#define SIMULATOR_GYRO_SYNC
//#define SIMULATOR_IMU_SYNC
//#define SIMULATOR_GYROPID_SYNC
// file name to save config
//#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2
// I think SITL don't need this
// suppress 'no pins defined' warning
// belows are internal stuff
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
#[inline]
unsafe extern "C" fn blackboxConfig() -> *const blackboxConfig_t {
    return &mut blackboxConfig_System;
}
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
// How many bytes can we transmit per loop iteration when writing headers?
static mut blackboxMaxHeaderBytesPerIteration: uint8_t = 0;
// How many bytes can we write *this* iteration without overflowing transmit buffers or overstressing the OpenLog?
#[no_mangle]
pub static mut blackboxHeaderBudget: int32_t = 0;
static mut blackboxPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
static mut blackboxPortSharing: portSharing_e = PORTSHARING_UNUSED;
// USE_SDCARD
#[no_mangle]
pub unsafe extern "C" fn blackboxOpen() {
    let mut sharedBlackboxAndMspPort: *mut serialPort_t =
        findSharedSerialPort(FUNCTION_BLACKBOX as libc::c_int as uint16_t,
                             FUNCTION_MSP);
    if !sharedBlackboxAndMspPort.is_null() {
        mspSerialReleasePortIfAllocated(sharedBlackboxAndMspPort);
    };
}
#[no_mangle]
pub unsafe extern "C" fn blackboxWrite(mut value: uint8_t) {
    match (*blackboxConfig()).device as libc::c_int { 3 | _ => { } }
    serialWrite(blackboxPort, value);
}
// Print the null-terminated string 's' to the blackbox device and return the number of bytes written
#[no_mangle]
pub unsafe extern "C" fn blackboxWriteString(mut s: *const libc::c_char)
 -> libc::c_int {
    let mut length: libc::c_int = 0;
    let mut pos: *const uint8_t = 0 as *const uint8_t;
    match (*blackboxConfig()).device as libc::c_int { 3 | _ => { } }
    // USE_FLASHFS
    // USE_SDCARD
    pos = s as *mut uint8_t;
    while *pos != 0 { serialWrite(blackboxPort, *pos); pos = pos.offset(1) }
    length =
        pos.wrapping_offset_from(s as *mut uint8_t) as libc::c_long as
            libc::c_int;
    return length;
}
/* *
 * If there is data waiting to be written to the blackbox device, attempt to write (a portion of) that now.
 *
 * Intended to be called regularly for the blackbox device to perform housekeeping.
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxDeviceFlush() {
    match (*blackboxConfig()).device as libc::c_int { _ => { } };
}
/* *
 * If there is data waiting to be written to the blackbox device, attempt to write (a portion of) that now.
 *
 * Returns true if all data has been written to the device.
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxDeviceFlushForce() -> bool {
    match (*blackboxConfig()).device as libc::c_int {
        3 => {
            // Nothing to speed up flushing on serial, as serial is continuously being drained out of its buffer
            return isSerialTransmitBufferEmpty(blackboxPort)
        }
        _ => {
            // USE_FLASHFS
            // USE_SDCARD
            return 0 as libc::c_int != 0
        }
    };
}
/* *
 * Attempt to open the logging device. Returns true if successful.
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxDeviceOpen() -> bool {
    match (*blackboxConfig()).device as libc::c_int {
        3 => {
            let mut portConfig: *mut serialPortConfig_t =
                findSerialPortConfig(FUNCTION_BLACKBOX);
            let mut baudRateIndex: baudRate_e = BAUD_AUTO;
            let mut portOptions: portOptions_e =
                (SERIAL_PARITY_NO as libc::c_int |
                     SERIAL_NOT_INVERTED as libc::c_int) as portOptions_e;
            if portConfig.is_null() { return 0 as libc::c_int != 0 }
            blackboxPortSharing =
                determinePortSharing(portConfig, FUNCTION_BLACKBOX);
            baudRateIndex =
                (*portConfig).blackbox_baudrateIndex as baudRate_e;
            if *baudRates.as_ptr().offset(baudRateIndex as isize) ==
                   230400 as libc::c_int as libc::c_uint {
                /*
                 * OpenLog's 230400 baud rate is very inaccurate, so it requires a larger inter-character gap in
                 * order to maintain synchronization.
                 */
                portOptions =
                    ::core::mem::transmute::<libc::c_uint,
                                             portOptions_e>(portOptions as
                                                                libc::c_uint |
                                                                SERIAL_STOPBITS_2
                                                                    as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint)
            } else {
                portOptions =
                    ::core::mem::transmute::<libc::c_uint,
                                             portOptions_e>(portOptions as
                                                                libc::c_uint |
                                                                SERIAL_STOPBITS_1
                                                                    as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint)
            }
            blackboxPort =
                openSerialPort((*portConfig).identifier, FUNCTION_BLACKBOX,
                               None, 0 as *mut libc::c_void,
                               *baudRates.as_ptr().offset(baudRateIndex as
                                                              isize), MODE_TX,
                               portOptions);
            /*
             * The slowest MicroSD cards have a write latency approaching 400ms. The OpenLog's buffer is about 900
             * bytes. In order for its buffer to be able to absorb this latency we must write slower than 6000 B/s.
             *
             * The OpenLager has a 125KB buffer for when the the MicroSD card is busy, so when the user configures
             * high baud rates, assume the OpenLager is in use and so there is no need to constrain the writes.
             *
             * In all other cases, constrain the writes as follows:
             *
             *     Bytes per loop iteration = floor((looptime_ns / 1000000.0) * 6000)
             *                              = floor((looptime_ns * 6000) / 1000000.0)
             *                              = floor((looptime_ns * 3) / 500.0)
             *                              = (looptime_ns * 3) / 500
             */
            match baudRateIndex as libc::c_uint {
                12 | 13 | 14 | 15 => {
                    // assume OpenLager in use, so do not constrain writes
                    blackboxMaxHeaderBytesPerIteration =
                        64 as libc::c_int as uint8_t
                }
                _ => {
                    blackboxMaxHeaderBytesPerIteration =
                        constrain(targetPidLooptime.wrapping_mul(3 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_div(500
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                      as libc::c_int, 1 as libc::c_int,
                                  64 as libc::c_int) as uint8_t
                }
            }
            return !blackboxPort.is_null()
        }
        _ => {
            // USE_FLASHFS
            // USE_SDCARD
            return 0 as libc::c_int != 0
        }
    };
}
/* *
 * Erase all blackbox logs
 */
/* *
 * Close the Blackbox logging device.
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxDeviceClose() {
    match (*blackboxConfig()).device as libc::c_int {
        3 => {
            // Can immediately close without attempting to flush any remaining data.
        // Since the serial port could be shared with other processes, we have to give it back here
            closeSerialPort(blackboxPort);
            blackboxPort = 0 as *mut serialPort_t;
            /*
         * Normally this would be handled by mw.c, but since we take an unknown amount
         * of time to shut down asynchronously, we're the only ones that know when to call it.
         */
            if blackboxPortSharing as libc::c_uint ==
                   PORTSHARING_SHARED as libc::c_int as libc::c_uint {
                mspSerialAllocatePorts();
            }
        }
        _ => { }
    };
}
// USE_SDCARD
/* *
 * Begin a new log (for devices which support separations between the logs of multiple flights).
 *
 * Keep calling until the function returns true (open is complete).
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxDeviceBeginLog() -> bool {
    match (*blackboxConfig()).device as libc::c_int { _ => { } }
    // USE_SDCARD
    return 1 as libc::c_int != 0;
}
/* *
 * Terminate the current log (for devices which support separations between the logs of multiple flights).
 *
 * retainLog - Pass true if the log should be kept, or false if the log should be discarded (if supported).
 *
 * Keep calling until this returns true
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxDeviceEndLog(mut retainLog: bool) -> bool {
    match (*blackboxConfig()).device as libc::c_int { _ => { } }
    // USE_SDCARD
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn isBlackboxDeviceFull() -> bool {
    match (*blackboxConfig()).device as libc::c_int {
        3 => { return 0 as libc::c_int != 0 }
        _ => {
            // USE_FLASHFS
            // USE_SDCARD
            return 0 as libc::c_int != 0
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn blackboxGetLogNumber() -> libc::c_uint {
    return 0 as libc::c_int as libc::c_uint;
}
/* *
 * Call once every loop iteration in order to maintain the global blackboxHeaderBudget with the number of bytes we can
 * transmit this iteration.
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxReplenishHeaderBudget() {
    let mut freeSpace: int32_t = 0;
    match (*blackboxConfig()).device as libc::c_int {
        3 => { freeSpace = serialTxBytesFree(blackboxPort) as int32_t }
        _ => { freeSpace = 0 as libc::c_int }
    }
    blackboxHeaderBudget =
        ({
             let mut _a: libc::c_int =
                 ({
                      let mut _a_0: int32_t = freeSpace;
                      let mut _b: libc::c_int =
                          blackboxHeaderBudget +
                              blackboxMaxHeaderBytesPerIteration as
                                  libc::c_int;
                      if _a_0 < _b { _a_0 } else { _b }
                  });
             let mut _b: libc::c_int = 256 as libc::c_int;
             if _a < _b { _a } else { _b }
         });
}
/* *
 * You must call this function before attempting to write Blackbox header bytes to ensure that the write will not
 * cause buffers to overflow. The number of bytes you can write is capped by the blackboxHeaderBudget. Calling this
 * reservation function doesn't decrease blackboxHeaderBudget, so you must manually decrement that variable by the
 * number of bytes you actually wrote.
 *
 * When the Blackbox device is FlashFS, a successful return code guarantees that no data will be lost if you write that
 * many bytes to the device (i.e. FlashFS's buffers won't overflow).
 *
 * When the device is a serial port, a successful return code guarantees that Cleanflight's serial Tx buffer will not
 * overflow, and the outgoing bandwidth is likely to be small enough to give the OpenLog time to absorb MicroSD card
 * latency. However the OpenLog could still end up silently dropping data.
 *
 * Returns:
 *  BLACKBOX_RESERVE_SUCCESS - Upon success
 *  BLACKBOX_RESERVE_TEMPORARY_FAILURE - The buffer is currently too full to service the request, try again later
 *  BLACKBOX_RESERVE_PERMANENT_FAILURE - The buffer is too small to ever service this request
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxDeviceReserveBufferSpace(mut bytes: int32_t)
 -> blackboxBufferReserveStatus_e {
    if bytes <= blackboxHeaderBudget { return BLACKBOX_RESERVE_SUCCESS }
    // Handle failure:
    match (*blackboxConfig()).device as libc::c_int {
        3 => {
            /*
         * One byte of the tx buffer isn't available for user data (due to its circular list implementation),
         * hence the -1. Note that the USB VCP implementation doesn't use a buffer and has txBufferSize set to zero.
         */
            if (*blackboxPort).txBufferSize != 0 &&
                   bytes >
                       (*blackboxPort).txBufferSize as int32_t -
                           1 as libc::c_int {
                return BLACKBOX_RESERVE_PERMANENT_FAILURE
            }
            return BLACKBOX_RESERVE_TEMPORARY_FAILURE
        }
        _ => {
            // USE_FLASHFS
            // USE_SDCARD
            return BLACKBOX_RESERVE_PERMANENT_FAILURE
        }
    };
}
// BLACKBOX
