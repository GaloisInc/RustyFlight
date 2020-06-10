use ::libc;
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
    fn serialRxBytesWaiting(instance: *const serialPort_t) -> uint32_t;
    #[no_mangle]
    fn serialWriteBuf(instance: *mut serialPort_t, data: *const uint8_t,
                      count: libc::c_int);
    #[no_mangle]
    fn serialRead(instance: *mut serialPort_t) -> uint8_t;
    #[no_mangle]
    fn openSerialPort(identifier: serialPortIdentifier_e,
                      function: serialPortFunction_e,
                      rxCallback: serialReceiveCallbackPtr,
                      rxCallbackData: *mut libc::c_void, baudrate: uint32_t,
                      mode: portMode_e, options: portOptions_e)
     -> *mut serialPort_t;
    #[no_mangle]
    fn findSerialPortConfig(function: serialPortFunction_e)
     -> *mut serialPortConfig_t;
    #[no_mangle]
    fn millis() -> timeMs_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
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
pub type serialPortConfig_t = serialPortConfig_s;
// millisecond time
pub type timeMs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rangefinderDev_s {
    pub delayMs: timeMs_t,
    pub maxRangeCm: int16_t,
    pub detectionConeDeciDegrees: int16_t,
    pub detectionConeExtendedDeciDegrees: int16_t,
    pub init: rangefinderOpInitFuncPtr,
    pub update: rangefinderOpStartFuncPtr,
    pub read: rangefinderOpReadFuncPtr,
}
pub type rangefinderOpReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut rangefinderDev_s) -> int32_t>;
pub type rangefinderOpStartFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut rangefinderDev_s) -> ()>;
pub type rangefinderOpInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut rangefinderDev_s) -> ()>;
pub type rangefinderDev_t = rangefinderDev_s;
pub type tfFrameState_e = libc::c_uint;
pub const TF_FRAME_STATE_WAIT_CKSUM: tfFrameState_e = 3;
pub const TF_FRAME_STATE_READING_PAYLOAD: tfFrameState_e = 2;
pub const TF_FRAME_STATE_WAIT_START2: tfFrameState_e = 1;
pub const TF_FRAME_STATE_WAIT_START1: tfFrameState_e = 0;
static mut tfDevtype: uint8_t = 0 as libc::c_int as uint8_t;
static mut tfSerialPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
static mut tfFrameState: tfFrameState_e = TF_FRAME_STATE_WAIT_START1;
static mut tfFrame: [uint8_t; 6] = [0; 6];
static mut tfReceivePosition: uint8_t = 0;
// these are full detection cone angles, maximum tilt is half of this
// detection cone angle as in device spec
// device spec is conservative, in practice have slightly larger detection cone
// function pointers
// TFmini
// Command for 100Hz sampling (10msec interval)
// At 100Hz scheduling, skew will cause 10msec delay at the most.
static mut tfCmdTFmini: [uint8_t; 8] =
    [0x42 as libc::c_int as uint8_t, 0x57 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t];
// TF02
// Same as TFmini for now..
static mut tfCmdTF02: [uint8_t; 8] =
    [0x42 as libc::c_int as uint8_t, 0x57 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t];
static mut lidarTFValue: int32_t = 0;
static mut lidarTFerrors: uint16_t = 0 as libc::c_int as uint16_t;
unsafe extern "C" fn lidarTFSendCommand() {
    match tfDevtype as libc::c_int {
        1 => {
            serialWriteBuf(tfSerialPort, tfCmdTFmini.as_mut_ptr(),
                           ::core::mem::size_of::<[uint8_t; 8]>() as
                               libc::c_ulong as libc::c_int);
        }
        2 => {
            serialWriteBuf(tfSerialPort, tfCmdTF02.as_mut_ptr(),
                           ::core::mem::size_of::<[uint8_t; 8]>() as
                               libc::c_ulong as libc::c_int);
        }
        _ => { }
    };
}
#[no_mangle]
pub unsafe extern "C" fn lidarTFInit(mut dev: *mut rangefinderDev_t) {
    tfFrameState = TF_FRAME_STATE_WAIT_START1;
    tfReceivePosition = 0 as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn lidarTFUpdate(mut dev: *mut rangefinderDev_t) {
    static mut lastFrameReceivedMs: timeMs_t = 0 as libc::c_int as timeMs_t;
    let timeNowMs: timeMs_t = millis();
    if tfSerialPort.is_null() { return }
    while serialRxBytesWaiting(tfSerialPort) != 0 {
        let mut c: uint8_t = serialRead(tfSerialPort);
        match tfFrameState as libc::c_uint {
            0 => {
                if c as libc::c_int == 0x59 as libc::c_int {
                    tfFrameState = TF_FRAME_STATE_WAIT_START2
                }
            }
            1 => {
                if c as libc::c_int == 0x59 as libc::c_int {
                    tfFrameState = TF_FRAME_STATE_READING_PAYLOAD
                } else { tfFrameState = TF_FRAME_STATE_WAIT_START1 }
            }
            2 => {
                let fresh0 = tfReceivePosition;
                tfReceivePosition = tfReceivePosition.wrapping_add(1);
                tfFrame[fresh0 as usize] = c;
                if tfReceivePosition as libc::c_int == 6 as libc::c_int {
                    tfFrameState = TF_FRAME_STATE_WAIT_CKSUM
                }
            }
            3 => {
                let mut cksum: uint8_t =
                    (0x59 as libc::c_int + 0x59 as libc::c_int) as uint8_t;
                let mut i: libc::c_int = 0 as libc::c_int;
                while i < 6 as libc::c_int {
                    cksum =
                        (cksum as libc::c_int +
                             tfFrame[i as usize] as libc::c_int) as uint8_t;
                    i += 1
                }
                if c as libc::c_int == cksum as libc::c_int {
                    let mut distance: uint16_t =
                        (tfFrame[0 as libc::c_int as usize] as libc::c_int |
                             (tfFrame[1 as libc::c_int as usize] as
                                  libc::c_int) << 8 as libc::c_int) as
                            uint16_t;
                    let mut strength: uint16_t =
                        (tfFrame[2 as libc::c_int as usize] as libc::c_int |
                             (tfFrame[3 as libc::c_int as usize] as
                                  libc::c_int) << 8 as libc::c_int) as
                            uint16_t;
                    if debugMode as libc::c_int ==
                           DEBUG_LIDAR_TF as libc::c_int {
                        debug[0 as libc::c_int as usize] = distance as int16_t
                    }
                    if debugMode as libc::c_int ==
                           DEBUG_LIDAR_TF as libc::c_int {
                        debug[1 as libc::c_int as usize] = strength as int16_t
                    }
                    if debugMode as libc::c_int ==
                           DEBUG_LIDAR_TF as libc::c_int {
                        debug[2 as libc::c_int as usize] =
                            tfFrame[4 as libc::c_int as usize] as int16_t
                    }
                    if debugMode as libc::c_int ==
                           DEBUG_LIDAR_TF as libc::c_int {
                        debug[3 as libc::c_int as usize] =
                            tfFrame[5 as libc::c_int as usize] as int16_t
                    }
                    match tfDevtype as libc::c_int {
                        1 => {
                            if distance as libc::c_int >= 40 as libc::c_int &&
                                   (distance as libc::c_int) <
                                       1200 as libc::c_int {
                                lidarTFValue = distance as int32_t;
                                if tfFrame[4 as libc::c_int as usize] as
                                       libc::c_int == 7 as libc::c_int {
                                    // When integral time is long (7), measured distance tends to be longer by 12~13.
                                    lidarTFValue -= 13 as libc::c_int
                                }
                            } else { lidarTFValue = -(1 as libc::c_int) }
                        }
                        2 => {
                            if distance as libc::c_int >= 40 as libc::c_int &&
                                   (distance as libc::c_int) <
                                       2200 as libc::c_int &&
                                   tfFrame[4 as libc::c_int as usize] as
                                       libc::c_int >= 7 as libc::c_int {
                                lidarTFValue = distance as int32_t
                            } else { lidarTFValue = -(1 as libc::c_int) }
                        }
                        _ => { }
                    }
                    lastFrameReceivedMs = timeNowMs
                } else {
                    // Checksum error. Simply discard the current frame.
                    lidarTFerrors = lidarTFerrors.wrapping_add(1)
                    //DEBUG_SET(DEBUG_LIDAR_TF, 3, lidarTFerrors);
                }
                tfFrameState = TF_FRAME_STATE_WAIT_START1;
                tfReceivePosition = 0 as libc::c_int as uint8_t
            }
            _ => { }
        }
    }
    // If valid frame hasn't been received for more than a timeout, resend command.
    if timeNowMs.wrapping_sub(lastFrameReceivedMs) >
           (100 as libc::c_int * 2 as libc::c_int) as libc::c_uint {
        lidarTFSendCommand();
    };
}
// Return most recent device output in cm
#[no_mangle]
pub unsafe extern "C" fn lidarTFGetDistance(mut dev: *mut rangefinderDev_t)
 -> int32_t {
    return lidarTFValue;
}
unsafe extern "C" fn lidarTFDetect(mut dev: *mut rangefinderDev_t,
                                   mut devtype: uint8_t) -> bool {
    let mut portConfig: *mut serialPortConfig_t =
        findSerialPortConfig(FUNCTION_LIDAR_TF);
    if portConfig.is_null() { return 0 as libc::c_int != 0 }
    tfSerialPort =
        openSerialPort((*portConfig).identifier, FUNCTION_LIDAR_TF, None,
                       0 as *mut libc::c_void,
                       115200 as libc::c_int as uint32_t, MODE_RXTX,
                       SERIAL_NOT_INVERTED);
    if tfSerialPort.is_null() { return 0 as libc::c_int != 0 }
    tfDevtype = devtype;
    (*dev).delayMs = 10 as libc::c_int as timeMs_t;
    (*dev).maxRangeCm =
        if devtype as libc::c_int == 1 as libc::c_int {
            1200 as libc::c_int
        } else { 2200 as libc::c_int } as int16_t;
    (*dev).detectionConeDeciDegrees = 900 as libc::c_int as int16_t;
    (*dev).detectionConeExtendedDeciDegrees = 900 as libc::c_int as int16_t;
    (*dev).init =
        Some(lidarTFInit as
                 unsafe extern "C" fn(_: *mut rangefinderDev_t) -> ());
    (*dev).update =
        Some(lidarTFUpdate as
                 unsafe extern "C" fn(_: *mut rangefinderDev_t) -> ());
    (*dev).read =
        Some(lidarTFGetDistance as
                 unsafe extern "C" fn(_: *mut rangefinderDev_t) -> int32_t);
    return 1 as libc::c_int != 0;
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
pub unsafe extern "C" fn lidarTFminiDetect(mut dev: *mut rangefinderDev_t)
 -> bool {
    return lidarTFDetect(dev, 1 as libc::c_int as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn lidarTF02Detect(mut dev: *mut rangefinderDev_t)
 -> bool {
    return lidarTFDetect(dev, 2 as libc::c_int as uint8_t);
}
