use ::libc;
extern "C" {
    #[no_mangle]
    fn memmove(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn serialRxBytesWaiting(instance: *const serialPort_t) -> uint32_t;
    #[no_mangle]
    fn serialRead(instance: *mut serialPort_t) -> uint8_t;
    #[no_mangle]
    fn findSerialPortConfig(function: serialPortFunction_e)
     -> *mut serialPortConfig_t;
    #[no_mangle]
    fn determinePortSharing(portConfig: *const serialPortConfig_t,
                            function: serialPortFunction_e) -> portSharing_e;
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
    fn rescheduleTask(taskId: cfTaskId_e, newPeriodMicros: uint32_t);
    #[no_mangle]
    fn respondToIbusRequest(ibusPacket: *const uint8_t) -> uint8_t;
    #[no_mangle]
    fn initSharedIbusTelemetry(port: *mut serialPort_t);
    //defined(TELEMETRY) && defined(TELEMETRY_IBUS)
    #[no_mangle]
    fn isChecksumOkIa6b(ibusPacket: *const uint8_t, length: uint8_t) -> bool;
    #[no_mangle]
    static mut telemetryConfig_System: telemetryConfig_t;
    #[no_mangle]
    fn telemetryDetermineEnabledState(portSharing: portSharing_e) -> bool;
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type size_t = libc::c_ulong;
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
pub type portSharing_e = libc::c_uint;
pub const PORTSHARING_SHARED: portSharing_e = 2;
pub const PORTSHARING_NOT_SHARED: portSharing_e = 1;
pub const PORTSHARING_UNUSED: portSharing_e = 0;
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
pub type cfTaskId_e = libc::c_uint;
pub const TASK_SELF: cfTaskId_e = 28;
pub const TASK_NONE: cfTaskId_e = 27;
pub const TASK_COUNT: cfTaskId_e = 27;
pub const TASK_PINIOBOX: cfTaskId_e = 26;
pub const TASK_ADC_INTERNAL: cfTaskId_e = 25;
pub const TASK_RCDEVICE: cfTaskId_e = 24;
pub const TASK_CAMCTRL: cfTaskId_e = 23;
pub const TASK_VTXCTRL: cfTaskId_e = 22;
pub const TASK_CMS: cfTaskId_e = 21;
pub const TASK_ESC_SENSOR: cfTaskId_e = 20;
pub const TASK_OSD: cfTaskId_e = 19;
pub const TASK_LEDSTRIP: cfTaskId_e = 18;
pub const TASK_TELEMETRY: cfTaskId_e = 17;
pub const TASK_DASHBOARD: cfTaskId_e = 16;
pub const TASK_ALTITUDE: cfTaskId_e = 15;
pub const TASK_BARO: cfTaskId_e = 14;
pub const TASK_COMPASS: cfTaskId_e = 13;
pub const TASK_GPS: cfTaskId_e = 12;
pub const TASK_BEEPER: cfTaskId_e = 11;
pub const TASK_BATTERY_ALERTS: cfTaskId_e = 10;
pub const TASK_BATTERY_CURRENT: cfTaskId_e = 9;
pub const TASK_BATTERY_VOLTAGE: cfTaskId_e = 8;
pub const TASK_DISPATCH: cfTaskId_e = 7;
pub const TASK_SERIAL: cfTaskId_e = 6;
pub const TASK_RX: cfTaskId_e = 5;
pub const TASK_ATTITUDE: cfTaskId_e = 4;
pub const TASK_ACCEL: cfTaskId_e = 3;
pub const TASK_GYROPID: cfTaskId_e = 2;
pub const TASK_MAIN: cfTaskId_e = 1;
pub const TASK_SYSTEM: cfTaskId_e = 0;
pub type telemetryConfig_t = telemetryConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct telemetryConfig_s {
    pub gpsNoFixLatitude: int16_t,
    pub gpsNoFixLongitude: int16_t,
    pub telemetry_inverted: uint8_t,
    pub halfDuplex: uint8_t,
    pub frsky_coordinate_format: frskyGpsCoordFormat_e,
    pub frsky_unit: frskyUnit_e,
    pub frsky_vfas_precision: uint8_t,
    pub hottAlarmSoundInterval: uint8_t,
    pub pidValuesAsTelemetry: uint8_t,
    pub report_cell_voltage: uint8_t,
    pub flysky_sensors: [uint8_t; 15],
    pub smartport_use_extra_sensors: uint8_t,
}
pub type frskyUnit_e = libc::c_uint;
pub const FRSKY_UNIT_IMPERIALS: frskyUnit_e = 1;
pub const FRSKY_UNIT_METRICS: frskyUnit_e = 0;
pub type frskyGpsCoordFormat_e = libc::c_uint;
pub const FRSKY_FORMAT_NMEA: frskyGpsCoordFormat_e = 1;
pub const FRSKY_FORMAT_DMS: frskyGpsCoordFormat_e = 0;
#[inline]
unsafe extern "C" fn telemetryConfig() -> *const telemetryConfig_t {
    return &mut telemetryConfig_System;
}
static mut ibusSerialPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
static mut ibusSerialPortConfig: *mut serialPortConfig_t =
    0 as *const serialPortConfig_t as *mut serialPortConfig_t;
// not used for all telemetry systems, e.g. HoTT only works at 19200.
/* The sent bytes will be echoed back since Tx and Rx are wired together, this counter
 * will keep track of how many rx chars that shall be discarded */
static mut outboundBytesToIgnoreOnRxCount: uint8_t =
    0 as libc::c_int as uint8_t;
static mut ibusTelemetryEnabled: bool = 0 as libc::c_int != 0;
static mut ibusPortSharing: portSharing_e = PORTSHARING_UNUSED;
static mut ibusReceiveBuffer: [uint8_t; 4] =
    [0 as libc::c_int as uint8_t, 0, 0, 0];
unsafe extern "C" fn pushOntoTail(mut buffer: *mut uint8_t,
                                  mut bufferLength: size_t,
                                  mut value: uint8_t) {
    memmove(buffer as *mut libc::c_void,
            buffer.offset(1 as libc::c_int as isize) as *const libc::c_void,
            bufferLength.wrapping_sub(1 as libc::c_int as libc::c_ulong));
    ibusReceiveBuffer[bufferLength.wrapping_sub(1 as libc::c_int as
                                                    libc::c_ulong) as usize] =
        value;
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
pub unsafe extern "C" fn initIbusTelemetry() {
    ibusSerialPortConfig = findSerialPortConfig(FUNCTION_TELEMETRY_IBUS);
    ibusPortSharing =
        determinePortSharing(ibusSerialPortConfig, FUNCTION_TELEMETRY_IBUS);
    ibusTelemetryEnabled = 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn handleIbusTelemetry() {
    if !ibusTelemetryEnabled { return }
    while serialRxBytesWaiting(ibusSerialPort) >
              0 as libc::c_int as libc::c_uint {
        let mut c: uint8_t = serialRead(ibusSerialPort);
        if outboundBytesToIgnoreOnRxCount != 0 {
            outboundBytesToIgnoreOnRxCount =
                outboundBytesToIgnoreOnRxCount.wrapping_sub(1)
        } else {
            pushOntoTail(ibusReceiveBuffer.as_mut_ptr(),
                         4 as libc::c_int as size_t, c);
            if isChecksumOkIa6b(ibusReceiveBuffer.as_mut_ptr(),
                                4 as libc::c_int as uint8_t) {
                outboundBytesToIgnoreOnRxCount =
                    (outboundBytesToIgnoreOnRxCount as libc::c_int +
                         respondToIbusRequest(ibusReceiveBuffer.as_mut_ptr())
                             as libc::c_int) as uint8_t
            }
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn checkIbusTelemetryState() -> bool {
    let mut newTelemetryEnabledValue: bool =
        telemetryDetermineEnabledState(ibusPortSharing);
    if newTelemetryEnabledValue as libc::c_int ==
           ibusTelemetryEnabled as libc::c_int {
        return 0 as libc::c_int != 0
    }
    if newTelemetryEnabledValue {
        rescheduleTask(TASK_TELEMETRY, 1000 as libc::c_int as uint32_t);
        configureIbusTelemetryPort();
    } else { freeIbusTelemetryPort(); }
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn configureIbusTelemetryPort() {
    if ibusSerialPortConfig.is_null() { return }
    if isSerialPortShared(ibusSerialPortConfig,
                          FUNCTION_RX_SERIAL as libc::c_int as uint16_t,
                          FUNCTION_TELEMETRY_IBUS) {
        // serialRx will open port and handle telemetry
        return
    }
    ibusSerialPort =
        openSerialPort((*ibusSerialPortConfig).identifier,
                       FUNCTION_TELEMETRY_IBUS, None, 0 as *mut libc::c_void,
                       115200 as libc::c_int as uint32_t, MODE_RXTX,
                       (SERIAL_BIDIR as libc::c_int |
                            (if (*telemetryConfig()).telemetry_inverted as
                                    libc::c_int != 0 {
                                 SERIAL_INVERTED as libc::c_int
                             } else { SERIAL_NOT_INVERTED as libc::c_int }))
                           as portOptions_e);
    if ibusSerialPort.is_null() { return }
    initSharedIbusTelemetry(ibusSerialPort);
    ibusTelemetryEnabled = 1 as libc::c_int != 0;
    outboundBytesToIgnoreOnRxCount = 0 as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn freeIbusTelemetryPort() {
    closeSerialPort(ibusSerialPort);
    ibusSerialPort = 0 as *mut serialPort_t;
    ibusTelemetryEnabled = 0 as libc::c_int != 0;
}
