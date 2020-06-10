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
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    fn getMotorDmaOutput(index: uint8_t) -> *mut motorDmaOutput_t;
    #[no_mangle]
    fn pwmAreMotorsEnabled() -> bool;
    #[no_mangle]
    static mut motorConfig_System: motorConfig_t;
    #[no_mangle]
    fn getMotorCount() -> uint8_t;
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
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  * @brief DMA Controller
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_Stream_TypeDef {
    pub CR: uint32_t,
    pub NDTR: uint32_t,
    pub PAR: uint32_t,
    pub M0AR: uint32_t,
    pub M1AR: uint32_t,
    pub FCR: uint32_t,
}
/* *
  * @brief TIM
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SMCR: uint32_t,
    pub DIER: uint32_t,
    pub SR: uint32_t,
    pub EGR: uint32_t,
    pub CCMR1: uint32_t,
    pub CCMR2: uint32_t,
    pub CCER: uint32_t,
    pub CNT: uint32_t,
    pub PSC: uint32_t,
    pub ARR: uint32_t,
    pub RCR: uint32_t,
    pub CCR1: uint32_t,
    pub CCR2: uint32_t,
    pub CCR3: uint32_t,
    pub CCR4: uint32_t,
    pub BDTR: uint32_t,
    pub DCR: uint32_t,
    pub DMAR: uint32_t,
    pub OR: uint32_t,
    pub CCMR3: uint32_t,
    pub CCR5: uint32_t,
    pub CCR6: uint32_t,
}
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
pub type pgn_t = uint16_t;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed_0 = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed_0 = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed_0 = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed_0 = 4095;
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_1,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_1 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
}
pub type pgRegistry_t = pgRegistry_s;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_2 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_2 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_2 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_2 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_2 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_2 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_2 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_2 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_2 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_2 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_2 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_2 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_2 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_2 = 8192;
pub const FEATURE_3D: C2RustUnnamed_2 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_2 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_2 = 512;
pub const FEATURE_GPS: C2RustUnnamed_2 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_2 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_2 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_2 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_2 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_2 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_2 = 1;
pub type ioTag_t = uint8_t;
pub type timerUsageFlag_e = libc::c_uint;
pub const TIM_USE_BEEPER: timerUsageFlag_e = 64;
pub const TIM_USE_TRANSPONDER: timerUsageFlag_e = 32;
pub const TIM_USE_LED: timerUsageFlag_e = 16;
pub const TIM_USE_SERVO: timerUsageFlag_e = 8;
pub const TIM_USE_MOTOR: timerUsageFlag_e = 4;
pub const TIM_USE_PWM: timerUsageFlag_e = 2;
pub const TIM_USE_PPM: timerUsageFlag_e = 1;
pub const TIM_USE_NONE: timerUsageFlag_e = 0;
pub const TIM_USE_ANY: timerUsageFlag_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerHardware_s {
    pub tim: *mut TIM_TypeDef,
    pub tag: ioTag_t,
    pub channel: uint8_t,
    pub usageFlags: timerUsageFlag_e,
    pub output: uint8_t,
    pub alternateFunction: uint8_t,
    pub dmaRef: *mut DMA_Stream_TypeDef,
    pub dmaChannel: uint32_t,
    pub dmaIrqHandler: uint8_t,
    pub dmaTimUPRef: *mut DMA_Stream_TypeDef,
    pub dmaTimUPChannel: uint32_t,
    pub dmaTimUPIrqHandler: uint8_t,
}
pub type timerHardware_t = timerHardware_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorDmaTimer_t {
    pub timer: *mut TIM_TypeDef,
    pub dmaBurstRef: *mut DMA_Stream_TypeDef,
    pub dmaBurstLength: uint16_t,
    pub dmaBurstBuffer: [uint32_t; 72],
    pub timerDmaSources: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorDmaOutput_t {
    pub ioTag: ioTag_t,
    pub timerHardware: *const timerHardware_t,
    pub value: uint16_t,
    pub timerDmaSource: uint16_t,
    pub configured: bool,
    pub timer: *mut motorDmaTimer_t,
    pub requestTelemetry: bool,
    pub dmaBuffer: [uint32_t; 18],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorDevConfig_s {
    pub motorPwmRate: uint16_t,
    pub motorPwmProtocol: uint8_t,
    pub motorPwmInversion: uint8_t,
    pub useUnsyncedPwm: uint8_t,
    pub useBurstDshot: uint8_t,
    pub ioTags: [ioTag_t; 8],
}
pub type motorDevConfig_t = motorDevConfig_s;
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
// TIMUP
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
pub type timeMs_t = uint32_t;
pub type timeUs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct escSensorConfig_s {
    pub halfDuplex: uint8_t,
    pub offset: uint16_t,
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
pub type escSensorConfig_t = escSensorConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct escSensorData_t {
    pub dataAge: uint8_t,
    pub temperature: int8_t,
    pub voltage: int16_t,
    pub current: int32_t,
    pub consumption: int32_t,
    pub rpm: int16_t,
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
// Set to false to listen on the TX pin for telemetry data
// offset consumed by the flight controller / VTX / cam / ... in milliampere
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
pub const DEBUG_ESC_NUM_TIMEOUTS: C2RustUnnamed_3 = 1;
pub type escSensorTriggerState_t = libc::c_uint;
pub const ESC_SENSOR_TRIGGER_PENDING: escSensorTriggerState_t = 2;
pub const ESC_SENSOR_TRIGGER_READY: escSensorTriggerState_t = 1;
pub const ESC_SENSOR_TRIGGER_STARTUP: escSensorTriggerState_t = 0;
pub const ESC_SENSOR_FRAME_PENDING: C2RustUnnamed_4 = 0;
pub const DEBUG_ESC_NUM_CRC_ERRORS: C2RustUnnamed_3 = 2;
pub const ESC_SENSOR_FRAME_FAILED: C2RustUnnamed_4 = 2;
pub const ESC_SENSOR_FRAME_COMPLETE: C2RustUnnamed_4 = 1;
pub type motorConfig_t = motorConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorConfig_s {
    pub dev: motorDevConfig_t,
    pub digitalIdleOffsetValue: uint16_t,
    pub minthrottle: uint16_t,
    pub maxthrottle: uint16_t,
    pub mincommand: uint16_t,
    pub motorPoleCount: uint8_t,
}
pub const DEBUG_ESC_MOTOR_INDEX: C2RustUnnamed_3 = 0;
pub const DEBUG_ESC_DATA_AGE: C2RustUnnamed_3 = 3;
// not used for all telemetry systems, e.g. HoTT only works at 19200.
// Idle value for DShot protocol, full motor output = 10000
// Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
// This is the maximum value for the ESCs at full power this value can be increased up to 2000
// This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
// Magnetic poles in the motors for calculating actual RPM from eRPM provided by ESC telemetry
/*
DEBUG INFORMATION
-----------------

set debug_mode = DEBUG_ESC_SENSOR in cli

*/
pub type C2RustUnnamed_3 = libc::c_uint;
pub type C2RustUnnamed_4 = libc::c_uint;
#[inline]
unsafe extern "C" fn escSensorConfig() -> *const escSensorConfig_t {
    return &mut escSensorConfig_System;
}
#[inline]
unsafe extern "C" fn motorConfig() -> *const motorConfig_t {
    return &mut motorConfig_System;
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
/*
KISS ESC TELEMETRY PROTOCOL
---------------------------

One transmission will have 10 times 8-bit bytes sent with 115200 baud and 3.6V.

Byte 0: Temperature
Byte 1: Voltage high byte
Byte 2: Voltage low byte
Byte 3: Current high byte
Byte 4: Current low byte
Byte 5: Consumption high byte
Byte 6: Consumption low byte
Byte 7: Rpm high byte
Byte 8: Rpm low byte
Byte 9: 8-bit CRC

*/
#[no_mangle]
pub static mut escSensorConfig_System: escSensorConfig_t =
    escSensorConfig_t{halfDuplex: 0, offset: 0,};
#[no_mangle]
pub static mut escSensorConfig_Copy: escSensorConfig_t =
    escSensorConfig_t{halfDuplex: 0, offset: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut escSensorConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (517 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<escSensorConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &escSensorConfig_System as
                                     *const escSensorConfig_t as
                                     *mut escSensorConfig_t as *mut uint8_t,
                             copy:
                                 &escSensorConfig_Copy as
                                     *const escSensorConfig_t as
                                     *mut escSensorConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{ptr:
                                                     &pgResetTemplate_escSensorConfig
                                                         as
                                                         *const escSensorConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_escSensorConfig: escSensorConfig_t =
    {
        let mut init =
            escSensorConfig_s{halfDuplex: 0 as libc::c_int as uint8_t,
                              offset: 0,};
        init
    };
static mut telemetryBuffer: [uint8_t; 10] =
    [0 as libc::c_int as uint8_t, 0, 0, 0, 0, 0, 0, 0, 0, 0];
static mut buffer: *mut uint8_t = 0 as *const uint8_t as *mut uint8_t;
static mut bufferSize: uint8_t = 0 as libc::c_int as uint8_t;
static mut bufferPosition: uint8_t = 0 as libc::c_int as uint8_t;
static mut escSensorPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
static mut escSensorData: [escSensorData_t; 8] =
    [escSensorData_t{dataAge: 0,
                     temperature: 0,
                     voltage: 0,
                     current: 0,
                     consumption: 0,
                     rpm: 0,}; 8];
static mut escSensorTriggerState: escSensorTriggerState_t =
    ESC_SENSOR_TRIGGER_STARTUP;
static mut escTriggerTimestamp: uint32_t = 0;
static mut escSensorMotor: uint8_t = 0 as libc::c_int as uint8_t;
// motor index
static mut combinedEscSensorData: escSensorData_t =
    escSensorData_t{dataAge: 0,
                    temperature: 0,
                    voltage: 0,
                    current: 0,
                    consumption: 0,
                    rpm: 0,};
static mut combinedDataNeedsUpdate: bool = 1 as libc::c_int != 0;
static mut totalTimeoutCount: uint16_t = 0 as libc::c_int as uint16_t;
static mut totalCrcErrorCount: uint16_t = 0 as libc::c_int as uint16_t;
#[no_mangle]
pub unsafe extern "C" fn startEscDataRead(mut frameBuffer: *mut uint8_t,
                                          mut frameLength: uint8_t) {
    buffer = frameBuffer as *mut uint8_t;
    ::core::ptr::write_volatile(&mut bufferPosition as *mut uint8_t,
                                0 as libc::c_int as uint8_t);
    ::core::ptr::write_volatile(&mut bufferSize as *mut uint8_t, frameLength);
}
#[no_mangle]
pub unsafe extern "C" fn getNumberEscBytesRead() -> uint8_t {
    return bufferPosition;
}
unsafe extern "C" fn isFrameComplete() -> bool {
    return bufferPosition as libc::c_int == bufferSize as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn isEscSensorActive() -> bool {
    return !escSensorPort.is_null();
}
#[no_mangle]
pub unsafe extern "C" fn getEscSensorData(mut motorNumber: uint8_t)
 -> *mut escSensorData_t {
    if !feature(FEATURE_ESC_SENSOR as libc::c_int as uint32_t) {
        return 0 as *mut escSensorData_t
    }
    if (motorNumber as libc::c_int) < getMotorCount() as libc::c_int {
        return &mut *escSensorData.as_mut_ptr().offset(motorNumber as isize)
                   as *mut escSensorData_t
    } else if motorNumber as libc::c_int == 255 as libc::c_int {
        if combinedDataNeedsUpdate {
            combinedEscSensorData.dataAge = 0 as libc::c_int as uint8_t;
            combinedEscSensorData.temperature = 0 as libc::c_int as int8_t;
            combinedEscSensorData.voltage = 0 as libc::c_int as int16_t;
            combinedEscSensorData.current = 0 as libc::c_int;
            combinedEscSensorData.consumption = 0 as libc::c_int;
            combinedEscSensorData.rpm = 0 as libc::c_int as int16_t;
            let mut i: libc::c_int = 0 as libc::c_int;
            while i < getMotorCount() as libc::c_int {
                combinedEscSensorData.dataAge =
                    ({
                         let mut _a: uint8_t = combinedEscSensorData.dataAge;
                         let mut _b: uint8_t =
                             escSensorData[i as usize].dataAge;
                         if _a as libc::c_int > _b as libc::c_int {
                             _a as libc::c_int
                         } else { _b as libc::c_int }
                     }) as uint8_t;
                combinedEscSensorData.temperature =
                    ({
                         let mut _a: int8_t =
                             combinedEscSensorData.temperature;
                         let mut _b: int8_t =
                             escSensorData[i as usize].temperature;
                         if _a as libc::c_int > _b as libc::c_int {
                             _a as libc::c_int
                         } else { _b as libc::c_int }
                     }) as int8_t;
                combinedEscSensorData.voltage =
                    (combinedEscSensorData.voltage as libc::c_int +
                         escSensorData[i as usize].voltage as libc::c_int) as
                        int16_t;
                combinedEscSensorData.current +=
                    escSensorData[i as usize].current;
                combinedEscSensorData.consumption +=
                    escSensorData[i as usize].consumption;
                combinedEscSensorData.rpm =
                    (combinedEscSensorData.rpm as libc::c_int +
                         escSensorData[i as usize].rpm as libc::c_int) as
                        int16_t;
                i = i + 1 as libc::c_int
            }
            combinedEscSensorData.voltage =
                (combinedEscSensorData.voltage as libc::c_int /
                     getMotorCount() as libc::c_int) as int16_t;
            combinedEscSensorData.rpm =
                (combinedEscSensorData.rpm as libc::c_int /
                     getMotorCount() as libc::c_int) as int16_t;
            combinedDataNeedsUpdate = 0 as libc::c_int != 0;
            if debugMode as libc::c_int == DEBUG_ESC_SENSOR as libc::c_int {
                debug[DEBUG_ESC_DATA_AGE as libc::c_int as usize] =
                    combinedEscSensorData.dataAge as int16_t
            }
        }
        return &mut combinedEscSensorData
    } else { return 0 as *mut escSensorData_t };
}
// Receive ISR callback
unsafe extern "C" fn escSensorDataReceive(mut c: uint16_t,
                                          mut data: *mut libc::c_void) {
    // KISS ESC sends some data during startup, ignore this for now (maybe future use)
    // startup data could be firmware version and serialnumber
    if isFrameComplete() { return }
    let fresh0 =
        ::core::ptr::read_volatile::<uint8_t>(&bufferPosition as
                                                  *const uint8_t);
    ::core::ptr::write_volatile(&mut bufferPosition as *mut uint8_t,
                                ::core::ptr::read_volatile::<uint8_t>(&bufferPosition
                                                                          as
                                                                          *const uint8_t).wrapping_add(1));
    ::core::ptr::write_volatile(buffer.offset(fresh0 as isize), c as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn escSensorInit() -> bool {
    let mut portConfig: *mut serialPortConfig_t =
        findSerialPortConfig(FUNCTION_ESC_SENSOR);
    if portConfig.is_null() { return 0 as libc::c_int != 0 }
    let mut options: portOptions_e =
        (SERIAL_NOT_INVERTED as libc::c_int |
             (if (*escSensorConfig()).halfDuplex as libc::c_int != 0 {
                  SERIAL_BIDIR as libc::c_int
              } else { 0 as libc::c_int })) as portOptions_e;
    // Initialize serial port
    escSensorPort =
        openSerialPort((*portConfig).identifier, FUNCTION_ESC_SENSOR,
                       Some(escSensorDataReceive as
                                unsafe extern "C" fn(_: uint16_t,
                                                     _: *mut libc::c_void)
                                    -> ()), 0 as *mut libc::c_void,
                       115200 as libc::c_int as uint32_t, MODE_RX, options);
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 8 as libc::c_int {
        escSensorData[i as usize].dataAge = 255 as libc::c_int as uint8_t;
        i = i + 1 as libc::c_int
    }
    return !escSensorPort.is_null();
}
unsafe extern "C" fn updateCrc8(mut crc: uint8_t, mut crc_seed: uint8_t)
 -> uint8_t {
    let mut crc_u: uint8_t = crc;
    crc_u = (crc_u as libc::c_int ^ crc_seed as libc::c_int) as uint8_t;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 8 as libc::c_int {
        crc_u =
            if crc_u as libc::c_int & 0x80 as libc::c_int != 0 {
                (0x7 as libc::c_int) ^
                    (crc_u as libc::c_int) << 1 as libc::c_int
            } else { ((crc_u as libc::c_int)) << 1 as libc::c_int } as
                uint8_t;
        i += 1
    }
    return crc_u;
}
#[no_mangle]
pub unsafe extern "C" fn calculateCrc8(mut Buf: *const uint8_t,
                                       BufLen: uint8_t) -> uint8_t {
    let mut crc: uint8_t = 0 as libc::c_int as uint8_t;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < BufLen as libc::c_int {
        crc = updateCrc8(*Buf.offset(i as isize), crc);
        i += 1
    }
    return crc;
}
unsafe extern "C" fn decodeEscFrame() -> uint8_t {
    if !isFrameComplete() {
        return ESC_SENSOR_FRAME_PENDING as libc::c_int as uint8_t
    }
    // Get CRC8 checksum
    let mut chksum: uint16_t =
        calculateCrc8(telemetryBuffer.as_mut_ptr(),
                      (10 as libc::c_int - 1 as libc::c_int) as uint8_t) as
            uint16_t; // last byte contains CRC value
    let mut tlmsum: uint16_t =
        telemetryBuffer[(10 as libc::c_int - 1 as libc::c_int) as usize] as
            uint16_t;
    let mut frameStatus: uint8_t = 0;
    if chksum as libc::c_int == tlmsum as libc::c_int {
        escSensorData[escSensorMotor as usize].dataAge =
            0 as libc::c_int as uint8_t;
        escSensorData[escSensorMotor as usize].temperature =
            telemetryBuffer[0 as libc::c_int as usize] as int8_t;
        escSensorData[escSensorMotor as usize].voltage =
            ((telemetryBuffer[1 as libc::c_int as usize] as libc::c_int) <<
                 8 as libc::c_int |
                 telemetryBuffer[2 as libc::c_int as usize] as libc::c_int) as
                int16_t;
        escSensorData[escSensorMotor as usize].current =
            (telemetryBuffer[3 as libc::c_int as usize] as libc::c_int) <<
                8 as libc::c_int |
                telemetryBuffer[4 as libc::c_int as usize] as libc::c_int;
        escSensorData[escSensorMotor as usize].consumption =
            (telemetryBuffer[5 as libc::c_int as usize] as libc::c_int) <<
                8 as libc::c_int |
                telemetryBuffer[6 as libc::c_int as usize] as libc::c_int;
        escSensorData[escSensorMotor as usize].rpm =
            ((telemetryBuffer[7 as libc::c_int as usize] as libc::c_int) <<
                 8 as libc::c_int |
                 telemetryBuffer[8 as libc::c_int as usize] as libc::c_int) as
                int16_t;
        combinedDataNeedsUpdate = 1 as libc::c_int != 0;
        frameStatus = ESC_SENSOR_FRAME_COMPLETE as libc::c_int as uint8_t;
        if debugMode as libc::c_int == DEBUG_ESC_SENSOR_RPM as libc::c_int {
            debug[escSensorMotor as usize] =
                (calcEscRpm(escSensorData[escSensorMotor as usize].rpm as
                                libc::c_int) / 10 as libc::c_int) as int16_t
        }
        // output actual rpm/10 to fit in 16bit signed.
        if debugMode as libc::c_int == DEBUG_ESC_SENSOR_TMP as libc::c_int {
            debug[escSensorMotor as usize] =
                escSensorData[escSensorMotor as usize].temperature as int16_t
        }
    } else { frameStatus = ESC_SENSOR_FRAME_FAILED as libc::c_int as uint8_t }
    return frameStatus;
}
unsafe extern "C" fn increaseDataAge() {
    if (escSensorData[escSensorMotor as usize].dataAge as libc::c_int) <
           255 as libc::c_int {
        escSensorData[escSensorMotor as usize].dataAge =
            escSensorData[escSensorMotor as usize].dataAge.wrapping_add(1);
        combinedDataNeedsUpdate = 1 as libc::c_int != 0
    };
}
unsafe extern "C" fn selectNextMotor() {
    escSensorMotor = escSensorMotor.wrapping_add(1);
    if escSensorMotor as libc::c_int == getMotorCount() as libc::c_int {
        escSensorMotor = 0 as libc::c_int as uint8_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn escSensorProcess(mut currentTimeUs: timeUs_t) {
    let currentTimeMs: timeMs_t =
        currentTimeUs.wrapping_div(1000 as libc::c_int as libc::c_uint);
    if escSensorPort.is_null() || !pwmAreMotorsEnabled() { return }
    let mut motor: *mut motorDmaOutput_t = 0 as *mut motorDmaOutput_t;
    match escSensorTriggerState as libc::c_uint {
        0 => {
            // Wait period of time before requesting telemetry (let the system boot first)
            if currentTimeMs >= 5000 as libc::c_int as libc::c_uint {
                escSensorTriggerState = ESC_SENSOR_TRIGGER_READY
            }
        }
        1 => {
            escTriggerTimestamp = currentTimeMs;
            startEscDataRead(telemetryBuffer.as_mut_ptr(),
                             10 as libc::c_int as uint8_t);
            motor = getMotorDmaOutput(escSensorMotor);
            ::core::ptr::write_volatile(&mut (*motor).requestTelemetry as
                                            *mut bool, 1 as libc::c_int != 0);
            escSensorTriggerState = ESC_SENSOR_TRIGGER_PENDING;
            if debugMode as libc::c_int == DEBUG_ESC_SENSOR as libc::c_int {
                debug[DEBUG_ESC_MOTOR_INDEX as libc::c_int as usize] =
                    (escSensorMotor as libc::c_int + 1 as libc::c_int) as
                        int16_t
            }
        }
        2 => {
            if currentTimeMs <
                   escTriggerTimestamp.wrapping_add(100 as libc::c_int as
                                                        libc::c_uint) {
                let mut state: uint8_t = decodeEscFrame();
                match state as libc::c_int {
                    1 => {
                        selectNextMotor();
                        escSensorTriggerState = ESC_SENSOR_TRIGGER_READY
                    }
                    2 => {
                        increaseDataAge();
                        selectNextMotor();
                        escSensorTriggerState = ESC_SENSOR_TRIGGER_READY;
                        if debugMode as libc::c_int ==
                               DEBUG_ESC_SENSOR as libc::c_int {
                            totalCrcErrorCount =
                                totalCrcErrorCount.wrapping_add(1);
                            debug[DEBUG_ESC_NUM_CRC_ERRORS as libc::c_int as
                                      usize] = totalCrcErrorCount as int16_t
                        }
                    }
                    0 | _ => { }
                }
            } else {
                // Move on to next ESC, we'll come back to this one
                increaseDataAge();
                selectNextMotor();
                escSensorTriggerState = ESC_SENSOR_TRIGGER_READY;
                if debugMode as libc::c_int == DEBUG_ESC_SENSOR as libc::c_int
                   {
                    totalTimeoutCount = totalTimeoutCount.wrapping_add(1);
                    debug[DEBUG_ESC_NUM_TIMEOUTS as libc::c_int as usize] =
                        totalTimeoutCount as int16_t
                }
            }
        }
        _ => { }
    };
}
#[no_mangle]
pub unsafe extern "C" fn calcEscRpm(mut erpm: libc::c_int) -> libc::c_int {
    return erpm * 100 as libc::c_int /
               ((*motorConfig()).motorPoleCount as libc::c_int /
                    2 as libc::c_int);
}
