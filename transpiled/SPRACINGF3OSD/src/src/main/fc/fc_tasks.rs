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
    fn usbVcpIsConnected() -> uint8_t;
    #[no_mangle]
    fn usbCableIsInserted() -> bool;
    #[no_mangle]
    static mut armingFlags: uint8_t;
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
    // return positive for ACK, negative on error, zero for no reply
    // don't know how to process command, try next handler
    // msp post process function, used for gracefully handling reboots, etc.
    #[no_mangle]
    fn mspFcProcessCommand(cmd: *mut mspPacket_t, reply: *mut mspPacket_t,
                           mspPostProcessFn: *mut mspPostProcessFnPtr)
     -> mspResult_e;
    #[no_mangle]
    fn mspFcProcessReply(reply: *mut mspPacket_t);
    #[no_mangle]
    fn osdSlaveInitialized() -> bool;
    // task api
    #[no_mangle]
    fn osdSlaveCheck(currentTimeUs: timeUs_t, currentDeltaTimeUs: timeDelta_t)
     -> bool;
    #[no_mangle]
    fn osdSlaveUpdate(currentTimeUs: timeUs_t);
    #[no_mangle]
    static mut serialConfig_System: serialConfig_t;
    #[no_mangle]
    fn transponderUpdate(currentTimeUs: timeUs_t);
    #[no_mangle]
    fn mspSerialWaiting() -> bool;
    #[no_mangle]
    fn mspSerialProcess(evaluateNonMspData: mspEvaluateNonMspData_e,
                        mspProcessCommandFn: mspProcessCommandFnPtr,
                        mspProcessReplyFn: mspProcessReplyFnPtr);
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
    // voltage
    // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    // minimum voltage per cell, this triggers battery critical alarm, in 0.1V units, default is 33 (3.3V)
    // warning voltage per cell, this triggers battery warning alarm, in 0.1V units, default is 35 (3.5V)
    // Between vbatmaxcellvoltage and 2*this is considered to be USB powered. Below this it is notpresent
    // Percentage of throttle when lvc is triggered
    // source of battery voltage meter used, either ADC or ESC
    // current
    // source of battery current meter used, either ADC, Virtual or ESC
    // mAh
    // warnings / alerts
    // Issue alerts based on VBat readings
    // Issue alerts based on total power consumption
    // Percentage of remaining capacity that should trigger a battery warning
    // hysteresis for alarm, default 1 = 0.1V
    // Cell voltage at which the battery is deemed to be "full" 0.1V units, default is 41 (4.1V)
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    fn batteryUpdateCurrentMeter(currentTimeUs: timeUs_t);
    #[no_mangle]
    fn batteryUpdateVoltage(currentTimeUs: timeUs_t);
    #[no_mangle]
    fn batteryUpdatePresence();
    #[no_mangle]
    fn batteryUpdateStates(currentTimeUs: timeUs_t);
    #[no_mangle]
    fn batteryUpdateAlarms();
    #[no_mangle]
    fn rescheduleTask(taskId: cfTaskId_e, newPeriodMicros: uint32_t);
    #[no_mangle]
    fn setTaskEnabled(taskId: cfTaskId_e, newEnabledState: bool);
    #[no_mangle]
    fn schedulerInit();
    #[no_mangle]
    fn taskSystemLoad(currentTime: timeUs_t);
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
// CMS state
// displayPort_t is used as a parameter group in 'displayport_msp.h' and 'displayport_max7456`.h'. Treat accordingly!
#[derive(Copy, Clone)]
#[repr(C)]
pub struct displayPortVTable_s {
    pub grab: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                         -> libc::c_int>,
    pub release: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                            -> libc::c_int>,
    pub clearScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                                -> libc::c_int>,
    pub drawScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                               -> libc::c_int>,
    pub screenSize: Option<unsafe extern "C" fn(_: *const displayPort_t)
                               -> libc::c_int>,
    pub writeString: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                                 _: uint8_t, _: uint8_t,
                                                 _: *const libc::c_char)
                                -> libc::c_int>,
    pub writeChar: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                               _: uint8_t, _: uint8_t,
                                               _: uint8_t) -> libc::c_int>,
    pub isTransferInProgress: Option<unsafe extern "C" fn(_:
                                                              *const displayPort_t)
                                         -> bool>,
    pub heartbeat: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                              -> libc::c_int>,
    pub resync: Option<unsafe extern "C" fn(_: *mut displayPort_t) -> ()>,
    pub isSynced: Option<unsafe extern "C" fn(_: *const displayPort_t)
                             -> bool>,
    pub txBytesFree: Option<unsafe extern "C" fn(_: *const displayPort_t)
                                -> uint32_t>,
}
pub type displayPort_t = displayPort_s;
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
pub struct displayPort_s {
    pub vTable: *const displayPortVTable_s,
    pub device: *mut libc::c_void,
    pub rows: uint8_t,
    pub cols: uint8_t,
    pub posX: uint8_t,
    pub posY: uint8_t,
    pub cleared: bool,
    pub cursorRow: int8_t,
    pub grabCount: int8_t,
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
// time difference, 32 bits always sufficient
pub type timeDelta_t = int32_t;
// microsecond time
pub type timeUs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct hsvColor_s {
    pub h: uint16_t,
    pub s: uint8_t,
    pub v: uint8_t,
}
pub type hsvColor_t = hsvColor_s;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_0 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_0 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_0 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_0 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_0 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_0 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_0 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_0 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_0 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_0 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_0 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_0 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_0 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_0 = 8192;
pub const FEATURE_3D: C2RustUnnamed_0 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_0 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_0 = 512;
pub const FEATURE_GPS: C2RustUnnamed_0 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_0 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_0 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_0 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_0 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_0 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_0 = 1;
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
// 0 - 359
// 0 - 255
// 0 - 255
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
pub type C2RustUnnamed_1 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_1 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_1 = 2;
pub const ARMED: C2RustUnnamed_1 = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialConfig_s {
    pub portConfigs: [serialPortConfig_t; 3],
    pub serial_update_rate_hz: uint16_t,
    pub reboot_character: uint8_t,
}
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
pub struct sbuf_s {
    pub ptr: *mut uint8_t,
    pub end: *mut uint8_t,
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
// simple buffer-based serializer/deserializer without implicit size check
pub type sbuf_t = sbuf_s;
pub type mspResult_e = libc::c_int;
pub const MSP_RESULT_CMD_UNKNOWN: mspResult_e = -2;
pub const MSP_RESULT_NO_REPLY: mspResult_e = 0;
pub const MSP_RESULT_ERROR: mspResult_e = -1;
pub const MSP_RESULT_ACK: mspResult_e = 1;
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
pub type mspPostProcessFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut serialPort_s) -> ()>;
pub type mspProcessCommandFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut mspPacket_t, _: *mut mspPacket_t,
                                _: *mut mspPostProcessFnPtr) -> mspResult_e>;
pub type mspProcessReplyFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut mspPacket_t) -> ()>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct modeColorIndexes_s {
    pub color: [uint8_t; 6],
}
pub type modeColorIndexes_t = modeColorIndexes_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct specialColorIndexes_s {
    pub color: [uint8_t; 11],
}
pub type specialColorIndexes_t = specialColorIndexes_s;
pub type serialConfig_t = serialConfig_s;
// data pointer must be first (sbuf_t* is equivalent to uint8_t **)
// which byte is used to reboot. Default 'R', could be changed carefully to something else.
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
pub struct rcdeviceSwitchState_s {
    pub isActivated: bool,
}
pub type rcdeviceSwitchState_t = rcdeviceSwitchState_s;
pub type mspEvaluateNonMspData_e = libc::c_uint;
pub const MSP_SKIP_NON_MSP_DATA: mspEvaluateNonMspData_e = 1;
pub const MSP_EVALUATE_NON_MSP_DATA: mspEvaluateNonMspData_e = 0;
pub type currentMeterSource_e = libc::c_uint;
pub const CURRENT_METER_COUNT: currentMeterSource_e = 5;
pub const CURRENT_METER_MSP: currentMeterSource_e = 4;
pub const CURRENT_METER_ESC: currentMeterSource_e = 3;
pub const CURRENT_METER_VIRTUAL: currentMeterSource_e = 2;
pub const CURRENT_METER_ADC: currentMeterSource_e = 1;
pub const CURRENT_METER_NONE: currentMeterSource_e = 0;
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
//
// meters
//
pub type voltageMeterSource_e = libc::c_uint;
pub const VOLTAGE_METER_COUNT: voltageMeterSource_e = 3;
pub const VOLTAGE_METER_ESC: voltageMeterSource_e = 2;
pub const VOLTAGE_METER_ADC: voltageMeterSource_e = 1;
pub const VOLTAGE_METER_NONE: voltageMeterSource_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct batteryConfig_s {
    pub vbatmaxcellvoltage: uint8_t,
    pub vbatmincellvoltage: uint8_t,
    pub vbatwarningcellvoltage: uint8_t,
    pub vbatnotpresentcellvoltage: uint8_t,
    pub lvcPercentage: uint8_t,
    pub voltageMeterSource: voltageMeterSource_e,
    pub currentMeterSource: currentMeterSource_e,
    pub batteryCapacity: uint16_t,
    pub useVBatAlerts: bool,
    pub useConsumptionAlerts: bool,
    pub consumptionWarningPercentage: uint8_t,
    pub vbathysteresis: uint8_t,
    pub vbatfullcellvoltage: uint8_t,
}
pub type batteryConfig_t = batteryConfig_s;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const TASK_PRIORITY_MAX: C2RustUnnamed_2 = 255;
pub const TASK_PRIORITY_REALTIME: C2RustUnnamed_2 = 6;
pub const TASK_PRIORITY_HIGH: C2RustUnnamed_2 = 5;
pub const TASK_PRIORITY_MEDIUM_HIGH: C2RustUnnamed_2 = 4;
pub const TASK_PRIORITY_MEDIUM: C2RustUnnamed_2 = 3;
pub const TASK_PRIORITY_LOW: C2RustUnnamed_2 = 1;
pub const TASK_PRIORITY_IDLE: C2RustUnnamed_2 = 0;
pub type cfTaskId_e = libc::c_uint;
pub const TASK_SELF: cfTaskId_e = 14;
pub const TASK_NONE: cfTaskId_e = 13;
pub const TASK_COUNT: cfTaskId_e = 13;
pub const TASK_OSD_SLAVE: cfTaskId_e = 12;
pub const TASK_TRANSPONDER: cfTaskId_e = 11;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct cfTask_t {
    pub taskName: *const libc::c_char,
    pub subTaskName: *const libc::c_char,
    pub checkFunc: Option<unsafe extern "C" fn(_: timeUs_t, _: timeDelta_t)
                              -> bool>,
    pub taskFunc: Option<unsafe extern "C" fn(_: timeUs_t) -> ()>,
    pub desiredPeriod: timeDelta_t,
    pub staticPriority: uint8_t,
    pub dynamicPriority: uint16_t,
    pub taskAgeCycles: uint16_t,
    pub taskLatestDeltaTime: timeDelta_t,
    pub lastExecutedAt: timeUs_t,
    pub lastSignaledAt: timeUs_t,
    pub movingSumExecutionTime: timeUs_t,
    pub maxExecutionTime: timeUs_t,
    pub totalExecutionTime: timeUs_t,
}
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[no_mangle]
pub static mut colors: *mut hsvColor_t =
    0 as *const hsvColor_t as *mut hsvColor_t;
#[no_mangle]
pub static mut modeColors: *const modeColorIndexes_t =
    0 as *const modeColorIndexes_t;
#[no_mangle]
pub static mut specialColors: specialColorIndexes_t =
    specialColorIndexes_t{color: [0; 11],};
#[inline]
unsafe extern "C" fn serialConfig() -> *const serialConfig_t {
    return &mut serialConfig_System;
}
// used for unit test
#[no_mangle]
pub static mut switchStates: [rcdeviceSwitchState_t; 3] =
    [rcdeviceSwitchState_t{isActivated: false,}; 3];
#[inline]
unsafe extern "C" fn batteryConfig() -> *const batteryConfig_t {
    return &mut batteryConfig_System;
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
unsafe extern "C" fn taskMain(mut currentTimeUs: timeUs_t) { }
unsafe extern "C" fn taskSerialCheck(mut currentTimeUs: timeUs_t,
                                     mut currentDeltaTimeUs: timeDelta_t)
 -> bool {
    return mspSerialWaiting();
}
unsafe extern "C" fn taskHandleSerial(mut currentTimeUs: timeUs_t) {
    if debugMode as libc::c_int == DEBUG_USB as libc::c_int {
        debug[0 as libc::c_int as usize] = usbCableIsInserted() as int16_t
    }
    if debugMode as libc::c_int == DEBUG_USB as libc::c_int {
        debug[1 as libc::c_int as usize] = usbVcpIsConnected() as int16_t
    }
    let mut evaluateMspData: bool =
        if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
            MSP_SKIP_NON_MSP_DATA as libc::c_int
        } else { MSP_EVALUATE_NON_MSP_DATA as libc::c_int } != 0;
    mspSerialProcess(evaluateMspData as mspEvaluateNonMspData_e,
                     Some(mspFcProcessCommand as
                              unsafe extern "C" fn(_: *mut mspPacket_t,
                                                   _: *mut mspPacket_t,
                                                   _:
                                                       *mut mspPostProcessFnPtr)
                                  -> mspResult_e),
                     Some(mspFcProcessReply as
                              unsafe extern "C" fn(_: *mut mspPacket_t)
                                  -> ()));
}
unsafe extern "C" fn taskBatteryAlerts(mut currentTimeUs: timeUs_t) {
    if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
        // the battery *might* fall out in flight, but if that happens the FC will likely be off too unless the user has battery backup.
        batteryUpdatePresence();
    }
    batteryUpdateStates(currentTimeUs);
    batteryUpdateAlarms();
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
// Prevent too long busy wait times
// USE_BARO || USE_GPS
#[no_mangle]
pub unsafe extern "C" fn fcTasksInit() {
    schedulerInit();
    setTaskEnabled(TASK_MAIN, 1 as libc::c_int != 0);
    setTaskEnabled(TASK_SERIAL, 1 as libc::c_int != 0);
    rescheduleTask(TASK_SERIAL,
                   (1000000 as libc::c_int /
                        (*serialConfig()).serial_update_rate_hz as
                            libc::c_int) as uint32_t);
    let useBatteryVoltage: bool =
        (*batteryConfig()).voltageMeterSource as libc::c_uint !=
            VOLTAGE_METER_NONE as libc::c_int as libc::c_uint;
    setTaskEnabled(TASK_BATTERY_VOLTAGE, useBatteryVoltage);
    let useBatteryCurrent: bool =
        (*batteryConfig()).currentMeterSource as libc::c_uint !=
            CURRENT_METER_NONE as libc::c_int as libc::c_uint;
    setTaskEnabled(TASK_BATTERY_CURRENT, useBatteryCurrent);
    let useBatteryAlerts: bool =
        (*batteryConfig()).useVBatAlerts as libc::c_int != 0 ||
            (*batteryConfig()).useConsumptionAlerts as libc::c_int != 0;
    setTaskEnabled(TASK_BATTERY_ALERTS,
                   (useBatteryVoltage as libc::c_int != 0 ||
                        useBatteryCurrent as libc::c_int != 0) &&
                       useBatteryAlerts as libc::c_int != 0);
    setTaskEnabled(TASK_TRANSPONDER,
                   feature(FEATURE_TRANSPONDER as libc::c_int as uint32_t));
    setTaskEnabled(TASK_OSD_SLAVE, osdSlaveInitialized());
}
#[no_mangle]
pub static mut cfTasks: [cfTask_t; 13] =
    unsafe {
        [{
             let mut init =
                 cfTask_t{taskName:
                              b"SYSTEM\x00" as *const u8 as
                                  *const libc::c_char,
                          subTaskName:
                              b"LOAD\x00" as *const u8 as *const libc::c_char,
                          checkFunc: None,
                          taskFunc:
                              Some(taskSystemLoad as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 10 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_MEDIUM_HIGH as libc::c_int as
                                  uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         {
             let mut init =
                 cfTask_t{taskName:
                              b"SYSTEM\x00" as *const u8 as
                                  *const libc::c_char,
                          subTaskName:
                              b"UPDATE\x00" as *const u8 as
                                  *const libc::c_char,
                          checkFunc: None,
                          taskFunc:
                              Some(taskMain as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 1000 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_MEDIUM_HIGH as libc::c_int as
                                  uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         cfTask_t{taskName: 0 as *const libc::c_char,
                  subTaskName: 0 as *const libc::c_char,
                  checkFunc: None,
                  taskFunc: None,
                  desiredPeriod: 0,
                  staticPriority: 0,
                  dynamicPriority: 0,
                  taskAgeCycles: 0,
                  taskLatestDeltaTime: 0,
                  lastExecutedAt: 0,
                  lastSignaledAt: 0,
                  movingSumExecutionTime: 0,
                  maxExecutionTime: 0,
                  totalExecutionTime: 0,},
         cfTask_t{taskName: 0 as *const libc::c_char,
                  subTaskName: 0 as *const libc::c_char,
                  checkFunc: None,
                  taskFunc: None,
                  desiredPeriod: 0,
                  staticPriority: 0,
                  dynamicPriority: 0,
                  taskAgeCycles: 0,
                  taskLatestDeltaTime: 0,
                  lastExecutedAt: 0,
                  lastSignaledAt: 0,
                  movingSumExecutionTime: 0,
                  maxExecutionTime: 0,
                  totalExecutionTime: 0,},
         cfTask_t{taskName: 0 as *const libc::c_char,
                  subTaskName: 0 as *const libc::c_char,
                  checkFunc: None,
                  taskFunc: None,
                  desiredPeriod: 0,
                  staticPriority: 0,
                  dynamicPriority: 0,
                  taskAgeCycles: 0,
                  taskLatestDeltaTime: 0,
                  lastExecutedAt: 0,
                  lastSignaledAt: 0,
                  movingSumExecutionTime: 0,
                  maxExecutionTime: 0,
                  totalExecutionTime: 0,},
         cfTask_t{taskName: 0 as *const libc::c_char,
                  subTaskName: 0 as *const libc::c_char,
                  checkFunc: None,
                  taskFunc: None,
                  desiredPeriod: 0,
                  staticPriority: 0,
                  dynamicPriority: 0,
                  taskAgeCycles: 0,
                  taskLatestDeltaTime: 0,
                  lastExecutedAt: 0,
                  lastSignaledAt: 0,
                  movingSumExecutionTime: 0,
                  maxExecutionTime: 0,
                  totalExecutionTime: 0,},
         {
             let mut init =
                 cfTask_t{taskName:
                              b"SERIAL\x00" as *const u8 as
                                  *const libc::c_char,
                          subTaskName: 0 as *const libc::c_char,
                          checkFunc:
                              Some(taskSerialCheck as
                                       unsafe extern "C" fn(_: timeUs_t,
                                                            _: timeDelta_t)
                                           -> bool),
                          taskFunc:
                              Some(taskHandleSerial as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 100 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_REALTIME as libc::c_int as
                                  uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         cfTask_t{taskName: 0 as *const libc::c_char,
                  subTaskName: 0 as *const libc::c_char,
                  checkFunc: None,
                  taskFunc: None,
                  desiredPeriod: 0,
                  staticPriority: 0,
                  dynamicPriority: 0,
                  taskAgeCycles: 0,
                  taskLatestDeltaTime: 0,
                  lastExecutedAt: 0,
                  lastSignaledAt: 0,
                  movingSumExecutionTime: 0,
                  maxExecutionTime: 0,
                  totalExecutionTime: 0,},
         {
             let mut init =
                 cfTask_t{taskName:
                              b"BATTERY_VOLTAGE\x00" as *const u8 as
                                  *const libc::c_char,
                          subTaskName: 0 as *const libc::c_char,
                          checkFunc: None,
                          taskFunc:
                              Some(batteryUpdateVoltage as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 50 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_MEDIUM as libc::c_int as uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         {
             let mut init =
                 cfTask_t{taskName:
                              b"BATTERY_CURRENT\x00" as *const u8 as
                                  *const libc::c_char,
                          subTaskName: 0 as *const libc::c_char,
                          checkFunc: None,
                          taskFunc:
                              Some(batteryUpdateCurrentMeter as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 50 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_MEDIUM as libc::c_int as uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         {
             let mut init =
                 cfTask_t{taskName:
                              b"BATTERY_ALERTS\x00" as *const u8 as
                                  *const libc::c_char,
                          subTaskName: 0 as *const libc::c_char,
                          checkFunc: None,
                          taskFunc:
                              Some(taskBatteryAlerts as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 5 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_MEDIUM as libc::c_int as uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         {
             let mut init =
                 cfTask_t{taskName:
                              b"TRANSPONDER\x00" as *const u8 as
                                  *const libc::c_char,
                          subTaskName: 0 as *const libc::c_char,
                          checkFunc: None,
                          taskFunc:
                              Some(transponderUpdate as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 250 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_LOW as libc::c_int as uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         {
             let mut init =
                 cfTask_t{taskName:
                              b"OSD_SLAVE\x00" as *const u8 as
                                  *const libc::c_char,
                          subTaskName: 0 as *const libc::c_char,
                          checkFunc:
                              Some(osdSlaveCheck as
                                       unsafe extern "C" fn(_: timeUs_t,
                                                            _: timeDelta_t)
                                           -> bool),
                          taskFunc:
                              Some(osdSlaveUpdate as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 60 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_HIGH as libc::c_int as uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         }]
    };
