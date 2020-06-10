use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
    #[no_mangle]
    static mut debug: [int16_t; 4];
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
    // increment when a major release is made (big new feature, etc)
    // increment when a minor release is made (small new feature, change etc)
    // increment when a bug is fixed
    // lower case hexadecimal digits.
    #[no_mangle]
    static shortGitRevision: *const libc::c_char;
    #[no_mangle]
    static buildDate: *const libc::c_char;
    // "MMM DD YYYY" MMM = Jan/Feb/...
    #[no_mangle]
    static buildTime: *const libc::c_char;
    #[no_mangle]
    static targetName: *const libc::c_char;
    #[no_mangle]
    fn sbufWriteU8(dst: *mut sbuf_t, val: uint8_t);
    #[no_mangle]
    fn sbufWriteU16(dst: *mut sbuf_t, val: uint16_t);
    #[no_mangle]
    fn sbufWriteU32(dst: *mut sbuf_t, val: uint32_t);
    #[no_mangle]
    fn sbufWriteData(dst: *mut sbuf_t, data: *const libc::c_void,
                     len: libc::c_int);
    #[no_mangle]
    fn sbufReadU8(src: *mut sbuf_t) -> uint8_t;
    #[no_mangle]
    fn sbufReadU16(src: *mut sbuf_t) -> uint16_t;
    #[no_mangle]
    fn sbufReadData(dst: *mut sbuf_t, data: *mut libc::c_void,
                    len: libc::c_int);
    #[no_mangle]
    fn sbufBytesRemaining(buf: *mut sbuf_t) -> libc::c_int;
    #[no_mangle]
    fn featureMask() -> uint32_t;
    #[no_mangle]
    fn i2cGetErrorCounter() -> uint16_t;
    #[no_mangle]
    fn max7456WriteNvm(char_address: uint8_t, font_data: *const uint8_t);
    // bootloader/IAP
    #[no_mangle]
    fn systemReset();
    #[no_mangle]
    fn systemResetToBootloader();
    #[no_mangle]
    static mut systemConfig_System: systemConfig_t;
    #[no_mangle]
    fn resetEEPROM();
    #[no_mangle]
    fn readEEPROM() -> bool;
    #[no_mangle]
    fn writeEEPROM();
    #[no_mangle]
    static mut osdSlaveIsLocked: bool;
    // msp api
    #[no_mangle]
    fn osdSlaveHeartbeat();
    #[no_mangle]
    fn osdSlaveClearScreen();
    #[no_mangle]
    fn osdSlaveWrite(x: uint8_t, y: uint8_t, s: *const libc::c_char);
    #[no_mangle]
    fn osdSlaveDrawScreen();
    #[no_mangle]
    static mut transponderConfig_System: transponderConfig_t;
    #[no_mangle]
    static transponderRequirements: [transponderRequirement_t; 3];
    #[no_mangle]
    fn transponderUpdateData();
    #[no_mangle]
    fn transponderStopRepeating();
    #[no_mangle]
    static mut vcdProfile_System: vcdProfile_t;
    #[no_mangle]
    static mut averageSystemLoadPercent: uint16_t;
    #[no_mangle]
    fn getTaskDeltaTime(taskId: cfTaskId_e) -> timeDelta_t;
    #[no_mangle]
    static mut currentSensorADCConfig_System: currentSensorADCConfig_t;
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    static mut voltageSensorADCConfig_SystemArray:
           [voltageSensorADCConfig_t; 1];
    //
// API for reading current meters by id.
//
    #[no_mangle]
    static currentMeterIds: [uint8_t; 0];
    #[no_mangle]
    static supportedCurrentMeterCount: uint8_t;
    #[no_mangle]
    fn currentMeterRead(id: currentMeterId_e,
                        currentMeter: *mut currentMeter_t);
    #[no_mangle]
    fn getBatteryCellCount() -> uint8_t;
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
    #[no_mangle]
    fn getMAhDrawn() -> int32_t;
    #[no_mangle]
    fn getAmperage() -> int32_t;
    #[no_mangle]
    fn getBatteryState() -> batteryState_e;
    //
// API for reading/configuring current meters by id.
//
    #[no_mangle]
    static voltageMeterADCtoIDMap: [uint8_t; 1];
    #[no_mangle]
    static supportedVoltageMeterCount: uint8_t;
    #[no_mangle]
    static voltageMeterIds: [uint8_t; 0];
    #[no_mangle]
    fn voltageMeterRead(id: voltageMeterId_e,
                        voltageMeter: *mut voltageMeter_t);
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
pub type timeDelta_t = int32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct hsvColor_s {
    pub h: uint16_t,
    pub s: uint8_t,
    pub v: uint8_t,
}
pub type hsvColor_t = hsvColor_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sbuf_s {
    pub ptr: *mut uint8_t,
    pub end: *mut uint8_t,
}
// 0 - 359
// 0 - 255
// 0 - 255
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
pub type ioTag_t = uint8_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vcdProfile_s {
    pub video_system: uint8_t,
    pub h_offset: int8_t,
    pub v_offset: int8_t,
}
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
pub type transponderProvider_e = libc::c_uint;
pub const TRANSPONDER_ERLT: transponderProvider_e = 3;
pub const TRANSPONDER_ARCITIMER: transponderProvider_e = 2;
pub const TRANSPONDER_ILAP: transponderProvider_e = 1;
pub const TRANSPONDER_NONE: transponderProvider_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct systemConfig_s {
    pub pidProfileIndex: uint8_t,
    pub activeRateProfile: uint8_t,
    pub debug_mode: uint8_t,
    pub task_statistics: uint8_t,
    pub rateProfile6PosSwitch: uint8_t,
    pub cpu_overclock: uint8_t,
    pub powerOnArmingGraceTime: uint8_t,
    pub boardIdentifier: [libc::c_char; 6],
}
pub type systemConfig_t = systemConfig_s;
// in seconds
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
// These must be consecutive, see 'reversedSources'
pub type C2RustUnnamed = libc::c_uint;
pub const INPUT_SOURCE_COUNT: C2RustUnnamed = 14;
pub const INPUT_GIMBAL_ROLL: C2RustUnnamed = 13;
pub const INPUT_GIMBAL_PITCH: C2RustUnnamed = 12;
pub const INPUT_RC_AUX4: C2RustUnnamed = 11;
pub const INPUT_RC_AUX3: C2RustUnnamed = 10;
pub const INPUT_RC_AUX2: C2RustUnnamed = 9;
pub const INPUT_RC_AUX1: C2RustUnnamed = 8;
pub const INPUT_RC_THROTTLE: C2RustUnnamed = 7;
pub const INPUT_RC_YAW: C2RustUnnamed = 6;
pub const INPUT_RC_PITCH: C2RustUnnamed = 5;
pub const INPUT_RC_ROLL: C2RustUnnamed = 4;
pub const INPUT_STABILIZED_THROTTLE: C2RustUnnamed = 3;
pub const INPUT_STABILIZED_YAW: C2RustUnnamed = 2;
pub const INPUT_STABILIZED_PITCH: C2RustUnnamed = 1;
pub const INPUT_STABILIZED_ROLL: C2RustUnnamed = 0;
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
pub type vcdProfile_t = vcdProfile_s;
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
pub type batteryConfig_t = batteryConfig_s;
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
pub type voltageMeterSource_e = libc::c_uint;
pub const VOLTAGE_METER_COUNT: voltageMeterSource_e = 3;
pub const VOLTAGE_METER_ESC: voltageMeterSource_e = 2;
pub const VOLTAGE_METER_ADC: voltageMeterSource_e = 1;
pub const VOLTAGE_METER_NONE: voltageMeterSource_e = 0;
pub type currentSensorADCConfig_t = currentSensorADCConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct currentSensorADCConfig_s {
    pub scale: int16_t,
    pub offset: int16_t,
}
pub const CURRENT_METER_ID_BATTERY_1: currentMeterId_e = 10;
pub type voltageSensorADCConfig_t = voltageSensorADCConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct voltageSensorADCConfig_s {
    pub vbatscale: uint8_t,
    pub vbatresdivval: uint8_t,
    pub vbatresdivmultiplier: uint8_t,
}
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
// scale the current sensor output voltage to milliamps. Value in mV/10A
// offset of the current sensor in mA
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
pub type transponderConfig_t = transponderConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct transponderConfig_s {
    pub provider: transponderProvider_e,
    pub reserved: uint8_t,
    pub data: [uint8_t; 9],
    pub ioTag: ioTag_t,
}
pub type transponderRequirement_t = transponderRequirement_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct transponderRequirement_s {
    pub provider: uint8_t,
    pub dataLength: uint8_t,
    pub transmitDelay: uint16_t,
    pub transmitJitter: uint16_t,
}
pub const MSP_REBOOT_BOOTLOADER: C2RustUnnamed_3 = 1;
pub const MSP_REBOOT_FIRMWARE: C2RustUnnamed_3 = 0;
pub const MSP_REBOOT_MSC: C2RustUnnamed_3 = 2;
pub const MSP_REBOOT_COUNT: C2RustUnnamed_3 = 3;
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
pub const CURRENT_SENSOR_ADC: C2RustUnnamed_0 = 1;
pub const VOLTAGE_SENSOR_ADC_VBAT: C2RustUnnamed_2 = 0;
pub const VOLTAGE_SENSOR_TYPE_ADC_RESISTOR_DIVIDER: C2RustUnnamed_1 = 0;
pub type currentMeter_t = currentMeter_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct currentMeter_s {
    pub amperage: int32_t,
    pub amperageLatest: int32_t,
    pub mAhDrawn: int32_t,
}
pub type currentMeterId_e = libc::c_uint;
pub const CURRENT_METER_ID_MSP_2: currentMeterId_e = 91;
pub const CURRENT_METER_ID_MSP_1: currentMeterId_e = 90;
pub const CURRENT_METER_ID_VIRTUAL_2: currentMeterId_e = 81;
pub const CURRENT_METER_ID_VIRTUAL_1: currentMeterId_e = 80;
pub const CURRENT_METER_ID_ESC_MOTOR_20: currentMeterId_e = 79;
pub const CURRENT_METER_ID_ESC_MOTOR_12: currentMeterId_e = 71;
pub const CURRENT_METER_ID_ESC_MOTOR_11: currentMeterId_e = 70;
pub const CURRENT_METER_ID_ESC_MOTOR_10: currentMeterId_e = 69;
pub const CURRENT_METER_ID_ESC_MOTOR_9: currentMeterId_e = 68;
pub const CURRENT_METER_ID_ESC_MOTOR_8: currentMeterId_e = 67;
pub const CURRENT_METER_ID_ESC_MOTOR_7: currentMeterId_e = 66;
pub const CURRENT_METER_ID_ESC_MOTOR_6: currentMeterId_e = 65;
pub const CURRENT_METER_ID_ESC_MOTOR_5: currentMeterId_e = 64;
pub const CURRENT_METER_ID_ESC_MOTOR_4: currentMeterId_e = 63;
pub const CURRENT_METER_ID_ESC_MOTOR_3: currentMeterId_e = 62;
pub const CURRENT_METER_ID_ESC_MOTOR_2: currentMeterId_e = 61;
pub const CURRENT_METER_ID_ESC_MOTOR_1: currentMeterId_e = 60;
pub const CURRENT_METER_ID_ESC_COMBINED_10: currentMeterId_e = 59;
pub const CURRENT_METER_ID_ESC_COMBINED_1: currentMeterId_e = 50;
pub const CURRENT_METER_ID_12V_10: currentMeterId_e = 49;
pub const CURRENT_METER_ID_12V_2: currentMeterId_e = 41;
pub const CURRENT_METER_ID_12V_1: currentMeterId_e = 40;
pub const CURRENT_METER_ID_9V_10: currentMeterId_e = 39;
pub const CURRENT_METER_ID_9V_2: currentMeterId_e = 31;
pub const CURRENT_METER_ID_9V_1: currentMeterId_e = 30;
pub const CURRENT_METER_ID_5V_10: currentMeterId_e = 29;
pub const CURRENT_METER_ID_5V_2: currentMeterId_e = 21;
pub const CURRENT_METER_ID_5V_1: currentMeterId_e = 20;
pub const CURRENT_METER_ID_BATTERY_10: currentMeterId_e = 19;
pub const CURRENT_METER_ID_BATTERY_2: currentMeterId_e = 11;
pub const CURRENT_METER_ID_NONE: currentMeterId_e = 0;
pub type voltageMeter_t = voltageMeter_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct voltageMeter_s {
    pub filtered: uint16_t,
    pub unfiltered: uint16_t,
    pub lowVoltageCutoff: bool,
}
pub type voltageMeterId_e = libc::c_uint;
pub const VOLTAGE_METER_ID_CELL_40: voltageMeterId_e = 119;
pub const VOLTAGE_METER_ID_CELL_2: voltageMeterId_e = 81;
pub const VOLTAGE_METER_ID_CELL_1: voltageMeterId_e = 80;
pub const VOLTAGE_METER_ID_ESC_MOTOR_20: voltageMeterId_e = 79;
pub const VOLTAGE_METER_ID_ESC_MOTOR_12: voltageMeterId_e = 71;
pub const VOLTAGE_METER_ID_ESC_MOTOR_11: voltageMeterId_e = 70;
pub const VOLTAGE_METER_ID_ESC_MOTOR_10: voltageMeterId_e = 69;
pub const VOLTAGE_METER_ID_ESC_MOTOR_9: voltageMeterId_e = 68;
pub const VOLTAGE_METER_ID_ESC_MOTOR_8: voltageMeterId_e = 67;
pub const VOLTAGE_METER_ID_ESC_MOTOR_7: voltageMeterId_e = 66;
pub const VOLTAGE_METER_ID_ESC_MOTOR_6: voltageMeterId_e = 65;
pub const VOLTAGE_METER_ID_ESC_MOTOR_5: voltageMeterId_e = 64;
pub const VOLTAGE_METER_ID_ESC_MOTOR_4: voltageMeterId_e = 63;
pub const VOLTAGE_METER_ID_ESC_MOTOR_3: voltageMeterId_e = 62;
pub const VOLTAGE_METER_ID_ESC_MOTOR_2: voltageMeterId_e = 61;
pub const VOLTAGE_METER_ID_ESC_MOTOR_1: voltageMeterId_e = 60;
pub const VOLTAGE_METER_ID_ESC_COMBINED_10: voltageMeterId_e = 59;
pub const VOLTAGE_METER_ID_ESC_COMBINED_1: voltageMeterId_e = 50;
pub const VOLTAGE_METER_ID_12V_10: voltageMeterId_e = 49;
pub const VOLTAGE_METER_ID_12V_2: voltageMeterId_e = 41;
pub const VOLTAGE_METER_ID_12V_1: voltageMeterId_e = 40;
pub const VOLTAGE_METER_ID_9V_10: voltageMeterId_e = 39;
pub const VOLTAGE_METER_ID_9V_2: voltageMeterId_e = 31;
pub const VOLTAGE_METER_ID_9V_1: voltageMeterId_e = 30;
pub const VOLTAGE_METER_ID_5V_10: voltageMeterId_e = 29;
pub const VOLTAGE_METER_ID_5V_2: voltageMeterId_e = 21;
pub const VOLTAGE_METER_ID_5V_1: voltageMeterId_e = 20;
pub const VOLTAGE_METER_ID_BATTERY_10: voltageMeterId_e = 19;
pub const VOLTAGE_METER_ID_BATTERY_2: voltageMeterId_e = 11;
pub const VOLTAGE_METER_ID_BATTERY_1: voltageMeterId_e = 10;
pub const VOLTAGE_METER_ID_NONE: voltageMeterId_e = 0;
pub type batteryState_e = libc::c_uint;
pub const BATTERY_INIT: batteryState_e = 4;
pub const BATTERY_NOT_PRESENT: batteryState_e = 3;
pub const BATTERY_CRITICAL: batteryState_e = 2;
pub const BATTERY_WARNING: batteryState_e = 1;
pub const BATTERY_OK: batteryState_e = 0;
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
pub type C2RustUnnamed_0 = libc::c_uint;
pub const CURRENT_SENSOR_MSP: C2RustUnnamed_0 = 3;
pub const CURRENT_SENSOR_ESC: C2RustUnnamed_0 = 2;
pub const CURRENT_SENSOR_VIRTUAL: C2RustUnnamed_0 = 0;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const VOLTAGE_SENSOR_TYPE_ESC: C2RustUnnamed_1 = 1;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const VOLTAGE_SENSOR_ADC_5V: C2RustUnnamed_2 = 3;
pub const VOLTAGE_SENSOR_ADC_9V: C2RustUnnamed_2 = 2;
pub const VOLTAGE_SENSOR_ADC_12V: C2RustUnnamed_2 = 1;
// See transponderProvider_e
// current read by current sensor in centiampere (1/100th A)
// current read by current sensor in centiampere (1/100th A) (unfiltered)
// milliampere hours drawn from the battery since start
// 4 UPPER CASE alpha numeric characters that identify the flight controller.
pub type C2RustUnnamed_3 = libc::c_uint;
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn systemConfig() -> *const systemConfig_t {
    return &mut systemConfig_System;
}
#[no_mangle]
pub static mut inputSource_e: C2RustUnnamed = INPUT_STABILIZED_ROLL;
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
unsafe extern "C" fn transponderConfig() -> *const transponderConfig_t {
    return &mut transponderConfig_System;
}
#[inline]
unsafe extern "C" fn transponderConfigMutable() -> *mut transponderConfig_t {
    return &mut transponderConfig_System;
}
#[inline]
unsafe extern "C" fn vcdProfile() -> *const vcdProfile_t {
    return &mut vcdProfile_System;
}
#[inline]
unsafe extern "C" fn vcdProfileMutable() -> *mut vcdProfile_t {
    return &mut vcdProfile_System;
}
#[inline]
unsafe extern "C" fn currentSensorADCConfigMutable()
 -> *mut currentSensorADCConfig_t {
    return &mut currentSensorADCConfig_System;
}
#[inline]
unsafe extern "C" fn currentSensorADCConfig()
 -> *const currentSensorADCConfig_t {
    return &mut currentSensorADCConfig_System;
}
#[inline]
unsafe extern "C" fn batteryConfigMutable() -> *mut batteryConfig_t {
    return &mut batteryConfig_System;
}
#[inline]
unsafe extern "C" fn voltageSensorADCConfigMutable(mut _index: libc::c_int)
 -> *mut voltageSensorADCConfig_t {
    return &mut *voltageSensorADCConfig_SystemArray.as_mut_ptr().offset(_index
                                                                            as
                                                                            isize)
               as *mut voltageSensorADCConfig_t;
}
#[inline]
unsafe extern "C" fn batteryConfig() -> *const batteryConfig_t {
    return &mut batteryConfig_System;
}
#[inline]
unsafe extern "C" fn voltageSensorADCConfig(mut _index: libc::c_int)
 -> *const voltageSensorADCConfig_t {
    return &mut *voltageSensorADCConfig_SystemArray.as_mut_ptr().offset(_index
                                                                            as
                                                                            isize)
               as *mut voltageSensorADCConfig_t;
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
static mut flightControllerIdentifier: *const libc::c_char =
    b"CLFL\x00" as *const u8 as *const libc::c_char;
static mut rebootMode: uint8_t = 0;
//USE_OSD_SLAVE
//USE_SERIAL_4WAY_BLHELI_INTERFACE
unsafe extern "C" fn mspRebootFn(mut serialPort: *mut serialPort_t) {
    match rebootMode as libc::c_int {
        0 => { systemReset(); }
        1 => { systemResetToBootloader(); }
        _ => { }
    }
    loop 
         // control should never return here.
         {
    };
}
// USE_OSD_SLAVE
/*
 * Returns true if the command was processd, false otherwise.
 * May set mspPostProcessFunc to a function to be called once the command has been processed
 */
unsafe extern "C" fn mspCommonProcessOutCommand(mut cmdMSP: uint8_t,
                                                mut dst: *mut sbuf_t,
                                                mut mspPostProcessFn:
                                                    *mut mspPostProcessFnPtr)
 -> bool {
    match cmdMSP as libc::c_int {
        1 => {
            sbufWriteU8(dst,
                        0 as libc::c_int as
                            uint8_t); // No other build targets currently have hardware revision detection.
            sbufWriteU8(dst, 1 as libc::c_int as uint8_t); // 1 == OSD
            sbufWriteU8(dst, 40 as libc::c_int as uint8_t);
        }
        2 => {
            sbufWriteData(dst,
                          flightControllerIdentifier as *const libc::c_void,
                          4 as libc::c_int);
        }
        3 => {
            sbufWriteU8(dst, 2 as libc::c_int as uint8_t);
            sbufWriteU8(dst, 5 as libc::c_int as uint8_t);
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
        }
        4 => {
            sbufWriteData(dst,
                          (*systemConfig()).boardIdentifier.as_ptr() as
                              *const libc::c_void, 4 as libc::c_int);
            sbufWriteU16(dst, 0 as libc::c_int as uint16_t);
            sbufWriteU8(dst, 1 as libc::c_int as uint8_t);
            // Board communication capabilities (uint8)
        // Bit 0: 1 iff the board has VCP
        // Bit 1: 1 iff the board supports software serial
            let mut commCapabilities: uint8_t = 0 as libc::c_int as uint8_t;
            commCapabilities =
                (commCapabilities as libc::c_int |
                     (1 as libc::c_int) << 0 as libc::c_int) as uint8_t;
            sbufWriteU8(dst, commCapabilities);
            // Target name with explicit length
            sbufWriteU8(dst,
                        strlen(targetName) as
                            uint8_t); // milliamp hours drawn from battery
            sbufWriteData(dst, targetName as *const libc::c_void,
                          strlen(targetName) as libc::c_int); // rssi
        }
        5 => {
            sbufWriteData(dst, buildDate as *const libc::c_void,
                          11 as
                              libc::c_int); // send current in 0.01 A steps, range is -320A to 320A
            sbufWriteData(dst, buildTime as *const libc::c_void,
                          8 as libc::c_int);
            sbufWriteData(dst, shortGitRevision as *const libc::c_void,
                          7 as libc::c_int);
        }
        110 => {
            sbufWriteU8(dst,
                        constrain(getBatteryVoltage() as libc::c_int,
                                  0 as libc::c_int, 255 as libc::c_int) as
                            uint8_t);
            sbufWriteU16(dst,
                         constrain(getMAhDrawn(), 0 as libc::c_int,
                                   0xffff as libc::c_int) as uint16_t);
            sbufWriteU16(dst, 0 as libc::c_int as uint16_t);
            sbufWriteU16(dst,
                         constrain(getAmperage(), -(0x8000 as libc::c_int),
                                   0x7fff as libc::c_int) as int16_t as
                             uint16_t);
        }
        254 => {
            let mut i: libc::c_int = 0 as libc::c_int;
            while i < 4 as libc::c_int {
                sbufWriteU16(dst, debug[i as usize] as uint16_t);
                i += 1
                // 4 variables are here for general monitoring purpose
            }
        }
        160 => {
            sbufWriteU32(dst, *(0x1ffff7ac as libc::c_int as *mut uint32_t));
            sbufWriteU32(dst, *(0x1ffff7b0 as libc::c_int as *mut uint32_t));
            sbufWriteU32(dst, *(0x1ffff7b4 as libc::c_int as *mut uint32_t));
        }
        36 => { sbufWriteU32(dst, featureMask()); }
        130 => {
            // battery characteristics
            sbufWriteU8(dst,
                        constrain(getBatteryCellCount() as libc::c_int,
                                  0 as libc::c_int, 255 as libc::c_int) as
                            uint8_t); // 0 indicates battery not detected.
            sbufWriteU16(dst, (*batteryConfig()).batteryCapacity); // in mAh
            // battery state
            sbufWriteU8(dst,
                        constrain(getBatteryVoltage() as libc::c_int,
                                  0 as libc::c_int, 255 as libc::c_int) as
                            uint8_t); // in 0.1V steps
            sbufWriteU16(dst,
                         constrain(getMAhDrawn(), 0 as libc::c_int,
                                   0xffff as libc::c_int) as
                             uint16_t); // milliamp hours drawn from battery
            sbufWriteU16(dst,
                         constrain(getAmperage(), -(0x8000 as libc::c_int),
                                   0x7fff as libc::c_int) as int16_t as
                             uint16_t); // send current in 0.01 A steps, range is -320A to 320A
            // battery alerts
            sbufWriteU8(dst, getBatteryState() as uint8_t);
        }
        128 => {
            // write out id and voltage meter values, once for each meter we support
            let mut count: uint8_t = supportedVoltageMeterCount;
            let mut i_0: libc::c_int = 0 as libc::c_int;
            while i_0 < count as libc::c_int {
                let mut meter: voltageMeter_t =
                    voltageMeter_t{filtered: 0,
                                   unfiltered: 0,
                                   lowVoltageCutoff: false,};
                let mut id: uint8_t =
                    *voltageMeterIds.as_ptr().offset(i_0 as isize);
                voltageMeterRead(id as voltageMeterId_e, &mut meter);
                sbufWriteU8(dst, id);
                sbufWriteU8(dst,
                            constrain(meter.filtered as libc::c_int,
                                      0 as libc::c_int, 255 as libc::c_int) as
                                uint8_t);
                i_0 += 1
            }
        }
        129 => {
            // write out id and current meter values, once for each meter we support
            let mut count_0: uint8_t = supportedCurrentMeterCount;
            let mut i_1: libc::c_int = 0 as libc::c_int;
            while i_1 < count_0 as libc::c_int {
                let mut meter_0: currentMeter_t =
                    currentMeter_t{amperage: 0,
                                   amperageLatest: 0,
                                   mAhDrawn: 0,};
                let mut id_0: uint8_t =
                    *currentMeterIds.as_ptr().offset(i_1 as isize);
                currentMeterRead(id_0 as currentMeterId_e, &mut meter_0);
                sbufWriteU8(dst, id_0);
                // send amperage in 0.001 A steps (mA). Negative range is truncated to zero
                sbufWriteU16(dst,
                             constrain(meter_0.mAhDrawn, 0 as libc::c_int,
                                       0xffff as libc::c_int) as
                                 uint16_t); // milliamp hours drawn from battery
                sbufWriteU16(dst,
                             constrain(meter_0.amperage * 10 as libc::c_int,
                                       0 as libc::c_int,
                                       0xffff as libc::c_int) as uint16_t);
                i_1 += 1
            }
        }
        56 => {
            // by using a sensor type and a sub-frame length it's possible to configure any type of voltage meter,
        // e.g. an i2c/spi/can sensor or any sensor not built directly into the FC such as ESC/RX/SPort/SBus that has
        // different configuration requirements.
            sbufWriteU8(dst,
                        1 as libc::c_int as
                            uint8_t); // voltage meters in payload
            let mut i_2: libc::c_int =
                VOLTAGE_SENSOR_ADC_VBAT as
                    libc::c_int; // length of id, type, vbatscale, vbatresdivval, vbatresdivmultipler, in bytes
            while i_2 < 1 as libc::c_int {
                let adcSensorSubframeLength: uint8_t =
                    (1 as libc::c_int + 1 as libc::c_int + 1 as libc::c_int +
                         1 as libc::c_int + 1 as libc::c_int) as
                        uint8_t; // ADC sensor sub-frame length
                sbufWriteU8(dst, adcSensorSubframeLength); // id of the sensor
                sbufWriteU8(dst,
                            voltageMeterADCtoIDMap[i_2 as
                                                       usize]); // indicate the type of sensor that the next part of the payload is for
                sbufWriteU8(dst,
                            VOLTAGE_SENSOR_TYPE_ADC_RESISTOR_DIVIDER as
                                libc::c_int as uint8_t);
                sbufWriteU8(dst, (*voltageSensorADCConfig(i_2)).vbatscale);
                sbufWriteU8(dst,
                            (*voltageSensorADCConfig(i_2)).vbatresdivval);
                sbufWriteU8(dst,
                            (*voltageSensorADCConfig(i_2)).vbatresdivmultiplier);
                i_2 += 1
            }
        }
        40 => {
            // the ADC and VIRTUAL sensors have the same configuration requirements, however this API reflects
        // that this situation may change and allows us to support configuration of any current sensor with
        // specialist configuration requirements.
            let mut currentMeterCount: libc::c_int =
                1 as
                    libc::c_int; // length of id, type, scale, offset, in bytes
            sbufWriteU8(dst,
                        currentMeterCount as uint8_t); // the id of the meter
            let adcSensorSubframeLength_0: uint8_t =
                (1 as libc::c_int + 1 as libc::c_int + 2 as libc::c_int +
                     2 as libc::c_int) as
                    uint8_t; // indicate the type of sensor that the next part of the payload is for
            sbufWriteU8(dst, adcSensorSubframeLength_0);
            sbufWriteU8(dst,
                        CURRENT_METER_ID_BATTERY_1 as libc::c_int as uint8_t);
            sbufWriteU8(dst, CURRENT_SENSOR_ADC as libc::c_int as uint8_t);
            sbufWriteU16(dst, (*currentSensorADCConfig()).scale as uint16_t);
            sbufWriteU16(dst, (*currentSensorADCConfig()).offset as uint16_t);
        }
        32 => {
            sbufWriteU8(dst, (*batteryConfig()).vbatmincellvoltage);
            sbufWriteU8(dst, (*batteryConfig()).vbatmaxcellvoltage);
            sbufWriteU8(dst, (*batteryConfig()).vbatwarningcellvoltage);
            sbufWriteU16(dst, (*batteryConfig()).batteryCapacity);
            sbufWriteU8(dst,
                        (*batteryConfig()).voltageMeterSource as uint8_t);
            sbufWriteU8(dst,
                        (*batteryConfig()).currentMeterSource as uint8_t);
        }
        82 => {
            // Backward compatibility to BFC 3.1.1 is lost for this message type
            sbufWriteU8(dst, 3 as libc::c_int as uint8_t);
            let mut i_3: libc::c_uint = 0 as libc::c_int as libc::c_uint;
            while i_3 < 3 as libc::c_int as libc::c_uint {
                sbufWriteU8(dst,
                            transponderRequirements[i_3 as usize].provider);
                sbufWriteU8(dst,
                            transponderRequirements[i_3 as usize].dataLength);
                i_3 = i_3.wrapping_add(1)
            }
            let mut provider: uint8_t =
                (*transponderConfig()).provider as uint8_t;
            sbufWriteU8(dst, provider);
            if provider != 0 {
                let mut requirementIndex: uint8_t =
                    (provider as libc::c_int - 1 as libc::c_int) as uint8_t;
                let mut providerDataLength: uint8_t =
                    transponderRequirements[requirementIndex as
                                                usize].dataLength;
                let mut i_4: libc::c_uint = 0 as libc::c_int as libc::c_uint;
                while i_4 < providerDataLength as libc::c_uint {
                    sbufWriteU8(dst,
                                (*transponderConfig()).data[i_4 as usize]);
                    i_4 = i_4.wrapping_add(1)
                }
            }
        }
        84 => {
            let mut osdFlags: uint8_t = 0 as libc::c_int as uint8_t;
            osdFlags =
                (osdFlags as libc::c_int |
                     (1 as libc::c_int) << 1 as libc::c_int) as uint8_t;
            osdFlags =
                (osdFlags as libc::c_int |
                     (1 as libc::c_int) << 4 as libc::c_int) as uint8_t;
            sbufWriteU8(dst, osdFlags);
            // send video system (AUTO/PAL/NTSC)
            sbufWriteU8(dst, (*vcdProfile()).video_system); // sensors
        }
        _ => { return 0 as libc::c_int != 0 }
    } // flight modes
    return 1 as libc::c_int != 0; // profile
}
unsafe extern "C" fn mspProcessOutCommand(mut cmdMSP: uint8_t,
                                          mut dst: *mut sbuf_t) -> bool {
    match cmdMSP as libc::c_int {
        150 | 101 => {
            sbufWriteU16(dst,
                         getTaskDeltaTime(TASK_SERIAL) as
                             uint16_t); // max profiles
            sbufWriteU16(dst, i2cGetErrorCounter());
            sbufWriteU16(dst, 0 as libc::c_int as uint16_t);
            sbufWriteU32(dst, 0 as libc::c_int as uint32_t);
            sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
            sbufWriteU16(dst,
                         constrain(averageSystemLoadPercent as libc::c_int,
                                   0 as libc::c_int, 100 as libc::c_int) as
                             uint16_t);
            if cmdMSP as libc::c_int == 150 as libc::c_int {
                sbufWriteU8(dst, 1 as libc::c_int as uint8_t);
                sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
                // control rate profile
            } else {
                sbufWriteU16(dst, 0 as libc::c_int as uint16_t);
                // gyro cycle time
            }
        }
        _ => { return 0 as libc::c_int != 0 }
    }
    return 1 as libc::c_int != 0;
}
// USE_OSD_SLAVE
unsafe extern "C" fn mspFcProcessOutCommandWithArg(mut cmdMSP: uint8_t,
                                                   mut src: *mut sbuf_t,
                                                   mut dst: *mut sbuf_t,
                                                   mut mspPostProcessFn:
                                                       *mut mspPostProcessFnPtr)
 -> mspResult_e {
    match cmdMSP as libc::c_int {
        68 => {
            if sbufBytesRemaining(src) != 0 {
                rebootMode = sbufReadU8(src);
                if rebootMode as libc::c_int >=
                       MSP_REBOOT_COUNT as libc::c_int ||
                       rebootMode as libc::c_int ==
                           MSP_REBOOT_MSC as libc::c_int {
                    return MSP_RESULT_ERROR
                }
            } else {
                rebootMode = MSP_REBOOT_FIRMWARE as libc::c_int as uint8_t
            }
            sbufWriteU8(dst, rebootMode);
            if !mspPostProcessFn.is_null() {
                *mspPostProcessFn =
                    Some(mspRebootFn as
                             unsafe extern "C" fn(_: *mut serialPort_t) -> ())
            }
        }
        _ => { return MSP_RESULT_CMD_UNKNOWN }
    }
    return MSP_RESULT_ACK;
}
unsafe extern "C" fn mspProcessInCommand(mut cmdMSP: uint8_t,
                                         mut src: *mut sbuf_t)
 -> mspResult_e {
    match cmdMSP as libc::c_int {
        208 => { resetEEPROM(); readEEPROM(); }
        250 => { writeEEPROM(); readEEPROM(); }
        _ => {
            // we do not know how to handle the (valid) message, indicate error MSP $M!
            return MSP_RESULT_ERROR
        }
    }
    return MSP_RESULT_ACK;
}
// USE_OSD_SLAVE
unsafe extern "C" fn mspCommonProcessInCommand(mut cmdMSP: uint8_t,
                                               mut src: *mut sbuf_t,
                                               mut mspPostProcessFn:
                                                   *mut mspPostProcessFnPtr)
 -> mspResult_e {
    let dataSize: libc::c_uint = sbufBytesRemaining(src) as libc::c_uint;
    // maybe unused due to compiler options
    match cmdMSP as libc::c_int {
        83 => {
            let mut provider: uint8_t = sbufReadU8(src);
            let mut bytesRemaining: uint8_t =
                dataSize.wrapping_sub(1 as libc::c_int as libc::c_uint) as
                    uint8_t;
            if provider as libc::c_int > 3 as libc::c_int {
                return MSP_RESULT_ERROR
            }
            let requirementIndex: uint8_t =
                (provider as libc::c_int - 1 as libc::c_int) as uint8_t;
            let transponderDataSize: uint8_t =
                transponderRequirements[requirementIndex as usize].dataLength;
            (*transponderConfigMutable()).provider =
                provider as transponderProvider_e;
            if !(provider as libc::c_int == TRANSPONDER_NONE as libc::c_int) {
                if bytesRemaining as libc::c_int !=
                       transponderDataSize as libc::c_int {
                    return MSP_RESULT_ERROR
                }
                if provider as libc::c_uint !=
                       (*transponderConfig()).provider as libc::c_uint {
                    transponderStopRepeating();
                }
                memset((*transponderConfigMutable()).data.as_mut_ptr() as
                           *mut libc::c_void, 0 as libc::c_int,
                       ::core::mem::size_of::<[uint8_t; 9]>() as
                           libc::c_ulong);
                let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
                while i < transponderDataSize as libc::c_uint {
                    (*transponderConfigMutable()).data[i as usize] =
                        sbufReadU8(src);
                    i = i.wrapping_add(1)
                }
                transponderUpdateData();
            }
        }
        57 => {
            let mut id: int8_t = sbufReadU8(src) as int8_t;
            //
        // find and configure an ADC voltage sensor
        //
            let mut voltageSensorADCIndex: int8_t = 0;
            voltageSensorADCIndex = 0 as libc::c_int as int8_t;
            while (voltageSensorADCIndex as libc::c_int) < 1 as libc::c_int {
                if id as libc::c_int ==
                       voltageMeterADCtoIDMap[voltageSensorADCIndex as usize]
                           as libc::c_int {
                    break ;
                }
                voltageSensorADCIndex += 1
            }
            if (voltageSensorADCIndex as libc::c_int) < 1 as libc::c_int {
                (*voltageSensorADCConfigMutable(voltageSensorADCIndex as
                                                    libc::c_int)).vbatscale =
                    sbufReadU8(src);
                (*voltageSensorADCConfigMutable(voltageSensorADCIndex as
                                                    libc::c_int)).vbatresdivval
                    = sbufReadU8(src);
                (*voltageSensorADCConfigMutable(voltageSensorADCIndex as
                                                    libc::c_int)).vbatresdivmultiplier
                    = sbufReadU8(src)
            } else {
                // if we had any other types of voltage sensor to configure, this is where we'd do it.
                sbufReadU8(src); // vbatlevel_warn1 in MWC2.3 GUI
                sbufReadU8(src); // vbatlevel_warn2 in MWC2.3 GUI
                sbufReadU8(src); // vbatlevel when buzzer starts to alert
            }
        }
        41 => {
            let mut id_0: libc::c_int = sbufReadU8(src) as libc::c_int;
            match id_0 {
                10 => {
                    (*currentSensorADCConfigMutable()).scale =
                        sbufReadU16(src) as int16_t;
                    (*currentSensorADCConfigMutable()).offset =
                        sbufReadU16(src) as int16_t
                }
                _ => { sbufReadU16(src); sbufReadU16(src); }
            }
        }
        33 => {
            (*batteryConfigMutable()).vbatmincellvoltage = sbufReadU8(src);
            (*batteryConfigMutable()).vbatmaxcellvoltage = sbufReadU8(src);
            (*batteryConfigMutable()).vbatwarningcellvoltage =
                sbufReadU8(src);
            (*batteryConfigMutable()).batteryCapacity = sbufReadU16(src);
            (*batteryConfigMutable()).voltageMeterSource =
                sbufReadU8(src) as voltageMeterSource_e;
            (*batteryConfigMutable()).currentMeterSource =
                sbufReadU8(src) as currentMeterSource_e
        }
        85 => {
            let addr: uint8_t = sbufReadU8(src);
            if addr as int8_t as libc::c_int == -(1 as libc::c_int) {
                /* Set general OSD settings */
                (*vcdProfileMutable()).video_system = sbufReadU8(src)
            } else if addr as int8_t as libc::c_int == -(2 as libc::c_int) {
                return MSP_RESULT_ERROR
            } else { return MSP_RESULT_ERROR }
        }
        87 => {
            let mut font_data: [uint8_t; 64] = [0; 64];
            let addr_0: uint8_t = sbufReadU8(src);
            let mut i_0: libc::c_int = 0 as libc::c_int;
            while i_0 < 54 as libc::c_int {
                font_data[i_0 as usize] = sbufReadU8(src);
                i_0 += 1
            }
            // !!TODO - replace this with a device independent implementation
            max7456WriteNvm(addr_0, font_data.as_mut_ptr());
        }
        _ => {
            // OSD || USE_OSD_SLAVE
            return mspProcessInCommand(cmdMSP, src)
        }
    }
    return MSP_RESULT_ACK;
}
/*
 * Returns MSP_RESULT_ACK, MSP_RESULT_ERROR or MSP_RESULT_NO_REPLY
 */
#[no_mangle]
pub unsafe extern "C" fn mspFcProcessCommand(mut cmd: *mut mspPacket_t,
                                             mut reply: *mut mspPacket_t,
                                             mut mspPostProcessFn:
                                                 *mut mspPostProcessFnPtr)
 -> mspResult_e {
    let mut ret: libc::c_int = MSP_RESULT_ACK as libc::c_int;
    let mut dst: *mut sbuf_t = &mut (*reply).buf;
    let mut src: *mut sbuf_t = &mut (*cmd).buf;
    let cmdMSP: uint8_t = (*cmd).cmd as uint8_t;
    // initialize reply by default
    (*reply).cmd = (*cmd).cmd;
    if mspCommonProcessOutCommand(cmdMSP, dst, mspPostProcessFn) {
        ret = MSP_RESULT_ACK as libc::c_int
    } else if mspProcessOutCommand(cmdMSP, dst) {
        ret = MSP_RESULT_ACK as libc::c_int
    } else {
        ret =
            mspFcProcessOutCommandWithArg(cmdMSP, src, dst, mspPostProcessFn)
                as libc::c_int;
        if !(ret != MSP_RESULT_CMD_UNKNOWN as libc::c_int) {
            ret =
                mspCommonProcessInCommand(cmdMSP, src, mspPostProcessFn) as
                    libc::c_int
        }
    }
    (*reply).result = ret as int16_t;
    return ret as mspResult_e;
}
#[no_mangle]
pub unsafe extern "C" fn mspFcProcessReply(mut reply: *mut mspPacket_t) {
    let mut src: *mut sbuf_t = &mut (*reply).buf;
    // potentially unused depending on compile options.
    match (*reply).cmd as libc::c_int {
        110 => {
            let mut batteryVoltage: uint8_t =
                sbufReadU8(src); // lock it as soon as a MSP_DISPLAYPORT message is received to prevent accidental CLI/DFU mode.
            let mut mAhDrawn: uint16_t = sbufReadU16(src);
            let mut rssi: uint16_t = sbufReadU16(src);
            let mut amperage: uint16_t = sbufReadU16(src);
        }
        182 => {
            osdSlaveIsLocked = 1 as libc::c_int != 0;
            let subCmd: libc::c_int = sbufReadU8(src) as libc::c_int;
            match subCmd {
                0 => {
                    // HEARTBEAT
                    osdSlaveHeartbeat();
                }
                2 => {
                    // CLEAR
                    osdSlaveClearScreen();
                }
                3 => {
                    // FIXME move this
                    let y: uint8_t = sbufReadU8(src); // row
                    let x: uint8_t = sbufReadU8(src); // column
                    sbufReadU8(src); // reserved
                    let mut buf: [libc::c_char; 31] = [0; 31];
                    let len: libc::c_int =
                        ({
                             let mut _a: libc::c_int =
                                 sbufBytesRemaining(src);
                             let mut _b: libc::c_int = 30 as libc::c_int;
                             if _a < _b { _a } else { _b }
                         });
                    sbufReadData(src,
                                 &mut buf as *mut [libc::c_char; 31] as
                                     *mut libc::c_void, len);
                    buf[len as usize] = 0 as libc::c_int as libc::c_char;
                    osdSlaveWrite(x, y, buf.as_mut_ptr());
                }
                4 => { osdSlaveDrawScreen(); }
                1 | _ => { }
            }
        }
        _ => { }
    };
}
#[no_mangle]
pub unsafe extern "C" fn mspInit() { }
