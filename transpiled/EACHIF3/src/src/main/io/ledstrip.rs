use ::libc;
extern "C" {
    #[no_mangle]
    fn strtol(_: *const libc::c_char, _: *mut *mut libc::c_char,
              _: libc::c_int) -> libc::c_long;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn strchr(_: *const libc::c_char, _: libc::c_int) -> *mut libc::c_char;
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
    // expand to t if bit is 1, f when bit is 0. Other bit values are not supported
    // Expand all argumens and call macro with them. When expansion of some argument contains ',', it will be passed as multiple arguments
// #define TAKE3(_1,_2,_3) CONCAT3(_1,_2,_3)
// #define MULTI2 A,B
// PP_CALL(TAKE3, MULTI2, C) expands to ABC
    /*
http://resnet.uoregon.edu/~gurney_j/jmpc/bitwise.html
*/
    /*
 * https://groups.google.com/forum/?hl=en#!msg/comp.lang.c/attFnqwhvGk/sGBKXvIkY3AJ
 * Return (v ? floor(log2(v)) : 0) when 0 <= v < 1<<[8, 16, 32, 64].
 * Inefficient algorithm, intended for compile-time constants.
 */
    // non ISO variant from linux kernel; checks ptr type, but triggers 'ISO C forbids braced-groups within expressions [-Wpedantic]'
//  __extension__ is here to disable this warning
    // using memcpy_fn will force memcpy function call, instead of inlining it. In most cases function call takes fewer instructions
//  than inlined version (inlining is cheaper for very small moves < 8 bytes / 2 store instructions)
    #[no_mangle]
    fn memcpy_fn(destination: *mut libc::c_void, source: *const libc::c_void,
                 num: size_t) -> *mut libc::c_void;
    #[no_mangle]
    fn scaleRange(x: libc::c_int, srcFrom: libc::c_int, srcTo: libc::c_int,
                  destFrom: libc::c_int, destTo: libc::c_int) -> libc::c_int;
    // Disabling this, in favour of tfp_format to be used in cli.c
//int tfp_printf(const char *fmt, ...);
    #[no_mangle]
    fn tfp_sprintf(s: *mut libc::c_char, fmt: *const libc::c_char, _: ...)
     -> libc::c_int;
    #[no_mangle]
    fn getLedHsv(index: uint16_t, color: *mut hsvColor_t);
    #[no_mangle]
    fn setLedHsv(index: uint16_t, color: *const hsvColor_t);
    #[no_mangle]
    fn ws2811UpdateStrip(ledFormat: ledStripFormatRGB_e);
    #[no_mangle]
    fn ws2811LedStripInit(ioTag: ioTag_t);
    #[no_mangle]
    fn setStripColor(color: *const hsvColor_t);
    #[no_mangle]
    fn isWS2811LedStripReady() -> bool;
    #[no_mangle]
    fn vtxCommonDevice() -> *mut vtxDevice_t;
    #[no_mangle]
    fn vtxCommonGetBandAndChannel(vtxDevice: *const vtxDevice_t,
                                  pBand: *mut uint8_t, pChannel: *mut uint8_t)
     -> bool;
    #[no_mangle]
    fn vtxCommonGetPowerIndex(vtxDevice: *const vtxDevice_t,
                              pIndex: *mut uint8_t) -> bool;
    #[no_mangle]
    fn vtxCommonGetPitMode(vtxDevice: *const vtxDevice_t,
                           pOnOff: *mut uint8_t) -> bool;
    #[no_mangle]
    static mut rcCommand: [libc::c_float; 4];
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn isArmingDisabled() -> bool;
    #[no_mangle]
    static mut flightModeFlags: uint16_t;
    #[no_mangle]
    fn failsafeIsActive() -> bool;
    #[no_mangle]
    fn timerioTagGetByUsage(usageFlag: timerUsageFlag_e, index: uint8_t)
     -> ioTag_t;
    #[no_mangle]
    fn isBeeperOn() -> bool;
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
    static vtx58frequencyTable: [[uint16_t; 8]; 5];
    #[no_mangle]
    static mut rcData: [int16_t; 18];
    #[no_mangle]
    fn rxIsReceivingSignal() -> bool;
    #[no_mangle]
    fn getRssiPercent() -> uint8_t;
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    fn getBatteryState() -> batteryState_e;
    #[no_mangle]
    fn calculateBatteryPercentageRemaining() -> uint8_t;
}
pub type size_t = libc::c_ulong;
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
pub const HSV_VALUE: C2RustUnnamed = 2;
pub const HSV_SATURATION: C2RustUnnamed = 1;
pub const HSV_HUE: C2RustUnnamed = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct hsvColor_s {
    pub h: uint16_t,
    pub s: uint8_t,
    pub v: uint8_t,
}
pub type hsvColor_t = hsvColor_s;
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
pub type pgn_t = uint16_t;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed_0 = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed_0 = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed_0 = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed_0 = 4095;
// function that resets a single parameter group instance
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
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
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
pub type timeDelta_t = int32_t;
pub type timeUs_t = uint32_t;
pub type ledModeIndex_e = libc::c_uint;
pub const LED_AUX_CHANNEL: ledModeIndex_e = 7;
pub const LED_SPECIAL: ledModeIndex_e = 6;
pub const LED_MODE_BARO: ledModeIndex_e = 5;
pub const LED_MODE_MAG: ledModeIndex_e = 4;
pub const LED_MODE_ANGLE: ledModeIndex_e = 3;
pub const LED_MODE_HORIZON: ledModeIndex_e = 2;
pub const LED_MODE_HEADFREE: ledModeIndex_e = 1;
pub const LED_MODE_ORIENTATION: ledModeIndex_e = 0;
pub type ledSpecialColorIds_e = libc::c_uint;
pub const LED_SCOLOR_GPSLOCKED: ledSpecialColorIds_e = 7;
pub const LED_SCOLOR_GPSNOLOCK: ledSpecialColorIds_e = 6;
pub const LED_SCOLOR_GPSNOSATS: ledSpecialColorIds_e = 5;
pub const LED_SCOLOR_BLINKBACKGROUND: ledSpecialColorIds_e = 4;
pub const LED_SCOLOR_BACKGROUND: ledSpecialColorIds_e = 3;
pub const LED_SCOLOR_ANIMATION: ledSpecialColorIds_e = 2;
pub const LED_SCOLOR_ARMED: ledSpecialColorIds_e = 1;
pub const LED_SCOLOR_DISARMED: ledSpecialColorIds_e = 0;
pub type ledDirectionId_e = libc::c_uint;
pub const LED_DIRECTION_DOWN: ledDirectionId_e = 5;
pub const LED_DIRECTION_UP: ledDirectionId_e = 4;
pub const LED_DIRECTION_WEST: ledDirectionId_e = 3;
pub const LED_DIRECTION_SOUTH: ledDirectionId_e = 2;
pub const LED_DIRECTION_EAST: ledDirectionId_e = 1;
pub const LED_DIRECTION_NORTH: ledDirectionId_e = 0;
pub type ledBaseFunctionId_e = libc::c_uint;
pub const LED_FUNCTION_THRUST_RING: ledBaseFunctionId_e = 6;
pub const LED_FUNCTION_GPS: ledBaseFunctionId_e = 5;
pub const LED_FUNCTION_RSSI: ledBaseFunctionId_e = 4;
pub const LED_FUNCTION_BATTERY: ledBaseFunctionId_e = 3;
pub const LED_FUNCTION_ARM_STATE: ledBaseFunctionId_e = 2;
pub const LED_FUNCTION_FLIGHT_MODE: ledBaseFunctionId_e = 1;
pub const LED_FUNCTION_COLOR: ledBaseFunctionId_e = 0;
pub type ledOverlayId_e = libc::c_uint;
pub const LED_OVERLAY_WARNING: ledOverlayId_e = 5;
pub const LED_OVERLAY_INDICATOR: ledOverlayId_e = 4;
pub const LED_OVERLAY_VTX: ledOverlayId_e = 3;
pub const LED_OVERLAY_BLINK: ledOverlayId_e = 2;
pub const LED_OVERLAY_LARSON_SCANNER: ledOverlayId_e = 1;
pub const LED_OVERLAY_THROTTLE: ledOverlayId_e = 0;
pub type ledStripFormatRGB_e = libc::c_uint;
pub const LED_RGB: ledStripFormatRGB_e = 1;
pub const LED_GRB: ledStripFormatRGB_e = 0;
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
pub type ledConfig_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ledCounts_s {
    pub count: uint8_t,
    pub ring: uint8_t,
    pub larson: uint8_t,
    pub ringSeqLen: uint8_t,
}
pub type ledCounts_t = ledCounts_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ledStripConfig_s {
    pub ledConfigs: [ledConfig_t; 32],
    pub colors: [hsvColor_t; 16],
    pub modeColors: [modeColorIndexes_t; 6],
    pub specialColors: specialColorIndexes_t,
    pub ledstrip_visual_beeper: uint8_t,
    pub ledstrip_aux_channel: uint8_t,
    pub ioTag: ioTag_t,
    pub ledstrip_grb_rgb: ledStripFormatRGB_e,
}
pub type ledStripConfig_t = ledStripConfig_s;
pub const timIndicator: timId_e = 6;
pub const timVtx: timId_e = 5;
pub const timWarning: timId_e = 4;
pub const timLarson: timId_e = 1;
pub const timBlink: timId_e = 0;
pub const PARSE_STATE_COUNT: parseState_e = 5;
pub const RING_COLORS: parseState_e = 4;
pub const FUNCTIONS: parseState_e = 3;
pub const DIRECTIONS: parseState_e = 2;
pub const Y_COORDINATE: parseState_e = 1;
pub const X_COORDINATE: parseState_e = 0;
pub type parseState_e = libc::c_uint;
pub type timId_e = libc::c_uint;
pub const timTimerCount: timId_e = 8;
pub const timRing: timId_e = 7;
pub const timRssi: timId_e = 3;
pub const timBattery: timId_e = 2;
// suppress LEDLOW mode if beeper is on
// function to apply layer.
// function must replan self using timer pointer
// when updateNow is true (timer triggered), state must be updated first,
//  before calculating led state. Otherwise update started by different trigger
//  may modify LED state.
pub type applyLayerFn_timed
    =
    unsafe extern "C" fn(_: bool, _: *mut timeUs_t) -> ();
pub const ARMED: C2RustUnnamed_3 = 1;
pub const COLOR_ORANGE: C2RustUnnamed_6 = 3;
pub type quadrant_e = libc::c_uint;
pub const QUADRANT_WEST: quadrant_e = 8;
pub const QUADRANT_EAST: quadrant_e = 4;
pub const QUADRANT_SOUTH: quadrant_e = 2;
pub const QUADRANT_NORTH: quadrant_e = 1;
pub const PITCH: rc_alias = 1;
pub const ROLL: rc_alias = 0;
pub const COLOR_DEEP_PINK: C2RustUnnamed_6 = 13;
pub const COLOR_DARK_VIOLET: C2RustUnnamed_6 = 11;
pub const COLOR_BLUE: C2RustUnnamed_6 = 10;
pub const COLOR_GREEN: C2RustUnnamed_6 = 6;
pub const COLOR_YELLOW: C2RustUnnamed_6 = 4;
pub const COLOR_RED: C2RustUnnamed_6 = 2;
pub const COLOR_WHITE: C2RustUnnamed_6 = 1;
pub const COLOR_BLACK: C2RustUnnamed_6 = 0;
pub type vtxDevice_t = vtxDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vtxDevice_s {
    pub vTable: *const vtxVTable_s,
    pub capability: vtxDeviceCapability_t,
    pub frequencyTable: *mut uint16_t,
    pub bandNames: *mut *mut libc::c_char,
    pub channelNames: *mut *mut libc::c_char,
    pub powerNames: *mut *mut libc::c_char,
    pub frequency: uint16_t,
    pub band: uint8_t,
    pub channel: uint8_t,
    pub powerIndex: uint8_t,
    pub pitMode: uint8_t,
}
// Array of [bandCount][channelCount]
// char *bandNames[bandCount]
// char *channelNames[channelCount]
// char *powerNames[powerCount]
// Band = 1, 1-based
// CH1 = 1, 1-based
// Lowest/Off = 0
// 0 = non-PIT, 1 = PIT
// VTX magic numbers
// RTC6705 RF Power index "---", 25 or 200 mW
// SmartAudio "---", 25, 200, 500, 800 mW
// Tramp "---", 25, 100, 200, 400, 600 mW
pub type vtxDeviceCapability_t = vtxDeviceCapability_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vtxDeviceCapability_s {
    pub bandCount: uint8_t,
    pub channelCount: uint8_t,
    pub powerCount: uint8_t,
    pub filler: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vtxVTable_s {
    pub process: Option<unsafe extern "C" fn(_: *mut vtxDevice_t, _: timeUs_t)
                            -> ()>,
    pub getDeviceType: Option<unsafe extern "C" fn(_: *const vtxDevice_t)
                                  -> vtxDevType_e>,
    pub isReady: Option<unsafe extern "C" fn(_: *const vtxDevice_t) -> bool>,
    pub setBandAndChannel: Option<unsafe extern "C" fn(_: *mut vtxDevice_t,
                                                       _: uint8_t, _: uint8_t)
                                      -> ()>,
    pub setPowerByIndex: Option<unsafe extern "C" fn(_: *mut vtxDevice_t,
                                                     _: uint8_t) -> ()>,
    pub setPitMode: Option<unsafe extern "C" fn(_: *mut vtxDevice_t,
                                                _: uint8_t) -> ()>,
    pub setFrequency: Option<unsafe extern "C" fn(_: *mut vtxDevice_t,
                                                  _: uint16_t) -> ()>,
    pub getBandAndChannel: Option<unsafe extern "C" fn(_: *const vtxDevice_t,
                                                       _: *mut uint8_t,
                                                       _: *mut uint8_t)
                                      -> bool>,
    pub getPowerIndex: Option<unsafe extern "C" fn(_: *const vtxDevice_t,
                                                   _: *mut uint8_t) -> bool>,
    pub getPitMode: Option<unsafe extern "C" fn(_: *const vtxDevice_t,
                                                _: *mut uint8_t) -> bool>,
    pub getFrequency: Option<unsafe extern "C" fn(_: *const vtxDevice_t,
                                                  _: *mut uint16_t) -> bool>,
}
pub type vtxDevType_e = libc::c_uint;
pub const VTXDEV_UNKNOWN: vtxDevType_e = 255;
pub const VTXDEV_TRAMP: vtxDevType_e = 4;
pub const VTXDEV_SMARTAUDIO: vtxDevType_e = 3;
pub const VTXDEV_RTC6705: vtxDevType_e = 1;
pub const VTXDEV_UNSUPPORTED: vtxDevType_e = 0;
pub const WARNING_FAILSAFE: warningFlags_e = 2;
pub const WARNING_LOW_BATTERY: warningFlags_e = 1;
pub const WARNING_ARMING_DISABLED: warningFlags_e = 0;
pub type warningFlags_e = libc::c_uint;
pub const BATTERY_OK: batteryState_e = 0;
pub type batteryState_e = libc::c_uint;
pub const BATTERY_INIT: batteryState_e = 4;
pub const BATTERY_NOT_PRESENT: batteryState_e = 3;
pub const BATTERY_CRITICAL: batteryState_e = 2;
pub const BATTERY_WARNING: batteryState_e = 1;
pub const VOLTAGE_METER_NONE: voltageMeterSource_e = 0;
pub type voltageMeterSource_e = libc::c_uint;
pub const VOLTAGE_METER_COUNT: voltageMeterSource_e = 3;
pub const VOLTAGE_METER_ESC: voltageMeterSource_e = 2;
pub const VOLTAGE_METER_ADC: voltageMeterSource_e = 1;
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
pub type currentMeterSource_e = libc::c_uint;
pub const CURRENT_METER_COUNT: currentMeterSource_e = 5;
pub const CURRENT_METER_MSP: currentMeterSource_e = 4;
pub const CURRENT_METER_ESC: currentMeterSource_e = 3;
pub const CURRENT_METER_VIRTUAL: currentMeterSource_e = 2;
pub const CURRENT_METER_ADC: currentMeterSource_e = 1;
pub const CURRENT_METER_NONE: currentMeterSource_e = 0;
pub type larsonParameters_t = larsonParameters_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct larsonParameters_s {
    pub currentBrightness: uint8_t,
    pub currentIndex: int8_t,
    pub direction: int8_t,
}
// map flight mode to led mode, in order of priority
// flightMode == 0 is always active
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_2 {
    pub flightMode: uint16_t,
    pub ledMode: uint8_t,
}
pub const ANGLE_MODE: C2RustUnnamed_4 = 1;
pub const HORIZON_MODE: C2RustUnnamed_4 = 2;
pub const BARO_MODE: C2RustUnnamed_4 = 8;
pub const HEADFREE_MODE: C2RustUnnamed_4 = 64;
pub const THROTTLE: rc_alias = 3;
pub type boxId_e = libc::c_uint;
pub const CHECKBOX_ITEM_COUNT: boxId_e = 41;
pub const BOXACROTRAINER: boxId_e = 40;
pub const BOXPIDAUDIO: boxId_e = 39;
pub const BOXUSER4: boxId_e = 38;
pub const BOXUSER3: boxId_e = 37;
pub const BOXUSER2: boxId_e = 36;
pub const BOXUSER1: boxId_e = 35;
pub const BOXPARALYZE: boxId_e = 34;
pub const BOXVTXPITMODE: boxId_e = 33;
pub const BOXBEEPGPSCOUNT: boxId_e = 32;
pub const BOXPREARM: boxId_e = 31;
pub const BOXFLIPOVERAFTERCRASH: boxId_e = 30;
pub const BOXCAMERA3: boxId_e = 29;
pub const BOXCAMERA2: boxId_e = 28;
pub const BOXCAMERA1: boxId_e = 27;
pub const BOXBLACKBOXERASE: boxId_e = 26;
pub const BOXFPVANGLEMIX: boxId_e = 25;
pub const BOX3D: boxId_e = 24;
pub const BOXAIRMODE: boxId_e = 23;
pub const BOXBLACKBOX: boxId_e = 22;
pub const BOXSERVO3: boxId_e = 21;
pub const BOXSERVO2: boxId_e = 20;
pub const BOXSERVO1: boxId_e = 19;
pub const BOXTELEMETRY: boxId_e = 18;
pub const BOXOSD: boxId_e = 17;
pub const BOXCALIB: boxId_e = 16;
pub const BOXLEDLOW: boxId_e = 15;
pub const BOXBEEPERON: boxId_e = 14;
pub const BOXCAMSTAB: boxId_e = 13;
pub const BOXHEADADJ: boxId_e = 12;
pub const BOXANTIGRAVITY: boxId_e = 11;
pub const BOXID_FLIGHTMODE_LAST: boxId_e = 10;
pub const BOXGPSRESCUE: boxId_e = 10;
pub const BOXFAILSAFE: boxId_e = 9;
pub const BOXPASSTHRU: boxId_e = 8;
pub const BOXHEADFREE: boxId_e = 7;
pub const BOXGPSHOLD: boxId_e = 6;
pub const BOXGPSHOME: boxId_e = 5;
pub const BOXBARO: boxId_e = 4;
pub const BOXMAG: boxId_e = 3;
pub const BOXHORIZON: boxId_e = 2;
pub const BOXANGLE: boxId_e = 1;
pub const BOXARM: boxId_e = 0;
pub type rc_alias = libc::c_uint;
pub const AUX8: rc_alias = 11;
pub const AUX7: rc_alias = 10;
pub const AUX6: rc_alias = 9;
pub const AUX5: rc_alias = 8;
pub const AUX4: rc_alias = 7;
pub const AUX3: rc_alias = 6;
pub const AUX2: rc_alias = 5;
pub const AUX1: rc_alias = 4;
pub const YAW: rc_alias = 2;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_3 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_3 = 2;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const GPS_RESCUE_MODE: C2RustUnnamed_4 = 2048;
pub const FAILSAFE_MODE: C2RustUnnamed_4 = 1024;
pub const PASSTHRU_MODE: C2RustUnnamed_4 = 256;
pub const GPS_HOLD_MODE: C2RustUnnamed_4 = 32;
pub const GPS_HOME_MODE: C2RustUnnamed_4 = 16;
pub const MAG_MODE: C2RustUnnamed_4 = 4;
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
pub type C2RustUnnamed_5 = libc::c_uint;
pub const INPUT_SOURCE_COUNT: C2RustUnnamed_5 = 14;
pub const INPUT_GIMBAL_ROLL: C2RustUnnamed_5 = 13;
pub const INPUT_GIMBAL_PITCH: C2RustUnnamed_5 = 12;
pub const INPUT_RC_AUX4: C2RustUnnamed_5 = 11;
pub const INPUT_RC_AUX3: C2RustUnnamed_5 = 10;
pub const INPUT_RC_AUX2: C2RustUnnamed_5 = 9;
pub const INPUT_RC_AUX1: C2RustUnnamed_5 = 8;
pub const INPUT_RC_THROTTLE: C2RustUnnamed_5 = 7;
pub const INPUT_RC_YAW: C2RustUnnamed_5 = 6;
pub const INPUT_RC_PITCH: C2RustUnnamed_5 = 5;
pub const INPUT_RC_ROLL: C2RustUnnamed_5 = 4;
pub const INPUT_STABILIZED_THROTTLE: C2RustUnnamed_5 = 3;
pub const INPUT_STABILIZED_YAW: C2RustUnnamed_5 = 2;
pub const INPUT_STABILIZED_PITCH: C2RustUnnamed_5 = 1;
pub const INPUT_STABILIZED_ROLL: C2RustUnnamed_5 = 0;
pub const COLOR_LIGHT_BLUE: C2RustUnnamed_6 = 9;
pub const COLOR_MINT_GREEN: C2RustUnnamed_6 = 7;
pub const COLOR_CYAN: C2RustUnnamed_6 = 8;
pub const COLOR_LIME_GREEN: C2RustUnnamed_6 = 5;
pub type C2RustUnnamed_6 = libc::c_uint;
pub const COLOR_MAGENTA: C2RustUnnamed_6 = 12;
#[inline]
unsafe extern "C" fn atoi(mut __nptr: *const libc::c_char) -> libc::c_int {
    return strtol(__nptr, 0 as *mut libc::c_void as *mut *mut libc::c_char,
                  10 as libc::c_int) as libc::c_int;
}
#[inline]
unsafe extern "C" fn ledStripConfig() -> *const ledStripConfig_t {
    return &mut ledStripConfig_System;
}
#[inline]
unsafe extern "C" fn ledStripConfigMutable() -> *mut ledStripConfig_t {
    return &mut ledStripConfig_System;
}
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
unsafe extern "C" fn ledGetX(mut lcfg: *const ledConfig_t) -> uint8_t {
    return (*lcfg >> 0 as libc::c_int + 4 as libc::c_int &
                0xf as libc::c_int as libc::c_uint) as uint8_t;
}
#[inline]
unsafe extern "C" fn ledGetY(mut lcfg: *const ledConfig_t) -> uint8_t {
    return (*lcfg >> 0 as libc::c_int + 0 as libc::c_int &
                0xf as libc::c_int as libc::c_uint) as uint8_t;
}
#[inline]
unsafe extern "C" fn ledGetFunction(mut lcfg: *const ledConfig_t) -> uint8_t {
    return (*lcfg >> 8 as libc::c_int &
                (((1 as libc::c_int) << 4 as libc::c_int) - 1 as libc::c_int)
                    as libc::c_uint) as uint8_t;
}
#[inline]
unsafe extern "C" fn ledGetOverlay(mut lcfg: *const ledConfig_t) -> uint8_t {
    return (*lcfg >> 12 as libc::c_int &
                (((1 as libc::c_int) << 6 as libc::c_int) - 1 as libc::c_int)
                    as libc::c_uint) as uint8_t;
}
#[inline]
unsafe extern "C" fn ledGetColor(mut lcfg: *const ledConfig_t) -> uint8_t {
    return (*lcfg >> 18 as libc::c_int &
                (((1 as libc::c_int) << 4 as libc::c_int) - 1 as libc::c_int)
                    as libc::c_uint) as uint8_t;
}
#[inline]
unsafe extern "C" fn ledGetDirection(mut lcfg: *const ledConfig_t)
 -> uint8_t {
    return (*lcfg >> 22 as libc::c_int &
                (((1 as libc::c_int) << 6 as libc::c_int) - 1 as libc::c_int)
                    as libc::c_uint) as uint8_t;
}
#[inline]
unsafe extern "C" fn ledGetOverlayBit(mut lcfg: *const ledConfig_t,
                                      mut id: libc::c_int) -> bool {
    return ledGetOverlay(lcfg) as libc::c_int >> id & 1 as libc::c_int != 0;
}
#[inline]
unsafe extern "C" fn ledGetDirectionBit(mut lcfg: *const ledConfig_t,
                                        mut id: libc::c_int) -> bool {
    return ledGetDirection(lcfg) as libc::c_int >> id & 1 as libc::c_int != 0;
}
#[inline]
unsafe extern "C" fn cmpTimeUs(mut a: timeUs_t, mut b: timeUs_t)
 -> timeDelta_t {
    return a.wrapping_sub(b) as timeDelta_t;
}
#[no_mangle]
pub static mut inputSource_e: C2RustUnnamed_5 = INPUT_STABILIZED_ROLL;
#[inline]
unsafe extern "C" fn batteryConfig() -> *const batteryConfig_t {
    return &mut batteryConfig_System;
}
#[no_mangle]
pub static mut ledStripConfig_System: ledStripConfig_t =
    ledStripConfig_t{ledConfigs: [0; 32],
                     colors: [hsvColor_t{h: 0, s: 0, v: 0,}; 16],
                     modeColors: [modeColorIndexes_t{color: [0; 6],}; 6],
                     specialColors: specialColorIndexes_t{color: [0; 11],},
                     ledstrip_visual_beeper: 0,
                     ledstrip_aux_channel: 0,
                     ioTag: 0,
                     ledstrip_grb_rgb: LED_GRB,};
#[no_mangle]
pub static mut ledStripConfig_Copy: ledStripConfig_t =
    ledStripConfig_t{ledConfigs: [0; 32],
                     colors: [hsvColor_t{h: 0, s: 0, v: 0,}; 16],
                     modeColors: [modeColorIndexes_t{color: [0; 6],}; 6],
                     specialColors: specialColorIndexes_t{color: [0; 11],},
                     ledstrip_visual_beeper: 0,
                     ledstrip_aux_channel: 0,
                     ioTag: 0,
                     ledstrip_grb_rgb: LED_GRB,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut ledStripConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (27 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<ledStripConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &ledStripConfig_System as
                                     *const ledStripConfig_t as
                                     *mut ledStripConfig_t as *mut uint8_t,
                             copy:
                                 &ledStripConfig_Copy as
                                     *const ledStripConfig_t as
                                     *mut ledStripConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut ledStripConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_ledStripConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut ledStripConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
    };
static mut ledStripInitialised: bool = 0 as libc::c_int != 0;
static mut ledStripEnabled: bool = 1 as libc::c_int != 0;
#[no_mangle]
pub static mut hsv: [hsvColor_t; 14] =
    [{
         let mut init =
             hsvColor_s{h: 0 as libc::c_int as uint16_t,
                        s: 0 as libc::c_int as uint8_t,
                        v: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             hsvColor_s{h: 0 as libc::c_int as uint16_t,
                        s: 255 as libc::c_int as uint8_t,
                        v: 255 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             hsvColor_s{h: 0 as libc::c_int as uint16_t,
                        s: 0 as libc::c_int as uint8_t,
                        v: 255 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             hsvColor_s{h: 30 as libc::c_int as uint16_t,
                        s: 0 as libc::c_int as uint8_t,
                        v: 255 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             hsvColor_s{h: 60 as libc::c_int as uint16_t,
                        s: 0 as libc::c_int as uint8_t,
                        v: 255 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             hsvColor_s{h: 90 as libc::c_int as uint16_t,
                        s: 0 as libc::c_int as uint8_t,
                        v: 255 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             hsvColor_s{h: 120 as libc::c_int as uint16_t,
                        s: 0 as libc::c_int as uint8_t,
                        v: 255 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             hsvColor_s{h: 150 as libc::c_int as uint16_t,
                        s: 0 as libc::c_int as uint8_t,
                        v: 255 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             hsvColor_s{h: 180 as libc::c_int as uint16_t,
                        s: 0 as libc::c_int as uint8_t,
                        v: 255 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             hsvColor_s{h: 210 as libc::c_int as uint16_t,
                        s: 0 as libc::c_int as uint8_t,
                        v: 255 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             hsvColor_s{h: 240 as libc::c_int as uint16_t,
                        s: 0 as libc::c_int as uint8_t,
                        v: 255 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             hsvColor_s{h: 270 as libc::c_int as uint16_t,
                        s: 0 as libc::c_int as uint8_t,
                        v: 255 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             hsvColor_s{h: 300 as libc::c_int as uint16_t,
                        s: 0 as libc::c_int as uint8_t,
                        v: 255 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             hsvColor_s{h: 330 as libc::c_int as uint16_t,
                        s: 0 as libc::c_int as uint8_t,
                        v: 255 as libc::c_int as uint8_t,};
         init
     }];
// macro to save typing on default colors
static mut ledGridRows: uint8_t = 0;
// grid offsets
static mut highestYValueForNorth: int8_t = 0;
static mut lowestYValueForSouth: int8_t = 0;
static mut highestXValueForWest: int8_t = 0;
static mut lowestXValueForEast: int8_t = 0;
static mut ledCounts: ledCounts_t =
    ledCounts_t{count: 0, ring: 0, larson: 0, ringSeqLen: 0,};
static mut defaultModeColors: [modeColorIndexes_t; 6] =
    [{
         let mut init =
             modeColorIndexes_s{color:
                                    [COLOR_WHITE as libc::c_int as uint8_t,
                                     COLOR_DARK_VIOLET as libc::c_int as
                                         uint8_t,
                                     COLOR_RED as libc::c_int as uint8_t,
                                     COLOR_DEEP_PINK as libc::c_int as
                                         uint8_t,
                                     COLOR_BLUE as libc::c_int as uint8_t,
                                     COLOR_ORANGE as libc::c_int as
                                         uint8_t],};
         init
     },
     {
         let mut init =
             modeColorIndexes_s{color:
                                    [COLOR_LIME_GREEN as libc::c_int as
                                         uint8_t,
                                     COLOR_DARK_VIOLET as libc::c_int as
                                         uint8_t,
                                     COLOR_ORANGE as libc::c_int as uint8_t,
                                     COLOR_DEEP_PINK as libc::c_int as
                                         uint8_t,
                                     COLOR_BLUE as libc::c_int as uint8_t,
                                     COLOR_ORANGE as libc::c_int as
                                         uint8_t],};
         init
     },
     {
         let mut init =
             modeColorIndexes_s{color:
                                    [COLOR_BLUE as libc::c_int as uint8_t,
                                     COLOR_DARK_VIOLET as libc::c_int as
                                         uint8_t,
                                     COLOR_YELLOW as libc::c_int as uint8_t,
                                     COLOR_DEEP_PINK as libc::c_int as
                                         uint8_t,
                                     COLOR_BLUE as libc::c_int as uint8_t,
                                     COLOR_ORANGE as libc::c_int as
                                         uint8_t],};
         init
     },
     {
         let mut init =
             modeColorIndexes_s{color:
                                    [COLOR_CYAN as libc::c_int as uint8_t,
                                     COLOR_DARK_VIOLET as libc::c_int as
                                         uint8_t,
                                     COLOR_YELLOW as libc::c_int as uint8_t,
                                     COLOR_DEEP_PINK as libc::c_int as
                                         uint8_t,
                                     COLOR_BLUE as libc::c_int as uint8_t,
                                     COLOR_ORANGE as libc::c_int as
                                         uint8_t],};
         init
     },
     {
         let mut init =
             modeColorIndexes_s{color:
                                    [COLOR_MINT_GREEN as libc::c_int as
                                         uint8_t,
                                     COLOR_DARK_VIOLET as libc::c_int as
                                         uint8_t,
                                     COLOR_ORANGE as libc::c_int as uint8_t,
                                     COLOR_DEEP_PINK as libc::c_int as
                                         uint8_t,
                                     COLOR_BLUE as libc::c_int as uint8_t,
                                     COLOR_ORANGE as libc::c_int as
                                         uint8_t],};
         init
     },
     {
         let mut init =
             modeColorIndexes_s{color:
                                    [COLOR_LIGHT_BLUE as libc::c_int as
                                         uint8_t,
                                     COLOR_DARK_VIOLET as libc::c_int as
                                         uint8_t,
                                     COLOR_RED as libc::c_int as uint8_t,
                                     COLOR_DEEP_PINK as libc::c_int as
                                         uint8_t,
                                     COLOR_BLUE as libc::c_int as uint8_t,
                                     COLOR_ORANGE as libc::c_int as
                                         uint8_t],};
         init
     }];
static mut defaultSpecialColors: [specialColorIndexes_t; 1] =
    [{
         let mut init =
             specialColorIndexes_s{color:
                                       [COLOR_GREEN as libc::c_int as uint8_t,
                                        COLOR_BLUE as libc::c_int as uint8_t,
                                        COLOR_WHITE as libc::c_int as uint8_t,
                                        COLOR_BLACK as libc::c_int as uint8_t,
                                        COLOR_BLACK as libc::c_int as uint8_t,
                                        COLOR_RED as libc::c_int as uint8_t,
                                        COLOR_ORANGE as libc::c_int as
                                            uint8_t,
                                        COLOR_GREEN as libc::c_int as uint8_t,
                                        0, 0, 0],};
         init
     }];
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
pub unsafe extern "C" fn pgResetFn_ledStripConfig(mut ledStripConfig_0:
                                                      *mut ledStripConfig_t) {
    memset((*ledStripConfig_0).ledConfigs.as_mut_ptr() as *mut libc::c_void,
           0 as libc::c_int,
           (32 as libc::c_int as
                libc::c_ulong).wrapping_mul(::core::mem::size_of::<ledConfig_t>()
                                                as libc::c_ulong));
    // copy hsv colors as default
    memset((*ledStripConfig_0).colors.as_mut_ptr() as *mut libc::c_void,
           0 as libc::c_int,
           (::core::mem::size_of::<[hsvColor_t; 14]>() as
                libc::c_ulong).wrapping_div(::core::mem::size_of::<hsvColor_t>()
                                                as
                                                libc::c_ulong).wrapping_mul(::core::mem::size_of::<hsvColor_t>()
                                                                                as
                                                                                libc::c_ulong));
    let mut colorIndex: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while (colorIndex as libc::c_ulong) <
              (::core::mem::size_of::<[hsvColor_t; 14]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<hsvColor_t>()
                                                   as libc::c_ulong) {
        (*ledStripConfig_0).colors[colorIndex as usize] =
            hsv[colorIndex as usize];
        colorIndex = colorIndex.wrapping_add(1)
    }
    memcpy_fn(&mut (*ledStripConfig_0).modeColors as
                  *mut [modeColorIndexes_t; 6] as *mut libc::c_void,
              &defaultModeColors as *const [modeColorIndexes_t; 6] as
                  *const libc::c_void,
              ::core::mem::size_of::<[modeColorIndexes_t; 6]>() as
                  libc::c_ulong);
    memcpy_fn(&mut (*ledStripConfig_0).specialColors as
                  *mut specialColorIndexes_t as *mut libc::c_void,
              &defaultSpecialColors as *const [specialColorIndexes_t; 1] as
                  *const libc::c_void,
              ::core::mem::size_of::<[specialColorIndexes_t; 1]>() as
                  libc::c_ulong);
    (*ledStripConfig_0).ledstrip_visual_beeper = 0 as libc::c_int as uint8_t;
    (*ledStripConfig_0).ledstrip_aux_channel =
        THROTTLE as libc::c_int as uint8_t;
    (*ledStripConfig_0).ioTag =
        timerioTagGetByUsage(TIM_USE_LED, 0 as libc::c_int as uint8_t);
}
static mut scaledThrottle: libc::c_int = 0;
static mut auxInput: libc::c_int = 0;
unsafe extern "C" fn updateDimensions() {
    let mut maxX: libc::c_int = 0 as libc::c_int;
    let mut minX: libc::c_int = 0xf as libc::c_int;
    let mut maxY: libc::c_int = 0 as libc::c_int;
    let mut minY: libc::c_int = 0xf as libc::c_int;
    let mut ledIndex: libc::c_int = 0 as libc::c_int;
    while ledIndex < ledCounts.count as libc::c_int {
        let mut ledConfig: *const ledConfig_t =
            &*(*(ledStripConfig as
                     unsafe extern "C" fn()
                         ->
                             *const ledStripConfig_t)()).ledConfigs.as_ptr().offset(ledIndex
                                                                                        as
                                                                                        isize)
                as *const ledConfig_t;
        let mut ledX: libc::c_int = ledGetX(ledConfig) as libc::c_int;
        maxX =
            ({
                 let mut _a: libc::c_int = ledX;
                 let mut _b: libc::c_int = maxX;
                 if _a > _b { _a } else { _b }
             });
        minX =
            ({
                 let mut _a: libc::c_int = ledX;
                 let mut _b: libc::c_int = minX;
                 if _a < _b { _a } else { _b }
             });
        let mut ledY: libc::c_int = ledGetY(ledConfig) as libc::c_int;
        maxY =
            ({
                 let mut _a: libc::c_int = ledY;
                 let mut _b: libc::c_int = maxY;
                 if _a > _b { _a } else { _b }
             });
        minY =
            ({
                 let mut _a: libc::c_int = ledY;
                 let mut _b: libc::c_int = minY;
                 if _a < _b { _a } else { _b }
             });
        ledIndex += 1
    }
    ledGridRows = (maxY - minY + 1 as libc::c_int) as uint8_t;
    if minX < maxX {
        lowestXValueForEast =
            ((minX + maxX) / 2 as libc::c_int + 1 as libc::c_int) as int8_t;
        highestXValueForWest =
            ((minX + maxX - 1 as libc::c_int) / 2 as libc::c_int) as int8_t
    } else {
        lowestXValueForEast =
            (0xf as libc::c_int / 2 as libc::c_int) as int8_t;
        highestXValueForWest =
            (lowestXValueForEast as libc::c_int - 1 as libc::c_int) as int8_t
    }
    if minY < maxY {
        lowestYValueForSouth =
            ((minY + maxY) / 2 as libc::c_int + 1 as libc::c_int) as int8_t;
        highestYValueForNorth =
            ((minY + maxY - 1 as libc::c_int) / 2 as libc::c_int) as int8_t
    } else {
        lowestYValueForSouth =
            (0xf as libc::c_int / 2 as libc::c_int) as int8_t;
        highestYValueForNorth =
            (lowestYValueForSouth as libc::c_int - 1 as libc::c_int) as int8_t
    };
}
unsafe extern "C" fn updateLedCount() {
    let mut count: libc::c_int = 0 as libc::c_int;
    let mut countRing: libc::c_int = 0 as libc::c_int;
    let mut countScanner: libc::c_int = 0 as libc::c_int;
    let mut ledIndex: libc::c_int = 0 as libc::c_int;
    while ledIndex < 32 as libc::c_int {
        let mut ledConfig: *const ledConfig_t =
            &*(*(ledStripConfig as
                     unsafe extern "C" fn()
                         ->
                             *const ledStripConfig_t)()).ledConfigs.as_ptr().offset(ledIndex
                                                                                        as
                                                                                        isize)
                as *const ledConfig_t;
        if *ledConfig == 0 { break ; }
        count += 1;
        if ledGetFunction(ledConfig) as libc::c_int ==
               LED_FUNCTION_THRUST_RING as libc::c_int {
            countRing += 1
        }
        if ledGetOverlayBit(ledConfig,
                            LED_OVERLAY_LARSON_SCANNER as libc::c_int) {
            countScanner += 1
        }
        ledIndex += 1
    }
    ledCounts.count = count as uint8_t;
    ledCounts.ring = countRing as uint8_t;
    ledCounts.larson = countScanner as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn reevaluateLedConfig() {
    updateLedCount();
    updateDimensions();
    updateLedRingCounts();
    updateRequiredOverlay();
}
// get specialColor by index
unsafe extern "C" fn getSC(mut index: ledSpecialColorIds_e)
 -> *const hsvColor_t {
    return &*(*(ledStripConfig as
                    unsafe extern "C" fn()
                        ->
                            *const ledStripConfig_t)()).colors.as_ptr().offset(*(*(ledStripConfig
                                                                                       as
                                                                                       unsafe extern "C" fn()
                                                                                           ->
                                                                                               *const ledStripConfig_t)()).specialColors.color.as_ptr().offset(index
                                                                                                                                                                   as
                                                                                                                                                                   isize)
                                                                                   as
                                                                                   isize)
               as *const hsvColor_t; // initialize to prevent warnings
}
static mut directionCodes: [libc::c_char; 6] =
    ['N' as i32 as libc::c_char, 'E' as i32 as libc::c_char,
     'S' as i32 as libc::c_char, 'W' as i32 as libc::c_char,
     'U' as i32 as libc::c_char, 'D' as i32 as libc::c_char];
static mut baseFunctionCodes: [libc::c_char; 7] =
    ['C' as i32 as libc::c_char, 'F' as i32 as libc::c_char,
     'A' as i32 as libc::c_char, 'L' as i32 as libc::c_char,
     'S' as i32 as libc::c_char, 'G' as i32 as libc::c_char,
     'R' as i32 as libc::c_char];
static mut overlayCodes: [libc::c_char; 6] =
    ['T' as i32 as libc::c_char, 'O' as i32 as libc::c_char,
     'B' as i32 as libc::c_char, 'V' as i32 as libc::c_char,
     'I' as i32 as libc::c_char, 'W' as i32 as libc::c_char];
#[no_mangle]
pub unsafe extern "C" fn parseLedStripConfig(mut ledIndex: libc::c_int,
                                             mut config: *const libc::c_char)
 -> bool {
    if ledIndex >= 32 as libc::c_int { return 0 as libc::c_int != 0 }
    static mut chunkSeparators: [libc::c_char; 5] =
        [',' as i32 as libc::c_char, ':' as i32 as libc::c_char,
         ':' as i32 as libc::c_char, ':' as i32 as libc::c_char,
         '\u{0}' as i32 as libc::c_char];
    let mut ledConfig: *mut ledConfig_t =
        &mut *(*(ledStripConfigMutable as
                     unsafe extern "C" fn()
                         ->
                             *mut ledStripConfig_t)()).ledConfigs.as_mut_ptr().offset(ledIndex
                                                                                          as
                                                                                          isize)
            as *mut ledConfig_t;
    memset(ledConfig as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<ledConfig_t>() as libc::c_ulong);
    let mut x: libc::c_int = 0 as libc::c_int;
    let mut y: libc::c_int = 0 as libc::c_int;
    let mut color: libc::c_int = 0 as libc::c_int;
    let mut baseFunction: libc::c_int = 0 as libc::c_int;
    let mut overlay_flags: libc::c_int = 0 as libc::c_int;
    let mut direction_flags: libc::c_int = 0 as libc::c_int;
    let mut parseState: parseState_e = X_COORDINATE;
    while (parseState as libc::c_uint) <
              PARSE_STATE_COUNT as libc::c_int as libc::c_uint {
        let mut chunk: [libc::c_char; 11] = [0; 11];
        let mut chunkSeparator: libc::c_char =
            chunkSeparators[parseState as usize];
        let mut chunkIndex: libc::c_int = 0 as libc::c_int;
        while *config as libc::c_int != 0 &&
                  *config as libc::c_int != chunkSeparator as libc::c_int &&
                  chunkIndex < 11 as libc::c_int - 1 as libc::c_int {
            let fresh0 = config;
            config = config.offset(1);
            let fresh1 = chunkIndex;
            chunkIndex = chunkIndex + 1;
            chunk[fresh1 as usize] = *fresh0
        }
        // skip separator
        let fresh2 = chunkIndex; // zero-terminate chunk
        chunkIndex = chunkIndex + 1;
        chunk[fresh2 as usize] = 0 as libc::c_int as libc::c_char;
        if *config as libc::c_int != chunkSeparator as libc::c_int {
            return 0 as libc::c_int != 0
        }
        config = config.offset(1);
        match parseState as libc::c_uint {
            0 => {
                x = atoi(chunk.as_mut_ptr())
                // prevent warning
            }
            1 => { y = atoi(chunk.as_mut_ptr()) }
            2 => {
                let mut ch: *mut libc::c_char = chunk.as_mut_ptr();
                while *ch != 0 {
                    let mut dir: ledDirectionId_e = LED_DIRECTION_NORTH;
                    while (dir as libc::c_uint) <
                              6 as libc::c_int as libc::c_uint {
                        if directionCodes[dir as usize] as libc::c_int ==
                               *ch as libc::c_int {
                            direction_flags |=
                                (1 as libc::c_int) << dir as libc::c_uint;
                            break ;
                        } else { dir += 1 }
                    }
                    ch = ch.offset(1)
                }
            }
            3 => {
                let mut ch_0: *mut libc::c_char = chunk.as_mut_ptr();
                while *ch_0 != 0 {
                    let mut fn_0: ledBaseFunctionId_e = LED_FUNCTION_COLOR;
                    while (fn_0 as libc::c_uint) <
                              7 as libc::c_int as libc::c_uint {
                        if baseFunctionCodes[fn_0 as usize] as libc::c_int ==
                               *ch_0 as libc::c_int {
                            baseFunction = fn_0 as libc::c_int;
                            break ;
                        } else { fn_0 += 1 }
                    }
                    let mut ol: ledOverlayId_e = LED_OVERLAY_THROTTLE;
                    while (ol as libc::c_uint) <
                              6 as libc::c_int as libc::c_uint {
                        if overlayCodes[ol as usize] as libc::c_int ==
                               *ch_0 as libc::c_int {
                            overlay_flags |=
                                (1 as libc::c_int) << ol as libc::c_uint;
                            break ;
                        } else { ol += 1 }
                    }
                    ch_0 = ch_0.offset(1)
                }
            }
            4 => {
                color = atoi(chunk.as_mut_ptr());
                if color >= 16 as libc::c_int { color = 0 as libc::c_int }
            }
            5 | _ => { }
        }
        parseState += 1
    }
    *ledConfig =
        (((x & 0xf as libc::c_int) << 4 as libc::c_int |
              (y & 0xf as libc::c_int) << 0 as libc::c_int) <<
             0 as libc::c_int | color << 18 as libc::c_int |
             direction_flags << 22 as libc::c_int |
             baseFunction << 8 as libc::c_int |
             overlay_flags << 12 as libc::c_int |
             (0 as libc::c_int) << 28 as libc::c_int) as ledConfig_t;
    reevaluateLedConfig();
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn generateLedConfig(mut ledConfig: *mut ledConfig_t,
                                           mut ledConfigBuffer:
                                               *mut libc::c_char,
                                           mut bufferSize: size_t) {
    let mut directions: [libc::c_char; 7] = [0; 7];
    let mut baseFunctionOverlays: [libc::c_char; 8] = [0; 8];
    memset(ledConfigBuffer as *mut libc::c_void, 0 as libc::c_int,
           bufferSize);
    let mut dptr: *mut libc::c_char = directions.as_mut_ptr();
    let mut dir: ledDirectionId_e = LED_DIRECTION_NORTH;
    while (dir as libc::c_uint) < 6 as libc::c_int as libc::c_uint {
        if ledGetDirectionBit(ledConfig, dir as libc::c_int) {
            let fresh3 = dptr;
            dptr = dptr.offset(1);
            *fresh3 = directionCodes[dir as usize]
        }
        dir += 1
    }
    *dptr = 0 as libc::c_int as libc::c_char;
    let mut fptr: *mut libc::c_char = baseFunctionOverlays.as_mut_ptr();
    let fresh4 = fptr;
    fptr = fptr.offset(1);
    *fresh4 = baseFunctionCodes[ledGetFunction(ledConfig) as usize];
    let mut ol: ledOverlayId_e = LED_OVERLAY_THROTTLE;
    while (ol as libc::c_uint) < 6 as libc::c_int as libc::c_uint {
        if ledGetOverlayBit(ledConfig, ol as libc::c_int) {
            let fresh5 = fptr;
            fptr = fptr.offset(1);
            *fresh5 = overlayCodes[ol as usize]
        }
        ol += 1
    }
    *fptr = 0 as libc::c_int as libc::c_char;
    // TODO - check buffer length
    tfp_sprintf(ledConfigBuffer,
                b"%u,%u:%s:%s:%u\x00" as *const u8 as *const libc::c_char,
                ledGetX(ledConfig) as libc::c_int,
                ledGetY(ledConfig) as libc::c_int, directions.as_mut_ptr(),
                baseFunctionOverlays.as_mut_ptr(),
                ledGetColor(ledConfig) as libc::c_int);
}
unsafe extern "C" fn getLedQuadrant(ledIndex: libc::c_int) -> quadrant_e {
    let mut ledConfig: *const ledConfig_t =
        &*(*(ledStripConfig as
                 unsafe extern "C" fn()
                     ->
                         *const ledStripConfig_t)()).ledConfigs.as_ptr().offset(ledIndex
                                                                                    as
                                                                                    isize)
            as *const ledConfig_t;
    let mut x: libc::c_int = ledGetX(ledConfig) as libc::c_int;
    let mut y: libc::c_int = ledGetY(ledConfig) as libc::c_int;
    let mut quad: libc::c_int = 0 as libc::c_int;
    if y <= highestYValueForNorth as libc::c_int {
        quad |= QUADRANT_NORTH as libc::c_int
    } else if y >= lowestYValueForSouth as libc::c_int {
        quad |= QUADRANT_SOUTH as libc::c_int
    }
    if x >= lowestXValueForEast as libc::c_int {
        quad |= QUADRANT_EAST as libc::c_int
    } else if x <= highestXValueForWest as libc::c_int {
        quad |= QUADRANT_WEST as libc::c_int
    }
    return quad as quadrant_e;
}
unsafe extern "C" fn getDirectionalModeColor(ledIndex: libc::c_int,
                                             mut modeColors_0:
                                                 *const modeColorIndexes_t)
 -> *mut hsvColor_t {
    let mut ledConfig: *const ledConfig_t =
        &*(*(ledStripConfig as
                 unsafe extern "C" fn()
                     ->
                         *const ledStripConfig_t)()).ledConfigs.as_ptr().offset(ledIndex
                                                                                    as
                                                                                    isize)
            as *const ledConfig_t;
    let ledDirection: libc::c_int = ledGetDirection(ledConfig) as libc::c_int;
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i < 6 as libc::c_int as libc::c_uint {
        if ledDirection & (1 as libc::c_int) << i != 0 {
            return &mut *(*(ledStripConfigMutable as
                                unsafe extern "C" fn()
                                    ->
                                        *mut ledStripConfig_t)()).colors.as_mut_ptr().offset(*(*modeColors_0).color.as_ptr().offset(i
                                                                                                                                        as
                                                                                                                                        isize)
                                                                                                 as
                                                                                                 isize)
                       as *mut hsvColor_t
        }
        i = i.wrapping_add(1)
    }
    return 0 as *mut hsvColor_t;
}
static mut flightModeToLed: [C2RustUnnamed_2; 5] =
    [{
         let mut init =
             C2RustUnnamed_2{flightMode:
                                 HEADFREE_MODE as libc::c_int as uint16_t,
                             ledMode:
                                 LED_MODE_HEADFREE as libc::c_int as
                                     uint8_t,};
         init
     },
     {
         let mut init =
             C2RustUnnamed_2{flightMode: BARO_MODE as libc::c_int as uint16_t,
                             ledMode:
                                 LED_MODE_BARO as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             C2RustUnnamed_2{flightMode:
                                 HORIZON_MODE as libc::c_int as uint16_t,
                             ledMode:
                                 LED_MODE_HORIZON as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             C2RustUnnamed_2{flightMode:
                                 ANGLE_MODE as libc::c_int as uint16_t,
                             ledMode:
                                 LED_MODE_ANGLE as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             C2RustUnnamed_2{flightMode: 0 as libc::c_int as uint16_t,
                             ledMode:
                                 LED_MODE_ORIENTATION as libc::c_int as
                                     uint8_t,};
         init
     }];
unsafe extern "C" fn applyLedFixedLayers() {
    let mut ledIndex: libc::c_int = 0 as libc::c_int;
    while ledIndex < ledCounts.count as libc::c_int {
        let mut ledConfig: *const ledConfig_t =
            &*(*(ledStripConfig as
                     unsafe extern "C" fn()
                         ->
                             *const ledStripConfig_t)()).ledConfigs.as_ptr().offset(ledIndex
                                                                                        as
                                                                                        isize)
                as *const ledConfig_t;
        let mut color: hsvColor_t = *getSC(LED_SCOLOR_BACKGROUND);
        let mut fn_0: libc::c_int = ledGetFunction(ledConfig) as libc::c_int;
        let mut hOffset: libc::c_int = 359 as libc::c_int + 1 as libc::c_int;
        let mut nextColor: hsvColor_t = hsvColor_t{h: 0, s: 0, v: 0,};
        let mut previousColor: hsvColor_t = hsvColor_t{h: 0, s: 0, v: 0,};
        match fn_0 {
            0 => {
                color =
                    (*ledStripConfig()).colors[ledGetColor(ledConfig) as
                                                   usize];
                nextColor =
                    (*ledStripConfig()).colors[((ledGetColor(ledConfig) as
                                                     libc::c_int +
                                                     1 as libc::c_int +
                                                     16 as libc::c_int) %
                                                    16 as libc::c_int) as
                                                   usize];
                previousColor =
                    (*ledStripConfig()).colors[((ledGetColor(ledConfig) as
                                                     libc::c_int -
                                                     1 as libc::c_int +
                                                     16 as libc::c_int) %
                                                    16 as libc::c_int) as
                                                   usize];
                if ledGetOverlayBit(ledConfig,
                                    LED_OVERLAY_THROTTLE as libc::c_int) {
                    //smooth fade with selected Aux channel of all HSV values from previousColor through color to nextColor
                    let mut centerPWM: libc::c_int =
                        (1000 as libc::c_int + 2000 as libc::c_int) /
                            2 as libc::c_int;
                    if auxInput < centerPWM {
                        color.h =
                            scaleRange(auxInput, 1000 as libc::c_int,
                                       centerPWM,
                                       previousColor.h as libc::c_int,
                                       color.h as libc::c_int) as uint16_t;
                        color.s =
                            scaleRange(auxInput, 1000 as libc::c_int,
                                       centerPWM,
                                       previousColor.s as libc::c_int,
                                       color.s as libc::c_int) as uint8_t;
                        color.v =
                            scaleRange(auxInput, 1000 as libc::c_int,
                                       centerPWM,
                                       previousColor.v as libc::c_int,
                                       color.v as libc::c_int) as uint8_t
                    } else {
                        color.h =
                            scaleRange(auxInput, centerPWM,
                                       2000 as libc::c_int,
                                       color.h as libc::c_int,
                                       nextColor.h as libc::c_int) as
                                uint16_t;
                        color.s =
                            scaleRange(auxInput, centerPWM,
                                       2000 as libc::c_int,
                                       color.s as libc::c_int,
                                       nextColor.s as libc::c_int) as uint8_t;
                        color.v =
                            scaleRange(auxInput, centerPWM,
                                       2000 as libc::c_int,
                                       color.v as libc::c_int,
                                       nextColor.v as libc::c_int) as uint8_t
                    }
                }
            }
            1 => {
                let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
                while (i as libc::c_ulong) <
                          (::core::mem::size_of::<[C2RustUnnamed_2; 5]>() as
                               libc::c_ulong).wrapping_div(::core::mem::size_of::<C2RustUnnamed_2>()
                                                               as
                                                               libc::c_ulong)
                      {
                    if flightModeToLed[i as usize].flightMode == 0 ||
                           flightModeFlags as libc::c_int &
                               flightModeToLed[i as usize].flightMode as
                                   libc::c_int != 0 {
                        let mut directionalColor: *const hsvColor_t =
                            getDirectionalModeColor(ledIndex,
                                                    &*(*(ledStripConfig as
                                                             unsafe extern "C" fn()
                                                                 ->
                                                                     *const ledStripConfig_t)()).modeColors.as_ptr().offset((*flightModeToLed.as_ptr().offset(i
                                                                                                                                                                  as
                                                                                                                                                                  isize)).ledMode
                                                                                                                                as
                                                                                                                                isize));
                        if !directionalColor.is_null() {
                            color = *directionalColor
                        }
                        break ;
                        // stop on first match
                    } else { i = i.wrapping_add(1) }
                }
            }
            2 => {
                color =
                    if armingFlags as libc::c_int & ARMED as libc::c_int != 0
                       {
                        *getSC(LED_SCOLOR_ARMED)
                    } else { *getSC(LED_SCOLOR_DISARMED) }
            }
            3 => {
                color =
                    hsv[COLOR_RED as libc::c_int as
                            usize]; // non-zero during blinks
                hOffset +=
                    scaleRange(calculateBatteryPercentageRemaining() as
                                   libc::c_int, 0 as libc::c_int,
                               100 as libc::c_int, -(30 as libc::c_int),
                               120 as libc::c_int)
            }
            4 => {
                color = hsv[COLOR_RED as libc::c_int as usize];
                hOffset +=
                    scaleRange(getRssiPercent() as libc::c_int,
                               0 as libc::c_int, 100 as libc::c_int,
                               -(30 as libc::c_int), 120 as libc::c_int)
            }
            _ => { }
        }
        if fn_0 != LED_FUNCTION_COLOR as libc::c_int &&
               ledGetOverlayBit(ledConfig,
                                LED_OVERLAY_THROTTLE as libc::c_int) as
                   libc::c_int != 0 {
            hOffset +=
                scaleRange(auxInput, 1000 as libc::c_int, 2000 as libc::c_int,
                           0 as libc::c_int,
                           359 as libc::c_int + 1 as libc::c_int)
        }
        color.h =
            ((color.h as libc::c_int + hOffset) %
                 (359 as libc::c_int + 1 as libc::c_int)) as uint16_t;
        setLedHsv(ledIndex as uint16_t, &mut color);
        ledIndex += 1
    };
}
unsafe extern "C" fn applyLedHsv(mut mask: uint32_t,
                                 mut color: *const hsvColor_t) {
    let mut ledIndex: libc::c_int = 0 as libc::c_int;
    while ledIndex < ledCounts.count as libc::c_int {
        let mut ledConfig: *const ledConfig_t =
            &*(*(ledStripConfig as
                     unsafe extern "C" fn()
                         ->
                             *const ledStripConfig_t)()).ledConfigs.as_ptr().offset(ledIndex
                                                                                        as
                                                                                        isize)
                as *const ledConfig_t;
        if *ledConfig & mask == mask {
            setLedHsv(ledIndex as uint16_t, color);
        }
        ledIndex += 1
    };
}
unsafe extern "C" fn applyLedWarningLayer(mut updateNow: bool,
                                          mut timer: *mut timeUs_t) {
    static mut warningFlashCounter: uint8_t = 0 as libc::c_int as uint8_t;
    static mut warningFlags: uint8_t = 0 as libc::c_int as uint8_t;
    if updateNow {
        // keep counter running, so it stays in sync with blink
        warningFlashCounter = warningFlashCounter.wrapping_add(1);
        warningFlashCounter =
            (warningFlashCounter as libc::c_int & 0xf as libc::c_int) as
                uint8_t;
        if warningFlashCounter as libc::c_int == 0 as libc::c_int {
            // update when old flags was processed
            warningFlags = 0 as libc::c_int as uint8_t; // w_w_
            if (*batteryConfig()).voltageMeterSource as libc::c_uint !=
                   VOLTAGE_METER_NONE as libc::c_int as libc::c_uint &&
                   getBatteryState() as libc::c_uint !=
                       BATTERY_OK as libc::c_int as libc::c_uint {
                warningFlags =
                    (warningFlags as libc::c_int |
                         (1 as libc::c_int) <<
                             WARNING_LOW_BATTERY as libc::c_int) as uint8_t
            }
            if failsafeIsActive() {
                warningFlags =
                    (warningFlags as libc::c_int |
                         (1 as libc::c_int) <<
                             WARNING_FAILSAFE as libc::c_int) as uint8_t
            }
            if armingFlags as libc::c_int & ARMED as libc::c_int == 0 &&
                   isArmingDisabled() as libc::c_int != 0 {
                warningFlags =
                    (warningFlags as libc::c_int |
                         (1 as libc::c_int) <<
                             WARNING_ARMING_DISABLED as libc::c_int) as
                        uint8_t
            }
        }
        *timer =
            (*timer as
                 libc::c_uint).wrapping_add((1000 as libc::c_int *
                                                 1000 as libc::c_int /
                                                 10 as libc::c_int) as
                                                libc::c_uint) as timeUs_t as
                timeUs_t
    }
    let mut warningColor: *const hsvColor_t = 0 as *const hsvColor_t;
    if warningFlags != 0 {
        let mut colorOn: bool =
            warningFlashCounter as libc::c_int % 2 as libc::c_int ==
                0 as libc::c_int;
        let mut warningId: warningFlags_e =
            (warningFlashCounter as libc::c_int / 4 as libc::c_int) as
                warningFlags_e;
        if warningFlags as libc::c_int &
               (1 as libc::c_int) << warningId as libc::c_uint != 0 {
            match warningId as libc::c_uint {
                0 => {
                    warningColor =
                        if colorOn as libc::c_int != 0 {
                            &*hsv.as_ptr().offset(COLOR_GREEN as libc::c_int
                                                      as isize) as
                                *const hsvColor_t
                        } else {
                            &*hsv.as_ptr().offset(COLOR_BLACK as libc::c_int
                                                      as isize) as
                                *const hsvColor_t
                        }
                }
                1 => {
                    warningColor =
                        if colorOn as libc::c_int != 0 {
                            &*hsv.as_ptr().offset(COLOR_RED as libc::c_int as
                                                      isize) as
                                *const hsvColor_t
                        } else {
                            &*hsv.as_ptr().offset(COLOR_BLACK as libc::c_int
                                                      as isize) as
                                *const hsvColor_t
                        }
                }
                2 => {
                    warningColor =
                        if colorOn as libc::c_int != 0 {
                            &*hsv.as_ptr().offset(COLOR_YELLOW as libc::c_int
                                                      as isize) as
                                *const hsvColor_t
                        } else {
                            &*hsv.as_ptr().offset(COLOR_BLUE as libc::c_int as
                                                      isize) as
                                *const hsvColor_t
                        }
                }
                _ => { }
            }
        }
    } else if isBeeperOn() {
        warningColor =
            &*hsv.as_ptr().offset(COLOR_ORANGE as libc::c_int as isize) as
                *const hsvColor_t
    }
    if !warningColor.is_null() {
        applyLedHsv((((1 as libc::c_int) <<
                          LED_OVERLAY_WARNING as libc::c_int) <<
                         12 as libc::c_int) as uint32_t, warningColor);
    };
}
unsafe extern "C" fn applyLedVtxLayer(mut updateNow: bool,
                                      mut timer: *mut timeUs_t) {
    static mut frequency: uint16_t = 0 as libc::c_int as uint16_t;
    static mut power: uint8_t = 255 as libc::c_int as uint8_t;
    static mut pit: uint8_t = 255 as libc::c_int as uint8_t;
    static mut showSettings: uint8_t = 0 as libc::c_int as uint8_t;
    static mut lastCheck: uint16_t = 0 as libc::c_int as uint16_t;
    static mut blink: bool = 0 as libc::c_int != 0;
    let mut vtxDevice: *const vtxDevice_t = vtxCommonDevice();
    if vtxDevice.is_null() { return }
    let mut band: uint8_t = 255 as libc::c_int as uint8_t;
    let mut channel: uint8_t = 255 as libc::c_int as uint8_t;
    let mut check: uint16_t = 0 as libc::c_int as uint16_t;
    if updateNow {
        // keep counter running, so it stays in sync with vtx
        vtxCommonGetBandAndChannel(vtxDevice, &mut band, &mut channel);
        vtxCommonGetPowerIndex(vtxDevice, &mut power);
        vtxCommonGetPitMode(vtxDevice, &mut pit);
        // check 5 times a second
        frequency =
            vtx58frequencyTable[(band as libc::c_int - 1 as libc::c_int) as
                                    usize][(channel as libc::c_int -
                                                1 as libc::c_int) as
                                               usize]; //subtracting 1 from band and channel so that correct frequency is returned.
                                                                //might not be correct for tramp but should fix smart audio.
        check =
            (pit as libc::c_int + ((power as libc::c_int) << 1 as libc::c_int)
                 + ((band as libc::c_int) << 4 as libc::c_int) +
                 ((channel as libc::c_int) << 8 as libc::c_int)) as uint16_t;
        if showSettings == 0 &&
               check as libc::c_int != lastCheck as libc::c_int {
            // check if last vtx values have changed.
            // display settings for 3 seconds.
            showSettings = 15 as libc::c_int as uint8_t
        } // quick way to check if any settings changed.
        lastCheck = check;
        if showSettings != 0 { showSettings = showSettings.wrapping_sub(1) }
        blink = !blink;
        *timer =
            (*timer as
                 libc::c_uint).wrapping_add((1000 as libc::c_int *
                                                 1000 as libc::c_int /
                                                 5 as libc::c_int) as
                                                libc::c_uint) as timeUs_t as
                timeUs_t
    }
    let mut color: hsvColor_t =
        {
            let mut init =
                hsvColor_s{h: 0 as libc::c_int as uint16_t,
                           s: 0 as libc::c_int as uint8_t,
                           v: 0 as libc::c_int as uint8_t,};
            init
        };
    if showSettings != 0 {
        // show settings
        let mut vtxLedCount: uint8_t = 0 as libc::c_int as uint8_t;
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < ledCounts.count as libc::c_int &&
                  (vtxLedCount as libc::c_int) < 6 as libc::c_int {
            let mut ledConfig: *const ledConfig_t =
                &*(*(ledStripConfig as
                         unsafe extern "C" fn()
                             ->
                                 *const ledStripConfig_t)()).ledConfigs.as_ptr().offset(i
                                                                                            as
                                                                                            isize)
                    as *const ledConfig_t;
            if ledGetOverlayBit(ledConfig, LED_OVERLAY_VTX as libc::c_int) {
                if vtxLedCount as libc::c_int == 0 as libc::c_int {
                    color.h = hsv[COLOR_GREEN as libc::c_int as usize].h;
                    color.s = hsv[COLOR_GREEN as libc::c_int as usize].s;
                    color.v =
                        if blink as libc::c_int != 0 {
                            15 as libc::c_int
                        } else { 0 as libc::c_int } as uint8_t
                    // blink received settings
                } else if vtxLedCount as libc::c_int > 0 as libc::c_int &&
                              power as libc::c_int >=
                                  vtxLedCount as libc::c_int && pit == 0 {
                    // show power
                    color.h = hsv[COLOR_ORANGE as libc::c_int as usize].h;
                    color.s = hsv[COLOR_ORANGE as libc::c_int as usize].s;
                    color.v =
                        if blink as libc::c_int != 0 {
                            15 as libc::c_int
                        } else { 0 as libc::c_int } as uint8_t
                    // blink received settings
                } else {
                    // turn rest off
                    color.h = hsv[COLOR_BLACK as libc::c_int as usize].h;
                    color.s = hsv[COLOR_BLACK as libc::c_int as usize].s;
                    color.v = hsv[COLOR_BLACK as libc::c_int as usize].v
                }
                setLedHsv(i as uint16_t, &mut color);
                vtxLedCount = vtxLedCount.wrapping_add(1)
            }
            i += 1
        }
    } else {
        // show frequency
        // calculate the VTX color based on frequency
        let mut colorIndex: libc::c_int =
            0 as libc::c_int; // blink when in pit mode
        if frequency as libc::c_int <= 5672 as libc::c_int {
            colorIndex = COLOR_WHITE as libc::c_int
        } else if frequency as libc::c_int <= 5711 as libc::c_int {
            colorIndex = COLOR_RED as libc::c_int
        } else if frequency as libc::c_int <= 5750 as libc::c_int {
            colorIndex = COLOR_ORANGE as libc::c_int
        } else if frequency as libc::c_int <= 5789 as libc::c_int {
            colorIndex = COLOR_YELLOW as libc::c_int
        } else if frequency as libc::c_int <= 5829 as libc::c_int {
            colorIndex = COLOR_GREEN as libc::c_int
        } else if frequency as libc::c_int <= 5867 as libc::c_int {
            colorIndex = COLOR_BLUE as libc::c_int
        } else if frequency as libc::c_int <= 5906 as libc::c_int {
            colorIndex = COLOR_DARK_VIOLET as libc::c_int
        } else { colorIndex = COLOR_DEEP_PINK as libc::c_int }
        let mut color_0: hsvColor_t =
            (*ledStripConfig()).colors[colorIndex as usize];
        color_0.v =
            if pit as libc::c_int != 0 {
                if blink as libc::c_int != 0 {
                    15 as libc::c_int
                } else { 0 as libc::c_int }
            } else { 255 as libc::c_int } as uint8_t;
        applyLedHsv((((1 as libc::c_int) << LED_OVERLAY_VTX as libc::c_int) <<
                         12 as libc::c_int) as uint32_t, &mut color_0);
    };
}
unsafe extern "C" fn applyLedBatteryLayer(mut updateNow: bool,
                                          mut timer: *mut timeUs_t) {
    static mut flash: bool = 0 as libc::c_int != 0;
    let mut timerDelayUs: libc::c_int =
        1000 as libc::c_int * 1000 as libc::c_int / 1 as libc::c_int;
    if updateNow {
        match getBatteryState() as libc::c_uint {
            0 => {
                flash = 1 as libc::c_int != 0;
                timerDelayUs =
                    1000 as libc::c_int * 1000 as libc::c_int /
                        1 as libc::c_int
            }
            1 => {
                flash = !flash;
                timerDelayUs =
                    1000 as libc::c_int * 1000 as libc::c_int /
                        2 as libc::c_int
            }
            _ => {
                flash = !flash;
                timerDelayUs =
                    1000 as libc::c_int * 1000 as libc::c_int /
                        8 as libc::c_int
            }
        }
    }
    *timer =
        (*timer as libc::c_uint).wrapping_add(timerDelayUs as libc::c_uint) as
            timeUs_t as timeUs_t;
    if !flash {
        let mut bgc: *const hsvColor_t = getSC(LED_SCOLOR_BACKGROUND);
        applyLedHsv(((LED_FUNCTION_BATTERY as libc::c_int) <<
                         8 as libc::c_int) as uint32_t, bgc);
    };
}
unsafe extern "C" fn applyLedRssiLayer(mut updateNow: bool,
                                       mut timer: *mut timeUs_t) {
    static mut flash: bool = 0 as libc::c_int != 0;
    let mut timerDelay: libc::c_int =
        1000 as libc::c_int * 1000 as libc::c_int / 1 as libc::c_int;
    if updateNow {
        let mut state: libc::c_int = getRssiPercent() as libc::c_int;
        if state > 50 as libc::c_int {
            flash = 1 as libc::c_int != 0;
            timerDelay =
                1000 as libc::c_int * 1000 as libc::c_int / 1 as libc::c_int
        } else if state > 20 as libc::c_int {
            flash = !flash;
            timerDelay =
                1000 as libc::c_int * 1000 as libc::c_int / 2 as libc::c_int
        } else {
            flash = !flash;
            timerDelay =
                1000 as libc::c_int * 1000 as libc::c_int / 8 as libc::c_int
        }
    }
    *timer =
        (*timer as libc::c_uint).wrapping_add(timerDelay as libc::c_uint) as
            timeUs_t as timeUs_t;
    if !flash {
        let mut bgc: *const hsvColor_t = getSC(LED_SCOLOR_BACKGROUND);
        applyLedHsv(((LED_FUNCTION_RSSI as libc::c_int) << 8 as libc::c_int)
                        as uint32_t, bgc);
    };
}
unsafe extern "C" fn applyLedIndicatorLayer(mut updateNow: bool,
                                            mut timer: *mut timeUs_t) {
    static mut flash: bool = 0 as libc::c_int != 0;
    if updateNow {
        if rxIsReceivingSignal() {
            // calculate update frequency
            let mut scale: libc::c_int =
                ({
                     let mut _a: libc::c_float =
                         ({
                              let mut _x: libc::c_float =
                                  rcCommand[ROLL as libc::c_int as
                                                usize]; // 0 - 500
                              if _x > 0 as libc::c_int as libc::c_float {
                                  _x
                              } else { -_x }
                          }); // start increasing frequency right after deadband
                     let mut _b: libc::c_float =
                         ({
                              let mut _x: libc::c_float =
                                  rcCommand[PITCH as libc::c_int as
                                                usize]; // 5 - 50Hz update, 2.5 - 25Hz blink
                              if _x > 0 as libc::c_int as libc::c_float {
                                  _x
                              } else { -_x }
                          }); // TODO - use user color?
                     if _a > _b { _a } else { _b }
                 }) as libc::c_int;
            scale = scale - 25 as libc::c_int;
            *timer =
                (*timer as
                     libc::c_uint).wrapping_add((1000 as libc::c_int *
                                                     1000 as libc::c_int /
                                                     (5 as libc::c_int +
                                                          45 as libc::c_int *
                                                              scale /
                                                              (500 as
                                                                   libc::c_int
                                                                   -
                                                                   25 as
                                                                       libc::c_int)))
                                                    as libc::c_uint) as
                    timeUs_t as timeUs_t;
            flash = !flash
        } else {
            *timer =
                (*timer as
                     libc::c_uint).wrapping_add((1000 as libc::c_int *
                                                     1000 as libc::c_int /
                                                     5 as libc::c_int) as
                                                    libc::c_uint) as timeUs_t
                    as timeUs_t
        }
    }
    if !flash { return }
    let mut flashColor: *const hsvColor_t =
        &*hsv.as_ptr().offset(COLOR_ORANGE as libc::c_int as isize) as
            *const hsvColor_t;
    let mut quadrants: quadrant_e = 0 as quadrant_e;
    if rcCommand[ROLL as libc::c_int as usize] >
           25 as libc::c_int as libc::c_float {
        quadrants =
            ::core::mem::transmute::<libc::c_uint,
                                     quadrant_e>(quadrants as libc::c_uint |
                                                     QUADRANT_EAST as
                                                         libc::c_int as
                                                         libc::c_uint)
    } else if rcCommand[ROLL as libc::c_int as usize] <
                  -(25 as libc::c_int) as libc::c_float {
        quadrants =
            ::core::mem::transmute::<libc::c_uint,
                                     quadrant_e>(quadrants as libc::c_uint |
                                                     QUADRANT_WEST as
                                                         libc::c_int as
                                                         libc::c_uint)
    }
    if rcCommand[PITCH as libc::c_int as usize] >
           25 as libc::c_int as libc::c_float {
        quadrants =
            ::core::mem::transmute::<libc::c_uint,
                                     quadrant_e>(quadrants as libc::c_uint |
                                                     QUADRANT_NORTH as
                                                         libc::c_int as
                                                         libc::c_uint)
    } else if rcCommand[PITCH as libc::c_int as usize] <
                  -(25 as libc::c_int) as libc::c_float {
        quadrants =
            ::core::mem::transmute::<libc::c_uint,
                                     quadrant_e>(quadrants as libc::c_uint |
                                                     QUADRANT_SOUTH as
                                                         libc::c_int as
                                                         libc::c_uint)
    }
    let mut ledIndex: libc::c_int = 0 as libc::c_int;
    while ledIndex < ledCounts.count as libc::c_int {
        let mut ledConfig: *const ledConfig_t =
            &*(*(ledStripConfig as
                     unsafe extern "C" fn()
                         ->
                             *const ledStripConfig_t)()).ledConfigs.as_ptr().offset(ledIndex
                                                                                        as
                                                                                        isize)
                as *const ledConfig_t;
        if ledGetOverlayBit(ledConfig, LED_OVERLAY_INDICATOR as libc::c_int) {
            if getLedQuadrant(ledIndex) as libc::c_uint &
                   quadrants as libc::c_uint != 0 {
                setLedHsv(ledIndex as uint16_t, flashColor);
            }
        }
        ledIndex += 1
    };
}
// 2 on
unsafe extern "C" fn updateLedRingCounts() {
    let mut seqLen: libc::c_int = 0;
    // try to split in segments/rings of exactly ROTATION_SEQUENCE_LED_COUNT leds
    if ledCounts.ring as libc::c_int % 6 as libc::c_int == 0 as libc::c_int {
        seqLen = 6 as libc::c_int
    } else {
        seqLen = ledCounts.ring as libc::c_int;
        // else split up in equal segments/rings of at most ROTATION_SEQUENCE_LED_COUNT leds
        // TODO - improve partitioning (15 leds -> 3x5)
        while seqLen > 6 as libc::c_int &&
                  seqLen % 2 as libc::c_int == 0 as libc::c_int {
            seqLen /= 2 as libc::c_int
        }
    }
    ledCounts.ringSeqLen = seqLen as uint8_t;
}
unsafe extern "C" fn applyLedThrustRingLayer(mut updateNow: bool,
                                             mut timer: *mut timeUs_t) {
    static mut rotationPhase: uint8_t = 0;
    let mut ledRingIndex: libc::c_int = 0 as libc::c_int;
    if updateNow {
        rotationPhase =
            if rotationPhase as libc::c_int > 0 as libc::c_int {
                (rotationPhase as libc::c_int) - 1 as libc::c_int
            } else {
                (ledCounts.ringSeqLen as libc::c_int) - 1 as libc::c_int
            } as uint8_t;
        *timer =
            (*timer as
                 libc::c_uint).wrapping_add((1000 as libc::c_int *
                                                 1000 as libc::c_int /
                                                 (5 as libc::c_int +
                                                      45 as libc::c_int *
                                                          scaledThrottle /
                                                          100 as libc::c_int))
                                                as libc::c_uint) as timeUs_t
                as timeUs_t
        // 5 - 50Hz update rate
    }
    let mut ledIndex: libc::c_int = 0 as libc::c_int;
    while ledIndex < ledCounts.count as libc::c_int {
        let mut ledConfig: *const ledConfig_t =
            &*(*(ledStripConfig as
                     unsafe extern "C" fn()
                         ->
                             *const ledStripConfig_t)()).ledConfigs.as_ptr().offset(ledIndex
                                                                                        as
                                                                                        isize)
                as *const ledConfig_t;
        if ledGetFunction(ledConfig) as libc::c_int ==
               LED_FUNCTION_THRUST_RING as libc::c_int {
            let mut applyColor: bool = false;
            if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
                applyColor =
                    ((ledRingIndex + rotationPhase as libc::c_int) %
                         ledCounts.ringSeqLen as libc::c_int) <
                        2 as libc::c_int
            } else {
                applyColor = ledRingIndex % 2 as libc::c_int == 0
                // alternating pattern
            }
            if applyColor {
                let mut ringColor: *const hsvColor_t =
                    &*(*(ledStripConfig as
                             unsafe extern "C" fn()
                                 ->
                                     *const ledStripConfig_t)()).colors.as_ptr().offset((ledGetColor
                                                                                             as
                                                                                             unsafe extern "C" fn(_:
                                                                                                                      *const ledConfig_t)
                                                                                                 ->
                                                                                                     uint8_t)(ledConfig)
                                                                                            as
                                                                                            isize)
                        as *const hsvColor_t;
                setLedHsv(ledIndex as uint16_t, ringColor);
            }
            ledRingIndex += 1
        }
        ledIndex += 1
    };
}
unsafe extern "C" fn brightnessForLarsonIndex(mut larsonParameters:
                                                  *mut larsonParameters_t,
                                              mut larsonIndex: uint8_t)
 -> libc::c_int {
    let mut offset: libc::c_int =
        larsonIndex as libc::c_int -
            (*larsonParameters).currentIndex as libc::c_int;
    static mut larsonLowValue: libc::c_int = 8 as libc::c_int;
    if ({
            let mut _x: libc::c_int = offset;
            (if _x > 0 as libc::c_int { _x } else { -_x })
        }) > 1 as libc::c_int {
        return larsonLowValue
    }
    if offset == 0 as libc::c_int {
        return (*larsonParameters).currentBrightness as libc::c_int
    }
    if (*larsonParameters).direction as libc::c_int == offset {
        return (*larsonParameters).currentBrightness as libc::c_int -
                   127 as libc::c_int
    }
    return 255 as libc::c_int -
               (*larsonParameters).currentBrightness as libc::c_int;
}
unsafe extern "C" fn larsonScannerNextStep(mut larsonParameters:
                                               *mut larsonParameters_t,
                                           mut delta: libc::c_int) {
    if (*larsonParameters).currentBrightness as libc::c_int >
           255 as libc::c_int - delta {
        (*larsonParameters).currentBrightness = 127 as libc::c_int as uint8_t;
        if (*larsonParameters).currentIndex as libc::c_int >=
               ledCounts.larson as libc::c_int ||
               ((*larsonParameters).currentIndex as libc::c_int) <
                   0 as libc::c_int {
            (*larsonParameters).direction =
                -((*larsonParameters).direction as libc::c_int) as int8_t
        }
        (*larsonParameters).currentIndex =
            ((*larsonParameters).currentIndex as libc::c_int +
                 (*larsonParameters).direction as libc::c_int) as int8_t
    } else {
        (*larsonParameters).currentBrightness =
            ((*larsonParameters).currentBrightness as libc::c_int + delta) as
                uint8_t
    };
}
unsafe extern "C" fn applyLarsonScannerLayer(mut updateNow: bool,
                                             mut timer: *mut timeUs_t) {
    static mut larsonParameters: larsonParameters_t =
        {
            let mut init =
                larsonParameters_s{currentBrightness:
                                       0 as libc::c_int as uint8_t,
                                   currentIndex: 0 as libc::c_int as int8_t,
                                   direction: 1 as libc::c_int as int8_t,};
            init
        };
    if updateNow {
        larsonScannerNextStep(&mut larsonParameters, 15 as libc::c_int);
        *timer =
            (*timer as
                 libc::c_uint).wrapping_add((1000 as libc::c_int *
                                                 1000 as libc::c_int /
                                                 60 as libc::c_int) as
                                                libc::c_uint) as timeUs_t as
                timeUs_t
    }
    let mut scannerLedIndex: libc::c_int = 0 as libc::c_int;
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i < ledCounts.count as libc::c_uint {
        let mut ledConfig: *const ledConfig_t =
            &*(*(ledStripConfig as
                     unsafe extern "C" fn()
                         ->
                             *const ledStripConfig_t)()).ledConfigs.as_ptr().offset(i
                                                                                        as
                                                                                        isize)
                as *const ledConfig_t;
        if ledGetOverlayBit(ledConfig,
                            LED_OVERLAY_LARSON_SCANNER as libc::c_int) {
            let mut ledColor: hsvColor_t = hsvColor_t{h: 0, s: 0, v: 0,};
            getLedHsv(i as uint16_t, &mut ledColor);
            ledColor.v =
                brightnessForLarsonIndex(&mut larsonParameters,
                                         scannerLedIndex as uint8_t) as
                    uint8_t;
            setLedHsv(i as uint16_t, &mut ledColor);
            scannerLedIndex += 1
        }
        i = i.wrapping_add(1)
    };
}
// blink twice, then wait ; either always or just when landing
unsafe extern "C" fn applyLedBlinkLayer(mut updateNow: bool,
                                        mut timer: *mut timeUs_t) {
    let blinkPattern: uint16_t =
        0x8005 as libc::c_int as uint16_t; // 0b1000000000000101;
    static mut blinkMask: uint16_t = 0; // b_b_____...
    if updateNow {
        blinkMask =
            (blinkMask as libc::c_int >> 1 as libc::c_int) as uint16_t;
        if blinkMask as libc::c_int <= 1 as libc::c_int {
            blinkMask = blinkPattern
        }
        *timer =
            (*timer as
                 libc::c_uint).wrapping_add((1000 as libc::c_int *
                                                 1000 as libc::c_int /
                                                 10 as libc::c_int) as
                                                libc::c_uint) as timeUs_t as
                timeUs_t
    }
    let mut ledOn: bool = blinkMask as libc::c_int & 1 as libc::c_int != 0;
    if !ledOn {
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < ledCounts.count as libc::c_int {
            let mut ledConfig: *const ledConfig_t =
                &*(*(ledStripConfig as
                         unsafe extern "C" fn()
                             ->
                                 *const ledStripConfig_t)()).ledConfigs.as_ptr().offset(i
                                                                                            as
                                                                                            isize)
                    as *const ledConfig_t;
            if ledGetOverlayBit(ledConfig, LED_OVERLAY_BLINK as libc::c_int) {
                setLedHsv(i as uint16_t, getSC(LED_SCOLOR_BLINKBACKGROUND));
            }
            i += 1
        }
    };
}
static mut timerVal: [timeUs_t; 8] = [0; 8];
static mut disabledTimerMask: uint16_t = 0;
static mut layerTable: [Option<applyLayerFn_timed>; 8] =
    [Some(applyLedBlinkLayer as
              unsafe extern "C" fn(_: bool, _: *mut timeUs_t) -> ()),
     Some(applyLarsonScannerLayer as
              unsafe extern "C" fn(_: bool, _: *mut timeUs_t) -> ()),
     Some(applyLedBatteryLayer as
              unsafe extern "C" fn(_: bool, _: *mut timeUs_t) -> ()),
     Some(applyLedRssiLayer as
              unsafe extern "C" fn(_: bool, _: *mut timeUs_t) -> ()),
     Some(applyLedWarningLayer as
              unsafe extern "C" fn(_: bool, _: *mut timeUs_t) -> ()),
     Some(applyLedVtxLayer as
              unsafe extern "C" fn(_: bool, _: *mut timeUs_t) -> ()),
     Some(applyLedIndicatorLayer as
              unsafe extern "C" fn(_: bool, _: *mut timeUs_t) -> ()),
     Some(applyLedThrustRingLayer as
              unsafe extern "C" fn(_: bool, _: *mut timeUs_t) -> ())];
#[no_mangle]
pub unsafe extern "C" fn isOverlayTypeUsed(mut overlayType: ledOverlayId_e)
 -> bool {
    let mut ledIndex: libc::c_int = 0 as libc::c_int;
    while ledIndex < ledCounts.count as libc::c_int {
        let mut ledConfig: *const ledConfig_t =
            &*(*(ledStripConfig as
                     unsafe extern "C" fn()
                         ->
                             *const ledStripConfig_t)()).ledConfigs.as_ptr().offset(ledIndex
                                                                                        as
                                                                                        isize)
                as *const ledConfig_t;
        if ledGetOverlayBit(ledConfig, overlayType as libc::c_int) {
            return 1 as libc::c_int != 0
        }
        ledIndex += 1
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn updateRequiredOverlay() {
    disabledTimerMask = 0 as libc::c_int as uint16_t;
    disabledTimerMask =
        (disabledTimerMask as libc::c_int |
             (!isOverlayTypeUsed(LED_OVERLAY_BLINK) as libc::c_int) <<
                 timBlink as libc::c_int) as uint16_t;
    disabledTimerMask =
        (disabledTimerMask as libc::c_int |
             (!isOverlayTypeUsed(LED_OVERLAY_LARSON_SCANNER) as libc::c_int)
                 << timLarson as libc::c_int) as uint16_t;
    disabledTimerMask =
        (disabledTimerMask as libc::c_int |
             (!isOverlayTypeUsed(LED_OVERLAY_WARNING) as libc::c_int) <<
                 timWarning as libc::c_int) as uint16_t;
    disabledTimerMask =
        (disabledTimerMask as libc::c_int |
             (!isOverlayTypeUsed(LED_OVERLAY_VTX) as libc::c_int) <<
                 timVtx as libc::c_int) as uint16_t;
    disabledTimerMask =
        (disabledTimerMask as libc::c_int |
             (!isOverlayTypeUsed(LED_OVERLAY_INDICATOR) as libc::c_int) <<
                 timIndicator as libc::c_int) as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn ledStripUpdate(mut currentTimeUs: timeUs_t) {
    if !(ledStripInitialised as libc::c_int != 0 &&
             isWS2811LedStripReady() as libc::c_int != 0) {
        return
    }
    if IS_RC_MODE_ACTIVE(BOXLEDLOW) as libc::c_int != 0 &&
           !((*ledStripConfig()).ledstrip_visual_beeper as libc::c_int != 0 &&
                 isBeeperOn() as libc::c_int != 0) {
        if ledStripEnabled {
            ledStripDisable();
            ledStripEnabled = 0 as libc::c_int != 0
        }
        return
    }
    ledStripEnabled = 1 as libc::c_int != 0;
    let now: uint32_t = currentTimeUs;
    // test all led timers, setting corresponding bits
    let mut timActive: uint32_t = 0 as libc::c_int as uint32_t;
    let mut timId: timId_e = timBlink;
    while (timId as libc::c_uint) <
              timTimerCount as libc::c_int as libc::c_uint {
        if disabledTimerMask as libc::c_int &
               (1 as libc::c_int) << timId as libc::c_uint == 0 {
            // sanitize timer value, so that it can be safely incremented. Handles inital timerVal value.
            let delta: timeDelta_t = cmpTimeUs(now, timerVal[timId as usize]);
            // max delay is limited to 5s
            if !(delta < 0 as libc::c_int &&
                     delta >
                         -(5 as libc::c_int * 1000 as libc::c_int *
                               1000 as libc::c_int)) {
                timActive |=
                    ((1 as libc::c_int) << timId as libc::c_uint) as
                        libc::c_uint; // not ready yet
                if delta >= 100 as libc::c_int * 1000 as libc::c_int ||
                       delta < 0 as libc::c_int {
                    timerVal[timId as usize] = now
                }
            }
        } // no change this update, keep old state
        timId += 1
    }
    if timActive == 0 { return }
    // apply all layers; triggered timed functions has to update timers
    scaledThrottle =
        if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
            scaleRange(rcData[THROTTLE as libc::c_int as usize] as
                           libc::c_int, 1000 as libc::c_int,
                       2000 as libc::c_int, 0 as libc::c_int,
                       100 as libc::c_int)
        } else { 0 as libc::c_int };
    auxInput =
        rcData[(*ledStripConfig()).ledstrip_aux_channel as usize] as
            libc::c_int;
    applyLedFixedLayers();
    let mut timId_0: timId_e = timBlink;
    while (timId_0 as libc::c_ulong) <
              (::core::mem::size_of::<[Option<applyLayerFn_timed>; 8]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<Option<applyLayerFn_timed>>()
                                                   as libc::c_ulong) {
        let mut timer: *mut uint32_t =
            &mut *timerVal.as_mut_ptr().offset(timId_0 as isize) as
                *mut timeUs_t;
        let mut updateNow: bool =
            timActive &
                ((1 as libc::c_int) << timId_0 as libc::c_uint) as
                    libc::c_uint != 0;
        Some((*layerTable.as_mut_ptr().offset(timId_0 as
                                                  isize)).expect("non-null function pointer")).expect("non-null function pointer")(updateNow,
                                                                                                                                   timer);
        timId_0 += 1
    }
    ws2811UpdateStrip((*ledStripConfig()).ledstrip_grb_rgb);
}
#[no_mangle]
pub unsafe extern "C" fn parseColor(mut index: libc::c_int,
                                    mut colorConfig: *const libc::c_char)
 -> bool {
    let mut remainingCharacters: *const libc::c_char = colorConfig;
    let mut color: *mut hsvColor_t =
        &mut *(*(ledStripConfigMutable as
                     unsafe extern "C" fn()
                         ->
                             *mut ledStripConfig_t)()).colors.as_mut_ptr().offset(index
                                                                                      as
                                                                                      isize)
            as *mut hsvColor_t;
    let mut result: bool = 1 as libc::c_int != 0;
    static mut hsv_limit: [uint16_t; 3] =
        [359 as libc::c_int as uint16_t, 255 as libc::c_int as uint16_t,
         255 as libc::c_int as uint16_t];
    let mut componentIndex: libc::c_int = 0 as libc::c_int;
    while result as libc::c_int != 0 &&
              componentIndex < HSV_VALUE as libc::c_int + 1 as libc::c_int {
        let mut val: libc::c_int = atoi(remainingCharacters);
        if val > hsv_limit[componentIndex as usize] as libc::c_int {
            result = 0 as libc::c_int != 0;
            break ;
        } else {
            match componentIndex {
                0 => { (*color).h = val as uint16_t }
                1 => { (*color).s = val as uint8_t }
                2 => { (*color).v = val as uint8_t }
                _ => { }
            }
            remainingCharacters = strchr(remainingCharacters, ',' as i32);
            if !remainingCharacters.is_null() {
                remainingCharacters = remainingCharacters.offset(1)
                // skip separator
            } else if componentIndex <
                          HSV_VALUE as libc::c_int + 1 as libc::c_int -
                              1 as libc::c_int {
                result = 0 as libc::c_int != 0
            }
            componentIndex += 1
        }
    }
    if !result {
        memset(color as *mut libc::c_void, 0 as libc::c_int,
               ::core::mem::size_of::<hsvColor_t>() as libc::c_ulong);
    }
    return result;
}
/*
 * Redefine a color in a mode.
 * */
#[no_mangle]
pub unsafe extern "C" fn setModeColor(mut modeIndex: ledModeIndex_e,
                                      mut modeColorIndex: libc::c_int,
                                      mut colorIndex: libc::c_int) -> bool {
    // check color
    if colorIndex < 0 as libc::c_int || colorIndex >= 16 as libc::c_int {
        return 0 as libc::c_int != 0
    }
    if (modeIndex as libc::c_uint) < 6 as libc::c_int as libc::c_uint {
        // modeIndex_e is unsigned, so one-sided test is enough
        if modeColorIndex < 0 as libc::c_int ||
               modeColorIndex >= 6 as libc::c_int {
            return 0 as libc::c_int != 0
        }
        (*ledStripConfigMutable()).modeColors[modeIndex as
                                                  usize].color[modeColorIndex
                                                                   as usize] =
            colorIndex as uint8_t
    } else if modeIndex as libc::c_uint ==
                  LED_SPECIAL as libc::c_int as libc::c_uint {
        if modeColorIndex < 0 as libc::c_int ||
               modeColorIndex >= 11 as libc::c_int {
            return 0 as libc::c_int != 0
        }
        (*ledStripConfigMutable()).specialColors.color[modeColorIndex as
                                                           usize] =
            colorIndex as uint8_t
    } else if modeIndex as libc::c_uint ==
                  LED_AUX_CHANNEL as libc::c_int as libc::c_uint {
        if modeColorIndex < 0 as libc::c_int ||
               modeColorIndex >= 1 as libc::c_int {
            return 0 as libc::c_int != 0
        }
        (*ledStripConfigMutable()).ledstrip_aux_channel =
            colorIndex as uint8_t
    } else { return 0 as libc::c_int != 0 }
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn ledStripInit() {
    colors = (*ledStripConfigMutable()).colors.as_mut_ptr();
    modeColors = (*ledStripConfig()).modeColors.as_ptr();
    specialColors = (*ledStripConfig()).specialColors;
    ledStripInitialised = 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn ledStripEnable() {
    reevaluateLedConfig();
    ledStripInitialised = 1 as libc::c_int != 0;
    ws2811LedStripInit((*ledStripConfig()).ioTag);
}
unsafe extern "C" fn ledStripDisable() {
    setStripColor(&*hsv.as_ptr().offset(COLOR_BLACK as libc::c_int as isize));
    ws2811UpdateStrip((*ledStripConfig()).ledstrip_grb_rgb);
}
