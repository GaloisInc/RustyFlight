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
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    // (Super) rates are constrained to [0, 100] for Betaflight rates, so values higher than 100 won't make a difference. Range extended for RaceFlight rates.
    #[no_mangle]
    static mut rcCommand: [libc::c_float; 4];
    #[no_mangle]
    fn calculateThrottleStatus() -> throttleStatus_e;
    #[no_mangle]
    fn beeper(mode: beeperMode_e);
    //
// Main API
//
    #[no_mangle]
    fn voltageMeterReset(voltageMeter_0: *mut voltageMeter_t);
    #[no_mangle]
    fn voltageMeterADCInit();
    #[no_mangle]
    fn voltageMeterADCRefresh();
    #[no_mangle]
    fn voltageMeterADCRead(adcChannel: voltageSensorADC_e,
                           voltageMeter_0: *mut voltageMeter_t);
    #[no_mangle]
    fn voltageMeterESCInit();
    #[no_mangle]
    fn voltageMeterESCRefresh();
    #[no_mangle]
    fn voltageMeterESCReadCombined(voltageMeter_0: *mut voltageMeter_t);
    //
// Current Meter API
//
    #[no_mangle]
    fn currentMeterReset(meter: *mut currentMeter_t);
    #[no_mangle]
    fn currentMeterADCInit();
    #[no_mangle]
    fn currentMeterADCRefresh(lastUpdateAt: int32_t);
    #[no_mangle]
    fn currentMeterADCRead(meter: *mut currentMeter_t);
    #[no_mangle]
    fn currentMeterVirtualInit();
    #[no_mangle]
    fn currentMeterVirtualRefresh(lastUpdateAt: int32_t, armed: bool,
                                  throttleLowAndMotorStop: bool,
                                  throttleOffset: int32_t);
    #[no_mangle]
    fn currentMeterVirtualRead(meter: *mut currentMeter_t);
    #[no_mangle]
    fn currentMeterESCRefresh(lastUpdateAt: int32_t);
    #[no_mangle]
    fn currentMeterESCReadCombined(meter: *mut currentMeter_t);
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
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
// microsecond time
pub type timeUs_t = uint32_t;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_3 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_3 = 2;
pub const ARMED: C2RustUnnamed_3 = 1;
pub type rc_alias = libc::c_uint;
pub const AUX8: rc_alias = 11;
pub const AUX7: rc_alias = 10;
pub const AUX6: rc_alias = 9;
pub const AUX5: rc_alias = 8;
pub const AUX4: rc_alias = 7;
pub const AUX3: rc_alias = 6;
pub const AUX2: rc_alias = 5;
pub const AUX1: rc_alias = 4;
pub const THROTTLE: rc_alias = 3;
pub const YAW: rc_alias = 2;
pub const PITCH: rc_alias = 1;
pub const ROLL: rc_alias = 0;
pub type throttleStatus_e = libc::c_uint;
pub const THROTTLE_HIGH: throttleStatus_e = 1;
pub const THROTTLE_LOW: throttleStatus_e = 0;
pub type beeperMode_e = libc::c_uint;
pub const BEEPER_ALL: beeperMode_e = 24;
pub const BEEPER_RC_SMOOTHING_INIT_FAIL: beeperMode_e = 23;
pub const BEEPER_CAM_CONNECTION_CLOSE: beeperMode_e = 22;
pub const BEEPER_CAM_CONNECTION_OPEN: beeperMode_e = 21;
pub const BEEPER_CRASH_FLIP_MODE: beeperMode_e = 20;
pub const BEEPER_BLACKBOX_ERASE: beeperMode_e = 19;
pub const BEEPER_USB: beeperMode_e = 18;
pub const BEEPER_SYSTEM_INIT: beeperMode_e = 17;
pub const BEEPER_ARMED: beeperMode_e = 16;
pub const BEEPER_DISARM_REPEAT: beeperMode_e = 15;
pub const BEEPER_MULTI_BEEPS: beeperMode_e = 14;
pub const BEEPER_READY_BEEP: beeperMode_e = 13;
pub const BEEPER_ACC_CALIBRATION_FAIL: beeperMode_e = 12;
pub const BEEPER_ACC_CALIBRATION: beeperMode_e = 11;
pub const BEEPER_RX_SET: beeperMode_e = 10;
pub const BEEPER_GPS_STATUS: beeperMode_e = 9;
pub const BEEPER_BAT_LOW: beeperMode_e = 8;
pub const BEEPER_BAT_CRIT_LOW: beeperMode_e = 7;
pub const BEEPER_ARMING_GPS_FIX: beeperMode_e = 6;
pub const BEEPER_ARMING: beeperMode_e = 5;
pub const BEEPER_DISARMING: beeperMode_e = 4;
pub const BEEPER_RX_LOST_LANDING: beeperMode_e = 3;
pub const BEEPER_RX_LOST: beeperMode_e = 2;
pub const BEEPER_GYRO_CALIBRATED: beeperMode_e = 1;
pub const BEEPER_SILENCE: beeperMode_e = 0;
pub type currentMeterSource_e = libc::c_uint;
pub const CURRENT_METER_COUNT: currentMeterSource_e = 5;
pub const CURRENT_METER_MSP: currentMeterSource_e = 4;
pub const CURRENT_METER_ESC: currentMeterSource_e = 3;
pub const CURRENT_METER_VIRTUAL: currentMeterSource_e = 2;
pub const CURRENT_METER_ADC: currentMeterSource_e = 1;
pub const CURRENT_METER_NONE: currentMeterSource_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct currentMeter_s {
    pub amperage: int32_t,
    pub amperageLatest: int32_t,
    pub mAhDrawn: int32_t,
}
pub type currentMeter_t = currentMeter_s;
pub type voltageMeterSource_e = libc::c_uint;
pub const VOLTAGE_METER_COUNT: voltageMeterSource_e = 3;
pub const VOLTAGE_METER_ESC: voltageMeterSource_e = 2;
pub const VOLTAGE_METER_ADC: voltageMeterSource_e = 1;
pub const VOLTAGE_METER_NONE: voltageMeterSource_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct voltageMeter_s {
    pub filtered: uint16_t,
    pub unfiltered: uint16_t,
    pub lowVoltageCutoff: bool,
}
// WARNING - do not mix usage of VOLTAGE_METER_* and VOLTAGE_SENSOR_*, they are separate concerns.
pub type voltageMeter_t = voltageMeter_s;
pub type voltageSensorADC_e = libc::c_uint;
pub const VOLTAGE_SENSOR_ADC_5V: voltageSensorADC_e = 3;
pub const VOLTAGE_SENSOR_ADC_9V: voltageSensorADC_e = 2;
pub const VOLTAGE_SENSOR_ADC_12V: voltageSensorADC_e = 1;
pub const VOLTAGE_SENSOR_ADC_VBAT: voltageSensorADC_e = 0;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct lowVoltageCutoff_s {
    pub enabled: bool,
    pub percentage: uint8_t,
    pub startTime: timeUs_t,
}
pub type lowVoltageCutoff_t = lowVoltageCutoff_s;
pub type batteryState_e = libc::c_uint;
pub const BATTERY_INIT: batteryState_e = 4;
pub const BATTERY_NOT_PRESENT: batteryState_e = 3;
pub const BATTERY_CRITICAL: batteryState_e = 2;
pub const BATTERY_WARNING: batteryState_e = 1;
pub const BATTERY_OK: batteryState_e = 0;
#[inline]
unsafe extern "C" fn cmp32(mut a: uint32_t, mut b: uint32_t) -> int32_t {
    return a.wrapping_sub(b) as int32_t;
}
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn constrainf(mut amt: libc::c_float,
                                mut low: libc::c_float,
                                mut high: libc::c_float) -> libc::c_float {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn batteryConfig() -> *const batteryConfig_t {
    return &mut batteryConfig_System;
}
// voltage in 0.1V steps
// voltage in 0.1V steps
//10 secs for the LVC to slowly kick in
// Battery monitoring stuff
#[no_mangle]
pub static mut batteryCellCount: uint8_t = 0;
// Note: this can be 0 when no battery is detected or when the battery voltage sensor is missing or disabled.
#[no_mangle]
pub static mut batteryWarningVoltage: uint16_t = 0;
#[no_mangle]
pub static mut batteryCriticalVoltage: uint16_t = 0;
static mut lowVoltageCutoff: lowVoltageCutoff_t =
    lowVoltageCutoff_t{enabled: false, percentage: 0, startTime: 0,};
//
static mut currentMeter: currentMeter_t =
    currentMeter_t{amperage: 0, amperageLatest: 0, mAhDrawn: 0,};
static mut voltageMeter: voltageMeter_t =
    voltageMeter_t{filtered: 0, unfiltered: 0, lowVoltageCutoff: false,};
static mut batteryState: batteryState_e = BATTERY_OK;
static mut voltageState: batteryState_e = BATTERY_OK;
static mut consumptionState: batteryState_e = BATTERY_OK;
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut batteryConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (11 as libc::c_int |
                                      (2 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<batteryConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &batteryConfig_System as
                                     *const batteryConfig_t as
                                     *mut batteryConfig_t as *mut uint8_t,
                             copy:
                                 &batteryConfig_Copy as *const batteryConfig_t
                                     as *mut batteryConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{ptr:
                                                     &pgResetTemplate_batteryConfig
                                                         as
                                                         *const batteryConfig_t
                                                         as
                                                         *mut libc::c_void,},}; //USB voltage - 2s or more check
            init
        }
    };
#[no_mangle]
pub static mut batteryConfig_System: batteryConfig_t =
    batteryConfig_t{vbatmaxcellvoltage: 0,
                    vbatmincellvoltage: 0,
                    vbatwarningcellvoltage: 0,
                    vbatnotpresentcellvoltage: 0,
                    lvcPercentage: 0,
                    voltageMeterSource: VOLTAGE_METER_NONE,
                    currentMeterSource: CURRENT_METER_NONE,
                    batteryCapacity: 0,
                    useVBatAlerts: false,
                    useConsumptionAlerts: false,
                    consumptionWarningPercentage: 0,
                    vbathysteresis: 0,
                    vbatfullcellvoltage: 0,};
#[no_mangle]
pub static mut batteryConfig_Copy: batteryConfig_t =
    batteryConfig_t{vbatmaxcellvoltage: 0,
                    vbatmincellvoltage: 0,
                    vbatwarningcellvoltage: 0,
                    vbatnotpresentcellvoltage: 0,
                    lvcPercentage: 0,
                    voltageMeterSource: VOLTAGE_METER_NONE,
                    currentMeterSource: CURRENT_METER_NONE,
                    batteryCapacity: 0,
                    useVBatAlerts: false,
                    useConsumptionAlerts: false,
                    consumptionWarningPercentage: 0,
                    vbathysteresis: 0,
                    vbatfullcellvoltage: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_batteryConfig: batteryConfig_t =
    {
        let mut init =
            batteryConfig_s{vbatmaxcellvoltage: 43 as libc::c_int as uint8_t,
                            vbatmincellvoltage: 33 as libc::c_int as uint8_t,
                            vbatwarningcellvoltage:
                                35 as libc::c_int as uint8_t,
                            vbatnotpresentcellvoltage:
                                30 as libc::c_int as uint8_t,
                            lvcPercentage: 100 as libc::c_int as uint8_t,
                            voltageMeterSource: VOLTAGE_METER_ADC,
                            currentMeterSource: CURRENT_METER_ADC,
                            batteryCapacity: 0 as libc::c_int as uint16_t,
                            useVBatAlerts: 1 as libc::c_int != 0,
                            useConsumptionAlerts: 0 as libc::c_int != 0,
                            consumptionWarningPercentage:
                                10 as libc::c_int as uint8_t,
                            vbathysteresis: 1 as libc::c_int as uint8_t,
                            vbatfullcellvoltage:
                                41 as libc::c_int as uint8_t,};
        init
    };
#[no_mangle]
pub unsafe extern "C" fn batteryUpdateVoltage(mut currentTimeUs: timeUs_t) {
    match (*batteryConfig()).voltageMeterSource as libc::c_uint {
        2 => {
            if feature(FEATURE_ESC_SENSOR as libc::c_int as uint32_t) {
                voltageMeterESCRefresh();
                voltageMeterESCReadCombined(&mut voltageMeter);
            }
        }
        1 => {
            voltageMeterADCRefresh();
            voltageMeterADCRead(VOLTAGE_SENSOR_ADC_VBAT, &mut voltageMeter);
        }
        0 | _ => { voltageMeterReset(&mut voltageMeter); }
    }
    if debugMode as libc::c_int == DEBUG_BATTERY as libc::c_int {
        debug[0 as libc::c_int as usize] = voltageMeter.unfiltered as int16_t;
        debug[1 as libc::c_int as usize] = voltageMeter.filtered as int16_t
    };
}
unsafe extern "C" fn updateBatteryBeeperAlert() {
    match getBatteryState() as libc::c_uint {
        1 => { beeper(BEEPER_BAT_LOW); }
        2 => { beeper(BEEPER_BAT_CRIT_LOW); }
        0 | 3 | 4 | _ => { }
    };
}
#[no_mangle]
pub unsafe extern "C" fn batteryUpdatePresence() {
    let mut isVoltageStable: bool =
        ({
             let mut _x: libc::c_int =
                 voltageMeter.filtered as libc::c_int -
                     voltageMeter.unfiltered as libc::c_int;
             (if _x > 0 as libc::c_int { _x } else { -_x })
         }) <= 2 as libc::c_int;
    let mut isVoltageFromBat: bool =
        voltageMeter.filtered as libc::c_int >=
            (*batteryConfig()).vbatnotpresentcellvoltage as libc::c_int &&
            voltageMeter.filtered as libc::c_int <=
                (*batteryConfig()).vbatmaxcellvoltage as libc::c_int ||
            voltageMeter.filtered as libc::c_int >
                (*batteryConfig()).vbatnotpresentcellvoltage as libc::c_int *
                    2 as libc::c_int;
    if (voltageState as libc::c_uint ==
            BATTERY_NOT_PRESENT as libc::c_int as libc::c_uint ||
            voltageState as libc::c_uint ==
                BATTERY_INIT as libc::c_int as libc::c_uint) &&
           isVoltageFromBat as libc::c_int != 0 &&
           isVoltageStable as libc::c_int != 0 {
        /* Want to disable battery getting detected around USB voltage or 0V*/
        /* battery has just been connected - calculate cells, warning voltages and reset state */
        let mut cells: libc::c_uint =
            (voltageMeter.filtered as libc::c_int /
                 (*batteryConfig()).vbatmaxcellvoltage as libc::c_int +
                 1 as libc::c_int) as libc::c_uint;
        if cells > 8 as libc::c_int as libc::c_uint {
            // something is wrong, we expect 8 cells maximum (and autodetection will be problematic at 6+ cells)
            cells = 8 as libc::c_int as libc::c_uint
        }
        voltageState = BATTERY_OK;
        consumptionState = voltageState;
        batteryCellCount = cells as uint8_t;
        batteryWarningVoltage =
            (batteryCellCount as libc::c_int *
                 (*batteryConfig()).vbatwarningcellvoltage as libc::c_int) as
                uint16_t;
        batteryCriticalVoltage =
            (batteryCellCount as libc::c_int *
                 (*batteryConfig()).vbatmincellvoltage as libc::c_int) as
                uint16_t;
        lowVoltageCutoff.percentage = 100 as libc::c_int as uint8_t;
        lowVoltageCutoff.startTime = 0 as libc::c_int as timeUs_t
    } else if voltageState as libc::c_uint !=
                  BATTERY_NOT_PRESENT as libc::c_int as libc::c_uint &&
                  isVoltageStable as libc::c_int != 0 && !isVoltageFromBat {
        /* battery has been disconnected - can take a while for filter cap to disharge so we use a threshold of batteryConfig()->vbatnotpresentcellvoltage */
        voltageState = BATTERY_NOT_PRESENT;
        consumptionState = voltageState;
        batteryCellCount = 0 as libc::c_int as uint8_t;
        batteryWarningVoltage = 0 as libc::c_int as uint16_t;
        batteryCriticalVoltage = 0 as libc::c_int as uint16_t
    }
    if debugMode as libc::c_int == DEBUG_BATTERY as libc::c_int {
        debug[2 as libc::c_int as usize] = batteryCellCount as int16_t;
        debug[3 as libc::c_int as usize] = isVoltageStable as int16_t
    };
}
unsafe extern "C" fn batteryUpdateVoltageState() {
    // alerts are currently used by beeper, osd and other subsystems
    match voltageState as libc::c_uint {
        0 => {
            if voltageMeter.filtered as libc::c_int <=
                   batteryWarningVoltage as libc::c_int -
                       (*batteryConfig()).vbathysteresis as libc::c_int {
                voltageState = BATTERY_WARNING
            }
        }
        1 => {
            if voltageMeter.filtered as libc::c_int <=
                   batteryCriticalVoltage as libc::c_int -
                       (*batteryConfig()).vbathysteresis as libc::c_int {
                voltageState = BATTERY_CRITICAL
            } else if voltageMeter.filtered as libc::c_int >
                          batteryWarningVoltage as libc::c_int {
                voltageState = BATTERY_OK
            }
        }
        2 => {
            if voltageMeter.filtered as libc::c_int >
                   batteryCriticalVoltage as libc::c_int {
                voltageState = BATTERY_WARNING
            }
        }
        _ => { }
    };
}
unsafe extern "C" fn batteryUpdateLVC(mut currentTimeUs: timeUs_t) {
    if ((*batteryConfig()).lvcPercentage as libc::c_int) < 100 as libc::c_int
       {
        if voltageState as libc::c_uint ==
               BATTERY_CRITICAL as libc::c_int as libc::c_uint &&
               !lowVoltageCutoff.enabled {
            lowVoltageCutoff.enabled = 1 as libc::c_int != 0;
            lowVoltageCutoff.startTime = currentTimeUs;
            lowVoltageCutoff.percentage = 100 as libc::c_int as uint8_t
        }
        if lowVoltageCutoff.enabled {
            if cmp32(currentTimeUs, lowVoltageCutoff.startTime) <
                   10000000 as libc::c_int {
                lowVoltageCutoff.percentage =
                    (100 as libc::c_int -
                         cmp32(currentTimeUs, lowVoltageCutoff.startTime) *
                             (100 as libc::c_int -
                                  (*batteryConfig()).lvcPercentage as
                                      libc::c_int) / 10000000 as libc::c_int)
                        as uint8_t
            } else {
                lowVoltageCutoff.percentage = (*batteryConfig()).lvcPercentage
            }
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn batteryUpdateStates(mut currentTimeUs: timeUs_t) {
    batteryUpdateVoltageState();
    batteryUpdateConsumptionState();
    batteryUpdateLVC(currentTimeUs);
    batteryState =
        ({
             let mut _a: batteryState_e = voltageState;
             let mut _b: batteryState_e = consumptionState;
             if _a as libc::c_uint > _b as libc::c_uint {
                 _a as libc::c_uint
             } else { _b as libc::c_uint }
         }) as batteryState_e;
}
#[no_mangle]
pub unsafe extern "C" fn getLowVoltageCutoff() -> *const lowVoltageCutoff_t {
    return &mut lowVoltageCutoff;
}
#[no_mangle]
pub unsafe extern "C" fn getBatteryState() -> batteryState_e {
    return batteryState;
}
#[no_mangle]
pub static mut batteryStateStrings: [*const libc::c_char; 5] =
    [b"OK\x00" as *const u8 as *const libc::c_char,
     b"WARNING\x00" as *const u8 as *const libc::c_char,
     b"CRITICAL\x00" as *const u8 as *const libc::c_char,
     b"NOT PRESENT\x00" as *const u8 as *const libc::c_char,
     b"INIT\x00" as *const u8 as *const libc::c_char];
#[no_mangle]
pub unsafe extern "C" fn getBatteryStateString() -> *const libc::c_char {
    return batteryStateStrings[getBatteryState() as usize];
}
#[no_mangle]
pub unsafe extern "C" fn batteryInit() {
    //
    // presence
    //
    batteryState = BATTERY_INIT;
    batteryCellCount = 0 as libc::c_int as uint8_t;
    //
    // voltage
    //
    voltageState = BATTERY_INIT;
    batteryWarningVoltage = 0 as libc::c_int as uint16_t;
    batteryCriticalVoltage = 0 as libc::c_int as uint16_t;
    lowVoltageCutoff.enabled = 0 as libc::c_int != 0;
    lowVoltageCutoff.percentage = 100 as libc::c_int as uint8_t;
    lowVoltageCutoff.startTime = 0 as libc::c_int as timeUs_t;
    voltageMeterReset(&mut voltageMeter);
    match (*batteryConfig()).voltageMeterSource as libc::c_uint {
        2 => { voltageMeterESCInit(); }
        1 => { voltageMeterADCInit(); }
        _ => { }
    }
    //
    // current
    //
    consumptionState = BATTERY_OK;
    currentMeterReset(&mut currentMeter);
    match (*batteryConfig()).currentMeterSource as libc::c_uint {
        1 => { currentMeterADCInit(); }
        2 => { currentMeterVirtualInit(); }
        3 | 4 | _ => { }
    };
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
/* *
 * terminology: meter vs sensors
 *
 * voltage and current sensors are used to collect data.
 * - e.g. voltage at an MCU ADC input pin, value from an ESC sensor.
 *   sensors require very specific configuration, such as resistor values.
 * voltage and current meters are used to process and expose data collected from sensors to the rest of the system.
 * - e.g. a meter exposes normalized, and often filtered, values from a sensor.
 *   meters require different or little configuration.
 *   meters also have different precision concerns, and may use different units to the sensors.
 *
 */
unsafe extern "C" fn batteryUpdateConsumptionState() {
    if (*batteryConfig()).useConsumptionAlerts as libc::c_int != 0 &&
           (*batteryConfig()).batteryCapacity as libc::c_int >
               0 as libc::c_int &&
           batteryCellCount as libc::c_int > 0 as libc::c_int {
        let mut batteryPercentageRemaining: uint8_t =
            calculateBatteryPercentageRemaining();
        if batteryPercentageRemaining as libc::c_int == 0 as libc::c_int {
            consumptionState = BATTERY_CRITICAL
        } else if batteryPercentageRemaining as libc::c_int <=
                      (*batteryConfig()).consumptionWarningPercentage as
                          libc::c_int {
            consumptionState = BATTERY_WARNING
        } else { consumptionState = BATTERY_OK }
    };
}
#[no_mangle]
pub unsafe extern "C" fn batteryUpdateCurrentMeter(mut currentTimeUs:
                                                       timeUs_t) {
    if batteryCellCount as libc::c_int == 0 as libc::c_int {
        currentMeterReset(&mut currentMeter);
        return
    }
    static mut ibatLastServiced: uint32_t = 0 as libc::c_int as uint32_t;
    let lastUpdateAt: int32_t = cmp32(currentTimeUs, ibatLastServiced);
    ibatLastServiced = currentTimeUs;
    match (*batteryConfig()).currentMeterSource as libc::c_uint {
        1 => {
            currentMeterADCRefresh(lastUpdateAt);
            currentMeterADCRead(&mut currentMeter);
        }
        2 => {
            let mut throttleStatus: throttleStatus_e =
                calculateThrottleStatus();
            let mut throttleLowAndMotorStop: bool =
                throttleStatus as libc::c_uint ==
                    THROTTLE_LOW as libc::c_int as libc::c_uint &&
                    feature(FEATURE_MOTOR_STOP as libc::c_int as uint32_t) as
                        libc::c_int != 0;
            let mut throttleOffset: int32_t =
                rcCommand[THROTTLE as libc::c_int as usize] as int32_t -
                    1000 as libc::c_int;
            currentMeterVirtualRefresh(lastUpdateAt,
                                       armingFlags as libc::c_int &
                                           ARMED as libc::c_int != 0,
                                       throttleLowAndMotorStop,
                                       throttleOffset);
            currentMeterVirtualRead(&mut currentMeter);
        }
        3 => {
            if feature(FEATURE_ESC_SENSOR as libc::c_int as uint32_t) {
                currentMeterESCRefresh(lastUpdateAt);
                currentMeterESCReadCombined(&mut currentMeter);
            }
        }
        4 => { }
        0 | _ => { currentMeterReset(&mut currentMeter); }
    };
}
#[no_mangle]
pub unsafe extern "C" fn calculateVbatPidCompensation() -> libc::c_float {
    let mut batteryScaler: libc::c_float = 1.0f32;
    if (*batteryConfig()).voltageMeterSource as libc::c_uint !=
           VOLTAGE_METER_NONE as libc::c_int as libc::c_uint &&
           batteryCellCount as libc::c_int > 0 as libc::c_int {
        // Up to 33% PID gain. Should be fine for 4,2to 3,3 difference
        batteryScaler =
            constrainf((*batteryConfig()).vbatmaxcellvoltage as libc::c_float
                           * batteryCellCount as libc::c_int as libc::c_float
                           / voltageMeter.filtered as libc::c_float, 1.0f32,
                       1.33f32)
    }
    return batteryScaler;
}
#[no_mangle]
pub unsafe extern "C" fn calculateBatteryPercentageRemaining() -> uint8_t {
    let mut batteryPercentage: uint8_t = 0 as libc::c_int as uint8_t;
    if batteryCellCount as libc::c_int > 0 as libc::c_int {
        let mut batteryCapacity: uint16_t =
            (*batteryConfig()).batteryCapacity;
        if batteryCapacity as libc::c_int > 0 as libc::c_int {
            batteryPercentage =
                constrain(((batteryCapacity as libc::c_float -
                                currentMeter.mAhDrawn as libc::c_float) *
                               100 as libc::c_int as libc::c_float /
                               batteryCapacity as libc::c_int as
                                   libc::c_float) as libc::c_int,
                          0 as libc::c_int, 100 as libc::c_int) as uint8_t
        } else {
            batteryPercentage =
                constrain((voltageMeter.filtered as
                               uint32_t).wrapping_sub(((*batteryConfig()).vbatmincellvoltage
                                                           as libc::c_int *
                                                           batteryCellCount as
                                                               libc::c_int) as
                                                          libc::c_uint).wrapping_mul(100
                                                                                         as
                                                                                         libc::c_int
                                                                                         as
                                                                                         libc::c_uint).wrapping_div((((*batteryConfig()).vbatmaxcellvoltage
                                                                                                                          as
                                                                                                                          libc::c_int
                                                                                                                          -
                                                                                                                          (*batteryConfig()).vbatmincellvoltage
                                                                                                                              as
                                                                                                                              libc::c_int)
                                                                                                                         *
                                                                                                                         batteryCellCount
                                                                                                                             as
                                                                                                                             libc::c_int)
                                                                                                                        as
                                                                                                                        libc::c_uint)
                              as libc::c_int, 0 as libc::c_int,
                          100 as libc::c_int) as uint8_t
        }
    }
    return batteryPercentage;
}
#[no_mangle]
pub unsafe extern "C" fn batteryUpdateAlarms() {
    // use the state to trigger beeper alerts
    if (*batteryConfig()).useVBatAlerts { updateBatteryBeeperAlert(); };
}
#[no_mangle]
pub unsafe extern "C" fn isBatteryVoltageConfigured() -> bool {
    return (*batteryConfig()).voltageMeterSource as libc::c_uint !=
               VOLTAGE_METER_NONE as libc::c_int as libc::c_uint;
}
#[no_mangle]
pub unsafe extern "C" fn getBatteryVoltage() -> uint16_t {
    return voltageMeter.filtered;
}
#[no_mangle]
pub unsafe extern "C" fn getBatteryVoltageLatest() -> uint16_t {
    return voltageMeter.unfiltered;
}
#[no_mangle]
pub unsafe extern "C" fn getBatteryCellCount() -> uint8_t {
    return batteryCellCount;
}
#[no_mangle]
pub unsafe extern "C" fn getBatteryAverageCellVoltage() -> uint16_t {
    return (voltageMeter.filtered as libc::c_int /
                batteryCellCount as libc::c_int) as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn isAmperageConfigured() -> bool {
    return (*batteryConfig()).currentMeterSource as libc::c_uint !=
               CURRENT_METER_NONE as libc::c_int as libc::c_uint;
}
#[no_mangle]
pub unsafe extern "C" fn getAmperage() -> int32_t {
    return currentMeter.amperage;
}
#[no_mangle]
pub unsafe extern "C" fn getAmperageLatest() -> int32_t {
    return currentMeter.amperageLatest;
}
#[no_mangle]
pub unsafe extern "C" fn getMAhDrawn() -> int32_t {
    return currentMeter.mAhDrawn;
}
