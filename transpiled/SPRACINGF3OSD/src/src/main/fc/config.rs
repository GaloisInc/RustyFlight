use ::libc;
extern "C" {
    #[no_mangle]
    fn isEEPROMStructureValid() -> bool;
    #[no_mangle]
    fn loadEEPROM() -> bool;
    #[no_mangle]
    fn writeConfigToEEPROM();
    #[no_mangle]
    fn featureClear(mask: uint32_t);
    #[no_mangle]
    fn pgResetAll();
    #[no_mangle]
    fn targetConfiguration();
    #[no_mangle]
    fn beeperConfirmationBeeps(beepCount: uint8_t);
    #[no_mangle]
    static mut serialConfig_System: serialConfig_t;
    #[no_mangle]
    fn isSerialConfigValid(serialConfig_0: *const serialConfig_t) -> bool;
    #[no_mangle]
    fn pgResetFn_serialConfig(serialConfig_0: *mut serialConfig_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pilotConfig_s {
    pub name: [libc::c_char; 17],
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
pub type pilotConfig_t = pilotConfig_s;
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
pub type serialConfig_t = serialConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialConfig_s {
    pub portConfigs: [serialPortConfig_t; 3],
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
pub type C2RustUnnamed_3 = libc::c_uint;
pub const INPUT_SOURCE_COUNT: C2RustUnnamed_3 = 14;
pub const INPUT_GIMBAL_ROLL: C2RustUnnamed_3 = 13;
pub const INPUT_GIMBAL_PITCH: C2RustUnnamed_3 = 12;
pub const INPUT_RC_AUX4: C2RustUnnamed_3 = 11;
pub const INPUT_RC_AUX3: C2RustUnnamed_3 = 10;
pub const INPUT_RC_AUX2: C2RustUnnamed_3 = 9;
pub const INPUT_RC_AUX1: C2RustUnnamed_3 = 8;
pub const INPUT_RC_THROTTLE: C2RustUnnamed_3 = 7;
pub const INPUT_RC_YAW: C2RustUnnamed_3 = 6;
pub const INPUT_RC_PITCH: C2RustUnnamed_3 = 5;
pub const INPUT_RC_ROLL: C2RustUnnamed_3 = 4;
pub const INPUT_STABILIZED_THROTTLE: C2RustUnnamed_3 = 3;
pub const INPUT_STABILIZED_YAW: C2RustUnnamed_3 = 2;
pub const INPUT_STABILIZED_PITCH: C2RustUnnamed_3 = 1;
pub const INPUT_STABILIZED_ROLL: C2RustUnnamed_3 = 0;
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
#[no_mangle]
pub static mut inputSource_e: C2RustUnnamed_3 = INPUT_STABILIZED_ROLL;
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
unsafe extern "C" fn serialConfigMutable() -> *mut serialConfig_t {
    return &mut serialConfig_System;
}
#[inline]
unsafe extern "C" fn serialConfig() -> *const serialConfig_t {
    return &mut serialConfig_System;
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
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut pilotConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (47 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<pilotConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &pilotConfig_System as *const pilotConfig_t
                                     as *mut pilotConfig_t as *mut uint8_t,
                             copy:
                                 &pilotConfig_Copy as *const pilotConfig_t as
                                     *mut pilotConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{ptr:
                                                     &pgResetTemplate_pilotConfig
                                                         as
                                                         *const pilotConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut pilotConfig_System: pilotConfig_t =
    pilotConfig_t{name: [0; 17],};
#[no_mangle]
pub static mut pilotConfig_Copy: pilotConfig_t =
    pilotConfig_t{name: [0; 17],};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_pilotConfig: pilotConfig_t =
    {
        let mut init =
            pilotConfig_s{name:
                              [0 as libc::c_int as libc::c_char, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],};
        init
    };
#[no_mangle]
pub static mut systemConfig_System: systemConfig_t =
    systemConfig_t{pidProfileIndex: 0,
                   activeRateProfile: 0,
                   debug_mode: 0,
                   task_statistics: 0,
                   rateProfile6PosSwitch: 0,
                   cpu_overclock: 0,
                   powerOnArmingGraceTime: 0,
                   boardIdentifier: [0; 6],};
#[no_mangle]
pub static mut systemConfig_Copy: systemConfig_t =
    systemConfig_t{pidProfileIndex: 0,
                   activeRateProfile: 0,
                   debug_mode: 0,
                   task_statistics: 0,
                   rateProfile6PosSwitch: 0,
                   cpu_overclock: 0,
                   powerOnArmingGraceTime: 0,
                   boardIdentifier: [0; 6],};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut systemConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (18 as libc::c_int |
                                      (2 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<systemConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &systemConfig_System as *const systemConfig_t
                                     as *mut systemConfig_t as *mut uint8_t,
                             copy:
                                 &systemConfig_Copy as *const systemConfig_t
                                     as *mut systemConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{ptr:
                                                     &pgResetTemplate_systemConfig
                                                         as
                                                         *const systemConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_systemConfig: systemConfig_t =
    {
        let mut init =
            systemConfig_s{pidProfileIndex: 0 as libc::c_int as uint8_t,
                           activeRateProfile: 0 as libc::c_int as uint8_t,
                           debug_mode: DEBUG_NONE as libc::c_int as uint8_t,
                           task_statistics: 1 as libc::c_int as uint8_t,
                           rateProfile6PosSwitch: 0,
                           cpu_overclock: 0 as libc::c_int as uint8_t,
                           powerOnArmingGraceTime:
                               5 as libc::c_int as uint8_t,
                           boardIdentifier: [83, 79, 70, 51, 0, 0],};
        init
    };
// USE_OSD_SLAVE
#[no_mangle]
pub unsafe extern "C" fn resetConfigs() {
    pgResetAll();
    targetConfiguration();
}
unsafe extern "C" fn activateConfig() {
    // USE_OSD_SLAVE
}
unsafe extern "C" fn validateAndFixConfig() {
    if !isSerialConfigValid(serialConfig()) {
        pgResetFn_serialConfig(serialConfigMutable());
    }
    featureClear(FEATURE_GPS as libc::c_int as uint32_t);
    // USE_OSD_SLAVE
    // clear features that are not supported.
// I have kept them all here in one place, some could be moved to sections of code above.
    featureClear(FEATURE_RX_PPM as libc::c_int as uint32_t);
    featureClear(FEATURE_RX_SERIAL as libc::c_int as uint32_t);
    featureClear(FEATURE_SOFTSERIAL as libc::c_int as uint32_t);
    featureClear(FEATURE_RANGEFINDER as libc::c_int as uint32_t);
    featureClear(FEATURE_TELEMETRY as libc::c_int as uint32_t);
    featureClear(FEATURE_RX_PARALLEL_PWM as libc::c_int as uint32_t);
    featureClear(FEATURE_RX_MSP as libc::c_int as uint32_t);
    featureClear(FEATURE_LED_STRIP as libc::c_int as uint32_t);
    featureClear(FEATURE_DASHBOARD as libc::c_int as uint32_t);
    featureClear(FEATURE_OSD as libc::c_int as uint32_t);
    featureClear((FEATURE_SERVO_TILT as libc::c_int |
                      FEATURE_CHANNEL_FORWARDING as libc::c_int) as uint32_t);
    featureClear(FEATURE_RX_SPI as libc::c_int as uint32_t);
    featureClear(FEATURE_SOFTSPI as libc::c_int as uint32_t);
    featureClear(FEATURE_ESC_SENSOR as libc::c_int as uint32_t);
    featureClear(FEATURE_DYNAMIC_FILTER as libc::c_int as uint32_t);
}
// USE_OSD_SLAVE
#[no_mangle]
pub unsafe extern "C" fn readEEPROM() -> bool {
    // Sanity check, read flash
    let mut success: bool = loadEEPROM();
    validateAndFixConfig();
    activateConfig();
    return success;
}
#[no_mangle]
pub unsafe extern "C" fn writeEEPROM() {
    validateAndFixConfig();
    writeConfigToEEPROM();
}
#[no_mangle]
pub unsafe extern "C" fn resetEEPROM() {
    resetConfigs();
    writeEEPROM();
    activateConfig();
}
#[no_mangle]
pub unsafe extern "C" fn ensureEEPROMStructureIsValid() {
    if isEEPROMStructureValid() { return }
    resetEEPROM();
}
#[no_mangle]
pub unsafe extern "C" fn saveConfigAndNotify() {
    writeEEPROM();
    readEEPROM();
    beeperConfirmationBeeps(1 as libc::c_int as uint8_t);
}
