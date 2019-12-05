use core;
use libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
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
    fn bitArrayGet(array: *const libc::c_void, bit: libc::c_uint) -> bool;
    #[no_mangle]
    fn bitArraySet(array: *mut libc::c_void, bit: libc::c_uint);
    #[no_mangle]
    fn bitArrayClr(array: *mut libc::c_void, bit: libc::c_uint);
    #[no_mangle]
    fn sbufWriteU8(dst: *mut sbuf_t, val: uint8_t);
    #[no_mangle]
    fn sbufWriteString(dst: *mut sbuf_t, string: *const libc::c_char);
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    static mut flightModeFlags: uint16_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    fn isMotorProtocolDshot() -> bool;
    #[no_mangle]
    static mut mixerConfig_System: mixerConfig_t;
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    static mut pinioBoxConfig_System: pinioBoxConfig_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive ( Copy, Clone )]
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
pub type C2RustUnnamed = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed = 1048576;
pub const FEATURE_OSD: C2RustUnnamed = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed = 8192;
pub const FEATURE_3D: C2RustUnnamed = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed = 512;
pub const FEATURE_GPS: C2RustUnnamed = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed = 1;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_0 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_0 = 2;
pub const ARMED: C2RustUnnamed_0 = 1;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const GPS_RESCUE_MODE: C2RustUnnamed_1 = 2048;
pub const FAILSAFE_MODE: C2RustUnnamed_1 = 1024;
pub const PASSTHRU_MODE: C2RustUnnamed_1 = 256;
pub const HEADFREE_MODE: C2RustUnnamed_1 = 64;
pub const GPS_HOLD_MODE: C2RustUnnamed_1 = 32;
pub const GPS_HOME_MODE: C2RustUnnamed_1 = 16;
pub const BARO_MODE: C2RustUnnamed_1 = 8;
pub const MAG_MODE: C2RustUnnamed_1 = 4;
pub const HORIZON_MODE: C2RustUnnamed_1 = 2;
pub const ANGLE_MODE: C2RustUnnamed_1 = 1;
pub type mixerMode = libc::c_uint;
pub const MIXER_QUADX_1234: mixerMode = 26;
pub const MIXER_CUSTOM_TRI: mixerMode = 25;
pub const MIXER_CUSTOM_AIRPLANE: mixerMode = 24;
pub const MIXER_CUSTOM: mixerMode = 23;
pub const MIXER_ATAIL4: mixerMode = 22;
pub const MIXER_SINGLECOPTER: mixerMode = 21;
pub const MIXER_DUALCOPTER: mixerMode = 20;
pub const MIXER_RX_TO_SERVO: mixerMode = 19;
pub const MIXER_HEX6H: mixerMode = 18;
pub const MIXER_VTAIL4: mixerMode = 17;
pub const MIXER_HELI_90_DEG: mixerMode = 16;
pub const MIXER_HELI_120_CCPM: mixerMode = 15;
pub const MIXER_AIRPLANE: mixerMode = 14;
pub const MIXER_OCTOFLATX: mixerMode = 13;
pub const MIXER_OCTOFLATP: mixerMode = 12;
pub const MIXER_OCTOX8: mixerMode = 11;
pub const MIXER_HEX6X: mixerMode = 10;
pub const MIXER_Y4: mixerMode = 9;
pub const MIXER_FLYING_WING: mixerMode = 8;
pub const MIXER_HEX6: mixerMode = 7;
pub const MIXER_Y6: mixerMode = 6;
pub const MIXER_GIMBAL: mixerMode = 5;
pub const MIXER_BICOPTER: mixerMode = 4;
pub const MIXER_QUADX: mixerMode = 3;
pub const MIXER_QUADP: mixerMode = 2;
pub const MIXER_TRI: mixerMode = 1;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct mixerConfig_s {
    pub mixerMode: uint8_t,
    pub yaw_motors_reversed: bool,
    pub crashflip_motor_percent: uint8_t,
}
pub type mixerConfig_t = mixerConfig_s;
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct boxBitmask_s {
    pub bits: [uint32_t; 2],
}
pub type boxBitmask_t = boxBitmask_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct box_s {
    pub boxId: uint8_t,
    pub boxName: *const libc::c_char,
    pub permanentId: uint8_t,
}
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
pub type box_t = box_s;
pub type assert_failed_FLIGHT_MODE_BOXID_MAP_INITIALIZER_does_not_match_boxId_e
    =
    [libc::c_char; 1];
pub type serializeBoxFn
    =
    unsafe extern "C" fn(_: *mut sbuf_s, _: *const box_t) -> ();
pub const SENSOR_ACC: C2RustUnnamed_2 = 2;
// see boxId_e
// GUI-readable box name
// permanent ID used to identify BOX. This ID is unique for one function, DO NOT REUSE IT
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
pub type pinioBoxConfig_t = pinioBoxConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pinioBoxConfig_s {
    pub permanentId: [uint8_t; 4],
}
pub const SENSOR_MAG: C2RustUnnamed_2 = 8;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_2 = 64;
pub const SENSOR_GPS: C2RustUnnamed_2 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_2 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_2 = 16;
pub const SENSOR_BARO: C2RustUnnamed_2 = 4;
pub const SENSOR_GYRO: C2RustUnnamed_2 = 1;
#[inline]
unsafe extern "C" fn mixerConfig() -> *const mixerConfig_t {
    return &mut mixerConfig_System;
}
#[inline]
unsafe extern "C" fn pinioBoxConfig() -> *const pinioBoxConfig_t {
    return &mut pinioBoxConfig_System;
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
// permanent IDs must uniquely identify BOX meaning, DO NOT REUSE THEM!
static mut boxes: [box_t; 41] =
    [{
         let mut init =
             box_s{boxId: BOXARM as libc::c_int as uint8_t,
                   boxName: b"ARM\x00" as *const u8 as *const libc::c_char,
                   permanentId: 0i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXANGLE as libc::c_int as uint8_t,
                   boxName: b"ANGLE\x00" as *const u8 as *const libc::c_char,
                   permanentId: 1i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXHORIZON as libc::c_int as uint8_t,
                   boxName:
                       b"HORIZON\x00" as *const u8 as *const libc::c_char,
                   permanentId: 2i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXBARO as libc::c_int as uint8_t,
                   boxName: b"BARO\x00" as *const u8 as *const libc::c_char,
                   permanentId: 3i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXANTIGRAVITY as libc::c_int as uint8_t,
                   boxName:
                       b"ANTI GRAVITY\x00" as *const u8 as
                           *const libc::c_char,
                   permanentId: 4i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXMAG as libc::c_int as uint8_t,
                   boxName: b"MAG\x00" as *const u8 as *const libc::c_char,
                   permanentId: 5i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXHEADFREE as libc::c_int as uint8_t,
                   boxName:
                       b"HEADFREE\x00" as *const u8 as *const libc::c_char,
                   permanentId: 6i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXHEADADJ as libc::c_int as uint8_t,
                   boxName:
                       b"HEADADJ\x00" as *const u8 as *const libc::c_char,
                   permanentId: 7i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXCAMSTAB as libc::c_int as uint8_t,
                   boxName:
                       b"CAMSTAB\x00" as *const u8 as *const libc::c_char,
                   permanentId: 8i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXGPSHOME as libc::c_int as uint8_t,
                   boxName:
                       b"GPS HOME\x00" as *const u8 as *const libc::c_char,
                   permanentId: 10i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXGPSHOLD as libc::c_int as uint8_t,
                   boxName:
                       b"GPS HOLD\x00" as *const u8 as *const libc::c_char,
                   permanentId: 11i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXPASSTHRU as libc::c_int as uint8_t,
                   boxName:
                       b"PASSTHRU\x00" as *const u8 as *const libc::c_char,
                   permanentId: 12i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXBEEPERON as libc::c_int as uint8_t,
                   boxName: b"BEEPER\x00" as *const u8 as *const libc::c_char,
                   permanentId: 13i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXLEDLOW as libc::c_int as uint8_t,
                   boxName: b"LEDLOW\x00" as *const u8 as *const libc::c_char,
                   permanentId: 15i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXCALIB as libc::c_int as uint8_t,
                   boxName: b"CALIB\x00" as *const u8 as *const libc::c_char,
                   permanentId: 17i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXOSD as libc::c_int as uint8_t,
                   boxName:
                       b"OSD DISABLE SW\x00" as *const u8 as
                           *const libc::c_char,
                   permanentId: 19i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXTELEMETRY as libc::c_int as uint8_t,
                   boxName:
                       b"TELEMETRY\x00" as *const u8 as *const libc::c_char,
                   permanentId: 20i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXSERVO1 as libc::c_int as uint8_t,
                   boxName: b"SERVO1\x00" as *const u8 as *const libc::c_char,
                   permanentId: 23i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXSERVO2 as libc::c_int as uint8_t,
                   boxName: b"SERVO2\x00" as *const u8 as *const libc::c_char,
                   permanentId: 24i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXSERVO3 as libc::c_int as uint8_t,
                   boxName: b"SERVO3\x00" as *const u8 as *const libc::c_char,
                   permanentId: 25i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXBLACKBOX as libc::c_int as uint8_t,
                   boxName:
                       b"BLACKBOX\x00" as *const u8 as *const libc::c_char,
                   permanentId: 26i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXFAILSAFE as libc::c_int as uint8_t,
                   boxName:
                       b"FAILSAFE\x00" as *const u8 as *const libc::c_char,
                   permanentId: 27i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXAIRMODE as libc::c_int as uint8_t,
                   boxName:
                       b"AIR MODE\x00" as *const u8 as *const libc::c_char,
                   permanentId: 28i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOX3D as libc::c_int as uint8_t,
                   boxName:
                       b"DISABLE / SWITCH 3D\x00" as *const u8 as
                           *const libc::c_char,
                   permanentId: 29i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXFPVANGLEMIX as libc::c_int as uint8_t,
                   boxName:
                       b"FPV ANGLE MIX\x00" as *const u8 as
                           *const libc::c_char,
                   permanentId: 30i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXBLACKBOXERASE as libc::c_int as uint8_t,
                   boxName:
                       b"BLACKBOX ERASE (>30s)\x00" as *const u8 as
                           *const libc::c_char,
                   permanentId: 31i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXCAMERA1 as libc::c_int as uint8_t,
                   boxName:
                       b"CAMERA CONTROL 1\x00" as *const u8 as
                           *const libc::c_char,
                   permanentId: 32i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXCAMERA2 as libc::c_int as uint8_t,
                   boxName:
                       b"CAMERA CONTROL 2\x00" as *const u8 as
                           *const libc::c_char,
                   permanentId: 33i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXCAMERA3 as libc::c_int as uint8_t,
                   boxName:
                       b"CAMERA CONTROL 3\x00" as *const u8 as
                           *const libc::c_char,
                   permanentId: 34i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXFLIPOVERAFTERCRASH as libc::c_int as uint8_t,
                   boxName:
                       b"FLIP OVER AFTER CRASH\x00" as *const u8 as
                           *const libc::c_char,
                   permanentId: 35i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXPREARM as libc::c_int as uint8_t,
                   boxName: b"PREARM\x00" as *const u8 as *const libc::c_char,
                   permanentId: 36i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXBEEPGPSCOUNT as libc::c_int as uint8_t,
                   boxName:
                       b"BEEP GPS SATELLITE COUNT\x00" as *const u8 as
                           *const libc::c_char,
                   permanentId: 37i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXVTXPITMODE as libc::c_int as uint8_t,
                   boxName:
                       b"VTX PIT MODE\x00" as *const u8 as
                           *const libc::c_char,
                   permanentId: 39i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXUSER1 as libc::c_int as uint8_t,
                   boxName: b"USER1\x00" as *const u8 as *const libc::c_char,
                   permanentId: 40i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXUSER2 as libc::c_int as uint8_t,
                   boxName: b"USER2\x00" as *const u8 as *const libc::c_char,
                   permanentId: 41i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXUSER3 as libc::c_int as uint8_t,
                   boxName: b"USER3\x00" as *const u8 as *const libc::c_char,
                   permanentId: 42i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXUSER4 as libc::c_int as uint8_t,
                   boxName: b"USER4\x00" as *const u8 as *const libc::c_char,
                   permanentId: 43i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXPIDAUDIO as libc::c_int as uint8_t,
                   boxName:
                       b"PID AUDIO\x00" as *const u8 as *const libc::c_char,
                   permanentId: 44i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXPARALYZE as libc::c_int as uint8_t,
                   boxName:
                       b"PARALYZE\x00" as *const u8 as *const libc::c_char,
                   permanentId: 45i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXGPSRESCUE as libc::c_int as uint8_t,
                   boxName:
                       b"GPS RESCUE\x00" as *const u8 as *const libc::c_char,
                   permanentId: 46i32 as uint8_t,};
         init
     },
     {
         let mut init =
             box_s{boxId: BOXACROTRAINER as libc::c_int as uint8_t,
                   boxName:
                       b"ACRO TRAINER\x00" as *const u8 as
                           *const libc::c_char,
                   permanentId: 47i32 as uint8_t,};
         init
     }];
// mask of enabled IDs, calculated on startup based on enabled features. boxId_e is used as bit index
static mut activeBoxIds: boxBitmask_t = boxBitmask_t{bits: [0; 2],};
#[no_mangle]
pub unsafe extern "C" fn findBoxByBoxId(mut boxId: boxId_e) -> *const box_t {
    let mut i: libc::c_uint = 0i32 as libc::c_uint;
    while (i as libc::c_ulong) <
              (::core::mem::size_of::<[box_t; 41]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<box_t>()
                                                   as libc::c_ulong) {
        let mut candidate: *const box_t =
            &*boxes.as_ptr().offset(i as isize) as *const box_t;
        if (*candidate).boxId as libc::c_uint == boxId as libc::c_uint {
            return candidate
        }
        i = i.wrapping_add(1)
    }
    return 0 as *const box_t;
}
#[no_mangle]
pub unsafe extern "C" fn findBoxByPermanentId(mut permanentId: uint8_t)
 -> *const box_t {
    let mut i: libc::c_uint = 0i32 as libc::c_uint;
    while (i as libc::c_ulong) <
              (::core::mem::size_of::<[box_t; 41]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<box_t>()
                                                   as libc::c_ulong) {
        let mut candidate: *const box_t =
            &*boxes.as_ptr().offset(i as isize) as *const box_t;
        if (*candidate).permanentId as libc::c_int ==
               permanentId as libc::c_int {
            return candidate
        }
        i = i.wrapping_add(1)
    }
    return 0 as *const box_t;
}
unsafe extern "C" fn activeBoxIdGet(mut boxId: boxId_e) -> bool {
    if boxId as libc::c_ulong >
           (::core::mem::size_of::<boxBitmask_t>() as
                libc::c_ulong).wrapping_mul(8i32 as libc::c_ulong) {
        return 0i32 != 0
    }
    return bitArrayGet(&mut activeBoxIds as *mut boxBitmask_t as
                           *const libc::c_void, boxId as libc::c_uint);
}
#[no_mangle]
pub unsafe extern "C" fn serializeBoxNameFn(mut dst: *mut sbuf_t,
                                            mut box_0: *const box_t) {
    sbufWriteString(dst, (*box_0).boxName);
    sbufWriteU8(dst, ';' as i32 as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn serializeBoxPermanentIdFn(mut dst: *mut sbuf_t,
                                                   mut box_0: *const box_t) {
    sbufWriteU8(dst, (*box_0).permanentId);
}
// serialize 'page' of boxNames.
// Each page contains at most 32 boxes
#[no_mangle]
pub unsafe extern "C" fn serializeBoxReply(mut dst: *mut sbuf_t,
                                           mut page: libc::c_int,
                                           mut serializeBox:
                                               Option<unsafe extern "C" fn(_:
                                                                               *mut sbuf_s,
                                                                           _:
                                                                               *const box_t)
                                                          -> ()>) {
    let mut boxIdx: libc::c_uint = 0i32 as libc::c_uint;
    let mut pageStart: libc::c_uint = (page * 32i32) as libc::c_uint;
    let mut pageEnd: libc::c_uint =
        pageStart.wrapping_add(32i32 as libc::c_uint);
    let mut id: boxId_e = BOXARM;
    while (id as libc::c_uint) <
              CHECKBOX_ITEM_COUNT as libc::c_int as libc::c_uint {
        if activeBoxIdGet(id) {
            if boxIdx >= pageStart && boxIdx < pageEnd {
                Some(serializeBox.expect("non-null function pointer")).expect("non-null function pointer")(dst,
                                                                                                           findBoxByBoxId(id));
            }
            boxIdx = boxIdx.wrapping_add(1)
            // count active boxes
        }
        id += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn initActiveBoxIds() {
    // calculate used boxes based on features and set corresponding activeBoxIds bits
    let mut ena: boxBitmask_t =
        boxBitmask_t{bits: [0; 2],}; // temporary variable to collect result
    memset(&mut ena as *mut boxBitmask_t as *mut libc::c_void, 0i32,
           ::core::mem::size_of::<boxBitmask_t>() as libc::c_ulong);
    // macro to enable boxId (BoxidMaskEnable). Reference to ena is hidden, local use only
    bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                BOXARM as libc::c_int as libc::c_uint);
    bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                BOXPREARM as libc::c_int as libc::c_uint);
    if !feature(FEATURE_AIRMODE as libc::c_int as uint32_t) {
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXAIRMODE as libc::c_int as libc::c_uint);
    }
    if !feature(FEATURE_ANTI_GRAVITY as libc::c_int as uint32_t) {
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXANTIGRAVITY as libc::c_int as libc::c_uint);
    }
    if sensors(SENSOR_ACC as libc::c_int as uint32_t) {
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXANGLE as libc::c_int as libc::c_uint);
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXHORIZON as libc::c_int as libc::c_uint);
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXHEADFREE as libc::c_int as libc::c_uint);
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXHEADADJ as libc::c_int as libc::c_uint);
    }
    if sensors(SENSOR_MAG as libc::c_int as uint32_t) {
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXMAG as libc::c_int as libc::c_uint);
    }
    bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                BOXFAILSAFE as libc::c_int as libc::c_uint);
    if (*mixerConfig()).mixerMode as libc::c_int ==
           MIXER_FLYING_WING as libc::c_int ||
           (*mixerConfig()).mixerMode as libc::c_int ==
               MIXER_AIRPLANE as libc::c_int ||
           (*mixerConfig()).mixerMode as libc::c_int ==
               MIXER_CUSTOM_AIRPLANE as libc::c_int {
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXPASSTHRU as libc::c_int as libc::c_uint);
    }
    bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                BOXBEEPERON as libc::c_int as libc::c_uint);
    if feature(FEATURE_LED_STRIP as libc::c_int as uint32_t) {
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXLEDLOW as libc::c_int as libc::c_uint);
    }
    bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                BOXBLACKBOX as libc::c_int as libc::c_uint);
    bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                BOXBLACKBOXERASE as libc::c_int as libc::c_uint);
    bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                BOXFPVANGLEMIX as libc::c_int as libc::c_uint);
    if feature(FEATURE_3D as libc::c_int as uint32_t) {
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOX3D as libc::c_int as libc::c_uint);
    }
    if isMotorProtocolDshot() {
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXFLIPOVERAFTERCRASH as libc::c_int as libc::c_uint);
    }
    if feature(FEATURE_SERVO_TILT as libc::c_int as uint32_t) {
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXCAMSTAB as libc::c_int as libc::c_uint);
    }
    if feature(FEATURE_INFLIGHT_ACC_CAL as libc::c_int as uint32_t) {
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXCALIB as libc::c_int as libc::c_uint);
    }
    bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                BOXOSD as libc::c_int as libc::c_uint);
    if feature(FEATURE_TELEMETRY as libc::c_int as uint32_t) {
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXTELEMETRY as libc::c_int as libc::c_uint);
    }
    if (*mixerConfig()).mixerMode as libc::c_int ==
           MIXER_CUSTOM_AIRPLANE as libc::c_int {
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXSERVO1 as libc::c_int as libc::c_uint);
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXSERVO2 as libc::c_int as libc::c_uint);
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXSERVO3 as libc::c_int as libc::c_uint);
    }
    bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                BOXCAMERA1 as libc::c_int as libc::c_uint);
    bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                BOXCAMERA2 as libc::c_int as libc::c_uint);
    bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                BOXCAMERA3 as libc::c_int as libc::c_uint);
    bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                BOXVTXPITMODE as libc::c_int as libc::c_uint);
    bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                BOXPARALYZE as libc::c_int as libc::c_uint);
    // Turn BOXUSERx only if pinioBox facility monitors them, as the facility is the only BOXUSERx observer.
    // Note that pinioBoxConfig can be set to monitor any box.
    let mut i: libc::c_int = 0i32;
    while i < 4i32 {
        if (*pinioBoxConfig()).permanentId[i as usize] as libc::c_int !=
               255i32 {
            let mut box_0: *const box_t =
                findBoxByPermanentId((*pinioBoxConfig()).permanentId[i as
                                                                         usize]);
            if !box_0.is_null() {
                match (*box_0).boxId as libc::c_int {
                    35 | 36 | 37 | 38 => {
                        bitArraySet(&mut ena as *mut boxBitmask_t as
                                        *mut libc::c_void,
                                    (*box_0).boxId as libc::c_uint);
                    }
                    _ => { }
                }
            }
        }
        i += 1
    }
    if sensors(SENSOR_ACC as libc::c_int as uint32_t) {
        bitArraySet(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                    BOXACROTRAINER as libc::c_int as libc::c_uint);
    }
    // USE_ACRO_TRAINER
    // check that all enabled IDs are in boxes array (check may be skipped when using findBoxById() functions)
    let mut boxId: boxId_e =
        BOXARM; // this should not happen, but handle it gracefully
    while (boxId as libc::c_uint) <
              CHECKBOX_ITEM_COUNT as libc::c_int as libc::c_uint {
        if bitArrayGet(&mut ena as *mut boxBitmask_t as *const libc::c_void,
                       boxId as libc::c_uint) as libc::c_int != 0 &&
               findBoxByBoxId(boxId).is_null() {
            bitArrayClr(&mut ena as *mut boxBitmask_t as *mut libc::c_void,
                        boxId as libc::c_uint);
        }
        boxId += 1
    }
    activeBoxIds = ena;
    // set global variable
}
// return state of given boxId box, handling ARM and FLIGHT_MODE
#[no_mangle]
pub unsafe extern "C" fn getBoxIdState(mut boxid: boxId_e) -> bool {
    let boxIdToFlightModeMap: [uint8_t; 11] =
        [0,
         ((32i32 *
               (ANGLE_MODE as libc::c_int as libc::c_long / 2i64 >> 31i32 >
                    0i32 as libc::c_long) as libc::c_int) as libc::c_long +
              ((16i32 *
                    (ANGLE_MODE as libc::c_int as libc::c_long * 1i64 >>
                         16i32 *
                             (ANGLE_MODE as libc::c_int as libc::c_long / 2i64
                                  >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int >>
                         16i32 *
                             (ANGLE_MODE as libc::c_int as libc::c_long / 2i64
                                  >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int > 65535i64) as libc::c_int) as
                   libc::c_long +
                   ((8i32 *
                         ((ANGLE_MODE as libc::c_int as libc::c_long * 1i64 >>
                               16i32 *
                                   (ANGLE_MODE as libc::c_int as libc::c_long
                                        / 2i64 >> 31i32 >
                                        0i32 as libc::c_long) as libc::c_int
                               >>
                               16i32 *
                                   (ANGLE_MODE as libc::c_int as libc::c_long
                                        / 2i64 >> 31i32 >
                                        0i32 as libc::c_long) as libc::c_int)
                              * 1i64 >>
                              16i32 *
                                  (ANGLE_MODE as libc::c_int as libc::c_long *
                                       1i64 >>
                                       16i32 *
                                           (ANGLE_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int >>
                                       16i32 *
                                           (ANGLE_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int > 65535i64) as
                                      libc::c_int > 255i32 as libc::c_long) as
                             libc::c_int) as libc::c_long +
                        (8i32 as libc::c_long -
                             90i32 as libc::c_long /
                                 (((ANGLE_MODE as libc::c_int as libc::c_long
                                        * 1i64 >>
                                        16i32 *
                                            (ANGLE_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (ANGLE_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (ANGLE_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (ANGLE_MODE as libc::c_int
                                                         as libc::c_long /
                                                         2i64 >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (ANGLE_MODE as libc::c_int
                                                         as libc::c_long /
                                                         2i64 >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((ANGLE_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (ANGLE_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (ANGLE_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (ANGLE_MODE as libc::c_int
                                                         as libc::c_long *
                                                         1i64 >>
                                                         16i32 *
                                                             (ANGLE_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (ANGLE_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      4i32 as libc::c_long +
                                      14i32 as libc::c_long |
                                      1i32 as libc::c_long) -
                             2i32 as libc::c_long /
                                 (((ANGLE_MODE as libc::c_int as libc::c_long
                                        * 1i64 >>
                                        16i32 *
                                            (ANGLE_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (ANGLE_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (ANGLE_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (ANGLE_MODE as libc::c_int
                                                         as libc::c_long /
                                                         2i64 >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (ANGLE_MODE as libc::c_int
                                                         as libc::c_long /
                                                         2i64 >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((ANGLE_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (ANGLE_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (ANGLE_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (ANGLE_MODE as libc::c_int
                                                         as libc::c_long *
                                                         1i64 >>
                                                         16i32 *
                                                             (ANGLE_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (ANGLE_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      2i32 as libc::c_long +
                                      1i32 as libc::c_long))))) as uint8_t,
         ((32i32 *
               (HORIZON_MODE as libc::c_int as libc::c_long / 2i64 >> 31i32 >
                    0i32 as libc::c_long) as libc::c_int) as libc::c_long +
              ((16i32 *
                    (HORIZON_MODE as libc::c_int as libc::c_long * 1i64 >>
                         16i32 *
                             (HORIZON_MODE as libc::c_int as libc::c_long /
                                  2i64 >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int >>
                         16i32 *
                             (HORIZON_MODE as libc::c_int as libc::c_long /
                                  2i64 >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int > 65535i64) as libc::c_int) as
                   libc::c_long +
                   ((8i32 *
                         ((HORIZON_MODE as libc::c_int as libc::c_long * 1i64
                               >>
                               16i32 *
                                   (HORIZON_MODE as libc::c_int as
                                        libc::c_long / 2i64 >> 31i32 >
                                        0i32 as libc::c_long) as libc::c_int
                               >>
                               16i32 *
                                   (HORIZON_MODE as libc::c_int as
                                        libc::c_long / 2i64 >> 31i32 >
                                        0i32 as libc::c_long) as libc::c_int)
                              * 1i64 >>
                              16i32 *
                                  (HORIZON_MODE as libc::c_int as libc::c_long
                                       * 1i64 >>
                                       16i32 *
                                           (HORIZON_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int >>
                                       16i32 *
                                           (HORIZON_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int > 65535i64) as
                                      libc::c_int > 255i32 as libc::c_long) as
                             libc::c_int) as libc::c_long +
                        (8i32 as libc::c_long -
                             90i32 as libc::c_long /
                                 (((HORIZON_MODE as libc::c_int as
                                        libc::c_long * 1i64 >>
                                        16i32 *
                                            (HORIZON_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (HORIZON_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (HORIZON_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (HORIZON_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (HORIZON_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((HORIZON_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (HORIZON_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (HORIZON_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (HORIZON_MODE as
                                                         libc::c_int as
                                                         libc::c_long * 1i64
                                                         >>
                                                         16i32 *
                                                             (HORIZON_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (HORIZON_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      4i32 as libc::c_long +
                                      14i32 as libc::c_long |
                                      1i32 as libc::c_long) -
                             2i32 as libc::c_long /
                                 (((HORIZON_MODE as libc::c_int as
                                        libc::c_long * 1i64 >>
                                        16i32 *
                                            (HORIZON_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (HORIZON_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (HORIZON_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (HORIZON_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (HORIZON_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((HORIZON_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (HORIZON_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (HORIZON_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (HORIZON_MODE as
                                                         libc::c_int as
                                                         libc::c_long * 1i64
                                                         >>
                                                         16i32 *
                                                             (HORIZON_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (HORIZON_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      2i32 as libc::c_long +
                                      1i32 as libc::c_long))))) as uint8_t,
         ((32i32 *
               (MAG_MODE as libc::c_int as libc::c_long / 2i64 >> 31i32 >
                    0i32 as libc::c_long) as libc::c_int) as libc::c_long +
              ((16i32 *
                    (MAG_MODE as libc::c_int as libc::c_long * 1i64 >>
                         16i32 *
                             (MAG_MODE as libc::c_int as libc::c_long / 2i64
                                  >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int >>
                         16i32 *
                             (MAG_MODE as libc::c_int as libc::c_long / 2i64
                                  >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int > 65535i64) as libc::c_int) as
                   libc::c_long +
                   ((8i32 *
                         ((MAG_MODE as libc::c_int as libc::c_long * 1i64 >>
                               16i32 *
                                   (MAG_MODE as libc::c_int as libc::c_long /
                                        2i64 >> 31i32 > 0i32 as libc::c_long)
                                       as libc::c_int >>
                               16i32 *
                                   (MAG_MODE as libc::c_int as libc::c_long /
                                        2i64 >> 31i32 > 0i32 as libc::c_long)
                                       as libc::c_int) * 1i64 >>
                              16i32 *
                                  (MAG_MODE as libc::c_int as libc::c_long *
                                       1i64 >>
                                       16i32 *
                                           (MAG_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int >>
                                       16i32 *
                                           (MAG_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int > 65535i64) as
                                      libc::c_int > 255i32 as libc::c_long) as
                             libc::c_int) as libc::c_long +
                        (8i32 as libc::c_long -
                             90i32 as libc::c_long /
                                 (((MAG_MODE as libc::c_int as libc::c_long *
                                        1i64 >>
                                        16i32 *
                                            (MAG_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (MAG_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (MAG_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (MAG_MODE as libc::c_int
                                                         as libc::c_long /
                                                         2i64 >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (MAG_MODE as libc::c_int
                                                         as libc::c_long /
                                                         2i64 >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((MAG_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (MAG_MODE as libc::c_int
                                                          as libc::c_long /
                                                          2i64 >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (MAG_MODE as libc::c_int
                                                          as libc::c_long /
                                                          2i64 >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (MAG_MODE as libc::c_int
                                                         as libc::c_long *
                                                         1i64 >>
                                                         16i32 *
                                                             (MAG_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (MAG_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      4i32 as libc::c_long +
                                      14i32 as libc::c_long |
                                      1i32 as libc::c_long) -
                             2i32 as libc::c_long /
                                 (((MAG_MODE as libc::c_int as libc::c_long *
                                        1i64 >>
                                        16i32 *
                                            (MAG_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (MAG_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (MAG_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (MAG_MODE as libc::c_int
                                                         as libc::c_long /
                                                         2i64 >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (MAG_MODE as libc::c_int
                                                         as libc::c_long /
                                                         2i64 >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((MAG_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (MAG_MODE as libc::c_int
                                                          as libc::c_long /
                                                          2i64 >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (MAG_MODE as libc::c_int
                                                          as libc::c_long /
                                                          2i64 >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (MAG_MODE as libc::c_int
                                                         as libc::c_long *
                                                         1i64 >>
                                                         16i32 *
                                                             (MAG_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (MAG_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      2i32 as libc::c_long +
                                      1i32 as libc::c_long))))) as uint8_t,
         ((32i32 *
               (BARO_MODE as libc::c_int as libc::c_long / 2i64 >> 31i32 >
                    0i32 as libc::c_long) as libc::c_int) as libc::c_long +
              ((16i32 *
                    (BARO_MODE as libc::c_int as libc::c_long * 1i64 >>
                         16i32 *
                             (BARO_MODE as libc::c_int as libc::c_long / 2i64
                                  >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int >>
                         16i32 *
                             (BARO_MODE as libc::c_int as libc::c_long / 2i64
                                  >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int > 65535i64) as libc::c_int) as
                   libc::c_long +
                   ((8i32 *
                         ((BARO_MODE as libc::c_int as libc::c_long * 1i64 >>
                               16i32 *
                                   (BARO_MODE as libc::c_int as libc::c_long /
                                        2i64 >> 31i32 > 0i32 as libc::c_long)
                                       as libc::c_int >>
                               16i32 *
                                   (BARO_MODE as libc::c_int as libc::c_long /
                                        2i64 >> 31i32 > 0i32 as libc::c_long)
                                       as libc::c_int) * 1i64 >>
                              16i32 *
                                  (BARO_MODE as libc::c_int as libc::c_long *
                                       1i64 >>
                                       16i32 *
                                           (BARO_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int >>
                                       16i32 *
                                           (BARO_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int > 65535i64) as
                                      libc::c_int > 255i32 as libc::c_long) as
                             libc::c_int) as libc::c_long +
                        (8i32 as libc::c_long -
                             90i32 as libc::c_long /
                                 (((BARO_MODE as libc::c_int as libc::c_long *
                                        1i64 >>
                                        16i32 *
                                            (BARO_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (BARO_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (BARO_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (BARO_MODE as libc::c_int
                                                         as libc::c_long /
                                                         2i64 >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (BARO_MODE as libc::c_int
                                                         as libc::c_long /
                                                         2i64 >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((BARO_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (BARO_MODE as libc::c_int
                                                          as libc::c_long /
                                                          2i64 >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (BARO_MODE as libc::c_int
                                                          as libc::c_long /
                                                          2i64 >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (BARO_MODE as libc::c_int
                                                         as libc::c_long *
                                                         1i64 >>
                                                         16i32 *
                                                             (BARO_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (BARO_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      4i32 as libc::c_long +
                                      14i32 as libc::c_long |
                                      1i32 as libc::c_long) -
                             2i32 as libc::c_long /
                                 (((BARO_MODE as libc::c_int as libc::c_long *
                                        1i64 >>
                                        16i32 *
                                            (BARO_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (BARO_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (BARO_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (BARO_MODE as libc::c_int
                                                         as libc::c_long /
                                                         2i64 >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (BARO_MODE as libc::c_int
                                                         as libc::c_long /
                                                         2i64 >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((BARO_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (BARO_MODE as libc::c_int
                                                          as libc::c_long /
                                                          2i64 >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (BARO_MODE as libc::c_int
                                                          as libc::c_long /
                                                          2i64 >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (BARO_MODE as libc::c_int
                                                         as libc::c_long *
                                                         1i64 >>
                                                         16i32 *
                                                             (BARO_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (BARO_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      2i32 as libc::c_long +
                                      1i32 as libc::c_long))))) as uint8_t,
         ((32i32 *
               (GPS_HOME_MODE as libc::c_int as libc::c_long / 2i64 >> 31i32 >
                    0i32 as libc::c_long) as libc::c_int) as libc::c_long +
              ((16i32 *
                    (GPS_HOME_MODE as libc::c_int as libc::c_long * 1i64 >>
                         16i32 *
                             (GPS_HOME_MODE as libc::c_int as libc::c_long /
                                  2i64 >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int >>
                         16i32 *
                             (GPS_HOME_MODE as libc::c_int as libc::c_long /
                                  2i64 >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int > 65535i64) as libc::c_int) as
                   libc::c_long +
                   ((8i32 *
                         ((GPS_HOME_MODE as libc::c_int as libc::c_long * 1i64
                               >>
                               16i32 *
                                   (GPS_HOME_MODE as libc::c_int as
                                        libc::c_long / 2i64 >> 31i32 >
                                        0i32 as libc::c_long) as libc::c_int
                               >>
                               16i32 *
                                   (GPS_HOME_MODE as libc::c_int as
                                        libc::c_long / 2i64 >> 31i32 >
                                        0i32 as libc::c_long) as libc::c_int)
                              * 1i64 >>
                              16i32 *
                                  (GPS_HOME_MODE as libc::c_int as
                                       libc::c_long * 1i64 >>
                                       16i32 *
                                           (GPS_HOME_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int >>
                                       16i32 *
                                           (GPS_HOME_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int > 65535i64) as
                                      libc::c_int > 255i32 as libc::c_long) as
                             libc::c_int) as libc::c_long +
                        (8i32 as libc::c_long -
                             90i32 as libc::c_long /
                                 (((GPS_HOME_MODE as libc::c_int as
                                        libc::c_long * 1i64 >>
                                        16i32 *
                                            (GPS_HOME_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (GPS_HOME_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (GPS_HOME_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (GPS_HOME_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (GPS_HOME_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((GPS_HOME_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (GPS_HOME_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (GPS_HOME_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (GPS_HOME_MODE as
                                                         libc::c_int as
                                                         libc::c_long * 1i64
                                                         >>
                                                         16i32 *
                                                             (GPS_HOME_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (GPS_HOME_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      4i32 as libc::c_long +
                                      14i32 as libc::c_long |
                                      1i32 as libc::c_long) -
                             2i32 as libc::c_long /
                                 (((GPS_HOME_MODE as libc::c_int as
                                        libc::c_long * 1i64 >>
                                        16i32 *
                                            (GPS_HOME_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (GPS_HOME_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (GPS_HOME_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (GPS_HOME_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (GPS_HOME_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((GPS_HOME_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (GPS_HOME_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (GPS_HOME_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (GPS_HOME_MODE as
                                                         libc::c_int as
                                                         libc::c_long * 1i64
                                                         >>
                                                         16i32 *
                                                             (GPS_HOME_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (GPS_HOME_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      2i32 as libc::c_long +
                                      1i32 as libc::c_long))))) as uint8_t,
         ((32i32 *
               (GPS_HOLD_MODE as libc::c_int as libc::c_long / 2i64 >> 31i32 >
                    0i32 as libc::c_long) as libc::c_int) as libc::c_long +
              ((16i32 *
                    (GPS_HOLD_MODE as libc::c_int as libc::c_long * 1i64 >>
                         16i32 *
                             (GPS_HOLD_MODE as libc::c_int as libc::c_long /
                                  2i64 >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int >>
                         16i32 *
                             (GPS_HOLD_MODE as libc::c_int as libc::c_long /
                                  2i64 >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int > 65535i64) as libc::c_int) as
                   libc::c_long +
                   ((8i32 *
                         ((GPS_HOLD_MODE as libc::c_int as libc::c_long * 1i64
                               >>
                               16i32 *
                                   (GPS_HOLD_MODE as libc::c_int as
                                        libc::c_long / 2i64 >> 31i32 >
                                        0i32 as libc::c_long) as libc::c_int
                               >>
                               16i32 *
                                   (GPS_HOLD_MODE as libc::c_int as
                                        libc::c_long / 2i64 >> 31i32 >
                                        0i32 as libc::c_long) as libc::c_int)
                              * 1i64 >>
                              16i32 *
                                  (GPS_HOLD_MODE as libc::c_int as
                                       libc::c_long * 1i64 >>
                                       16i32 *
                                           (GPS_HOLD_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int >>
                                       16i32 *
                                           (GPS_HOLD_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int > 65535i64) as
                                      libc::c_int > 255i32 as libc::c_long) as
                             libc::c_int) as libc::c_long +
                        (8i32 as libc::c_long -
                             90i32 as libc::c_long /
                                 (((GPS_HOLD_MODE as libc::c_int as
                                        libc::c_long * 1i64 >>
                                        16i32 *
                                            (GPS_HOLD_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (GPS_HOLD_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (GPS_HOLD_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (GPS_HOLD_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (GPS_HOLD_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((GPS_HOLD_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (GPS_HOLD_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (GPS_HOLD_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (GPS_HOLD_MODE as
                                                         libc::c_int as
                                                         libc::c_long * 1i64
                                                         >>
                                                         16i32 *
                                                             (GPS_HOLD_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (GPS_HOLD_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      4i32 as libc::c_long +
                                      14i32 as libc::c_long |
                                      1i32 as libc::c_long) -
                             2i32 as libc::c_long /
                                 (((GPS_HOLD_MODE as libc::c_int as
                                        libc::c_long * 1i64 >>
                                        16i32 *
                                            (GPS_HOLD_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (GPS_HOLD_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (GPS_HOLD_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (GPS_HOLD_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (GPS_HOLD_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((GPS_HOLD_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (GPS_HOLD_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (GPS_HOLD_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (GPS_HOLD_MODE as
                                                         libc::c_int as
                                                         libc::c_long * 1i64
                                                         >>
                                                         16i32 *
                                                             (GPS_HOLD_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (GPS_HOLD_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      2i32 as libc::c_long +
                                      1i32 as libc::c_long))))) as uint8_t,
         ((32i32 *
               (HEADFREE_MODE as libc::c_int as libc::c_long / 2i64 >> 31i32 >
                    0i32 as libc::c_long) as libc::c_int) as libc::c_long +
              ((16i32 *
                    (HEADFREE_MODE as libc::c_int as libc::c_long * 1i64 >>
                         16i32 *
                             (HEADFREE_MODE as libc::c_int as libc::c_long /
                                  2i64 >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int >>
                         16i32 *
                             (HEADFREE_MODE as libc::c_int as libc::c_long /
                                  2i64 >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int > 65535i64) as libc::c_int) as
                   libc::c_long +
                   ((8i32 *
                         ((HEADFREE_MODE as libc::c_int as libc::c_long * 1i64
                               >>
                               16i32 *
                                   (HEADFREE_MODE as libc::c_int as
                                        libc::c_long / 2i64 >> 31i32 >
                                        0i32 as libc::c_long) as libc::c_int
                               >>
                               16i32 *
                                   (HEADFREE_MODE as libc::c_int as
                                        libc::c_long / 2i64 >> 31i32 >
                                        0i32 as libc::c_long) as libc::c_int)
                              * 1i64 >>
                              16i32 *
                                  (HEADFREE_MODE as libc::c_int as
                                       libc::c_long * 1i64 >>
                                       16i32 *
                                           (HEADFREE_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int >>
                                       16i32 *
                                           (HEADFREE_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int > 65535i64) as
                                      libc::c_int > 255i32 as libc::c_long) as
                             libc::c_int) as libc::c_long +
                        (8i32 as libc::c_long -
                             90i32 as libc::c_long /
                                 (((HEADFREE_MODE as libc::c_int as
                                        libc::c_long * 1i64 >>
                                        16i32 *
                                            (HEADFREE_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (HEADFREE_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (HEADFREE_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (HEADFREE_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (HEADFREE_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((HEADFREE_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (HEADFREE_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (HEADFREE_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (HEADFREE_MODE as
                                                         libc::c_int as
                                                         libc::c_long * 1i64
                                                         >>
                                                         16i32 *
                                                             (HEADFREE_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (HEADFREE_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      4i32 as libc::c_long +
                                      14i32 as libc::c_long |
                                      1i32 as libc::c_long) -
                             2i32 as libc::c_long /
                                 (((HEADFREE_MODE as libc::c_int as
                                        libc::c_long * 1i64 >>
                                        16i32 *
                                            (HEADFREE_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (HEADFREE_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (HEADFREE_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (HEADFREE_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (HEADFREE_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((HEADFREE_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (HEADFREE_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (HEADFREE_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (HEADFREE_MODE as
                                                         libc::c_int as
                                                         libc::c_long * 1i64
                                                         >>
                                                         16i32 *
                                                             (HEADFREE_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (HEADFREE_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      2i32 as libc::c_long +
                                      1i32 as libc::c_long))))) as uint8_t,
         ((32i32 *
               (PASSTHRU_MODE as libc::c_int as libc::c_long / 2i64 >> 31i32 >
                    0i32 as libc::c_long) as libc::c_int) as libc::c_long +
              ((16i32 *
                    (PASSTHRU_MODE as libc::c_int as libc::c_long * 1i64 >>
                         16i32 *
                             (PASSTHRU_MODE as libc::c_int as libc::c_long /
                                  2i64 >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int >>
                         16i32 *
                             (PASSTHRU_MODE as libc::c_int as libc::c_long /
                                  2i64 >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int > 65535i64) as libc::c_int) as
                   libc::c_long +
                   ((8i32 *
                         ((PASSTHRU_MODE as libc::c_int as libc::c_long * 1i64
                               >>
                               16i32 *
                                   (PASSTHRU_MODE as libc::c_int as
                                        libc::c_long / 2i64 >> 31i32 >
                                        0i32 as libc::c_long) as libc::c_int
                               >>
                               16i32 *
                                   (PASSTHRU_MODE as libc::c_int as
                                        libc::c_long / 2i64 >> 31i32 >
                                        0i32 as libc::c_long) as libc::c_int)
                              * 1i64 >>
                              16i32 *
                                  (PASSTHRU_MODE as libc::c_int as
                                       libc::c_long * 1i64 >>
                                       16i32 *
                                           (PASSTHRU_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int >>
                                       16i32 *
                                           (PASSTHRU_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int > 65535i64) as
                                      libc::c_int > 255i32 as libc::c_long) as
                             libc::c_int) as libc::c_long +
                        (8i32 as libc::c_long -
                             90i32 as libc::c_long /
                                 (((PASSTHRU_MODE as libc::c_int as
                                        libc::c_long * 1i64 >>
                                        16i32 *
                                            (PASSTHRU_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (PASSTHRU_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (PASSTHRU_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (PASSTHRU_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (PASSTHRU_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((PASSTHRU_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (PASSTHRU_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (PASSTHRU_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (PASSTHRU_MODE as
                                                         libc::c_int as
                                                         libc::c_long * 1i64
                                                         >>
                                                         16i32 *
                                                             (PASSTHRU_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (PASSTHRU_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      4i32 as libc::c_long +
                                      14i32 as libc::c_long |
                                      1i32 as libc::c_long) -
                             2i32 as libc::c_long /
                                 (((PASSTHRU_MODE as libc::c_int as
                                        libc::c_long * 1i64 >>
                                        16i32 *
                                            (PASSTHRU_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (PASSTHRU_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (PASSTHRU_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (PASSTHRU_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (PASSTHRU_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((PASSTHRU_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (PASSTHRU_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (PASSTHRU_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (PASSTHRU_MODE as
                                                         libc::c_int as
                                                         libc::c_long * 1i64
                                                         >>
                                                         16i32 *
                                                             (PASSTHRU_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (PASSTHRU_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      2i32 as libc::c_long +
                                      1i32 as libc::c_long))))) as uint8_t,
         ((32i32 *
               (FAILSAFE_MODE as libc::c_int as libc::c_long / 2i64 >> 31i32 >
                    0i32 as libc::c_long) as libc::c_int) as libc::c_long +
              ((16i32 *
                    (FAILSAFE_MODE as libc::c_int as libc::c_long * 1i64 >>
                         16i32 *
                             (FAILSAFE_MODE as libc::c_int as libc::c_long /
                                  2i64 >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int >>
                         16i32 *
                             (FAILSAFE_MODE as libc::c_int as libc::c_long /
                                  2i64 >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int > 65535i64) as libc::c_int) as
                   libc::c_long +
                   ((8i32 *
                         ((FAILSAFE_MODE as libc::c_int as libc::c_long * 1i64
                               >>
                               16i32 *
                                   (FAILSAFE_MODE as libc::c_int as
                                        libc::c_long / 2i64 >> 31i32 >
                                        0i32 as libc::c_long) as libc::c_int
                               >>
                               16i32 *
                                   (FAILSAFE_MODE as libc::c_int as
                                        libc::c_long / 2i64 >> 31i32 >
                                        0i32 as libc::c_long) as libc::c_int)
                              * 1i64 >>
                              16i32 *
                                  (FAILSAFE_MODE as libc::c_int as
                                       libc::c_long * 1i64 >>
                                       16i32 *
                                           (FAILSAFE_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int >>
                                       16i32 *
                                           (FAILSAFE_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int > 65535i64) as
                                      libc::c_int > 255i32 as libc::c_long) as
                             libc::c_int) as libc::c_long +
                        (8i32 as libc::c_long -
                             90i32 as libc::c_long /
                                 (((FAILSAFE_MODE as libc::c_int as
                                        libc::c_long * 1i64 >>
                                        16i32 *
                                            (FAILSAFE_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (FAILSAFE_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (FAILSAFE_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (FAILSAFE_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (FAILSAFE_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((FAILSAFE_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (FAILSAFE_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (FAILSAFE_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (FAILSAFE_MODE as
                                                         libc::c_int as
                                                         libc::c_long * 1i64
                                                         >>
                                                         16i32 *
                                                             (FAILSAFE_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (FAILSAFE_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      4i32 as libc::c_long +
                                      14i32 as libc::c_long |
                                      1i32 as libc::c_long) -
                             2i32 as libc::c_long /
                                 (((FAILSAFE_MODE as libc::c_int as
                                        libc::c_long * 1i64 >>
                                        16i32 *
                                            (FAILSAFE_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (FAILSAFE_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (FAILSAFE_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (FAILSAFE_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (FAILSAFE_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((FAILSAFE_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (FAILSAFE_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (FAILSAFE_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (FAILSAFE_MODE as
                                                         libc::c_int as
                                                         libc::c_long * 1i64
                                                         >>
                                                         16i32 *
                                                             (FAILSAFE_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (FAILSAFE_MODE as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      2i32 as libc::c_long +
                                      1i32 as libc::c_long))))) as uint8_t,
         ((32i32 *
               (GPS_RESCUE_MODE as libc::c_int as libc::c_long / 2i64 >> 31i32
                    > 0i32 as libc::c_long) as libc::c_int) as libc::c_long +
              ((16i32 *
                    (GPS_RESCUE_MODE as libc::c_int as libc::c_long * 1i64 >>
                         16i32 *
                             (GPS_RESCUE_MODE as libc::c_int as libc::c_long /
                                  2i64 >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int >>
                         16i32 *
                             (GPS_RESCUE_MODE as libc::c_int as libc::c_long /
                                  2i64 >> 31i32 > 0i32 as libc::c_long) as
                                 libc::c_int > 65535i64) as libc::c_int) as
                   libc::c_long +
                   ((8i32 *
                         ((GPS_RESCUE_MODE as libc::c_int as libc::c_long *
                               1i64 >>
                               16i32 *
                                   (GPS_RESCUE_MODE as libc::c_int as
                                        libc::c_long / 2i64 >> 31i32 >
                                        0i32 as libc::c_long) as libc::c_int
                               >>
                               16i32 *
                                   (GPS_RESCUE_MODE as libc::c_int as
                                        libc::c_long / 2i64 >> 31i32 >
                                        0i32 as libc::c_long) as libc::c_int)
                              * 1i64 >>
                              16i32 *
                                  (GPS_RESCUE_MODE as libc::c_int as
                                       libc::c_long * 1i64 >>
                                       16i32 *
                                           (GPS_RESCUE_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int >>
                                       16i32 *
                                           (GPS_RESCUE_MODE as libc::c_int as
                                                libc::c_long / 2i64 >> 31i32 >
                                                0i32 as libc::c_long) as
                                               libc::c_int > 65535i64) as
                                      libc::c_int > 255i32 as libc::c_long) as
                             libc::c_int) as libc::c_long +
                        (8i32 as libc::c_long -
                             90i32 as libc::c_long /
                                 (((GPS_RESCUE_MODE as libc::c_int as
                                        libc::c_long * 1i64 >>
                                        16i32 *
                                            (GPS_RESCUE_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (GPS_RESCUE_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (GPS_RESCUE_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (GPS_RESCUE_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (GPS_RESCUE_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((GPS_RESCUE_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (GPS_RESCUE_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (GPS_RESCUE_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (GPS_RESCUE_MODE as
                                                         libc::c_int as
                                                         libc::c_long * 1i64
                                                         >>
                                                         16i32 *
                                                             (GPS_RESCUE_MODE
                                                                  as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (GPS_RESCUE_MODE
                                                                  as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      4i32 as libc::c_long +
                                      14i32 as libc::c_long |
                                      1i32 as libc::c_long) -
                             2i32 as libc::c_long /
                                 (((GPS_RESCUE_MODE as libc::c_int as
                                        libc::c_long * 1i64 >>
                                        16i32 *
                                            (GPS_RESCUE_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int >>
                                        16i32 *
                                            (GPS_RESCUE_MODE as libc::c_int as
                                                 libc::c_long / 2i64 >> 31i32
                                                 > 0i32 as libc::c_long) as
                                                libc::c_int) * 1i64 >>
                                       16i32 *
                                           (GPS_RESCUE_MODE as libc::c_int as
                                                libc::c_long * 1i64 >>
                                                16i32 *
                                                    (GPS_RESCUE_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >>
                                                16i32 *
                                                    (GPS_RESCUE_MODE as
                                                         libc::c_int as
                                                         libc::c_long / 2i64
                                                         >> 31i32 >
                                                         0i32 as libc::c_long)
                                                        as libc::c_int >
                                                65535i64) as libc::c_int >>
                                       8i32 *
                                           ((GPS_RESCUE_MODE as libc::c_int as
                                                 libc::c_long * 1i64 >>
                                                 16i32 *
                                                     (GPS_RESCUE_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 16i32 *
                                                     (GPS_RESCUE_MODE as
                                                          libc::c_int as
                                                          libc::c_long / 2i64
                                                          >> 31i32 >
                                                          0i32 as
                                                              libc::c_long) as
                                                         libc::c_int) * 1i64
                                                >>
                                                16i32 *
                                                    (GPS_RESCUE_MODE as
                                                         libc::c_int as
                                                         libc::c_long * 1i64
                                                         >>
                                                         16i32 *
                                                             (GPS_RESCUE_MODE
                                                                  as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int
                                                         >>
                                                         16i32 *
                                                             (GPS_RESCUE_MODE
                                                                  as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_long
                                                                  / 2i64 >>
                                                                  31i32 >
                                                                  0i32 as
                                                                      libc::c_long)
                                                                 as
                                                                 libc::c_int >
                                                         65535i64) as
                                                        libc::c_int >
                                                255i32 as libc::c_long) as
                                               libc::c_int) /
                                      2i32 as libc::c_long +
                                      1i32 as libc::c_long))))) as uint8_t];
    // we assume that all boxId below BOXID_FLIGHTMODE_LAST except BOXARM are mapped to flightmode
    if boxid as libc::c_uint == BOXARM as libc::c_int as libc::c_uint {
        return armingFlags as libc::c_int & ARMED as libc::c_int != 0
    } else if boxid as libc::c_uint <=
                  BOXID_FLIGHTMODE_LAST as libc::c_int as libc::c_uint {
        return flightModeFlags as libc::c_int &
                   1i32 << boxIdToFlightModeMap[boxid as usize] as libc::c_int
                   != 0
    } else { return IS_RC_MODE_ACTIVE(boxid) };
}
// pack used flightModeFlags into supplied array
// returns number of bits used
#[no_mangle]
pub unsafe extern "C" fn packFlightModeFlags(mut mspFlightModeFlags:
                                                 *mut boxBitmask_t)
 -> libc::c_int {
    // Serialize the flags in the order we delivered them, ignoring BOXNAMES and BOXINDEXES
    memset(mspFlightModeFlags as *mut libc::c_void, 0i32,
           ::core::mem::size_of::<boxBitmask_t>() as libc::c_ulong);
    // map boxId_e enabled bits to MSP status indexes
    // only active boxIds are sent in status over MSP, other bits are not counted
    let mut mspBoxIdx: libc::c_uint =
        0i32 as
            libc::c_uint; // index of active boxId (matches sent permanentId and boxNames)
    let mut boxId: boxId_e = BOXARM; // box is enabled
    while (boxId as libc::c_uint) <
              CHECKBOX_ITEM_COUNT as libc::c_int as libc::c_uint {
        if activeBoxIdGet(boxId) {
            if getBoxIdState(boxId) {
                bitArraySet(mspFlightModeFlags as *mut libc::c_void,
                            mspBoxIdx);
            }
            mspBoxIdx = mspBoxIdx.wrapping_add(1)
            // box is active, count it
        }
        boxId += 1
    }
    // return count of used bits
    return mspBoxIdx as libc::c_int;
}
// USE_OSD_SLAVE
