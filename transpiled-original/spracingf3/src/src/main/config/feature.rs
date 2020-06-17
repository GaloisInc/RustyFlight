use core;
use libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type pgn_t = uint16_t;
pub type C2RustUnnamed = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed = 4095;
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_0,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed_0 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<unsafe extern "C" fn(_: *mut libc::c_void,
                                          _: libc::c_int) -> ()>,
}
pub type pgRegistry_t = pgRegistry_s;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_1 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_1 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_1 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_1 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_1 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_1 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_1 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_1 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_1 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_1 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_1 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_1 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_1 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_1 = 8192;
pub const FEATURE_3D: C2RustUnnamed_1 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_1 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_1 = 512;
pub const FEATURE_GPS: C2RustUnnamed_1 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_1 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_1 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_1 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_1 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_1 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_1 = 1;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct featureConfig_s {
    pub enabledFeatures: uint32_t,
}
pub type featureConfig_t = featureConfig_s;
#[inline]
unsafe extern "C" fn featureConfigMutable() -> *mut featureConfig_t {
    return &mut featureConfig_System;
}
#[inline]
unsafe extern "C" fn featureConfig() -> *const featureConfig_t {
    return &mut featureConfig_System;
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
static mut activeFeaturesLatch: uint32_t = 0i32 as uint32_t;
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut featureConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn: (19i32 | 0i32 << 12i32) as pgn_t,
                             size:
                                 (::core::mem::size_of::<featureConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &featureConfig_System as
                                     *const featureConfig_t as
                                     *mut featureConfig_t as *mut uint8_t,
                             copy:
                                 &featureConfig_Copy as *const featureConfig_t
                                     as *mut featureConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     &pgResetTemplate_featureConfig
                                                         as
                                                         *const featureConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut featureConfig_System: featureConfig_t =
    featureConfig_t{enabledFeatures: 0,};
#[no_mangle]
pub static mut featureConfig_Copy: featureConfig_t =
    featureConfig_t{enabledFeatures: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_featureConfig: featureConfig_t =
    {
        let mut init =
            featureConfig_s{enabledFeatures:
                                (FEATURE_RSSI_ADC as libc::c_int |
                                     FEATURE_TELEMETRY as libc::c_int |
                                     FEATURE_RX_PPM as libc::c_int |
                                     FEATURE_DYNAMIC_FILTER as libc::c_int |
                                     FEATURE_ANTI_GRAVITY as libc::c_int) as
                                    uint32_t,};
        init
    };
#[no_mangle]
pub unsafe extern "C" fn intFeatureSet(mut mask: uint32_t,
                                       mut features: *mut uint32_t) {
    *features |= mask;
}
#[no_mangle]
pub unsafe extern "C" fn intFeatureClear(mut mask: uint32_t,
                                         mut features: *mut uint32_t) {
    *features &= !mask;
}
#[no_mangle]
pub unsafe extern "C" fn intFeatureClearAll(mut features: *mut uint32_t) {
    *features = 0i32 as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn latchActiveFeatures() {
    activeFeaturesLatch = (*featureConfig()).enabledFeatures;
}
#[no_mangle]
pub unsafe extern "C" fn featureConfigured(mut mask: uint32_t) -> bool {
    return (*featureConfig()).enabledFeatures & mask != 0;
}
#[no_mangle]
pub unsafe extern "C" fn feature(mut mask: uint32_t) -> bool {
    return activeFeaturesLatch & mask != 0;
}
#[no_mangle]
pub unsafe extern "C" fn featureSet(mut mask: uint32_t) {
    intFeatureSet(mask,
                  &mut (*(featureConfigMutable as
                              unsafe extern "C" fn()
                                  ->
                                      *mut featureConfig_t)()).enabledFeatures);
}
#[no_mangle]
pub unsafe extern "C" fn featureClear(mut mask: uint32_t) {
    intFeatureClear(mask,
                    &mut (*(featureConfigMutable as
                                unsafe extern "C" fn()
                                    ->
                                        *mut featureConfig_t)()).enabledFeatures);
}
#[no_mangle]
pub unsafe extern "C" fn featureClearAll() {
    intFeatureClearAll(&mut (*(featureConfigMutable as
                                   unsafe extern "C" fn()
                                       ->
                                           *mut featureConfig_t)()).enabledFeatures);
}
#[no_mangle]
pub unsafe extern "C" fn featureMask() -> uint32_t {
    return (*featureConfig()).enabledFeatures;
}
