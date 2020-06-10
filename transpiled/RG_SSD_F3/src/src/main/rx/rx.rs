use ::libc;
extern "C" {
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
    #[no_mangle]
    static mut debug: [int16_t; 4];
    #[no_mangle]
    static mut debugMode: uint8_t;
    #[no_mangle]
    fn scaleRange(x: libc::c_int, srcFrom: libc::c_int, srcTo: libc::c_int,
                  destFrom: libc::c_int, destTo: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    fn featureClear(mask: uint32_t);
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn millis() -> timeMs_t;
    #[no_mangle]
    fn adcGetChannel(channel: uint8_t) -> uint16_t;
    #[no_mangle]
    fn isPPMDataBeingReceived() -> bool;
    #[no_mangle]
    fn resetPPMDataReceivedState();
    #[no_mangle]
    fn isPWMDataBeingReceived() -> bool;
    #[no_mangle]
    static mut modeActivationConditions_SystemArray:
           [modeActivationCondition_t; 20];
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    fn failsafeOnRxSuspend(suspendPeriod: uint32_t);
    #[no_mangle]
    fn failsafeOnRxResume();
    #[no_mangle]
    fn failsafeOnValidDataReceived();
    #[no_mangle]
    fn failsafeOnValidDataFailed();
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
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
    fn rxPwmInit(rxConfig_0: *const rxConfig_t,
                 rxRuntimeConfig_0: *mut rxRuntimeConfig_t);
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
    fn fportRxInit(initialRxConfig: *const rxConfig_t,
                   rxRuntimeConfig_0: *mut rxRuntimeConfig_t) -> bool;
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
    fn sbusInit(initialRxConfig: *const rxConfig_t,
                rxRuntimeConfig_0: *mut rxRuntimeConfig_t) -> bool;
    #[no_mangle]
    fn spektrumInit(rxConfig_0: *const rxConfig_t,
                    rxRuntimeConfig_0: *mut rxRuntimeConfig_t) -> bool;
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
    fn sumdInit(rxConfig_0: *const rxConfig_t,
                rxRuntimeConfig_0: *mut rxRuntimeConfig_t) -> bool;
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
    fn sumhInit(rxConfig_0: *const rxConfig_t,
                rxRuntimeConfig_0: *mut rxRuntimeConfig_t) -> bool;
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
    fn xBusInit(rxConfig_0: *const rxConfig_t,
                rxRuntimeConfig_0: *mut rxRuntimeConfig_t) -> bool;
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
    fn ibusInit(rxConfig_0: *const rxConfig_t,
                rxRuntimeConfig_0: *mut rxRuntimeConfig_t) -> bool;
    #[no_mangle]
    fn crsfRxInit(initialRxConfig: *const rxConfig_s,
                  rxRuntimeConfig_0: *mut rxRuntimeConfig_s) -> bool;
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
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const ADC_CHANNEL_COUNT: C2RustUnnamed_3 = 4;
pub const ADC_RSSI: C2RustUnnamed_3 = 3;
pub const ADC_EXTERNAL1: C2RustUnnamed_3 = 2;
pub const ADC_CURRENT: C2RustUnnamed_3 = 1;
pub const ADC_BATTERY: C2RustUnnamed_3 = 0;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct modeActivationCondition_s {
    pub modeId: boxId_e,
    pub auxChannelIndex: uint8_t,
    pub range: channelRange_t,
    pub modeLogic: modeLogic_e,
    pub linkedTo: boxId_e,
}
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
pub type modeLogic_e = libc::c_uint;
pub const MODELOGIC_AND: modeLogic_e = 1;
pub const MODELOGIC_OR: modeLogic_e = 0;
// steps are 25 apart
// a value of 0 corresponds to a channel value of 900 or less
// a value of 48 corresponds to a channel value of 2100 or more
// 48 steps between 900 and 2100
pub type channelRange_t = channelRange_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct channelRange_s {
    pub startStep: uint8_t,
    pub endStep: uint8_t,
}
pub type modeActivationCondition_t = modeActivationCondition_s;
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
pub struct rxConfig_s {
    pub rcmap: [uint8_t; 8],
    pub serialrx_provider: uint8_t,
    pub serialrx_inverted: uint8_t,
    pub halfDuplex: uint8_t,
    pub spektrum_bind_pin_override_ioTag: ioTag_t,
    pub spektrum_bind_plug_ioTag: ioTag_t,
    pub spektrum_sat_bind: uint8_t,
    pub spektrum_sat_bind_autoreset: uint8_t,
    pub rssi_channel: uint8_t,
    pub rssi_scale: uint8_t,
    pub rssi_invert: uint8_t,
    pub midrc: uint16_t,
    pub mincheck: uint16_t,
    pub maxcheck: uint16_t,
    pub rcInterpolation: uint8_t,
    pub rcInterpolationChannels: uint8_t,
    pub rcInterpolationInterval: uint8_t,
    pub fpvCamAngleDegrees: uint8_t,
    pub airModeActivateThreshold: uint8_t,
    pub rx_min_usec: uint16_t,
    pub rx_max_usec: uint16_t,
    pub max_aux_channel: uint8_t,
    pub rssi_src_frame_errors: uint8_t,
    pub rssi_offset: int8_t,
    pub rc_smoothing_type: uint8_t,
    pub rc_smoothing_input_cutoff: uint8_t,
    pub rc_smoothing_derivative_cutoff: uint8_t,
    pub rc_smoothing_debug_axis: uint8_t,
    pub rc_smoothing_input_type: uint8_t,
    pub rc_smoothing_derivative_type: uint8_t,
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
pub type rxConfig_t = rxConfig_s;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const RX_FRAME_DROPPED: C2RustUnnamed_4 = 8;
pub const RX_FRAME_PROCESSING_REQUIRED: C2RustUnnamed_4 = 4;
pub const RX_FRAME_FAILSAFE: C2RustUnnamed_4 = 2;
pub const RX_FRAME_COMPLETE: C2RustUnnamed_4 = 1;
pub const RX_FRAME_PENDING: C2RustUnnamed_4 = 0;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const SERIALRX_FPORT: C2RustUnnamed_5 = 12;
pub const SERIALRX_TARGET_CUSTOM: C2RustUnnamed_5 = 11;
pub const SERIALRX_SRXL: C2RustUnnamed_5 = 10;
pub const SERIALRX_CRSF: C2RustUnnamed_5 = 9;
pub const SERIALRX_JETIEXBUS: C2RustUnnamed_5 = 8;
pub const SERIALRX_IBUS: C2RustUnnamed_5 = 7;
pub const SERIALRX_XBUS_MODE_B_RJ01: C2RustUnnamed_5 = 6;
pub const SERIALRX_XBUS_MODE_B: C2RustUnnamed_5 = 5;
pub const SERIALRX_SUMH: C2RustUnnamed_5 = 4;
pub const SERIALRX_SUMD: C2RustUnnamed_5 = 3;
pub const SERIALRX_SBUS: C2RustUnnamed_5 = 2;
pub const SERIALRX_SPEKTRUM2048: C2RustUnnamed_5 = 1;
pub const SERIALRX_SPEKTRUM1024: C2RustUnnamed_5 = 0;
pub type C2RustUnnamed_6 = libc::c_uint;
pub const RX_FAILSAFE_MODE_INVALID: C2RustUnnamed_6 = 3;
pub const RX_FAILSAFE_MODE_SET: C2RustUnnamed_6 = 2;
pub const RX_FAILSAFE_MODE_HOLD: C2RustUnnamed_6 = 1;
pub const RX_FAILSAFE_MODE_AUTO: C2RustUnnamed_6 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxFailsafeChannelConfig_s {
    pub mode: uint8_t,
    pub step: uint8_t,
}
pub type rxFailsafeChannelConfig_t = rxFailsafeChannelConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxChannelRangeConfig_s {
    pub min: uint16_t,
    pub max: uint16_t,
}
pub type rxChannelRangeConfig_t = rxChannelRangeConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxRuntimeConfig_s {
    pub channelCount: uint8_t,
    pub rxRefreshRate: uint16_t,
    pub rcReadRawFn: rcReadRawDataFnPtr,
    pub rcFrameStatusFn: rcFrameStatusFnPtr,
    pub rcProcessFrameFn: rcProcessFrameFnPtr,
    pub channelData: *mut uint16_t,
    pub frameData: *mut libc::c_void,
}
pub type rcProcessFrameFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s) -> bool>;
// mapping of radio channels to internal RPYTA+ order
// type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
// invert the serial RX protocol compared to it's default setting
// allow rx to operate in half duplex mode on F4, ignored for F1 and F3.
// number of bind pulses for Spektrum satellite receivers
// whenever we will reset (exit) binding mode after hard reboot
// Some radios have not a neutral point centered on 1500. can be changed here
// minimum rc end
// maximum rc end
// Camera angle to be scaled into rc commands
// Throttle setpoint percent where airmode gets activated
// true to use frame drop flags in the rx protocol
// offset applied to the RSSI value before it is returned
// Determines the smoothing algorithm to use: INTERPOLATION or FILTER
// Filter cutoff frequency for the input filter (0 = auto)
// Filter cutoff frequency for the setpoint weight derivative filter (0 = auto)
// Axis to log as debug values when debug_mode = RC_SMOOTHING
// Input filter type (0 = PT1, 1 = BIQUAD)
// Derivative filter type (0 = OFF, 1 = PT1, 2 = BIQUAD)
// See rxFailsafeChannelMode_e
// used by receiver driver to return channel data
pub type rcFrameStatusFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut rxRuntimeConfig_s) -> uint8_t>;
pub type rcReadRawDataFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s, _: uint8_t)
               -> uint16_t>;
pub type rxRuntimeConfig_t = rxRuntimeConfig_s;
pub type rssiSource_e = libc::c_uint;
pub const RSSI_SOURCE_FRAME_ERRORS: rssiSource_e = 5;
pub const RSSI_SOURCE_MSP: rssiSource_e = 4;
pub const RSSI_SOURCE_RX_PROTOCOL: rssiSource_e = 3;
pub const RSSI_SOURCE_RX_CHANNEL: rssiSource_e = 2;
pub const RSSI_SOURCE_ADC: rssiSource_e = 1;
pub const RSSI_SOURCE_NONE: rssiSource_e = 0;
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
unsafe extern "C" fn cmpTimeUs(mut a: timeUs_t, mut b: timeUs_t)
 -> timeDelta_t {
    return a.wrapping_sub(b) as timeDelta_t;
}
#[inline]
unsafe extern "C" fn modeActivationConditions(mut _index: libc::c_int)
 -> *const modeActivationCondition_t {
    return &mut *modeActivationConditions_SystemArray.as_mut_ptr().offset(_index
                                                                              as
                                                                              isize)
               as *mut modeActivationCondition_t;
}
#[inline]
unsafe extern "C" fn rxConfig() -> *const rxConfig_t {
    return &mut rxConfig_System;
}
#[inline]
unsafe extern "C" fn rxFailsafeChannelConfigs(mut _index: libc::c_int)
 -> *const rxFailsafeChannelConfig_t {
    return &mut *rxFailsafeChannelConfigs_SystemArray.as_mut_ptr().offset(_index
                                                                              as
                                                                              isize)
               as *mut rxFailsafeChannelConfig_t;
}
#[inline]
unsafe extern "C" fn rxChannelRangeConfigs(mut _index: libc::c_int)
 -> *const rxChannelRangeConfig_t {
    return &mut *rxChannelRangeConfigs_SystemArray.as_mut_ptr().offset(_index
                                                                           as
                                                                           isize)
               as *mut rxChannelRangeConfig_t;
}
// number of RC channels as reported by current input driver
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
pub static mut rcChannelLetters: [libc::c_char; 21] =
    [65, 69, 82, 84, 49, 50, 51, 52, 53, 54, 55, 56, 97, 98, 99, 100, 101,
     102, 103, 104, 0];
static mut rssi: uint16_t = 0 as libc::c_int as uint16_t;
// range: [0;1023]
static mut lastMspRssiUpdateUs: timeUs_t = 0 as libc::c_int as timeUs_t;
#[no_mangle]
pub static mut rssiSource: rssiSource_e = RSSI_SOURCE_NONE;
static mut rxDataProcessingRequired: bool = 0 as libc::c_int != 0;
static mut auxiliaryProcessingRequired: bool = 0 as libc::c_int != 0;
static mut rxSignalReceived: bool = 0 as libc::c_int != 0;
static mut rxFlightChannelsValid: bool = 0 as libc::c_int != 0;
static mut rxIsInFailsafeMode: bool = 1 as libc::c_int != 0;
static mut rxChannelCount: uint8_t = 0;
static mut rxNextUpdateAtUs: timeUs_t = 0 as libc::c_int as timeUs_t;
static mut needRxSignalBefore: uint32_t = 0 as libc::c_int as uint32_t;
static mut needRxSignalMaxDelayUs: uint32_t = 0;
static mut suspendRxSignalUntil: uint32_t = 0 as libc::c_int as uint32_t;
static mut skipRxSamples: uint8_t = 0 as libc::c_int as uint8_t;
static mut rcRaw: [int16_t; 18] = [0; 18];
// interval [1000;2000]
#[no_mangle]
pub static mut rcData: [int16_t; 18] = [0; 18];
// interval [1000;2000]
#[no_mangle]
pub static mut rcInvalidPulsPeriod: [uint32_t; 18] = [0; 18];
// flush 2 samples to drop wrong measurements (timing independent)
#[no_mangle]
pub static mut rxRuntimeConfig: rxRuntimeConfig_t =
    rxRuntimeConfig_t{channelCount: 0,
                      rxRefreshRate: 0,
                      rcReadRawFn: None,
                      rcFrameStatusFn: None,
                      rcProcessFrameFn: None,
                      channelData: 0 as *const uint16_t as *mut uint16_t,
                      frameData:
                          0 as *const libc::c_void as *mut libc::c_void,};
static mut rcSampleIndex: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub static mut rxChannelRangeConfigs_SystemArray: [rxChannelRangeConfig_t; 4]
           =
    [rxChannelRangeConfig_t{min: 0, max: 0,}; 4];
#[no_mangle]
pub static mut rxChannelRangeConfigs_CopyArray: [rxChannelRangeConfig_t; 4] =
    [rxChannelRangeConfig_t{min: 0, max: 0,}; 4];
// Initialized in run_static_initializers
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut rxChannelRangeConfigs_Registry: pgRegistry_t =
    pgRegistry_t{pgn: 0,
                 size: 0,
                 address: 0 as *const uint8_t as *mut uint8_t,
                 copy: 0 as *const uint8_t as *mut uint8_t,
                 ptr: 0 as *const *mut uint8_t as *mut *mut uint8_t,
                 reset:
                     C2RustUnnamed_1{ptr:
                                         0 as *const libc::c_void as
                                             *mut libc::c_void,},};
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_rxChannelRangeConfigs(mut rxChannelRangeConfigs_0:
                                                             *mut rxChannelRangeConfig_t) {
    // set default calibration to full range and 1:1 mapping
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 4 as libc::c_int {
        (*rxChannelRangeConfigs_0.offset(i as isize)).min =
            1000 as libc::c_int as uint16_t;
        (*rxChannelRangeConfigs_0.offset(i as isize)).max =
            2000 as libc::c_int as uint16_t;
        i += 1
    };
}
#[no_mangle]
pub static mut rxFailsafeChannelConfigs_SystemArray:
           [rxFailsafeChannelConfig_t; 18] =
    [rxFailsafeChannelConfig_t{mode: 0, step: 0,}; 18];
// Initialized in run_static_initializers
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut rxFailsafeChannelConfigs_Registry: pgRegistry_t =
    pgRegistry_t{pgn: 0,
                 size: 0,
                 address: 0 as *const uint8_t as *mut uint8_t,
                 copy: 0 as *const uint8_t as *mut uint8_t,
                 ptr: 0 as *const *mut uint8_t as *mut *mut uint8_t,
                 reset:
                     C2RustUnnamed_1{ptr:
                                         0 as *const libc::c_void as
                                             *mut libc::c_void,},};
#[no_mangle]
pub static mut rxFailsafeChannelConfigs_CopyArray:
           [rxFailsafeChannelConfig_t; 18] =
    [rxFailsafeChannelConfig_t{mode: 0, step: 0,}; 18];
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_rxFailsafeChannelConfigs(mut rxFailsafeChannelConfigs_0:
                                                                *mut rxFailsafeChannelConfig_t) {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 18 as libc::c_int {
        (*rxFailsafeChannelConfigs_0.offset(i as isize)).mode =
            if i < 4 as libc::c_int {
                RX_FAILSAFE_MODE_AUTO as libc::c_int
            } else { RX_FAILSAFE_MODE_HOLD as libc::c_int } as uint8_t;
        (*rxFailsafeChannelConfigs_0.offset(i as isize)).step =
            if i == THROTTLE as libc::c_int {
                (constrain(885 as libc::c_int, 750 as libc::c_int,
                           2250 as libc::c_int) - 750 as libc::c_int) /
                    25 as libc::c_int
            } else {
                (constrain(1500 as libc::c_int, 750 as libc::c_int,
                           2250 as libc::c_int) - 750 as libc::c_int) /
                    25 as libc::c_int
            } as uint8_t;
        i += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn resetAllRxChannelRangeConfigurations(mut rxChannelRangeConfig:
                                                                  *mut rxChannelRangeConfig_t) {
    // set default calibration to full range and 1:1 mapping
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 4 as libc::c_int {
        (*rxChannelRangeConfig).min = 1000 as libc::c_int as uint16_t;
        (*rxChannelRangeConfig).max = 2000 as libc::c_int as uint16_t;
        rxChannelRangeConfig = rxChannelRangeConfig.offset(1);
        i += 1
    };
}
unsafe extern "C" fn nullReadRawRC(mut rxRuntimeConfig_0:
                                       *const rxRuntimeConfig_t,
                                   mut channel: uint8_t) -> uint16_t {
    return 0 as libc::c_int as uint16_t;
}
unsafe extern "C" fn nullFrameStatus(mut rxRuntimeConfig_0:
                                         *mut rxRuntimeConfig_t) -> uint8_t {
    return RX_FRAME_PENDING as libc::c_int as uint8_t;
}
unsafe extern "C" fn nullProcessFrame(mut rxRuntimeConfig_0:
                                          *const rxRuntimeConfig_t) -> bool {
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn isPulseValid(mut pulseDuration: uint16_t) -> bool {
    return pulseDuration as libc::c_int >=
               (*rxConfig()).rx_min_usec as libc::c_int &&
               pulseDuration as libc::c_int <=
                   (*rxConfig()).rx_max_usec as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn serialRxInit(mut rxConfig_0: *const rxConfig_t,
                                      mut rxRuntimeConfig_0:
                                          *mut rxRuntimeConfig_t) -> bool {
    let mut enabled: bool = 0 as libc::c_int != 0;
    match (*rxConfig_0).serialrx_provider as libc::c_int {
        10 | 0 | 1 => {
            enabled = spektrumInit(rxConfig_0, rxRuntimeConfig_0)
        }
        2 => { enabled = sbusInit(rxConfig_0, rxRuntimeConfig_0) }
        3 => { enabled = sumdInit(rxConfig_0, rxRuntimeConfig_0) }
        4 => { enabled = sumhInit(rxConfig_0, rxRuntimeConfig_0) }
        5 | 6 => { enabled = xBusInit(rxConfig_0, rxRuntimeConfig_0) }
        7 => { enabled = ibusInit(rxConfig_0, rxRuntimeConfig_0) }
        9 => { enabled = crsfRxInit(rxConfig_0, rxRuntimeConfig_0) }
        12 => { enabled = fportRxInit(rxConfig_0, rxRuntimeConfig_0) }
        _ => { enabled = 0 as libc::c_int != 0 }
    }
    return enabled;
}
// !!TODO remove this extern, only needed once for channelCount
#[no_mangle]
pub unsafe extern "C" fn rxInit() {
    rxRuntimeConfig.rcReadRawFn =
        Some(nullReadRawRC as
                 unsafe extern "C" fn(_: *const rxRuntimeConfig_t, _: uint8_t)
                     -> uint16_t);
    rxRuntimeConfig.rcFrameStatusFn =
        Some(nullFrameStatus as
                 unsafe extern "C" fn(_: *mut rxRuntimeConfig_t) -> uint8_t);
    rxRuntimeConfig.rcProcessFrameFn =
        Some(nullProcessFrame as
                 unsafe extern "C" fn(_: *const rxRuntimeConfig_t) -> bool);
    rcSampleIndex = 0 as libc::c_int as uint8_t;
    needRxSignalMaxDelayUs =
        (1000000 as libc::c_int / 10 as libc::c_int) as uint32_t;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 18 as libc::c_int {
        rcData[i as usize] = (*rxConfig()).midrc as int16_t;
        rcInvalidPulsPeriod[i as usize] =
            millis().wrapping_add(300 as libc::c_int as libc::c_uint);
        i += 1
    }
    rcData[THROTTLE as libc::c_int as usize] =
        if feature(FEATURE_3D as libc::c_int as uint32_t) as libc::c_int != 0
           {
            (*rxConfig()).midrc as libc::c_int
        } else { (*rxConfig()).rx_min_usec as libc::c_int } as int16_t;
    // Initialize ARM switch to OFF position when arming via switch is defined
    // TODO - move to rc_mode.c
    let mut i_0: libc::c_int = 0 as libc::c_int;
    while i_0 < 20 as libc::c_int {
        let mut modeActivationCondition: *const modeActivationCondition_t =
            modeActivationConditions(i_0);
        if (*modeActivationCondition).modeId as libc::c_uint ==
               BOXARM as libc::c_int as libc::c_uint &&
               ((*modeActivationCondition).range.startStep as libc::c_int) <
                   (*modeActivationCondition).range.endStep as libc::c_int {
            // ARM switch is defined, determine an OFF value
            let mut value: uint16_t = 0;
            if (*modeActivationCondition).range.startStep as libc::c_int >
                   0 as libc::c_int {
                value =
                    (900 as libc::c_int +
                         25 as libc::c_int *
                             ((*modeActivationCondition).range.startStep as
                                  libc::c_int - 1 as libc::c_int)) as uint16_t
            } else {
                value =
                    (900 as libc::c_int +
                         25 as libc::c_int *
                             ((*modeActivationCondition).range.endStep as
                                  libc::c_int + 1 as libc::c_int)) as uint16_t
            }
            // Initialize ARM AUX channel to OFF value
            rcData[((*modeActivationCondition).auxChannelIndex as libc::c_int
                        + 4 as libc::c_int) as usize] = value as int16_t
        }
        i_0 += 1
    }
    if feature(FEATURE_RX_SERIAL as libc::c_int as uint32_t) {
        let enabled: bool = serialRxInit(rxConfig(), &mut rxRuntimeConfig);
        if !enabled {
            featureClear(FEATURE_RX_SERIAL as libc::c_int as uint32_t);
            rxRuntimeConfig.rcReadRawFn =
                Some(nullReadRawRC as
                         unsafe extern "C" fn(_: *const rxRuntimeConfig_t,
                                              _: uint8_t) -> uint16_t);
            rxRuntimeConfig.rcFrameStatusFn =
                Some(nullFrameStatus as
                         unsafe extern "C" fn(_: *mut rxRuntimeConfig_t)
                             -> uint8_t)
        }
    }
    if feature(FEATURE_RX_PPM as libc::c_int as uint32_t) as libc::c_int != 0
           ||
           feature(FEATURE_RX_PARALLEL_PWM as libc::c_int as uint32_t) as
               libc::c_int != 0 {
        rxPwmInit(rxConfig(), &mut rxRuntimeConfig);
    }
    if feature(FEATURE_RSSI_ADC as libc::c_int as uint32_t) {
        rssiSource = RSSI_SOURCE_ADC
    } else if (*rxConfig()).rssi_channel as libc::c_int > 0 as libc::c_int {
        rssiSource = RSSI_SOURCE_RX_CHANNEL
    }
    rxChannelCount =
        ({
             let mut _a: libc::c_int =
                 (*rxConfig()).max_aux_channel as libc::c_int +
                     4 as libc::c_int;
             let mut _b: uint8_t = rxRuntimeConfig.channelCount;
             if _a < _b as libc::c_int { _a } else { _b as libc::c_int }
         }) as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn rxIsReceivingSignal() -> bool {
    return rxSignalReceived;
}
#[no_mangle]
pub unsafe extern "C" fn rxAreFlightChannelsValid() -> bool {
    return rxFlightChannelsValid;
}
#[no_mangle]
pub unsafe extern "C" fn suspendRxSignal() {
    suspendRxSignalUntil =
        micros().wrapping_add(1500000 as libc::c_int as libc::c_uint);
    skipRxSamples = 2 as libc::c_int as uint8_t;
    failsafeOnRxSuspend(1500000 as libc::c_int as uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn resumeRxSignal() {
    suspendRxSignalUntil = micros();
    skipRxSamples = 2 as libc::c_int as uint8_t;
    failsafeOnRxResume();
}
#[no_mangle]
pub unsafe extern "C" fn rxUpdateCheck(mut currentTimeUs: timeUs_t,
                                       mut currentDeltaTime: timeDelta_t)
 -> bool {
    let mut signalReceived: bool = 0 as libc::c_int != 0;
    let mut useDataDrivenProcessing: bool = 1 as libc::c_int != 0;
    if feature(FEATURE_RX_PPM as libc::c_int as uint32_t) {
        if isPPMDataBeingReceived() {
            signalReceived = 1 as libc::c_int != 0;
            rxIsInFailsafeMode = 0 as libc::c_int != 0;
            needRxSignalBefore =
                currentTimeUs.wrapping_add(needRxSignalMaxDelayUs);
            resetPPMDataReceivedState();
        }
    } else if feature(FEATURE_RX_PARALLEL_PWM as libc::c_int as uint32_t) {
        if isPWMDataBeingReceived() {
            signalReceived = 1 as libc::c_int != 0;
            rxIsInFailsafeMode = 0 as libc::c_int != 0;
            needRxSignalBefore =
                currentTimeUs.wrapping_add(needRxSignalMaxDelayUs);
            useDataDrivenProcessing = 0 as libc::c_int != 0
        }
    } else {
        let frameStatus: uint8_t =
            rxRuntimeConfig.rcFrameStatusFn.expect("non-null function pointer")(&mut rxRuntimeConfig);
        if frameStatus as libc::c_int & RX_FRAME_COMPLETE as libc::c_int != 0
           {
            rxIsInFailsafeMode =
                frameStatus as libc::c_int & RX_FRAME_FAILSAFE as libc::c_int
                    != 0 as libc::c_int;
            let mut rxFrameDropped: bool =
                frameStatus as libc::c_int & RX_FRAME_DROPPED as libc::c_int
                    != 0 as libc::c_int;
            signalReceived =
                !(rxIsInFailsafeMode as libc::c_int != 0 ||
                      rxFrameDropped as libc::c_int != 0);
            if signalReceived {
                needRxSignalBefore =
                    currentTimeUs.wrapping_add(needRxSignalMaxDelayUs)
            }
            if frameStatus as libc::c_int &
                   (RX_FRAME_FAILSAFE as libc::c_int |
                        RX_FRAME_DROPPED as libc::c_int) != 0 {
                // No (0%) signal
                setRssi(0 as libc::c_int as uint16_t,
                        RSSI_SOURCE_FRAME_ERRORS);
            } else {
                // Valid (100%) signal
                setRssi(1023 as libc::c_int as uint16_t,
                        RSSI_SOURCE_FRAME_ERRORS);
            }
        }
        if frameStatus as libc::c_int &
               RX_FRAME_PROCESSING_REQUIRED as libc::c_int != 0 {
            auxiliaryProcessingRequired = 1 as libc::c_int != 0
        }
    }
    if signalReceived {
        rxSignalReceived = 1 as libc::c_int != 0
    } else if currentTimeUs >= needRxSignalBefore {
        rxSignalReceived = 0 as libc::c_int != 0
    }
    if signalReceived as libc::c_int != 0 &&
           useDataDrivenProcessing as libc::c_int != 0 ||
           cmpTimeUs(currentTimeUs, rxNextUpdateAtUs) > 0 as libc::c_int {
        rxDataProcessingRequired = 1 as libc::c_int != 0
    }
    return rxDataProcessingRequired as libc::c_int != 0 ||
               auxiliaryProcessingRequired as libc::c_int != 0;
    // data driven or 50Hz
}
unsafe extern "C" fn calculateChannelMovingAverage(mut chan: uint8_t,
                                                   mut sample: uint16_t)
 -> uint16_t {
    static mut rcSamples: [[int16_t; 3]; 12] = [[0; 3]; 12];
    static mut rcDataMean: [int16_t; 12] = [0; 12];
    static mut rxSamplesCollected: bool = 0 as libc::c_int != 0;
    let currentSampleIndex: uint8_t =
        (rcSampleIndex as libc::c_int % 3 as libc::c_int) as uint8_t;
    // update the recent samples and compute the average of them
    rcSamples[chan as usize][currentSampleIndex as usize] = sample as int16_t;
    // avoid returning an incorrect average which would otherwise occur before enough samples
    if !rxSamplesCollected {
        if (rcSampleIndex as libc::c_int) < 3 as libc::c_int { return sample }
        rxSamplesCollected = 1 as libc::c_int != 0
    }
    rcDataMean[chan as usize] = 0 as libc::c_int as int16_t;
    let mut sampleIndex: libc::c_int = 0 as libc::c_int;
    while sampleIndex < 3 as libc::c_int {
        rcDataMean[chan as usize] =
            (rcDataMean[chan as usize] as libc::c_int +
                 rcSamples[chan as usize][sampleIndex as usize] as
                     libc::c_int) as int16_t;
        sampleIndex += 1
    }
    return (rcDataMean[chan as usize] as libc::c_int / 3 as libc::c_int) as
               uint16_t;
}
unsafe extern "C" fn getRxfailValue(mut channel: uint8_t) -> uint16_t {
    let mut channelFailsafeConfig: *const rxFailsafeChannelConfig_t =
        rxFailsafeChannelConfigs(channel as libc::c_int);
    match (*channelFailsafeConfig).mode as libc::c_int {
        0 => {
            match channel as libc::c_int {
                0 | 1 | 2 => { return (*rxConfig()).midrc }
                3 => {
                    if feature(FEATURE_3D as libc::c_int as uint32_t) {
                        return (*rxConfig()).midrc
                    } else { return (*rxConfig()).rx_min_usec }
                }
                _ => { }
            }
        }
        2 => {
            return (750 as libc::c_int +
                        25 as libc::c_int *
                            (*channelFailsafeConfig).step as libc::c_int) as
                       uint16_t
        }
        3 | 1 | _ => { }
    }
    /* no break */
    return rcData[channel as usize] as uint16_t;
}
unsafe extern "C" fn applyRxChannelRangeConfiguraton(mut sample: libc::c_int,
                                                     mut range:
                                                         *const rxChannelRangeConfig_t)
 -> uint16_t {
    // Avoid corruption of channel with a value of PPM_RCVR_TIMEOUT
    if sample == 0 as libc::c_int { return 0 as libc::c_int as uint16_t }
    sample =
        scaleRange(sample, (*range).min as libc::c_int,
                   (*range).max as libc::c_int, 1000 as libc::c_int,
                   2000 as libc::c_int);
    sample = constrain(sample, 750 as libc::c_int, 2250 as libc::c_int);
    return sample as uint16_t;
}
unsafe extern "C" fn readRxChannelsApplyRanges() {
    let mut channel: libc::c_int = 0 as libc::c_int;
    while channel < rxChannelCount as libc::c_int {
        let rawChannel: uint8_t =
            if channel < 8 as libc::c_int {
                (*rxConfig()).rcmap[channel as usize] as libc::c_int
            } else { channel } as uint8_t;
        // sample the channel
        let mut sample: uint16_t =
            rxRuntimeConfig.rcReadRawFn.expect("non-null function pointer")(&mut rxRuntimeConfig,
                                                                            rawChannel);
        // apply the rx calibration
        if channel < 4 as libc::c_int {
            sample =
                applyRxChannelRangeConfiguraton(sample as libc::c_int,
                                                rxChannelRangeConfigs(channel))
        } // after that apply rxfail value
        rcRaw[channel as usize] = sample as int16_t;
        channel += 1
    };
}
unsafe extern "C" fn detectAndApplySignalLossBehaviour() {
    let currentTimeMs: uint32_t = millis();
    let useValueFromRx: bool =
        rxSignalReceived as libc::c_int != 0 && !rxIsInFailsafeMode;
    if debugMode as libc::c_int == DEBUG_RX_SIGNAL_LOSS as libc::c_int {
        debug[0 as libc::c_int as usize] = rxSignalReceived as int16_t
    }
    if debugMode as libc::c_int == DEBUG_RX_SIGNAL_LOSS as libc::c_int {
        debug[1 as libc::c_int as usize] = rxIsInFailsafeMode as int16_t
    }
    rxFlightChannelsValid = 1 as libc::c_int != 0;
    let mut current_block_22: u64;
    let mut channel: libc::c_int = 0 as libc::c_int;
    while channel < rxChannelCount as libc::c_int {
        let mut sample: uint16_t = rcRaw[channel as usize] as uint16_t;
        let validPulse: bool =
            useValueFromRx as libc::c_int != 0 &&
                isPulseValid(sample) as libc::c_int != 0;
        if validPulse {
            rcInvalidPulsPeriod[channel as usize] =
                currentTimeMs.wrapping_add(300 as libc::c_int as
                                               libc::c_uint);
            current_block_22 = 10652014663920648156;
        } else if cmp32(currentTimeMs, rcInvalidPulsPeriod[channel as usize])
                      < 0 as libc::c_int {
            current_block_22 = 7651349459974463963;
        } else {
            sample = getRxfailValue(channel as uint8_t);
            if channel < 4 as libc::c_int {
                rxFlightChannelsValid = 0 as libc::c_int != 0
            }
            current_block_22 = 10652014663920648156;
        }
        match current_block_22 {
            10652014663920648156 => {
                if feature((FEATURE_RX_PARALLEL_PWM as libc::c_int |
                                FEATURE_RX_PPM as libc::c_int) as uint32_t) {
                    // smooth output for PWM and PPM
                    rcData[channel as usize] =
                        calculateChannelMovingAverage(channel as uint8_t,
                                                      sample) as int16_t
                } else { rcData[channel as usize] = sample as int16_t }
            }
            _ => { }
        }
        channel += 1
        // skip to next channel to hold channel value MAX_INVALID_PULS_TIME
    }
    if rxFlightChannelsValid as libc::c_int != 0 &&
           !IS_RC_MODE_ACTIVE(BOXFAILSAFE) {
        failsafeOnValidDataReceived();
    } else {
        rxIsInFailsafeMode = 1 as libc::c_int != 0;
        failsafeOnValidDataFailed();
        let mut channel_0: libc::c_int = 0 as libc::c_int;
        while channel_0 < rxChannelCount as libc::c_int {
            rcData[channel_0 as usize] =
                getRxfailValue(channel_0 as uint8_t) as int16_t;
            channel_0 += 1
        }
    }
    if debugMode as libc::c_int == DEBUG_RX_SIGNAL_LOSS as libc::c_int {
        debug[3 as libc::c_int as usize] =
            rcData[THROTTLE as libc::c_int as usize]
    };
}
#[no_mangle]
pub unsafe extern "C" fn calculateRxChannelsAndUpdateFailsafe(mut currentTimeUs:
                                                                  timeUs_t)
 -> bool {
    if auxiliaryProcessingRequired {
        auxiliaryProcessingRequired =
            !rxRuntimeConfig.rcProcessFrameFn.expect("non-null function pointer")(&mut rxRuntimeConfig)
    }
    if !rxDataProcessingRequired { return 0 as libc::c_int != 0 }
    rxDataProcessingRequired = 0 as libc::c_int != 0;
    rxNextUpdateAtUs =
        currentTimeUs.wrapping_add((1000000 as libc::c_int /
                                        33 as libc::c_int) as libc::c_uint);
    // only proceed when no more samples to skip and suspend period is over
    if skipRxSamples != 0 {
        if currentTimeUs > suspendRxSignalUntil {
            skipRxSamples = skipRxSamples.wrapping_sub(1)
        }
        return 1 as libc::c_int != 0
    }
    readRxChannelsApplyRanges();
    detectAndApplySignalLossBehaviour();
    rcSampleIndex = rcSampleIndex.wrapping_add(1);
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn parseRcChannels(mut input: *const libc::c_char,
                                         mut rxConfig_0: *mut rxConfig_t) {
    let mut c: *const libc::c_char = input;
    while *c != 0 {
        let mut s: *const libc::c_char =
            strchr(rcChannelLetters.as_ptr(), *c as libc::c_int);
        if !s.is_null() &&
               s < rcChannelLetters.as_ptr().offset(8 as libc::c_int as isize)
           {
            (*rxConfig_0).rcmap[s.wrapping_offset_from(rcChannelLetters.as_ptr())
                                    as libc::c_long as usize] =
                c.wrapping_offset_from(input) as libc::c_long as uint8_t
        }
        c = c.offset(1)
    };
}
#[no_mangle]
pub unsafe extern "C" fn setRssiDirect(mut newRssi: uint16_t,
                                       mut source: rssiSource_e) {
    if source as libc::c_uint != rssiSource as libc::c_uint { return }
    rssi = newRssi;
}
#[no_mangle]
pub unsafe extern "C" fn setRssi(mut rssiValue: uint16_t,
                                 mut source: rssiSource_e) {
    if source as libc::c_uint != rssiSource as libc::c_uint { return }
    static mut rssiSamples: [uint16_t; 16] = [0; 16];
    static mut rssiSampleIndex: uint8_t = 0 as libc::c_int as uint8_t;
    static mut sum: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    sum = sum.wrapping_add(rssiValue as libc::c_uint);
    sum =
        sum.wrapping_sub(rssiSamples[rssiSampleIndex as usize] as
                             libc::c_uint);
    rssiSamples[rssiSampleIndex as usize] = rssiValue;
    rssiSampleIndex =
        ((rssiSampleIndex as libc::c_int + 1 as libc::c_int) %
             16 as libc::c_int) as uint8_t;
    let mut rssiMean: int16_t =
        sum.wrapping_div(16 as libc::c_int as libc::c_uint) as int16_t;
    rssi = rssiMean as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn setRssiMsp(mut newMspRssi: uint8_t) {
    if rssiSource as libc::c_uint ==
           RSSI_SOURCE_NONE as libc::c_int as libc::c_uint {
        rssiSource = RSSI_SOURCE_MSP
    }
    if rssiSource as libc::c_uint ==
           RSSI_SOURCE_MSP as libc::c_int as libc::c_uint {
        rssi =
            ((newMspRssi as uint16_t as libc::c_int) << 2 as libc::c_int) as
                uint16_t;
        lastMspRssiUpdateUs = micros()
    };
}
unsafe extern "C" fn updateRSSIPWM() {
    // Read value of AUX channel as rssi
    let mut pwmRssi: int16_t =
        rcData[((*rxConfig()).rssi_channel as libc::c_int - 1 as libc::c_int)
                   as usize];
    // RSSI_Invert option
    if (*rxConfig()).rssi_invert != 0 {
        pwmRssi =
            (2000 as libc::c_int - pwmRssi as libc::c_int +
                 1000 as libc::c_int) as int16_t
    }
    // Range of rawPwmRssi is [1000;2000]. rssi should be in [0;1023];
    setRssiDirect(constrain(((pwmRssi as libc::c_int - 1000 as libc::c_int) as
                                 libc::c_float / 1000.0f32 *
                                 1023 as libc::c_int as libc::c_float) as
                                libc::c_int, 0 as libc::c_int,
                            1023 as libc::c_int) as uint16_t,
                  RSSI_SOURCE_RX_CHANNEL);
}
unsafe extern "C" fn updateRSSIADC(mut currentTimeUs: timeUs_t) {
    static mut rssiUpdateAt: uint32_t = 0 as libc::c_int as uint32_t;
    if (currentTimeUs.wrapping_sub(rssiUpdateAt) as int32_t) <
           0 as libc::c_int {
        return
    }
    rssiUpdateAt =
        currentTimeUs.wrapping_add((1000000 as libc::c_int /
                                        50 as libc::c_int) as libc::c_uint);
    let adcRssiSample: uint16_t =
        adcGetChannel(ADC_RSSI as libc::c_int as uint8_t);
    let mut rssiValue: uint16_t =
        (adcRssiSample as libc::c_int /
             (4096 as libc::c_int / 1024 as libc::c_int)) as uint16_t;
    // RSSI_Invert option
    if (*rxConfig()).rssi_invert != 0 {
        rssiValue =
            (1023 as libc::c_int - rssiValue as libc::c_int) as uint16_t
    }
    setRssi(rssiValue, RSSI_SOURCE_ADC);
}
#[no_mangle]
pub unsafe extern "C" fn updateRSSI(mut currentTimeUs: timeUs_t) {
    match rssiSource as libc::c_uint {
        2 => { updateRSSIPWM(); }
        1 => { updateRSSIADC(currentTimeUs); }
        4 => {
            if cmpTimeUs(micros(), lastMspRssiUpdateUs) >
                   1500000 as libc::c_int {
                rssi = 0 as libc::c_int as uint16_t
            }
        }
        _ => { }
    };
}
#[no_mangle]
pub unsafe extern "C" fn getRssi() -> uint16_t {
    return ((*rxConfig()).rssi_scale as libc::c_int as libc::c_float /
                100.0f32 * rssi as libc::c_int as libc::c_float +
                (*rxConfig()).rssi_offset as libc::c_int as libc::c_float *
                    (1024 as libc::c_int as libc::c_float / 100.0f32)) as
               uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn getRssiPercent() -> uint8_t {
    return scaleRange(getRssi() as libc::c_int, 0 as libc::c_int,
                      1023 as libc::c_int, 0 as libc::c_int,
                      100 as libc::c_int) as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn rxGetRefreshRate() -> uint16_t {
    return rxRuntimeConfig.rxRefreshRate;
}
unsafe extern "C" fn run_static_initializers() {
    rxChannelRangeConfigs_Registry =
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (44 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 ((::core::mem::size_of::<rxChannelRangeConfig_t>()
                                       as
                                       libc::c_ulong).wrapping_mul(4 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_ulong)
                                      |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &mut rxChannelRangeConfigs_SystemArray as
                                     *mut [rxChannelRangeConfig_t; 4] as
                                     *mut uint8_t,
                             copy:
                                 &mut rxChannelRangeConfigs_CopyArray as
                                     *mut [rxChannelRangeConfig_t; 4] as
                                     *mut uint8_t,
                             ptr: 0 as *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut rxChannelRangeConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_rxChannelRangeConfigs
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut rxChannelRangeConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        };
    rxFailsafeChannelConfigs_Registry =
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (43 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 ((::core::mem::size_of::<rxFailsafeChannelConfig_t>()
                                       as
                                       libc::c_ulong).wrapping_mul(18 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_ulong)
                                      |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &mut rxFailsafeChannelConfigs_SystemArray as
                                     *mut [rxFailsafeChannelConfig_t; 18] as
                                     *mut uint8_t,
                             copy:
                                 &mut rxFailsafeChannelConfigs_CopyArray as
                                     *mut [rxFailsafeChannelConfig_t; 18] as
                                     *mut uint8_t,
                             ptr: 0 as *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut rxFailsafeChannelConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_rxFailsafeChannelConfigs
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut rxFailsafeChannelConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
