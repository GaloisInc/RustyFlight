use ::libc;
extern "C" {
    #[no_mangle]
    fn parseRcChannels(input: *const libc::c_char, rxConfig: *mut rxConfig_s);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
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
pub type C2RustUnnamed = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed = 4095;
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
    pub reset: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
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
pub type C2RustUnnamed_1 = libc::c_uint;
pub const INTERPOLATION_CHANNELS_RPT: C2RustUnnamed_1 = 4;
pub const INTERPOLATION_CHANNELS_T: C2RustUnnamed_1 = 3;
pub const INTERPOLATION_CHANNELS_RPYT: C2RustUnnamed_1 = 2;
pub const INTERPOLATION_CHANNELS_RPY: C2RustUnnamed_1 = 1;
pub const INTERPOLATION_CHANNELS_RP: C2RustUnnamed_1 = 0;
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
pub type C2RustUnnamed_2 = libc::c_uint;
pub const RC_SMOOTHING_MANUAL: C2RustUnnamed_2 = 3;
pub const RC_SMOOTHING_AUTO: C2RustUnnamed_2 = 2;
pub const RC_SMOOTHING_DEFAULT: C2RustUnnamed_2 = 1;
pub const RC_SMOOTHING_OFF: C2RustUnnamed_2 = 0;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const RC_SMOOTHING_TYPE_FILTER: C2RustUnnamed_3 = 1;
pub const RC_SMOOTHING_TYPE_INTERPOLATION: C2RustUnnamed_3 = 0;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const RC_SMOOTHING_INPUT_BIQUAD: C2RustUnnamed_4 = 1;
pub const RC_SMOOTHING_INPUT_PT1: C2RustUnnamed_4 = 0;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const RC_SMOOTHING_DERIVATIVE_BIQUAD: C2RustUnnamed_5 = 2;
pub const RC_SMOOTHING_DERIVATIVE_PT1: C2RustUnnamed_5 = 1;
pub const RC_SMOOTHING_DERIVATIVE_OFF: C2RustUnnamed_5 = 0;
pub type C2RustUnnamed_6 = libc::c_uint;
pub const SERIALRX_FPORT: C2RustUnnamed_6 = 12;
pub const SERIALRX_TARGET_CUSTOM: C2RustUnnamed_6 = 11;
pub const SERIALRX_SRXL: C2RustUnnamed_6 = 10;
pub const SERIALRX_CRSF: C2RustUnnamed_6 = 9;
pub const SERIALRX_JETIEXBUS: C2RustUnnamed_6 = 8;
pub const SERIALRX_IBUS: C2RustUnnamed_6 = 7;
pub const SERIALRX_XBUS_MODE_B_RJ01: C2RustUnnamed_6 = 6;
pub const SERIALRX_XBUS_MODE_B: C2RustUnnamed_6 = 5;
pub const SERIALRX_SUMH: C2RustUnnamed_6 = 4;
pub const SERIALRX_SUMD: C2RustUnnamed_6 = 3;
pub const SERIALRX_SBUS: C2RustUnnamed_6 = 2;
pub const SERIALRX_SPEKTRUM2048: C2RustUnnamed_6 = 1;
pub const SERIALRX_SPEKTRUM1024: C2RustUnnamed_6 = 0;
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
pub static mut rxConfig_System: rxConfig_t =
    rxConfig_t{rcmap: [0; 8],
               serialrx_provider: 0,
               serialrx_inverted: 0,
               halfDuplex: 0,
               spektrum_bind_pin_override_ioTag: 0,
               spektrum_bind_plug_ioTag: 0,
               spektrum_sat_bind: 0,
               spektrum_sat_bind_autoreset: 0,
               rssi_channel: 0,
               rssi_scale: 0,
               rssi_invert: 0,
               midrc: 0,
               mincheck: 0,
               maxcheck: 0,
               rcInterpolation: 0,
               rcInterpolationChannels: 0,
               rcInterpolationInterval: 0,
               fpvCamAngleDegrees: 0,
               airModeActivateThreshold: 0,
               rx_min_usec: 0,
               rx_max_usec: 0,
               max_aux_channel: 0,
               rssi_src_frame_errors: 0,
               rssi_offset: 0,
               rc_smoothing_type: 0,
               rc_smoothing_input_cutoff: 0,
               rc_smoothing_derivative_cutoff: 0,
               rc_smoothing_debug_axis: 0,
               rc_smoothing_input_type: 0,
               rc_smoothing_derivative_type: 0,};
#[no_mangle]
pub static mut rxConfig_Copy: rxConfig_t =
    rxConfig_t{rcmap: [0; 8],
               serialrx_provider: 0,
               serialrx_inverted: 0,
               halfDuplex: 0,
               spektrum_bind_pin_override_ioTag: 0,
               spektrum_bind_plug_ioTag: 0,
               spektrum_sat_bind: 0,
               spektrum_sat_bind_autoreset: 0,
               rssi_channel: 0,
               rssi_scale: 0,
               rssi_invert: 0,
               midrc: 0,
               mincheck: 0,
               maxcheck: 0,
               rcInterpolation: 0,
               rcInterpolationChannels: 0,
               rcInterpolationInterval: 0,
               fpvCamAngleDegrees: 0,
               airModeActivateThreshold: 0,
               rx_min_usec: 0,
               rx_max_usec: 0,
               max_aux_channel: 0,
               rssi_src_frame_errors: 0,
               rssi_offset: 0,
               rc_smoothing_type: 0,
               rc_smoothing_input_cutoff: 0,
               rc_smoothing_derivative_cutoff: 0,
               rc_smoothing_debug_axis: 0,
               rc_smoothing_input_type: 0,
               rc_smoothing_derivative_type: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut rxConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (24 as libc::c_int |
                                      (2 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<rxConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &rxConfig_System as *const rxConfig_t as
                                     *mut rxConfig_t as *mut uint8_t,
                             copy:
                                 &rxConfig_Copy as *const rxConfig_t as
                                     *mut rxConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut rxConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_rxConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut rxConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_rxConfig(mut rxConfig: *mut rxConfig_t) {
    *rxConfig =
        {
            let mut init =
                rxConfig_s{rcmap: [0; 8],
                           serialrx_provider:
                               SERIALRX_SBUS as libc::c_int as uint8_t,
                           serialrx_inverted: 0 as libc::c_int as uint8_t,
                           halfDuplex: 0 as libc::c_int as uint8_t,
                           spektrum_bind_pin_override_ioTag:
                               0 as libc::c_int as ioTag_t,
                           spektrum_bind_plug_ioTag:
                               0 as libc::c_int as ioTag_t,
                           spektrum_sat_bind: 0 as libc::c_int as uint8_t,
                           spektrum_sat_bind_autoreset:
                               1 as libc::c_int as uint8_t,
                           rssi_channel: 0 as libc::c_int as uint8_t,
                           rssi_scale: 100 as libc::c_int as uint8_t,
                           rssi_invert: 0 as libc::c_int as uint8_t,
                           midrc: 1500 as libc::c_int as uint16_t,
                           mincheck: 1050 as libc::c_int as uint16_t,
                           maxcheck: 1900 as libc::c_int as uint16_t,
                           rcInterpolation:
                               RC_SMOOTHING_AUTO as libc::c_int as uint8_t,
                           rcInterpolationChannels:
                               INTERPOLATION_CHANNELS_RPYT as libc::c_int as
                                   uint8_t,
                           rcInterpolationInterval:
                               19 as libc::c_int as uint8_t,
                           fpvCamAngleDegrees: 0 as libc::c_int as uint8_t,
                           airModeActivateThreshold:
                               32 as libc::c_int as uint8_t,
                           rx_min_usec: 885 as libc::c_int as uint16_t,
                           rx_max_usec: 2115 as libc::c_int as uint16_t,
                           max_aux_channel: 6 as libc::c_int as uint8_t,
                           rssi_src_frame_errors: 0 as libc::c_int as uint8_t,
                           rssi_offset: 0 as libc::c_int as int8_t,
                           rc_smoothing_type:
                               RC_SMOOTHING_TYPE_INTERPOLATION as libc::c_int
                                   as uint8_t,
                           rc_smoothing_input_cutoff:
                               0 as libc::c_int as uint8_t,
                           rc_smoothing_derivative_cutoff:
                               0 as libc::c_int as uint8_t,
                           rc_smoothing_debug_axis:
                               ROLL as libc::c_int as uint8_t,
                           rc_smoothing_input_type:
                               RC_SMOOTHING_INPUT_BIQUAD as libc::c_int as
                                   uint8_t,
                           rc_smoothing_derivative_type:
                               RC_SMOOTHING_DERIVATIVE_BIQUAD as libc::c_int
                                   as uint8_t,};
            init
        };
    parseRcChannels(b"AETR1234\x00" as *const u8 as *const libc::c_char,
                    rxConfig);
}
