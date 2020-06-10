use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
pub type C2RustUnnamed_1 = libc::c_uint;
pub const BEEPER_ALL: C2RustUnnamed_1 = 24;
pub const BEEPER_RC_SMOOTHING_INIT_FAIL: C2RustUnnamed_1 = 23;
pub const BEEPER_CAM_CONNECTION_CLOSE: C2RustUnnamed_1 = 22;
pub const BEEPER_CAM_CONNECTION_OPEN: C2RustUnnamed_1 = 21;
pub const BEEPER_CRASH_FLIP_MODE: C2RustUnnamed_1 = 20;
pub const BEEPER_BLACKBOX_ERASE: C2RustUnnamed_1 = 19;
pub const BEEPER_USB: C2RustUnnamed_1 = 18;
pub const BEEPER_SYSTEM_INIT: C2RustUnnamed_1 = 17;
pub const BEEPER_ARMED: C2RustUnnamed_1 = 16;
pub const BEEPER_DISARM_REPEAT: C2RustUnnamed_1 = 15;
pub const BEEPER_MULTI_BEEPS: C2RustUnnamed_1 = 14;
pub const BEEPER_READY_BEEP: C2RustUnnamed_1 = 13;
pub const BEEPER_ACC_CALIBRATION_FAIL: C2RustUnnamed_1 = 12;
pub const BEEPER_ACC_CALIBRATION: C2RustUnnamed_1 = 11;
pub const BEEPER_RX_SET: C2RustUnnamed_1 = 10;
pub const BEEPER_GPS_STATUS: C2RustUnnamed_1 = 9;
pub const BEEPER_BAT_LOW: C2RustUnnamed_1 = 8;
pub const BEEPER_BAT_CRIT_LOW: C2RustUnnamed_1 = 7;
pub const BEEPER_ARMING_GPS_FIX: C2RustUnnamed_1 = 6;
pub const BEEPER_ARMING: C2RustUnnamed_1 = 5;
pub const BEEPER_DISARMING: C2RustUnnamed_1 = 4;
pub const BEEPER_RX_LOST_LANDING: C2RustUnnamed_1 = 3;
pub const BEEPER_RX_LOST: C2RustUnnamed_1 = 2;
pub const BEEPER_GYRO_CALIBRATED: C2RustUnnamed_1 = 1;
pub const BEEPER_SILENCE: C2RustUnnamed_1 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct beeperConfig_s {
    pub beeper_off_flags: uint32_t,
    pub dshotBeaconTone: uint8_t,
    pub dshotBeaconOffFlags: uint32_t,
}
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
pub type beeperConfig_t = beeperConfig_s;
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
pub static mut beeperConfig_System: beeperConfig_t =
    beeperConfig_t{beeper_off_flags: 0,
                   dshotBeaconTone: 0,
                   dshotBeaconOffFlags: 0,};
#[no_mangle]
pub static mut beeperConfig_Copy: beeperConfig_t =
    beeperConfig_t{beeper_off_flags: 0,
                   dshotBeaconTone: 0,
                   dshotBeaconOffFlags: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut beeperConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (502 as libc::c_int |
                                      (2 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<beeperConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &beeperConfig_System as *const beeperConfig_t
                                     as *mut beeperConfig_t as *mut uint8_t,
                             copy:
                                 &beeperConfig_Copy as *const beeperConfig_t
                                     as *mut beeperConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     &pgResetTemplate_beeperConfig
                                                         as
                                                         *const beeperConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_beeperConfig: beeperConfig_t =
    {
        let mut init =
            beeperConfig_s{beeper_off_flags: 0,
                           dshotBeaconTone: 1 as libc::c_int as uint8_t,
                           dshotBeaconOffFlags:
                               ((1 as libc::c_int) <<
                                    BEEPER_RX_LOST as libc::c_int -
                                        1 as libc::c_int |
                                    (1 as libc::c_int) <<
                                        BEEPER_RX_SET as libc::c_int -
                                            1 as libc::c_int) as uint32_t,};
        init
    };
