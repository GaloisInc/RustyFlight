use core;
use libc;
extern "C" {
    #[no_mangle]
    fn degreesToRadians(degrees: int16_t) -> libc::c_float;
    #[no_mangle]
    fn buildRotationMatrix(delta: *mut fp_angles_t,
                           matrix: *mut [libc::c_float; 3]);
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct fp_angles {
    pub roll: libc::c_float,
    pub pitch: libc::c_float,
    pub yaw: libc::c_float,
}
// Floating point Euler angles.
// Be carefull, could be either of degrees or radians.
pub type fp_angles_def = fp_angles;
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union fp_angles_t {
    pub raw: [libc::c_float; 3],
    pub angles: fp_angles_def,
}
pub type C2RustUnnamed = libc::c_uint;
pub const Z: C2RustUnnamed = 2;
pub const Y: C2RustUnnamed = 1;
pub const X: C2RustUnnamed = 0;
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_1,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed_1 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<unsafe extern "C" fn(_: *mut libc::c_void,
                                          _: libc::c_int) -> ()>,
}
pub type pgRegistry_t = pgRegistry_s;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const CW270_DEG_FLIP: C2RustUnnamed_2 = 8;
pub const CW180_DEG_FLIP: C2RustUnnamed_2 = 7;
pub const CW90_DEG_FLIP: C2RustUnnamed_2 = 6;
pub const CW0_DEG_FLIP: C2RustUnnamed_2 = 5;
pub const CW270_DEG: C2RustUnnamed_2 = 4;
pub const CW180_DEG: C2RustUnnamed_2 = 3;
pub const CW90_DEG: C2RustUnnamed_2 = 2;
pub const CW0_DEG: C2RustUnnamed_2 = 1;
pub const ALIGN_DEFAULT: C2RustUnnamed_2 = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct boardAlignment_s {
    pub rollDegrees: int32_t,
    pub pitchDegrees: int32_t,
    pub yawDegrees: int32_t,
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
pub type boardAlignment_t = boardAlignment_s;
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
static mut standardBoardAlignment: bool = 1i32 != 0;
// board orientation correction
static mut boardRotation: [[libc::c_float; 3]; 3] = [[0.; 3]; 3];
// matrix
// no template required since defaults are zero
#[no_mangle]
pub static mut boardAlignment_System: boardAlignment_t =
    boardAlignment_t{rollDegrees: 0, pitchDegrees: 0, yawDegrees: 0,};
#[no_mangle]
pub static mut boardAlignment_Copy: boardAlignment_t =
    boardAlignment_t{rollDegrees: 0, pitchDegrees: 0, yawDegrees: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut boardAlignment_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn: (2i32 | 0i32 << 12i32) as pgn_t,
                             size:
                                 (::core::mem::size_of::<boardAlignment_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &boardAlignment_System as
                                     *const boardAlignment_t as
                                     *mut boardAlignment_t as *mut uint8_t,
                             copy:
                                 &boardAlignment_Copy as
                                     *const boardAlignment_t as
                                     *mut boardAlignment_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{ptr:
                                                     0 as *const libc::c_void
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
unsafe extern "C" fn isBoardAlignmentStandard(mut boardAlignment:
                                                  *const boardAlignment_t)
 -> bool {
    return (*boardAlignment).rollDegrees == 0 &&
               (*boardAlignment).pitchDegrees == 0 &&
               (*boardAlignment).yawDegrees == 0;
}
#[no_mangle]
pub unsafe extern "C" fn initBoardAlignment(mut boardAlignment:
                                                *const boardAlignment_t) {
    if isBoardAlignmentStandard(boardAlignment) { return }
    standardBoardAlignment = 0i32 != 0;
    let mut rotationAngles: fp_angles_t = fp_angles_t{raw: [0.; 3],};
    rotationAngles.angles.roll =
        degreesToRadians((*boardAlignment).rollDegrees as int16_t);
    rotationAngles.angles.pitch =
        degreesToRadians((*boardAlignment).pitchDegrees as int16_t);
    rotationAngles.angles.yaw =
        degreesToRadians((*boardAlignment).yawDegrees as int16_t);
    buildRotationMatrix(&mut rotationAngles, boardRotation.as_mut_ptr());
}
unsafe extern "C" fn alignBoard(mut vec: *mut libc::c_float) {
    let mut x: libc::c_float = *vec.offset(X as libc::c_int as isize);
    let mut y: libc::c_float = *vec.offset(Y as libc::c_int as isize);
    let mut z: libc::c_float = *vec.offset(Z as libc::c_int as isize);
    *vec.offset(X as libc::c_int as isize) =
        boardRotation[0][X as libc::c_int as usize] * x +
            boardRotation[1][X as libc::c_int as usize] * y +
            boardRotation[2][X as libc::c_int as usize] * z;
    *vec.offset(Y as libc::c_int as isize) =
        boardRotation[0][Y as libc::c_int as usize] * x +
            boardRotation[1][Y as libc::c_int as usize] * y +
            boardRotation[2][Y as libc::c_int as usize] * z;
    *vec.offset(Z as libc::c_int as isize) =
        boardRotation[0][Z as libc::c_int as usize] * x +
            boardRotation[1][Z as libc::c_int as usize] * y +
            boardRotation[2][Z as libc::c_int as usize] * z;
}
#[no_mangle]
pub unsafe extern "C" fn alignSensors(mut dest: *mut libc::c_float,
                                      mut rotation: uint8_t) {
    let x: libc::c_float = *dest.offset(X as libc::c_int as isize);
    let y: libc::c_float = *dest.offset(Y as libc::c_int as isize);
    let z: libc::c_float = *dest.offset(Z as libc::c_int as isize);
    match rotation as libc::c_int {
        2 => {
            *dest.offset(X as libc::c_int as isize) = y;
            *dest.offset(Y as libc::c_int as isize) = -x;
            *dest.offset(Z as libc::c_int as isize) = z
        }
        3 => {
            *dest.offset(X as libc::c_int as isize) = -x;
            *dest.offset(Y as libc::c_int as isize) = -y;
            *dest.offset(Z as libc::c_int as isize) = z
        }
        4 => {
            *dest.offset(X as libc::c_int as isize) = -y;
            *dest.offset(Y as libc::c_int as isize) = x;
            *dest.offset(Z as libc::c_int as isize) = z
        }
        5 => {
            *dest.offset(X as libc::c_int as isize) = -x;
            *dest.offset(Y as libc::c_int as isize) = y;
            *dest.offset(Z as libc::c_int as isize) = -z
        }
        6 => {
            *dest.offset(X as libc::c_int as isize) = y;
            *dest.offset(Y as libc::c_int as isize) = x;
            *dest.offset(Z as libc::c_int as isize) = -z
        }
        7 => {
            *dest.offset(X as libc::c_int as isize) = x;
            *dest.offset(Y as libc::c_int as isize) = -y;
            *dest.offset(Z as libc::c_int as isize) = -z
        }
        8 => {
            *dest.offset(X as libc::c_int as isize) = -y;
            *dest.offset(Y as libc::c_int as isize) = -x;
            *dest.offset(Z as libc::c_int as isize) = -z
        }
        1 | _ => {
            *dest.offset(X as libc::c_int as isize) = x;
            *dest.offset(Y as libc::c_int as isize) = y;
            *dest.offset(Z as libc::c_int as isize) = z
        }
    }
    if !standardBoardAlignment { alignBoard(dest); };
}
