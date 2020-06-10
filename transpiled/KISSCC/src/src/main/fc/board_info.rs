use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn strncpy(_: *mut libc::c_char, _: *const libc::c_char, _: libc::c_ulong)
     -> *mut libc::c_char;
    #[no_mangle]
    static mut boardConfig_System: boardConfig_t;
}
pub type __uint8_t = libc::c_uchar;
pub type uint8_t = __uint8_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct boardConfig_s {
    pub signature: [uint8_t; 32],
    pub manufacturerId: [libc::c_char; 5],
    pub boardName: [libc::c_char; 21],
    pub boardInformationSet: uint8_t,
    pub signatureSet: uint8_t,
}
// Warning: This configuration is meant to be applied when loading the initial
// configuration for a generic board, and stay fixed after this, to enable
// identification of the hardware that this is running on.
// Do not modify this parameter group directly, use 'fc/board_info.h' instead.
pub type boardConfig_t = boardConfig_s;
#[inline]
unsafe extern "C" fn boardConfig() -> *const boardConfig_t {
    return &mut boardConfig_System;
}
#[inline]
unsafe extern "C" fn boardConfigMutable() -> *mut boardConfig_t {
    return &mut boardConfig_System;
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
static mut boardInformationSet: bool = 0 as libc::c_int != 0;
static mut manufacturerId: [libc::c_char; 5] = [0; 5];
static mut boardName: [libc::c_char; 21] = [0; 21];
static mut signatureSet: bool = 0 as libc::c_int != 0;
static mut signature: [uint8_t; 32] = [0; 32];
#[no_mangle]
pub unsafe extern "C" fn initBoardInformation() {
    boardInformationSet = (*boardConfig()).boardInformationSet != 0;
    if boardInformationSet {
        strncpy(manufacturerId.as_mut_ptr(),
                (*boardConfig()).manufacturerId.as_ptr(),
                4 as libc::c_int as libc::c_ulong);
        strncpy(boardName.as_mut_ptr(), (*boardConfig()).boardName.as_ptr(),
                20 as libc::c_int as libc::c_ulong);
    }
    signatureSet = (*boardConfig()).signatureSet != 0;
    if signatureSet {
        memcpy(signature.as_mut_ptr() as *mut libc::c_void,
               (*boardConfig()).signature.as_ptr() as *const libc::c_void,
               32 as libc::c_int as libc::c_ulong);
    };
}
#[no_mangle]
pub unsafe extern "C" fn getManufacturerId() -> *const libc::c_char {
    return manufacturerId.as_mut_ptr();
}
#[no_mangle]
pub unsafe extern "C" fn getBoardName() -> *const libc::c_char {
    return boardName.as_mut_ptr();
}
#[no_mangle]
pub unsafe extern "C" fn boardInformationIsSet() -> bool {
    return boardInformationSet;
}
#[no_mangle]
pub unsafe extern "C" fn setManufacturerId(mut newManufacturerId:
                                               *const libc::c_char) -> bool {
    if !boardInformationSet {
        strncpy(manufacturerId.as_mut_ptr(), newManufacturerId,
                4 as libc::c_int as libc::c_ulong);
        return 1 as libc::c_int != 0
    } else { return 0 as libc::c_int != 0 };
}
#[no_mangle]
pub unsafe extern "C" fn setBoardName(mut newBoardName: *const libc::c_char)
 -> bool {
    if !boardInformationSet {
        strncpy(boardName.as_mut_ptr(), newBoardName,
                20 as libc::c_int as libc::c_ulong);
        return 1 as libc::c_int != 0
    } else { return 0 as libc::c_int != 0 };
}
#[no_mangle]
pub unsafe extern "C" fn persistBoardInformation() -> bool {
    if !boardInformationSet {
        strncpy((*boardConfigMutable()).manufacturerId.as_mut_ptr(),
                manufacturerId.as_mut_ptr(),
                4 as libc::c_int as libc::c_ulong);
        strncpy((*boardConfigMutable()).boardName.as_mut_ptr(),
                boardName.as_mut_ptr(), 20 as libc::c_int as libc::c_ulong);
        (*boardConfigMutable()).boardInformationSet =
            1 as libc::c_int as uint8_t;
        initBoardInformation();
        return 1 as libc::c_int != 0
    } else { return 0 as libc::c_int != 0 };
}
// USE_BOARD_INFO
