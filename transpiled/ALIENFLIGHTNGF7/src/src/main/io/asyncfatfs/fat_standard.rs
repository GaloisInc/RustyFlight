use ::libc;
extern "C" {
    #[no_mangle]
    fn toupper(_: libc::c_int) -> libc::c_int;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct fatDirectoryEntry_t {
    pub filename: [libc::c_char; 11],
    pub attrib: uint8_t,
    pub ntReserved: uint8_t,
    pub creationTimeTenths: uint8_t,
    pub creationTime: uint16_t,
    pub creationDate: uint16_t,
    pub lastAccessDate: uint16_t,
    pub firstClusterHigh: uint16_t,
    pub lastWriteTime: uint16_t,
    pub lastWriteDate: uint16_t,
    pub firstClusterLow: uint16_t,
    pub fileSize: uint32_t,
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
#[no_mangle]
pub unsafe extern "C" fn fat16_isEndOfChainMarker(mut clusterNumber: uint16_t)
 -> bool {
    return clusterNumber as libc::c_int >= 0xfff8 as libc::c_int;
}
// Pass the cluster number after fat32_decodeClusterNumber().
#[no_mangle]
pub unsafe extern "C" fn fat32_isEndOfChainMarker(mut clusterNumber: uint32_t)
 -> bool {
    return clusterNumber >= 0xffffff8 as libc::c_int as libc::c_uint;
}
/* *
 * FAT32 cluster numbers are really only 28 bits, and the top 4 bits must be left alone and not treated as part of the
 * cluster number (so various FAT drivers can use those bits for their own purposes, or they can be used in later
 * extensions)
 */
#[no_mangle]
pub unsafe extern "C" fn fat32_decodeClusterNumber(mut clusterNumber:
                                                       uint32_t) -> uint32_t {
    return clusterNumber & 0xfffffff as libc::c_int as libc::c_uint;
}
// fat32 needs fat32_decodeClusterNumber() applied first.
#[no_mangle]
pub unsafe extern "C" fn fat_isFreeSpace(mut clusterNumber: uint32_t)
 -> bool {
    return clusterNumber == 0 as libc::c_int as libc::c_uint;
}
#[no_mangle]
pub unsafe extern "C" fn fat_isDirectoryEntryTerminator(mut entry:
                                                            *mut fatDirectoryEntry_t)
 -> bool {
    return (*entry).filename[0 as libc::c_int as usize] as libc::c_int ==
               0 as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn fat_isDirectoryEntryEmpty(mut entry:
                                                       *mut fatDirectoryEntry_t)
 -> bool {
    return (*entry).filename[0 as libc::c_int as usize] as libc::c_uchar as
               libc::c_int == 0xe5 as libc::c_int;
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
// Signature bytes found at index 510 and 511 in the volume ID sector
/* *
 * Convert the given "prefix.ext" style filename to the FAT format to be stored on disk.
 *
 * fatFilename must point to a buffer which is FAT_FILENAME_LENGTH bytes long. The buffer is not null-terminated.
 */
#[no_mangle]
pub unsafe extern "C" fn fat_convertFilenameToFATStyle(mut filename:
                                                           *const libc::c_char,
                                                       mut fatFilename:
                                                           *mut uint8_t) {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 8 as libc::c_int {
        if *filename as libc::c_int == '\u{0}' as i32 ||
               *filename as libc::c_int == '.' as i32 {
            *fatFilename = ' ' as i32 as uint8_t
        } else {
            *fatFilename =
                toupper(*filename as libc::c_uchar as libc::c_int) as uint8_t;
            filename = filename.offset(1)
        }
        fatFilename = fatFilename.offset(1);
        i += 1
    }
    if *filename as libc::c_int == '.' as i32 {
        filename = filename.offset(1)
    }
    let mut i_0: libc::c_int = 0 as libc::c_int;
    while i_0 < 3 as libc::c_int {
        if *filename as libc::c_int == '\u{0}' as i32 {
            *fatFilename = ' ' as i32 as uint8_t
        } else {
            *fatFilename =
                toupper(*filename as libc::c_uchar as libc::c_int) as uint8_t;
            filename = filename.offset(1)
        }
        fatFilename = fatFilename.offset(1);
        i_0 += 1
    };
}
