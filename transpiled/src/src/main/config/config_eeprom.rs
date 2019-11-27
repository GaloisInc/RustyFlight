use core;
use libc;
extern "C" {
    #[no_mangle]
    fn crc16_ccitt_update(crc: uint16_t, data: *const libc::c_void,
                          length: uint32_t) -> uint16_t;
    #[no_mangle]
    fn config_streamer_init(c: *mut config_streamer_t);
    #[no_mangle]
    fn config_streamer_start(c: *mut config_streamer_t, base: uintptr_t,
                             size: libc::c_int);
    #[no_mangle]
    fn config_streamer_write(c: *mut config_streamer_t, p: *const uint8_t,
                             size: uint32_t) -> libc::c_int;
    #[no_mangle]
    fn config_streamer_flush(c: *mut config_streamer_t) -> libc::c_int;
    #[no_mangle]
    fn config_streamer_finish(c: *mut config_streamer_t) -> libc::c_int;
    #[no_mangle]
    static __pg_registry_start: [pgRegistry_t; 0];
    #[no_mangle]
    static __pg_registry_end: [pgRegistry_t; 0];
    #[no_mangle]
    fn pgLoad(reg: *const pgRegistry_t, from: *const libc::c_void,
              size: libc::c_int, version: libc::c_int) -> bool;
    #[no_mangle]
    fn pgReset(reg: *const pgRegistry_t);
    #[no_mangle]
    fn failureMode(mode: failureMode_e);
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
    static mut __config_start: uint8_t;
    // configured via linker script when building binaries.
    #[no_mangle]
    static mut __config_end: uint8_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type uintptr_t = libc::c_ulong;
#[derive ( Copy, Clone )]
#[repr(C, packed)]
pub struct configHeader_t {
    pub eepromConfigVersion: uint8_t,
    pub magic_be: uint8_t,
}
#[derive ( Copy, Clone )]
#[repr(C, packed)]
pub struct configFooter_t {
    pub terminator: uint16_t,
}
#[derive ( Copy, Clone )]
#[repr(C, packed)]
pub struct configRecord_t {
    pub size: uint16_t,
    pub pgn: pgn_t,
    pub version: uint8_t,
    pub flags: uint8_t,
    pub pg: [uint8_t; 0],
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
pub type pgn_t = uint16_t;
pub type pgRegistry_t = pgRegistry_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<unsafe extern "C" fn(_: *mut libc::c_void,
                                          _: libc::c_int) -> ()>,
}
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
// function that resets a single parameter group instance
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
pub type configRecordFlags_e = libc::c_uint;
pub const CR_CLASSICATION_PROFILE_LAST: configRecordFlags_e = 0;
pub const CR_CLASSICATION_SYSTEM: configRecordFlags_e = 0;
pub const PGR_PGN_MASK: C2RustUnnamed_1 = 4095;
pub type failureMode_e = libc::c_uint;
pub const FAILURE_GYRO_INIT_FAILED: failureMode_e = 6;
pub const FAILURE_FLASH_WRITE_FAILED: failureMode_e = 5;
pub const FAILURE_INVALID_EEPROM_CONTENTS: failureMode_e = 4;
pub const FAILURE_ACC_INCOMPATIBLE: failureMode_e = 3;
pub const FAILURE_ACC_INIT: failureMode_e = 2;
pub const FAILURE_MISSING_ACC: failureMode_e = 1;
pub const FAILURE_DEVELOPER: failureMode_e = 0;
/* base */
/* size */
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
// Streams data out to the EEPROM, padding to the write size as
// needed, and updating the checksum as it goes.
pub type config_streamer_t = config_streamer_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct config_streamer_s {
    pub address: uintptr_t,
    pub size: libc::c_int,
    pub buffer: C2RustUnnamed_0,
    pub at: libc::c_int,
    pub err: libc::c_int,
    pub unlocked: bool,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed_0 {
    pub b: [uint8_t; 4],
    pub w: uint32_t,
}
pub const PGR_SIZE_MASK: C2RustUnnamed_1 = 4095;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed_1 = 0;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed_1 = 61440;
#[inline]
unsafe extern "C" fn pgN(mut reg: *const pgRegistry_t) -> uint16_t {
    return ((*reg).pgn as libc::c_int & PGR_PGN_MASK as libc::c_int) as
               uint16_t;
}
#[inline]
unsafe extern "C" fn pgVersion(mut reg: *const pgRegistry_t) -> uint8_t {
    return ((*reg).pgn as libc::c_int >> 12i32) as uint8_t;
}
#[inline]
unsafe extern "C" fn pgSize(mut reg: *const pgRegistry_t) -> uint16_t {
    return ((*reg).size as libc::c_int & PGR_SIZE_MASK as libc::c_int) as
               uint16_t;
}
static mut eepromConfigSize: uint16_t = 0;
#[no_mangle]
pub unsafe extern "C" fn initEEPROM() { }
#[no_mangle]
pub unsafe extern "C" fn isEEPROMVersionValid() -> bool {
    let mut p: *const uint8_t = &mut __config_start;
    let mut header: *const configHeader_t = p as *const configHeader_t;
    if (*header).eepromConfigVersion as libc::c_int != 171i32 {
        return 0i32 != 0
    }
    return 1i32 != 0;
}
// Scan the EEPROM config. Returns true if the config is valid.
#[no_mangle]
pub unsafe extern "C" fn isEEPROMStructureValid() -> bool {
    let mut p: *const uint8_t = &mut __config_start;
    let mut header: *const configHeader_t = p as *const configHeader_t;
    if (*header).magic_be as libc::c_int != 0xbei32 { return 0i32 != 0 }
    let mut crc: uint16_t = 0xffffi32 as uint16_t;
    crc =
        crc16_ccitt_update(crc, header as *const libc::c_void,
                           ::core::mem::size_of::<configHeader_t>() as
                               libc::c_ulong as uint32_t);
    p =
        p.offset(::core::mem::size_of::<configHeader_t>() as libc::c_ulong as
                     isize);
    loop  {
        let mut record: *const configRecord_t = p as *const configRecord_t;
        if (*record).size as libc::c_int == 0i32 { break ; }
        if p.offset((*record).size as libc::c_int as isize) >=
               &mut __config_end as *mut uint8_t as *const uint8_t ||
               ((*record).size as libc::c_ulong) <
                   ::core::mem::size_of::<configRecord_t>() as libc::c_ulong {
            // Too big or too small.
            return 0i32 != 0
        }
        crc =
            crc16_ccitt_update(crc, p as *const libc::c_void,
                               (*record).size as uint32_t);
        p = p.offset((*record).size as libc::c_int as isize)
    }
    let mut footer: *const configFooter_t = p as *const configFooter_t;
    crc =
        crc16_ccitt_update(crc, footer as *const libc::c_void,
                           ::core::mem::size_of::<configFooter_t>() as
                               libc::c_ulong as uint32_t);
    p =
        p.offset(::core::mem::size_of::<configFooter_t>() as libc::c_ulong as
                     isize);
    // include stored CRC in the CRC calculation
    let mut storedCrc: *const uint16_t = p as *const uint16_t;
    crc =
        crc16_ccitt_update(crc, storedCrc as *const libc::c_void,
                           ::core::mem::size_of::<uint16_t>() as libc::c_ulong
                               as uint32_t);
    p =
        p.offset(::core::mem::size_of::<*const uint16_t>() as libc::c_ulong as
                     isize);
    eepromConfigSize =
        p.wrapping_offset_from(&mut __config_start) as libc::c_long as
            uint16_t;
    // CRC has the property that if the CRC itself is included in the calculation the resulting CRC will have constant value
    return crc as libc::c_int == 0x1d0fi32;
}
#[no_mangle]
pub unsafe extern "C" fn getEEPROMConfigSize() -> uint16_t {
    return eepromConfigSize;
}
// find config record for reg + classification (profile info) in EEPROM
// return NULL when record is not found
// this function assumes that EEPROM content is valid
unsafe extern "C" fn findEEPROM(mut reg: *const pgRegistry_t,
                                mut classification: configRecordFlags_e)
 -> *const configRecord_t {
    let mut p: *const uint8_t = &mut __config_start; // skip header
    p =
        p.offset(::core::mem::size_of::<configHeader_t>() as libc::c_ulong as
                     isize);
    loop  {
        let mut record: *const configRecord_t = p as *const configRecord_t;
        if (*record).size as libc::c_int == 0i32 ||
               p.offset((*record).size as libc::c_int as isize) >=
                   &mut __config_end as *mut uint8_t as *const uint8_t ||
               ((*record).size as libc::c_ulong) <
                   ::core::mem::size_of::<configRecord_t>() as libc::c_ulong {
            break ;
        }
        if pgN(reg) as libc::c_int == (*record).pgn as libc::c_int &&
               ((*record).flags as libc::c_int & 0x3i32) as libc::c_uint ==
                   classification as libc::c_uint {
            return record
        }
        p = p.offset((*record).size as libc::c_int as isize)
    }
    // record not found
    return 0 as *const configRecord_t;
}
// Initialize all PG records from EEPROM.
// This functions processes all PGs sequentially, scanning EEPROM for each one. This is suboptimal,
//   but each PG is loaded/initialized exactly once and in defined order.
#[no_mangle]
pub unsafe extern "C" fn loadEEPROM() -> bool {
    let mut success: bool = 1i32 != 0;
    let mut reg: *const pgRegistry_t = __pg_registry_start.as_ptr();
    while reg < __pg_registry_end.as_ptr() {
        let mut rec: *const configRecord_t =
            findEEPROM(reg, CR_CLASSICATION_SYSTEM);
        if !rec.is_null() {
            // config from EEPROM is available, use it to initialize PG. pgLoad will handle version mismatch
            if !pgLoad(reg, (*rec).pg.as_ptr() as *const libc::c_void,
                       ((*rec).size as libc::c_ulong).wrapping_sub(6u64) as
                           libc::c_int, (*rec).version as libc::c_int) {
                success = 0i32 != 0
            }
        } else { pgReset(reg); success = 0i32 != 0 }
        reg = reg.offset(1)
    }
    return success;
}
unsafe extern "C" fn writeSettingsToEEPROM() -> bool {
    let mut streamer: config_streamer_t =
        config_streamer_t{address: 0,
                          size: 0,
                          buffer: C2RustUnnamed_0{b: [0; 4],},
                          at: 0,
                          err: 0,
                          unlocked: false,};
    config_streamer_init(&mut streamer);
    config_streamer_start(&mut streamer,
                          &mut __config_start as *mut uint8_t as uintptr_t,
                          (&mut __config_end as
                               *mut uint8_t).wrapping_offset_from(&mut __config_start)
                              as libc::c_long as libc::c_int);
    let mut header: configHeader_t =
        {
            let mut init =
                configHeader_t{eepromConfigVersion: 171i32 as uint8_t,
                               magic_be: 0xbei32 as uint8_t,};
            init
        };
    config_streamer_write(&mut streamer,
                          &mut header as *mut configHeader_t as *mut uint8_t,
                          ::core::mem::size_of::<configHeader_t>() as
                              libc::c_ulong as uint32_t);
    let mut crc: uint16_t = 0xffffi32 as uint16_t;
    crc =
        crc16_ccitt_update(crc,
                           &mut header as *mut configHeader_t as *mut uint8_t
                               as *const libc::c_void,
                           ::core::mem::size_of::<configHeader_t>() as
                               libc::c_ulong as uint32_t);
    let mut reg: *const pgRegistry_t = __pg_registry_start.as_ptr();
    while reg < __pg_registry_end.as_ptr() {
        let regSize: uint16_t = pgSize(reg);
        let mut record: configRecord_t =
            {
                let mut init =
                    configRecord_t{size:
                                       (::core::mem::size_of::<configRecord_t>()
                                            as
                                            libc::c_ulong).wrapping_add(regSize
                                                                            as
                                                                            libc::c_ulong)
                                           as uint16_t,
                                   pgn: pgN(reg),
                                   version: pgVersion(reg),
                                   flags: 0i32 as uint8_t,
                                   pg: [],};
                init
            };
        record.flags =
            (record.flags as libc::c_int |
                 CR_CLASSICATION_SYSTEM as libc::c_int) as uint8_t;
        config_streamer_write(&mut streamer,
                              &mut record as *mut configRecord_t as
                                  *mut uint8_t,
                              ::core::mem::size_of::<configRecord_t>() as
                                  libc::c_ulong as uint32_t);
        crc =
            crc16_ccitt_update(crc,
                               &mut record as *mut configRecord_t as
                                   *mut uint8_t as *const libc::c_void,
                               ::core::mem::size_of::<configRecord_t>() as
                                   libc::c_ulong as uint32_t);
        config_streamer_write(&mut streamer, (*reg).address,
                              regSize as uint32_t);
        crc =
            crc16_ccitt_update(crc, (*reg).address as *const libc::c_void,
                               regSize as uint32_t);
        reg = reg.offset(1)
    }
    let mut footer: configFooter_t =
        {
            let mut init = configFooter_t{terminator: 0i32 as uint16_t,};
            init
        };
    config_streamer_write(&mut streamer,
                          &mut footer as *mut configFooter_t as *mut uint8_t,
                          ::core::mem::size_of::<configFooter_t>() as
                              libc::c_ulong as uint32_t);
    crc =
        crc16_ccitt_update(crc,
                           &mut footer as *mut configFooter_t as *mut uint8_t
                               as *const libc::c_void,
                           ::core::mem::size_of::<configFooter_t>() as
                               libc::c_ulong as uint32_t);
    // include inverted CRC in big endian format in the CRC
    let invertedBigEndianCrc: uint16_t =
        !((crc as libc::c_int & 0xffi32) << 8i32 | crc as libc::c_int >> 8i32)
            as uint16_t;
    config_streamer_write(&mut streamer,
                          &invertedBigEndianCrc as *const uint16_t as
                              *mut uint8_t,
                          ::core::mem::size_of::<uint16_t>() as libc::c_ulong
                              as uint32_t);
    config_streamer_flush(&mut streamer);
    let success: bool = config_streamer_finish(&mut streamer) == 0i32;
    return success;
}
#[no_mangle]
pub unsafe extern "C" fn writeConfigToEEPROM() {
    let mut success: bool = 0i32 != 0;
    // write it
    let mut attempt: libc::c_int = 0i32;
    while attempt < 3i32 && !success {
        if writeSettingsToEEPROM() { success = 1i32 != 0 }
        attempt += 1
    }
    if success as libc::c_int != 0 &&
           isEEPROMVersionValid() as libc::c_int != 0 &&
           isEEPROMStructureValid() as libc::c_int != 0 {
        return
    }
    // Flash write failed - just die now
    failureMode(FAILURE_FLASH_WRITE_FAILED);
}
