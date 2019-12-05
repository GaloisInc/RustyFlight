use core;
use libc;
extern "C" {
    // Disabling this, in favour of tfp_format to be used in cli.c
//int tfp_printf(const char *fmt, ...);
    #[no_mangle]
    fn tfp_sprintf(s: *mut libc::c_char, fmt: *const libc::c_char, _: ...)
     -> libc::c_int;
    #[no_mangle]
    fn millis() -> timeMs_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type __int64_t = libc::c_long;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type int64_t = __int64_t;
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
// documentary
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
pub type timeMs_t = uint32_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct timeConfig_s {
    pub tz_offsetMinutes: int16_t,
}
pub type timeConfig_t = timeConfig_s;
pub type rtcTime_t = int64_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct _dateTime_s {
    pub year: uint16_t,
    pub month: uint8_t,
    pub day: uint8_t,
    pub hours: uint8_t,
    pub minutes: uint8_t,
    pub seconds: uint8_t,
    pub millis: uint16_t,
}
pub type dateTime_t = _dateTime_s;
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
#[inline]
unsafe extern "C" fn timeConfig() -> *const timeConfig_t {
    return &mut timeConfig_System;
}
/* base */
/* size */
// rtcTime_t when the system was started.
// Calculated in rtcSet().
static mut started: rtcTime_t = 0i32 as rtcTime_t;
static mut days: [[uint16_t; 12]; 4] =
    [[0i32 as uint16_t, 31i32 as uint16_t, 60i32 as uint16_t,
      91i32 as uint16_t, 121i32 as uint16_t, 152i32 as uint16_t,
      182i32 as uint16_t, 213i32 as uint16_t, 244i32 as uint16_t,
      274i32 as uint16_t, 305i32 as uint16_t, 335i32 as uint16_t],
     [366i32 as uint16_t, 397i32 as uint16_t, 425i32 as uint16_t,
      456i32 as uint16_t, 486i32 as uint16_t, 517i32 as uint16_t,
      547i32 as uint16_t, 578i32 as uint16_t, 609i32 as uint16_t,
      639i32 as uint16_t, 670i32 as uint16_t, 700i32 as uint16_t],
     [731i32 as uint16_t, 762i32 as uint16_t, 790i32 as uint16_t,
      821i32 as uint16_t, 851i32 as uint16_t, 882i32 as uint16_t,
      912i32 as uint16_t, 943i32 as uint16_t, 974i32 as uint16_t,
      1004i32 as uint16_t, 1035i32 as uint16_t, 1065i32 as uint16_t],
     [1096i32 as uint16_t, 1127i32 as uint16_t, 1155i32 as uint16_t,
      1186i32 as uint16_t, 1216i32 as uint16_t, 1247i32 as uint16_t,
      1277i32 as uint16_t, 1308i32 as uint16_t, 1339i32 as uint16_t,
      1369i32 as uint16_t, 1400i32 as uint16_t, 1430i32 as uint16_t]];
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut timeConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn: (526i32 | 0i32 << 12i32) as pgn_t,
                             size:
                                 (::core::mem::size_of::<timeConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &timeConfig_System as *const timeConfig_t as
                                     *mut timeConfig_t as *mut uint8_t,
                             copy:
                                 &timeConfig_Copy as *const timeConfig_t as
                                     *mut timeConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     &pgResetTemplate_timeConfig
                                                         as
                                                         *const timeConfig_t
                                                         as
                                                         *mut libc::c_void,},}; // 0-59
            init
        }
    };
#[no_mangle]
pub static mut timeConfig_Copy: timeConfig_t =
    timeConfig_t{tz_offsetMinutes: 0,};
#[no_mangle]
pub static mut timeConfig_System: timeConfig_t =
    timeConfig_t{tz_offsetMinutes: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_timeConfig: timeConfig_t =
    {
        let mut init =
            timeConfig_s{tz_offsetMinutes: 0i32 as int16_t,}; // 0-59
        init
    };
unsafe extern "C" fn dateTimeToRtcTime(mut dt: *mut dateTime_t) -> rtcTime_t {
    let mut second: libc::c_uint = (*dt).seconds as libc::c_uint; // 0-23
    let mut minute: libc::c_uint = (*dt).minutes as libc::c_uint; // 0-30
    let mut hour: libc::c_uint = (*dt).hours as libc::c_uint; // 0-11
    let mut day: libc::c_uint =
        ((*dt).day as libc::c_int - 1i32) as libc::c_uint; // 0-99
    let mut month: libc::c_uint =
        ((*dt).month as libc::c_int - 1i32) as libc::c_uint;
    let mut year: libc::c_uint =
        ((*dt).year as libc::c_int - 2000i32) as libc::c_uint;
    let mut unixTime: int32_t =
        year.wrapping_div(4i32 as
                              libc::c_uint).wrapping_mul((365i32 * 4i32 +
                                                              1i32) as
                                                             libc::c_uint).wrapping_add(days[year.wrapping_rem(4i32
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                                                                 as
                                                                                                 usize][month
                                                                                                            as
                                                                                                            usize]
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(day).wrapping_mul(24i32
                                                                                                                                             as
                                                                                                                                             libc::c_uint).wrapping_add(hour).wrapping_mul(60i32
                                                                                                                                                                                               as
                                                                                                                                                                                               libc::c_uint).wrapping_add(minute).wrapping_mul(60i32
                                                                                                                                                                                                                                                   as
                                                                                                                                                                                                                                                   libc::c_uint).wrapping_add(second).wrapping_add(946684800i32
                                                                                                                                                                                                                                                                                                       as
                                                                                                                                                                                                                                                                                                       libc::c_uint)
            as int32_t;
    return rtcTimeMake(unixTime, (*dt).millis);
}
unsafe extern "C" fn rtcTimeToDateTime(mut dt: *mut dateTime_t,
                                       mut t: rtcTime_t) {
    let mut unixTime: int32_t =
        (t / 1000i32 as libc::c_long - 946684800i32 as libc::c_long) as
            int32_t;
    (*dt).seconds = (unixTime % 60i32) as uint8_t;
    unixTime /= 60i32;
    (*dt).minutes = (unixTime % 60i32) as uint8_t;
    unixTime /= 60i32;
    (*dt).hours = (unixTime % 24i32) as uint8_t;
    unixTime /= 24i32;
    let mut years: libc::c_uint =
        (unixTime / (365i32 * 4i32 + 1i32) * 4i32) as libc::c_uint;
    unixTime %= 365i32 * 4i32 + 1i32;
    let mut year: libc::c_uint = 0;
    year = 3i32 as libc::c_uint;
    while year > 0i32 as libc::c_uint {
        if unixTime >= days[year as usize][0] as libc::c_int { break ; }
        year = year.wrapping_sub(1)
    }
    let mut month: libc::c_uint = 0;
    month = 11i32 as libc::c_uint;
    while month > 0i32 as libc::c_uint {
        if unixTime >= days[year as usize][month as usize] as libc::c_int {
            break ;
        }
        month = month.wrapping_sub(1)
    }
    (*dt).year =
        years.wrapping_add(year).wrapping_add(2000i32 as libc::c_uint) as
            uint16_t;
    (*dt).month = month.wrapping_add(1i32 as libc::c_uint) as uint8_t;
    (*dt).day =
        (unixTime - days[year as usize][month as usize] as libc::c_int + 1i32)
            as uint8_t;
    (*dt).millis = (t % 1000i32 as libc::c_long) as uint16_t;
}
unsafe extern "C" fn rtcGetDefaultDateTime(mut dateTime: *mut dateTime_t) {
    (*dateTime).year = 0i32 as uint16_t;
    (*dateTime).month = 1i32 as uint8_t;
    (*dateTime).day = 1i32 as uint8_t;
    (*dateTime).hours = 0i32 as uint8_t;
    (*dateTime).minutes = 0i32 as uint8_t;
    (*dateTime).seconds = 0i32 as uint8_t;
    (*dateTime).millis = 0i32 as uint16_t;
}
unsafe extern "C" fn rtcIsDateTimeValid(mut dateTime: *mut dateTime_t)
 -> bool {
    return (*dateTime).year as libc::c_int >= 2000i32 &&
               ((*dateTime).month as libc::c_int >= 1i32 &&
                    (*dateTime).month as libc::c_int <= 12i32) &&
               ((*dateTime).day as libc::c_int >= 1i32 &&
                    (*dateTime).day as libc::c_int <= 31i32) &&
               (*dateTime).hours as libc::c_int <= 23i32 &&
               (*dateTime).minutes as libc::c_int <= 59i32 &&
               (*dateTime).seconds as libc::c_int <= 59i32 &&
               (*dateTime).millis as libc::c_int <= 999i32;
}
unsafe extern "C" fn dateTimeWithOffset(mut dateTimeOffset: *mut dateTime_t,
                                        mut dateTimeInitial: *mut dateTime_t,
                                        mut minutes: int16_t) {
    let mut initialTime: rtcTime_t = dateTimeToRtcTime(dateTimeInitial);
    let mut offsetTime: rtcTime_t =
        rtcTimeMake(rtcTimeGetSeconds(&mut initialTime) +
                        minutes as libc::c_int * 60i32,
                    rtcTimeGetMillis(&mut initialTime));
    rtcTimeToDateTime(dateTimeOffset, offsetTime);
}
unsafe extern "C" fn dateTimeFormat(mut buf: *mut libc::c_char,
                                    mut dateTime: *mut dateTime_t,
                                    mut offsetMinutes: int16_t,
                                    mut shortVersion: bool) -> bool {
    let mut local: dateTime_t =
        dateTime_t{year: 0,
                   month: 0,
                   day: 0,
                   hours: 0,
                   minutes: 0,
                   seconds: 0,
                   millis: 0,};
    let mut tz_hours: libc::c_int = 0i32;
    let mut tz_minutes: libc::c_int = 0i32;
    let mut retVal: bool = 1i32 != 0;
    // Apply offset if necessary
    if offsetMinutes as libc::c_int != 0i32 {
        tz_hours = offsetMinutes as libc::c_int / 60i32;
        tz_minutes =
            ({
                 let mut _x: libc::c_int =
                     offsetMinutes as libc::c_int % 60i32;
                 if _x > 0i32 { _x } else { -_x }
             });
        dateTimeWithOffset(&mut local, dateTime, offsetMinutes);
        dateTime = &mut local
    }
    if !rtcIsDateTimeValid(dateTime) {
        rtcGetDefaultDateTime(&mut local);
        dateTime = &mut local;
        retVal = 0i32 != 0
    }
    if shortVersion {
        tfp_sprintf(buf,
                    b"%04u-%02u-%02u %02u:%02u:%02u\x00" as *const u8 as
                        *const libc::c_char, (*dateTime).year as libc::c_int,
                    (*dateTime).month as libc::c_int,
                    (*dateTime).day as libc::c_int,
                    (*dateTime).hours as libc::c_int,
                    (*dateTime).minutes as libc::c_int,
                    (*dateTime).seconds as libc::c_int);
    } else {
        // Changes to this format might require updates in
        // dateTimeSplitFormatted()
        // Datetime is in ISO_8601 format, https://en.wikipedia.org/wiki/ISO_8601
        tfp_sprintf(buf,
                    b"%04u-%02u-%02uT%02u:%02u:%02u.%03u%c%02d:%02d\x00" as
                        *const u8 as *const libc::c_char,
                    (*dateTime).year as libc::c_int,
                    (*dateTime).month as libc::c_int,
                    (*dateTime).day as libc::c_int,
                    (*dateTime).hours as libc::c_int,
                    (*dateTime).minutes as libc::c_int,
                    (*dateTime).seconds as libc::c_int,
                    (*dateTime).millis as libc::c_int,
                    if tz_hours >= 0i32 { '+' as i32 } else { '-' as i32 },
                    ({
                         let mut _x: libc::c_int = tz_hours;
                         if _x > 0i32 { _x } else { -_x }
                     }), tz_minutes);
    }
    return retVal;
}
#[no_mangle]
pub unsafe extern "C" fn rtcTimeMake(mut secs: int32_t,
                                     mut millis_0: uint16_t) -> rtcTime_t {
    return secs as rtcTime_t * 1000i32 as libc::c_long +
               millis_0 as libc::c_long;
}
#[no_mangle]
pub unsafe extern "C" fn rtcTimeGetSeconds(mut t: *mut rtcTime_t) -> int32_t {
    return (*t / 1000i32 as libc::c_long) as int32_t;
}
#[no_mangle]
pub unsafe extern "C" fn rtcTimeGetMillis(mut t: *mut rtcTime_t) -> uint16_t {
    return (*t % 1000i32 as libc::c_long) as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn dateTimeFormatUTC(mut buf: *mut libc::c_char,
                                           mut dt: *mut dateTime_t) -> bool {
    return dateTimeFormat(buf, dt, 0i32 as int16_t, 0i32 != 0);
}
#[no_mangle]
pub unsafe extern "C" fn dateTimeFormatLocal(mut buf: *mut libc::c_char,
                                             mut dt: *mut dateTime_t)
 -> bool {
    return dateTimeFormat(buf, dt, (*timeConfig()).tz_offsetMinutes,
                          0i32 != 0);
}
#[no_mangle]
pub unsafe extern "C" fn dateTimeFormatLocalShort(mut buf: *mut libc::c_char,
                                                  mut dt: *mut dateTime_t)
 -> bool {
    return dateTimeFormat(buf, dt, (*timeConfig()).tz_offsetMinutes,
                          1i32 != 0);
}
#[no_mangle]
pub unsafe extern "C" fn dateTimeUTCToLocal(mut utcDateTime: *mut dateTime_t,
                                            mut localDateTime:
                                                *mut dateTime_t) {
    dateTimeWithOffset(localDateTime, utcDateTime,
                       (*timeConfig()).tz_offsetMinutes);
}
#[no_mangle]
pub unsafe extern "C" fn dateTimeSplitFormatted(mut formatted:
                                                    *mut libc::c_char,
                                                mut date:
                                                    *mut *mut libc::c_char,
                                                mut time:
                                                    *mut *mut libc::c_char)
 -> bool {
    // Just look for the T and replace it with a zero
    // XXX: Keep in sync with dateTimeFormat()
    let mut p: *mut libc::c_char = formatted;
    while *p != 0 {
        if *p as libc::c_int == 'T' as i32 {
            *date = formatted;
            *time = p.offset(1);
            *p = '\u{0}' as i32 as libc::c_char;
            return 1i32 != 0
        }
        p = p.offset(1)
    }
    return 0i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn rtcHasTime() -> bool {
    return started != 0i32 as libc::c_long;
}
#[no_mangle]
pub unsafe extern "C" fn rtcGet(mut t: *mut rtcTime_t) -> bool {
    if !rtcHasTime() { return 0i32 != 0 }
    *t = started + millis() as libc::c_long;
    return 1i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn rtcSet(mut t: *mut rtcTime_t) -> bool {
    started = *t - millis() as libc::c_long;
    return 1i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn rtcGetDateTime(mut dt: *mut dateTime_t) -> bool {
    let mut t: rtcTime_t = 0;
    if rtcGet(&mut t) { rtcTimeToDateTime(dt, t); return 1i32 != 0 }
    // No time stored, fill dt with 0000-01-01T00:00:00.000
    rtcGetDefaultDateTime(dt);
    return 0i32 != 0;
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
// time difference, 32 bits always sufficient
// millisecond time
// microsecond time
// Offset from UTC in minutes, might be positive or negative
// Milliseconds since Jan 1 1970
// full year
// 1-12
// 1-31
// 0-23
// 0-59
// 0-59
// 0-999
// buf must be at least FORMATTED_DATE_TIME_BUFSIZE
// dateTimeSplitFormatted splits a formatted date into its date
// and time parts. Note that the string pointed by formatted will
// be modifed and will become invalid after calling this function.
#[no_mangle]
pub unsafe extern "C" fn rtcSetDateTime(mut dt: *mut dateTime_t) -> bool {
    let mut t: rtcTime_t = dateTimeToRtcTime(dt);
    return rtcSet(&mut t);
}
