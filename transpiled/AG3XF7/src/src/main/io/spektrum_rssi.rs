use ::libc;
extern "C" {
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    static mut spekChannelData: [uint32_t; 12];
    #[no_mangle]
    static mut srxlEnabled: bool;
    #[no_mangle]
    static mut resolution: int32_t;
    #[no_mangle]
    static mut rssi_channel: uint8_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct dbm_table_s {
    pub dBm: int8_t,
    pub reportAs: uint8_t,
}
pub type dbm_table_t = dbm_table_s;
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
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
// Spektrum Rx type. Determined by bind method.
static mut spektrumSatInternal: bool = 1 as libc::c_int != 0;
// Assume internal,bound by BF.
// Variables used for calculating a signal strength from satellite fade.
//  This is time-variant and computed every second based on the fade
//  count over the last second.
static mut spek_fade_last_sec: uint32_t = 0 as libc::c_int as uint32_t;
// Stores the timestamp of the last second.
static mut spek_fade_last_sec_count: uint16_t = 0 as libc::c_int as uint16_t;
// Stores the fade count at the last second.
// Linear mapping and interpolation function
#[no_mangle]
pub unsafe extern "C" fn map(mut x: int32_t, mut in_min: int32_t,
                             mut in_max: int32_t, mut out_min: int32_t,
                             mut out_max: int32_t) -> int32_t {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// Conversion table from dBm to a percentage scale aproximating a more linear RSSI vs Distance curve.
static mut dbmTable: [dbm_table_t; 13] =
    [{
         let mut init =
             dbm_table_s{dBm: -(42 as libc::c_int) as int8_t,
                         reportAs: 101 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             dbm_table_s{dBm: -(49 as libc::c_int) as int8_t,
                         reportAs: 100 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             dbm_table_s{dBm: -(56 as libc::c_int) as int8_t,
                         reportAs: 98 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             dbm_table_s{dBm: -(61 as libc::c_int) as int8_t,
                         reportAs: 95 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             dbm_table_s{dBm: -(66 as libc::c_int) as int8_t,
                         reportAs: 89 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             dbm_table_s{dBm: -(69 as libc::c_int) as int8_t,
                         reportAs: 83 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             dbm_table_s{dBm: -(71 as libc::c_int) as int8_t,
                         reportAs: 78 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             dbm_table_s{dBm: -(73 as libc::c_int) as int8_t,
                         reportAs: 72 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             dbm_table_s{dBm: -(74 as libc::c_int) as int8_t,
                         reportAs: 69 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             dbm_table_s{dBm: -(75 as libc::c_int) as int8_t,
                         reportAs: 66 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             dbm_table_s{dBm: -(76 as libc::c_int) as int8_t,
                         reportAs: 63 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             dbm_table_s{dBm: -(77 as libc::c_int) as int8_t,
                         reportAs: 60 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             dbm_table_s{dBm: -(92 as libc::c_int) as int8_t,
                         reportAs: 0 as libc::c_int as uint8_t,};
         init
     }];
// Convert dBm to Range %
unsafe extern "C" fn dBm2range(mut dBm: int8_t) -> int8_t {
    let mut retval: int8_t =
        dbmTable[0 as libc::c_int as usize].reportAs as int8_t;
    let mut i: uint8_t = 1 as libc::c_int as uint8_t;
    while (i as libc::c_ulong) <
              (::core::mem::size_of::<[dbm_table_t; 13]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<dbm_table_t>()
                                                   as libc::c_ulong) {
        if dBm as libc::c_int >= dbmTable[i as usize].dBm as libc::c_int {
            // Linear interpolation between table points.
            retval =
                map(dBm as int32_t,
                    dbmTable[(i as libc::c_int - 1 as libc::c_int) as
                                 usize].dBm as int32_t,
                    dbmTable[i as usize].dBm as int32_t,
                    dbmTable[(i as libc::c_int - 1 as libc::c_int) as
                                 usize].reportAs as int32_t,
                    dbmTable[i as usize].reportAs as int32_t) as int8_t;
            break ;
        } else { i = i.wrapping_add(1) }
    }
    retval =
        constrain(retval as libc::c_int, 0 as libc::c_int, 100 as libc::c_int)
            as int8_t;
    return retval;
}
#[no_mangle]
pub unsafe extern "C" fn spektrumHandleRSSI(mut spekFrame: *mut uint8_t) {
    static mut spek_last_rssi: int8_t = -(42 as libc::c_int) as int8_t;
    // Fetch RSSI
    if srxlEnabled {
        // Real RSSI reported omly by SRXL Telemetry Rx, in dBm.
        let mut rssi: int8_t =
            *spekFrame.offset(0 as libc::c_int as isize) as int8_t;
        if rssi as libc::c_int <= -(100 as libc::c_int) {
            // If Rx reports -100 dBm or less, it is a fade out and frame loss.
        // If it is a temporary fade, real RSSI will come back in the next frame, in that case.
        // we should not report 0% back as OSD keeps a "minimum RSSI" value. Instead keep last good report
        // If it is a total link loss, failsafe will kick in.
        // We could count the fades here, but currentlly to no use
            // Ignore report and Keep last known good value
            rssi = spek_last_rssi
        }
        if rssi_channel as libc::c_int != 0 as libc::c_int {
            // Do an dBm to percent conversion with an approxatelly linear distance
            // and map the percentage to RSSI RC channel range
            spekChannelData[rssi_channel as usize] =
                map(dBm2range(rssi) as int32_t, 0 as libc::c_int,
                    100 as libc::c_int, 0 as libc::c_int, resolution) as
                    uint16_t as uint32_t
        }
        spek_last_rssi = rssi
    } else {
        // USE_SPEKTRUM_REAL_RSSI
        // Fake RSSI value computed from fades
        let current_secs: uint32_t =
            micros().wrapping_div(1000 as libc::c_int as
                                      libc::c_uint).wrapping_div((1000 as
                                                                      libc::c_int
                                                                      /
                                                                      2 as
                                                                          libc::c_int)
                                                                     as
                                                                     libc::c_uint);
        let mut fade: uint16_t = 0;
        let mut system: uint8_t = 0;
        if spektrumSatInternal {
            // Internal Rx, bind values 3, 5, 7, 9
            fade = *spekFrame.offset(0 as libc::c_int as isize) as uint16_t;
            system = *spekFrame.offset(1 as libc::c_int as isize);
            // Try to detect system type by assuming Internal until we find ANY frame telling otherwise.
            if (system as libc::c_int == 0x1 as libc::c_int) as libc::c_int |
                   (system as libc::c_int == 0x12 as libc::c_int) as
                       libc::c_int |
                   (system as libc::c_int == 0xa2 as libc::c_int) as
                       libc::c_int |
                   (system as libc::c_int == 0xb2 as libc::c_int) as
                       libc::c_int == 0 {
                spektrumSatInternal = 0 as libc::c_int != 0
                // Nope, this is an externally bound Sat Rx
            }
        } else {
            // External Rx, bind values 4, 6, 8, 10
            fade =
                (((*spekFrame.offset(0 as libc::c_int as isize) as
                       libc::c_int) << 8 as libc::c_int) +
                     *spekFrame.offset(1 as libc::c_int as isize) as
                         libc::c_int) as uint16_t
        }
        if spek_fade_last_sec == 0 as libc::c_int as libc::c_uint {
            // Get fade count, different format depending on Rx rype and how Rx is bound. Initially assumed Internal
            // This is the first frame status received.
            spek_fade_last_sec_count = fade;
            spek_fade_last_sec = current_secs
        } else if spek_fade_last_sec != current_secs {
            // If the difference is > 1, then we missed several seconds worth of frames and
            // should just throw out the fade calc (as it's likely a full signal loss).
            if current_secs.wrapping_sub(spek_fade_last_sec) ==
                   1 as libc::c_int as libc::c_uint {
                if rssi_channel as libc::c_int != 0 as libc::c_int {
                    spekChannelData[rssi_channel as usize] =
                        map(fade as libc::c_int -
                                spek_fade_last_sec_count as libc::c_int,
                            40 as libc::c_int / 2 as libc::c_int,
                            0 as libc::c_int, 0 as libc::c_int, resolution) as
                            uint16_t as uint32_t
                }
            }
            spek_fade_last_sec_count = fade;
            spek_fade_last_sec = current_secs
        }
    };
    // USE_SPEKTRUM_FAKE_RSSI
}
// USE_SERIAL_RX
// USE_SPEKTRUM_REAL_RSSI || USE_SPEKTRUM_FAKE_RSSI
