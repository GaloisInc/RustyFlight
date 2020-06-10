use ::libc;
extern "C" {
    #[no_mangle]
    fn crc16_ccitt(crc: uint16_t, a: libc::c_uchar) -> uint16_t;
    #[no_mangle]
    fn NRF24L01_WritePayload(data: *const uint8_t, length: uint8_t)
     -> uint8_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
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
// This file borrows heavily from project Deviation,
// see http://deviationtx.com
static mut xn297_data_scramble: [uint8_t; 30] =
    [0xbc as libc::c_int as uint8_t, 0xe5 as libc::c_int as uint8_t,
     0x66 as libc::c_int as uint8_t, 0xd as libc::c_int as uint8_t,
     0xae as libc::c_int as uint8_t, 0x8c as libc::c_int as uint8_t,
     0x88 as libc::c_int as uint8_t, 0x12 as libc::c_int as uint8_t,
     0x69 as libc::c_int as uint8_t, 0xee as libc::c_int as uint8_t,
     0x1f as libc::c_int as uint8_t, 0xc7 as libc::c_int as uint8_t,
     0x62 as libc::c_int as uint8_t, 0x97 as libc::c_int as uint8_t,
     0xd5 as libc::c_int as uint8_t, 0xb as libc::c_int as uint8_t,
     0x79 as libc::c_int as uint8_t, 0xca as libc::c_int as uint8_t,
     0xcc as libc::c_int as uint8_t, 0x1b as libc::c_int as uint8_t,
     0x5d as libc::c_int as uint8_t, 0x19 as libc::c_int as uint8_t,
     0x10 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
     0xd3 as libc::c_int as uint8_t, 0xdc as libc::c_int as uint8_t,
     0x3f as libc::c_int as uint8_t, 0x8e as libc::c_int as uint8_t,
     0xc5 as libc::c_int as uint8_t, 0x2f as libc::c_int as uint8_t];
static mut xn297_crc_xorout: [uint16_t; 26] =
    [0x9ba7 as libc::c_int as uint16_t, 0x8bbb as libc::c_int as uint16_t,
     0x85e1 as libc::c_int as uint16_t, 0x3e8c as libc::c_int as uint16_t,
     0x451e as libc::c_int as uint16_t, 0x18e6 as libc::c_int as uint16_t,
     0x6b24 as libc::c_int as uint16_t, 0xe7ab as libc::c_int as uint16_t,
     0x3828 as libc::c_int as uint16_t, 0x814b as libc::c_int as uint16_t,
     0xd461 as libc::c_int as uint16_t, 0xf494 as libc::c_int as uint16_t,
     0x2503 as libc::c_int as uint16_t, 0x691d as libc::c_int as uint16_t,
     0xfe8b as libc::c_int as uint16_t, 0x9ba7 as libc::c_int as uint16_t,
     0x8b17 as libc::c_int as uint16_t, 0x2920 as libc::c_int as uint16_t,
     0x8b5f as libc::c_int as uint16_t, 0x61b1 as libc::c_int as uint16_t,
     0xd391 as libc::c_int as uint16_t, 0x7401 as libc::c_int as uint16_t,
     0x2138 as libc::c_int as uint16_t, 0x129f as libc::c_int as uint16_t,
     0xb3a0 as libc::c_int as uint16_t, 0x2988 as libc::c_int as uint16_t];
unsafe extern "C" fn bitReverse(mut bIn: uint8_t) -> uint8_t {
    let mut bOut: uint8_t = 0 as libc::c_int as uint8_t;
    let mut ii: libc::c_int = 0 as libc::c_int;
    while ii < 8 as libc::c_int {
        bOut =
            ((bOut as libc::c_int) << 1 as libc::c_int |
                 bIn as libc::c_int & 1 as libc::c_int) as uint8_t;
        bIn = (bIn as libc::c_int >> 1 as libc::c_int) as uint8_t;
        ii += 1
    }
    return bOut;
}
#[no_mangle]
pub unsafe extern "C" fn XN297_UnscramblePayload(mut data: *mut uint8_t,
                                                 mut len: libc::c_int,
                                                 mut rxAddr: *const uint8_t)
 -> uint16_t {
    let mut crc: uint16_t = 0xb5d2 as libc::c_int as uint16_t;
    if !rxAddr.is_null() {
        let mut ii: libc::c_int = 0 as libc::c_int;
        while ii < 5 as libc::c_int {
            crc =
                crc16_ccitt(crc,
                            *rxAddr.offset((5 as libc::c_int -
                                                1 as libc::c_int - ii) as
                                               isize));
            ii += 1
        }
    }
    let mut ii_0: libc::c_int = 0 as libc::c_int;
    while ii_0 < len {
        crc = crc16_ccitt(crc, *data.offset(ii_0 as isize));
        *data.offset(ii_0 as isize) =
            bitReverse((*data.offset(ii_0 as isize) as libc::c_int ^
                            xn297_data_scramble[ii_0 as usize] as libc::c_int)
                           as uint8_t);
        ii_0 += 1
    }
    crc =
        (crc as libc::c_int ^ xn297_crc_xorout[len as usize] as libc::c_int)
            as uint16_t;
    return crc;
}
#[no_mangle]
pub unsafe extern "C" fn XN297_WritePayload(mut data: *mut uint8_t,
                                            mut len: libc::c_int,
                                            mut rxAddr: *const uint8_t)
 -> uint8_t {
    let mut packet: [uint8_t; 32] = [0; 32];
    let mut crc: uint16_t = 0xb5d2 as libc::c_int as uint16_t;
    let mut ii: libc::c_int = 0 as libc::c_int;
    while ii < 5 as libc::c_int {
        packet[ii as usize] =
            *rxAddr.offset((5 as libc::c_int - 1 as libc::c_int - ii) as
                               isize);
        crc = crc16_ccitt(crc, packet[ii as usize]);
        ii += 1
    }
    let mut ii_0: libc::c_int = 0 as libc::c_int;
    while ii_0 < len {
        let bOut: uint8_t = bitReverse(*data.offset(ii_0 as isize));
        packet[(ii_0 + 5 as libc::c_int) as usize] =
            (bOut as libc::c_int ^
                 xn297_data_scramble[ii_0 as usize] as libc::c_int) as
                uint8_t;
        crc = crc16_ccitt(crc, packet[(ii_0 + 5 as libc::c_int) as usize]);
        ii_0 += 1
    }
    crc =
        (crc as libc::c_int ^ xn297_crc_xorout[len as usize] as libc::c_int)
            as uint16_t;
    packet[(5 as libc::c_int + len) as usize] =
        (crc as libc::c_int >> 8 as libc::c_int) as uint8_t;
    packet[(5 as libc::c_int + len + 1 as libc::c_int) as usize] =
        (crc as libc::c_int & 0xff as libc::c_int) as uint8_t;
    return NRF24L01_WritePayload(packet.as_mut_ptr(),
                                 (5 as libc::c_int + len + 2 as libc::c_int)
                                     as uint8_t);
}
