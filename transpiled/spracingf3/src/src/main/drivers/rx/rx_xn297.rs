use core;
use libc;
extern "C" {
    #[no_mangle]
    fn crc16_ccitt(crc: uint16_t, a: libc::c_uchar) -> uint16_t;
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
    // This file is copied with modifications from project Deviation,
// see http://deviationtx.com, file iface_nrf24l01.h
    // bit value
    // Register map of nRF24L01
    // Auto Acknowledge
    // Address Width
    // automatic RETRansmission
    // RF CHannel
    //Received Power Detector in the nRF23L01+, called CD (Carrier Detect) in the nRF24L01
    // Payload Width
    // DYNamic PayloaD
    // Bit position mnemonics
    // Pre-shifted and combined bits
    // Pipes
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
    [0xbci32 as uint8_t, 0xe5i32 as uint8_t, 0x66i32 as uint8_t,
     0xdi32 as uint8_t, 0xaei32 as uint8_t, 0x8ci32 as uint8_t,
     0x88i32 as uint8_t, 0x12i32 as uint8_t, 0x69i32 as uint8_t,
     0xeei32 as uint8_t, 0x1fi32 as uint8_t, 0xc7i32 as uint8_t,
     0x62i32 as uint8_t, 0x97i32 as uint8_t, 0xd5i32 as uint8_t,
     0xbi32 as uint8_t, 0x79i32 as uint8_t, 0xcai32 as uint8_t,
     0xcci32 as uint8_t, 0x1bi32 as uint8_t, 0x5di32 as uint8_t,
     0x19i32 as uint8_t, 0x10i32 as uint8_t, 0x24i32 as uint8_t,
     0xd3i32 as uint8_t, 0xdci32 as uint8_t, 0x3fi32 as uint8_t,
     0x8ei32 as uint8_t, 0xc5i32 as uint8_t, 0x2fi32 as uint8_t];
static mut xn297_crc_xorout: [uint16_t; 26] =
    [0x9ba7i32 as uint16_t, 0x8bbbi32 as uint16_t, 0x85e1i32 as uint16_t,
     0x3e8ci32 as uint16_t, 0x451ei32 as uint16_t, 0x18e6i32 as uint16_t,
     0x6b24i32 as uint16_t, 0xe7abi32 as uint16_t, 0x3828i32 as uint16_t,
     0x814bi32 as uint16_t, 0xd461i32 as uint16_t, 0xf494i32 as uint16_t,
     0x2503i32 as uint16_t, 0x691di32 as uint16_t, 0xfe8bi32 as uint16_t,
     0x9ba7i32 as uint16_t, 0x8b17i32 as uint16_t, 0x2920i32 as uint16_t,
     0x8b5fi32 as uint16_t, 0x61b1i32 as uint16_t, 0xd391i32 as uint16_t,
     0x7401i32 as uint16_t, 0x2138i32 as uint16_t, 0x129fi32 as uint16_t,
     0xb3a0i32 as uint16_t, 0x2988i32 as uint16_t];
unsafe extern "C" fn bitReverse(mut bIn: uint8_t) -> uint8_t {
    let mut bOut: uint8_t = 0i32 as uint8_t;
    let mut ii: libc::c_int = 0i32;
    while ii < 8i32 {
        bOut =
            ((bOut as libc::c_int) << 1i32 | bIn as libc::c_int & 1i32) as
                uint8_t;
        bIn = (bIn as libc::c_int >> 1i32) as uint8_t;
        ii += 1
    }
    return bOut;
}
#[no_mangle]
pub unsafe extern "C" fn XN297_UnscramblePayload(mut data: *mut uint8_t,
                                                 mut len: libc::c_int,
                                                 mut rxAddr: *const uint8_t)
 -> uint16_t {
    let mut crc: uint16_t = 0xb5d2i32 as uint16_t;
    if !rxAddr.is_null() {
        let mut ii: libc::c_int = 0i32;
        while ii < 5i32 {
            crc =
                crc16_ccitt(crc, *rxAddr.offset((5i32 - 1i32 - ii) as isize));
            ii += 1
        }
    }
    let mut ii_0: libc::c_int = 0i32;
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
    let mut crc: uint16_t = 0xb5d2i32 as uint16_t;
    let mut ii: libc::c_int = 0i32;
    while ii < 5i32 {
        packet[ii as usize] = *rxAddr.offset((5i32 - 1i32 - ii) as isize);
        crc = crc16_ccitt(crc, packet[ii as usize]);
        ii += 1
    }
    let mut ii_0: libc::c_int = 0i32;
    while ii_0 < len {
        let bOut: uint8_t = bitReverse(*data.offset(ii_0 as isize));
        packet[(ii_0 + 5i32) as usize] =
            (bOut as libc::c_int ^
                 xn297_data_scramble[ii_0 as usize] as libc::c_int) as
                uint8_t;
        crc = crc16_ccitt(crc, packet[(ii_0 + 5i32) as usize]);
        ii_0 += 1
    }
    crc =
        (crc as libc::c_int ^ xn297_crc_xorout[len as usize] as libc::c_int)
            as uint16_t;
    packet[(5i32 + len) as usize] = (crc as libc::c_int >> 8i32) as uint8_t;
    packet[(5i32 + len + 1i32) as usize] =
        (crc as libc::c_int & 0xffi32) as uint8_t;
    return NRF24L01_WritePayload(packet.as_mut_ptr(),
                                 (5i32 + len + 2i32) as uint8_t);
}
