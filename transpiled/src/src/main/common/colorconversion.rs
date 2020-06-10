use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rgbColor24bpp_s {
    pub r: uint8_t,
    pub g: uint8_t,
    pub b: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union rgbColor24bpp_t {
    pub rgb: rgbColor24bpp_s,
    pub raw: [uint8_t; 3],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct hsvColor_s {
    pub h: uint16_t,
    pub s: uint8_t,
    pub v: uint8_t,
}
pub type hsvColor_t = hsvColor_s;
// 0 - 359
// 0 - 255
// 0 - 255
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
/*
 * Source below found here: http://www.kasperkamperman.com/blog/arduino/arduino-programming-hsb-to-rgb/
 */
#[no_mangle]
pub unsafe extern "C" fn hsvToRgb24(mut c: *const hsvColor_t)
 -> *mut rgbColor24bpp_t {
    static mut r: rgbColor24bpp_t =
        rgbColor24bpp_t{rgb: rgbColor24bpp_s{r: 0, g: 0, b: 0,},};
    let mut val: uint16_t = (*c).v as uint16_t;
    let mut sat: uint16_t =
        (255 as libc::c_int - (*c).s as libc::c_int) as uint16_t;
    let mut base: uint32_t = 0;
    let mut hue: uint16_t = (*c).h;
    if sat as libc::c_int == 0 as libc::c_int {
        // Acromatic color (gray). Hue doesn't mind.
        r.rgb.r = val as uint8_t;
        r.rgb.g = val as uint8_t;
        r.rgb.b = val as uint8_t
    } else {
        base =
            ((255 as libc::c_int - sat as libc::c_int) * val as libc::c_int >>
                 8 as libc::c_int) as uint32_t;
        match hue as libc::c_int / 60 as libc::c_int {
            0 => {
                r.rgb.r = val as uint8_t;
                r.rgb.g =
                    (val as
                         libc::c_uint).wrapping_sub(base).wrapping_mul(hue as
                                                                           libc::c_uint).wrapping_div(60
                                                                                                          as
                                                                                                          libc::c_int
                                                                                                          as
                                                                                                          libc::c_uint).wrapping_add(base)
                        as uint8_t;
                r.rgb.b = base as uint8_t
            }
            1 => {
                r.rgb.r =
                    (val as
                         libc::c_uint).wrapping_sub(base).wrapping_mul((60 as
                                                                            libc::c_int
                                                                            -
                                                                            hue
                                                                                as
                                                                                libc::c_int
                                                                                %
                                                                                60
                                                                                    as
                                                                                    libc::c_int)
                                                                           as
                                                                           libc::c_uint).wrapping_div(60
                                                                                                          as
                                                                                                          libc::c_int
                                                                                                          as
                                                                                                          libc::c_uint).wrapping_add(base)
                        as uint8_t;
                r.rgb.g = val as uint8_t;
                r.rgb.b = base as uint8_t
            }
            2 => {
                r.rgb.r = base as uint8_t;
                r.rgb.g = val as uint8_t;
                r.rgb.b =
                    (val as
                         libc::c_uint).wrapping_sub(base).wrapping_mul((hue as
                                                                            libc::c_int
                                                                            %
                                                                            60
                                                                                as
                                                                                libc::c_int)
                                                                           as
                                                                           libc::c_uint).wrapping_div(60
                                                                                                          as
                                                                                                          libc::c_int
                                                                                                          as
                                                                                                          libc::c_uint).wrapping_add(base)
                        as uint8_t
            }
            3 => {
                r.rgb.r = base as uint8_t;
                r.rgb.g =
                    (val as
                         libc::c_uint).wrapping_sub(base).wrapping_mul((60 as
                                                                            libc::c_int
                                                                            -
                                                                            hue
                                                                                as
                                                                                libc::c_int
                                                                                %
                                                                                60
                                                                                    as
                                                                                    libc::c_int)
                                                                           as
                                                                           libc::c_uint).wrapping_div(60
                                                                                                          as
                                                                                                          libc::c_int
                                                                                                          as
                                                                                                          libc::c_uint).wrapping_add(base)
                        as uint8_t;
                r.rgb.b = val as uint8_t
            }
            4 => {
                r.rgb.r =
                    (val as
                         libc::c_uint).wrapping_sub(base).wrapping_mul((hue as
                                                                            libc::c_int
                                                                            %
                                                                            60
                                                                                as
                                                                                libc::c_int)
                                                                           as
                                                                           libc::c_uint).wrapping_div(60
                                                                                                          as
                                                                                                          libc::c_int
                                                                                                          as
                                                                                                          libc::c_uint).wrapping_add(base)
                        as uint8_t;
                r.rgb.g = base as uint8_t;
                r.rgb.b = val as uint8_t
            }
            5 => {
                r.rgb.r = val as uint8_t;
                r.rgb.g = base as uint8_t;
                r.rgb.b =
                    (val as
                         libc::c_uint).wrapping_sub(base).wrapping_mul((60 as
                                                                            libc::c_int
                                                                            -
                                                                            hue
                                                                                as
                                                                                libc::c_int
                                                                                %
                                                                                60
                                                                                    as
                                                                                    libc::c_int)
                                                                           as
                                                                           libc::c_uint).wrapping_div(60
                                                                                                          as
                                                                                                          libc::c_int
                                                                                                          as
                                                                                                          libc::c_uint).wrapping_add(base)
                        as uint8_t
            }
            _ => { }
        }
    }
    return &mut r;
}
