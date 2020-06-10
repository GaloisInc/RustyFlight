use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
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
    fn hsvToRgb24(c: *const hsvColor_t) -> *mut rgbColor24bpp_t;
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
    // for 50us delay
    // number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes)
    #[no_mangle]
    fn ws2811LedStripDMAEnable();
    #[no_mangle]
    fn ws2811LedStripHardwareInit(ioTag: ioTag_t);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
pub type ledStripFormatRGB_e = libc::c_uint;
pub const LED_RGB: ledStripFormatRGB_e = 1;
pub const LED_GRB: ledStripFormatRGB_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct modeColorIndexes_s {
    pub color: [uint8_t; 6],
}
pub type modeColorIndexes_t = modeColorIndexes_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct specialColorIndexes_s {
    pub color: [uint8_t; 11],
}
pub type specialColorIndexes_t = specialColorIndexes_s;
#[no_mangle]
pub static mut modeColors: *const modeColorIndexes_t =
    0 as *const modeColorIndexes_t;
#[no_mangle]
pub static mut specialColors: specialColorIndexes_t =
    specialColorIndexes_t{color: [0; 11],};
#[no_mangle]
pub static mut colors: *mut hsvColor_t =
    0 as *const hsvColor_t as *mut hsvColor_t;
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
 * "Note that the timing on the WS2812/WS2812B LEDs has changed as of batches from WorldSemi
 * manufactured made in October 2013, and timing tolerance for approx 10-30% of parts is very small.
 * Recommendation from WorldSemi is now: 0 = 400ns high/850ns low, and 1 = 850ns high, 400ns low"
 *
 * Currently the timings are 0 = 350ns high/800ns and 1 = 700ns high/650ns low.
 */
#[no_mangle]
#[link_section = ".fastram_bss"]
pub static mut ledStripDMABuffer: [uint32_t; 810] = [0; 810];
#[no_mangle]
pub static mut ws2811LedDataTransferInProgress: uint8_t =
    0 as libc::c_int as uint8_t;
#[no_mangle]
pub static mut BIT_COMPARE_1: uint16_t = 0 as libc::c_int as uint16_t;
#[no_mangle]
pub static mut BIT_COMPARE_0: uint16_t = 0 as libc::c_int as uint16_t;
static mut ledColorBuffer: [hsvColor_t; 32] =
    [hsvColor_t{h: 0, s: 0, v: 0,}; 32];
#[no_mangle]
pub unsafe extern "C" fn setLedHsv(mut index: uint16_t,
                                   mut color: *const hsvColor_t) {
    ledColorBuffer[index as usize] = *color;
}
#[no_mangle]
pub unsafe extern "C" fn getLedHsv(mut index: uint16_t,
                                   mut color: *mut hsvColor_t) {
    *color = ledColorBuffer[index as usize];
}
#[no_mangle]
pub unsafe extern "C" fn setLedValue(mut index: uint16_t, value: uint8_t) {
    ledColorBuffer[index as usize].v = value;
}
#[no_mangle]
pub unsafe extern "C" fn scaleLedValue(mut index: uint16_t,
                                       scalePercent: uint8_t) {
    ledColorBuffer[index as usize].v =
        (ledColorBuffer[index as usize].v as uint16_t as libc::c_int *
             scalePercent as libc::c_int / 100 as libc::c_int) as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn setStripColor(mut color: *const hsvColor_t) {
    let mut index: uint16_t = 0;
    index = 0 as libc::c_int as uint16_t;
    while (index as libc::c_int) < 32 as libc::c_int {
        setLedHsv(index, color);
        index = index.wrapping_add(1)
    };
}
#[no_mangle]
pub unsafe extern "C" fn setStripColors(mut colors_0: *const hsvColor_t) {
    let mut index: uint16_t = 0;
    index = 0 as libc::c_int as uint16_t;
    while (index as libc::c_int) < 32 as libc::c_int {
        let fresh0 = colors_0;
        colors_0 = colors_0.offset(1);
        setLedHsv(index, fresh0);
        index = index.wrapping_add(1)
    };
}
#[no_mangle]
pub unsafe extern "C" fn ws2811LedStripInit(mut ioTag: ioTag_t) {
    memset(ledStripDMABuffer.as_mut_ptr() as *mut libc::c_void,
           0 as libc::c_int,
           ::core::mem::size_of::<[uint32_t; 810]>() as libc::c_ulong);
    ws2811LedStripHardwareInit(ioTag);
    let hsv_white: hsvColor_t =
        {
            let mut init =
                hsvColor_s{h: 0 as libc::c_int as uint16_t,
                           s: 255 as libc::c_int as uint8_t,
                           v: 255 as libc::c_int as uint8_t,};
            init
        };
    setStripColor(&hsv_white);
    // RGB or GRB ordering doesn't matter for white
    ws2811UpdateStrip(LED_RGB);
}
#[no_mangle]
pub unsafe extern "C" fn isWS2811LedStripReady() -> bool {
    return ws2811LedDataTransferInProgress == 0;
}
static mut dmaBufferOffset: uint16_t = 0;
static mut ledIndex: int16_t = 0;
unsafe extern "C" fn fastUpdateLEDDMABuffer(mut ledFormat:
                                                ledStripFormatRGB_e,
                                            mut color: *mut rgbColor24bpp_t) {
    let mut packed_colour: uint32_t = 0;
    match ledFormat as libc::c_uint {
        1 => {
            // WS2811 drivers use RGB format
            packed_colour =
                (((*color).rgb.r as libc::c_int) << 16 as libc::c_int |
                     ((*color).rgb.g as libc::c_int) << 8 as libc::c_int |
                     (*color).rgb.b as libc::c_int) as uint32_t
        }
        0 | _ => {
            // WS2812 drivers use GRB format
            packed_colour =
                (((*color).rgb.g as libc::c_int) << 16 as libc::c_int |
                     ((*color).rgb.r as libc::c_int) << 8 as libc::c_int |
                     (*color).rgb.b as libc::c_int) as uint32_t
        }
    }
    let mut index: int8_t = 23 as libc::c_int as int8_t;
    while index as libc::c_int >= 0 as libc::c_int {
        let fresh1 = dmaBufferOffset;
        dmaBufferOffset = dmaBufferOffset.wrapping_add(1);
        ledStripDMABuffer[fresh1 as usize] =
            if packed_colour &
                   ((1 as libc::c_int) << index as libc::c_int) as
                       libc::c_uint != 0 {
                BIT_COMPARE_1 as libc::c_int
            } else { BIT_COMPARE_0 as libc::c_int } as uint32_t;
        index -= 1
    };
}
/*
 * This method is non-blocking unless an existing LED update is in progress.
 * it does not wait until all the LEDs have been updated, that happens in the background.
 */
#[no_mangle]
pub unsafe extern "C" fn ws2811UpdateStrip(mut ledFormat:
                                               ledStripFormatRGB_e) {
    static mut rgb24: *mut rgbColor24bpp_t =
        0 as *const rgbColor24bpp_t as *mut rgbColor24bpp_t;
    // don't wait - risk of infinite block, just get an update next time round
    if ws2811LedDataTransferInProgress != 0 {
        return
    } // reset buffer memory index
    dmaBufferOffset = 0 as libc::c_int as uint16_t; // reset led index
    ledIndex = 0 as libc::c_int as int16_t;
    // fill transmit buffer with correct compare values to achieve
    // correct pulse widths according to color values
    while (ledIndex as libc::c_int) < 32 as libc::c_int {
        rgb24 =
            hsvToRgb24(&mut *ledColorBuffer.as_mut_ptr().offset(ledIndex as
                                                                    isize));
        fastUpdateLEDDMABuffer(ledFormat, rgb24);
        ledIndex += 1
    }
    ::core::ptr::write_volatile(&mut ws2811LedDataTransferInProgress as
                                    *mut uint8_t,
                                1 as libc::c_int as uint8_t);
    ws2811LedStripDMAEnable();
}
