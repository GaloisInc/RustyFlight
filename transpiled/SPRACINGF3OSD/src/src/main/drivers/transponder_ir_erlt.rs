use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub union transponderIrDMABuffer_s {
    pub arcitimer: [uint8_t; 620],
    pub ilap: [uint8_t; 720],
    pub erlt: [uint8_t; 200],
}
/* ** ******** ***/
/*
 * Implementation note:
 * Using around over 700 bytes for a transponder DMA buffer is a little excessive, likely an alternative implementation that uses a fast
 * ISR to generate the output signal dynamically based on state would be more memory efficient and would likely be more appropriate for
 * other targets.  However this approach requires very little CPU time and is just fire-and-forget.
 *
 * On an STM32F303CC 720 bytes is currently fine and that is the target for which this code was designed for.
 */
pub type transponderIrDMABuffer_t = transponderIrDMABuffer_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct transponder_s {
    pub gap_toggles: uint8_t,
    pub timer_hz: uint32_t,
    pub timer_carrier_hz: uint32_t,
    pub bitToggleOne: uint16_t,
    pub dma_buffer_size: uint32_t,
    pub transponderIrDMABuffer: transponderIrDMABuffer_t,
    pub vTable: *const transponderVTable,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct transponderVTable {
    pub updateTransponderDMABuffer: Option<unsafe extern "C" fn(_:
                                                                    *mut transponder_t,
                                                                _:
                                                                    *const uint8_t)
                                               -> ()>,
}
pub type transponder_t = transponder_s;
// 620
// 720
// 91-200
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
static mut dmaBufferOffset: uint16_t = 0;
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
pub unsafe extern "C" fn transponderIrInitERLT(mut transponder:
                                                   *mut transponder_t) {
    (*transponder).dma_buffer_size =
        200 as libc::c_int as
            uint32_t; //transponderData is stored inverted, so invert before using
    (*transponder).vTable = &erltTansponderVTable; //sum of one bits
    (*transponder).timer_hz =
        18 as libc::c_int as uint32_t; //reset buffer count
    (*transponder).timer_carrier_hz = 38000 as libc::c_int as uint32_t;
    memset(&mut (*transponder).transponderIrDMABuffer.erlt as
               *mut [uint8_t; 200] as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<[uint8_t; 200]>() as libc::c_ulong);
}
#[no_mangle]
pub unsafe extern "C" fn addBitToBuffer(mut transponder: *mut transponder_t,
                                        mut cycles: uint8_t,
                                        mut pulsewidth: uint8_t) {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < cycles as libc::c_int {
        let fresh0 = dmaBufferOffset;
        dmaBufferOffset = dmaBufferOffset.wrapping_add(1);
        (*transponder).transponderIrDMABuffer.erlt[fresh0 as usize] =
            pulsewidth;
        i += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn updateTransponderDMABufferERLT(mut transponder:
                                                            *mut transponder_t,
                                                        mut transponderData:
                                                            *const uint8_t) {
    let mut byteToSend: uint8_t =
        !(*transponderData as libc::c_int) as uint8_t;
    let mut paritysum: uint8_t = 0 as libc::c_int as uint8_t;
    dmaBufferOffset = 0 as libc::c_int as uint16_t;
    //start bit 1, always pulsed, bit value = 0
    addBitToBuffer(transponder, 10 as libc::c_int as uint8_t,
                   (*transponder).bitToggleOne as uint8_t);
    //start bit 2, always not pulsed, bit value = 0
    addBitToBuffer(transponder, 10 as libc::c_int as uint8_t,
                   0 as libc::c_int as uint8_t);
    //add data bits, only the 6 LSB
    let mut i: libc::c_int = 5 as libc::c_int;
    while i >= 0 as libc::c_int {
        let mut bv: uint8_t =
            (byteToSend as libc::c_int >> i & 0x1 as libc::c_int) as uint8_t;
        paritysum = (paritysum as libc::c_int + bv as libc::c_int) as uint8_t;
        addBitToBuffer(transponder,
                       if bv as libc::c_int != 0 {
                           25 as libc::c_int
                       } else { 10 as libc::c_int } as uint8_t,
                       if i % 2 as libc::c_int != 0 {
                           (*transponder).bitToggleOne as libc::c_int
                       } else { 0 as libc::c_int } as uint8_t);
        i -= 1
    }
    //parity bit, always pulsed, bit value is zero if sum is even, one if odd
    addBitToBuffer(transponder,
                   if paritysum as libc::c_int % 2 as libc::c_int != 0 {
                       25 as libc::c_int
                   } else { 10 as libc::c_int } as uint8_t,
                   (*transponder).bitToggleOne as uint8_t);
    //add final zero after the pulsed parity bit to stop pulses until the next update
    let fresh1 = dmaBufferOffset;
    dmaBufferOffset = dmaBufferOffset.wrapping_add(1);
    (*transponder).transponderIrDMABuffer.erlt[fresh1 as usize] =
        0 as libc::c_int as uint8_t;
    //reset buffer size to that required by this ERLT id
    (*transponder).dma_buffer_size = dmaBufferOffset as uint32_t;
}
#[no_mangle]
pub static mut erltTansponderVTable: transponderVTable =
    unsafe {
        {
            let mut init =
                transponderVTable{updateTransponderDMABuffer:
                                      Some(updateTransponderDMABufferERLT as
                                               unsafe extern "C" fn(_:
                                                                        *mut transponder_t,
                                                                    _:
                                                                        *const uint8_t)
                                                   -> ()),};
            init
        }
    };
