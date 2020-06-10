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
    pub arcitimer: [uint32_t; 620],
    pub ilap: [uint32_t; 720],
    pub erlt: [uint32_t; 200],
}
/* ** ******** ***/
/* ** ILAP ***/
// start + 8 data + stop
//720
/* ** ******** ***/
/* ** ERLT ***/
// actually ERLT is variable length 91-196 depending on the ERLT id
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
// aRCiTimer transponder codes:
//
// ID1 0x1F, 0xFC, 0x8F, 0x3, 0xF0, 0x1, 0xF8, 0x1F, 0x0           // E00370FC0FFE07E0FF
// ID2 0xFF, 0x83, 0xFF, 0xC1, 0x7, 0xFF, 0x3, 0xF0, 0x1           // 007C003EF800FC0FFE
// ID3 0x7, 0x7E, 0xE0, 0x7, 0x7E, 0xE0, 0x0, 0x38, 0x0            // F8811FF8811FFFC7FF
// ID4 0xFF, 0x83, 0xFF, 0xC1, 0x7, 0xE0, 0x7F, 0xF0, 0x1          // 007C003EF81F800FFE
// ID5 0xF, 0xF0, 0x0, 0xFF, 0x0, 0xF, 0xF0, 0xF, 0x0              // F00FFF00FFF00FF0FF
// ID6 0xFF, 0x83, 0xF, 0x3E, 0xF8, 0xE0, 0x83, 0xFF, 0xF          // 007CF0C1071F7C00F0
// ID7 0x1F, 0xFC, 0xF, 0xC0, 0xFF, 0x0, 0xFC, 0xF, 0x3E           // E003F03F00FF03F0C1
// ID8 0xFF, 0x3, 0xF0, 0x1, 0xF8, 0xE0, 0xC1, 0xFF, 0x1           // 00FC0FFE071F3E00FE
// ID9 0x1F, 0x7C, 0x40, 0xF, 0xF0, 0x61, 0xC7, 0x3F, 0x0          // E083BFF00F9E38C0FF
#[no_mangle]
pub unsafe extern "C" fn transponderIrInitArcitimer(mut transponder:
                                                        *mut transponder_t) {
    // from drivers/transponder_ir.h
    (*transponder).gap_toggles = 0 as libc::c_int as uint8_t;
    (*transponder).dma_buffer_size =
        (155 as libc::c_int * 4 as libc::c_int) as uint32_t;
    (*transponder).vTable = &arcitimerTansponderVTable;
    (*transponder).timer_hz = 24 as libc::c_int as uint32_t;
    (*transponder).timer_carrier_hz = 41886 as libc::c_int as uint32_t;
    memset(&mut (*transponder).transponderIrDMABuffer.arcitimer as
               *mut [uint32_t; 620] as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<[uint32_t; 620]>() as libc::c_ulong);
}
#[no_mangle]
pub unsafe extern "C" fn updateTransponderDMABufferArcitimer(mut transponder:
                                                                 *mut transponder_t,
                                                             mut transponderData:
                                                                 *const uint8_t) {
    let mut byteIndex: uint8_t = 0;
    let mut bitIndex: uint8_t = 0;
    let mut hightStateIndex: uint8_t = 0;
    byteIndex = 0 as libc::c_int as uint8_t;
    while (byteIndex as libc::c_int) < 9 as libc::c_int {
        let mut byteToSend: uint8_t = *transponderData;
        transponderData = transponderData.offset(1);
        bitIndex = 0 as libc::c_int as uint8_t;
        while (bitIndex as libc::c_int) < 8 as libc::c_int {
            let mut isHightState: bool =
                byteToSend as libc::c_int &
                    (1 as libc::c_int) << bitIndex as libc::c_int != 0;
            hightStateIndex = 0 as libc::c_int as uint8_t;
            while (hightStateIndex as libc::c_int) < 4 as libc::c_int {
                (*transponder).transponderIrDMABuffer.arcitimer[dmaBufferOffset
                                                                    as usize]
                    =
                    if isHightState as libc::c_int != 0 {
                        (*transponder).bitToggleOne as libc::c_int
                    } else { 0 as libc::c_int } as uint32_t;
                dmaBufferOffset = dmaBufferOffset.wrapping_add(1);
                hightStateIndex = hightStateIndex.wrapping_add(1)
            }
            bitIndex = bitIndex.wrapping_add(1)
        }
        byteIndex = byteIndex.wrapping_add(1)
    }
    dmaBufferOffset = 0 as libc::c_int as uint16_t;
}
#[no_mangle]
pub static mut arcitimerTansponderVTable: transponderVTable =
    unsafe {
        {
            let mut init =
                transponderVTable{updateTransponderDMABuffer:
                                      Some(updateTransponderDMABufferArcitimer
                                               as
                                               unsafe extern "C" fn(_:
                                                                        *mut transponder_t,
                                                                    _:
                                                                        *const uint8_t)
                                                   -> ()),};
            init
        }
    };
