use ::libc;
extern "C" {
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
    fn millis() -> timeMs_t;
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn isEscHi(selEsc: uint8_t) -> bool;
    #[no_mangle]
    fn setEscHi(selEsc: uint8_t);
    #[no_mangle]
    fn setEscLo(selEsc: uint8_t);
    #[no_mangle]
    fn setEscInput(selEsc: uint8_t);
    #[no_mangle]
    fn setEscOutput(selEsc: uint8_t);
    #[no_mangle]
    static mut selected_esc: uint8_t;
    #[no_mangle]
    fn isMcuConnected() -> bool;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ioMem_s {
    pub D_NUM_BYTES: uint8_t,
    pub D_FLASH_ADDR_H: uint8_t,
    pub D_FLASH_ADDR_L: uint8_t,
    pub D_PTR_I: *mut uint8_t,
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
/*
 * Author: 4712
*/
pub type ioMem_t = ioMem_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub union uint8_16_u {
    pub bytes: [uint8_t; 2],
    pub word: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union uint8_32_u {
    pub bytes: [uint8_t; 4],
    pub words: [uint16_t; 2],
    pub dword: uint32_t,
}
//#define STOP_BIT_TIME     ((BIT_TIME * 9) + BIT_TIME_HALVE)
unsafe extern "C" fn suart_getc_(mut bt: *mut uint8_t) -> uint8_t {
    let mut btime: uint32_t = 0;
    let mut start_time: uint32_t = 0;
    let mut wait_time: uint32_t =
        millis().wrapping_add(2 as libc::c_int as libc::c_uint);
    while isEscHi(selected_esc) {
        // check for startbit begin
        if millis() >= wait_time { return 0 as libc::c_int as uint8_t }
    }
    // start bit
    start_time = micros();
    btime =
        start_time.wrapping_add(((52 as libc::c_int >> 1 as libc::c_int) +
                                     (52 as libc::c_int >> 1 as libc::c_int >>
                                          1 as libc::c_int)) as libc::c_uint);
    let mut bitmask: uint16_t = 0 as libc::c_int as uint16_t;
    let mut bit: uint8_t = 0 as libc::c_int as uint8_t;
    while micros() < btime { }
    loop  {
        if isEscHi(selected_esc) {
            bitmask =
                (bitmask as libc::c_int |
                     (1 as libc::c_int) << bit as libc::c_int) as uint16_t
        }
        btime = btime.wrapping_add(52 as libc::c_int as libc::c_uint);
        bit = bit.wrapping_add(1);
        if bit as libc::c_int == 10 as libc::c_int { break ; }
        while micros() < btime { }
    }
    // check start bit and stop bit
    if bitmask as libc::c_int & 1 as libc::c_int != 0 ||
           bitmask as libc::c_int & (1 as libc::c_int) << 9 as libc::c_int ==
               0 {
        return 0 as libc::c_int as uint8_t
    }
    *bt = (bitmask as libc::c_int >> 1 as libc::c_int) as uint8_t;
    return 1 as libc::c_int as uint8_t;
}
unsafe extern "C" fn suart_putc_(mut tx_b: *mut uint8_t) {
    // shift out stopbit first
    let mut bitmask: uint16_t =
        ((*tx_b as libc::c_int) << 2 as libc::c_int | 1 as libc::c_int |
             (1 as libc::c_int) << 10 as libc::c_int) as uint16_t;
    let mut btime: uint32_t = micros();
    loop  {
        if bitmask as libc::c_int & 1 as libc::c_int != 0 {
            setEscHi(selected_esc);
            // 1
        } else {
            setEscLo(selected_esc);
            // 0
        } // stopbit shifted out - but don't wait
        btime = btime.wrapping_add(52 as libc::c_int as libc::c_uint);
        bitmask = (bitmask as libc::c_int >> 1 as libc::c_int) as uint16_t;
        if bitmask as libc::c_int == 0 as libc::c_int { break ; }
        while micros() < btime { }
    };
}
static mut CRC_16: uint8_16_u = uint8_16_u{bytes: [0; 2],};
static mut LastCRC_16: uint8_16_u = uint8_16_u{bytes: [0; 2],};
unsafe extern "C" fn ByteCrc(mut bt: *mut uint8_t) {
    let mut xb: uint8_t = *bt;
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < 8 as libc::c_int {
        if xb as libc::c_int & 0x1 as libc::c_int ^
               CRC_16.word as libc::c_int & 0x1 as libc::c_int !=
               0 as libc::c_int {
            CRC_16.word =
                (CRC_16.word as libc::c_int >> 1 as libc::c_int) as uint16_t;
            CRC_16.word =
                (CRC_16.word as libc::c_int ^ 0xa001 as libc::c_int) as
                    uint16_t
        } else {
            CRC_16.word =
                (CRC_16.word as libc::c_int >> 1 as libc::c_int) as uint16_t
        }
        xb = (xb as libc::c_int >> 1 as libc::c_int) as uint8_t;
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn BL_ReadBuf(mut pstring: *mut uint8_t, mut len: uint8_t)
 -> uint8_t {
    let mut current_block: u64;
    // len 0 means 256
    CRC_16.word = 0 as libc::c_int as uint16_t;
    LastCRC_16.word = 0 as libc::c_int as uint16_t;
    let mut LastACK: uint8_t = 0xff as libc::c_int as uint8_t;
    loop  {
        if suart_getc_(pstring) == 0 {
            current_block = 8884322835945113982;
            break ;
        }
        ByteCrc(pstring);
        pstring = pstring.offset(1);
        len = len.wrapping_sub(1);
        if !(len as libc::c_int > 0 as libc::c_int) {
            current_block = 7351195479953500246;
            break ;
        }
    }
    match current_block {
        7351195479953500246 => {
            if isMcuConnected() {
                //With CRC read 3 more
                if !(suart_getc_(&mut *LastCRC_16.bytes.as_mut_ptr().offset(0
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize))
                         == 0) {
                    if !(suart_getc_(&mut *LastCRC_16.bytes.as_mut_ptr().offset(1
                                                                                    as
                                                                                    libc::c_int
                                                                                    as
                                                                                    isize))
                             == 0) {
                        if !(suart_getc_(&mut LastACK) == 0) {
                            if CRC_16.word as libc::c_int !=
                                   LastCRC_16.word as libc::c_int {
                                LastACK = 0xc2 as libc::c_int as uint8_t
                            }
                        }
                    }
                }
            } else { (suart_getc_(&mut LastACK)) == 0; }
        }
        _ => { }
    }
    return (LastACK as libc::c_int == 0x30 as libc::c_int) as libc::c_int as
               uint8_t;
}
unsafe extern "C" fn BL_SendBuf(mut pstring: *mut uint8_t, mut len: uint8_t) {
    setEscOutput(selected_esc);
    CRC_16.word = 0 as libc::c_int as uint16_t;
    loop  {
        suart_putc_(pstring);
        ByteCrc(pstring);
        pstring = pstring.offset(1);
        len = len.wrapping_sub(1);
        if !(len as libc::c_int > 0 as libc::c_int) { break ; }
    }
    if isMcuConnected() {
        suart_putc_(&mut *CRC_16.bytes.as_mut_ptr().offset(0 as libc::c_int as
                                                               isize));
        suart_putc_(&mut *CRC_16.bytes.as_mut_ptr().offset(1 as libc::c_int as
                                                               isize));
    }
    setEscInput(selected_esc);
}
#[no_mangle]
pub unsafe extern "C" fn BL_ConnectEx(mut pDeviceInfo: *mut uint8_32_u)
 -> uint8_t {
    //DeviceInfo.dword=0; is set before
    let mut BootInfo: [uint8_t; 9] = [0; 9];
    let mut BootMsg: [uint8_t; 3] =
        *::core::mem::transmute::<&[u8; 3], &mut [uint8_t; 3]>(b"471");
    // x * 0 + 9
    let mut BootInit: [uint8_t; 21] =
        [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0xd as libc::c_int as uint8_t, 'B' as i32 as uint8_t,
         'L' as i32 as uint8_t, 'H' as i32 as uint8_t, 'e' as i32 as uint8_t,
         'l' as i32 as uint8_t, 'i' as i32 as uint8_t,
         0xf4 as libc::c_int as uint8_t, 0x7d as libc::c_int as uint8_t];
    BL_SendBuf(BootInit.as_mut_ptr(), 21 as libc::c_int as uint8_t);
    if BL_ReadBuf(BootInfo.as_mut_ptr(),
                  (4 as libc::c_int + 4 as libc::c_int) as uint8_t) == 0 {
        return 0 as libc::c_int as uint8_t
    }
    // BootInfo has no CRC  (ACK byte already analyzed... )
    // Format = BootMsg("471c") SIGNATURE_001, SIGNATURE_002, BootVersion (always 6), BootPages (,ACK)
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < 4 as libc::c_int - 1 as libc::c_int {
        // Check only the first 3 letters -> 471x OK
        if BootInfo[i as usize] as libc::c_int !=
               BootMsg[i as usize] as libc::c_int {
            return 0 as libc::c_int as uint8_t
        }
        i = i.wrapping_add(1)
    }
    //only 2 bytes used $1E9307 -> 0x9307
    (*pDeviceInfo).bytes[2 as libc::c_int as usize] =
        BootInfo[(4 as libc::c_int - 1 as libc::c_int) as
                     usize]; //sends simply 4 x 0x00 (CRC =00)
    (*pDeviceInfo).bytes[1 as libc::c_int as usize] =
        BootInfo[4 as libc::c_int as usize];
    (*pDeviceInfo).bytes[0 as libc::c_int as usize] =
        BootInfo[(4 as libc::c_int + 1 as libc::c_int) as usize];
    return 1 as libc::c_int as uint8_t;
}
unsafe extern "C" fn BL_GetACK(mut Timeout: uint32_t) -> uint8_t {
    let mut LastACK: uint8_t = 0xff as libc::c_int as uint8_t;
    while suart_getc_(&mut LastACK) == 0 && Timeout != 0 {
        Timeout = Timeout.wrapping_sub(1)
    }
    return LastACK;
}
#[no_mangle]
pub unsafe extern "C" fn BL_SendCMDKeepAlive() -> uint8_t {
    let mut sCMD: [uint8_t; 2] =
        [0xfd as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
    BL_SendBuf(sCMD.as_mut_ptr(), 2 as libc::c_int as uint8_t);
    if BL_GetACK(1 as libc::c_int as uint32_t) as libc::c_int !=
           0xc1 as libc::c_int {
        return 0 as libc::c_int as uint8_t
    }
    return 1 as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn BL_SendCMDRunRestartBootloader(mut pDeviceInfo:
                                                            *mut uint8_32_u) {
    let mut sCMD: [uint8_t; 2] =
        [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
    (*pDeviceInfo).bytes[0 as libc::c_int as usize] =
        1 as libc::c_int as uint8_t;
    BL_SendBuf(sCMD.as_mut_ptr(), 2 as libc::c_int as uint8_t);
}
unsafe extern "C" fn BL_SendCMDSetAddress(mut pMem: *mut ioMem_t) -> uint8_t 
 //supports only 16 bit Adr
 {
    // skip if adr == 0xFFFF
    if (*pMem).D_FLASH_ADDR_H as libc::c_int == 0xff as libc::c_int &&
           (*pMem).D_FLASH_ADDR_L as libc::c_int == 0xff as libc::c_int {
        return 1 as libc::c_int as uint8_t
    }
    let mut sCMD: [uint8_t; 4] =
        [0xff as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         (*pMem).D_FLASH_ADDR_H, (*pMem).D_FLASH_ADDR_L];
    BL_SendBuf(sCMD.as_mut_ptr(), 4 as libc::c_int as uint8_t);
    return (BL_GetACK(2 as libc::c_int as uint32_t) as libc::c_int ==
                0x30 as libc::c_int) as libc::c_int as uint8_t;
}
unsafe extern "C" fn BL_SendCMDSetBuffer(mut pMem: *mut ioMem_t) -> uint8_t {
    let mut sCMD: [uint8_t; 4] =
        [0xfe as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, (*pMem).D_NUM_BYTES];
    if (*pMem).D_NUM_BYTES as libc::c_int == 0 as libc::c_int {
        // set high byte
        sCMD[2 as libc::c_int as usize] = 1 as libc::c_int as uint8_t
    }
    BL_SendBuf(sCMD.as_mut_ptr(), 4 as libc::c_int as uint8_t);
    if BL_GetACK(2 as libc::c_int as uint32_t) as libc::c_int !=
           0xff as libc::c_int {
        return 0 as libc::c_int as uint8_t
    }
    BL_SendBuf((*pMem).D_PTR_I, (*pMem).D_NUM_BYTES);
    return (BL_GetACK(40 as libc::c_int as uint32_t) as libc::c_int ==
                0x30 as libc::c_int) as libc::c_int as uint8_t;
}
unsafe extern "C" fn BL_ReadA(mut cmd: uint8_t, mut pMem: *mut ioMem_t)
 -> uint8_t {
    if BL_SendCMDSetAddress(pMem) != 0 {
        let mut sCMD: [uint8_t; 2] = [cmd, (*pMem).D_NUM_BYTES];
        BL_SendBuf(sCMD.as_mut_ptr(), 2 as libc::c_int as uint8_t);
        return BL_ReadBuf((*pMem).D_PTR_I, (*pMem).D_NUM_BYTES)
    }
    return 0 as libc::c_int as uint8_t;
}
unsafe extern "C" fn BL_WriteA(mut cmd: uint8_t, mut pMem: *mut ioMem_t,
                               mut timeout: uint32_t) -> uint8_t {
    if BL_SendCMDSetAddress(pMem) != 0 {
        if BL_SendCMDSetBuffer(pMem) == 0 {
            return 0 as libc::c_int as uint8_t
        }
        let mut sCMD: [uint8_t; 2] = [cmd, 0x1 as libc::c_int as uint8_t];
        BL_SendBuf(sCMD.as_mut_ptr(), 2 as libc::c_int as uint8_t);
        return (BL_GetACK(timeout) as libc::c_int == 0x30 as libc::c_int) as
                   libc::c_int as uint8_t
    }
    return 0 as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn BL_ReadFlash(mut interface_mode: uint8_t,
                                      mut pMem: *mut ioMem_t) -> uint8_t {
    if interface_mode as libc::c_int == 2 as libc::c_int {
        return BL_ReadA(0x7 as libc::c_int as uint8_t, pMem)
    } else { return BL_ReadA(0x3 as libc::c_int as uint8_t, pMem) };
}
#[no_mangle]
pub unsafe extern "C" fn BL_ReadEEprom(mut pMem: *mut ioMem_t) -> uint8_t {
    return BL_ReadA(0x4 as libc::c_int as uint8_t, pMem);
}
#[no_mangle]
pub unsafe extern "C" fn BL_PageErase(mut pMem: *mut ioMem_t) -> uint8_t {
    if BL_SendCMDSetAddress(pMem) != 0 {
        let mut sCMD: [uint8_t; 2] =
            [0x2 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t];
        BL_SendBuf(sCMD.as_mut_ptr(), 2 as libc::c_int as uint8_t);
        return (BL_GetACK((1400 as libc::c_int / 2 as libc::c_int) as
                              uint32_t) as libc::c_int == 0x30 as libc::c_int)
                   as libc::c_int as uint8_t
    }
    return 0 as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn BL_WriteEEprom(mut pMem: *mut ioMem_t) -> uint8_t {
    return BL_WriteA(0x5 as libc::c_int as uint8_t, pMem,
                     (3000 as libc::c_int / 2 as libc::c_int) as uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn BL_WriteFlash(mut pMem: *mut ioMem_t) -> uint8_t {
    return BL_WriteA(0x1 as libc::c_int as uint8_t, pMem,
                     (500 as libc::c_int / 2 as libc::c_int) as uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn BL_VerifyFlash(mut pMem: *mut ioMem_t) -> uint8_t {
    if BL_SendCMDSetAddress(pMem) != 0 {
        if BL_SendCMDSetBuffer(pMem) == 0 {
            return 0 as libc::c_int as uint8_t
        }
        let mut sCMD: [uint8_t; 2] =
            [0x4 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t];
        BL_SendBuf(sCMD.as_mut_ptr(), 2 as libc::c_int as uint8_t);
        return BL_GetACK((40 as libc::c_int / 2 as libc::c_int) as uint32_t)
    }
    return 0 as libc::c_int as uint8_t;
}
