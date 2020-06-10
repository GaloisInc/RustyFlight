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
    fn delayMicroseconds(us: timeUs_t);
    #[no_mangle]
    fn micros() -> timeUs_t;
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
    #[no_mangle]
    static mut selected_esc: uint8_t;
    #[no_mangle]
    fn isEscHi(selEsc: uint8_t) -> bool;
    #[no_mangle]
    fn isEscLo(selEsc: uint8_t) -> bool;
    #[no_mangle]
    fn setEscHi(selEsc: uint8_t);
    #[no_mangle]
    fn setEscLo(selEsc: uint8_t);
    #[no_mangle]
    fn setEscInput(selEsc: uint8_t);
    #[no_mangle]
    fn setEscOutput(selEsc: uint8_t);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
static mut StkInBuf: [uint8_t; 16] = [0; 16];
//5 s
static mut LastBitTime: uint32_t = 0;
static mut HiLoTsh: uint32_t = 0;
static mut SeqNumber: uint8_t = 0;
static mut StkCmd: uint8_t = 0;
static mut ckSumIn: uint8_t = 0;
static mut ckSumOut: uint8_t = 0;
// dummy
// dummy
unsafe extern "C" fn StkSendByte(mut dat: uint8_t) {
    ckSumOut = (ckSumOut as libc::c_int ^ dat as libc::c_int) as uint8_t;
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < 8 as libc::c_int {
        if dat as libc::c_int & 0x1 as libc::c_int != 0 {
            // 1-bits are encoded as 64.0us high, 72.8us low (135.8us total).
            setEscHi(selected_esc);
            delayMicroseconds((2 as libc::c_int * 32 as libc::c_int) as
                                  timeUs_t);
            setEscLo(selected_esc);
            delayMicroseconds((2 as libc::c_int * 32 as libc::c_int) as
                                  timeUs_t);
        } else {
            // 0-bits are encoded as 27.8us high, 34.5us low, 34.4us high, 37.9 low (134.6us total)
            setEscHi(selected_esc);
            delayMicroseconds(32 as libc::c_int as timeUs_t);
            setEscLo(selected_esc);
            delayMicroseconds(32 as libc::c_int as timeUs_t);
            setEscHi(selected_esc);
            delayMicroseconds(32 as libc::c_int as timeUs_t);
            setEscLo(selected_esc);
            delayMicroseconds(32 as libc::c_int as timeUs_t);
        }
        dat = (dat as libc::c_int >> 1 as libc::c_int) as uint8_t;
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn StkSendPacketHeader() {
    setEscOutput(selected_esc);
    StkSendByte(0xff as libc::c_int as uint8_t);
    StkSendByte(0xff as libc::c_int as uint8_t);
    StkSendByte(0x7f as libc::c_int as uint8_t);
    ckSumOut = 0 as libc::c_int as uint8_t;
    StkSendByte(0x1b as libc::c_int as uint8_t);
    SeqNumber = SeqNumber.wrapping_add(1);
    StkSendByte(SeqNumber);
}
unsafe extern "C" fn StkSendPacketFooter() {
    StkSendByte(ckSumOut);
    setEscHi(selected_esc);
    delayMicroseconds(32 as libc::c_int as timeUs_t);
    setEscInput(selected_esc);
}
unsafe extern "C" fn ReadBit() -> int8_t {
    let mut current_block: u64;
    let mut btimer: uint32_t = micros();
    let mut timeout_timer: uint32_t =
        btimer.wrapping_add(250 as libc::c_int as libc::c_uint);
    loop  {
        if !isEscHi(selected_esc) {
            current_block = 7095457783677275021;
            break ;
        }
        if micros() > timeout_timer {
            current_block = 13972525247421188594;
            break ;
        }
    }
    match current_block {
        7095457783677275021 => {
            loop  {
                if !isEscLo(selected_esc) {
                    current_block = 8515828400728868193;
                    break ;
                }
                if micros() > timeout_timer {
                    current_block = 13972525247421188594;
                    break ;
                }
            }
            match current_block {
                13972525247421188594 => { }
                _ => {
                    LastBitTime = micros().wrapping_sub(btimer);
                    if LastBitTime <= HiLoTsh {
                        timeout_timer =
                            timeout_timer.wrapping_add(250 as libc::c_int as
                                                           libc::c_uint);
                        loop  {
                            if !isEscHi(selected_esc) {
                                current_block = 9606288038608642794;
                                break ;
                            }
                            if micros() > timeout_timer {
                                current_block = 13972525247421188594;
                                break ;
                            }
                        }
                        match current_block {
                            13972525247421188594 => { }
                            _ => {
                                loop  {
                                    if !isEscLo(selected_esc) {
                                        current_block = 6057473163062296781;
                                        break ;
                                    }
                                    if micros() > timeout_timer {
                                        current_block = 13972525247421188594;
                                        break ;
                                    }
                                }
                                match current_block {
                                    13972525247421188594 => { }
                                    _ => {
                                        //lo-bit
                                        return 0 as libc::c_int as int8_t
                                    }
                                }
                            }
                        }
                    } else { return 1 as libc::c_int as int8_t }
                }
            }
        }
        _ => { }
    }
    return -(1 as libc::c_int) as int8_t;
}
unsafe extern "C" fn ReadByte(mut bt: *mut uint8_t) -> uint8_t {
    let mut current_block: u64;
    *bt = 0 as libc::c_int as uint8_t;
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    loop  {
        if !((i as libc::c_int) < 8 as libc::c_int) {
            current_block = 6937071982253665452;
            break ;
        }
        let mut bit: int8_t = ReadBit();
        if bit as libc::c_int == -(1 as libc::c_int) {
            current_block = 3620398755094719438;
            break ;
        }
        if bit as libc::c_int == 1 as libc::c_int {
            *bt =
                (*bt as libc::c_int | (1 as libc::c_int) << i as libc::c_int)
                    as uint8_t
        }
        i = i.wrapping_add(1)
    }
    match current_block {
        3620398755094719438 => { return 0 as libc::c_int as uint8_t }
        _ => {
            ckSumIn =
                (ckSumIn as libc::c_int ^ *bt as libc::c_int) as uint8_t;
            return 1 as libc::c_int as uint8_t
        }
    };
}
unsafe extern "C" fn StkReadLeader() -> uint8_t {
    let mut bit: int8_t = 0;
    let mut current_block: u64;
    // Reset learned timing
    HiLoTsh =
        (2 as libc::c_int * 32 as libc::c_int + 32 as libc::c_int) as
            uint32_t;
    // Wait for the first bit
    let mut waitcycl: uint32_t = 0; //250uS each
    if StkCmd as libc::c_int == 0x15 as libc::c_int ||
           StkCmd as libc::c_int == 0x12 as libc::c_int {
        waitcycl =
            (1000 as libc::c_int / 250 as libc::c_int * 5000 as libc::c_int)
                as uint32_t
    } else if StkCmd as libc::c_int == 0x1 as libc::c_int {
        waitcycl =
            (1000 as libc::c_int / 250 as libc::c_int / 2 as libc::c_int) as
                uint32_t
    } else {
        waitcycl =
            (1000 as libc::c_int / 250 as libc::c_int * 35 as libc::c_int) as
                uint32_t
    }
    while waitcycl > 0 as libc::c_int as libc::c_uint {
        //check is not timeout
        if ReadBit() as libc::c_int > -(1 as libc::c_int) { break ; }
        waitcycl = waitcycl.wrapping_sub(1)
    }
    //Skip the first bits
    if !(waitcycl == 0 as libc::c_int as libc::c_uint) {
        let mut i: uint8_t = 0 as libc::c_int as uint8_t;
        loop  {
            if !((i as libc::c_int) < 10 as libc::c_int) {
                current_block = 8457315219000651999;
                break ;
            }
            if ReadBit() as libc::c_int == -(1 as libc::c_int) {
                current_block = 8311509850290084810;
                break ;
            }
            i = i.wrapping_add(1)
        }
        match current_block {
            8311509850290084810 => { }
            _ => {
                // learn timing
                HiLoTsh =
                    (LastBitTime >>
                         1 as
                             libc::c_int).wrapping_add(LastBitTime >>
                                                           2 as libc::c_int);
                // Read until we get a 0 bit
                bit = 0; // hi byte Msg len
                loop  {
                    bit = ReadBit(); // lo byte Msg len
                    if bit as libc::c_int == -(1 as libc::c_int) {
                        current_block = 8311509850290084810; // NumTX
                        break ; // NumRX
                    } // RxStartAdr
                    if !(bit as libc::c_int > 0 as libc::c_int) {
                        current_block = 14401909646449704462; // {TxData} Cmd
                        break ; // {TxData} AdrHi
                    }
                } // {TxData} AdrLoch
                match current_block {
                    8311509850290084810 => { }
                    _ => { return 1 as libc::c_int as uint8_t }
                }
            }
        }
    } // {TxData} 0
    return 0 as libc::c_int as uint8_t;
}
unsafe extern "C" fn StkRcvPacket(mut pstring: *mut uint8_t) -> uint8_t {
    let mut current_block: u64;
    let mut bt: uint8_t = 0 as libc::c_int as uint8_t;
    let mut Len: uint8_16_u = uint8_16_u{bytes: [0; 2],};
    if !(StkReadLeader() == 0) {
        ckSumIn = 0 as libc::c_int as uint8_t;
        if !(ReadByte(&mut bt) == 0 ||
                 bt as libc::c_int != 0x1b as libc::c_int) {
            if !(ReadByte(&mut bt) == 0 ||
                     bt as libc::c_int != SeqNumber as libc::c_int) {
                ReadByte(&mut *Len.bytes.as_mut_ptr().offset(1 as libc::c_int
                                                                 as isize));
                if !(Len.bytes[1 as libc::c_int as usize] as libc::c_int >
                         1 as libc::c_int) {
                    ReadByte(&mut *Len.bytes.as_mut_ptr().offset(0 as
                                                                     libc::c_int
                                                                     as
                                                                     isize));
                    if !((Len.bytes[0 as libc::c_int as usize] as libc::c_int)
                             < 1 as libc::c_int) {
                        if !(ReadByte(&mut bt) == 0 ||
                                 bt as libc::c_int != 0xe as libc::c_int) {
                            if !(ReadByte(&mut bt) == 0 ||
                                     bt as libc::c_int !=
                                         StkCmd as libc::c_int) {
                                if !(ReadByte(&mut bt) == 0 ||
                                         bt as libc::c_int !=
                                             0 as libc::c_int) {
                                    let mut i: uint16_t =
                                        0 as libc::c_int as uint16_t;
                                    loop  {
                                        if !((i as libc::c_int) <
                                                 Len.word as libc::c_int -
                                                     2 as libc::c_int) {
                                            current_block =
                                                5948590327928692120;
                                            break ;
                                        }
                                        if ReadByte(pstring) == 0 {
                                            current_block =
                                                14401909646449704462;
                                            break ;
                                        }
                                        pstring = pstring.offset(1);
                                        i = i.wrapping_add(1)
                                    }
                                    match current_block {
                                        14401909646449704462 => { }
                                        _ => {
                                            ReadByte(&mut bt);
                                            if !(ckSumIn as libc::c_int !=
                                                     0 as libc::c_int) {
                                                return 1 as libc::c_int as
                                                           uint8_t
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return 0 as libc::c_int as uint8_t;
}
unsafe extern "C" fn _CMD_SPI_MULTI_EX(mut ResByte: *mut uint8_t,
                                       mut Cmd: uint8_t, mut AdrHi: uint8_t,
                                       mut AdrLo: uint8_t) -> uint8_t {
    StkCmd = 0x1d as libc::c_int as uint8_t;
    StkSendPacketHeader();
    StkSendByte(0 as libc::c_int as uint8_t);
    StkSendByte(8 as libc::c_int as uint8_t);
    StkSendByte(0xe as libc::c_int as uint8_t);
    StkSendByte(0x1d as libc::c_int as uint8_t);
    StkSendByte(4 as libc::c_int as uint8_t);
    StkSendByte(4 as libc::c_int as uint8_t);
    StkSendByte(0 as libc::c_int as uint8_t);
    StkSendByte(Cmd);
    StkSendByte(AdrHi);
    StkSendByte(AdrLo);
    StkSendByte(0 as libc::c_int as uint8_t);
    StkSendPacketFooter();
    if StkRcvPacket(StkInBuf.as_mut_ptr() as *mut libc::c_void as
                        *mut uint8_t) != 0 {
        // NumRX + 3
        if StkInBuf[0 as libc::c_int as usize] as libc::c_int ==
               0 as libc::c_int &&
               (StkInBuf[1 as libc::c_int as usize] as libc::c_int ==
                    Cmd as libc::c_int ||
                    StkInBuf[1 as libc::c_int as usize] as libc::c_int ==
                        0 as libc::c_int) &&
               StkInBuf[2 as libc::c_int as usize] as libc::c_int ==
                   0 as libc::c_int {
            ::core::ptr::write_volatile(ResByte,
                                        StkInBuf[3 as libc::c_int as usize])
        }
        return 1 as libc::c_int as uint8_t
    }
    return 0 as libc::c_int as uint8_t;
}
unsafe extern "C" fn _CMD_LOAD_ADDRESS(mut pMem: *mut ioMem_t) -> uint8_t {
    // ignore 0xFFFF
    // assume address is set before and we read or write the immediately following package
    if (*pMem).D_FLASH_ADDR_H as libc::c_int == 0xff as libc::c_int &&
           (*pMem).D_FLASH_ADDR_L as libc::c_int == 0xff as libc::c_int {
        return 1 as libc::c_int as uint8_t
    } // hi byte Msg len
    StkCmd = 0x6 as libc::c_int as uint8_t; // lo byte Msg len
    StkSendPacketHeader(); // hi byte Msg len
    StkSendByte(0 as libc::c_int as uint8_t); // lo byte Msg len
    StkSendByte(5 as libc::c_int as uint8_t); // high byte Msg len
    StkSendByte(0xe as libc::c_int as uint8_t); // low byte Msg len
    StkSendByte(0x6 as libc::c_int as uint8_t); // mode
    StkSendByte(0 as libc::c_int as uint8_t); // delay
    StkSendByte(0 as libc::c_int as uint8_t); // cmd1
    StkSendByte((*pMem).D_FLASH_ADDR_H); // cmd2
    StkSendByte((*pMem).D_FLASH_ADDR_L); // cmd3
    StkSendPacketFooter(); // poll1
    return StkRcvPacket(StkInBuf.as_mut_ptr() as *mut libc::c_void as
                            *mut uint8_t); // poll2
}
unsafe extern "C" fn _CMD_READ_MEM_ISP(mut pMem: *mut ioMem_t) -> uint8_t {
    let mut LenHi: uint8_t = 0;
    if (*pMem).D_NUM_BYTES as libc::c_int > 0 as libc::c_int {
        LenHi = 0 as libc::c_int as uint8_t
    } else { LenHi = 1 as libc::c_int as uint8_t }
    StkSendPacketHeader();
    StkSendByte(0 as libc::c_int as uint8_t);
    StkSendByte(4 as libc::c_int as uint8_t);
    StkSendByte(0xe as libc::c_int as uint8_t);
    StkSendByte(StkCmd);
    StkSendByte(LenHi);
    StkSendByte((*pMem).D_NUM_BYTES);
    StkSendByte(0xa0 as libc::c_int as uint8_t);
    StkSendPacketFooter();
    return StkRcvPacket((*pMem).D_PTR_I);
}
unsafe extern "C" fn _CMD_PROGRAM_MEM_ISP(mut pMem: *mut ioMem_t) -> uint8_t {
    let mut Len: uint8_16_u = uint8_16_u{bytes: [0; 2],};
    let mut LenLo: uint8_t = (*pMem).D_NUM_BYTES;
    let mut LenHi: uint8_t = 0;
    if LenLo != 0 {
        LenHi = 0 as libc::c_int as uint8_t;
        Len.word = (LenLo as libc::c_int + 10 as libc::c_int) as uint16_t
    } else {
        LenHi = 1 as libc::c_int as uint8_t;
        Len.word = (256 as libc::c_int + 10 as libc::c_int) as uint16_t
    }
    StkSendPacketHeader();
    StkSendByte(Len.bytes[1 as libc::c_int as usize]);
    StkSendByte(Len.bytes[0 as libc::c_int as usize]);
    StkSendByte(0xe as libc::c_int as uint8_t);
    StkSendByte(StkCmd);
    StkSendByte(LenHi);
    StkSendByte(LenLo);
    StkSendByte(0 as libc::c_int as uint8_t);
    StkSendByte(0 as libc::c_int as uint8_t);
    StkSendByte(0 as libc::c_int as uint8_t);
    StkSendByte(0 as libc::c_int as uint8_t);
    StkSendByte(0 as libc::c_int as uint8_t);
    StkSendByte(0 as libc::c_int as uint8_t);
    StkSendByte(0 as libc::c_int as uint8_t);
    loop  {
        StkSendByte(*(*pMem).D_PTR_I);
        (*pMem).D_PTR_I = (*pMem).D_PTR_I.offset(1);
        LenLo = LenLo.wrapping_sub(1);
        if !(LenLo != 0) { break ; }
    }
    StkSendPacketFooter();
    return StkRcvPacket(StkInBuf.as_mut_ptr() as *mut libc::c_void as
                            *mut uint8_t);
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
#[no_mangle]
pub unsafe extern "C" fn Stk_SignOn() -> uint8_t {
    StkCmd = 0x1 as libc::c_int as uint8_t; // hi byte Msg len
    StkSendPacketHeader(); // lo byte Msg len
    StkSendByte(0 as libc::c_int as uint8_t); // high byte Msg len
    StkSendByte(1 as libc::c_int as uint8_t); // low byte Msg len
    StkSendByte(0xe as libc::c_int as
                    uint8_t); // ChipErase_eraseDelay atmega8
    StkSendByte(0x1 as libc::c_int as
                    uint8_t); // ChipErase_pollMethod atmega8
    StkSendPacketFooter();
    return StkRcvPacket(StkInBuf.as_mut_ptr() as *mut libc::c_void as
                            *mut uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn Stk_ConnectEx(mut pDeviceInfo: *mut uint8_32_u)
 -> uint8_t {
    if Stk_SignOn() != 0 {
        if _CMD_SPI_MULTI_EX(&mut *(*pDeviceInfo).bytes.as_mut_ptr().offset(1
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                 as *mut uint8_t as *mut uint8_t,
                             0x30 as libc::c_int as uint8_t,
                             0 as libc::c_int as uint8_t,
                             1 as libc::c_int as uint8_t) != 0 {
            if _CMD_SPI_MULTI_EX(&mut *(*pDeviceInfo).bytes.as_mut_ptr().offset(0
                                                                                    as
                                                                                    libc::c_int
                                                                                    as
                                                                                    isize)
                                     as *mut uint8_t as *mut uint8_t,
                                 0x30 as libc::c_int as uint8_t,
                                 0 as libc::c_int as uint8_t,
                                 2 as libc::c_int as uint8_t) != 0 {
                return 1 as libc::c_int as uint8_t
            }
        }
    }
    return 0 as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn Stk_Chip_Erase() -> uint8_t {
    StkCmd = 0x12 as libc::c_int as uint8_t;
    StkSendPacketHeader();
    StkSendByte(0 as libc::c_int as uint8_t);
    StkSendByte(7 as libc::c_int as uint8_t);
    StkSendByte(0xe as libc::c_int as uint8_t);
    StkSendByte(0x12 as libc::c_int as uint8_t);
    StkSendByte(20 as libc::c_int as uint8_t);
    StkSendByte(0 as libc::c_int as uint8_t);
    StkSendByte(0xac as libc::c_int as uint8_t);
    StkSendByte(0x88 as libc::c_int as uint8_t);
    StkSendByte(0x13 as libc::c_int as uint8_t);
    StkSendByte(0x76 as libc::c_int as uint8_t);
    StkSendPacketFooter();
    return StkRcvPacket(StkInBuf.as_mut_ptr());
}
#[no_mangle]
pub unsafe extern "C" fn Stk_ReadFlash(mut pMem: *mut ioMem_t) -> uint8_t {
    if _CMD_LOAD_ADDRESS(pMem) != 0 {
        StkCmd = 0x14 as libc::c_int as uint8_t;
        return _CMD_READ_MEM_ISP(pMem)
    }
    return 0 as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn Stk_ReadEEprom(mut pMem: *mut ioMem_t) -> uint8_t {
    if _CMD_LOAD_ADDRESS(pMem) != 0 {
        StkCmd = 0x16 as libc::c_int as uint8_t;
        return _CMD_READ_MEM_ISP(pMem)
    }
    return 0 as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn Stk_WriteFlash(mut pMem: *mut ioMem_t) -> uint8_t {
    if _CMD_LOAD_ADDRESS(pMem) != 0 {
        StkCmd = 0x13 as libc::c_int as uint8_t;
        return _CMD_PROGRAM_MEM_ISP(pMem)
    }
    return 0 as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn Stk_WriteEEprom(mut pMem: *mut ioMem_t) -> uint8_t {
    if _CMD_LOAD_ADDRESS(pMem) != 0 {
        StkCmd = 0x15 as libc::c_int as uint8_t;
        return _CMD_PROGRAM_MEM_ISP(pMem)
    }
    return 0 as libc::c_int as uint8_t;
}
