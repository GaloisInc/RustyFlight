use core;
use libc;
extern "C" {
    #[no_mangle]
    fn serialWrite(instance: *mut serialPort_t, ch: uint8_t);
    #[no_mangle]
    fn isSerialTransmitBufferEmpty(instance: *const serialPort_t) -> bool;
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
    fn uli2a(num: libc::c_ulong, base: libc::c_uint, uc: libc::c_int,
             bf: *mut libc::c_char);
    #[no_mangle]
    fn li2a(num: libc::c_long, bf: *mut libc::c_char);
    #[no_mangle]
    fn ui2a(num: libc::c_uint, base: libc::c_uint, uc: libc::c_int,
            bf: *mut libc::c_char);
    #[no_mangle]
    fn i2a(num: libc::c_int, bf: *mut libc::c_char);
    #[no_mangle]
    fn a2i(ch: libc::c_char, src: *mut *const libc::c_char, base: libc::c_int,
           nump: *mut libc::c_int) -> libc::c_char;
}
pub type __builtin_va_list = [__va_list_tag; 1];
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct __va_list_tag {
    pub gp_offset: libc::c_uint,
    pub fp_offset: libc::c_uint,
    pub overflow_arg_area: *mut libc::c_void,
    pub reg_save_area: *mut libc::c_void,
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type va_list = __builtin_va_list;
pub type portMode_e = libc::c_uint;
pub const MODE_RXTX: portMode_e = 3;
pub const MODE_TX: portMode_e = 2;
pub const MODE_RX: portMode_e = 1;
pub type portOptions_e = libc::c_uint;
pub const SERIAL_BIDIR_NOPULL: portOptions_e = 32;
pub const SERIAL_BIDIR_PP: portOptions_e = 16;
pub const SERIAL_BIDIR_OD: portOptions_e = 0;
pub const SERIAL_BIDIR: portOptions_e = 8;
pub const SERIAL_UNIDIR: portOptions_e = 0;
pub const SERIAL_PARITY_EVEN: portOptions_e = 4;
pub const SERIAL_PARITY_NO: portOptions_e = 0;
pub const SERIAL_STOPBITS_2: portOptions_e = 2;
pub const SERIAL_STOPBITS_1: portOptions_e = 0;
pub const SERIAL_INVERTED: portOptions_e = 1;
pub const SERIAL_NOT_INVERTED: portOptions_e = 0;
// Define known line control states which may be passed up by underlying serial driver callback
pub type serialReceiveCallbackPtr
    =
    Option<unsafe extern "C" fn(_: uint16_t, _: *mut libc::c_void) -> ()>;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct serialPort_s {
    pub vTable: *const serialPortVTable,
    pub mode: portMode_e,
    pub options: portOptions_e,
    pub baudRate: uint32_t,
    pub rxBufferSize: uint32_t,
    pub txBufferSize: uint32_t,
    pub rxBuffer: *mut uint8_t,
    pub txBuffer: *mut uint8_t,
    pub rxBufferHead: uint32_t,
    pub rxBufferTail: uint32_t,
    pub txBufferHead: uint32_t,
    pub txBufferTail: uint32_t,
    pub rxCallback: serialReceiveCallbackPtr,
    pub rxCallbackData: *mut libc::c_void,
    pub identifier: uint8_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct serialPortVTable {
    pub serialWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                 _: uint8_t) -> ()>,
    pub serialTotalRxWaiting: Option<unsafe extern "C" fn(_:
                                                              *const serialPort_t)
                                         -> uint32_t>,
    pub serialTotalTxFree: Option<unsafe extern "C" fn(_: *const serialPort_t)
                                      -> uint32_t>,
    pub serialRead: Option<unsafe extern "C" fn(_: *mut serialPort_t)
                               -> uint8_t>,
    pub serialSetBaudRate: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                       _: uint32_t) -> ()>,
    pub isSerialTransmitBufferEmpty: Option<unsafe extern "C" fn(_:
                                                                     *const serialPort_t)
                                                -> bool>,
    pub setMode: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                             _: portMode_e) -> ()>,
    pub setCtrlLineStateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                        _:
                                                            Option<unsafe extern "C" fn(_:
                                                                                            *mut libc::c_void,
                                                                                        _:
                                                                                            uint16_t)
                                                                       -> ()>,
                                                        _: *mut libc::c_void)
                                       -> ()>,
    pub setBaudRateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                   _:
                                                       Option<unsafe extern "C" fn(_:
                                                                                       *mut serialPort_t,
                                                                                   _:
                                                                                       uint32_t)
                                                                  -> ()>,
                                                   _: *mut serialPort_t)
                                  -> ()>,
    pub writeBuf: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                              _: *const libc::c_void,
                                              _: libc::c_int) -> ()>,
    pub beginWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
    pub endWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
}
// used by serial drivers to return frames to app
pub type serialPort_t = serialPort_s;
pub type putcf
    =
    Option<unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_char) -> ()>;
/*
 * Copyright (c) 2004,2012 Kustaa Nyholm / SpareTimeLabs
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution.
 *
 * Neither the name of the Kustaa Nyholm or SpareTimeLabs nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */
static mut printfSerialPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
static mut stdout_putf: putcf = None;
static mut stdout_putp: *mut libc::c_void =
    0 as *const libc::c_void as *mut libc::c_void;
// print bf, padded from left to at least n characters.
// padding is zero ('0') if z!=0, space (' ') otherwise
unsafe extern "C" fn putchw(mut putp: *mut libc::c_void, mut putf: putcf,
                            mut n: libc::c_int, mut z: libc::c_char,
                            mut bf: *mut libc::c_char) -> libc::c_int {
    let mut written: libc::c_int = 0i32;
    let mut fc: libc::c_char =
        if z as libc::c_int != 0 { '0' as i32 } else { ' ' as i32 } as
            libc::c_char;
    let mut ch: libc::c_char = 0;
    let mut p: *mut libc::c_char = bf;
    loop  {
        let fresh0 = p;
        p = p.offset(1);
        if !(*fresh0 as libc::c_int != 0 && n > 0i32) { break ; }
        n -= 1
    }
    loop  {
        let fresh1 = n;
        n = n - 1;
        if !(fresh1 > 0i32) { break ; }
        putf.expect("non-null function pointer")(putp, fc);
        written += 1
    }
    loop  {
        let fresh2 = bf;
        bf = bf.offset(1);
        ch = *fresh2;
        if !(ch != 0) { break ; }
        putf.expect("non-null function pointer")(putp, ch);
        written += 1
    }
    return written;
}
// retrun number of bytes written
#[no_mangle]
pub unsafe extern "C" fn tfp_format(mut putp: *mut libc::c_void,
                                    mut putf: putcf,
                                    mut fmt: *const libc::c_char,
                                    mut va: ::core::ffi::VaList)
 -> libc::c_int {
    let mut bf: [libc::c_char; 12] = [0; 12];
    let mut written: libc::c_int = 0i32;
    let mut ch: libc::c_char = 0;
    loop  {
        let fresh3 = fmt;
        fmt = fmt.offset(1);
        ch = *fresh3;
        if !(ch != 0) { break ; }
        if ch as libc::c_int != '%' as i32 {
            putf.expect("non-null function pointer")(putp, ch);
            written += 1
        } else {
            let mut lz: libc::c_char = 0i32 as libc::c_char;
            let mut lng: libc::c_char = 0i32 as libc::c_char;
            let mut w: libc::c_int = 0i32;
            let fresh4 = fmt;
            fmt = fmt.offset(1);
            ch = *fresh4;
            if ch as libc::c_int == '0' as i32 {
                let fresh5 = fmt;
                fmt = fmt.offset(1);
                ch = *fresh5;
                lz = 1i32 as libc::c_char
            }
            if ch as libc::c_int >= '0' as i32 &&
                   ch as libc::c_int <= '9' as i32 {
                ch = a2i(ch, &mut fmt, 10i32, &mut w)
            }
            if ch as libc::c_int == 'l' as i32 {
                let fresh6 = fmt;
                fmt = fmt.offset(1);
                ch = *fresh6;
                lng = 1i32 as libc::c_char
            }
            match ch as libc::c_int {
                0 => { break ; }
                117 => {
                    if lng != 0 {
                        uli2a(va.arg::<libc::c_ulong>(),
                              10i32 as libc::c_uint, 0i32, bf.as_mut_ptr());
                    } else {
                        ui2a(va.arg::<libc::c_uint>(), 10i32 as libc::c_uint,
                             0i32, bf.as_mut_ptr());
                    }
                    written += putchw(putp, putf, w, lz, bf.as_mut_ptr())
                }
                100 => {
                    if lng != 0 {
                        li2a(va.arg::<libc::c_ulong>() as libc::c_long,
                             bf.as_mut_ptr());
                    } else { i2a(va.arg::<libc::c_int>(), bf.as_mut_ptr()); }
                    written += putchw(putp, putf, w, lz, bf.as_mut_ptr())
                }
                120 | 88 => {
                    if lng != 0 {
                        uli2a(va.arg::<libc::c_ulong>(),
                              16i32 as libc::c_uint,
                              (ch as libc::c_int == 'X' as i32) as
                                  libc::c_int, bf.as_mut_ptr());
                    } else {
                        ui2a(va.arg::<libc::c_uint>(), 16i32 as libc::c_uint,
                             (ch as libc::c_int == 'X' as i32) as libc::c_int,
                             bf.as_mut_ptr());
                    }
                    written += putchw(putp, putf, w, lz, bf.as_mut_ptr())
                }
                99 => {
                    putf.expect("non-null function pointer")(putp,
                                                             va.arg::<libc::c_int>()
                                                                 as
                                                                 libc::c_char);
                    written += 1
                }
                115 => {
                    written +=
                        putchw(putp, putf, w, 0i32 as libc::c_char,
                               va.arg::<*mut libc::c_char>())
                }
                37 => {
                    putf.expect("non-null function pointer")(putp, ch);
                    written += 1
                }
                110 => { *va.arg::<*mut libc::c_int>() = written }
                _ => { }
            }
        }
    }
    return written;
}
/*
File: printf.h

Copyright (c) 2004,2012 Kustaa Nyholm / SpareTimeLabs

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or other
materials provided with the distribution.

Neither the name of the Kustaa Nyholm or SpareTimeLabs nor the names of its
contributors may be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.

----------------------------------------------------------------------

This library is realy just two files: 'printf.h' and 'printf.c'.

They provide a simple and small (+200 loc) printf functionality to
be used in embedded systems.

I've found them so usefull in debugging that I do not bother with a
debugger at all.

They are distributed in source form, so to use them, just compile them
into your project.

Two printf variants are provided: printf and sprintf.

The formats supported by this implementation are: 'd' 'u' 'c' 's' 'x' 'X'.

Zero padding and field width are also supported.

If the library is compiled with 'PRINTF_SUPPORT_LONG' defined then the
long specifier is also
supported. Note that this will pull in some long math routines (pun intended!)
and thus make your executable noticably longer.

The memory foot print of course depends on the target cpu, compiler and
compiler options, but a rough guestimate (based on a H8S target) is about
1.4 kB for code and some twenty 'int's and 'char's, say 60 bytes of stack space.
Not too bad. Your milage may vary. By hacking the source code you can
get rid of some hunred bytes, I'm sure, but personally I feel the balance of
functionality and flexibility versus  code size is close to optimal for
many embedded systems.

To use the printf you need to supply your own character output function,
something like :

void putc ( void* p, char c)
    {
    while (!SERIAL_PORT_EMPTY) ;
    SERIAL_PORT_TX_REGISTER = c;
    }

Before you can call printf you need to initialize it to use your
character output function with something like:

init_printf(NULL,putc);

Notice the 'NULL' in 'init_printf' and the parameter 'void* p' in 'putc',
the NULL (or any pointer) you pass into the 'init_printf' will eventually be
passed to your 'putc' routine. This allows you to pass some storage space (or
anything realy) to the character output function, if necessary.
This is not often needed but it was implemented like that because it made
implementing the sprintf function so neat (look at the source code).

The code is re-entrant, except for the 'init_printf' function, so it
is safe to call it from interupts too, although this may result in mixed output.
If you rely on re-entrancy, take care that your 'putc' function is re-entrant!

The printf and sprintf functions are actually macros that translate to
'tfp_printf' and 'tfp_sprintf'. This makes it possible
to use them along with 'stdio.h' printf's in a single source file.
You just need to undef the names before you include the 'stdio.h'.
Note that these are not function like macros, so if you have variables
or struct members with these names, things will explode in your face.
Without variadic macros this is the best we can do to wrap these
fucnction. If it is a problem just give up the macros and use the
functions directly or rename them.

For further details see source code.

regs Kusti, 23.10.2004
*/
#[no_mangle]
pub unsafe extern "C" fn init_printf(mut putp: *mut libc::c_void,
                                     mut putf:
                                         Option<unsafe extern "C" fn(_:
                                                                         *mut libc::c_void,
                                                                     _:
                                                                         libc::c_char)
                                                    -> ()>) {
    stdout_putf = putf;
    stdout_putp = putp;
}
#[no_mangle]
pub unsafe extern "C" fn tfp_printf(mut fmt: *const libc::c_char,
                                    mut args: ...) -> libc::c_int {
    let mut va: ::core::ffi::VaListImpl;
    va = args.clone();
    let mut written: libc::c_int =
        tfp_format(stdout_putp, stdout_putf, fmt, va.as_va_list());
    while !isSerialTransmitBufferEmpty(printfSerialPort) { }
    return written;
}
unsafe extern "C" fn putcp(mut p: *mut libc::c_void, mut c: libc::c_char) {
    let ref mut fresh7 = *(p as *mut *mut libc::c_char);
    let fresh8 = *fresh7;
    *fresh7 = (*fresh7).offset(1);
    *fresh8 = c;
}
// Disabling this, in favour of tfp_format to be used in cli.c
//int tfp_printf(const char *fmt, ...);
#[no_mangle]
pub unsafe extern "C" fn tfp_sprintf(mut s: *mut libc::c_char,
                                     mut fmt: *const libc::c_char,
                                     mut args: ...) -> libc::c_int {
    let mut va: ::core::ffi::VaListImpl;
    va = args.clone();
    let mut written: libc::c_int =
        tfp_format(&mut s as *mut *mut libc::c_char as *mut libc::c_void,
                   Some(putcp as
                            unsafe extern "C" fn(_: *mut libc::c_void,
                                                 _: libc::c_char) -> ()), fmt,
                   va.as_va_list());
    putcp(&mut s as *mut *mut libc::c_char as *mut libc::c_void,
          0i32 as libc::c_char);
    return written;
}
unsafe extern "C" fn _putc(mut p: *mut libc::c_void, mut c: libc::c_char) {
    serialWrite(printfSerialPort, c as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn printfSupportInit() {
    init_printf(0 as *mut libc::c_void,
                Some(_putc as
                         unsafe extern "C" fn(_: *mut libc::c_void,
                                              _: libc::c_char) -> ()));
}
#[no_mangle]
pub unsafe extern "C" fn setPrintfSerialPort(mut serialPort:
                                                 *mut serialPort_t) {
    printfSerialPort = serialPort;
}
