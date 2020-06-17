use core;
use libc;
extern "C" {
    pub type _IO_wide_data;
    pub type _IO_codecvt;
    pub type _IO_marker;
    pub type dyad_Stream;
    #[no_mangle]
    static mut stderr: *mut FILE;
    #[no_mangle]
    fn fprintf(_: *mut FILE, _: *const libc::c_char, _: ...) -> libc::c_int;
    #[no_mangle]
    fn __errno_location() -> *mut libc::c_int;
    #[no_mangle]
    fn dyad_setNoDelay(stream: *mut dyad_Stream, opt: libc::c_int);
    #[no_mangle]
    fn dyad_setTimeout(stream: *mut dyad_Stream, seconds: libc::c_double);
    #[no_mangle]
    fn dyad_write(stream: *mut dyad_Stream, data: *const libc::c_void,
                  size: libc::c_int);
    #[no_mangle]
    fn dyad_close(stream: *mut dyad_Stream);
    #[no_mangle]
    fn dyad_addListener(stream: *mut dyad_Stream, event: libc::c_int,
                        callback: dyad_Callback, udata: *mut libc::c_void);
    #[no_mangle]
    fn dyad_listenEx(stream: *mut dyad_Stream, host: *const libc::c_char,
                     port: libc::c_int, backlog: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn dyad_newStream() -> *mut dyad_Stream;
    #[no_mangle]
    fn pthread_mutex_init(__mutex: *mut pthread_mutex_t,
                          __mutexattr: *const pthread_mutexattr_t)
     -> libc::c_int;
    #[no_mangle]
    fn pthread_mutex_unlock(__mutex: *mut pthread_mutex_t) -> libc::c_int;
    #[no_mangle]
    fn pthread_mutex_lock(__mutex: *mut pthread_mutex_t) -> libc::c_int;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type __off_t = libc::c_long;
pub type __off64_t = libc::c_long;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type size_t = libc::c_ulong;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct _IO_FILE {
    pub _flags: libc::c_int,
    pub _IO_read_ptr: *mut libc::c_char,
    pub _IO_read_end: *mut libc::c_char,
    pub _IO_read_base: *mut libc::c_char,
    pub _IO_write_base: *mut libc::c_char,
    pub _IO_write_ptr: *mut libc::c_char,
    pub _IO_write_end: *mut libc::c_char,
    pub _IO_buf_base: *mut libc::c_char,
    pub _IO_buf_end: *mut libc::c_char,
    pub _IO_save_base: *mut libc::c_char,
    pub _IO_backup_base: *mut libc::c_char,
    pub _IO_save_end: *mut libc::c_char,
    pub _markers: *mut _IO_marker,
    pub _chain: *mut _IO_FILE,
    pub _fileno: libc::c_int,
    pub _flags2: libc::c_int,
    pub _old_offset: __off_t,
    pub _cur_column: libc::c_ushort,
    pub _vtable_offset: libc::c_schar,
    pub _shortbuf: [libc::c_char; 1],
    pub _lock: *mut libc::c_void,
    pub _offset: __off64_t,
    pub _codecvt: *mut _IO_codecvt,
    pub _wide_data: *mut _IO_wide_data,
    pub _freeres_list: *mut _IO_FILE,
    pub _freeres_buf: *mut libc::c_void,
    pub __pad5: size_t,
    pub _mode: libc::c_int,
    pub _unused2: [libc::c_char; 20],
}
pub type _IO_lock_t = ();
pub type FILE = _IO_FILE;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct __pthread_internal_list {
    pub __prev: *mut __pthread_internal_list,
    pub __next: *mut __pthread_internal_list,
}
pub type __pthread_list_t = __pthread_internal_list;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct __pthread_mutex_s {
    pub __lock: libc::c_int,
    pub __count: libc::c_uint,
    pub __owner: libc::c_int,
    pub __nusers: libc::c_uint,
    pub __kind: libc::c_int,
    pub __spins: libc::c_short,
    pub __elision: libc::c_short,
    pub __list: __pthread_list_t,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union pthread_mutexattr_t {
    pub __size: [libc::c_char; 4],
    pub __align: libc::c_int,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union pthread_mutex_t {
    pub __data: __pthread_mutex_s,
    pub __size: [libc::c_char; 40],
    pub __align: libc::c_long,
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
pub type portMode_e = libc::c_uint;
pub const MODE_RXTX: portMode_e = 3;
pub const MODE_TX: portMode_e = 2;
pub const MODE_RX: portMode_e = 1;
pub type portOptions_e = libc::c_uint;
pub const SERIAL_BIDIR_NOPULL: portOptions_e = 32;
pub const SERIAL_BIDIR_PP: portOptions_e = 16;
// disable pulls in BIDIR RX mode
/*
     * Note on SERIAL_BIDIR_PP
     * With SERIAL_BIDIR_PP, the very first start bit of back-to-back bytes
     * is lost and the first data byte will be lost by a framing error.
     * To ensure the first start bit to be sent, prepend a zero byte (0x00)
     * to actual data bytes.
     */
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
// used by serial drivers to return frames to app
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
pub type serialPort_t = serialPort_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct dyad_Event {
    pub type_0: libc::c_int,
    pub udata: *mut libc::c_void,
    pub stream: *mut dyad_Stream,
    pub remote: *mut dyad_Stream,
    pub msg: *const libc::c_char,
    pub data: *mut libc::c_char,
    pub size: libc::c_int,
}
pub type dyad_Callback
    =
    Option<unsafe extern "C" fn(_: *mut dyad_Event) -> ()>;
pub type C2RustUnnamed = libc::c_uint;
pub const DYAD_EVENT_TICK: C2RustUnnamed = 11;
pub const DYAD_EVENT_TIMEOUT: C2RustUnnamed = 10;
pub const DYAD_EVENT_ERROR: C2RustUnnamed = 9;
pub const DYAD_EVENT_LINE: C2RustUnnamed = 8;
pub const DYAD_EVENT_DATA: C2RustUnnamed = 7;
pub const DYAD_EVENT_READY: C2RustUnnamed = 6;
pub const DYAD_EVENT_CLOSE: C2RustUnnamed = 5;
pub const DYAD_EVENT_CONNECT: C2RustUnnamed = 4;
pub const DYAD_EVENT_LISTEN: C2RustUnnamed = 3;
pub const DYAD_EVENT_ACCEPT: C2RustUnnamed = 2;
pub const DYAD_EVENT_DESTROY: C2RustUnnamed = 1;
pub const DYAD_EVENT_NULL: C2RustUnnamed = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct tcpPort_t {
    pub port: serialPort_t,
    pub rxBuffer: [uint8_t; 1400],
    pub txBuffer: [uint8_t; 1400],
    pub serv: *mut dyad_Stream,
    pub conn: *mut dyad_Stream,
    pub txLock: pthread_mutex_t,
    pub rxLock: pthread_mutex_t,
    pub connected: bool,
    pub clientCount: uint16_t,
    pub id: uint8_t,
}
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
// Forward
static mut tcpSerialPorts: [tcpPort_t; 8] =
    [tcpPort_t{port:
                   serialPort_t{vTable: 0 as *const serialPortVTable,
                                mode: 0 as portMode_e,
                                options: SERIAL_NOT_INVERTED,
                                baudRate: 0,
                                rxBufferSize: 0,
                                txBufferSize: 0,
                                rxBuffer: 0 as *const uint8_t as *mut uint8_t,
                                txBuffer: 0 as *const uint8_t as *mut uint8_t,
                                rxBufferHead: 0,
                                rxBufferTail: 0,
                                txBufferHead: 0,
                                txBufferTail: 0,
                                rxCallback: None,
                                rxCallbackData:
                                    0 as *const libc::c_void as
                                        *mut libc::c_void,
                                identifier: 0,},
               rxBuffer: [0; 1400],
               txBuffer: [0; 1400],
               serv: 0 as *const dyad_Stream as *mut dyad_Stream,
               conn: 0 as *const dyad_Stream as *mut dyad_Stream,
               txLock:
                   pthread_mutex_t{__data:
                                       __pthread_mutex_s{__lock: 0,
                                                         __count: 0,
                                                         __owner: 0,
                                                         __nusers: 0,
                                                         __kind: 0,
                                                         __spins: 0,
                                                         __elision: 0,
                                                         __list:
                                                             __pthread_list_t{__prev:
                                                                                  0
                                                                                      as
                                                                                      *const __pthread_internal_list
                                                                                      as
                                                                                      *mut __pthread_internal_list,
                                                                              __next:
                                                                                  0
                                                                                      as
                                                                                      *const __pthread_internal_list
                                                                                      as
                                                                                      *mut __pthread_internal_list,},},},
               rxLock:
                   pthread_mutex_t{__data:
                                       __pthread_mutex_s{__lock: 0,
                                                         __count: 0,
                                                         __owner: 0,
                                                         __nusers: 0,
                                                         __kind: 0,
                                                         __spins: 0,
                                                         __elision: 0,
                                                         __list:
                                                             __pthread_list_t{__prev:
                                                                                  0
                                                                                      as
                                                                                      *const __pthread_internal_list
                                                                                      as
                                                                                      *mut __pthread_internal_list,
                                                                              __next:
                                                                                  0
                                                                                      as
                                                                                      *const __pthread_internal_list
                                                                                      as
                                                                                      *mut __pthread_internal_list,},},},
               connected: false,
               clientCount: 0,
               id: 0,}; 8];
static mut tcpPortInitialized: [bool; 8] = [false; 8];
static mut tcpStart: bool = 0i32 != 0;
#[no_mangle]
pub unsafe extern "C" fn tcpIsStart() -> bool { return tcpStart; }
unsafe extern "C" fn onData(mut e: *mut dyad_Event) {
    let mut s: *mut tcpPort_t = (*e).udata as *mut tcpPort_t;
    tcpDataIn(s, (*e).data as *mut uint8_t, (*e).size);
}
unsafe extern "C" fn onClose(mut e: *mut dyad_Event) {
    let mut s: *mut tcpPort_t = (*e).udata as *mut tcpPort_t;
    (*s).clientCount = (*s).clientCount.wrapping_sub(1);
    (*s).conn = 0 as *mut dyad_Stream;
    fprintf(stderr,
            b"[CLS]UART%u: %d,%d\n\x00" as *const u8 as *const libc::c_char,
            (*s).id as libc::c_int + 1i32, (*s).connected as libc::c_int,
            (*s).clientCount as libc::c_int);
    if (*s).clientCount as libc::c_int == 0i32 { (*s).connected = 0i32 != 0 };
}
unsafe extern "C" fn onAccept(mut e: *mut dyad_Event) {
    let mut s: *mut tcpPort_t = (*e).udata as *mut tcpPort_t;
    fprintf(stderr,
            b"New connection on UART%u, %d\n\x00" as *const u8 as
                *const libc::c_char, (*s).id as libc::c_int + 1i32,
            (*s).clientCount as libc::c_int);
    (*s).connected = 1i32 != 0;
    if (*s).clientCount as libc::c_int > 0i32 {
        dyad_close((*e).remote);
        return
    }
    (*s).clientCount = (*s).clientCount.wrapping_add(1);
    fprintf(stderr,
            b"[NEW]UART%u: %d,%d\n\x00" as *const u8 as *const libc::c_char,
            (*s).id as libc::c_int + 1i32, (*s).connected as libc::c_int,
            (*s).clientCount as libc::c_int);
    (*s).conn = (*e).remote;
    dyad_setNoDelay((*e).remote, 1i32);
    dyad_setTimeout((*e).remote, 120i32 as libc::c_double);
    dyad_addListener((*e).remote, DYAD_EVENT_DATA as libc::c_int,
                     Some(onData as
                              unsafe extern "C" fn(_: *mut dyad_Event) -> ()),
                     (*e).udata);
    dyad_addListener((*e).remote, DYAD_EVENT_CLOSE as libc::c_int,
                     Some(onClose as
                              unsafe extern "C" fn(_: *mut dyad_Event) -> ()),
                     (*e).udata);
}
unsafe extern "C" fn tcpReconfigure(mut s: *mut tcpPort_t,
                                    mut id: libc::c_int) -> *mut tcpPort_t {
    if tcpPortInitialized[id as usize] {
        fprintf(stderr,
                b"port is already initialized!\n\x00" as *const u8 as
                    *const libc::c_char);
        return s
    }
    if pthread_mutex_init(&mut (*s).txLock, 0 as *const pthread_mutexattr_t)
           != 0i32 {
        fprintf(stderr,
                b"TX mutex init failed - %d\n\x00" as *const u8 as
                    *const libc::c_char, *__errno_location());
        // TODO: clean up & re-init
        return 0 as *mut tcpPort_t
    }
    if pthread_mutex_init(&mut (*s).rxLock, 0 as *const pthread_mutexattr_t)
           != 0i32 {
        fprintf(stderr,
                b"RX mutex init failed - %d\n\x00" as *const u8 as
                    *const libc::c_char, *__errno_location());
        // TODO: clean up & re-init
        return 0 as *mut tcpPort_t
    }
    tcpStart = 1i32 != 0;
    tcpPortInitialized[id as usize] = 1i32 != 0;
    (*s).connected = 0i32 != 0;
    (*s).clientCount = 0i32 as uint16_t;
    (*s).id = id as uint8_t;
    (*s).conn = 0 as *mut dyad_Stream;
    (*s).serv = dyad_newStream();
    dyad_setNoDelay((*s).serv, 1i32);
    dyad_addListener((*s).serv, DYAD_EVENT_ACCEPT as libc::c_int,
                     Some(onAccept as
                              unsafe extern "C" fn(_: *mut dyad_Event) -> ()),
                     s as *mut libc::c_void);
    if dyad_listenEx((*s).serv, 0 as *const libc::c_char, 5760i32 + id + 1i32,
                     10i32) == 0i32 {
        fprintf(stderr,
                b"bind port %u for UART%u\n\x00" as *const u8 as
                    *const libc::c_char,
                (5760i32 as
                     libc::c_uint).wrapping_add(id as
                                                    libc::c_uint).wrapping_add(1i32
                                                                                   as
                                                                                   libc::c_uint),
                (id as libc::c_uint).wrapping_add(1i32 as libc::c_uint));
    } else {
        fprintf(stderr,
                b"bind port %u for UART%u failed!!\n\x00" as *const u8 as
                    *const libc::c_char,
                (5760i32 as
                     libc::c_uint).wrapping_add(id as
                                                    libc::c_uint).wrapping_add(1i32
                                                                                   as
                                                                                   libc::c_uint),
                (id as libc::c_uint).wrapping_add(1i32 as libc::c_uint));
    }
    return s;
}
#[no_mangle]
pub unsafe extern "C" fn serTcpOpen(mut id: libc::c_int,
                                    mut rxCallback: serialReceiveCallbackPtr,
                                    mut rxCallbackData: *mut libc::c_void,
                                    mut baudRate: uint32_t,
                                    mut mode: portMode_e,
                                    mut options: portOptions_e)
 -> *mut serialPort_t {
    let mut s: *mut tcpPort_t = 0 as *mut tcpPort_t;
    if id >= 0i32 && id < 8i32 {
        s =
            tcpReconfigure(&mut *tcpSerialPorts.as_mut_ptr().offset(id as
                                                                        isize),
                           id)
    }
    if s.is_null() { return 0 as *mut serialPort_t }
    (*s).port.vTable = &tcpVTable;
    // common serial initialisation code should move to serialPort::init()
    (*s).port.rxBufferTail = 0i32 as uint32_t;
    (*s).port.rxBufferHead = (*s).port.rxBufferTail;
    (*s).port.txBufferTail = 0i32 as uint32_t;
    (*s).port.txBufferHead = (*s).port.txBufferTail;
    (*s).port.rxBufferSize = 1400i32 as uint32_t;
    (*s).port.txBufferSize = 1400i32 as uint32_t;
    (*s).port.rxBuffer = (*s).rxBuffer.as_mut_ptr() as *mut uint8_t;
    (*s).port.txBuffer = (*s).txBuffer.as_mut_ptr() as *mut uint8_t;
    // callback works for IRQ-based RX ONLY
    (*s).port.rxCallback = rxCallback;
    (*s).port.rxCallbackData = rxCallbackData;
    (*s).port.mode = mode;
    (*s).port.baudRate = baudRate;
    (*s).port.options = options;
    return s as *mut serialPort_t;
}
#[no_mangle]
pub unsafe extern "C" fn tcpTotalRxBytesWaiting(mut instance:
                                                    *const serialPort_t)
 -> uint32_t {
    let mut s: *mut tcpPort_t = instance as *mut tcpPort_t;
    let mut count: uint32_t = 0;
    pthread_mutex_lock(&mut (*s).rxLock);
    if (*s).port.rxBufferHead >= (*s).port.rxBufferTail {
        count = (*s).port.rxBufferHead.wrapping_sub((*s).port.rxBufferTail)
    } else {
        count =
            (*s).port.rxBufferSize.wrapping_add((*s).port.rxBufferHead).wrapping_sub((*s).port.rxBufferTail)
    }
    pthread_mutex_unlock(&mut (*s).rxLock);
    return count;
}
#[no_mangle]
pub unsafe extern "C" fn tcpTotalTxBytesFree(mut instance:
                                                 *const serialPort_t)
 -> uint32_t {
    let mut s: *mut tcpPort_t = instance as *mut tcpPort_t;
    let mut bytesUsed: uint32_t = 0;
    pthread_mutex_lock(&mut (*s).txLock);
    if (*s).port.txBufferHead >= (*s).port.txBufferTail {
        bytesUsed =
            (*s).port.txBufferHead.wrapping_sub((*s).port.txBufferTail)
    } else {
        bytesUsed =
            (*s).port.txBufferSize.wrapping_add((*s).port.txBufferHead).wrapping_sub((*s).port.txBufferTail)
    }
    let mut bytesFree: uint32_t =
        (*s).port.txBufferSize.wrapping_sub(1i32 as
                                                libc::c_uint).wrapping_sub(bytesUsed);
    pthread_mutex_unlock(&mut (*s).txLock);
    return bytesFree;
}
#[no_mangle]
pub unsafe extern "C" fn isTcpTransmitBufferEmpty(mut instance:
                                                      *const serialPort_t)
 -> bool {
    let mut s: *mut tcpPort_t = instance as *mut tcpPort_t;
    pthread_mutex_lock(&mut (*s).txLock);
    let mut isEmpty: bool = (*s).port.txBufferTail == (*s).port.txBufferHead;
    pthread_mutex_unlock(&mut (*s).txLock);
    return isEmpty;
}
#[no_mangle]
pub unsafe extern "C" fn tcpRead(mut instance: *mut serialPort_t) -> uint8_t {
    let mut ch: uint8_t = 0;
    let mut s: *mut tcpPort_t = instance as *mut tcpPort_t;
    pthread_mutex_lock(&mut (*s).rxLock);
    ch = *(*s).port.rxBuffer.offset((*s).port.rxBufferTail as isize);
    if (*s).port.rxBufferTail.wrapping_add(1i32 as libc::c_uint) >=
           (*s).port.rxBufferSize {
        (*s).port.rxBufferTail = 0i32 as uint32_t
    } else { (*s).port.rxBufferTail = (*s).port.rxBufferTail.wrapping_add(1) }
    pthread_mutex_unlock(&mut (*s).rxLock);
    return ch;
}
#[no_mangle]
pub unsafe extern "C" fn tcpWrite(mut instance: *mut serialPort_t,
                                  mut ch: uint8_t) {
    let mut s: *mut tcpPort_t = instance as *mut tcpPort_t;
    pthread_mutex_lock(&mut (*s).txLock);
    ::core::ptr::write_volatile((*s).port.txBuffer.offset((*s).port.txBufferHead
                                                              as isize), ch);
    if (*s).port.txBufferHead.wrapping_add(1i32 as libc::c_uint) >=
           (*s).port.txBufferSize {
        (*s).port.txBufferHead = 0i32 as uint32_t
    } else { (*s).port.txBufferHead = (*s).port.txBufferHead.wrapping_add(1) }
    pthread_mutex_unlock(&mut (*s).txLock);
    tcpDataOut(s);
}
#[no_mangle]
pub unsafe extern "C" fn tcpDataOut(mut instance: *mut tcpPort_t) {
    let mut s: *mut tcpPort_t = instance;
    if (*s).conn.is_null() { return }
    pthread_mutex_lock(&mut (*s).txLock);
    if (*s).port.txBufferHead < (*s).port.txBufferTail {
        // send data till end of buffer
        let mut chunk: libc::c_int =
            (*s).port.txBufferSize.wrapping_sub((*s).port.txBufferTail) as
                libc::c_int;
        dyad_write((*s).conn,
                   &mut *(*s).port.txBuffer.offset((*s).port.txBufferTail as
                                                       isize) as *mut uint8_t
                       as *const libc::c_void, chunk);
        (*s).port.txBufferTail = 0i32 as uint32_t
    }
    let mut chunk_0: libc::c_int =
        (*s).port.txBufferHead.wrapping_sub((*s).port.txBufferTail) as
            libc::c_int;
    if chunk_0 != 0 {
        dyad_write((*s).conn,
                   &mut *(*s).port.txBuffer.offset((*s).port.txBufferTail as
                                                       isize) as *mut uint8_t
                       as *const libc::c_void, chunk_0);
    }
    (*s).port.txBufferTail = (*s).port.txBufferHead;
    pthread_mutex_unlock(&mut (*s).txLock);
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
// tcpPort API
#[no_mangle]
pub unsafe extern "C" fn tcpDataIn(mut instance: *mut tcpPort_t,
                                   mut ch: *mut uint8_t,
                                   mut size: libc::c_int) {
    let mut s: *mut tcpPort_t = instance;
    pthread_mutex_lock(&mut (*s).rxLock);
    loop  {
        let fresh0 = size;
        size = size - 1;
        if !(fresh0 != 0) { break ; }
        //        printf("%c", *ch);
        let fresh1 = ch;
        ch = ch.offset(1);
        ::core::ptr::write_volatile((*s).port.rxBuffer.offset((*s).port.rxBufferHead
                                                                  as isize),
                                    *fresh1);
        if (*s).port.rxBufferHead.wrapping_add(1i32 as libc::c_uint) >=
               (*s).port.rxBufferSize {
            (*s).port.rxBufferHead = 0i32 as uint32_t
        } else {
            (*s).port.rxBufferHead = (*s).port.rxBufferHead.wrapping_add(1)
        }
    }
    pthread_mutex_unlock(&mut (*s).rxLock);
    //    printf("\n");
}
static mut tcpVTable: serialPortVTable =
    unsafe {
        {
            let mut init =
                serialPortVTable{serialWrite:
                                     Some(tcpWrite as
                                              unsafe extern "C" fn(_:
                                                                       *mut serialPort_t,
                                                                   _: uint8_t)
                                                  -> ()),
                                 serialTotalRxWaiting:
                                     Some(tcpTotalRxBytesWaiting as
                                              unsafe extern "C" fn(_:
                                                                       *const serialPort_t)
                                                  -> uint32_t),
                                 serialTotalTxFree:
                                     Some(tcpTotalTxBytesFree as
                                              unsafe extern "C" fn(_:
                                                                       *const serialPort_t)
                                                  -> uint32_t),
                                 serialRead:
                                     Some(tcpRead as
                                              unsafe extern "C" fn(_:
                                                                       *mut serialPort_t)
                                                  -> uint8_t),
                                 serialSetBaudRate: None,
                                 isSerialTransmitBufferEmpty:
                                     Some(isTcpTransmitBufferEmpty as
                                              unsafe extern "C" fn(_:
                                                                       *const serialPort_t)
                                                  -> bool),
                                 setMode: None,
                                 setCtrlLineStateCb: None,
                                 setBaudRateCb: None,
                                 writeBuf: None,
                                 beginWrite: None,
                                 endWrite: None,};
            init
        }
    };
