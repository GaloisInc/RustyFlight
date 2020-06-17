use core;
use libc;
extern "C" {
    pub type sockaddr_x25;
    pub type sockaddr_un;
    pub type sockaddr_ns;
    pub type sockaddr_iso;
    pub type sockaddr_ipx;
    pub type sockaddr_inarp;
    pub type sockaddr_eon;
    pub type sockaddr_dl;
    pub type sockaddr_ax25;
    pub type sockaddr_at;
    pub type _IO_wide_data;
    pub type _IO_codecvt;
    pub type _IO_marker;
    #[no_mangle]
    fn close(__fd: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn select(__nfds: libc::c_int, __readfds: *mut fd_set,
              __writefds: *mut fd_set, __exceptfds: *mut fd_set,
              __timeout: *mut timeval) -> libc::c_int;
    #[no_mangle]
    fn socket(__domain: libc::c_int, __type: libc::c_int,
              __protocol: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn bind(__fd: libc::c_int, __addr: __CONST_SOCKADDR_ARG, __len: socklen_t)
     -> libc::c_int;
    #[no_mangle]
    fn getsockname(__fd: libc::c_int, __addr: __SOCKADDR_ARG,
                   __len: *mut socklen_t) -> libc::c_int;
    #[no_mangle]
    fn connect(__fd: libc::c_int, __addr: __CONST_SOCKADDR_ARG,
               __len: socklen_t) -> libc::c_int;
    #[no_mangle]
    fn getpeername(__fd: libc::c_int, __addr: __SOCKADDR_ARG,
                   __len: *mut socklen_t) -> libc::c_int;
    #[no_mangle]
    fn send(__fd: libc::c_int, __buf: *const libc::c_void, __n: size_t,
            __flags: libc::c_int) -> ssize_t;
    #[no_mangle]
    fn recv(__fd: libc::c_int, __buf: *mut libc::c_void, __n: size_t,
            __flags: libc::c_int) -> ssize_t;
    #[no_mangle]
    fn getsockopt(__fd: libc::c_int, __level: libc::c_int,
                  __optname: libc::c_int, __optval: *mut libc::c_void,
                  __optlen: *mut socklen_t) -> libc::c_int;
    #[no_mangle]
    fn setsockopt(__fd: libc::c_int, __level: libc::c_int,
                  __optname: libc::c_int, __optval: *const libc::c_void,
                  __optlen: socklen_t) -> libc::c_int;
    #[no_mangle]
    fn listen(__fd: libc::c_int, __n: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn accept(__fd: libc::c_int, __addr: __SOCKADDR_ARG,
              __addr_len: *mut socklen_t) -> libc::c_int;
    #[no_mangle]
    fn getaddrinfo(__name: *const libc::c_char,
                   __service: *const libc::c_char, __req: *const addrinfo,
                   __pai: *mut *mut addrinfo) -> libc::c_int;
    #[no_mangle]
    fn freeaddrinfo(__ai: *mut addrinfo);
    #[no_mangle]
    fn fcntl(__fd: libc::c_int, __cmd: libc::c_int, _: ...) -> libc::c_int;
    #[no_mangle]
    fn gettimeofday(__tv: *mut timeval, __tz: __timezone_ptr_t)
     -> libc::c_int;
    #[no_mangle]
    fn inet_ntop(__af: libc::c_int, __cp: *const libc::c_void,
                 __buf: *mut libc::c_char, __len: socklen_t)
     -> *const libc::c_char;
    #[no_mangle]
    fn printf(_: *const libc::c_char, _: ...) -> libc::c_int;
    #[no_mangle]
    fn sprintf(_: *mut libc::c_char, _: *const libc::c_char, _: ...)
     -> libc::c_int;
    #[no_mangle]
    fn vsprintf(_: *mut libc::c_char, _: *const libc::c_char,
                _: ::core::ffi::VaList) -> libc::c_int;
    #[no_mangle]
    fn fgetc(__stream: *mut FILE) -> libc::c_int;
    #[no_mangle]
    fn realloc(_: *mut libc::c_void, _: libc::c_ulong) -> *mut libc::c_void;
    #[no_mangle]
    fn free(__ptr: *mut libc::c_void);
    #[no_mangle]
    fn exit(_: libc::c_int) -> !;
    #[no_mangle]
    fn memmove(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn strerror(_: libc::c_int) -> *mut libc::c_char;
    #[no_mangle]
    fn signal(__sig: libc::c_int, __handler: __sighandler_t)
     -> __sighandler_t;
    #[no_mangle]
    fn __errno_location() -> *mut libc::c_int;
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
pub type __off_t = libc::c_long;
pub type __off64_t = libc::c_long;
pub type __time_t = libc::c_long;
pub type __suseconds_t = libc::c_long;
pub type __ssize_t = libc::c_long;
pub type __socklen_t = libc::c_uint;
pub type ssize_t = __ssize_t;
pub type size_t = libc::c_ulong;
pub type socklen_t = __socklen_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct timeval {
    pub tv_sec: __time_t,
    pub tv_usec: __suseconds_t,
}
pub type __fd_mask = libc::c_long;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct fd_set {
    pub fds_bits: [__fd_mask; 16],
}
pub type __socket_type = libc::c_uint;
pub const SOCK_NONBLOCK: __socket_type = 2048;
pub const SOCK_CLOEXEC: __socket_type = 524288;
pub const SOCK_PACKET: __socket_type = 10;
pub const SOCK_DCCP: __socket_type = 6;
pub const SOCK_SEQPACKET: __socket_type = 5;
pub const SOCK_RDM: __socket_type = 4;
pub const SOCK_RAW: __socket_type = 3;
pub const SOCK_DGRAM: __socket_type = 2;
pub const SOCK_STREAM: __socket_type = 1;
pub type sa_family_t = libc::c_ushort;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct sockaddr {
    pub sa_family: sa_family_t,
    pub sa_data: [libc::c_char; 14],
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct sockaddr_storage {
    pub ss_family: sa_family_t,
    pub __ss_padding: [libc::c_char; 118],
    pub __ss_align: libc::c_ulong,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union __SOCKADDR_ARG {
    pub __sockaddr__: *mut sockaddr,
    pub __sockaddr_at__: *mut sockaddr_at,
    pub __sockaddr_ax25__: *mut sockaddr_ax25,
    pub __sockaddr_dl__: *mut sockaddr_dl,
    pub __sockaddr_eon__: *mut sockaddr_eon,
    pub __sockaddr_in__: *mut sockaddr_in,
    pub __sockaddr_in6__: *mut sockaddr_in6,
    pub __sockaddr_inarp__: *mut sockaddr_inarp,
    pub __sockaddr_ipx__: *mut sockaddr_ipx,
    pub __sockaddr_iso__: *mut sockaddr_iso,
    pub __sockaddr_ns__: *mut sockaddr_ns,
    pub __sockaddr_un__: *mut sockaddr_un,
    pub __sockaddr_x25__: *mut sockaddr_x25,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct sockaddr_in6 {
    pub sin6_family: sa_family_t,
    pub sin6_port: in_port_t,
    pub sin6_flowinfo: uint32_t,
    pub sin6_addr: in6_addr,
    pub sin6_scope_id: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct in6_addr {
    pub __in6_u: C2RustUnnamed,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed {
    pub __u6_addr8: [uint8_t; 16],
    pub __u6_addr16: [uint16_t; 8],
    pub __u6_addr32: [uint32_t; 4],
}
pub type in_port_t = uint16_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct sockaddr_in {
    pub sin_family: sa_family_t,
    pub sin_port: in_port_t,
    pub sin_addr: in_addr,
    pub sin_zero: [libc::c_uchar; 8],
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct in_addr {
    pub s_addr: in_addr_t,
}
pub type in_addr_t = uint32_t;
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union __CONST_SOCKADDR_ARG {
    pub __sockaddr__: *const sockaddr,
    pub __sockaddr_at__: *const sockaddr_at,
    pub __sockaddr_ax25__: *const sockaddr_ax25,
    pub __sockaddr_dl__: *const sockaddr_dl,
    pub __sockaddr_eon__: *const sockaddr_eon,
    pub __sockaddr_in__: *const sockaddr_in,
    pub __sockaddr_in6__: *const sockaddr_in6,
    pub __sockaddr_inarp__: *const sockaddr_inarp,
    pub __sockaddr_ipx__: *const sockaddr_ipx,
    pub __sockaddr_iso__: *const sockaddr_iso,
    pub __sockaddr_ns__: *const sockaddr_ns,
    pub __sockaddr_un__: *const sockaddr_un,
    pub __sockaddr_x25__: *const sockaddr_x25,
}
pub type C2RustUnnamed_0 = libc::c_uint;
pub const IPPROTO_MAX: C2RustUnnamed_0 = 256;
pub const IPPROTO_RAW: C2RustUnnamed_0 = 255;
pub const IPPROTO_MPLS: C2RustUnnamed_0 = 137;
pub const IPPROTO_UDPLITE: C2RustUnnamed_0 = 136;
pub const IPPROTO_SCTP: C2RustUnnamed_0 = 132;
pub const IPPROTO_COMP: C2RustUnnamed_0 = 108;
pub const IPPROTO_PIM: C2RustUnnamed_0 = 103;
pub const IPPROTO_ENCAP: C2RustUnnamed_0 = 98;
pub const IPPROTO_BEETPH: C2RustUnnamed_0 = 94;
pub const IPPROTO_MTP: C2RustUnnamed_0 = 92;
pub const IPPROTO_AH: C2RustUnnamed_0 = 51;
pub const IPPROTO_ESP: C2RustUnnamed_0 = 50;
pub const IPPROTO_GRE: C2RustUnnamed_0 = 47;
pub const IPPROTO_RSVP: C2RustUnnamed_0 = 46;
pub const IPPROTO_IPV6: C2RustUnnamed_0 = 41;
pub const IPPROTO_DCCP: C2RustUnnamed_0 = 33;
pub const IPPROTO_TP: C2RustUnnamed_0 = 29;
pub const IPPROTO_IDP: C2RustUnnamed_0 = 22;
pub const IPPROTO_UDP: C2RustUnnamed_0 = 17;
pub const IPPROTO_PUP: C2RustUnnamed_0 = 12;
pub const IPPROTO_EGP: C2RustUnnamed_0 = 8;
pub const IPPROTO_TCP: C2RustUnnamed_0 = 6;
pub const IPPROTO_IPIP: C2RustUnnamed_0 = 4;
pub const IPPROTO_IGMP: C2RustUnnamed_0 = 2;
pub const IPPROTO_ICMP: C2RustUnnamed_0 = 1;
pub const IPPROTO_IP: C2RustUnnamed_0 = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct addrinfo {
    pub ai_flags: libc::c_int,
    pub ai_family: libc::c_int,
    pub ai_socktype: libc::c_int,
    pub ai_protocol: libc::c_int,
    pub ai_addrlen: socklen_t,
    pub ai_addr: *mut sockaddr,
    pub ai_canonname: *mut libc::c_char,
    pub ai_next: *mut addrinfo,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct timezone {
    pub tz_minuteswest: libc::c_int,
    pub tz_dsttime: libc::c_int,
}
pub type __timezone_ptr_t = *mut timezone;
pub type va_list = __builtin_va_list;
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
pub type __sighandler_t = Option<unsafe extern "C" fn(_: libc::c_int) -> ()>;
/* *
 * Copyright (c) 2016 rxi
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the MIT license. See LICENSE for details.
 */
pub type dyad_Socket = libc::c_int;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct dyad_Stream {
    pub state: libc::c_int,
    pub flags: libc::c_int,
    pub sockfd: dyad_Socket,
    pub address: *mut libc::c_char,
    pub port: libc::c_int,
    pub bytesSent: libc::c_int,
    pub bytesReceived: libc::c_int,
    pub lastActivity: libc::c_double,
    pub timeout: libc::c_double,
    pub listeners: C2RustUnnamed_3,
    pub lineBuffer: C2RustUnnamed_2,
    pub writeBuffer: C2RustUnnamed_1,
    pub next: *mut dyad_Stream,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct C2RustUnnamed_1 {
    pub data: *mut libc::c_char,
    pub length: libc::c_int,
    pub capacity: libc::c_int,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct C2RustUnnamed_2 {
    pub data: *mut libc::c_char,
    pub length: libc::c_int,
    pub capacity: libc::c_int,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct C2RustUnnamed_3 {
    pub data: *mut Listener,
    pub length: libc::c_int,
    pub capacity: libc::c_int,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct Listener {
    pub event: libc::c_int,
    pub callback: dyad_Callback,
    pub udata: *mut libc::c_void,
}
pub type dyad_Callback
    =
    Option<unsafe extern "C" fn(_: *mut dyad_Event) -> ()>;
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
pub type dyad_PanicCallback
    =
    Option<unsafe extern "C" fn(_: *const libc::c_char) -> ()>;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const DYAD_EVENT_TICK: C2RustUnnamed_4 = 11;
pub const DYAD_EVENT_TIMEOUT: C2RustUnnamed_4 = 10;
pub const DYAD_EVENT_ERROR: C2RustUnnamed_4 = 9;
pub const DYAD_EVENT_LINE: C2RustUnnamed_4 = 8;
pub const DYAD_EVENT_DATA: C2RustUnnamed_4 = 7;
pub const DYAD_EVENT_READY: C2RustUnnamed_4 = 6;
pub const DYAD_EVENT_CLOSE: C2RustUnnamed_4 = 5;
pub const DYAD_EVENT_CONNECT: C2RustUnnamed_4 = 4;
pub const DYAD_EVENT_LISTEN: C2RustUnnamed_4 = 3;
pub const DYAD_EVENT_ACCEPT: C2RustUnnamed_4 = 2;
pub const DYAD_EVENT_DESTROY: C2RustUnnamed_4 = 1;
pub const DYAD_EVENT_NULL: C2RustUnnamed_4 = 0;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const DYAD_STATE_LISTENING: C2RustUnnamed_5 = 4;
pub const DYAD_STATE_CONNECTED: C2RustUnnamed_5 = 3;
pub const DYAD_STATE_CONNECTING: C2RustUnnamed_5 = 2;
pub const DYAD_STATE_CLOSING: C2RustUnnamed_5 = 1;
pub const DYAD_STATE_CLOSED: C2RustUnnamed_5 = 0;
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed_6 {
    pub sa: sockaddr,
    pub sas: sockaddr_storage,
    pub sai: sockaddr_in,
    pub sai6: sockaddr_in6,
}
pub const SELECT_READ: C2RustUnnamed_7 = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct SelectSet {
    pub capacity: libc::c_int,
    pub maxfd: dyad_Socket,
    pub fds: [*mut fd_set; 3],
}
pub const SELECT_EXCEPT: C2RustUnnamed_7 = 2;
pub const SELECT_WRITE: C2RustUnnamed_7 = 1;
pub const SELECT_MAX: C2RustUnnamed_7 = 3;
pub type C2RustUnnamed_7 = libc::c_uint;
#[inline]
unsafe extern "C" fn __bswap_16(mut __bsx: __uint16_t) -> __uint16_t {
    return (__bsx as libc::c_int >> 8i32 & 0xffi32 |
                (__bsx as libc::c_int & 0xffi32) << 8i32) as __uint16_t;
}
unsafe extern "C" fn dyad_realloc(mut ptr: *mut libc::c_void,
                                  mut n: libc::c_int) -> *mut libc::c_void {
    ptr = realloc(ptr, n as libc::c_ulong);
    if ptr.is_null() && n != 0i32 {
        panic(b"out of memory\x00" as *const u8 as *const libc::c_char);
    }
    return ptr;
}
unsafe extern "C" fn dyad_free(mut ptr: *mut libc::c_void) { free(ptr); }
unsafe extern "C" fn vec_expand(mut data: *mut *mut libc::c_char,
                                mut length: *mut libc::c_int,
                                mut capacity: *mut libc::c_int,
                                mut memsz: libc::c_int) {
    if *length + 1i32 > *capacity {
        if *capacity == 0i32 { *capacity = 1i32 } else { *capacity <<= 1i32 }
        *data =
            dyad_realloc(*data as *mut libc::c_void, *capacity * memsz) as
                *mut libc::c_char
    };
}
unsafe extern "C" fn vec_splice(mut data: *mut *mut libc::c_char,
                                mut length: *mut libc::c_int,
                                mut capacity: *mut libc::c_int,
                                mut memsz: libc::c_int,
                                mut start: libc::c_int,
                                mut count: libc::c_int) {
    memmove((*data).offset((start * memsz) as isize) as *mut libc::c_void,
            (*data).offset(((start + count) * memsz) as isize) as
                *const libc::c_void,
            ((*length - start - count) * memsz) as libc::c_ulong);
}
unsafe extern "C" fn select_deinit(mut s: *mut SelectSet) {
    let mut i: libc::c_int = 0;
    i = 0i32;
    while i < SELECT_MAX as libc::c_int {
        dyad_free((*s).fds[i as usize] as *mut libc::c_void);
        (*s).fds[i as usize] = 0 as *mut fd_set;
        i += 1
    }
    (*s).capacity = 0i32;
}
unsafe extern "C" fn select_grow(mut s: *mut SelectSet) {
    let mut i: libc::c_int = 0;
    let mut oldCapacity: libc::c_int = (*s).capacity;
    (*s).capacity =
        if (*s).capacity != 0 { ((*s).capacity) << 1i32 } else { 1i32 };
    i = 0i32;
    while i < SELECT_MAX as libc::c_int {
        (*s).fds[i as usize] =
            dyad_realloc((*s).fds[i as usize] as *mut libc::c_void,
                         ((*s).capacity as
                              libc::c_ulong).wrapping_mul(::core::mem::size_of::<fd_set>()
                                                              as
                                                              libc::c_ulong)
                             as libc::c_int) as *mut fd_set;
        memset((*s).fds[i as usize].offset(oldCapacity as isize) as
                   *mut libc::c_void, 0i32,
               (((*s).capacity - oldCapacity) as
                    libc::c_ulong).wrapping_mul(::core::mem::size_of::<fd_set>()
                                                    as libc::c_ulong));
        i += 1
    };
}
unsafe extern "C" fn select_zero(mut s: *mut SelectSet) {
    let mut i: libc::c_int = 0;
    if (*s).capacity == 0i32 { return }
    (*s).maxfd = 0i32;
    i = 0i32;
    while i < SELECT_MAX as libc::c_int {
        memset((*s).fds[i as usize] as *mut libc::c_void, 0i32,
               ((*s).capacity as
                    libc::c_ulong).wrapping_mul(::core::mem::size_of::<fd_set>()
                                                    as libc::c_ulong));
        i += 1
    };
}
unsafe extern "C" fn select_add(mut s: *mut SelectSet, mut set: libc::c_int,
                                mut fd: dyad_Socket) {
    let mut p: *mut libc::c_uint = 0 as *mut libc::c_uint;
    while (*s).capacity * 1024i32 < fd { select_grow(s); }
    p = (*s).fds[set as usize] as *mut libc::c_uint;
    *p.offset((fd as
                   libc::c_ulong).wrapping_div((::core::mem::size_of::<libc::c_uint>()
                                                    as
                                                    libc::c_ulong).wrapping_mul(8i32
                                                                                    as
                                                                                    libc::c_ulong))
                  as isize) |=
        (1i32 <<
             (fd as
                  libc::c_ulong).wrapping_rem((::core::mem::size_of::<libc::c_uint>()
                                                   as
                                                   libc::c_ulong).wrapping_mul(8i32
                                                                                   as
                                                                                   libc::c_ulong)))
            as libc::c_uint;
    if fd > (*s).maxfd { (*s).maxfd = fd };
}
unsafe extern "C" fn select_has(mut s: *mut SelectSet, mut set: libc::c_int,
                                mut fd: dyad_Socket) -> libc::c_int {
    let mut p: *mut libc::c_uint = 0 as *mut libc::c_uint;
    if (*s).maxfd < fd { return 0i32 }
    p = (*s).fds[set as usize] as *mut libc::c_uint;
    return (*p.offset((fd as
                           libc::c_ulong).wrapping_div((::core::mem::size_of::<libc::c_uint>()
                                                            as
                                                            libc::c_ulong).wrapping_mul(8i32
                                                                                            as
                                                                                            libc::c_ulong))
                          as isize) &
                (1i32 <<
                     (fd as
                          libc::c_ulong).wrapping_rem((::core::mem::size_of::<libc::c_uint>()
                                                           as
                                                           libc::c_ulong).wrapping_mul(8i32
                                                                                           as
                                                                                           libc::c_ulong)))
                    as libc::c_uint) as libc::c_int;
}
static mut dyad_streams: *mut dyad_Stream =
    0 as *const dyad_Stream as *mut dyad_Stream;
static mut dyad_streamCount: libc::c_int = 0;
static mut dyad_panicMsgBuffer: [libc::c_char; 128] = [0; 128];
static mut panicCallback: dyad_PanicCallback = None;
static mut dyad_selectSet: SelectSet =
    SelectSet{capacity: 0,
              maxfd: 0,
              fds: [0 as *const fd_set as *mut fd_set; 3],};
static mut dyad_updateTimeout: libc::c_double = 1i32 as libc::c_double;
static mut dyad_tickInterval: libc::c_double = 1i32 as libc::c_double;
static mut dyad_lastTick: libc::c_double = 0i32 as libc::c_double;
unsafe extern "C" fn panic(mut fmt: *const libc::c_char, mut args: ...) {
    let mut args_0: ::core::ffi::VaListImpl;
    args_0 = args.clone();
    vsprintf(dyad_panicMsgBuffer.as_mut_ptr(), fmt, args_0.as_va_list());
    if panicCallback.is_some() {
        panicCallback.expect("non-null function pointer")(dyad_panicMsgBuffer.as_mut_ptr());
    } else {
        printf(b"dyad panic: %s\n\x00" as *const u8 as *const libc::c_char,
               dyad_panicMsgBuffer.as_mut_ptr());
    }
    exit(1i32);
}
unsafe extern "C" fn createEvent(mut type_0: libc::c_int) -> dyad_Event {
    let mut e: dyad_Event =
        dyad_Event{type_0: 0,
                   udata: 0 as *mut libc::c_void,
                   stream: 0 as *mut dyad_Stream,
                   remote: 0 as *mut dyad_Stream,
                   msg: 0 as *const libc::c_char,
                   data: 0 as *mut libc::c_char,
                   size: 0,};
    memset(&mut e as *mut dyad_Event as *mut libc::c_void, 0i32,
           ::core::mem::size_of::<dyad_Event>() as libc::c_ulong);
    e.type_0 = type_0;
    return e;
}
unsafe extern "C" fn destroyClosedStreams() {
    let mut stream: *mut dyad_Stream = dyad_streams;
    while !stream.is_null() {
        if (*stream).state == DYAD_STATE_CLOSED as libc::c_int {
            let mut next: *mut dyad_Stream = (*stream).next;
            stream_destroy(stream);
            stream = next
        } else { stream = (*stream).next }
    };
}
unsafe extern "C" fn updateTickTimer() {
    /* Update tick timer */
    if dyad_lastTick == 0i32 as libc::c_double {
        dyad_lastTick = dyad_getTime()
    }
    while dyad_lastTick < dyad_getTime() {
        /* Emit event on all streams */
        let mut stream: *mut dyad_Stream = 0 as *mut dyad_Stream;
        let mut e: dyad_Event = createEvent(DYAD_EVENT_TICK as libc::c_int);
        e.msg = b"a tick has occured\x00" as *const u8 as *const libc::c_char;
        stream = dyad_streams;
        while !stream.is_null() {
            stream_emitEvent(stream, &mut e);
            stream = (*stream).next
        }
        dyad_lastTick += dyad_tickInterval
    };
}
unsafe extern "C" fn updateStreamTimeouts() {
    let mut currentTime: libc::c_double = dyad_getTime();
    let mut stream: *mut dyad_Stream = 0 as *mut dyad_Stream;
    let mut e: dyad_Event = createEvent(DYAD_EVENT_TIMEOUT as libc::c_int);
    e.msg = b"stream timed out\x00" as *const u8 as *const libc::c_char;
    stream = dyad_streams;
    while !stream.is_null() {
        if (*stream).timeout != 0. {
            if currentTime - (*stream).lastActivity > (*stream).timeout {
                stream_emitEvent(stream, &mut e);
                dyad_close(stream);
            }
        }
        stream = (*stream).next
    };
}
/*===========================================================================*/
/* Stream                                                                    */
/*===========================================================================*/
unsafe extern "C" fn stream_destroy(mut stream: *mut dyad_Stream) {
    let mut e: dyad_Event =
        dyad_Event{type_0: 0,
                   udata: 0 as *mut libc::c_void,
                   stream: 0 as *mut dyad_Stream,
                   remote: 0 as *mut dyad_Stream,
                   msg: 0 as *const libc::c_char,
                   data: 0 as *mut libc::c_char,
                   size: 0,};
    let mut next: *mut *mut dyad_Stream = 0 as *mut *mut dyad_Stream;
    /* Close socket */
    if (*stream).sockfd != -1i32 { close((*stream).sockfd); }
    /* Emit destroy event */
    e = createEvent(DYAD_EVENT_DESTROY as libc::c_int);
    e.msg =
        b"the stream has been destroyed\x00" as *const u8 as
            *const libc::c_char;
    stream_emitEvent(stream, &mut e);
    /* Remove from list and decrement count */
    next = &mut dyad_streams;
    while *next != stream { next = &mut (**next).next }
    *next = (*stream).next;
    dyad_streamCount -= 1;
    /* Destroy and free */
    dyad_free((*stream).listeners.data as *mut libc::c_void);
    dyad_free((*stream).lineBuffer.data as *mut libc::c_void);
    dyad_free((*stream).writeBuffer.data as *mut libc::c_void);
    dyad_free((*stream).address as *mut libc::c_void);
    dyad_free(stream as *mut libc::c_void);
}
unsafe extern "C" fn stream_emitEvent(mut stream: *mut dyad_Stream,
                                      mut e: *mut dyad_Event) {
    let mut i: libc::c_int = 0;
    (*e).stream = stream;
    i = 0i32;
    while i < (*stream).listeners.length {
        let mut listener: *mut Listener =
            &mut *(*stream).listeners.data.offset(i as isize) as
                *mut Listener;
        if (*listener).event == (*e).type_0 {
            (*e).udata = (*listener).udata;
            (*listener).callback.expect("non-null function pointer")(e);
        }
        /* Check to see if this listener was removed: If it was we decrement `i`
     * since the next listener will now be in this ones place */
        if listener !=
               &mut *(*stream).listeners.data.offset(i as isize) as
                   *mut Listener {
            i -= 1
        }
        i += 1
    };
}
unsafe extern "C" fn stream_error(mut stream: *mut dyad_Stream,
                                  mut msg: *const libc::c_char,
                                  mut err: libc::c_int) {
    let mut buf: [libc::c_char; 256] = [0; 256];
    let mut e: dyad_Event = createEvent(DYAD_EVENT_ERROR as libc::c_int);
    if err != 0 {
        sprintf(buf.as_mut_ptr(),
                b"%.160s (%.80s)\x00" as *const u8 as *const libc::c_char,
                msg, strerror(err));
        e.msg = buf.as_mut_ptr()
    } else { e.msg = msg }
    stream_emitEvent(stream, &mut e);
    dyad_close(stream);
}
unsafe extern "C" fn stream_initAddress(mut stream: *mut dyad_Stream) {
    let mut addr: C2RustUnnamed_6 =
        C2RustUnnamed_6{sa: sockaddr{sa_family: 0, sa_data: [0; 14],},};
    let mut size: socklen_t = 0;
    memset(&mut addr as *mut C2RustUnnamed_6 as *mut libc::c_void, 0i32,
           ::core::mem::size_of::<C2RustUnnamed_6>() as libc::c_ulong);
    size =
        ::core::mem::size_of::<C2RustUnnamed_6>() as libc::c_ulong as
            socklen_t;
    dyad_free((*stream).address as *mut libc::c_void);
    (*stream).address = 0 as *mut libc::c_char;
    if getpeername((*stream).sockfd,
                   __SOCKADDR_ARG{__sockaddr__:
                                      &mut addr.sa as *mut sockaddr,},
                   &mut size) == -1i32 {
        if getsockname((*stream).sockfd,
                       __SOCKADDR_ARG{__sockaddr__:
                                          &mut addr.sa as *mut sockaddr,},
                       &mut size) == -1i32 {
            return
        }
    }
    if addr.sas.ss_family as libc::c_int == 10i32 {
        (*stream).address =
            dyad_realloc(0 as *mut libc::c_void, 46i32) as *mut libc::c_char;
        inet_ntop(10i32,
                  &mut addr.sai6.sin6_addr as *mut in6_addr as
                      *const libc::c_void, (*stream).address,
                  46i32 as socklen_t);
        (*stream).port = __bswap_16(addr.sai6.sin6_port) as libc::c_int
    } else {
        (*stream).address =
            dyad_realloc(0 as *mut libc::c_void, 16i32) as *mut libc::c_char;
        inet_ntop(2i32,
                  &mut addr.sai.sin_addr as *mut in_addr as
                      *const libc::c_void, (*stream).address,
                  16i32 as socklen_t);
        (*stream).port = __bswap_16(addr.sai.sin_port) as libc::c_int
    };
}
unsafe extern "C" fn stream_setSocketNonBlocking(mut stream: *mut dyad_Stream,
                                                 mut opt: libc::c_int) {
    let mut flags: libc::c_int = fcntl((*stream).sockfd, 3i32);
    fcntl((*stream).sockfd, 4i32,
          if opt != 0 { (flags) | 0o4000i32 } else { (flags) & !0o4000i32 });
}
unsafe extern "C" fn stream_setSocket(mut stream: *mut dyad_Stream,
                                      mut sockfd: dyad_Socket) {
    (*stream).sockfd = sockfd;
    stream_setSocketNonBlocking(stream, 1i32);
    stream_initAddress(stream);
}
unsafe extern "C" fn stream_initSocket(mut stream: *mut dyad_Stream,
                                       mut domain: libc::c_int,
                                       mut type_0: libc::c_int,
                                       mut protocol: libc::c_int)
 -> libc::c_int {
    (*stream).sockfd = socket(domain, type_0, protocol);
    if (*stream).sockfd == -1i32 {
        stream_error(stream,
                     b"could not create socket\x00" as *const u8 as
                         *const libc::c_char, *__errno_location());
        return -1i32
    }
    stream_setSocket(stream, (*stream).sockfd);
    return 0i32;
}
unsafe extern "C" fn stream_hasListenerForEvent(mut stream: *mut dyad_Stream,
                                                mut event: libc::c_int)
 -> libc::c_int {
    let mut i: libc::c_int = 0;
    i = 0i32;
    while i < (*stream).listeners.length {
        let mut listener: *mut Listener =
            &mut *(*stream).listeners.data.offset(i as isize) as
                *mut Listener;
        if (*listener).event == event { return 1i32 }
        i += 1
    }
    return 0i32;
}
unsafe extern "C" fn stream_handleReceivedData(mut stream: *mut dyad_Stream) {
    loop  {
        /* Receive data */
        let mut e: dyad_Event =
            dyad_Event{type_0: 0,
                       udata: 0 as *mut libc::c_void,
                       stream: 0 as *mut dyad_Stream,
                       remote: 0 as *mut dyad_Stream,
                       msg: 0 as *const libc::c_char,
                       data: 0 as *mut libc::c_char,
                       size: 0,};
        let mut data: [libc::c_char; 8192] = [0; 8192];
        let mut size: libc::c_int =
            recv((*stream).sockfd, data.as_mut_ptr() as *mut libc::c_void,
                 (::core::mem::size_of::<[libc::c_char; 8192]>() as
                      libc::c_ulong).wrapping_sub(1i32 as libc::c_ulong),
                 0i32) as libc::c_int;
        if size <= 0i32 {
            if size == 0i32 || *__errno_location() != 11i32 {
                /* Handle disconnect */
                dyad_close(stream);
                return
            } else {
                /* No more data */
                return
            }
        }
        data[size as usize] = 0i32 as libc::c_char;
        /* Update status */
        (*stream).bytesReceived += size;
        (*stream).lastActivity = dyad_getTime();
        /* Emit data event */
        e = createEvent(DYAD_EVENT_DATA as libc::c_int);
        e.msg = b"received data\x00" as *const u8 as *const libc::c_char;
        e.data = data.as_mut_ptr();
        e.size = size;
        stream_emitEvent(stream, &mut e);
        /* Check stream state in case it was closed during one of the data event
     * handlers. */
        if (*stream).state != DYAD_STATE_CONNECTED as libc::c_int { return }
        /* Handle line event */
        if stream_hasListenerForEvent(stream, DYAD_EVENT_LINE as libc::c_int)
               != 0 {
            let mut i: libc::c_int = 0;
            let mut start: libc::c_int = 0;
            let mut buf: *mut libc::c_char = 0 as *mut libc::c_char;
            i = 0i32;
            while i < size {
                vec_expand(&mut (*stream).lineBuffer.data as
                               *mut *mut libc::c_char,
                           &mut (*stream).lineBuffer.length,
                           &mut (*stream).lineBuffer.capacity,
                           ::core::mem::size_of::<libc::c_char>() as
                               libc::c_ulong as libc::c_int);
                let fresh0 = (*stream).lineBuffer.length;
                (*stream).lineBuffer.length = (*stream).lineBuffer.length + 1;
                *(*stream).lineBuffer.data.offset(fresh0 as isize) =
                    data[i as usize];
                i += 1
            }
            start = 0i32;
            buf = (*stream).lineBuffer.data;
            i = 0i32;
            while i < (*stream).lineBuffer.length {
                if *buf.offset(i as isize) as libc::c_int == '\n' as i32 {
                    let mut e_0: dyad_Event =
                        dyad_Event{type_0: 0,
                                   udata: 0 as *mut libc::c_void,
                                   stream: 0 as *mut dyad_Stream,
                                   remote: 0 as *mut dyad_Stream,
                                   msg: 0 as *const libc::c_char,
                                   data: 0 as *mut libc::c_char,
                                   size: 0,};
                    *buf.offset(i as isize) = '\u{0}' as i32 as libc::c_char;
                    e_0 = createEvent(DYAD_EVENT_LINE as libc::c_int);
                    e_0.msg =
                        b"received line\x00" as *const u8 as
                            *const libc::c_char;
                    e_0.data =
                        &mut *buf.offset(start as isize) as *mut libc::c_char;
                    e_0.size = i - start;
                    /* Check and strip carriage return */
                    if e_0.size > 0i32 &&
                           *e_0.data.offset((e_0.size - 1i32) as isize) as
                               libc::c_int == '\r' as i32 {
                        e_0.size -= 1;
                        *e_0.data.offset(e_0.size as isize) =
                            '\u{0}' as i32 as libc::c_char
                    }
                    stream_emitEvent(stream, &mut e_0);
                    start = i + 1i32;
                    /* Check stream state in case it was closed during one of the line
           * event handlers. */
                    if (*stream).state != DYAD_STATE_CONNECTED as libc::c_int
                       {
                        return
                    }
                }
                i += 1
            }
            if start == (*stream).lineBuffer.length {
                (*stream).lineBuffer.length = 0i32
            } else {
                vec_splice(&mut (*stream).lineBuffer.data as
                               *mut *mut libc::c_char,
                           &mut (*stream).lineBuffer.length,
                           &mut (*stream).lineBuffer.capacity,
                           ::core::mem::size_of::<libc::c_char>() as
                               libc::c_ulong as libc::c_int, 0i32, start);
                (*stream).lineBuffer.length -= start
            }
        }
    };
}
unsafe extern "C" fn stream_acceptPendingConnections(mut stream:
                                                         *mut dyad_Stream) {
    loop  {
        let mut remote: *mut dyad_Stream = 0 as *mut dyad_Stream;
        let mut e: dyad_Event =
            dyad_Event{type_0: 0,
                       udata: 0 as *mut libc::c_void,
                       stream: 0 as *mut dyad_Stream,
                       remote: 0 as *mut dyad_Stream,
                       msg: 0 as *const libc::c_char,
                       data: 0 as *mut libc::c_char,
                       size: 0,};
        let mut err: libc::c_int = 0i32;
        let mut sockfd: dyad_Socket =
            accept((*stream).sockfd,
                   __SOCKADDR_ARG{__sockaddr__:
                                      0 as *mut libc::c_void as
                                          *mut sockaddr,},
                   0 as *mut socklen_t);
        if sockfd == -1i32 {
            err = *__errno_location();
            if err == 11i32 {
                /* No more waiting sockets */
                return
            }
        }
        /* Create client stream */
        remote = dyad_newStream();
        (*remote).state = DYAD_STATE_CONNECTED as libc::c_int;
        /* Set stream's socket */
        stream_setSocket(remote, sockfd);
        /* Emit accept event */
        e = createEvent(DYAD_EVENT_ACCEPT as libc::c_int);
        e.msg =
            b"accepted connection\x00" as *const u8 as *const libc::c_char;
        e.remote = remote;
        stream_emitEvent(stream, &mut e);
        /* Handle invalid socket -- the stream is still made and the ACCEPT event
     * is still emitted, but its shut immediately with an error */
        if (*remote).sockfd == -1i32 {
            stream_error(remote,
                         b"failed to create socket on accept\x00" as *const u8
                             as *const libc::c_char, err);
            return
        }
    };
}
unsafe extern "C" fn stream_flushWriteBuffer(mut stream: *mut dyad_Stream)
 -> libc::c_int {
    (*stream).flags &= !(1i32 << 1i32);
    if (*stream).writeBuffer.length > 0i32 {
        /* Send data */
        let mut size: libc::c_int =
            send((*stream).sockfd,
                 (*stream).writeBuffer.data as *const libc::c_void,
                 (*stream).writeBuffer.length as size_t, 0i32) as libc::c_int;
        if size <= 0i32 {
            if *__errno_location() == 11i32 {
                /* No more data can be written */
                return 0i32
            } else {
                /* Handle disconnect */
                dyad_close(stream);
                return 0i32
            }
        }
        if size == (*stream).writeBuffer.length {
            (*stream).writeBuffer.length = 0i32
        } else {
            vec_splice(&mut (*stream).writeBuffer.data as
                           *mut *mut libc::c_char,
                       &mut (*stream).writeBuffer.length,
                       &mut (*stream).writeBuffer.capacity,
                       ::core::mem::size_of::<libc::c_char>() as libc::c_ulong
                           as libc::c_int, 0i32, size);
            (*stream).writeBuffer.length -= size
        }
        /* Update status */
        (*stream).bytesSent += size;
        (*stream).lastActivity = dyad_getTime()
    }
    if (*stream).writeBuffer.length == 0i32 {
        let mut e: dyad_Event =
            dyad_Event{type_0: 0,
                       udata: 0 as *mut libc::c_void,
                       stream: 0 as *mut dyad_Stream,
                       remote: 0 as *mut dyad_Stream,
                       msg: 0 as *const libc::c_char,
                       data: 0 as *mut libc::c_char,
                       size: 0,};
        /* If this is a 'closing' stream we can properly close it now */
        if (*stream).state == DYAD_STATE_CLOSING as libc::c_int {
            dyad_close(stream);
            return 0i32
        }
        /* Set ready flag and emit 'ready for data' event */
        (*stream).flags |= 1i32 << 0i32;
        e = createEvent(DYAD_EVENT_READY as libc::c_int);
        e.msg =
            b"stream is ready for more data\x00" as *const u8 as
                *const libc::c_char;
        stream_emitEvent(stream, &mut e);
    }
    /* Return 1 to indicate that more data can immediately be written to the
   * stream's socket */
    return 1i32;
}
/*===========================================================================*/
/* API                                                                       */
/*===========================================================================*/
/*---------------------------------------------------------------------------*/
/* Core                                                                      */
/*---------------------------------------------------------------------------*/
#[no_mangle]
pub unsafe extern "C" fn dyad_update() {
    let mut stream: *mut dyad_Stream = 0 as *mut dyad_Stream;
    let mut tv: timeval = timeval{tv_sec: 0, tv_usec: 0,};
    destroyClosedStreams();
    updateTickTimer();
    updateStreamTimeouts();
    /* Create fd sets for select() */
    select_zero(&mut dyad_selectSet);
    stream = dyad_streams;
    while !stream.is_null() {
        match (*stream).state {
            3 => {
                select_add(&mut dyad_selectSet, SELECT_READ as libc::c_int,
                           (*stream).sockfd);
                if (*stream).flags & 1i32 << 0i32 == 0 ||
                       (*stream).writeBuffer.length != 0i32 {
                    select_add(&mut dyad_selectSet,
                               SELECT_WRITE as libc::c_int, (*stream).sockfd);
                }
            }
            1 => {
                select_add(&mut dyad_selectSet, SELECT_WRITE as libc::c_int,
                           (*stream).sockfd);
            }
            2 => {
                select_add(&mut dyad_selectSet, SELECT_WRITE as libc::c_int,
                           (*stream).sockfd);
                select_add(&mut dyad_selectSet, SELECT_EXCEPT as libc::c_int,
                           (*stream).sockfd);
            }
            4 => {
                select_add(&mut dyad_selectSet, SELECT_READ as libc::c_int,
                           (*stream).sockfd);
            }
            _ => { }
        }
        stream = (*stream).next
    }
    /* Init timeout value and do select */
    tv.tv_sec = dyad_updateTimeout as __time_t;
    tv.tv_usec =
        ((dyad_updateTimeout - tv.tv_sec as libc::c_double) * 1e6f64) as
            __suseconds_t;
    select(dyad_selectSet.maxfd + 1i32,
           dyad_selectSet.fds[SELECT_READ as libc::c_int as usize],
           dyad_selectSet.fds[SELECT_WRITE as libc::c_int as usize],
           dyad_selectSet.fds[SELECT_EXCEPT as libc::c_int as usize],
           &mut tv);
    /* Handle streams */
    stream = dyad_streams;
    while !stream.is_null() {
        let mut current_block_36: u64;
        match (*stream).state {
            3 => {
                if select_has(&mut dyad_selectSet, SELECT_READ as libc::c_int,
                              (*stream).sockfd) != 0 {
                    stream_handleReceivedData(stream);
                    if (*stream).state == DYAD_STATE_CLOSED as libc::c_int {
                        current_block_36 = 11793792312832361944;
                    } else { current_block_36 = 11000567119642394172; }
                } else { current_block_36 = 11000567119642394172; }
            }
            1 => { current_block_36 = 11000567119642394172; }
            2 => {
                let mut current_block_32: u64;
                if select_has(&mut dyad_selectSet,
                              SELECT_WRITE as libc::c_int, (*stream).sockfd)
                       != 0 {
                    /* Check socket for error */
                    let mut optval: libc::c_int = 0i32;
                    let mut optlen: socklen_t =
                        ::core::mem::size_of::<libc::c_int>() as libc::c_ulong
                            as socklen_t;
                    let mut e: dyad_Event =
                        dyad_Event{type_0: 0,
                                   udata: 0 as *mut libc::c_void,
                                   stream: 0 as *mut dyad_Stream,
                                   remote: 0 as *mut dyad_Stream,
                                   msg: 0 as *const libc::c_char,
                                   data: 0 as *mut libc::c_char,
                                   size: 0,};
                    getsockopt((*stream).sockfd, 1i32, 4i32,
                               &mut optval as *mut libc::c_int as
                                   *mut libc::c_void, &mut optlen);
                    if optval != 0i32 {
                        current_block_32 = 9979783880681039505;
                    } else {
                        /* Handle succeselful connection */
                        (*stream).state = DYAD_STATE_CONNECTED as libc::c_int;
                        (*stream).lastActivity = dyad_getTime();
                        stream_initAddress(stream);
                        /* Emit connect event */
                        e = createEvent(DYAD_EVENT_CONNECT as libc::c_int);
                        e.msg =
                            b"connected to server\x00" as *const u8 as
                                *const libc::c_char;
                        stream_emitEvent(stream, &mut e);
                        current_block_32 = 6717214610478484138;
                    }
                } else if select_has(&mut dyad_selectSet,
                                     SELECT_EXCEPT as libc::c_int,
                                     (*stream).sockfd) != 0 {
                    current_block_32 = 9979783880681039505;
                } else { current_block_32 = 6717214610478484138; }
                match current_block_32 {
                    9979783880681039505 =>
                    /* Handle failed connection */
                    {
                        stream_error(stream,
                                     b"could not connect to server\x00" as
                                         *const u8 as *const libc::c_char,
                                     0i32);
                    }
                    _ => { }
                }
                current_block_36 = 11793792312832361944;
            }
            4 => {
                if select_has(&mut dyad_selectSet, SELECT_READ as libc::c_int,
                              (*stream).sockfd) != 0 {
                    stream_acceptPendingConnections(stream);
                }
                current_block_36 = 11793792312832361944;
            }
            _ => { current_block_36 = 11793792312832361944; }
        }
        match current_block_36 {
            11000567119642394172 =>
            /* Fall through */
            {
                if select_has(&mut dyad_selectSet,
                              SELECT_WRITE as libc::c_int, (*stream).sockfd)
                       != 0 {
                    stream_flushWriteBuffer(stream);
                }
            }
            _ => { }
        }
        /* If data was just now written to the stream we should immediately try to
     * send it */
        if (*stream).flags & 1i32 << 1i32 != 0 &&
               (*stream).state != DYAD_STATE_CLOSED as libc::c_int {
            stream_flushWriteBuffer(stream);
        }
        stream = (*stream).next
    };
}
#[no_mangle]
pub unsafe extern "C" fn dyad_init() {
    /* Stops the SIGPIPE signal being raised when writing to a closed socket */
    signal(13i32,
           ::core::mem::transmute::<libc::intptr_t,
                                    __sighandler_t>(1i32 as libc::intptr_t));
}
#[no_mangle]
pub unsafe extern "C" fn dyad_shutdown() {
    /* Close and destroy all the streams */
    while !dyad_streams.is_null() {
        dyad_close(dyad_streams);
        stream_destroy(dyad_streams);
    }
    /* Clear up everything */
    select_deinit(&mut dyad_selectSet);
}
#[no_mangle]
pub unsafe extern "C" fn dyad_getVersion() -> *const libc::c_char {
    return b"0.2.1\x00" as *const u8 as *const libc::c_char;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_getTime() -> libc::c_double {
    let mut tv: timeval = timeval{tv_sec: 0, tv_usec: 0,};
    gettimeofday(&mut tv, 0 as *mut timezone);
    return tv.tv_sec as libc::c_double +
               tv.tv_usec as libc::c_double / 1e6f64;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_getStreamCount() -> libc::c_int {
    return dyad_streamCount;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_setTickInterval(mut seconds: libc::c_double) {
    dyad_tickInterval = seconds;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_setUpdateTimeout(mut seconds: libc::c_double) {
    dyad_updateTimeout = seconds;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_atPanic(mut func: dyad_PanicCallback)
 -> dyad_PanicCallback {
    let mut old: dyad_PanicCallback = panicCallback;
    panicCallback = func;
    return old;
}
/*---------------------------------------------------------------------------*/
/* Stream                                                                    */
/*---------------------------------------------------------------------------*/
#[no_mangle]
pub unsafe extern "C" fn dyad_newStream() -> *mut dyad_Stream {
    let mut stream: *mut dyad_Stream =
        dyad_realloc(0 as *mut libc::c_void,
                     ::core::mem::size_of::<dyad_Stream>() as libc::c_ulong as
                         libc::c_int) as *mut dyad_Stream;
    memset(stream as *mut libc::c_void, 0i32,
           ::core::mem::size_of::<dyad_Stream>() as libc::c_ulong);
    (*stream).state = DYAD_STATE_CLOSED as libc::c_int;
    (*stream).sockfd = -1i32;
    (*stream).lastActivity = dyad_getTime();
    /* Add to list and increment count */
    (*stream).next = dyad_streams;
    dyad_streams = stream;
    dyad_streamCount += 1;
    return stream;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_addListener(mut stream: *mut dyad_Stream,
                                          mut event: libc::c_int,
                                          mut callback: dyad_Callback,
                                          mut udata: *mut libc::c_void) {
    let mut listener: Listener =
        Listener{event: 0, callback: None, udata: 0 as *mut libc::c_void,};
    listener.event = event;
    listener.callback = callback;
    listener.udata = udata;
    vec_expand(&mut (*stream).listeners.data as *mut *mut Listener as
                   *mut *mut libc::c_char, &mut (*stream).listeners.length,
               &mut (*stream).listeners.capacity,
               ::core::mem::size_of::<Listener>() as libc::c_ulong as
                   libc::c_int);
    let fresh1 = (*stream).listeners.length;
    (*stream).listeners.length = (*stream).listeners.length + 1;
    *(*stream).listeners.data.offset(fresh1 as isize) = listener;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_removeListener(mut stream: *mut dyad_Stream,
                                             mut event: libc::c_int,
                                             mut callback: dyad_Callback,
                                             mut udata: *mut libc::c_void) {
    let mut i: libc::c_int = (*stream).listeners.length;
    loop  {
        let fresh2 = i;
        i = i - 1;
        if !(fresh2 != 0) { break ; }
        let mut x: *mut Listener =
            &mut *(*stream).listeners.data.offset(i as isize) as
                *mut Listener;
        if (*x).event == event && (*x).callback == callback &&
               (*x).udata == udata {
            vec_splice(&mut (*stream).listeners.data as *mut *mut Listener as
                           *mut *mut libc::c_char,
                       &mut (*stream).listeners.length,
                       &mut (*stream).listeners.capacity,
                       ::core::mem::size_of::<Listener>() as libc::c_ulong as
                           libc::c_int, i, 1i32);
            (*stream).listeners.length -= 1i32
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn dyad_removeAllListeners(mut stream: *mut dyad_Stream,
                                                 mut event: libc::c_int) {
    if event == DYAD_EVENT_NULL as libc::c_int {
        (*stream).listeners.length = 0i32
    } else {
        let mut i: libc::c_int = (*stream).listeners.length;
        loop  {
            let fresh3 = i;
            i = i - 1;
            if !(fresh3 != 0) { break ; }
            if (*(*stream).listeners.data.offset(i as isize)).event == event {
                vec_splice(&mut (*stream).listeners.data as *mut *mut Listener
                               as *mut *mut libc::c_char,
                           &mut (*stream).listeners.length,
                           &mut (*stream).listeners.capacity,
                           ::core::mem::size_of::<Listener>() as libc::c_ulong
                               as libc::c_int, i, 1i32);
                (*stream).listeners.length -= 1i32
            }
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn dyad_close(mut stream: *mut dyad_Stream) {
    let mut e: dyad_Event =
        dyad_Event{type_0: 0,
                   udata: 0 as *mut libc::c_void,
                   stream: 0 as *mut dyad_Stream,
                   remote: 0 as *mut dyad_Stream,
                   msg: 0 as *const libc::c_char,
                   data: 0 as *mut libc::c_char,
                   size: 0,};
    if (*stream).state == DYAD_STATE_CLOSED as libc::c_int { return }
    (*stream).state = DYAD_STATE_CLOSED as libc::c_int;
    /* Close socket */
    if (*stream).sockfd != -1i32 {
        close((*stream).sockfd);
        (*stream).sockfd = -1i32
    }
    /* Emit event */
    e = createEvent(DYAD_EVENT_CLOSE as libc::c_int);
    e.msg = b"stream closed\x00" as *const u8 as *const libc::c_char;
    stream_emitEvent(stream, &mut e);
    /* Clear buffers */
    (*stream).lineBuffer.length = 0i32;
    (*stream).writeBuffer.length = 0i32;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_end(mut stream: *mut dyad_Stream) {
    if (*stream).state == DYAD_STATE_CLOSED as libc::c_int { return }
    if (*stream).writeBuffer.length > 0i32 {
        (*stream).state = DYAD_STATE_CLOSING as libc::c_int
    } else { dyad_close(stream); };
}
#[no_mangle]
pub unsafe extern "C" fn dyad_listenEx(mut stream: *mut dyad_Stream,
                                       mut host: *const libc::c_char,
                                       mut port: libc::c_int,
                                       mut backlog: libc::c_int)
 -> libc::c_int {
    let mut hints: addrinfo =
        addrinfo{ai_flags: 0,
                 ai_family: 0,
                 ai_socktype: 0,
                 ai_protocol: 0,
                 ai_addrlen: 0,
                 ai_addr: 0 as *mut sockaddr,
                 ai_canonname: 0 as *mut libc::c_char,
                 ai_next: 0 as *mut addrinfo,};
    let mut ai: *mut addrinfo = 0 as *mut addrinfo;
    let mut err: libc::c_int = 0;
    let mut optval: libc::c_int = 0;
    let mut buf: [libc::c_char; 64] = [0; 64];
    let mut e: dyad_Event =
        dyad_Event{type_0: 0,
                   udata: 0 as *mut libc::c_void,
                   stream: 0 as *mut dyad_Stream,
                   remote: 0 as *mut dyad_Stream,
                   msg: 0 as *const libc::c_char,
                   data: 0 as *mut libc::c_char,
                   size: 0,};
    /* Get addrinfo */
    memset(&mut hints as *mut addrinfo as *mut libc::c_void, 0i32,
           ::core::mem::size_of::<addrinfo>() as libc::c_ulong);
    hints.ai_family = 0i32;
    hints.ai_socktype = SOCK_STREAM as libc::c_int;
    hints.ai_flags = 0x1i32;
    sprintf(buf.as_mut_ptr(), b"%d\x00" as *const u8 as *const libc::c_char,
            port);
    err = getaddrinfo(host, buf.as_mut_ptr(), &mut hints, &mut ai);
    if err != 0 {
        stream_error(stream,
                     b"could not get addrinfo\x00" as *const u8 as
                         *const libc::c_char, *__errno_location());
    } else {
        /* Init socket */
        err =
            stream_initSocket(stream, (*ai).ai_family, (*ai).ai_socktype,
                              (*ai).ai_protocol);
        if !(err != 0) {
            /* Set SO_REUSEADDR so that the socket can be immediately bound without
   * having to wait for any closed socket on the same port to timeout */
            optval = 1i32;
            setsockopt((*stream).sockfd, 1i32, 2i32,
                       &mut optval as *mut libc::c_int as *const libc::c_void,
                       ::core::mem::size_of::<libc::c_int>() as libc::c_ulong
                           as socklen_t);
            /* Bind and listen */
            err =
                bind((*stream).sockfd,
                     __CONST_SOCKADDR_ARG{__sockaddr__: (*ai).ai_addr,},
                     (*ai).ai_addrlen);
            if err != 0 {
                stream_error(stream,
                             b"could not bind socket\x00" as *const u8 as
                                 *const libc::c_char, *__errno_location());
            } else {
                err = listen((*stream).sockfd, backlog);
                if err != 0 {
                    stream_error(stream,
                                 b"socket failed on listen\x00" as *const u8
                                     as *const libc::c_char,
                                 *__errno_location());
                } else {
                    (*stream).state = DYAD_STATE_LISTENING as libc::c_int;
                    (*stream).port = port;
                    stream_initAddress(stream);
                    /* Emit listening event */
                    e = createEvent(DYAD_EVENT_LISTEN as libc::c_int);
                    e.msg =
                        b"socket is listening\x00" as *const u8 as
                            *const libc::c_char;
                    stream_emitEvent(stream, &mut e);
                    freeaddrinfo(ai);
                    return 0i32
                }
            }
        }
    }
    if !ai.is_null() { freeaddrinfo(ai); }
    return -1i32;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_listen(mut stream: *mut dyad_Stream,
                                     mut port: libc::c_int) -> libc::c_int {
    return dyad_listenEx(stream, 0 as *const libc::c_char, port, 511i32);
}
#[no_mangle]
pub unsafe extern "C" fn dyad_connect(mut stream: *mut dyad_Stream,
                                      mut host: *const libc::c_char,
                                      mut port: libc::c_int) -> libc::c_int {
    let mut hints: addrinfo =
        addrinfo{ai_flags: 0,
                 ai_family: 0,
                 ai_socktype: 0,
                 ai_protocol: 0,
                 ai_addrlen: 0,
                 ai_addr: 0 as *mut sockaddr,
                 ai_canonname: 0 as *mut libc::c_char,
                 ai_next: 0 as *mut addrinfo,};
    let mut ai: *mut addrinfo = 0 as *mut addrinfo;
    let mut err: libc::c_int = 0;
    let mut buf: [libc::c_char; 64] = [0; 64];
    /* Resolve host */
    memset(&mut hints as *mut addrinfo as *mut libc::c_void, 0i32,
           ::core::mem::size_of::<addrinfo>() as libc::c_ulong);
    hints.ai_family = 0i32;
    hints.ai_socktype = SOCK_STREAM as libc::c_int;
    sprintf(buf.as_mut_ptr(), b"%d\x00" as *const u8 as *const libc::c_char,
            port);
    err = getaddrinfo(host, buf.as_mut_ptr(), &mut hints, &mut ai);
    if err != 0 {
        stream_error(stream,
                     b"could not resolve host\x00" as *const u8 as
                         *const libc::c_char, 0i32);
    } else {
        /* Start connecting */
        err =
            stream_initSocket(stream, (*ai).ai_family, (*ai).ai_socktype,
                              (*ai).ai_protocol);
        if !(err != 0) {
            connect((*stream).sockfd,
                    __CONST_SOCKADDR_ARG{__sockaddr__: (*ai).ai_addr,},
                    (*ai).ai_addrlen);
            (*stream).state = DYAD_STATE_CONNECTING as libc::c_int;
            freeaddrinfo(ai);
            return 0i32
        }
    }
    if !ai.is_null() { freeaddrinfo(ai); }
    return -1i32;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_write(mut stream: *mut dyad_Stream,
                                    mut data: *const libc::c_void,
                                    mut size: libc::c_int) {
    let mut p: *const libc::c_char = data as *const libc::c_char;
    loop  {
        let fresh4 = size;
        size = size - 1;
        if !(fresh4 != 0) { break ; }
        vec_expand(&mut (*stream).writeBuffer.data as *mut *mut libc::c_char,
                   &mut (*stream).writeBuffer.length,
                   &mut (*stream).writeBuffer.capacity,
                   ::core::mem::size_of::<libc::c_char>() as libc::c_ulong as
                       libc::c_int);
        let fresh5 = p;
        p = p.offset(1);
        let fresh6 = (*stream).writeBuffer.length;
        (*stream).writeBuffer.length = (*stream).writeBuffer.length + 1;
        *(*stream).writeBuffer.data.offset(fresh6 as isize) = *fresh5
    }
    (*stream).flags |= 1i32 << 1i32;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_vwritef(mut stream: *mut dyad_Stream,
                                      mut fmt: *const libc::c_char,
                                      mut args: ::core::ffi::VaList) {
    let mut current_block: u64;
    let mut buf: [libc::c_char; 512] = [0; 512];
    let mut str: *mut libc::c_char = 0 as *mut libc::c_char;
    let mut f: [libc::c_char; 3] =
        *::core::mem::transmute::<&[u8; 3],
                                  &mut [libc::c_char; 3]>(b"%_\x00");
    let mut fp: *mut FILE = 0 as *mut FILE;
    let mut c: libc::c_int = 0;
    while *fmt != 0 {
        if *fmt as libc::c_int == '%' as i32 {
            fmt = fmt.offset(1);
            match *fmt as libc::c_int {
                114 => {
                    fp = args.arg::<*mut FILE>();
                    if fp.is_null() {
                        str =
                            b"(null)\x00" as *const u8 as *const libc::c_char
                                as *mut libc::c_char;
                        current_block = 3805594055242509767;
                    } else {
                        loop  {
                            c = fgetc(fp);
                            if !(c != -1i32) { break ; }
                            vec_expand(&mut (*stream).writeBuffer.data as
                                           *mut *mut libc::c_char,
                                       &mut (*stream).writeBuffer.length,
                                       &mut (*stream).writeBuffer.capacity,
                                       ::core::mem::size_of::<libc::c_char>()
                                           as libc::c_ulong as libc::c_int);
                            let fresh7 = (*stream).writeBuffer.length;
                            (*stream).writeBuffer.length =
                                (*stream).writeBuffer.length + 1;
                            *(*stream).writeBuffer.data.offset(fresh7 as
                                                                   isize) =
                                c as libc::c_char
                        }
                        current_block = 3938820862080741272;
                    }
                }
                99 => {
                    vec_expand(&mut (*stream).writeBuffer.data as
                                   *mut *mut libc::c_char,
                               &mut (*stream).writeBuffer.length,
                               &mut (*stream).writeBuffer.capacity,
                               ::core::mem::size_of::<libc::c_char>() as
                                   libc::c_ulong as libc::c_int);
                    let fresh8 = (*stream).writeBuffer.length;
                    (*stream).writeBuffer.length =
                        (*stream).writeBuffer.length + 1;
                    *(*stream).writeBuffer.data.offset(fresh8 as isize) =
                        args.arg::<libc::c_int>() as libc::c_char;
                    current_block = 3938820862080741272;
                }
                115 => {
                    str = args.arg::<*mut libc::c_char>();
                    if str.is_null() {
                        str =
                            b"(null)\x00" as *const u8 as *const libc::c_char
                                as *mut libc::c_char
                    }
                    current_block = 3805594055242509767;
                }
                98 => {
                    str = args.arg::<*mut libc::c_char>();
                    c = args.arg::<libc::c_int>();
                    loop  {
                        let fresh11 = c;
                        c = c - 1;
                        if !(fresh11 != 0) { break ; }
                        vec_expand(&mut (*stream).writeBuffer.data as
                                       *mut *mut libc::c_char,
                                   &mut (*stream).writeBuffer.length,
                                   &mut (*stream).writeBuffer.capacity,
                                   ::core::mem::size_of::<libc::c_char>() as
                                       libc::c_ulong as libc::c_int);
                        let fresh12 = str;
                        str = str.offset(1);
                        let fresh13 = (*stream).writeBuffer.length;
                        (*stream).writeBuffer.length =
                            (*stream).writeBuffer.length + 1;
                        *(*stream).writeBuffer.data.offset(fresh13 as isize) =
                            *fresh12
                    }
                    current_block = 3938820862080741272;
                }
                _ => {
                    f[1] = *fmt;
                    match *fmt as libc::c_int {
                        102 | 103 => {
                            sprintf(buf.as_mut_ptr(), f.as_mut_ptr(),
                                    args.arg::<libc::c_double>());
                        }
                        100 | 105 => {
                            sprintf(buf.as_mut_ptr(), f.as_mut_ptr(),
                                    args.arg::<libc::c_int>());
                        }
                        120 | 88 => {
                            sprintf(buf.as_mut_ptr(), f.as_mut_ptr(),
                                    args.arg::<libc::c_uint>());
                        }
                        112 => {
                            sprintf(buf.as_mut_ptr(), f.as_mut_ptr(),
                                    args.arg::<*mut libc::c_void>());
                        }
                        _ => {
                            buf[0] = *fmt;
                            buf[1] = '\u{0}' as i32 as libc::c_char
                        }
                    }
                    str = buf.as_mut_ptr();
                    current_block = 3805594055242509767;
                }
            }
            match current_block {
                3938820862080741272 => { }
                _ => {
                    while *str != 0 {
                        vec_expand(&mut (*stream).writeBuffer.data as
                                       *mut *mut libc::c_char,
                                   &mut (*stream).writeBuffer.length,
                                   &mut (*stream).writeBuffer.capacity,
                                   ::core::mem::size_of::<libc::c_char>() as
                                       libc::c_ulong as libc::c_int);
                        let fresh9 = str;
                        str = str.offset(1);
                        let fresh10 = (*stream).writeBuffer.length;
                        (*stream).writeBuffer.length =
                            (*stream).writeBuffer.length + 1;
                        *(*stream).writeBuffer.data.offset(fresh10 as isize) =
                            *fresh9
                    }
                }
            }
        } else {
            vec_expand(&mut (*stream).writeBuffer.data as
                           *mut *mut libc::c_char,
                       &mut (*stream).writeBuffer.length,
                       &mut (*stream).writeBuffer.capacity,
                       ::core::mem::size_of::<libc::c_char>() as libc::c_ulong
                           as libc::c_int);
            let fresh14 = (*stream).writeBuffer.length;
            (*stream).writeBuffer.length = (*stream).writeBuffer.length + 1;
            *(*stream).writeBuffer.data.offset(fresh14 as isize) = *fmt
        }
        fmt = fmt.offset(1)
    }
    (*stream).flags |= 1i32 << 1i32;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_writef(mut stream: *mut dyad_Stream,
                                     mut fmt: *const libc::c_char,
                                     mut args: ...) {
    let mut args_0: ::core::ffi::VaListImpl;
    args_0 = args.clone();
    dyad_vwritef(stream, fmt, args_0.as_va_list());
}
#[no_mangle]
pub unsafe extern "C" fn dyad_setTimeout(mut stream: *mut dyad_Stream,
                                         mut seconds: libc::c_double) {
    (*stream).timeout = seconds;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_setNoDelay(mut stream: *mut dyad_Stream,
                                         mut opt: libc::c_int) {
    opt = (opt != 0) as libc::c_int;
    setsockopt((*stream).sockfd, IPPROTO_TCP as libc::c_int, 1i32,
               &mut opt as *mut libc::c_int as *const libc::c_void,
               ::core::mem::size_of::<libc::c_int>() as libc::c_ulong as
                   socklen_t);
}
#[no_mangle]
pub unsafe extern "C" fn dyad_getState(mut stream: *mut dyad_Stream)
 -> libc::c_int {
    return (*stream).state;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_getAddress(mut stream: *mut dyad_Stream)
 -> *const libc::c_char {
    return if !(*stream).address.is_null() {
               (*stream).address as *const libc::c_char
           } else { b"\x00" as *const u8 as *const libc::c_char };
}
#[no_mangle]
pub unsafe extern "C" fn dyad_getPort(mut stream: *mut dyad_Stream)
 -> libc::c_int {
    return (*stream).port;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_getBytesSent(mut stream: *mut dyad_Stream)
 -> libc::c_int {
    return (*stream).bytesSent;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_getBytesReceived(mut stream: *mut dyad_Stream)
 -> libc::c_int {
    return (*stream).bytesReceived;
}
#[no_mangle]
pub unsafe extern "C" fn dyad_getSocket(mut stream: *mut dyad_Stream)
 -> dyad_Socket {
    return (*stream).sockfd;
}
