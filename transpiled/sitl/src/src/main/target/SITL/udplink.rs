use core;
use libc;
use c2rust_asm_casts;
use c2rust_asm_casts::AsmCastTrait;
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
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn fcntl(__fd: libc::c_int, __cmd: libc::c_int, _: ...) -> libc::c_int;
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
    fn sendto(__fd: libc::c_int, __buf: *const libc::c_void, __n: size_t,
              __flags: libc::c_int, __addr: __CONST_SOCKADDR_ARG,
              __addr_len: socklen_t) -> ssize_t;
    #[no_mangle]
    fn recvfrom(__fd: libc::c_int, __buf: *mut libc::c_void, __n: size_t,
                __flags: libc::c_int, __addr: __SOCKADDR_ARG,
                __addr_len: *mut socklen_t) -> ssize_t;
    #[no_mangle]
    fn setsockopt(__fd: libc::c_int, __level: libc::c_int,
                  __optname: libc::c_int, __optval: *const libc::c_void,
                  __optlen: socklen_t) -> libc::c_int;
    #[no_mangle]
    fn inet_addr(__cp: *const libc::c_char) -> in_addr_t;
}
pub type size_t = libc::c_ulong;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type __time_t = libc::c_long;
pub type __suseconds_t = libc::c_long;
pub type __ssize_t = libc::c_long;
pub type __socklen_t = libc::c_uint;
pub type ssize_t = __ssize_t;
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
pub type socklen_t = __socklen_t;
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
pub type uint32_t = __uint32_t;
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
pub type uint16_t = __uint16_t;
pub type uint8_t = __uint8_t;
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
pub struct udpLink_t {
    pub fd: libc::c_int,
    pub si: sockaddr_in,
    pub recv: sockaddr_in,
    pub port: libc::c_int,
    pub addr: *mut libc::c_char,
    pub isServer: bool,
}
#[inline]
unsafe extern "C" fn __bswap_16(mut __bsx: __uint16_t) -> __uint16_t {
    return (__bsx as libc::c_int >> 8i32 & 0xffi32 |
                (__bsx as libc::c_int & 0xffi32) << 8i32) as __uint16_t;
}
#[inline]
unsafe extern "C" fn __bswap_32(mut __bsx: __uint32_t) -> __uint32_t {
    return (__bsx & 0xff000000u32) >> 24i32 | (__bsx & 0xff0000u32) >> 8i32 |
               (__bsx & 0xff00u32) << 8i32 | (__bsx & 0xffu32) << 24i32;
}
/* *
 * Copyright (c) 2017 cs8425
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the MIT license.
 */
#[no_mangle]
pub unsafe extern "C" fn udpInit(mut link: *mut udpLink_t,
                                 mut addr: *const libc::c_char,
                                 mut port: libc::c_int, mut isServer: bool)
 -> libc::c_int {
    let mut one: libc::c_int = 1i32; // can multi-bind
    (*link).fd =
        socket(2i32, SOCK_DGRAM as libc::c_int,
               IPPROTO_UDP as libc::c_int); // nonblock
    if (*link).fd == -1i32 { return -2i32 }
    setsockopt((*link).fd, 1i32, 2i32,
               &mut one as *mut libc::c_int as *const libc::c_void,
               ::core::mem::size_of::<libc::c_int>() as libc::c_ulong as
                   socklen_t);
    fcntl((*link).fd, 4i32, fcntl((*link).fd, 3i32, 0i32) | 0o4000i32);
    (*link).isServer = isServer;
    memset(&mut (*link).si as *mut sockaddr_in as *mut libc::c_void, 0i32,
           ::core::mem::size_of::<sockaddr_in>() as libc::c_ulong);
    (*link).si.sin_family = 2i32 as sa_family_t;
    (*link).si.sin_port = __bswap_16(port as __uint16_t);
    (*link).port = port;
    if addr.is_null() {
        (*link).si.sin_addr.s_addr = __bswap_32(0i32 as in_addr_t)
    } else { (*link).si.sin_addr.s_addr = inet_addr(addr) }
    if isServer {
        if bind((*link).fd,
                __CONST_SOCKADDR_ARG{__sockaddr__:
                                         &mut (*link).si as *mut sockaddr_in
                                             as *const sockaddr,},
                ::core::mem::size_of::<sockaddr_in>() as libc::c_ulong as
                    socklen_t) == -1i32 {
            return -1i32
        }
    }
    return 0i32;
}
#[no_mangle]
pub unsafe extern "C" fn udpSend(mut link: *mut udpLink_t,
                                 mut data: *const libc::c_void,
                                 mut size: size_t) -> libc::c_int {
    return sendto((*link).fd, data, size, 0i32,
                  __CONST_SOCKADDR_ARG{__sockaddr__:
                                           &mut (*link).si as *mut sockaddr_in
                                               as *mut sockaddr,},
                  ::core::mem::size_of::<sockaddr_in>() as libc::c_ulong as
                      socklen_t) as libc::c_int;
}
/* *
 * Copyright (c) 2017 cs8425
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the MIT license.
 */
#[no_mangle]
pub unsafe extern "C" fn udpRecv(mut link: *mut udpLink_t,
                                 mut data: *mut libc::c_void,
                                 mut size: size_t, mut timeout_ms: uint32_t)
 -> libc::c_int {
    let mut fds: fd_set = fd_set{fds_bits: [0; 16],};
    let mut tv: timeval = timeval{tv_sec: 0, tv_usec: 0,};
    let mut __d0: libc::c_int = 0;
    let mut __d1: libc::c_int = 0;
    let fresh0 = &mut __d0;
    let fresh1;
    let fresh2 = &mut __d1;
    let fresh3;
    let fresh4 =
        (::core::mem::size_of::<fd_set>() as
             libc::c_ulong).wrapping_div(::core::mem::size_of::<__fd_mask>()
                                             as libc::c_ulong);
    let fresh5 = &mut *fds.fds_bits.as_mut_ptr().offset(0) as *mut __fd_mask;
    asm!("cld; rep; stosq" : "={cx}" (fresh1), "={di}" (fresh3) : "{ax}"
         (0i32), "0" (c2rust_asm_casts::AsmCast::cast_in(fresh0, fresh4)), "1"
         (c2rust_asm_casts::AsmCast::cast_in(fresh2, fresh5)) : "memory" :
         "volatile");
    c2rust_asm_casts::AsmCast::cast_out(fresh0, fresh4, fresh1);
    c2rust_asm_casts::AsmCast::cast_out(fresh2, fresh5, fresh3);
    fds.fds_bits[((*link).fd /
                      (8i32 *
                           ::core::mem::size_of::<__fd_mask>() as
                               libc::c_ulong as libc::c_int)) as usize] |=
        (1u64 <<
             (*link).fd %
                 (8i32 *
                      ::core::mem::size_of::<__fd_mask>() as libc::c_ulong as
                          libc::c_int)) as __fd_mask;
    tv.tv_sec = timeout_ms.wrapping_div(1000i32 as libc::c_uint) as __time_t;
    tv.tv_usec =
        (timeout_ms.wrapping_rem(1000i32 as libc::c_uint) as
             libc::c_ulong).wrapping_mul(1000u64) as __suseconds_t;
    if select((*link).fd + 1i32, &mut fds, 0 as *mut fd_set, 0 as *mut fd_set,
              &mut tv) != 1i32 {
        return -1i32
    }
    let mut len: socklen_t = 0;
    let mut ret: libc::c_int = 0;
    ret =
        recvfrom((*link).fd, data, size, 0i32,
                 __SOCKADDR_ARG{__sockaddr__:
                                    &mut (*link).recv as *mut sockaddr_in as
                                        *mut sockaddr,}, &mut len) as
            libc::c_int;
    return ret;
}
