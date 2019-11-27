use core;
use libc;
use c2rust_bitfields;
use c2rust_bitfields::BitfieldStruct;
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type ioTag_t = uint8_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct rxConfig_s {
    pub rcmap: [uint8_t; 8],
    pub serialrx_provider: uint8_t,
    pub serialrx_inverted: uint8_t,
    pub halfDuplex: uint8_t,
    pub spektrum_bind_pin_override_ioTag: ioTag_t,
    pub spektrum_bind_plug_ioTag: ioTag_t,
    pub spektrum_sat_bind: uint8_t,
    pub spektrum_sat_bind_autoreset: uint8_t,
    pub rssi_channel: uint8_t,
    pub rssi_scale: uint8_t,
    pub rssi_invert: uint8_t,
    pub midrc: uint16_t,
    pub mincheck: uint16_t,
    pub maxcheck: uint16_t,
    pub rcInterpolation: uint8_t,
    pub rcInterpolationChannels: uint8_t,
    pub rcInterpolationInterval: uint8_t,
    pub fpvCamAngleDegrees: uint8_t,
    pub airModeActivateThreshold: uint8_t,
    pub rx_min_usec: uint16_t,
    pub rx_max_usec: uint16_t,
    pub max_aux_channel: uint8_t,
    pub rssi_src_frame_errors: uint8_t,
    pub rssi_offset: int8_t,
    pub rc_smoothing_type: uint8_t,
    pub rc_smoothing_input_cutoff: uint8_t,
    pub rc_smoothing_derivative_cutoff: uint8_t,
    pub rc_smoothing_debug_axis: uint8_t,
    pub rc_smoothing_input_type: uint8_t,
    pub rc_smoothing_derivative_type: uint8_t,
}
pub type rxConfig_t = rxConfig_s;
pub type C2RustUnnamed = libc::c_uint;
pub const RX_FRAME_DROPPED: C2RustUnnamed = 8;
pub const RX_FRAME_PROCESSING_REQUIRED: C2RustUnnamed = 4;
pub const RX_FRAME_FAILSAFE: C2RustUnnamed = 2;
pub const RX_FRAME_COMPLETE: C2RustUnnamed = 1;
pub const RX_FRAME_PENDING: C2RustUnnamed = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct rxRuntimeConfig_s {
    pub channelCount: uint8_t,
    pub rxRefreshRate: uint16_t,
    pub rcReadRawFn: rcReadRawDataFnPtr,
    pub rcFrameStatusFn: rcFrameStatusFnPtr,
    pub rcProcessFrameFn: rcProcessFrameFnPtr,
    pub channelData: *mut uint16_t,
    pub frameData: *mut libc::c_void,
}
pub type rcProcessFrameFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s) -> bool>;
pub type rcFrameStatusFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut rxRuntimeConfig_s) -> uint8_t>;
pub type rcReadRawDataFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s, _: uint8_t)
               -> uint16_t>;
pub type rxRuntimeConfig_t = rxRuntimeConfig_s;
#[derive ( Copy, Clone, BitfieldStruct )]
#[repr(C, packed)]
pub struct sbusChannels_s {
    #[bitfield(name = "chan0", ty = "libc::c_uint", bits = "0..=10")]
    #[bitfield(name = "chan1", ty = "libc::c_uint", bits = "11..=21")]
    #[bitfield(name = "chan2", ty = "libc::c_uint", bits = "22..=32")]
    #[bitfield(name = "chan3", ty = "libc::c_uint", bits = "33..=43")]
    #[bitfield(name = "chan4", ty = "libc::c_uint", bits = "44..=54")]
    #[bitfield(name = "chan5", ty = "libc::c_uint", bits = "55..=65")]
    #[bitfield(name = "chan6", ty = "libc::c_uint", bits = "66..=76")]
    #[bitfield(name = "chan7", ty = "libc::c_uint", bits = "77..=87")]
    #[bitfield(name = "chan8", ty = "libc::c_uint", bits = "88..=98")]
    #[bitfield(name = "chan9", ty = "libc::c_uint", bits = "99..=109")]
    #[bitfield(name = "chan10", ty = "libc::c_uint", bits = "110..=120")]
    #[bitfield(name = "chan11", ty = "libc::c_uint", bits = "121..=131")]
    #[bitfield(name = "chan12", ty = "libc::c_uint", bits = "132..=142")]
    #[bitfield(name = "chan13", ty = "libc::c_uint", bits = "143..=153")]
    #[bitfield(name = "chan14", ty = "libc::c_uint", bits = "154..=164")]
    #[bitfield(name = "chan15", ty = "libc::c_uint", bits = "165..=175")]
    pub chan0_chan1_chan2_chan3_chan4_chan5_chan6_chan7_chan8_chan9_chan10_chan11_chan12_chan13_chan14_chan15: [u8; 22],
    pub flags: uint8_t,
}
pub type sbusChannels_t = sbusChannels_s;
#[no_mangle]
pub unsafe extern "C" fn sbusChannelsDecode(mut rxRuntimeConfig:
                                                *mut rxRuntimeConfig_t,
                                            mut channels:
                                                *const sbusChannels_t)
 -> uint8_t {
    let mut sbusChannelData: *mut uint16_t = (*rxRuntimeConfig).channelData;
    *sbusChannelData.offset(0) = (*channels).chan0() as uint16_t;
    *sbusChannelData.offset(1) = (*channels).chan1() as uint16_t;
    *sbusChannelData.offset(2) = (*channels).chan2() as uint16_t;
    *sbusChannelData.offset(3) = (*channels).chan3() as uint16_t;
    *sbusChannelData.offset(4) = (*channels).chan4() as uint16_t;
    *sbusChannelData.offset(5) = (*channels).chan5() as uint16_t;
    *sbusChannelData.offset(6) = (*channels).chan6() as uint16_t;
    *sbusChannelData.offset(7) = (*channels).chan7() as uint16_t;
    *sbusChannelData.offset(8) = (*channels).chan8() as uint16_t;
    *sbusChannelData.offset(9) = (*channels).chan9() as uint16_t;
    *sbusChannelData.offset(10) = (*channels).chan10() as uint16_t;
    *sbusChannelData.offset(11) = (*channels).chan11() as uint16_t;
    *sbusChannelData.offset(12) = (*channels).chan12() as uint16_t;
    *sbusChannelData.offset(13) = (*channels).chan13() as uint16_t;
    *sbusChannelData.offset(14) = (*channels).chan14() as uint16_t;
    *sbusChannelData.offset(15) = (*channels).chan15() as uint16_t;
    if (*channels).flags as libc::c_int & 1i32 << 0i32 != 0 {
        *sbusChannelData.offset(16) = 1812i32 as uint16_t
    } else { *sbusChannelData.offset(16) = 173i32 as uint16_t }
    if (*channels).flags as libc::c_int & 1i32 << 1i32 != 0 {
        *sbusChannelData.offset(17) = 1812i32 as uint16_t
    } else { *sbusChannelData.offset(17) = 173i32 as uint16_t }
    if (*channels).flags as libc::c_int & 1i32 << 3i32 != 0 {
        // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
        // internal failsafe enabled and rx failsafe flag set
        // RX *should* still be sending valid channel data (repeated), so use it.
        return (RX_FRAME_COMPLETE as libc::c_int |
                    RX_FRAME_FAILSAFE as libc::c_int) as uint8_t
    }
    if (*channels).flags as libc::c_int & 1i32 << 2i32 != 0 {
        // The received data is a repeat of the last valid data so can be considered complete.
        return (RX_FRAME_COMPLETE as libc::c_int |
                    RX_FRAME_DROPPED as libc::c_int) as uint8_t
    }
    return RX_FRAME_COMPLETE as libc::c_int as uint8_t;
}
unsafe extern "C" fn sbusChannelsReadRawRC(mut rxRuntimeConfig:
                                               *const rxRuntimeConfig_t,
                                           mut chan: uint8_t) -> uint16_t {
    // Linear fitting values read from OpenTX-ppmus and comparing with values received by X4R
    // http://www.wolframalpha.com/input/?i=linear+fit+%7B173%2C+988%7D%2C+%7B1812%2C+2012%7D%2C+%7B993%2C+1500%7D
    return (5i32 *
                *(*rxRuntimeConfig).channelData.offset(chan as isize) as
                    libc::c_int / 8i32 + 880i32) as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn sbusChannelsInit(mut rxConfig: *const rxConfig_t,
                                          mut rxRuntimeConfig:
                                              *mut rxRuntimeConfig_t) {
    (*rxRuntimeConfig).rcReadRawFn =
        Some(sbusChannelsReadRawRC as
                 unsafe extern "C" fn(_: *const rxRuntimeConfig_t, _: uint8_t)
                     -> uint16_t);
    let mut b: libc::c_int = 0i32;
    while b < 18i32 {
        *(*rxRuntimeConfig).channelData.offset(b as isize) =
            (16i32 * (*rxConfig).midrc as libc::c_int / 10i32 - 1408i32) as
                uint16_t;
        b += 1
    };
}
