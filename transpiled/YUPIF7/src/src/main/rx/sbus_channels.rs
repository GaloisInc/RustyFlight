use ::libc;
use ::c2rust_bitfields;
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
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
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
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
// used by receiver driver to return channel data
pub type rcFrameStatusFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut rxRuntimeConfig_s) -> uint8_t>;
pub type rcReadRawDataFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s, _: uint8_t)
               -> uint16_t>;
pub type rxRuntimeConfig_t = rxRuntimeConfig_s;
#[derive(Copy, Clone, BitfieldStruct)]
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
    *sbusChannelData.offset(0 as libc::c_int as isize) =
        (*channels).chan0() as uint16_t;
    *sbusChannelData.offset(1 as libc::c_int as isize) =
        (*channels).chan1() as uint16_t;
    *sbusChannelData.offset(2 as libc::c_int as isize) =
        (*channels).chan2() as uint16_t;
    *sbusChannelData.offset(3 as libc::c_int as isize) =
        (*channels).chan3() as uint16_t;
    *sbusChannelData.offset(4 as libc::c_int as isize) =
        (*channels).chan4() as uint16_t;
    *sbusChannelData.offset(5 as libc::c_int as isize) =
        (*channels).chan5() as uint16_t;
    *sbusChannelData.offset(6 as libc::c_int as isize) =
        (*channels).chan6() as uint16_t;
    *sbusChannelData.offset(7 as libc::c_int as isize) =
        (*channels).chan7() as uint16_t;
    *sbusChannelData.offset(8 as libc::c_int as isize) =
        (*channels).chan8() as uint16_t;
    *sbusChannelData.offset(9 as libc::c_int as isize) =
        (*channels).chan9() as uint16_t;
    *sbusChannelData.offset(10 as libc::c_int as isize) =
        (*channels).chan10() as uint16_t;
    *sbusChannelData.offset(11 as libc::c_int as isize) =
        (*channels).chan11() as uint16_t;
    *sbusChannelData.offset(12 as libc::c_int as isize) =
        (*channels).chan12() as uint16_t;
    *sbusChannelData.offset(13 as libc::c_int as isize) =
        (*channels).chan13() as uint16_t;
    *sbusChannelData.offset(14 as libc::c_int as isize) =
        (*channels).chan14() as uint16_t;
    *sbusChannelData.offset(15 as libc::c_int as isize) =
        (*channels).chan15() as uint16_t;
    if (*channels).flags as libc::c_int &
           (1 as libc::c_int) << 0 as libc::c_int != 0 {
        *sbusChannelData.offset(16 as libc::c_int as isize) =
            1812 as libc::c_int as uint16_t
    } else {
        *sbusChannelData.offset(16 as libc::c_int as isize) =
            173 as libc::c_int as uint16_t
    }
    if (*channels).flags as libc::c_int &
           (1 as libc::c_int) << 1 as libc::c_int != 0 {
        *sbusChannelData.offset(17 as libc::c_int as isize) =
            1812 as libc::c_int as uint16_t
    } else {
        *sbusChannelData.offset(17 as libc::c_int as isize) =
            173 as libc::c_int as uint16_t
    }
    if (*channels).flags as libc::c_int &
           (1 as libc::c_int) << 3 as libc::c_int != 0 {
        // number of RC channels as reported by current input driver
        // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
        // internal failsafe enabled and rx failsafe flag set
        // RX *should* still be sending valid channel data (repeated), so use it.
        return (RX_FRAME_COMPLETE as libc::c_int |
                    RX_FRAME_FAILSAFE as libc::c_int) as uint8_t
    }
    if (*channels).flags as libc::c_int &
           (1 as libc::c_int) << 2 as libc::c_int != 0 {
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
    return (5 as libc::c_int *
                *(*rxRuntimeConfig).channelData.offset(chan as isize) as
                    libc::c_int / 8 as libc::c_int + 880 as libc::c_int) as
               uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn sbusChannelsInit(mut rxConfig: *const rxConfig_t,
                                          mut rxRuntimeConfig:
                                              *mut rxRuntimeConfig_t) {
    (*rxRuntimeConfig).rcReadRawFn =
        Some(sbusChannelsReadRawRC as
                 unsafe extern "C" fn(_: *const rxRuntimeConfig_t, _: uint8_t)
                     -> uint16_t);
    let mut b: libc::c_int = 0 as libc::c_int;
    while b < 18 as libc::c_int {
        *(*rxRuntimeConfig).channelData.offset(b as isize) =
            (16 as libc::c_int * (*rxConfig).midrc as libc::c_int /
                 10 as libc::c_int - 1408 as libc::c_int) as uint16_t;
        b += 1
    };
}
