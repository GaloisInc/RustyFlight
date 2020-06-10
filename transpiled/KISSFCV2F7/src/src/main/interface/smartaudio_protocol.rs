use ::libc;
extern "C" {
    #[no_mangle]
    fn crc8_dvb_s2_update(crc: uint8_t, data: *const libc::c_void,
                          length: uint32_t) -> uint8_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type size_t = libc::c_ulong;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct smartaudioSettings_s {
    pub version: uint8_t,
    pub unlocked: uint8_t,
    pub channel: uint8_t,
    pub power: uint8_t,
    pub frequency: uint16_t,
    pub pitmodeFrequency: uint16_t,
    pub userFrequencyMode: bool,
    pub pitmodeEnabled: bool,
    pub pitmodeInRangeActive: bool,
    pub pitmodeOutRangeActive: bool,
}
pub type smartaudioSettings_t = smartaudioSettings_s;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct smartaudioFrameHeader_s {
    pub startCode: uint16_t,
    pub length: uint8_t,
    pub command: uint8_t,
}
pub type smartaudioFrameHeader_t = smartaudioFrameHeader_s;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct smartaudioCommandOnlyFrame_s {
    pub header: smartaudioFrameHeader_t,
    pub crc: uint8_t,
}
pub type smartaudioCommandOnlyFrame_t = smartaudioCommandOnlyFrame_s;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct smartaudioU8Frame_s {
    pub header: smartaudioFrameHeader_t,
    pub payload: uint8_t,
    pub crc: uint8_t,
}
pub type smartaudioU8Frame_t = smartaudioU8Frame_s;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct smartaudioU16Frame_s {
    pub header: smartaudioFrameHeader_t,
    pub payload: uint16_t,
    pub crc: uint8_t,
}
pub type smartaudioU16Frame_t = smartaudioU16Frame_s;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct smartaudioU8ResponseFrame_s {
    pub header: smartaudioFrameHeader_t,
    pub payload: uint8_t,
    pub reserved: uint8_t,
    pub crc: uint8_t,
}
pub type smartaudioU8ResponseFrame_t = smartaudioU8ResponseFrame_s;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct smartaudioU16ResponseFrame_s {
    pub header: smartaudioFrameHeader_t,
    pub payload: uint16_t,
    pub reserved: uint8_t,
    pub crc: uint8_t,
}
pub type smartaudioU16ResponseFrame_t = smartaudioU16ResponseFrame_s;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct smartaudioSettingsResponseFrame_s {
    pub header: smartaudioFrameHeader_t,
    pub channel: uint8_t,
    pub power: uint8_t,
    pub operationMode: uint8_t,
    pub frequency: uint16_t,
    pub crc: uint8_t,
}
pub type smartaudioSettingsResponseFrame_t
    =
    smartaudioSettingsResponseFrame_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub union smartaudioFrame_u {
    pub commandOnlyFrame: smartaudioCommandOnlyFrame_t,
    pub u8RequestFrame: smartaudioU8Frame_t,
    pub u16RequestFrame: smartaudioU16Frame_t,
}
pub type smartaudioFrame_t = smartaudioFrame_u;
unsafe extern "C" fn smartaudioFrameInit(command: uint8_t,
                                         mut header:
                                             *mut smartaudioFrameHeader_t,
                                         payloadLength: uint8_t) {
    (*header).startCode =
        (0xaa as libc::c_int + 0x55 as libc::c_int) as uint16_t;
    (*header).length = payloadLength;
    (*header).command = command;
}
unsafe extern "C" fn smartaudioUnpackOperationMode(mut settings:
                                                       *mut smartaudioSettings_t,
                                                   operationMode: uint8_t,
                                                   settingsResponse: bool) {
    if settingsResponse {
        // operation mode bit order is different between 'Get Settings' and 'Set Mode' responses.
        (*settings).userFrequencyMode =
            operationMode as libc::c_int & 0x1 as libc::c_int != 0;
        (*settings).pitmodeEnabled =
            operationMode as libc::c_int & 0x2 as libc::c_int != 0;
        (*settings).pitmodeInRangeActive =
            operationMode as libc::c_int & 0x4 as libc::c_int != 0;
        (*settings).pitmodeOutRangeActive =
            operationMode as libc::c_int & 0x8 as libc::c_int != 0;
        (*settings).unlocked =
            (operationMode as libc::c_int & 0x10 as libc::c_int) as uint8_t
    } else {
        (*settings).pitmodeInRangeActive =
            operationMode as libc::c_int & 0x1 as libc::c_int != 0;
        (*settings).pitmodeOutRangeActive =
            operationMode as libc::c_int & 0x2 as libc::c_int != 0;
        (*settings).pitmodeEnabled =
            operationMode as libc::c_int & 0x4 as libc::c_int != 0;
        (*settings).unlocked =
            (operationMode as libc::c_int & 0x8 as libc::c_int) as uint8_t
    };
}
unsafe extern "C" fn smartaudioUnpackFrequency(mut settings:
                                                   *mut smartaudioSettings_t,
                                               frequency: uint16_t) {
    if frequency as libc::c_int & (1 as libc::c_int) << 14 as libc::c_int != 0
       {
        (*settings).pitmodeFrequency =
            (frequency as libc::c_int &
                 (0x3fff as libc::c_int) << 8 as libc::c_int |
                 frequency as libc::c_int &
                     0x3fff as libc::c_int >> 8 as libc::c_int &
                     0xff as libc::c_int) as uint16_t
    } else {
        (*settings).frequency =
            (frequency as libc::c_int &
                 (0x3fff as libc::c_int) << 8 as libc::c_int |
                 frequency as libc::c_int &
                     0x3fff as libc::c_int >> 8 as libc::c_int &
                     0xff as libc::c_int) as uint16_t
    };
}
unsafe extern "C" fn smartaudioUnpackSettings(mut settings:
                                                  *mut smartaudioSettings_t,
                                              mut frame:
                                                  *const smartaudioSettingsResponseFrame_t) {
    (*settings).channel = (*frame).channel;
    (*settings).power = (*frame).power;
    smartaudioUnpackFrequency(settings, (*frame).frequency);
    smartaudioUnpackOperationMode(settings, (*frame).operationMode,
                                  1 as libc::c_int != 0);
}
unsafe extern "C" fn smartaudioPackOperationMode(mut settings:
                                                     *const smartaudioSettings_t)
 -> uint8_t {
    let mut operationMode: uint8_t = 0 as libc::c_int as uint8_t;
    operationMode =
        (operationMode as libc::c_int |
             ((*settings).pitmodeInRangeActive as libc::c_int) <<
                 0 as libc::c_int) as uint8_t;
    operationMode =
        (operationMode as libc::c_int |
             ((*settings).pitmodeOutRangeActive as libc::c_int) <<
                 1 as libc::c_int) as uint8_t;
    operationMode =
        (operationMode as libc::c_int |
             ((*settings).pitmodeEnabled as libc::c_int) << 2 as libc::c_int)
            as uint8_t;
    operationMode =
        (operationMode as libc::c_int |
             ((*settings).unlocked as libc::c_int) << 3 as libc::c_int) as
            uint8_t;
    return operationMode;
}
#[no_mangle]
pub unsafe extern "C" fn smartaudioFrameGetSettings(mut smartaudioFrame:
                                                        *mut smartaudioFrame_t)
 -> size_t {
    let mut frame: *mut smartaudioCommandOnlyFrame_t =
        smartaudioFrame as *mut smartaudioCommandOnlyFrame_t;
    smartaudioFrameInit(0x3 as libc::c_int as uint8_t, &mut (*frame).header,
                        0 as libc::c_int as uint8_t);
    (*frame).crc =
        crc8_dvb_s2_update(0 as libc::c_int as uint8_t,
                           frame as *const libc::c_void,
                           (::core::mem::size_of::<smartaudioCommandOnlyFrame_t>()
                                as
                                libc::c_ulong).wrapping_sub(::core::mem::size_of::<uint8_t>()
                                                                as
                                                                libc::c_ulong)
                               as uint32_t);
    return ::core::mem::size_of::<smartaudioCommandOnlyFrame_t>() as
               libc::c_ulong;
}
#[no_mangle]
pub unsafe extern "C" fn smartaudioFrameGetPitmodeFrequency(mut smartaudioFrame:
                                                                *mut smartaudioFrame_t)
 -> size_t {
    let mut frame: *mut smartaudioU16Frame_t =
        smartaudioFrame as *mut smartaudioU16Frame_t;
    smartaudioFrameInit(0x9 as libc::c_int as uint8_t, &mut (*frame).header,
                        ::core::mem::size_of::<uint16_t>() as libc::c_ulong as
                            uint8_t);
    (*frame).payload = ((1 as libc::c_int) << 15 as libc::c_int) as uint16_t;
    (*frame).crc =
        crc8_dvb_s2_update(0 as libc::c_int as uint8_t,
                           frame as *const libc::c_void,
                           (::core::mem::size_of::<smartaudioU16Frame_t>() as
                                libc::c_ulong).wrapping_sub(::core::mem::size_of::<uint8_t>()
                                                                as
                                                                libc::c_ulong)
                               as uint32_t);
    return ::core::mem::size_of::<smartaudioU16Frame_t>() as libc::c_ulong;
}
#[no_mangle]
pub unsafe extern "C" fn smartaudioFrameSetPower(mut smartaudioFrame:
                                                     *mut smartaudioFrame_t,
                                                 power: uint8_t) -> size_t {
    let mut frame: *mut smartaudioU8Frame_t =
        smartaudioFrame as *mut smartaudioU8Frame_t;
    smartaudioFrameInit(0x5 as libc::c_int as uint8_t, &mut (*frame).header,
                        ::core::mem::size_of::<uint8_t>() as libc::c_ulong as
                            uint8_t);
    (*frame).payload = power;
    (*frame).crc =
        crc8_dvb_s2_update(0 as libc::c_int as uint8_t,
                           frame as *const libc::c_void,
                           (::core::mem::size_of::<smartaudioU8Frame_t>() as
                                libc::c_ulong).wrapping_sub(::core::mem::size_of::<uint8_t>()
                                                                as
                                                                libc::c_ulong)
                               as uint32_t);
    return ::core::mem::size_of::<smartaudioU8Frame_t>() as libc::c_ulong;
}
#[no_mangle]
pub unsafe extern "C" fn smartaudioFrameSetBandChannel(mut smartaudioFrame:
                                                           *mut smartaudioFrame_t,
                                                       band: uint8_t,
                                                       channel: uint8_t)
 -> size_t {
    let mut frame: *mut smartaudioU8Frame_t =
        smartaudioFrame as *mut smartaudioU8Frame_t;
    smartaudioFrameInit(0x7 as libc::c_int as uint8_t, &mut (*frame).header,
                        ::core::mem::size_of::<uint8_t>() as libc::c_ulong as
                            uint8_t);
    (*frame).payload =
        (band as libc::c_int * 8 as libc::c_int + channel as libc::c_int) as
            uint8_t;
    (*frame).crc =
        crc8_dvb_s2_update(0 as libc::c_int as uint8_t,
                           frame as *const libc::c_void,
                           (::core::mem::size_of::<smartaudioU8Frame_t>() as
                                libc::c_ulong).wrapping_sub(::core::mem::size_of::<uint8_t>()
                                                                as
                                                                libc::c_ulong)
                               as uint32_t);
    return ::core::mem::size_of::<smartaudioU8Frame_t>() as libc::c_ulong;
}
#[no_mangle]
pub unsafe extern "C" fn smartaudioFrameSetFrequency(mut smartaudioFrame:
                                                         *mut smartaudioFrame_t,
                                                     frequency: uint16_t,
                                                     pitmodeFrequency: bool)
 -> size_t {
    let mut frame: *mut smartaudioU16Frame_t =
        smartaudioFrame as *mut smartaudioU16Frame_t;
    smartaudioFrameInit(0x9 as libc::c_int as uint8_t, &mut (*frame).header,
                        ::core::mem::size_of::<uint16_t>() as libc::c_ulong as
                            uint8_t);
    (*frame).payload =
        (frequency as libc::c_int |
             (if pitmodeFrequency as libc::c_int != 0 {
                  ((1 as libc::c_int)) << 15 as libc::c_int
              } else { 0 as libc::c_int }) << 8 as libc::c_int |
             (frequency as libc::c_int |
                  (if pitmodeFrequency as libc::c_int != 0 {
                       ((1 as libc::c_int)) << 15 as libc::c_int
                   } else { 0 as libc::c_int }) >> 8 as libc::c_int) &
                 0xff as libc::c_int) as uint16_t;
    (*frame).crc =
        crc8_dvb_s2_update(0 as libc::c_int as uint8_t,
                           frame as *const libc::c_void,
                           (::core::mem::size_of::<smartaudioU16Frame_t>() as
                                libc::c_ulong).wrapping_sub(::core::mem::size_of::<uint8_t>()
                                                                as
                                                                libc::c_ulong)
                               as uint32_t);
    return ::core::mem::size_of::<smartaudioU16Frame_t>() as libc::c_ulong;
}
#[no_mangle]
pub unsafe extern "C" fn smartaudioFrameSetOperationMode(mut smartaudioFrame:
                                                             *mut smartaudioFrame_t,
                                                         mut settings:
                                                             *const smartaudioSettings_t)
 -> size_t {
    let mut frame: *mut smartaudioU8Frame_t =
        smartaudioFrame as *mut smartaudioU8Frame_t;
    smartaudioFrameInit(0xb as libc::c_int as uint8_t, &mut (*frame).header,
                        ::core::mem::size_of::<uint8_t>() as libc::c_ulong as
                            uint8_t);
    (*frame).payload = smartaudioPackOperationMode(settings);
    (*frame).crc =
        crc8_dvb_s2_update(0 as libc::c_int as uint8_t,
                           frame as *const libc::c_void,
                           (::core::mem::size_of::<smartaudioU8Frame_t>() as
                                libc::c_ulong).wrapping_sub(::core::mem::size_of::<uint8_t>()
                                                                as
                                                                libc::c_ulong)
                               as uint32_t);
    return ::core::mem::size_of::<smartaudioU8Frame_t>() as libc::c_ulong;
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
#[no_mangle]
pub unsafe extern "C" fn smartaudioParseResponseBuffer(mut settings:
                                                           *mut smartaudioSettings_t,
                                                       mut buffer:
                                                           *const uint8_t)
 -> bool {
    let mut header: *const smartaudioFrameHeader_t =
        buffer as
            *const smartaudioFrameHeader_t; // subtract crc byte from length
    let fullFrameLength: uint8_t =
        (::core::mem::size_of::<smartaudioFrameHeader_t>() as
             libc::c_ulong).wrapping_add((*header).length as libc::c_ulong) as
            uint8_t;
    let headerPayloadLength: uint8_t =
        (fullFrameLength as libc::c_int - 1 as libc::c_int) as uint8_t;
    let mut endPtr: *const uint8_t =
        buffer.offset(fullFrameLength as libc::c_int as isize);
    if crc8_dvb_s2_update(*endPtr, buffer as *const libc::c_void,
                          headerPayloadLength as uint32_t) as libc::c_int != 0
           ||
           (*header).startCode as libc::c_int !=
               0xaa as libc::c_int + 0x55 as libc::c_int {
        return 0 as libc::c_int != 0
    }
    match (*header).command as libc::c_int {
        1 => {
            let mut resp: *const smartaudioSettingsResponseFrame_t =
                buffer as *const smartaudioSettingsResponseFrame_t;
            (*settings).version = 1 as libc::c_int as uint8_t;
            smartaudioUnpackSettings(settings, resp);
        }
        9 => {
            let mut resp_0: *const smartaudioSettingsResponseFrame_t =
                buffer as *const smartaudioSettingsResponseFrame_t;
            (*settings).version = 2 as libc::c_int as uint8_t;
            smartaudioUnpackSettings(settings, resp_0);
        }
        2 => {
            let mut resp_1: *const smartaudioU16ResponseFrame_t =
                buffer as *const smartaudioU16ResponseFrame_t;
            (*settings).channel =
                ((*resp_1).payload as libc::c_int >> 8 as libc::c_int &
                     0xff as libc::c_int) as uint8_t;
            (*settings).power =
                ((*resp_1).payload as libc::c_int & 0xff as libc::c_int) as
                    uint8_t
        }
        3 => {
            let mut resp_2: *const smartaudioU8ResponseFrame_t =
                buffer as *const smartaudioU8ResponseFrame_t;
            (*settings).channel = (*resp_2).payload
        }
        4 => {
            let mut resp_3: *const smartaudioU16ResponseFrame_t =
                buffer as *const smartaudioU16ResponseFrame_t;
            smartaudioUnpackFrequency(settings, (*resp_3).payload);
        }
        5 => {
            let mut resp_4: *const smartaudioU8ResponseFrame_t =
                buffer as *const smartaudioU8ResponseFrame_t;
            smartaudioUnpackOperationMode(settings, (*resp_4).payload,
                                          0 as libc::c_int != 0);
        }
        _ => { return 0 as libc::c_int != 0 }
    }
    return 1 as libc::c_int != 0;
}
