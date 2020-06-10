use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
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
    fn bitArrayGet(array: *const libc::c_void, bit: libc::c_uint) -> bool;
    #[no_mangle]
    fn bitArraySet(array: *mut libc::c_void, bit: libc::c_uint);
    #[no_mangle]
    fn bitArrayClr(array: *mut libc::c_void, bit: libc::c_uint);
    #[no_mangle]
    fn bitArrayXor(dest: *mut libc::c_void, size: size_t,
                   op1: *mut libc::c_void, op2: *mut libc::c_void);
    #[no_mangle]
    fn bitArrayCopy(array: *mut libc::c_void, from: libc::c_uint,
                    to: libc::c_uint);
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut rcData: [int16_t; 18];
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type size_t = libc::c_ulong;
pub type pgn_t = uint16_t;
pub type C2RustUnnamed = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed = 4095;
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
}
pub type pgRegistry_t = pgRegistry_s;
pub type boxId_e = libc::c_uint;
pub const CHECKBOX_ITEM_COUNT: boxId_e = 41;
pub const BOXACROTRAINER: boxId_e = 40;
pub const BOXPIDAUDIO: boxId_e = 39;
pub const BOXUSER4: boxId_e = 38;
pub const BOXUSER3: boxId_e = 37;
pub const BOXUSER2: boxId_e = 36;
pub const BOXUSER1: boxId_e = 35;
pub const BOXPARALYZE: boxId_e = 34;
pub const BOXVTXPITMODE: boxId_e = 33;
pub const BOXBEEPGPSCOUNT: boxId_e = 32;
pub const BOXPREARM: boxId_e = 31;
pub const BOXFLIPOVERAFTERCRASH: boxId_e = 30;
pub const BOXCAMERA3: boxId_e = 29;
pub const BOXCAMERA2: boxId_e = 28;
pub const BOXCAMERA1: boxId_e = 27;
pub const BOXBLACKBOXERASE: boxId_e = 26;
pub const BOXFPVANGLEMIX: boxId_e = 25;
pub const BOX3D: boxId_e = 24;
pub const BOXAIRMODE: boxId_e = 23;
pub const BOXBLACKBOX: boxId_e = 22;
pub const BOXSERVO3: boxId_e = 21;
pub const BOXSERVO2: boxId_e = 20;
pub const BOXSERVO1: boxId_e = 19;
pub const BOXTELEMETRY: boxId_e = 18;
pub const BOXOSD: boxId_e = 17;
pub const BOXCALIB: boxId_e = 16;
pub const BOXLEDLOW: boxId_e = 15;
pub const BOXBEEPERON: boxId_e = 14;
pub const BOXCAMSTAB: boxId_e = 13;
pub const BOXHEADADJ: boxId_e = 12;
pub const BOXANTIGRAVITY: boxId_e = 11;
pub const BOXID_FLIGHTMODE_LAST: boxId_e = 10;
pub const BOXGPSRESCUE: boxId_e = 10;
pub const BOXFAILSAFE: boxId_e = 9;
pub const BOXPASSTHRU: boxId_e = 8;
pub const BOXHEADFREE: boxId_e = 7;
pub const BOXGPSHOLD: boxId_e = 6;
pub const BOXGPSHOME: boxId_e = 5;
pub const BOXBARO: boxId_e = 4;
pub const BOXMAG: boxId_e = 3;
pub const BOXHORIZON: boxId_e = 2;
pub const BOXANGLE: boxId_e = 1;
pub const BOXARM: boxId_e = 0;
pub type modeLogic_e = libc::c_uint;
pub const MODELOGIC_AND: modeLogic_e = 1;
pub const MODELOGIC_OR: modeLogic_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct boxBitmask_s {
    pub bits: [uint32_t; 2],
}
// type to hold enough bits for CHECKBOX_ITEM_COUNT. Struct used for value-like behavior
pub type boxBitmask_t = boxBitmask_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct channelRange_s {
    pub startStep: uint8_t,
    pub endStep: uint8_t,
}
// steps are 25 apart
// a value of 0 corresponds to a channel value of 900 or less
// a value of 48 corresponds to a channel value of 2100 or more
// 48 steps between 900 and 2100
pub type channelRange_t = channelRange_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct modeActivationCondition_s {
    pub modeId: boxId_e,
    pub auxChannelIndex: uint8_t,
    pub range: channelRange_t,
    pub modeLogic: modeLogic_e,
    pub linkedTo: boxId_e,
}
pub type modeActivationCondition_t = modeActivationCondition_s;
pub const FEATURE_AIRMODE: C2RustUnnamed_1 = 4194304;
// microsecond time
pub type timeUs_t = uint32_t;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_1 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_1 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_1 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_1 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_1 = 33554432;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_1 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_1 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_1 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_1 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_1 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_1 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_1 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_1 = 8192;
pub const FEATURE_3D: C2RustUnnamed_1 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_1 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_1 = 512;
pub const FEATURE_GPS: C2RustUnnamed_1 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_1 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_1 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_1 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_1 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_1 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_1 = 1;
#[inline]
unsafe extern "C" fn modeActivationConditions(mut _index: libc::c_int)
 -> *const modeActivationCondition_t {
    return &mut *modeActivationConditions_SystemArray.as_mut_ptr().offset(_index
                                                                              as
                                                                              isize)
               as *mut modeActivationCondition_t;
}
#[inline]
unsafe extern "C" fn modeActivationConditionsMutable(mut _index: libc::c_int)
 -> *mut modeActivationCondition_t {
    return &mut *modeActivationConditions_SystemArray.as_mut_ptr().offset(_index
                                                                              as
                                                                              isize)
               as *mut modeActivationCondition_t;
}
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[no_mangle]
pub static mut rcModeActivationMask: boxBitmask_t =
    boxBitmask_t{bits: [0; 2],};
// one bit per mode defined in boxId_e
static mut stickyModesEverDisabled: boxBitmask_t =
    boxBitmask_t{bits: [0; 2],};
// Initialized in run_static_initializers
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut modeActivationConditions_Registry: pgRegistry_t =
    pgRegistry_t{pgn: 0,
                 size: 0,
                 address: 0 as *const uint8_t as *mut uint8_t,
                 copy: 0 as *const uint8_t as *mut uint8_t,
                 ptr: 0 as *const *mut uint8_t as *mut *mut uint8_t,
                 reset:
                     C2RustUnnamed_0{ptr:
                                         0 as *const libc::c_void as
                                             *mut libc::c_void,},};
#[no_mangle]
pub static mut modeActivationConditions_SystemArray:
           [modeActivationCondition_t; 20] =
    [modeActivationCondition_t{modeId: BOXARM,
                               auxChannelIndex: 0,
                               range:
                                   channelRange_t{startStep: 0, endStep: 0,},
                               modeLogic: MODELOGIC_OR,
                               linkedTo: BOXARM,}; 20];
#[no_mangle]
pub static mut modeActivationConditions_CopyArray:
           [modeActivationCondition_t; 20] =
    [modeActivationCondition_t{modeId: BOXARM,
                               auxChannelIndex: 0,
                               range:
                                   channelRange_t{startStep: 0, endStep: 0,},
                               modeLogic: MODELOGIC_OR,
                               linkedTo: BOXARM,}; 20];
#[no_mangle]
pub unsafe extern "C" fn IS_RC_MODE_ACTIVE(mut boxId: boxId_e) -> bool {
    return bitArrayGet(&mut rcModeActivationMask as *mut boxBitmask_t as
                           *const libc::c_void, boxId as libc::c_uint);
}
#[no_mangle]
pub unsafe extern "C" fn rcModeUpdate(mut newState: *mut boxBitmask_t) {
    rcModeActivationMask = *newState;
}
#[no_mangle]
pub unsafe extern "C" fn isAirmodeActive() -> bool {
    return IS_RC_MODE_ACTIVE(BOXAIRMODE) as libc::c_int != 0 ||
               feature(FEATURE_AIRMODE as libc::c_int as uint32_t) as
                   libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn isRangeActive(mut auxChannelIndex: uint8_t,
                                       mut range: *const channelRange_t)
 -> bool {
    if !(((*range).startStep as libc::c_int) <
             (*range).endStep as libc::c_int) {
        return 0 as libc::c_int != 0
    }
    let channelValue: uint16_t =
        constrain(rcData[(auxChannelIndex as libc::c_int + 4 as libc::c_int)
                             as usize] as libc::c_int, 900 as libc::c_int,
                  2100 as libc::c_int - 1 as libc::c_int) as uint16_t;
    return channelValue as libc::c_int >=
               900 as libc::c_int +
                   (*range).startStep as libc::c_int * 25 as libc::c_int &&
               (channelValue as libc::c_int) <
                   900 as libc::c_int +
                       (*range).endStep as libc::c_int * 25 as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn updateMasksForMac(mut mac:
                                               *const modeActivationCondition_t,
                                           mut andMask: *mut boxBitmask_t,
                                           mut newMask: *mut boxBitmask_t) {
    let mut bAnd: bool =
        (*mac).modeLogic as libc::c_uint ==
            MODELOGIC_AND as libc::c_int as libc::c_uint ||
            bitArrayGet(andMask as *const libc::c_void,
                        (*mac).modeId as libc::c_uint) as libc::c_int != 0;
    let mut bAct: bool = isRangeActive((*mac).auxChannelIndex, &(*mac).range);
    if bAnd {
        bitArraySet(andMask as *mut libc::c_void,
                    (*mac).modeId as libc::c_uint);
    }
    if bAnd as libc::c_int != bAct as libc::c_int {
        bitArraySet(newMask as *mut libc::c_void,
                    (*mac).modeId as libc::c_uint);
    };
}
#[no_mangle]
pub unsafe extern "C" fn updateMasksForStickyModes(mut mac:
                                                       *const modeActivationCondition_t,
                                                   mut andMask:
                                                       *mut boxBitmask_t,
                                                   mut newMask:
                                                       *mut boxBitmask_t) {
    if IS_RC_MODE_ACTIVE((*mac).modeId) {
        bitArrayClr(andMask as *mut libc::c_void,
                    (*mac).modeId as libc::c_uint);
        bitArraySet(newMask as *mut libc::c_void,
                    (*mac).modeId as libc::c_uint);
    } else if bitArrayGet(&mut stickyModesEverDisabled as *mut boxBitmask_t as
                              *const libc::c_void,
                          (*mac).modeId as libc::c_uint) {
        updateMasksForMac(mac, andMask, newMask);
    } else if micros() as libc::c_double >= 5e6f64 &&
                  !isRangeActive((*mac).auxChannelIndex, &(*mac).range) {
        bitArraySet(&mut stickyModesEverDisabled as *mut boxBitmask_t as
                        *mut libc::c_void, (*mac).modeId as libc::c_uint);
    };
}
#[no_mangle]
pub unsafe extern "C" fn updateActivatedModes() {
    let mut newMask: boxBitmask_t = boxBitmask_t{bits: [0; 2],};
    let mut andMask: boxBitmask_t = boxBitmask_t{bits: [0; 2],};
    let mut stickyModes: boxBitmask_t = boxBitmask_t{bits: [0; 2],};
    memset(&mut andMask as *mut boxBitmask_t as *mut libc::c_void,
           0 as libc::c_int,
           ::core::mem::size_of::<boxBitmask_t>() as libc::c_ulong);
    memset(&mut newMask as *mut boxBitmask_t as *mut libc::c_void,
           0 as libc::c_int,
           ::core::mem::size_of::<boxBitmask_t>() as libc::c_ulong);
    memset(&mut stickyModes as *mut boxBitmask_t as *mut libc::c_void,
           0 as libc::c_int,
           ::core::mem::size_of::<boxBitmask_t>() as libc::c_ulong);
    bitArraySet(&mut stickyModes as *mut boxBitmask_t as *mut libc::c_void,
                BOXPARALYZE as libc::c_int as libc::c_uint);
    // determine which conditions set/clear the mode
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 20 as libc::c_int {
        let mut mac: *const modeActivationCondition_t =
            modeActivationConditions(i);
        // Skip linked macs for now to fully determine target states
        if !((*mac).linkedTo as u64 != 0) {
            if bitArrayGet(&mut stickyModes as *mut boxBitmask_t as
                               *const libc::c_void,
                           (*mac).modeId as libc::c_uint) {
                updateMasksForStickyModes(mac, &mut andMask, &mut newMask);
            } else if ((*mac).modeId as libc::c_uint) <
                          CHECKBOX_ITEM_COUNT as libc::c_int as libc::c_uint {
                updateMasksForMac(mac, &mut andMask, &mut newMask);
            }
        }
        i += 1
    }
    bitArrayXor(&mut newMask as *mut boxBitmask_t as *mut libc::c_void,
                ::core::mem::size_of::<*mut boxBitmask_t>() as libc::c_ulong,
                &mut newMask as *mut boxBitmask_t as *mut libc::c_void,
                &mut andMask as *mut boxBitmask_t as *mut libc::c_void);
    // Update linked modes
    let mut i_0: libc::c_int = 0 as libc::c_int;
    while i_0 < 20 as libc::c_int {
        let mut mac_0: *const modeActivationCondition_t =
            modeActivationConditions(i_0);
        if !((*mac_0).linkedTo as u64 == 0) {
            bitArrayCopy(&mut newMask as *mut boxBitmask_t as
                             *mut libc::c_void,
                         (*mac_0).linkedTo as libc::c_uint,
                         (*mac_0).modeId as libc::c_uint);
        }
        i_0 += 1
    }
    rcModeUpdate(&mut newMask);
}
#[no_mangle]
pub unsafe extern "C" fn isModeActivationConditionPresent(mut modeId: boxId_e)
 -> bool {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 20 as libc::c_int {
        let mut mac: *const modeActivationCondition_t =
            modeActivationConditions(i);
        if (*mac).modeId as libc::c_uint == modeId as libc::c_uint &&
               (((*mac).range.startStep as libc::c_int) <
                    (*mac).range.endStep as libc::c_int ||
                    (*mac).linkedTo as libc::c_uint != 0) {
            return 1 as libc::c_int != 0
        }
        i += 1
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn removeModeActivationCondition(modeId: boxId_e) {
    let mut offset: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i < 20 as libc::c_int as libc::c_uint {
        let mut mac: *mut modeActivationCondition_t =
            modeActivationConditionsMutable(i as libc::c_int);
        if (*mac).modeId as libc::c_uint == modeId as libc::c_uint &&
               offset == 0 {
            offset = offset.wrapping_add(1)
        }
        if offset != 0 {
            while i.wrapping_add(offset) < 20 as libc::c_int as libc::c_uint
                      &&
                      (*modeActivationConditions(i.wrapping_add(offset) as
                                                     libc::c_int)).modeId as
                          libc::c_uint == modeId as libc::c_uint {
                offset = offset.wrapping_add(1)
            }
            if i.wrapping_add(offset) < 20 as libc::c_int as libc::c_uint {
                memcpy(mac as *mut libc::c_void,
                       modeActivationConditions(i.wrapping_add(offset) as
                                                    libc::c_int) as
                           *const libc::c_void,
                       ::core::mem::size_of::<modeActivationCondition_t>() as
                           libc::c_ulong);
            } else {
                memset(mac as *mut libc::c_void, 0 as libc::c_int,
                       ::core::mem::size_of::<modeActivationCondition_t>() as
                           libc::c_ulong);
            }
        }
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn run_static_initializers() {
    modeActivationConditions_Registry =
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (41 as libc::c_int |
                                      (1 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 ((::core::mem::size_of::<modeActivationCondition_t>()
                                       as
                                       libc::c_ulong).wrapping_mul(20 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_ulong)
                                      |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &mut modeActivationConditions_SystemArray as
                                     *mut [modeActivationCondition_t; 20] as
                                     *mut uint8_t,
                             copy:
                                 &mut modeActivationConditions_CopyArray as
                                     *mut [modeActivationCondition_t; 20] as
                                     *mut uint8_t,
                             ptr: 0 as *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     0 as
                                                         *mut libc::c_void,},};
            init
        }
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
