use ::libc;
extern "C" {
    #[no_mangle]
    static timerHardware: [timerHardware_t; 0];
    #[no_mangle]
    fn transponderIrInit(ioTag: ioTag_t, provider: transponderProvider_e)
     -> bool;
    #[no_mangle]
    fn transponderIrUpdateData(transponderData: *const uint8_t);
    #[no_mangle]
    fn transponderIrTransmit();
    #[no_mangle]
    fn isTransponderIrReady() -> bool;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* * 
  * @brief DMA Controller
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_Channel_TypeDef {
    pub CCR: uint32_t,
    pub CNDTR: uint32_t,
    pub CPAR: uint32_t,
    pub CMAR: uint32_t,
}
/* *
  * @brief TIM
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint32_t,
    pub SMCR: uint32_t,
    pub DIER: uint32_t,
    pub SR: uint32_t,
    pub EGR: uint32_t,
    pub CCMR1: uint32_t,
    pub CCMR2: uint32_t,
    pub CCER: uint32_t,
    pub CNT: uint32_t,
    pub PSC: uint16_t,
    pub RESERVED9: uint16_t,
    pub ARR: uint32_t,
    pub RCR: uint16_t,
    pub RESERVED10: uint16_t,
    pub CCR1: uint32_t,
    pub CCR2: uint32_t,
    pub CCR3: uint32_t,
    pub CCR4: uint32_t,
    pub BDTR: uint32_t,
    pub DCR: uint16_t,
    pub RESERVED12: uint16_t,
    pub DMAR: uint16_t,
    pub RESERVED13: uint16_t,
    pub OR: uint16_t,
    pub CCMR3: uint32_t,
    pub CCR5: uint32_t,
    pub CCR6: uint32_t,
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
pub type pgn_t = uint16_t;
pub type C2RustUnnamed = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed = 4095;
// function that resets a single parameter group instance
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
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
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
pub type timerUsageFlag_e = libc::c_uint;
pub const TIM_USE_BEEPER: timerUsageFlag_e = 64;
pub const TIM_USE_TRANSPONDER: timerUsageFlag_e = 32;
pub const TIM_USE_LED: timerUsageFlag_e = 16;
pub const TIM_USE_SERVO: timerUsageFlag_e = 8;
pub const TIM_USE_MOTOR: timerUsageFlag_e = 4;
pub const TIM_USE_PWM: timerUsageFlag_e = 2;
pub const TIM_USE_PPM: timerUsageFlag_e = 1;
pub const TIM_USE_NONE: timerUsageFlag_e = 0;
pub const TIM_USE_ANY: timerUsageFlag_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerHardware_s {
    pub tim: *mut TIM_TypeDef,
    pub tag: ioTag_t,
    pub channel: uint8_t,
    pub usageFlags: timerUsageFlag_e,
    pub output: uint8_t,
    pub alternateFunction: uint8_t,
    pub dmaRef: *mut DMA_Channel_TypeDef,
    pub dmaIrqHandler: uint8_t,
    pub dmaTimUPRef: *mut DMA_Channel_TypeDef,
    pub dmaTimUPIrqHandler: uint8_t,
}
pub type timerHardware_t = timerHardware_s;
pub type transponderProvider_e = libc::c_uint;
pub const TRANSPONDER_ERLT: transponderProvider_e = 3;
pub const TRANSPONDER_ARCITIMER: transponderProvider_e = 2;
pub const TRANSPONDER_ILAP: transponderProvider_e = 1;
pub const TRANSPONDER_NONE: transponderProvider_e = 0;
// TIMUP
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
// time difference, 32 bits always sufficient
pub type timeDelta_t = int32_t;
pub type timeUs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct transponderConfig_s {
    pub provider: transponderProvider_e,
    pub reserved: uint8_t,
    pub data: [uint8_t; 9],
    pub ioTag: ioTag_t,
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
pub type transponderConfig_t = transponderConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct transponderRequirement_s {
    pub provider: uint8_t,
    pub dataLength: uint8_t,
    pub transmitDelay: uint16_t,
    pub transmitJitter: uint16_t,
}
pub type transponderRequirement_t = transponderRequirement_s;
#[inline]
unsafe extern "C" fn transponderConfig() -> *const transponderConfig_t {
    return &mut transponderConfig_System;
}
// See transponderProvider_e
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
#[link_section = ".pg_registry"]
#[used]
pub static mut transponderConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (17 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<transponderConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &transponderConfig_System as
                                     *const transponderConfig_t as
                                     *mut transponderConfig_t as *mut uint8_t,
                             copy:
                                 &transponderConfig_Copy as
                                     *const transponderConfig_t as
                                     *mut transponderConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut transponderConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_transponderConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut transponderConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
    };
#[no_mangle]
pub static mut transponderConfig_Copy: transponderConfig_t =
    transponderConfig_t{provider: TRANSPONDER_NONE,
                        reserved: 0,
                        data: [0; 9],
                        ioTag: 0,};
#[no_mangle]
pub static mut transponderConfig_System: transponderConfig_t =
    transponderConfig_t{provider: TRANSPONDER_NONE,
                        reserved: 0,
                        data: [0; 9],
                        ioTag: 0,};
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_transponderConfig(mut transponderConfig_0:
                                                         *mut transponderConfig_t) {
    *transponderConfig_0 =
        {
            let mut init =
                transponderConfig_s{provider: TRANSPONDER_ILAP,
                                    reserved: 0 as libc::c_int as uint8_t,
                                    data:
                                        [0x12 as libc::c_int as uint8_t,
                                         0x34 as libc::c_int as uint8_t,
                                         0x56 as libc::c_int as uint8_t,
                                         0x78 as libc::c_int as uint8_t,
                                         0x9a as libc::c_int as uint8_t,
                                         0xbc as libc::c_int as uint8_t,
                                         0 as libc::c_int as uint8_t,
                                         0 as libc::c_int as uint8_t,
                                         0 as libc::c_int as uint8_t],
                                    ioTag: 0 as libc::c_int as ioTag_t,};
            init
        };
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 9 as libc::c_int {
        if (*timerHardware.as_ptr().offset(i as isize)).usageFlags as
               libc::c_uint &
               TIM_USE_TRANSPONDER as libc::c_int as libc::c_uint != 0 {
            (*transponderConfig_0).ioTag =
                (*timerHardware.as_ptr().offset(i as isize)).tag;
            break ;
        } else { i += 1 }
    };
}
static mut transponderInitialised: bool = 0 as libc::c_int != 0;
static mut transponderRepeat: bool = 0 as libc::c_int != 0;
// timers
static mut nextUpdateAtUs: timeUs_t = 0 as libc::c_int as timeUs_t;
static mut jitterDurations: [uint8_t; 15] =
    [0 as libc::c_int as uint8_t, 9 as libc::c_int as uint8_t,
     4 as libc::c_int as uint8_t, 8 as libc::c_int as uint8_t,
     3 as libc::c_int as uint8_t, 9 as libc::c_int as uint8_t,
     6 as libc::c_int as uint8_t, 7 as libc::c_int as uint8_t,
     1 as libc::c_int as uint8_t, 6 as libc::c_int as uint8_t,
     9 as libc::c_int as uint8_t, 7 as libc::c_int as uint8_t,
     8 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t,
     6 as libc::c_int as uint8_t];
#[no_mangle]
pub static mut transponderRequirements: [transponderRequirement_t; 3] =
    [{
         let mut init =
             transponderRequirement_s{provider:
                                          TRANSPONDER_ILAP as libc::c_int as
                                              uint8_t,
                                      dataLength: 6 as libc::c_int as uint8_t,
                                      transmitDelay:
                                          4500 as libc::c_int as uint16_t,
                                      transmitJitter:
                                          10000 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             transponderRequirement_s{provider:
                                          TRANSPONDER_ARCITIMER as libc::c_int
                                              as uint8_t,
                                      dataLength: 9 as libc::c_int as uint8_t,
                                      transmitDelay:
                                          4500 as libc::c_int as uint16_t,
                                      transmitJitter:
                                          10000 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             transponderRequirement_s{provider:
                                          TRANSPONDER_ERLT as libc::c_int as
                                              uint8_t,
                                      dataLength: 1 as libc::c_int as uint8_t,
                                      transmitDelay:
                                          22500 as libc::c_int as uint16_t,
                                      transmitJitter:
                                          5000 as libc::c_int as uint16_t,};
         init
     }];
#[no_mangle]
pub unsafe extern "C" fn transponderUpdate(mut currentTimeUs: timeUs_t) {
    static mut jitterIndex: uint32_t = 0 as libc::c_int as uint32_t;
    if !(transponderInitialised as libc::c_int != 0 &&
             transponderRepeat as libc::c_int != 0 &&
             isTransponderIrReady() as libc::c_int != 0) {
        return
    }
    let updateNow: bool =
        currentTimeUs.wrapping_sub(nextUpdateAtUs) as timeDelta_t as
            libc::c_long >= 0 as libc::c_long;
    if !updateNow { return }
    let mut provider: uint8_t = (*transponderConfig()).provider as uint8_t;
    // TODO use a random number generator for random jitter?  The idea here is to avoid multiple transmitters transmitting at the same time.
    let fresh0 = jitterIndex;
    jitterIndex = jitterIndex.wrapping_add(1);
    let mut jitter: uint32_t =
        (transponderRequirements[(provider as libc::c_int - 1 as libc::c_int)
                                     as usize].transmitJitter as libc::c_int /
             10 as libc::c_int *
             jitterDurations[fresh0 as usize] as libc::c_int) as uint32_t;
    if jitterIndex as libc::c_ulong >=
           (::core::mem::size_of::<[uint8_t; 15]>() as
                libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                as libc::c_ulong) {
        jitterIndex = 0 as libc::c_int as uint32_t
    }
    nextUpdateAtUs =
        currentTimeUs.wrapping_add(transponderRequirements[(provider as
                                                                libc::c_int -
                                                                1 as
                                                                    libc::c_int)
                                                               as
                                                               usize].transmitDelay
                                       as libc::c_uint).wrapping_add(jitter);
    transponderIrTransmit();
}
#[no_mangle]
pub unsafe extern "C" fn transponderInit() {
    transponderInitialised =
        transponderIrInit((*transponderConfig()).ioTag,
                          (*transponderConfig()).provider);
    if !transponderInitialised { return }
    transponderIrUpdateData((*transponderConfig()).data.as_ptr());
}
#[no_mangle]
pub unsafe extern "C" fn transponderStopRepeating() {
    transponderRepeat = 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn transponderStartRepeating() {
    if !transponderInitialised { return }
    transponderRepeat = 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn transponderUpdateData() {
    if !transponderInitialised { return }
    transponderIrUpdateData((*transponderConfig()).data.as_ptr());
}
#[no_mangle]
pub unsafe extern "C" fn transponderTransmitOnce() {
    if !transponderInitialised { return }
    transponderIrTransmit();
}
