use ::libc;
extern "C" {
    #[no_mangle]
    fn RCC_GetClocksFreq(RCC_Clocks: *mut RCC_ClocksTypeDef);
    #[no_mangle]
    fn ledToggle(led: libc::c_int);
    #[no_mangle]
    fn ledSet(led: libc::c_int, state: bool);
    #[no_mangle]
    fn systemResetToBootloader();
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SCB_Type {
    pub CPUID: uint32_t,
    pub ICSR: uint32_t,
    pub VTOR: uint32_t,
    pub AIRCR: uint32_t,
    pub SCR: uint32_t,
    pub CCR: uint32_t,
    pub SHP: [uint8_t; 12],
    pub SHCSR: uint32_t,
    pub CFSR: uint32_t,
    pub HFSR: uint32_t,
    pub DFSR: uint32_t,
    pub MMFAR: uint32_t,
    pub BFAR: uint32_t,
    pub AFSR: uint32_t,
    pub PFR: [uint32_t; 2],
    pub DFR: uint32_t,
    pub ADR: uint32_t,
    pub MMFR: [uint32_t; 4],
    pub ISAR: [uint32_t; 5],
    pub RESERVED0: [uint32_t; 5],
    pub CPACR: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SysTick_Type {
    pub CTRL: uint32_t,
    pub LOAD: uint32_t,
    pub VAL: uint32_t,
    pub CALIB: uint32_t,
}
/* *
  ******************************************************************************
  * @file    stm32f30x_rcc.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the RCC 
  *          firmware library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F30x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup RCC
  * @{
  */
/* Exported types ------------------------------------------------------------*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_ClocksTypeDef {
    pub SYSCLK_Frequency: uint32_t,
    pub HCLK_Frequency: uint32_t,
    pub PCLK1_Frequency: uint32_t,
    pub PCLK2_Frequency: uint32_t,
    pub ADC12CLK_Frequency: uint32_t,
    pub ADC34CLK_Frequency: uint32_t,
    pub I2C1CLK_Frequency: uint32_t,
    pub I2C2CLK_Frequency: uint32_t,
    pub I2C3CLK_Frequency: uint32_t,
    pub TIM1CLK_Frequency: uint32_t,
    pub HRTIM1CLK_Frequency: uint32_t,
    pub TIM8CLK_Frequency: uint32_t,
    pub USART1CLK_Frequency: uint32_t,
    pub USART2CLK_Frequency: uint32_t,
    pub USART3CLK_Frequency: uint32_t,
    pub UART4CLK_Frequency: uint32_t,
    pub UART5CLK_Frequency: uint32_t,
    pub TIM15CLK_Frequency: uint32_t,
    pub TIM16CLK_Frequency: uint32_t,
    pub TIM17CLK_Frequency: uint32_t,
}
pub type failureMode_e = libc::c_uint;
pub const FAILURE_GYRO_INIT_FAILED: failureMode_e = 6;
pub const FAILURE_FLASH_WRITE_FAILED: failureMode_e = 5;
pub const FAILURE_INVALID_EEPROM_CONTENTS: failureMode_e = 4;
pub const FAILURE_ACC_INCOMPATIBLE: failureMode_e = 3;
pub const FAILURE_ACC_INIT: failureMode_e = 2;
pub const FAILURE_MISSING_ACC: failureMode_e = 1;
pub const FAILURE_DEVELOPER: failureMode_e = 0;
// set BASEPRI_MAX, with global memory barrier, returns true
#[inline]
unsafe extern "C" fn __basepriSetMemRetVal(mut prio: uint8_t) -> uint8_t {
    __set_BASEPRI_MAX(prio as libc::c_int);
    return 1 as libc::c_int as uint8_t;
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
// cycles per microsecond
static mut usTicks: uint32_t = 0 as libc::c_int as uint32_t;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static mut sysTickUptime: uint32_t = 0 as libc::c_int as uint32_t;
static mut sysTickValStamp: uint32_t = 0 as libc::c_int as uint32_t;
// cached value of RCC->CSR
#[no_mangle]
pub static mut cachedRccCsrValue: uint32_t = 0;
#[no_mangle]
pub unsafe extern "C" fn cycleCounterInit() {
    let mut clocks: RCC_ClocksTypeDef =
        RCC_ClocksTypeDef{SYSCLK_Frequency: 0,
                          HCLK_Frequency: 0,
                          PCLK1_Frequency: 0,
                          PCLK2_Frequency: 0,
                          ADC12CLK_Frequency: 0,
                          ADC34CLK_Frequency: 0,
                          I2C1CLK_Frequency: 0,
                          I2C2CLK_Frequency: 0,
                          I2C3CLK_Frequency: 0,
                          TIM1CLK_Frequency: 0,
                          HRTIM1CLK_Frequency: 0,
                          TIM8CLK_Frequency: 0,
                          USART1CLK_Frequency: 0,
                          USART2CLK_Frequency: 0,
                          USART3CLK_Frequency: 0,
                          UART4CLK_Frequency: 0,
                          UART5CLK_Frequency: 0,
                          TIM15CLK_Frequency: 0,
                          TIM16CLK_Frequency: 0,
                          TIM17CLK_Frequency: 0,};
    RCC_GetClocksFreq(&mut clocks);
    usTicks =
        clocks.SYSCLK_Frequency.wrapping_div(1000000 as libc::c_int as
                                                 libc::c_uint);
}
// SysTick
static mut sysTickPending: libc::c_int = 0 as libc::c_int;
#[no_mangle]
pub unsafe extern "C" fn SysTick_Handler() {
    let mut __basepri_save: uint8_t = __get_BASEPRI() as uint8_t;
    let mut __ToDo: uint8_t =
        __basepriSetMemRetVal((((0 as libc::c_int) <<
                                    (4 as libc::c_int as
                                         libc::c_uint).wrapping_sub((7 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint).wrapping_sub(0x500
                                                                                                        as
                                                                                                        libc::c_int
                                                                                                        as
                                                                                                        uint32_t
                                                                                                        >>
                                                                                                        8
                                                                                                            as
                                                                                                            libc::c_int))
                                    |
                                    1 as libc::c_int &
                                        0xf as libc::c_int >>
                                            (7 as libc::c_int as
                                                 libc::c_uint).wrapping_sub(0x500
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                uint32_t
                                                                                >>
                                                                                8
                                                                                    as
                                                                                    libc::c_int))
                                   << 4 as libc::c_int & 0xf0 as libc::c_int)
                                  as uint8_t);
    while __ToDo != 0 {
        ::core::ptr::write_volatile(&mut sysTickUptime as *mut uint32_t,
                                    ::core::ptr::read_volatile::<uint32_t>(&sysTickUptime
                                                                               as
                                                                               *const uint32_t).wrapping_add(1));
        ::core::ptr::write_volatile(&mut sysTickValStamp as *mut uint32_t,
                                    (*((0xe000e000 as
                                            libc::c_ulong).wrapping_add(0x10
                                                                            as
                                                                            libc::c_ulong)
                                           as *mut SysTick_Type)).VAL);
        ::core::ptr::write_volatile(&mut sysTickPending as *mut libc::c_int,
                                    0 as libc::c_int);
        __ToDo = 0 as libc::c_int as uint8_t
    };
}
// Return system uptime in microseconds (rollover in 70minutes)
#[no_mangle]
pub unsafe extern "C" fn microsISR() -> uint32_t {
    let mut ms: uint32_t = 0;
    let mut pending: uint32_t = 0;
    let mut cycle_cnt: uint32_t = 0;
    let mut __basepri_save: uint8_t = __get_BASEPRI() as uint8_t;
    let mut __ToDo: uint8_t =
        __basepriSetMemRetVal((((0 as libc::c_int) <<
                                    (4 as libc::c_int as
                                         libc::c_uint).wrapping_sub((7 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint).wrapping_sub(0x500
                                                                                                        as
                                                                                                        libc::c_int
                                                                                                        as
                                                                                                        uint32_t
                                                                                                        >>
                                                                                                        8
                                                                                                            as
                                                                                                            libc::c_int))
                                    |
                                    1 as libc::c_int &
                                        0xf as libc::c_int >>
                                            (7 as libc::c_int as
                                                 libc::c_uint).wrapping_sub(0x500
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                uint32_t
                                                                                >>
                                                                                8
                                                                                    as
                                                                                    libc::c_int))
                                   << 4 as libc::c_int & 0xf0 as libc::c_int)
                                  as uint8_t);
    while __ToDo != 0 {
        cycle_cnt =
            (*((0xe000e000 as
                    libc::c_ulong).wrapping_add(0x10 as libc::c_ulong) as
                   *mut SysTick_Type)).VAL;
        if (*((0xe000e000 as
                   libc::c_ulong).wrapping_add(0x10 as libc::c_ulong) as
                  *mut SysTick_Type)).CTRL as libc::c_ulong &
               (1 as libc::c_ulong) << 16 as libc::c_uint != 0 {
            // Update pending.
            // Record it for multiple calls within the same rollover period
            // (Will be cleared when serviced).
            // Note that multiple rollovers are not considered.
            ::core::ptr::write_volatile(&mut sysTickPending as
                                            *mut libc::c_int,
                                        1 as libc::c_int);
            // Read VAL again to ensure the value is read after the rollover.
            cycle_cnt =
                (*((0xe000e000 as
                        libc::c_ulong).wrapping_add(0x10 as libc::c_ulong) as
                       *mut SysTick_Type)).VAL
        }
        ms = sysTickUptime;
        pending = sysTickPending as uint32_t;
        __ToDo = 0 as libc::c_int as uint8_t
    }
    return ms.wrapping_add(pending).wrapping_mul(1000 as libc::c_int as
                                                     libc::c_uint).wrapping_add(usTicks.wrapping_mul(1000
                                                                                                         as
                                                                                                         libc::c_int
                                                                                                         as
                                                                                                         libc::c_uint).wrapping_sub(cycle_cnt).wrapping_div(usTicks));
}
#[no_mangle]
pub unsafe extern "C" fn micros() -> uint32_t {
    let mut ms: uint32_t = 0;
    let mut cycle_cnt: uint32_t = 0;
    // Call microsISR() in interrupt and elevated (non-zero) BASEPRI context
    if (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
              as *mut SCB_Type)).ICSR as libc::c_ulong &
           0x1ff as libc::c_ulong != 0 || __get_BASEPRI() != 0 {
        return microsISR()
    }
    loop  {
        ms = sysTickUptime;
        cycle_cnt =
            (*((0xe000e000 as
                    libc::c_ulong).wrapping_add(0x10 as libc::c_ulong) as
                   *mut SysTick_Type)).VAL;
        if !(ms != sysTickUptime || cycle_cnt > sysTickValStamp) { break ; }
    }
    return ms.wrapping_mul(1000 as libc::c_int as
                               libc::c_uint).wrapping_add(usTicks.wrapping_mul(1000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   libc::c_uint).wrapping_sub(cycle_cnt).wrapping_div(usTicks));
}
// Return system uptime in milliseconds (rollover in 49 days)
#[no_mangle]
pub unsafe extern "C" fn millis() -> uint32_t { return sysTickUptime; }
#[no_mangle]
pub unsafe extern "C" fn delayMicroseconds(mut us: uint32_t) {
    let mut now: uint32_t = micros();
    while micros().wrapping_sub(now) < us { };
}
#[no_mangle]
pub unsafe extern "C" fn delay(mut ms: uint32_t) {
    loop  {
        let fresh0 = ms;
        ms = ms.wrapping_sub(1);
        if !(fresh0 != 0) { break ; }
        delayMicroseconds(1000 as libc::c_int as uint32_t);
    };
}
unsafe extern "C" fn indicate(mut count: uint8_t, mut duration: uint16_t) {
    if count != 0 {
        ledSet(1 as libc::c_int, 1 as libc::c_int != 0);
        ledSet(0 as libc::c_int, 0 as libc::c_int != 0);
        loop  {
            let fresh1 = count;
            count = count.wrapping_sub(1);
            if !(fresh1 != 0) { break ; }
            ledToggle(1 as libc::c_int);
            ledToggle(0 as libc::c_int);
            delay(duration as uint32_t);
            ledToggle(1 as libc::c_int);
            ledToggle(0 as libc::c_int);
            delay(duration as uint32_t);
        }
    };
}
// failure
#[no_mangle]
pub unsafe extern "C" fn indicateFailure(mut mode: failureMode_e,
                                         mut codeRepeatsRemaining:
                                             libc::c_int) {
    loop  {
        let fresh2 = codeRepeatsRemaining;
        codeRepeatsRemaining = codeRepeatsRemaining - 1;
        if !(fresh2 != 0) { break ; }
        indicate(5 as libc::c_int as uint8_t, 50 as libc::c_int as uint16_t);
        delay(500 as libc::c_int as uint32_t);
        indicate((mode as
                      libc::c_uint).wrapping_add(1 as libc::c_int as
                                                     libc::c_uint) as uint8_t,
                 250 as libc::c_int as uint16_t);
        delay(1000 as libc::c_int as uint32_t);
    };
}
#[no_mangle]
pub unsafe extern "C" fn failureMode(mut mode: failureMode_e) {
    indicateFailure(mode, 10 as libc::c_int);
    systemResetToBootloader();
}
