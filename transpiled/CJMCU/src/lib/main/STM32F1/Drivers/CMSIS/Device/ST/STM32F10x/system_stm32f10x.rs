use ::libc;
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
pub struct RCC_TypeDef {
    pub CR: uint32_t,
    pub CFGR: uint32_t,
    pub CIR: uint32_t,
    pub APB2RSTR: uint32_t,
    pub APB1RSTR: uint32_t,
    pub AHBENR: uint32_t,
    pub APB2ENR: uint32_t,
    pub APB1ENR: uint32_t,
    pub BDCR: uint32_t,
    pub CSR: uint32_t,
}
pub const RESET: C2RustUnnamed = 0;
pub type C2RustUnnamed = libc::c_uint;
pub const SET: C2RustUnnamed = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FLASH_TypeDef {
    pub ACR: uint32_t,
    pub KEYR: uint32_t,
    pub OPTKEYR: uint32_t,
    pub SR: uint32_t,
    pub CR: uint32_t,
    pub AR: uint32_t,
    pub RESERVED: uint32_t,
    pub OBR: uint32_t,
    pub WRPR: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub CRL: uint32_t,
    pub CRH: uint32_t,
    pub IDR: uint32_t,
    pub ODR: uint32_t,
    pub BSRR: uint32_t,
    pub BRR: uint32_t,
    pub LCKR: uint32_t,
}
pub type C2RustUnnamed_0 = libc::c_uint;
pub const SRC_HSE: C2RustUnnamed_0 = 2;
pub const SRC_HSI: C2RustUnnamed_0 = 1;
pub const SRC_NONE: C2RustUnnamed_0 = 0;
#[no_mangle]
pub static mut SystemCoreClock: uint32_t =
    72000000 as libc::c_int as uint32_t;
/* !< System Clock Frequency (Core Clock) */
static mut AHBPrescTable: [uint8_t; 16] =
    [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     1 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t,
     3 as libc::c_int as uint8_t, 4 as libc::c_int as uint8_t,
     6 as libc::c_int as uint8_t, 7 as libc::c_int as uint8_t,
     8 as libc::c_int as uint8_t, 9 as libc::c_int as uint8_t];
#[no_mangle]
pub static mut hse_value: uint32_t = 8000000 as libc::c_int as uint32_t;
/* *
  ******************************************************************************
  * @file    system_stm32f10x.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Header File.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
/* * @addtogroup CMSIS
  * @{
  */
/* * @addtogroup stm32f10x_system
  * @{
  */
/* *
  * @brief Define to prevent recursive inclusion
  */
/* * @addtogroup STM32F10x_System_Includes
  * @{
  */
/* *
  * @}
  */
/* * @addtogroup STM32F10x_System_Exported_types
  * @{
  */
/* !< System Clock Frequency (Core Clock) */
/* *
  * @}
  */
/* * @addtogroup STM32F10x_System_Exported_Constants
  * @{
  */
/* *
  * @}
  */
/* * @addtogroup STM32F10x_System_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @addtogroup STM32F10x_System_Exported_Functions
  * @{
  */
#[no_mangle]
pub unsafe extern "C" fn SystemInit() {
    /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
    /* Set HSION bit */
    let ref mut fresh0 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x1 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
    /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
    let ref mut fresh1 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xf8ff0000 as libc::c_uint) as uint32_t
                                    as uint32_t);
    /* Reset HSEON, CSSON and PLLON bits */
    let ref mut fresh2 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xfef6ffff as libc::c_uint) as uint32_t
                                    as uint32_t);
    /* Reset HSEBYP bit */
    let ref mut fresh3 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh3,
                                (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xfffbffff as libc::c_uint) as uint32_t
                                    as uint32_t);
    /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
    let ref mut fresh4 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    ::core::ptr::write_volatile(fresh4,
                                (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xff80ffff as libc::c_uint) as uint32_t
                                    as uint32_t);
    /* Disable all interrupts and clear pending bits  */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CIR as
                                    *mut uint32_t,
                                0x9f0000 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd00
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut SCB_Type)).VTOR as
                                    *mut uint32_t,
                                0x8000000 as libc::c_int as uint32_t);
    /* Vector Table Relocation in Internal FLASH. */
}
#[no_mangle]
pub unsafe extern "C" fn SystemCoreClockUpdate() {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    let mut pllmull: uint32_t = 0 as libc::c_int as uint32_t;
    let mut pllsource: uint32_t = 0 as libc::c_int as uint32_t;
    /* Get SYSCLK source ------------------------------------------------------- */
    tmp =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR & 0xc as libc::c_int as uint32_t;
    match tmp {
        0 => {
            /* HSI used as system clock */
            SystemCoreClock = 8000000 as libc::c_int as uint32_t
        }
        4 => {
            /* HSE used as system clock */
            SystemCoreClock = hse_value
        }
        8 => {
            /* PLL used as system clock */
            /* Get PLL clock source and multiplication factor ---------------------- */
            pllmull =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x1000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR &
                    0x3c0000 as libc::c_int as uint32_t;
            pllsource =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x1000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR &
                    0x10000 as libc::c_int as uint32_t;
            pllmull =
                (pllmull >>
                     18 as
                         libc::c_int).wrapping_add(2 as libc::c_int as
                                                       libc::c_uint);
            if pllsource == 0 as libc::c_int as libc::c_uint {
                /* HSI oscillator clock divided by 2 selected as PLL clock entry */
                SystemCoreClock =
                    (8000000 as libc::c_int as uint32_t >>
                         1 as libc::c_int).wrapping_mul(pllmull)
            } else if (*((0x40000000 as libc::c_int as
                              uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                         libc::c_uint).wrapping_add(0x1000
                                                                                        as
                                                                                        libc::c_int
                                                                                        as
                                                                                        libc::c_uint)
                             as *mut RCC_TypeDef)).CFGR &
                          0x20000 as libc::c_int as uint32_t !=
                          RESET as libc::c_int as uint32_t {
                /* HSE selected as PLL clock entry */
                /* HSE oscillator clock divided by 2 */
                SystemCoreClock =
                    (hse_value >> 1 as libc::c_int).wrapping_mul(pllmull)
            } else { SystemCoreClock = hse_value.wrapping_mul(pllmull) }
        }
        _ => { SystemCoreClock = 8000000 as libc::c_int as uint32_t }
    }
    /* Compute HCLK clock frequency ---------------- */
    /* Get HCLK prescaler */
    tmp =
        AHBPrescTable[(((*((0x40000000 as libc::c_int as
                                uint32_t).wrapping_add(0x20000 as libc::c_int
                                                           as
                                                           libc::c_uint).wrapping_add(0x1000
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint)
                               as *mut RCC_TypeDef)).CFGR &
                            0xf0 as libc::c_int as uint32_t) >>
                           4 as libc::c_int) as usize] as uint32_t;
    /* HCLK clock frequency */
    SystemCoreClock >>= tmp;
}
// Set system clock to 72 (HSE) or 64 (HSI) MHz
#[no_mangle]
pub unsafe extern "C" fn SetSysClock(mut overclock: bool) {
    let mut StartUpCounter: uint32_t = 0 as libc::c_int as uint32_t;
    let mut status: uint32_t = 0 as libc::c_int as uint32_t;
    let mut clocksrc: uint32_t = SRC_NONE as libc::c_int as uint32_t;
    let mut RCC_CRH: *mut uint32_t =
        &mut (*((0x40000000 as libc::c_int as
                     uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                libc::c_uint).wrapping_add(0x1000
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint)
                    as *mut GPIO_TypeDef)).CRH;
    let mut RCC_CFGR_PLLMUL: uint32_t = 0x1c0000 as libc::c_int as uint32_t;
    // First, try running off HSE
    let ref mut fresh5 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh5,
                                (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x10000 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
    let ref mut fresh6 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh6,
                                (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x10 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
    loop 
         // Wait till HSE is ready
         {
        ::core::ptr::write_volatile(&mut status as *mut uint32_t,
                                    (*((0x40000000 as libc::c_int as
                                            uint32_t).wrapping_add(0x20000 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_uint).wrapping_add(0x1000
                                                                                                      as
                                                                                                      libc::c_int
                                                                                                      as
                                                                                                      libc::c_uint)
                                           as *mut RCC_TypeDef)).CR &
                                        0x20000 as libc::c_int as uint32_t);
        ::core::ptr::write_volatile(&mut StartUpCounter as *mut uint32_t,
                                    ::core::ptr::read_volatile::<uint32_t>(&StartUpCounter
                                                                               as
                                                                               *const uint32_t).wrapping_add(1));
        if !(status == 0 as libc::c_int as libc::c_uint &&
                 StartUpCounter !=
                     0x500 as libc::c_int as uint16_t as libc::c_uint) {
            break ;
        }
    }
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x20000 as libc::c_int as
                                          libc::c_uint).wrapping_add(0x1000 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint)
              as *mut RCC_TypeDef)).CR & 0x20000 as libc::c_int as uint32_t !=
           RESET as libc::c_int as libc::c_uint {
        // external xtal started up, we're good to go
        ::core::ptr::write_volatile(&mut clocksrc as *mut uint32_t,
                                    SRC_HSE as libc::c_int as uint32_t)
    } else {
        // If HSE fails to start-up, try to enable HSI and configure for 64MHz operation
        let ref mut fresh7 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh7,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut StartUpCounter as *mut uint32_t,
                                    0 as libc::c_int as uint32_t);
        loop  {
            ::core::ptr::write_volatile(&mut status as *mut uint32_t,
                                        (*((0x40000000 as libc::c_int as
                                                uint32_t).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x1000
                                                                                                          as
                                                                                                          libc::c_int
                                                                                                          as
                                                                                                          libc::c_uint)
                                               as *mut RCC_TypeDef)).CR &
                                            0x2 as libc::c_int as uint32_t);
            ::core::ptr::write_volatile(&mut StartUpCounter as *mut uint32_t,
                                        ::core::ptr::read_volatile::<uint32_t>(&StartUpCounter
                                                                                   as
                                                                                   *const uint32_t).wrapping_add(1));
            if !(status == 0 as libc::c_int as libc::c_uint &&
                     StartUpCounter !=
                         0x500 as libc::c_int as uint16_t as libc::c_uint) {
                break ;
            }
        }
        if (*((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x20000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0x1000
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                  as *mut RCC_TypeDef)).CR & 0x2 as libc::c_int as uint32_t !=
               RESET as libc::c_int as libc::c_uint {
            // we're on internal RC
            ::core::ptr::write_volatile(&mut clocksrc as *mut uint32_t,
                                        SRC_HSI as libc::c_int as uint32_t)
        } else {
            loop  { }
            // Unable to continue.
        }
    }
    // Enable Prefetch Buffer
    let ref mut fresh8 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x2000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut FLASH_TypeDef)).ACR;
    ::core::ptr::write_volatile(fresh8,
                                (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x10 as libc::c_int as uint8_t as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
    // Flash 2 wait state
    let ref mut fresh9 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x2000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut FLASH_TypeDef)).ACR;
    ::core::ptr::write_volatile(fresh9,
                                (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x3 as libc::c_int as uint8_t as
                                           libc::c_int) as uint32_t) as
                                    uint32_t as uint32_t);
    let ref mut fresh10 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x2000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut FLASH_TypeDef)).ACR;
    ::core::ptr::write_volatile(fresh10,
                                (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x2 as libc::c_int as uint8_t as
                                         uint32_t) as uint32_t as uint32_t);
    // HCLK = SYSCLK
    let ref mut fresh11 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    ::core::ptr::write_volatile(fresh11,
                                (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0 as libc::c_int as uint32_t) as uint32_t
                                    as uint32_t);
    // PCLK2 = HCLK
    let ref mut fresh12 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    ::core::ptr::write_volatile(fresh12,
                                (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0 as libc::c_int as uint32_t) as uint32_t
                                    as uint32_t);
    // PCLK1 = HCLK
    let ref mut fresh13 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    ::core::ptr::write_volatile(fresh13,
                                (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x400 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
    ::core::ptr::write_volatile(RCC_CRH,
                                (::core::ptr::read_volatile::<uint32_t>(RCC_CRH
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0xf as libc::c_int as uint32_t) <<
                                           (0x1c0000 as libc::c_int as
                                                uint32_t >>
                                                16 as libc::c_int))) as
                                    uint32_t as uint32_t);
    // Configure PLL
    hse_value = 8000000 as libc::c_int as uint32_t;
    let ref mut fresh14 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    ::core::ptr::write_volatile(fresh14,
                                (::core::ptr::read_volatile::<uint32_t>(fresh14
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x10000 as libc::c_int as uint32_t |
                                           0x20000 as libc::c_int as uint32_t
                                           |
                                           0x3c0000 as libc::c_int as
                                               uint32_t)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(RCC_CRH,
                                (::core::ptr::read_volatile::<uint32_t>(RCC_CRH
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x8 as libc::c_int as uint32_t) <<
                                         (0x1c0000 as libc::c_int as uint32_t
                                              >> 16 as libc::c_int)) as
                                    uint32_t as uint32_t);
    let ref mut fresh15 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut GPIO_TypeDef)).ODR;
    ::core::ptr::write_volatile(fresh15,
                                (::core::ptr::read_volatile::<uint32_t>(fresh15
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x8000 as libc::c_int as uint16_t as
                                           libc::c_int) as uint32_t) as
                                    uint32_t as uint32_t);
    // On CJMCU new revision boards (Late 2014) bit 15 of GPIOC->IDR is '1'.
    ::core::ptr::write_volatile(&mut RCC_CFGR_PLLMUL as *mut uint32_t,
                                0x1c0000 as libc::c_int as uint32_t);
    match clocksrc {
        2 => {
            if overclock {
                if RCC_CFGR_PLLMUL == 0x100000 as libc::c_int as uint32_t {
                    ::core::ptr::write_volatile(&mut RCC_CFGR_PLLMUL as
                                                    *mut uint32_t,
                                                0x140000 as libc::c_int as
                                                    uint32_t)
                } else if RCC_CFGR_PLLMUL ==
                              0x1c0000 as libc::c_int as uint32_t {
                    ::core::ptr::write_volatile(&mut RCC_CFGR_PLLMUL as
                                                    *mut uint32_t,
                                                0x200000 as libc::c_int as
                                                    uint32_t)
                }
            }
            // overclock=false : PLL configuration: PLLCLK = HSE * 9 = 72 MHz || HSE * 6 = 72 MHz
            // overclock=true  : PLL configuration: PLLCLK = HSE * 10 = 80 MHz || HSE * 7 = 84 MHz
            let ref mut fresh16 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x1000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR;
            ::core::ptr::write_volatile(fresh16,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh16
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x10000 as libc::c_int as
                                                  uint32_t | RCC_CFGR_PLLMUL))
                                            as uint32_t as uint32_t)
        }
        1 => {
            // PLL configuration: PLLCLK = HSI / 2 * 16 = 64 MHz
            let ref mut fresh17 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x1000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR;
            ::core::ptr::write_volatile(fresh17,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh17
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0 as libc::c_int as uint32_t |
                                                  0x380000 as libc::c_int as
                                                      uint32_t)) as uint32_t
                                            as uint32_t)
        }
        _ => { }
    }
    // Enable PLL
    let ref mut fresh18 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh18,
                                (::core::ptr::read_volatile::<uint32_t>(fresh18
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x1000000 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
    // Wait till PLL is ready
    while (*((0x40000000 as libc::c_int as
                  uint32_t).wrapping_add(0x20000 as libc::c_int as
                                             libc::c_uint).wrapping_add(0x1000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                 as *mut RCC_TypeDef)).CR &
              0x2000000 as libc::c_int as uint32_t ==
              0 as libc::c_int as libc::c_uint {
    }
    // Select PLL as system clock source
    let ref mut fresh19 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    ::core::ptr::write_volatile(fresh19,
                                (::core::ptr::read_volatile::<uint32_t>(fresh19
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x3 as libc::c_int as uint32_t)) as
                                    uint32_t as uint32_t);
    let ref mut fresh20 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    ::core::ptr::write_volatile(fresh20,
                                (::core::ptr::read_volatile::<uint32_t>(fresh20
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x2 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
    // Wait till PLL is used as system clock source
    while (*((0x40000000 as libc::c_int as
                  uint32_t).wrapping_add(0x20000 as libc::c_int as
                                             libc::c_uint).wrapping_add(0x1000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                 as *mut RCC_TypeDef)).CFGR & 0xc as libc::c_int as uint32_t
              != 0x8 as libc::c_int as uint32_t {
    }
    SystemCoreClockUpdate();
}
