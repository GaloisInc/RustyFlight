use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub MODER: uint32_t,
    pub OTYPER: uint32_t,
    pub OSPEEDR: uint32_t,
    pub PUPDR: uint32_t,
    pub IDR: uint32_t,
    pub ODR: uint32_t,
    pub BSRR: uint32_t,
    pub LCKR: uint32_t,
    pub AFR: [uint32_t; 2],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_TypeDef {
    pub CR: uint32_t,
    pub PLLCFGR: uint32_t,
    pub CFGR: uint32_t,
    pub CIR: uint32_t,
    pub AHB1RSTR: uint32_t,
    pub AHB2RSTR: uint32_t,
    pub AHB3RSTR: uint32_t,
    pub RESERVED0: uint32_t,
    pub APB1RSTR: uint32_t,
    pub APB2RSTR: uint32_t,
    pub RESERVED1: [uint32_t; 2],
    pub AHB1ENR: uint32_t,
    pub AHB2ENR: uint32_t,
    pub AHB3ENR: uint32_t,
    pub RESERVED2: uint32_t,
    pub APB1ENR: uint32_t,
    pub APB2ENR: uint32_t,
    pub RESERVED3: [uint32_t; 2],
    pub AHB1LPENR: uint32_t,
    pub AHB2LPENR: uint32_t,
    pub AHB3LPENR: uint32_t,
    pub RESERVED4: uint32_t,
    pub APB1LPENR: uint32_t,
    pub APB2LPENR: uint32_t,
    pub RESERVED5: [uint32_t; 2],
    pub BDCR: uint32_t,
    pub CSR: uint32_t,
    pub RESERVED6: [uint32_t; 2],
    pub SSCGR: uint32_t,
    pub PLLI2SCFGR: uint32_t,
    pub PLLSAICFGR: uint32_t,
    pub DCKCFGR1: uint32_t,
    pub DCKCFGR2: uint32_t,
}
pub type ErrorStatus = libc::c_uint;
pub const SUCCESS: ErrorStatus = 1;
pub const ERROR: ErrorStatus = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_GPIO_InitTypeDef {
    pub Pin: uint32_t,
    pub Mode: uint32_t,
    pub Speed: uint32_t,
    pub OutputType: uint32_t,
    pub Pull: uint32_t,
    pub Alternate: uint32_t,
}
#[inline(always)]
unsafe extern "C" fn __RBIT(mut value: uint32_t) -> uint32_t {
    let mut result: uint32_t = 0;
    let mut s: uint32_t =
        (4 as
             libc::c_uint).wrapping_mul(8 as
                                            libc::c_uint).wrapping_sub(1 as
                                                                           libc::c_uint);
    result = value;
    value >>= 1 as libc::c_uint;
    while value != 0 as libc::c_uint {
        result <<= 1 as libc::c_uint;
        result |= value & 1 as libc::c_uint;
        s = s.wrapping_sub(1);
        value >>= 1 as libc::c_uint
    }
    result <<= s;
    return result;
}
#[inline]
unsafe extern "C" fn LL_GPIO_SetPinMode(mut GPIOx: *mut GPIO_TypeDef,
                                        mut Pin: uint32_t,
                                        mut Mode: uint32_t) {
    ::core::ptr::write_volatile(&mut (*GPIOx).MODER as *mut uint32_t,
                                (*GPIOx).MODER &
                                    !(((0x3 as libc::c_uint) <<
                                           0 as libc::c_uint) <<
                                          (__RBIT(Pin).leading_zeros() as i32
                                               as uint8_t as
                                               libc::c_uint).wrapping_mul(2 as
                                                                              libc::c_uint))
                                    |
                                    Mode <<
                                        (__RBIT(Pin).leading_zeros() as i32 as
                                             uint8_t as
                                             libc::c_uint).wrapping_mul(2 as
                                                                            libc::c_uint));
}
#[inline]
unsafe extern "C" fn LL_GPIO_SetPinOutputType(mut GPIOx: *mut GPIO_TypeDef,
                                              mut PinMask: uint32_t,
                                              mut OutputType: uint32_t) {
    ::core::ptr::write_volatile(&mut (*GPIOx).OTYPER as *mut uint32_t,
                                (*GPIOx).OTYPER & !PinMask |
                                    PinMask.wrapping_mul(OutputType));
}
#[inline]
unsafe extern "C" fn LL_GPIO_SetPinSpeed(mut GPIOx: *mut GPIO_TypeDef,
                                         mut Pin: uint32_t,
                                         mut Speed: uint32_t) {
    ::core::ptr::write_volatile(&mut (*GPIOx).OSPEEDR as *mut uint32_t,
                                (*GPIOx).OSPEEDR &
                                    !(((0x3 as libc::c_uint) <<
                                           0 as libc::c_uint) <<
                                          (__RBIT(Pin).leading_zeros() as i32
                                               as uint8_t as
                                               libc::c_uint).wrapping_mul(2 as
                                                                              libc::c_uint))
                                    |
                                    Speed <<
                                        (__RBIT(Pin).leading_zeros() as i32 as
                                             uint8_t as
                                             libc::c_uint).wrapping_mul(2 as
                                                                            libc::c_uint));
}
#[inline]
unsafe extern "C" fn LL_GPIO_SetPinPull(mut GPIOx: *mut GPIO_TypeDef,
                                        mut Pin: uint32_t,
                                        mut Pull: uint32_t) {
    ::core::ptr::write_volatile(&mut (*GPIOx).PUPDR as *mut uint32_t,
                                (*GPIOx).PUPDR &
                                    !(((0x3 as libc::c_uint) <<
                                           0 as libc::c_uint) <<
                                          (__RBIT(Pin).leading_zeros() as i32
                                               as uint8_t as
                                               libc::c_uint).wrapping_mul(2 as
                                                                              libc::c_uint))
                                    |
                                    Pull <<
                                        (__RBIT(Pin).leading_zeros() as i32 as
                                             uint8_t as
                                             libc::c_uint).wrapping_mul(2 as
                                                                            libc::c_uint));
}
#[inline]
unsafe extern "C" fn LL_GPIO_SetAFPin_0_7(mut GPIOx: *mut GPIO_TypeDef,
                                          mut Pin: uint32_t,
                                          mut Alternate: uint32_t) {
    ::core::ptr::write_volatile(&mut (*GPIOx).AFR[0 as libc::c_int as usize]
                                    as *mut uint32_t,
                                (*GPIOx).AFR[0 as libc::c_int as usize] &
                                    !(((0xf as libc::c_uint) <<
                                           0 as libc::c_uint) <<
                                          (__RBIT(Pin).leading_zeros() as i32
                                               as uint8_t as
                                               libc::c_uint).wrapping_mul(4 as
                                                                              libc::c_uint))
                                    |
                                    Alternate <<
                                        (__RBIT(Pin).leading_zeros() as i32 as
                                             uint8_t as
                                             libc::c_uint).wrapping_mul(4 as
                                                                            libc::c_uint));
}
#[inline]
unsafe extern "C" fn LL_GPIO_SetAFPin_8_15(mut GPIOx: *mut GPIO_TypeDef,
                                           mut Pin: uint32_t,
                                           mut Alternate: uint32_t) {
    ::core::ptr::write_volatile(&mut (*GPIOx).AFR[1 as libc::c_int as usize]
                                    as *mut uint32_t,
                                (*GPIOx).AFR[1 as libc::c_int as usize] &
                                    !(((0xf as libc::c_uint) <<
                                           0 as libc::c_uint) <<
                                          (__RBIT(Pin >>
                                                      8 as
                                                          libc::c_uint).leading_zeros()
                                               as i32 as uint8_t as
                                               libc::c_uint).wrapping_mul(4 as
                                                                              libc::c_uint))
                                    |
                                    Alternate <<
                                        (__RBIT(Pin >>
                                                    8 as
                                                        libc::c_uint).leading_zeros()
                                             as i32 as uint8_t as
                                             libc::c_uint).wrapping_mul(4 as
                                                                            libc::c_uint));
}
/* *
  * @brief  Force AHB1 peripherals reset.
  * @rmtoll AHB1RSTR     GPIOARST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOBRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOCRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIODRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOERST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOFRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOGRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOHRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOIRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOJRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOKRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     CRCRST        LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     DMA1RST       LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     DMA2RST       LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     DMA2DRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     ETHMACRST     LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     OTGHSRST      LL_AHB1_GRP1_ForceReset
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOE
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOF
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOG
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOI
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOJ (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOK (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA2D (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETHMAC (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_OTGHS
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
#[inline]
unsafe extern "C" fn LL_AHB1_GRP1_ForceReset(mut Periphs: uint32_t) {
    let ref mut fresh0 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1RSTR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | Periphs) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Release AHB1 peripherals reset.
  * @rmtoll AHB1RSTR     GPIOARST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOBRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOCRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIODRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOERST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOFRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOGRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOHRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOIRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOJRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOKRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     CRCRST        LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     DMA1RST       LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     DMA2RST       LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     DMA2DRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     ETHMACRST     LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     OTGHSRST      LL_AHB1_GRP1_ReleaseReset
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOE
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOF
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOG
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOI
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOJ (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOK (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA2D (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETHMAC (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_OTGHS
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
#[inline]
unsafe extern "C" fn LL_AHB1_GRP1_ReleaseReset(mut Periphs: uint32_t) {
    let ref mut fresh1 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1RSTR;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !Periphs) as uint32_t
                                    as uint32_t);
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_ll_gpio.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   GPIO LL module driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_LL_Driver
  * @{
  */
/* * @addtogroup GPIO_LL
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* * @addtogroup GPIO_LL_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup GPIO_LL_Exported_Functions
  * @{
  */
/* * @addtogroup GPIO_LL_EF_Init
  * @{
  */
/* *
  * @brief  De-initialize GPIO registers (Registers restored to their default values).
  * @param  GPIOx GPIO Port
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: GPIO registers are de-initialized
  *          - ERROR:   Wrong GPIO Port
  */
#[no_mangle]
pub unsafe extern "C" fn LL_GPIO_DeInit(mut GPIOx: *mut GPIO_TypeDef)
 -> ErrorStatus {
    let mut status: ErrorStatus = SUCCESS;
    /* Check the parameters */
    /* Force and Release reset on clock of GPIOx Port */
    if GPIOx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut GPIO_TypeDef {
        LL_AHB1_GRP1_ForceReset((0x1 as libc::c_uint) << 0 as libc::c_uint);
        LL_AHB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 0 as libc::c_uint);
    } else if GPIOx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x400
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut GPIO_TypeDef {
        LL_AHB1_GRP1_ForceReset((0x1 as libc::c_uint) << 1 as libc::c_uint);
        LL_AHB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 1 as libc::c_uint);
    } else if GPIOx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x800
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut GPIO_TypeDef {
        LL_AHB1_GRP1_ForceReset((0x1 as libc::c_uint) << 2 as libc::c_uint);
        LL_AHB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 2 as libc::c_uint);
    } else if GPIOx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0xc00
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut GPIO_TypeDef {
        LL_AHB1_GRP1_ForceReset((0x1 as libc::c_uint) << 3 as libc::c_uint);
        LL_AHB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 3 as libc::c_uint);
    } else if GPIOx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x1000
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut GPIO_TypeDef {
        LL_AHB1_GRP1_ForceReset((0x1 as libc::c_uint) << 4 as libc::c_uint);
        LL_AHB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 4 as libc::c_uint);
    } else if GPIOx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x1400
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut GPIO_TypeDef {
        LL_AHB1_GRP1_ForceReset((0x1 as libc::c_uint) << 5 as libc::c_uint);
        LL_AHB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 5 as libc::c_uint);
    } else if GPIOx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x1800
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut GPIO_TypeDef {
        LL_AHB1_GRP1_ForceReset((0x1 as libc::c_uint) << 6 as libc::c_uint);
        LL_AHB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 6 as libc::c_uint);
    } else if GPIOx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x1c00
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut GPIO_TypeDef {
        LL_AHB1_GRP1_ForceReset((0x1 as libc::c_uint) << 7 as libc::c_uint);
        LL_AHB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 7 as libc::c_uint);
    } else if GPIOx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x2000
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut GPIO_TypeDef {
        LL_AHB1_GRP1_ForceReset((0x1 as libc::c_uint) << 8 as libc::c_uint);
        LL_AHB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 8 as libc::c_uint);
    } else if GPIOx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x2400
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut GPIO_TypeDef {
        LL_AHB1_GRP1_ForceReset((0x1 as libc::c_uint) << 9 as libc::c_uint);
        LL_AHB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 9 as libc::c_uint);
    } else if GPIOx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x2800
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut GPIO_TypeDef {
        LL_AHB1_GRP1_ForceReset((0x1 as libc::c_uint) << 10 as libc::c_uint);
        LL_AHB1_GRP1_ReleaseReset((0x1 as libc::c_uint) <<
                                      10 as libc::c_uint);
    } else {
        /* GPIOD */
        /* GPIOE */
        /* GPIOF */
        /* GPIOG */
        /* GPIOH */
        /* GPIOI */
        /* GPIOJ */
        /* GPIOK */
        status = ERROR
    }
    return status;
}
/* *
  * @brief  Initialize GPIO registers according to the specified parameters in GPIO_InitStruct.
  * @param  GPIOx GPIO Port
  * @param  GPIO_InitStruct: pointer to a @ref LL_GPIO_InitTypeDef structure
  *         that contains the configuration information for the specified GPIO peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: GPIO registers are initialized according to GPIO_InitStruct content
  *          - ERROR:   Not applicable
  */
#[no_mangle]
pub unsafe extern "C" fn LL_GPIO_Init(mut GPIOx: *mut GPIO_TypeDef,
                                      mut GPIO_InitStruct:
                                          *mut LL_GPIO_InitTypeDef)
 -> ErrorStatus {
    let mut pinpos: uint32_t = 0 as libc::c_uint;
    let mut currentpin: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* ------------------------- Configure the port pins ---------------- */
  /* Initialize  pinpos on first pin set */
    pinpos =
        __RBIT((*GPIO_InitStruct).Pin).leading_zeros() as i32 as uint8_t as
            uint32_t;
    /* Configure the port pins */
    while (*GPIO_InitStruct).Pin >> pinpos != 0 as libc::c_uint {
        /* Get current io position */
        currentpin = (*GPIO_InitStruct).Pin & (0x1 as libc::c_uint) << pinpos;
        if currentpin != 0 {
            /* Pin Mode configuration */
            LL_GPIO_SetPinMode(GPIOx, currentpin, (*GPIO_InitStruct).Mode);
            if (*GPIO_InitStruct).Mode ==
                   (0x1 as libc::c_uint) << 0 as libc::c_uint ||
                   (*GPIO_InitStruct).Mode ==
                       (0x2 as libc::c_uint) << 0 as libc::c_uint {
                /* Check Speed mode parameters */
                /* Speed mode configuration */
                LL_GPIO_SetPinSpeed(GPIOx, currentpin,
                                    (*GPIO_InitStruct).Speed);
            }
            /* Pull-up Pull down resistor configuration*/
            LL_GPIO_SetPinPull(GPIOx, currentpin, (*GPIO_InitStruct).Pull);
            if (*GPIO_InitStruct).Mode ==
                   (0x2 as libc::c_uint) << 0 as libc::c_uint {
                /* Check Alternate parameter */
                /* Speed mode configuration */
                if (__RBIT(currentpin).leading_zeros() as i32 as uint8_t as
                        libc::c_uint) < 0x8 as libc::c_uint {
                    LL_GPIO_SetAFPin_0_7(GPIOx, currentpin,
                                         (*GPIO_InitStruct).Alternate);
                } else {
                    LL_GPIO_SetAFPin_8_15(GPIOx, currentpin,
                                          (*GPIO_InitStruct).Alternate);
                }
            }
        }
        pinpos = pinpos.wrapping_add(1)
    }
    if (*GPIO_InitStruct).Mode == (0x1 as libc::c_uint) << 0 as libc::c_uint
           ||
           (*GPIO_InitStruct).Mode ==
               (0x2 as libc::c_uint) << 0 as libc::c_uint {
        /* Check Output mode parameters */
        /* Output mode configuration*/
        LL_GPIO_SetPinOutputType(GPIOx, (*GPIO_InitStruct).Pin,
                                 (*GPIO_InitStruct).OutputType);
    }
    return SUCCESS;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_ll_gpio.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of GPIO LL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_LL_Driver
  * @{
  */
/* * @defgroup GPIO_LL GPIO
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* * @defgroup GPIO_LL_Private_Macros GPIO Private Macros
  * @{
  */
/* *
  * @}
  */
/*USE_FULL_LL_DRIVER*/
/* Exported types ------------------------------------------------------------*/
/* * @defgroup GPIO_LL_ES_INIT GPIO Exported Init structures
  * @{
  */
/* *
  * @brief LL GPIO Init Structure definition
  */
/* !< Specifies the GPIO pins to be configured.
                              This parameter can be any value of @ref GPIO_LL_EC_PIN */
/* !< Specifies the operating mode for the selected pins.
                              This parameter can be a value of @ref GPIO_LL_EC_MODE.

                              GPIO HW configuration can be modified afterwards using unitary function @ref LL_GPIO_SetPinMode().*/
/* !< Specifies the speed for the selected pins.
                              This parameter can be a value of @ref GPIO_LL_EC_SPEED.

                              GPIO HW configuration can be modified afterwards using unitary function @ref LL_GPIO_SetPinSpeed().*/
/* !< Specifies the operating output type for the selected pins.
                              This parameter can be a value of @ref GPIO_LL_EC_OUTPUT.

                              GPIO HW configuration can be modified afterwards using unitary function @ref LL_GPIO_SetPinOutputType().*/
/* !< Specifies the operating Pull-up/Pull down for the selected pins.
                              This parameter can be a value of @ref GPIO_LL_EC_PULL.

                              GPIO HW configuration can be modified afterwards using unitary function @ref LL_GPIO_SetPinPull().*/
/* !< Specifies the Peripheral to be connected to the selected pins.
                              This parameter can be a value of @ref GPIO_LL_EC_AF.

                              GPIO HW configuration can be modified afterwards using unitary function @ref LL_GPIO_SetAFPin_0_7() and LL_GPIO_SetAFPin_8_15().*/
/* *
  * @}
  */
/* USE_FULL_LL_DRIVER */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup GPIO_LL_Exported_Constants GPIO Exported Constants
  * @{
  */
/* * @defgroup GPIO_LL_EC_PIN PIN
  * @{
  */
/* !< Select pin 0 */
/* !< Select pin 1 */
/* !< Select pin 2 */
/* !< Select pin 3 */
/* !< Select pin 4 */
/* !< Select pin 5 */
/* !< Select pin 6 */
/* !< Select pin 7 */
/* !< Select pin 8 */
/* !< Select pin 9 */
/* !< Select pin 10 */
/* !< Select pin 11 */
/* !< Select pin 12 */
/* !< Select pin 13 */
/* !< Select pin 14 */
/* !< Select pin 15 */
/* !< Select all pins */
/* *
  * @}
  */
/* * @defgroup GPIO_LL_EC_MODE Mode
  * @{
  */
/* !< Select input mode */
/* !< Select output mode */
/* !< Select alternate function mode */
/* !< Select analog mode */
/* *
  * @}
  */
/* * @defgroup GPIO_LL_EC_OUTPUT Output Type
  * @{
  */
/* !< Select push-pull as output type */
/* !< Select open-drain as output type */
/* *
  * @}
  */
/* * @defgroup GPIO_LL_EC_SPEED Output Speed
  * @{
  */
/* !< Select I/O low output speed    */
/* !< Select I/O medium output speed */
/* !< Select I/O fast output speed   */
/* !< Select I/O high output speed   */
/* *
  * @}
  */
/* * @defgroup GPIO_LL_EC_PULL Pull Up Pull Down
  * @{
  */
/* !< Select I/O no pull */
/* !< Select I/O pull up */
/* !< Select I/O pull down */
/* *
  * @}
  */
/* * @defgroup GPIO_LL_EC_AF Alternate Function
  * @{
  */
/* !< Select alternate function 0 */
/* !< Select alternate function 1 */
/* !< Select alternate function 2 */
/* !< Select alternate function 3 */
/* !< Select alternate function 4 */
/* !< Select alternate function 5 */
/* !< Select alternate function 6 */
/* !< Select alternate function 7 */
/* !< Select alternate function 8 */
/* !< Select alternate function 9 */
/* !< Select alternate function 10 */
/* !< Select alternate function 11 */
/* !< Select alternate function 12 */
/* !< Select alternate function 13 */
/* !< Select alternate function 14 */
/* !< Select alternate function 15 */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup GPIO_LL_Exported_Macros GPIO Exported Macros
  * @{
  */
/* * @defgroup GPIO_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */
/* *
  * @brief  Write a value in GPIO register
  * @param  __INSTANCE__ GPIO Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
/* *
  * @brief  Read a value in GPIO register
  * @param  __INSTANCE__ GPIO Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup GPIO_LL_Exported_Functions GPIO Exported Functions
  * @{
  */
/* * @defgroup GPIO_LL_EF_Port_Configuration Port Configuration
  * @{
  */
/* *
  * @brief  Configure gpio mode for a dedicated pin on dedicated port.
  * @note   I/O mode can be Input mode, General purpose output, Alternate function mode or Analog.
  * @note   Warning: only one pin can be passed as parameter.
  * @rmtoll MODER        MODEy         LL_GPIO_SetPinMode
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @param  Mode This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_MODE_INPUT
  *         @arg @ref LL_GPIO_MODE_OUTPUT
  *         @arg @ref LL_GPIO_MODE_ALTERNATE
  *         @arg @ref LL_GPIO_MODE_ANALOG
  * @retval None
  */
/* *
  * @brief  Return gpio mode for a dedicated pin on dedicated port.
  * @note   I/O mode can be Input mode, General purpose output, Alternate function mode or Analog.
  * @note   Warning: only one pin can be passed as parameter.
  * @rmtoll MODER        MODEy         LL_GPIO_GetPinMode
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_GPIO_MODE_INPUT
  *         @arg @ref LL_GPIO_MODE_OUTPUT
  *         @arg @ref LL_GPIO_MODE_ALTERNATE
  *         @arg @ref LL_GPIO_MODE_ANALOG
  */
/* *
  * @brief  Configure gpio output type for several pins on dedicated port.
  * @note   Output type as to be set when gpio pin is in output or
  *         alternate modes. Possible type are Push-pull or Open-drain.
  * @rmtoll OTYPER       OTy           LL_GPIO_SetPinOutputType
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @param  OutputType This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_OUTPUT_PUSHPULL
  *         @arg @ref LL_GPIO_OUTPUT_OPENDRAIN
  * @retval None
  */
/* *
  * @brief  Return gpio output type for several pins on dedicated port.
  * @note   Output type as to be set when gpio pin is in output or
  *         alternate modes. Possible type are Push-pull or Open-drain.
  * @note   Warning: only one pin can be passed as parameter.
  * @rmtoll OTYPER       OTy           LL_GPIO_GetPinOutputType
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_GPIO_OUTPUT_PUSHPULL
  *         @arg @ref LL_GPIO_OUTPUT_OPENDRAIN
  */
/* *
  * @brief  Configure gpio speed for a dedicated pin on dedicated port.
  * @note   I/O speed can be Low, Medium, Fast or High speed.
  * @note   Warning: only one pin can be passed as parameter.
  * @note   Refer to datasheet for frequency specifications and the power
  *         supply and load conditions for each speed.
  * @rmtoll OSPEEDR      OSPEEDy       LL_GPIO_SetPinSpeed
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @param  Speed This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_SPEED_FREQ_LOW
  *         @arg @ref LL_GPIO_SPEED_FREQ_MEDIUM
  *         @arg @ref LL_GPIO_SPEED_FREQ_HIGH
  *         @arg @ref LL_GPIO_SPEED_FREQ_VERY_HIGH
  * @retval None
  */
/* *
  * @brief  Return gpio speed for a dedicated pin on dedicated port.
  * @note   I/O speed can be Low, Medium, Fast or High speed.
  * @note   Warning: only one pin can be passed as parameter.
  * @note   Refer to datasheet for frequency specifications and the power
  *         supply and load conditions for each speed.
  * @rmtoll OSPEEDR      OSPEEDy       LL_GPIO_GetPinSpeed
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_GPIO_SPEED_FREQ_LOW
  *         @arg @ref LL_GPIO_SPEED_FREQ_MEDIUM
  *         @arg @ref LL_GPIO_SPEED_FREQ_HIGH
  *         @arg @ref LL_GPIO_SPEED_FREQ_VERY_HIGH
  */
/* *
  * @brief  Configure gpio pull-up or pull-down for a dedicated pin on a dedicated port.
  * @note   Warning: only one pin can be passed as parameter.
  * @rmtoll PUPDR        PUPDy         LL_GPIO_SetPinPull
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @param  Pull This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PULL_NO
  *         @arg @ref LL_GPIO_PULL_UP
  *         @arg @ref LL_GPIO_PULL_DOWN
  * @retval None
  */
/* *
  * @brief  Return gpio pull-up or pull-down for a dedicated pin on a dedicated port
  * @note   Warning: only one pin can be passed as parameter.
  * @rmtoll PUPDR        PUPDy         LL_GPIO_GetPinPull
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_GPIO_PULL_NO
  *         @arg @ref LL_GPIO_PULL_UP
  *         @arg @ref LL_GPIO_PULL_DOWN
  */
/* *
  * @brief  Configure gpio alternate function of a dedicated pin from 0 to 7 for a dedicated port.
  * @note   Possible values are from AF0 to AF15 depending on target.
  * @note   Warning: only one pin can be passed as parameter.
  * @rmtoll AFRL         AFSELy        LL_GPIO_SetAFPin_0_7
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  * @param  Alternate This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_AF_0
  *         @arg @ref LL_GPIO_AF_1
  *         @arg @ref LL_GPIO_AF_2
  *         @arg @ref LL_GPIO_AF_3
  *         @arg @ref LL_GPIO_AF_4
  *         @arg @ref LL_GPIO_AF_5
  *         @arg @ref LL_GPIO_AF_6
  *         @arg @ref LL_GPIO_AF_7
  *         @arg @ref LL_GPIO_AF_8
  *         @arg @ref LL_GPIO_AF_9
  *         @arg @ref LL_GPIO_AF_10
  *         @arg @ref LL_GPIO_AF_11
  *         @arg @ref LL_GPIO_AF_12
  *         @arg @ref LL_GPIO_AF_13
  *         @arg @ref LL_GPIO_AF_14
  *         @arg @ref LL_GPIO_AF_15
  * @retval None
  */
/* *
  * @brief  Return gpio alternate function of a dedicated pin from 0 to 7 for a dedicated port.
  * @rmtoll AFRL         AFSELy        LL_GPIO_GetAFPin_0_7
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_GPIO_AF_0
  *         @arg @ref LL_GPIO_AF_1
  *         @arg @ref LL_GPIO_AF_2
  *         @arg @ref LL_GPIO_AF_3
  *         @arg @ref LL_GPIO_AF_4
  *         @arg @ref LL_GPIO_AF_5
  *         @arg @ref LL_GPIO_AF_6
  *         @arg @ref LL_GPIO_AF_7
  *         @arg @ref LL_GPIO_AF_8
  *         @arg @ref LL_GPIO_AF_9
  *         @arg @ref LL_GPIO_AF_10
  *         @arg @ref LL_GPIO_AF_11
  *         @arg @ref LL_GPIO_AF_12
  *         @arg @ref LL_GPIO_AF_13
  *         @arg @ref LL_GPIO_AF_14
  *         @arg @ref LL_GPIO_AF_15
  */
/* *
  * @brief  Configure gpio alternate function of a dedicated pin from 8 to 15 for a dedicated port.
  * @note   Possible values are from AF0 to AF15 depending on target.
  * @note   Warning: only one pin can be passed as parameter.
  * @rmtoll AFRH         AFSELy        LL_GPIO_SetAFPin_8_15
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @param  Alternate This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_AF_0
  *         @arg @ref LL_GPIO_AF_1
  *         @arg @ref LL_GPIO_AF_2
  *         @arg @ref LL_GPIO_AF_3
  *         @arg @ref LL_GPIO_AF_4
  *         @arg @ref LL_GPIO_AF_5
  *         @arg @ref LL_GPIO_AF_6
  *         @arg @ref LL_GPIO_AF_7
  *         @arg @ref LL_GPIO_AF_8
  *         @arg @ref LL_GPIO_AF_9
  *         @arg @ref LL_GPIO_AF_10
  *         @arg @ref LL_GPIO_AF_11
  *         @arg @ref LL_GPIO_AF_12
  *         @arg @ref LL_GPIO_AF_13
  *         @arg @ref LL_GPIO_AF_14
  *         @arg @ref LL_GPIO_AF_15
  * @retval None
  */
/* *
  * @brief  Return gpio alternate function of a dedicated pin from 8 to 15 for a dedicated port.
  * @note   Possible values are from AF0 to AF15 depending on target.
  * @rmtoll AFRH         AFSELy        LL_GPIO_GetAFPin_8_15
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_GPIO_AF_0
  *         @arg @ref LL_GPIO_AF_1
  *         @arg @ref LL_GPIO_AF_2
  *         @arg @ref LL_GPIO_AF_3
  *         @arg @ref LL_GPIO_AF_4
  *         @arg @ref LL_GPIO_AF_5
  *         @arg @ref LL_GPIO_AF_6
  *         @arg @ref LL_GPIO_AF_7
  *         @arg @ref LL_GPIO_AF_8
  *         @arg @ref LL_GPIO_AF_9
  *         @arg @ref LL_GPIO_AF_10
  *         @arg @ref LL_GPIO_AF_11
  *         @arg @ref LL_GPIO_AF_12
  *         @arg @ref LL_GPIO_AF_13
  *         @arg @ref LL_GPIO_AF_14
  *         @arg @ref LL_GPIO_AF_15
  */
/* *
  * @brief  Lock configuration of several pins for a dedicated port.
  * @note   When the lock sequence has been applied on a port bit, the
  *         value of this port bit can no longer be modified until the
  *         next reset.
  * @note   Each lock bit freezes a specific configuration register
  *         (control and alternate function registers).
  * @rmtoll LCKR         LCKK          LL_GPIO_LockPin
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
/* *
  * @brief  Return 1 if all pins passed as parameter, of a dedicated port, are locked. else Return 0.
  * @rmtoll LCKR         LCKy          LL_GPIO_IsPinLocked
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Return 1 if one of the pin of a dedicated port is locked. else return 0.
  * @rmtoll LCKR         LCKK          LL_GPIO_IsAnyPinLocked
  * @param  GPIOx GPIO Port
  * @retval State of bit (1 or 0).
  */
/* *
  * @}
  */
/* * @defgroup GPIO_LL_EF_Data_Access Data Access
  * @{
  */
/* *
  * @brief  Return full input data register value for a dedicated port.
  * @rmtoll IDR          IDy           LL_GPIO_ReadInputPort
  * @param  GPIOx GPIO Port
  * @retval Input data register value of port
  */
/* *
  * @brief  Return if input data level for several pins of dedicated port is high or low.
  * @rmtoll IDR          IDy           LL_GPIO_IsInputPinSet
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Write output data register for the port.
  * @rmtoll ODR          ODy           LL_GPIO_WriteOutputPort
  * @param  GPIOx GPIO Port
  * @param  PortValue Level value for each pin of the port
  * @retval None
  */
/* *
  * @brief  Return full output data register value for a dedicated port.
  * @rmtoll ODR          ODy           LL_GPIO_ReadOutputPort
  * @param  GPIOx GPIO Port
  * @retval Output data register value of port
  */
/* *
  * @brief  Return if input data level for several pins of dedicated port is high or low.
  * @rmtoll ODR          ODy           LL_GPIO_IsOutputPinSet
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Set several pins to high level on dedicated gpio port.
  * @rmtoll BSRR         BSy           LL_GPIO_SetOutputPin
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
/* *
  * @brief  Set several pins to low level on dedicated gpio port.
  * @rmtoll BSRR         BRy           LL_GPIO_ResetOutputPin
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
/* *
  * @brief  Toggle data value for several pin of dedicated port.
  * @rmtoll ODR          ODy           LL_GPIO_TogglePin
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
/* *
  * @}
  */
/* * @defgroup GPIO_LL_EF_Init Initialization and de-initialization functions
  * @{
  */
/* *
  * @brief Set each @ref LL_GPIO_InitTypeDef field to default value.
  * @param GPIO_InitStruct: pointer to a @ref LL_GPIO_InitTypeDef structure
  *                          whose fields will be set to default values.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_GPIO_StructInit(mut GPIO_InitStruct:
                                                *mut LL_GPIO_InitTypeDef) {
    /* Reset GPIO init structure parameters values */
    (*GPIO_InitStruct).Pin =
        0x1 as libc::c_uint | 0x2 as libc::c_uint | 0x4 as libc::c_uint |
            0x8 as libc::c_uint | 0x10 as libc::c_uint | 0x20 as libc::c_uint
            | 0x40 as libc::c_uint | 0x80 as libc::c_uint |
            0x100 as libc::c_uint | 0x200 as libc::c_uint |
            0x400 as libc::c_uint | 0x800 as libc::c_uint |
            0x1000 as libc::c_uint | 0x2000 as libc::c_uint |
            0x4000 as libc::c_uint | 0x8000 as libc::c_uint;
    (*GPIO_InitStruct).Mode = (0x3 as libc::c_uint) << 0 as libc::c_uint;
    (*GPIO_InitStruct).Speed = 0 as libc::c_uint;
    (*GPIO_InitStruct).OutputType = 0 as libc::c_uint;
    (*GPIO_InitStruct).Pull = 0 as libc::c_uint;
    (*GPIO_InitStruct).Alternate = 0 as libc::c_uint;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* USE_FULL_LL_DRIVER */
/* *
  * @}
  */
/* defined (GPIOA) || defined (GPIOB) || defined (GPIOC) || defined (GPIOD) || defined (GPIOE) || defined (GPIOF) || defined (GPIOG) || defined (GPIOH) || defined (GPIOI) || defined (GPIOJ) || defined (GPIOK) */
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
