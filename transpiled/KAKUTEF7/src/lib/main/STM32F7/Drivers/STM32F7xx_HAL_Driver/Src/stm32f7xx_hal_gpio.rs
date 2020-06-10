use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  * @brief External Interrupt/Event Controller
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EXTI_TypeDef {
    pub IMR: uint32_t,
    pub EMR: uint32_t,
    pub RTSR: uint32_t,
    pub FTSR: uint32_t,
    pub SWIER: uint32_t,
    pub PR: uint32_t,
}
/* *
  * @brief General Purpose I/O
  */
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
/* *
  * @brief System configuration controller
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SYSCFG_TypeDef {
    pub MEMRMP: uint32_t,
    pub PMC: uint32_t,
    pub EXTICR: [uint32_t; 4],
    pub RESERVED: [uint32_t; 2],
    pub CMPCR: uint32_t,
}
/* *
  * @brief Reset and Clock Control
  */
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
/* *
  ******************************************************************************
  * @file    stm32f7xx.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    30-December-2016
  * @brief   CMSIS STM32F7xx Device Peripheral Access Layer Header File.           
  *            
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Configuration section that allows to select:
  *              - The STM32F7xx device used in the target application
  *              - To use or not the peripheral�s drivers in application code(i.e. 
  *                code will be based on direct access to peripheral�s registers 
  *                rather than drivers API), this option is controlled by 
  *                "#define USE_HAL_DRIVER"
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
/* * @addtogroup CMSIS
  * @{
  */
/* * @addtogroup stm32f7xx
  * @{
  */
/* __cplusplus */
/* * @addtogroup Library_configuration_section
  * @{
  */
/* *
  * @brief STM32 Family
  */
/* STM32F7 */
/* Uncomment the line below according to the target STM32 device used in your
   application 
  */
/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.
  */
/* USE_HAL_DRIVER */
/* *
  * @brief CMSIS Device version number V1.2.0
  */
/* !< [31:24] main version */
/* !< [23:16] sub1 version */
/* !< [15:8]  sub2 version */
/* !< [7:0]  release candidate */
/* *
  * @}
  */
/* * @addtogroup Device_Included
  * @{
  */
/* *
  * @}
  */
/* * @addtogroup Exported_types
  * @{
  */
pub type C2RustUnnamed = libc::c_uint;
pub const SET: C2RustUnnamed = 1;
pub const RESET: C2RustUnnamed = 0;
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_def.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   This file contains HAL common defines, enumeration, macros and 
  *          structures definitions. 
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
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  HAL Status structures definition  
  */
pub type HAL_StatusTypeDef = libc::c_uint;
pub const HAL_TIMEOUT: HAL_StatusTypeDef = 3;
pub const HAL_BUSY: HAL_StatusTypeDef = 2;
pub const HAL_ERROR: HAL_StatusTypeDef = 1;
pub const HAL_OK: HAL_StatusTypeDef = 0;
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_gpio.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of GPIO HAL module.
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
/* * @addtogroup STM32F7xx_HAL_Driver
  * @{
  */
/* * @addtogroup GPIO
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup GPIO_Exported_Types GPIO Exported Types
  * @{
  */
/* * 
  * @brief GPIO Init structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_InitTypeDef {
    pub Pin: uint32_t,
    pub Mode: uint32_t,
    pub Pull: uint32_t,
    pub Speed: uint32_t,
    pub Alternate: uint32_t,
}
/* * 
  * @brief  GPIO Bit SET and Bit RESET enumeration 
  */
pub type GPIO_PinState = libc::c_uint;
pub const GPIO_PIN_SET: GPIO_PinState = 1;
pub const GPIO_PIN_RESET: GPIO_PinState = 0;
/* *
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* * @defgroup GPIO_Exported_Functions GPIO Exported Functions
  * @{
  */
/* * @defgroup GPIO_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
  [..]
    This section provides functions allowing to initialize and de-initialize the GPIOs
    to be ready for use.
 
@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the GPIOx peripheral according to the specified parameters in the GPIO_Init.
  * @param  GPIOx: where x can be (A..K) to select the GPIO peripheral.
  * @param  GPIO_Init: pointer to a GPIO_InitTypeDef structure that contains
  *         the configuration information for the specified GPIO peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_GPIO_Init(mut GPIOx: *mut GPIO_TypeDef,
                                       mut GPIO_Init: *mut GPIO_InitTypeDef) {
    let mut position: uint32_t = 0 as libc::c_int as uint32_t;
    let mut ioposition: uint32_t = 0 as libc::c_int as uint32_t;
    let mut iocurrent: uint32_t = 0 as libc::c_int as uint32_t;
    let mut temp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Configure the port pins */
    position = 0 as libc::c_int as uint32_t;
    while position < 16 as libc::c_uint {
        /* Get the IO position */
        ioposition = (0x1 as libc::c_int as uint32_t) << position;
        /* Get the current IO position */
        iocurrent = (*GPIO_Init).Pin & ioposition;
        if iocurrent == ioposition {
            /*--------------------- GPIO Mode Configuration ------------------------*/
      /* In case of Alternate function mode selection */
            if (*GPIO_Init).Mode == 0x2 as libc::c_uint ||
                   (*GPIO_Init).Mode == 0x12 as libc::c_uint {
                /* Check the Alternate function parameter */
                /* Configure Alternate function mapped with the current IO */
                temp = (*GPIOx).AFR[(position >> 3 as libc::c_int) as usize];
                temp &=
                    !((0xf as libc::c_int as uint32_t) <<
                          (position &
                               0x7 as libc::c_int as
                                   uint32_t).wrapping_mul(4 as libc::c_int as
                                                              libc::c_uint));
                temp |=
                    (*GPIO_Init).Alternate <<
                        (position &
                             0x7 as libc::c_int as
                                 uint32_t).wrapping_mul(4 as libc::c_int as
                                                            libc::c_uint);
                ::core::ptr::write_volatile(&mut (*GPIOx).AFR[(position >>
                                                                   3 as
                                                                       libc::c_int)
                                                                  as usize] as
                                                *mut uint32_t, temp)
            }
            /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
            temp = (*GPIOx).MODER;
            temp &=
                !(((0x3 as libc::c_uint) << 0 as libc::c_uint) <<
                      position.wrapping_mul(2 as libc::c_int as
                                                libc::c_uint));
            temp |=
                ((*GPIO_Init).Mode & 0x3 as libc::c_uint) <<
                    position.wrapping_mul(2 as libc::c_int as libc::c_uint);
            ::core::ptr::write_volatile(&mut (*GPIOx).MODER as *mut uint32_t,
                                        temp);
            /* In case of Output or Alternate function mode selection */
            if (*GPIO_Init).Mode == 0x1 as libc::c_uint ||
                   (*GPIO_Init).Mode == 0x2 as libc::c_uint ||
                   (*GPIO_Init).Mode == 0x11 as libc::c_uint ||
                   (*GPIO_Init).Mode == 0x12 as libc::c_uint {
                /* Check the Speed parameter */
                /* Configure the IO Speed */
                temp = (*GPIOx).OSPEEDR;
                temp &=
                    !(((0x3 as libc::c_uint) << 0 as libc::c_uint) <<
                          position.wrapping_mul(2 as libc::c_int as
                                                    libc::c_uint));
                temp |=
                    (*GPIO_Init).Speed <<
                        position.wrapping_mul(2 as libc::c_int as
                                                  libc::c_uint);
                ::core::ptr::write_volatile(&mut (*GPIOx).OSPEEDR as
                                                *mut uint32_t, temp);
                temp = (*GPIOx).OTYPER;
                temp &= !((0x1 as libc::c_uint) << position);
                temp |=
                    (((*GPIO_Init).Mode & 0x10 as libc::c_uint) >>
                         4 as libc::c_int) << position;
                ::core::ptr::write_volatile(&mut (*GPIOx).OTYPER as
                                                *mut uint32_t, temp)
            }
            /* Configure the IO Output Type */
            /* Activate the Pull-up or Pull down resistor for the current IO */
            temp = (*GPIOx).PUPDR;
            temp &=
                !(((0x3 as libc::c_uint) << 0 as libc::c_uint) <<
                      position.wrapping_mul(2 as libc::c_int as
                                                libc::c_uint));
            temp |=
                (*GPIO_Init).Pull <<
                    position.wrapping_mul(2 as libc::c_int as libc::c_uint);
            ::core::ptr::write_volatile(&mut (*GPIOx).PUPDR as *mut uint32_t,
                                        temp);
            /*--------------------- EXTI Mode Configuration ------------------------*/
      /* Configure the External Interrupt or event for the current IO */
            if (*GPIO_Init).Mode & 0x10000000 as libc::c_uint ==
                   0x10000000 as libc::c_uint {
                /* Enable SYSCFG Clock */
                let mut tmpreg: uint32_t = 0;
                let ref mut fresh0 =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x20000 as
                                                           libc::c_uint).wrapping_add(0x3800
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut RCC_TypeDef)).APB2ENR;
                ::core::ptr::write_volatile(fresh0,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     14 as libc::c_uint) as
                                                uint32_t as uint32_t);
                ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                            (*((0x40000000 as
                                                    libc::c_uint).wrapping_add(0x20000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x3800
                                                                                                                  as
                                                                                                                  libc::c_uint)
                                                   as
                                                   *mut RCC_TypeDef)).APB2ENR
                                                &
                                                (0x1 as libc::c_uint) <<
                                                    14 as libc::c_uint);
                temp =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x10000 as
                                                           libc::c_uint).wrapping_add(0x3800
                                                                                          as
                                                                                          libc::c_uint)
                           as
                           *mut SYSCFG_TypeDef)).EXTICR[(position >>
                                                             2 as libc::c_int)
                                                            as usize];
                temp &=
                    !((0xf as libc::c_int as uint32_t) <<
                          (4 as libc::c_int as
                               libc::c_uint).wrapping_mul(position &
                                                              0x3 as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_uint));
                temp |=
                    ((if GPIOx ==
                             (0x40000000 as
                                  libc::c_uint).wrapping_add(0x20000 as
                                                                 libc::c_uint).wrapping_add(0
                                                                                                as
                                                                                                libc::c_uint)
                                 as *mut GPIO_TypeDef {
                          0 as libc::c_uint
                      } else {
                          (if GPIOx ==
                                  (0x40000000 as
                                       libc::c_uint).wrapping_add(0x20000 as
                                                                      libc::c_uint).wrapping_add(0x400
                                                                                                     as
                                                                                                     libc::c_uint)
                                      as *mut GPIO_TypeDef {
                               1 as libc::c_uint
                           } else {
                               (if GPIOx ==
                                       (0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut GPIO_TypeDef {
                                    2 as libc::c_uint
                                } else {
                                    (if GPIOx ==
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0xc00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut GPIO_TypeDef {
                                         3 as libc::c_uint
                                     } else {
                                         (if GPIOx ==
                                                 (0x40000000 as
                                                      libc::c_uint).wrapping_add(0x20000
                                                                                     as
                                                                                     libc::c_uint).wrapping_add(0x1000
                                                                                                                    as
                                                                                                                    libc::c_uint)
                                                     as *mut GPIO_TypeDef {
                                              4 as libc::c_uint
                                          } else {
                                              (if GPIOx ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x1400
                                                                                                                         as
                                                                                                                         libc::c_uint)
                                                          as *mut GPIO_TypeDef
                                                  {
                                                   5 as libc::c_uint
                                               } else {
                                                   (if GPIOx ==
                                                           (0x40000000 as
                                                                libc::c_uint).wrapping_add(0x20000
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(0x1800
                                                                                                                              as
                                                                                                                              libc::c_uint)
                                                               as
                                                               *mut GPIO_TypeDef
                                                       {
                                                        6 as libc::c_uint
                                                    } else {
                                                        (if GPIOx ==
                                                                (0x40000000 as
                                                                     libc::c_uint).wrapping_add(0x20000
                                                                                                    as
                                                                                                    libc::c_uint).wrapping_add(0x1c00
                                                                                                                                   as
                                                                                                                                   libc::c_uint)
                                                                    as
                                                                    *mut GPIO_TypeDef
                                                            {
                                                             7 as libc::c_uint
                                                         } else {
                                                             (if GPIOx ==
                                                                     (0x40000000
                                                                          as
                                                                          libc::c_uint).wrapping_add(0x20000
                                                                                                         as
                                                                                                         libc::c_uint).wrapping_add(0x2000
                                                                                                                                        as
                                                                                                                                        libc::c_uint)
                                                                         as
                                                                         *mut GPIO_TypeDef
                                                                 {
                                                                  8 as
                                                                      libc::c_uint
                                                              } else {
                                                                  (if GPIOx ==
                                                                          (0x40000000
                                                                               as
                                                                               libc::c_uint).wrapping_add(0x20000
                                                                                                              as
                                                                                                              libc::c_uint).wrapping_add(0x2400
                                                                                                                                             as
                                                                                                                                             libc::c_uint)
                                                                              as
                                                                              *mut GPIO_TypeDef
                                                                      {
                                                                       9 as
                                                                           libc::c_uint
                                                                   } else {
                                                                       10 as
                                                                           libc::c_uint
                                                                   })
                                                              })
                                                         })
                                                    })
                                               })
                                          })
                                     })
                                })
                           })
                      }) as uint8_t as uint32_t) <<
                        (4 as libc::c_int as
                             libc::c_uint).wrapping_mul(position &
                                                            0x3 as libc::c_int
                                                                as
                                                                libc::c_uint);
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x10000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x3800
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut SYSCFG_TypeDef)).EXTICR[(position
                                                                                          >>
                                                                                          2
                                                                                              as
                                                                                              libc::c_int)
                                                                                         as
                                                                                         usize]
                                                as *mut uint32_t, temp);
                /* Clear EXTI line configuration */
                temp =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x10000 as
                                                           libc::c_uint).wrapping_add(0x3c00
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut EXTI_TypeDef)).IMR;
                temp &= !iocurrent;
                if (*GPIO_Init).Mode & 0x10000 as libc::c_uint ==
                       0x10000 as libc::c_uint {
                    temp |= iocurrent
                }
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x10000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x3c00
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut EXTI_TypeDef)).IMR
                                                as *mut uint32_t, temp);
                temp =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x10000 as
                                                           libc::c_uint).wrapping_add(0x3c00
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut EXTI_TypeDef)).EMR;
                temp &= !iocurrent;
                if (*GPIO_Init).Mode & 0x20000 as libc::c_uint ==
                       0x20000 as libc::c_uint {
                    temp |= iocurrent
                }
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x10000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x3c00
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut EXTI_TypeDef)).EMR
                                                as *mut uint32_t, temp);
                /* Clear Rising Falling edge configuration */
                temp =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x10000 as
                                                           libc::c_uint).wrapping_add(0x3c00
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut EXTI_TypeDef)).RTSR;
                temp &= !iocurrent;
                if (*GPIO_Init).Mode & 0x100000 as libc::c_uint ==
                       0x100000 as libc::c_uint {
                    temp |= iocurrent
                }
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x10000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x3c00
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut EXTI_TypeDef)).RTSR
                                                as *mut uint32_t, temp);
                temp =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x10000 as
                                                           libc::c_uint).wrapping_add(0x3c00
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut EXTI_TypeDef)).FTSR;
                temp &= !iocurrent;
                if (*GPIO_Init).Mode & 0x200000 as libc::c_uint ==
                       0x200000 as libc::c_uint {
                    temp |= iocurrent
                }
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x10000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x3c00
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut EXTI_TypeDef)).FTSR
                                                as *mut uint32_t, temp)
            }
        }
        position = position.wrapping_add(1)
    };
}
/* *
  * @brief  De-initializes the GPIOx peripheral registers to their default reset values.
  * @param  GPIOx: where x can be (A..K) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_GPIO_DeInit(mut GPIOx: *mut GPIO_TypeDef,
                                         mut GPIO_Pin: uint32_t) {
    let mut position: uint32_t = 0;
    let mut ioposition: uint32_t = 0 as libc::c_int as uint32_t;
    let mut iocurrent: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Configure the port pins */
    position = 0 as libc::c_int as uint32_t;
    while position < 16 as libc::c_uint {
        /* Get the IO position */
        ioposition = (0x1 as libc::c_int as uint32_t) << position;
        /* Get the current IO position */
        iocurrent = GPIO_Pin & ioposition;
        if iocurrent == ioposition {
            /*------------------------- GPIO Mode Configuration --------------------*/
      /* Configure IO Direction in Input Floating Mode */
            ::core::ptr::write_volatile(&mut (*GPIOx).MODER as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*GPIOx).MODER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(((0x3 as libc::c_uint) <<
                                                    0 as libc::c_uint) <<
                                                   position.wrapping_mul(2 as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)))
                                            as uint32_t as uint32_t);
            /* Configure the default Alternate Function in current IO */
            ::core::ptr::write_volatile(&mut (*GPIOx).AFR[(position >>
                                                               3 as
                                                                   libc::c_int)
                                                              as usize] as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*GPIOx).AFR[(position
                                                                                                   >>
                                                                                                   3
                                                                                                       as
                                                                                                       libc::c_int)
                                                                                                  as
                                                                                                  usize]
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0xf as libc::c_int as
                                                    uint32_t) <<
                                                   (position &
                                                        0x7 as libc::c_int as
                                                            uint32_t).wrapping_mul(4
                                                                                       as
                                                                                       libc::c_int
                                                                                       as
                                                                                       libc::c_uint)))
                                            as uint32_t as uint32_t);
            /* Configure the default value for IO Speed */
            ::core::ptr::write_volatile(&mut (*GPIOx).OSPEEDR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*GPIOx).OSPEEDR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(((0x3 as libc::c_uint) <<
                                                    0 as libc::c_uint) <<
                                                   position.wrapping_mul(2 as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)))
                                            as uint32_t as uint32_t);
            /* Configure the default value IO Output Type */
            ::core::ptr::write_volatile(&mut (*GPIOx).OTYPER as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*GPIOx).OTYPER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   position)) as uint32_t as
                                            uint32_t);
            /* Deactivate the Pull-up and Pull-down resistor for the current IO */
            ::core::ptr::write_volatile(&mut (*GPIOx).PUPDR as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*GPIOx).PUPDR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(((0x3 as libc::c_uint) <<
                                                    0 as libc::c_uint) <<
                                                   position.wrapping_mul(2 as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)))
                                            as uint32_t as uint32_t);
            /*------------------------- EXTI Mode Configuration --------------------*/
            tmp =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x10000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as
                       *mut SYSCFG_TypeDef)).EXTICR[(position >>
                                                         2 as libc::c_int) as
                                                        usize];
            tmp &=
                (0xf as libc::c_int as uint32_t) <<
                    (4 as libc::c_int as
                         libc::c_uint).wrapping_mul(position &
                                                        0x3 as libc::c_int as
                                                            libc::c_uint);
            if tmp ==
                   ((if GPIOx ==
                            (0x40000000 as
                                 libc::c_uint).wrapping_add(0x20000 as
                                                                libc::c_uint).wrapping_add(0
                                                                                               as
                                                                                               libc::c_uint)
                                as *mut GPIO_TypeDef {
                         0 as libc::c_uint
                     } else {
                         (if GPIOx ==
                                 (0x40000000 as
                                      libc::c_uint).wrapping_add(0x20000 as
                                                                     libc::c_uint).wrapping_add(0x400
                                                                                                    as
                                                                                                    libc::c_uint)
                                     as *mut GPIO_TypeDef {
                              1 as libc::c_uint
                          } else {
                              (if GPIOx ==
                                      (0x40000000 as
                                           libc::c_uint).wrapping_add(0x20000
                                                                          as
                                                                          libc::c_uint).wrapping_add(0x800
                                                                                                         as
                                                                                                         libc::c_uint)
                                          as *mut GPIO_TypeDef {
                                   2 as libc::c_uint
                               } else {
                                   (if GPIOx ==
                                           (0x40000000 as
                                                libc::c_uint).wrapping_add(0x20000
                                                                               as
                                                                               libc::c_uint).wrapping_add(0xc00
                                                                                                              as
                                                                                                              libc::c_uint)
                                               as *mut GPIO_TypeDef {
                                        3 as libc::c_uint
                                    } else {
                                        (if GPIOx ==
                                                (0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x1000
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as *mut GPIO_TypeDef {
                                             4 as libc::c_uint
                                         } else {
                                             (if GPIOx ==
                                                     (0x40000000 as
                                                          libc::c_uint).wrapping_add(0x20000
                                                                                         as
                                                                                         libc::c_uint).wrapping_add(0x1400
                                                                                                                        as
                                                                                                                        libc::c_uint)
                                                         as *mut GPIO_TypeDef
                                                 {
                                                  5 as libc::c_uint
                                              } else {
                                                  (if GPIOx ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x1800
                                                                                                                             as
                                                                                                                             libc::c_uint)
                                                              as
                                                              *mut GPIO_TypeDef
                                                      {
                                                       6 as libc::c_uint
                                                   } else {
                                                       (if GPIOx ==
                                                               (0x40000000 as
                                                                    libc::c_uint).wrapping_add(0x20000
                                                                                                   as
                                                                                                   libc::c_uint).wrapping_add(0x1c00
                                                                                                                                  as
                                                                                                                                  libc::c_uint)
                                                                   as
                                                                   *mut GPIO_TypeDef
                                                           {
                                                            7 as libc::c_uint
                                                        } else {
                                                            (if GPIOx ==
                                                                    (0x40000000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x20000
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0x2000
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                                                        as
                                                                        *mut GPIO_TypeDef
                                                                {
                                                                 8 as
                                                                     libc::c_uint
                                                             } else {
                                                                 (if GPIOx ==
                                                                         (0x40000000
                                                                              as
                                                                              libc::c_uint).wrapping_add(0x20000
                                                                                                             as
                                                                                                             libc::c_uint).wrapping_add(0x2400
                                                                                                                                            as
                                                                                                                                            libc::c_uint)
                                                                             as
                                                                             *mut GPIO_TypeDef
                                                                     {
                                                                      9 as
                                                                          libc::c_uint
                                                                  } else {
                                                                      10 as
                                                                          libc::c_uint
                                                                  })
                                                             })
                                                        })
                                                   })
                                              })
                                         })
                                    })
                               })
                          })
                     }) as uint8_t as uint32_t) <<
                       (4 as libc::c_int as
                            libc::c_uint).wrapping_mul(position &
                                                           0x3 as libc::c_int
                                                               as
                                                               libc::c_uint) {
                /* Configure the External Interrupt or event for the current IO */
                tmp =
                    (0xf as libc::c_int as uint32_t) <<
                        (4 as libc::c_int as
                             libc::c_uint).wrapping_mul(position &
                                                            0x3 as libc::c_int
                                                                as
                                                                libc::c_uint);
                let ref mut fresh1 =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x10000 as
                                                           libc::c_uint).wrapping_add(0x3800
                                                                                          as
                                                                                          libc::c_uint)
                           as
                           *mut SYSCFG_TypeDef)).EXTICR[(position >>
                                                             2 as libc::c_int)
                                                            as usize];
                ::core::ptr::write_volatile(fresh1,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint & !tmp) as
                                                uint32_t as uint32_t);
                /* Clear EXTI line configuration */
                let ref mut fresh2 =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x10000 as
                                                           libc::c_uint).wrapping_add(0x3c00
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut EXTI_TypeDef)).IMR;
                ::core::ptr::write_volatile(fresh2,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint & !iocurrent)
                                                as uint32_t as uint32_t);
                let ref mut fresh3 =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x10000 as
                                                           libc::c_uint).wrapping_add(0x3c00
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut EXTI_TypeDef)).EMR;
                ::core::ptr::write_volatile(fresh3,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint & !iocurrent)
                                                as uint32_t as uint32_t);
                /* Clear Rising Falling edge configuration */
                let ref mut fresh4 =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x10000 as
                                                           libc::c_uint).wrapping_add(0x3c00
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut EXTI_TypeDef)).RTSR;
                ::core::ptr::write_volatile(fresh4,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint & !iocurrent)
                                                as uint32_t as uint32_t);
                let ref mut fresh5 =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x10000 as
                                                           libc::c_uint).wrapping_add(0x3c00
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut EXTI_TypeDef)).FTSR;
                ::core::ptr::write_volatile(fresh5,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint & !iocurrent)
                                                as uint32_t as uint32_t)
            }
        }
        position = position.wrapping_add(1)
    };
}
/* *
  * @}
  */
/* * @defgroup GPIO_Exported_Functions_Group2 IO operation functions 
 *  @brief   GPIO Read and Write
 *
@verbatim
 ===============================================================================
                       ##### IO operation functions #####
 ===============================================================================

@endverbatim
  * @{
  */
/* *
  * @brief  Reads the specified input port pin.
  * @param  GPIOx: where x can be (A..K) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bit to read.
  *         This parameter can be GPIO_PIN_x where x can be (0..15).
  * @retval The input port pin value.
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_GPIO_ReadPin(mut GPIOx: *mut GPIO_TypeDef,
                                          mut GPIO_Pin: uint16_t)
 -> GPIO_PinState {
    let mut bitstatus: GPIO_PinState = GPIO_PIN_RESET;
    /* Check the parameters */
    if (*GPIOx).IDR & GPIO_Pin as libc::c_uint !=
           GPIO_PIN_RESET as libc::c_int as uint32_t {
        bitstatus = GPIO_PIN_SET
    } else { bitstatus = GPIO_PIN_RESET }
    return bitstatus;
}
/* *
  * @brief  Sets or clears the selected data port bit.
  *
  * @note   This function uses GPIOx_BSRR register to allow atomic read/modify
  *         accesses. In this way, there is no risk of an IRQ occurring between
  *         the read and the modify access.
  *
  * @param  GPIOx: where x can be (A..K) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @param  PinState: specifies the value to be written to the selected bit.
  *          This parameter can be one of the GPIO_PinState enum values:
  *            @arg GPIO_PIN_RESET: to clear the port pin
  *            @arg GPIO_PIN_SET: to set the port pin
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_GPIO_WritePin(mut GPIOx: *mut GPIO_TypeDef,
                                           mut GPIO_Pin: uint16_t,
                                           mut PinState: GPIO_PinState) {
    /* Check the parameters */
    if PinState as libc::c_uint !=
           GPIO_PIN_RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*GPIOx).BSRR as *mut uint32_t,
                                    GPIO_Pin as uint32_t)
    } else {
        ::core::ptr::write_volatile(&mut (*GPIOx).BSRR as *mut uint32_t,
                                    (GPIO_Pin as uint32_t) <<
                                        16 as libc::c_int)
    };
}
/* *
  * @brief  Toggles the specified GPIO pins.
  * @param  GPIOx: Where x can be (A..I) to select the GPIO peripheral.
  * @param  GPIO_Pin: Specifies the pins to be toggled.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_GPIO_TogglePin(mut GPIOx: *mut GPIO_TypeDef,
                                            mut GPIO_Pin: uint16_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*GPIOx).ODR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*GPIOx).ODR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint ^
                                     GPIO_Pin as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Locks GPIO Pins configuration registers.
  * @note   The locked registers are GPIOx_MODER, GPIOx_OTYPER, GPIOx_OSPEEDR,
  *         GPIOx_PUPDR, GPIOx_AFRL and GPIOx_AFRH.
  * @note   The configuration of the locked GPIO pins can no longer be modified
  *         until the next reset.
  * @param  GPIOx: where x can be (A..F) to select the GPIO peripheral for STM32F7 family
  * @param  GPIO_Pin: specifies the port bit to be locked.
  *         This parameter can be any combination of GPIO_PIN_x where x can be (0..15).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_GPIO_LockPin(mut GPIOx: *mut GPIO_TypeDef,
                                          mut GPIO_Pin: uint16_t)
 -> HAL_StatusTypeDef {
    let mut tmp: uint32_t = (0x1 as libc::c_uint) << 16 as libc::c_uint;
    /* Check the parameters */
    /* Apply lock key write sequence */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&tmp
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     GPIO_Pin as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Set LCKx bit(s): LCKK='1' + LCK[15-0] */
    ::core::ptr::write_volatile(&mut (*GPIOx).LCKR as *mut uint32_t, tmp);
    /* Reset LCKx bit(s): LCKK='0' + LCK[15-0] */
    ::core::ptr::write_volatile(&mut (*GPIOx).LCKR as *mut uint32_t,
                                GPIO_Pin as uint32_t);
    /* Set LCKx bit(s): LCKK='1' + LCK[15-0] */
    ::core::ptr::write_volatile(&mut (*GPIOx).LCKR as *mut uint32_t, tmp);
    /* Read LCKK bit*/
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t, (*GPIOx).LCKR);
    if (*GPIOx).LCKR & (0x1 as libc::c_uint) << 16 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        return HAL_OK
    } else { return HAL_ERROR };
}
/* *
  * @brief  This function handles EXTI interrupt request.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_GPIO_EXTI_IRQHandler(mut GPIO_Pin: uint16_t) {
    /* EXTI line interrupt detected */
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x10000 as
                                              libc::c_uint).wrapping_add(0x3c00
                                                                             as
                                                                             libc::c_uint)
              as *mut EXTI_TypeDef)).PR & GPIO_Pin as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x10000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut EXTI_TypeDef)).PR as
                                        *mut uint32_t, GPIO_Pin as uint32_t);
        HAL_GPIO_EXTI_Callback(GPIO_Pin);
    };
}
/* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup GPIO_Exported_Constants GPIO Exported Constants
  * @{
  */
/* * @defgroup GPIO_pins_define GPIO pins define
  * @{
  */
/* Pin 0 selected    */
/* Pin 1 selected    */
/* Pin 2 selected    */
/* Pin 3 selected    */
/* Pin 4 selected    */
/* Pin 5 selected    */
/* Pin 6 selected    */
/* Pin 7 selected    */
/* Pin 8 selected    */
/* Pin 9 selected    */
/* Pin 10 selected   */
/* Pin 11 selected   */
/* Pin 12 selected   */
/* Pin 13 selected   */
/* Pin 14 selected   */
/* Pin 15 selected   */
/* All pins selected */
/* PIN mask for assert test */
/* *
  * @}
  */
/* * @defgroup GPIO_mode_define GPIO mode define
  * @brief GPIO Configuration Mode 
  *        Elements values convention: 0xX0yz00YZ
  *           - X  : GPIO mode or EXTI Mode
  *           - y  : External IT or Event trigger detection 
  *           - z  : IO configuration on External IT or Event
  *           - Y  : Output type (Push Pull or Open Drain)
  *           - Z  : IO Direction mode (Input, Output, Alternate or Analog)
  * @{
  */
/* !< Input Floating Mode                   */
/* !< Output Push Pull Mode                 */
/* !< Output Open Drain Mode                */
/* !< Alternate Function Push Pull Mode     */
/* !< Alternate Function Open Drain Mode    */
/* !< Analog Mode  */
/* !< External Interrupt Mode with Rising edge trigger detection          */
/* !< External Interrupt Mode with Falling edge trigger detection         */
/* !< External Interrupt Mode with Rising/Falling edge trigger detection  */
/* !< External Event Mode with Rising edge trigger detection               */
/* !< External Event Mode with Falling edge trigger detection              */
/* !< External Event Mode with Rising/Falling edge trigger detection       */
/* *
  * @}
  */
/* * @defgroup GPIO_speed_define  GPIO speed define
  * @brief GPIO Output Maximum frequency
  * @{
  */
/* !< Low speed     */
/* !< Medium speed  */
/* !< Fast speed    */
/* !< High speed    */
/* *
  * @}
  */
/* * @defgroup GPIO_pull_define GPIO pull define
   * @brief GPIO Pull-Up or Pull-Down Activation
   * @{
   */
/* !< No Pull-up or Pull-down activation  */
/* !< Pull-up activation                  */
/* !< Pull-down activation                */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup GPIO_Exported_Macros GPIO Exported Macros
  * @{
  */
/* *
  * @brief  Checks whether the specified EXTI line flag is set or not.
  * @param  __EXTI_LINE__: specifies the EXTI line flag to check.
  *         This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval The new state of __EXTI_LINE__ (SET or RESET).
  */
/* *
  * @brief  Clears the EXTI's line pending flags.
  * @param  __EXTI_LINE__: specifies the EXTI lines flags to clear.
  *         This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
  * @retval None
  */
/* *
  * @brief  Checks whether the specified EXTI line is asserted or not.
  * @param  __EXTI_LINE__: specifies the EXTI line to check.
  *          This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval The new state of __EXTI_LINE__ (SET or RESET).
  */
/* *
  * @brief  Clears the EXTI's line pending bits.
  * @param  __EXTI_LINE__: specifies the EXTI lines to clear.
  *          This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
  * @retval None
  */
/* *
  * @brief  Generates a Software interrupt on selected EXTI line.
  * @param  __EXTI_LINE__: specifies the EXTI line to check.
  *          This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval None
  */
/* *
  * @}
  */
/* Include GPIO HAL Extension module */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup GPIO_Exported_Functions
  * @{
  */
/* * @addtogroup GPIO_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions *****************************/
/* *
  * @}
  */
/* * @addtogroup GPIO_Exported_Functions_Group2
  * @{
  */
/* IO operation functions *****************************************************/
/* *
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_GPIO_EXTI_Callback(mut GPIO_Pin: uint16_t) {
    /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* HAL_GPIO_MODULE_ENABLED */
/* *
  * @}
  */
/* *
  * @}
  */
