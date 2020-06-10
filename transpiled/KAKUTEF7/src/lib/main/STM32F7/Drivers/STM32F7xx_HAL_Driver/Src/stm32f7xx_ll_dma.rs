use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
/* *
  * @brief DMA Controller
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_Stream_TypeDef {
    pub CR: uint32_t,
    pub NDTR: uint32_t,
    pub PAR: uint32_t,
    pub M0AR: uint32_t,
    pub M1AR: uint32_t,
    pub FCR: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_TypeDef {
    pub LISR: uint32_t,
    pub HISR: uint32_t,
    pub LIFCR: uint32_t,
    pub HIFCR: uint32_t,
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
pub type ErrorStatus = libc::c_uint;
pub const SUCCESS: ErrorStatus = 1;
pub const ERROR: ErrorStatus = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_DMA_InitTypeDef {
    pub PeriphOrM2MSrcAddress: uint32_t,
    pub MemoryOrM2MDstAddress: uint32_t,
    pub Direction: uint32_t,
    pub Mode: uint32_t,
    pub PeriphOrM2MSrcIncMode: uint32_t,
    pub MemoryOrM2MDstIncMode: uint32_t,
    pub PeriphOrM2MSrcDataSize: uint32_t,
    pub MemoryOrM2MDstDataSize: uint32_t,
    pub NbData: uint32_t,
    pub Channel: uint32_t,
    pub Priority: uint32_t,
    pub FIFOMode: uint32_t,
    pub FIFOThreshold: uint32_t,
    pub MemBurst: uint32_t,
    pub PeriphBurst: uint32_t,
}
// Initialized in run_static_initializers
static mut STREAM_OFFSET_TAB: [uint8_t; 8] = [0; 8];
#[inline]
unsafe extern "C" fn LL_DMA_DisableStream(mut DMAx: *mut DMA_TypeDef,
                                          mut Stream: uint32_t) {
    let ref mut fresh0 =
        (*((DMAx as
                uint32_t).wrapping_add(STREAM_OFFSET_TAB[Stream as usize] as
                                           libc::c_uint) as
               *mut DMA_Stream_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
#[inline]
unsafe extern "C" fn LL_DMA_ConfigTransfer(mut DMAx: *mut DMA_TypeDef,
                                           mut Stream: uint32_t,
                                           mut Configuration: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((DMAx as
                                             uint32_t).wrapping_add(STREAM_OFFSET_TAB[Stream
                                                                                          as
                                                                                          usize]
                                                                        as
                                                                        libc::c_uint)
                                            as *mut DMA_Stream_TypeDef)).CR as
                                    *mut uint32_t,
                                (*((DMAx as
                                        uint32_t).wrapping_add(STREAM_OFFSET_TAB[Stream
                                                                                     as
                                                                                     usize]
                                                                   as
                                                                   libc::c_uint)
                                       as *mut DMA_Stream_TypeDef)).CR &
                                    !((0x3 as libc::c_uint) <<
                                          6 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              8 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              9 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              10 as libc::c_uint |
                                          (0x3 as libc::c_uint) <<
                                              11 as libc::c_uint |
                                          (0x3 as libc::c_uint) <<
                                              13 as libc::c_uint |
                                          (0x3 as libc::c_uint) <<
                                              16 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              5 as libc::c_uint) |
                                    Configuration);
}
#[inline]
unsafe extern "C" fn LL_DMA_SetDataLength(mut DMAx: *mut DMA_TypeDef,
                                          mut Stream: uint32_t,
                                          mut NbData: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((DMAx as
                                             uint32_t).wrapping_add(STREAM_OFFSET_TAB[Stream
                                                                                          as
                                                                                          usize]
                                                                        as
                                                                        libc::c_uint)
                                            as *mut DMA_Stream_TypeDef)).NDTR
                                    as *mut uint32_t,
                                (*((DMAx as
                                        uint32_t).wrapping_add(STREAM_OFFSET_TAB[Stream
                                                                                     as
                                                                                     usize]
                                                                   as
                                                                   libc::c_uint)
                                       as *mut DMA_Stream_TypeDef)).NDTR &
                                    !((0xffff as libc::c_uint) <<
                                          0 as libc::c_uint) | NbData);
}
#[inline]
unsafe extern "C" fn LL_DMA_SetChannelSelection(mut DMAx: *mut DMA_TypeDef,
                                                mut Stream: uint32_t,
                                                mut Channel: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((DMAx as
                                             uint32_t).wrapping_add(STREAM_OFFSET_TAB[Stream
                                                                                          as
                                                                                          usize]
                                                                        as
                                                                        libc::c_uint)
                                            as *mut DMA_Stream_TypeDef)).CR as
                                    *mut uint32_t,
                                (*((DMAx as
                                        uint32_t).wrapping_add(STREAM_OFFSET_TAB[Stream
                                                                                     as
                                                                                     usize]
                                                                   as
                                                                   libc::c_uint)
                                       as *mut DMA_Stream_TypeDef)).CR &
                                    !((0x7 as libc::c_uint) <<
                                          25 as libc::c_uint) | Channel);
}
#[inline]
unsafe extern "C" fn LL_DMA_SetMemoryBurstxfer(mut DMAx: *mut DMA_TypeDef,
                                               mut Stream: uint32_t,
                                               mut Mburst: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((DMAx as
                                             uint32_t).wrapping_add(STREAM_OFFSET_TAB[Stream
                                                                                          as
                                                                                          usize]
                                                                        as
                                                                        libc::c_uint)
                                            as *mut DMA_Stream_TypeDef)).CR as
                                    *mut uint32_t,
                                (*((DMAx as
                                        uint32_t).wrapping_add(STREAM_OFFSET_TAB[Stream
                                                                                     as
                                                                                     usize]
                                                                   as
                                                                   libc::c_uint)
                                       as *mut DMA_Stream_TypeDef)).CR &
                                    !((0x3 as libc::c_uint) <<
                                          23 as libc::c_uint) | Mburst);
}
#[inline]
unsafe extern "C" fn LL_DMA_SetPeriphBurstxfer(mut DMAx: *mut DMA_TypeDef,
                                               mut Stream: uint32_t,
                                               mut Pburst: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((DMAx as
                                             uint32_t).wrapping_add(STREAM_OFFSET_TAB[Stream
                                                                                          as
                                                                                          usize]
                                                                        as
                                                                        libc::c_uint)
                                            as *mut DMA_Stream_TypeDef)).CR as
                                    *mut uint32_t,
                                (*((DMAx as
                                        uint32_t).wrapping_add(STREAM_OFFSET_TAB[Stream
                                                                                     as
                                                                                     usize]
                                                                   as
                                                                   libc::c_uint)
                                       as *mut DMA_Stream_TypeDef)).CR &
                                    !((0x3 as libc::c_uint) <<
                                          21 as libc::c_uint) | Pburst);
}
#[inline]
unsafe extern "C" fn LL_DMA_ConfigFifo(mut DMAx: *mut DMA_TypeDef,
                                       mut Stream: uint32_t,
                                       mut FifoMode: uint32_t,
                                       mut FifoThreshold: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((DMAx as
                                             uint32_t).wrapping_add(STREAM_OFFSET_TAB[Stream
                                                                                          as
                                                                                          usize]
                                                                        as
                                                                        libc::c_uint)
                                            as *mut DMA_Stream_TypeDef)).FCR
                                    as *mut uint32_t,
                                (*((DMAx as
                                        uint32_t).wrapping_add(STREAM_OFFSET_TAB[Stream
                                                                                     as
                                                                                     usize]
                                                                   as
                                                                   libc::c_uint)
                                       as *mut DMA_Stream_TypeDef)).FCR &
                                    !((0x3 as libc::c_uint) <<
                                          0 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              2 as libc::c_uint) |
                                    (FifoMode | FifoThreshold));
}
#[inline]
unsafe extern "C" fn LL_DMA_SetMemoryAddress(mut DMAx: *mut DMA_TypeDef,
                                             mut Stream: uint32_t,
                                             mut MemoryAddress: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((DMAx as
                                             uint32_t).wrapping_add(STREAM_OFFSET_TAB[Stream
                                                                                          as
                                                                                          usize]
                                                                        as
                                                                        libc::c_uint)
                                            as *mut DMA_Stream_TypeDef)).M0AR
                                    as *mut uint32_t, MemoryAddress);
}
#[inline]
unsafe extern "C" fn LL_DMA_SetPeriphAddress(mut DMAx: *mut DMA_TypeDef,
                                             mut Stream: uint32_t,
                                             mut PeriphAddress: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((DMAx as
                                             uint32_t).wrapping_add(STREAM_OFFSET_TAB[Stream
                                                                                          as
                                                                                          usize]
                                                                        as
                                                                        libc::c_uint)
                                            as *mut DMA_Stream_TypeDef)).PAR
                                    as *mut uint32_t, PeriphAddress);
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
    let ref mut fresh2 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1RSTR;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !Periphs) as uint32_t
                                    as uint32_t);
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_ll_dma.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   DMA LL module driver.
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
/* * @defgroup DMA_LL DMA
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* * @addtogroup DMA_LL_Private_Macros
  * @{
  */
/* DMA_CHANNEL_SELECTION_8_15 */
/* *
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup DMA_LL_Exported_Functions
  * @{
  */
/* * @addtogroup DMA_LL_EF_Init
  * @{
  */
/* *
  * @brief  De-initialize the DMA registers to their default reset values.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  *         @arg @ref LL_DMA_STREAM_ALL
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DMA registers are de-initialized
  *          - ERROR: DMA registers are not de-initialized
  */
#[no_mangle]
pub unsafe extern "C" fn LL_DMA_DeInit(mut DMAx: *mut DMA_TypeDef,
                                       mut Stream: uint32_t) -> uint32_t {
    let mut tmp: *mut DMA_Stream_TypeDef =
        (0x40000000 as
             libc::c_uint).wrapping_add(0x20000 as
                                            libc::c_uint).wrapping_add(0x6000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x10
                                                                                                          as
                                                                                                          libc::c_uint)
            as *mut DMA_Stream_TypeDef;
    let mut status: ErrorStatus = SUCCESS;
    /* Check the DMA Instance DMAx and Stream parameters*/
    if Stream == 0xffff0000 as libc::c_uint {
        if DMAx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x6000
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut DMA_TypeDef {
            /* Force reset of DMA clock */
            LL_AHB1_GRP1_ForceReset((0x1 as libc::c_uint) <<
                                        21 as libc::c_uint);
            /* Release reset of DMA clock */
            LL_AHB1_GRP1_ReleaseReset((0x1 as libc::c_uint) <<
                                          21 as libc::c_uint);
        } else if DMAx ==
                      (0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0x6400
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut DMA_TypeDef {
            /* Force reset of DMA clock */
            LL_AHB1_GRP1_ForceReset((0x1 as libc::c_uint) <<
                                        22 as libc::c_uint);
            /* Release reset of DMA clock */
            LL_AHB1_GRP1_ReleaseReset((0x1 as libc::c_uint) <<
                                          22 as libc::c_uint);
        } else { status = ERROR }
    } else {
        /* Disable the selected Stream */
        LL_DMA_DisableStream(DMAx, Stream);
        /* Get the DMA Stream Instance */
        tmp =
            if DMAx as uint32_t ==
                   (0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x6000
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut DMA_TypeDef as uint32_t &&
                   Stream == 0 as libc::c_uint {
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x6000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x10
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as *mut DMA_Stream_TypeDef
            } else if DMAx as uint32_t ==
                          (0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x6400
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut DMA_TypeDef as uint32_t &&
                          Stream == 0 as libc::c_uint {
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x6400
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x10
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as *mut DMA_Stream_TypeDef
            } else if DMAx as uint32_t ==
                          (0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x6000
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut DMA_TypeDef as uint32_t &&
                          Stream == 0x1 as libc::c_uint {
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x6000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x28
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as *mut DMA_Stream_TypeDef
            } else if DMAx as uint32_t ==
                          (0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x6400
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut DMA_TypeDef as uint32_t &&
                          Stream == 0x1 as libc::c_uint {
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x6400
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x28
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as *mut DMA_Stream_TypeDef
            } else if DMAx as uint32_t ==
                          (0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x6000
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut DMA_TypeDef as uint32_t &&
                          Stream == 0x2 as libc::c_uint {
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x6000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x40
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as *mut DMA_Stream_TypeDef
            } else if DMAx as uint32_t ==
                          (0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x6400
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut DMA_TypeDef as uint32_t &&
                          Stream == 0x2 as libc::c_uint {
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x6400
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x40
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as *mut DMA_Stream_TypeDef
            } else if DMAx as uint32_t ==
                          (0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x6000
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut DMA_TypeDef as uint32_t &&
                          Stream == 0x3 as libc::c_uint {
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x6000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x58
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as *mut DMA_Stream_TypeDef
            } else if DMAx as uint32_t ==
                          (0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x6400
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut DMA_TypeDef as uint32_t &&
                          Stream == 0x3 as libc::c_uint {
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x6400
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x58
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as *mut DMA_Stream_TypeDef
            } else if DMAx as uint32_t ==
                          (0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x6000
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut DMA_TypeDef as uint32_t &&
                          Stream == 0x4 as libc::c_uint {
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x6000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x70
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as *mut DMA_Stream_TypeDef
            } else if DMAx as uint32_t ==
                          (0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x6400
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut DMA_TypeDef as uint32_t &&
                          Stream == 0x4 as libc::c_uint {
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x6400
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x70
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as *mut DMA_Stream_TypeDef
            } else if DMAx as uint32_t ==
                          (0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x6000
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut DMA_TypeDef as uint32_t &&
                          Stream == 0x5 as libc::c_uint {
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x6000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x88
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as *mut DMA_Stream_TypeDef
            } else if DMAx as uint32_t ==
                          (0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x6400
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut DMA_TypeDef as uint32_t &&
                          Stream == 0x5 as libc::c_uint {
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x6400
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x88
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as *mut DMA_Stream_TypeDef
            } else if DMAx as uint32_t ==
                          (0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x6000
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut DMA_TypeDef as uint32_t &&
                          Stream == 0x6 as libc::c_uint {
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x6000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0xa0
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as *mut DMA_Stream_TypeDef
            } else if DMAx as uint32_t ==
                          (0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x6400
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut DMA_TypeDef as uint32_t &&
                          Stream == 0x6 as libc::c_uint {
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x6400
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0xa0
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as *mut DMA_Stream_TypeDef
            } else if DMAx as uint32_t ==
                          (0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x6000
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut DMA_TypeDef as uint32_t &&
                          Stream == 0x7 as libc::c_uint {
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x6000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0xb8
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as *mut DMA_Stream_TypeDef
            } else {
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x6400
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0xb8
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as *mut DMA_Stream_TypeDef
            };
        /* Reset DMAx_Streamy configuration register */
        ::core::ptr::write_volatile(&mut (*tmp).CR as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Reset DMAx_Streamy remaining bytes register */
        ::core::ptr::write_volatile(&mut (*tmp).NDTR as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Reset DMAx_Streamy peripheral address register */
        ::core::ptr::write_volatile(&mut (*tmp).PAR as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Reset DMAx_Streamy memory address register */
        ::core::ptr::write_volatile(&mut (*tmp).M0AR as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Reset DMAx_Streamy memory address register */
        ::core::ptr::write_volatile(&mut (*tmp).M1AR as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Reset DMAx_Streamy FIFO control register */
        ::core::ptr::write_volatile(&mut (*tmp).FCR as *mut uint32_t,
                                    0x21 as libc::c_uint);
        /* Reset Channel register field for DMAx Stream*/
        LL_DMA_SetChannelSelection(DMAx, Stream, 0 as libc::c_uint);
        if Stream == 0 as libc::c_uint {
            /* Reset the Stream0 pending flags */
            ::core::ptr::write_volatile(&mut (*DMAx).LIFCR as *mut uint32_t,
                                        0x3f as libc::c_uint)
        } else if Stream == 0x1 as libc::c_uint {
            /* Reset the Stream1 pending flags */
            ::core::ptr::write_volatile(&mut (*DMAx).LIFCR as *mut uint32_t,
                                        0xf40 as libc::c_uint)
        } else if Stream == 0x2 as libc::c_uint {
            /* Reset the Stream2 pending flags */
            ::core::ptr::write_volatile(&mut (*DMAx).LIFCR as *mut uint32_t,
                                        0x3f0000 as libc::c_uint)
        } else if Stream == 0x3 as libc::c_uint {
            /* Reset the Stream3 pending flags */
            ::core::ptr::write_volatile(&mut (*DMAx).LIFCR as *mut uint32_t,
                                        0xf400000 as libc::c_uint)
        } else if Stream == 0x4 as libc::c_uint {
            /* Reset the Stream4 pending flags */
            ::core::ptr::write_volatile(&mut (*DMAx).HIFCR as *mut uint32_t,
                                        0x3f as libc::c_uint)
        } else if Stream == 0x5 as libc::c_uint {
            /* Reset the Stream5 pending flags */
            ::core::ptr::write_volatile(&mut (*DMAx).HIFCR as *mut uint32_t,
                                        0xf40 as libc::c_uint)
        } else if Stream == 0x6 as libc::c_uint {
            /* Reset the Stream6 pending flags */
            ::core::ptr::write_volatile(&mut (*DMAx).HIFCR as *mut uint32_t,
                                        0x3f0000 as libc::c_uint)
        } else if Stream == 0x7 as libc::c_uint {
            /* Reset the Stream7 pending flags */
            ::core::ptr::write_volatile(&mut (*DMAx).HIFCR as *mut uint32_t,
                                        0xf400000 as libc::c_uint)
        } else { status = ERROR }
    }
    return status as uint32_t;
}
/* *
  * @brief  Initialize the DMA registers according to the specified parameters in DMA_InitStruct.
  * @note   To convert DMAx_Streamy Instance to DMAx Instance and Streamy, use helper macros :
  *         @arg @ref __LL_DMA_GET_INSTANCE
  *         @arg @ref __LL_DMA_GET_STREAM
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  DMA_InitStruct pointer to a @ref LL_DMA_InitTypeDef structure.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DMA registers are initialized
  *          - ERROR: Not applicable
  */
#[no_mangle]
pub unsafe extern "C" fn LL_DMA_Init(mut DMAx: *mut DMA_TypeDef,
                                     mut Stream: uint32_t,
                                     mut DMA_InitStruct:
                                         *mut LL_DMA_InitTypeDef)
 -> uint32_t {
    /* Check the DMA Instance DMAx and Stream parameters*/
    /* Check the memory burst, peripheral burst and FIFO threshold parameters only
     when FIFO mode is enabled */
    ((*DMA_InitStruct).FIFOMode) != 0 as libc::c_uint;
    /*---------------------------- DMAx SxCR Configuration ------------------------
   * Configure DMAx_Streamy: data transfer direction, data transfer mode,
   *                          peripheral and memory increment mode,
   *                          data size alignment and  priority level with parameters :
   * - Direction:      DMA_SxCR_DIR[1:0] bits
   * - Mode:           DMA_SxCR_CIRC bit
   * - PeriphOrM2MSrcIncMode:  DMA_SxCR_PINC bit
   * - MemoryOrM2MDstIncMode:  DMA_SxCR_MINC bit
   * - PeriphOrM2MSrcDataSize: DMA_SxCR_PSIZE[1:0] bits
   * - MemoryOrM2MDstDataSize: DMA_SxCR_MSIZE[1:0] bits
   * - Priority:               DMA_SxCR_PL[1:0] bits
   */
    LL_DMA_ConfigTransfer(DMAx, Stream,
                          (*DMA_InitStruct).Direction | (*DMA_InitStruct).Mode
                              | (*DMA_InitStruct).PeriphOrM2MSrcIncMode |
                              (*DMA_InitStruct).MemoryOrM2MDstIncMode |
                              (*DMA_InitStruct).PeriphOrM2MSrcDataSize |
                              (*DMA_InitStruct).MemoryOrM2MDstDataSize |
                              (*DMA_InitStruct).Priority);
    if (*DMA_InitStruct).FIFOMode != 0 as libc::c_uint {
        /*---------------------------- DMAx SxFCR Configuration ------------------------
     * Configure DMAx_Streamy:  fifo mode and fifo threshold with parameters :
     * - FIFOMode:                DMA_SxFCR_DMDIS bit
     * - FIFOThreshold:           DMA_SxFCR_FTH[1:0] bits
     */
        LL_DMA_ConfigFifo(DMAx, Stream, (*DMA_InitStruct).FIFOMode,
                          (*DMA_InitStruct).FIFOThreshold);
        /*---------------------------- DMAx SxCR Configuration --------------------------
     * Configure DMAx_Streamy:  memory burst transfer with parameters :
     * - MemBurst:                DMA_SxCR_MBURST[1:0] bits
     */
        LL_DMA_SetMemoryBurstxfer(DMAx, Stream, (*DMA_InitStruct).MemBurst);
        /*---------------------------- DMAx SxCR Configuration --------------------------
     * Configure DMAx_Streamy:  peripheral burst transfer with parameters :
     * - PeriphBurst:             DMA_SxCR_PBURST[1:0] bits
     */
        LL_DMA_SetPeriphBurstxfer(DMAx, Stream,
                                  (*DMA_InitStruct).PeriphBurst);
    }
    /*-------------------------- DMAx SxM0AR Configuration --------------------------
   * Configure the memory or destination base address with parameter :
   * - MemoryOrM2MDstAddress:     DMA_SxM0AR_M0A[31:0] bits
   */
    LL_DMA_SetMemoryAddress(DMAx, Stream,
                            (*DMA_InitStruct).MemoryOrM2MDstAddress);
    /*-------------------------- DMAx SxPAR Configuration ---------------------------
   * Configure the peripheral or source base address with parameter :
   * - PeriphOrM2MSrcAddress:     DMA_SxPAR_PA[31:0] bits
   */
    LL_DMA_SetPeriphAddress(DMAx, Stream,
                            (*DMA_InitStruct).PeriphOrM2MSrcAddress);
    /*--------------------------- DMAx SxNDTR Configuration -------------------------
   * Configure the peripheral base address with parameter :
   * - NbData:                    DMA_SxNDT[15:0] bits
   */
    LL_DMA_SetDataLength(DMAx, Stream, (*DMA_InitStruct).NbData);
    /*--------------------------- DMA SxCR_CHSEL Configuration ----------------------
   * Configure the peripheral base address with parameter :
   * - PeriphRequest:             DMA_SxCR_CHSEL[3:0] bits
   */
    LL_DMA_SetChannelSelection(DMAx, Stream, (*DMA_InitStruct).Channel);
    return SUCCESS as libc::c_int as uint32_t;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_ll_dma.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of DMA LL module.
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
/* * @defgroup DMA_LL DMA
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* * @defgroup DMA_LL_Private_Variables DMA Private Variables
  * @{
  */
/* Array used to get the DMA stream register offset versus stream index LL_DMA_STREAM_x */
/* *
  * @}
  */
/* Private constants ---------------------------------------------------------*/
/* * @defgroup DMA_LL_Private_Constants DMA Private Constants
  * @{
  */
/* DMA_SxCR_CHSEL_3 */
/* *
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* * @defgroup DMA_LL_ES_INIT DMA Exported Init structure
  * @{
  */
/* !< Specifies the peripheral base address for DMA transfer
                                        or as Source base address in case of memory to memory transfer direction.

                                        This parameter must be a value between Min_Data = 0 and Max_Data = 0xFFFFFFFF. */
/* !< Specifies the memory base address for DMA transfer
                                        or as Destination base address in case of memory to memory transfer direction.

                                        This parameter must be a value between Min_Data = 0 and Max_Data = 0xFFFFFFFF. */
/* !< Specifies if the data will be transferred from memory to peripheral,
                                        from memory to memory or from peripheral to memory.
                                        This parameter can be a value of @ref DMA_LL_EC_DIRECTION

                                        This feature can be modified afterwards using unitary function @ref LL_DMA_SetDataTransferDirection(). */
/* !< Specifies the normal or circular operation mode.
                                        This parameter can be a value of @ref DMA_LL_EC_MODE
                                        @note The circular buffer mode cannot be used if the memory to memory
                                              data transfer direction is configured on the selected Stream

                                        This feature can be modified afterwards using unitary function @ref LL_DMA_SetMode(). */
/* !< Specifies whether the Peripheral address or Source address in case of memory to memory transfer direction
                                        is incremented or not.
                                        This parameter can be a value of @ref DMA_LL_EC_PERIPH

                                        This feature can be modified afterwards using unitary function @ref LL_DMA_SetPeriphIncMode(). */
/* !< Specifies whether the Memory address or Destination address in case of memory to memory transfer direction
                                        is incremented or not.
                                        This parameter can be a value of @ref DMA_LL_EC_MEMORY

                                        This feature can be modified afterwards using unitary function @ref LL_DMA_SetMemoryIncMode(). */
/* !< Specifies the Peripheral data size alignment or Source data size alignment (byte, half word, word)
                                        in case of memory to memory transfer direction.
                                        This parameter can be a value of @ref DMA_LL_EC_PDATAALIGN

                                        This feature can be modified afterwards using unitary function @ref LL_DMA_SetPeriphSize(). */
/* !< Specifies the Memory data size alignment or Destination data size alignment (byte, half word, word)
                                        in case of memory to memory transfer direction.
                                        This parameter can be a value of @ref DMA_LL_EC_MDATAALIGN

                                        This feature can be modified afterwards using unitary function @ref LL_DMA_SetMemorySize(). */
/* !< Specifies the number of data to transfer, in data unit.
                                        The data unit is equal to the source buffer configuration set in PeripheralSize
                                        or MemorySize parameters depending in the transfer direction.
                                        This parameter must be a value between Min_Data = 0 and Max_Data = 0x0000FFFF

                                        This feature can be modified afterwards using unitary function @ref LL_DMA_SetDataLength(). */
/* !< Specifies the peripheral channel.
                                        This parameter can be a value of @ref DMA_LL_EC_CHANNEL

                                        This feature can be modified afterwards using unitary function @ref LL_DMA_SetChannelSelection(). */
/* !< Specifies the channel priority level.
                                        This parameter can be a value of @ref DMA_LL_EC_PRIORITY

                                        This feature can be modified afterwards using unitary function @ref LL_DMA_SetStreamPriorityLevel(). */
/* !< Specifies if the FIFO mode or Direct mode will be used for the specified stream.
                                        This parameter can be a value of @ref DMA_LL_FIFOMODE
                                        @note The Direct mode (FIFO mode disabled) cannot be used if the 
                                        memory-to-memory data transfer is configured on the selected stream

                                        This feature can be modified afterwards using unitary functions @ref LL_DMA_EnableFifoMode() or @ref LL_DMA_EnableFifoMode() . */
/* !< Specifies the FIFO threshold level.
                                        This parameter can be a value of @ref DMA_LL_EC_FIFOTHRESHOLD

                                        This feature can be modified afterwards using unitary function @ref LL_DMA_SetFIFOThreshold(). */
/* !< Specifies the Burst transfer configuration for the memory transfers. 
                                        It specifies the amount of data to be transferred in a single non interruptible
                                        transaction.
                                        This parameter can be a value of @ref DMA_LL_EC_MBURST 
                                        @note The burst mode is possible only if the address Increment mode is enabled. 

                                        This feature can be modified afterwards using unitary function @ref LL_DMA_SetMemoryBurstxfer(). */
/* !< Specifies the Burst transfer configuration for the peripheral transfers. 
                                        It specifies the amount of data to be transferred in a single non interruptible 
                                        transaction. 
                                        This parameter can be a value of @ref DMA_LL_EC_PBURST
                                        @note The burst mode is possible only if the address Increment mode is enabled. 

                                        This feature can be modified afterwards using unitary function @ref LL_DMA_SetPeriphBurstxfer(). */
/* *
  * @}
  */
/*USE_FULL_LL_DRIVER*/
/* Exported constants --------------------------------------------------------*/
/* * @defgroup DMA_LL_Exported_Constants DMA Exported Constants
  * @{
  */
/* * @defgroup DMA_LL_EC_STREAM STREAM
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EC_DIRECTION DIRECTION
  * @{
  */
/* !< Peripheral to memory direction */
/* !< Memory to peripheral direction */
/* !< Memory to memory direction     */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EC_MODE MODE
  * @{
  */
/* !< Normal Mode                  */
/* !< Circular Mode                */
/* !< Peripheral flow control mode */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EC_DOUBLEBUFFER_MODE DOUBLE BUFFER MODE
  * @{
  */
/* !< Disable double buffering mode */
/* !< Enable double buffering mode  */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EC_PERIPH PERIPH
  * @{
  */
/* !< Peripheral increment mode Disable */
/* !< Peripheral increment mode Enable  */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EC_MEMORY MEMORY
  * @{
  */
/* !< Memory increment mode Disable */
/* !< Memory increment mode Enable  */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EC_PDATAALIGN PDATAALIGN
  * @{
  */
/* !< Peripheral data alignment : Byte     */
/* !< Peripheral data alignment : HalfWord */
/* !< Peripheral data alignment : Word     */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EC_MDATAALIGN MDATAALIGN
  * @{
  */
/* !< Memory data alignment : Byte     */
/* !< Memory data alignment : HalfWord */
/* !< Memory data alignment : Word     */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EC_OFFSETSIZE OFFSETSIZE
  * @{
  */
/* !< Peripheral increment offset size is linked to the PSIZE */
/* !< Peripheral increment offset size is fixed to 4 (32-bit alignment) */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EC_PRIORITY PRIORITY
  * @{
  */
/* !< Priority level : Low       */
/* !< Priority level : Medium    */
/* !< Priority level : High      */
/* !< Priority level : Very_High */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EC_CHANNEL CHANNEL
  * @{
  */
/* Select Channel0 of DMA Instance */
/* Select Channel1 of DMA Instance */
/* Select Channel2 of DMA Instance */
/* Select Channel3 of DMA Instance */
/* Select Channel4 of DMA Instance */
/* Select Channel5 of DMA Instance */
/* Select Channel6 of DMA Instance */
/* Select Channel7 of DMA Instance */
/* DMA_CHANNEL_SELECTION_8_15 */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EC_MBURST MBURST
  * @{
  */
/* !< Memory burst single transfer configuration */
/* !< Memory burst of 4 beats transfer configuration */
/* !< Memory burst of 8 beats transfer configuration */
/* !< Memory burst of 16 beats transfer configuration */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EC_PBURST PBURST
  * @{
  */
/* !< Peripheral burst single transfer configuration */
/* !< Peripheral burst of 4 beats transfer configuration */
/* !< Peripheral burst of 8 beats transfer configuration */
/* !< Peripheral burst of 16 beats transfer configuration */
/* *
  * @}
  */
/* * @defgroup DMA_LL_FIFOMODE DMA_LL_FIFOMODE
  * @{
  */
/* !< FIFO mode disable (direct mode is enabled) */
/* !< FIFO mode enable  */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EC_FIFOSTATUS_0 FIFOSTATUS 0
  * @{
  */
/* !< 0 < fifo_level < 1/4    */
/* !< 1/4 < fifo_level < 1/2  */
/* !< 1/2 < fifo_level < 3/4  */
/* !< 3/4 < fifo_level < full */
/* !< FIFO is empty           */
/* !< FIFO is full            */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EC_FIFOTHRESHOLD FIFOTHRESHOLD
  * @{
  */
/* !< FIFO threshold 1 quart full configuration  */
/* !< FIFO threshold half full configuration     */
/* !< FIFO threshold 3 quarts full configuration */
/* !< FIFO threshold full configuration          */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EC_CURRENTTARGETMEM CURRENTTARGETMEM
  * @{
  */
/* !< Set CurrentTarget Memory to Memory 0  */
/* !< Set CurrentTarget Memory to Memory 1  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup DMA_LL_Exported_Macros DMA Exported Macros
  * @{
  */
/* * @defgroup DMA_LL_EM_WRITE_READ Common Write and read registers macros
  * @{
  */
/* *
  * @brief  Write a value in DMA register
  * @param  __INSTANCE__ DMA Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
/* *
  * @brief  Read a value in DMA register
  * @param  __INSTANCE__ DMA Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EM_CONVERT_DMAxCHANNELy Convert DMAxStreamy
  * @{
  */
/* *
  * @brief  Convert DMAx_Streamy into DMAx
  * @param  __STREAM_INSTANCE__ DMAx_Streamy
  * @retval DMAx
  */
/* *
  * @brief  Convert DMAx_Streamy into LL_DMA_STREAM_y
  * @param  __STREAM_INSTANCE__ DMAx_Streamy
  * @retval LL_DMA_CHANNEL_y
  */
/* *
  * @brief  Convert DMA Instance DMAx and LL_DMA_STREAM_y into DMAx_Streamy
  * @param  __DMA_INSTANCE__ DMAx
  * @param  __STREAM__ LL_DMA_STREAM_y
  * @retval DMAx_Streamy
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
 /* * @defgroup DMA_LL_Exported_Functions DMA Exported Functions
  * @{
  */
/* * @defgroup DMA_LL_EF_Configuration Configuration
  * @{
  */
/* *
  * @brief Enable DMA stream.
  * @rmtoll CR          EN            LL_DMA_EnableStream
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
/* *
  * @brief Disable DMA stream.
  * @rmtoll CR          EN            LL_DMA_DisableStream
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
/* *
  * @brief Check if DMA stream is enabled or disabled.
  * @rmtoll CR          EN            LL_DMA_IsEnabledStream
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Configure all parameters linked to DMA transfer.
  * @rmtoll CR          DIR           LL_DMA_ConfigTransfer\n
  *         CR          CIRC          LL_DMA_ConfigTransfer\n
  *         CR          PINC          LL_DMA_ConfigTransfer\n
  *         CR          MINC          LL_DMA_ConfigTransfer\n
  *         CR          PSIZE         LL_DMA_ConfigTransfer\n
  *         CR          MSIZE         LL_DMA_ConfigTransfer\n
  *         CR          PL            LL_DMA_ConfigTransfer\n
  *         CR          PFCTRL        LL_DMA_ConfigTransfer
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  Configuration This parameter must be a combination of all the following values:
  *         @arg @ref LL_DMA_DIRECTION_PERIPH_TO_MEMORY or @ref LL_DMA_DIRECTION_MEMORY_TO_PERIPH or @ref LL_DMA_DIRECTION_MEMORY_TO_MEMORY
  *         @arg @ref LL_DMA_MODE_NORMAL or @ref LL_DMA_MODE_CIRCULAR  or @ref LL_DMA_MODE_PFCTRL
  *         @arg @ref LL_DMA_PERIPH_INCREMENT or @ref LL_DMA_PERIPH_NOINCREMENT
  *         @arg @ref LL_DMA_MEMORY_INCREMENT or @ref LL_DMA_MEMORY_NOINCREMENT
  *         @arg @ref LL_DMA_PDATAALIGN_BYTE or @ref LL_DMA_PDATAALIGN_HALFWORD or @ref LL_DMA_PDATAALIGN_WORD
  *         @arg @ref LL_DMA_MDATAALIGN_BYTE or @ref LL_DMA_MDATAALIGN_HALFWORD or @ref LL_DMA_MDATAALIGN_WORD
  *         @arg @ref LL_DMA_PRIORITY_LOW or @ref LL_DMA_PRIORITY_MEDIUM or @ref LL_DMA_PRIORITY_HIGH or @ref LL_DMA_PRIORITY_VERYHIGH
  *@retval None
  */
/* *
  * @brief Set Data transfer direction (read from peripheral or from memory).
  * @rmtoll CR          DIR           LL_DMA_SetDataTransferDirection
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  Direction This parameter can be one of the following values:
  *         @arg @ref LL_DMA_DIRECTION_PERIPH_TO_MEMORY
  *         @arg @ref LL_DMA_DIRECTION_MEMORY_TO_PERIPH
  *         @arg @ref LL_DMA_DIRECTION_MEMORY_TO_MEMORY
  * @retval None
  */
/* *
  * @brief Get Data transfer direction (read from peripheral or from memory).
  * @rmtoll CR          DIR           LL_DMA_GetDataTransferDirection
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_DIRECTION_PERIPH_TO_MEMORY
  *         @arg @ref LL_DMA_DIRECTION_MEMORY_TO_PERIPH
  *         @arg @ref LL_DMA_DIRECTION_MEMORY_TO_MEMORY
  */
/* *
  * @brief Set DMA mode normal, circular or peripheral flow control.
  * @rmtoll CR          CIRC           LL_DMA_SetMode\n
  *         CR          PFCTRL         LL_DMA_SetMode
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  Mode This parameter can be one of the following values:
  *         @arg @ref LL_DMA_MODE_NORMAL
  *         @arg @ref LL_DMA_MODE_CIRCULAR
  *         @arg @ref LL_DMA_MODE_PFCTRL
  * @retval None
  */
/* *
  * @brief Get DMA mode normal, circular or peripheral flow control.
  * @rmtoll CR          CIRC           LL_DMA_GetMode\n
  *         CR          PFCTRL         LL_DMA_GetMode
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_MODE_NORMAL
  *         @arg @ref LL_DMA_MODE_CIRCULAR
  *         @arg @ref LL_DMA_MODE_PFCTRL
  */
/* *
  * @brief Set Peripheral increment mode.
  * @rmtoll CR          PINC           LL_DMA_SetPeriphIncMode
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  IncrementMode This parameter can be one of the following values:
  *         @arg @ref LL_DMA_PERIPH_NOINCREMENT
  *         @arg @ref LL_DMA_PERIPH_INCREMENT
  * @retval None
  */
/* *
  * @brief Get Peripheral increment mode.
  * @rmtoll CR          PINC           LL_DMA_GetPeriphIncMode
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_PERIPH_NOINCREMENT
  *         @arg @ref LL_DMA_PERIPH_INCREMENT
  */
/* *
  * @brief Set Memory increment mode.
  * @rmtoll CR          MINC           LL_DMA_SetMemoryIncMode
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  IncrementMode This parameter can be one of the following values:
  *         @arg @ref LL_DMA_MEMORY_NOINCREMENT
  *         @arg @ref LL_DMA_MEMORY_INCREMENT
  * @retval None
  */
/* *
  * @brief Get Memory increment mode.
  * @rmtoll CR          MINC           LL_DMA_GetMemoryIncMode
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_MEMORY_NOINCREMENT
  *         @arg @ref LL_DMA_MEMORY_INCREMENT
  */
/* *
  * @brief Set Peripheral size.
  * @rmtoll CR          PSIZE           LL_DMA_SetPeriphSize
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  Size This parameter can be one of the following values:
  *         @arg @ref LL_DMA_PDATAALIGN_BYTE
  *         @arg @ref LL_DMA_PDATAALIGN_HALFWORD
  *         @arg @ref LL_DMA_PDATAALIGN_WORD
  * @retval None
  */
/* *
  * @brief Get Peripheral size.
  * @rmtoll CR          PSIZE           LL_DMA_GetPeriphSize
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_PDATAALIGN_BYTE
  *         @arg @ref LL_DMA_PDATAALIGN_HALFWORD
  *         @arg @ref LL_DMA_PDATAALIGN_WORD
  */
/* *
  * @brief Set Memory size.
  * @rmtoll CR          MSIZE           LL_DMA_SetMemorySize
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  Size This parameter can be one of the following values:
  *         @arg @ref LL_DMA_MDATAALIGN_BYTE
  *         @arg @ref LL_DMA_MDATAALIGN_HALFWORD
  *         @arg @ref LL_DMA_MDATAALIGN_WORD
  * @retval None
  */
/* *
  * @brief Get Memory size.
  * @rmtoll CR          MSIZE           LL_DMA_GetMemorySize
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_MDATAALIGN_BYTE
  *         @arg @ref LL_DMA_MDATAALIGN_HALFWORD
  *         @arg @ref LL_DMA_MDATAALIGN_WORD
  */
/* *
  * @brief Set Peripheral increment offset size.
  * @rmtoll CR          PINCOS           LL_DMA_SetIncOffsetSize
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  OffsetSize This parameter can be one of the following values:
  *         @arg @ref LL_DMA_OFFSETSIZE_PSIZE
  *         @arg @ref LL_DMA_OFFSETSIZE_FIXEDTO4
  * @retval None
  */
/* *
  * @brief Get Peripheral increment offset size.
  * @rmtoll CR          PINCOS           LL_DMA_GetIncOffsetSize
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_OFFSETSIZE_PSIZE
  *         @arg @ref LL_DMA_OFFSETSIZE_FIXEDTO4
  */
/* *
  * @brief Set Stream priority level.
  * @rmtoll CR          PL           LL_DMA_SetStreamPriorityLevel
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  Priority This parameter can be one of the following values:
  *         @arg @ref LL_DMA_PRIORITY_LOW
  *         @arg @ref LL_DMA_PRIORITY_MEDIUM
  *         @arg @ref LL_DMA_PRIORITY_HIGH
  *         @arg @ref LL_DMA_PRIORITY_VERYHIGH
  * @retval None
  */
/* *
  * @brief Get Stream priority level.
  * @rmtoll CR          PL           LL_DMA_GetStreamPriorityLevel
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_PRIORITY_LOW
  *         @arg @ref LL_DMA_PRIORITY_MEDIUM
  *         @arg @ref LL_DMA_PRIORITY_HIGH
  *         @arg @ref LL_DMA_PRIORITY_VERYHIGH
  */
/* *
  * @brief Set Number of data to transfer.
  * @rmtoll NDTR          NDT           LL_DMA_SetDataLength
  * @note   This action has no effect if
  *         stream is enabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  NbData Between 0 to 0xFFFFFFFF
  * @retval None
  */
/* *
  * @brief Get Number of data to transfer.
  * @rmtoll NDTR          NDT           LL_DMA_GetDataLength
  * @note   Once the stream is enabled, the return value indicate the
  *         remaining bytes to be transmitted.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Between 0 to 0xFFFFFFFF
  */
/* *
  * @brief Select Channel number associated to the Stream.
  * @rmtoll CR          CHSEL           LL_DMA_SetChannelSelection
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  *         @arg @ref LL_DMA_CHANNEL_8 (*)
  *         @arg @ref LL_DMA_CHANNEL_9 (*)
  *         @arg @ref LL_DMA_CHANNEL_10 (*)
  *         @arg @ref LL_DMA_CHANNEL_11 (*)
  *         @arg @ref LL_DMA_CHANNEL_12 (*)
  *         @arg @ref LL_DMA_CHANNEL_13 (*)
  *         @arg @ref LL_DMA_CHANNEL_14 (*)
  *         @arg @ref LL_DMA_CHANNEL_15 (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  */
/* *
  * @brief Get the Channel number associated to the Stream.
  * @rmtoll CR          CHSEL           LL_DMA_GetChannelSelection
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  *         @arg @ref LL_DMA_CHANNEL_8 (*)
  *         @arg @ref LL_DMA_CHANNEL_9 (*)
  *         @arg @ref LL_DMA_CHANNEL_10 (*)
  *         @arg @ref LL_DMA_CHANNEL_11 (*)
  *         @arg @ref LL_DMA_CHANNEL_12 (*)
  *         @arg @ref LL_DMA_CHANNEL_13 (*)
  *         @arg @ref LL_DMA_CHANNEL_14 (*)
  *         @arg @ref LL_DMA_CHANNEL_15 (*)
  *
  *         (*) value not defined in all devices.
  */
/* *
  * @brief Set Memory burst transfer configuration.
  * @rmtoll CR          MBURST           LL_DMA_SetMemoryBurstxfer
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  Mburst This parameter can be one of the following values:
  *         @arg @ref LL_DMA_MBURST_SINGLE
  *         @arg @ref LL_DMA_MBURST_INC4
  *         @arg @ref LL_DMA_MBURST_INC8
  *         @arg @ref LL_DMA_MBURST_INC16
  * @retval None
  */
/* *
  * @brief Get Memory burst transfer configuration.
  * @rmtoll CR          MBURST           LL_DMA_GetMemoryBurstxfer
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_MBURST_SINGLE
  *         @arg @ref LL_DMA_MBURST_INC4
  *         @arg @ref LL_DMA_MBURST_INC8
  *         @arg @ref LL_DMA_MBURST_INC16
  */
/* *
  * @brief Set  Peripheral burst transfer configuration.
  * @rmtoll CR          PBURST           LL_DMA_SetPeriphBurstxfer
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  Pburst This parameter can be one of the following values:
  *         @arg @ref LL_DMA_PBURST_SINGLE
  *         @arg @ref LL_DMA_PBURST_INC4
  *         @arg @ref LL_DMA_PBURST_INC8
  *         @arg @ref LL_DMA_PBURST_INC16
  * @retval None
  */
/* *
  * @brief Get Peripheral burst transfer configuration.
  * @rmtoll CR          PBURST           LL_DMA_GetPeriphBurstxfer
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_PBURST_SINGLE
  *         @arg @ref LL_DMA_PBURST_INC4
  *         @arg @ref LL_DMA_PBURST_INC8
  *         @arg @ref LL_DMA_PBURST_INC16
  */
/* *
  * @brief Set Current target (only in double buffer mode) to Memory 1 or Memory 0.
  * @rmtoll CR          CT           LL_DMA_SetCurrentTargetMem 
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param CurrentMemory This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CURRENTTARGETMEM0
  *         @arg @ref LL_DMA_CURRENTTARGETMEM1
  * @retval None
  */
/* *
  * @brief Set Current target (only in double buffer mode) to Memory 1 or Memory 0.
  * @rmtoll CR          CT           LL_DMA_GetCurrentTargetMem 
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_CURRENTTARGETMEM0
  *         @arg @ref LL_DMA_CURRENTTARGETMEM1
  */
/* *
  * @brief Enable the double buffer mode.
  * @rmtoll CR          DBM           LL_DMA_EnableDoubleBufferMode
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
/* *
  * @brief Disable the double buffer mode.
  * @rmtoll CR          DBM           LL_DMA_DisableDoubleBufferMode 
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
/* *
  * @brief Get FIFO status.
  * @rmtoll FCR          FS          LL_DMA_GetFIFOStatus
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_FIFOSTATUS_0_25
  *         @arg @ref LL_DMA_FIFOSTATUS_25_50
  *         @arg @ref LL_DMA_FIFOSTATUS_50_75
  *         @arg @ref LL_DMA_FIFOSTATUS_75_100
  *         @arg @ref LL_DMA_FIFOSTATUS_EMPTY
  *         @arg @ref LL_DMA_FIFOSTATUS_FULL
  */
/* *
  * @brief Disable Fifo mode.
  * @rmtoll FCR          DMDIS          LL_DMA_DisableFifoMode
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
/* *
  * @brief Enable Fifo mode.
  * @rmtoll FCR          DMDIS          LL_DMA_EnableFifoMode 
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
/* *
  * @brief Select FIFO threshold.
  * @rmtoll FCR         FTH          LL_DMA_SetFIFOThreshold
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  Threshold This parameter can be one of the following values:
  *         @arg @ref LL_DMA_FIFOTHRESHOLD_1_4
  *         @arg @ref LL_DMA_FIFOTHRESHOLD_1_2
  *         @arg @ref LL_DMA_FIFOTHRESHOLD_3_4
  *         @arg @ref LL_DMA_FIFOTHRESHOLD_FULL
  * @retval None
  */
/* *
  * @brief Get FIFO threshold.
  * @rmtoll FCR         FTH          LL_DMA_GetFIFOThreshold
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_FIFOTHRESHOLD_1_4
  *         @arg @ref LL_DMA_FIFOTHRESHOLD_1_2
  *         @arg @ref LL_DMA_FIFOTHRESHOLD_3_4
  *         @arg @ref LL_DMA_FIFOTHRESHOLD_FULL
  */
/* *
  * @brief Configure the FIFO .
  * @rmtoll FCR         FTH          LL_DMA_ConfigFifo\n
  *         FCR         DMDIS        LL_DMA_ConfigFifo
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  FifoMode This parameter can be one of the following values:
  *         @arg @ref LL_DMA_FIFOMODE_ENABLE
  *         @arg @ref LL_DMA_FIFOMODE_DISABLE
  * @param  FifoThreshold This parameter can be one of the following values:
  *         @arg @ref LL_DMA_FIFOTHRESHOLD_1_4
  *         @arg @ref LL_DMA_FIFOTHRESHOLD_1_2
  *         @arg @ref LL_DMA_FIFOTHRESHOLD_3_4
  *         @arg @ref LL_DMA_FIFOTHRESHOLD_FULL
  * @retval None
  */
/* *
  * @brief Configure the Source and Destination addresses.
  * @note   This API must not be called when the DMA stream is enabled.
  * @rmtoll M0AR        M0A         LL_DMA_ConfigAddresses\n 
  *         PAR         PA          LL_DMA_ConfigAddresses
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  SrcAddress Between 0 to 0xFFFFFFFF
  * @param  DstAddress Between 0 to 0xFFFFFFFF
  * @param  Direction This parameter can be one of the following values:
  *         @arg @ref LL_DMA_DIRECTION_PERIPH_TO_MEMORY
  *         @arg @ref LL_DMA_DIRECTION_MEMORY_TO_PERIPH
  *         @arg @ref LL_DMA_DIRECTION_MEMORY_TO_MEMORY
  * @retval None
  */
/* Direction Memory to Periph */
/* Direction Periph to Memory and Memory to Memory */
/* *
  * @brief  Set the Memory address.
  * @rmtoll M0AR        M0A         LL_DMA_SetMemoryAddress
  * @note   Interface used for direction LL_DMA_DIRECTION_PERIPH_TO_MEMORY or LL_DMA_DIRECTION_MEMORY_TO_PERIPH only.
  * @note   This API must not be called when the DMA channel is enabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  MemoryAddress Between 0 to 0xFFFFFFFF
  * @retval None
  */
/* *
  * @brief  Set the Peripheral address.
  * @rmtoll PAR        PA         LL_DMA_SetPeriphAddress
  * @note   Interface used for direction LL_DMA_DIRECTION_PERIPH_TO_MEMORY or LL_DMA_DIRECTION_MEMORY_TO_PERIPH only.
  * @note   This API must not be called when the DMA channel is enabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  PeriphAddress Between 0 to 0xFFFFFFFF
  * @retval None
  */
/* *
  * @brief  Get the Memory address.
  * @rmtoll M0AR        M0A         LL_DMA_GetMemoryAddress
  * @note   Interface used for direction LL_DMA_DIRECTION_PERIPH_TO_MEMORY or LL_DMA_DIRECTION_MEMORY_TO_PERIPH only.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Between 0 to 0xFFFFFFFF
  */
/* *
  * @brief  Get the Peripheral address.
  * @rmtoll PAR        PA         LL_DMA_GetPeriphAddress
  * @note   Interface used for direction LL_DMA_DIRECTION_PERIPH_TO_MEMORY or LL_DMA_DIRECTION_MEMORY_TO_PERIPH only.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Between 0 to 0xFFFFFFFF
  */
/* *
  * @brief  Set the Memory to Memory Source address.
  * @rmtoll PAR        PA         LL_DMA_SetM2MSrcAddress
  * @note   Interface used for direction LL_DMA_DIRECTION_MEMORY_TO_MEMORY only.
  * @note   This API must not be called when the DMA channel is enabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  MemoryAddress Between 0 to 0xFFFFFFFF
  * @retval None
  */
/* *
  * @brief  Set the Memory to Memory Destination address.
  * @rmtoll M0AR        M0A         LL_DMA_SetM2MDstAddress
  * @note   Interface used for direction LL_DMA_DIRECTION_MEMORY_TO_MEMORY only.
  * @note   This API must not be called when the DMA channel is enabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  MemoryAddress Between 0 to 0xFFFFFFFF
  * @retval None
  */
/* *
  * @brief  Get the Memory to Memory Source address.
  * @rmtoll PAR        PA         LL_DMA_GetM2MSrcAddress
  * @note   Interface used for direction LL_DMA_DIRECTION_MEMORY_TO_MEMORY only.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Between 0 to 0xFFFFFFFF
  */
/* *
  * @brief  Get the Memory to Memory Destination address.
  * @rmtoll M0AR        M0A         LL_DMA_GetM2MDstAddress
  * @note   Interface used for direction LL_DMA_DIRECTION_MEMORY_TO_MEMORY only.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Between 0 to 0xFFFFFFFF
  */
/* *
  * @brief Set Memory 1 address (used in case of Double buffer mode).
  * @rmtoll M1AR        M1A         LL_DMA_SetMemory1Address
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  Address Between 0 to 0xFFFFFFFF
  * @retval None
  */
/* *
  * @brief Get Memory 1 address (used in case of Double buffer mode).
  * @rmtoll M1AR        M1A         LL_DMA_GetMemory1Address
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval Between 0 to 0xFFFFFFFF
  */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EF_FLAG_Management FLAG_Management
  * @{
  */
/* *
  * @brief Get Stream 0 half transfer flag.
  * @rmtoll LISR  HTIF0    LL_DMA_IsActiveFlag_HT0
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 1 half transfer flag.
  * @rmtoll LISR  HTIF1    LL_DMA_IsActiveFlag_HT1
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 2 half transfer flag.
  * @rmtoll LISR  HTIF2    LL_DMA_IsActiveFlag_HT2
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 3 half transfer flag.
  * @rmtoll LISR  HTIF3    LL_DMA_IsActiveFlag_HT3
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 4 half transfer flag.
  * @rmtoll HISR  HTIF4    LL_DMA_IsActiveFlag_HT4
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 5 half transfer flag.
  * @rmtoll HISR  HTIF0    LL_DMA_IsActiveFlag_HT5
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 6 half transfer flag.
  * @rmtoll HISR  HTIF6    LL_DMA_IsActiveFlag_HT6
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 7 half transfer flag.
  * @rmtoll HISR  HTIF7    LL_DMA_IsActiveFlag_HT7
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 0 transfer complete flag.
  * @rmtoll LISR  TCIF0    LL_DMA_IsActiveFlag_TC0
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 1 transfer complete flag.
  * @rmtoll LISR  TCIF1    LL_DMA_IsActiveFlag_TC1
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 2 transfer complete flag.
  * @rmtoll LISR  TCIF2    LL_DMA_IsActiveFlag_TC2
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 3 transfer complete flag.
  * @rmtoll LISR  TCIF3    LL_DMA_IsActiveFlag_TC3
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 4 transfer complete flag.
  * @rmtoll HISR  TCIF4    LL_DMA_IsActiveFlag_TC4
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 5 transfer complete flag.
  * @rmtoll HISR  TCIF0    LL_DMA_IsActiveFlag_TC5
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 6 transfer complete flag.
  * @rmtoll HISR  TCIF6    LL_DMA_IsActiveFlag_TC6
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 7 transfer complete flag.
  * @rmtoll HISR  TCIF7    LL_DMA_IsActiveFlag_TC7
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 0 transfer error flag.
  * @rmtoll LISR  TEIF0    LL_DMA_IsActiveFlag_TE0
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 1 transfer error flag.
  * @rmtoll LISR  TEIF1    LL_DMA_IsActiveFlag_TE1
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 2 transfer error flag.
  * @rmtoll LISR  TEIF2    LL_DMA_IsActiveFlag_TE2
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 3 transfer error flag.
  * @rmtoll LISR  TEIF3    LL_DMA_IsActiveFlag_TE3
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 4 transfer error flag.
  * @rmtoll HISR  TEIF4    LL_DMA_IsActiveFlag_TE4
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 5 transfer error flag.
  * @rmtoll HISR  TEIF0    LL_DMA_IsActiveFlag_TE5
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 6 transfer error flag.
  * @rmtoll HISR  TEIF6    LL_DMA_IsActiveFlag_TE6
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 7 transfer error flag.
  * @rmtoll HISR  TEIF7    LL_DMA_IsActiveFlag_TE7
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 0 direct mode error flag.
  * @rmtoll LISR  DMEIF0    LL_DMA_IsActiveFlag_DME0
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 1 direct mode error flag.
  * @rmtoll LISR  DMEIF1    LL_DMA_IsActiveFlag_DME1
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 2 direct mode error flag.
  * @rmtoll LISR  DMEIF2    LL_DMA_IsActiveFlag_DME2
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 3 direct mode error flag.
  * @rmtoll LISR  DMEIF3    LL_DMA_IsActiveFlag_DME3
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 4 direct mode error flag.
  * @rmtoll HISR  DMEIF4    LL_DMA_IsActiveFlag_DME4
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 5 direct mode error flag.
  * @rmtoll HISR  DMEIF0    LL_DMA_IsActiveFlag_DME5
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 6 direct mode error flag.
  * @rmtoll HISR  DMEIF6    LL_DMA_IsActiveFlag_DME6
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 7 direct mode error flag.
  * @rmtoll HISR  DMEIF7    LL_DMA_IsActiveFlag_DME7
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 0 FIFO error flag.
  * @rmtoll LISR  FEIF0    LL_DMA_IsActiveFlag_FE0
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 1 FIFO error flag.
  * @rmtoll LISR  FEIF1    LL_DMA_IsActiveFlag_FE1
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 2 FIFO error flag.
  * @rmtoll LISR  FEIF2    LL_DMA_IsActiveFlag_FE2
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 3 FIFO error flag.
  * @rmtoll LISR  FEIF3    LL_DMA_IsActiveFlag_FE3
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 4 FIFO error flag.
  * @rmtoll HISR  FEIF4    LL_DMA_IsActiveFlag_FE4
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 5 FIFO error flag.
  * @rmtoll HISR  FEIF0    LL_DMA_IsActiveFlag_FE5
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 6 FIFO error flag.
  * @rmtoll HISR  FEIF6    LL_DMA_IsActiveFlag_FE6
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Get Stream 7 FIFO error flag.
  * @rmtoll HISR  FEIF7    LL_DMA_IsActiveFlag_FE7
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Clear Stream 0 half transfer flag.
  * @rmtoll LIFCR  CHTIF0    LL_DMA_ClearFlag_HT0
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 1 half transfer flag.
  * @rmtoll LIFCR  CHTIF1    LL_DMA_ClearFlag_HT1
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 2 half transfer flag.
  * @rmtoll LIFCR  CHTIF2    LL_DMA_ClearFlag_HT2
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 3 half transfer flag.
  * @rmtoll LIFCR  CHTIF3    LL_DMA_ClearFlag_HT3
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 4 half transfer flag.
  * @rmtoll HIFCR  CHTIF4    LL_DMA_ClearFlag_HT4
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 5 half transfer flag.
  * @rmtoll HIFCR  CHTIF5    LL_DMA_ClearFlag_HT5
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 6 half transfer flag.
  * @rmtoll HIFCR  CHTIF6    LL_DMA_ClearFlag_HT6
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 7 half transfer flag.
  * @rmtoll HIFCR  CHTIF7    LL_DMA_ClearFlag_HT7
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 0 transfer complete flag.
  * @rmtoll LIFCR  CTCIF0    LL_DMA_ClearFlag_TC0
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 1 transfer complete flag.
  * @rmtoll LIFCR  CTCIF1    LL_DMA_ClearFlag_TC1
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 2 transfer complete flag.
  * @rmtoll LIFCR  CTCIF2    LL_DMA_ClearFlag_TC2
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 3 transfer complete flag.
  * @rmtoll LIFCR  CTCIF3    LL_DMA_ClearFlag_TC3
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 4 transfer complete flag.
  * @rmtoll HIFCR  CTCIF4    LL_DMA_ClearFlag_TC4
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 5 transfer complete flag.
  * @rmtoll HIFCR  CTCIF5    LL_DMA_ClearFlag_TC5
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 6 transfer complete flag.
  * @rmtoll HIFCR  CTCIF6    LL_DMA_ClearFlag_TC6
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 7 transfer complete flag.
  * @rmtoll HIFCR  CTCIF7    LL_DMA_ClearFlag_TC7
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 0 transfer error flag.
  * @rmtoll LIFCR  CTEIF0    LL_DMA_ClearFlag_TE0
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 1 transfer error flag.
  * @rmtoll LIFCR  CTEIF1    LL_DMA_ClearFlag_TE1
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 2 transfer error flag.
  * @rmtoll LIFCR  CTEIF2    LL_DMA_ClearFlag_TE2
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 3 transfer error flag.
  * @rmtoll LIFCR  CTEIF3    LL_DMA_ClearFlag_TE3
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 4 transfer error flag.
  * @rmtoll HIFCR  CTEIF4    LL_DMA_ClearFlag_TE4
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 5 transfer error flag.
  * @rmtoll HIFCR  CTEIF5    LL_DMA_ClearFlag_TE5
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 6 transfer error flag.
  * @rmtoll HIFCR  CTEIF6    LL_DMA_ClearFlag_TE6
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 7 transfer error flag.
  * @rmtoll HIFCR  CTEIF7    LL_DMA_ClearFlag_TE7
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 0 direct mode error flag.
  * @rmtoll LIFCR  CDMEIF0    LL_DMA_ClearFlag_DME0
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 1 direct mode error flag.
  * @rmtoll LIFCR  CDMEIF1    LL_DMA_ClearFlag_DME1
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 2 direct mode error flag.
  * @rmtoll LIFCR  CDMEIF2    LL_DMA_ClearFlag_DME2
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 3 direct mode error flag.
  * @rmtoll LIFCR  CDMEIF3    LL_DMA_ClearFlag_DME3
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 4 direct mode error flag.
  * @rmtoll HIFCR  CDMEIF4    LL_DMA_ClearFlag_DME4
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 5 direct mode error flag.
  * @rmtoll HIFCR  CDMEIF5    LL_DMA_ClearFlag_DME5
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 6 direct mode error flag.
  * @rmtoll HIFCR  CDMEIF6    LL_DMA_ClearFlag_DME6
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 7 direct mode error flag.
  * @rmtoll HIFCR  CDMEIF7    LL_DMA_ClearFlag_DME7
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 0 FIFO error flag.
  * @rmtoll LIFCR  CFEIF0    LL_DMA_ClearFlag_FE0
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 1 FIFO error flag.
  * @rmtoll LIFCR  CFEIF1    LL_DMA_ClearFlag_FE1
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 2 FIFO error flag.
  * @rmtoll LIFCR  CFEIF2    LL_DMA_ClearFlag_FE2
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 3 FIFO error flag.
  * @rmtoll LIFCR  CFEIF3    LL_DMA_ClearFlag_FE3
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 4 FIFO error flag.
  * @rmtoll HIFCR  CFEIF4    LL_DMA_ClearFlag_FE4
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 5 FIFO error flag.
  * @rmtoll HIFCR  CFEIF5    LL_DMA_ClearFlag_FE5
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 6 FIFO error flag.
  * @rmtoll HIFCR  CFEIF6    LL_DMA_ClearFlag_FE6
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @brief Clear Stream 7 FIFO error flag.
  * @rmtoll HIFCR  CFEIF7    LL_DMA_ClearFlag_FE7
  * @param  DMAx DMAx Instance
  * @retval None
  */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EF_IT_Management IT_Management
  * @{
  */
/* *
  * @brief Enable Half transfer interrupt.
  * @rmtoll CR        HTIE         LL_DMA_EnableIT_HT
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
/* *
  * @brief Enable Transfer error interrupt.
  * @rmtoll CR        TEIE         LL_DMA_EnableIT_TE
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
/* *
  * @brief Enable Transfer complete interrupt.
  * @rmtoll CR        TCIE         LL_DMA_EnableIT_TC
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
/* *
  * @brief Enable Direct mode error interrupt.
  * @rmtoll CR        DMEIE         LL_DMA_EnableIT_DME
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
/* *
  * @brief Enable FIFO error interrupt.
  * @rmtoll FCR        FEIE         LL_DMA_EnableIT_FE
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
/* *
  * @brief Disable Half transfer interrupt.
  * @rmtoll CR        HTIE         LL_DMA_DisableIT_HT
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
/* *
  * @brief Disable Transfer error interrupt.
  * @rmtoll CR        TEIE         LL_DMA_DisableIT_TE
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
/* *
  * @brief Disable Transfer complete interrupt.
  * @rmtoll CR        TCIE         LL_DMA_DisableIT_TC
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
/* *
  * @brief Disable Direct mode error interrupt.
  * @rmtoll CR        DMEIE         LL_DMA_DisableIT_DME
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
/* *
  * @brief Disable FIFO error interrupt.
  * @rmtoll FCR        FEIE         LL_DMA_DisableIT_FE
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval None
  */
/* *
  * @brief Check if Half transfer interrup is enabled.
  * @rmtoll CR        HTIE         LL_DMA_IsEnabledIT_HT
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Check if Transfer error nterrup is enabled.
  * @rmtoll CR        TEIE         LL_DMA_IsEnabledIT_TE
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Check if Transfer complete interrup is enabled.
  * @rmtoll CR        TCIE         LL_DMA_IsEnabledIT_TC
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Check if Direct mode error interrupt is enabled.
  * @rmtoll CR        DMEIE         LL_DMA_IsEnabledIT_DME
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief Check if FIFO error interrup is enabled.
  * @rmtoll FCR        FEIE         LL_DMA_IsEnabledIT_FE
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @retval State of bit (1 or 0).
  */
/* *
  * @}
  */
/* * @defgroup DMA_LL_EF_Init Initialization and de-initialization functions
  * @{
  */
/* *
  * @brief  Set each @ref LL_DMA_InitTypeDef field to default value.
  * @param  DMA_InitStruct Pointer to a @ref LL_DMA_InitTypeDef structure.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_DMA_StructInit(mut DMA_InitStruct:
                                               *mut LL_DMA_InitTypeDef) {
    /* Set DMA_InitStruct fields to default values */
    (*DMA_InitStruct).PeriphOrM2MSrcAddress = 0 as libc::c_uint;
    (*DMA_InitStruct).MemoryOrM2MDstAddress = 0 as libc::c_uint;
    (*DMA_InitStruct).Direction = 0 as libc::c_uint;
    (*DMA_InitStruct).Mode = 0 as libc::c_uint;
    (*DMA_InitStruct).PeriphOrM2MSrcIncMode = 0 as libc::c_uint;
    (*DMA_InitStruct).MemoryOrM2MDstIncMode = 0 as libc::c_uint;
    (*DMA_InitStruct).PeriphOrM2MSrcDataSize = 0 as libc::c_uint;
    (*DMA_InitStruct).MemoryOrM2MDstDataSize = 0 as libc::c_uint;
    (*DMA_InitStruct).NbData = 0 as libc::c_uint;
    (*DMA_InitStruct).Channel = 0 as libc::c_uint;
    (*DMA_InitStruct).Priority = 0 as libc::c_uint;
    (*DMA_InitStruct).FIFOMode = 0 as libc::c_uint;
    (*DMA_InitStruct).FIFOThreshold = 0 as libc::c_uint;
    (*DMA_InitStruct).MemBurst = 0 as libc::c_uint;
    (*DMA_InitStruct).PeriphBurst = 0 as libc::c_uint;
}
unsafe extern "C" fn run_static_initializers() {
    STREAM_OFFSET_TAB =
        [(0x40000000 as
              libc::c_uint).wrapping_add(0x20000 as
                                             libc::c_uint).wrapping_add(0x6000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x10
                                                                                                           as
                                                                                                           libc::c_uint).wrapping_sub((0x40000000
                                                                                                                                           as
                                                                                                                                           libc::c_uint).wrapping_add(0x20000
                                                                                                                                                                          as
                                                                                                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                                                                                                         as
                                                                                                                                                                                                         libc::c_uint))
             as uint8_t,
         (0x40000000 as
              libc::c_uint).wrapping_add(0x20000 as
                                             libc::c_uint).wrapping_add(0x6000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x28
                                                                                                           as
                                                                                                           libc::c_uint).wrapping_sub((0x40000000
                                                                                                                                           as
                                                                                                                                           libc::c_uint).wrapping_add(0x20000
                                                                                                                                                                          as
                                                                                                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                                                                                                         as
                                                                                                                                                                                                         libc::c_uint))
             as uint8_t,
         (0x40000000 as
              libc::c_uint).wrapping_add(0x20000 as
                                             libc::c_uint).wrapping_add(0x6000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x40
                                                                                                           as
                                                                                                           libc::c_uint).wrapping_sub((0x40000000
                                                                                                                                           as
                                                                                                                                           libc::c_uint).wrapping_add(0x20000
                                                                                                                                                                          as
                                                                                                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                                                                                                         as
                                                                                                                                                                                                         libc::c_uint))
             as uint8_t,
         (0x40000000 as
              libc::c_uint).wrapping_add(0x20000 as
                                             libc::c_uint).wrapping_add(0x6000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x58
                                                                                                           as
                                                                                                           libc::c_uint).wrapping_sub((0x40000000
                                                                                                                                           as
                                                                                                                                           libc::c_uint).wrapping_add(0x20000
                                                                                                                                                                          as
                                                                                                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                                                                                                         as
                                                                                                                                                                                                         libc::c_uint))
             as uint8_t,
         (0x40000000 as
              libc::c_uint).wrapping_add(0x20000 as
                                             libc::c_uint).wrapping_add(0x6000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x70
                                                                                                           as
                                                                                                           libc::c_uint).wrapping_sub((0x40000000
                                                                                                                                           as
                                                                                                                                           libc::c_uint).wrapping_add(0x20000
                                                                                                                                                                          as
                                                                                                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                                                                                                         as
                                                                                                                                                                                                         libc::c_uint))
             as uint8_t,
         (0x40000000 as
              libc::c_uint).wrapping_add(0x20000 as
                                             libc::c_uint).wrapping_add(0x6000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x88
                                                                                                           as
                                                                                                           libc::c_uint).wrapping_sub((0x40000000
                                                                                                                                           as
                                                                                                                                           libc::c_uint).wrapping_add(0x20000
                                                                                                                                                                          as
                                                                                                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                                                                                                         as
                                                                                                                                                                                                         libc::c_uint))
             as uint8_t,
         (0x40000000 as
              libc::c_uint).wrapping_add(0x20000 as
                                             libc::c_uint).wrapping_add(0x6000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0xa0
                                                                                                           as
                                                                                                           libc::c_uint).wrapping_sub((0x40000000
                                                                                                                                           as
                                                                                                                                           libc::c_uint).wrapping_add(0x20000
                                                                                                                                                                          as
                                                                                                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                                                                                                         as
                                                                                                                                                                                                         libc::c_uint))
             as uint8_t,
         (0x40000000 as
              libc::c_uint).wrapping_add(0x20000 as
                                             libc::c_uint).wrapping_add(0x6000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0xb8
                                                                                                           as
                                                                                                           libc::c_uint).wrapping_sub((0x40000000
                                                                                                                                           as
                                                                                                                                           libc::c_uint).wrapping_add(0x20000
                                                                                                                                                                          as
                                                                                                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                                                                                                         as
                                                                                                                                                                                                         libc::c_uint))
             as uint8_t]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* USE_FULL_LL_DRIVER */
/* *
  * @}
  */
/* DMA1 || DMA2 */
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
