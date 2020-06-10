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
                                    !((0xf as libc::c_uint) <<
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
