use core;
use libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct SYSCFG_TypeDef {
    pub CFGR1: uint32_t,
    pub RCR: uint32_t,
    pub EXTICR: [uint32_t; 4],
    pub CFGR2: uint32_t,
    pub RESERVED0: uint32_t,
    pub RESERVED1: uint32_t,
    pub RESERVED2: uint32_t,
    pub RESERVED4: uint32_t,
    pub RESERVED5: uint32_t,
    pub RESERVED6: uint32_t,
    pub RESERVED7: uint32_t,
    pub RESERVED8: uint32_t,
    pub RESERVED9: uint32_t,
    pub RESERVED10: uint32_t,
    pub RESERVED11: uint32_t,
    pub RESERVED12: uint32_t,
    pub RESERVED13: uint32_t,
    pub CFGR3: uint32_t,
}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup SYSCFG_Private_Functions
  * @{
  */
/* * @defgroup SYSCFG_Group1 SYSCFG Initialization and Configuration functions
 *  @brief   SYSCFG Initialization and Configuration functions 
 *
@verbatim
 ===============================================================================
         ##### SYSCFG Initialization and Configuration functions #####
 ===============================================================================

@endverbatim
  * @{
  */
/* *
  * @brief  Deinitializes the SYSCFG registers to their default reset values.
  * @param  None
  * @retval None
  * @note   MEM_MODE bits are not affected by APB reset.
  *         MEM_MODE bits took the value from the user option bytes.
  */
#[no_mangle]
pub unsafe extern "C" fn SYSCFG_DeInit() {
    /* Reset SYSCFG_CFGR1 register to reset value without affecting MEM_MODE bits */
    let ref mut fresh0 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as *mut SYSCFG_TypeDef)).CFGR1;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & 0x3i32 as uint32_t) as
                                    uint32_t as uint32_t);
    /* Set FPU Interrupt Enable bits to default value */
    let ref mut fresh1 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as *mut SYSCFG_TypeDef)).CFGR1;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x7c000000i32 as libc::c_uint) as
                                    uint32_t as uint32_t);
    /* Reset RAM Write protection bits to default value */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x10000i32
                                                                        as
                                                                        libc::c_uint).wrapping_add(0i32
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut SYSCFG_TypeDef)).RCR as
                                    *mut uint32_t, 0i32 as uint32_t);
    /* Set EXTICRx registers to reset value */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x10000i32
                                                                        as
                                                                        libc::c_uint).wrapping_add(0i32
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut SYSCFG_TypeDef)).EXTICR[0]
                                    as *mut uint32_t, 0i32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x10000i32
                                                                        as
                                                                        libc::c_uint).wrapping_add(0i32
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut SYSCFG_TypeDef)).EXTICR[1]
                                    as *mut uint32_t, 0i32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x10000i32
                                                                        as
                                                                        libc::c_uint).wrapping_add(0i32
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut SYSCFG_TypeDef)).EXTICR[2]
                                    as *mut uint32_t, 0i32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x10000i32
                                                                        as
                                                                        libc::c_uint).wrapping_add(0i32
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut SYSCFG_TypeDef)).EXTICR[3]
                                    as *mut uint32_t, 0i32 as uint32_t);
    /* Set CFGR2 register to reset value */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x10000i32
                                                                        as
                                                                        libc::c_uint).wrapping_add(0i32
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut SYSCFG_TypeDef)).CFGR2 as
                                    *mut uint32_t, 0i32 as uint32_t);
    /* Set CFGR3 register to reset value */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x10000i32
                                                                        as
                                                                        libc::c_uint).wrapping_add(0i32
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut SYSCFG_TypeDef)).CFGR3 as
                                    *mut uint32_t, 0i32 as uint32_t);
}
/* *
  * @brief  Configures the memory mapping at address 0x00000000.
  * @param  SYSCFG_MemoryRemap: selects the memory remapping.
  *   This parameter can be one of the following values:
  *     @arg SYSCFG_MemoryRemap_Flash: Main Flash memory mapped at 0x00000000  
  *     @arg SYSCFG_MemoryRemap_SystemMemory: System Flash memory mapped at 0x00000000
  *     @arg SYSCFG_MemoryRemap_SRAM: Embedded SRAM mapped at 0x00000000
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SYSCFG_MemoryRemapConfig(mut SYSCFG_MemoryRemap:
                                                      uint32_t) {
    let mut tmpcfgr1: uint32_t = 0i32 as uint32_t;
    /* Check the parameter */
    /* Get CFGR1 register value */
    tmpcfgr1 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as *mut SYSCFG_TypeDef)).CFGR1;
    /* Clear MEM_MODE bits */
    tmpcfgr1 &= !(0x3i32 as uint32_t);
    /* Set the new MEM_MODE bits value */
    tmpcfgr1 |= SYSCFG_MemoryRemap;
    /* Set CFGR1 register with the new memory remap configuration */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x10000i32
                                                                        as
                                                                        libc::c_uint).wrapping_add(0i32
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut SYSCFG_TypeDef)).CFGR1 as
                                    *mut uint32_t, tmpcfgr1);
}
/* *
  * @brief  Configures the DMA channels remapping.
  * @param  SYSCFG_DMARemap: selects the DMA channels remap.
  *   This parameter can be one of the following values:
  *     @arg SYSCFG_DMARemap_TIM17: Remap TIM17 DMA requests from DMA1 channel1 to channel2
  *     @arg SYSCFG_DMARemap_TIM16: Remap TIM16 DMA requests from DMA1 channel3 to channel4
  *     @arg SYSCFG_DMARemap_TIM6DAC1Ch1: Remap TIM6/DAC1 DMA requests from DMA2 channel 3 to DMA1 channel 3
  *     @arg SYSCFG_DMARemap_TIM7DAC1Ch2: Remap TIM7/DAC2 DMA requests from DMA2 channel 4 to DMA1 channel 4
  *     @arg SYSCFG_DMARemap_ADC2ADC4: Remap ADC2 and ADC4 DMA requests from DMA2 channel1/channel3 to channel3/channel4
  *     @arg SYSCFG_DMARemap_DAC2Ch1: Remap DAC2 DMA requests to DMA1 channel5
  *     @arg SYSCFG_DMARemapCh2_SPI1_RX: Remap SPI1 RX DMA1 CH2 requests
  *     @arg SYSCFG_DMARemapCh4_SPI1_RX: Remap SPI1 RX DMA CH4 requests        
  *     @arg SYSCFG_DMARemapCh6_SPI1_RX: Remap SPI1 RX DMA CH6 requests       
  *     @arg SYSCFG_DMARemapCh3_SPI1_TX: Remap SPI1 TX DMA CH2 requests      
  *     @arg SYSCFG_DMARemapCh5_SPI1_TX: Remap SPI1 TX DMA CH5 requests       
  *     @arg SYSCFG_DMARemapCh7_SPI1_TX: Remap SPI1 TX DMA CH7 requests       
  *     @arg SYSCFG_DMARemapCh7_I2C1_RX: Remap I2C1 RX DMA CH7 requests
  *     @arg SYSCFG_DMARemapCh3_I2C1_RX: Remap I2C1 RX DMA CH3 requests       
  *     @arg SYSCFG_DMARemapCh5_I2C1_RX: Remap I2C1 RX DMA CH5 requests      
  *     @arg SYSCFG_DMARemapCh6_I2C1_TX: Remap I2C1 TX DMA CH6 requests       
  *     @arg SYSCFG_DMARemapCh2_I2C1_TX: Remap I2C1 TX DMA CH2 requests       
  *     @arg SYSCFG_DMARemapCh4_I2C1_TX: Remap I2C1 TX DMA CH4 requests   
  *     @arg SYSCFG_DMARemapCh4_ADC2: Remap ADC2 DMA1 Ch4 requests    
  *     @arg SYSCFG_DMARemapCh2_ADC2: Remap ADC2 DMA1 Ch2 requests
  * @param  NewState: new state of the DMA channel remapping. 
  *         This parameter can be: Enable or Disable.
  * @note   When enabled, DMA channel of the selected peripheral is remapped
  * @note   When disabled, Default DMA channel is mapped to the selected peripheral
  * @note
  *           By default TIM17 DMA requests is mapped to channel 1
  *           use SYSCFG_DMAChannelRemapConfig(SYSCFG_DMARemap_TIM17, Enable)
  *           to remap TIM17 DMA requests to DMA1 channel 2
  *           use SYSCFG_DMAChannelRemapConfig(SYSCFG_DMARemap_TIM17, Disable)
  *           to map TIM17 DMA requests to DMA1 channel 1 (default mapping)
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SYSCFG_DMAChannelRemapConfig(mut SYSCFG_DMARemap:
                                                          uint32_t,
                                                      mut NewState:
                                                          FunctionalState) {
    /* Check the parameters */
    if SYSCFG_DMARemap & 0x80000000u32 != 0x80000000u32 {
        if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint
           {
            /* Remap the DMA channel */
            let ref mut fresh2 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x10000i32 as
                                                   libc::c_uint).wrapping_add(0i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut SYSCFG_TypeDef)).CFGR1;
            ::core::ptr::write_volatile(fresh2,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             SYSCFG_DMARemap) as uint32_t as
                                            uint32_t)
        } else {
            /* use the default DMA channel mapping */
            let ref mut fresh3 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x10000i32 as
                                                   libc::c_uint).wrapping_add(0i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut SYSCFG_TypeDef)).CFGR1;
            ::core::ptr::write_volatile(fresh3,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !SYSCFG_DMARemap) as uint32_t as
                                            uint32_t)
        }
    } else if NewState as libc::c_uint !=
                  DISABLE as libc::c_int as libc::c_uint {
        /* Remap the DMA channel */
        let ref mut fresh4 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut SYSCFG_TypeDef)).CFGR3;
        ::core::ptr::write_volatile(fresh4,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | SYSCFG_DMARemap) as
                                        uint32_t as uint32_t)
    } else {
        /* use the default DMA channel mapping */
        let ref mut fresh5 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut SYSCFG_TypeDef)).CFGR3;
        ::core::ptr::write_volatile(fresh5,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !SYSCFG_DMARemap)
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Configures the remapping capabilities of DAC/TIM triggers.
  * @param  SYSCFG_TriggerRemap: selects the trigger to be remapped.
  *   This parameter can be one of the following values:
  *     @arg SYSCFG_TriggerRemap_DACTIM3: Remap DAC trigger from TIM8 to TIM3
  *     @arg SYSCFG_TriggerRemap_TIM1TIM17: Remap TIM1 ITR3 from TIM4 TRGO to TIM17 OC
  *     @arg SYSCFG_TriggerRemap_DACHRTIM1_TRIG1: Remap DAC trigger to HRTIM1 TRIG1
  *     @arg SYSCFG_TriggerRemap_DACHRTIM1_TRIG2: Remap DAC trigger to HRTIM1 TRIG2    
  * @param  NewState: new state of the trigger mapping. 
  *         This parameter can be: ENABLE or DISABLE.
  * @note   ENABLE:  Enable fast mode plus driving capability for selected pin
  * @note   DISABLE: Disable fast mode plus driving capability for selected pin
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SYSCFG_TriggerRemapConfig(mut SYSCFG_TriggerRemap:
                                                       uint32_t,
                                                   mut NewState:
                                                       FunctionalState) {
    /* Check the parameters */
    if SYSCFG_TriggerRemap & 0x80000000u32 != 0x80000000u32 {
        if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint
           {
            /* Remap the trigger */
            let ref mut fresh6 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x10000i32 as
                                                   libc::c_uint).wrapping_add(0i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut SYSCFG_TypeDef)).CFGR1;
            ::core::ptr::write_volatile(fresh6,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             SYSCFG_TriggerRemap) as uint32_t
                                            as uint32_t)
        } else {
            /* Use the default trigger mapping */
            let ref mut fresh7 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x10000i32 as
                                                   libc::c_uint).wrapping_add(0i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut SYSCFG_TypeDef)).CFGR1;
            ::core::ptr::write_volatile(fresh7,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !SYSCFG_TriggerRemap) as uint32_t
                                            as uint32_t)
        }
    } else if NewState as libc::c_uint !=
                  DISABLE as libc::c_int as libc::c_uint {
        /* Remap the trigger */
        let ref mut fresh8 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut SYSCFG_TypeDef)).CFGR3;
        ::core::ptr::write_volatile(fresh8,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         SYSCFG_TriggerRemap) as uint32_t as
                                        uint32_t)
    } else {
        /* Use the default trigger mapping */
        let ref mut fresh9 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut SYSCFG_TypeDef)).CFGR3;
        ::core::ptr::write_volatile(fresh9,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !SYSCFG_TriggerRemap) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Configures the remapping capabilities of encoder mode.
  * @ note This feature implement the so-called M/T method for measuring speed
  *        and position using quadrature encoders.  
  * @param  SYSCFG_EncoderRemap: selects the remap option for encoder mode.
  *   This parameter can be one of the following values:
  *     @arg SYSCFG_EncoderRemap_No: No remap
  *     @arg SYSCFG_EncoderRemap_TIM2: Timer 2 IC1 and IC2 connected to TIM15 IC1 and IC2
  *     @arg SYSCFG_EncoderRemap_TIM3: Timer 3 IC1 and IC2 connected to TIM15 IC1 and IC2
  *     @arg SYSCFG_EncoderRemap_TIM4: Timer 4 IC1 and IC2 connected to TIM15 IC1 and IC2
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SYSCFG_EncoderRemapConfig(mut SYSCFG_EncoderRemap:
                                                       uint32_t) {
    /* Check the parameter */
    /* Reset the encoder mode remapping bits */
    let ref mut fresh10 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as *mut SYSCFG_TypeDef)).CFGR1;
    ::core::ptr::write_volatile(fresh10,
                                (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xc00000i32 as uint32_t)) as uint32_t
                                    as uint32_t);
    /* Set the selected configuration */
    let ref mut fresh11 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as *mut SYSCFG_TypeDef)).CFGR1;
    ::core::ptr::write_volatile(fresh11,
                                (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | SYSCFG_EncoderRemap) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Remaps the USB interrupt lines.
  * @param  NewState: new state of the mapping of USB interrupt lines. 
  *         This parameter can be:
  * @param  ENABLE: Remap the USB interrupt line as following:
  *         @arg  USB Device High Priority (USB_HP) interrupt mapped to line 74.
  *         @arg  USB Device Low Priority (USB_LP) interrupt mapped to line 75.
  *         @arg  USB Wakeup Interrupt (USB_WKUP) interrupt mapped to line 76.
  * @param  DISABLE: Use the default USB interrupt line:
  *         @arg  USB Device High Priority (USB_HP) interrupt mapped to line 19.
  *         @arg  USB Device Low Priority (USB_LP) interrupt mapped to line 20.
  *         @arg  USB Wakeup Interrupt (USB_WKUP) interrupt mapped to line 42.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SYSCFG_USBInterruptLineRemapCmd(mut NewState:
                                                             FunctionalState) {
    /* Check the parameter */
    /* Remap the USB interupt lines */
    ::core::ptr::write_volatile((0x42000000i32 as
                                     uint32_t).wrapping_add((0x40000000i32 as
                                                                 uint32_t).wrapping_add(0x10000i32
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0i32
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000i32
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0i32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint).wrapping_mul(32i32
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0x5i32
                                                                                                                                                                                                                                                     *
                                                                                                                                                                                                                                                     4i32)
                                                                                                                                                                                                                                                    as
                                                                                                                                                                                                                                                    libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Configures the I2C fast mode plus driving capability.
  * @param  SYSCFG_I2CFastModePlus: selects the pin.
  *   This parameter can be one of the following values:
  *     @arg SYSCFG_I2CFastModePlus_PB6: Configure fast mode plus driving capability for PB6
  *     @arg SYSCFG_I2CFastModePlus_PB7: Configure fast mode plus driving capability for PB7
  *     @arg SYSCFG_I2CFastModePlus_PB8: Configure fast mode plus driving capability for PB8
  *     @arg SYSCFG_I2CFastModePlus_PB9: Configure fast mode plus driving capability for PB9
  *     @arg SYSCFG_I2CFastModePlus_I2C1: Configure fast mode plus driving capability for I2C1 pins
  *     @arg SYSCFG_I2CFastModePlus_I2C2: Configure fast mode plus driving capability for I2C2 pins
  * @param  NewState: new state of the DMA channel remapping. 
  *         This parameter can be:
  *     @arg ENABLE: Enable fast mode plus driving capability for selected I2C pin
  *     @arg DISABLE: Disable fast mode plus driving capability for selected I2C pin
  * @note  For I2C1, fast mode plus driving capability can be enabled on all selected
  *        I2C1 pins using SYSCFG_I2CFastModePlus_I2C1 parameter or independently
  *        on each one of the following pins PB6, PB7, PB8 and PB9.
  * @note  For remaing I2C1 pins (PA14, PA15...) fast mode plus driving capability
  *        can be enabled only by using SYSCFG_I2CFastModePlus_I2C1 parameter.
  * @note  For all I2C2 pins fast mode plus driving capability can be enabled
  *        only by using SYSCFG_I2CFastModePlus_I2C2 parameter.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SYSCFG_I2CFastModePlusConfig(mut SYSCFG_I2CFastModePlus:
                                                          uint32_t,
                                                      mut NewState:
                                                          FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable fast mode plus driving capability for selected I2C pin */
        let ref mut fresh12 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut SYSCFG_TypeDef)).CFGR1;
        ::core::ptr::write_volatile(fresh12,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         SYSCFG_I2CFastModePlus) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable fast mode plus driving capability for selected I2C pin */
        let ref mut fresh13 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut SYSCFG_TypeDef)).CFGR1;
        ::core::ptr::write_volatile(fresh13,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !SYSCFG_I2CFastModePlus) as uint32_t
                                        as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the selected SYSCFG interrupts.
  * @param  SYSCFG_IT: specifies the SYSCFG interrupt sources to be enabled or disabled.
  *   This parameter can be one of the following values:
  *     @arg SYSCFG_IT_IXC: Inexact Interrupt
  *     @arg SYSCFG_IT_IDC: Input denormal Interrupt
  *     @arg SYSCFG_IT_OFC: Overflow Interrupt
  *     @arg SYSCFG_IT_UFC: Underflow Interrupt
  *     @arg SYSCFG_IT_DZC: Divide-by-zero Interrupt
  *     @arg SYSCFG_IT_IOC: Invalid operation Interrupt
  * @param  NewState: new state of the specified SYSCFG interrupts.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SYSCFG_ITConfig(mut SYSCFG_IT: uint32_t,
                                         mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected SYSCFG interrupts */
        let ref mut fresh14 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut SYSCFG_TypeDef)).CFGR1;
        ::core::ptr::write_volatile(fresh14,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh14
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | SYSCFG_IT) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the selected SYSCFG interrupts */
        let ref mut fresh15 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut SYSCFG_TypeDef)).CFGR1;
        ::core::ptr::write_volatile(fresh15,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh15
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !SYSCFG_IT) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Selects the GPIO pin used as EXTI Line.
  * @param  EXTI_PortSourceGPIOx : selects the GPIO port to be used as source 
  *                                for EXTI lines where x can be (A, B, C, D, E or F).
  * @param  EXTI_PinSourcex: specifies the EXTI line to be configured.
  *         This parameter can be EXTI_PinSourcex where x can be (0..15)
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SYSCFG_EXTILineConfig(mut EXTI_PortSourceGPIOx:
                                                   uint8_t,
                                               mut EXTI_PinSourcex: uint8_t) {
    let mut tmp: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmp =
        (0xfi32 as uint32_t) <<
            0x4i32 *
                (EXTI_PinSourcex as libc::c_int &
                     0x3i32 as uint8_t as libc::c_int);
    let ref mut fresh16 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as
               *mut SYSCFG_TypeDef)).EXTICR[(EXTI_PinSourcex as libc::c_int >>
                                                 0x2i32) as usize];
    ::core::ptr::write_volatile(fresh16,
                                (::core::ptr::read_volatile::<uint32_t>(fresh16
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !tmp) as uint32_t as
                                    uint32_t);
    let ref mut fresh17 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as
               *mut SYSCFG_TypeDef)).EXTICR[(EXTI_PinSourcex as libc::c_int >>
                                                 0x2i32) as usize];
    ::core::ptr::write_volatile(fresh17,
                                (::core::ptr::read_volatile::<uint32_t>(fresh17
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (EXTI_PortSourceGPIOx as uint32_t) <<
                                         0x4i32 *
                                             (EXTI_PinSourcex as libc::c_int &
                                                  0x3i32 as uint8_t as
                                                      libc::c_int)) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Connects the selected parameter to the break input of TIM1.
  * @note   The selected configuration is locked and can be unlocked by system reset
  * @param  SYSCFG_Break: selects the configuration to be connected to break
  *         input of TIM1
  *   This parameter can be any combination of the following values:
  *     @arg SYSCFG_Break_PVD: PVD interrupt is connected to the break input of TIM1.
  *     @arg SYSCFG_Break_SRAMParity: SRAM Parity error is connected to the break input of TIM1.
  *     @arg SYSCFG_Break_HardFault: Lockup output of CortexM4 is connected to the break input of TIM1.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SYSCFG_BreakConfig(mut SYSCFG_Break: uint32_t) {
    /* Check the parameter */
    let ref mut fresh18 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as *mut SYSCFG_TypeDef)).CFGR2;
    ::core::ptr::write_volatile(fresh18,
                                (::core::ptr::read_volatile::<uint32_t>(fresh18
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | SYSCFG_Break) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Disables the parity check on RAM.
  * @note   Disabling the parity check on RAM locks the configuration bit.
  *         To re-enable the parity check on RAM perform a system reset.  
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SYSCFG_BypassParityCheckDisable() {
    /* Disable the adddress parity check on RAM */
    ::core::ptr::write_volatile((0x42000000i32 as
                                     uint32_t).wrapping_add((0x40000000i32 as
                                                                 uint32_t).wrapping_add(0x10000i32
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0i32
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000i32
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0x18i32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint).wrapping_mul(32i32
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0x4i32
                                                                                                                                                                                                                                                     *
                                                                                                                                                                                                                                                     4i32)
                                                                                                                                                                                                                                                    as
                                                                                                                                                                                                                                                    libc::c_uint)
                                    as *mut uint32_t, 0x1i32 as uint32_t);
}
/* *
  * @brief  Enables the ICODE SRAM write protection.
  * @note   Enabling the ICODE SRAM write protection locks the configuration bit.
  *         To disable the ICODE SRAM write protection perform a system reset.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SYSCFG_SRAMWRPEnable(mut SYSCFG_SRAMWRP: uint32_t) {
    /* Check the parameter */
    /* Enable the write-protection on the selected ICODE SRAM page */
    let ref mut fresh19 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as *mut SYSCFG_TypeDef)).RCR;
    ::core::ptr::write_volatile(fresh19,
                                (::core::ptr::read_volatile::<uint32_t>(fresh19
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | SYSCFG_SRAMWRP) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Checks whether the specified SYSCFG flag is set or not.
  * @param  SYSCFG_Flag: specifies the SYSCFG flag to check. 
  *   This parameter can be one of the following values:
  *     @arg SYSCFG_FLAG_PE: SRAM parity error flag.
  * @retval The new state of SYSCFG_Flag (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn SYSCFG_GetFlagStatus(mut SYSCFG_Flag: uint32_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameter */
    /* Check the status of the specified SPI flag */
    if (*((0x40000000i32 as
               uint32_t).wrapping_add(0x10000i32 as
                                          libc::c_uint).wrapping_add(0i32 as
                                                                         libc::c_uint)
              as *mut SYSCFG_TypeDef)).CFGR2 & 0x100i32 as uint32_t !=
           RESET as libc::c_int as uint32_t {
        /* SYSCFG_Flag is set */
        bitstatus = SET
    } else {
        /* SYSCFG_Flag is reset */
        bitstatus = RESET
    }
    /* Return the SYSCFG_Flag status */
    return bitstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f30x_syscfg.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the SYSCFG firmware 
  *          library.
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
/* !< Define to prevent recursive inclusion -----------------------------------*/
/* !< Includes ----------------------------------------------------------------*/
/* * @addtogroup STM32F30x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup SYSCFG
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* * @defgroup SYSCFG_Exported_Constants
  * @{
  */
/* * @defgroup SYSCFG_EXTI_Port_Sources 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SYSCFG_EXTI_Pin_sources 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SYSCFG_Memory_Remap_Config 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SYSCFG_DMA_Remap_Config 
  * @{
  */
/* !< Remap TIM17 DMA requests from channel1 to channel2 */
/* !< Remap TIM16 DMA requests from channel3 to channel4 */
/* !< Remap ADC2 and ADC4 DMA requests */
/* Remap TIM6/DAC1 Ch1 DMA requests */
/* Remap TIM7/DAC1 Ch2 DMA requests */
/* Remap DAC2 Ch1 DMA requests */
/* Remap SPI1 RX DMA CH2 requests */
/* Remap SPI1 RX DMA CH4 requests */
/* Remap SPI1 RX DMA CH6 requests */
/* Remap SPI1 TX DMA CH2 requests */
/* Remap SPI1 TX DMA CH5 requests */
/* Remap SPI1 TX DMA CH7 requests */
/* Remap I2C1 RX DMA CH7 requests */
/* Remap I2C1 RX DMA CH3 requests */
/* Remap I2C1 RX DMA CH5 requests */
/* Remap I2C1 TX DMA CH6 requests */
/* Remap I2C1 TX DMA CH2 requests */
/* Remap I2C1 TX DMA CH4 requests */
/* Remap ADC2 DMA1 Ch4 requests */
/* Remap ADC2 DMA1 Ch2 requests */
/* SYSCFG_DMA_Remap_Legacy */
/* !< Remap TIM6/DAC1 DMA requests */
/* !< Remap TIM7/DAC2 DMA requests */
/* *
  * @}
  */
/* * @defgroup SYSCFG_Trigger_Remap_Config 
  * @{
  */
/* !< Remap DAC trigger to TIM3 */
/* !< Remap TIM1 ITR3 to TIM17 OC */
/* !< Remap DAC trigger to HRTIM1 TRIG1 */
/* !< Remap DAC trigger to HRTIM1 TRIG2 */
/* *
  * @}
  */
/* * @defgroup SYSCFG_EncoderRemap_Config 
  * @{
  */
/* !< No redirection */
/* !< Timer 2 IC1 and IC2 connected to TIM15 IC1 and IC2 */
/* !< Timer 3 IC1 and IC2 connected to TIM15 IC1 and IC2 */
/* !< Timer 4 IC1 and IC2 connected to TIM15 IC1 and IC2 */
/* *
  * @}
  */
/* * @defgroup SYSCFG_I2C_FastModePlus_Config 
  * @{
  */
/* !< Enable Fast Mode Plus on PB6 */
/* !< Enable Fast Mode Plus on PB7 */
/* !< Enable Fast Mode Plus on PB8 */
/* !< Enable Fast Mode Plus on PB9 */
/* !< Enable Fast Mode Plus on I2C1 pins */
/* !< Enable Fast Mode Plus on I2C2 pins */
/* *
  * @}
  */
/* * @defgroup SYSCFG_FPU_Interrupt_Config 
  * @{
  */
/* !< Inexact Interrupt enable (interrupt disabled by default) */
/* !< Input denormal Interrupt enable */
/* !< Overflow Interrupt enable */
/* !< Underflow Interrupt enable */
/* !< Divide-by-zero Interrupt enable */
/* !< Invalid operation Interrupt enable */
/* *
  * @}
  */
/* * @defgroup SYSCFG_Lock_Config
  * @{
  */
/* !< Enables and locks the PVD connection with TIM1/8/15/16/17 Break Input and also the PVD_EN and PVDSEL[2:0] bits of the Power Control Interface */
/* !< Enables and locks the SRAM_PARITY error signal with Break Input of TIM1/8/15/16/17 */
/* !< Enables and locks the LOCKUP output of CortexM4 with Break Input of TIM1/8/15/16/17 */
/* *
  * @}
  */
/* * @defgroup SYSCFG_SRAMWRP_Config
  * @{
  */
/* !< ICODE SRAM Write protection page 0 */
/* !< ICODE SRAM Write protection page 1 */
/* !< ICODE SRAM Write protection page 2 */
/* !< ICODE SRAM Write protection page 3 */
/* !< ICODE SRAM Write protection page 4 */
/* !< ICODE SRAM Write protection page 5 */
/* !< ICODE SRAM Write protection page 6 */
/* !< ICODE SRAM Write protection page 7 */
/* *
  * @}
  */
/* * @defgroup SYSCFG_flags_definition 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/*  Function used to set the SYSCFG configuration to the default reset state **/
/* SYSCFG configuration functions *********************************************/
/* *
  * @brief  Clears the selected SYSCFG flag.
  * @param  SYSCFG_Flag: selects the flag to be cleared.
  *   This parameter can be any combination of the following values:
  *     @arg SYSCFG_FLAG_PE: SRAM parity error flag.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SYSCFG_ClearFlag(mut SYSCFG_Flag: uint32_t) {
    /* Check the parameter */
    let ref mut fresh20 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as *mut SYSCFG_TypeDef)).CFGR2;
    ::core::ptr::write_volatile(fresh20,
                                (::core::ptr::read_volatile::<uint32_t>(fresh20
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | SYSCFG_Flag) as
                                    uint32_t as uint32_t);
}
/* *
  * @}
  */ 
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
