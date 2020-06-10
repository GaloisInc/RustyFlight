use ::libc;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* !< Read Only */
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
pub type ITStatus = FlagStatus;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_TypeDef {
    pub ISR: uint32_t,
    pub IFCR: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_InitTypeDef {
    pub DMA_PeripheralBaseAddr: uint32_t,
    pub DMA_MemoryBaseAddr: uint32_t,
    pub DMA_DIR: uint32_t,
    pub DMA_BufferSize: uint32_t,
    pub DMA_PeripheralInc: uint32_t,
    pub DMA_MemoryInc: uint32_t,
    pub DMA_PeripheralDataSize: uint32_t,
    pub DMA_MemoryDataSize: uint32_t,
    pub DMA_Mode: uint32_t,
    pub DMA_Priority: uint32_t,
    pub DMA_M2M: uint32_t,
}
/* *
  * @}
  */
/* * @defgroup DMA_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_Private_Functions
  * @{
  */
/* *
  * @brief  Deinitializes the DMAy Channelx registers to their default reset
  *         values.
  * @param  DMAy_Channelx: where y can be 1 or 2 to select the DMA and
  *   x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the DMA Channel.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DMA_DeInit(mut DMAy_Channelx:
                                        *mut DMA_Channel_TypeDef) {
    /* Check the parameters */
    /* Disable the selected DMAy Channelx */
    ::core::ptr::write_volatile(&mut (*DMAy_Channelx).CCR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*DMAy_Channelx).CCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x1 as libc::c_int as uint16_t as
                                           libc::c_int) as uint16_t as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Reset DMAy Channelx control register */
    ::core::ptr::write_volatile(&mut (*DMAy_Channelx).CCR as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    /* Reset DMAy Channelx remaining bytes register */
    ::core::ptr::write_volatile(&mut (*DMAy_Channelx).CNDTR as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    /* Reset DMAy Channelx peripheral address register */
    ::core::ptr::write_volatile(&mut (*DMAy_Channelx).CPAR as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    /* Reset DMAy Channelx memory address register */
    ::core::ptr::write_volatile(&mut (*DMAy_Channelx).CMAR as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    if DMAy_Channelx ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x8 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut DMA_Channel_TypeDef {
        /* Reset interrupt pending bits for DMA1 Channel1 */
        let ref mut fresh0 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_TypeDef)).IFCR;
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_int as uint32_t |
                                              0x2 as libc::c_int as uint32_t |
                                              0x4 as libc::c_int as uint32_t |
                                              0x8 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    } else if DMAy_Channelx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x1c
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut DMA_Channel_TypeDef {
        /* Reset interrupt pending bits for DMA1 Channel2 */
        let ref mut fresh1 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_TypeDef)).IFCR;
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x10 as libc::c_int as uint32_t |
                                              0x20 as libc::c_int as uint32_t
                                              |
                                              0x40 as libc::c_int as uint32_t
                                              |
                                              0x80 as libc::c_int as
                                                  uint32_t)) as uint32_t as
                                        uint32_t)
    } else if DMAy_Channelx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x30
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut DMA_Channel_TypeDef {
        /* Reset interrupt pending bits for DMA1 Channel3 */
        let ref mut fresh2 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_TypeDef)).IFCR;
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x100 as libc::c_int as uint32_t |
                                              0x200 as libc::c_int as uint32_t
                                              |
                                              0x400 as libc::c_int as uint32_t
                                              |
                                              0x800 as libc::c_int as
                                                  uint32_t)) as uint32_t as
                                        uint32_t)
    } else if DMAy_Channelx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x44
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut DMA_Channel_TypeDef {
        /* Reset interrupt pending bits for DMA1 Channel4 */
        let ref mut fresh3 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_TypeDef)).IFCR;
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1000 as libc::c_int as uint32_t |
                                              0x2000 as libc::c_int as
                                                  uint32_t |
                                              0x4000 as libc::c_int as
                                                  uint32_t |
                                              0x8000 as libc::c_int as
                                                  uint32_t)) as uint32_t as
                                        uint32_t)
    } else if DMAy_Channelx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x58
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut DMA_Channel_TypeDef {
        /* Reset interrupt pending bits for DMA1 Channel5 */
        let ref mut fresh4 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_TypeDef)).IFCR;
        ::core::ptr::write_volatile(fresh4,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x10000 as libc::c_int as uint32_t |
                                              0x20000 as libc::c_int as
                                                  uint32_t |
                                              0x40000 as libc::c_int as
                                                  uint32_t |
                                              0x80000 as libc::c_int as
                                                  uint32_t)) as uint32_t as
                                        uint32_t)
    } else if DMAy_Channelx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x6c
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut DMA_Channel_TypeDef {
        /* Reset interrupt pending bits for DMA1 Channel6 */
        let ref mut fresh5 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_TypeDef)).IFCR;
        ::core::ptr::write_volatile(fresh5,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x100000 as libc::c_int as uint32_t
                                              |
                                              0x200000 as libc::c_int as
                                                  uint32_t |
                                              0x400000 as libc::c_int as
                                                  uint32_t |
                                              0x800000 as libc::c_int as
                                                  uint32_t)) as uint32_t as
                                        uint32_t)
    } else if DMAy_Channelx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x80
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut DMA_Channel_TypeDef {
        /* Reset interrupt pending bits for DMA1 Channel7 */
        let ref mut fresh6 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_TypeDef)).IFCR;
        ::core::ptr::write_volatile(fresh6,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1000000 as libc::c_int as uint32_t
                                              |
                                              0x2000000 as libc::c_int as
                                                  uint32_t |
                                              0x4000000 as libc::c_int as
                                                  uint32_t |
                                              0x8000000 as libc::c_int as
                                                  uint32_t)) as uint32_t as
                                        uint32_t)
    } else if DMAy_Channelx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x408
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut DMA_Channel_TypeDef {
        /* Reset interrupt pending bits for DMA2 Channel1 */
        let ref mut fresh7 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x400
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_TypeDef)).IFCR;
        ::core::ptr::write_volatile(fresh7,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_int as uint32_t |
                                              0x2 as libc::c_int as uint32_t |
                                              0x4 as libc::c_int as uint32_t |
                                              0x8 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    } else if DMAy_Channelx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x41c
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut DMA_Channel_TypeDef {
        /* Reset interrupt pending bits for DMA2 Channel2 */
        let ref mut fresh8 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x400
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_TypeDef)).IFCR;
        ::core::ptr::write_volatile(fresh8,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x10 as libc::c_int as uint32_t |
                                              0x20 as libc::c_int as uint32_t
                                              |
                                              0x40 as libc::c_int as uint32_t
                                              |
                                              0x80 as libc::c_int as
                                                  uint32_t)) as uint32_t as
                                        uint32_t)
    } else if DMAy_Channelx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x430
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut DMA_Channel_TypeDef {
        /* Reset interrupt pending bits for DMA2 Channel3 */
        let ref mut fresh9 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x400
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_TypeDef)).IFCR;
        ::core::ptr::write_volatile(fresh9,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x100 as libc::c_int as uint32_t |
                                              0x200 as libc::c_int as uint32_t
                                              |
                                              0x400 as libc::c_int as uint32_t
                                              |
                                              0x800 as libc::c_int as
                                                  uint32_t)) as uint32_t as
                                        uint32_t)
    } else if DMAy_Channelx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x444
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut DMA_Channel_TypeDef {
        /* Reset interrupt pending bits for DMA2 Channel4 */
        let ref mut fresh10 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x400
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_TypeDef)).IFCR;
        ::core::ptr::write_volatile(fresh10,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1000 as libc::c_int as uint32_t |
                                              0x2000 as libc::c_int as
                                                  uint32_t |
                                              0x4000 as libc::c_int as
                                                  uint32_t |
                                              0x8000 as libc::c_int as
                                                  uint32_t)) as uint32_t as
                                        uint32_t)
    } else if DMAy_Channelx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x458
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut DMA_Channel_TypeDef {
        /* Reset interrupt pending bits for DMA2 Channel5 */
        let ref mut fresh11 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x400
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_TypeDef)).IFCR;
        ::core::ptr::write_volatile(fresh11,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x10000 as libc::c_int as uint32_t |
                                              0x20000 as libc::c_int as
                                                  uint32_t |
                                              0x40000 as libc::c_int as
                                                  uint32_t |
                                              0x80000 as libc::c_int as
                                                  uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Initializes the DMAy Channelx according to the specified
  *         parameters in the DMA_InitStruct.
  * @param  DMAy_Channelx: where y can be 1 or 2 to select the DMA and 
  *   x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the DMA Channel.
  * @param  DMA_InitStruct: pointer to a DMA_InitTypeDef structure that
  *         contains the configuration information for the specified DMA Channel.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DMA_Init(mut DMAy_Channelx: *mut DMA_Channel_TypeDef,
                                  mut DMA_InitStruct: *mut DMA_InitTypeDef) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /*--------------------------- DMAy Channelx CCR Configuration -----------------*/
  /* Get the DMAy_Channelx CCR value */
    tmpreg = (*DMAy_Channelx).CCR;
    /* Clear MEM2MEM, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits */
    tmpreg &= 0xffff800f as libc::c_uint;
    /* Configure DMAy Channelx: data transfer, data size, priority level and mode */
  /* Set DIR bit according to DMA_DIR value */
  /* Set CIRC bit according to DMA_Mode value */
  /* Set PINC bit according to DMA_PeripheralInc value */
  /* Set MINC bit according to DMA_MemoryInc value */
  /* Set PSIZE bits according to DMA_PeripheralDataSize value */
  /* Set MSIZE bits according to DMA_MemoryDataSize value */
  /* Set PL bits according to DMA_Priority value */
  /* Set the MEM2MEM bit according to DMA_M2M value */
    tmpreg |=
        (*DMA_InitStruct).DMA_DIR | (*DMA_InitStruct).DMA_Mode |
            (*DMA_InitStruct).DMA_PeripheralInc |
            (*DMA_InitStruct).DMA_MemoryInc |
            (*DMA_InitStruct).DMA_PeripheralDataSize |
            (*DMA_InitStruct).DMA_MemoryDataSize |
            (*DMA_InitStruct).DMA_Priority | (*DMA_InitStruct).DMA_M2M;
    /* Write to DMAy Channelx CCR */
    ::core::ptr::write_volatile(&mut (*DMAy_Channelx).CCR as *mut uint32_t,
                                tmpreg);
    /*--------------------------- DMAy Channelx CNDTR Configuration ---------------*/
  /* Write to DMAy Channelx CNDTR */
    ::core::ptr::write_volatile(&mut (*DMAy_Channelx).CNDTR as *mut uint32_t,
                                (*DMA_InitStruct).DMA_BufferSize);
    /*--------------------------- DMAy Channelx CPAR Configuration ----------------*/
  /* Write to DMAy Channelx CPAR */
    ::core::ptr::write_volatile(&mut (*DMAy_Channelx).CPAR as *mut uint32_t,
                                (*DMA_InitStruct).DMA_PeripheralBaseAddr);
    /*--------------------------- DMAy Channelx CMAR Configuration ----------------*/
  /* Write to DMAy Channelx CMAR */
    ::core::ptr::write_volatile(&mut (*DMAy_Channelx).CMAR as *mut uint32_t,
                                (*DMA_InitStruct).DMA_MemoryBaseAddr);
}
/* *
  * @brief  Fills each DMA_InitStruct member with its default value.
  * @param  DMA_InitStruct : pointer to a DMA_InitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DMA_StructInit(mut DMA_InitStruct:
                                            *mut DMA_InitTypeDef) {
    /*-------------- Reset DMA init structure parameters values ------------------*/
  /* Initialize the DMA_PeripheralBaseAddr member */
    (*DMA_InitStruct).DMA_PeripheralBaseAddr = 0 as libc::c_int as uint32_t;
    /* Initialize the DMA_MemoryBaseAddr member */
    (*DMA_InitStruct).DMA_MemoryBaseAddr = 0 as libc::c_int as uint32_t;
    /* Initialize the DMA_DIR member */
    (*DMA_InitStruct).DMA_DIR = 0 as libc::c_int as uint32_t;
    /* Initialize the DMA_BufferSize member */
    (*DMA_InitStruct).DMA_BufferSize = 0 as libc::c_int as uint32_t;
    /* Initialize the DMA_PeripheralInc member */
    (*DMA_InitStruct).DMA_PeripheralInc = 0 as libc::c_int as uint32_t;
    /* Initialize the DMA_MemoryInc member */
    (*DMA_InitStruct).DMA_MemoryInc = 0 as libc::c_int as uint32_t;
    /* Initialize the DMA_PeripheralDataSize member */
    (*DMA_InitStruct).DMA_PeripheralDataSize = 0 as libc::c_int as uint32_t;
    /* Initialize the DMA_MemoryDataSize member */
    (*DMA_InitStruct).DMA_MemoryDataSize = 0 as libc::c_int as uint32_t;
    /* Initialize the DMA_Mode member */
    (*DMA_InitStruct).DMA_Mode = 0 as libc::c_int as uint32_t;
    /* Initialize the DMA_Priority member */
    (*DMA_InitStruct).DMA_Priority = 0 as libc::c_int as uint32_t;
    /* Initialize the DMA_M2M member */
    (*DMA_InitStruct).DMA_M2M = 0 as libc::c_int as uint32_t;
}
/* *
  * @brief  Enables or disables the specified DMAy Channelx.
  * @param  DMAy_Channelx: where y can be 1 or 2 to select the DMA and 
  *   x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the DMA Channel.
  * @param  NewState: new state of the DMAy Channelx. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DMA_Cmd(mut DMAy_Channelx: *mut DMA_Channel_TypeDef,
                                 mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected DMAy Channelx */
        ::core::ptr::write_volatile(&mut (*DMAy_Channelx).CCR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*DMAy_Channelx).CCR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1 as libc::c_int as uint16_t as
                                             libc::c_uint) as uint32_t as
                                        uint32_t)
    } else {
        /* Disable the selected DMAy Channelx */
        ::core::ptr::write_volatile(&mut (*DMAy_Channelx).CCR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*DMAy_Channelx).CCR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x1 as libc::c_int as uint16_t as
                                               libc::c_int) as uint16_t as
                                             libc::c_uint) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Enables or disables the specified DMAy Channelx interrupts.
  * @param  DMAy_Channelx: where y can be 1 or 2 to select the DMA and 
  *   x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the DMA Channel.
  * @param  DMA_IT: specifies the DMA interrupts sources to be enabled
  *   or disabled. 
  *   This parameter can be any combination of the following values:
  *     @arg DMA_IT_TC:  Transfer complete interrupt mask
  *     @arg DMA_IT_HT:  Half transfer interrupt mask
  *     @arg DMA_IT_TE:  Transfer error interrupt mask
  * @param  NewState: new state of the specified DMA interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DMA_ITConfig(mut DMAy_Channelx:
                                          *mut DMA_Channel_TypeDef,
                                      mut DMA_IT: uint32_t,
                                      mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected DMA interrupts */
        ::core::ptr::write_volatile(&mut (*DMAy_Channelx).CCR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*DMAy_Channelx).CCR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | DMA_IT) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the selected DMA interrupts */
        ::core::ptr::write_volatile(&mut (*DMAy_Channelx).CCR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*DMAy_Channelx).CCR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !DMA_IT) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Sets the number of data units in the current DMAy Channelx transfer.
  * @param  DMAy_Channelx: where y can be 1 or 2 to select the DMA and 
  *         x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the DMA Channel.
  * @param  DataNumber: The number of data units in the current DMAy Channelx
  *         transfer.   
  * @note   This function can only be used when the DMAy_Channelx is disabled.                 
  * @retval None.
  */
#[no_mangle]
pub unsafe extern "C" fn DMA_SetCurrDataCounter(mut DMAy_Channelx:
                                                    *mut DMA_Channel_TypeDef,
                                                mut DataNumber: uint16_t) {
    /* Check the parameters */
    /*--------------------------- DMAy Channelx CNDTR Configuration ---------------*/
  /* Write to DMAy Channelx CNDTR */
    ::core::ptr::write_volatile(&mut (*DMAy_Channelx).CNDTR as *mut uint32_t,
                                DataNumber as uint32_t);
}
/* *
  * @brief  Returns the number of remaining data units in the current
  *         DMAy Channelx transfer.
  * @param  DMAy_Channelx: where y can be 1 or 2 to select the DMA and 
  *   x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the DMA Channel.
  * @retval The number of remaining data units in the current DMAy Channelx
  *         transfer.
  */
#[no_mangle]
pub unsafe extern "C" fn DMA_GetCurrDataCounter(mut DMAy_Channelx:
                                                    *mut DMA_Channel_TypeDef)
 -> uint16_t {
    /* Check the parameters */
    /* Return the number of remaining data units for DMAy Channelx */
    return (*DMAy_Channelx).CNDTR as uint16_t;
}
/* *
  * @brief  Checks whether the specified DMAy Channelx flag is set or not.
  * @param  DMAy_FLAG: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg DMA1_FLAG_GL1: DMA1 Channel1 global flag.
  *     @arg DMA1_FLAG_TC1: DMA1 Channel1 transfer complete flag.
  *     @arg DMA1_FLAG_HT1: DMA1 Channel1 half transfer flag.
  *     @arg DMA1_FLAG_TE1: DMA1 Channel1 transfer error flag.
  *     @arg DMA1_FLAG_GL2: DMA1 Channel2 global flag.
  *     @arg DMA1_FLAG_TC2: DMA1 Channel2 transfer complete flag.
  *     @arg DMA1_FLAG_HT2: DMA1 Channel2 half transfer flag.
  *     @arg DMA1_FLAG_TE2: DMA1 Channel2 transfer error flag.
  *     @arg DMA1_FLAG_GL3: DMA1 Channel3 global flag.
  *     @arg DMA1_FLAG_TC3: DMA1 Channel3 transfer complete flag.
  *     @arg DMA1_FLAG_HT3: DMA1 Channel3 half transfer flag.
  *     @arg DMA1_FLAG_TE3: DMA1 Channel3 transfer error flag.
  *     @arg DMA1_FLAG_GL4: DMA1 Channel4 global flag.
  *     @arg DMA1_FLAG_TC4: DMA1 Channel4 transfer complete flag.
  *     @arg DMA1_FLAG_HT4: DMA1 Channel4 half transfer flag.
  *     @arg DMA1_FLAG_TE4: DMA1 Channel4 transfer error flag.
  *     @arg DMA1_FLAG_GL5: DMA1 Channel5 global flag.
  *     @arg DMA1_FLAG_TC5: DMA1 Channel5 transfer complete flag.
  *     @arg DMA1_FLAG_HT5: DMA1 Channel5 half transfer flag.
  *     @arg DMA1_FLAG_TE5: DMA1 Channel5 transfer error flag.
  *     @arg DMA1_FLAG_GL6: DMA1 Channel6 global flag.
  *     @arg DMA1_FLAG_TC6: DMA1 Channel6 transfer complete flag.
  *     @arg DMA1_FLAG_HT6: DMA1 Channel6 half transfer flag.
  *     @arg DMA1_FLAG_TE6: DMA1 Channel6 transfer error flag.
  *     @arg DMA1_FLAG_GL7: DMA1 Channel7 global flag.
  *     @arg DMA1_FLAG_TC7: DMA1 Channel7 transfer complete flag.
  *     @arg DMA1_FLAG_HT7: DMA1 Channel7 half transfer flag.
  *     @arg DMA1_FLAG_TE7: DMA1 Channel7 transfer error flag.
  *     @arg DMA2_FLAG_GL1: DMA2 Channel1 global flag.
  *     @arg DMA2_FLAG_TC1: DMA2 Channel1 transfer complete flag.
  *     @arg DMA2_FLAG_HT1: DMA2 Channel1 half transfer flag.
  *     @arg DMA2_FLAG_TE1: DMA2 Channel1 transfer error flag.
  *     @arg DMA2_FLAG_GL2: DMA2 Channel2 global flag.
  *     @arg DMA2_FLAG_TC2: DMA2 Channel2 transfer complete flag.
  *     @arg DMA2_FLAG_HT2: DMA2 Channel2 half transfer flag.
  *     @arg DMA2_FLAG_TE2: DMA2 Channel2 transfer error flag.
  *     @arg DMA2_FLAG_GL3: DMA2 Channel3 global flag.
  *     @arg DMA2_FLAG_TC3: DMA2 Channel3 transfer complete flag.
  *     @arg DMA2_FLAG_HT3: DMA2 Channel3 half transfer flag.
  *     @arg DMA2_FLAG_TE3: DMA2 Channel3 transfer error flag.
  *     @arg DMA2_FLAG_GL4: DMA2 Channel4 global flag.
  *     @arg DMA2_FLAG_TC4: DMA2 Channel4 transfer complete flag.
  *     @arg DMA2_FLAG_HT4: DMA2 Channel4 half transfer flag.
  *     @arg DMA2_FLAG_TE4: DMA2 Channel4 transfer error flag.
  *     @arg DMA2_FLAG_GL5: DMA2 Channel5 global flag.
  *     @arg DMA2_FLAG_TC5: DMA2 Channel5 transfer complete flag.
  *     @arg DMA2_FLAG_HT5: DMA2 Channel5 half transfer flag.
  *     @arg DMA2_FLAG_TE5: DMA2 Channel5 transfer error flag.
  * @retval The new state of DMAy_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn DMA_GetFlagStatus(mut DMAy_FLAG: uint32_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Calculate the used DMAy */
    if DMAy_FLAG & 0x10000000 as libc::c_int as uint32_t !=
           RESET as libc::c_int as uint32_t {
        /* Get DMA2 ISR register value */
        tmpreg =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x400
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_TypeDef)).ISR
    } else {
        /* Get DMA1 ISR register value */
        tmpreg =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_TypeDef)).ISR
    }
    /* Check the status of the specified DMAy flag */
    if tmpreg & DMAy_FLAG != RESET as libc::c_int as uint32_t {
        /* DMAy_FLAG is set */
        bitstatus = SET
    } else {
        /* DMAy_FLAG is reset */
        bitstatus = RESET
    }
    /* Return the DMAy_FLAG status */
    return bitstatus;
}
/* *
  * @brief  Clears the DMAy Channelx's pending flags.
  * @param  DMAy_FLAG: specifies the flag to clear.
  *   This parameter can be any combination (for the same DMA) of the following values:
  *     @arg DMA1_FLAG_GL1: DMA1 Channel1 global flag.
  *     @arg DMA1_FLAG_TC1: DMA1 Channel1 transfer complete flag.
  *     @arg DMA1_FLAG_HT1: DMA1 Channel1 half transfer flag.
  *     @arg DMA1_FLAG_TE1: DMA1 Channel1 transfer error flag.
  *     @arg DMA1_FLAG_GL2: DMA1 Channel2 global flag.
  *     @arg DMA1_FLAG_TC2: DMA1 Channel2 transfer complete flag.
  *     @arg DMA1_FLAG_HT2: DMA1 Channel2 half transfer flag.
  *     @arg DMA1_FLAG_TE2: DMA1 Channel2 transfer error flag.
  *     @arg DMA1_FLAG_GL3: DMA1 Channel3 global flag.
  *     @arg DMA1_FLAG_TC3: DMA1 Channel3 transfer complete flag.
  *     @arg DMA1_FLAG_HT3: DMA1 Channel3 half transfer flag.
  *     @arg DMA1_FLAG_TE3: DMA1 Channel3 transfer error flag.
  *     @arg DMA1_FLAG_GL4: DMA1 Channel4 global flag.
  *     @arg DMA1_FLAG_TC4: DMA1 Channel4 transfer complete flag.
  *     @arg DMA1_FLAG_HT4: DMA1 Channel4 half transfer flag.
  *     @arg DMA1_FLAG_TE4: DMA1 Channel4 transfer error flag.
  *     @arg DMA1_FLAG_GL5: DMA1 Channel5 global flag.
  *     @arg DMA1_FLAG_TC5: DMA1 Channel5 transfer complete flag.
  *     @arg DMA1_FLAG_HT5: DMA1 Channel5 half transfer flag.
  *     @arg DMA1_FLAG_TE5: DMA1 Channel5 transfer error flag.
  *     @arg DMA1_FLAG_GL6: DMA1 Channel6 global flag.
  *     @arg DMA1_FLAG_TC6: DMA1 Channel6 transfer complete flag.
  *     @arg DMA1_FLAG_HT6: DMA1 Channel6 half transfer flag.
  *     @arg DMA1_FLAG_TE6: DMA1 Channel6 transfer error flag.
  *     @arg DMA1_FLAG_GL7: DMA1 Channel7 global flag.
  *     @arg DMA1_FLAG_TC7: DMA1 Channel7 transfer complete flag.
  *     @arg DMA1_FLAG_HT7: DMA1 Channel7 half transfer flag.
  *     @arg DMA1_FLAG_TE7: DMA1 Channel7 transfer error flag.
  *     @arg DMA2_FLAG_GL1: DMA2 Channel1 global flag.
  *     @arg DMA2_FLAG_TC1: DMA2 Channel1 transfer complete flag.
  *     @arg DMA2_FLAG_HT1: DMA2 Channel1 half transfer flag.
  *     @arg DMA2_FLAG_TE1: DMA2 Channel1 transfer error flag.
  *     @arg DMA2_FLAG_GL2: DMA2 Channel2 global flag.
  *     @arg DMA2_FLAG_TC2: DMA2 Channel2 transfer complete flag.
  *     @arg DMA2_FLAG_HT2: DMA2 Channel2 half transfer flag.
  *     @arg DMA2_FLAG_TE2: DMA2 Channel2 transfer error flag.
  *     @arg DMA2_FLAG_GL3: DMA2 Channel3 global flag.
  *     @arg DMA2_FLAG_TC3: DMA2 Channel3 transfer complete flag.
  *     @arg DMA2_FLAG_HT3: DMA2 Channel3 half transfer flag.
  *     @arg DMA2_FLAG_TE3: DMA2 Channel3 transfer error flag.
  *     @arg DMA2_FLAG_GL4: DMA2 Channel4 global flag.
  *     @arg DMA2_FLAG_TC4: DMA2 Channel4 transfer complete flag.
  *     @arg DMA2_FLAG_HT4: DMA2 Channel4 half transfer flag.
  *     @arg DMA2_FLAG_TE4: DMA2 Channel4 transfer error flag.
  *     @arg DMA2_FLAG_GL5: DMA2 Channel5 global flag.
  *     @arg DMA2_FLAG_TC5: DMA2 Channel5 transfer complete flag.
  *     @arg DMA2_FLAG_HT5: DMA2 Channel5 half transfer flag.
  *     @arg DMA2_FLAG_TE5: DMA2 Channel5 transfer error flag.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DMA_ClearFlag(mut DMAy_FLAG: uint32_t) {
    /* Check the parameters */
    /* Calculate the used DMAy */
    if DMAy_FLAG & 0x10000000 as libc::c_int as uint32_t !=
           RESET as libc::c_int as uint32_t {
        /* Clear the selected DMAy flags */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x400
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_TypeDef)).IFCR as
                                        *mut uint32_t, DMAy_FLAG)
    } else {
        /* Clear the selected DMAy flags */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_TypeDef)).IFCR as
                                        *mut uint32_t, DMAy_FLAG)
    };
}
/* *
  * @brief  Checks whether the specified DMAy Channelx interrupt has occurred or not.
  * @param  DMAy_IT: specifies the DMAy interrupt source to check. 
  *   This parameter can be one of the following values:
  *     @arg DMA1_IT_GL1: DMA1 Channel1 global interrupt.
  *     @arg DMA1_IT_TC1: DMA1 Channel1 transfer complete interrupt.
  *     @arg DMA1_IT_HT1: DMA1 Channel1 half transfer interrupt.
  *     @arg DMA1_IT_TE1: DMA1 Channel1 transfer error interrupt.
  *     @arg DMA1_IT_GL2: DMA1 Channel2 global interrupt.
  *     @arg DMA1_IT_TC2: DMA1 Channel2 transfer complete interrupt.
  *     @arg DMA1_IT_HT2: DMA1 Channel2 half transfer interrupt.
  *     @arg DMA1_IT_TE2: DMA1 Channel2 transfer error interrupt.
  *     @arg DMA1_IT_GL3: DMA1 Channel3 global interrupt.
  *     @arg DMA1_IT_TC3: DMA1 Channel3 transfer complete interrupt.
  *     @arg DMA1_IT_HT3: DMA1 Channel3 half transfer interrupt.
  *     @arg DMA1_IT_TE3: DMA1 Channel3 transfer error interrupt.
  *     @arg DMA1_IT_GL4: DMA1 Channel4 global interrupt.
  *     @arg DMA1_IT_TC4: DMA1 Channel4 transfer complete interrupt.
  *     @arg DMA1_IT_HT4: DMA1 Channel4 half transfer interrupt.
  *     @arg DMA1_IT_TE4: DMA1 Channel4 transfer error interrupt.
  *     @arg DMA1_IT_GL5: DMA1 Channel5 global interrupt.
  *     @arg DMA1_IT_TC5: DMA1 Channel5 transfer complete interrupt.
  *     @arg DMA1_IT_HT5: DMA1 Channel5 half transfer interrupt.
  *     @arg DMA1_IT_TE5: DMA1 Channel5 transfer error interrupt.
  *     @arg DMA1_IT_GL6: DMA1 Channel6 global interrupt.
  *     @arg DMA1_IT_TC6: DMA1 Channel6 transfer complete interrupt.
  *     @arg DMA1_IT_HT6: DMA1 Channel6 half transfer interrupt.
  *     @arg DMA1_IT_TE6: DMA1 Channel6 transfer error interrupt.
  *     @arg DMA1_IT_GL7: DMA1 Channel7 global interrupt.
  *     @arg DMA1_IT_TC7: DMA1 Channel7 transfer complete interrupt.
  *     @arg DMA1_IT_HT7: DMA1 Channel7 half transfer interrupt.
  *     @arg DMA1_IT_TE7: DMA1 Channel7 transfer error interrupt.
  *     @arg DMA2_IT_GL1: DMA2 Channel1 global interrupt.
  *     @arg DMA2_IT_TC1: DMA2 Channel1 transfer complete interrupt.
  *     @arg DMA2_IT_HT1: DMA2 Channel1 half transfer interrupt.
  *     @arg DMA2_IT_TE1: DMA2 Channel1 transfer error interrupt.
  *     @arg DMA2_IT_GL2: DMA2 Channel2 global interrupt.
  *     @arg DMA2_IT_TC2: DMA2 Channel2 transfer complete interrupt.
  *     @arg DMA2_IT_HT2: DMA2 Channel2 half transfer interrupt.
  *     @arg DMA2_IT_TE2: DMA2 Channel2 transfer error interrupt.
  *     @arg DMA2_IT_GL3: DMA2 Channel3 global interrupt.
  *     @arg DMA2_IT_TC3: DMA2 Channel3 transfer complete interrupt.
  *     @arg DMA2_IT_HT3: DMA2 Channel3 half transfer interrupt.
  *     @arg DMA2_IT_TE3: DMA2 Channel3 transfer error interrupt.
  *     @arg DMA2_IT_GL4: DMA2 Channel4 global interrupt.
  *     @arg DMA2_IT_TC4: DMA2 Channel4 transfer complete interrupt.
  *     @arg DMA2_IT_HT4: DMA2 Channel4 half transfer interrupt.
  *     @arg DMA2_IT_TE4: DMA2 Channel4 transfer error interrupt.
  *     @arg DMA2_IT_GL5: DMA2 Channel5 global interrupt.
  *     @arg DMA2_IT_TC5: DMA2 Channel5 transfer complete interrupt.
  *     @arg DMA2_IT_HT5: DMA2 Channel5 half transfer interrupt.
  *     @arg DMA2_IT_TE5: DMA2 Channel5 transfer error interrupt.
  * @retval The new state of DMAy_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn DMA_GetITStatus(mut DMAy_IT: uint32_t) -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Calculate the used DMA */
    if DMAy_IT & 0x10000000 as libc::c_int as uint32_t !=
           RESET as libc::c_int as uint32_t {
        /* Get DMA2 ISR register value */
        tmpreg =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x400
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_TypeDef)).ISR
    } else {
        /* Get DMA1 ISR register value */
        tmpreg =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_TypeDef)).ISR
    }
    /* Check the status of the specified DMAy interrupt */
    if tmpreg & DMAy_IT != RESET as libc::c_int as uint32_t {
        /* DMAy_IT is set */
        bitstatus = SET
    } else {
        /* DMAy_IT is reset */
        bitstatus = RESET
    }
    /* Return the DMA_IT status */
    return bitstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f10x_dma.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the DMA firmware 
  *          library.
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
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup DMA
  * @{
  */
/* * @defgroup DMA_Exported_Types
  * @{
  */
/* * 
  * @brief  DMA Init structure definition
  */
/* !< Specifies the peripheral base address for DMAy Channelx. */
/* !< Specifies the memory base address for DMAy Channelx. */
/* !< Specifies if the peripheral is the source or destination.
                                        This parameter can be a value of @ref DMA_data_transfer_direction */
/* !< Specifies the buffer size, in data unit, of the specified Channel. 
                                        The data unit is equal to the configuration set in DMA_PeripheralDataSize
                                        or DMA_MemoryDataSize members depending in the transfer direction. */
/* !< Specifies whether the Peripheral address register is incremented or not.
                                        This parameter can be a value of @ref DMA_peripheral_incremented_mode */
/* !< Specifies whether the memory address register is incremented or not.
                                        This parameter can be a value of @ref DMA_memory_incremented_mode */
/* !< Specifies the Peripheral data width.
                                        This parameter can be a value of @ref DMA_peripheral_data_size */
/* !< Specifies the Memory data width.
                                        This parameter can be a value of @ref DMA_memory_data_size */
/* !< Specifies the operation mode of the DMAy Channelx.
                                        This parameter can be a value of @ref DMA_circular_normal_mode.
                                        @note: The circular buffer mode cannot be used if the memory-to-memory
                                              data transfer is configured on the selected Channel */
/* !< Specifies the software priority for the DMAy Channelx.
                                        This parameter can be a value of @ref DMA_priority_level */
/* !< Specifies if the DMAy Channelx will be used in memory-to-memory transfer.
                                        This parameter can be a value of @ref DMA_memory_to_memory */
/* *
  * @}
  */
/* * @defgroup DMA_Exported_Constants
  * @{
  */
/* * @defgroup DMA_data_transfer_direction 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_peripheral_incremented_mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_memory_incremented_mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_peripheral_data_size 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_memory_data_size 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_circular_normal_mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_priority_level 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_memory_to_memory 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_interrupts_definition 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_flags_definition 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_Buffer_Size 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup DMA_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_Exported_Functions
  * @{
  */
/* *
  * @brief  Clears the DMAy Channelx's interrupt pending bits.
  * @param  DMAy_IT: specifies the DMAy interrupt pending bit to clear.
  *   This parameter can be any combination (for the same DMA) of the following values:
  *     @arg DMA1_IT_GL1: DMA1 Channel1 global interrupt.
  *     @arg DMA1_IT_TC1: DMA1 Channel1 transfer complete interrupt.
  *     @arg DMA1_IT_HT1: DMA1 Channel1 half transfer interrupt.
  *     @arg DMA1_IT_TE1: DMA1 Channel1 transfer error interrupt.
  *     @arg DMA1_IT_GL2: DMA1 Channel2 global interrupt.
  *     @arg DMA1_IT_TC2: DMA1 Channel2 transfer complete interrupt.
  *     @arg DMA1_IT_HT2: DMA1 Channel2 half transfer interrupt.
  *     @arg DMA1_IT_TE2: DMA1 Channel2 transfer error interrupt.
  *     @arg DMA1_IT_GL3: DMA1 Channel3 global interrupt.
  *     @arg DMA1_IT_TC3: DMA1 Channel3 transfer complete interrupt.
  *     @arg DMA1_IT_HT3: DMA1 Channel3 half transfer interrupt.
  *     @arg DMA1_IT_TE3: DMA1 Channel3 transfer error interrupt.
  *     @arg DMA1_IT_GL4: DMA1 Channel4 global interrupt.
  *     @arg DMA1_IT_TC4: DMA1 Channel4 transfer complete interrupt.
  *     @arg DMA1_IT_HT4: DMA1 Channel4 half transfer interrupt.
  *     @arg DMA1_IT_TE4: DMA1 Channel4 transfer error interrupt.
  *     @arg DMA1_IT_GL5: DMA1 Channel5 global interrupt.
  *     @arg DMA1_IT_TC5: DMA1 Channel5 transfer complete interrupt.
  *     @arg DMA1_IT_HT5: DMA1 Channel5 half transfer interrupt.
  *     @arg DMA1_IT_TE5: DMA1 Channel5 transfer error interrupt.
  *     @arg DMA1_IT_GL6: DMA1 Channel6 global interrupt.
  *     @arg DMA1_IT_TC6: DMA1 Channel6 transfer complete interrupt.
  *     @arg DMA1_IT_HT6: DMA1 Channel6 half transfer interrupt.
  *     @arg DMA1_IT_TE6: DMA1 Channel6 transfer error interrupt.
  *     @arg DMA1_IT_GL7: DMA1 Channel7 global interrupt.
  *     @arg DMA1_IT_TC7: DMA1 Channel7 transfer complete interrupt.
  *     @arg DMA1_IT_HT7: DMA1 Channel7 half transfer interrupt.
  *     @arg DMA1_IT_TE7: DMA1 Channel7 transfer error interrupt.
  *     @arg DMA2_IT_GL1: DMA2 Channel1 global interrupt.
  *     @arg DMA2_IT_TC1: DMA2 Channel1 transfer complete interrupt.
  *     @arg DMA2_IT_HT1: DMA2 Channel1 half transfer interrupt.
  *     @arg DMA2_IT_TE1: DMA2 Channel1 transfer error interrupt.
  *     @arg DMA2_IT_GL2: DMA2 Channel2 global interrupt.
  *     @arg DMA2_IT_TC2: DMA2 Channel2 transfer complete interrupt.
  *     @arg DMA2_IT_HT2: DMA2 Channel2 half transfer interrupt.
  *     @arg DMA2_IT_TE2: DMA2 Channel2 transfer error interrupt.
  *     @arg DMA2_IT_GL3: DMA2 Channel3 global interrupt.
  *     @arg DMA2_IT_TC3: DMA2 Channel3 transfer complete interrupt.
  *     @arg DMA2_IT_HT3: DMA2 Channel3 half transfer interrupt.
  *     @arg DMA2_IT_TE3: DMA2 Channel3 transfer error interrupt.
  *     @arg DMA2_IT_GL4: DMA2 Channel4 global interrupt.
  *     @arg DMA2_IT_TC4: DMA2 Channel4 transfer complete interrupt.
  *     @arg DMA2_IT_HT4: DMA2 Channel4 half transfer interrupt.
  *     @arg DMA2_IT_TE4: DMA2 Channel4 transfer error interrupt.
  *     @arg DMA2_IT_GL5: DMA2 Channel5 global interrupt.
  *     @arg DMA2_IT_TC5: DMA2 Channel5 transfer complete interrupt.
  *     @arg DMA2_IT_HT5: DMA2 Channel5 half transfer interrupt.
  *     @arg DMA2_IT_TE5: DMA2 Channel5 transfer error interrupt.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DMA_ClearITPendingBit(mut DMAy_IT: uint32_t) {
    /* Check the parameters */
    /* Calculate the used DMAy */
    if DMAy_IT & 0x10000000 as libc::c_int as uint32_t !=
           RESET as libc::c_int as uint32_t {
        /* Clear the selected DMAy interrupt pending bits */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x400
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_TypeDef)).IFCR as
                                        *mut uint32_t, DMAy_IT)
    } else {
        /* Clear the selected DMAy interrupt pending bits */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_TypeDef)).IFCR as
                                        *mut uint32_t, DMAy_IT)
    };
}
/* ****************** (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
